/**
 * @file motor.cpp
 * @defgroup motor Motor Subsystem
 * @brief BLDC Motor Control System File
 *
 * This file contains implementation for a comprehensive BLDC motor control
 * system using FreeRTOS, Hall effect sensors, and power monitoring via ADC. The
 * system provides speed control, 6-step commutation, real-time power
 * monitoring, and safe motor operation.
 *
 * ## System Architecture:
 * - **Motor Control Task**: Handles Hall sensor interrupts and commutation
 * - **Speed Controller Task**: Implements PID control for RPM regulation
 * - **Power Task**: Monitors current consumption and calculates power
 * - **Timer-based Ramping**: Provides soft start/stop functionality
 *
 * ## Motor Control Interface:
 * ```c
 * // Initialize motor system (call once at startup)
 * if (CreateMotorTask() != 0) {
 *     // Handle initialization failure
 * }
 *
 * // Basic motor control
 * setRPM = 2000;        // Set target speed (0-3250 RPM)
 * estop = 0;            // Enable motor (0=run, 1=stop)
 *
 * // Emergency stop sequence
 * estop = 1;            // Triggers soft deceleration stop, will also setRPM to
 * 0
 * // Motor will ramp down automatically using esteps
 *
 * // Check motor status
 * uint32_t localRPM = 0;
 * uint16_t localOutput = 0;
 * if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(100) == pdFALSE)){
 *   localRPM = currentRPM;
 *   localOutput = currentOutput;
 *   xSemaphoreGive(motorMutex);
 * }
 * ```
 *
 * ## Power Monitoring Interface:
 *
 * ```c
 * // Safe power monitoring access
 * extern double filtered_ch0, filtered_ch4;  // Filtered ADC values
 *
 * // Read power data safely (disable interrupts during multi-step calculations)
 * double current_a = readCurrent((uint16_t)filtered_ch0);
 * double current_b = readCurrent((uint16_t)filtered_ch4);
 * double total_power = estimatePower(24.0, current_a, current_b);
 * ```
 *
 * ## Safety Considerations:
 * - **Power Monitoring**: ADC values updated asynchronously - use critical
 * sections
 * - **RPM Limits**: System enforces MAXRPM (3250) limit automatically
 * - **Soft Start/Stop**: Timer-based ramping prevents sudden speed changes
 * - **Hall Sensor Validation**: System handles sensor failures with polling
 * fallback
 * - **Current Sensing**: Dual-phase monitoring with 7mΩ sense resistors
 *
 * ## System Timing:
 * - Power Task: ~400ms update rate with ADC interrupt triggering
 * - Speed Controller: 100ms PID update cycle
 * - RPM Ramping: 250ms timer intervals (4Hz)
 * - Motor Commutation: Interrupt-driven (μs response time)
 *
 * ## Hardware Requirements:
 * - TM4C microcontroller with FreeRTOS
 * - 3x Hall effect sensors (GPIO ports M, H, N)
 * - 2x Current sensors with ADC inputs (channels 0, 4)
 * - BLDC motor with compatible commutation sequence
 * - 24V motor supply with current sensing capability
 *
 * @author James Prince & Samuel Smith
 * @version 1.0
 * @{
 */

/* Standard includes. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <shared.h>
/* Kernel includes. */
#include <FreeRTOS.h>
#include <math.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <task.h>

/* Hardware includes. */
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "sensor_task.h"
#include "utils/uartstdio.h"
#include <adc.h>
#include <gpio.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <interrupt.h>
#include <motorlib.h>
#include <rom_map.h>
#include <rtos_hw_drivers.h>
#include <sysctl.h>
#include <timer.h>
#include <uartstdio.h>

// Prototypes of Functions
extern volatile uint32_t g_ui32SysClock;
static void motorTask(void *_);
static void speedControllerTask(void *_);
static void setupADC(void);
static void motorTestTask(void *_);
static void _adcScanTask(void *_);
static void setupGPIO(void);
static void powerTask(void *pvParameters);
void setupTimers();


/** @brief Utility macro to set specific bit in a number */
#define setBitTo(n, b, v) (n & ~(1 << b)) | (v << b)

/** @brief Convert FreeRTOS ticks to milliseconds */
#define TICKS_TO_MS(ticks) (((double)ticks) / ((double)configTICK_RATE_HZ))

/** */
#define MAXBUFFER 40

typedef struct
{
  uint32_t ch0;
  uint32_t ch4;
} adcSample;

volatile adcSample adcBuffer[MAXBUFFER];
volatile uint8_t adcHead = 0;
volatile uint8_t adcTail = 0;

/** @brief Maximum RPM rating of the motor */
#define MAXRPM 3250

/** @brief PWM period setting (10-100 range) */
#define PERIOD 100

/** @brief Low-pass filter coefficient for ADC signal filtering */
static const double alpha = 0.5;

/** @brief PID Proportional gain constant for speed control */
const double Kp = 0.032;

/** @brief PID Integral gain constant for speed control */
const double Ki = 0.075;

/** @brief PID Derivative gain constant for speed control */
const double Kd = 0.00005;

/** @brief Task handle for power monitoring task */
TaskHandle_t powerTaskHandle;

/** @brief Filtered ADC value for current sensor channel 0 */
double filtered_ch0;

/** @brief Filtered ADC value for current sensor channel 4 */
double filtered_ch4;

/** @brief Semaphore signaled by Hall effect sensor interrupts */
SemaphoreHandle_t hallEffectFlag;

/** @brief Motor direction flag (deprecated - causes stalling) */
bool direction;

/** @brief Motor enable state (1=on, 0=off) */
bool motorState;

/**
 * @brief Current RPM of the motor
 * @warning motorMulex should be taken before accessing!
 */
uint32_t currentRPM = 0;

/**
 * @breif Current Output to PWM
 * @warning motorMulex should be taken before accessing!
 */
uint16_t currentOutput = 0;

/** @brief motorMulex for Current RPM and output*/
SemaphoreHandle_t estopMutex;

/**
 * @brief Target RPM setpoint for speed controller
 * @details Set this variable to change motor target speed
 */
volatile double setRPM;

/** @brief Current soft RPM value (ramped setpoint) */
volatile double softSetRPM;

/**
 * @brief Emergency stop flag
 * @details Set to 1 to initiate soft motor stop, 0 to enable
 */
bool estop = 0;

/** @brief Maximum current limit for motor */
sharedValues maxCurrentLimit;

/** @brief RPM ramping step size for acceleration */
double steps;

/** @brief RPM ramping step size for emergency stop deceleration */
double esteps;


/** @brief Shared RPM values */
sharedValues rpmData;
/** @brief Shared Power Values */
sharedValues powerData;
/** @brief Shared PWM value */
sharedValues pwmData;

/**
 * @brief Current Hall effect sensor state (3-bit combination)
 * @details Bit 2: Sensor C, Bit 1: Sensor B, Bit 0: Sensor A
 */
volatile uint8_t hallEffectState;

/** @brief Counter for Hall effect sensor transitions (for RPM calculation) */
volatile uint8_t hallEffect_Count;

/**
 * @brief GPIO Port H Interrupt Handler
 *
 * Handles interrupts from GPIO Port H, specifically for Hall Effect Sensor B.
 * This function is called on both rising and falling edges of the sensor
 * signal. Updates the Hall Effect Sensor state and increments the transition
 * counter.
 *
 * @note This is an interrupt service routine (ISR)
 * @see hallEffectState, hallEffect_Count
 */
void xGPIOHandlerH(void)
{
  uint32_t intStatus = GPIOIntStatus(GPIO_PORTH_BASE, true);
  GPIOIntClear(GPIO_PORTH_BASE, intStatus);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int readGPIO = GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) >> 2;
  hallEffectState = setBitTo(hallEffectState, 1, readGPIO);
  hallEffect_Count += readGPIO;
  xSemaphoreGiveFromISR(hallEffectFlag, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief GPIO Port N Interrupt Handler
 *
 * Handles interrupts from GPIO Port N, specifically for Hall Effect Sensor C.
 * This function is called on both rising and falling edges of the sensor
 * signal. Updates the Hall Effect Sensor state and increments the transition
 * counter.
 *
 * @note This is an interrupt service routine (ISR)
 * @see hallEffectState, hallEffect_Count
 */
void xGPIOHandlerN(void)
{
  uint32_t intStatus = GPIOIntStatus(GPIO_PORTN_BASE, true);
  GPIOIntClear(GPIO_PORTN_BASE, intStatus);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int readGPIO = GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) >> 2;
  hallEffectState = setBitTo(hallEffectState, 2, readGPIO);
  hallEffect_Count += readGPIO;
  xSemaphoreGiveFromISR(hallEffectFlag, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief GPIO Port M Interrupt Handler
 *
 * Handles interrupts from GPIO Port M, specifically for Hall Effect Sensor A.
 * This function is called on both rising and falling edges of the sensor
 * signal. Updates the Hall Effect Sensor state and increments the transition
 * counter.
 *
 * @note This is an interrupt service routine (ISR)
 * @see hallEffectState, hallEffect_Count
 */
void xGPIOHandlerM(void)
{
  uint32_t intStatus = GPIOIntStatus(GPIO_PORTM_BASE, true);
  GPIOIntClear(GPIO_PORTM_BASE, intStatus);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int readGPIO = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3) >> 3;
  hallEffectState = setBitTo(hallEffectState, 0, readGPIO);
  hallEffect_Count += readGPIO;
  xSemaphoreGiveFromISR(hallEffectFlag, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Timer 1A interrupt handler for RPM ramping
 *
 * Updates softSetRPM gradually toward setRPM target.
 * Handles both normal acceleration and emergency stop deceleration.
 * Called at 4Hz rate for smooth speed transitions.
 *
 * @note ISR function - implements soft start/stop functionality
 * @see setRPM, softSetRPM, estop
 */
void xTimer1AHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  double internalSteps = steps;
  double setRPM = rpmData.values.raw;
  if (motorState == 0 && setRPM <= 0)
  {
    return;
  }
  motorState = 1;
  if (estop)
  {
    rpmData.values.raw = 0;
    internalSteps = esteps;
  }
  if (setRPM - internalSteps <= softSetRPM &&
      softSetRPM <= setRPM + internalSteps)
  {
    softSetRPM = setRPM;
    if (setRPM <= 0)
    {
      motorState = 0;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return;
  }
  if (softSetRPM > setRPM)
  {
    softSetRPM -= internalSteps;
  }
  else
  {
    softSetRPM += internalSteps;
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief ADC1 Sequence 0 Interrupt Handler
 *
 * Handles ADC conversion complete interrupts for current sensing.
 * Notifies the power monitoring task when new ADC data is available.
 *
 * @note This is an interrupt service routine (ISR)
 * @see powerTask, powerTaskHandle
 */
void xADC1Sequence0(void)
{
  ADCIntClear(ADC1_BASE, 0);
  uint32_t raw[2];
  ADCSequenceDataGet(ADC1_BASE, 0, raw);

  uint32_t nextHead = (adcHead + 1) % MAXBUFFER;

  if (nextHead == adcTail)
  {
    // Buffer full, drop oldest
    adcTail = (adcTail + 1) % MAXBUFFER;
  }

  adcBuffer[adcHead].ch0 = raw[0];
  adcBuffer[adcHead].ch4 = raw[1];
  adcHead = nextHead;
}

/**
 * @brief Create Motor Control Tasks
 *
 * Initializes the motor control system by setting up GPIO, ADC, and motor
 * library. Creates all necessary FreeRTOS tasks for motor control, speed
 * control, and power monitoring.
 *
 * @return int Status code (0 = success, -1 = failure on HallEffectFlag, -2 =
 * failure on estopMutex)
 *
 * @note This function should be called once during system initialization
 * @see motorTask, speedControllerTask, powerTask
 */
int CreateMotorTask(void)
{
  setupGPIO();
  setupADC();
  setupTimers();
  initMotorLib(PERIOD);
  enableMotor();
  setDuty(5);
  hallEffectFlag = xSemaphoreCreateBinary();
  if (hallEffectFlag == NULL)
  {
    return -1;
  }

  estopMutex = xSemaphoreCreateMutex();
  if (estopMutex == NULL)
  {
    return -2;
  }

  rpmData.mutex = xSemaphoreCreateMutex();
  if (rpmData.mutex == NULL){
      return -3;
  }
  powerData.mutex = xSemaphoreCreateMutex();
  if (powerData.mutex == NULL){
      return -4;
  }
  pwmData.mutex = xSemaphoreCreateMutex();
  if (pwmData.mutex == NULL){
      return -5;
  }

  maxCurrentLimit.mutex = xSemaphoreCreateMutex();
  if (maxCurrentLimit.mutex == NULL)
  {
    return -6;
  }

  UARTprintf("Creating Motor Tasks\n");
  xTaskCreate(motorTask, "Motor Task", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 10, NULL);
  xTaskCreate(speedControllerTask, "Controller Task", configMINIMAL_STACK_SIZE,
              NULL, tskIDLE_PRIORITY + 9, NULL);
  xTaskCreate(powerTask, "Power Task", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 8, &powerTaskHandle);

  // Debug task
  // xTaskCreate(adcScanTask, "ADC Task", configMINIMAL_STACK_SIZE, NULL,
  //         tskIDLE_PRIORITY + 7, &powerTaskHandle);

  // xTaskCreate(motorTestTask, "Motor Test Task", configMINIMAL_STACK_SIZE,
  // NULL,
  //         tskIDLE_PRIORITY + 1, NULL);
  return 0;
}

/**
 * @brief Decode Hall effect sensor state and update motor commutation
 *
 * Implements 6-step BLDC commutation based on 3-bit Hall sensor state.
 * Updates motor phases through motorlib updateMotor() function.
 * Includes kickstart logic for sensor state 0x00.
 *
 * @note Only operates when motorState is enabled
 * @see hallEffectState, motorState, updateMotor
 */
void hallEffectDecode()
{
  static bool flipMotorState = 0;
  if (!motorState)
  {
    stopMotor(false);
    disableMotor();
    flipMotorState = motorState;
    return;
  }
  if (flipMotorState != motorState)
  {
    enableMotor();
    flipMotorState = motorState;
  }
  switch (hallEffectState)
  {
  case 0x00: // Kick Start
  case 0x01: // 001
  case 0x05: // 101
    updateMotor(1, 0, 0);
    break;
  case 0x02: // 010
  case 0x03: // 011
    updateMotor(0, 1, 0);
    break;
  case 0x04: // 100
  case 0x06: // 110
  default:
    updateMotor(0, 0, 1);
    break;
  }
}

/**
 * @brief Poll Hall effect sensors and update motor (fallback method)
 *
 * Reads all three Hall sensors directly from GPIO and updates commutation.
 * Used as backup when interrupt-driven updates fail or timeout.
 * Automatically calls hallEffectDecode() after reading sensors.
 *
 * @note Fallback for interrupt-driven commutation
 * @see hallEffectDecode, motorTask
 */
void checkMotor()
{
  if (!motorState)
  {
    stopMotor(false);
    disableMotor();
    return;
  }
  hallEffectState = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3) >> 2 |
                    GPIOPinRead(GPIO_PORTH_BASE, GPIO_PIN_2) |
                    GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_2) >> 2;
  hallEffectDecode();
}

/*
 * @brief Getter Function for safe getting of estop state
 *
 */
uint8_t eStopGetter(TickType_t blockingTime){
    bool returnedValue;
    if (xSemaphoreTake(estopMutex, blockingTime) == pdTRUE){
        returnedValue = estop;
        xSemaphoreGive(estopMutex);
        return returnedValue;
    }
    return -1;
}

/*
 * @brief Setter Function for setting the estop state
 *
 */
uint8_t eStopSetter(bool set, TickType_t blockingTime){
    if (xSemaphoreTake(estopMutex, blockingTime) == pdTRUE){
        estop = set;
        xSemaphoreGive(estopMutex);
        return set;
    }
    return -1;
}



/**
 * @brief Calculate motor RPM from Hall effect transitions
 *
 * Computes RPM based on Hall effect transition count and time delta.
 * Uses 12 transitions per revolution for calculation.
 * Scales result to RPM units with time compensation.
 *
 * @param dt Time delta in ticks since last calculation
 * @return double Motor RPM
 * @see hallEffect_Count, pdMS_TO_TICKS
 */
double RPMCalculate(TickType_t dt)
{
  double deltaScaler = pdMS_TO_TICKS(1000) / ((double)dt);
  return (((double)hallEffect_Count) / 12) * 60 * deltaScaler;
}

/**
 * @brief PID speed controller for motor
 *
 * Implements proportional-integral-derivative control for speed regulation.
 * Calculates PWM duty cycle based on RPM error and PID gains.
 * Includes special handling for startup and reset conditions.
 *
 * @param RPM Current motor RPM measurement
 * @param dt Time delta in milliseconds since last update
 * @return int PWM duty cycle (0-PERIOD), -1 on error
 * @note Special case: RPM=0, dt=-1 resets integral term
 * @see setRPM, Kp, Ki, Kd, setDuty
 */
uint16_t PIDController(double RPM, double dt)
{
  static double ki_error = 0;
  static double prev_error = 0;
  double kp = Kp;
  double ki = Ki;
  double kd = Kd;
  if (RPM == 0 && dt == -1)
  {
    ki_error = 0;
    prev_error = 0;
    return 0;
  }
  if (softSetRPM == 0)
  {
    ki_error = 0;
    prev_error = 0;
    return 0;
  }
  if (RPM < 0 || dt <= 0)
  { // Error
    return -1;
  }
  if (RPM < 100)
  { // Helps with a gradial start
    return 16;
  }

  double error = softSetRPM - RPM;
  ki_error += error * dt;
  int output =
      (int)(kp * error + ki * ki_error + kd * ((error - prev_error) / dt));
  if (output < 0)
  {
    output = 5;
  }
  if (output > PERIOD)
  {
    output = PERIOD;
  };
  return (uint16_t)output;
}

/**
 * @brief Speed Controller Task
 *
 * Implements PID (Proportional-Integral-Derivative) speed control loop.
 * Calculates motor RPM from Hall Effect sensor transitions and
 * adjusts motor PWM duty cycle to maintain target speed.
 *
 * @param _ Unused parameter (required by FreeRTOS task signature)
 *
 * @note This is a FreeRTOS task function that runs continuously, please start
 * by calling CreateMotorTask()
 * @see RPMCalculate, PIDController
 */
static void speedControllerTask(void *_)
{
  double RPM = 0;
  sharedValues *rpmPointer = &rpmData;
  sharedValues *pwmPointer = &pwmData;
  static double lastRPM = 0;
  TickType_t lastTick = 0;
  vTaskDelay(pdMS_TO_TICKS(1000));
  for (;;)
  {
    TickType_t currentTick = xTaskGetTickCount();
    TickType_t deltaTime = currentTick - lastTick;
    lastTick = currentTick;
    RPM = RPMCalculate(deltaTime);
    uint16_t output = PIDController(RPM, (double)TICKS_TO_MS(deltaTime));
    if (output >= 0)
        setDuty((uint16_t)output);
    currentTick = xTaskGetTickCount();
    hallEffect_Count = 0;

    uint8_t errorFlag = setterVal(rpmPointer,RPM,1, 100);
    uint32_t rpmDiff = fabs((uint32_t)RPM - (uint32_t)lastRPM);
    if (rpmDiff > 25 && errorFlag >= 0)
    { 
      lastRPM = RPM; // Update for next iteration
      xQueueSend(eventQueue, &rpmPointer, 0);
    }
    setterVal(pwmPointer, output, 1, 100);
    vTaskDelayUntil(&currentTick, pdMS_TO_TICKS(100));
  }
}

/**
 * @brief Motor Control Task
 *
 * Main motor control task that waits for Hall Effect sensor interrupts
 * and updates motor commutation accordingly. Falls back to polling if
 * no interrupt is received within the timeout period.
 *
 * @param _ Unused parameter (required by FreeRTOS task signature)
 *
 * @note This is a FreeRTOS task function that runs continuously, please start
 * by calling CreateMotorTask()
 * @see hallEffectDecode, checkMotor
 */
static void motorTask(void *_)
{
  for (;;)
  {
    if (xSemaphoreTake(hallEffectFlag, pdMS_TO_TICKS(500)) == pdPASS)
    {
      hallEffectDecode();
    }
    else
    {
      checkMotor();
    }
  }
}

/**
 * @brief Read Current from ADC Value
 *
 * Converts ADC reading to actual current value using the current
 * sensor specifications and calibration parameters.
 *
 * @param adc_value Raw ADC reading (0-4095 for 12-bit ADC)
 * @return double Current in Amperes
 *
 * @note Uses 7mΩ sense resistor and 10x gain current sensor
 * @see VREF, R_SENSE, GAIN, ADC_MAX
 */
double readCurrent(uint16_t adc_value)
{
  const double VREF = 3.3;
  const double R_SENSE = 0.007;  // 7mOhm
  const double GAIN = 10;        // Gain of the current sensor
  const double ADC_MAX = 4095.0; // 12-bit ADC max value

  // Based on formula from motor datasheet at 8.3.4.1 for calculating current
  double v_sox =
      ((double)(adc_value) / ADC_MAX) * VREF; // Scale adc value to voltage
  double numerator = (VREF / 2) - v_sox;      // Vout = Vref/2 - Vout
  double denominator = GAIN * R_SENSE;        // Self explanatory
  double current =
      fabs(numerator / denominator); // Current = Vout / (Gain * R_Sense)

  return current; // Return the absolute value of the current
}

/**
 * @brief Estimate Total Motor Power
 *
 * Calculates total motor power consumption from voltage and
 * current measurements from two motor phases.
 *
 * @param voltage Motor supply voltage in Volts
 * @param current_a Current in phase A in Amperes
 * @param current_b Current in phase B in Amperes
 * @return double Estimated total power in Watts
 *
 * @note Multiplies by 1.3 to account for unmeasured third phase
 */
double estimatePower(double voltage, double current_a, double current_b)
{
  double power = voltage * (current_a + current_b); // P = V * (Ia + Ib)
  return 1.3 * power;                               // We only have two phases so by rights we should multiply
                                                    // by 1.3 for the extra third
}

/**
 * @brief Power Monitoring Task
 *
 * Monitors motor power consumption by reading current from two motor phases
 * via ADC. Applies filtering and calculates total power consumption.
 *
 * @param pvParameters Task parameters (unused)
 *
 * @note This is a FreeRTOS task function that runs continuously
 * @see readCurrent, estimatePower
 */
void powerTask(void *pvParameters)
{
  sharedValues *dataPointer = &powerData;
  bool settlingFlag = 0;
  // Accumulation variables for higher accuracy
  double sum_ch0 = 0.0;
  double sum_ch4 = 0.0;
  uint32_t sample_count = 0;

  UARTprintf("Power Task Created\n");

  vTaskDelay(pdMS_TO_TICKS(100)); // Wait for the system to stabilize

  ADCProcessorTrigger(ADC1_BASE, 0); // Start the ADC processor
  UARTprintf("ADC Triggered\n");
  setterVal(&maxCurrentLimit, 833, 0, portMAX_DELAY); // Set default max current limit to 600mA


  for (;;)
  {
    while (adcTail != adcHead)
    {
      uint16_t ch0 = adcBuffer[adcTail].ch0;
      uint16_t ch4 = adcBuffer[adcTail].ch4;
      adcTail = (adcTail + 1) % MAXBUFFER;

      sum_ch0 += ch0;
      sum_ch4 += ch4;
      sample_count++;
    }

    if (sample_count > 0)
    {
      static double lastPower = 0;
      double avg_ch0 = sum_ch0 / sample_count;
      double avg_ch4 = sum_ch4 / sample_count;

      double current_a_raw = readCurrent((uint16_t)adcBuffer[0].ch0);
      double current_b_raw = readCurrent((uint16_t)adcBuffer[0].ch4);

      double raw_instantaneous_power = estimatePower(24.0, current_a_raw, current_b_raw);

      filtered_ch0 = alpha * avg_ch0 + (1 - alpha) * filtered_ch0;
      filtered_ch4 = alpha * avg_ch4 + (1 - alpha) * filtered_ch4;

      double current_a = readCurrent((uint16_t)filtered_ch0);
      double current_b = readCurrent((uint16_t)filtered_ch4);
      

      val maxcurrentlimit;
      uint8_t errorFlag = getter(&maxCurrentLimit, &maxcurrentlimit, portMAX_DELAY); // Get the current limit


      // Check if the current exceeds the maximum limit
      if (errorFlag == 0)
      {
        if (!settlingFlag && (current_a + current_b) * 1000 > maxcurrentlimit.raw){
          continue;
        } else {
          settlingFlag = 1; 
        }
        if ((current_a + current_b) * 1000 > maxcurrentlimit.raw)
        {
          // UARTprintf("[Power] Current limit exceeded: %d mA, (Total Current: %d.%03d) stopping motor\n",
                    //  (int)maxcurrentlimit.raw, (int)(current_a + current_b), (int)((current_a + current_b - (int)(current_a + current_b)) * 1000));
          eStopSetter(1, portMAX_DELAY);
        }
      }
      setterVal(&maxCurrentLimit, (current_a + current_b) * 1000, 1, portMAX_DELAY);
      // UARTprintf(
      //     "Power: %d.%03d, Current A: %d.%03d, Current B: %d.%03d, Total Current: %d.%03d Max Limit: %d mA\n",
      //     (int) raw_instantaneous_power, (int)((raw_instantaneous_power - (int)raw_instantaneous_power) * 1000),
      //     (int)current_a, (int)((current_a - (int)current_a) * 1000),
      //     (int)current_b, (int)((current_b - (int)current_b) * 1000),
      //     (int)(current_a + current_b),
      //     (int)((current_a + current_b - (int)(current_a + current_b)) * 1000),
      //     (int)maxcurrentlimit.raw);

      double instantaneous_power = estimatePower(24.0, current_a, current_b);
      val msg = {
          .raw = raw_instantaneous_power,
          .filtered = instantaneous_power
      };
      setter(dataPointer, msg, portMAX_DELAY);
      xQueueSend(eventQueue, &dataPointer, 0);

      sum_ch0 = 0.0;
      sum_ch4 = 0.0;
      sample_count = 0;
      
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Back off to let system breathe
  }
}

/**
 * @brief Setup GPIO for Motor Control
 *
 * Configures GPIO pins for Hall Effect sensors and ADC inputs.
 * Sets up interrupt handling for Hall Effect sensor state changes.
 * Enables pull-up resistors and configures interrupt types.
 *
 * @note Configures ports N, H, M for Hall sensors and E, D for ADC
 * @see xGPIOHandlerH, xGPIOHandlerN, xGPIOHandlerM
 */
void setupGPIO(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

  SysCtlPeripheralEnable(
      SYSCTL_PERIPH_GPIOE); // These should be the GPIO for the ADC
  SysCtlPeripheralEnable(
      SYSCTL_PERIPH_GPIOD); // These should be the GPIO for the ADC

  while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPION) &&
           SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH) &&
           SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM) &&
           SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)))
  {
  }

  // Configure as ADC Inputs
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // PE3
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7); // PD7

  GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_2); // 2
  GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2); // 2
  GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3); // 3

  GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPU);

  GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
  GPIOIntTypeSet(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
  GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);

  GPIOIntEnable(GPIO_PORTN_BASE, GPIO_INT_PIN_2);
  GPIOIntEnable(GPIO_PORTH_BASE, GPIO_INT_PIN_2);
  GPIOIntEnable(GPIO_PORTM_BASE, GPIO_INT_PIN_3);

  IntEnable(INT_GPION);
  IntEnable(INT_GPIOH);
  IntEnable(INT_GPIOM);
}

/**
 * @brief Configure Timer1 for RPM ramping control
 *
 * Sets up Timer1A for periodic interrupts at 4Hz rate.
 * Used for gradual RPM changes and soft start/stop functionality.
 * Calculates step sizes for normal and emergency stop ramping.
 *
 * @note Timer runs at system clock / 4 for smooth transitions
 * @see xTimer1AHandler, steps, esteps
 */
void setupTimers()
{
  // Timer
  steps = 500 / 100;
  esteps = 1000 / 100;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1))
  {
  }
  TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / 100);
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(INT_TIMER1A);
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER1_BASE, TIMER_A);

  //
}

/**
 * @brief Setup ADC for Current Sensing
 *
 * Configures ADC1 for dual-channel current sensing.
 * Sets up sequencer 0 to read from channels 0 and 4 with
 * processor triggering and interrupt generation.
 *
 * @note Configures 2-step sequence: CH0 → CH4 with interrupt
 * @see xADC1Sequence0
 */
void setupADC()
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  while (
      !SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1) &&
      !SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2))
  {
  }

  TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER2_BASE, TIMER_A, g_ui32SysClock / 150);
  TimerControlTrigger(TIMER2_BASE, TIMER_A, true);
  ADCSequenceDisable(ADC1_BASE, 0);

  ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_TIMER, 0);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH0);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 1,
                           ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);

  ADCSequenceEnable(ADC1_BASE, 0);
  ADCIntClear(ADC1_BASE, 0);
  ADCIntEnable(ADC1_BASE, 0);
  IntEnable(INT_ADC1SS0);
  TimerEnable(TIMER2_BASE, TIMER_A);
}

