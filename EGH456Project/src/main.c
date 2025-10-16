/**
 * @mainpage Electric Vehicle Embedded Control System
 *
 * @brief Real-time embedded software for sensing, motor control, and user interface in a battery-electric vehicle.
 *
 * This project is part of the EGH456 Assignment Description for Semester 1, 2025.
 * It involves the development of embedded software to safely monitor and control
 * an electric vehicle's core systems using the Tiva TM4C1294NCPDT microcontroller.
 *
 * The system is designed to control and monitor a 3-phase Brushless DC (BLDC) motor,
 * ensuring safe operation under various driving and fault conditions. Key functionalities include:
 * - Real-time acquisition and filtering of sensor data
 * - Monitoring of motor parameters such as rotational velocity and power
 * - Handling motor startup, braking, and emergency stop procedures
 * - Displaying critical system status and diagnostics via a user interface
 *
 * The software is built to support real-time, multitasking behavior using FreeRTOS.
 * The development hardware includes a motor testing kit with compatible sensor and driver boards,
 * interfaced directly with the microcontroller’s I/O.
 *
 * This assignment showcases embedded system engineering practices for future battery-electric and hybrid-electric vehicles,
 * aligning with industry trends in sustainability and autonomous vehicle technologies.
 */

/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include <FreeRTOS.h>
#include <timer.h>
#include <inc/hw_ints.h>
#include <task.h>
#include <semphr.h>

/* Hardware includes. */
#include <inc/hw_memmap.h>
#include <inc/hw_sysctl.h>
#include <interrupt.h>
#include <gpio.h>
#include <rom.h>

#include <rom_map.h>
#include <pin_map.h>
#include <uart.h>
#include <utils/uartstdio.h>
#include <sysctl.h>
#include <rtos_hw_drivers.h>
#include "driverlib/i2c.h"
#include "../lib/drivers/opt3001.h"
#include "../lib/drivers/i2cOptDriver.h"
#include "sensor_task.h"
#include <grlib/grlib.h>
#include "Kentec320x240x16_ssd2119_spi.h"
#include <widget.h>
#include "gui.h"
#include "touch.h"
#include <time.h>
#include <hibernate.h>
#include <inc/hw_hibernate.h>
#include "inc/hw_types.h"
#include "inc/tm4c1294ncpdt.h"
#include <shared.h>
#define RTC_INITIALIZED_FLAG 0xA5A5A5A5

/* The system clock frequency. */
volatile uint32_t g_ui32SysClock;
extern uint8_t CreateMotorTask(void);
static void SetupHardware(void);
static void ConfigureUART(void);
static void SetupRTC(void);
static void prvConfigureI2C(void);
extern void guiTask(void *params); // already declared in gui.h

/**
 * @brief Starting function, that call the initialization functions
 *
 * Set up all the subsystems, and initializes all the MircoController
 */
int main(void)
{
    SetupHardware();
    SetupRTC();
    ConfigureUART();
    uint8_t errorFlag = 0;
    errorFlag = createQueue();
    if (errorFlag != 0)
    {
        UARTprintf("Error: Motor Task returned: %d", errorFlag);
        for(;;);
    }
    errorFlag = CreateMotorTask();
    if (errorFlag != 0)
    {
        UARTprintf("Error: Motor Task returned: %d", errorFlag);
        for (;;)
            ;
    }

    errorFlag = CreateSensorTask();
    if (errorFlag != 0)
    {
        UARTprintf("Error: Motor Task returned: %d", errorFlag);
        for (;;)
            ;
    }

    errorFlag = CreateGUITask();
    if (errorFlag != 0)
    {
        UARTprintf("Error: GUI Task failed to start\n");
        for (;;)
            ;
    }
    //
    errorFlag = CreateGUISensorTask();
    if (errorFlag != 0)
    {
        UARTprintf("Error: GUI Sensor Task failed to start\n");
        for (;;)
            ;
    }

    errorFlag = CreateLEDTask();
    if (errorFlag != 0)
    {
        UARTprintf("Error: LED Task failed to start\n");
        for (;;)
            ;
    }

    IntMasterEnable();

    // CreateSensorTask(); // This will create the task for all sensors
    vTaskStartScheduler();
    for (;;)
        ;
}

/**
 * @brief Sets up the Real-Time Clock (RTC) for the hibernate module.
 *
 * This function initializes the hibernate peripheral, enables the 32kHz clock,
 * sets the counter mode to RTC, and ensures the RTC is running. If it is not
 * already running, it initializes the RTC to 0.
 */
void SetupRTC(void)
{
    HibernateEnableExpClk(SysCtlClockGet());
    HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
    HibernateRTCEnable();
    HibernateCounterMode(HIBERNATE_COUNTER_24HR);

    uint32_t rtcFlag;
    HibernateDataGet(&rtcFlag, 1); // Read HIB_DATA[0]

    if (rtcFlag != RTC_INITIALIZED_FLAG)
    {
        struct tm initTime = {
            .tm_sec = 0,
            .tm_min = 55,
            .tm_hour = 9,
            .tm_mday = 29,
            .tm_mon = 4,    // May (0-based)
            .tm_year = 125, // 2025 - 1900
            .tm_wday = 4    // Tuesday
        };

        HibernateCalendarSet(&initTime);

        // Set the flag to indicate RTC was initialized
        rtcFlag = RTC_INITIALIZED_FLAG;
        HibernateDataSet(&rtcFlag, 1); // Store into HIB_DATA[0]
    }
}

/**
 * @brief Initializes system clock and configures pinout.
 *
 * Sets up the system clock using the internal PLL and configures
 * the default pinout for the board.
 */
static void SetupHardware(void)
{
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240),
                                            configCPU_CLOCK_HZ);

    

    PinoutSet(false, false);
    prvConfigureI2C();
}

/**
 * @brief Configures UART0 for serial communication.
 *
 * Enables the required peripherals, configures the appropriate GPIO
 * pins for UART function, and sets up UART0 with a baud rate of 9600.
 */
static void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 9600, g_ui32SysClock);
}

/**
 * @brief Hook function called when a memory allocation fails.
 *
 * This function will only be called if configUSE_MALLOC_FAILED_HOOK is set to 1
 * in FreeRTOSConfig.h. It is invoked when pvPortMalloc() fails to allocate memory.
 * pvPortMalloc() is used internally by the kernel for dynamic memory allocations
 * such as when creating tasks, queues, timers, or semaphores.
 *
 * Note: The system is halted here by disabling interrupts and entering an infinite loop.
 *
 * @note This function is provided by FreeRTOS.
 */
void vApplicationMallocFailedHook(void)
{
    IntMasterDisable();
    for (;;)
        ;
}

/**
 * @brief Hook function called during each iteration of the idle task.
 *
 * This function will only be called if configUSE_IDLE_HOOK is set to 1
 * in FreeRTOSConfig.h. It runs in the context of the idle task and should not
 * contain any blocking operations. It must return to allow proper cleanup of
 * deleted tasks.
 *
 * @note This function is provided by FreeRTOS.
 */
void vApplicationIdleHook(void)
{
}

/**
 * @brief Hook function called when a stack overflow is detected.
 *
 * This function will only be called if configCHECK_FOR_STACK_OVERFLOW is
 * defined to 1 or 2 in FreeRTOSConfig.h. It provides an opportunity to
 * handle the error, such as logging or halting the system.
 *
 * @param pxTask Handle of the task that caused the stack overflow.
 * @param pcTaskName Name of the task that caused the stack overflow.
 *
 * @note This function is provided by FreeRTOS.
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    IntMasterDisable();
    for (;;)
        ;
}

/**
 * @brief Custom implementation of malloc to trap unwanted allocations.
 *
 * This implementation disables interrupts and halts the system if malloc()
 * is called, as dynamic memory allocation is not expected or allowed in
 * this configuration.
 *
 * @note This shouldn't be called
 * @param xSize Size of the memory allocation request.
 * @return This function does not return.
 */
void *malloc(size_t xSize)
{
    IntMasterDisable();
    for (;;)
        ;
}

/**
 * @brief Hook function called on every RTOS tick interrupt.
 *
 * This function will be called if configUSE_TICK_HOOK is set to 1
 * in FreeRTOSConfig.h. It runs in interrupt context, so only ISR-safe
 * FreeRTOS API functions (those ending in FromISR) should be used here.
 *
 * @note This function is provided by FreeRTOS.
 */
void vApplicationTickHook(void)
{
}

/**
 * @brief Configures the I2C peripheral for communication with the OPT3001 sensor.
 *
 * This function enables the required peripherals, configures the GPIO pins for I2C
 * functionality, and initializes the I2C master block. It also sets up an interrupt
 * handler for I2C events and creates a semaphore for I2C communication.
 */

static void prvConfigureI2C(void)
{
    /* Enable the GPIO Peripheral used by the I2C. */

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    /* Enable the I2C0 peripheral. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Setup for assignment. Please give me marks for tis Chris and Quinlan

    /* Configure the pin muxing for I2C0 functions on port B2 and B3. */

    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);

    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    /* Initialize the I2C master block. */
    I2CMasterInitExpClk(I2C2_BASE, g_ui32SysClock, false);

    I2CMasterIntEnableEx(I2C2_BASE, I2C_MASTER_INT_DATA);
    IntEnable(INT_I2C2);

    xI2CSemaphore = xSemaphoreCreateBinary(); // Create the semaphore for I2C
    if (xI2CSemaphore == NULL)
    {
        UARTprintf("Failed to create I2C semaphore\n");
        return;
    }
}
