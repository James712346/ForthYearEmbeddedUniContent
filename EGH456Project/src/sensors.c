/**
 * @file sensor_task.c
 * @brief Sensor subsystem implementation for FreeRTOS-based embedded system
 * @details This file contains implementations for various sensors including OPT3001 light sensor,
 *          VL53L0X distance sensor, SHT31 temperature/humidity sensor, and BMI160 IMU sensor.
 *          All sensors communicate via I2C with semaphore-based synchronization.
 * @author Samuel Smith
 * @version 1.0
 * @defgroup sensor Sensor Subsystem
 * @brief Complete sensor subsystem for environmental and motion sensing
 * @{
 */

/* Standard includes. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Kernel includes. */
#include <FreeRTOS.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <task.h>

/* Other includes*/
#include "utils/uartstdio.h"
#include "opt3001.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "rtos_hw_drivers.h"
#include "sensor_task.h"
#include "driverlib/i2c.h"
#include "vl53l0x.h"
#include "bmi160.h"
#include <shared.h>

// Prototypes of Functions
static void prvInitSensorOPT3001(void);
static void vLightSensorTask(void *pvParameters);
// we aint using this shit no more and i hate this stupid warning
// static void vDistanceSensorTask(void *pvParameters);

/** @brief Shared Light data */
sharedValues lightData;

/** @brief Shared temp data */
sharedValues tempData;
/** @brief Shared Hum data */
sharedValues humiData;
/** @brief Binary semaphore for I2C bus synchronization */
SemaphoreHandle_t xI2CSemaphore = NULL;


/**
 * @brief I2C2 interrupt handler
 * @details Handles I2C2 peripheral interrupts and releases the I2C semaphore
 *          to unblock waiting tasks. Uses ISR-safe semaphore operations.
 * @note This function is called from interrupt context
 */
void I2C2IntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    I2CMasterIntClear(I2C2_BASE); // Clear the interrupt

    xSemaphoreGiveFromISR(xI2CSemaphore, &xHigherPriorityTaskWoken); // Give the semaphore to unblock the task

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Yield to the higher priority task if needed
}

/**
 * @brief Read temperature and humidity from SHT31 sensor
 * @details Sends measurement command to SHT31, waits for conversion, then reads
 *          6 bytes of data (temp MSB, temp LSB, temp CRC, hum MSB, hum LSB, hum CRC)
 * @param[out] temperature Pointer to store temperature value in Celsius
 * @param[out] humidity Pointer to store humidity value in percentage
 * @return true if reading successful, false if timeout or communication error
 * @note Uses I2C semaphore for bus synchronization
 * @warning Assumes I2C semaphore is properly initialized
 */
bool sht31_read_temperature_humidity(float *temperature, float *humidity)
{
    uint8_t cmd[] = {0x24, 0x00};

    // UARTprintf("SHT31: Sending measurement command...\n");
    I2CMasterSlaveAddrSet(I2C_BASE, SHT31_ADDR, false);
    I2CMasterDataPut(I2C_BASE, cmd[0]);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2CSemaphore, I2C_TIMEOUT_TICKS) != pdPASS)
    {
        // UARTprintf("SHT31: Timeout on first command byte\n");
        return false;
    }

    I2CMasterDataPut(I2C_BASE, cmd[1]);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    if (xSemaphoreTake(xI2CSemaphore, I2C_TIMEOUT_TICKS) != pdPASS)
    {
        // UARTprintf("SHT31: Timeout on second command byte\n");
        return false;
    }

    // UARTprintf("SHT31: Command sent. Waiting for conversion...\n");
    vTaskDelay(pdMS_TO_TICKS(20)); // Max conversion time = 15ms

    I2CMasterSlaveAddrSet(I2C_BASE, SHT31_ADDR, true);
    uint8_t data[6];
    for (int i = 0; i < 6; i++)
    {
        uint32_t cmd = (i == 0) ? I2C_MASTER_CMD_BURST_RECEIVE_START : (i == 5) ? I2C_MASTER_CMD_BURST_RECEIVE_FINISH
                                                                                : I2C_MASTER_CMD_BURST_RECEIVE_CONT;

        I2CMasterControl(I2C_BASE, cmd);
        if (xSemaphoreTake(xI2CSemaphore, I2C_TIMEOUT_TICKS) != pdPASS)
        {
            // UARTprintf("SHT31: Timeout reading byte %d\n", i);
            return false;
        }

        data[i] = I2CMasterDataGet(I2C_BASE);
    }

    // UARTprintf("SHT31: Raw data = %02X %02X | %02X | %02X %02X | %02X\n",
    //            data[0], data[1], data[2], data[3], data[4], data[5]);

    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t hum_raw = (data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    *humidity = 100.0f * ((float)hum_raw / 65535.0f);

    // int32_t temp_scaled = (int32_t)(*temperature * 100);
    // int32_t hum_scaled = (int32_t)(*humidity * 100);

    // UARTprintf("SHT31: Temp = %d.%02d C, Hum = %d.%02d %%\n",
    //            temp_scaled / 100, abs(temp_scaled % 100),
    //            hum_scaled / 100, hum_scaled % 100);
    //
    return true;
}

/**
 * @brief BMI160 I2C read function callback
 * @details Callback function for BMI160 driver to read data from sensor registers
 * @param[in] dev_id I2C device address
 * @param[in] reg_addr Register address to read from
 * @param[out] data Buffer to store read data
 * @param[in] len Number of bytes to read
 * @return BMI160_OK on success, BMI160_E_COM_FAIL on communication failure
 * @note Uses I2C semaphore for bus synchronization
 */
int8_t bmi160_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // Send register address (write phase)
    I2CMasterSlaveAddrSet(I2C_BASE, dev_id, false); // false = write
    I2CMasterDataPut(I2C_BASE, reg_addr);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    if (xSemaphoreTake(xI2CSemaphore, I2C_TIMEOUT_TICKS) != pdPASS)
    {
        // UARTprintf("BMI160: Timeout writing reg addr 0x%02X\n", reg_addr);
        return BMI160_E_COM_FAIL;
    }

    // Switch to read mode
    I2CMasterSlaveAddrSet(I2C_BASE, dev_id, true); // true = read

    for (uint16_t i = 0; i < len; i++)
    {
        uint32_t cmd;
        if (i == 0)
            cmd = (len == 1) ? I2C_MASTER_CMD_SINGLE_RECEIVE : I2C_MASTER_CMD_BURST_RECEIVE_START;
        else if (i == len - 1)
            cmd = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
        else
            cmd = I2C_MASTER_CMD_BURST_RECEIVE_CONT;

        I2CMasterControl(I2C_BASE, cmd);
        if (xSemaphoreTake(xI2CSemaphore, I2C_TIMEOUT_TICKS) != pdPASS)
        {
            // UARTprintf("BMI160: Timeout receiving byte %d\n", i);
            return BMI160_E_COM_FAIL;
        }

        data[i] = I2CMasterDataGet(I2C_BASE);
    }

    return BMI160_OK;
}

/**
 * @brief BMI160 I2C write function callback
 * @details Callback function for BMI160 driver to write data to sensor registers
 * @param[in] dev_id I2C device address
 * @param[in] reg_addr Register address to write to
 * @param[in] data Buffer containing data to write
 * @param[in] len Number of bytes to write
 * @return BMI160_OK on success, BMI160_E_COM_FAIL on communication failure
 * @note Uses I2C semaphore for bus synchronization
 */
int8_t bmi160_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // Write register address first
    I2CMasterSlaveAddrSet(I2C_BASE, dev_id, false); // false = write

    I2CMasterDataPut(I2C_BASE, reg_addr);
    I2CMasterControl(I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    if (xSemaphoreTake(xI2CSemaphore, I2C_TIMEOUT_TICKS) != pdPASS)
    {
        // UARTprintf("BMI160: Timeout on reg_addr send\n");
        return BMI160_E_COM_FAIL;
    }

    // Send all data bytes
    for (uint16_t i = 0; i < len; i++)
    {
        I2CMasterDataPut(I2C_BASE, data[i]);

        // Choose proper control command
        uint32_t cmd = (i == len - 1) ? I2C_MASTER_CMD_BURST_SEND_FINISH : I2C_MASTER_CMD_BURST_SEND_CONT;
        I2CMasterControl(I2C_BASE, cmd);

        if (xSemaphoreTake(xI2CSemaphore, I2C_TIMEOUT_TICKS) != pdPASS)
        {
            // UARTprintf("BMI160: Timeout on data byte %d\n", i);
            return BMI160_E_COM_FAIL;
        }
    }

    return BMI160_OK;
}

/**
 * @brief BMI160 delay function callback
 * @details Callback function for BMI160 driver to implement delays
 * @param[in] ms Delay time in milliseconds
 * @note Uses FreeRTOS vTaskDelay for non-blocking delay
 */
void bmi160_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Light sensor task implementation
 * @details Continuously reads OPT3001 light sensor, applies filtering, and sends data to queue
 * @param[in] pvParameters Task parameters (unused)
 * @note Task runs at 2Hz (500ms delay)
 * @note Applies exponential moving average filter with alpha = 0.2
 */
static void vLightSensorTask(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(150));
    sharedValues *dataPointer = &lightData;

    float lux;
    uint16_t raw;
    float luxFiltered = 0.0f;
    float alpha = 0.2f; // Smoothing factor for the filter
    /* Initialize the sensor. */
    prvInitSensorOPT3001();
    // TickType_t start_tick,end_tick;
    //  uint32_t elapsed_ms;

    /* Shit works, lets cook*/

    while (1)
    {
        // start_tick = xTaskGetTickCount(); // Get the current tick count

        if (sensorOpt3001Read(&raw))
        {
            sensorOpt3001Convert(raw, &lux);
            luxFiltered = alpha * lux + (1 - alpha) * luxFiltered; // Exponential moving average
            // UARTprintf("%d.%02d Lux\n", (int)lux, (int)(lux * 100) % 100);
            val msg = {
                .filtered = (double)luxFiltered,
                .raw = (double)lux,
                
            };

            setter(dataPointer, msg, portMAX_DELAY);
            xQueueSend(eventQueue, &dataPointer, 0);
        }
        // end_tick = xTaskGetTickCount(); // Get the tick count after the read
        // elapsed_ms = (end_tick - start_tick) * portTICK_PERIOD_MS; // Calculate elapsed time in ms
        // UARTprintf("Elapsed time: %d ms\n", elapsed_ms); // Print elapsed time -  1ms
        vTaskDelay(pdMS_TO_TICKS(500)); // 2Hz delay
    }
}


/**
 * @brief SHT31 temperature and humidity sensor task
 * @details Continuously reads temperature and humidity from SHT31 sensor
 * @param[in] pvParameters Task parameters (unused)
 * @note Task runs at 1Hz (1000ms delay)
 * @note Includes error handling with retry mechanism
 */
void sht31_task(void *pvParameters)
{
    sharedValues *tempPointer = &tempData;
    sharedValues *humiPointer = &humiData;
    vTaskDelay(pdMS_TO_TICKS(150)); // Allow time for the sensor to power up
    float temperature = 0;
    float humidity = 0;

    float filtered_temperature = 0.0f;
    float filtered_humidity = 0.0f;
    float alpha = 0.2f; // Smoothing factor for the filter

    // TickType_t start_tick,end_tick;
    // uint32_t elapsed_ms;

    while (1)
    {

        if (!sht31_read_temperature_humidity(&temperature, &humidity))
        {
            vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 1 second before retrying
        }
        filtered_temperature = alpha * temperature + (1 - alpha) * filtered_temperature;
        filtered_humidity = alpha * humidity + (1 - alpha) * filtered_humidity;
        val tempMsg = {
            .filtered = (double)filtered_temperature,
            .raw = (double)temperature, // Raw temperature reading
        };
        val humiMsg = {
            .raw = (double)humidity,    // Raw humidity reading
            .filtered = (double)filtered_humidity
        };
        setter(tempPointer, tempMsg, portMAX_DELAY);
        setter(humiPointer, humiMsg, portMAX_DELAY);
        xQueueSend(eventQueue, &tempPointer, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Every 1Hz
    }
}

/** @} */

/**
 * @brief Create and initialize all sensor tasks
 * @details Creates FreeRTOS tasks for sensor reading, initializes I2C semaphore and sensor data queue
 * @note Creates binary semaphore for I2C bus synchronization
 * @note Creates queue for inter-task sensor data communication
 * @note Currently only BMI160 task is enabled, others are commented out
 * @warning Function returns early if semaphore creation fails
 */
uint8_t CreateSensorTask(void)
{

    // Create the I2C Semaphore
    xI2CSemaphore = xSemaphoreCreateBinary();
    if (xI2CSemaphore == NULL)
    {
        return -1;
    }

    //  Power should be priority 5, but we need to test the power first
    //  Speed should be priority 4, but we need to test the speed first
    lightData.mutex = xSemaphoreCreateMutex();
    if (lightData.mutex == NULL)
    {
        return -2;
    }
    tempData.mutex = xSemaphoreCreateMutex();
    if (tempData.mutex == NULL)
    {
        return -3;
    }
    humiData.mutex = xSemaphoreCreateMutex();
    if (humiData.mutex == NULL)
    {
        return -4;
    }

    xTaskCreate(sht31_task, "SHT31", configMINIMAL_STACK_SIZE + 128, NULL, tskIDLE_PRIORITY + 3, NULL);       // Task priority will be set to 1 after initialization
    xTaskCreate(vLightSensorTask, "LightSensor", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL); // Task priority will be set to 2 after initialization
    // xTaskCreate(vDistanceSensorTask, "DistanceSensor", configMINIMAL_STACK_SIZE +128, NULL, tskIDLE_PRIORITY + 4, NULL);
    // xTaskCreate(bmi160_task, "BMI160", configMINIMAL_STACK_SIZE + 128, NULL, tskIDLE_PRIORITY + 1, NULL); // Task priority will be set to 6 after initialization
    return 0;
}

/**
 * @brief Initialize OPT3001 light sensor
 * @details Initializes OPT3001 sensor and performs self-test with retry mechanism
 * @note Blocks until sensor test passes or manual intervention
 * @note Uses 1-second delay between retry attempts
 * @warning Function may block indefinitely if sensor hardware fails
 */
static void prvInitSensorOPT3001(void)
{
    /* Initialize the sensor. */
    if (sensorOpt3001Init())
    {
        UARTprintf("Sensor Initialized\n");
    }
    else
    {
        UARTprintf("Sensor Initialization Failed\n");
    }

    bool success = sensorOpt3001Test();
    while (!success)
    {
        UARTprintf("Test Failed, Trying again\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second before retrying
        success = sensorOpt3001Test();
    }

    if (success)
    {
        UARTprintf("All Sensor tests passed! \n\n");
    }
}

/** @} */
