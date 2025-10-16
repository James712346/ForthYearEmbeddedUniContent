#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

uint8_t CreateSensorTask(void);
bool sensorOpt3001Test(void);
extern SemaphoreHandle_t xI2CSemaphore;
bool sht31_read(float *temperature, float *humidity);
int8_t bmi160_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
extern bool estop;



// Writing these backwards because of suffering
// #define OPT3001_LOW_LIMIT_REGISTER 0xFF0F // 40.95 lux (E = 0, M = 4095)
// #define OPT3001_HIGH_LIMIT_REGISTER 0xFF6F // 2620.8 lux (E = 7, M = 2202)

#define OPT_INT_GPIO_BASE GPIO_PORTM_BASE //#define OPT_INT_GPIO_BASE GPIO_PORTD_BASE
#define SHT31_ADDR         0x44
#define SHT31_MEAS_HIGHREP    0x2400
#define I2C_BASE           I2C2_BASE 
#define I2C_TIMEOUT_TICKS  pdMS_TO_TICKS(100)
#define AVG_WINDOW_SIZE 10
#define LUX_THRESHOLD 5.0f

#define TEMP_COOL_THRESHOLD 24.0f
#define TEMP_HEAT_THRESHOLD 20.0f

typedef enum {
    EVT_LIGHT,
    EVT_HVAC,
    EVT_POWER,
    EVT_MOTOR
} event_id_t;

typedef struct {
    event_id_t id;
    double raw1;
    double raw2;
    double value1; // lux, temperature or current
    double value2; // humidity if needed otherwise 0
} event_msg_t;

extern QueueHandle_t sensorQueue;

#endif /* SENSOR_TASK_H */
