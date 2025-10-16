#include <stddef.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include "semphr.h"
#define MAXQUEUESIZE 20

extern QueueHandle_t eventQueue;

typedef struct {
    double filtered;
    volatile double raw;
} val;

typedef struct {
    SemaphoreHandle_t mutex;
    val values;
} sharedValues;

uint8_t createQueue();
uint8_t getter(sharedValues *dataPoint, val* buff, TickType_t blockingTime);
uint8_t setter(sharedValues *dataPoint, val values, TickType_t blockingTime);
uint8_t setterVal(sharedValues *dataPoint, double value, bool setting, TickType_t blockingTime);
double getterMaxCurrentLimit(TickType_t blockingTime);
double setterMaxCurrentLimit(double value, TickType_t blockingTime);

// External Variable from other components
extern sharedValues lightData;
extern sharedValues humiData;
extern sharedValues tempData;
extern sharedValues rpmData;
extern sharedValues pwmData;
extern sharedValues powerData; 
extern sharedValues maxCurrentLimit;
