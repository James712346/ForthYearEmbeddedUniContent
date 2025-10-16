#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
uint8_t eStopGetter(TickType_t blockingTime);
uint8_t eStopSetter(bool set, TickType_t blockingTime);