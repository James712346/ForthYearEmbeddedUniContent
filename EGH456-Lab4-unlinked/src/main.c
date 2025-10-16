//*****************************************************************************
//
// light_opt3001.c - Example to use OPT3001
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
//
//*****************************************************************************
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
/*-----------------------------------------------------------*/

//
//Light Measurement with the OPT3001
//
// This example demonstrates the basic use of the OPT3001 light sensor
//
// The Sensors BoosterPack must be installed on BoosterPack 1 interface.
// Code changes will be needed if installed on BoosterPack 2 interface.
//
//
//*****************************************************************************


//*****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     1
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// Global variable for storage of actual system clock frequency.
//
//*****************************************************************************
volatile uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif



SemaphoreHandle_t xI2CSemaphore = NULL;

//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int main(void)
{
    xI2CSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xI2CSemaphore);
    vCreateLEDTask();


    vTaskStartScheduler();


    for (;;) {}
}
