/*
 * led_task
 *
 * Copyright (C) 2022 Texas Instruments Incorporated
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"

#include "drivers/opt3001.h"

#define SYSTICKS_PER_SECOND     1
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)

volatile uint32_t g_ui32SysClock;

extern SemaphoreHandle_t xI2CSemaphore;
static void SensorInitTask(void *pvParameters);
static void ReadSensor(void *pvParameters);
static void AlertTask(void);
void vCreateLEDTask(void);

void GPIOMHandler(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    GPIOIntClear(GPIO_PORTM_BASE, GPIO_PIN_6);
    xSemaphoreGiveFromISR(xI2CSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//*****************************************************************************
void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 9600, g_ui32SysClock);
    SysCtlDelay(g_ui32SysClock);
}

//*****************************************************************************
void vCreateLEDTask(void)
{
    bool success;

    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                         SYSCTL_OSC_MAIN   |
                                         SYSCTL_USE_PLL    |
                                         SYSCTL_CFG_VCO_480),
                                         120000000);

    ConfigureUART();
    UARTprintf("OPT001 Example\n");

    UARTprintf("1\n");
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));

    UARTprintf("2\n");
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);

    UARTprintf("3\n");
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    // Int Pin M6
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_6,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_6);
    IntEnable(INT_GPIOM);

    UARTprintf("3\n");
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);
    // Enable the Interrupts
    UARTprintf("4\n");
    IntMasterEnable();

    xTaskCreate(SensorInitTask, "Init", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 2, NULL);


}

//*****************************************************************************
static void SensorInitTask(void *pvParameters)
{
    UARTprintf("5\n");
    sensorOpt3001Init();
    UARTprintf("Testing OPT3001 Sensor:\n");

    // while (!sensorOpt3001Test()) {
    //     SysCtlDelay(g_ui32SysClock);
    //     UARTprintf("Test Failed, Trying again\n");
    // }

    UARTprintf("All Tests Passed!\n\n");
    xTaskCreate(ReadSensor,     "LED",  configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(AlertTask,     "AlertLED",  configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskDelete(NULL);
}

//*****************************************************************************
static void ReadSensor(void *pvParameters)
{
    bool success;
    uint16_t rawData = 0;
    float convertedLux = 0;

    while (1) {
        SysCtlDelay(g_ui32SysClock / 100);

        success = sensorOpt3001Read(&rawData);
        if (success) {
            sensorOpt3001Convert(rawData, &convertedLux);
            if (convertedLux >= 2560 || convertedLux <= 40.95){
                continue;
            }
            UARTprintf("%d.%02d Lux\n", (int)convertedLux, (int)((convertedLux - (int) convertedLux) * 100));
        }
    }
}

static void AlertTask(void){
    bool success;
    uint16_t rawData = 0;
    float convertedLux = 0;

    while (1) {
        SysCtlDelay(g_ui32SysClock / 100);
        if (xSemaphoreTake(xI2CSemaphore, portMAX_DELAY) != pdTRUE){
            continue;
        }
        success = sensorOpt3001Read(&rawData);
        if (success) {
            sensorOpt3001Convert(rawData, &convertedLux);
            if (convertedLux >= 2560){
                UARTprintf("High Light Event: %d.%02d Lux\n", (int)convertedLux, (int)((convertedLux - (int) convertedLux) * 100));
            } else {
                UARTprintf("Low Light Event: %d.%02d Lux\n", (int)convertedLux, (int)((convertedLux - (int) convertedLux) * 100));
            }
        }

    }
}
