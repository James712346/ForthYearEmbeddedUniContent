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

/******************************************************************************
 *
 * vLEDTask turns on the initial LED, configurations the buttons, and creates
 * prvProcessSwitchInputTask which handles processing the ISR result to adjust
 * the LED per the button inputs.
 *
 * prvProcessSwitchInputTask uses semaphore take to wait until it receives a
 * semaphore from the button ISR.  Once it does, then it processes the button
 * pressed.  Each time the SW1 or SW2 button is pressed, the LED index is
 * updated based on which button has been pressed and the corresponding LED
 * lights up.
 *
 * When either user switch SW1 or SW2 on the EK-TM4C1294XL is pressed, an
 * interrupt is generated and the switch pressed is logged in the global
 * variable g_pui32ButtonPressed.  Then the binary semaphore is given to
 * prvProcessSwitchInputTask before yielding to it.  This is an example of
 * using binary semaphores to defer ISR processing to a task.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "semphr.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "drivers/opt3001.h"
#define SYSTICKS_PER_SECOND     1
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)
/*-----------------------------------------------------------*/

/*
 * Time stamp global variable.
 */
volatile uint32_t g_ui32TimeStamp = 0;

/*
 * Global variable to log the last GPIO button pressed.
 */
volatile static uint32_t g_pui32ButtonPressed = NULL;

/*
 * The binary semaphore used by the switch ISR & task.
 */

volatile uint8_t ui8LEDIndex = 0;
volatile uint32_t g_ui32SysClock;

/*
 * The tasks as described in the comments at the top of this file.
 */

static void ReadSensor( void *pvParameters );


/*
 * Called by main() to do example specific hardware configurations and to
 * create the Process Switch task.
 */
void vCreateLEDTask( void );



//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, g_ui32SysClock);

    SysCtlDelay(g_ui32SysClock); // ~1 second delay at 120MHz to ensure UART initialises before using UARTprintf
}

/*-----------------------------------------------------------*/

void vCreateLEDTask( void )
{
    bool      success;
    uint16_t  rawData = 0;
    float     convertedLux = 0;

    //
    // Configure the system frequency.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);


    //
    // Initialize the UART.
    //
    ConfigureUART();
    

    //
    // Clear the terminal and print the welcome message.
    //
    UARTprintf("OPT001 Example\n");

    //
    // The I2C0 peripheral must be enabled before use.
    //
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
    UARTprintf("1\n");
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));

    //
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    // This step is not necessary if your part does not support pin muxing.
    //
    UARTprintf("2\n");
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    UARTprintf("3\n");
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);

    UARTprintf("3\n");
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);
    
    // Enable the Interrupts
    I2CMasterIntClear(I2C2_BASE);
    IntPrioritySet(INT_I2C2, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    I2CMasterIntEnable(I2C2_BASE);
    IntEnable(INT_I2C2); 
    //
    // Enable interrupts to the processor.
    //
    UARTprintf("4\n");
    IntMasterEnable();

    //Initialise sensor
    UARTprintf("5\n");
    sensorOpt3001Init();

    // Test that sensor is set up correctly
    UARTprintf("Testing OPT3001 Sensor:\n");
    success = sensorOpt3001Test();

    //stay here until sensor is working
    while (!success) {
        SysCtlDelay(g_ui32SysClock);
        UARTprintf("Test Failed, Trying again\n");
        success = sensorOpt3001Test();
    }

    UARTprintf("All Tests Passed!\n\n");

 


    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name for the LED Task - for debug only as it is not used by
     *    the kernel.
     *  - The size of the stack to allocate to the task.
     *  - The parameter passed to the task - just to check the functionality.
     *  - The priority assigned to the task.
     *  - The task handle is not required, so NULL is passed. */
    xTaskCreate( ReadSensor,
                 "LED",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );

}
/*-----------------------------------------------------------*/



static void ReadSensor(void *pvParameters){
    bool      success;
    uint16_t  rawData = 0;
    float     convertedLux = 0;
    // Loop Forever
    while(1)
    {
        SysCtlDelay(g_ui32SysClock/100);

        //Read and convert OPT values
        success = sensorOpt3001Read(&rawData);

        if (success) {
            sensorOpt3001Convert(rawData, &convertedLux);

            // Construct Text
            int lux_int = (int)convertedLux;
            UARTprintf("Lux: %5d\n", lux_int);
        }

    }
}






