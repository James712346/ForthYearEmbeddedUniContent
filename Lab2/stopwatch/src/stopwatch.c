//*****************************************************************************
//
// timers.c - Timers example.
//
// Copyright (c) 2013-2020 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "grlib.h"
#include "utils/ustdlib.h"
#include "widget.h"
#include "canvas.h"

#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"

/*
 * States
 */
#define INACTIVE 0
#define STOPWATCH 1

volatile uint8_t currentState = 0;

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator throught the UART.
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;

// data structures for graphics library
tContext sContext;
tRectangle sRect;

// The switches tie the GPIO to ground, so the GPIOs need to be configured
// with pull-ups, and a value of 0 means the switch is pressed.
//
//*****************************************************************************

#define BUTTONS_GPIO_PERIPH     SYSCTL_PERIPH_GPIOJ
#define BUTTONS_GPIO_BASE       GPIO_PORTJ_BASE
#define NUM_BUTTONS             2
#define USR_SW1                 GPIO_PIN_0
#define USR_SW2                 GPIO_PIN_1
#define ALL_BUTTONS             (USR_SW1 | USR_SW2)
static uint8_t g_ui8ButtonStates = ALL_BUTTONS;
volatile uint32_t g_pui32ButtonPressed; // Global variable to log the last GPIO button pressed.

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

// Counter Functions
volatile uint8_t counter0 = 0;
volatile uint8_t counter1 = 0;
bool stopTimer = false;
bool TimerStarted = false;
uint8_t ClearTimer(){ 
    counter0 = 0;
    counter1 = 0;
    currentState = INACTIVE; 
    return 0;
}

uint8_t StartTimer(){
    // Enable the timers.
    currentState = STOPWATCH;
    TimerStarted = true;
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);
    return 0;
}

uint8_t StopTimer(){
    // Disable the timers.
    TimerStarted = false;
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerDisable(TIMER1_BASE, TIMER_A);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    return 0;
}
int previousCounter = 0;
void renderInactive(){
    counter1 = 0;
    counter0 = 0;
    if (previousCounter == -1){
        return;
    }
    previousCounter = -1;
    sRect.i16XMin = 0;
    sRect.i16YMin = 25;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = GrContextDpyHeightGet(&sContext) -1;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    GrContextForegroundSet(&sContext, ClrWhite);
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, "Instructions", -1,GrContextDpyWidthGet(&sContext) / 2, 35, 0);
    GrContextFontSet(&sContext, &g_sFontCm14);
    GrStringDrawCentered(&sContext, "To Start the Timer, PRESS SW1", -1,GrContextDpyWidthGet(&sContext) / 2, 65, 0);
    GrStringDrawCentered(&sContext, "To Stop the Timer, PRESS SW1", -1,GrContextDpyWidthGet(&sContext) / 2, 100, 0);
    GrStringDrawCentered(&sContext, "To Reset the Timer, PRESS SW2", -1,GrContextDpyWidthGet(&sContext) / 2, 135, 0);

}


void renderCounter(){
    if (counter1 == previousCounter){
        return;
    }
    previousCounter = counter1;
    sRect.i16XMin = 0;
    sRect.i16YMin = 25;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = GrContextDpyHeightGet(&sContext) -1;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    GrContextForegroundSet(&sContext, ClrWhite);

    char display_string[30];
    if (counter0 == 20){
        usprintf(display_string, "Time limit reached!");
        StopTimer();
    } else {
        usprintf(display_string, "%d.%d", counter0, counter1);
    }
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, display_string, -1,GrContextDpyWidthGet(&sContext) / 2, GrContextDpyHeightGet(&sContext) / 2, 0);

}


void render(){
    switch (currentState) {
        case STOPWATCH:
            renderCounter();
            break;
        case INACTIVE:
            renderInactive();
            break;
        default:
            break;
    }
}

void
Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    HWREGBITW(&g_ui32Flags, 0) ^= 1;

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, g_ui32Flags);

    IntMasterDisable();
    if (counter0 < 20) counter0++;
    IntMasterEnable();

}

void
Timer1IntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    HWREGBITW(&g_ui32Flags, 1) ^= 1;

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, g_ui32Flags);

    IntMasterDisable();
    if (counter1 < 10 && counter0 < 20) counter1++;
    if (counter1 >= 10 || counter0 >= 20) counter1 = 0;
    IntMasterEnable();
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
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
}

//*****************************************************************************
//
// Button int handler (referenced in vector table) and configuration function
//
//*****************************************************************************
void xButtonsHandler( void )
{
    uint32_t ui32Status;
    

    /* Read the buttons interrupt status to find the cause of the interrupt. */
    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    /* Clear the interrupt. */
    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);
    /* Log which button was pressed to trigger the ISR. */
    if (((ui32Status & USR_SW1) == USR_SW1))
    {
        if (g_pui32ButtonPressed == 0){
            if (TimerStarted){
                StopTimer();
            } else if (counter0 < 20){
                StartTimer();
            } else {
                ClearTimer();
                StartTimer();
            }
        }
        g_pui32ButtonPressed |=1;
    }
    else if ((ui32Status & USR_SW2) == USR_SW2)
    {
        StopTimer();
        ClearTimer();
    }
}

static void prvConfigureButton( void )

{
    //
    // Enable the GPIO port to which the pushbuttons are connected.
    //
    SysCtlPeripheralEnable(BUTTONS_GPIO_PERIPH);

    //
    // Set each of the button GPIO pins as an input with a pull-up.
    //
    GPIODirModeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(BUTTONS_GPIO_BASE, ALL_BUTTONS,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //
    // Initialize the debounced button state with the current state read from
    // the GPIO bank.
    //
    g_ui8ButtonStates = GPIOPinRead(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Configure both switches to trigger an interrupt on a falling edge. */
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);

    /* Enable the interrupt for LaunchPad GPIO Port in the GPIO peripheral. */
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);

    /* Enable the Port F interrupt in the NVIC. */
    IntEnable(INT_GPIOJ);

    /* Enable global interrupts in the NVIC. */
    IntMasterEnable();
}



//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, "Stopwatch", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 8, 0);                     

    //
    // Initialize the UART and write status.
    //
    ConfigureUART();

    //
    // Initialise the push button interrupt
    //
    prvConfigureButton();

    //
    // Enable the GPIO port that is used for the on-board LEDs.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Enable the GPIO pins for the LEDs (PN0 & PN1).
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    IntPrioritySet(INT_TIMER0A, 0x40);
    IntPrioritySet(INT_TIMER1A, 0x40);
    IntPrioritySet(INT_GPIOJ, 0x00);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    // Set the clock to .10 second, and 1 second
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, g_ui32SysClock);
    TimerLoadSet(TIMER1_BASE, TIMER_A, g_ui32SysClock / 10);

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);


    //
    // Loop forever while the timers run, updating the screen and printing via UART
    //
    while(1)
    {
        render();
        g_pui32ButtonPressed = (g_pui32ButtonPressed << 2) & 0xFF;
    }
}
