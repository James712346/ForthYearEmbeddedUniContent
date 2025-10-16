/*
 * queue_example
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
 * This example demonstrates passing a complex structure by value as well as
 * by reference using FreeRTOS's queue between multiple tasks.  Two queues are
 * set up with one for passing messages by value and another for passing by
 * reference.  A total of six tasks are created with four of them sending
 * messages through the two queues while the other two tasks receiving the
 * message from the two queues either by value or by reference.  The four tasks
 * transmit their messages at different times.  Their time stamp is sent as
 * the data as part of the message.  Once the data is received, the receiving
 * tasks print the task ID and its corresponding time stamp on the terminal
 * window.
 *
 * This example uses UARTprintf for output of UART messages.  UARTprintf is not
 * a thread-safe API and is only being used for simplicity of the demonstration
 * and in a controlled manner.
 *
 * Open a terminal with 115,200 8-N-1 to see the output for this demo.
 *
 */

/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "event_groups.h"

/* Hardware includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include <driverlib/timer.h>
#include <inc/hw_ints.h>
#include <semphr.h>
#include <driverlib/i2c.h>
/* Display includes. */
#include "grlib.h"
#include "widget.h"
#include "canvas.h"

#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"

/*-----------------------------------------------------------*/
#define MAX_LUX 100
#define MIN_LUX 5
#define FILTER_WINDOW 10
#define MAX_DATA_POINTS 100
#define MAX_RANGE 100

#define VENT_HIGH_THRESHOLD (1UL << 0UL)
#define VENT_LOW_THRESHOLD (1UL << 1UL) 
#define EVENT_BTN_TOGGLE (1UL << 2UL)

/* The system clock frequency. */
volatile float g_lux = 0.0f;
volatile bool g_luxUpdated = false;
uint32_t g_ui32SysClock;

EventGroupHandle_t xEventGroup;


SemaphoreHandle_t g_xLightSensorSemaphore;
SemaphoreHandle_t g_xDataSemaphore;
SemaphoreHandle_t g_xButtonSemaphore;
SemaphoreHandle_t printing;
SemaphoreHandle_t xI2CSemaphore = NULL;

typedef struct {
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
    uint32_t fillColor;
    uint32_t outlineColor;
} GraphCanvas;

static GraphCanvas g_sGraphCanvas;
static int g_iGraphData[MAX_DATA_POINTS];
static int g_iDataLength = 0;
tContext ctx;

/* Set up the hardware ready to run this demo. */
static void prvSetupHardware( void );

/* This function sets up UART0 to be used for a console to display information
 * as the example is running. */
static void prvConfigureUART(void);

/* API to start the queue task. */
extern void vQueueTask( void );
QueueHandle_t g_xLightSensorQueue;
static void ReadLight(void *pvParameters);
extern void I2C0IntHandler(void);
static void ConfigureTimers(void);
static void buttonTask(void *prvParameters);
void Timer3AIntHandler(void);
static void DisplayLight(void *pvParameters);
/*-----------------------------------------------------------*/

typedef struct {
    uint16_t raw_lux;         
    float lux_value;         
    float filtered_lux;       
    TickType_t timestamp;     
} LightSensorData_t;


static void prvDisplayTask( void *params );

void xButtonHandler(void){
    BaseType_t xButtonTask = pdFALSE;
    uint32_t ui32Status;

    ui32Status = GPIOIntStatus(BUTTONS_GPIO_BASE, true);

    GPIOIntClear(BUTTONS_GPIO_BASE, ui32Status);
    if ((ui32Status & USR_SW1) || (ui32Status & USR_SW2)) {
        xSemaphoreGiveFromISR(g_xButtonSemaphore, &xButtonTask);
    }
    portYIELD_FROM_ISR(xButtonTask);
}

void drawGraphCanvas(GraphCanvas *canvas) {
    tRectangle graphBounds;
    graphBounds.i16XMax = canvas->x + canvas->width -1;
    graphBounds.i16XMin = canvas->x;
    graphBounds.i16YMax = canvas->y + canvas->height-1;
    graphBounds.i16YMin = canvas->y;
    GrContextForegroundSet(&ctx, canvas->fillColor);
    GrRectFill(&ctx, &graphBounds);

    GrContextForegroundSet(&ctx, canvas->outlineColor);
    GrRectDraw(&ctx, &graphBounds);

    int xMin = canvas->x;
    int yMin = canvas->y;
    int xMax = canvas->x + canvas->width - 1;
    int yMax = canvas->y + canvas->height - 1;

    GrLineDraw(&ctx, xMin, yMax, xMax, yMax);
    GrLineDraw(&ctx, xMin, yMax, xMin, yMin);

    int scalingFactorX = canvas->width / MAX_DATA_POINTS;
    int scalingFactorY = canvas->height / MAX_RANGE;

    for (int i = 1; i < g_iDataLength; i++) {
        int x0 = xMin + (i - 1) * scalingFactorX;
        int y0 = yMax - g_iGraphData[i - 1] * scalingFactorY;
        int x1 = xMin + i * scalingFactorX;
        int y1 = yMax - g_iGraphData[i] * scalingFactorY;

        GrLineDraw(&ctx, x0, y0, x1, y1);
    }
}

void addDataPoints(int value) {
    if (g_iDataLength < MAX_DATA_POINTS) {
        g_iGraphData[g_iDataLength++] = value;
    } else {
        for (int i = 1; i < MAX_DATA_POINTS; i++) {
            g_iGraphData[i - 1] = g_iGraphData[i];
        }
        g_iGraphData[MAX_DATA_POINTS - 1] = value;
    }

    drawGraphCanvas(&g_sGraphCanvas);
}

void graphInit( int32_t x, int32_t y, int32_t w, int32_t h,
               uint32_t fill, uint32_t outline) {
    g_sGraphCanvas.x = x;
    g_sGraphCanvas.y = y;
    g_sGraphCanvas.width = w;
    g_sGraphCanvas.height = h;
    g_sGraphCanvas.fillColor = fill;
    g_sGraphCanvas.outlineColor = outline;
}

static void prvConfigureButton(void){
    ButtonsInit();
    GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);
    GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);
    IntEnable(INT_GPIOJ);
    IntMasterEnable();
}

void vCreateLEDTask(void)
{
    /* 1) Set system clock to 120 MHz */
    g_ui32SysClock = SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ |
        SYSCTL_OSC_MAIN   |
        SYSCTL_USE_PLL    |
        SYSCTL_CFG_VCO_480,
        120000000
    );
    
    /* 2) Configure UART for debug output */
    UARTprintf("Initializing system...\n");
    
    /* 3) Set interrupt priority grouping */
    IntPriorityGroupingSet(3);
    
    /* 4) Enable I2C2 and GPIO Port N */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    prvConfigureButton();
    /* 5) Configure PN4 = SDA, PN5 = SCL */
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);   // mux PN4 to I2C2 SDA
    GPIOPinConfigure(GPIO_PN5_I2C2SCL);   // mux PN5 to I2C2 SCL
    GPIOPinTypeI2C   (GPIO_PORTN_BASE, GPIO_PIN_4);  // SDA
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);  // SCL
    
    /* 6) Initialize I2C2 master */
    I2CMasterInitExpClk(I2C2_BASE, g_ui32SysClock, false);
    
    /* 7) Enable I2C2 interrupt on DATA and STOP conditions */
    I2CMasterIntEnable(I2C2_BASE);
    I2CMasterIntEnableEx(
        I2C2_BASE,
        I2C_MASTER_INT_DATA |
        I2C_MASTER_INT_STOP
    );
    
    /* 8) Register and enable the I2C2 ISR in the NVIC */
    IntRegister(INT_I2C2, I2C0IntHandler);
    IntEnable(   INT_I2C2);
    
    /* 9) Globally enable interrupts */
    IntMasterEnable();
    
    /* 10) Enable GPIOJ for buttons PJ0 & PJ1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(
        GPIO_PORTJ_BASE,
        GPIO_PIN_0 | GPIO_PIN_1,
        GPIO_STRENGTH_2MA,
        GPIO_PIN_TYPE_STD_WPU
    );
    
    /* 11) Configure LED outputs on PN0, PN1 and PF0, PF4 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    

    g_xDataSemaphore = xSemaphoreCreateBinary();
    g_xButtonSemaphore = xSemaphoreCreateBinary();
    xEventGroup  =  xEventGroupCreate();
    
    // Create binary semaphores for timer signaling
    g_xLightSensorSemaphore = xSemaphoreCreateBinary();
    printing = xSemaphoreCreateBinary();

    // Create the queue - sized to hold 5 sensor data messages
    g_xLightSensorQueue = xQueueCreate(5, sizeof(LightSensorData_t));
    if (g_xLightSensorQueue == NULL) {
        UARTprintf("Failed to create light sensor queue!\n");
    }
    
    UARTprintf("SETUP DONE\n");
    
    ConfigureTimers();
    
    xTaskCreate(
        ReadLight,
        "LightSens",
        configMINIMAL_STACK_SIZE * 2,
        NULL,
        tskIDLE_PRIORITY + 2,  
        NULL
    );
    
    // Create the display task
    xTaskCreate(
        DisplayLight,
        "LightDisp",
        configMINIMAL_STACK_SIZE * 2,
        NULL,
        tskIDLE_PRIORITY + 1,  // Lower priority than sensor task
        NULL
    );
    xTaskCreate(
        buttonTask,
        "ButtonTask",
        configMINIMAL_STACK_SIZE * 2,
        NULL,
        tskIDLE_PRIORITY + 1,  // Lower priority than sensor task
        NULL
    );
}

static void buttonTask(void *prvParameters){
    for (;;){
        if (xSemaphoreTake(g_xButtonSemaphore, portMAX_DELAY) == pdPASS){
            if (xEventGroupGetBits(xEventGroup) & EVENT_BTN_TOGGLE){
                xEventGroupClearBits(xEventGroup,EVENT_BTN_TOGGLE);
            } else {
                xEventGroupSetBits(xEventGroup,EVENT_BTN_TOGGLE);
            }
        }
    }
}

static void DisplayLight(void *pvParameters)
{
    LightSensorData_t receivedData;
    UARTprintf("raw,filtered\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    GrContextInit(&ctx, &g_sKentec320x240x16_SSD2119);
    graphInit(0, 0, GrContextDpyWidthGet(&ctx), 200,
              ClrBlack, ClrWhite);
    int previousBit = 0;
    for (;;) {
        if (xQueueReceive(g_xLightSensorQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
            int value = xEventGroupGetBits(xEventGroup);
            int32_t raw_int = (int32_t)(receivedData.lux_value * 100);
            int32_t filtered_int = (int32_t)(receivedData.filtered_lux * 100);
            addDataPoints(filtered_int/100);
            UARTprintf("A%d.%02dB%d.%02d\n", 
                       raw_int / 100, raw_int % 100,
                       filtered_int / 100, filtered_int % 100);
            if (value & VENT_LOW_THRESHOLD){
                UARTprintf("a low threshold value was received\n");
                xEventGroupClearBits(xEventGroup, VENT_LOW_THRESHOLD);
            } else if (value & VENT_HIGH_THRESHOLD){
                UARTprintf("a high threshold value was received\n");
                xEventGroupClearBits(xEventGroup, VENT_HIGH_THRESHOLD);
            }
            if ((value & EVENT_BTN_TOGGLE) != previousBit){
                UARTprintf("BTN FLIP\n");
                previousBit = value & EVENT_BTN_TOGGLE;
                xEventGroupClearBits(xEventGroup, EVENT_BTN_TOGGLE);
            }
        }
    }
}

int main( void )
{

    xI2CSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xI2CSemaphore);
    /* Prepare the hardware to run this demo. */
    prvSetupHardware();

    /* create the queue task. */
    vcreateQueueTasks();
    vCreateLEDTask();
    /* Start the tasks running. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks to be created.  See the memory management section on the
    FreeRTOS web site for more details. */
    for( ;; );
}

static void ReadLight(void *pvParameters)
{
    uint16_t raw_lux;
    float lux;
    static float lux_buffer[FILTER_WINDOW] = {0};
    uint32_t buf_index = 0;
    uint32_t buf_count = 0;
    int i = 0;
    LightSensorData_t sensorData;

    UARTprintf("Initializing light sensor...\n");
    sensorOpt3001Init();
    bool opt_test = sensorOpt3001Test();
    UARTprintf("OPT3001 Test: %s\n", opt_test ? "PASSED" : "FAILED");
    vTaskDelay(pdMS_TO_TICKS(500));

    for (;;) {
        if (xSemaphoreTake(g_xLightSensorSemaphore, portMAX_DELAY) == pdTRUE) {
            if (sensorOpt3001Read(&raw_lux)) {
                sensorOpt3001Convert(raw_lux, &lux);
                if (lux > MAX_LUX){
                    xEventGroupSetBits(xEventGroup, VENT_HIGH_THRESHOLD);
                } else if (lux < MIN_LUX){
                    xEventGroupSetBits(xEventGroup, VENT_LOW_THRESHOLD);
                }
                lux_buffer[buf_index] = lux;
                buf_index = (buf_index + 1) % FILTER_WINDOW;
                if (buf_count < FILTER_WINDOW) {
                    buf_count++;
                }

                float sum = 0;
                for (uint32_t k = 0; k < buf_count; k++) {
                    sum += lux_buffer[k];
                }
                float avg_lux = sum / buf_count;

                sensorData.raw_lux = raw_lux;
                sensorData.lux_value = lux;
                sensorData.filtered_lux = avg_lux;
                sensorData.timestamp = xTaskGetTickCount();

                xQueueSend(g_xLightSensorQueue, &sensorData, 0);
                
                i++;
            }
        }
    }
}
/*-----------------------------------------------------------*/

static void prvConfigureUART(void)
{
    /* Enable GPIO port A which is used for UART0 pins.
     * TODO: change this to whichever GPIO port you are using. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Configure the pin muxing for UART0 functions on port A0 and A1.
     * This step is not necessary if your part does not support pin muxing.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    /* Enable UART0 so that we can configure the clock. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Use the internal 16MHz oscillator as the UART clock source. */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    /* Select the alternate (UART) function for these pins.
     * TODO: change this to select the port/pin you are using. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the UART for console I/O. */
    UARTStdioConfig(0, 9600, 16000000);

}
/*-----------------------------------------------------------*/

static void ConfigureTimers(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);  // Enable Timer3 peripheral
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3));
    
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);  // Configure Timer3 as periodic
    
    TimerLoadSet(TIMER3_BASE, TIMER_A, g_ui32SysClock / 10 - 1);  // 10Hz for Timer3A
    
    TimerIntRegister(TIMER3_BASE, TIMER_A, Timer3AIntHandler); 
    
    IntPrioritySet(INT_TIMER3A, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_TIMER3B, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);  
    TimerIntEnable(TIMER3_BASE, TIMER_TIMB_TIMEOUT);  // Enable Timer3B interrupt
    
    IntEnable(INT_TIMER3A);  
    IntEnable(INT_TIMER3B); 
    
    TimerEnable(TIMER3_BASE, TIMER_A); 
    TimerEnable(TIMER3_BASE, TIMER_B);  
}

void Timer3AIntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 
                ~GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0) & GPIO_PIN_0);
    
    xSemaphoreGiveFromISR(g_xLightSensorSemaphore, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




static void prvSetupHardware( void )
{
    /* Run from the PLL at configCPU_CLOCK_HZ MHz. */
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
            SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);

    /* Configure device pins. */
    PinoutSet(false, false);

    /* Configure UART0 to send messages to terminal. */
    prvConfigureUART();
    ConfigureTimers();
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/

void *malloc( size_t xSize )
{
    /* There should not be a heap defined, so trap any attempts to call
    malloc. */
    IntMasterDisable();
    for( ;; );
}
/*-----------------------------------------------------------*/


