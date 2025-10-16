/* Standard includes. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "semphr.h"
#include "task.h"

/* Hardware includes. */
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "drivers/rtos_hw_drivers.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

/* Display includes. */
#include "grlib.h"
#include "widget.h"
#include "canvas.h"

#include "drivers/Kentec320x240x16_ssd2119_spi.h"
#include "drivers/touch.h"

#include "utils/uartstdio.h"

#define MAX_DATA_POINTS 20
#define MAX_RANGE 100

typedef struct {
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
    uint32_t fillColor;
    uint32_t outlineColor;
    tContext *pContext;
} GraphCanvas;

static GraphCanvas g_sGraphCanvas;
static int g_iGraphData[MAX_DATA_POINTS];
static int g_iDataLength = 0;
tContext ctx;

extern volatile uint32_t g_ui32SysClock;
static void prvDisplayTask( void *params );

void vCreateTasks( void ){
    xTaskCreate(prvDisplayTask, "Display", configMINIMAL_STACK_SIZE, NULL,
              tskIDLE_PRIORITY + 1, NULL);
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

static void prvDisplayTask( void *params ){
    UARTprintf("Test");
    
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    GrContextInit(&ctx, &g_sKentec320x240x16_SSD2119);
    graphInit(0, 0, GrContextDpyWidthGet(&ctx), 200,
              ClrBlack, ClrWhite);

    addDataPoints(10);
    addDataPoints(20);
    addDataPoints(30);
    addDataPoints(40);

    for (;;){
        addDataPoints(40);
        vTaskDelay(pdMS_TO_TICKS(100));
    };
}


