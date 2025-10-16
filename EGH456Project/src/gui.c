/**
 * @defgroup gui GUI Subsystem
 * @brief GUI is interface between the user and the system
 */
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Kernel Includes */
#include <FreeRTOS.h>
#include <task.h>

/* TivaWare DriverLib - Low-level Hardware Access */
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/tm4c1294ncpdt.h"

#include <adc.h>
#include <driverlib/hibernate.h>
#include <flash.h>
#include <fpu.h>
#include <gpio.h>
#include <rom.h>
#include <rom_map.h>
#include <sysctl.h>
#include <systick.h>
#include <timer.h>
#include <uart.h>
#include <udma.h>
#include "time.h"

/* TivaWare Graphics Library (grlib) and Widgets */
#include <grlib/grlib.h>
#include <widget.h>
#include <canvas.h>
#include <checkbox.h>
#include <container.h>
#include <pushbutton.h>
#include <radiobutton.h>
#include <slider.h>

/* Utility Libraries */
#include <utils/ustdlib.h>
#include <uartstdio.h>

/* Display and Touch Drivers */
#include <Kentec320x240x16_ssd2119_spi.h>
#include <touch.h>

/* Application Includes */
#include "gui.h"
#include "gui_widgets.h"
#include "interrupt.h"
#include "portmacro.h"
#include <motor.h>
#include <shared.h>

extern int32_t g_i32HeatingTempThreshold;
extern int32_t g_i32CoolingTempThreshold;
// Global Variables
extern uint32_t g_ui32SysClock;
extern tCanvasWidget g_sStatusClock;
static tContext g_sContext;
static volatile ScreenState currentScreen = SCREEN_HOME;
tWidget *g_pActiveScreen = NULL;
char g_timeString[32];
char g_dateString[32];
volatile bool isNight = false;

extern tCanvasWidget g_sHomeClockBanner;
extern tCanvasWidget g_sMotorClockBanner;
extern tCanvasWidget g_sPlotClockBanner;
extern tCanvasWidget g_sSettingsClockBanner;
extern tCanvasWidget g_sStatusClockBanner;
plot_type_t g_currentPlot = PLOT_LIGHT;

// Function Prototypes
static void GUI_DrawScreen(ScreenState screen);

/**
 * @brief GUI FreeRTOS Task
 * Processes widget message queue at ~30Hz
 */
void guiTask(void *pvParameters)
{
    GUI_SetScreen(SCREEN_HOME);
    TickType_t lastUpdate = xTaskGetTickCount();

    while (1)
    {
        WidgetMessageQueueProcess();

        if ((xTaskGetTickCount() - lastUpdate) >= pdMS_TO_TICKS(1000))
        {
            struct tm now;
            if (HibernateCalendarGet(&now) == 0)
            {
                usnprintf(g_timeString, sizeof(g_timeString),
                          "%02d:%02d:%02d", now.tm_hour, now.tm_min, now.tm_sec);

                switch (currentScreen)
                {
                case SCREEN_HOME:
                    if (g_pActiveScreen == (tWidget *)&g_sHomePanel)
                    {
                        CanvasTextSet(&g_sHomeClockBanner, g_timeString);
                        WidgetPaint((tWidget *)&g_sHomeClockBanner);
                    }
                    break;

                case SCREEN_MOTOR:
                    if (g_pActiveScreen == (tWidget *)&g_sMotorPanel)
                    {
                        CanvasTextSet(&g_sMotorClockBanner, g_timeString);
                        WidgetPaint((tWidget *)&g_sMotorClockBanner);
                        static int lastEStop = -1;

                        int currentEStop = eStopGetter(0);
                        if (currentScreen == SCREEN_MOTOR && currentEStop != lastEStop)
                        {
                            UpdateMotorUI();
                            lastEStop = currentEStop;
                        }
                    }
                    break;

                case SCREEN_PLOTS:
                    if (g_pActiveScreen == (tWidget *)&g_sPlotPanel)
                    {
                        CanvasTextSet(&g_sPlotClockBanner, g_timeString);
                        WidgetPaint((tWidget *)&g_sPlotClockBanner);
                    }
                    break;

                case SCREEN_SETTINGS:
                    if (g_pActiveScreen == (tWidget *)&g_sSettingsPanel)
                    {
                        CanvasTextSet(&g_sSettingsClockBanner, g_timeString);
                        WidgetPaint((tWidget *)&g_sSettingsClockBanner);
                    }
                    break;

                case SCREEN_STATUS:
                    if (g_pActiveScreen == (tWidget *)&g_sStatusPanel)
                    {
                        // Time in banner
                        CanvasTextSet(&g_sStatusClockBanner, g_timeString);
                        WidgetPaint((tWidget *)&g_sStatusClockBanner);

                        // Date in panel
                        usnprintf(g_dateString, sizeof(g_dateString),
                                  "Date: %02d/%02d/%04d",
                                  now.tm_mday, now.tm_mon + 1, now.tm_year + 1900);
                        CanvasTextSet(&g_sStatusClock, g_dateString);
                        WidgetPaint((tWidget *)&g_sStatusClock);
                    }
                    break;

                default:
                    break;
                }
            }

            lastUpdate = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

void guiSensorTask(void *pvParams)
{
    sharedValues *event;
    val msg;
    char label[32];
    char label2[32];

    while (1)
    {

        const char *plotLabel;
        // Set event to related plot data
        switch (g_currentPlot)
        {
        case PLOT_LIGHT:
            event = &lightData;
            plotLabel = "Light";
            break;
        case PLOT_SPEED:
            event = &rpmData;
            plotLabel = "RPM";
            break;
        case PLOT_POWER:
            event = &powerData;
            plotLabel = "Power";
            break;
        case PLOT_TEMPERATURE:
            event = &tempData;
            plotLabel = "Temp";
            break;
        case PLOT_HUMIDITY:
            event = &humiData;
            plotLabel = "Humi";
            break;
        case PLOT_CURRENT:
            event = &maxCurrentLimit;
            plotLabel = "Current";
            break;
        default:
            event = &lightData;
            break;
        }
        getter(event, &msg, portMAX_DELAY);
        UARTprintf("%sRaw%d.%03d%sFilter%d.%03d\n", plotLabel,
                   (int)msg.raw, (int)((msg.raw - (int)msg.raw) * 1000), plotLabel,
                   (int)msg.filtered, (int)((msg.filtered - (int)msg.filtered) * 1000));
        if (currentScreen == SCREEN_PLOTS)
        {
            GUI_AddDataPoint((int)msg.filtered);
            WidgetPaint((tWidget *)&g_sPlotArea);
            vTaskDelay(pdMS_TO_TICKS(200));
            // UARTprintf("DATA: %d\n", (int)msg.filtered);
        }
        if (xQueueReceive(eventQueue, &event, pdMS_TO_TICKS(100)) == pdPASS)
        {
            getter(event, &msg, portMAX_DELAY);
            if (event == &lightData)
            {
                if ((int)msg.filtered < 0 || (int)msg.filtered > 100000)
                    goto sensorDelay;
                // UARTprintf("[Sensor] Light: %d Lux\n", (int)msg.filtered);
                if ((int)msg.filtered < 5)
                {
                    g_sStatusDayNight.pcText = "Ambient: Nighttime";
                    isNight = true;
                }
                else
                {
                    g_sStatusDayNight.pcText = "Ambient: Daytime";
                    isNight = false;
                }

                if (currentScreen == SCREEN_STATUS && g_pActiveScreen == (tWidget *)&g_sStatusPanel)
                {
                    usnprintf(label, sizeof(label), "Light: %d Lux", (int)msg.filtered);
                    CanvasTextSet(&g_sStatusLight, label);
                    WidgetPaint((tWidget *)&g_sStatusLight);
                    // Update ambient status based on lux

                    WidgetPaint((tWidget *)&g_sStatusDayNight);
                    val humiValue;
                    uint8_t errorFlag = getter(&humiData, &humiValue, portMAX_DELAY);
                    if (errorFlag != 0)
                    {
                        goto sensorDelay;
                    }
                    usnprintf(label2, sizeof(label2), "Humidity: %dRH", (int)humiValue.filtered);
                    CanvasTextSet(&g_sStatusHumidity, label2);
                    WidgetPaint((tWidget *)&g_sStatusHumidity);
                }
            }
            else if (event == &tempData)
            {
                if ((int)msg.filtered < -20 || (int)msg.filtered > 60)
                    goto sensorDelay;
                // UARTprintf("[Sensor] Temp: %dC Humidity: %dRH\n", (int)msg.filtered, (int)msg.filtered2);

                if (currentScreen == SCREEN_STATUS && g_pActiveScreen == (tWidget *)&g_sStatusPanel)
                {
                    usnprintf(label, sizeof(label), "Temp: %dC", (int)msg.filtered);
                    CanvasTextSet(&g_sStatusTemperature, label);
                    WidgetPaint((tWidget *)&g_sStatusTemperature);
                    // Update cooling status based on temperature
                    if ((int)msg.filtered > g_i32HeatingTempThreshold)
                    {
                        g_sStatusCooling.pcText = "AC: Cooling";
                    }
                    else if ((int)msg.filtered < g_i32CoolingTempThreshold)
                    {
                        g_sStatusCooling.pcText = "AC: Heating";
                    }
                    else
                    {
                        g_sStatusCooling.pcText = "AC: --";
                    }

                    WidgetPaint((tWidget *)&g_sStatusCooling);
                }
            }
            else if (event == &powerData)
            {
                if ((int)msg.filtered < 0 || (int)msg.filtered > 100)
                    goto sensorDelay; // Assume 0–100W power range
                // UARTprintf("[Sensor] Power: %d W\n", (int)msg.filtered);

                if (currentScreen == SCREEN_STATUS && g_pActiveScreen == (tWidget *)&g_sStatusPanel)
                {
                    usnprintf(label, sizeof(label), "Power: %d W", (int)msg.filtered);
                    CanvasTextSet(&g_sStatusPower, label);
                    WidgetPaint((tWidget *)&g_sStatusPower);
                }
                else
                {
                    g_sStatusPower.pcText = "Power: -- W";
                }
            }
            else if (event == &rpmData)
            {
                // Sanitize value before using it
                if ((int)msg.filtered >= 0 && (int)msg.filtered < 10000.0)
                {
                    // Cast only after confirming value is within safe int range
                    currentRPM = (int)msg.filtered;

                    // UARTprintf("[Sensor] Motor RPM: %d -> %d\n", msg.filtered, currentRPM);
                    // UARTprintf("[Sensor] Motor RPM: %d\n", currentRPM);

                    // Update STATUS screen label
                    if (currentScreen == SCREEN_STATUS && g_pActiveScreen == (tWidget *)&g_sStatusPanel)
                    {
                        usnprintf(label, sizeof(label), "Current RPM: %d", currentRPM);
                        CanvasTextSet(&g_sStatusActualRPM, label);
                        WidgetPaint((tWidget *)&g_sStatusActualRPM);
                    }
                }
                else
                {
                    // UARTprintf("[Sensor] Motor RPM: INVALID (%d)\n", (int)msg.filtered);
                }
            }
            else
            {
                // UARTprintf("[Sensor] Unknown event ID: %d\n", (int)event);
            }
        }

    sensorDelay:
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void LEDTask(void *pvParams)
{
    while (1)
    {
        if (isNight)
            GPIO_PORTN_DATA_R |= (1 << 1);
        else
            GPIO_PORTN_DATA_R &= ~(1 << 1);
        uint8_t estopValue = eStopGetter(200);
        if (motorRunning && !estopValue)
            GPIO_PORTN_DATA_R |= (1 << 0);
        else
            GPIO_PORTN_DATA_R &= ~(1 << 0);

        if (estopValue == 1)
            GPIO_PORTF_AHB_DATA_R |= (1 << 4);
        else
            GPIO_PORTF_AHB_DATA_R &= ~(1 << 4);

        vTaskDelay(pdMS_TO_TICKS(200)); // check every 200ms
    }
}

/**
 * @brief Touchscreen debug callback
 * Prints touch message and coordinates via UART
 */
int32_t TouchTestCallback(uint32_t ui32Message, int32_t i32X, int32_t i32Y)
{
    // UARTprintf("[TOUCH] Message: 0x%08x  X: %d  Y: %d\n", ui32Message, i32X, i32Y);
    return 0;
}

/**
 * @brief Initializes the GUI hardware and display context
 *
 * This function sets up the ADC for touch input, initializes the display,
 * and prepares the graphics context for rendering.
 */
void setupGUIHardware()
{
    // setupTouchADC1();

    Kentec320x240x16_SSD2119Init(g_ui32SysClock);
    GrContextInit(&g_sContext, &g_sKentec320x240x16_SSD2119);

    TouchScreenInit(g_ui32SysClock);
    TouchScreenCallbackSet(WidgetPointerMessage);
    // TouchScreenCallbackSet(TouchTestCallback);
}
void setupLEDHardware()
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;

    // Enable GPIO pin for LED D1 & D2 (PN1 & PN0) as inputs - 0x02 & 0x01
    GPIO_PORTN_DEN_R |= (1 << 1) | (1 << 0); // Binary: 0b00000011, Hex: 0x03

    // Set the direction for LED D1 & D2 (PN1 & PN0) as outputs
    GPIO_PORTN_DIR_R |= (1 << 1) | (1 << 0); // Binary: 0b00000011, Hex: 0x03

    // Enable GPIO pin for LED D3 & D4 (PF4 & PF0) as inputs - 0x05 & 0x01
    GPIO_PORTF_AHB_DEN_R |= (1 << 4); // Binary: 0b00010001, Hex: 0x11

    // Set the direction for LED D3 & D4 (PF4 & PF0) as outputs
    GPIO_PORTF_AHB_DIR_R |= (1 << 4); // Binary: 0b00010001, Hex: 0x11
}

/**
 * @brief Creates the GUI task and initializes the display
 *
 * This function sets up the GUI hardware, initializes the active screen,
 * and creates the FreeRTOS task that will handle GUI updates.
 *
 * @return 0 on success, non-zero error code on failure
 */
uint8_t CreateGUITask(void)
{
    UARTprintf("Creating Display Tasks\n");

    setupGUIHardware();

    // Pre-populate time/date strings so initial screen draw has valid values
    struct tm now;
    if (HibernateCalendarGet(&now) == 0)
    {
        usnprintf(g_timeString, sizeof(g_timeString),
                  "%02d:%02d:%02d", now.tm_hour, now.tm_min, now.tm_sec);
        usnprintf(g_dateString, sizeof(g_dateString),
                  "Date: %02d/%02d/%04d", now.tm_mday, now.tm_mon + 1, now.tm_year + 1900);
    }
    else
    {
        usnprintf(g_timeString, sizeof(g_timeString), "--:--:--");
        usnprintf(g_dateString, sizeof(g_dateString), "Date: --/--/----");
    }

    GUI_SetScreen(SCREEN_HOME);

    BaseType_t result = xTaskCreate(guiTask, "GUI", 4096, NULL, 1, NULL);
    return (result == pdPASS) ? 0 : 1;
}

uint8_t CreateGUISensorTask(void)
{
    BaseType_t result = xTaskCreate(guiSensorTask, "GUI", 4096, NULL, 2, NULL);
    return (result == pdPASS) ? 0 : 1;
}

uint8_t CreateLEDTask(void)
{
    UARTprintf("Creating LED Tasks\n");

    setupLEDHardware();

    BaseType_t result = xTaskCreate(LEDTask, "GUI", 4096, NULL, 1, NULL);
    return (result == pdPASS) ? 0 : 1;
}

/**
 * @brief Sets the current GUI screen
 */
void GUI_SetScreen(ScreenState screen)
{
    currentScreen = screen;
    GUI_DrawScreen(currentScreen);
}

/**
 * @brief Draw and switch to the specified GUI screen
 *
 * This function handles deactivating the previous screen, building the selected
 * screen layout, and updating the active widget tree. It should be called when
 * changing screens via navigation buttons.
 *
 * @param screen ScreenState enum indicating which screen to draw
 */
static void GUI_DrawScreen(ScreenState screen)
{
    // Store reference to current screen so we can remove it later
    tWidget *oldScreen = g_pActiveScreen;

    // Switch based on the new screen to activate
    switch (screen)
    {
    case SCREEN_HOME:
        // Build home screen widgets and update active screen pointer
        GUI_BuildHomeScreen(&g_sContext);
        g_pActiveScreen = (tWidget *)&g_sHomePanel;
        break;

    case SCREEN_MOTOR:
        // Build motor screen widgets and update active screen pointer
        GUI_BuildMotorScreen(&g_sContext);
        g_pActiveScreen = (tWidget *)&g_sMotorPanel;
        break;

    case SCREEN_STATUS:
        // Build system status screen and update active screen pointer
        GUI_BuildStatusScreen(&g_sContext);
        g_pActiveScreen = (tWidget *)&g_sStatusPanel;
        break;

    case SCREEN_PLOTS:
        // Build sensor plotting screen and update active screen pointer
        GUI_BuildPlotScreen(&g_sContext);
        g_pActiveScreen = (tWidget *)&g_sPlotPanel;
        break;

    case SCREEN_SETTINGS:
        // Build settings screen widgets and update active screen pointer
        GUI_BuildSettingsScreen(&g_sContext);
        g_pActiveScreen = (tWidget *)&g_sSettingsPanel;
        break;

    default:
        // Invalid screen state; set to NULL
        g_pActiveScreen = NULL;
        break;
    }

    // Remove previous screen from the widget tree if it exists
    if (oldScreen != NULL)
    {
        WidgetRemove(oldScreen);
    }

    // Add and paint the new screen if valid
    if (g_pActiveScreen != NULL)
    {
        WidgetAdd(WIDGET_ROOT, g_pActiveScreen);
        WidgetPaint(WIDGET_ROOT); // repaint entire root widget
    }

    // Debugging output if needed
    // UARTprintf("Final g_pActiveScreen: %p\n", g_pActiveScreen);
    // if (g_pActiveScreen != NULL)
    // {
    //     UARTprintf("ActiveScreen->psChild: %p\n", g_pActiveScreen->psChild);
    // }
}

/**
 * @brief Timer2A interrupt handler for GUI timing
 *
 * This ISR is triggered every 500ms (2Hz) and can be used
 * to implement GUI animations, periodic refresh, or debugging.
 * Currently prints a debug message over UART.
 */
void xTimer2AHandler(void)
{
    //
    // Clear the interrupt flag to acknowledge the timer interrupt.
    //
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Debug message to confirm interrupt is firing.
    //
}
/** @} */
