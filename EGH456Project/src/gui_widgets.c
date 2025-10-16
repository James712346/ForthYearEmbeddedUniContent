/**
 * @addtogroup gui
 * @{
 */
/* Standard Library */
#include <stdint.h>
#include <stdbool.h>

/* FreeRTOS Kernel */
#include <FreeRTOS.h>
#include <task.h>

/* TivaWare: Low-Level DriverLib */
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/flash.h"
#include "driverlib/gpio.h"
#include "driverlib/udma.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/fpu.h"

/* TivaWare: grlib & GUI Widgets */
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

/* Display and Touch */
#include <Kentec320x240x16_ssd2119_spi.h>
#include <touch.h>

/* Application Modules */
#include "hibernate.h"
#include "gui.h"
#include "gui_widgets.h"
#include "gui_display.h"
#include "portmacro.h"
#include "shared.h"
#include "motor.h"

/* Forward Declarations */
static void OnButtonPress(tWidget *psWidget);
static void OnSliderChange(tWidget *psWidget, int32_t value);
static void PlotPanelPaint(tWidget *psWidget, tContext *ctx);
void UpdateMotorUI(void);

// Function prototypes for zoom callbacks
extern void OnZoomIn(tWidget *psWidget);
extern void OnZoomOut(tWidget *psWidget);
extern void OnResetZoom(tWidget *psWidget);

/* External Shared Variables */
extern uint32_t currentRPM;
extern plot_type_t g_currentPlot;
extern char g_timeString[32];
extern char g_dateString[32];

/* State Flags */
bool g_bPlotVisible = true;
bool eStopWasAcknowledged = false;
bool motorRunning = false;

/* Constants */
const char *plotNames[] = {"Light", "Speed", "Power", "Temp", "Humidity", "Current"};

/* Slider Values */
int32_t g_i32DesiredRPM = 1000;
int32_t g_i32HeatingTempThreshold = 10;
int32_t g_i32CoolingTempThreshold = 20;
int32_t g_i32CurrentLimit = 20;

// === HOME Screen ===
tCanvasWidget g_sHomePanel;
tCanvasWidget g_sHomeBanner;
tCanvasWidget g_sHomeClockBanner;
tPushButtonWidget g_sButtonMotor;
tPushButtonWidget g_sButtonStatus;
tPushButtonWidget g_sButtonPlots;
tPushButtonWidget g_sButtonSettings;

// === MOTOR Screen ===
tCanvasWidget g_sMotorPanel;
tCanvasWidget g_sMotorBanner;
tCanvasWidget g_sMotorClockBanner;
tPushButtonWidget g_sMotorBackButton;
tPushButtonWidget g_sMotorToggleButton;
tSliderWidget g_sRPMSlider;
tCanvasWidget g_sRPMLabel;
tCanvasWidget g_sMotorStatusPanel;
tCanvasWidget g_sMotorAckLabel;

// === STATUS Screen ===
tCanvasWidget g_sStatusPanel;
tCanvasWidget g_sStatusBanner;
tCanvasWidget g_sStatusClockBanner;
tPushButtonWidget g_sStatusBackButton;
tCanvasWidget g_sStatusMotorState;
tCanvasWidget g_sStatusDesiredRPM;
tCanvasWidget g_sStatusActualRPM;
tCanvasWidget g_sStatusAccel;
tCanvasWidget g_sStatusLight;
tCanvasWidget g_sStatusPower;
tCanvasWidget g_sStatusClock;
tCanvasWidget g_sStatusDayNight;
tCanvasWidget g_sStatusTemperature;
tCanvasWidget g_sStatusHumidity;
tCanvasWidget g_sStatusCooling;

// === PLOT Screen ===
tCanvasWidget g_sPlotPanel;
tCanvasWidget g_sPlotBanner;
tCanvasWidget g_sPlotClockBanner;
tCanvasWidget g_sPlotArea;
tPushButtonWidget g_sPlotBackButton;
tPushButtonWidget g_sPlotCycleButton;
tPushButtonWidget g_sPlotToggleButton;
tCanvasWidget g_sPlotUnitLabel;
tPushButtonWidget g_sZoomInButton;
tPushButtonWidget g_sZoomOutButton;
tPushButtonWidget g_sResetZoomButton;
extern char g_plotUnitText[16];

// === SETTINGS Screen ===
tCanvasWidget g_sSettingsPanel;
tCanvasWidget g_sSettingsBanner;
tCanvasWidget g_sSettingsClockBanner;
tPushButtonWidget g_sSettingsBackButton;
tSliderWidget g_sTempHeatingSlider;
tCanvasWidget g_sTempHeatingLabel;
tSliderWidget g_sTempCooling;
tCanvasWidget g_sTempCoolingLabel;
tSliderWidget g_sCurrentSlider;
tCanvasWidget g_sCurrentLabel;
tPushButtonWidget g_sEStopTestButton;

/**
 * @brief Build and initialize the Home screen UI.
 *
 * Displays a 2x2 grid of navigation buttons (Motor, Status, Plots, Settings),
 * along with a top banner and real-time clock.
 */
void GUI_BuildHomeScreen(tContext *ctx)
{
    //*************************************************************************
    // Initialize Main Panel
    //*************************************************************************
    g_sHomePanel.sBase.psChild = NULL;
    g_sHomePanel.sBase.psParent = NULL;
    g_sHomePanel.sBase.psNext = NULL;
    g_sHomePanel.sBase.psDisplay = &g_sKentec320x240x16_SSD2119;
    g_sHomePanel.sBase.sPosition.i16XMin = 0;
    g_sHomePanel.sBase.sPosition.i16YMin = 0;
    g_sHomePanel.sBase.sPosition.i16XMax = 319;
    g_sHomePanel.sBase.sPosition.i16YMax = 239;
    g_sHomePanel.sBase.pfnMsgProc = CanvasMsgProc;

    g_sHomePanel.ui32Style = CANVAS_STYLE_FILL;
    g_sHomePanel.ui32FillColor = ClrBlack;
    g_sHomePanel.ui32OutlineColor = 0;
    g_sHomePanel.ui32TextColor = 0;
    g_sHomePanel.psFont = 0;
    g_sHomePanel.pcText = 0;
    g_sHomePanel.pui8Image = 0;
    g_sHomePanel.pfnOnPaint = 0;

    //*************************************************************************
    // Top Banner (Title + Clock)
    //*************************************************************************
    CanvasInit(&g_sHomeBanner, &g_sKentec320x240x16_SSD2119, 0, 0, 320, 24);
    g_sHomeBanner.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER;
    g_sHomeBanner.ui32FillColor = ClrDarkGray;
    g_sHomeBanner.ui32TextColor = ClrWhite;
    g_sHomeBanner.psFont = &g_sFontCm18;
    g_sHomeBanner.pcText = "EGH456 Assessment";
    g_sHomeBanner.pfnOnPaint = 0;
    WidgetAdd((tWidget *)&g_sHomePanel, (tWidget *)&g_sHomeBanner);

    CanvasInit(&g_sHomeClockBanner, &g_sKentec320x240x16_SSD2119, 240, 0, 100, 24);
    g_sHomeClockBanner.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sHomeClockBanner.ui32TextColor = ClrWhite;
    g_sHomeClockBanner.ui32FillColor = ClrDarkGray;
    g_sHomeClockBanner.psFont = &g_sFontCm16;
    g_sHomeClockBanner.pcText = g_timeString;
    WidgetAdd((tWidget *)&g_sHomePanel, (tWidget *)&g_sHomeClockBanner);

    //*************************************************************************
    // Layout Calculations for 2x2 Button Grid
    //*************************************************************************
    const int btnW = 140;
    const int btnH = 80;
    const int marginX = (320 - (btnW * 2)) / 3; // ~13px
    const int bannerH = 24;
    const int marginY = (240 - bannerH - (btnH * 2)) / 3;

    int x1 = marginX;
    int x2 = marginX * 2 + btnW;
    int y1 = bannerH + marginY;
    int y2 = bannerH + marginY * 2 + btnH;

    //*************************************************************************
    // Motor Button
    //*************************************************************************
    RectangularButtonInit(&g_sButtonMotor, &g_sKentec320x240x16_SSD2119, x1, y1, btnW, btnH);
    g_sButtonMotor.ui32Style = PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT;
    g_sButtonMotor.ui32FillColor = ClrBlue;
    g_sButtonMotor.ui32PressFillColor = ClrNavy;
    g_sButtonMotor.ui32OutlineColor = ClrWhite;
    g_sButtonMotor.ui32TextColor = ClrWhite;
    g_sButtonMotor.psFont = &g_sFontCm20;
    g_sButtonMotor.pcText = "Motor";
    g_sButtonMotor.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sHomePanel, (tWidget *)&g_sButtonMotor);

    //*************************************************************************
    // Status Button
    //*************************************************************************
    RectangularButtonInit(&g_sButtonStatus, &g_sKentec320x240x16_SSD2119, x2, y1, btnW, btnH);
    g_sButtonStatus.ui32Style = PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT;
    g_sButtonStatus.ui32FillColor = ClrGreen;
    g_sButtonStatus.ui32PressFillColor = ClrDarkGreen;
    g_sButtonStatus.ui32OutlineColor = ClrWhite;
    g_sButtonStatus.ui32TextColor = ClrWhite;
    g_sButtonStatus.psFont = &g_sFontCm20;
    g_sButtonStatus.pcText = "Status";
    g_sButtonStatus.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sHomePanel, (tWidget *)&g_sButtonStatus);

    //*************************************************************************
    // Plots Button
    //*************************************************************************
    RectangularButtonInit(&g_sButtonPlots, &g_sKentec320x240x16_SSD2119, x1, y2, btnW, btnH);
    g_sButtonPlots.ui32Style = PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT;
    g_sButtonPlots.ui32FillColor = ClrRed;
    g_sButtonPlots.ui32PressFillColor = ClrMaroon;
    g_sButtonPlots.ui32OutlineColor = ClrWhite;
    g_sButtonPlots.ui32TextColor = ClrWhite;
    g_sButtonPlots.psFont = &g_sFontCm20;
    g_sButtonPlots.pcText = "Plots";
    g_sButtonPlots.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sHomePanel, (tWidget *)&g_sButtonPlots);

    //*************************************************************************
    // Settings Button
    //*************************************************************************
    RectangularButtonInit(&g_sButtonSettings, &g_sKentec320x240x16_SSD2119, x2, y2, btnW, btnH);
    g_sButtonSettings.ui32Style = PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT;
    g_sButtonSettings.ui32FillColor = ClrDarkBlue;
    g_sButtonSettings.ui32PressFillColor = ClrBlack;
    g_sButtonSettings.ui32OutlineColor = ClrWhite;
    g_sButtonSettings.ui32TextColor = ClrWhite;
    g_sButtonSettings.psFont = &g_sFontCm20;
    g_sButtonSettings.pcText = "Settings";
    g_sButtonSettings.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sHomePanel, (tWidget *)&g_sButtonSettings);
}

/**
 * @brief Build and initialize the Motor Control screen UI.
 *
 * This screen includes motor control buttons, RPM controls,
 * live RPM display, and motor status feedback including E-Stop.
 */
void GUI_BuildMotorScreen(tContext *ctx)
{
    //*************************************************************************
    // Initialize Main Panel
    //*************************************************************************
    g_sMotorPanel.sBase.psChild = NULL;
    g_sMotorPanel.sBase.psParent = NULL;
    g_sMotorPanel.sBase.psNext = NULL;
    g_sMotorPanel.sBase.psDisplay = &g_sKentec320x240x16_SSD2119;
    g_sMotorPanel.sBase.sPosition.i16XMin = 0;
    g_sMotorPanel.sBase.sPosition.i16YMin = 0;
    g_sMotorPanel.sBase.sPosition.i16XMax = 319;
    g_sMotorPanel.sBase.sPosition.i16YMax = 239;
    g_sMotorPanel.sBase.pfnMsgProc = CanvasMsgProc;

    g_sMotorPanel.ui32Style = CANVAS_STYLE_FILL;
    g_sMotorPanel.ui32FillColor = ClrBlack;
    g_sMotorPanel.ui32TextColor = 0;
    g_sMotorPanel.psFont = 0;
    g_sMotorPanel.pcText = 0;
    g_sMotorPanel.pui8Image = 0;
    g_sMotorPanel.pfnOnPaint = 0;

    //*************************************************************************
    // Top Banner (Title + Clock)
    //*************************************************************************
    CanvasInit(&g_sMotorBanner, &g_sKentec320x240x16_SSD2119, 0, 0, 320, 24);
    g_sMotorBanner.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER;
    g_sMotorBanner.ui32FillColor = ClrDarkGray;
    g_sMotorBanner.ui32TextColor = ClrWhite;
    g_sMotorBanner.psFont = &g_sFontCm20;
    g_sMotorBanner.pcText = "Motor Control";
    WidgetAdd((tWidget *)&g_sMotorPanel, (tWidget *)&g_sMotorBanner);

    CanvasInit(&g_sMotorClockBanner, &g_sKentec320x240x16_SSD2119, 240, 0, 100, 24);
    g_sMotorClockBanner.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sMotorClockBanner.ui32TextColor = ClrWhite;
    g_sMotorClockBanner.ui32FillColor = ClrDarkGray;
    g_sMotorClockBanner.psFont = &g_sFontCm16;
    g_sMotorClockBanner.pcText = g_timeString;
    WidgetAdd((tWidget *)&g_sMotorPanel, (tWidget *)&g_sMotorClockBanner);

    //*************************************************************************
    // Start / Stop Toggle Button
    //*************************************************************************
    RectangularButtonInit(&g_sMotorToggleButton, &g_sKentec320x240x16_SSD2119, 100, 40, 120, 40);
    g_sMotorToggleButton.ui32Style = PB_STYLE_FILL | PB_STYLE_TEXT;
    g_sMotorToggleButton.psFont = &g_sFontCm20;
    g_sMotorToggleButton.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sMotorPanel, (tWidget *)&g_sMotorToggleButton);

    // E-Stop Acknowledgement Label
    CanvasInit(&g_sMotorAckLabel, &g_sKentec320x240x16_SSD2119, 0, 180, 320, 20);
    g_sMotorAckLabel.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sMotorAckLabel.ui32TextColor = ClrYellow;
    g_sMotorAckLabel.ui32FillColor = ClrBlack;
    g_sMotorAckLabel.psFont = &g_sFontCm16;
    g_sMotorAckLabel.pcText = "";
    WidgetAdd((tWidget *)&g_sMotorPanel, (tWidget *)&g_sMotorAckLabel);

    //*************************************************************************
    // RPM Slider (100–3500)
    //*************************************************************************
    SliderInit(&g_sRPMSlider, &g_sKentec320x240x16_SSD2119, 30, 95, 260, 30);
    SliderRangeSet(&g_sRPMSlider, 100, 3500);
    SliderValueSet(&g_sRPMSlider, g_i32DesiredRPM);

    g_sRPMSlider.ui32Style = SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE;
    g_sRPMSlider.ui32FillColor = ClrGray;
    g_sRPMSlider.ui32OutlineColor = ClrWhite;
    g_sRPMSlider.pfnOnChange = OnSliderChange;

    WidgetAdd((tWidget *)&g_sMotorPanel, (tWidget *)&g_sRPMSlider);

    //*************************************************************************
    // RPM Label (Below Slider)
    //*************************************************************************
    CanvasInit(&g_sRPMLabel, &g_sKentec320x240x16_SSD2119, 0, 130, 320, 20);
    g_sRPMLabel.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sRPMLabel.ui32TextColor = ClrWhite;
    g_sRPMLabel.ui32FillColor = ClrBlack;
    g_sRPMLabel.psFont = &g_sFontCm20;
    WidgetAdd((tWidget *)&g_sMotorPanel, (tWidget *)&g_sRPMLabel);
    static char rpmLabel[32];
    usnprintf(rpmLabel, sizeof(rpmLabel), "RPM: %d", g_i32DesiredRPM);
    CanvasTextSet(&g_sRPMLabel, rpmLabel);

    //*************************************************************************
    // Motor Status Panel (Running / E-Stopped / Stopped)
    //*************************************************************************
    CanvasInit(&g_sMotorStatusPanel, &g_sKentec320x240x16_SSD2119, 0, 155, 320, 24);
    g_sMotorStatusPanel.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER;
    g_sMotorStatusPanel.ui32TextColor = ClrWhite;
    g_sMotorStatusPanel.psFont = &g_sFontCm20;
    WidgetAdd((tWidget *)&g_sMotorPanel, (tWidget *)&g_sMotorStatusPanel);

    //*************************************************************************
    // Back Button (Bottom Left)
    //*************************************************************************
    RectangularButtonInit(&g_sMotorBackButton, &g_sKentec320x240x16_SSD2119, 5, 204, 70, 30);
    g_sMotorBackButton.ui32Style = PB_STYLE_FILL | PB_STYLE_TEXT;
    g_sMotorBackButton.ui32FillColor = ClrGray;
    g_sMotorBackButton.ui32PressFillColor = ClrDarkGray;
    g_sMotorBackButton.ui32OutlineColor = 0;
    g_sMotorBackButton.ui32TextColor = ClrWhite;
    g_sMotorBackButton.psFont = &g_sFontCm20;
    g_sMotorBackButton.pcText = "Back";
    g_sMotorBackButton.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sMotorPanel, (tWidget *)&g_sMotorBackButton);
}

/**
 * @brief Build and initialize the Status screen UI.
 *
 * This screen displays live sensor data and vehicle state:
 * - Motor status and RPM (actual and desired)
 * - Time, date, ambient light classification (day/night)
 * - Environmental readings: temperature, humidity, light, power usage
 * - AC system state (heating/cooling)
 */
void GUI_BuildStatusScreen(tContext *ctx)
{
    //*************************************************************************
    // Initialize Main Panel
    //*************************************************************************
    g_sStatusPanel.sBase.psChild = NULL;
    g_sStatusPanel.sBase.psParent = NULL;
    g_sStatusPanel.sBase.psNext = NULL;
    g_sStatusPanel.sBase.psDisplay = &g_sKentec320x240x16_SSD2119;
    g_sStatusPanel.sBase.sPosition.i16XMin = 0;
    g_sStatusPanel.sBase.sPosition.i16YMin = 0;
    g_sStatusPanel.sBase.sPosition.i16XMax = 319;
    g_sStatusPanel.sBase.sPosition.i16YMax = 239;
    g_sStatusPanel.sBase.pfnMsgProc = CanvasMsgProc;

    g_sStatusPanel.ui32Style = CANVAS_STYLE_FILL;
    g_sStatusPanel.ui32FillColor = ClrBlack;
    g_sStatusPanel.ui32TextColor = 0;
    g_sStatusPanel.psFont = 0;
    g_sStatusPanel.pcText = 0;
    g_sStatusPanel.pui8Image = 0;
    g_sStatusPanel.pfnOnPaint = 0;

    //*************************************************************************
    // Banner and Clock (Top Bar)
    //*************************************************************************
    CanvasInit(&g_sStatusBanner, &g_sKentec320x240x16_SSD2119, 0, 0, 320, 24);
    g_sStatusBanner.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER;
    g_sStatusBanner.ui32FillColor = ClrDarkGray;
    g_sStatusBanner.ui32TextColor = ClrWhite;
    g_sStatusBanner.psFont = &g_sFontCm20;
    g_sStatusBanner.pcText = "Status Overview";
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusBanner);

    CanvasInit(&g_sStatusClockBanner, &g_sKentec320x240x16_SSD2119, 240, 0, 100, 24);
    g_sStatusClockBanner.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusClockBanner.ui32TextColor = ClrWhite;
    g_sStatusClockBanner.ui32FillColor = ClrDarkGray; // match main banner
    g_sStatusClockBanner.psFont = &g_sFontCm16;
    g_sStatusClockBanner.pcText = g_timeString;
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusClockBanner);

    //*************************************************************************
    // Motor Status and RPM
    //*************************************************************************
    CanvasInit(&g_sStatusMotorState, &g_sKentec320x240x16_SSD2119, 0, 40, 320, 20);
    g_sStatusMotorState.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusMotorState.psFont = &g_sFontCm20;
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusMotorState);

    // Set the motor status text and colors based on state
    static char motorStatusLabel[32];
    if (eStopGetter(200) == 1)
    {
        usnprintf(motorStatusLabel, sizeof(motorStatusLabel), "Motor Status: E-STOPPED");
        g_sStatusMotorState.ui32TextColor = ClrWhite;
        g_sStatusMotorState.ui32FillColor = ClrRed;
    }
    else if (motorRunning)
    {
        usnprintf(motorStatusLabel, sizeof(motorStatusLabel), "Motor Status: RUNNING");
        g_sStatusMotorState.ui32TextColor = ClrWhite;
        g_sStatusMotorState.ui32FillColor = ClrGreen;
    }
    else
    {
        usnprintf(motorStatusLabel, sizeof(motorStatusLabel), "Motor Status: STOPPED");
        g_sStatusMotorState.ui32TextColor = ClrWhite;
        g_sStatusMotorState.ui32FillColor = ClrBlue;
    }

    // Apply the text and repaint
    CanvasTextSet(&g_sStatusMotorState, motorStatusLabel);
    WidgetPaint((tWidget *)&g_sStatusMotorState);

    CanvasInit(&g_sStatusDesiredRPM, &g_sKentec320x240x16_SSD2119, 5, 60, 150, 20);
    g_sStatusDesiredRPM.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusDesiredRPM.ui32TextColor = ClrWhite;
    g_sStatusDesiredRPM.ui32FillColor = ClrBlack;
    g_sStatusDesiredRPM.psFont = &g_sFontCm16;
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusDesiredRPM);
    static char rpmLabel[32];
    usnprintf(rpmLabel, sizeof(rpmLabel), "Desired RPM: %d", g_i32DesiredRPM);
    CanvasTextSet(&g_sStatusDesiredRPM, rpmLabel);

    CanvasInit(&g_sStatusActualRPM, &g_sKentec320x240x16_SSD2119, 5, 82, 150, 20);
    g_sStatusActualRPM.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusActualRPM.ui32TextColor = ClrWhite;
    g_sStatusActualRPM.ui32FillColor = ClrBlack;
    g_sStatusActualRPM.psFont = &g_sFontCm16;
    g_sStatusActualRPM.pcText = "Current RPM: 0"; // placeholder
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusActualRPM);

    //*************************************************************************
    // System Clock and Day/Night Classification
    //*************************************************************************
    CanvasInit(&g_sStatusClock, &g_sKentec320x240x16_SSD2119, 5, 104, 150, 20);
    g_sStatusClock.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusClock.ui32TextColor = ClrWhite;
    g_sStatusClock.ui32FillColor = ClrBlack;
    g_sStatusClock.psFont = &g_sFontCm16;
    g_sStatusClock.pcText = g_dateString;
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusClock);

    CanvasInit(&g_sStatusDayNight, &g_sKentec320x240x16_SSD2119, 5, 126, 150, 20);
    g_sStatusDayNight.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusDayNight.ui32TextColor = ClrWhite;
    g_sStatusDayNight.ui32FillColor = ClrBlack;
    g_sStatusDayNight.psFont = &g_sFontCm16;
    g_sStatusDayNight.pcText = "Ambient: --";
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusDayNight);

    //*************************************************************************
    // Environmental Sensor Readings
    //*************************************************************************
    CanvasInit(&g_sStatusTemperature, &g_sKentec320x240x16_SSD2119, 165, 60, 150, 20);
    g_sStatusTemperature.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusTemperature.ui32TextColor = ClrWhite;
    g_sStatusTemperature.ui32FillColor = ClrBlack;
    g_sStatusTemperature.psFont = &g_sFontCm16;
    g_sStatusTemperature.pcText = "Temp: -- C";
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusTemperature);

    CanvasInit(&g_sStatusHumidity, &g_sKentec320x240x16_SSD2119, 165, 82, 150, 20);
    g_sStatusHumidity.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusHumidity.ui32TextColor = ClrWhite;
    g_sStatusHumidity.ui32FillColor = ClrBlack;
    g_sStatusHumidity.psFont = &g_sFontCm16;
    g_sStatusHumidity.pcText = "Humidity: -- RH";
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusHumidity);

    CanvasInit(&g_sStatusLight, &g_sKentec320x240x16_SSD2119, 165, 104, 150, 20);
    g_sStatusLight.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusLight.ui32TextColor = ClrWhite;
    g_sStatusLight.ui32FillColor = ClrBlack;
    g_sStatusLight.psFont = &g_sFontCm16;
    g_sStatusLight.pcText = "Light: -- Lux"; // placeholder
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusLight);

    CanvasInit(&g_sStatusPower, &g_sKentec320x240x16_SSD2119, 165, 126, 150, 20);
    g_sStatusPower.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusPower.ui32TextColor = ClrWhite;
    g_sStatusPower.ui32FillColor = ClrBlack;
    g_sStatusPower.psFont = &g_sFontCm16;
    g_sStatusPower.pcText = "Power: Calculating..";
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusPower);

    CanvasInit(&g_sStatusCooling, &g_sKentec320x240x16_SSD2119, 165, 148, 150, 20);
    g_sStatusCooling.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sStatusCooling.ui32TextColor = ClrWhite;
    g_sStatusCooling.ui32FillColor = ClrBlack;
    g_sStatusCooling.psFont = &g_sFontCm16;
    g_sStatusCooling.pcText = "AC: --";
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusCooling);

    //*************************************************************************
    // Back Button (Bottom Left)
    //*************************************************************************
    RectangularButtonInit(&g_sStatusBackButton, &g_sKentec320x240x16_SSD2119, 5, 204, 70, 30);
    g_sStatusBackButton.ui32Style = PB_STYLE_FILL | PB_STYLE_TEXT;
    g_sStatusBackButton.ui32FillColor = ClrGray;
    g_sStatusBackButton.ui32PressFillColor = ClrDarkGray;
    g_sStatusBackButton.ui32OutlineColor = 0;
    g_sStatusBackButton.ui32TextColor = ClrWhite;
    g_sStatusBackButton.psFont = &g_sFontCm20;
    g_sStatusBackButton.pcText = "Back";
    g_sStatusBackButton.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sStatusPanel, (tWidget *)&g_sStatusBackButton);
}

/**
 * @brief Build and initialize the Plot screen UI.
 *
 * Displays a dynamic plot area for sensor data (Light, RPM, Power, Temp, etc.),
 * and includes controls for cycling between plots, toggling visibility, zooming,
 * and navigating back to the home screen.
 */
void GUI_BuildPlotScreen(tContext *ctx)
{
    GUI_SetActivePlot(g_currentPlot);
    //*************************************************************************
    // Initialize Main Panel
    //*************************************************************************
    g_sPlotPanel.sBase.psChild = NULL;
    g_sPlotPanel.sBase.psParent = NULL;
    g_sPlotPanel.sBase.psNext = NULL;
    g_sPlotPanel.sBase.psDisplay = &g_sKentec320x240x16_SSD2119;
    g_sPlotPanel.sBase.sPosition.i16XMin = 0;
    g_sPlotPanel.sBase.sPosition.i16YMin = 0;
    g_sPlotPanel.sBase.sPosition.i16XMax = 319;
    g_sPlotPanel.sBase.sPosition.i16YMax = 239;
    g_sPlotPanel.sBase.pfnMsgProc = CanvasMsgProc;

    g_sPlotPanel.ui32Style = CANVAS_STYLE_FILL;
    g_sPlotPanel.ui32FillColor = ClrBlack;

    //*************************************************************************
    // Top Banner (Title + Clock)
    //*************************************************************************
    CanvasInit(&g_sPlotBanner, &g_sKentec320x240x16_SSD2119, 0, 0, 320, 24);
    g_sPlotBanner.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER;
    g_sPlotBanner.ui32FillColor = ClrDarkGray;
    g_sPlotBanner.ui32TextColor = ClrWhite;
    g_sPlotBanner.psFont = &g_sFontCm20;
    g_sPlotBanner.pcText = "Sensor Plot Viewer";
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sPlotBanner);

    CanvasInit(&g_sPlotClockBanner, &g_sKentec320x240x16_SSD2119, 240, 0, 100, 24);
    g_sPlotClockBanner.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sPlotClockBanner.ui32TextColor = ClrWhite;
    g_sPlotClockBanner.ui32FillColor = ClrDarkGray;
    g_sPlotClockBanner.psFont = &g_sFontCm16;
    g_sPlotClockBanner.pcText = g_timeString;
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sPlotClockBanner);

    //*************************************************************************
    // Navigation and Control Buttons (Bottom Row)
    //*************************************************************************
    // Cycle Button
    RectangularButtonInit(&g_sPlotCycleButton, &g_sKentec320x240x16_SSD2119, 5, 30, 80, 30);
    g_sPlotCycleButton.ui32Style = PB_STYLE_FILL | PB_STYLE_TEXT;
    g_sPlotCycleButton.ui32FillColor = ClrGray;
    g_sPlotCycleButton.ui32PressFillColor = ClrDarkGray;
    g_sPlotCycleButton.ui32OutlineColor = ClrWhite;
    g_sPlotCycleButton.ui32TextColor = ClrWhite;
    g_sPlotCycleButton.psFont = &g_sFontCm20;
    g_sPlotCycleButton.pcText = plotNames[g_currentPlot];
    g_sPlotCycleButton.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sPlotCycleButton);

    // Toggle Button (right of cycle)
    RectangularButtonInit(&g_sPlotToggleButton, &g_sKentec320x240x16_SSD2119, 90, 30, 75, 30);
    g_sPlotToggleButton.ui32Style = PB_STYLE_FILL | PB_STYLE_TEXT;
    g_sPlotToggleButton.ui32FillColor = ClrGray;
    g_sPlotToggleButton.ui32PressFillColor = ClrDarkGray;
    g_sPlotToggleButton.ui32TextColor = ClrWhite;
    g_sPlotToggleButton.ui32OutlineColor = ClrWhite;
    g_sPlotToggleButton.psFont = &g_sFontCm16;
    g_sPlotToggleButton.pcText = "Hide Plot";
    g_sPlotToggleButton.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sPlotToggleButton);

    // Back Button (bottom-left)
    RectangularButtonInit(&g_sPlotBackButton, &g_sKentec320x240x16_SSD2119, 5, 204, 70, 30);
    g_sPlotBackButton.ui32Style = PB_STYLE_FILL | PB_STYLE_TEXT;
    g_sPlotBackButton.ui32FillColor = ClrGray;
    g_sPlotBackButton.ui32PressFillColor = ClrDarkGray;
    g_sPlotBackButton.ui32OutlineColor = 0;
    g_sPlotBackButton.ui32TextColor = ClrWhite;
    g_sPlotBackButton.psFont = &g_sFontCm20;
    g_sPlotBackButton.pcText = "Back";
    g_sPlotBackButton.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sPlotBackButton);

    // Zoom In Button
    RectangularButtonInit(&g_sZoomInButton, &g_sKentec320x240x16_SSD2119, 170, 30, 40, 30);
    g_sZoomInButton.ui32Style = RB_STYLE_FILL | RB_STYLE_TEXT;
    g_sZoomInButton.ui32FillColor = ClrGray;
    g_sZoomInButton.ui32PressFillColor = ClrDarkGray;
    g_sZoomInButton.ui32OutlineColor = 0;
    g_sZoomInButton.ui32TextColor = ClrWhite;
    g_sZoomInButton.psFont = &g_sFontCm20;
    g_sZoomInButton.pcText = "+";
    g_sZoomInButton.pfnOnClick = OnZoomIn;
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sZoomInButton);

    // Zoom Out Button
    RectangularButtonInit(&g_sZoomOutButton, &g_sKentec320x240x16_SSD2119, 215, 30, 40, 30);
    g_sZoomOutButton.ui32Style = RB_STYLE_FILL | RB_STYLE_TEXT;
    g_sZoomOutButton.ui32FillColor = ClrGray;
    g_sZoomOutButton.ui32PressFillColor = ClrDarkGray;
    g_sZoomOutButton.ui32OutlineColor = 0;
    g_sZoomOutButton.ui32TextColor = ClrWhite;
    g_sZoomOutButton.psFont = &g_sFontCm20;
    g_sZoomOutButton.pcText = "-";
    g_sZoomOutButton.pfnOnClick = OnZoomOut;
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sZoomOutButton);

    // Reset Button
    RectangularButtonInit(&g_sResetZoomButton, &g_sKentec320x240x16_SSD2119, 260, 30, 40, 30);
    g_sResetZoomButton.ui32Style = RB_STYLE_FILL | RB_STYLE_TEXT;
    g_sResetZoomButton.ui32FillColor = ClrGray;
    g_sResetZoomButton.ui32PressFillColor = ClrDarkGray;
    g_sResetZoomButton.ui32OutlineColor = 0;
    g_sResetZoomButton.ui32TextColor = ClrWhite;
    g_sResetZoomButton.psFont = &g_sFontCm20;
    g_sResetZoomButton.pcText = "R";
    g_sResetZoomButton.pfnOnClick = OnResetZoom;
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sResetZoomButton);

    CanvasInit(&g_sPlotUnitLabel, &g_sKentec320x240x16_SSD2119,
               5, 70, 40, 20);
    g_sPlotUnitLabel.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sPlotUnitLabel.ui32TextColor = ClrWhite;
    g_sPlotUnitLabel.ui32FillColor = ClrBlack;
    g_sPlotUnitLabel.psFont = &g_sFontCm14;
    g_sPlotUnitLabel.pcText = g_plotUnitText;
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sPlotUnitLabel);

    //*************************************************************************
    // Plot Area (Canvas drawn by app using PlotPanelPaint)
    //*************************************************************************
    CanvasInit(&g_sPlotArea, &g_sKentec320x240x16_SSD2119,
               GRAPH_LEFT, GRAPH_TOP, GRAPH_WIDTH, GRAPH_HEIGHT);
    g_sPlotArea.ui32Style = CANVAS_STYLE_APP_DRAWN;
    g_sPlotArea.pfnOnPaint = PlotPanelPaint; // calls GUI_DrawGraph
    WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sPlotArea);
}

/**
 * @brief Build and initialize the Settings screen UI.
 *
 * Allows the user to configure system thresholds:
 * - Acceleration limit (m/s^2)
 * - Minimum distance (cm)
 * - Maximum current (A)
 *
 * Each threshold is controlled using a slider, with a live label display.
 * Includes a back button and banner with clock.
 */
void GUI_BuildSettingsScreen(tContext *ctx)
{
    static char label[32];
    //*************************************************************************
    // Initialize Main Panel
    //*************************************************************************
    g_sSettingsPanel.sBase.psChild = NULL;
    g_sSettingsPanel.sBase.psParent = NULL;
    g_sSettingsPanel.sBase.psNext = NULL;
    g_sSettingsPanel.sBase.psDisplay = &g_sKentec320x240x16_SSD2119;
    g_sSettingsPanel.sBase.sPosition.i16XMin = 0;
    g_sSettingsPanel.sBase.sPosition.i16YMin = 0;
    g_sSettingsPanel.sBase.sPosition.i16XMax = 319;
    g_sSettingsPanel.sBase.sPosition.i16YMax = 239;
    g_sSettingsPanel.sBase.pfnMsgProc = CanvasMsgProc;

    g_sSettingsPanel.ui32Style = CANVAS_STYLE_FILL;
    g_sSettingsPanel.ui32FillColor = ClrBlack;
    g_sSettingsPanel.ui32TextColor = 0;
    g_sSettingsPanel.psFont = 0;
    g_sSettingsPanel.pcText = 0;
    g_sSettingsPanel.pui8Image = 0;
    g_sSettingsPanel.pfnOnPaint = 0;

    //*************************************************************************
    // Top Banner (Title + Clock)
    //*************************************************************************
    CanvasInit(&g_sSettingsBanner, &g_sKentec320x240x16_SSD2119, 0, 0, 320, 24);
    g_sSettingsBanner.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER;
    g_sSettingsBanner.ui32FillColor = ClrDarkGray;
    g_sSettingsBanner.ui32TextColor = ClrWhite;
    g_sSettingsBanner.psFont = &g_sFontCm20;
    g_sSettingsBanner.pcText = "Settings";
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sSettingsBanner);

    CanvasInit(&g_sSettingsClockBanner, &g_sKentec320x240x16_SSD2119, 240, 0, 100, 24);
    g_sSettingsClockBanner.ui32Style = CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_HCENTER | CANVAS_STYLE_TEXT_OPAQUE;
    g_sSettingsClockBanner.ui32TextColor = ClrWhite;
    g_sSettingsClockBanner.ui32FillColor = ClrDarkGray; // match main banner
    g_sSettingsClockBanner.psFont = &g_sFontCm16;
    g_sSettingsClockBanner.pcText = g_timeString;
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sSettingsClockBanner);

    //*************************************************************************
    // Threshold Sliders and Labels
    //*************************************************************************
    int yStart = 35;
    int spacing = 45;

    // === Acceleration Limit Slider ===
    SliderInit(&g_sTempHeatingSlider, &g_sKentec320x240x16_SSD2119, 30, yStart, 260, 20);
    SliderRangeSet(&g_sTempHeatingSlider, -10, 25); // m/s^2
    SliderValueSet(&g_sTempHeatingSlider, 10);
    g_sTempHeatingSlider.ui32Style = SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE;
    g_sTempHeatingSlider.ui32FillColor = ClrGray;
    g_sTempHeatingSlider.ui32OutlineColor = ClrWhite;
    g_sTempHeatingSlider.pfnOnChange = OnSliderChange;
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sTempHeatingSlider);
    static char heatingLabel[32];
    CanvasInit(&g_sTempHeatingLabel, &g_sKentec320x240x16_SSD2119, 0, yStart + 20, 320, 20);
    g_sTempHeatingLabel.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT |
                              CANVAS_STYLE_TEXT_OPAQUE | CANVAS_STYLE_TEXT_HCENTER;
    g_sTempHeatingLabel.ui32FillColor = ClrBlack;
    g_sTempHeatingLabel.ui32TextColor = ClrWhite;
    g_sTempHeatingLabel.psFont = &g_sFontCm20;
    usnprintf(heatingLabel, sizeof(heatingLabel), "Heating Threshold: %d C", g_i32HeatingTempThreshold);
    CanvasTextSet(&g_sTempHeatingLabel, heatingLabel);
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sTempHeatingLabel);

    // === Distance Limit Slider ===
    yStart += spacing;
    static char coolingLabel[32];
    SliderInit(&g_sTempCooling, &g_sKentec320x240x16_SSD2119, 30, yStart, 260, 20);
    SliderRangeSet(&g_sTempCooling, 0, 35); // cm
    SliderValueSet(&g_sTempCooling, 20);
    g_sTempCooling.ui32Style = SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE;
    g_sTempCooling.ui32FillColor = ClrGray;
    g_sTempCooling.ui32OutlineColor = ClrWhite;
    g_sTempCooling.pfnOnChange = OnSliderChange;
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sTempCooling);

    CanvasInit(&g_sTempCoolingLabel, &g_sKentec320x240x16_SSD2119, 0, yStart + 20, 320, 20);
    g_sTempCoolingLabel.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT |
                                 CANVAS_STYLE_TEXT_OPAQUE | CANVAS_STYLE_TEXT_HCENTER;
    g_sTempCoolingLabel.ui32FillColor = ClrBlack;
    g_sTempCoolingLabel.ui32TextColor = ClrWhite;
    g_sTempCoolingLabel.psFont = &g_sFontCm20;
    usnprintf(coolingLabel, sizeof(coolingLabel), "Cooling Threshold: %d C", g_i32CoolingTempThreshold);
    CanvasTextSet(&g_sTempCoolingLabel, coolingLabel);
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sTempCoolingLabel);

    // === Current Limit Slider ===
    val maxcurrentlimit;
    uint8_t errorFlag = getter(&maxCurrentLimit, &maxcurrentlimit, portMAX_DELAY);
    yStart += spacing;
    SliderInit(&g_sCurrentSlider, &g_sKentec320x240x16_SSD2119, 30, yStart, 260, 20);
    SliderRangeSet(&g_sCurrentSlider, 0, 1000); // Amps
    SliderValueSet(&g_sCurrentSlider, (uint32_t)maxcurrentlimit.raw);
    g_sCurrentSlider.ui32Style = SL_STYLE_FILL | SL_STYLE_BACKG_FILL | SL_STYLE_OUTLINE;
    g_sCurrentSlider.ui32FillColor = ClrGray;
    g_sCurrentSlider.ui32OutlineColor = ClrWhite;
    g_sCurrentSlider.pfnOnChange = OnSliderChange;
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sCurrentSlider);

    CanvasInit(&g_sCurrentLabel, &g_sKentec320x240x16_SSD2119, 0, yStart + 20, 320, 20);
    g_sCurrentLabel.ui32Style = CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT |
                                CANVAS_STYLE_TEXT_OPAQUE | CANVAS_STYLE_TEXT_HCENTER;
    g_sCurrentLabel.ui32FillColor = ClrBlack;
    g_sCurrentLabel.ui32TextColor = ClrWhite;
    g_sCurrentLabel.psFont = &g_sFontCm20;
    
    usnprintf(label, sizeof(label), "Current Limit: %d mA", (int32_t)maxcurrentlimit.raw);
    CanvasTextSet(&g_sCurrentLabel, label);
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sCurrentLabel);

    //*************************************************************************
    // E-Stop Test Button (Bottom Right)
    //*************************************************************************
    RectangularButtonInit(&g_sEStopTestButton, &g_sKentec320x240x16_SSD2119, 215, 204, 100, 30);
    g_sEStopTestButton.ui32Style = PB_STYLE_FILL | PB_STYLE_TEXT;
    g_sEStopTestButton.ui32FillColor = ClrRed;
    g_sEStopTestButton.ui32PressFillColor = ClrMaroon;
    g_sEStopTestButton.ui32TextColor = ClrWhite;
    g_sEStopTestButton.ui32OutlineColor = ClrWhite;
    g_sEStopTestButton.psFont = &g_sFontCm16;
    g_sEStopTestButton.pcText = "Test E-Stop";
    g_sEStopTestButton.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sEStopTestButton);

    //*************************************************************************
    // Back Button (Bottom Left)
    //*************************************************************************
    RectangularButtonInit(&g_sSettingsBackButton, &g_sKentec320x240x16_SSD2119, 5, 204, 70, 30);
    g_sSettingsBackButton.ui32Style = PB_STYLE_FILL | PB_STYLE_TEXT;
    g_sSettingsBackButton.ui32FillColor = ClrGray;
    g_sSettingsBackButton.ui32PressFillColor = ClrDarkGray;
    g_sSettingsBackButton.ui32OutlineColor = 0;
    g_sSettingsBackButton.ui32TextColor = ClrWhite;
    g_sSettingsBackButton.psFont = &g_sFontCm20;
    g_sSettingsBackButton.pcText = "Back";
    g_sSettingsBackButton.pfnOnClick = OnButtonPress;
    WidgetAdd((tWidget *)&g_sSettingsPanel, (tWidget *)&g_sSettingsBackButton);
}

/**
 * @brief Callback for all navigation and action buttons.
 *
 * This function handles screen navigation, motor control toggling, plot cycling,
 * and plot visibility toggling based on which button was pressed.
 *
 * @param psWidget Pointer to the widget that triggered the callback.
 */
static void OnButtonPress(tWidget *psWidget)
{
    // === Navigation Buttons to Switch Screens ===

    // Motor screen button pressed
    if (psWidget == (tWidget *)&g_sButtonMotor)
    {
        GUI_SetScreen(SCREEN_MOTOR); // Switch to motor control screen
        UpdateMotorUI();             // Refresh motor UI elements
        // UARTprintf("Switched to Screen: Motor\n");
    }
    // Status screen button pressed
    else if (psWidget == (tWidget *)&g_sButtonStatus)
    {
        GUI_SetScreen(SCREEN_STATUS); // Switch to status screen
        // UARTprintf("Switched to Screen: Status\n");
    }
    // Plot screen button pressed
    else if (psWidget == (tWidget *)&g_sButtonPlots)
    {
        GUI_SetScreen(SCREEN_PLOTS); // Switch to sensor plot screen
        // UARTprintf("Switched to Screen: Plots\n");
    }
    // Settings screen button pressed
    else if (psWidget == (tWidget *)&g_sButtonSettings)
    {
        GUI_SetScreen(SCREEN_SETTINGS); // Switch to settings screen
        // UARTprintf("Switched to Screen: Settings\n");
    }

    // === Back Buttons to Return to Home Screen ===

    else if (psWidget == (tWidget *)&g_sMotorBackButton)
    {
        GUI_SetScreen(SCREEN_HOME); // Return from motor screen
        // UARTprintf("Switched from Motor to HOME\n");
    }
    else if (psWidget == (tWidget *)&g_sStatusBackButton)
    {
        GUI_SetScreen(SCREEN_HOME); // Return from status screen
        // UARTprintf("Switched from Status to HOME\n");
    }
    else if (psWidget == (tWidget *)&g_sPlotBackButton)
    {
        GUI_SetScreen(SCREEN_HOME); // Return from plots screen
        // UARTprintf("Switched from Plot to HOME\n");
    }
    else if (psWidget == (tWidget *)&g_sSettingsBackButton)
    {
        GUI_SetScreen(SCREEN_HOME); // Return from settings screen
        // UARTprintf("Switched from Settings to HOME\n");
    }

    // === Motor Toggle Button (Start/Stop or Acknowledge E-stop) ===

    else if (psWidget == (tWidget *)&g_sMotorToggleButton)
    {
        if (eStopGetter(200) == 1)
        {
            // Acknowledge e-stop condition
            eStopSetter(0, portMAX_DELAY);
            eStopWasAcknowledged = true;
            motorRunning = false;
            // UARTprintf("E-Stop acknowledged\n");
        }
        else
        {
            // Toggle motor running state
            motorRunning = !motorRunning;
            setterVal(&rpmData, motorRunning ? (double)g_i32DesiredRPM : 0, 0, portMAX_DELAY);
            eStopWasAcknowledged = false;
        }

        UpdateMotorUI(); // Refresh UI to reflect new state
    }

    // === Plot Cycle Button (Switch Between Sensor Plots) ===

    else if (psWidget == (tWidget *)&g_sPlotCycleButton)
    {
        g_currentPlot = (g_currentPlot + 1) % 6; // Cycle through 5 plot types

        GUI_SetActivePlot(g_currentPlot); // Reset buffer and scaling

        // Update button and banner label text
        const char *plotNames[] = {"Light", "Speed", "Power", "Temp", "Humidity", "Current"};
        g_sPlotCycleButton.pcText = plotNames[g_currentPlot];

        // Repaint affected widgets
        WidgetPaint((tWidget *)&g_sPlotCycleButton);
        WidgetPaint((tWidget *)&g_sPlotBanner);
        WidgetPaint((tWidget *)&g_sPlotArea);

        // UARTprintf("Switched to Plot: %s\n", plotNames[g_currentPlot]);
    }

    // === Plot Toggle Button (Show/Hide Plot) ===

    else if (psWidget == (tWidget *)&g_sPlotToggleButton)
    {
        g_bPlotVisible = !g_bPlotVisible; // Toggle visibility state

        // Update button label accordingly
        g_sPlotToggleButton.pcText = g_bPlotVisible ? "Hide Plot" : "Show Plot";
        WidgetPaint((tWidget *)&g_sPlotToggleButton);

        // Refresh the plot area
        WidgetRemove((tWidget *)&g_sPlotArea);
        WidgetAdd((tWidget *)&g_sPlotPanel, (tWidget *)&g_sPlotArea);
        WidgetPaint((tWidget *)&g_sPlotArea);
    }

    // === Test E-Stop Button ===
    // Manually trigger an emergency stop for testing purposes.
    // This simulates a sensor threshold breach and should activate
    // the E-Stop acknowledgment logic in the motor screen.

    else if (psWidget == (tWidget *)&g_sEStopTestButton)
    {
        eStopSetter(1, portMAX_DELAY);
        motorRunning = false; // Stop motor if running
        // UARTprintf("[Test] E-Stop triggered manually.\n");
    }
}

/**
 * @brief Updates the Motor screen UI elements based on motor and E-Stop state.
 *
 * This function changes the appearance and label of the motor control button
 * and status panel depending on whether the motor is running, stopped, or in
 * an emergency stop state. It also displays a temporary acknowledgment message
 * when the E-Stop is cleared.
 */
void UpdateMotorUI(void)
{
    // If an emergency stop is active, show "Acknowledge" button and red status
    if (eStopGetter(200) == 1)
    {
        g_sMotorToggleButton.pcText = "Acknowledge";
        g_sMotorToggleButton.ui32FillColor = ClrOrange;
        g_sMotorToggleButton.ui32PressFillColor = ClrDarkOrange;

        g_sMotorStatusPanel.ui32FillColor = ClrRed;
        g_sMotorStatusPanel.pcText = "Status: E-STOPPED";
    }
    // If motor is running and no E-Stop, show "Stop" button and green status
    else if (motorRunning)
    {
        g_sMotorToggleButton.pcText = "Stop";
        g_sMotorToggleButton.ui32FillColor = ClrRed;
        g_sMotorToggleButton.ui32PressFillColor = ClrMaroon;

        g_sMotorStatusPanel.ui32FillColor = ClrGreen;
        g_sMotorStatusPanel.pcText = "Status: RUNNING";

        eStopWasAcknowledged = false;
        CanvasTextSet(&g_sMotorAckLabel, "");
    }
    // If motor is stopped and no E-Stop, show "Start" button and blue status
    else
    {
        g_sMotorToggleButton.pcText = "Start";
        g_sMotorToggleButton.ui32FillColor = ClrGreen;
        g_sMotorToggleButton.ui32PressFillColor = ClrDarkGreen;

        g_sMotorStatusPanel.ui32FillColor = ClrBlue;
        g_sMotorStatusPanel.pcText = "Status: STOPPED";

        if (eStopWasAcknowledged)
        {
            eStopWasAcknowledged = false; // Reset flag after displaying message
            CanvasTextSet(&g_sMotorAckLabel, "E-Stop Acknowledged");
        }
        else
        {
            CanvasTextSet(&g_sMotorAckLabel, "");
        }
    }

    // Repaint all modified widgets to reflect changes on screen
    WidgetPaint((tWidget *)&g_sMotorAckLabel);
    WidgetPaint((tWidget *)&g_sMotorToggleButton);
    WidgetPaint((tWidget *)&g_sMotorStatusPanel);
}

/**
 * @brief Callback function triggered when a GUI slider is moved.
 *
 * This function updates the associated value and label text for each of the following:
 * - Desired motor RPM
 * - Acceleration limit
 * - Distance limit
 * - Current limit
 *
 * It also updates the GUI label and sends a UART debug message.
 *
 * @param psWidget The pointer to the slider widget that was changed.
 * @param value The new value from the slider.
 */
void OnSliderChange(tWidget *psWidget, int32_t value)
{
    static char label[32]; // Buffer for constructing label text

    // === Handle RPM Slider ===
    if (psWidget == (tWidget *)&g_sRPMSlider)
    {
        // Update desired RPM with new slider value
        g_i32DesiredRPM = value;
        setterVal(&rpmData, motorRunning ? (double)g_i32DesiredRPM : 0, 0, portMAX_DELAY);
        // Update the RPM label text and repaint it on the screen
        usnprintf(label, sizeof(label), "RPM: %d", value);
        CanvasTextSet(&g_sRPMLabel, label);
        WidgetPaint((tWidget *)&g_sRPMLabel);

        // Debug output to UART
        // UARTprintf("Slider Changed: Desired RPM = %d\n", g_i32DesiredRPM);
    }

    // === Handle Acceleration Limit Slider ===
    else if (psWidget == (tWidget *)&g_sTempHeatingSlider)
    {
        g_i32HeatingTempThreshold = value;

        // Update label to reflect new acceleration limit
        usnprintf(label, sizeof(label), "Heating Threshold: %d C", value);
        CanvasTextSet(&g_sTempHeatingLabel, label);
        WidgetPaint((tWidget *)&g_sTempHeatingLabel);

        // UARTprintf("Slider Changed: Acceleration Limit = %d m/s^2\n", g_i32HeatingTempThreshold);
    }

    // === Handle Distance Limit Slider ===
    else if (psWidget == (tWidget *)&g_sTempCooling)
    {
        g_i32CoolingTempThreshold = value;

        // Update label to reflect new distance limit
        usnprintf(label, sizeof(label), "Cooling Threshold: %d C", value);
        CanvasTextSet(&g_sTempCoolingLabel, label);
        WidgetPaint((tWidget *)&g_sTempCoolingLabel);

        // UARTprintf("Slider Changed: Distance Limit = %d cm\n", g_i32CoolingTempThreshold);
    }

    // === Handle Current Limit Slider ===
    else if (psWidget == (tWidget *)&g_sCurrentSlider)
    {
        g_i32CurrentLimit = value;
        setterVal(&maxCurrentLimit, (double)g_i32CurrentLimit, 0, portMAX_DELAY);
        // Update label to reflect new current limit
        val maxcurrentlimit;
        uint8_t errorFlag = getter(&maxCurrentLimit, &maxcurrentlimit, portMAX_DELAY);
        usnprintf(label, sizeof(label), "Current Limit: %d mA", (int32_t)maxcurrentlimit.raw);
        CanvasTextSet(&g_sCurrentLabel, label);
        WidgetPaint((tWidget *)&g_sCurrentLabel);

        // UARTprintf("Slider Changed: Current Limit = %d mA\n", g_i32CurrentLimit);
    }
}

/**
 * @brief Custom paint handler for the plot panel canvas.
 *
 * This function is responsible for rendering the graph area.
 * It clears the background, and depending on the visibility flag,
 * either draws the graph or a "Plot Hidden" message.
 *
 * @param psWidget Pointer to the widget being painted (not used here).
 * @param ctx Pointer to the graphics context for drawing.
 */
void PlotPanelPaint(tWidget *psWidget, tContext *ctx)
{
    // === Define Plot Area Dimensions ===
    const int left = GRAPH_LEFT;
    const int top = GRAPH_TOP;
    const int width = GRAPH_WIDTH;
    const int height = GRAPH_HEIGHT;

    // Define the rectangle region for the graph
    tRectangle graphRect = {left, top, left + width, top + height};

    // === Clear the Plot Area ===
    GrContextForegroundSet(ctx, ClrBlack); // Set fill color to black
    GrRectFill(ctx, &graphRect);           // Fill the plot rectangle to clear old content

    // === Check if Plot is Visible ===
    if (!g_bPlotVisible)
    {
        // If not visible, show "Plot Hidden" text in gray
        GrContextForegroundSet(ctx, ClrGray); // Set text color to gray
        GrContextFontSet(ctx, &g_sFontCm16);  // Set font size
        GrStringDrawCentered(
            ctx,
            "Plot Hidden",    // Text to display
            -1,               // Length (-1 = auto)
            left + width / 2, // X center of the plot area
            top + height / 2, // Y center of the plot area
            false             // Don't opaque background
        );
        return; // Exit early since plot is not visible
    }

    // === Render the Actual Graph ===
    GUI_DrawGraph(ctx); // Call the graph drawing routine
}
/* @} */
