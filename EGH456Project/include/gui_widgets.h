#ifndef GUI_WIDGETS_H
#define GUI_WIDGETS_H

#include <grlib/grlib.h>
#include <widget.h>
#include <canvas.h>
#include <pushbutton.h>
#include "gui_display.h" // Shared GUI display configuration

//*****************************************************************************
// Widget Declarations for Each Screen
//*****************************************************************************

// === Screen Panels ===
extern tCanvasWidget g_sHomePanel;       // Home screen container
extern tCanvasWidget g_sMotorPanel;      // Motor screen container
extern tCanvasWidget g_sStatusPanel;     // Status screen container
extern tCanvasWidget g_sPlotPanel;       // Plot screen container
extern tCanvasWidget g_sSettingsPanel;   // Settings screen container

// === Clock Banners (Top Banner Clock for Each Screen) ===
extern tCanvasWidget g_sHomeClockBanner;
extern tCanvasWidget g_sMotorClockBanner;
extern tCanvasWidget g_sPlotClockBanner;
extern tCanvasWidget g_sSettingsClockBanner;
extern tCanvasWidget g_sStatusClockBanner;

// === Plot Screen Widgets ===
extern tPushButtonWidget g_sPlotCycleButton; // Button to cycle through plot types
extern tPushButtonWidget g_sZoomButton;      // Placeholder for zoom functionality
extern tCanvasWidget g_sPlotArea;            // Canvas used for live sensor plotting
extern tCanvasWidget g_sPlotUnitLabel;

// === Status Screen Widgets ===
extern tCanvasWidget g_sStatusLight;         // Light sensor display
extern tCanvasWidget g_sStatusTemperature;   // Temperature reading display
extern tCanvasWidget g_sStatusHumidity;      // Humidity reading display
extern tCanvasWidget g_sStatusActualRPM;     // Actual motor RPM display
extern tCanvasWidget g_sStatusPower;         // Motor power usage display
extern tCanvasWidget g_sStatusClock;         // Current time display
extern tCanvasWidget g_sStatusDayNight;      // Day/Night mode label
extern tCanvasWidget g_sStatusCooling;       // AC status label (Cooling/Heating)

//*****************************************************************************
// Screen Builder Functions
//*****************************************************************************

/**
 * @brief Construct and initialize all widgets for each screen.
 */
void GUI_BuildHomeScreen(tContext *ctx);
void GUI_BuildMotorScreen(tContext *ctx);
void GUI_BuildStatusScreen(tContext *ctx);
void GUI_BuildPlotScreen(tContext *ctx);
void GUI_BuildSettingsScreen(tContext *ctx);

//*****************************************************************************
// Shared Application State
//*****************************************************************************

extern uint32_t currentRPM; // Current RPM used across screens
extern bool motorRunning; // Motor running state

#endif // GUI_WIDGETS_H
