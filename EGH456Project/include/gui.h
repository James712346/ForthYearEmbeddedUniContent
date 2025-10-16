#ifndef GUI_H
#define GUI_H

#include <grlib/grlib.h>
#include <grlib/widget.h>


typedef enum {
    SCREEN_HOME,
    SCREEN_MOTOR,
    SCREEN_STATUS,
    SCREEN_PLOTS,
    SCREEN_SETTINGS
} ScreenState;

typedef enum {
    PLOT_LIGHT,
    PLOT_SPEED,
    PLOT_POWER,
    PLOT_TEMPERATURE,
    PLOT_HUMIDITY,
    PLOT_CURRENT,
    NUM_PLOTS
} plot_type_t;


// void GUI_Init(void);             Not needed, as display initilisation is called in main.c
void GUI_SetScreen(ScreenState s);  // Switch active screen
void GUI_Update(void);              // Refresh current screen (sensor values)
void GUI_Task(void *params);        // GUI FreeRTOS task
uint8_t CreateGUITask(void);
uint8_t CreateGUISensorTask(void); // Create GUI sensor task
uint8_t CreateLEDTask(void);
extern tWidget *g_pActiveScreen;
extern char g_timeString[32];       // Current time string for display
extern char g_dateString[32];
extern plot_type_t g_currentPlot;





#endif