#ifndef GUI_DISPLAY_H
#define GUI_DISPLAY_H

#include <stdint.h>
#include <grlib/grlib.h>
#include "gui.h"

//*****************************************************************************
// Graph Constants
//*****************************************************************************

#define MAX_POINTS 100   // Maximum number of data points in the plot buffer
#define GRAPH_LEFT 45    // X offset from the left edge of the screen
#define GRAPH_TOP 70     // Y offset from the top of the screen
#define GRAPH_WIDTH 260  // Width of the plotting area in pixels
#define GRAPH_HEIGHT 120 // Height of the plotting area in pixels

//*****************************************************************************
// Plot Control API
//*****************************************************************************

void OnZoomIn(tWidget *psWidget);
void OnZoomOut(tWidget *psWidget);
void OnResetZoom(tWidget *psWidget);

/**
 * @brief Selects which sensor plot to display.
 *
 * This updates the active buffer and label for rendering.
 *
 * @param plotType Enum value of the selected sensor plot type.
 */
void GUI_SetActivePlot(plot_type_t plotType);

/**
 * @brief Adds a new data point to the current plot buffer.
 *
 * This value is scaled to fit within the plot area. When the buffer
 * reaches MAX_POINTS, older values are shifted to make room for new ones.
 *
 * @param value Y-axis value (scaled for screen height)
 */
void GUI_AddDataPoint(float value);

/**
 * @brief Renders the current plot to the screen.
 *
 * Draws all stored data points as a line graph on the screen.
 * Intended to be called from the Canvas paint handler.
 *
 * @param ctx Pointer to an initialized graphics context.
 */
void GUI_DrawGraph(tContext *ctx);

#endif // GUI_DISPLAY_H
