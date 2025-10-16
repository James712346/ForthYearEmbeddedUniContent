/**
 * @addtogroup gui
 * @{
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <grlib/grlib.h>
#include "gui_display.h"
#include "gui.h"
#include "utils/ustdlib.h"
#include "uartstdio.h"
#include "widget.h"
#include "canvas.h"
#include "gui_widgets.h"

//*****************************************************************************
// Data Structures
//*****************************************************************************

/**
 * @brief Structure representing a single sample (data point).
 */
typedef struct
{
    float value;
} GraphSample;

/**
 * @brief Structure representing a buffer of samples for a specific sensor type.
 */
typedef struct
{
    GraphSample samples[MAX_POINTS]; // Circular buffer of samples
    uint16_t index;                  // Index to write the next sample
    uint16_t maxPoints;              // Total buffer size (MAX_POINTS)
    float maxValue;                  // Maximum Y-axis scale for this plot
} PlotData;

//*****************************************************************************
// Plot Buffers for Each Sensor Type
//*****************************************************************************

static PlotData plotLight = {.maxPoints = MAX_POINTS, .maxValue = 1000.0f};   // Lux
static PlotData plotSpeed = {.maxPoints = MAX_POINTS, .maxValue = 5000.0f};   // RPM
static PlotData plotPower = {.maxPoints = MAX_POINTS, .maxValue = 50.0f};     // Watts
static PlotData plotTemp = {.maxPoints = MAX_POINTS, .maxValue = 60.0f};      // °C
static PlotData plotHumidity = {.maxPoints = MAX_POINTS, .maxValue = 100.0f}; // RH
static PlotData plotCurrent = {.maxPoints = MAX_POINTS, .maxValue = 1000.0f}; // mA

static PlotData *activePlot = &plotLight; // Currently active plot buffer

extern tCanvasWidget g_sPlotArea;
extern plot_type_t g_currentPlot;

char g_plotUnitText[16] = "--";

extern bool g_bPlotVisible; // Controlled by GUI toggle

void OnZoomIn(tWidget *psWidget)
{
    if (activePlot && activePlot->maxValue > 10)
    {
        activePlot->maxValue /= 1.5f;
        WidgetPaint((tWidget *)&g_sPlotArea);
    }
}

void OnZoomOut(tWidget *psWidget)
{
    if (activePlot && activePlot->maxValue < 100000)
    {
        activePlot->maxValue *= 1.5f;
        WidgetPaint((tWidget *)&g_sPlotArea);
    }
}

void OnResetZoom(tWidget *psWidget)
{
    switch (g_currentPlot)
    {
    case PLOT_LIGHT:
        activePlot->maxValue = 3000.0f;
        break;
    case PLOT_SPEED:
        activePlot->maxValue = 5000.0f;
        break;
    case PLOT_POWER:
        activePlot->maxValue = 50.0f;
        break;
    case PLOT_TEMPERATURE:
        activePlot->maxValue = 60.0f;
        break;
    case PLOT_HUMIDITY:
        activePlot->maxValue = 100.0f;
        break;
    case PLOT_CURRENT:
        activePlot->maxValue = 1000.0f;
        break;
    }

    WidgetPaint((tWidget *)&g_sPlotArea);
}

//*****************************************************************************
// GUI_AddDataPoint
//*****************************************************************************

/**
 * @brief Adds a new data point to the active plot's circular buffer.
 *
 * Automatically wraps around when the buffer reaches its limit.
 *
 * @param value New sensor value to add (Y-axis).
 */
void GUI_AddDataPoint(float value)
{
    if (!activePlot)
        return;

    activePlot->samples[activePlot->index].value = value;
    activePlot->index = (activePlot->index + 1) % activePlot->maxPoints;
}

//*****************************************************************************
// GUI_DrawGraph
//*****************************************************************************

/**
 * @brief Draws the active plot's data as a line graph on the screen.
 *
 * Clears the canvas area first, then connects sequential data points.
 *
 * @param ctx Pointer to an initialized graphics context.
 */
void GUI_DrawGraph(tContext *ctx)
{
    const int left = GRAPH_LEFT;
    const int top = GRAPH_TOP;
    const int width = GRAPH_WIDTH;
    const int height = GRAPH_HEIGHT;

    tRectangle graphRect = {left, top, left + width, top + height};

    // Clear background
    GrContextForegroundSet(ctx, ClrBlack);
    GrRectFill(ctx, &graphRect);

    // Skip if plot is not visible or not initialized
    if (!activePlot || !g_bPlotVisible)
        return;

    GrContextForegroundSet(ctx, ClrBlue); // Text color
    GrContextFontSet(ctx, &g_sFontCm12);  // Label font

    const int numTicks = 5;
    char label[8];
    const int labelInset = 4; // space between plot border and text

    for (int i = 0; i <= numTicks; ++i)
    {
        float value = (activePlot->maxValue / numTicks) * i;
        int y = top + height - (int)((value / activePlot->maxValue) * height);

        usnprintf(label, sizeof(label), "%d", (int)value);

        // Draw label *inside* the plot area, slightly offset from left edge
        int labelY = y - 6;

        // Prevent clipping at the top
        if (i == numTicks)
        {
            labelY += 6; // push it down
        }

        // Prevent clipping at the bottom
        if (i == 0)
        {
            labelY -= 4; // raise it slightly
        }

        GrStringDraw(ctx, label, -1, left + labelInset, labelY, false);

        // Optional: horizontal grid line for visual alignment
        GrLineDrawH(ctx, left, left + width, y);
    }

    GrContextForegroundSet(ctx, ClrRed);

    // Draw connected lines between samples
    for (int i = 1; i < activePlot->maxPoints; i++)
    {
        int idx1 = (activePlot->index + i - 1) % activePlot->maxPoints;
        int idx2 = (activePlot->index + i) % activePlot->maxPoints;

        int x1 = left + ((i - 1) * width) / activePlot->maxPoints;
        int x2 = left + (i * width) / activePlot->maxPoints;

        float v1 = activePlot->samples[idx1].value;
        float v2 = activePlot->samples[idx2].value;

        int y1 = top + height - (int)((v1 / activePlot->maxValue) * height);
        int y2 = top + height - (int)((v2 / activePlot->maxValue) * height);

        GrLineDraw(ctx, x1, y1, x2, y2);
    }
}

//*****************************************************************************
// GUI_SetActivePlot
//*****************************************************************************

/**
 * @brief Sets the active plot buffer based on the selected sensor type.
 *
 * Also resets the buffer index to start a fresh plot for the new type.
 *
 * @param plotType Enum value indicating which sensor to display.
 */
void GUI_SetActivePlot(plot_type_t plotType)
{
    switch (plotType)
    {
    case PLOT_LIGHT:
        activePlot = &plotLight;
        usnprintf(g_plotUnitText, sizeof(g_plotUnitText), "Lux");
        break;
    case PLOT_SPEED:
        activePlot = &plotSpeed;
        usnprintf(g_plotUnitText, sizeof(g_plotUnitText), "RPM");
        break;
    case PLOT_POWER:
        activePlot = &plotPower;
        usnprintf(g_plotUnitText, sizeof(g_plotUnitText), "W");
        break;
    case PLOT_TEMPERATURE:
        activePlot = &plotTemp;
        usnprintf(g_plotUnitText, sizeof(g_plotUnitText), "C");
        break;
    case PLOT_HUMIDITY:
        activePlot = &plotHumidity;
        usnprintf(g_plotUnitText, sizeof(g_plotUnitText), "%%RH");
        break;
    case PLOT_CURRENT:
        activePlot = &plotCurrent;
        usnprintf(g_plotUnitText, sizeof(g_plotUnitText), "mA");
        break;
    default:
        activePlot = &plotLight;
        usnprintf(g_plotUnitText, sizeof(g_plotUnitText), "Lux");
        break;
    }

    // Request redraw of the label
    WidgetPaint((tWidget *)&g_sPlotUnitLabel);

    if (activePlot)
    {
        // Reset buffer index to start drawing from left
        activePlot->index = 0;

        // Optional: clear sample data
        // memset(activePlot->samples, 0, sizeof(activePlot->samples));
    }
}
/** @} */
