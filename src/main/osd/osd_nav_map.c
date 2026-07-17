/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// OSD navigation minimap: a character-cell map of home, the stored flight
// plan and the flown trail. Rendering uses only glyphs present in the
// standard Betaflight font so MSP DisplayPort goggles (DJI O3/O4, HDZero,
// Walksnail) work without font changes.
//
// The map is emitted one row-string per OSD scheduler pass (the same
// multi-pass mechanism the artificial horizon and stick overlay use), so a
// full map costs mapRows passes per OSD frame.
//
// All map geometry is local ENU centimetres relative to home (home is the
// origin); waypoints are converted from their stored absolute coordinates
// once per frame.

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OSD_NAV_MAP

#include "common/maths.h"
#include "common/printf.h"
#include "common/vector.h"

#include "drivers/display.h"
#include "drivers/osd_symbols.h"

#include "fc/runtime_config.h"

#include "flight/flight_plan_nav.h"
#include "flight/imu.h"
#include "flight/nav_trail.h"

#include "io/gps.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "osd/osd_nav_map.h"

#include "pg/flight_plan.h"
#include "pg/osd_nav_map.h"

#define NAV_MAP_COLS          14
#define NAV_MAP_ROWS          8
#define NAV_MAP_CELL_ASPECT   1.5f   // character cells are ~1.5x taller than wide
#define NAV_MAP_FIT_MARGIN    1.25f  // auto-zoom head-room around fitted points
#define NAV_MAP_ZOOM_IN_FILL  0.8f   // zoom in once content comfortably fits the smaller step
#define NAV_MAP_MIN_SCALE_M   20
#define NAV_MAP_OVER_HOME_CM  1000.0f
#define NAV_MAP_MARKER_MAX    9      // single-glyph markers: digits '1'..'9' only

typedef struct navMapFrame_s {
    uint8_t grid[NAV_MAP_ROWS][NAV_MAP_COLS];
    // projection
    float centerEastCm;
    float centerNorthCm;
    float cmPerCol;
    float cmPerRow;
    float sinRot;
    float cosRot;
    bool rotated;
    int16_t mapRotationDeg;   // rotation actually applied to the view
} navMapFrame_t;

static navMapFrame_t frame;
static bool framePrepared = false;
static uint8_t renderRow = 0;
static uint32_t autoScaleM = 0;
static uint8_t animPhase = 0;
static uint8_t trailBarRowPlusOne[NAV_MAP_COLS];   // per-frame stroke dedup, 0 = column clean
static uint32_t lastScaleShownM = 0;
static uint8_t scaleFlashFrames = 0;
static bool scaleZoomedOut = false;

static uint16_t wrapDeg360(int32_t deg)
{
    deg %= 360;
    if (deg < 0) {
        deg += 360;
    }
    return deg;
}

uint32_t osdNavMapSelectScaleM(uint32_t requiredWidthM)
{
    static const uint16_t scaleStepsM[] = { 20, 30, 50, 75, 100, 150, 200, 300, 400, 600, 800, 1200,
                                            1600, 2400, 3200, 4800, 6400, 9600, 12800, 19200, 25600, 38400, 51200 };
    for (unsigned i = 0; i < ARRAYLEN(scaleStepsM); i++) {
        if (requiredWidthM <= scaleStepsM[i]) {
            return scaleStepsM[i];
        }
    }
    return scaleStepsM[ARRAYLEN(scaleStepsM) - 1];
}

// modifier waypoints mutate executor state; their lat/lon carry no position
static bool waypointIsPositional(const waypoint_t *wp)
{
    switch ((waypointType_e)wp->type) {
    case WAYPOINT_TYPE_FLYOVER:
    case WAYPOINT_TYPE_FLYBY:
    case WAYPOINT_TYPE_HOLD:
    case WAYPOINT_TYPE_LAND:
    case WAYPOINT_TYPE_TAKEOFF:
        return true;
    default:
        return false;
    }
}

static void waypointEnuCm(const waypoint_t *wp, vector2_t *posCm)
{
    const gpsLocation_t loc = { .lat = wp->latitude, .lon = wp->longitude, .altCm = 0 };
    GPS_distance2d(&GPS_home_llh, &loc, posCm);
}

// rotate a home-relative ENU position into screen space (x right, y up)
static void worldToView(const vector2_t *posCm, float *viewX, float *viewY)
{
    const float relEast = posCm->x - frame.centerEastCm;
    const float relNorth = posCm->y - frame.centerNorthCm;
    if (frame.rotated) {
        *viewX = relEast * frame.cosRot - relNorth * frame.sinRot;
        *viewY = relEast * frame.sinRot + relNorth * frame.cosRot;
    } else {
        *viewX = relEast;
        *viewY = relNorth;
    }
}

// convert view space to fractional interior cell coordinates
static void viewToCellF(float viewX, float viewY, float *colF, float *rowF)
{
    const int interiorCols = NAV_MAP_COLS - 2;
    const int interiorRows = NAV_MAP_ROWS - 2;
    *colF = (interiorCols - 1) * 0.5f + viewX / frame.cmPerCol;
    *rowF = (interiorRows - 1) * 0.5f - viewY / frame.cmPerRow;
}

static bool cellInInterior(int col, int row)
{
    return col >= 0 && col < NAV_MAP_COLS - 2 && row >= 0 && row < NAV_MAP_ROWS - 2;
}

// plot into the interior (col/row are interior coordinates; border is +1)
static void plotCell(int col, int row, uint8_t glyph)
{
    if (cellInInterior(col, row)) {
        frame.grid[row + 1][col + 1] = glyph;
    }
}

static void plotCellClamped(int col, int row, uint8_t glyph)
{
    col = constrain(col, 0, NAV_MAP_COLS - 3);
    row = constrain(row, 0, NAV_MAP_ROWS - 3);
    frame.grid[row + 1][col + 1] = glyph;
}

// draw a solid line between two world points, clipped to the visible area
static void plotWorldLine(const vector2_t *fromCm, const vector2_t *toCm)
{
    float x0, y0, x1, y1;
    worldToView(fromCm, &x0, &y0);
    worldToView(toCm, &x1, &y1);

    // Liang-Barsky clip in view space against the visible rectangle
    const float halfW = (NAV_MAP_COLS - 2) * 0.5f * frame.cmPerCol;
    const float halfH = (NAV_MAP_ROWS - 2) * 0.5f * frame.cmPerRow;
    float t0 = 0.0f, t1 = 1.0f;
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float p[4] = { -dx, dx, -dy, dy };
    const float q[4] = { x0 + halfW, halfW - x0, y0 + halfH, halfH - y0 };
    for (int i = 0; i < 4; i++) {
        if (fabsf(p[i]) < 1e-6f) {
            if (q[i] < 0.0f) {
                return; // parallel and outside
            }
        } else {
            const float t = q[i] / p[i];
            if (p[i] < 0.0f) {
                t0 = fmaxf(t0, t);
            } else {
                t1 = fminf(t1, t);
            }
        }
    }
    if (t0 > t1) {
        return;
    }

    float colF0, rowF0, colF1, rowF1;
    viewToCellF(x0 + t0 * dx, y0 + t0 * dy, &colF0, &rowF0);
    viewToCellF(x0 + t1 * dx, y0 + t1 * dy, &colF1, &rowF1);

    // scanline stroke: shallow lines place exactly ONE artificial-horizon bar
    // glyph per column at the line's true height (nine sub-cell positions,
    // index 0 = top of cell); steep lines place one thin vertical bar per
    // row. One glyph per scan step means the stroke is always a single cell
    // thick - no doubled ink when the line rides a cell boundary.
    const float dCol = colF1 - colF0;
    const float dRow = rowF1 - rowF0;
    const float adCol = fabsf(dCol);
    const float adRow = fabsf(dRow);

    if (adCol < 0.05f && adRow < 0.05f) {
        const int row = lrintf(rowF0);
        const float d = rowF0 - row;
        plotCell(lrintf(colF0), row, SYM_AH_BAR9_0 + constrain(lrintf((d + 0.5f) * 8.0f), 0, 8));
        return;
    }

    if (adRow <= adCol) {
        const int cStart = lrintf(fminf(colF0, colF1));
        const int cEnd = lrintf(fmaxf(colF0, colF1));
        for (int col = cStart; col <= cEnd; col++) {
            const float t = constrainf(((float)col - colF0) / dCol, 0.0f, 1.0f);
            const float rF = rowF0 + t * dRow;
            const int row = lrintf(rF);
            // polyline joins: consecutive segments share an endpoint column and
            // can round it to rows one apart, doubling the stroke - one bar per
            // column per frame keeps the stroke single-cell thick
            if (col >= 0 && col < NAV_MAP_COLS) {
                if (trailBarRowPlusOne[col] != 0 && trailBarRowPlusOne[col] != (uint8_t)(row + 1)) {
                    continue;
                }
                trailBarRowPlusOne[col] = (uint8_t)(row + 1);
            }
            const float d = rF - row;
            plotCell(col, row, SYM_AH_BAR9_0 + constrain(lrintf((d + 0.5f) * 8.0f), 0, 8));
        }
    } else {
        const int rStart = lrintf(fminf(rowF0, rowF1));
        const int rEnd = lrintf(fmaxf(rowF0, rowF1));
        for (int row = rStart; row <= rEnd; row++) {
            const float t = constrainf(((float)row - rowF0) / dRow, 0.0f, 1.0f);
            const int col = lrintf(colF0 + t * dCol);
            plotCell(col, row, SYM_STICK_OVERLAY_VERTICAL);
        }
    }
}

// one continuous smoothed line from launch to the craft. Points are laid
// down by distance flown - hovering adds nothing, the line just waits -
// and the final segment tracks the craft so the path extends live.
static void plotTrail(const vector2_t *craftCm, bool positionValid)
{
    const unsigned count = navTrailCount();
    vector2_t prev = { .x = 0.0f, .y = 0.0f };
    bool prevValid = false;
    for (unsigned i = 0; i < count; i++) {
        const navTrailPoint_t *pt = navTrailPointAt(i);
        const vector2_t cur = { .x = pt->eastM * 100.0f, .y = pt->northM * 100.0f };
        if (prevValid) {
            plotWorldLine(&prev, &cur);
        }
        prev = cur;
        prevValid = true;
    }
    if (prevValid && positionValid) {
        plotWorldLine(&prev, craftCm);
    }
}

// radar-style range ring: tick marks around home at a round distance, sized
// to sit inside the map. It rescales with the zoom, so zoom steps read as
// geometry changing size instead of a number changing value.
static void plotRangeRing(uint32_t scaleM)
{
    static const uint16_t ringStepsM[] = { 25, 50, 100, 250, 500, 1000, 2500, 5000, 10000, 25000 };
    const float halfHeightM = (NAV_MAP_ROWS - 2) * 0.5f * frame.cmPerRow / 100.0f;
    const float maxRadiusM = 0.85f * fminf(0.5f * scaleM, halfHeightM);
    uint32_t radiusM = 0;
    for (unsigned i = 0; i < ARRAYLEN(ringStepsM); i++) {
        if (ringStepsM[i] <= maxRadiusM) {
            radiusM = ringStepsM[i];
        }
    }
    if (radiusM == 0) {
        return;
    }
    const float radiusCm = radiusM * 100.0f;
    static const float tickEast[4]  = { 0.0f, 1.0f, 0.0f, -1.0f };
    static const float tickNorth[4] = { 1.0f, 0.0f, -1.0f, 0.0f };
    for (int i = 0; i < 4; i++) {   // ticks at the circle's compass points
        const vector2_t p = { .x = tickEast[i] * radiusCm, .y = tickNorth[i] * radiusCm };
        float vx, vy, cF, rF;
        worldToView(&p, &vx, &vy);
        viewToCellF(vx, vy, &cF, &rF);
        plotCell(lrintf(cF), lrintf(rF), '+');
    }
}

static void worldToCellRounded(const vector2_t *posCm, int *col, int *row)
{
    float viewX, viewY, colF, rowF;
    worldToView(posCm, &viewX, &viewY);
    viewToCellF(viewX, viewY, &colF, &rowF);
    *col = lrintf(colF);
    *row = lrintf(rowF);
}

static void plotMarker(const vector2_t *posCm, uint8_t glyph, bool clampToEdge)
{
    int col, row;
    worldToCellRounded(posCm, &col, &row);
    if (clampToEdge) {
        plotCellClamped(col, row, glyph);
    } else {
        plotCell(col, row, glyph);
    }
}

static void drawBorderAndScale(uint32_t scaleM)
{
    // frame with the stick-overlay box-drawing glyphs
    for (int col = 0; col < NAV_MAP_COLS; col++) {
        frame.grid[0][col] = SYM_STICK_OVERLAY_HORIZONTAL;
        frame.grid[NAV_MAP_ROWS - 1][col] = SYM_STICK_OVERLAY_HORIZONTAL;
    }
    for (int row = 1; row < NAV_MAP_ROWS - 1; row++) {
        frame.grid[row][0] = SYM_STICK_OVERLAY_VERTICAL;
        frame.grid[row][NAV_MAP_COLS - 1] = SYM_STICK_OVERLAY_VERTICAL;
    }

    // top-border orientation cue: which way is up
    if (frame.rotated) {
        frame.grid[0][1] = SYM_ARROW_SMALL_UP;          // nose-up
    } else {
        frame.grid[0][1] = 'N';
        frame.grid[0][2] = SYM_ARROW_SMALL_UP;
    }

    // map width (scale) embedded in the bottom border, right aligned,
    // bracketed so it reads as "this map is this wide", not telemetry
    char valText[10];
    const int32_t converted = lrintf(osdGetMetersToSelectedUnit(scaleM));
    if (converted >= 10000) {
        tfp_sprintf(valText, "%d.%dK", (int)(converted / 1000), (int)((converted % 1000) / 100));
    } else {
        tfp_sprintf(valText, "%d%c", (int)converted, osdGetMetersToSelectedUnitSymbol());
    }

    // the scale label only appears for ~2 s after a zoom step - pulsing
    // doubled-outward when zooming out, inverted when zooming in - then
    // vanishes. The range ring carries the sense of scale the rest of the
    // time, so the map stays clean.
    if (scaleM != lastScaleShownM) {
        scaleFlashFrames = 24;
        // the first scale (map appearing) announces itself without the pulse
        scaleZoomedOut = (lastScaleShownM != 0) && (scaleM > lastScaleShownM);
        if (lastScaleShownM == 0) {
            scaleFlashFrames = 18;
        }
        lastScaleShownM = scaleM;
    }
    if (scaleFlashFrames == 0) {
        return;
    }
    scaleFlashFrames--;
    char scaleText[16];
    if (scaleFlashFrames > 12 && ((animPhase >> 1) & 1)) {
        if (scaleZoomedOut) {
            tfp_sprintf(scaleText, "%c%c%s%c%c", SYM_AH_LEFT, SYM_AH_LEFT, valText, SYM_AH_RIGHT, SYM_AH_RIGHT);
        } else {
            tfp_sprintf(scaleText, "%c%s%c", SYM_AH_RIGHT, valText, SYM_AH_LEFT);
        }
    } else {
        tfp_sprintf(scaleText, "%c%s%c", SYM_AH_LEFT, valText, SYM_AH_RIGHT);
    }
    const int len = strlen(scaleText);
    const int start = NAV_MAP_COLS - 1 - len;
    if (start > 0) {
        for (int i = 0; i < len; i++) {
            frame.grid[NAV_MAP_ROWS - 1][start + i] = scaleText[i];
        }
    }
}

static void drawCenteredMapText(const char *text)
{
    const int len = strlen(text);
    int start = (NAV_MAP_COLS - len) / 2;
    if (start < 1) {
        start = 1;
    }
    for (int i = 0; i < len && (start + i) < NAV_MAP_COLS - 1; i++) {
        frame.grid[NAV_MAP_ROWS / 2][start + i] = text[i];
    }
}

// numbered markers at the plan's positional waypoints; modifier entries
// (ALT_CHANGE, DELAY, YAW_RATE) are skipped - their lat/lon are meaningless.
// Numbering follows the plan index so the map matches the CLI/OSD "WP n"
// readouts. The waypoint the executor is flying to blinks.
static void plotWaypointMarkers(void)
{
    const uint8_t count = MIN(flightPlanConfig()->waypointCount, (uint8_t)MAX_WAYPOINTS);
    const bool navActive = flightPlanNavIsActive();
    const uint8_t activeIndex = flightPlanNavGetCurrentIndex();

    for (uint8_t i = 0; i < count && i < NAV_MAP_MARKER_MAX; i++) {
        const waypoint_t *wp = &flightPlanConfig()->waypoints[i];
        if (!waypointIsPositional(wp)) {
            continue;
        }
        if (navActive && i == activeIndex && ((animPhase >> 2) & 1)) {
            continue;   // blink the active marker
        }
        vector2_t posCm;
        waypointEnuCm(wp, &posCm);
        plotMarker(&posCm, '1' + i, false);
    }
}

static void prepareFrame(void)
{
    const osdNavMapConfig_t *cfg = osdNavMapConfig();

    memset(frame.grid, SYM_BLANK, sizeof(frame.grid));
    memset(trailBarRowPlusOne, 0, sizeof(trailBarRowPlusOne));
    animPhase++;

    const bool homeValid = STATE(GPS_FIX_HOME);
    const bool fixValid = STATE(GPS_FIX);
    const bool positionValid = homeValid && fixValid;

    const vector2_t home = { .x = 0.0f, .y = 0.0f };
    vector2_t craft = home;
    if (positionValid) {
        GPS_distance2d(&GPS_home_llh, &gpsSol.llh, &craft);
    }
    const int headingDeg = wrapDeg360(DECIDEGREES_TO_DEGREES((int32_t)attitude.values.yaw));

    // orientation: heading-up turns the map with the aircraft's nose, so home
    // sits at the top of the map exactly when the craft faces home
    frame.rotated = (cfg->mode == OSD_NAV_MAP_MODE_HEADING_UP) && positionValid;
    frame.mapRotationDeg = frame.rotated ? headingDeg : 0;
    if (frame.rotated) {
        const float rotRad = DEGREES_TO_RADIANS(frame.mapRotationDeg);
        frame.sinRot = sin_approx(rotRad);
        frame.cosRot = cos_approx(rotRad);
    } else {
        frame.sinRot = 0.0f;
        frame.cosRot = 1.0f;
    }

    // view centre
    if (cfg->centre == OSD_NAV_MAP_CENTRE_CRAFT) {
        frame.centerEastCm = craft.x;
        frame.centerNorthCm = craft.y;
    } else {
        frame.centerEastCm = 0.0f;
        frame.centerNorthCm = 0.0f;
    }

    // auto-zoom: fit the craft and every positional waypoint of the plan.
    // Home is deliberately NOT fitted: with home at the view centre it
    // contributes nothing, and on a craft-centred map the off-map edge arrow
    // carries the home direction so the local view can stay tight.
    const int interiorCols = NAV_MAP_COLS - 2;
    const int interiorRows = NAV_MAP_ROWS - 2;
    float maxX = 100.0f, maxY = 100.0f;
    {
        float viewX, viewY;
        worldToView(&craft, &viewX, &viewY);
        maxX = fmaxf(maxX, fabsf(viewX));
        maxY = fmaxf(maxY, fabsf(viewY));
    }
    if (homeValid) {
        const uint8_t count = MIN(flightPlanConfig()->waypointCount, (uint8_t)MAX_WAYPOINTS);
        for (uint8_t i = 0; i < count; i++) {
            const waypoint_t *wp = &flightPlanConfig()->waypoints[i];
            if (!waypointIsPositional(wp)) {
                continue;
            }
            vector2_t posCm;
            float viewX, viewY;
            waypointEnuCm(wp, &posCm);
            worldToView(&posCm, &viewX, &viewY);
            maxX = fmaxf(maxX, fabsf(viewX));
            maxY = fmaxf(maxY, fabsf(viewY));
        }
    }
    // convert the height requirement into an equivalent width requirement
    const float heightAsWidth = maxY * interiorCols / (NAV_MAP_CELL_ASPECT * interiorRows);
    const uint32_t requiredM = lrintf(2.0f * fmaxf(maxX, heightAsWidth) * NAV_MAP_FIT_MARGIN / 100.0f);
    uint32_t newScale = osdNavMapSelectScaleM(requiredM);
    const uint32_t minScaleM = MAX(cfg->minScaleM, NAV_MAP_MIN_SCALE_M);
    if (newScale < minScaleM) {
        newScale = minScaleM;
    }
    // anchored expansion: grow immediately so nothing falls off the map, but
    // only zoom back in once the content comfortably fits the smaller step -
    // stops the scale flapping at a step boundary
    if (autoScaleM == 0 || newScale > autoScaleM
        || (newScale < autoScaleM && requiredM < NAV_MAP_ZOOM_IN_FILL * newScale)) {
        autoScaleM = newScale;
    }
    const uint32_t scaleM = autoScaleM;
    frame.cmPerCol = scaleM * 100.0f / interiorCols;
    frame.cmPerRow = frame.cmPerCol * NAV_MAP_CELL_ASPECT;

    drawBorderAndScale(scaleM);

    if (!homeValid) {
        drawCenteredMapText(fixValid ? "SET HOME" : "WAIT GPS");
        return;
    }

    // layers, lowest priority first
    plotRangeRing(scaleM);
    plotTrail(&craft, positionValid);
    plotWaypointMarkers();

    // home: flag when in view; when it leaves the map, a direction arrow
    // pinned to the nearest edge shows which way home lies
    {
        float viewX, viewY, colF, rowF;
        worldToView(&home, &viewX, &viewY);
        viewToCellF(viewX, viewY, &colF, &rowF);
        const int homeCol = lrintf(colF);
        const int homeRow = lrintf(rowF);
        if (cellInInterior(homeCol, homeRow)) {
            plotCell(homeCol, homeRow, SYM_HOMEFLAG);
        } else {
            const int screenBearing = lrintf(RADIANS_TO_DEGREES(atan2_approx(viewX, viewY)));
            plotCellClamped(homeCol, homeRow,
                            osdGetDirectionSymbolFromHeading(wrapDeg360(screenBearing)));
        }
    }

    // craft marker: over-home ring when (nearly) on top of home, otherwise an
    // arrow showing the nose direction. Nose-up maps show the arrow straight
    // up by definition.
    if (positionValid) {
        if (vector2Norm(&craft) < fminf(frame.cmPerCol, NAV_MAP_OVER_HOME_CM)) {
            plotMarker(&craft, SYM_OVER_HOME, true);
        } else {
            const int screenHeading = frame.rotated ? 0 : headingDeg;
            plotMarker(&craft, osdGetDirectionSymbolFromHeading(screenHeading), true);
        }
    } else {
        drawCenteredMapText("NO GPS");
    }
}

#ifdef UNIT_TEST
void osdNavMapResetRenderStateForTest(void)
{
    framePrepared = false;
    renderRow = 0;
    autoScaleM = 0;
    animPhase = 0;
    lastScaleShownM = 0;
    scaleFlashFrames = 0;
    scaleZoomedOut = false;
}

uint32_t osdNavMapScaleMForTest(void)
{
    return autoScaleM;
}
#endif

void osdElementNavMap(osdElementParms_t *element)
{
    const displayPort_t *displayPort = element->osdDisplayPort;

    if (!framePrepared) {
        prepareFrame();
        framePrepared = true;
        renderRow = 0;
    }

    // vertical clip: rows beyond the screen are dropped
    if (element->elemPosY + renderRow >= displayPort->rows) {
        element->drawElement = false;
        framePrepared = false;
        return;
    }

    element->attr = DISPLAYPORT_SEVERITY_NORMAL;
    memcpy(element->buff, frame.grid[renderRow], NAV_MAP_COLS);
    element->buff[NAV_MAP_COLS] = '\0';

    // horizontal clip against the right screen edge
    const int maxLen = displayPort->cols - element->elemPosX;
    if (maxLen <= 0) {
        element->drawElement = false;
        framePrepared = false;
        return;
    }
    if ((int)strlen((char *)element->buff) > maxLen) {
        element->buff[maxLen] = '\0';
    }

    element->elemOffsetY = renderRow;

    renderRow++;
    if (renderRow < NAV_MAP_ROWS) {
        element->rendered = false;
    } else {
        framePrepared = false;
    }
}

#endif // USE_OSD_NAV_MAP
