/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

// Navigation HUD OSD renderer: rasterises the state prepared by
// flight/nav_hud.c into a character-cell minimap plus status footers, and an
// optional flight-director element. Rendering uses only glyphs present in the
// standard Betaflight font so MSP DisplayPort goggles (DJI O3/O4, HDZero,
// Walksnail) work without font changes.
//
// The map is emitted one row-string per OSD scheduler pass (the same
// multi-pass mechanism the artificial horizon and stick overlay use), so a
// full map costs mapRows + footer passes per OSD frame.

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OSD_NAV_HUD

#include "common/maths.h"
#include "common/printf.h"
#include "common/vector.h"

#include "drivers/display.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"

#include "flight/nav_hud.h"
#ifdef USE_NAV_MISSION
#include "flight/nav_mission.h"
#endif

#if defined(USE_GPS_RESCUE) && !defined(USE_WING)
#include "flight/gps_rescue.h"
#endif

#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "osd/osd_nav_hud.h"

#include "pg/nav_hud.h"

#define NAV_HUD_MAP_MAX_COLS   27
#define NAV_HUD_MAP_MAX_ROWS   13
#define NAV_HUD_MAX_FOOTERS    4
#define NAV_HUD_SPRITE_COUNT   3      // vertical sub-positions of the stick overlay dot sprites
#define NAV_HUD_CELL_ASPECT    1.5f   // character cells are ~1.5x taller than wide
#define NAV_HUD_FIT_MARGIN     1.25f  // auto-zoom head-room around fitted points
#define NAV_HUD_ZOOM_IN_FILL   0.8f       // zoom in once content comfortably fits the smaller step
#define NAV_HUD_HOME_CENTER_MIN_SCALE_M 150  // home-centred maps never zoom in past this: arriving
                                             // home reads as flying from the map edge into the centre
#define NAV_HUD_HOME_ICON_ONLY_CM 914      // 30 ft

typedef struct navHudFrame_s {
    uint8_t mapCols;
    uint8_t mapRows;
    uint8_t footerCount;
    uint8_t grid[NAV_HUD_MAP_MAX_ROWS][NAV_HUD_MAP_MAX_COLS];
    char footer[NAV_HUD_MAX_FOOTERS][NAV_HUD_MAP_MAX_COLS + 1];
    uint8_t footerAttr[NAV_HUD_MAX_FOOTERS];
    // projection
    float centerEastCm;
    float centerNorthCm;
    float cmPerCol;
    float cmPerRow;
    float sinRot;
    float cosRot;
    bool rotated;
    int16_t mapRotationDeg;   // rotation actually applied to the view
} navHudFrame_t;

static navHudFrame_t frame;
static bool framePrepared = false;
static uint8_t renderRow = 0;
static uint32_t autoScaleM = 0;
static uint8_t navHudAnimPhase = 0;
static bool wasHoldingDisplay = false;
static uint8_t holdEngagePhase = 0;
static uint8_t expandRowShift = 0;
static uint8_t expandColShift = 0;
static uint8_t trailBarRowPlusOne[NAV_HUD_MAP_MAX_COLS];   // per-frame stroke dedup, 0 = column clean
static uint32_t lastScaleShownM = 0;
static uint8_t scaleFlashFrames = 0;
static bool scaleZoomedOut = false;

static navHudMode_e effectiveMode(const navHudState_t *state)
{
    navHudMode_e mode = navHudConfig()->mode;
    if (mode != NAV_HUD_MODE_OFF && navHudConfig()->rescueExpand && mode < NAV_HUD_MODE_FULL
        && (state->rescueActive || navHudWaypointCount() > 0)) {
        // more map when it matters: during rescue, and whenever a waypoint
        // session is underway (grows up/right via the overflow anchoring)
        mode++;
    }
    return mode;
}

static void mapSizeForMode(navHudMode_e mode, uint8_t *cols, uint8_t *rows)
{
    switch (mode) {
    case NAV_HUD_MODE_COMPACT:
        *cols = 14;
        *rows = 8;
        break;
    case NAV_HUD_MODE_FULL:
        *cols = 27;
        *rows = 13;
        break;
    case NAV_HUD_MODE_STANDARD:
    default:
        *cols = 19;
        *rows = 9;
        break;
    }
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
    const int interiorCols = frame.mapCols - 2;
    const int interiorRows = frame.mapRows - 2;
    *colF = (interiorCols - 1) * 0.5f + viewX / frame.cmPerCol;
    *rowF = (interiorRows - 1) * 0.5f - viewY / frame.cmPerRow;
}

static bool cellInInterior(int col, int row)
{
    return col >= 0 && col < frame.mapCols - 2 && row >= 0 && row < frame.mapRows - 2;
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
    col = constrain(col, 0, frame.mapCols - 3);
    row = constrain(row, 0, frame.mapRows - 3);
    frame.grid[row + 1][col + 1] = glyph;
}

typedef enum {
    ROUTE_STYLE_TRAIL,         // solid sub-cell stroke - where you HAVE flown
    ROUTE_STYLE_GUIDE,         // dashed sub-cell stroke - where you're being guided
} routeStyle_e;

// draw a line between two world points, clipped to the visible area
static void plotWorldLine(const vector2_t *fromCm, const vector2_t *toCm, routeStyle_e style)
{
    float x0, y0, x1, y1;
    worldToView(fromCm, &x0, &y0);
    worldToView(toCm, &x1, &y1);

    // Liang-Barsky clip in view space against the visible rectangle
    const float halfW = (frame.mapCols - 2) * 0.5f * frame.cmPerCol;
    const float halfH = (frame.mapRows - 2) * 0.5f * frame.cmPerRow;
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
    // thick - no doubled ink when the line rides a cell boundary. Guide
    // strokes draw every other step, anchored to absolute grid parity so the
    // dashes don't crawl as the endpoints move.
    const float dCol = colF1 - colF0;
    const float dRow = rowF1 - rowF0;
    const float adCol = fabsf(dCol);
    const float adRow = fabsf(dRow);
    const bool dashed = (style == ROUTE_STYLE_GUIDE);

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
            if (dashed && (col & 1)) {
                continue;
            }
            const float t = constrainf(((float)col - colF0) / dCol, 0.0f, 1.0f);
            const float rF = rowF0 + t * dRow;
            const int row = lrintf(rF);
            // polyline joins: consecutive segments share an endpoint column and
            // can round it to rows one apart, doubling the stroke - one bar per
            // column per frame keeps the solid stroke single-cell thick
            if (!dashed && col >= 0 && col < NAV_HUD_MAP_MAX_COLS) {
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
            if (dashed && (row & 1)) {
                continue;
            }
            const float t = constrainf(((float)row - rowF0) / dRow, 0.0f, 1.0f);
            const int col = lrintf(colF0 + t * dCol);
            plotCell(col, row, SYM_STICK_OVERLAY_VERTICAL);
        }
    }
}

static void plotTrail(const navHudState_t *state)
{
    // one continuous smoothed line from launch to the craft. Points are laid
    // down by distance flown - hovering adds nothing, the line just waits -
    // and the final segment tracks the craft so the path extends live as you
    // fly. Segments flown during rescue draw in ':' so the rescue path can
    // be compared against the outbound one.
    const unsigned count = navHudTrailCount();
    vector2_t prev = { .x = 0.0f, .y = 0.0f };
    bool prevValid = false;
    for (unsigned i = 0; i < count; i++) {
        const navHudTrailPoint_t *pt = navHudTrailPointAt(i);
        const vector2_t cur = { .x = pt->eastM * 100.0f, .y = pt->northM * 100.0f };
        if (prevValid) {
            plotWorldLine(&prev, &cur, ROUTE_STYLE_TRAIL);
        }
        prev = cur;
        prevValid = true;
    }
    if (prevValid && state->positionValid) {
        plotWorldLine(&prev, &state->craftPosCm, ROUTE_STYLE_TRAIL);
    }
}

// radar-style range ring: a faint dotted circle around home at a round
// distance, sized to sit inside the map. It rescales with the zoom, so zoom
// steps read as geometry changing size instead of a number changing value.
static void plotRangeRing(uint32_t scaleM)
{
    static const uint16_t ringStepsM[] = { 25, 50, 100, 250, 500, 1000, 2500, 5000, 10000, 25000 };
    const float halfHeightM = (frame.mapRows - 2) * 0.5f * frame.cmPerRow / 100.0f;
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
    for (int i = 0; i < 4; i++) {   // ticks at the circle's compass points:
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

// hold-point marker with a slow shimmer: the dots around the anchor alternate
// between the horizontal and vertical neighbours roughly every 0.7 s
static void plotHoldAnchor(const vector2_t *posCm)
{
    int col, row;
    worldToCellRounded(posCm, &col, &row);

    if ((navHudAnimPhase >> 3) & 1) {
        plotCell(col, row - 1, SYM_STICK_OVERLAY_SPRITE_LOW);   // dot hugging from above
        plotCell(col, row + 1, SYM_STICK_OVERLAY_SPRITE_HIGH);  // dot hugging from below
    } else {
        plotCell(col - 1, row, SYM_STICK_OVERLAY_SPRITE_MID);
        plotCell(col + 1, row, SYM_STICK_OVERLAY_SPRITE_MID);
    }
    plotCell(col, row, SYM_STICK_OVERLAY_CENTER);
}

// centred ground-speed readout in the top border: [dial icon][value][unit glyph],
// set off from the border line by one blank cell each side.
// Returns the last column used, or 0 when nothing was drawn.
static int drawTopSpeed(const navHudState_t *state)
{
    if (!navHudConfig()->showSpeed || !state->gpsValid) {
        return 0;
    }

    char text[10];
    int pos = 0;
    if (state->headingSuspect && ((navHudAnimPhase >> 3) & 1)) {
        // the compass is fighting the GPS course: surface it where the pilot looks
        pos = tfp_sprintf(text, "MAG?");
    } else {
        text[pos++] = SYM_SPEED;
        pos += tfp_sprintf(text + pos, "%d", (int)osdGetSpeedToSelectedUnit(state->groundSpeedCmS));
        text[pos++] = osdGetSpeedToSelectedUnitSymbol();
        text[pos] = '\0';
    }

    const int len = pos;
    int start = (frame.mapCols - len) / 2;
    if (start < 6) {
        start = 6;  // keep clear of the corner cue / satellite count
    }
    if (start + len >= frame.mapCols - 1) {
        return 0;
    }

    frame.grid[0][start - 1] = SYM_BLANK;
    for (int i = 0; i < len; i++) {
        frame.grid[0][start + i] = text[i];
    }
    frame.grid[0][start + len] = SYM_BLANK;
    return start + len;
}

static void drawTopSatelliteCount(const navHudState_t *state, int minStartCol)
{
    char satText[8];
    tfp_sprintf(satText, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, state->satellites);

    const int len = strlen(satText);
    const int start = frame.mapCols - 1 - len;
    if (start <= 3 || start <= minStartCol + 1) {
        return;  // the speed readout has priority in the top border
    }

    for (int i = 0; i < len; i++) {
        frame.grid[0][start + i] = satText[i];
    }
}

static void drawBorderAndLabels(const navHudState_t *state, uint32_t scaleM)
{
    // frame with the stick-overlay box-drawing glyphs
    for (int col = 0; col < frame.mapCols; col++) {
        frame.grid[0][col] = SYM_STICK_OVERLAY_HORIZONTAL;
        frame.grid[frame.mapRows - 1][col] = SYM_STICK_OVERLAY_HORIZONTAL;
    }
    for (int row = 1; row < frame.mapRows - 1; row++) {
        frame.grid[row][0] = SYM_STICK_OVERLAY_VERTICAL;
        frame.grid[row][frame.mapCols - 1] = SYM_STICK_OVERLAY_VERTICAL;
    }

    // top-border corner: with home pinned at the centre a "which way is home"
    // cue is redundant, so the corner carries the satellite count instead
    if (navHudConfig()->center == NAV_HUD_CENTER_HOME) {
        char satText[8];
        tfp_sprintf(satText, "%c%c%d", SYM_SAT_L, SYM_SAT_R, state->satellites);
        for (int i = 0; satText[i] && i + 2 < frame.mapCols; i++) {
            frame.grid[0][1 + i] = satText[i];
        }
    } else if (state->homeValid && state->positionValid) {
        const int homeScreenDeg = frame.rotated
            ? navHudWrapDeg360(state->bearingToHomeDeg - frame.mapRotationDeg)
            : state->bearingToHomeDeg;
        frame.grid[0][1] = SYM_HOMEFLAG;
        if (state->distanceToHomeCm >= NAV_HUD_HOME_ICON_ONLY_CM) {
            frame.grid[0][2] = osdGetDirectionSymbolFromHeading(homeScreenDeg);
        }
    } else if (frame.rotated) {
        frame.grid[0][1] = SYM_ARROW_SMALL_UP;
    } else {
        frame.grid[0][1] = 'N';
        frame.grid[0][2] = SYM_ARROW_SMALL_UP;
    }

    const int speedEndCol = drawTopSpeed(state);
    if (navHudConfig()->center != NAV_HUD_CENTER_HOME) {
        drawTopSatelliteCount(state, speedEndCol);
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
    if (scaleFlashFrames > 12 && ((navHudAnimPhase >> 1) & 1)) {
        if (scaleZoomedOut) {
            tfp_sprintf(scaleText, "%c%c%s%c%c", SYM_AH_LEFT, SYM_AH_LEFT, valText, SYM_AH_RIGHT, SYM_AH_RIGHT);
        } else {
            tfp_sprintf(scaleText, "%c%s%c", SYM_AH_RIGHT, valText, SYM_AH_LEFT);
        }
    } else {
        tfp_sprintf(scaleText, "%c%s%c", SYM_AH_LEFT, valText, SYM_AH_RIGHT);
    }
    const int len = strlen(scaleText);
    const int start = frame.mapCols - 1 - len;
    if (start > 0) {
        for (int i = 0; i < len; i++) {
            frame.grid[frame.mapRows - 1][start + i] = scaleText[i];
        }
    }
}

static void drawCenteredMapTextAt(const char *text, int row)
{
    const int len = strlen(text);
    int start = (frame.mapCols - len) / 2;
    if (start < 1) {
        start = 1;
    }
    for (int i = 0; i < len && (start + i) < frame.mapCols - 1; i++) {
        frame.grid[row][start + i] = text[i];
    }
}

static void drawCenteredMapText(const char *text)
{
    drawCenteredMapTextAt(text, frame.mapRows / 2);
}

static void formatCompactNavValue(char *buf, uint8_t symbol, int32_t metres)
{
    const int32_t converted = lrintf(osdGetMetersToSelectedUnit(metres));
    const int32_t absConverted = ABS(converted);
    const char sign = converted < 0 ? '-' : '\0';

    if (absConverted >= 1000 && frame.mapCols <= 13) {
        if (sign) {
            tfp_sprintf(buf, "%c-%d.%dK", symbol, (int)(absConverted / 1000), (int)((absConverted % 1000) / 100));
        } else {
            tfp_sprintf(buf, "%c%d.%dK", symbol, (int)(absConverted / 1000), (int)((absConverted % 1000) / 100));
        }
    } else if (absConverted >= 10000) {
        if (sign) {
            tfp_sprintf(buf, "%c-%dK", symbol, (int)(absConverted / 1000));
        } else {
            tfp_sprintf(buf, "%c%dK", symbol, (int)(absConverted / 1000));
        }
    } else {
        tfp_sprintf(buf, "%c%d%c", symbol, (int)converted, osdGetMetersToSelectedUnitSymbol());
    }
}

static void composeHomeAltLine(char *line, const navHudState_t *state)
{
    char homeBuf[12];
    char altBuf[12];
    formatCompactNavValue(homeBuf, SYM_HOMEFLAG, state->distanceToHomeCm / 100);
    formatCompactNavValue(altBuf, SYM_ALTITUDE, state->altitudeCm / 100);

    memset(line, ' ', frame.mapCols);
    line[frame.mapCols] = '\0';

    const int homeLen = strlen(homeBuf);
    const int altLen = strlen(altBuf);
    const int maxLeft = MIN(homeLen, frame.mapCols);
    for (int i = 0; i < maxLeft; i++) {
        line[i] = homeBuf[i];
    }

    int altStart = frame.mapCols - altLen;
    if (altStart <= homeLen) {
        altStart = homeLen + 1;
    }
    if (altStart < frame.mapCols) {
        for (int i = 0; i < altLen && altStart + i < frame.mapCols; i++) {
            line[altStart + i] = altBuf[i];
        }
    }
}

static void addHomeAltFooter(const navHudState_t *state, uint8_t attr)
{
    composeHomeAltLine(frame.footer[frame.footerCount], state);
    frame.footerAttr[frame.footerCount] = attr;
    frame.footerCount++;
}

static void drawAcquisitionPanel(const navHudState_t *state)
{
    const bool wide = frame.mapCols >= 17;     // STANDARD and FULL interiors
    const int midRow = frame.mapRows / 2;
    char line[NAV_HUD_MAP_MAX_COLS + 1];

    if (!state->gpsValid) {
        const char *base = wide ? "WAIT GPS FIX" : "WAIT GPS";
        const unsigned dots = (navHudAnimPhase >> 2) & 3;
        int pos = tfp_sprintf(line, "%s", base);
        for (unsigned i = 0; i < dots; i++) {
            line[pos++] = '.';
        }
        line[pos] = '\0';
        drawCenteredMapTextAt(line, wide ? midRow - 1 : midRow);
    } else {
        drawCenteredMapTextAt(wide ? "GPS LOCKED" : "GPS OK", wide ? midRow - 2 : 1);
        if (frame.mapRows >= 9) {
            drawCenteredMapTextAt("ARM TO SET HOME", midRow + 1);
        } else {
            drawCenteredMapTextAt("SET HOME", midRow + 1);
        }
    }
}

// converging-bracket banner for position/altitude hold: the brackets tighten
// onto the label over the first second after engaging, then hold snug.
static void composeHoldBanner(char *line, const navHudState_t *state)
{
    const int width = frame.mapCols;
    const char *label;
    if (state->posHoldActive && state->altHoldActive) {
        label = (width >= 19) ? "POS+ALT HOLD" : "HOLDING";
    } else if (state->posHoldActive) {
        label = "POS HOLD";
    } else {
        label = "ALT HOLD";
    }

    const int elapsed = (uint8_t)(navHudAnimPhase - holdEngagePhase);
    int gap = 4 - MIN(elapsed / 3, 3);                  // 4 -> 1 cells inside the brackets
    const int textLen = strlen(label);
    if (textLen + 2 * gap + 2 > width) {
        gap = MAX((width - textLen - 2) / 2, 0);
    }
    const int total = textLen + 2 * gap + 2;
    int start = (width - total) / 2;
    if (start < 0) {
        start = 0;
    }

    memset(line, ' ', width);
    line[width] = '\0';
    line[start] = SYM_AH_RIGHT;                         // bracket pointing at the label
    memcpy(&line[start + 1 + gap], label, MIN(textLen, width - start - 1 - gap));
    if (start + total - 1 < width) {
        line[start + total - 1] = SYM_AH_LEFT;
    }
}

static const char *phaseText(const navHudState_t *state)
{
    switch (state->phase) {
    case NAV_HUD_PHASE_INIT:
        return "INIT";
    case NAV_HUD_PHASE_CLIMB:
        return "CLIMB";
    case NAV_HUD_PHASE_ORIENT:
        return "ORIENT";
    case NAV_HUD_PHASE_TURN:
        return "TURN";
    case NAV_HUD_PHASE_RETURN:
        return "RETURN";
    case NAV_HUD_PHASE_DESCEND:
        return "DESCEND";
    case NAV_HUD_PHASE_LAND:
        return "LAND";
    case NAV_HUD_PHASE_HOLD:
        return "HOLD";
    case NAV_HUD_PHASE_ABORTED:
        return "EMERG";
    case NAV_HUD_PHASE_UNAVAILABLE:
        return "N/A";
    default:
        return "";
    }
}

static const char *failureText(uint8_t failure)
{
#if defined(USE_GPS_RESCUE) && !defined(USE_WING)
    switch ((rescueFailureState_e)failure) {
    case RESCUE_FLYAWAY:
        return "FLYAWAY";
    case RESCUE_GPSLOST:
        return "GPS LOST";
    case RESCUE_LOWSATS:
        return "LOW SATS";
    case RESCUE_STALLED:
        return "STALLED";
    case RESCUE_NO_HOME_POINT:
        return "NO HOME";
    default:
        return "FAIL";
    }
#else
    UNUSED(failure);
    return "FAIL";
#endif
}

static void formatEta(char *buf, const navHudState_t *state)
{
    if (state->etaValid) {
        tfp_sprintf(buf, "%u:%02u", state->etaSeconds / 60, state->etaSeconds % 60);
    } else {
        strcpy(buf, "-:--");
    }
}

static void composeFooters(const navHudState_t *state)
{
    const navHudConfig_t *cfg = navHudConfig();

    frame.footerCount = 0;

    if (navHudWaypointHintActive() && frame.footerCount < NAV_HUD_MAX_FOOTERS) {
        // one-time teachable moment after the very first waypoint drop
        tfp_sprintf(frame.footer[frame.footerCount], "WP1 HOLD=DEL");
        frame.footerAttr[frame.footerCount] = DISPLAYPORT_SEVERITY_INFO;
        frame.footerCount++;
    }

    const uint8_t deletedWp = navHudWaypointDeleteMsg();
    if (deletedWp != 0 && frame.footerCount < NAV_HUD_MAX_FOOTERS) {
        // brief confirmation that a hold-to-delete actually landed
        tfp_sprintf(frame.footer[frame.footerCount], "WP%u DELETED", deletedWp);
        frame.footerAttr[frame.footerCount] = DISPLAYPORT_SEVERITY_WARNING;
        frame.footerCount++;
    }

#ifdef USE_NAV_MISSION
    if (navMissionGetPhase() != NAV_MISSION_IDLE && frame.footerCount < NAV_HUD_MAX_FOOTERS) {
        char *line = frame.footer[frame.footerCount];
        uint8_t attr = DISPLAYPORT_SEVERITY_INFO;
        switch (navMissionGetPhase()) {
        case NAV_MISSION_FLYING: {
            if (navMissionIsDescending()) {
                // parked overhead, descending to the recorded altitude
                const int wpAltM = navHudWaypointAltCm(navMissionActiveIndex()) / 100;
                tfp_sprintf(line, "WP%u/%u %c%d%c", navMissionActiveIndex() + 1, navHudWaypointCount(),
                            SYM_ARROW_SMALL_DOWN,
                            (int)osdGetMetersToSelectedUnit(wpAltM), osdGetMetersToSelectedUnitSymbol());
                break;
            }
            const vector2_t *wp = navHudWaypointAt(navMissionActiveIndex());
            int distM = 0;
            if (wp) {
                const vector2_t d = { .x = wp->x - state->craftPosCm.x, .y = wp->y - state->craftPosCm.y };
                distM = lrintf(vector2Norm(&d) / 100.0f);
            }
            tfp_sprintf(line, "WP%u/%u %d%c", navMissionActiveIndex() + 1, navHudWaypointCount(),
                        (int)osdGetMetersToSelectedUnit(distM), osdGetMetersToSelectedUnitSymbol());
            break;
        }
        case NAV_MISSION_DONE:
            tfp_sprintf(line, "WP END");
            break;
        default:
            tfp_sprintf(line, "WP ABORT");
            attr = DISPLAYPORT_SEVERITY_WARNING;
            break;
        }
        frame.footerAttr[frame.footerCount] = attr;
        frame.footerCount++;
    }
#endif

    if (!state->rescueActive) {
        if (!state->homeValid) {
            return;
        }

        if (state->posHoldActive || state->altHoldActive) {
            composeHoldBanner(frame.footer[frame.footerCount], state);
            frame.footerAttr[frame.footerCount] = DISPLAYPORT_SEVERITY_INFO;
            frame.footerCount++;
        }

        addHomeAltFooter(state, (!state->gpsGlitch && !state->stale) ? DISPLAYPORT_SEVERITY_NORMAL : DISPLAYPORT_SEVERITY_WARNING);
        return;
    }

    // rescue banner
    {
        char *line = frame.footer[frame.footerCount];
        if (!state->rescueHealthy && state->rescueFailure != 0) {
            tfp_sprintf(line, "RTH %s", failureText(state->rescueFailure));
            frame.footerAttr[frame.footerCount] = DISPLAYPORT_SEVERITY_CRITICAL;
        } else {
            tfp_sprintf(line, "RTH %s%s", phaseText(state), (state->trackTrend < 0 && state->phase == NAV_HUD_PHASE_RETURN) ? "!" : "");
            frame.footerAttr[frame.footerCount] = DISPLAYPORT_SEVERITY_INFO;
        }
        frame.footerCount++;
    }

    addHomeAltFooter(state, (!state->gpsGlitch && !state->stale) ? DISPLAYPORT_SEVERITY_NORMAL : DISPLAYPORT_SEVERITY_WARNING);

    // ETA / target speed, shown only when there is room for a third rescue line.
    if (frame.mapCols >= 27 && cfg->showEta) {
        char *line = frame.footer[frame.footerCount];
        const int spdUnit = osdGetSpeedToSelectedUnit(state->groundSpeedCmS);
        char etaBuf[8];
        formatEta(etaBuf, state);
        if (cfg->showTargets) {
            const int targetSpdUnit = osdGetSpeedToSelectedUnit(state->targetSpeedCmS);
            tfp_sprintf(line, "%c%s %c%d/%d%c", SYM_FLY_M, etaBuf, SYM_SPEED, spdUnit, targetSpdUnit, osdGetSpeedToSelectedUnitSymbol());
        } else {
            tfp_sprintf(line, "%c%s %c%d%c", SYM_FLY_M, etaBuf, SYM_SPEED, spdUnit, osdGetSpeedToSelectedUnitSymbol());
        }
        frame.footerAttr[frame.footerCount] = DISPLAYPORT_SEVERITY_NORMAL;
        frame.footerCount++;
    }

    // cross-track / GPS health
    if (frame.mapCols >= 27 && (cfg->showXtrack || cfg->showGpsHealth)) {
        char *line = frame.footer[frame.footerCount];
        int pos = 0;
        if (cfg->showXtrack && state->routeValid) {
            const int xtkUnit = lrintf(osdGetMetersToSelectedUnit(ABS(state->crossTrackCm) / 100));
            pos += tfp_sprintf(line + pos, "X%d%c%c ", xtkUnit, osdGetMetersToSelectedUnitSymbol(), (state->crossTrackCm >= 0) ? 'R' : 'L');
        }
        if (cfg->showGpsHealth) {
            pos += tfp_sprintf(line + pos, "%c%c%2d", SYM_SAT_L, SYM_SAT_R, state->satellites);
        }
        if (pos > 0) {
            frame.footerAttr[frame.footerCount] = DISPLAYPORT_SEVERITY_NORMAL;
            frame.footerCount++;
        }
    }
}

static void prepareFrame(const navHudState_t *state, navHudMode_e mode)
{
    const navHudConfig_t *cfg = navHudConfig();

    mapSizeForMode(mode, &frame.mapCols, &frame.mapRows);
    memset(frame.grid, SYM_BLANK, sizeof(frame.grid));
    memset(trailBarRowPlusOne, 0, sizeof(trailBarRowPlusOne));
    navHudAnimPhase++;

    // hold-banner animation restarts each time a hold mode engages
    const bool holdingNow = (state->posHoldActive || state->altHoldActive) && !state->rescueActive;
    if (holdingNow && !wasHoldingDisplay) {
        holdEngagePhase = navHudAnimPhase;
    }
    wasHoldingDisplay = holdingNow;

    const vector2_t home = { .x = 0.0f, .y = 0.0f };
    const vector2_t *craft = &state->craftPosCm;

    // orientation: heading-up turns the map with the aircraft's nose, so home
    // sits at the top of the map exactly when the craft faces home. Yaw comes
    // from the attitude estimate (gyro-driven, corrected upstream by GPS
    // course and, when fitted and calibrated, the compass).
    frame.rotated = (cfg->orientation == NAV_HUD_ORIENTATION_HEADING_UP)
                    && state->homeValid && state->positionValid;
    // if the compass is fighting the GPS course, rotate by course instead:
    // motion-up beats a convincingly wrong nose-up
    const bool rotateByCourse = state->headingSuspect && state->cogValid;
    frame.mapRotationDeg = frame.rotated ? (rotateByCourse ? state->cogDeg : state->headingDeg) : 0;
    if (frame.rotated) {
        const float rotRad = DEGREES_TO_RADIANS(frame.mapRotationDeg);
        frame.sinRot = sin_approx(rotRad);
        frame.cosRot = cos_approx(rotRad);
    } else {
        frame.sinRot = 0.0f;
        frame.cosRot = 1.0f;
    }

    // view center
    switch (cfg->center) {
    case NAV_HUD_CENTER_HOME:
        frame.centerEastCm = 0.0f;
        frame.centerNorthCm = 0.0f;
        break;
    case NAV_HUD_CENTER_CRAFT:
        frame.centerEastCm = craft->x;
        frame.centerNorthCm = craft->y;
        break;
    case NAV_HUD_CENTER_AUTO:
    default:
        frame.centerEastCm = craft->x * 0.5f;
        frame.centerNorthCm = craft->y * 0.5f;
        break;
    }

    // during landing, zoom onto the home point for touchdown accuracy
    if (state->rescueActive && state->phase == NAV_HUD_PHASE_LAND) {
        frame.centerEastCm = 0.0f;
        frame.centerNorthCm = 0.0f;
    }

    // scale: map width in metres
    const int interiorCols = frame.mapCols - 2;
    const int interiorRows = frame.mapRows - 2;
    uint32_t scaleM;
    if (state->rescueActive && state->phase == NAV_HUD_PHASE_LAND) {
        scaleM = 50;
    } else if (cfg->autoZoom) {
        // fit craft, home and (during rescue) the route start
        float maxX = 100.0f, maxY = 100.0f;
        const vector2_t *fitPoints[3] = { craft, &home, NULL };
        unsigned fitCount = 2;
        if (!state->rescueActive && state->posHoldActive && state->positionValid) {
            // position hold: close-up on the craft and its anchor; home stays
            // cued in the border and edge-pinned on the map
            fitPoints[1] = state->targetPosValid ? &state->targetPosCm : craft;
        } else if (state->routeValid && state->phase != NAV_HUD_PHASE_DESCEND && state->phase != NAV_HUD_PHASE_LAND) {
            // keep the whole planned route in view while flying it, but once the
            // descent starts zoom in on the approach instead of the far engage point
            fitPoints[fitCount++] = &state->rescueStartCm;
        }
        for (unsigned i = 0; i < fitCount; i++) {
            float viewX, viewY;
            worldToView(fitPoints[i], &viewX, &viewY);
            maxX = fmaxf(maxX, fabsf(viewX));
            maxY = fmaxf(maxY, fabsf(viewY));
        }
        // a waypoint session keeps the whole mission in view
        for (unsigned i = 0; i < navHudWaypointCount(); i++) {
            float viewX, viewY;
            worldToView(navHudWaypointAt(i), &viewX, &viewY);
            maxX = fmaxf(maxX, fabsf(viewX));
            maxY = fmaxf(maxY, fabsf(viewY));
        }
        // convert the height requirement into an equivalent width requirement
        const float heightAsWidth = maxY * interiorCols / (NAV_HUD_CELL_ASPECT * interiorRows);
        const uint32_t requiredM = lrintf(2.0f * fmaxf(maxX, heightAsWidth) * NAV_HUD_FIT_MARGIN / 100.0f);
        uint32_t newScale = navHudSelectAutoScaleM(requiredM);
        if (cfg->center == NAV_HUD_CENTER_HOME && newScale < NAV_HUD_HOME_CENTER_MIN_SCALE_M) {
            newScale = NAV_HUD_HOME_CENTER_MIN_SCALE_M;
        }
        if (autoScaleM == 0 || newScale > autoScaleM
            || (newScale < autoScaleM && requiredM < NAV_HUD_ZOOM_IN_FILL * newScale)) {
            autoScaleM = newScale;
        }
        scaleM = autoScaleM;
    } else {
        scaleM = MAX(cfg->fixedScaleM, 20);
    }
    frame.cmPerCol = scaleM * 100.0f / interiorCols;
    frame.cmPerRow = frame.cmPerCol * NAV_HUD_CELL_ASPECT;

    drawBorderAndLabels(state, scaleM);

    if (!state->homeValid) {
        drawAcquisitionPanel(state);
    } else if (!state->gpsValid && state->stale) {
        drawCenteredMapText("NO GPS");
    } else {
        // layers, lowest priority first
        if (cfg->rangeRing && state->homeValid) {
            plotRangeRing(scaleM);
        }

        if (cfg->projectedTrack && state->cogValid && state->positionValid) {
            const float cogRad = DEGREES_TO_RADIANS(state->cogDeg);
            const float projectCm = state->groundSpeedCmS * 10.0f;  // 10 s ahead
            const vector2_t projected = {
                .x = craft->x + sin_approx(cogRad) * projectCm,
                .y = craft->y + cos_approx(cogRad) * projectCm,
            };
            plotWorldLine(craft, &projected, ROUTE_STYLE_GUIDE);
        }

        // guidance: one dashed stroke from the craft to wherever it is being
        // guided - the active mission waypoint when a mission is flying,
        // otherwise straight home (Betaflight flies straight anyway). It sits
        // UNDER the trail so the flown path stays unbroken where they overlap.
#ifdef USE_NAV_MISSION
        const bool missionFlying = navMissionGetPhase() == NAV_MISSION_FLYING
                                   && navMissionActiveIndex() < navHudWaypointCount();
#else
        const bool missionFlying = false;
#endif
        vector2_t guideEnd = home;
#ifdef USE_NAV_MISSION
        if (missionFlying) {
            guideEnd = *navHudWaypointAt(navMissionActiveIndex());
        }
#endif
        const bool guideActive = state->positionValid
            && (missionFlying || (state->rescueActive ? cfg->rescueRoute : cfg->homeLine));
        if (guideActive) {
            plotWorldLine(craft, &guideEnd, ROUTE_STYLE_GUIDE);
            // during rescue, mark where the descent will begin on that line
            const float distCm = vector2Norm(&state->craftPosCm);
            if (state->rescueActive && state->descentDistanceCm > 0 && distCm > state->descentDistanceCm) {
                const float f = state->descentDistanceCm / distCm;
                const vector2_t descentPoint = { .x = craft->x * f, .y = craft->y * f };
                plotMarker(&descentPoint, SYM_ARROW_SMALL_DOWN, false);
            }
        }

        if (cfg->breadcrumbs) {
            plotTrail(state);
        }

        // numbered waypoints dropped with the waypoint switch; the one the
        // mission is currently flying to blinks. In 3D, a compact altitude
        // tag (1:500) draws beside the marker, into blank cells only, so it
        // never overwrites the trail
        for (unsigned i = 0; i < navHudWaypointCount(); i++) {
            if (missionFlying && i == navMissionActiveIndex() && ((navHudAnimPhase >> 2) & 1)) {
                continue;
            }
            plotMarker(navHudWaypointAt(i), '1' + i, false);
            if (cfg->mission3D) {
                int col, row;
                worldToCellRounded(navHudWaypointAt(i), &col, &row);
                char tag[8];
                const int altUnit = (int)osdGetMetersToSelectedUnit(navHudWaypointAltCm(i) / 100);
                tfp_sprintf(tag, ":%d", altUnit);
                const int len = strlen(tag);
                bool fitsRight = true;
                for (int c = 0; c < len; c++) {
                    const int tc = col + 1 + c;
                    if (!cellInInterior(tc, row) || frame.grid[row + 1][tc + 1] != SYM_BLANK) {
                        fitsRight = false;
                        break;
                    }
                }
                if (fitsRight) {
                    for (int c = 0; c < len; c++) {
                        frame.grid[row + 1][col + 2 + c] = tag[c];
                    }
                } else {
                    // no room on the right (waypoints often sit near the fit
                    // edge): draw "45:" to the left of the marker instead
                    tfp_sprintf(tag, "%d:", altUnit);
                    bool fitsLeft = true;
                    for (int c = 0; c < len; c++) {
                        const int tc = col - len + c;
                        if (!cellInInterior(tc, row) || frame.grid[row + 1][tc + 1] != SYM_BLANK) {
                            fitsLeft = false;
                            break;
                        }
                    }
                    if (fitsLeft) {
                        for (int c = 0; c < len; c++) {
                            frame.grid[row + 1][col - len + 1 + c] = tag[c];
                        }
                    }
                }
            }
        }

        if (state->rescueActive && state->targetPosValid) {
            plotMarker(&state->targetPosCm, SYM_STICK_OVERLAY_CENTER, false);
        }

        // position hold: anchor marker with a gentle shimmer where the
        // autopilot is actually holding, so drift away from it is visible
        if (!state->rescueActive && state->posHoldActive && state->targetPosValid && state->positionValid) {
            plotHoldAnchor(&state->targetPosCm);
        }

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
                                osdGetDirectionSymbolFromHeading(navHudWrapDeg360(screenBearing)));
            }
        }

        // craft marker: over-home ring, else an arrow from the best available
        // heading source (fused IMU yaw, then GPS course over ground), else a
        // neutral dot when no instrument can supply a direction
        if (state->positionValid) {
            if (state->distanceToHomeCm < fminf(frame.cmPerCol, 1000.0f)) {
                plotMarker(craft, SYM_OVER_HOME, true);
            } else if (state->headingValid || frame.rotated) {
                // nose-up: the craft arrow is straight up by definition
                const int screenHeading = frame.rotated ? 0
                    : ((state->headingSuspect && state->cogValid) ? state->cogDeg : state->headingDeg);
                plotMarker(craft, osdGetDirectionSymbolFromHeading(screenHeading), true);
            } else if (state->cogValid) {
                // map can only be rotated with a valid heading, so COG plots as-is
                plotMarker(craft, osdGetDirectionSymbolFromHeading(state->cogDeg), true);
            } else {
                plotMarker(craft, SYM_STICK_OVERLAY_CENTER, true);
            }
        }

        if (state->stale) {
            drawCenteredMapText("STALE");
        }
    }

    composeFooters(state);
}

#ifdef UNIT_TEST
void osdNavHudResetRenderStateForTest(void)
{
    framePrepared = false;
    renderRow = 0;
    autoScaleM = 0;
    navHudAnimPhase = 0;
    wasHoldingDisplay = false;
    holdEngagePhase = 0;
    expandRowShift = 0;
    expandColShift = 0;
    lastScaleShownM = 0;
    scaleFlashFrames = 0;
    scaleZoomedOut = false;
}
#endif

void osdElementNavHud(osdElementParms_t *element)
{
    const displayPort_t *displayPort = element->osdDisplayPort;

    if (!framePrepared) {
        navHudUpdate(micros());
        const navHudState_t *state = navHudGetState();
        const navHudMode_e mode = effectiveMode(state);
        if (mode == NAV_HUD_MODE_OFF) {
            element->drawElement = false;
            return;
        }
        prepareFrame(state, mode);

        // anchor the user-placed footprint: when the map grows (rescue
        // auto-expand, extra rescue footers) and the element sits in the
        // lower/right half of the screen, expand up/left so nothing is
        // pushed off-screen
        const int totalRowsNow = frame.mapRows + frame.footerCount;
        const int rowOverflow = element->elemPosY + totalRowsNow - displayPort->rows;
        const int colOverflow = element->elemPosX + frame.mapCols - displayPort->cols;
        expandRowShift = (rowOverflow > 0) ? MIN(rowOverflow, element->elemPosY) : 0;
        expandColShift = (colOverflow > 0) ? MIN(colOverflow, element->elemPosX) : 0;

        framePrepared = true;
        renderRow = 0;
    }

    const uint8_t totalRows = frame.mapRows + frame.footerCount;

    // vertical clip: rows beyond the screen are dropped
    if (element->elemPosY + renderRow - expandRowShift >= displayPort->rows) {
        element->drawElement = false;
        framePrepared = false;
        return;
    }

    element->attr = DISPLAYPORT_SEVERITY_NORMAL;
    if (renderRow < frame.mapRows) {
        memcpy(element->buff, frame.grid[renderRow], frame.mapCols);
        element->buff[frame.mapCols] = '\0';
    } else {
        const uint8_t footerIndex = renderRow - frame.mapRows;
        strcpy(element->buff, frame.footer[footerIndex]);
        element->attr = frame.footerAttr[footerIndex];
    }

    // horizontal clip against the right screen edge
    const int maxLen = displayPort->cols - (element->elemPosX - expandColShift);
    if (maxLen <= 0) {
        element->drawElement = false;
        framePrepared = false;
        return;
    }
    if ((int)strlen(element->buff) > maxLen) {
        element->buff[maxLen] = '\0';
    }

    element->elemOffsetX = (uint8_t)(0 - expandColShift);
    element->elemOffsetY = (uint8_t)(renderRow - expandRowShift);

    renderRow++;
    if (renderRow < totalRows) {
        element->rendered = false;
    } else {
        framePrepared = false;
    }
}

#endif // USE_OSD_NAV_HUD
