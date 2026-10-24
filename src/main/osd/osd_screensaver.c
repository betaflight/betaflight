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

// OSD screensaver, shown while the craft sits disarmed: after osd_screensaver_timeout seconds without
// stick or switch movement (0 = never), or at once while the SCREENSAVER mode is on. It stops on any
// movement (unless the mode holds it), arming, failsafe or a CMS menu.
//
// An effect draws the screen a row at a time into a line, so no frame buffer is kept, only the
// effect's own state. The screen is written whole every frame, each cell once, a few rows a call to
// keep the OSD task short; the display drivers send only what changed (MAX7456 per character, the
// MSP display port per cell), and the MSP display port keeps it within the DJI goggles' limit.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OSD_SCREENSAVER

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/time.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"

#include "osd/osd.h"
#include "osd/osd_screensaver.h"

#include "rx/rx.h"

#define SCREENSAVER_MAX_COLS        64
#define SCREENSAVER_CHANNELS        8       // sticks and the first switches
#define SCREENSAVER_MOVEMENT_US     25      // smaller changes of a channel are noise
#define SCREENSAVER_ROWS_PER_CALL   5

// DJI O3 / O4 goggles draw their own widgets over the OSD on its 53x20 canvas: the recording timers
// top right and the status bar bottom right, always there. The screensaver leaves those cells blank,
// so it does not cover them, and spends neither link bytes nor the goggles' frame budget there. Their
// notifications come and go, and the screensaver runs under them.
typedef struct screensaverZone_s {
    uint8_t row0, row1, col0, col1;     // inclusive
} screensaverZone_t;

static const screensaverZone_t djiZones[] = {
    { 5, 6, 44, 49 },                   // recording timers of the goggles and the air unit
    { 18, 18, 35, 48 },                 // voltage, link, bitrate, goggles battery
};

typedef struct screensaverEffect_s {
    uint8_t maxFps;                                 // at most this many new frames a second, whatever the OSD rate
    void (*start)(int rows, int cols);             // NULL: nothing to set up
    bool (*frame)(uint32_t ms);                     // advance to `ms` since the start; true: call again (work
                                                    // split over calls of the OSD task), the same `ms`
    void (*row)(int row, char *line, int cols);     // the glyphs of a row, ' ' for blank
} screensaverEffect_t;

static int16_t lastChannel[SCREENSAVER_CHANNELS];
static timeUs_t lastMovementUs;
static timeUs_t startUs;
static uint32_t randomState = 1;
static uint16_t frame;
static uint8_t nextRow;                 // row the next call draws, 0 = a new frame
static bool framing;                    // the effect is working out the frame
static bool advancing;                  // this OSD frame is a new frame of the effect
static uint32_t frameMs;
static timeUs_t nextFrameUs;
// The canvas against the 53x20 HD one the effects are made for: their sizes scale with its height
static float canvasScale = 1.0f;
static float canvasWidthScale = 1.0f;
static float halfCols, halfRows;        // the centre of the canvas
static uint8_t activeEffect;
static bool cycling;                    // CYCLE: the effects in turn
static bool active;

static uint32_t screensaverRandom(void)
{
    // xorshift32
    randomState ^= randomState << 13;
    randomState ^= randomState >> 17;
    randomState ^= randomState << 5;

    return randomState;
}

static float randomBetween(float low, float high)
{
    return low + (high - low) * (screensaverRandom() & 0xFFFF) * (1.0f / 65535.0f);
}

static int floorInt(float x)
{
    const int i = (int)x;

    return i - (x < i);
}

static uint32_t hash4(int a, int b, int c, uint32_t salt)
{
    uint32_t h = (uint32_t)a * 0x9E3779B1u ^ (uint32_t)b * 0x85EBCA77u ^ (uint32_t)c * 0xC2B2AE3Du ^ salt * 0x27D4EB2Fu;

    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;

    return h;
}

// ---- The state of the effect that runs: one at a time, so they share it -------------------------

#define TUNNEL_RINGS        10
#define TERRAIN_RIDGES      6           // the most there are up to TERRAIN_FAR
#define TERRAIN_HIDDEN      255

typedef struct rainColumn_s {
    int8_t head;        // row of the head, above the screen while the column waits
    uint8_t length;
    uint8_t rowCs;      // centiseconds the head takes for a row
    uint8_t streak;     // counts the streaks of the column, for the glyphs
    uint32_t nextMs;    // when the head moves on
} rainColumn_t;

typedef struct tunnelRing_s {
    float x, y;         // centre on the screen, cell widths from the screen centre
    float inner2, outer2, radius2;
} tunnelRing_t;

#define STARS_COUNT         60
#define ARROWS_COUNT        6

typedef struct arrowPath_s {
    float f[4], p[4];                   // frequencies and phases: x, x, y, y
} arrowPath_t;

// In 1/256 of a unit, as the field holds many
#define STARS_ONE           256.0f

typedef struct starState_s {
    int16_t x, y, z;
} starState_t;

typedef struct terrainState_s {
    uint8_t ridges[TERRAIN_RIDGES][SCREENSAVER_MAX_COLS + 1];  // heights at the cell edges, ninths of a row
    uint8_t floor[SCREENSAVER_MAX_COLS + 1];                   // the floating horizon
    float travel, camX;
    int next;                                                   // world index of the next ridge to try
} terrainState_t;

typedef struct cubeState_s {
    float x[8], y[8];                   // corners on the screen, cell widths from its centre, y down
    uint8_t edges[12][2];               // the edges seen
    uint8_t edgeCount;
    uint8_t cornerSeen;                 // bit per corner
} cubeState_t;

static union {
    rainColumn_t rain[SCREENSAVER_MAX_COLS];
    tunnelRing_t tunnelRings[TUNNEL_RINGS];
    terrainState_t terrain;
    cubeState_t cube;
    starState_t stars[STARS_COUNT];
    arrowPath_t arrows[ARROWS_COUNT];
} fx;

// ---- Matrix: digital rain -----------------------------------------------------------------------
//
// Each column holds one streak: a head of dense letters, a body, a fade and a tail of sparse marks,
// falling one row every 0.1 to 0.3 s. The glyph of a cell is a hash of where it is and of the streak,
// so nothing but the streaks is stored; body glyphs change now and then.

// Milliseconds a streak takes for a row, picked per streak
static const uint8_t rainRowCs[] = { 10, 15, 20, 25, 30 };

// Glyphs, densest first; all are in every OSD font
static const char rainHead[] = "MWNBQRDHO8";
static const char rainBody[] = "GKU0A69XSP5EC34";
static const char rainFade[] = "ZYFT7J1I";
static const char rainTail[] = ":-.";

static uint8_t rainRows, rainCols;

static void rainNewStreak(rainColumn_t *c, uint32_t ms, bool first)
{
    const uint32_t r = screensaverRandom();

    c->length = 5 + r % 8;
    c->rowCs = rainRowCs[(r >> 8) % ARRAYLEN(rainRowCs)];
    c->nextMs = ms + (r >> 12) % (c->rowCs * 10);
    c->streak++;
    if (first) {
        c->head = (int)((r >> 16) % (2 * rainRows)) - rainRows;
    } else {
        // Half the columns wait a while before their next streak, which keeps the screen within the DJI
        // goggles' limit with room to spare
        c->head = -1 - (int)((r >> 16) % 6) - (((r >> 24) & 1) ? 8 + (int)((r >> 20) % 23) : 0);
    }
}

static void rainStart(int rows, int cols)
{
    rainRows = rows;
    rainCols = cols;
    for (int i = 0; i < cols; i++) {
        rainNewStreak(&fx.rain[i], 0, true);
    }
}

static bool rainFrame(uint32_t ms)
{
    for (int i = 0; i < rainCols; i++) {
        rainColumn_t *c = &fx.rain[i];

        while ((int32_t)(ms - c->nextMs) >= 0) {
            c->nextMs += c->rowCs * 10;
            if (++c->head - c->length >= rainRows) {
                rainNewStreak(c, ms, false);
                break;
            }
        }
    }

    return false;
}

static char rainGlyph(const rainColumn_t *c, int row, int col)
{
    const int age = c->head - row;

    if (age < 0 || age >= c->length) {
        return ' ';
    }
    if (age == 0) {
        return rainHead[hash4(row, col, c->streak, 1) % (sizeof(rainHead) - 1)];
    }
    if (age == c->length - 1) {
        return rainTail[hash4(row, col, c->streak, 2) % (sizeof(rainTail) - 1)];
    }
    if (age >= c->length - 3) {
        return rainFade[hash4(row, col, c->streak, 3) % (sizeof(rainFade) - 1)];
    }

    // A body glyph changes every 32 frames, each cell at its own phase
    const uint32_t phase = hash4(row, col, c->streak, 4);

    return rainBody[hash4(row, col, c->streak, 5 + ((frame + phase) >> 5)) % (sizeof(rainBody) - 1)];
}

static void rainRow(int row, char *line, int cols)
{
    for (int col = 0; col < cols; col++) {
        line[col] = rainGlyph(&fx.rain[col], row, col);
    }
}

// ---- Tunnel: rings flying at the camera down a bending tube -------------------------------------
//
// The far end of the tube, its exit, wanders over the screen on a smooth path, and the tube bends to
// it: a ring at depth z is a circle whose centre is offset towards the exit by how deep it is to the
// power 1.5, nothing for the nearest ring and all of it at the far end. A cell is tested against the
// rings near to far: on the line of a ring it shows that ring; outside a ring
// it lies beyond the wall where the tube bends away, and stays blank. So far rings show only where the
// nearer ones let them be seen. Squared distances, no square roots.

#define TUNNEL_SPACING      0.9f        // tube radii between two rings
#define TUNNEL_NEAR         0.7f        // the nearest ring, tube radii in front of the camera
#define TUNNEL_FOCAL        18.0f       // cell widths of screen per tube radius at depth 1
#define TUNNEL_MIN_RADIUS   3.5f        // smaller rings, cell widths, are left out: they would merge
#define TUNNEL_SPEED        0.8f        // tube radii a second
#define TUNNEL_LINE         0.5f        // half the width of a ring's line, cell widths
#define TUNNEL_EXIT_X       0.9f        // how far the exit wanders, of half the screen
#define TUNNEL_EXIT_Y       0.8f
#define CELL_ASPECT         1.5f        // a cell is 1.5 times as tall as it is wide

static uint8_t tunnelRingCount;

static bool tunnelFrame(uint32_t ms)
{
    const float t = ms / 1000.0f;
    const float travel = t * TUNNEL_SPEED;
    // The exit, on a Lissajous path over the screen
    const float exitX = TUNNEL_EXIT_X * halfCols * (0.7f * sin_approx(0.41f * t) + 0.3f * sin_approx(0.67f * t + 1.0f));
    const float exitY = TUNNEL_EXIT_Y * halfRows * CELL_ASPECT * (0.7f * sin_approx(0.53f * t + 2.0f) + 0.3f * sin_approx(0.31f * t));
    // The first ring a little in front of the camera, the next ones every TUNNEL_SPACING
    const float phase = travel / TUNNEL_SPACING;
    const float first = (1.0f - (phase - (int)phase)) * TUNNEL_SPACING;
    const float far = TUNNEL_FOCAL / TUNNEL_MIN_RADIUS;                // the farthest ring drawn, at any scale

    tunnelRingCount = 0;
    for (int k = 0; k < TUNNEL_RINGS; k++) {
        const float z = TUNNEL_NEAR + first + k * TUNNEL_SPACING;
        if (z > far) {
            break;
        }
        tunnelRing_t *ring = &fx.tunnelRings[tunnelRingCount++];
        const float depth = (z - TUNNEL_NEAR) / (far - TUNNEL_NEAR);    // 0 near .. 1 at the exit
        const float r = (TUNNEL_FOCAL * canvasScale) / z;
        const float w = TUNNEL_LINE;
        const float bend = depth * sqrtf(depth);
        ring->x = exitX * bend;
        ring->y = exitY * bend;
        ring->radius2 = r * r;
        ring->inner2 = (r - w) > 0 ? (r - w) * (r - w) : 0;
        ring->outer2 = (r + w) * (r + w);
    }

    return false;
}

// Glyphs of the artificial horizon: a short dash at nine heights in the cell, top to bottom
#define GLYPH_AH_TOP        0x80
#define GLYPH_AH_STEPS      9
#define GLYPH_VERTICAL      0x7C        // a thin bar, two thirds of the cell high
#define GLYPH_RISING        0x2F        // slash
#define GLYPH_FALLING       0x5C        // backslash
#define SLOPE_FLAT          0.45f       // rows a column: under it a dash of the horizon
#define SLOPE_STEEP         2.2f        // rows a column: over it a bar, between the two a slash

// A point of ring k is seen when it lies inside every nearer ring
static bool tunnelSeen(int k, float x, float y)
{
    for (int j = 0; j < k; j++) {
        const float dx = x - fx.tunnelRings[j].x, dy = y - fx.tunnelRings[j].y;
        if (dx * dx + dy * dy > fx.tunnelRings[j].radius2) {
            return false;
        }
    }

    return true;
}

// The glyph of ring k in a cell, 0 when the ring does not pass through it: one cell a column where
// the ring is flatter than a cell's diagonal, one a row where it is steeper. Flat parts are drawn at
// their height in the cell with a dash of the artificial horizon, slanting parts with a slash or a
// backslash, steep ones with a bar.
static char tunnelRingGlyph(int k, float x, float yTop)
{
    const tunnelRing_t *ring = &fx.tunnelRings[k];
    const float dx = x - ring->x;
    const float h2 = ring->radius2 - dx * dx;

    if (h2 > 0.0f) {
        const float h = sqrtf(h2);
        if (fabsf(dx) <= h * CELL_ASPECT) {             // flat: under a cell's height a column
            for (int side = -1; side <= 1; side += 2) {
                const float y = ring->y + side * h;
                if (y >= yTop && y < yTop + CELL_ASPECT && tunnelSeen(k, x, y)) {
                    if (fabsf(dx) < h * CELL_ASPECT * SLOPE_FLAT) {
                        return GLYPH_AH_TOP + (int)((y - yTop) * (GLYPH_AH_STEPS / CELL_ASPECT));
                    }
                    // y grows down the screen: the ring rises to the right where dx and dy have one sign
                    return (dx > 0) == (side > 0) ? GLYPH_RISING : GLYPH_FALLING;
                }
            }
            return 0;
        }
    }

    const float y = yTop + CELL_ASPECT / 2;
    const float dy = y - ring->y;
    const float w2 = ring->radius2 - dy * dy;

    if (w2 > 0.0f) {
        const float w = sqrtf(w2);
        for (int side = -1; side <= 1; side += 2) {
            const float xr = ring->x + side * w;
            if (xr >= x - 0.5f && xr < x + 0.5f && tunnelSeen(k, xr, y)) {
                if (w > fabsf(dy) * CELL_ASPECT * SLOPE_STEEP) {
                    return GLYPH_VERTICAL;
                }
                return (side > 0) == (dy > 0) ? GLYPH_RISING : GLYPH_FALLING;
            }
        }
    }

    return 0;
}

static void tunnelRow(int row, char *line, int cols)
{
    const float yTop = (row - halfRows) * CELL_ASPECT;

    for (int col = 0; col < cols; col++) {
        const float x = col + 0.5f - halfCols;
        const float y = yTop + CELL_ASPECT / 2;
        char c = ' ';

        for (int k = 0; k < tunnelRingCount; k++) {
            const char g = tunnelRingGlyph(k, x, yTop);
            if (g) {
                c = g;
                break;
            }
            // Far beyond the wall of this ring: nothing farther shows here
            const float dx = x - fx.tunnelRings[k].x, dy = y - fx.tunnelRings[k].y;
            if (dx * dx + dy * dy > fx.tunnelRings[k].outer2) {
                break;
            }
        }
        line[col] = c;
    }
}

// ---- Terrain: flying down a canyon ----------------------------------------------------------------
//
// A canyon winds through the hills: a flat floor along its axis, walls rising on both sides to ragged
// crests of value noise. The camera flies low over the floor, following the axis a little ahead, so the
// canyon turns ahead of it. The ground is drawn as ridges across the view at even distances, which come
// at the camera as it flies on. Hidden lines go with a floating horizon: ridges
// near to far, a point of a ridge is seen only above all that is drawn in its column already. The
// ridges are kept as their heights on the screen at the cell edges, in ninths of a row, the dashes of
// the artificial horizon: flat runs are dashes at their height, slopes slashes, steep ones bars.

#define TERRAIN_SPACING     0.5f        // world units between two ridges
#define TERRAIN_LOD1        2.0f        // farther, every second ridge
#define TERRAIN_LOD2        4.0f        // farther, every fourth
#define TERRAIN_FAR         5.0f        // and none from here
#define TERRAIN_NEAR        0.5f        // the nearest ridge, world units in front of the camera
#define TERRAIN_FOCAL       10.0f       // cell widths of screen per world unit at depth 1
#define TERRAIN_EYE         1.4f        // the camera above the ground at 0
#define CANYON_FLOOR        2.0f        // half the width of the flat floor
#define CANYON_SLOPE        2.5f        // the walls rise over this
#define CANYON_HEIGHT       3.0f        // up to the crests
#define CANYON_BUMPS        0.82f       // floor noise over this rises into a bump
#define CANYON_BUMP_HEIGHT  1.2f
#define CANYON_LOOK         1.5f        // the camera follows the axis this far ahead
#define TERRAIN_SPEED       0.8f        // world units a second
#define TERRAIN_HORIZON     0.35f       // the horizon, a part of the screen from the top
#define SUBROWS             GLYPH_AH_STEPS

static float valueNoise(float x, float z)
{
    const int xi = floorInt(x), zi = floorInt(z);
    const float xf = xi, zf = zi;
    float u = x - xf, v = z - zf;

    u = u * u * (3.0f - 2.0f * u);
    v = v * v * (3.0f - 2.0f * v);
    const float n00 = (hash4(xi, zi, 0, 9) & 0xFFFF) * (1.0f / 65535.0f);
    const float n10 = (hash4(xi + 1, zi, 0, 9) & 0xFFFF) * (1.0f / 65535.0f);
    const float n01 = (hash4(xi, zi + 1, 0, 9) & 0xFFFF) * (1.0f / 65535.0f);
    const float n11 = (hash4(xi + 1, zi + 1, 0, 9) & 0xFFFF) * (1.0f / 65535.0f);

    return (n00 + (n10 - n00) * u) + ((n01 + (n11 - n01) * u) - (n00 + (n10 - n00) * u)) * v;
}

static float canyonAxis(float z)
{
    return 3.0f * sin_approx(0.12f * z) + 1.5f * sin_approx(0.07f * z + 1.0f);
}

static float terrainHeight(float x, float z)
{
    // 0 on the floor, 1 at the foot of the crests
    float wall = (fabsf(x - canyonAxis(z)) - CANYON_FLOOR) / CANYON_SLOPE;
    wall = constrainf(wall, 0.0f, 1.0f);
    wall = wall * wall * (3.0f - 2.0f * wall);

    // Crests of very different heights
    const float crest = 0.15f + 0.6f * valueNoise(x * 0.3f, z * 0.3f) + 0.25f * valueNoise(x * 0.9f, z * 0.9f);
    // A bump on the floor now and then
    const float bump = MAX(valueNoise(x * 0.8f + 31.0f, z * 0.8f) - CANYON_BUMPS, 0.0f) / (1.0f - CANYON_BUMPS);

    return CANYON_HEIGHT * wall * crest * crest * 1.6f + CANYON_BUMP_HEIGHT * bump * (1.0f - wall);
}

static uint8_t terrainRows, terrainCols;
static uint8_t terrainCount;                                        // ridges worked out of the frame
static uint8_t terrainDrawn;                                        // ridges of the frame drawn

// Heights are kept in ninths of a row in a byte: canvases taller than this draw ridges in its top part
#define TERRAIN_MAX_ROWS    ((TERRAIN_HIDDEN - 1) / SUBROWS)

static void terrainStart(int rows, int cols)
{
    terrainRows = MIN(rows, TERRAIN_MAX_ROWS);
    terrainCols = cols;
    terrainCount = 0;
    terrainDrawn = 0;
}

// The next ridge drawn from world index `next` on: the ridges lie every TERRAIN_SPACING in the world, and
// on the screen they crowd towards the horizon, so the farther ones are drawn every second, then every
// fourth one. Its depth in front of the camera.
static float terrainNextRidge(void)
{
    terrainState_t *t = &fx.terrain;

    for (;; t->next++) {
        const float z = TERRAIN_NEAR + t->next * TERRAIN_SPACING - t->travel;
        const int every = z < TERRAIN_LOD1 ? 1 : z < TERRAIN_LOD2 ? 2 : 4;
        if (t->next % every == 0) {
            return z;
        }
    }
}

// One ridge a call: the hills are a lot of noise to work out. The ridges go out to a fixed depth, so a
// far one shows from when it comes up at the horizon until it passes the camera.
static bool terrainFrame(uint32_t ms)
{
    terrainState_t *t = &fx.terrain;

    if (terrainCount == 0) {                                        // a new frame
        t->travel = ms / 1000.0f * TERRAIN_SPEED;
        t->camX = canyonAxis(t->travel + TERRAIN_NEAR + CANYON_LOOK);
        t->next = (int)(t->travel / TERRAIN_SPACING) + 1;
        memset(t->floor, terrainRows * SUBROWS, sizeof(t->floor));
    }

    const float z = terrainNextRidge();                             // in front of the camera
    const float zw = t->next * TERRAIN_SPACING;                     // and in the world
    const float horizon = terrainRows * TERRAIN_HORIZON;
    uint8_t *ridge = t->ridges[terrainCount++];

    for (int b = 0; b <= terrainCols; b++) {
        const float x = t->camX + (b - terrainCols / 2.0f) * z / (TERRAIN_FOCAL * canvasScale);
        const float y = horizon + (TERRAIN_FOCAL * canvasScale) * (TERRAIN_EYE - terrainHeight(x, zw)) / z / CELL_ASPECT;
        const int sub = lrintf(y * SUBROWS);

        if (sub >= t->floor[b]) {
            ridge[b] = TERRAIN_HIDDEN;                              // behind a nearer ridge, or below the screen
        } else if (sub < 0) {
            ridge[b] = TERRAIN_HIDDEN;                              // above the screen
            t->floor[b] = 0;
        } else {
            ridge[b] = sub;
            t->floor[b] = sub;
        }
    }
    t->next++;

    if (terrainCount < TERRAIN_RIDGES && terrainNextRidge() < TERRAIN_FAR) {
        return true;
    }
    terrainDrawn = terrainCount;
    terrainCount = 0;

    return false;
}

static void terrainRow(int row, char *line, int cols)
{
    for (int col = 0; col < cols; col++) {
        char c = ' ';

        for (int k = 0; k < terrainDrawn; k++) {
            const int y0 = fx.terrain.ridges[k][col], y1 = fx.terrain.ridges[k][col + 1];
            if (y0 == TERRAIN_HIDDEN || y1 == TERRAIN_HIDDEN) {
                continue;
            }
            const int top = MIN(y0, y1), bottom = MAX(y0, y1);
            if (row < top / SUBROWS || row > bottom / SUBROWS) {
                continue;
            }
            const int dy = y1 - y0;                                 // ninths of a row a column, down the screen
            if (top / SUBROWS == bottom / SUBROWS) {
                c = ABS(dy) < SUBROWS * SLOPE_FLAT ? GLYPH_AH_TOP + (y0 + y1) / 2 - row * SUBROWS
                    : dy < 0 ? GLYPH_RISING : GLYPH_FALLING;
            } else {
                // Across rows: one glyph a row, in the column where the ridge crosses the middle of it.
                // A short step over the edge of two rows that crosses no middle goes in the row of its own
                // middle, or the line would have a gap.
                const int middle = row * SUBROWS + SUBROWS / 2;
                const bool crossesMiddle = (top / SUBROWS) * SUBROWS + SUBROWS / 2 >= top
                    || (bottom / SUBROWS) * SUBROWS + SUBROWS / 2 <= bottom;
                if (crossesMiddle ? (middle < top || middle > bottom) : row != (y0 + y1) / 2 / SUBROWS) {
                    continue;
                }
                if (!crossesMiddle && ABS(dy) < SUBROWS * SLOPE_FLAT) {
                    c = GLYPH_AH_TOP + constrain((y0 + y1) / 2 - row * SUBROWS, 0, SUBROWS - 1);
                } else {
                    c = ABS(dy) > SUBROWS * SLOPE_STEEP ? GLYPH_VERTICAL : dy < 0 ? GLYPH_RISING : GLYPH_FALLING;
                }
            }
            break;
        }
        line[col] = c;
    }
}

// ---- Cube: a wireframe cube turning about three axes ----------------------------------------------
//
// A face is seen when its normal, turned, points at the camera: for a cube of half size 1 seen from D
// away, when its z is under -1/D. Only the edges of the faces seen are drawn, so the hidden ones are
// left out; the corners get a mark. An edge goes through a cell when the line crosses it: in the middle
// of the cell's column where the edge is flatter than a cell's diagonal, of its row where steeper.

#define CUBE_DISTANCE       4.5f        // camera to the centre, half sizes
#define CUBE_FOCAL          30.0f       // cell widths of screen per half size at depth 1
#define GLYPH_CORNER        '+'

// Faces, corners in turn; corner i is at x = bit 0, y = bit 1, z = bit 2 (0: -1, 1: +1)
static const uint8_t cubeFaces[6][4] = {
    { 0, 4, 6, 2 }, { 1, 3, 7, 5 }, { 0, 1, 5, 4 }, { 2, 6, 7, 3 }, { 0, 2, 3, 1 }, { 4, 5, 7, 6 },
};
static const int8_t cubeNormals[6][3] = {
    { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 },
};

static bool cubeFrame(uint32_t ms)
{
    cubeState_t *cube = &fx.cube;
    const float t = ms / 1000.0f;
    const float a = 0.53f * t, b = 0.71f * t + 1.0f, c = 0.29f * t + 2.0f;
    const float sa = sin_approx(a), ca = cos_approx(a), sb = sin_approx(b), cb = cos_approx(b), sc = sin_approx(c), cc = cos_approx(c);
    // Rotation about x by a, then y by b, then z by c
    const float m[3][3] = {
        { cb * cc, sa * sb * cc - ca * sc, ca * sb * cc + sa * sc },
        { cb * sc, sa * sb * sc + ca * cc, ca * sb * sc - sa * cc },
        { -sb,     sa * cb,                ca * cb },
    };

    for (int i = 0; i < 8; i++) {
        const float p[3] = { (i & 1) ? 1.0f : -1.0f, (i & 2) ? 1.0f : -1.0f, (i & 4) ? 1.0f : -1.0f };
        const float x = m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2];
        const float y = m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2];
        const float z = m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + CUBE_DISTANCE;
        cube->x[i] = (CUBE_FOCAL * canvasScale) * x / z;
        cube->y[i] = -(CUBE_FOCAL * canvasScale) * y / z;
    }

    cube->edgeCount = 0;
    cube->cornerSeen = 0;
    for (int f = 0; f < 6; f++) {
        const float nz = m[2][0] * cubeNormals[f][0] + m[2][1] * cubeNormals[f][1] + m[2][2] * cubeNormals[f][2];
        if (nz >= -1.0f / CUBE_DISTANCE) {
            continue;                   // turned away
        }
        for (int e = 0; e < 4; e++) {
            const uint8_t p = MIN(cubeFaces[f][e], cubeFaces[f][(e + 1) & 3]);
            const uint8_t q = MAX(cubeFaces[f][e], cubeFaces[f][(e + 1) & 3]);
            bool known = false;
            for (int k = 0; k < cube->edgeCount; k++) {
                known |= cube->edges[k][0] == p && cube->edges[k][1] == q;
            }
            if (!known) {
                cube->edges[cube->edgeCount][0] = p;
                cube->edges[cube->edgeCount][1] = q;
                cube->edgeCount++;
            }
            cube->cornerSeen |= 1 << p | 1 << q;
        }
    }

    return false;
}

// The glyph of a line from (x0, y0) to (x1, y1) in the cell at x (its middle) and yTop (its top), 0
// when the line misses it; cell widths, y down
static char lineGlyph(float x0, float y0, float x1, float y1, float x, float yTop)
{
    const float dx = x1 - x0, dy = y1 - y0;

    if (fabsf(dy) <= fabsf(dx) * CELL_ASPECT) {
        // Flatter than a cell's diagonal: where it crosses the middle of the column
        if (x < MIN(x0, x1) || x >= MAX(x0, x1)) {
            return 0;
        }
        const float y = y0 + (x - x0) * dy / dx;
        if (y < yTop || y >= yTop + CELL_ASPECT) {
            return 0;
        }
        if (fabsf(dy) < fabsf(dx) * CELL_ASPECT * SLOPE_FLAT) {
            return GLYPH_AH_TOP + (int)((y - yTop) * (GLYPH_AH_STEPS / CELL_ASPECT));
        }
        return (dx > 0) != (dy > 0) ? GLYPH_RISING : GLYPH_FALLING;
    }

    // Steeper: where it crosses the middle of the row
    const float y = yTop + CELL_ASPECT / 2;
    if (y < MIN(y0, y1) || y >= MAX(y0, y1)) {
        return 0;
    }
    const float xl = x0 + (y - y0) * dx / dy;
    if (xl < x - 0.5f || xl >= x + 0.5f) {
        return 0;
    }
    if (fabsf(dy) > fabsf(dx) * CELL_ASPECT * SLOPE_STEEP) {
        return GLYPH_VERTICAL;
    }
    return (dx > 0) != (dy > 0) ? GLYPH_RISING : GLYPH_FALLING;
}

static void cubeRow(int row, char *line, int cols)
{
    const cubeState_t *cube = &fx.cube;
    const float yTop = (row - halfRows) * CELL_ASPECT;

    for (int col = 0; col < cols; col++) {
        const float x = col + 0.5f - halfCols;
        char c = ' ';

        for (int i = 0; i < 8 && c == ' '; i++) {
            if ((cube->cornerSeen >> i & 1) && x - 0.5f <= cube->x[i] && cube->x[i] < x + 0.5f
                && yTop <= cube->y[i] && cube->y[i] < yTop + CELL_ASPECT) {
                c = GLYPH_CORNER;
            }
        }
        for (int k = 0; k < cube->edgeCount && c == ' '; k++) {
            const int p = cube->edges[k][0], q = cube->edges[k][1];
            const char g = lineGlyph(cube->x[p], cube->y[p], cube->x[q], cube->y[q], x, yTop);
            if (g) {
                c = g;
            }
        }
        line[col] = c;
    }
}

// ---- Stars: flying through a field of stars ---------------------------------------------------------
//
// Stars fill the view ahead, as wide as the screen at every depth; the camera flies on through them
// and drifts from side to side, so the point they stream out of wanders over the screen. Nearer stars are brighter (. + *), and the nearest
// draw a trail back to where they were a moment ago, longer as they come closer. A star that passes the
// camera or leaves the screen comes back far ahead, somewhere else.

#define STARS_FAR           12.0f
#define STARS_NEAR          0.4f
#define STARS_FOCAL         12.0f       // cell widths of screen per unit at depth 1
#define STARS_SPEED         2.5f        // units a second
#define STARS_TRAIL_DEPTH   2.5f        // nearer stars draw a trail
#define STARS_TRAIL         0.5f        // the trail reaches this much farther back

static uint32_t starsLastMs;

// A star at depth z anywhere on the screen
static void starsPlace(starState_t *star, float z)
{
    star->z = lrintf(z * STARS_ONE);
    star->x = lrintf(randomBetween(-1.0f, 1.0f) * z * halfCols * STARS_ONE / (STARS_FOCAL * canvasScale));
    star->y = lrintf(randomBetween(-1.0f, 1.0f) * z * halfRows * CELL_ASPECT * STARS_ONE / (STARS_FOCAL * canvasScale));
}

static void starsStart(int rows, int cols)
{
    UNUSED(rows);
    UNUSED(cols);
    starsLastMs = 0;
    for (int i = 0; i < STARS_COUNT; i++) {
        starsPlace(&fx.stars[i], randomBetween(STARS_NEAR, STARS_FAR));
    }
}

// A star on the screen, cell widths from its centre, y down, as if it were `back` units farther
static void starsProject(const starState_t *star, float back, float *sx, float *sy)
{
    const float scale = (STARS_FOCAL * canvasScale) / (star->z + back * STARS_ONE);

    *sx = star->x * scale;
    *sy = star->y * scale;
}

static bool starsFrame(uint32_t ms)
{
    const float t = ms / 1000.0f;
    const float dt = (ms - starsLastMs) / 1000.0f;
    // The camera drifts sideways as it flies on: the stars move the other way
    const float vx = 1.2f * sin_approx(0.23f * t), vy = 0.8f * sin_approx(0.17f * t + 1.0f);

    starsLastMs = ms;
    for (int i = 0; i < STARS_COUNT; i++) {
        starState_t *star = &fx.stars[i];
        star->x -= lrintf(vx * dt * STARS_ONE);
        star->y -= lrintf(vy * dt * STARS_ONE);
        star->z -= lrintf(STARS_SPEED * dt * STARS_ONE);

        float sx, sy;
        starsProject(star, 0.0f, &sx, &sy);
        if (star->z < STARS_NEAR * STARS_ONE || fabsf(sx) > halfCols + 1 || fabsf(sy) > halfRows * CELL_ASPECT + 1) {
            starsPlace(star, randomBetween(STARS_FAR * 0.7f, STARS_FAR));
        }
    }

    return false;
}

static void starsRow(int row, char *line, int cols)
{
    const float yTop = (row - halfRows) * CELL_ASPECT;

    memset(line, ' ', cols);
    for (int i = 0; i < STARS_COUNT; i++) {
        const starState_t *star = &fx.stars[i];
        float sx, sy;
        starsProject(star, 0.0f, &sx, &sy);

        // The trail first, so the star itself is drawn over it
        if (star->z < STARS_TRAIL_DEPTH * STARS_ONE) {
            float tx, ty;
            starsProject(star, STARS_TRAIL, &tx, &ty);
            if (MIN(sy, ty) < yTop + CELL_ASPECT && MAX(sy, ty) >= yTop) {
                const int c0 = MAX(floorInt(MIN(sx, tx) + halfCols), 0);
                const int c1 = MIN(floorInt(MAX(sx, tx) + halfCols), cols - 1);
                for (int col = c0; col <= c1; col++) {
                    const char g = lineGlyph(tx, ty, sx, sy, col + 0.5f - halfCols, yTop);
                    if (g && line[col] == ' ') {
                        line[col] = g;
                    }
                }
            }
        }

        if (sy >= yTop && sy < yTop + CELL_ASPECT) {
            const int col = floorInt(sx + halfCols);
            if (col >= 0 && col < cols) {
                line[col] = star->z > STARS_FAR * 0.6f * STARS_ONE ? '.' : star->z > STARS_FAR * 0.3f * STARS_ONE ? '+' : '*';
            }
        }
    }
}

// ---- Arrows: arrows flying along smooth paths ------------------------------------------------------
//
// Each arrow follows its own path, a sum of two sines across and two down with frequencies and phases of
// its own, and points along it: its velocity, worked out from the path, picks one of the 16 arrows of
// the font, 22.5 degrees apart. Behind it a trail of dots where it was a moment ago.

#define ARROWS_TRAIL        8
#define ARROWS_TRAIL_S      0.08f       // seconds between two dots of the trail
#define GLYPH_ARROW_DOWN    0x60        // 16 arrows: down, then turning to the right, up, left

static void arrowsStart(int rows, int cols)
{
    UNUSED(rows);
    UNUSED(cols);
    for (int i = 0; i < ARROWS_COUNT; i++) {
        for (int k = 0; k < 4; k++) {
            fx.arrows[i].f[k] = randomBetween(0.5f, 1.4f);
            fx.arrows[i].p[k] = randomBetween(0.0f, 2 * M_PIf);
        }
    }
}

// Where an arrow is at time t, cell widths from the screen centre, y down; and which way it goes
static void arrowAt(const arrowPath_t *a, float t, float *x, float *y, float *vx, float *vy)
{
    const float w = halfCols * 0.9f, h = halfRows * CELL_ASPECT * 0.85f;

    *x = w * (0.65f * sin_approx(a->f[0] * t + a->p[0]) + 0.35f * sin_approx(a->f[1] * t + a->p[1]));
    *y = h * (0.65f * sin_approx(a->f[2] * t + a->p[2]) + 0.35f * sin_approx(a->f[3] * t + a->p[3]));
    if (vx) {
        *vx = w * (0.65f * a->f[0] * cos_approx(a->f[0] * t + a->p[0]) + 0.35f * a->f[1] * cos_approx(a->f[1] * t + a->p[1]));
        *vy = h * (0.65f * a->f[2] * cos_approx(a->f[2] * t + a->p[2]) + 0.35f * a->f[3] * cos_approx(a->f[3] * t + a->p[3]));
    }
}

static float arrowsT;

static bool arrowsFrame(uint32_t ms)
{
    arrowsT = ms / 1000.0f;

    return false;
}

static void arrowsPut(char *line, int cols, int row, float x, float y, char glyph, bool over)
{
    const int r = floorInt(y / CELL_ASPECT + halfRows);
    const int c = floorInt(x + halfCols);

    if (r == row && c >= 0 && c < cols && (over || line[c] == ' ')) {
        line[c] = glyph;
    }
}

static void arrowsRow(int row, char *line, int cols)
{
    memset(line, ' ', cols);
    // Trails first, the arrows over them
    for (int i = 0; i < ARROWS_COUNT; i++) {
        for (int k = 1; k <= ARROWS_TRAIL; k++) {
            float x, y;
            arrowAt(&fx.arrows[i], arrowsT - k * ARROWS_TRAIL_S, &x, &y, NULL, NULL);
            arrowsPut(line, cols, row, x, y, k <= 2 ? ':' : '.', false);
        }
    }
    for (int i = 0; i < ARROWS_COUNT; i++) {
        float x, y, vx, vy;
        arrowAt(&fx.arrows[i], arrowsT, &x, &y, &vx, &vy);
        // y grows down the screen: down is 0, right 4, up 8, left 12
        const int k = lrintf(atan2_approx(vx, vy) * (8.0f / M_PIf));
        arrowsPut(line, cols, row, x, y, GLYPH_ARROW_DOWN + (k & 15), true);
    }
}

// ---- Waves: smooth curves drifting right to left, after the PlayStation 3 menu -------------------
//
// A few curves, each a sum of two sines along the screen whose phases move on with time, so they drift
// to the left and twist about one another. They are drawn as the canyon's ridges are, from their heights
// at the cell edges in ninths of a row: dashes of the horizon, slashes and bars.

#define WAVES_COUNT         4

static void wavesStart(int rows, int cols)
{
    terrainRows = MIN(rows, TERRAIN_MAX_ROWS);
    terrainCols = cols;
}

static bool wavesFrame(uint32_t ms)
{
    const float t = ms / 1000.0f;
    const float middle = terrainRows * 0.62f;

    for (int k = 0; k < WAVES_COUNT; k++) {
        uint8_t *wave = fx.terrain.ridges[k];
        // Each wave its own lengths, speeds, height and offset; all of them drift to the left
        const float w1 = (0.07f + 0.02f * k) * canvasWidthScale, w2 = (0.16f - 0.025f * k) * canvasWidthScale;
        const float v1 = 0.55f + 0.12f * k, v2 = 0.9f - 0.1f * k;
        const float a1 = (2.6f - 0.3f * k) * canvasScale, a2 = (1.1f + 0.2f * k) * canvasScale;

        for (int b = 0; b <= terrainCols; b++) {
            const float y = middle + (k - 1.5f) * 0.6f * canvasScale
                + a1 * sin_approx(w1 * b + v1 * t + k * 1.7f) + a2 * sin_approx(w2 * b + v2 * t + k * 2.9f);
            const int sub = lrintf(y * SUBROWS);
            wave[b] = (sub < 0 || sub >= terrainRows * SUBROWS) ? TERRAIN_HIDDEN : sub;
        }
    }
    terrainDrawn = WAVES_COUNT;

    return false;
}

// ---- Effects ----------------------------------------------------------------------------------

// The frame rate an effect can have on a DJI air unit's 115200 baud link (about 9 KB/s free for the OSD):
// the tunnel and the canyon change almost every cell of their lines every frame, the others far fewer
static const screensaverEffect_t effects[OSD_SCREENSAVER_EFFECT_COUNT] = {
    [OSD_SCREENSAVER_MATRIX] = { 15, rainStart, rainFrame, rainRow },
    [OSD_SCREENSAVER_TUNNEL] = { 12, NULL, tunnelFrame, tunnelRow },
    [OSD_SCREENSAVER_TERRAIN] = { 12, terrainStart, terrainFrame, terrainRow },
    [OSD_SCREENSAVER_CUBE] = { 20, NULL, cubeFrame, cubeRow },
    [OSD_SCREENSAVER_STARS] = { 20, starsStart, starsFrame, starsRow },
    [OSD_SCREENSAVER_ARROWS] = { 20, arrowsStart, arrowsFrame, arrowsRow },
    [OSD_SCREENSAVER_WAVES] = { 20, wavesStart, wavesFrame, terrainRow },
};

// ---- Screensaver --------------------------------------------------------------------------------

static void effectStart(const displayPort_t *displayPort, uint8_t effect, timeUs_t currentTimeUs)
{
    activeEffect = effect;
    frame = 0;
    nextRow = 0;
    framing = false;
    nextFrameUs = currentTimeUs;
    startUs = currentTimeUs;
    if (effects[effect].start) {
        effects[effect].start(displayPort->rows, MIN(displayPort->cols, SCREENSAVER_MAX_COLS));
    }
}

static void screensaverStart(const displayPort_t *displayPort, timeUs_t currentTimeUs)
{
    const uint8_t effect = osdConfig()->screensaver_effect;

    randomState = micros() | 1;
    canvasScale = (float)displayPort->rows / OSD_HD_ROWS;
    canvasWidthScale = (float)OSD_HD_COLS / MIN(displayPort->cols, SCREENSAVER_MAX_COLS);
    halfCols = MIN(displayPort->cols, SCREENSAVER_MAX_COLS) / 2.0f;
    halfRows = displayPort->rows / 2.0f;
    cycling = effect >= OSD_SCREENSAVER_EFFECT_COUNT;
    effectStart(displayPort, cycling ? OSD_SCREENSAVER_MATRIX : effect, currentTimeUs);
}

bool osdScreensaverActive(displayPort_t *displayPort, timeUs_t currentTimeUs)
{
    const uint16_t timeout = osdConfig()->screensaver_timeout;
    const int channels = MIN(rxRuntimeState.channelCount, SCREENSAVER_CHANNELS);
    bool moved = false;

    for (int i = 0; i < channels; i++) {
        const int value = rcData[i];
        if (ABS(value - lastChannel[i]) > SCREENSAVER_MOVEMENT_US) {
            lastChannel[i] = value;
            moved = true;
        }
    }

    const bool held = IS_RC_MODE_ACTIVE(BOXSCREENSAVER);

    if (ARMING_FLAG(ARMED) || failsafeIsActive() || displayIsGrabbed(displayPort) || (!held && (!timeout || moved))) {
        lastMovementUs = currentTimeUs;
        active = false;
        return false;
    }

    // Once on it stays on until woken: the time since the last movement is only compared before, while
    // it is under 10 minutes, as microsecond time wraps after 71 minutes
    if (!active) {
        if (!held && cmpTimeUs(currentTimeUs, lastMovementUs) < (timeDelta_t)timeout * 1000000) {
            return false;
        }
        active = true;
        screensaverStart(displayPort, currentTimeUs);
    }

    return true;
}

static bool inDjiZone(int row, int col)
{
    for (unsigned i = 0; i < ARRAYLEN(djiZones); i++) {
        if (row >= djiZones[i].row0 && row <= djiZones[i].row1 && col >= djiZones[i].col0 && col <= djiZones[i].col1) {
            return true;
        }
    }

    return false;
}

bool osdScreensaverDraw(displayPort_t *displayPort, bool mspDisplayPort, timeUs_t currentTimeUs)
{
    const int rows = displayPort->rows;
    const int cols = MIN(displayPort->cols, SCREENSAVER_MAX_COLS);
    const bool djiCanvas = mspDisplayPort && cols == OSD_HD_COLS && rows == OSD_HD_ROWS;
    const screensaverEffect_t *effect = &effects[activeEffect];
    char line[SCREENSAVER_MAX_COLS + 1];

    // A frame begins with the effect's own work, a call or more, then a few rows a call. Over the effect's
    // frame rate an OSD frame draws the same rows again: the OSD clears the screen at every frame, and the
    // display drivers send nothing for rows that did not change.
    if (nextRow == 0) {
        if (!framing && cycling && cmpTimeUs(currentTimeUs, startUs) >= (timeDelta_t)osdConfig()->screensaver_cycle * 1000000) {
            // The next effect, from a fresh start of its own
            effectStart(displayPort, (activeEffect + 1) % OSD_SCREENSAVER_EFFECT_COUNT, currentTimeUs);
            effect = &effects[activeEffect];
        }
        if (!framing) {
            const timeDelta_t period = 1000000 / effect->maxFps;
            advancing = cmpTimeUs(currentTimeUs, nextFrameUs) >= 0;
            if (advancing) {
                nextFrameUs = cmpTimeUs(currentTimeUs, nextFrameUs) >= period ? currentTimeUs + period : nextFrameUs + period;
                frame++;
                frameMs = (currentTimeUs - startUs) / 1000;
            }
        }
        if (advancing) {
            framing = effect->frame(frameMs);
            if (framing) {
                return true;
            }
        }
    }

    const int last = MIN(nextRow + SCREENSAVER_ROWS_PER_CALL, rows);

    for (int row = nextRow; row < last; row++) {
        effect->row(row, line, cols);
        if (djiCanvas) {
            for (int col = 0; col < cols; col++) {
                if (inDjiZone(row, col)) {
                    line[col] = ' ';
                }
            }
        }
        line[cols] = '\0';
        displayWrite(displayPort, 0, row, DISPLAYPORT_SEVERITY_NORMAL, line);
    }

    nextRow = last < rows ? last : 0;

    return nextRow != 0;
}

#endif // USE_OSD_SCREENSAVER
