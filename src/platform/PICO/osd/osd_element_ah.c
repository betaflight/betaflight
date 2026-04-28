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

#include "platform.h"

#if ENABLE_FB_OSD && defined OSD_FB_PICO_PIXEL_MODE

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/maths.h"
#include "common/printf.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "flight/imu.h"
#include "osd/osd.h"

#include "osd_pico_internal.h"

// #define AH_IMPLEMENTATION 1
#define AH_IMPLEMENTATION 2

#if AH_IMPLEMENTATION == 1
typedef struct {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
    bool outOfRange;
} info_ah_t;

static info_ah_t infoArtificialHorizon;
bool renderAHComplete = true;

// cf. osd_element.c implementation osdElementArtificialHorizon
#define AH_SYMBOL_COUNT 9
#define AH_WIDTH_CHARS 5
void cacheArtificialHorizonInfo(uint8_t x, uint8_t y)
{
    // Takes about 5us (every 20ms)
    // Adjust to central y value of character-based AH element.
    y += (AH_SYMBOL_COUNT - 1) / 2;

    // Get pitch and roll limits in tenths of degrees
    const int ahSign = osdConfig()->ahInvert ? -1 : 1;
    const int maxPitch = osdConfig()->ahMaxPitch * 10;
    // roll is uncontrained now. // const int maxRoll = osdConfig()->ahMaxRoll * 10;
    // const int rollAngle = constrain(attitude.values.roll * ahSign, -maxRoll, maxRoll);
    const int rollAngle = attitude.values.roll * ahSign;
    int pitchAngleUnconstrained = attitude.values.pitch * ahSign;
    int pitchAngle = constrain(pitchAngleUnconstrained, -maxPitch, maxPitch);
    infoArtificialHorizon.outOfRange = pitchAngle != pitchAngleUnconstrained;

    // Note that pitch is positive for the board / camera pointing down, and y coords increase going down the screen.
    static const int barScale = AH_WIDTH_CHARS * charWidth; // The AH bar should fit nicely between the Sidebars.
    const int displacementScale = (fb_ny - 64) / 2; // going to fit maxPitch to screen (vertically), less a bit for overscan.
    const float d2r = 3.14159265f * 2 / 360 / 10; // Extra scale factor of 10 for 10th of degree -> radian.
    // float trig functions are pretty quick on RP2350
    float cr = cosf(rollAngle * d2r);
    float sr = sinf(rollAngle * d2r);
    int xoff = 0;
    int yoff = 0;
    if (maxPitch > 0) {
        float tp = tanf(pitchAngle * d2r);
        float tscale = tp * displacementScale / tanf(maxPitch * d2r);
        xoff = tscale * sr;
        yoff = tscale * cr;
    }

    int xc = x * PICO_OSD_CHAR_WIDTH + PICO_OSD_CHAR_WIDTH / 2 - xoff;
    int yc = y * PICO_OSD_CHAR_HEIGHT + PICO_OSD_CHAR_HEIGHT / 2 - yoff;
    infoArtificialHorizon.x1 = xc + barScale * cr;
    infoArtificialHorizon.y1 = yc - barScale * sr;
    infoArtificialHorizon.x2 = xc - barScale * cr;
    infoArtificialHorizon.y2 = yc + barScale * sr;

    renderAHComplete = false;
}

bool renderAHUntil(uint32_t limit_micros)
{
    static bool first = true;
    ADJUST_LIMIT_MICROS(10);

    if (first) {
        iterLineInit(infoArtificialHorizon.x1, infoArtificialHorizon.y1, infoArtificialHorizon.x2, infoArtificialHorizon.y2);
        first = false;
    }

    bool done = false;
    bool oor = infoArtificialHorizon.outOfRange;
    while (cmpTimeUs(limit_micros, micros()) > 0 && !done) {
        done = oor ? iterDashedDLineNext() :  iterDLineNext();
        // line with arrow rather than dashed line?
    }

    if (done) {
        // All done. Prepare for next time.
        first = true;
        renderAHComplete = true;
    }

    return renderAHComplete;
}

#elif AH_IMPLEMENTATION == 2

// Pitch ladder artificial horizon.
// Draws horizontal rungs at regular pitch intervals, rotating with roll,
// shifting vertically with pitch. Numbers label each rung.

#define AH_SYMBOL_COUNT 9
#define AH_WIDTH_CHARS 5

// #define AH_LADDER_STEP_DECIDEG 100    // 10 degrees between rungs
#define AH_LADDER_STEP_DECIDEG 50        // 5 degrees between rungs
#define AH_LADDER_NUM_RUNGS 5            // Number of rungs to display (odd recommended)
#define AH_LADDER_HALF_WIDTH 40          // Half-width of rung lines (pixels)
#define AH_LADDER_GAP_HALF 12            // Half-width of center gap
#define AH_LADDER_HORIZON_HALF_WIDTH 55  // Wider horizon line
#define AH_LADDER_TICK_LEN 6             // Tick mark length at rung ends
#define AH_LABEL_PAD 4                   // Offset pitch numbers at the sides
#define AH_LADDER_HORIZON_GAP

typedef struct {
    int16_t cx, cy;             // Center pixel position of AH element
    float cr, sr;               // cos(roll), sin(roll)
    float pixelsPerDeciDeg;     // Pitch-to-pixel scale
    int16_t pitchDeciDeg;       // Current pitch in tenths of degrees (after ahSign)
} info_ah_ladder_t;

static info_ah_ladder_t infoAH;
bool renderAHComplete = true;

void cacheArtificialHorizonInfo(uint8_t x, uint8_t y)
{
    y += (AH_SYMBOL_COUNT - 1) / 2;

    const int ahSign = osdConfig()->ahInvert ? -1 : 1;
    const int maxPitch = osdConfig()->ahMaxPitch * 10;
    infoAH.pitchDeciDeg = attitude.values.pitch * ahSign;
    int rollDeciDeg = attitude.values.roll * ahSign;

    const float d2r = 3.14159265f / 1800.0f; // deciDeg to radians
    infoAH.cr = cosf(rollDeciDeg * d2r);
    infoAH.sr = sinf(rollDeciDeg * d2r);

    infoAH.cx = x * PICO_OSD_CHAR_WIDTH + PICO_OSD_CHAR_WIDTH / 2;
    infoAH.cy = y * PICO_OSD_CHAR_HEIGHT + PICO_OSD_CHAR_HEIGHT / 2;

    const int displacementScale = (fb_ny - 64) / 2;
    if (maxPitch > 0) {
        infoAH.pixelsPerDeciDeg = (float)displacementScale / maxPitch;
    } else {
        infoAH.pixelsPerDeciDeg = 0.5f;
    }

    renderAHComplete = false;
}

bool renderAHUntil(uint32_t limit_micros)
{
    // Per-rung sub-states:
    //  0: init left (or full) line segment
    //  1: draw that line segment
    //  2: init right line segment
    //  3: draw right line segment
    //  4: draw tick marks at line ends
    //  5: init left label
    //  6: render left label
    //  7: init right label
    //  8: render right label

    static bool first = true;
    static int rungIdx;
    static int subState;
    static int numRungs;
    static int16_t rungPitches[2 * (AH_LADDER_NUM_RUNGS / 2) + 1];
    static char labelBuf[5]; // e.g. "-90\0"

    ADJUST_LIMIT_MICROS(10);

    if (first) {
        // Show AH_LADDER_NUM_RUNGS rungs centered on the nearest rung to current pitch.
        const int step = AH_LADDER_STEP_DECIDEG;
        int p = infoAH.pitchDeciDeg;
        // Round to nearest rung (works for both positive and negative pitch)
        int centerRung = ((p + (p >= 0 ? step / 2 : -step / 2)) / step) * step;
        int halfSpan = (AH_LADDER_NUM_RUNGS / 2) * step;

        numRungs = 0;
        for (int dd = centerRung - halfSpan; dd <= centerRung + halfSpan; dd += step) {
            rungPitches[numRungs++] = dd;
        }
        rungIdx = 0;
        subState = 0;
        first = false;
    }

    while (rungIdx < numRungs && cmpTimeUs(limit_micros, micros()) > 0) {
        const int16_t rp = rungPitches[rungIdx];
        const float voff = (infoAH.pitchDeciDeg - rp) * infoAH.pixelsPerDeciDeg;
        const int rcx = infoAH.cx - (int)(voff * infoAH.sr);
        const int rcy = infoAH.cy - (int)(voff * infoAH.cr);
        const bool isHorizon = (rp == 0);
        const int hw = isHorizon ? AH_LADDER_HORIZON_HALF_WIDTH : AH_LADDER_HALF_WIDTH;
        const int ghw = AH_LADDER_GAP_HALF;
        // Below-horizon rungs (positive rp with default ahSign) are dashed.
        // Labels are -rp/10, so positive rp gives negative labels (below horizon).
        const bool isDashed = (rp > 0);

        switch (subState) {
        case 0: {
            // Left line segment (or full line for horizon)
            int x1 = rcx - (int)(hw * infoAH.cr);
            int y1 = rcy + (int)(hw * infoAH.sr);
            int x2, y2;
#ifdef AH_LADDER_HORIZON_GAP
            x2 = rcx - (int)(ghw * infoAH.cr);
            y2 = rcy + (int)(ghw * infoAH.sr);
#else
            if (isHorizon) {
                x2 = rcx + (int)(hw * infoAH.cr);
                y2 = rcy - (int)(hw * infoAH.sr);
            } else {
                x2 = rcx - (int)(ghw * infoAH.cr);
                y2 = rcy + (int)(ghw * infoAH.sr);
            }
#endif
            iterLineInit(x1, y1, x2, y2);
            subState++;
            break;
        }

        case 1:
            if (isDashed ? iterDashedDLineNext() : iterDLineNext()) {
#ifdef AH_LADDER_HORIZON_GAP
                subState++;
#else
                if (isHorizon) {
                    subState = 0;
                    rungIdx++;
                } else {
                    subState++;
                }
#endif
            }
            break;

        case 2: {
            // Right line segment
            int x1 = rcx + (int)(ghw * infoAH.cr);
            int y1 = rcy - (int)(ghw * infoAH.sr);
            int x2 = rcx + (int)(hw * infoAH.cr);
            int y2 = rcy - (int)(hw * infoAH.sr);
            iterLineInit(x1, y1, x2, y2);
            subState++;
            break;
        }

        case 3:
            if (isDashed ? iterDashedDLineNext() : iterDLineNext()) {
#ifdef AH_LADDER_HORIZON_GAP
                if (isHorizon) {
                    subState = 0;
                    rungIdx++;
                } else {
                    subState++;
                }
#else
                subState++;
#endif
            }
            break;

        case 4: {
            // Tick marks at rung ends, pointing toward the horizon.
            // rp < 0 (above horizon): ticks point down (+sr, +cr)
            // rp > 0 (below horizon): ticks point up (-sr, -cr)
            float tdx, tdy;
            if (rp < 0) {
                tdx = infoAH.sr;
                tdy = infoAH.cr;
            } else {
                tdx = -infoAH.sr;
                tdy = -infoAH.cr;
            }

            int lx = rcx - (int)(hw * infoAH.cr);
            int ly = rcy + (int)(hw * infoAH.sr);
            int rx = rcx + (int)(hw * infoAH.cr);
            int ry = rcy - (int)(hw * infoAH.sr);

            for (int t = -1; t <= AH_LADDER_TICK_LEN; t++) {
                plot(lx + (int)(tdx * t), ly + (int)(tdy * t), 2);
                plot(rx + (int)(tdx * t), ry + (int)(tdy * t), 2);
            }

            subState++;
            break;
        }

        case 5: {
            // Init left label
            int deg = -rp / 10;
            tfp_sprintf(labelBuf, "%d", deg);
            int labelLen = strlen(labelBuf);
            int lx = rcx - (int)(hw * infoAH.cr);
            int ly = rcy + (int)(hw * infoAH.sr);
            int strX = lx - labelLen * (PICO_OSD_CHAR_WIDTH - 1) - AH_LABEL_PAD;
            int strY = ly - PICO_OSD_GLYPH_HEIGHT / 2;
            renderStringInit(labelBuf, strX, strY, PICO_OSD_CHAR_WIDTH - 1);
            subState++;
            break;
        }

        case 6:
            if (renderStringNext()) {
                subState++;
            }
            break;

        case 7: {
            // Init right label
            int deg = -rp / 10;
            tfp_sprintf(labelBuf, "%d", deg);
            int rx = rcx + (int)(hw * infoAH.cr);
            int ry = rcy - (int)(hw * infoAH.sr);
            int strX = rx + AH_LABEL_PAD;
            int strY = ry - PICO_OSD_GLYPH_HEIGHT / 2;
            renderStringInit(labelBuf, strX, strY, PICO_OSD_CHAR_WIDTH - 1);
            subState++;
            break;
        }

        case 8:
            if (renderStringNext()) {
                subState = 0;
                rungIdx++;
            }
            break;
        }
    }

    if (rungIdx >= numRungs) {
        first = true;
        renderAHComplete = true;
    }

    return renderAHComplete;
}

#else
#error unknown AH_IMPLMENTATION
#endif

#endif // #if ENABLE_FB_OSD && defined OSD_FB_PICO_PIXEL_MODE

