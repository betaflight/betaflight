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

#include "common/maths.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "flight/imu.h"
#include "osd/osd.h"

#include "osd_pico_internal.h"

#define AH_IMPLEMENTATION 1
// #define AH_IMPLEMENTATION 2

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
//        done = oor ? iterDashedDLineNext() :  iterDLineNext();
        done = oor ? iterDashedQLineNext() :  iterQLineNext();
        UNUSED(iterDashedDLineNext);
        UNUSED(iterDLineNext);
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

#else
#error unknown AH_IMPLMENTATION
#endif

#endif // #if ENABLE_FB_OSD && defined OSD_FB_PICO_PIXEL_MODE

