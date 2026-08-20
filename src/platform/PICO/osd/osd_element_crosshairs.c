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

#if ENABLE_FB_OSD && OSD_FB_PICO_ENABLE_PIXEL_MODE

#include <math.h>
#include <stdint.h>
#include <string.h>

//#include "drivers/system.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"
//#include "flight/imu.h"
#include "osd/osd.h"

#include "osd_pico_internal.h"

// OSD_CROSSHAIRS implementation for small font.

typedef struct {
    int16_t x, y;               // Start position (top left) for 2x2 char block to centre on Crosshairs element
} info_crosshairs_t;

static info_crosshairs_t infoCrosshairs;
bool renderCrosshairsComplete = true;

void cacheCrosshairsInfo(uint8_t x, uint8_t y)
{
    // Original x,y is top left of 3x1 char block centred correctly
    // Adjust for 2x2 (and 12x18 -> 8x8 for small font).
    infoCrosshairs.x = (x + 0.5f) * PICO_OSD_CHAR_WIDTH;
    infoCrosshairs.y = y * PICO_OSD_CHAR_HEIGHT;
    renderCrosshairsComplete = false;
}

bool renderCrosshairsUntil(uint32_t limit_micros)
{
    static int32_t index = 0;
    while (!renderCrosshairsComplete && cmpTimeUs(limit_micros, micros()) > 0) {
        switch (index) {
        case 0:
            renderCharAt(SYM_CROSSHAIRS_TL, infoCrosshairs.x, infoCrosshairs.y);
            index++;
            break;

        case 1:
            renderCharAt(SYM_CROSSHAIRS_TR, infoCrosshairs.x + PICO_OSD_GLYPH_WIDTH, infoCrosshairs.y);
            index++;
            break;

        case 2:
            renderCharAt(SYM_CROSSHAIRS_BL, infoCrosshairs.x, infoCrosshairs.y + PICO_OSD_GLYPH_HEIGHT);
            index++;
            break;

        case 3:
            renderCharAt(SYM_CROSSHAIRS_BR, infoCrosshairs.x + PICO_OSD_GLYPH_WIDTH, infoCrosshairs.y + PICO_OSD_GLYPH_HEIGHT);
            index = 0;
            renderCrosshairsComplete = true;
            break;
        }
    }

    return renderCrosshairsComplete;
}

#endif // #if ENABLE_FB_OSD && OSD_FB_PICO_ENABLE_PIXEL_MODE

