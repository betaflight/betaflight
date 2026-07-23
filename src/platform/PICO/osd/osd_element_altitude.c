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

#if ENABLE_FB_OSD && OSD_FB_PICO_ENABLE_PIXEL_MODE && OSD_FB_ELEMENT_ENABLE_ALTITUDE

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/maths.h"
#include "common/printf.h"
#include "drivers/osd_symbols.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/position.h"
#include "sensors/sensors.h"

#include "osd/osd.h"
#include "osd/osd_elements.h"

#include "osd_pico_internal.h"

bool renderAltitudeComplete = true;

typedef struct {
    char altString[6]; // -9999 to 99999 should be enough (ft or m)
    int16_t x;
    int16_t y;
    int16_t val;
    // int32_t max; // for trying out different ladder appearance above alt_max
    bool isBlink;
    char unitSymbol;
} info_altitude_t;

static info_altitude_t infoAltitude;

// TODO hard code zero decimal places for now
void cacheAltitudeInfo(uint8_t x, uint8_t y, uint8_t elemType, bool isBlink)
{
    bool haveBaro = false;
    bool haveGps = false;
#ifdef USE_BARO
    haveBaro = sensors(SENSOR_BARO);
#endif // USE_BARO
#ifdef USE_GPS
    haveGps = sensors(SENSOR_GPS) && STATE(GPS_FIX);
#endif // USE_GPS
    int32_t altCm = getEstimatedAltitudeCm();

#ifdef USE_GPS
    if (elemType >= 2) {
        // Types 2, 3 => ASL
        altCm = getAltitudeAsl();
    }
#endif

    int32_t altMax = osdConfig()->alt_alarm;
    int32_t altInUnits = osdGetMetersToSelectedUnit(altCm) / 100;

    if ((altInUnits >= altMax) ){ /* && ARMING_FLAG(ARMED)) {*/
        // element->attr = DISPLAYPORT_SEVERITY_CRITICAL;
        // -> selects a different font / colour in msp displayports.
    }

    int32_t altDisplay = constrain(altInUnits, -9999, 99999); // max 5 digits

    infoAltitude.unitSymbol = '-'; // NB '-' == SYM_HYPHEN
    if (haveBaro || haveGps) {
        infoAltitude.unitSymbol = osdGetMetersToSelectedUnitSymbol();
        tfp_sprintf(infoAltitude.altString, "%3d", altDisplay);
    } else {
        tfp_sprintf(infoAltitude.altString, "ALT--");
    }

    infoAltitude.x = x * PICO_OSD_CHAR_WIDTH;
    infoAltitude.y = y * PICO_OSD_CHAR_HEIGHT + (PICO_OSD_CHAR_HEIGHT / 2); // centred on centre of text
    infoAltitude.val = altDisplay;
    // infoAltitude.max = altMax;
    infoAltitude.isBlink = isBlink;

    renderAltitudeComplete = false;
}


bool renderAltitudeUntil(uint32_t limit_micros)
{
//    ADJUST_LIMIT_MICROS(8);

// Slightly more compact strings than standard by reducing horizontal distance from one character to the next.
#define CHAR_ADVANCE (PICO_OSD_CHAR_WIDTH - 1)
#define ALTITUDE_STRING_OFFSET (4)
#define ALTITUDE_CHAR_SPACING (14)
#define ALTITUDE_DELTA (8) // each char in the ladder represents this many units of altitude // maybe from config()
#define ALTITUDE_NUM_CHARS (9) // number of chars in the ladder

    static int state;
    static int16_t strX;
    static int16_t strY;
    static int16_t strWidth;
    static int8_t index; // substate used in some cases.
    static int8_t minIndex;
    static int8_t subIndex; // sub-substate used in some cases.
    static int8_t charOffset;
    static int8_t charNum;

    while (!renderAltitudeComplete && cmpTimeUs(limit_micros, micros()) > 0) {
        switch (state) {
        case 0:
            // Prepare main string
            // cooords for top-left of string, bias upwards because glyphs are typically anchored to the top
            strX = infoAltitude.x + ALTITUDE_STRING_OFFSET;
            strY = infoAltitude.y - (PICO_OSD_GLYPH_HEIGHT - 1)/2;
            renderStringInit(infoAltitude.isBlink ? "" : infoAltitude.altString, strX, strY, CHAR_ADVANCE);
            state++;
            break;

        case 1:
            // Main string
            if (renderStringNext()) {
                // Prepare black fill rectangle around main string
                strWidth = strlen(infoAltitude.altString) * CHAR_ADVANCE;
                iterRectInit(strX - 1, strY - 2, strX + strWidth, strY + PICO_OSD_GLYPH_HEIGHT + 1);
                state++;
            }
            break;

        case 2:
            if (iterBlackFillRectNext()) {
                // Prepare box around main string (first line)
                iterLineInit(strX - 1, strY - 3, strX + strWidth, strY - 3);
                state++;
            }

            break;

        case 3:
            // Box around main bearing indicator
            if (iterLineNext()) {
                iterLineInit(strX - 1, strY + PICO_OSD_GLYPH_HEIGHT + 1, strX + strWidth, strY + PICO_OSD_GLYPH_HEIGHT + 1);
                state++;
            }

            break;

        case 4:
            if (iterLineNext()) {
                iterLineInit(strX - 2, strY - 2, strX - 2, strY + PICO_OSD_GLYPH_HEIGHT);
                state++;
            }

            break;

        case 5:
            if (iterLineNext()) {
                iterLineInit(strX + strWidth + 1, strY - 2, strX + strWidth + 1, strY + PICO_OSD_GLYPH_HEIGHT);
                state++;
            }

            break;
        case 6:
            if (iterLineNext()) {
                // Add decoration to box, prepare for altitude ladder.
                plot(strX - 3, infoAltitude.y - 1, 2);
                plot(strX - 3, infoAltitude.y    , 2);
                plot(strX - 3, infoAltitude.y + 1, 2);
                plot(strX - 4, infoAltitude.y    , 2);
                // Calculate the index of char at / below the pointer (for val>0) / above the pointer (for val<0), and the corresponding position offset.
                charNum = infoAltitude.val / ALTITUDE_DELTA;
                charOffset = ((infoAltitude.val * ALTITUDE_CHAR_SPACING) / ALTITUDE_DELTA) % ALTITUDE_CHAR_SPACING;
                minIndex = charNum - ALTITUDE_NUM_CHARS / 2;

                // centering (even up the number of notches above and below the central pointer)
                if (charOffset > ALTITUDE_CHAR_SPACING / 2) {
                    if (infoAltitude.val < 0) {
                        minIndex--;
                    } else {
                        minIndex++;
                    }
                }

                index = minIndex;
                subIndex = 0;
                state++;
            }

            break;

        case 7:
            if (index < minIndex + ALTITUDE_NUM_CHARS) {
                if (index == 0) {
                    // double horizontal line at maker for alt = 0.
                    int32_t charY = infoAltitude.y - (index - charNum) * ALTITUDE_CHAR_SPACING + charOffset;
                    switch (subIndex) {
                    case 0:
                        iterLineInit(infoAltitude.x - PICO_OSD_GLYPH_WIDTH, charY, infoAltitude.x, charY);
                        subIndex++;
                        break;
                    case 1:
                        if (iterLineNext()) {
                            iterLineInit(infoAltitude.x - PICO_OSD_GLYPH_WIDTH, charY+1, infoAltitude.x, charY+1);
                            subIndex++;
                        }
                        break;
                    case 2:
                        if (iterLineNext()) {
                            index++;
                        }
                        break;
                    }
                } else {
                    int32_t charY = strY - (index - charNum) * ALTITUDE_CHAR_SPACING + charOffset;
                    int32_t altAtChar = index * ALTITUDE_DELTA;
                    // char charSelect = altAtChar < 0 ? SYM_ALT_BELOWZERO : altAtChar < infoAltitude.max ? SYM_ALT_LADDER : SYM_ALT_ABOVEMAX;
                    char charSelect = altAtChar < 0 ? SYM_ALT_BELOWZERO : SYM_ALT_LADDER;
                    renderCharAt(charSelect, infoAltitude.x - PICO_OSD_GLYPH_WIDTH, charY);
                    index++;
                }
            } else {
                state++;
            }

            break;
        case 8:
            // final
        case 9:
            renderAltitudeComplete = true;
            state = 0;
            break;
        }
    }

    return renderAltitudeComplete;
}


#endif // #if ENABLE_FB_OSD && OSD_FB_PICO_ENABLE_PIXEL_MODE

