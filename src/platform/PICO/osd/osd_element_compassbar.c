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

#include <stdint.h>
#include <string.h>

#include "common/printf.h"
#include "drivers/time.h"
#include "flight/imu.h"

#include "osd/osd.h"
#include "osd_pico_internal.h"

bool renderCompassBarComplete = true;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t deciDegrees;
    // offset for lower numbers
} info_compassBar_t;

static info_compassBar_t infoCompassBar;

void cacheCompassBarInfo(uint8_t x, uint8_t y)
{
    // cf. memcpy(element->buff, compassBar + osdGetHeadingIntoDiscreteDirections(DECIDEGREES_TO_DEGREES(attitude.values.yaw), 16), 9);
    // was string of 9 chars. Cache the central position in pixel coordinates based on that.
    infoCompassBar.x = (x + 4.5) * PICO_OSD_CHAR_WIDTH;
    infoCompassBar.y = (y + 0.5) * PICO_OSD_CHAR_HEIGHT;
    infoCompassBar.deciDegrees = attitude.values.yaw;
    renderCompassBarComplete = false;
}

// #define COMPASSBAR_SECONDARY_BEARINGS

#ifdef COMPASSBAR_SECONDARY_BEARINGS
#define COMPASSBAR_WIDTH (128)
#define COMPASSBAR_NOTCH_WIDTH (COMPASSBAR_WIDTH / 5)
// notch = major notch. DEG_PER_NOTCH should divide into 90 to catch the compass points N, E, S, W.
#define COMPASSBAR_DEG_PER_NOTCH (10)
#else
#define COMPASSBAR_WIDTH (128)
#define COMPASSBAR_NOTCH_WIDTH (COMPASSBAR_WIDTH / 3)
#define COMPASSBAR_DEG_PER_NOTCH (45)
#endif

static void bearingToString(char *str, int16_t bearing, bool renderNonCompassPoints)
{
    switch (bearing) {
    case 0:
        tfp_sprintf(str, "N");
        break;
    case 90:
        tfp_sprintf(str, "E");
        break;
    case 180:
        tfp_sprintf(str, "S");
        break;
    case 270:
        tfp_sprintf(str, "W");
        break;
#ifndef COMPASSBAR_SECONDARY_BEARINGS
    case 45:
        tfp_sprintf(str, "NE");
        break;
    case 135:
        tfp_sprintf(str, "SE");
        break;
    case 225:
        tfp_sprintf(str, "SW");
        break;
    case 315:
        tfp_sprintf(str, "NW");
        break;
#endif
    default:
        if (renderNonCompassPoints) {
            tfp_sprintf(str, "%03d", bearing);
        } else {
            str[0] = 0;
        }
    }
}

bool renderCompassBarUntil(uint32_t limit_micros)
{
    ADJUST_LIMIT_MICROS(8);

// Slightly more compact strings than standard by reducing horizontal distance from one character to the next.
#define CHAR_ADVANCE (PICO_OSD_CHAR_WIDTH - 1)
#define COMPASSBAR_STRING_OFFSET ((CHAR_ADVANCE + PICO_OSD_CHAR_WIDTH / 2) - 1)

    static char str[4]; // up to 3 digits.
    static int state;
    static int16_t notchBearing; // central notch
    static int16_t notchOffset;
    static int8_t index; // substate used in several cases.

    const int16_t leftx = infoCompassBar.x - COMPASSBAR_WIDTH / 2;
    const int16_t rightx = leftx + COMPASSBAR_WIDTH;
    const int decoVSize = PICO_OSD_GLYPH_HEIGHT*1.2;
    while (!renderCompassBarComplete && cmpTimeUs(limit_micros, micros()) > 0) {
        switch (state) {
        case 0:
            tfp_sprintf(str, "%03d", (infoCompassBar.deciDegrees / 10) % 1000);
            iterLineInit(leftx, infoCompassBar.y, leftx + COMPASSBAR_WIDTH, infoCompassBar.y);
            state++;
            break;

        case 1:
            // Main horizontal line
            if (iterLineNext()) {
                state++;
            }
            break;
        case 2:
            // Decoration at LH side
            for (int i=0; i<(decoVSize + 3); ++i) {
                plot(leftx, infoCompassBar.y - 2 + i, 2);
            }

            plot(leftx - 1, infoCompassBar.y - 3, 2);
            plot(leftx - 2, infoCompassBar.y - 4, 2);
            for (int i=0; i<4; ++i) {
                plot(leftx-1-i, infoCompassBar.y + decoVSize + 1 + i, 2);
            }

            state++;
            break;

        case 3:
            // Decoration at RH side
            for (int i=0; i<(decoVSize + 3); ++i) {
                plot(rightx, infoCompassBar.y - 2 + i, 2);
            }

            plot(rightx + 1, infoCompassBar.y - 3, 2);
            plot(rightx + 2, infoCompassBar.y - 4, 2);
            for (int i=0; i<4; ++i) {
                plot(rightx + 1 + i, infoCompassBar.y + decoVSize + 1 + i, 2);
            }

            // Prepare main bearing string
            renderStringInit(str, infoCompassBar.x - COMPASSBAR_STRING_OFFSET, infoCompassBar.y - (PICO_OSD_CHAR_HEIGHT + 1), CHAR_ADVANCE);
            state++;
            break;

        case 4:
            // Main bearing string
            if (renderStringNext()) {
                state++;
                iterRectInit(infoCompassBar.x - COMPASSBAR_STRING_OFFSET - 1, infoCompassBar.y - PICO_OSD_CHAR_HEIGHT - 3,
                             infoCompassBar.x + COMPASSBAR_STRING_OFFSET + 1, infoCompassBar.y - 4);
            }

            break;

        case 5:
            if (iterBlackFillRectNext()) {
                state++;
                index = 0;
            }

            break;

        case 6:
            // Box around main bearing indicator
            switch (index) {
            case 0:
                iterLineInit(infoCompassBar.x - COMPASSBAR_STRING_OFFSET - 1, infoCompassBar.y - PICO_OSD_CHAR_HEIGHT - 4,
                             infoCompassBar.x + COMPASSBAR_STRING_OFFSET + 1, infoCompassBar.y - PICO_OSD_CHAR_HEIGHT - 4);

                index++;
                break;

            case 1:
                if (iterLineNext()) {
                    index++;
                    iterLineInit(infoCompassBar.x - COMPASSBAR_STRING_OFFSET - 1, infoCompassBar.y - 3,
                                 infoCompassBar.x + COMPASSBAR_STRING_OFFSET + 1, infoCompassBar.y - 3);
                }
                break;

            case 2:
                if (iterLineNext()) {
                    index++;
                    iterLineInit(infoCompassBar.x - COMPASSBAR_STRING_OFFSET - 2, infoCompassBar.y - PICO_OSD_CHAR_HEIGHT - 3,
                                 infoCompassBar.x - COMPASSBAR_STRING_OFFSET - 2, infoCompassBar.y - 4);
                }
                break;

            case 3:
                if (iterLineNext()) {
                    index++;
                    iterLineInit(infoCompassBar.x + COMPASSBAR_STRING_OFFSET + 2, infoCompassBar.y - PICO_OSD_CHAR_HEIGHT - 3,
                                 infoCompassBar.x + COMPASSBAR_STRING_OFFSET + 2, infoCompassBar.y - 4);
                }
                break;

            case 4:
                if (iterLineNext()) {
                    // Prepare bearing and position for most-central notch. Round the bearing to fit with COMPASSBAR_DEG_PER_NOTCH.
                    notchBearing = (infoCompassBar.deciDegrees / (10 * COMPASSBAR_DEG_PER_NOTCH)) * COMPASSBAR_DEG_PER_NOTCH;
                    notchOffset = ((notchBearing - infoCompassBar.deciDegrees / 10) * COMPASSBAR_NOTCH_WIDTH) / COMPASSBAR_DEG_PER_NOTCH;
                    index = 0;
                    state++;
                }
                break;
            }
            break;

        case 7:
        {
            // Major and minor notches. Move out from central notch, first to the right, then to the left, until out of range.
            int notchX = infoCompassBar.x + notchOffset + index * COMPASSBAR_NOTCH_WIDTH;
            if (index >= 0) {
                if (notchX < rightx) {
                    for (int i=1; i<=decoVSize / 2; ++i) {
                        plot(notchX, infoCompassBar.y + i, 2);
                    }

                    notchX += COMPASSBAR_NOTCH_WIDTH / 2;
                    if (notchX < rightx) {
                        for (int i=1; i<= decoVSize / 4; ++i) {
                            plot(notchX, infoCompassBar.y + i, 2);
                        }

                        index++;
                    } else {
                        index = -1;
                    }
                } else {
                    index = -1;
                }
            } else {
                notchX += COMPASSBAR_NOTCH_WIDTH / 2;
                if (notchX > leftx) {
                    for (int i=1; i<=decoVSize / 4; ++i) {
                        plot(notchX, infoCompassBar.y + i, 2);
                    }

                    notchX -= COMPASSBAR_NOTCH_WIDTH / 2;
                    if (notchX > leftx) {
                        for (int i=1; i<= decoVSize / 2; ++i) {
                            plot(notchX, infoCompassBar.y + i, 2);
                        }

                        index--;
                    } else {
                        index = 0;
                        state++;
                    }
                } else {
                    index = 0;
                    state++;
                }
            }
        }

        break;

        case 8:
        {
            // Repeat notch calculations for the larger notches, for bearings / compass directions
            static int16_t notchX;
            static int16_t bearing;
            static int16_t strLength;
            static int16_t sOffset;
            static bool stringStarted;

            if (!stringStarted) {
                notchX = infoCompassBar.x + notchOffset + index * COMPASSBAR_NOTCH_WIDTH;
                bearing = (360 + notchBearing + COMPASSBAR_DEG_PER_NOTCH * index) % 360;
#ifdef COMPASSBAR_SECONDARY_BEARINGS
                bearingToString(str, bearing, true);
#else
                bearingToString(str, bearing, false);
#endif                
                strLength = strlen(str);
                sOffset = (strLength * CHAR_ADVANCE) / 2;
            }

            if (index >= 0) {
                if (notchX < rightx - sOffset - 1) {
                    if (!stringStarted) {
                        renderStringInit(str, notchX - sOffset,  infoCompassBar.y + decoVSize / 2 + 3, CHAR_ADVANCE);
                        stringStarted = true;
                    }

                    if (renderStringNext()) {
                        stringStarted = false;
                        index++;
                    }
                } else {
                    index = -1;
                }
            } else {
                if (notchX > leftx + sOffset) {
                    if (!stringStarted) {
                        renderStringInit(str, notchX - sOffset, infoCompassBar.y + decoVSize / 2 + 3, CHAR_ADVANCE);
                        stringStarted = true;
                    }

                    if (renderStringNext()) {
                        stringStarted = false;
                        index--;
                    }
                } else {
                    index = 0;
                    state++;
                }
            }
        }

        break;

        case 9:
            renderCompassBarComplete = true;
            state = 0;
            break;
        }
    }

    return renderCompassBarComplete;
}


#endif // #if ENABLE_FB_OSD && defined OSD_FB_PICO_PIXEL_MODE

