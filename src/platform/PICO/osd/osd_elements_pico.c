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

#if ENABLE_FB_OSD

// #define DEBUG_TESTCARD

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "common/printf.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "fc/rc_controls.h"
#include "flight/imu.h"
#include "osd/osd.h"
#include "rx/rx.h"

// local
#include "osd_pico_internal.h"
#include "font_betaflight.h"
#include "osd_element_ah.h"
#include "osd_element_compassbar.h"

// Current format for font data stored in memory is based on 12x18 glyphs.
#define FONTDATA_CHAR_WIDTH  12
#define FONTDATA_CHAR_HEIGHT 18
#define FONTDATA_BYTES_PER_CHAR ((FONTDATA_CHAR_WIDTH / 4) * FONTDATA_CHAR_HEIGHT)

typedef enum {
    bgItemPendingCache = 0,
    bgItemPendingRender,
    bgItemComplete
} bgItemState_e;

static volatile bgItemState_e bgSidebarsState;
static volatile bgItemState_e bgStickLeftState;
static volatile bgItemState_e bgStickRightState;

void setBackgroundItemsPending(void)
{
    bgSidebarsState = bgItemPendingCache;
    bgStickLeftState = bgItemPendingCache;
    bgStickRightState = bgItemPendingCache;
}

static void renderCharAtAligned(uint8_t ch, int px, int py);
static void renderCharAtAlignedEx(uint8_t ch, int px, int py, int bpc, int rows);

#ifdef OSD_FB_PICO_PIXEL_MODE

static void renderCharAt(uint8_t ch, int px, int py);

typedef struct {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t yMid;
} info_sidebars_t;

static info_sidebars_t infoSidebars;

#define AH_SIDEBAR_WIDTH_POS 7
#define AH_SIDEBAR_HEIGHT_POS 3
static void cacheSidebarsInfo(uint8_t x, uint8_t y)
{
    // Cache the top left corner and right edge in buffer coords
    // given the centre in char coords.
    // Sidebars are static (background), unchanging until reboot (or config change),
    // so only calculate once.

    if (bgSidebarsState == bgItemPendingCache) {
        infoSidebars.x1 = (x - AH_SIDEBAR_WIDTH_POS) * PICO_OSD_CHAR_WIDTH + PICO_OSD_CHAR_WIDTH / 2;
        infoSidebars.y1 = y  * PICO_OSD_CHAR_HEIGHT - AH_SIDEBAR_HEIGHT_POS * 18; // hard code 18 = original char height
        infoSidebars.yMid = y * PICO_OSD_CHAR_HEIGHT + PICO_OSD_CHAR_HEIGHT / 2;
        infoSidebars.x2 = (x + AH_SIDEBAR_WIDTH_POS) * PICO_OSD_CHAR_WIDTH + PICO_OSD_CHAR_WIDTH / 2;
        bgSidebarsState = bgItemPendingRender;
   }
}

typedef struct {
    int16_t xLeft;
    int16_t yTop;
    int16_t xStick;
    int16_t yStick;
} info_stick_t;

static info_stick_t infoStickLeft;
static info_stick_t infoStickRight;
static bool renderStickLeftComplete = true;
static bool renderStickRightComplete = true;

typedef struct radioControls_s {
    uint8_t left_vertical;
    uint8_t left_horizontal;
    uint8_t right_vertical;
    uint8_t right_horizontal;
} radioControls_t;

static const radioControls_t radioModes[4] = {
    { PITCH,    YAW,    THROTTLE,   ROLL }, // Mode 1
    { THROTTLE, YAW,    PITCH,      ROLL }, // Mode 2
    { PITCH,    ROLL,   THROTTLE,   YAW  }, // Mode 3
    { THROTTLE, ROLL,   PITCH,      YAW  }, // Mode 4
};

// Stick overlay size
#ifdef DEBUG_OSD_TEST_SMALLFONT
static const int stickWidth = 60;
static const int stickHeight = 60;
#else
static const int stickWidth = 86;
static const int stickHeight = 86;
#endif

static void cacheStickBackgroundInfo(info_stick_t *infoPtr, uint8_t x, uint8_t y)
{
    infoPtr->xLeft = charWidth * x;
    infoPtr->yTop = charHeight * y;
}

static void cacheStickInfo(info_stick_t *infoPtr, rc_alias_e vert, rc_alias_e horiz)
{
#ifdef DEBUG_OSD_STICKS_TEST
    UNUSED(vert);
    UNUSED(horiz);
    float tr = micros()*(6.283f/1000000.0f / 3);
    infoPtr->xStick = (int16_t)(infoPtr->xLeft + (stickWidth/2 - 3) * (1 + cosf(tr)) + 3);
    infoPtr->yStick = (int16_t)(infoPtr->yTop + (stickHeight/2 - 3) * (1 + sinf(tr)) + 3);
#else

    const float cursorX = constrainf(rcData[horiz], PWM_RANGE_MIN, PWM_RANGE_MAX);
    const float cursorY = constrainf(rcData[vert], PWM_RANGE_MIN, PWM_RANGE_MAX);

    infoPtr->xStick = (int16_t)scaleRangef(cursorX, PWM_RANGE_MIN, PWM_RANGE_MAX, infoPtr->xLeft, infoPtr->xLeft + stickWidth);

    // note y inverted, cf. osd_elements.c
    infoPtr->yStick = (int16_t)scaleRangef(cursorY, PWM_RANGE_MIN, PWM_RANGE_MAX, infoPtr->yTop + stickHeight, infoPtr->yTop);
#endif // DEBUG_OSD_STICKS_TEST
}

static void cacheStickLeftBackgroundInfo(uint8_t x, uint8_t y)
{
    if (bgStickLeftState == bgItemPendingCache) {
        cacheStickBackgroundInfo(&infoStickLeft, x, y);
        bgStickLeftState = bgItemPendingRender;
    }
}

static void cacheStickRightBackgroundInfo(uint8_t x, uint8_t y)
{
    if (bgStickRightState == bgItemPendingCache) {
        cacheStickBackgroundInfo(&infoStickRight, x, y);
        bgStickRightState = bgItemPendingRender;
    }
}

static void cacheStickLeftInfo(void)
{
    rc_alias_e vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].left_vertical;
    rc_alias_e horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].left_horizontal;
    cacheStickInfo(&infoStickLeft, vertical_channel, horizontal_channel);
    renderStickLeftComplete = false;
}

static void cacheStickRightInfo(void)
{
    rc_alias_e vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_vertical;
    rc_alias_e horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_horizontal;
    cacheStickInfo(&infoStickRight, vertical_channel, horizontal_channel);
    renderStickRightComplete = false;
}

typedef struct {
    const char *str;
    uint16_t strLength;
    uint16_t x;
    uint16_t y;
    int16_t charAdvance;
    uint16_t strIndex;
} renderStringData_t;

static renderStringData_t renderStringData;

void renderStringInit(const char *str, uint16_t initX, uint16_t y, int charAdvance)
{
    renderStringData.str = str;
    renderStringData.strLength = strlen(str);
    renderStringData.x = initX;
    renderStringData.y = y;
    renderStringData.charAdvance = charAdvance;
    renderStringData.strIndex = 0;
}

bool renderStringNext(void)
{
    if (renderStringData.strIndex < renderStringData.strLength) {
        renderCharAt(renderStringData.str[renderStringData.strIndex++], renderStringData.x, renderStringData.y);
        renderStringData.x += renderStringData.charAdvance;
    }

    return !(renderStringData.strIndex < renderStringData.strLength);
}

static bool renderSidebarsUntil(uint32_t limit_micros)
{
    static int count = -1;
    static const int maxCount = (2*AH_SIDEBAR_HEIGHT_POS + 1) * 18 + 1; // hard code 18 = original charHeight

    if (bgSidebarsState != bgItemPendingRender) {
        return true; // Nothing to do here.
    }

    int x1 = infoSidebars.x1;
    int x2 = infoSidebars.x2;

    if (count < 0) {
        // Render the central indicators.
        int yMid = infoSidebars.yMid;
        for (int i=1; i<6; ++i) {
            plot(x1 + 16 - i, yMid + i, 2);
            plot(x1 + 16 - i, yMid + i - 1, 1);
            plot(x1 + 16 - i, yMid - i, 2);
            plot(x1 + 16 - i, yMid - i - 1, 1);
            plot(x2 - 16 + i, yMid + i, 2);
            plot(x2 - 16 + i, yMid + i - 1, 1);
            plot(x2 - 16 + i, yMid - i, 2);
            plot(x2 - 16 + i, yMid - i - 1, 1);
        }

        count++;
    }

    int y = count + infoSidebars.y1;
    while (cmpTimeUs(limit_micros, micros()) > 0 && count < maxCount) {
        // bprintf("y = %d, x1=%d, x2=%d, y1 = %d", y,x1,x2, y1);
        // This is borderline for wanting to break down further (not to exceed limit_micros of around 20us by too much)
        if (count % 16 == 0) {
            dhLine(x1-2, y, 5);
            dhLine(x2-2, y, 5);
        } else if (count % 8 == 0) {
            dhLine(x1-5, y, 11);
            dhLine(x2-5, y, 11);
        }

        y++;
        count++;
    }

    if (count == maxCount) {
        count = -1; // Restart would be with central indicators
        bgSidebarsState = bgItemComplete;
        return true; // All done with Sidebars.
    }

    return false;
}

static int renderSticksBackgroundNext(info_stick_t *infoStick, int16_t xMid, int16_t yMid, int subState)
{
    // Draw horizontal, vertical bars, and small lines at the end of those bars.
    switch (subState) {
    case 0:
        iterLineInit(infoStick->xLeft, yMid, infoStick->xLeft + stickWidth, yMid);
        subState++;
        break;

    case 1:
        if (iterLineNext()) {
            iterLineInit(xMid, infoStick->yTop, xMid, infoStick->yTop + stickHeight);
            subState++;
        }

        break;

    case 2:
        if (iterLineNext()) {
            iterLineInit(infoStick->xLeft, yMid - 2, infoStick->xLeft, yMid + 2);
            subState++;
        }

        break;

    case 3:
        if (iterLineNext()) {
            iterLineInit(infoStick->xLeft + stickWidth, yMid - 2, infoStick->xLeft + stickWidth, yMid + 2);
            subState++;
        }

        break;

    case 4:
        if (iterLineNext()) {
            iterLineInit(xMid - 2, infoStick->yTop, xMid + 2, infoStick->yTop);
            subState++;
        }

        break;

    case 5:
        if (iterLineNext()) {
            iterLineInit(xMid - 2, infoStick->yTop + stickHeight, xMid + 2, infoStick->yTop + stickHeight);
            subState++;
        }

        break;

    case 6:
        if (iterLineNext()) {
            subState = -1;
        }

        break;
    }

    return subState;
}

bool renderSticksBackgroundUntil(uint32_t limit_micros)
{
    static int subState = 0;

    int16_t xMid = infoStickLeft.xLeft + stickWidth / 2;
    int16_t yMid = infoStickLeft.yTop + stickHeight / 2;
    while (bgStickLeftState == bgItemPendingRender && cmpTimeUs(limit_micros, micros()) > 0) {
        subState = renderSticksBackgroundNext(&infoStickLeft, xMid, yMid, subState);
        if (subState < 0) {
            subState = 0;
            bgStickLeftState = bgItemComplete; // This won't get retriggered unless config/profile changes.
        }
    }

    xMid = infoStickRight.xLeft + stickWidth / 2;
    yMid = infoStickRight.yTop + stickHeight / 2;
    while (bgStickRightState == bgItemPendingRender && cmpTimeUs(limit_micros, micros()) > 0) {
        subState = renderSticksBackgroundNext(&infoStickRight, xMid, yMid, subState);
        if (subState < 0) {
            subState = 0;
            bgStickRightState = bgItemComplete;
        }
    }

    return (bgStickLeftState != bgItemPendingRender && bgStickRightState != bgItemPendingRender);
}

static void plotBlob(int x, int y)
{
    plot(x-2, y-2, 2);
    plot(x+2, y-2, 2);
    plot(x-2, y+2, 2);
    plot(x+2, y+2, 2);
    plot(x-1, y-1, 2);
    plot(x+1, y-1, 2);
    plot(x,   y,   2);
    plot(x-1, y+1, 2);
    plot(x+1, y+1, 2);
    plot(x,   y-1, 2);
    plot(x-1, y,   2);
    plot(x+1, y,   2);
    plot(x,   y+1, 2);
}

bool renderStickLeftUntil(uint32_t limit_micros)
{
    if (cmpTimeUs(limit_micros, micros()) > 0) {
        plotBlob(infoStickLeft.xStick, infoStickLeft.yStick);
        renderStickLeftComplete = true;
    }

    return renderStickLeftComplete;
}

bool renderStickRightUntil(uint32_t limit_micros)
{
    if (cmpTimeUs(limit_micros, micros()) > 0) {
        plotBlob(infoStickRight.xStick, infoStickRight.yStick);
        renderStickRightComplete = true;
    }

    return renderStickRightComplete;
}

static void renderCharAt(uint8_t ch, int px, int py)
{
    if (ch == 0x20 || ch == 0) {
        return;
    }

    if (px < 0 || py < 0 || px + PICO_OSD_GLYPH_WIDTH > fb_nx || py + PICO_OSD_GLYPH_HEIGHT > fb_ny) {
        return;
    }

    int xBitShift = 2 * (px % 4); // sub-byte (bit) offset
    if (xBitShift == 0) {
        // Byte-aligned.
        renderCharAtAligned(ch, px, py);
    } else {
        const uint8_t *fontp = &fontData[ch * FONTDATA_BYTES_PER_CHAR];
        uint8_t *bufPtr = osdBufferA + py * PICO_OSD_BUF_WIDTH + (px / 4);
        uint8_t mask1 = 0xff << xBitShift;
        uint8_t mask2 = ~mask1;

        for (int j=0; j<PICO_OSD_GLYPH_HEIGHT; ++j) {
#if OSD_BYTES_PER_CHAR == 2
            uint32_t fontBits = (*((uint16_t *)fontp)) << xBitShift;
            fontp += 3;
            bufPtr[0] = (bufPtr[0] & mask2) | fontBits;
            bufPtr[1] = fontBits >> 8;
            bufPtr[2] = (bufPtr[2] & mask1) | fontBits >> 16;
            bufPtr += PICO_OSD_BUF_WIDTH;
#elif OSD_BYTES_PER_CHAR == 3
            uint32_t fontBits = fontp[2];
            fontBits = fontBits << 8 | fontp[1];
            fontBits = fontBits << 8 | fontp[0];
            fontBits <<= xBitShift;
            fontp += 3;
            bufPtr[0] = (bufPtr[0] & mask2) | fontBits;
            bufPtr[1] = fontBits >> 8;
            bufPtr[2] = fontBits >> 16;
            bufPtr[3] = (bufPtr[3] & mask1) | fontBits >> 24;
            bufPtr += PICO_OSD_BUF_WIDTH;
#endif
        }
    }
}

static bool drawBackgroundItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
{
    switch (item) {
    case OSD_HORIZON_SIDEBARS:
        cacheSidebarsInfo(elemPosX, elemPosY);
        return true;

    case OSD_STICK_OVERLAY_LEFT:
        cacheStickLeftBackgroundInfo(elemPosX, elemPosY);
        return true;

    case OSD_STICK_OVERLAY_RIGHT:
        cacheStickRightBackgroundInfo(elemPosX, elemPosY);
        return true;

    default:
        // Not handled here
        return false;
    }
}

#ifdef DEBUG_OSD_TEST_GPS
#include "fc/runtime_config.h"
#include "io/gps.h"
#include "sensors/sensors.h"
static void enforceSensorGPS(void)
{
    sensorsSet(SENSOR_GPS);
#ifdef USE_GPS
    float tr = micros()*(6.283f/1000000.0f / 64);
    gpsSol.llh.lon = cosf(tr) * 1.5f * GPS_DEGREES_DIVIDER;
    gpsSol.llh.lat = sinf(tr) * 1.5f * GPS_DEGREES_DIVIDER;
    gpsSol.llh.altCm = (0.8f + sinf(tr)) * 8000;
#endif
}
#endif

static bool drawForegroundItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
{
#ifdef DEBUG_OSD_TEST_GPS
    ENABLE_STATE(GPS_FIX_EVER | GPS_FIX);
    enforceSensorGPS();
#endif
    switch (item) {
    // Cache information for rendering an osd item later on.
    case OSD_ARTIFICIAL_HORIZON:
//#define testnoahhere
#ifdef testnoahhere
        // Useful for comparing with original char based AH.
        UNUSED(cacheArtificialHorizonInfo);
        return false;
#else
        cacheArtificialHorizonInfo(elemPosX, elemPosY);
        return true;
#endif

    case OSD_STICK_OVERLAY_LEFT:
        cacheStickLeftInfo();
        return true;

    case OSD_STICK_OVERLAY_RIGHT:
        cacheStickRightInfo();
        return true;

    case OSD_COMPASS_BAR:
        cacheCompassBarInfo(elemPosX, elemPosY);
        return true;

    default:
        // Not handled here
        return false;
    }
}
#endif // #ifdef OSD_FB_PICO_PIXEL_MODE

bool logoVisible;
static bool renderLogoComplete = true;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t fontStart;
    uint8_t cols;
    uint8_t rows;
} info_logo_t;

static info_logo_t infoLogo;

void cacheLogoInfo(uint16_t midX, uint16_t midY, uint16_t fontOffset, uint8_t logoCols, uint8_t logoRows)
{
    logoVisible = true;
    infoLogo.x = midX * PICO_OSD_CHAR_WIDTH - logoCols * 6; // subtract half column width times standard char width (12)
    infoLogo.y = midY * PICO_OSD_CHAR_HEIGHT - (logoRows + 1) * 18; // subtract 1+rows times standard char height (18)
    infoLogo.fontStart = fontOffset;
    infoLogo.cols = logoCols;
    infoLogo.rows = logoRows;
    renderLogoComplete = false;
    bprintf("* cacheLogoInfo %d %d %d %d %d", infoLogo.fontStart, infoLogo.x, infoLogo.y, infoLogo.cols, infoLogo.rows);
}

static bool renderLogoUntil(uint32_t limit_micros)
{
    static int16_t fontIndex = -1;
    static int16_t currentX;
    static int16_t currentY;

    if (fontIndex < 0) {
        fontIndex = infoLogo.fontStart;
        currentX = 0;
        currentY = 0;
    }

    while (currentY < infoLogo.rows) {
        // Logo is from standard 12x18 font data (3 bytes per char width).
        int cyRow = infoLogo.y + currentY * 18;

        while (cmpTimeUs(limit_micros, micros()) > 0 && currentX < infoLogo.cols) {
            renderCharAtAlignedEx(fontIndex, infoLogo.x + currentX * 12, cyRow, 3, 18);
            fontIndex++;
            currentX++;
        }

        if (currentX < infoLogo.cols) {
            // timed out
            break;
        }

        // completed a char line
        currentX = 0;
        currentY++;
    }

    if (currentY == infoLogo.rows) {
        // Reached the end, reset.
        fontIndex = -1;
        renderLogoComplete = true;
    }

    return renderLogoComplete;
}

//#undef OSD_FB_PICO_POSTPROCESS
//#define OSD_FB_PICO_POSTPROCESS 6
#if OSD_FB_PICO_POSTPROCESS
static bool postProcessComplete = true;

static bool postProcessUntil(uint32_t limit_micros)
{
    ADJUST_LIMIT_MICROS(10);
    // pp_mode 1 <- UL only left/upper black
    // pp_mode 2 <- UDLR
    // pp_mode 3 <- UD LL RR or approximation
    // pp_mode 6 <- UDLR and diagonals
    // Plot Black points around White points (don't overwrite a White point).

    uint32_t *plotBufferW = (uint32_t *)(plotToBackground ? osdBufferBackground : osdBufferA);
    static int y;
    static uint32_t *pWord;

    DEBUG_COUNTER_INST(c1);

    if (!pWord) {
        pWord = plotBufferW;
        y = 0;
        DEBUG_ZERO(dd3);
    }

    const uint32_t *maxPWord = plotBufferW + fb_words;
    // assert fb_words divisible by 32
    // and by 23

    while (pWord < maxPWord && (cmpTimeUs(limit_micros, micros()) > 0)) {
        uint32_t wordNext = *pWord;
        uint32_t wordThis = 0;
        bool notLastLine = y < fb_ny - 1;
        for (int dosome = PICO_OSD_LINE_WORDS - 1; dosome >= 0; dosome--) {

            uint32_t wordPrev = wordThis;
            wordThis = wordNext;
            wordNext = dosome ? *(pWord + 1) : 0;

            uint32_t whiteThis = wordThis & 0x55555555; // pick out all of the OSD_W (low bits) of each bit pair (OSD_EN, OSD_W).
            uint32_t blackUpdates;

#if OSD_FB_PICO_POSTPROCESS == 1
            blackUpdates = (whiteThis >> 1);
            blackUpdates |= (wordNext & 0x1) << 31;

            if (notLastLine) {
                blackUpdates |= (*(pWord + PICO_OSD_LINE_WORDS) & 0x55555555) << 1;
            }
            UNUSED(wordPrev);
#elif OSD_FB_PICO_POSTPROCESS == 2
            blackUpdates = (whiteThis >> 1) | (whiteThis << 3); // set OSD_EN according to adjacent OSD_W.
            blackUpdates |= (wordPrev & 0x40000000) >> 29;
            blackUpdates |= (wordNext & 0x1) << 31;

            if (y != 0) {
                blackUpdates |= (*(pWord - PICO_OSD_LINE_WORDS) & 0x55555555) << 1;
            }

            if (notLastLine) {
                blackUpdates |= (*(pWord + PICO_OSD_LINE_WORDS) & 0x55555555) << 1;
            }
#elif OSD_FB_PICO_POSTPROCESS == 3
                blackUpdates = (whiteThis >> 1) | (whiteThis << 3) | (whiteThis >> 3) | (whiteThis << 5); // set OSD_EN according to adjacent OSD_W.
                blackUpdates |= ((wordPrev & 0x40000000) >> 29) | ((wordPrev & 0x50000000) >> 27);
                blackUpdates |= ((wordNext & 0x1) << 31) | ((wordNext & 0x5) << 29);

                if (y != 0) {
                    blackUpdates |= (*(pWord - PICO_OSD_LINE_WORDS) & 0x55555555) << 1;
                }

                if (notLastLine) {
                    blackUpdates |= (*(pWord + PICO_OSD_LINE_WORDS) & 0x55555555) << 1;
                }

                break;
#elif OSD_FB_PICO_POSTPROCESS == 6
                blackUpdates = (whiteThis >> 1) | (whiteThis << 3); // set OSD_EN according to adjacent OSD_W.
                blackUpdates |= (wordPrev & 0x40000000) >> 29;
                blackUpdates |= (wordNext & 0x1) << 31;

                // NB misses out on diagonals that arise from previous or following words on lines above/below.
                if (y != 0) {
                    uint32_t wordUp = *(pWord - PICO_OSD_LINE_WORDS) & 0x55555555;
                    blackUpdates |= wordUp << 1 | wordUp << 3 | wordUp >> 1;
                }

                if (notLastLine) {
                    uint32_t wordUp = *(pWord + PICO_OSD_LINE_WORDS) & 0x55555555;
                    blackUpdates |= wordUp << 1 | wordUp << 3 | wordUp >> 1;
                }
#else
                blackUpdates = 0;
                UNUSED(blackUpdates);
                UNUSED(whiteThis);
                UNUSED(wordPrev);
                UNUSED(notLastLine);
                // unknown
#endif

            *pWord++ = wordThis | blackUpdates;
        }

        y++;
    }

    DEBUG_COUNTER_ACC(dd3, c1);

    if (pWord == maxPWord) {
        pWord = 0;
        postProcessComplete = true;
        DEBUG_ZERO(c1);
    }

    return postProcessComplete;
}
#endif

bool osdPioDrawBackgroundItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
{
#ifdef OSD_FB_PICO_PIXEL_MODE
    DEBUG_COUNTER_INST(c1);
    bool ret = drawBackgroundItem(item, elemPosX, elemPosY);
    DEBUG_COUNTER_ACC(drawBGTot, c1);
    return ret;
#else
    UNUSED(item);
    UNUSED(elemPosX);
    UNUSED(elemPosY);
    return false;
#endif
}

bool osdPioDrawForegroundItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
{
#ifdef OSD_FB_PICO_PIXEL_MODE
    DEBUG_COUNTER_INST(c1);
    bool ret = drawForegroundItem(item, elemPosX, elemPosY);
    DEBUG_COUNTER_ACC(drawFGTot,c1);
    return ret;
#else
    UNUSED(item);
    UNUSED(elemPosX);
    UNUSED(elemPosY);
    return false;
#endif
}

static bool renderCharsComplete = true;

#ifdef DEBUG_OSD_TEST_SMALLFONT
static void renderCharAtAlignedEx(uint8_t ch, int px, int py, int bpc, int rows)
{
    // NOTE only supports bpc == 2 or 3 (width in bytes per char)
    // NOTE renderCharAtAligned assumes that the character at px, py willl fit into the frame buffer.
    const uint8_t *fontp = &fontData[ch * FONTDATA_BYTES_PER_CHAR];
    uint8_t *bufPtr = osdBufferA + py * PICO_OSD_BUF_WIDTH + (px / 4);
    if (bpc == 2) {
        for (int j=0; j<rows; ++j) {
            *((uint16_t *)bufPtr) = *((uint16_t *)fontp);
            fontp += 3;
            bufPtr += PICO_OSD_BUF_WIDTH;
        }
    } else {
        for (int j=0; j<rows; ++j) {
            *((uint16_t *)bufPtr) = *((uint16_t *)fontp);
            fontp += 2;
            bufPtr[2] = *fontp++;
            bufPtr += PICO_OSD_BUF_WIDTH;
        }
    }
}

static void renderCharAtAligned(uint8_t ch, int px, int py)
{
    renderCharAtAlignedEx(ch, px, py, 2, PICO_OSD_GLYPH_HEIGHT);
}

#else // DEBUG_OSD_TEST_SMALLFONT

static void renderCharAtAligned(uint8_t ch, int px, int py)
{
    // NOTE renderCharAtAligned assumes that the character at px, py willl fit into the frame buffer.
    const uint8_t *fontp = &fontData[ch * FONTDATA_BYTES_PER_CHAR];
    uint8_t *bufPtr = osdBufferA + py * PICO_OSD_BUF_WIDTH + (px / 4);
    for (int j=0; j<PICO_OSD_GLYPH_HEIGHT; ++j) {
        *((uint16_t *)bufPtr) = *((uint16_t *)fontp); fontp += 2;
        bufPtr[2] = *fontp++;
        bufPtr += PICO_OSD_BUF_WIDTH;
    }
}

static void renderCharAtAlignedEx(uint8_t ch, int px, int py, int bpc, int rows)
{
    // not SMALLFONT, always bpc = 3, rows = 18 (PICO_OSD_GLYPH_HEIGHT)
    UNUSED(bpc);
    UNUSED(rows);
    renderCharAtAligned(ch, px, py);
}

#endif // DEBUG_OSD_TEST_SMALLFONT

bool renderCharsUntil(uint32_t limit_micros)
{
    // Saved state.
    // Position: currentChar (and cached currentY, currentX).
    static int currentChar;
    static int currentY;
    static int currentX;
    ADJUST_LIMIT_MICROS(4);
    // NOTE charsPerLine doesn't match up with pixels or bytes per line,
    // because we have some spare: 368 pixels not 360 for alignment reasons

    // REM if we use hoffs or equivalent, do same in plot and other drawing routines
    //    const int hoffs = 0; //0..2 (using 90 of 92 bytes)

    if (0 == currentChar) {
        currentY = 0;
        currentX = 0;
    }

    while (currentY < charLines) {
        if (osdCharLineInUse[currentY]) {
            int cyRow = currentY * PICO_OSD_CHAR_HEIGHT;
            while (cmpTimeUs(limit_micros, micros()) > 0 && currentX < charsPerLine) {
                uint8_t c = osdCharBuffer[currentChar++];
                // Buffer is always cleared after vsync before we start updating it. Any required characters
                // will have been written to the buffer, so we can ignore empty characters.
                // *** TODO check char 0 and char 32 (spc) are always transparent (max7456 code clears to 0x20)
                if (c!=0x20 && c!=0) {
                    renderCharAtAligned(c, currentX * PICO_OSD_CHAR_WIDTH, cyRow);
                }

                currentX++;
            }

            if (currentX < charsPerLine) {
                // timed out
                break;
            }

            // Completed a char line
            currentX = 0;
            currentY++;
        } else {
            currentY++;
            currentChar += charsPerLine;
        }
    }

    if (currentY == charLines) {
        // Reached the end, reset.
        currentChar = 0;
        renderCharsComplete = true;
    }

    return renderCharsComplete;
}

// Update screen buffer (paint characters etc to buffer), up until a time limit.
// Store state so that we can resume.
// Return false when complete (no more to do).
bool osdPioRenderScreenUntil(uint32_t limit_micros)
{
    static bool firstOfVsync = true;

#ifdef DEBUG_OSD_FB_PICO
    uint32_t cRender = getCycleCounter();
    ++dd7;
    static uint32_t cycFirst;
#endif
    if (firstOfVsync) {
        firstOfVsync = false;
        renderCharsComplete = false;
        if (logoVisible) {
            renderLogoComplete = false;
        }

#if OSD_FB_PICO_POSTPROCESS
        postProcessComplete = false;
#endif
#ifdef DEBUG_OSD_FB_PICO
        cycFirst = getCycleCounter();
        renderStartCycles += cycFirst - startVsyncCycles;
        renderStartCyclesMax = MAX(renderStartCyclesMax, getCycleCounter() - startVsyncCycles);
#endif
    }

#ifdef DEBUG_TESTCARD
    UNUSED(limit_micros);
    void plotTestCard(void);
    plotTestCard();
    transferredSinceVSync = true;
    return false;
#endif

    DEBUG_COUNTER_INST(c1);

#ifdef OSD_FB_PICO_PIXEL_MODE
    // Proceed with rendering background elements if/as required, if not timed out.
    selectBackgroundBuffer();
    bool complete =
        renderSticksBackgroundUntil(limit_micros) &&
        renderSidebarsUntil(limit_micros);
    selectForegroundBuffer();

    // Continue with foreground elements, if not timed out.
    // (Note each item has a static bool flag render..Complete that should be initialised to true.)
    // Render functions are started and completed in strictly sequential order.
    complete = complete &&
        (renderAHComplete || renderAHUntil(limit_micros)) &&
        (renderCharsComplete || renderCharsUntil(limit_micros)) &&
        (renderStickLeftComplete || renderStickLeftUntil(limit_micros)) &&
        (renderCompassBarComplete || renderCompassBarUntil(limit_micros)) &&
        (renderStickRightComplete || renderStickRightUntil(limit_micros));
#else
    // Not in pixel mode, render chars only.
    bool complete = renderCharsComplete || renderCharsUntil(limit_micros);
#endif

    complete = complete && (!logoVisible || renderLogoComplete || renderLogoUntil(limit_micros));

#if OSD_FB_PICO_POSTPROCESS
    // Testing a post-process operation on the foreground buffer
    // Post processing can e.g. surround white pixels with a black border
    complete = complete && (postProcessComplete || postProcessUntil(limit_micros));

#endif

#ifdef DEBUG_OSD_FB_PICO
    uint32_t cd = getCycleCounter() - c1;
    renderTot += cd;
    if (cd > maxcycles) {
        maxcycles = cd;
    }
#endif

    if (complete) {
#ifdef DEBUG_OSD_FB_PICO
        tusr++;

        uint32_t cycNow = getCycleCounter();
        uint32_t rdd = cycNow - startVsyncCycles;
        renderEndCycles += rdd;
        if (rdd > renderEndCyclesMax) {
            dd1 = cycNow - cycFirst;
            renderEndCyclesMax = rdd;
        }
        dd2 += cycNow - cRender;
#endif

        firstOfVsync = true;
        transferredSinceVSync = true;
        return false; // Nothing more to draw.
    }

    DEBUG_COUNTER_ACC(dd2, cRender);
    return true; // More still to draw.
}

#ifdef DEBUG_TESTCARD
void plotTestCard(void)
{
    for (int i=0; i<fb_nx; ++i) {
        plot(i, 0, 2);
        plot(i, fb_ny/2-1, 2);
        plot(i, fb_ny-1, 2);
    }
    for (int i=0; i<fb_ny; ++i) {
        plot(0, i, 2);
        plot(fb_nx/2-1, i, 2);
        plot(fb_nx-1, i, 2);
    }
    int xx2 = fb_nx/2;
    int yy2 = fb_ny/2;
    for (int k=2; k<10; ++k) {
        int q = fb_nx/2/k;
        int r = fb_ny/2/k;
        for (int j=0; j<16; ++j) {
            plot(xx2-q,j,2);
            plot(xx2+q,j,2);
            plot(xx2-q,fb_ny-1-j,2);
            plot(xx2+q,fb_ny-1-j,2);
            plot(j,yy2-r,2);
            plot(j,yy2+r,2);
            plot(fb_nx-1-j,yy2-r,2);
            plot(fb_nx-1-j,yy2+r,2);
        }
        for (int i=xx2-q; i<xx2+q; ++i) {
            plot(i,k-1,2);
            plot(i,fb_ny - k,2);
        }
        for (int i=yy2-r; i<yy2+r; ++i) {
            plot(k-1,i,2);
            plot(fb_nx-k,i,2);
        }
    }
    // white diagonals to corners and centres of sides
    for (int i=0; i<64; ++i) {
        plot(i, i, 2);
        plot(i, fb_ny/2 - 1 -i, 2);
        plot(i, fb_ny/2 + i, 2);
        plot(i, fb_ny - 1 - i, 2);
        plot(fb_nx -1 -i, i, 2);
        plot(fb_nx -1 -i, fb_ny/2 - 1 -i, 2);
        plot(fb_nx -1 -i, fb_ny/2 + i, 2);
        plot(fb_nx -1 -i, fb_ny - 1 - i, 2);
        plot(fb_nx/2 -1 -i, i, 2);
        plot(fb_nx/2  +i, i, 2);
        plot(fb_nx/2 -1 -i, fb_ny -1 -i, 2);
        plot(fb_nx/2  +i, fb_ny -1 -i, 2);
    }
}
#endif

#endif // ENABLE_FB_OSD
