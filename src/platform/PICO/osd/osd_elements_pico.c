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

#ifdef USE_FB_OSD

// #define DEBUG_TESTCARD

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "drivers/system.h"
#include "drivers/time.h"
#include "fc/rc_controls.h"
#include "flight/imu.h"
#include "osd/osd.h"
#include "rx/rx.h"

// local
#include "osd_pico_internal.h"
#include "font_betaflight.h"

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

typedef struct {
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t yMid;
} info_sidebars_t;

static info_sidebars_t infoSidebars;

// cf. osd_element.c implementation osdBackgroundHorizonSidebars
static const int charWidth = PICO_OSD_CHAR_WIDTH;
static const int charHeight = PICO_OSD_CHAR_HEIGHT;
static const int charHalfWidth = charWidth / 2;
static const int charHalfHeight = charHeight / 2;
#define AH_SIDEBAR_WIDTH_POS 7
#define AH_SIDEBAR_HEIGHT_POS 3
static void cacheSidebarsInfo(uint8_t x, uint8_t y)
{
    // Cache the top left cornder and right edge in buffer coords
    // given the centre in char coords.
    // Sidebars are static (background), unchanging until reboot (or config change),
    // so only calculate once.

    if (bgSidebarsState == bgItemPendingCache) {
        infoSidebars.x1 = (x - AH_SIDEBAR_WIDTH_POS) * charWidth + charHalfWidth;
        infoSidebars.y1 = (y - AH_SIDEBAR_HEIGHT_POS) * charHeight; // not  + charHalfHeight because sub 0.5char*charHeight
        infoSidebars.yMid = y * charHeight + charHalfHeight;
        infoSidebars.x2 = (x + AH_SIDEBAR_WIDTH_POS) * charWidth + charHalfWidth;;
//        infoSidebars.y2 = (y + AH_SIDEBAR_HEIGHT_POS) * charHeight;
        bgSidebarsState = bgItemPendingRender;
   }
}

typedef struct {
    uint16_t x1;
    uint16_t y1;
    uint16_t x2;
    uint16_t y2;
    bool outOfRange;
} info_ah_t;

static info_ah_t infoArtificialHorizon;
static bool cachedAH;

// cf. osd_element.c implementation osdElementArtificialHorizon
#define AH_SYMBOL_COUNT 9
static void cacheArtificialHorizonInfo(uint8_t x, uint8_t y)
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
    static const int barScale = (AH_SIDEBAR_WIDTH_POS - 2) * charWidth; // The AH bar should fit nicely between the Sidebars.
    const int displacementScale = (fb_ny - 64) / 2; // going to fit maxPitch to screen (vertically), less a bit for overscan.
    const float d2r = 3.14159265f * 2 / 360 / 10; // Extra scale factor of 10 for 10th of degree -> radian.
    // float trig functions are pretty quick on RP2350
    float tp = tanf(pitchAngle * d2r);
    float cr = cosf(rollAngle * d2r);
    float sr = sinf(rollAngle * d2r);
    float tscale = tp * displacementScale / tanf(maxPitch * d2r);
    int xc = x * charWidth + charHalfWidth - tscale * sr;
    int yc = y * charHeight + charHalfHeight - tscale * cr;
    infoArtificialHorizon.x1 = xc + barScale * cr;
    infoArtificialHorizon.y1 = yc - barScale * sr;
    infoArtificialHorizon.x2 = xc - barScale * cr;
    infoArtificialHorizon.y2 = yc + barScale * sr;

    cachedAH = true;
}

typedef struct {
    uint16_t xLeft;
    uint16_t yTop;
    uint16_t xStick;
    uint16_t yStick;
} info_stick_t;

static info_stick_t infoStickLeft;
static info_stick_t infoStickRight;
static bool cachedStickLeft;
static bool cachedStickRight;

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
#define OSD_STICK_OVERLAY_WIDTH 7
#define OSD_STICK_OVERLAY_HEIGHT 5

static const int stickWidth = charWidth * OSD_STICK_OVERLAY_WIDTH;
static const int stickHeight = charHeight * OSD_STICK_OVERLAY_HEIGHT;

//#define TEST_STICK_INPUTS
static void cacheStickBackgroundInfo(info_stick_t *infoPtr, uint8_t x, uint8_t y)
{
    infoPtr->xLeft = charWidth * x;
    infoPtr->yTop = charHeight * y;
}

static void cacheStickInfo(info_stick_t *infoPtr, rc_alias_e vert, rc_alias_e horiz)
{
#ifdef TEST_STICK_INPUTS
    UNUSED(vert);
    UNUSED(horiz);
    float tr = micros()*(6.283f/1000000.0f / 3);
    infoPtr->xStick = (uint16_t)(infoPtr->xLeft + stickWidth/2 * (1 + cosf(tr)));
    infoPtr->yStick = (uint16_t)(infoPtr->yTop + stickHeight/2 * (1 + sinf(tr)));
#else
    
    const float cursorX = constrainf(rcData[horiz], PWM_RANGE_MIN, PWM_RANGE_MAX);
    const float cursorY = constrainf(rcData[vert], PWM_RANGE_MIN, PWM_RANGE_MAX);


    infoPtr->xStick = (uint16_t)scaleRangef(cursorX, PWM_RANGE_MIN, PWM_RANGE_MAX, infoPtr->xLeft, infoPtr->xLeft + stickWidth);

    // note y inverted, cf. osd_elements.c
    infoPtr->yStick = (uint16_t)scaleRangef(cursorY, PWM_RANGE_MIN, PWM_RANGE_MAX, infoPtr->yTop + stickHeight, infoPtr->yTop);
#endif
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
    cachedStickLeft = true;
}

static void cacheStickRightInfo(void)
{
    rc_alias_e vertical_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_vertical;
    rc_alias_e horizontal_channel = radioModes[osdConfig()->overlay_radio_mode-1].right_horizontal;
    cacheStickInfo(&infoStickRight, vertical_channel, horizontal_channel);
    cachedStickRight = true;
}

bool drawBackgroundItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
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

bool osdPioDrawBackgroundItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
{
    DEBUG_COUNTER_INST(c1);
    bool ret = drawBackgroundItem(item, elemPosX, elemPosY);
    DEBUG_COUNTER_ACC(drawBGTot, c1);
    return ret;
}

bool drawForegroundItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
{    
//#define testNoPixelElements
#ifdef testNoPixelElements
    UNUSED(item);
    UNUSED(elemPosX);
    UNUSED(elemPosY);
    UNUSED(cacheArtificialHorizonInfo);
    return false;
#else
    switch (item) {
    // Cache information for rendering an osd item later on.
    case OSD_ARTIFICIAL_HORIZON:
//#define testnoahhere
#ifdef testnoahhere
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

    default:
        // Not handled here
        return false;
    }
#endif
}

bool osdPioDrawForegroundItem(osd_items_e item, uint8_t elemPosX, uint8_t elemPosY)
{
    DEBUG_COUNTER_INST(c1);
    bool ret = drawForegroundItem(item, elemPosX, elemPosY);
    DEBUG_COUNTER_ACC(drawFGTot,c1);
    return ret;
}

static bool renderSidebarsUntil(uint32_t limit_micros)
{
    static int count = -1;
    static const int maxCount = (2*AH_SIDEBAR_HEIGHT_POS + 1) * charHeight + 1;

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
    while (micros() < limit_micros && count < maxCount) {
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

static bool renderAHUntil(uint32_t limit_micros)
{
    static bool first = true;

    if (!cachedAH) {
        return true; // Nothing to do here.
    }

    if (first) {
        iterLineInit(infoArtificialHorizon.x1, infoArtificialHorizon.y1, infoArtificialHorizon.x2, infoArtificialHorizon.y2);
        first = false;
    }

    bool done = false;
    bool oor = infoArtificialHorizon.outOfRange;
    while (micros() < limit_micros && !done) {
//        done = oor ? iterDashedDLineNext() :  iterDLineNext();
        done = oor ? iterDashedQLineNext() :  iterQLineNext();
        UNUSED(iterDashedDLineNext);
        UNUSED(iterDLineNext);
        // line with arrow rather than dashed line?
    }

    if (done) {
        // All done. Prepare for next time.
        first = true;
        cachedAH = false;
        return true; // Done with AH for this round.
    }

    return false;
}

bool renderCharsUntil(uint32_t limit_micros)
{
    // Saved state.
    // Position: currentChar (and cached currentY, currentX, currentPtr).
    static int currentChar;
    static int currentY;
    static int currentX;
    static uint8_t *currentPtr;

    // ** NOTE ** charsPerLine doesn't correspond with pixels or bytes per line,
    // because we have some spare: 368 pixels not 360 for alignment reasons

    // TODO rename these...
    // REM *** if we use hoffs or equivalent, do same in plot and other drawing routines
    const int hoffs = 0; //0..2 (using 90 of 92 bytes)
    const int pxpc = PICO_OSD_CHAR_WIDTH;
    const int bxpc = pxpc / 4; // 4 pixels per byte -> 3 bytes to go across by 1 char
    const int pypc = PICO_OSD_CHAR_HEIGHT;
    const int bpc  = bxpc * pypc;
    const int fbbpl = fb_nx / 4; // bytes per line = pixels per line / pixels per byte
    const int bpCharLine = pypc * fbbpl;
    const int fbbpNextLine = bpCharLine - charsPerLine * bxpc; // byte increment from  (top left of) last char of line to first of next line.

    if (0 == currentChar) {
        currentY = 0;
        currentX = 0;
        currentPtr = osdBufferA + hoffs;
    }

    DEBUG_INC(tus);
    while (currentY < charLines) {
        if (osdCharLineInUse[currentY]) {
            // currentPtr is pointer to topleft of char dest on osdBufferA
            while (cmpTimeUs(limit_micros, micros()) > 0 && currentX < charsPerLine) {
                uint8_t c = osdCharBuffer[currentChar++];
                // Buffer is always cleared after vsync before we start updating it. So, we can
                // ignore empty characters.
                // *** TODO check char 0 and char 32 (spc) are always transparent (max7456 code clears to 0x20)
                if (c!=0x20 && c!=0) {
                    // 1 char = 12 pixels = 3 bytes. 4 chars = 48 pixels = 12 bytes = 3 words
                    const uint8_t * fontp = &fontData[c * bpc]; // 3 bytes per 12 pixel char line, 18 lines
                    uint8_t * bufPtr = currentPtr;
                    for (int j=0; j<pypc; ++j) {
                        // write out loop of bxpc (bytes per char = 3)
                        *bufPtr++ = *fontp++;
                        *bufPtr++ = *fontp++;
                        *bufPtr++ = *fontp++;
                        bufPtr += fbbpl - 3; // new line, back 3 bytes
                    }
                }

                currentPtr += bxpc;
                currentX++;
            }

            if (currentX < charsPerLine) {
                // timed out
                break;
            }

            // Completed a char line, pointer is at top left of last character of line.
            currentX = 0;
            currentY++;
            currentPtr += fbbpNextLine;
        } else {
            currentY++;
            currentChar += charsPerLine;
            currentPtr += bpCharLine;
        }
    }

    if (currentChar == numChars) { // equivalently currentY == charLines
        // Reached the end, reset.
        currentChar = 0;
        return true;
    }

    return false;
}

bool renderSticksBackgroundUntil(uint32_t limit_micros)
{
    static int subState;

    while (bgStickLeftState == bgItemPendingRender && micros() < limit_micros) {
        int xMid = infoStickLeft.xLeft + stickWidth / 2;
        int yMid = infoStickLeft.yTop + stickHeight / 2;
        if (subState == 0) {
            dhLine(infoStickLeft.xLeft, yMid, stickWidth);
            subState++;
        } else {
            dvLine(xMid, infoStickLeft.yTop, stickHeight);
            subState = 0;
            bgStickLeftState = bgItemComplete; // This won't get retriggered unless config/profile changes.
        }
    }

    while (bgStickRightState == bgItemPendingRender && micros() < limit_micros) {
        int xMid = infoStickRight.xLeft + stickWidth / 2;
        int yMid = infoStickRight.yTop + stickHeight / 2;
        if (subState == 0) {
            dhLine(infoStickRight.xLeft, yMid, stickWidth);
            subState++;
        } else {
            dvLine(xMid, infoStickRight.yTop, stickHeight);
            subState = 0;
            bgStickRightState = bgItemComplete;
        }
    }

    return (bgStickLeftState != bgItemPendingRender && bgStickRightState != bgItemPendingRender);
}

static void plotBlob(int x, int y)
{
    plot(x-2, y-2, 2);
    plot(x-1, y-2, 2);
    plot(x, y-2, 2);
    plot(x+1, y-2, 2);
    plot(x+2, y-2, 2);
    plot(x-2, y-1, 2);
    plot(x-2, y, 2);
    plot(x-2, y+1, 2);
    plot(x-2, y+2, 2);
    plot(x-1, y+2, 2);
    plot(x, y+2, 2);
    plot(x+1, y+2, 2);
    plot(x+2, y+2, 2);
    plot(x+2, y+1, 2);
    plot(x+2, y, 2);
    plot(x+2, y-1, 2);

    plot(x-1, y-1, 1);
    plot(x-1, y, 1);
    plot(x-1, y+1, 1);
    plot(x, y+1, 1);
    plot(x+1, y+1, 1);
    plot(x+1, y, 1);
    plot(x+1, y-1, 1);
    plot(x, y-1, 1);
}
    
bool renderSticksForegroundUntil(uint32_t limit_micros)
{
    if (cachedStickLeft && micros() < limit_micros) {
        plotBlob(infoStickLeft.xStick, infoStickLeft.yStick);
        cachedStickLeft = false;
    }

    if (cachedStickRight && micros() < limit_micros) {
        plotBlob(infoStickRight.xStick, infoStickRight.yStick);
        cachedStickRight = false;
    }

    return (!cachedStickLeft && !cachedStickRight);
}

#if 0
// Prototyping - slow version of psotProcessUntil
static bool isWhite(int x, int y)
{
    uint8_t *plotBuffer = plotToBackground ? osdBufferBackground : osdBufferA;
    uint8_t * pByte = plotBuffer + PICO_OSD_BUF_WIDTH * y;
    pByte += (int)(x/4); // 4 pixels per byte
    static uint8_t masks[4] = {0b00000011, 0b00001100, 0b00110000, 0b11000000};
    uint8_t col = *(pByte) & masks[x%4];
    return col & 0b01010101;
}

static bool postProcessUntil0(uint32_t limit_micros)
{
    UNUSED(limit_micros);
//    uint8_t *plotBuffer = plotToBackground ? osdBufferBackground : osdBufferA;
//    for (int y=0; y<fb_ny; ++y) {
// Testing with/without
    for (int y=85; y<200; ++y) {
//        uint8_t * pByte = plotBuffer + PICO_OSD_BUF_WIDTH * y;
        for (int x=0; x<fb_nx; ++x) {
            // if not white but adjacent (say orthongonally) to white, ensure black
            if (!isWhite(x,y)) {
                if (isWhite(x-1,y) || isWhite(x+1,y) || isWhite(x,y-1) || isWhite(x,y+1)) {
                    plot(x,y,1);
                }
            }
        }
    }

    return true;
}
#endif

static bool postProcessUntil(uint32_t limit_micros)
{
    // Plot Black points around every White point (don't overwrite a White point).
    UNUSED(limit_micros); // TODO limit time taken, exit and resume
    uint32_t *plotBufferW = (uint32_t *)(plotToBackground ? osdBufferBackground : osdBufferA);
    static int y;
    static int wordIndex; // index of word along a line, in 0..22
    static uint32_t *pWord;
    static uint32_t wordPrev;
    static uint32_t wordThis;
    static uint32_t wordNext;

    if (!pWord) {
        pWord = plotBufferW;
        y = 0;
    }

    while (pWord < plotBufferW + fb_words) {
        if (wordIndex == 0) {
            wordThis = 0;
            wordNext = *pWord;
        }

        wordPrev = wordThis;
        wordThis = wordNext;

        wordIndex++;
        if (wordIndex == PICO_OSD_LINE_WORDS) {
            // we are on the last word of a line, don't peek at the next word, reset line counter.
            wordNext = 0;
            wordIndex = 0;
        } else {
            wordNext = *(pWord + 1);
        }

        uint32_t whiteThis = wordThis & 0x55555555; // pick out all of the OSD_W (low bits) of each bit pair (OSD_EN, OSD_W).
        uint32_t blackUpdates = (whiteThis >> 1) | (whiteThis << 3); // set OSD_EN according to adjacent OSD_W.
        blackUpdates |= (wordPrev & 0x40000000) >> 29;
        blackUpdates |= (wordNext & 0x1) << 31;
        if (y != 0) {
            blackUpdates |= (*(pWord - PICO_OSD_LINE_WORDS) & 0x55555555) << 1;
        }

        if (y < fb_ny - 1) {
            blackUpdates |= (*(pWord + PICO_OSD_LINE_WORDS) & 0x55555555) << 1;
        }

        *pWord++ = wordThis | blackUpdates;
        if (wordIndex == 0) {
            y++;
        }
    }

    pWord = 0;
    return true;
}

// Update screen buffer (paint characters etc to buffer), up until a time limit.
// Store state so that we can resume.
// Return false when complete (no more to do).
bool osdPioRenderScreenUntil(uint32_t limit_micros)
{
    static bool firstOfVsync = true;
#ifdef OSD_DEBUG
    uint32_t cRender = getCycleCounter();
    ++dd7;
    static uint32_t cycFirst;
#endif
    if (firstOfVsync) {
        firstOfVsync = false;
#ifdef OSD_DEBUG
        cycFirst = getCycleCounter();
        renderStartCycles += cycFirst - startVsyncCycles;
        renderStartCyclesMax = MAX(renderStartCyclesMax, getCycleCounter() - startVsyncCycles);
#endif
    }

#ifdef DEBUG_TESTCARD
    UNUSED(limit_micros);
    plotTestCard();
    transferredSinceVsync = true;
    return false;
#endif

    DEBUG_COUNTER_INST(c1);

    // Proceed with rendering background elements if/as required, if not timed out.
    selectBackgroundBuffer();
    bool complete =
        renderSticksBackgroundUntil(limit_micros) &&
        renderSidebarsUntil(limit_micros);
    selectForegroundBuffer();

    // Continue with foreground elements, if not timed out.
    complete = complete &&
        renderAHUntil(limit_micros) &&
        renderCharsUntil(limit_micros) &&
        renderSticksForegroundUntil(limit_micros);

#if 1
    UNUSED(postProcessUntil);
#else
    // Testing a post-process operation on the background buffer to start with
    // Post processing can e.g. surround white pixels with a black border
    selectBackgroundBuffer();
    complete = complete && postProcessUntil(limit_micros);
    selectForegroundBuffer();
#endif

#ifdef OSD_DEBUG
    uint32_t cd = getCycleCounter() - c1;
    renderTot += cd;
    if (cd > maxcycles) {
        maxcycles = cd;
    }
#endif

    if (complete) {
#ifdef OSD_DEBUG
        tusr++;

        uint32_t cycNow = getCycleCounter();
        renderEndCycles += cycNow - startVsyncCycles;
        uint32_t rdd = cycNow - startVsyncCycles;
        if (rdd > renderEndCyclesMax) {
            dd1 = cycNow - cycFirst;
            renderEndCyclesMax = rdd;
        }
#endif

        firstOfVsync = true;
        transferredSinceVsync = true;
#ifdef OSD_DEBUG
        dd2 += cycNow - cRender;
#endif
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

#endif // USE_FB_OSD
