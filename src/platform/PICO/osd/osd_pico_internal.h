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

#pragma once

// Uncomment to enable OSD debug counters and timing
// #define DEBUG_OSD_FB_PICO

#include <stdint.h>
#include "drivers/display.h"
#include "osd/osd.h"

#ifdef DEBUG_OSD_TEST_SMALLFONT
#define OSD_BYTES_PER_CHAR 2
// 5x8 but embedded in mcm-style data structure at top left with transparent padding
// so call it 8x8
#define PICO_OSD_GLYPH_WIDTH 8
#define PICO_OSD_GLYPH_HEIGHT 8
#define PICO_OSD_CHAR_WIDTH  8
#define PICO_OSD_CHAR_HEIGHT 12
#else
#define OSD_BYTES_PER_CHAR 3
#define PICO_OSD_GLYPH_WIDTH 12
#define PICO_OSD_GLYPH_HEIGHT 18
#define PICO_OSD_CHAR_WIDTH  12
#define PICO_OSD_CHAR_HEIGHT 18
#endif

// chars OSD_SD_ROWS x OSD_SD_COLS (30 x 16)
// 360 / 8 = 45 x 288
// 2 bits per pixel

// 23 -> 23*4*4 = 368 pixels -> 30.67 chars
// 288 for PAL field
// PIO hard coded to 23 words of pixel data per line (=> 368 pixels)

#define PICO_OSD_LINE_WORDS 23
#define PICO_OSD_BUF_WIDTH (PICO_OSD_LINE_WORDS*4)

// These are currently hard-coded according to the standard SD-style layout
// based on char width 12, char height 18
#define PICO_OSD_BUF_HEIGHT_NTSC (18 * 13)
#define PICO_OSD_BUF_HEIGHT_PAL (18 * 16)
#define PICO_OSD_BUF_HEIGHT_MAX PICO_OSD_BUF_HEIGHT_PAL
#define PICO_OSD_BUF_LENGTH (PICO_OSD_BUF_WIDTH * PICO_OSD_BUF_HEIGHT_MAX)

// 18*13 = 234, 18*16 = 288
STATIC_ASSERT(PICO_OSD_BUF_HEIGHT_NTSC == 234, pico_ntsc_lines_failed);
STATIC_ASSERT(PICO_OSD_BUF_HEIGHT_PAL == 288, pico_pal_lines_failed);

#define PICO_OSD_DISPLAY_WORDS_NTSC (PICO_OSD_LINE_WORDS * PICO_OSD_BUF_HEIGHT_NTSC)
#define PICO_OSD_DISPLAY_WORDS_PAL  (PICO_OSD_LINE_WORDS * PICO_OSD_BUF_HEIGHT_PAL)

// 30 * 16 = 480
#define OSD_CHAR_BUFFER_LENGTH (OSD_SD_COLS * OSD_SD_ROWS)

// Limit time taken for an individual call to fbOsdDrawScreen.
// NB not a true limit, we are allowed to start a new operation if time has not gone past this,
// so it might end up being exceeded by the length of the longest individual operation.

// set this low to diagnose long operations
// #define OSD_DRAWSCREEN_TIME_LIMIT_US 5

#ifndef OSD_DRAWSCREEN_TIME_LIMIT_US
#define OSD_DRAWSCREEN_TIME_LIMIT_US 20
#endif


extern bool transferredSinceVSync;

extern const int fb_nx;
extern int fb_ny;
extern int charLines;
extern const int charsPerLine;
extern int numChars;
extern int fb_words;

extern uint8_t osdCharLineInUse[OSD_SD_ROWS];
extern uint8_t osdCharBuffer[OSD_CHAR_BUFFER_LENGTH];
extern uint8_t *osdBufferA;
extern uint8_t *osdBufferBackground;
extern bool plotToBackground;

// For render functions that might take more, say, 5us.
// Defer running a render function if, by estimate, it would push the time taken past limit_micros.
// Make sure the function doesn't get completely starved of time by allowing at least a few us
// if the function is the first to be attempted.
#define ADJUST_LIMIT_MICROS(op_time_estimate) do \
        limit_micros -= MIN(op_time_estimate, OSD_DRAWSCREEN_TIME_LIMIT_US - 3); \
    while (0)

// Actual character width and height, as variables for convenience
static const int charWidth = PICO_OSD_CHAR_WIDTH;
static const int charHeight = PICO_OSD_CHAR_HEIGHT;

void selectBackgroundBuffer(void);
void selectForegroundBuffer(void);
void setBackgroundItemsPending(void);

#ifdef OSD_FB_PICO_PIXEL_MODE
void plot(int x, int y, int c);
void hLine(int x, int y, int count, int col);
void dhLine(int x, int y, int count);
void dvLine(int x, int y, int count);

void iterLineInit(int x1, int y1, int x2, int y2);
bool iterLineNext(void);
bool iterDLineNext(void);
bool iterQLineNext(void);
bool iterDashedDLineNext(void);
bool iterDashedQLineNext(void);
#endif // OSD_FB_PICO_PIXEL_MODE

// trace / debugging
#ifdef DEBUG_OSD_FB_PICO
extern uint32_t startVsyncCycles;
extern uint32_t startVsyncCyclesPrev;
extern int tus;
extern int tusr;
extern uint32_t maxcycles;
extern int nisz;
extern int dmb;
extern uint32_t maxAHI;
extern uint32_t renderTot;
extern uint32_t drawBGTot;
extern uint32_t drawFGTot;
extern uint32_t renderStartCycles;
extern uint32_t renderEndCycles;
extern uint32_t renderStartCyclesMax;
extern uint32_t renderEndCyclesMax;
extern uint32_t renderWasCheck;
extern uint32_t renderWasCheckD;
extern uint32_t renderWasTransfer;
extern uint32_t dd1,dd2,dd3,dd4,dd5,dd6,dd7,dd8;

// Macros to avoid having to wrap all debug code in #ifdef DEBUG_OSD_FB_PICO
#define DEBUG_ZERO(x) x=0
#define DEBUG_INC(x) ++x
#define DEBUG_COUNTER_INST(x) uint32_t x = getCycleCounter()
#define DEBUG_COUNTER(x) x = getCycleCounter()
#define DEBUG_COUNTER_DIFF(x,y) x = getCycleCounter() - y
#define DEBUG_COUNTER_ACC(x,y) x += getCycleCounter() - y

#else

#define DEBUG_ZERO(x)
#define DEBUG_INC(x)
#define DEBUG_COUNTER_INST(x)
#define DEBUG_COUNTER(x)
#define DEBUG_COUNTER_DIFF(x,y)
#define DEBUG_COUNTER_ACC(x,y)

#endif
