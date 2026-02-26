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
// #define OSD_FB_PICO_DEBUG

#include <stdint.h>
#include "drivers/display.h"
#include "osd/osd.h"

// each char 12 x 18 pixels
#define PICO_OSD_CHAR_WIDTH  12
#define PICO_OSD_CHAR_HEIGHT 18

// chars OSD_SD_ROWS x OSD_SD_COLS (30 x 16)
// 360 / 8 = 45 x 288
// 2 bits per pixel

// 23 -> 23*4*4 = 368 pixels -> 30.67 chars
// 288 for PAL field
// PIO hard coded to 23 words of pixel data per line (=> 368 pixels)
#define PICO_OSD_LINE_WORDS 23
#define PICO_OSD_BUF_WIDTH (PICO_OSD_LINE_WORDS*4)

#define PICO_OSD_BUF_HEIGHT_NTSC (PICO_OSD_CHAR_HEIGHT * VIDEO_LINES_NTSC)
#define PICO_OSD_BUF_HEIGHT_PAL (PICO_OSD_CHAR_HEIGHT * VIDEO_LINES_PAL)
#define PICO_OSD_BUF_HEIGHT_MAX PICO_OSD_BUF_HEIGHT_PAL
#define PICO_OSD_BUF_LENGTH (PICO_OSD_BUF_WIDTH * PICO_OSD_BUF_HEIGHT_MAX)

// 18*13 = 234, 18*16 = 288
STATIC_ASSERT(PICO_OSD_BUF_HEIGHT_NTSC == 234, pico_ntsc_lines_failed);
STATIC_ASSERT(PICO_OSD_BUF_HEIGHT_PAL == 288, pico_pal_lines_failed);

#define PICO_OSD_DISPLAY_WORDS_NTSC (PICO_OSD_LINE_WORDS * PICO_OSD_BUF_HEIGHT_NTSC)
#define PICO_OSD_DISPLAY_WORDS_PAL  (PICO_OSD_LINE_WORDS * PICO_OSD_BUF_HEIGHT_PAL)

// 30 * 16 = 480
#define OSD_CHAR_BUFFER_LENGTH (OSD_SD_COLS * OSD_SD_ROWS)

extern bool transferredSinceVsync;

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

void selectBackgroundBuffer(void);
void selectForegroundBuffer(void);
void setBackgroundItemsPending(void);

void plot(int x, int y, int c);
void hLine(int x, int y, int count, int col);
void dhLine(int x, int y, int count);
void dvLine(int x, int y, int count);

void iterLineInit(int x1, int y1, int x2, int y2);
bool iterDLineNext(void);
bool iterQLineNext(void);
bool iterDashedDLineNext(void);
bool iterDashedQLineNext(void);

// trace / debugging
#ifdef OSD_FB_PICO_DEBUG
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

// Macros to avoid having to wrap all debug code in #ifdef OSD_FB_PICO_DEBUG
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
