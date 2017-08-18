/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "drivers/display.h"

#ifndef WHITEBRIGHTNESS
  #define WHITEBRIGHTNESS 0x01
#endif
#ifndef BLACKBRIGHTNESS
  #define BLACKBRIGHTNESS 0x00
#endif

/** PAL or NTSC, value is number of chars total */
#define TINYOSD_VIDEO_COLS                35
#define TINYOSD_VIDEO_LINES_NTSC          13
#define TINYOSD_VIDEO_LINES_PAL           16
#define TINYOSD_VIDEO_BUFFER_CHARS_NTSC   (TINYOSD_VIDEO_COLS * TINYOSD_VIDEO_LINES_NTSC)
#define TINYOSD_VIDEO_BUFFER_CHARS_PAL    (TINYOSD_VIDEO_COLS * TINYOSD_VIDEO_LINES_PAL)
#define TINYOSD_VIDEO_BUFFER_SIZE         TINYOSD_VIDEO_BUFFER_CHARS_PAL


// calculate size of dirty table (roudn up to nearest int!)
#define TINYOSD_VIDEO_BUFFER_DIRTY_SIZE ((TINYOSD_VIDEO_BUFFER_SIZE+8/2)/8)

#define SCREEN_BUFFER_GET_DIRTY_FLAG(__pos)   (screenBufferDirty[__pos / 8] & (1 << (__pos % 8)))
#define SCREEN_BUFFER_SET_DIRTY_FLAG(__pos)   { screenBufferDirty[__pos / 8] |= (1 << (__pos % 8)); }
#define SCREEN_BUFFER_CLEAR_DIRTY_FLAG(__pos) { screenBufferDirty[__pos / 8] &= ~(1 << (__pos % 8)); }

#define SCREEN_BUFFER_SET(__pos, __val) { screenBuffer[(__pos)] = (__val); SCREEN_BUFFER_SET_DIRTY_FLAG(__pos); };
#define SCREEN_BUFFER_GET(__pos) (screenBuffer[(__pos)])


//extern uint16_t maxScreenSize;
#define TINYOSD_CRC8_UPDATE(__crc, __val) ((__crc) = tinyosd_crc8_table[(__crc) ^ (__val)])
#define TINYOSD_CRC8_INIT(__crc, __ival) { (__crc) = (__ival); }

#define TINYOSD_COMMAND_SET_STATUS        0x00
#define TINYOSD_COMMAND_FILL_SCREEN       0x01
#define TINYOSD_COMMAND_WRITE_STICKDATA   0x07
//
//
#define TINYOSD_COMMAND_WRITE_PAGE_0      0x10
#define TINYOSD_COMMAND_WRITE_PAGE_1      0x11
#define TINYOSD_COMMAND_WRITE_PAGE_2      0x12
#define TINYOSD_COMMAND_WRITE_PAGE_3      0x13

int tinyOSDGrab(displayPort_t * displayPort);
int tinyOSDRelease(displayPort_t *displayPort);

struct vcdProfile_s;
void    tinyOSDHardwareReset(void);
bool    tinyOSDInit(const struct vcdProfile_s *vcdProfile);
int    tinyOSDDrawScreen(displayPort_t *);
void    tinyOSDWriteNvm(uint8_t char_address, const uint8_t *font_data);
uint8_t tinyOSDGetRowsCount(void);
uint8_t tinyOSDGetColsCount(void);
int tinyOSDWriteString(displayPort_t *, uint8_t x, uint8_t y, const char *buff);
int tinyOSDWriteChar(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t c);
int tinyOSDClearScreen(displayPort_t *);
void tinyOSDRefreshAll(void);
bool tinyOSDIsTransferInProgress(const displayPort_t *);
int tinyOSDHeartbeat(displayPort_t *displayPort);
void tinyOSDResync(displayPort_t *displayPort);
uint32_t tinyOSDTxBytesFree(const displayPort_t *displayPort);
uint8_t* tinyOSDGetScreenBuffer(void);
