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

/** PAL or NTSC, value is number of chars total */
#define OPENTCO_OSD_VIDEO_COLS                35
#define OPENTCO_OSD_VIDEO_LINES_NTSC          13
#define OPENTCO_OSD_VIDEO_LINES_PAL           16
#define OPENTCO_OSD_VIDEO_BUFFER_CHARS_NTSC   (OPENTCO_OSD_VIDEO_COLS * OPENTCO_OSD_VIDEO_LINES_NTSC)
#define OPENTCO_OSD_VIDEO_BUFFER_CHARS_PAL    (OPENTCO_OSD_VIDEO_COLS * OPENTCO_OSD_VIDEO_LINES_PAL)
#define OPENTCO_OSD_VIDEO_BUFFER_SIZE         OPENTCO_OSD_VIDEO_BUFFER_CHARS_PAL

#define OPENTCO_OSD_STICKSIZE_X  96.0f
#define OPENTCO_OSD_STICKSIZE_Y 128.0f


#define OPENTCO_OSD_CYCLETIME_US         (1000000/25)  // 25 Hz


int opentcoOSDGrab(displayPort_t * displayPort);
int opentcoOSDRelease(displayPort_t *displayPort);

struct vcdProfile_s;
void    opentcoOSDHardwareReset(void);
bool    opentcoOSDInit(const struct vcdProfile_s *vcdProfile);
int     opentcoOSDDrawScreen(displayPort_t *);
void    opentcoOSDWriteNvm(uint8_t char_address, const uint8_t *font_data);

uint8_t opentcoOSDGetRowsCount(void);
uint8_t opentcoOSDGetColsCount(void);

int  opentcoOSDWriteString(displayPort_t *, uint8_t x, uint8_t y, const char *buff);
int  opentcoOSDWriteChar(displayPort_t *, uint8_t x, uint8_t y, uint8_t c);
int  opentcoOSDReloadProfile(displayPort_t *);
int  opentcoOSDClearScreen(displayPort_t *);
int  opentcoOSDFillRegion(displayPort_t*, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t value);
void opentcoOSDRefreshAll(void);
bool opentcoOSDIsTransferInProgress(const displayPort_t *);
int  opentcoOSDHeartbeat(displayPort_t *displayPort);
void opentcoOSDResync(displayPort_t *displayPort);

uint32_t opentcoOSDTxBytesFree(const displayPort_t *displayPort);

