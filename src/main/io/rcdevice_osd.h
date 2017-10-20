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

#include "drivers/display.h"

#define RCDEVICE_PROTOCOL_OSD_VIDEO_LINES_NTSC  13
#define RCDEVICE_PROTOCOL_OSD_VIDEO_LINES_PAL   16

struct vcdProfile_s;

bool rcdeviceOSDInit(const struct vcdProfile_s *vcdProfile);
int rcdeviceOSDGrab(displayPort_t *displayPort);
int rcdeviceOSDRelease(displayPort_t *displayPort);

int rcdeviceOSDDrawScreen(displayPort_t *);

int rcdeviceOSDWriteString(displayPort_t *, uint8_t x, uint8_t y, const char *buff);
int rcdeviceOSDWriteChar(displayPort_t *, uint8_t x, uint8_t y, uint8_t c);
int rcdeviceOSDReloadProfile(displayPort_t *);
int rcdeviceOSDClearScreen(displayPort_t *);
bool rcdeviceOSDIsTransferInProgress(const displayPort_t *);
int rcdeviceOSDHeartbeat(displayPort_t *displayPort);
void rcdeviceOSDResync(displayPort_t *displayPort);
uint32_t rcdeviceOSDTxBytesFree(const displayPort_t *displayPort);
int rcdeviceScreenSize(const displayPort_t *displayPort);
