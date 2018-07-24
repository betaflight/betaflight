/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifdef USE_OSD_SLAVE
#include "common/time.h"

struct displayPort_s;

extern bool osdSlaveIsLocked;

// init
void osdSlaveInit(struct displayPort_s *osdDisplayPort);
bool osdSlaveInitialized(void);

// task api
bool osdSlaveCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
void osdSlaveUpdate(timeUs_t currentTimeUs);

// msp api
void osdSlaveHeartbeat(void);
void osdSlaveClearScreen(void);
void osdSlaveWriteChar(const uint8_t x, const uint8_t y, const uint8_t c);
void osdSlaveWrite(const uint8_t x, const uint8_t y, const char *s);

void osdSlaveDrawScreen(void);

#endif
