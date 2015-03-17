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

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

typedef enum BlackboxDevice {
    BLACKBOX_DEVICE_SERIAL = 0,

#ifdef USE_FLASHFS
    BLACKBOX_DEVICE_FLASH,
#endif

    BLACKBOX_DEVICE_END
} BlackboxDevice;

uint8_t blackboxWriteChunkSize;

void blackboxWrite(uint8_t value);

int blackboxPrintf(const char *fmt, ...);
int blackboxPrint(const char *s);

void blackboxWriteUnsignedVB(uint32_t value);
void blackboxWriteSignedVB(int32_t value);
void blackboxWriteS16(int16_t value);
void blackboxWriteTag2_3S32(int32_t *values);
void blackboxWriteTag8_4S16(int32_t *values);
void blackboxWriteTag8_8SVB(int32_t *values, int valueCount);

bool blackboxDeviceFlush(void);
bool blackboxDeviceOpen(void);
void blackboxDeviceClose(void);

bool isBlackboxDeviceFull(void);
