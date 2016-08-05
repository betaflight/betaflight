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
    BLACKBOX_DEVICE_FLASH = 1,
#endif
#ifdef USE_SDCARD
    BLACKBOX_DEVICE_SDCARD = 2,
#endif

    BLACKBOX_DEVICE_END
} BlackboxDevice;

typedef enum {
    BLACKBOX_RESERVE_SUCCESS,
    BLACKBOX_RESERVE_TEMPORARY_FAILURE,
    BLACKBOX_RESERVE_PERMANENT_FAILURE
} blackboxBufferReserveStatus_e;

/*
 * We want to limit how bursty our writes to the device are. Note that this will also restrict the maximum size of a
 * header write we can make:
 */
#define BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET 256

/*
 * Ideally, each iteration in which we are logging headers would write a similar amount of data to the device as a
 * regular logging iteration. This way we won't hog the CPU by making a gigantic write:
 */
#define BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION 64

extern int32_t blackboxHeaderBudget;

void blackboxWrite(uint8_t value);

int blackboxPrintf(const char *fmt, ...);
void blackboxPrintfHeaderLine(const char *fmt, ...);
int blackboxPrint(const char *s);

void blackboxWriteUnsignedVB(uint32_t value);
void blackboxWriteSignedVB(int32_t value);
void blackboxWriteSignedVBArray(int32_t *array, int count);
void blackboxWriteSigned16VBArray(int16_t *array, int count);
void blackboxWriteS16(int16_t value);
void blackboxWriteTag2_3S32(int32_t *values);
void blackboxWriteTag8_4S16(int32_t *values);
void blackboxWriteTag8_8SVB(int32_t *values, int valueCount);
void blackboxWriteU32(int32_t value);
void blackboxWriteFloat(float value);

void blackboxDeviceFlush(void);
bool blackboxDeviceFlushForce(void);
bool blackboxDeviceOpen(void);
void blackboxDeviceClose(void);

bool blackboxDeviceBeginLog(void);
bool blackboxDeviceEndLog(bool retainLog);

bool isBlackboxDeviceFull(void);

void blackboxReplenishHeaderBudget();
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes);
