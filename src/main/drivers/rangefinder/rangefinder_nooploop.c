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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_NOOPLOOP

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_nooploop.h"

#define NOOPLOOP_FRAME_SYNC_BYTE_0 0x57
#define NOOPLOOP_FRAME_SYNC_BYTE_1 0x00
#define NOOPLOOP_FRAME_LENGTH      16

// Nooploop TOFSense F2 frame format (16 bytes total)
// Byte 0: Sync byte 0 (0x57)
// Byte 1: Sync byte 1 (0x00)
// Bytes 2-7: Reserved/unused (per vendor examples)
// Bytes 8-10: Distance in mm (24-bit signed little-endian)
// Byte 11: Status (0xFF indicates invalid/out-of-range in vendor examples)
// Bytes 12-14: Signal strength (uint16 little-endian) + reserved
// Byte 15: Checksum (8-bit sum of bytes 0-14)

#define NOOPLOOP_TIMEOUT_MS        (100 * 2)
#define NOOPLOOP_TASK_PERIOD_MS    10

#define NOOPLOOP_BAUDRATE           115200

#define NOOPLOOP_DETECTION_CONE_DECIDEGREES 900

static serialPort_t *nooploopSerialPort = NULL;

typedef struct {
    rangefinderType_e rfType;
    uint16_t rangeMin;
    uint16_t rangeMax;
} nooploopInfo_t;

static const nooploopInfo_t *devInfo = NULL;
static const nooploopInfo_t devInfos[] = {
    { .rfType = RANGEFINDER_NOOPLOOP_F,      .rangeMin = 2, .rangeMax = 1000 },
    { .rfType = RANGEFINDER_NOOPLOOP_FP,     .rangeMin = 2, .rangeMax = 1700 },
    { .rfType = RANGEFINDER_NOOPLOOP_F2,     .rangeMin = 2, .rangeMax = 500  },
    { .rfType = RANGEFINDER_NOOPLOOP_F2P,    .rangeMin = 2, .rangeMax = 1700 },
    { .rfType = RANGEFINDER_NOOPLOOP_F2PH,   .rangeMin = 2, .rangeMax = 3300 },
    { .rfType = RANGEFINDER_NOOPLOOP_F2MINI, .rangeMin = 2, .rangeMax = 500  },
};

typedef enum {
    NOOPLOOP_FRAME_STATE_WAIT_SYNC0,
    NOOPLOOP_FRAME_STATE_WAIT_SYNC1,
    NOOPLOOP_FRAME_STATE_READING,
} nooploopFrameState_e;

static nooploopFrameState_e nooploopFrameState;
static uint8_t nooploopFrame[NOOPLOOP_FRAME_LENGTH];
static uint8_t nooploopFramePos;

static int32_t nooploopLatestDistance = RANGEFINDER_NO_NEW_DATA;
static bool nooploopHasNewData = false;

static const nooploopInfo_t* findInfo(rangefinderType_e rfType)
{
    for (const nooploopInfo_t* p = devInfos; p < ARRAYEND(devInfos); p++) {
        if (p->rfType == rfType) {
            return p;
        }
    }
    return NULL;
}

static inline int32_t nooploopParseInt24(const uint8_t b0, const uint8_t b1, const uint8_t b2)
{
    int32_t value = (int32_t)(b0 | (b1 << 8) | (b2 << 16));
    if (value & 0x800000) {
        value |= 0xFF000000;
    }
    return value;
}

static void nooploopInit(rangefinderDev_t *dev)
{
    UNUSED(dev);
    nooploopFrameState = NOOPLOOP_FRAME_STATE_WAIT_SYNC0;
    nooploopFramePos = 0;
    nooploopLatestDistance = RANGEFINDER_NO_NEW_DATA;
    nooploopHasNewData = false;
}

static int32_t nooploopProcessFrame(const uint8_t *frame)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < (NOOPLOOP_FRAME_LENGTH - 1); i++) {
        checksum += frame[i];
    }

    if (checksum != frame[NOOPLOOP_FRAME_LENGTH - 1]) {
        return RANGEFINDER_NO_NEW_DATA;
    }

    const int32_t distanceMm = nooploopParseInt24(frame[8], frame[9], frame[10]);
    const uint8_t status = frame[11];

    if (distanceMm < 0) {
        return RANGEFINDER_OUT_OF_RANGE;
    }

    if (devInfo && (distanceMm < (devInfo->rangeMin * 10) || distanceMm > (devInfo->rangeMax * 10))) {
        return RANGEFINDER_OUT_OF_RANGE;
    }

    // The status byte is provided by the Nooploop sensor. Based on
    // the vendor examples and ArduPilot backend, 0xFF indicates
    // invalid/out-of-range readings. Other values are treated as valid.
    if (status == 0xFF) {
        return RANGEFINDER_OUT_OF_RANGE;
    }

    return distanceMm / 10; // mm -> cm
}

static void nooploopUpdate(rangefinderDev_t *dev)
{
    UNUSED(dev);

    static timeMs_t lastFrameReceivedMs = 0;
    static timeMs_t firstUpdateMs = 0;
    const timeMs_t timeNowMs = millis();

    if (nooploopSerialPort == NULL) {
        return;
    }

    if (firstUpdateMs == 0) {
        firstUpdateMs = timeNowMs;
    }

    while (serialRxBytesWaiting(nooploopSerialPort)) {
        const uint8_t c = serialRead(nooploopSerialPort);

        switch (nooploopFrameState) {
        case NOOPLOOP_FRAME_STATE_WAIT_SYNC0:
            if (c == NOOPLOOP_FRAME_SYNC_BYTE_0) {
                nooploopFrame[0] = c;
                nooploopFrameState = NOOPLOOP_FRAME_STATE_WAIT_SYNC1;
            }
            break;

        case NOOPLOOP_FRAME_STATE_WAIT_SYNC1:
            if (c == NOOPLOOP_FRAME_SYNC_BYTE_1) {
                nooploopFrame[1] = c;
                nooploopFramePos = 2;
                nooploopFrameState = NOOPLOOP_FRAME_STATE_READING;
            } else if (c == NOOPLOOP_FRAME_SYNC_BYTE_0) {
                nooploopFrame[0] = c;
                nooploopFrameState = NOOPLOOP_FRAME_STATE_WAIT_SYNC1;
            } else {
                nooploopFrameState = NOOPLOOP_FRAME_STATE_WAIT_SYNC0;
            }
            break;

        case NOOPLOOP_FRAME_STATE_READING:
            nooploopFrame[nooploopFramePos++] = c;
            if (nooploopFramePos == NOOPLOOP_FRAME_LENGTH) {
                const int32_t distance = nooploopProcessFrame(nooploopFrame);
                if (distance != RANGEFINDER_NO_NEW_DATA) {
                    nooploopLatestDistance = distance;
                    nooploopHasNewData = true;
                    lastFrameReceivedMs = timeNowMs;
                }
                nooploopFrameState = NOOPLOOP_FRAME_STATE_WAIT_SYNC0;
                nooploopFramePos = 0;
            }
            break;
        }
    }

    if ((lastFrameReceivedMs && cmp32(timeNowMs, lastFrameReceivedMs) > NOOPLOOP_TIMEOUT_MS)
        || (!lastFrameReceivedMs && cmp32(timeNowMs, firstUpdateMs) > NOOPLOOP_TIMEOUT_MS)) {
        if (nooploopLatestDistance != RANGEFINDER_HARDWARE_FAILURE) {
            nooploopLatestDistance = RANGEFINDER_HARDWARE_FAILURE;
            nooploopHasNewData = true;
        }
        lastFrameReceivedMs = timeNowMs;
    }
}

static int32_t nooploopGetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);

    if (!nooploopHasNewData) {
        return RANGEFINDER_NO_NEW_DATA;
    }

    nooploopHasNewData = false;
    return nooploopLatestDistance;
}

bool nooploopDetect(rangefinderDev_t *dev, rangefinderType_e rfType)
{
    const nooploopInfo_t *inf = findInfo(rfType);
    if (!inf) {
        return false;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_NL);
    if (!portConfig) {
        return false;
    }

    nooploopSerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_NL, NULL, NULL, NOOPLOOP_BAUDRATE, MODE_RXTX, 0);
    if (nooploopSerialPort == NULL) {
        return false;
    }

    dev->delayMs = NOOPLOOP_TASK_PERIOD_MS;
    dev->maxRangeCm = inf->rangeMax;

    dev->detectionConeDeciDegrees = NOOPLOOP_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = NOOPLOOP_DETECTION_CONE_DECIDEGREES;

    dev->init = &nooploopInit;
    dev->update = &nooploopUpdate;
    dev->read = &nooploopGetDistance;

    devInfo = inf;

    return true;
}

#endif
