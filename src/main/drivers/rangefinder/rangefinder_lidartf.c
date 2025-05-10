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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RANGEFINDER_TF

#include "build/debug.h"
#include "build/build_config.h"

#include "io/serial.h"

#include "drivers/time.h"
#include "drivers/rangefinder/rangefinder.h"
#include "drivers/rangefinder/rangefinder_lidartf.h"

typedef struct {
    rangefinderType_e rfType;
    uint16_t rangeMin;
    uint16_t rangeMax;
} lidarTFInfo_t;


static const lidarTFInfo_t *devInfo = NULL;
static const lidarTFInfo_t devInfos[] = {
    { .rfType = RANGEFINDER_TFMINI, .rangeMin = 40, .rangeMax = 1200},
    { .rfType = RANGEFINDER_TF02,   .rangeMin = 40, .rangeMax = 2200},
    { .rfType = RANGEFINDER_TFNOVA, .rangeMin = 10, .rangeMax = 1400},
};

#define TF_FRAME_LENGTH    6             // Excluding sync bytes (0x59) x 2 and checksum
#define TF_FRAME_SYNC_BYTE 0x59
#define TF_TIMEOUT_MS      (100 * 2)
#define TF_TASK_PERIOD_MS  10

//
// Benewake TFmini frame format
// Byte Off Description
// 1    -   SYNC
// 2    -   SYNC
// 3    0   Measured distance (LSB)
// 4    1   Measured distance (MSB)
// 5    2   Signal strength (LSB)
// 6    3   Signal strength (MSB)
// 7    4   Integral time
// 8    5   Reserved
// 9    -   Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Credibility
// 1. If distance is 12m (1200cm), then OoR.
//
#define TF_MINI_FRAME_INTEGRAL_TIME 4

//
// Benewake TF02 frame format (From SJ-GU-TF02-01 Version: A01)
// Byte Off Description
// 1    -   SYNC
// 2    -   SYNC
// 3    0   Measured distance (LSB)
// 4    1   Measured distance (MSB)
// 5    2   Signal strength (LSB)
// 6    3   Signal strength (MSB)
// 7    4   SIG (Reliability in 1~8, less than 7 is unreliable)
// 8    5   TIME (Exposure time, 3 or 6)
// 9    -   Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Credibility
// 1. If SIG is less than 7, unreliable
// 2. If distance is 22m (2200cm), then OoR.
//
#define TF_02_FRAME_SIG 4

//
// Benewake TFnova frame format
// Byte Off Description
// 1    -   SYNC
// 2    -   SYNC
// 3    0   Measured distance (LSB)
// 4    1   Measured distance (MSB)
// 5    2   Signal strength (LSB)
// 6    3   Signal strength (MSB)
// 7    4   Temp (Chip Temperature, degrees Celsius)
// 8    5   Confidence (Confidence level 0-100)
// 9    -   Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Credibility
// 1. If Confidence level < 90, unreliable
// 2. If distance is 14m (1400cm), then OoR.

#define TF_NOVA_FRAME_CONFIDENCE 5

#define TF_DETECTION_CONE_DECIDEGREES 900  // TODO

static serialPort_t *tfSerialPort = NULL;

typedef enum {
    TF_FRAME_STATE_WAIT_START1,
    TF_FRAME_STATE_WAIT_START2,
    TF_FRAME_STATE_READING_PAYLOAD,
    TF_FRAME_STATE_WAIT_CKSUM,
} tfFrameState_e;

static tfFrameState_e tfFrameState;
static uint8_t tfFrame[TF_FRAME_LENGTH];
static uint8_t tfReceivePosition;

// TFmini and TF02
// Command for 100Hz sampling (10msec interval)
// At 100Hz scheduling, skew will cause 10msec delay at the most.
// This command format does not match latest Benewake documentation
static const uint8_t tfConfigCmd[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

static int lidarTFValue;
static unsigned lidarTFerrors = 0;


static const lidarTFInfo_t* findInfo(rangefinderType_e rfType)
{
    for (const lidarTFInfo_t* p = devInfos; p < ARRAYEND(devInfos); p++) {
        if (p->rfType == rfType) {
            return p;
        }
    }
    return NULL;
}

// configure/reconfigure device
// may be called multiple times (when frames are missing)
static void lidarTFConfig(rangefinderDev_t *dev, const lidarTFInfo_t* inf)
{
    UNUSED(dev);
    switch (inf->rfType) {
    case RANGEFINDER_TFMINI:
    case RANGEFINDER_TF02:
        serialWriteBuf(tfSerialPort, tfConfigCmd, sizeof(tfConfigCmd));
        break;
    default:
        break;
    }
}

static void lidarTFInit(rangefinderDev_t *dev)
{
    UNUSED(dev);

    tfFrameState = TF_FRAME_STATE_WAIT_START1;
    tfReceivePosition = 0;
}

static int tfProcessFrame(const uint8_t* frame, int len)
{
    UNUSED(len);
    uint16_t distance = frame[0] | (frame[1] << 8);
    uint16_t strength = frame[2] | (frame[3] << 8);

    DEBUG_SET(DEBUG_LIDAR_TF, 0, distance);  // 0,1
    DEBUG_SET(DEBUG_LIDAR_TF, 1, strength);  // 2,3
    DEBUG_SET(DEBUG_LIDAR_TF, 2, frame[4]);
    DEBUG_SET(DEBUG_LIDAR_TF, 3, frame[5]);

    // common distance check
    if (distance < devInfo->rangeMin || distance > devInfo->rangeMax) {
        return RANGEFINDER_OUT_OF_RANGE;
    }

    switch (devInfo->rfType) {
    case RANGEFINDER_TFMINI:
        if (frame[TF_MINI_FRAME_INTEGRAL_TIME] == 7) {
            // When integral time is long (7), measured distance tends to be longer by 12~13.
            distance -= 13;
        }
        break;
    case RANGEFINDER_TF02:
        if (frame[TF_02_FRAME_SIG] < 7) {
            return RANGEFINDER_OUT_OF_RANGE;

        }
        break;
    case RANGEFINDER_TFNOVA:
        if (frame[TF_NOVA_FRAME_CONFIDENCE] < 90) {
            return RANGEFINDER_OUT_OF_RANGE;
        }
        break;
    default:
        return RANGEFINDER_HARDWARE_FAILURE;  // internal error
    }
    // distance is valid
    return distance;
}

static void lidarTFUpdate(rangefinderDev_t *dev)
{
    static timeMs_t lastFrameReceivedMs = 0;
     static timeMs_t lastReconfMs = 0;
    const timeMs_t timeNowMs = millis();

    if (tfSerialPort == NULL || devInfo == NULL) {
        return;
    }

    while (serialRxBytesWaiting(tfSerialPort)) {
        uint8_t c = serialRead(tfSerialPort);
        switch (tfFrameState) {
        case TF_FRAME_STATE_WAIT_START1:
            if (c == TF_FRAME_SYNC_BYTE) {
                tfFrameState = TF_FRAME_STATE_WAIT_START2;
            }
            break;

        case TF_FRAME_STATE_WAIT_START2:
            if (c == TF_FRAME_SYNC_BYTE) {
                tfFrameState = TF_FRAME_STATE_READING_PAYLOAD;
            } else {
                tfFrameState = TF_FRAME_STATE_WAIT_START1;
            }
            break;

        case TF_FRAME_STATE_READING_PAYLOAD:
            tfFrame[tfReceivePosition++] = c;
            if (tfReceivePosition == TF_FRAME_LENGTH) {
                tfFrameState = TF_FRAME_STATE_WAIT_CKSUM;
            }
            break;

        case TF_FRAME_STATE_WAIT_CKSUM: {
            uint8_t cksum = TF_FRAME_SYNC_BYTE + TF_FRAME_SYNC_BYTE;  // SYNC bytes are checksummed too, but not stored
            for (int i = 0; i < TF_FRAME_LENGTH; i++) {
                cksum += tfFrame[i];
            }

            if (c == cksum) {
                lidarTFValue = tfProcessFrame(tfFrame, TF_FRAME_LENGTH - 1);  // no checksum
                lastFrameReceivedMs = timeNowMs;
            } else {
                // Checksum error. Simply ignore the current frame.
                ++lidarTFerrors;
                //DEBUG_SET(DEBUG_LIDAR_TF, 3, lidarTFerrors);
            }
            tfFrameState = TF_FRAME_STATE_WAIT_START1;
            tfReceivePosition = 0;
            break;
        }
        }
    }

    // If valid frame hasn't been received for more than a timeout, try reinit.
    if (cmp32(timeNowMs, lastFrameReceivedMs) > TF_TIMEOUT_MS
        && cmp32(timeNowMs, lastReconfMs) > 500) {
        lidarTFConfig(dev, devInfo);
        lastReconfMs = timeNowMs;    // delay sensor reconf
    }
}

// Return most recent device output in cm
// TODO - handle timeout; return value only once (see lidarMT)
static int32_t lidarTFGetDistance(rangefinderDev_t *dev)
{
    UNUSED(dev);

    return lidarTFValue;
}

bool lidarTFDetect(rangefinderDev_t *dev, rangefinderType_e rfType)
{
    const lidarTFInfo_t* inf = findInfo(rfType);
    if (!inf) {
        return false; // this type is not TF
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);
    if (!portConfig) {
        return false;
    }

    tfSerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, 115200, MODE_RXTX, 0);
    if (tfSerialPort == NULL) {
        return false;
    }

    dev->delayMs = TF_TASK_PERIOD_MS;
    dev->maxRangeCm = inf->rangeMax;

    dev->detectionConeDeciDegrees = TF_DETECTION_CONE_DECIDEGREES;
    dev->detectionConeExtendedDeciDegrees = TF_DETECTION_CONE_DECIDEGREES;

    dev->init = &lidarTFInit;
    dev->update = &lidarTFUpdate;
    dev->read = &lidarTFGetDistance;

    devInfo = inf;

    // configure device
    lidarTFConfig(dev, devInfo);

    return true;
}

#endif
