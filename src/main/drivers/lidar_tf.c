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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#include "build/build_config.h"

#include "io/serial.h"

#include "lidar_tf.h"

#ifdef USE_LIDAR_TF

#define TF_FRAME_LENGTH 9  // Including sync bytes (0x59) x 2 and checksum
#define TF_FRAME_SYNC_BYTE 0x59

static serialPort_t *tfSerialPort = NULL;

typedef enum {
    TF_FRAME_STATE_WAIT_START1,
    TF_FRAME_STATE_WAIT_START2,
    TF_FRAME_STATE_READING_PAYLOAD,
    TF_FRAME_STATE_WAIT_CKSUM,
} tfFrameState_e;

static tfFrameState_e tfFrameState;

//
// Benewake TFmini frame format
// Byte
// 0: SYNC
// 1: SYNC
// 2: Measured distance (LSB)
// 3: Measured distance (MSB)
// 4: Signal strength (LSB)
// 5: Signal strength (MSB)
// 6: Quality (*1)
// 7: ?
// 8: Checksum (Unsigned 8-bit sum of bytes 0~7)
//
// Note *1: TFmini product specification (Version A00) specifies byte 6 is
// reserved and byte 7 is quality, but it seems like byte 7 contains
// 7 for good measurement and 2 for bad measurement.
// 

static uint8_t tfFrame[TF_FRAME_LENGTH];
static uint8_t tfPosition;

// TFmini
// Command for 100Hz sampling (10msec interval)
// At 100Hz scheduling, skew will cause 10msec delay at the most.
static uint8_t tfCmdStandard[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

int32_t lidarTFValue;

void lidarTFInit(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_LIDAR_TF);

    if (!portConfig) {
        return;
    }

    tfSerialPort = openSerialPort(portConfig->identifier, FUNCTION_LIDAR_TF, NULL, NULL, 115200, MODE_RXTX, 0);

    if (tfSerialPort == NULL) {
        return;
    }

    serialWriteBuf(tfSerialPort, tfCmdStandard, ARRAYLEN(tfCmdStandard));

    tfFrameState = TF_FRAME_STATE_WAIT_START1;
    tfFrame[0] = TF_FRAME_SYNC_BYTE;
    tfFrame[1] = TF_FRAME_SYNC_BYTE;
    tfPosition = 2;
}

void lidarTFCompute(uint16_t distance, uint16_t strength, uint8_t quality)
{
    UNUSED(strength);

    if (quality < 7) {
        goto invalid;
    }

    // XXX Todo: attitude compensation
    lidarTFValue = distance;
    return;

invalid:;
    lidarTFValue = -1;
    return;
}

void lidarTFUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (tfSerialPort == NULL) {
        return;
    }

    while (serialRxBytesWaiting(tfSerialPort)) {
        uint8_t c = serialRead(tfSerialPort);
        switch (tfFrameState) {
        case TF_FRAME_STATE_WAIT_START1:
            if (c == 0x59) {
                tfFrameState = TF_FRAME_STATE_WAIT_START2;
            }
            break;

        case TF_FRAME_STATE_WAIT_START2:
            if (c == 0x59) {
                tfFrameState = TF_FRAME_STATE_READING_PAYLOAD;
            } else {
                tfFrameState = TF_FRAME_STATE_WAIT_START1;
            }
            break;

        case TF_FRAME_STATE_READING_PAYLOAD:
            tfFrame[tfPosition++] = c;
            if (tfPosition == TF_FRAME_LENGTH - 1) {
                tfFrameState = TF_FRAME_STATE_WAIT_CKSUM;
            }
            break;

        case TF_FRAME_STATE_WAIT_CKSUM:
            {
                uint8_t cksum = 0;
                for (int i = 0 ; i < TF_FRAME_LENGTH - 1 ; i++) {
                    cksum += tfFrame[i];
                }

                if (c == cksum) {
                    lidarTFCompute(tfFrame[2] | (tfFrame[3] << 8), tfFrame[4] | (tfFrame[5] << 8), tfFrame[6]);
#if 0
                    debug[0] = (tfFrame[3] << 8) | tfFrame[2];
                    debug[1] = (tfFrame[5] << 8) | tfFrame[4];
                    debug[2] = (tfFrame[7] << 8) | tfFrame[6];
#endif
                    debug[0] = lidarTFValue;
                } else {
#if 0
                    // Checksum error. Simply discard the current frame.
                    debug[3]++;
#endif
                }
            }

            tfFrameState = TF_FRAME_STATE_WAIT_START1;
            tfPosition = 2;

            break;
        }
    }
}

int32_t lidarTFGetDistance(void)
{
    return lidarTFValue;
}
#endif
