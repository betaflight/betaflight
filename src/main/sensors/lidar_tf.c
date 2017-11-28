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

#include "sensors/lidar_tf.h"

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
static uint8_t tfFrame[TF_FRAME_LENGTH];
static uint8_t tfPosition;

// TFmini
// Command for 100Hz sampling (10msec interval)
// At 100Hz scheduling, skew will cause 10msec delay at the most.
static uint8_t tfCmdStandard[] = { 0x42, 0x57, 0x02, 0x00, 0x00, 0x00, 0x01, 0x06 };

bool lidarTFValid = false;
uint32_t lidarTFAltitude;
uint32_t lidarTFQuality;

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
    UNUSED(quality);

    lidarTFQuality = quality;

    if (distance < 20 || strength > 300) {
        goto invalid;
    }

    // XXX Todo: attitude compensation
    lidarTFAltitude = distance;
    lidarTFValid = true;
    return;

invalid:;
    lidarTFValid = false;
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
                    lidarTFCompute((tfFrame[3] << 8) | tfFrame[2], (tfFrame[5] << 8) | tfFrame[4], tfFrame[7]);
                    debug[0]++;
                    debug[1] = (tfFrame[3] << 8) | tfFrame[2];
                    debug[2] = (tfFrame[5] << 8) | tfFrame[4];
                    debug[3] = lidarTFValid;
                } else {
                    // Checksum error. Simply discard the current frame.
                }
            }

            tfFrameState = TF_FRAME_STATE_WAIT_START1;
            tfPosition = 2;

            break;
        }
    }
}
#endif
