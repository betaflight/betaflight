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
#include <stdlib.h>

#include "platform.h"

#ifdef USE_SERIALRX_SUMD

#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/sumd.h"

#include "telemetry/telemetry.h"

// driver for SUMD receiver using UART2

// FIXME test support for more than 8 channels, should probably work up to 12 channels

#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAX_CHANNEL 16
#define SUMD_BUFFSIZE (SUMD_MAX_CHANNEL * 2 + 5) // 6 channels + 5 = 17 bytes for 6 channels

#define SUMD_BAUDRATE 115200

static bool sumdFrameDone = false;
static uint16_t sumdChannels[SUMD_MAX_CHANNEL];
static uint16_t crc;

#define CRC_POLYNOME 0x1021

// CRC calculation, adds a 8 bit unsigned to 16 bit crc
static void CRC16(uint8_t value)
{
    uint8_t i;

    crc = crc ^ (int16_t)value << 8;
    for (i = 0; i < 8; i++) {
    if (crc & 0x8000)
        crc = (crc << 1) ^ CRC_POLYNOME;
    else
        crc = (crc << 1);
    }
}

static uint8_t sumd[SUMD_BUFFSIZE] = { 0, };
static uint8_t sumdChannelCount;

// Receive ISR callback
static void sumdDataReceive(uint16_t c)
{
    timeUs_t sumdTime;
    static timeUs_t sumdTimeLast;
    static uint8_t sumdIndex;

    sumdTime = micros();
    if (cmpTimeUs(sumdTime, sumdTimeLast) > 4000)
        sumdIndex = 0;
    sumdTimeLast = sumdTime;

    if (sumdIndex == 0) {
        if (c != SUMD_SYNCBYTE)
            return;
        else
        {
            sumdFrameDone = false; // lazy main loop didnt fetch the stuff
            crc = 0;
        }
    }
    if (sumdIndex == 2)
        sumdChannelCount = (uint8_t)c;
    if (sumdIndex < SUMD_BUFFSIZE)
        sumd[sumdIndex] = (uint8_t)c;
    sumdIndex++;
    if (sumdIndex < sumdChannelCount * 2 + 4)
        CRC16((uint8_t)c);
    else
        if (sumdIndex == sumdChannelCount * 2 + 5) {
            sumdIndex = 0;
            sumdFrameDone = true;
        }
}

#define SUMD_OFFSET_CHANNEL_1_HIGH 3
#define SUMD_OFFSET_CHANNEL_1_LOW 4
#define SUMD_BYTES_PER_CHANNEL 2


#define SUMD_FRAME_STATE_OK 0x01
#define SUMD_FRAME_STATE_FAILSAFE 0x81

uint8_t sumdFrameStatus(void)
{
    uint8_t channelIndex;

    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!sumdFrameDone) {
        return frameStatus;
    }

    sumdFrameDone = false;

    // verify CRC
    if (crc != ((sumd[SUMD_BYTES_PER_CHANNEL * sumdChannelCount + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
            (sumd[SUMD_BYTES_PER_CHANNEL * sumdChannelCount + SUMD_OFFSET_CHANNEL_1_LOW])))
        return frameStatus;

    switch (sumd[1]) {
        case SUMD_FRAME_STATE_FAILSAFE:
            frameStatus = RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
            break;
        case SUMD_FRAME_STATE_OK:
            frameStatus = RX_FRAME_COMPLETE;
            break;
        default:
            return frameStatus;
    }

    if (sumdChannelCount > SUMD_MAX_CHANNEL)
        sumdChannelCount = SUMD_MAX_CHANNEL;

    for (channelIndex = 0; channelIndex < sumdChannelCount; channelIndex++) {
        sumdChannels[channelIndex] = (
            (sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
            sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_LOW]
        );
    }
    return frameStatus;
}

static uint16_t sumdReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return sumdChannels[chan] / 8;
}

bool sumdInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);

    rxRuntimeConfig->channelCount = SUMD_MAX_CHANNEL;
    rxRuntimeConfig->rxRefreshRate = 11000;

    rxRuntimeConfig->rcReadRawFn = sumdReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = sumdFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    serialPort_t *sumdPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        sumdDataReceive,
        SUMD_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        SERIAL_NOT_INVERTED | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

#ifdef TELEMETRY
    if (portShared) {
        telemetrySharedPort = sumdPort;
    }
#endif

    return sumdPort != NULL;
}
#endif // USE_SERIALRX_SUMD
