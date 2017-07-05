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

#ifdef USE_SERIALRX_XBUS

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/xbus.h"

#include "telemetry/telemetry.h"

//
// Serial driver for JR's XBus (MODE B) receiver
//

#define XBUS_CHANNEL_COUNT 12
#define XBUS_RJ01_CHANNEL_COUNT 12

// Frame is: ID(1 byte) + 12*channel(2 bytes) + CRC(2 bytes) = 27
#define XBUS_FRAME_SIZE 27

#define XBUS_RJ01_FRAME_SIZE 33
#define XBUS_RJ01_MESSAGE_LENGTH 30
#define XBUS_RJ01_OFFSET_BYTES 3

#define XBUS_CRC_AND_VALUE 0x8000
#define XBUS_CRC_POLY 0x1021

#define XBUS_BAUDRATE 115200
#define XBUS_RJ01_BAUDRATE 250000
#define XBUS_MAX_FRAME_TIME 8000

// NOTE!
// This is actually based on ID+LENGTH (nibble each)
// 0xA - Multiplex ID (also used by JR, no idea why)
// 0x1 - 12 channels
// 0x2 - 16 channels
// However, the JR XG14 that is used for test at the moment
// does only use 0xA1 as its output. This is why the implementation
// is based on these numbers only. Maybe update this in the future?
#define XBUS_START_OF_FRAME_BYTE (0xA1)

// Pulse length convertion from [0...4095] to µs:
//      800µs  -> 0x000
//      1500µs -> 0x800
//      2200µs -> 0xFFF
// Total range is: 2200 - 800 = 1400 <==> 4095
// Use formula: 800 + value * 1400 / 4096 (i.e. a shift by 12)
#define XBUS_CONVERT_TO_USEC(V) (800 + ((V * 1400) >> 12))

static bool xBusFrameReceived = false;
static bool xBusDataIncoming = false;
static uint8_t xBusFramePosition;
static uint8_t xBusFrameLength;
static uint8_t xBusChannelCount;
static uint8_t xBusProvider;


// Use max values for ram areas
static volatile uint8_t xBusFrame[XBUS_RJ01_FRAME_SIZE];
static uint16_t xBusChannelData[XBUS_RJ01_CHANNEL_COUNT];

// The xbus mode B CRC calculations
static uint16_t xBusCRC16(uint16_t crc, uint8_t value)
{
    uint8_t i;

    crc = crc ^ (int16_t)value << 8;

    for (i = 0; i < 8; i++) {
        if (crc & XBUS_CRC_AND_VALUE) {
            crc = crc << 1 ^ XBUS_CRC_POLY;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

// Full RJ01 message CRC calculations
uint8_t xBusRj01CRC8(uint8_t inData, uint8_t seed)
{
    uint8_t bitsLeft;
    uint8_t temp;

    for (bitsLeft = 8; bitsLeft > 0; bitsLeft--) {
        temp = ((seed ^ inData) & 0x01);

        if (temp == 0) {
            seed >>= 1;
        } else {
            seed ^= 0x18;
            seed >>= 1;
            seed |= 0x80;
        }

        inData >>= 1;
    }

    return seed;
}


static void xBusUnpackModeBFrame(uint8_t offsetBytes)
{
    // Calculate the CRC of the incoming frame
    uint16_t crc = 0;
    uint16_t inCrc = 0;
    uint8_t i = 0;
    uint16_t value;
    uint8_t frameAddr;

    // Calculate on all bytes except the final two CRC bytes
    for (i = 0; i < XBUS_FRAME_SIZE - 2; i++) {
        inCrc = xBusCRC16(inCrc, xBusFrame[i+offsetBytes]);
    }

    // Get the received CRC
    crc = ((uint16_t)xBusFrame[offsetBytes + XBUS_FRAME_SIZE - 2]) << 8;
    crc = crc + ((uint16_t)xBusFrame[offsetBytes + XBUS_FRAME_SIZE - 1]);

    if (crc == inCrc) {
        // Unpack the data, we have a valid frame
        for (i = 0; i < xBusChannelCount; i++) {

            frameAddr = offsetBytes + 1 + i * 2;
            value = ((uint16_t)xBusFrame[frameAddr]) << 8;
            value = value + ((uint16_t)xBusFrame[frameAddr + 1]);

            // Convert to internal format
            xBusChannelData[i] = XBUS_CONVERT_TO_USEC(value);
        }

        xBusFrameReceived = true;
    }

}

static void xBusUnpackRJ01Frame(void)
{
    // Calculate the CRC of the incoming frame
    uint8_t outerCrc = 0;
    uint8_t i = 0;

    // When using the Align RJ01 receiver with
    // a MODE B setting in the radio (XG14 tested)
    // the MODE_B -frame is packed within some
    // at the moment unknown bytes before and after:
    // 0xA1 LEN __ 0xA1 12*(High + Low) CRC1 CRC2 + __ __ CRC_OUTER
    // Compared to a standard MODE B frame that only
    // contains the "middle" package.
    // Hence, at the moment, the unknown header and footer
    // of the RJ01 MODEB packages are discarded.
    // However, the LAST byte (CRC_OUTER) is infact an 8-bit
    // CRC for the whole package, using the Dallas-One-Wire CRC
    // method.
    // So, we check both these values as well as the provided length
    // of the outer/full message (LEN)

    //
    // Check we have correct length of message
    //
    if (xBusFrame[1] != XBUS_RJ01_MESSAGE_LENGTH)
    {
        // Unknown package as length is not ok
        return;
    }

    //
    // CRC calculation & check for full message
    //
    for (i = 0; i < xBusFrameLength - 1; i++) {
        outerCrc = xBusRj01CRC8(outerCrc, xBusFrame[i]);
    }

    if (outerCrc != xBusFrame[xBusFrameLength - 1])
    {
        // CRC does not match, skip this frame
        return;
    }

    // Now unpack the "embedded MODE B frame"
    xBusUnpackModeBFrame(XBUS_RJ01_OFFSET_BYTES);
}

// Receive ISR callback
static void xBusDataReceive(uint16_t c)
{
    timeUs_t now;
    static timeUs_t xBusTimeLast;
    timeDelta_t xBusTimeInterval;

    // Check if we shall reset frame position due to time
    now = micros();
    xBusTimeInterval = cmpTimeUs(now, xBusTimeLast);
    xBusTimeLast = now;
    if (xBusTimeInterval > XBUS_MAX_FRAME_TIME) {
        xBusFramePosition = 0;
        xBusDataIncoming = false;
    }

    // Check if we shall start a frame?
    if ((xBusFramePosition == 0) && (c == XBUS_START_OF_FRAME_BYTE)) {
        xBusDataIncoming = true;
    }

    // Only do this if we are receiving to a frame
    if (xBusDataIncoming == true) {
        // Store in frame copy
        xBusFrame[xBusFramePosition] = (uint8_t)c;
        xBusFramePosition++;
    }

    // Done?
    if (xBusFramePosition == xBusFrameLength) {
        switch (xBusProvider) {
        case SERIALRX_XBUS_MODE_B:
            xBusUnpackModeBFrame(0);
            break;
        case SERIALRX_XBUS_MODE_B_RJ01:
            xBusUnpackRJ01Frame();
            break;
        }
        xBusDataIncoming = false;
        xBusFramePosition = 0;
    }
}

// Indicate time to read a frame from the data...
uint8_t xBusFrameStatus(void)
{
    if (!xBusFrameReceived) {
        return RX_FRAME_PENDING;
    }

    xBusFrameReceived = false;

    return RX_FRAME_COMPLETE;
}

static uint16_t xBusReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    uint16_t data;

    // Deliver the data wanted
    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    data = xBusChannelData[chan];

    return data;
}

bool xBusInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    uint32_t baudRate;

    switch (rxConfig->serialrx_provider) {
    case SERIALRX_XBUS_MODE_B:
        rxRuntimeConfig->channelCount = XBUS_CHANNEL_COUNT;
        xBusFrameReceived = false;
        xBusDataIncoming = false;
        xBusFramePosition = 0;
        baudRate = XBUS_BAUDRATE;
        xBusFrameLength = XBUS_FRAME_SIZE;
        xBusChannelCount = XBUS_CHANNEL_COUNT;
        xBusProvider = SERIALRX_XBUS_MODE_B;
        break;
    case SERIALRX_XBUS_MODE_B_RJ01:
        rxRuntimeConfig->channelCount = XBUS_RJ01_CHANNEL_COUNT;
        xBusFrameReceived = false;
        xBusDataIncoming = false;
        xBusFramePosition = 0;
        baudRate = XBUS_RJ01_BAUDRATE;
        xBusFrameLength = XBUS_RJ01_FRAME_SIZE;
        xBusChannelCount = XBUS_RJ01_CHANNEL_COUNT;
        xBusProvider = SERIALRX_XBUS_MODE_B_RJ01;
        break;
    default:
        return false;
        break;
    }

    rxRuntimeConfig->rxRefreshRate = 11000;

    rxRuntimeConfig->rcReadRawFn = xBusReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = xBusFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    serialPort_t *xBusPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        xBusDataReceive,
        baudRate,
        portShared ? MODE_RXTX : MODE_RX,
        SERIAL_NOT_INVERTED | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
        );

#ifdef TELEMETRY
    if (portShared) {
        telemetrySharedPort = xBusPort;
    }
#endif

    return xBusPort != NULL;
}
#endif // USE_SERIALRX_XBUS
