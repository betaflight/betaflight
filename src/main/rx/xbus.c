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

#include <platform.h>

#include "config/parameter_group.h"

#include "drivers/dma.h"
#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "rx/xbus.h"

#include "common/crc.h"

#define XBUS_RJ01_CHANNEL_COUNT 12
// Frame is: ID(1 byte) + 12*channel(2 bytes) + CRC(2 bytes) = 27
#define XBUS_FRAME_SIZE 27
#define XBUS_MAX_FRAME_SIZE 33

#define XBUS_RJ01_OFFSET_BYTES 3

#define XBUS_MESSAGE_LENGTH_POSITION 1
#define XBUS_BAUDRATE 115200
#define XBUS_RJ01_BAUDRATE 250000
#define XBUS_MAX_FRAME_TIME 8000

#define XBUS_START_OF_FRAME_BYTE (0xA1)

// Pulse length convertion from [0...4095] to µs:
//      800µs  -> 0x000
//      1500µs -> 0x800
//      2200µs -> 0xFFF
// Total range is: 2200 - 800 = 1400 <==> 4095
// Use formula: 800 + value * 1400 / 4096 (i.e. a shift by 12)
#define XBUS_CONVERT_TO_USEC(V)	(800 + ((V * 1400) >> 12))

static bool xBusFrameReceived = false;
static bool xBusDataIncoming = false;
static uint8_t xBusFramePosition;
static uint8_t xBusFrameLength;
static uint8_t xBusChannelCount;


// Use max values for ram areas
static volatile uint8_t xBusFrame[XBUS_MAX_FRAME_SIZE];
static uint16_t xBusChannelData[XBUS_RJ01_CHANNEL_COUNT];

static void xBusDataReceive(uint16_t c);
static uint16_t xBusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

bool xBusInit(rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    uint32_t baudRate;


    rxRuntimeConfig->channelCount = XBUS_RJ01_CHANNEL_COUNT;
    xBusFrameReceived = false;
    xBusDataIncoming = false;
    xBusFramePosition = 0;
    baudRate = XBUS_RJ01_BAUDRATE;
    xBusChannelCount = XBUS_RJ01_CHANNEL_COUNT;
    
    

    if (callback) {
        *callback = xBusReadRawRC;
    }

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    serialPort_t *xBusPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, xBusDataReceive, baudRate, MODE_RX, SERIAL_NOT_INVERTED);

    return xBusPort != NULL;
}

// Full Xbus message CRC8 calculations
uint8_t xBusCRC8(uint8_t inData, uint8_t seed)
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
    uint8_t i = 0;
    uint16_t value;
    uint8_t frameAddr;

    // crc should be 0, if we have no biterrors
    for (i = 0; i < XBUS_FRAME_SIZE; i++) {
        crc = crc16_CCITT(crc, xBusFrame[i+offsetBytes]);
    }


    if (crc == 0) {
        // Unpack the data, we have a valid frame, only 12 channel unpack also when receive 16 channel
        for (i = 0; i < xBusChannelCount; i++) {

            frameAddr = offsetBytes + 1 + i * 2;
            value = ((uint16_t)xBusFrame[frameAddr]) << 8;
            value = value + ((uint16_t)xBusFrame[frameAddr + 1]);

            // Convert to internal format
            xBusChannelData[i] = XBUS_CONVERT_TO_USEC(value);
        }

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
    // Note that the unknown number of bytes differ between receivers,
    // but the "CRC_OUTER" is always the last byte, and the MODE B frame
    // always starts 3 bytes into the frame
    // Compared to a standard MODE B frame that only
    // contains the "middle" package.
    // Hence, at the moment, the unknown header and footer
    // of the RJ01 MODEB packages are discarded.
    // However, the LAST byte (CRC_OUTER) is infact an 8-bit
    // CRC for the whole package, using the Dallas-One-Wire CRC
    // method.

    // CRC calculation & check for full message (CRC_OUTER)
    //
    for (i = 0; i < xBusFrameLength - 1; i++) {
        outerCrc = xBusCRC8(outerCrc, xBusFrame[i]);
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
    uint32_t now;
    static uint32_t xBusTimeLast, xBusTimeInterval;

    // Check if we shall reset frame position due to time
    now = micros();
    xBusTimeInterval = now - xBusTimeLast;
    xBusTimeLast = now;
    if (xBusTimeInterval > XBUS_MAX_FRAME_TIME) {
        xBusFramePosition = 0;
        xBusDataIncoming = false;
    }

    // Too long message?
    if (xBusFramePosition >= XBUS_MAX_FRAME_SIZE) {
        // Something wrong...stop in order to avoid
        // overwriting the buffer
        xBusFramePosition = 0;
        xBusDataIncoming = false;
    }

    // Check if we shall start a frame?
    if ((xBusFramePosition == 0) && (c == XBUS_START_OF_FRAME_BYTE)) {
        xBusDataIncoming = true;
    }

    // Only do this if we are receiving to a frame
    if (xBusDataIncoming == true) {
        // Store in frame buffer
        xBusFrame[xBusFramePosition] = (uint8_t)c;

        // Figure out the FRAME LENGTH (it might vary depending on receiver)
        // Note: This is only in XBUS_RJ01 mode.
        if (xBusFramePosition == XBUS_MESSAGE_LENGTH_POSITION) {
            xBusFrameLength = xBusFrame[xBusFramePosition]
                              + XBUS_RJ01_OFFSET_BYTES;
        }

        xBusFramePosition++;
    }

    // Done?
    if (xBusFramePosition == xBusFrameLength) {
        xBusFrameReceived = true;
    }
}

// Indicate time to read a frame from the data...
uint8_t xBusFrameStatus(void)
{
    if (!xBusFrameReceived) {
        return SERIAL_RX_FRAME_PENDING;
    }

    xBusUnpackRJ01Frame();

    xBusDataIncoming = false;
    xBusFramePosition = 0;

    xBusFrameReceived = false;

    return SERIAL_RX_FRAME_COMPLETE;
}

static uint16_t xBusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    uint16_t data;

    // Deliver the data wanted
    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    data = xBusChannelData[chan];

    return data;
}
