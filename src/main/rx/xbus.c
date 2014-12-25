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

#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "rx/xbus.h"

//
// Serial driver for JR's XBus (MODE B) receiver
//

#define XBUS_CHANNEL_COUNT 12

// Frame is: ID(1 byte) + 12*channel(2 bytes) + CRC(2 bytes) = 27
#define XBUS_FRAME_SIZE 27
#define XBUS_CRC_BYTE_1 25
#define XBUS_CRC_BYTE_2 26

#define XBUS_CRC_AND_VALUE 0x8000
#define XBUS_CRC_POLY 0x1021

#define XBUS_BAUDRATE 115200
#define XBUS_MAX_FRAME_TIME 5000

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
#define XBUS_CONVERT_TO_USEC(V)	(800 + ((V * 1400) >> 12))

static bool xBusFrameReceived = false;
static bool xBusDataIncoming = false;
static uint8_t xBusFramePosition;

static volatile uint8_t xBusFrame[XBUS_FRAME_SIZE];
static uint16_t xBusChannelData[XBUS_CHANNEL_COUNT];

static void xBusDataReceive(uint16_t c);
static uint16_t xBusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

static serialPort_t *xBusPort;

void xBusUpdateSerialRxFunctionConstraint(functionConstraint_t *functionConstraint)
{
    functionConstraint->minBaudRate = XBUS_BAUDRATE;
    functionConstraint->maxBaudRate = XBUS_BAUDRATE;
    functionConstraint->requiredSerialPortFeatures = SPF_SUPPORTS_CALLBACK;
}

bool xBusInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    switch (rxConfig->serialrx_provider) {
        case SERIALRX_XBUS_MODE_B:
            rxRuntimeConfig->channelCount = XBUS_CHANNEL_COUNT;
            xBusFrameReceived = false;
            xBusDataIncoming = false;
            xBusFramePosition = 0;
            break;
        default:
            return false;
            break;
    }

    xBusPort = openSerialPort(FUNCTION_SERIAL_RX, xBusDataReceive, XBUS_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED);
    if (callback) {
        *callback = xBusReadRawRC;
    }

    return xBusPort != NULL;
}

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

static void xBusUnpackFrame(void)
{
    // Calculate the CRC of the incoming frame
    uint16_t crc = 0;
    uint16_t inCrc = 0;
    uint8_t i = 0;
    uint16_t value;
    uint8_t frameAddr;

    // Calculate on all bytes except the final two CRC bytes
    for (i = 0; i < XBUS_FRAME_SIZE - 2; i++) {
        inCrc = xBusCRC16(inCrc, xBusFrame[i]);
    }

    // Get the received CRC
    crc = ((uint16_t)xBusFrame[XBUS_CRC_BYTE_1]) << 8;
    crc = crc + ((uint16_t)xBusFrame[XBUS_CRC_BYTE_2]);

    if (crc == inCrc) {
        // Unpack the data, we have a valid frame
        for (i = 0; i < XBUS_CHANNEL_COUNT; i++) {

            frameAddr = 1 + i * 2;
            value = ((uint16_t)xBusFrame[frameAddr]) << 8;
            value = value + ((uint16_t)xBusFrame[frameAddr + 1]);

            // Convert to internal format
            xBusChannelData[i] = XBUS_CONVERT_TO_USEC(value);
        }

        xBusFrameReceived = true;
    }

}

// Receive ISR callback
static void xBusDataReceive(uint16_t c)
{
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
    if (xBusFramePosition == XBUS_FRAME_SIZE) {
        xBusUnpackFrame();
        xBusDataIncoming = false;
        xBusFramePosition = 0;
    }
}

// Indicate time to read a frame from the data...
bool xBusFrameComplete(void)
{
    return xBusFrameReceived;
}

static uint16_t xBusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    uint16_t data;

    // Mark frame as read
    if (xBusFrameReceived) {
        xBusFrameReceived = false;
    }

    // Deliver the data wanted
    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    data = xBusChannelData[chan];

    return data;
}
