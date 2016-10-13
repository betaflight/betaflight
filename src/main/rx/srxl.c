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
#include "build/build_config.h"

#include "config/parameter_group.h"

#include "drivers/system.h"

#include "drivers/serial.h"
#include "drivers/dma.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "rx/srxl.h"

#include "common/crc.h"



#define SRXL_CHANNEL_COUNT_A1 12
#define SRXL_CHANNEL_COUNT_A2 16

#define SRXL_CHANNEL_COUNT_MAX SRXL_CHANNEL_COUNT_A2



// Frame is: ID(1 byte) + 12*channel(2 bytes) + CRC(2 bytes) = 27
#define SRXL_FRAME_SIZE_A1 (SRXL_CHANNEL_COUNT_A1*2 + 3)
// Frame is: ID(1 byte) + 16*channel(2 bytes) + CRC(2 bytes) = 35
#define SRXL_FRAME_SIZE_A2 (SRXL_CHANNEL_COUNT_A2*2 + 3)

#define SRXL_BAUDRATE 115200
#define SRXL_MAX_FRAME_TIME 8000

// NOTE!
// This is actually based on ID+LENGTH (nibble each)
// 0xA - Multiplex ID (also used by JR, no idea why)
// 0x1 - 12 channels
// 0x2 - 16 channels
// Add identifier here if another manufacturer uses another ID

#define SRXL_START_OF_FRAME_BYTE_A1 (0xA1)		//12 channels
#define SRXL_START_OF_FRAME_BYTE_A2 (0xA2)		//16 channels transfer

// Pulse length convertion from [0...4095] to µs:
//      800µs  -> 0x000
//      1500µs -> 0x800
//      2200µs -> 0xFFF
// Total range is: 2200 - 800 = 1400 <==> 4095
// Use formula: 800 + value * 1400 / 4096 (i.e. a shift by 12)
#define SRXL_CONVERT_TO_USEC(V)	(800 + ((V * 1400) >> 12))

static bool srxlFrameReceived = false;
static bool srxlDataIncoming = false;
static uint8_t srxlFramePosition;
static uint8_t srxlFrameLength;
static uint8_t srxlChannelCount;

static void srxlDataReceive(uint16_t c);

// Use max values for ram areas
static volatile uint8_t srxlFrame[SRXL_FRAME_SIZE_A2];	//size 35 for 16 channels in SRXL 0xA2
static uint16_t srxlChannelData[SRXL_CHANNEL_COUNT_MAX];
static uint16_t srxlReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);

bool srxlInit(rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    uint32_t baudRate;
    rxRuntimeConfig->channelCount = SRXL_CHANNEL_COUNT_MAX;
    srxlFrameReceived = false;
    srxlDataIncoming = false;
    srxlFramePosition = 0;
    baudRate = SRXL_BAUDRATE;
    srxlFrameLength = SRXL_FRAME_SIZE_A2;		//default for 12 channel
    srxlChannelCount = SRXL_CHANNEL_COUNT_MAX;

    if (callback) {
        *callback = srxlReadRawRC;
    }

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    serialPort_t *srxlPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, srxlDataReceive, baudRate, MODE_RX, SERIAL_NOT_INVERTED);

    return srxlPort != NULL;
}


// Receive ISR callback
static void srxlDataReceive(uint16_t c)
{
    uint32_t now;
    static uint32_t srxlTimeLast, srxlTimeInterval;

    // Check if we shall reset frame position due to time
    now = micros();
    srxlTimeInterval = now - srxlTimeLast;
    srxlTimeLast = now;
    if (srxlTimeInterval > SRXL_MAX_FRAME_TIME) {
        srxlFramePosition = 0;
        srxlDataIncoming = false;
    }

    // Check if we shall start a frame?
    if (srxlFramePosition == 0)	{
        if (c == SRXL_START_OF_FRAME_BYTE_A1) {
    		    srxlDataIncoming = true;
    		    srxlFrameLength = SRXL_FRAME_SIZE_A1;	//decrease framesize (when receiver change, otherwise board must reboot)
            srxlChannelCount = SRXL_CHANNEL_COUNT_A1;
        }
        else if (c == SRXL_START_OF_FRAME_BYTE_A2) {//16channel packet
    		    srxlDataIncoming = true;
    		    srxlFrameLength = SRXL_FRAME_SIZE_A2;	//increase framesize
			      srxlChannelCount = SRXL_CHANNEL_COUNT_A2;
        }
    }

    // Only do this if we are receiving to a frame
    if (srxlDataIncoming == true) {
        // Store in frame copy
        srxlFrame[srxlFramePosition] = (uint8_t)c;
        srxlFramePosition++;
    }

    // Done?
    if (srxlFramePosition == srxlFrameLength) {
        srxlFrameReceived = true;
        srxlDataIncoming = false;
        srxlFramePosition = 0;
    }
}

// Indicate time to read a frame from the data...
uint8_t srxlFrameStatus(void)
{
    uint8_t i = 0;
    uint16_t value;
    uint8_t frameAddr;
    uint16_t crc_calc = 0;

    if (!srxlFrameReceived) {
        return SERIAL_RX_FRAME_PENDING;
    }

    srxlFrameReceived = false;
	   //is CRC OK
    for (i = 0; i < srxlFrameLength; i++) {
        crc_calc =  crc16_CCITT(crc_calc, srxlFrame[i]);
        }

     //crc_calc is 0 if crc check is OK
     if(crc_calc == 0) {
		      // save data
        for (i = 0; i < srxlChannelCount; i++) {
            frameAddr = 1 + i * 2;
            value = ((uint16_t)srxlFrame[frameAddr]) << 8;
            value = value + ((uint16_t)srxlFrame[frameAddr + 1]);

            // Convert to internal format
            srxlChannelData[i] = SRXL_CONVERT_TO_USEC(value);
        }
        return SERIAL_RX_FRAME_COMPLETE;
    }
    return SERIAL_RX_FRAME_PENDING;
}

static uint16_t srxlReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    return srxlChannelData[chan];
}
