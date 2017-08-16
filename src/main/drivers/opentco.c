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


#include "common/maths.h"
#include "common/crc.h"
#include "opentco.h"

#include <stdbool.h>

static bool opentcoLock        = false;
static serialPort_t *opentcoSerialPort = 0;

static uint8_t opentcoBuffer[OPENTCO_MAX_FRAME_LENGTH];

bool opentcoInit(serialPortFunction_e function)
{

    // find tinyosd serial port
    serialPortConfig_t *portConfig = findSerialPortConfig(function);

    // FIXME:
    // check if portConfig was openend by other opentc device (cam, vtx, or osd)
    // and reuse that port without calling init again!

    if (!portConfig) {
        // could not get port -> abort
        return false;
    }

    // fetch baudrate
    baudRate_e baudRateIndex =  portConfig->blackbox_baudrateIndex;
    uint32_t baudrate = baudRates[baudRateIndex];

    // open assigned serial port (no callback for RX right now)
    opentcoSerialPort = openSerialPort(portConfig->identifier, FUNCTION_NONE, NULL, baudrate, MODE_RXTX, SERIAL_NOT_INVERTED);

    // verify opening
    if (!opentcoSerialPort) {
        return false;
    }

    return true;
}

void opentcoInitializeFrame(sbuf_t *dst, uint8_t device, uint8_t command)
{
    // point to the buffer
    dst->ptr = opentcoBuffer;
    dst->end = ARRAYEND(opentcoBuffer);

    // add frame header
    sbufWriteU8(dst, OPENTCO_PROTOCOL_HEADER);

    // add device & command
    sbufWriteU8(dst, ((device & 0x0F)<<4) | (command & 0x0F));
}

static void opentcoAppendCRC(sbuf_t *dst)
{
    uint8_t crc = 0;
    uint8_t *end = sbufPtr(dst);
    uint8_t *start = opentcoBuffer;

    // iterate over all elements and calculate crc
    for (uint8_t *ptr = start; ptr < end; ++ptr) {
        crc = crc8_dvb_s2(crc, *ptr);
    }

    // append data to stream
    sbufWriteU8(dst, crc);
}

void opentcoSendFrame(sbuf_t *dst)
{
    // add crc over (all) data
    opentcoAppendCRC(dst);

    // switch to reader
    sbufSwitchToReader(dst, opentcoBuffer);

    // send data if possible
    if (!opentcoLock) {
        opentcoLock = true;
        //opentcoBuffer[1] = sbufBytesRemaining(dst);
        serialWriteBuf(opentcoSerialPort, sbufPtr(dst), sbufBytesRemaining(dst));
        opentcoLock = false;
    }
}


