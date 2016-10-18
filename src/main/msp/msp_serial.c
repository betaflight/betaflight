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
#include <string.h>

#include "platform.h"

#include "common/utils.h"

#include "drivers/buf_writer.h"
#include "drivers/serial.h"

#include "fc/runtime_config.h"

#include "io/serial.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"


static mspProcessCommandFnPtr mspProcessCommandFn;
static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];
bufWriter_t *writer;


static void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    mspPortToReset->port = serialPort;
}

void mspSerialAllocatePorts(void)
{
    uint8_t portIndex = 0;
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);
    while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
        mspPort_t *mspPort = &mspPorts[portIndex];
        if (mspPort->port) {
            portIndex++;
            continue;
        }

        serialPort_t *serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        if (serialPort) {
            resetMspPort(mspPort, serialPort);
            portIndex++;
        }

        portConfig = findNextSerialPortConfig(FUNCTION_MSP);
    }
}

void mspSerialReleasePortIfAllocated(serialPort_t *serialPort)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t *candidateMspPort = &mspPorts[portIndex];
        if (candidateMspPort->port == serialPort) {
            closeSerialPort(serialPort);
            memset(candidateMspPort, 0, sizeof(mspPort_t));
        }
    }
}

bool mspSerialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    if (mspPort->c_state == MSP_IDLE) {
        if (c == '$') {
            mspPort->c_state = MSP_HEADER_START;
        } else {
            return false;
        }
    } else if (mspPort->c_state == MSP_HEADER_START) {
        mspPort->c_state = (c == 'M') ? MSP_HEADER_M : MSP_IDLE;
    } else if (mspPort->c_state == MSP_HEADER_M) {
        mspPort->c_state = (c == '<') ? MSP_HEADER_ARROW : MSP_IDLE;
    } else if (mspPort->c_state == MSP_HEADER_ARROW) {
        if (c > MSP_PORT_INBUF_SIZE) {
            mspPort->c_state = MSP_IDLE;

        } else {
            mspPort->dataSize = c;
            mspPort->offset = 0;
            mspPort->checksum = 0;
            mspPort->indRX = 0;
            mspPort->checksum ^= c;
            mspPort->c_state = MSP_HEADER_SIZE;
        }
    } else if (mspPort->c_state == MSP_HEADER_SIZE) {
        mspPort->cmdMSP = c;
        mspPort->checksum ^= c;
        mspPort->c_state = MSP_HEADER_CMD;
    } else if (mspPort->c_state == MSP_HEADER_CMD && mspPort->offset < mspPort->dataSize) {
        mspPort->checksum ^= c;
        mspPort->inBuf[mspPort->offset++] = c;
    } else if (mspPort->c_state == MSP_HEADER_CMD && mspPort->offset >= mspPort->dataSize) {
        if (mspPort->checksum == c) {
            mspPort->c_state = MSP_COMMAND_RECEIVED;
        } else {
            mspPort->c_state = MSP_IDLE;
        }
    }
    return true;
}

static mspPostProcessFnPtr mspSerialProcessReceivedCommand(mspPort_t *mspPort)
{
    mspPostProcessFnPtr mspPostProcessFn = NULL;
    mspProcessCommandFn(mspPort, &mspPostProcessFn);

    mspPort->c_state = MSP_IDLE;
    return mspPostProcessFn;
}

#ifdef USE_DPRINTF
#include "common/printf.h"
#define DPRINTF_SERIAL_PORT SERIAL_PORT_USART3
extern serialPort_t *debugSerialPort;
#define dprintf(x) if (debugSerialPort) printf x
#else
#define dprintf(x)
#endif

void mspSerialPush(int cmd, uint8_t *data, int buflen)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // Big enough for a OSD line
        uint8_t buf[sizeof(bufWriter_t) + 30];

        writer = bufWriterInit(buf, sizeof(buf), (bufWrite_t)serialWriteBufShim, mspPort->port);

        mspServerPush(mspPort, cmd, data, buflen);

        bufWriterFlush(writer);
    }
}

/*
 * Process MSP commands from serial ports configured as MSP ports.
 *
 * Called periodically by the scheduler.
 */
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // Big enough to fit a MSP_STATUS in one write.
        uint8_t buf[sizeof(bufWriter_t) + 20];
        writer = bufWriterInit(buf, sizeof(buf), (bufWrite_t)serialWriteBufShim, mspPort->port);

        mspPostProcessFnPtr mspPostProcessFn = NULL;
        while (serialRxBytesWaiting(mspPort->port)) {

            const uint8_t c = serialRead(mspPort->port);
            const bool consumed = mspSerialProcessReceivedData(mspPort, c);

            if (!consumed && evaluateNonMspData == MSP_EVALUATE_NON_MSP_DATA) {
                serialEvaluateNonMspData(mspPort->port, c);
            }

            if (mspPort->c_state == MSP_COMMAND_RECEIVED) {
                mspPostProcessFn = mspSerialProcessReceivedCommand(mspPort);
                break; // process one command at a time so as not to block.
            }
        }

        bufWriterFlush(writer);

        if (mspPostProcessFn) {
            waitForSerialPortToFinishTransmitting(mspPort->port);
            mspPostProcessFn(mspPort->port);
        }
    }
}

void mspSerialInit(mspProcessCommandFnPtr mspProcessCommandFnToUse)
{
    mspProcessCommandFn = mspProcessCommandFnToUse;
    memset(mspPorts, 0, sizeof(mspPorts));
    mspSerialAllocatePorts();
}
