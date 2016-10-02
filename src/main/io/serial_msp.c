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
#include "drivers/system.h"

#include "fc/runtime_config.h"

#include "flight/mixer.h"

#include "io/serial.h"
#include "io/serial_msp.h"

#include "msp/msp.h"


static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];
bufWriter_t *writer;


static void resetMspPort(mspPort_t *mspPortToReset, serialPort_t *serialPort)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));

    mspPortToReset->port = serialPort;
}

void mspSerialAllocatePorts(void)
{
    serialPort_t *serialPort;

    uint8_t portIndex = 0;

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);

    while (portConfig && portIndex < MAX_MSP_PORT_COUNT) {
        mspPort_t *mspPort = &mspPorts[portIndex];
        if (mspPort->port) {
            portIndex++;
            continue;
        }

        serialPort = openSerialPort(portConfig->identifier, FUNCTION_MSP, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        if (serialPort) {
            resetMspPort(mspPort, serialPort);
            portIndex++;
        }

        portConfig = findNextSerialPortConfig(FUNCTION_MSP);
    }
}

void mspSerialReleasePortIfAllocated(serialPort_t *serialPort)
{
    uint8_t portIndex;
    for (portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t *candidateMspPort = &mspPorts[portIndex];
        if (candidateMspPort->port == serialPort) {
            closeSerialPort(serialPort);
            memset(candidateMspPort, 0, sizeof(mspPort_t));
        }
    }
}

void mspSerialInit(void)
{
    mspInit();
    memset(mspPorts, 0, sizeof(mspPorts));
    mspSerialAllocatePorts();
}

/*
 * Process MSP commands from serial ports configured as MSP ports.
 *
 * Called periodically by the scheduler.
 */
void mspSerialProcess(void)
{
    for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {
        mspPort_t * const mspPort = &mspPorts[portIndex];
        if (!mspPort->port) {
            continue;
        }

        // Big enough to fit a MSP_STATUS in one write.
        uint8_t buf[sizeof(bufWriter_t) + 20];
        writer = bufWriterInit(buf, sizeof(buf), (bufWrite_t)serialWriteBufShim, mspPort->port);

        while (serialRxBytesWaiting(mspPort->port)) {

            const uint8_t c = serialRead(mspPort->port);
            const bool consumed = mspProcessReceivedData(mspPort, c);

            if (!consumed && !ARMING_FLAG(ARMED)) {
                evaluateOtherData(mspPort->port, c);
            }

            if (mspPort->c_state == COMMAND_RECEIVED) {
                mspProcessReceivedCommand(mspPort);
                break; // process one command at a time so as not to block.
            }
        }

        bufWriterFlush(writer);

        if (mspPostProcessFn) {
            mspPostProcessFn(mspPort);
            mspPostProcessFn = NULL;
        }
    }
}
