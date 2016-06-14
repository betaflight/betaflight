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
//#include <math.h>

#include "build_config.h"

#include "debug.h"

#include <platform.h>

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/streambuf.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/buf_writer.h"

#include "io/serial.h"
#include "osd/fc_state.h"

#include "msp/msp_protocol.h"
#include "msp/msp.h"
#include "msp/msp_serial.h"

#include "osd/msp_client_osd.h"

/*
    This is a simple first-cut implementation of an MSP client so that the OSD can talk to the FC.

    Later on, we'll configure the frequency of each command.
*/

#define MSP_CLIENT_TIMEOUT_INTERVAL (500 * 1000) // 1/2 second

uint8_t commandToSend;

bool mspRequestFCSimpleCommandSender(mspPacket_t *request)
{
    request->cmd = commandToSend;

    return true;
}

static uint8_t commandsToSend[] = {
    MSP_STATUS,
    MSP_ANALOG,
    MSP_MOTOR
};

mspClientStatus_t mspClientStatus;

void mspClientProcess(void)
{
    static uint8_t index = 0;

    bool busy = mspPorts[1].commandSenderFn != NULL;
    if (busy) {
        return;
    }
    commandToSend = commandsToSend[index];
    mspPorts[1].commandSenderFn = mspRequestFCSimpleCommandSender;

    index++;
    if (index >= ARRAYLEN(commandsToSend)) {
        index = 0;
    }


    //
    // handle timeout of received data.
    //
    uint32_t now = micros();
    mspClientStatus.timeoutOccured = (cmp32(now, mspClientStatus.lastReplyAt) >= MSP_CLIENT_TIMEOUT_INTERVAL);
}

// return positive for ACK, negative on error, zero for no reply
int mspClientProcessInCommand(mspPacket_t *cmd)
{
    sbuf_t * src = &cmd->buf;
    int len = sbufBytesRemaining(src);

    mspClientStatus.lastReplyAt = micros();

    UNUSED(len);

    switch (cmd->cmd) {
        case MSP_STATUS:
            fcStatus.cycleTime = sbufReadU16(src);
            fcStatus.i2cErrors = sbufReadU16(src);
            fcStatus.sensors = sbufReadU16(src);
            fcStatus.fcState = sbufReadU32(src);
            fcStatus.profile = sbufReadU8(src);
        break;

        case MSP_ANALOG:
            fcStatus.vbat = sbufReadU8(src);
            fcStatus.mAhDrawn = sbufReadU16(src);
            fcStatus.rssi = scaleRange(sbufReadU16(src), 0, 1023, 0, 1000);
            fcStatus.amperage = sbufReadU16(src);
            break;

        case MSP_MOTOR:
            for (unsigned i = 0; i < 8 && i < OSD_MAX_MOTORS; i++) {
                fcMotors[i] = sbufReadU16(src);
            }
        break;

        default:
            // we do not know how to handle the (valid) message, try another message handler
            return 0;
    }
    return 1;     // message was handled succesfully
}


