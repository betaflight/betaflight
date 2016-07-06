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
#include <string.h>
#include <math.h>

#include "build/build_config.h"
#include "build/debug.h"
#include <platform.h>
#include "scheduler/scheduler.h"

#include "common/axis.h"
#include "common/utils.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "fc/runtime_config.h"
#include "fc/config.h"

#include "msp/msp.h"

// handle received command, possibly generate reply.
// return nonzero when reply was generated (including reported error)
int mspProcessCommand(mspPacket_t *command, mspPacket_t *reply)
{
    // initialize reply by default
    reply->cmd = command->cmd;

    int status = mspServerCommandHandler(command, reply);
    reply->result = status;

    return status;
}

#ifdef USE_MSP_CLIENT
void mspProcessReply(mspPacket_t *reply)
{
	int status = mspClientReplyHandler(reply);
	UNUSED(status);
}
#endif
