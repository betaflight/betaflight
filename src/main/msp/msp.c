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

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/feature.h"
#include "config/profile.h"

#include "fc/runtime_config.h"
#include "fc/config.h"

#include "msp/msp.h"

// criteria is passed by value; cast as (void *)
// TODO - the search is quadratic, the code should first map mspId to pgN, then search for pgN
uint8_t pgMatcherForMSPSet(const pgRegistry_t *candidate, const void *criteria)
{
    int mspIdForSet = (intptr_t)criteria;

    for (unsigned i = 0; i < pgToMSPMapSize; i++) {
        const pgToMSPMapEntry_t *entry = &pgToMSPMap[i];
        if (entry->pgn == pgN(candidate) && entry->mspIdForSet == mspIdForSet) {
            return true;
        }
    }
    return false;
}

// criteria is passed by value; cast as (void *)
uint8_t pgMatcherForMSP(const pgRegistry_t *candidate, const void *criteria)
{
    int mspId = (intptr_t)criteria;

    for (unsigned i = 0; i < pgToMSPMapSize; i++) {
        const pgToMSPMapEntry_t *entry = &pgToMSPMap[i];
        if (entry->pgn == pgN(candidate) && entry->mspId == mspId) {
            return true;
        }
    }
    return false;
}

// process commands that match registered parameter_group
int mspServerProcessPgCommand(mspPacket_t *command, mspPacket_t *reply)
{
    sbuf_t *src = &command->buf;
    sbuf_t *dst = &reply->buf;
    int cmdLength = sbufBytesRemaining(src);

    // MSP IN - read config structure
    {
        const pgRegistry_t *reg = pgMatcher(pgMatcherForMSP, (void*)(intptr_t)command->cmd);
        if (reg) {
            // this works for system and profile settings as
            //  the profile index will be ignored by pgLoad if the reg is a system registration.
            int stored = pgStore(reg, sbufPtr(dst), sbufBytesRemaining(dst), getCurrentProfile());
            if(stored < 0)
                return stored;
            sbufAdvance(dst, stored);       // commit saved data
            return 1;
        }
    }
    // MSP OUT - write config structure
    {
        const pgRegistry_t *reg = pgMatcher(pgMatcherForMSPSet, (void*)(intptr_t)command->cmd);
        if (reg != NULL) {
            // this works for system and profile settings as
            //  the profile index will be ignored by pgLoad if the reg is a system registration.
            pgLoad(reg, sbufPtr(src), cmdLength, getCurrentProfile());
            sbufAdvance(src, cmdLength);    // consume data explicitly
            return 1;
        }
    }
    return 0;
}

// handle received command, possibly generate reply.
// return nonzero when reply was generated (including reported error)
int mspServerProcessCommand(mspPacket_t *command, mspPacket_t *reply)
{
    // initialize reply by default
    reply->cmd = command->cmd;
    int status;
    do {
        if((status = mspServerProcessInCommand(command)) != 0)
            break;
        if((status = mspServerProcessOutCommand(command, reply)) != 0)
            break;
        if((status = mspServerProcessPgCommand(command, reply)) != 0)
            break;
        // command was not handled, return error
        status = -1;
    } while(0);
    reply->result = status;
    return status;
}

#ifdef USE_MSP_CLIENT
// handle received command
int mspClientProcessCommand(mspPacket_t *command, mspPacket_t *reply)
{
    // initialize reply by default
    reply->cmd = command->cmd;
    int status;
    do {
        if((status = mspClientProcessInCommand(command)) != 0)
            break;
        // command was not handled, return error
        status = -1;
    } while(0);
    reply->result = status;
    return status;
}
#endif
