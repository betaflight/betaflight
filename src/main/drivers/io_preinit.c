/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/resource.h"
#include "drivers/system.h"

typedef struct ioPreinit_s {
    ioTag_t iotag;
    uint8_t iocfg;
    bool init;
    ioPreinitOwner_e owner;
} ioPreinit_t;

static ioPreinit_t ioPreinitArray[IO_PREINIT_COUNT];
static int ioPreinitCount = 0;

static void ioPreinitPin(ioPreinit_t *preinit, int index)
{
    IO_t io = IOGetByTag(preinit->iotag);
    IOInit(io, OWNER_PREINIT, RESOURCE_INDEX(index));
    IOConfigGPIO(io, preinit->iocfg);
    if (preinit->init) {
        IOHi(io);
    } else {
        IOLo(io);
    }
}

void ioPreinitRegister(ioTag_t iotag, uint8_t iocfg, bool init, ioPreinitOwner_e owner)
{
    if (!iotag) {
        return;
    }

    if (ioPreinitCount == IO_PREINIT_COUNT) {
        indicateFailure(FAILURE_DEVELOPER, 5);
        return;
    }

    ioPreinitArray[ioPreinitCount].iotag = iotag;
    ioPreinitArray[ioPreinitCount].iocfg = iocfg;
    ioPreinitArray[ioPreinitCount].init = init;
    ioPreinitArray[ioPreinitCount].owner = owner;
    ++ioPreinitCount;
}

void ioPreinit(ioPreinitOwner_e owner)
{
    for (int i = 0; i < ioPreinitCount; i++) {
        if (owner != PREINIT_OWNER_ALL && ioPreinitArray[i].owner != owner) {
            continue;
        }
        ioPreinitPin(&ioPreinitArray[i], i);
    }
}

void ioPreinitByIO(const IO_t io)
{
    for (int i = 0; i < ioPreinitCount; i++) {
        if (io == IOGetByTag(ioPreinitArray[i].iotag)) {
            ioPreinitPin(&ioPreinitArray[i], i);
            return;
        }
    }
}

void ioPreinitByTag(ioTag_t tag)
{
    ioPreinitByIO(IOGetByTag(tag));
}
