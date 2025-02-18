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

static unsigned preinitIndex = 0;

void ioPreinitByIO(const IO_t io, uint8_t iocfg, ioPreinitPinState_e init)
{
    if (!io) {
        return;
    }
    preinitIndex++;

    IOInit(io, OWNER_PREINIT, preinitIndex);
    IOConfigGPIO(io, iocfg);

    switch(init) {
    case PREINIT_PIN_STATE_LOW:
        IOLo(io);
        break;
    case PREINIT_PIN_STATE_HIGH:
        IOHi(io);
        break;
    default:
        break; // Do nothing
    }
}

void ioPreinitByTag(ioTag_t tag, uint8_t iocfg, ioPreinitPinState_e init)
{
    ioPreinitByIO(IOGetByTag(tag), iocfg, init);
}
