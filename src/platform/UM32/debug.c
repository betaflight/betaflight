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

#include "platform.h"
#include "build/debug.h"
#include "drivers/io.h"
#include "drivers/dshot.h"

// No EX_CODE marker: this whole file is routed to QSPI XIP via the
// file-level EX_FLASH_SRC list in mk/UM324xx.mk (object matched by the
// .ex_flash section in link/um32_flash_4xf.ld).
void debugInit(void)
{
    IO_t io = IOGetByTag(DEFIO_TAG_E(PA13)); // SWDIO
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PA14));      // SWCLK
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }

    //System PIN for internal resource.
    io = IOGetByTag(DEFIO_TAG_E(PE5));
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    }

    io = IOGetByTag(DEFIO_TAG_E(PE7));
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    }

    io = IOGetByTag(DEFIO_TAG_E(PD8));
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    }

#ifndef USE_SDCARD_SDIO
    io = IOGetByTag(DEFIO_TAG_E(PC8));
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    }

    io = IOGetByTag(DEFIO_TAG_E(PC9));
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    }
#endif

    io = IOGetByTag(DEFIO_TAG_E(PA9));
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    }
}
