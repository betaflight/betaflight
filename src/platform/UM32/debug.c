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

EX_CODE void debugInit(void)
{
    IO_t io = IOGetByTag(DEFIO_TAG_E(PA13)); // SWDIO
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PA14));      // SWCLK
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }

    //system pin for QSPI internal used.
    io = IOGetByTag(DEFIO_TAG_E(PE10));      // QSPI_CLK
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SYSTEM, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PD3));      // QSPI_BK1CS
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SYSTEM, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PD4));      // QSPI_BK1IO0
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SYSTEM, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PD5));      // QSPI_BK1IO1
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SYSTEM, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PD6));      // QSPI_BK1IO2
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SYSTEM, 0);
    }
    io = IOGetByTag(DEFIO_TAG_E(PE15));      // QSPI_BK1IO3
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SYSTEM, 0);
    }

    //System PIN for internal resource.
    if(isDshotBitbangActive(&motorConfig()->dev)){
        io = IOGetByTag(DEFIO_TAG_E(PE5));
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);

        io = IOGetByTag(DEFIO_TAG_E(PE7));
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);

        io = IOGetByTag(DEFIO_TAG_E(PD8));
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    } else {
        io = IOGetByTag(DEFIO_TAG_E(PC8));
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);

        io = IOGetByTag(DEFIO_TAG_E(PC9));
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);

        io = IOGetByTag(DEFIO_TAG_E(PA9));
        IOInit(io,  OWNER_SYSTEM,  0);
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    }
}
