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

#include <platform.h>

#ifdef USE_SPI

#include "drivers/bus_spi.h"
#include "drivers/io.h"

// Bring a pin for possible CS line to pull-up state in preparation for
// sequential initialization by relevant drivers.

// There are two versions:
// spiPreInitCs set the pin to input with pullup (IOCFG_IPU) for safety at this point.
// spiPreInitCsOutPU which actually drive the pin for digital hi.
//
// The later is required for SPI slave devices on some targets, interfaced through level shifters, such as Kakute F4.
// Note that with this handling, a pin declared as CS pin for MAX7456 needs special care when re-purposing the pin for other, especially, input uses.
// This will/should be fixed when we go fully reconfigurable.

void spiPreInitCs(ioTag_t iotag)
{
    IO_t io = IOGetByTag(iotag);
    if (io) {
        IOInit(io, OWNER_SPI_PREINIT, 0);
        IOConfigGPIO(io, IOCFG_IPU);
    }
}

void spiPreInitCsOutPU(ioTag_t iotag)
{
    IO_t io = IOGetByTag(iotag);
    if (io) {
        IOInit(io, OWNER_SPI_PREINIT, 0);
        IOConfigGPIO(io, IOCFG_OUT_PP);
        IOHi(io);
    }
}
#endif
