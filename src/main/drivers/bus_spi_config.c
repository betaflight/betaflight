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

#include "drivers/bus_spi.h"
#include "drivers/io.h"

// Bring a pin for possible CS line to pull-up state in preparation for
// sequential initialization by relevant drivers.
// Note that the pin is set to input for safety at this point.

void spiPreInitCs(ioTag_t iotag)
{
    IO_t io = IOGetByTag(iotag);
    if (io) {
        IOInit(io, OWNER_SPI_PREINIT, 0);
        IOConfigGPIO(io, IOCFG_IPU);
    }
}
