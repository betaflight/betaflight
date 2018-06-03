/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SPI

#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "pg/bus_spi.h"

// Bring a pin for possible CS line to pull-up state in preparation for
// sequential initialization by relevant drivers.

// There are two versions locally:
// spiPreInitCsIPU set the pin to input with pullup (IOCFG_IPU) for safety.
// spiPreInitCsOPU actually drive the pin for digital hi.
//
// The later is required for SPI slave devices on some targets, interfaced through level shifters, such as Kakute F4.
//
// Two ioTag_t array PGs, spiPreinitIPUConfig and spiPreinitOPUConfig are used to
// determine pin to be IPU or OPU.
// The IPU array is initialized with a hard coded initialization array,
// while the OPU array is initialized from target dependent config.c.
// With generic targets, both arrays are setup with resource commands.

static void spiPreInitCsIPU(ioTag_t iotag, int index)
{
    IO_t io = IOGetByTag(iotag);
    if (io) {
        IOInit(io, OWNER_SPI_PREINIT_IPU, index);
        IOConfigGPIO(io, IOCFG_IPU);
        IOHi(io);
    }
}

static void spiPreInitCsOPU(ioTag_t iotag, int index)
{
    IO_t io = IOGetByTag(iotag);
    if (io) {
        IOInit(io, OWNER_SPI_PREINIT_OPU, index);
        IOConfigGPIO(io, IOCFG_OUT_PP);
        IOHi(io);
    }
}

void spiPreInit(void)
{
    for (int i = 0 ; i < SPI_PREINIT_IPU_COUNT ; i++) {
        if (spiPreinitIPUConfig(i)->csnTag) {
            spiPreInitCsIPU(spiPreinitIPUConfig(i)->csnTag, i);
        }
    }

    for (int i = 0 ; i < SPI_PREINIT_OPU_COUNT ; i++) {
        if (spiPreinitOPUConfig(i)->csnTag) {
            spiPreInitCsOPU(spiPreinitOPUConfig(i)->csnTag, i);
        }
    }
}

// Back to pre-init state

void spiPreinitCsByIO(IO_t io)
{
    for (int i = 0 ; i < SPI_PREINIT_IPU_COUNT ; i++) {
        if (IOGetByTag(spiPreinitIPUConfig(i)->csnTag) == io) {
            spiPreInitCsIPU(spiPreinitIPUConfig(i)->csnTag, i);
            return;
        }
    }

    for (int i = 0 ; i < SPI_PREINIT_OPU_COUNT ; i++) {
        if (IOGetByTag(spiPreinitOPUConfig(i)->csnTag) == io) {
            spiPreInitCsOPU(spiPreinitOPUConfig(i)->csnTag, i);
            return;
        }
    }
}

void spiPreinitCsByTag(ioTag_t iotag)
{
    spiPreinitCsByIO(IOGetByTag(iotag));
}

#endif
