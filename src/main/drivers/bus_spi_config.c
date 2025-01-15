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
#include "drivers/resource.h"
#include "drivers/system.h"

#include "drivers/flash/flash.h"
#include "drivers/max7456.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/sdcard.h"

#include "pg/flash.h"
#include "pg/max7456.h"
#include "pg/rx_spi.h"
#include "pg/sdcard.h"

typedef struct spiPreinit_s {
    ioTag_t iotag;
    uint8_t iocfg;
    bool init;
} spiPreinit_t;

static spiPreinit_t spiPreinitArray[SPI_PREINIT_COUNT];
static int spiPreinitCount = 0;

void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, bool init)
{
    if (!iotag) {
        return;
    }

    if (spiPreinitCount == SPI_PREINIT_COUNT) {
        indicateFailure(FAILURE_DEVELOPER, 5);
        return;
    }

    spiPreinitArray[spiPreinitCount].iotag = iotag;
    spiPreinitArray[spiPreinitCount].iocfg = iocfg;
    spiPreinitArray[spiPreinitCount].init = init;
    ++spiPreinitCount;
}

static void spiPreinitPin(spiPreinit_t *preinit, int index)
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

void spiPreinit(void)
{
#ifdef USE_SDCARD_SPI
    sdcard_preInit(sdcardConfig());
#endif

#if defined(RTC6705_CS_PIN) && !defined(USE_VTX_RTC6705_SOFTSPI) // RTC6705 soft SPI initialisation handled elsewhere.
    // XXX Waiting for "RTC6705 cleanup #7114" to be done
#endif

#ifdef USE_FLASH_CHIP
    flashPreInit(flashConfig());
#endif

#if defined(USE_RX_SPI)
    rxSpiDevicePreInit(rxSpiConfig());
#endif

#ifdef USE_MAX7456
    max7456PreInit(max7456Config());
#endif

    for (int i = 0; i < spiPreinitCount; i++) {
        spiPreinitPin(&spiPreinitArray[i], i);
    }
}

void spiPreinitByIO(const IO_t io)
{
    for (int i = 0; i < spiPreinitCount; i++) {
        if (io == IOGetByTag(spiPreinitArray[i].iotag)) {
            spiPreinitPin(&spiPreinitArray[i], i);
            return;
        }
    }
}

void spiPreinitByTag(ioTag_t tag)
{
    spiPreinitByIO(IOGetByTag(tag));
}
#endif
