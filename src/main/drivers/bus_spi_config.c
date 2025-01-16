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

void spiPreinit(void)
{
#ifdef USE_SDCARD_SPI
    sdcard_preinit(sdcardConfig());
#endif

#if defined(RTC6705_CS_PIN) && !defined(USE_VTX_RTC6705_SOFTSPI) // RTC6705 soft SPI initialisation handled elsewhere.
    // XXX Waiting for "RTC6705 cleanup #7114" to be done
#endif

#ifdef USE_FLASH_CHIP
    flashPreinit(flashConfig());
#endif

#if defined(USE_RX_SPI)
    rxSpiDevicePreinit(rxSpiConfig());
#endif

#ifdef USE_MAX7456
    max7456Preinit(max7456Config());
#endif

    ioPreinit(PREINIT_OWNER_SPI);
}

#endif
