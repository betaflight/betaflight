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

#ifdef USE_FLASH_CHIP

#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/bus_octospi.h"
#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "flash.h"

#ifndef FLASH_CS_PIN
#define FLASH_CS_PIN NONE
#endif

#ifndef FLASH_SPI_INSTANCE
#define FLASH_SPI_INSTANCE NULL
#endif

#ifndef FLASH_QUADSPI_INSTANCE
#define FLASH_QUADSPI_INSTANCE NULL
#endif

PG_REGISTER_WITH_RESET_FN(flashConfig_t, flashConfig, PG_FLASH_CONFIG, 0);

void pgResetFn_flashConfig(flashConfig_t *flashConfig)
{
    // CS pin can be used by all IO interfaces, not just SPI.
    flashConfig->csTag = IO_TAG(FLASH_CS_PIN);

#if defined(USE_FLASH_SPI) && defined(FLASH_SPI_INSTANCE)
    flashConfig->spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance(FLASH_SPI_INSTANCE));
#endif
#if defined(USE_FLASH_QUADSPI) && defined(FLASH_QUADSPI_INSTANCE)
    flashConfig->quadSpiDevice = QUADSPI_DEV_TO_CFG(quadSpiDeviceByInstance((QUADSPI_TypeDef *)FLASH_QUADSPI_INSTANCE));
#endif
#if defined(USE_FLASH_OCTOSPI) && defined(FLASH_OCTOSPI_INSTANCE)
    flashConfig->octoSpiDevice = OCTOSPI_DEV_TO_CFG(octoSpiDeviceByInstance(FLASH_OCTOSPI_INSTANCE));
#endif
}
#endif
