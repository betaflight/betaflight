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

#include "platform.h"

#ifdef USE_FLASHFS

#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "flash.h"

PG_REGISTER_WITH_RESET_FN(flashConfig_t, flashConfig, PG_FLASH_CONFIG, 0);

void pgResetFn_flashConfig(flashConfig_t *flashConfig)
{
#ifdef M25P16_CS_PIN
    flashConfig->csTag = IO_TAG(M25P16_CS_PIN);
#else
    flashConfig->csTag = IO_TAG_NONE;
#endif
    flashConfig->spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance(M25P16_SPI_INSTANCE));
}
#endif
