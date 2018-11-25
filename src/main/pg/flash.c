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
#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "flash.h"

PG_REGISTER_WITH_RESET_FN(flashConfig_t, flashConfig, PG_FLASH_CONFIG, 0);

void pgResetFn_flashConfig(flashConfig_t *flashConfig)
{
    flashConfig->csTag = IO_TAG(FLASH_CS_PIN);
    flashConfig->spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance(FLASH_SPI_INSTANCE));
}
#endif
