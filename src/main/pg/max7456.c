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

#include "platform.h"

#ifdef USE_MAX7456

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/io.h"
#include "drivers/bus_spi.h"

#include "max7456.h"

#ifndef MAX7456_CLOCK_CONFIG_DEFAULT
#define MAX7456_CLOCK_CONFIG_DEFAULT    MAX7456_CLOCK_CONFIG_NOMINAL
#endif

#ifndef MAX7456_SPI_CS_PIN
#define MAX7456_SPI_CS_PIN              NONE
#endif

#ifndef MAX7456_SPI_INSTANCE
#define MAX7456_SPI_INSTANCE            NULL
#endif

PG_REGISTER_WITH_RESET_FN(max7456Config_t, max7456Config, PG_MAX7456_CONFIG, 0);

void pgResetFn_max7456Config(max7456Config_t *config)
{
    config->clockConfig = MAX7456_CLOCK_CONFIG_DEFAULT;
    config->csTag = IO_TAG(MAX7456_SPI_CS_PIN);
    config->spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance(MAX7456_SPI_INSTANCE));
    config->preInitOPU = false;
}
#endif // USE_MAX7456
