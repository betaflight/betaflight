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

#ifdef USE_VTX_RTC6705

#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "vtx_io.h"

PG_REGISTER_WITH_RESET_FN(vtxIOConfig_t, vtxIOConfig, PG_VTX_IO_CONFIG, 0);

void pgResetFn_vtxIOConfig(vtxIOConfig_t *vtxIOConfig)
{
    // common
    vtxIOConfig->csTag = IO_TAG(RTC6705_CS_PIN);
    vtxIOConfig->powerTag = IO_TAG(RTC6705_POWER_PIN);

    // software SPI
    vtxIOConfig->clockTag = IO_TAG(RTC6705_SPICLK_PIN);
    vtxIOConfig->dataTag = IO_TAG(RTC6705_SPI_SDO_PIN);

    // hardware spi
    vtxIOConfig->spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance(RTC6705_SPI_INSTANCE));
}
#endif
