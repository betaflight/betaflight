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

#if defined(USE_RX_CC2500)

#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx_spi_cc2500.h"

PG_REGISTER_WITH_RESET_TEMPLATE(rxCc2500SpiConfig_t, rxCc2500SpiConfig, PG_RX_CC2500_SPI_CONFIG, 2);

#if defined(RX_CC2500_SPI_DISABLE_CHIP_DETECTION)
#define CC2500_SPI_CHIP_DETECTION false
#else
#define CC2500_SPI_CHIP_DETECTION true
#endif

PG_RESET_TEMPLATE(rxCc2500SpiConfig_t, rxCc2500SpiConfig,
    .autoBind = false,
    .bindTxId = {0, 0, 0},
    .bindOffset = 0,
    .bindHopData = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    .rxNum = 0,
    .a1Source = FRSKY_SPI_A1_SOURCE_VBAT,
    .chipDetectEnabled = CC2500_SPI_CHIP_DETECTION,
    .txEnIoTag = IO_TAG(RX_CC2500_SPI_TX_EN_PIN),
    .lnaEnIoTag = IO_TAG(RX_CC2500_SPI_LNA_EN_PIN),
    .antSelIoTag = IO_TAG(RX_CC2500_SPI_ANT_SEL_PIN),
);
#endif
