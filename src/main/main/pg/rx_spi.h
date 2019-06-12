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

#pragma once

#include "drivers/io_types.h"

#include "pg/pg.h"

typedef struct rxSpiConfig_s {
    // RX protocol
    uint8_t rx_spi_protocol;                // type of SPI RX protocol
                                            // nrf24: 0 = v202 250kbps. (Must be enabled by FEATURE_RX_NRF24 first.)
    uint32_t rx_spi_id;
    uint8_t rx_spi_rf_channel_count;

    // SPI Bus
    ioTag_t csnTag;
    uint8_t spibus;

    ioTag_t bindIoTag;
    ioTag_t ledIoTag;
    uint8_t ledInversion;

    ioTag_t extiIoTag;
} rxSpiConfig_t;

PG_DECLARE(rxSpiConfig_t, rxSpiConfig);
