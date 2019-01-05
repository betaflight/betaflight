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

#include "pg/pg.h"

#include "rx/rx_spi.h"

#define MAX_MISSING_PKT 100
#define RC_CHANNEL_COUNT_SFHSS 8

#define DEBUG_DATA_STATE            0
#define DEBUG_DATA_MISSING_FRAME    1
#define DEBUG_DATA_OFFSET_MAX       2
#define DEBUG_DATA_OFFSET_MIN       3

#define STATE_INIT          0
#define STATE_HUNT          1
#define STATE_SYNC          2
#define STATE_BIND          10
#define STATE_BIND_TUNING1  11
#define STATE_BIND_TUNING2  12
#define STATE_BIND_TUNING3  13
#define STATE_BIND_COMPLETE 14

typedef struct rxSfhssSpiConfig_s {
    uint8_t bindTxId[2];
    int8_t  bindOffset;
} rxSfhssSpiConfig_t;

PG_DECLARE(rxSfhssSpiConfig_t, rxSfhssSpiConfig);

bool sfhssSpiInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeConfig_t *rxRuntimeConfig);
rx_spi_received_e sfhssSpiDataReceived(uint8_t *packet);
void sfhssSpiSetRcData(uint16_t *rcData, const uint8_t *payload);

void sfhssSpiBind(void);
