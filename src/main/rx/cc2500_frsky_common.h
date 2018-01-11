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

#pragma once

#include "pg/pg.h"

#include "rx/rx_spi.h"

typedef struct rxFrSkySpiConfig_s {
    bool autoBind;
    uint8_t bindTxId[2];
    int8_t  bindOffset;
    uint8_t bindHopData[50];
    uint8_t rxNum;
} rxFrSkySpiConfig_t;

PG_DECLARE(rxFrSkySpiConfig_t, rxFrSkySpiConfig);

bool frSkySpiInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
rx_spi_received_e frSkySpiDataReceived(uint8_t *packet);
void frSkySpiSetRcData(uint16_t *rcData, const uint8_t *payload);

void frSkySpiBind(void);
