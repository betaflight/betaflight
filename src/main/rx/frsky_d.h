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

#include <stdbool.h>
#include <stdint.h>

#include "rx_spi.h"

typedef struct frSkyDConfig_s {
    bool autoBind;
    uint8_t bindTxId[2];
    int8_t  bindOffset;
    uint8_t bindHopData[50];
} frSkyDConfig_t;

PG_DECLARE(frSkyDConfig_t, frSkyDConfig);

struct rxConfig_s;
struct rxRuntimeConfig_s;
void frSkyDInit(const struct rxConfig_s *rxConfig, struct rxRuntimeConfig_s *rxRuntimeConfig);
void frSkyDSetRcData(uint16_t *rcData, const uint8_t *payload);
rx_spi_received_e frSkyDDataReceived(uint8_t *payload);
void frSkyDBind();
