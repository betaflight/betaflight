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

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/vtx_table.h"

typedef struct vtxTableConfig_s {
    uint8_t  bands;
    uint8_t  channels;
    uint16_t frequency[VTX_TABLE_MAX_BANDS][VTX_TABLE_MAX_CHANNELS];
    char     bandNames[VTX_TABLE_MAX_BANDS][VTX_TABLE_BAND_NAME_LENGTH + 1];
    char     bandLetters[VTX_TABLE_MAX_BANDS];
    char     channelNames[VTX_TABLE_MAX_CHANNELS][VTX_TABLE_CHANNEL_NAME_LENGTH + 1];

    uint8_t  powerLevels;
    uint16_t powerValues[VTX_TABLE_MAX_POWER_LEVELS];
    char     powerLabels[VTX_TABLE_MAX_POWER_LEVELS][VTX_TABLE_POWER_LABEL_LENGTH + 1];
} vtxTableConfig_t;

struct vtxTableConfig_s;
PG_DECLARE(struct vtxTableConfig_s, vtxTableConfig);
