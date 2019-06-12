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

#include "drivers/io_types.h"

#include "pg/pg.h"

typedef struct vtxIOConfig_s {
    // common settings for both hardware and software SPI
    ioTag_t csTag;
    ioTag_t powerTag;

    // settings for software SPI only
    ioTag_t dataTag;
    ioTag_t clockTag;

    // settings for hardware SPI only
    uint8_t spiDevice;
} vtxIOConfig_t;

PG_DECLARE(vtxIOConfig_t, vtxIOConfig);
