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

typedef struct max7456Config_s {
    uint8_t clockConfig; // SPI clock modifier
    ioTag_t csTag;
    uint8_t spiDevice;
    bool preInitOPU;
} max7456Config_t;

// clockConfig values
#define MAX7456_CLOCK_CONFIG_HALF       0  // Force half clock
#define MAX7456_CLOCK_CONFIG_NOMINAL    1  // Nominal clock (default)
#define MAX7456_CLOCK_CONFIG_DOUBLE     2  // Double clock

PG_DECLARE(max7456Config_t, max7456Config);
