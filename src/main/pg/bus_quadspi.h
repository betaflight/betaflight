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

#include "drivers/bus_quadspi.h"
#include "drivers/io_types.h"

#include "pg/pg.h"

#ifdef USE_QUADSPI

typedef struct quadSpiConfig_s {
    ioTag_t ioTagClk;

    ioTag_t ioTagBK1IO0;
    ioTag_t ioTagBK1IO1;
    ioTag_t ioTagBK1IO2;
    ioTag_t ioTagBK1IO3;
    ioTag_t ioTagBK1CS;

    ioTag_t ioTagBK2IO0;
    ioTag_t ioTagBK2IO1;
    ioTag_t ioTagBK2IO2;
    ioTag_t ioTagBK2IO3;
    ioTag_t ioTagBK2CS;

    uint8_t mode;

    // CS pins can be under software control, useful when using BK1CS as the CS pin for BK2 in non-DUAL-FLASH mode.
    uint8_t csFlags;
} quadSpiConfig_t;

PG_DECLARE_ARRAY(quadSpiConfig_t, QUADSPIDEV_COUNT, quadSpiConfig);

#endif
