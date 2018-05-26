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

#include "drivers/bus_spi.h"
#include "drivers/io_types.h"

#include "pg/pg.h"

typedef struct spiPinConfig_s {
    ioTag_t ioTagSck;
    ioTag_t ioTagMiso;
    ioTag_t ioTagMosi;
} spiPinConfig_t;

PG_DECLARE_ARRAY(spiPinConfig_t, SPIDEV_COUNT, spiPinConfig);

// Place holder for CS pins for pre-initialization

typedef struct spiCs_s {
    ioTag_t csnTag;
} spiCs_t;

PG_DECLARE_ARRAY(spiCs_t, SPI_PREINIT_IPU_COUNT, spiPreinitIPUConfig);
PG_DECLARE_ARRAY(spiCs_t, SPI_PREINIT_OPU_COUNT, spiPreinitOPUConfig);
