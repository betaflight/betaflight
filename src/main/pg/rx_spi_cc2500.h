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

typedef enum {
  FRSKY_SPI_A1_SOURCE_VBAT = 0,
  FRSKY_SPI_A1_SOURCE_EXTADC,
  FRSKY_SPI_A1_SOURCE_CONST
} frSkySpiA1Source_e;

typedef struct rxCc2500SpiConfig_s {
    uint8_t autoBind;
    uint8_t bindTxId[3];
    int8_t  bindOffset;
    uint8_t bindHopData[50];
    uint8_t rxNum;
    uint8_t a1Source;
    uint8_t chipDetectEnabled;
    ioTag_t txEnIoTag;
    ioTag_t lnaEnIoTag;
    ioTag_t antSelIoTag;
} rxCc2500SpiConfig_t;

PG_DECLARE(rxCc2500SpiConfig_t, rxCc2500SpiConfig);
