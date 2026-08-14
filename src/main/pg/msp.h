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
#include "msp/msp_serial.h"

#include "pg/pg.h"

typedef struct mspConfig_s {
    uint8_t halfDuplex; // allow msp to operate in half duplex mode
    int8_t msp_uart[MAX_MSP_PORT_COUNT];  // serialPortIdentifier_e per slot; SERIAL_PORT_NONE = unused
} mspConfig_t;

PG_DECLARE(mspConfig_t, mspConfig);
