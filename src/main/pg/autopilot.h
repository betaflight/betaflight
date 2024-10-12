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

#include "pg/pg.h"

typedef struct autopilotConfig_s {
    uint8_t landing_altitude_m;   // altitude below which landing behaviours can change, metres
    uint16_t hover_throttle;      // value used at the start of a rescue or position hold
    uint16_t throttle_min;
    uint16_t throttle_max;
    uint8_t altitude_P;
    uint8_t altitude_I;
    uint8_t altitude_D;
    uint8_t altitude_F;
    uint8_t position_P;
    uint8_t position_I;
    uint8_t position_D;
    uint8_t position_J;
    uint8_t position_cutoff;
} autopilotConfig_t;

PG_DECLARE(autopilotConfig_t, autopilotConfig);

