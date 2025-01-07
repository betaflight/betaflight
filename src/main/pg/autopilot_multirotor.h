/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef USE_WING

#include <stdint.h>

#include "pg/pg.h"

typedef struct apConfig_s {
    uint8_t ap_landing_altitude_m;   // altitude below which landing behaviours can change, metres
    uint16_t ap_hover_throttle;      // value used at the start of a rescue or position hold
    uint16_t ap_throttle_min;
    uint16_t ap_throttle_max;
    uint8_t ap_altitude_P;
    uint8_t ap_altitude_I;
    uint8_t ap_altitude_D;
    uint8_t ap_altitude_F;
    uint8_t ap_position_P;
    uint8_t ap_position_I;
    uint8_t ap_position_D;
    uint8_t ap_position_A;
    uint8_t ap_position_cutoff;
    uint8_t ap_max_angle;
} apConfig_t;

PG_DECLARE(apConfig_t, apConfig);

#endif // !USE_WING
