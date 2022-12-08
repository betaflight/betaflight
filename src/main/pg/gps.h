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

#include <stdbool.h>
#include <stdint.h>

#include "pg/pg.h"

typedef struct gpsConfig_s {
    uint8_t provider;
    uint8_t sbasMode;
    uint8_t autoConfig;
    uint8_t autoBaud;
    uint8_t gps_ublox_mode;
    bool gps_ublox_use_galileo;
    bool gps_set_home_point_once;
    bool gps_use_3d_speed;
    bool sbas_integrity;
} gpsConfig_t;

PG_DECLARE(gpsConfig_t, gpsConfig);
