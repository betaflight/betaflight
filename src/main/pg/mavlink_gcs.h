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

#include <stdint.h>

#include "pg/pg.h"

typedef enum {
    MAVLINK_GCS_FS_ACTION_OFF = 0,
    MAVLINK_GCS_FS_ACTION_AUTO_LAND,
    MAVLINK_GCS_FS_ACTION_GPS_RESCUE,
} mavlinkGcsFailsafeAction_e;

typedef struct mavlinkGcsConfig_s {
    uint8_t sysid;              // 0: lock to the first ground station seen
    uint8_t failsafeTimeoutS;
    uint8_t failsafeAction;     // mavlinkGcsFailsafeAction_e
} mavlinkGcsConfig_t;

PG_DECLARE(mavlinkGcsConfig_t, mavlinkGcsConfig);
