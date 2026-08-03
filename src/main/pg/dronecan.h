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

typedef struct dronecanConfig_s {
    uint8_t enabled;    // 0 = off, 1 = on
    uint8_t node_id;    // DroneCAN node ID (1..127). 0 == unset (node stays inactive).
    uint8_t device;     // CAN device number (1..CANDEV_COUNT), 1-based to match CLI
} dronecanConfig_t;

PG_DECLARE(dronecanConfig_t, dronecanConfig);
