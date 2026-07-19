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

#include "platform.h"

#if ENABLE_DRONECAN_DNA

#include <stdint.h>

#include "pg/pg.h"

#include "io/dronecan/dronecan_msg.h"

// Persisted dynamic node-ID allocation table. Each occupied slot binds a peer's
// 16-byte unique ID to the node ID this FC handed it, so the same device keeps
// its ID across reboots. nodeId == 0 marks an empty slot.
#define DRONECAN_DNA_MAX_ENTRIES    16

typedef struct dronecanDnaEntry_s {
    uint8_t uniqueId[UAVCAN_DNA_UNIQUE_ID_LEN];
    uint8_t nodeId;
} dronecanDnaEntry_t;

typedef struct dronecanDnaConfig_s {
    dronecanDnaEntry_t entry[DRONECAN_DNA_MAX_ENTRIES];
} dronecanDnaConfig_t;

PG_DECLARE(dronecanDnaConfig_t, dronecanDnaConfig);

#endif // ENABLE_DRONECAN_DNA
