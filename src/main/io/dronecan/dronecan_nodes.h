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

#if ENABLE_DRONECAN

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

// Data classes observed from a remote node, so the CLI can show what each
// device on the bus is actually delivering (a GPS+compass module sets two).
#define DRONECAN_NODE_SENSOR_GPS        (1U << 0)
#define DRONECAN_NODE_SENSOR_MAG        (1U << 1)
#define DRONECAN_NODE_SENSOR_AIRSPEED   (1U << 2)
#define DRONECAN_NODE_SENSOR_ESC        (1U << 3)

// DSDL allows 80 bytes; reverse-DNS style names in the wild fit well inside
// this, and the table is per-node RAM.
#define DRONECAN_NODE_NAME_MAX          32U

typedef struct dronecanNodeEntry_s {
    uint8_t  nodeId;
    uint8_t  health;                    // UAVCAN_NODE_HEALTH_*
    uint8_t  mode;                      // UAVCAN_NODE_MODE_*
    uint8_t  sensorFlags;               // DRONECAN_NODE_SENSOR_*
    uint8_t  infoRetries;
    bool     infoValid;
    uint32_t uptimeSec;
    timeUs_t lastHeardUs;
    char     name[DRONECAN_NODE_NAME_MAX + 1];
} dronecanNodeEntry_t;

void dronecanNodesInit(void);
void dronecanNodesUpdate(timeUs_t currentTimeUs);

// Called by the sensor RX handlers to attribute a data class to its source
// node. Safe before the node's first NodeStatus — the entry is created on
// demand.
void dronecanNodesNoteSensor(uint8_t nodeId, uint8_t sensorFlag);

uint8_t dronecanNodesCount(void);
const dronecanNodeEntry_t *dronecanNodesGet(uint8_t index);

#endif // ENABLE_DRONECAN
