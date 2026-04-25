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

// DSDL signature / data-type-ID constants for the subset of uavcan.protocol.*
// types consumed by the phase-1 DroneCAN stack. Values come from the upstream
// DSDL definitions and are deterministic (CRC-64-WE of the canonical DSDL
// text). Don't regenerate them locally — matching these to the rest of the
// DroneCAN ecosystem is what keeps our frames interoperable.

#pragma once

#include <stdint.h>

// uavcan.protocol.NodeStatus — broadcast @ 1 Hz (mandatory for any node).
#define UAVCAN_NODE_STATUS_ID           341U
#define UAVCAN_NODE_STATUS_SIGNATURE    0x0f0868d0c1a7c6f1ULL
#define UAVCAN_NODE_STATUS_PAYLOAD_LEN  7U

// uavcan.protocol.GetNodeInfo — service type (request is empty).
#define UAVCAN_GET_NODE_INFO_ID         1U
#define UAVCAN_GET_NODE_INFO_SIGNATURE  0xee468a8121c46a9eULL

// NodeStatus.health
#define UAVCAN_NODE_HEALTH_OK           0U
#define UAVCAN_NODE_HEALTH_WARNING      1U
#define UAVCAN_NODE_HEALTH_ERROR        2U
#define UAVCAN_NODE_HEALTH_CRITICAL     3U

// NodeStatus.mode
#define UAVCAN_NODE_MODE_OPERATIONAL        0U
#define UAVCAN_NODE_MODE_INITIALIZATION     1U
#define UAVCAN_NODE_MODE_MAINTENANCE        2U
#define UAVCAN_NODE_MODE_SOFTWARE_UPDATE    3U
#define UAVCAN_NODE_MODE_OFFLINE            7U
