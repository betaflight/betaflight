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

// uavcan.equipment.gnss.Fix2 — broadcast GNSS solution (position, velocity,
// fix state). Superset of the legacy Fix message; most CAN GNSS modules
// publish this variant.
#define UAVCAN_GNSS_FIX2_ID             1063U
#define UAVCAN_GNSS_FIX2_SIGNATURE      0xca41e928aebcb2d9ULL

// Fix2.status enum values.
#define UAVCAN_GNSS_FIX2_STATUS_NO_FIX      0U
#define UAVCAN_GNSS_FIX2_STATUS_TIME_ONLY   1U
#define UAVCAN_GNSS_FIX2_STATUS_2D_FIX      2U
#define UAVCAN_GNSS_FIX2_STATUS_3D_FIX      3U

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

// uavcan.equipment.esc.RawCommand — broadcast throttle command.
//   saturated int14[<=20] cmd   (tail-array optimised: no length prefix on the
//   wire; element count is inferred from the transfer payload length). Each
//   element is a signed 14-bit value; for throttle the useful range is
//   0..8191 (negative would request reverse on bidirectional ESCs).
#define UAVCAN_ESC_RAWCOMMAND_ID            1030U
#define UAVCAN_ESC_RAWCOMMAND_SIGNATURE     0x217f5c87d7ec951dULL
#define UAVCAN_ESC_RAWCOMMAND_BITS          14U
#define UAVCAN_ESC_RAWCOMMAND_MAX           8191    // 2^13 - 1 (positive throttle ceiling)
#define UAVCAN_ESC_RAWCOMMAND_MAX_MOTORS    20U     // DSDL array bound

// uavcan.equipment.esc.Status — broadcast ESC telemetry. Fixed layout, no TAO.
//   uint32  error_count       @  0
//   float16 voltage           @ 32  (volt)
//   float16 current           @ 48  (ampere, may be negative under regen)
//   float16 temperature       @ 64  (kelvin)
//   int18   rpm               @ 80  (signed; negative = reverse)
//   uint7   power_rating_pct  @ 98
//   uint5   esc_index         @ 105 (zero-based; matches RawCommand cmd[] index)
// Total 110 bits = 14 bytes.
#define UAVCAN_ESC_STATUS_ID                1034U
#define UAVCAN_ESC_STATUS_SIGNATURE         0xa9af28aea2fbb254ULL
#define ESC_STATUS_OFFSET_ERROR_COUNT       0U
#define ESC_STATUS_OFFSET_VOLTAGE           32U
#define ESC_STATUS_OFFSET_CURRENT           48U
#define ESC_STATUS_OFFSET_TEMPERATURE       64U
#define ESC_STATUS_OFFSET_RPM               80U
#define ESC_STATUS_OFFSET_POWER_PCT         98U
#define ESC_STATUS_OFFSET_ESC_INDEX         105U
