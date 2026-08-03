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

#include "canard.h"

#include "common/time.h"

// Maximum number of subscribers a single stack instance accepts. NodeStatus
// + GetNodeInfo use two slots out of the box; consumers added in follow-up
// PRs (GPS, ESC telemetry, etc.) register into the remainder. Bump if a
// target needs more — each slot is ~16 bytes of .bss.
#define DRONECAN_MAX_SUBSCRIBERS    8

typedef void (*dronecanRxHandler)(CanardInstance *ins, CanardRxTransfer *transfer);

typedef struct dronecanSubscriber_s {
    uint64_t signature;                 // DSDL data_type_signature for CRC seed
    uint16_t dataTypeId;                // service or message ID
    uint8_t  transferType;              // CanardTransferTypeBroadcast/Request/Response
    dronecanRxHandler handler;          // invoked from task context when a full
                                        // transfer has been reassembled
} dronecanSubscriber_t;

// Start the stack on the CAN device selected by dronecanConfig(). No-op if
// the PG flag is clear, the node ID is 0, or the underlying CAN device failed
// to initialise. Idempotent — safe to call from fc/init.c unconditionally.
void dronecanInit(void);

// Returns true after dronecanInit() has brought up the transport and the
// local node ID is valid. The scheduler uses this to decide whether the
// dronecan task is queued.
bool dronecanIsInitialised(void);

// Scheduler entry point. Drains the RX ISR ring into libcanard, walks the TX
// queue into the CAN driver, runs the 1 Hz NodeStatus boundary, and sweeps
// stale transfers once per second.
void dronecanUpdate(timeUs_t currentTimeUs);

// Register an RX handler for a specific DSDL data type. Returns false if the
// subscriber table is full. Handlers fire from task context (never from the
// CAN ISR) so they can safely touch PG state and cross-subsystem globals.
bool dronecanRegisterSubscriber(const dronecanSubscriber_t *subscriber);

// Handle to the singleton Canard instance, so node-side publishers and
// responders can call canardBroadcastObj / canardRequestOrRespondObj without
// each one having to re-discover the pool.
CanardInstance *dronecanGetInstance(void);

#endif // ENABLE_DRONECAN
