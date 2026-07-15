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

#include "platform.h"

#if ENABLE_DRONECAN_DNA

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "canard.h"

#include "common/time.h"
#include "common/utils.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "scheduler/scheduler.h"

#include "pg/dronecan.h"
#include "pg/dronecan_dna.h"

#include "io/dronecan/dronecan.h"
#include "io/dronecan/dronecan_dna.h"
#include "io/dronecan/dronecan_msg.h"

//-----------------------------------------------------------------------------
// Centralised allocator. The FC hands node IDs to anonymous peers (e.g. ESCs)
// via the three-stage uavcan.protocol.dynamic_node_id.Allocation handshake.
//
// A single in-flight session is tracked: the unique_id is accumulated 6 bytes
// per request; each intermediate request is echoed back (node_id 0); once all
// 16 bytes are in, a free node ID is assigned, persisted, and returned with the
// full unique_id. first_part_of_unique_id always resets the accumulator, so a
// fresh stage-1 from any peer cleanly restarts the sequence.
//-----------------------------------------------------------------------------

static uint8_t  dnaAccum[UAVCAN_DNA_UNIQUE_ID_LEN];
static uint8_t  dnaAccumLen;
static bool     dnaSessionActive;
static timeUs_t dnaLastActivityUs;
static uint8_t  dnaTransferId;

// Node IDs observed live on the bus (one bit per ID). Statically-configured
// peers and nodes bound by another allocator never appear in the persisted
// table, so the allocator must also exclude anyone it has heard NodeStatus
// from. Never cleared: a node that goes quiet may only be rebooting.
static uint32_t dnaSeenNodeIds[(CANARD_MAX_NODE_ID + 32) / 32];

static void handleNodeStatus(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    const uint8_t id = t->source_node_id;
    if (id >= CANARD_MIN_NODE_ID && id <= CANARD_MAX_NODE_ID) {
        dnaSeenNodeIds[id / 32] |= 1U << (id % 32);
    }
}

static bool dnaNodeIdSeen(uint8_t id)
{
    return (dnaSeenNodeIds[id / 32] & (1U << (id % 32))) != 0;
}

//-----------------------------------------------------------------------------
// Persistent table helpers
//-----------------------------------------------------------------------------

// Return the node ID previously bound to this unique_id, or 0 if none.
static uint8_t dnaTableLookup(const uint8_t uniqueId[UAVCAN_DNA_UNIQUE_ID_LEN])
{
    for (int i = 0; i < DRONECAN_DNA_MAX_ENTRIES; i++) {
        const dronecanDnaEntry_t *e = &dronecanDnaConfig()->entry[i];
        if (e->nodeId != 0
                && memcmp(e->uniqueId, uniqueId, UAVCAN_DNA_UNIQUE_ID_LEN) == 0) {
            return e->nodeId;
        }
    }
    return 0;
}

// Persist a new binding. Writes EEPROM only while disarmed — allocations happen
// at peer power-up, so this is a rare, ground-only event.
static void dnaTableStore(const uint8_t uniqueId[UAVCAN_DNA_UNIQUE_ID_LEN], uint8_t nodeId)
{
    for (int i = 0; i < DRONECAN_DNA_MAX_ENTRIES; i++) {
        dronecanDnaEntry_t *e = &dronecanDnaConfigMutable()->entry[i];
        if (e->nodeId == 0) {
            memcpy(e->uniqueId, uniqueId, UAVCAN_DNA_UNIQUE_ID_LEN);
            e->nodeId = nodeId;
            if (!ARMING_FLAG(ARMED)) {
                // The flash write stalls this task for milliseconds; tell the
                // scheduler to ignore it rather than flag a timing overrun.
                schedulerIgnoreTaskExecTime();
                writeEEPROM();
            }
            return;
        }
    }
    // Table full: the binding still works for this power cycle (the response is
    // sent regardless), it just won't survive a reboot.
}

// Pick a free node ID, honouring the peer's preference where possible. Searches
// up from the preferred (or 125) towards 125, then down towards 1, skipping our
// own node ID, any already in the table, and any heard live on the bus.
// Returns 0 if none is available.
static uint8_t dnaFindFreeNodeId(uint8_t preferred)
{
    bool occupied[CANARD_MAX_NODE_ID + 1];
    memset(occupied, 0, sizeof(occupied));

    const uint8_t ownId = dronecanConfig()->node_id;
    if (ownId >= CANARD_MIN_NODE_ID && ownId <= CANARD_MAX_NODE_ID) {
        occupied[ownId] = true;
    }
    for (int i = 0; i < DRONECAN_DNA_MAX_ENTRIES; i++) {
        const uint8_t id = dronecanDnaConfig()->entry[i].nodeId;
        if (id >= CANARD_MIN_NODE_ID && id <= CANARD_MAX_NODE_ID) {
            occupied[id] = true;
        }
    }
    for (uint8_t id = CANARD_MIN_NODE_ID; id <= CANARD_MAX_NODE_ID; id++) {
        if (dnaNodeIdSeen(id)) {
            occupied[id] = true;
        }
    }

    // Dynamic IDs live in [1, 125]; 126/127 are reserved for static use.
    const uint8_t topId = 125;
    uint8_t start = (preferred >= CANARD_MIN_NODE_ID && preferred <= topId) ? preferred : topId;

    for (uint8_t id = start; id <= topId; id++) {
        if (!occupied[id]) {
            return id;
        }
    }
    for (uint8_t id = start; id >= CANARD_MIN_NODE_ID; id--) {
        if (!occupied[id]) {
            return id;
        }
    }
    return 0;
}

//-----------------------------------------------------------------------------
// Allocation message: build + send the (broadcast) response
//-----------------------------------------------------------------------------

static void dnaRespond(uint8_t nodeId, const uint8_t *uniqueId, uint8_t len)
{
    // node_id (7) + first_part (1) = 1 byte, then len unique_id bytes.
    uint8_t payload[1U + UAVCAN_DNA_UNIQUE_ID_LEN];
    memset(payload, 0, sizeof(payload));

    const uint8_t firstPart = 0;    // responses never set first_part
    canardEncodeScalar(payload, UAVCAN_DNA_OFFSET_NODE_ID,    7, &nodeId);
    canardEncodeScalar(payload, UAVCAN_DNA_OFFSET_FIRST_PART, 1, &firstPart);
    for (uint8_t i = 0; i < len; i++) {
        canardEncodeScalar(payload, UAVCAN_DNA_OFFSET_UNIQUE_ID + i * 8U, 8, &uniqueId[i]);
    }

    CanardTxTransfer tx;
    canardInitTxTransfer(&tx);
    tx.transfer_type       = CanardTransferTypeBroadcast;
    tx.data_type_signature = UAVCAN_DNA_ALLOCATION_SIGNATURE;
    tx.data_type_id        = UAVCAN_DNA_ALLOCATION_ID;
    tx.inout_transfer_id   = &dnaTransferId;
    tx.priority            = CANARD_TRANSFER_PRIORITY_LOW;
    tx.payload             = payload;
    tx.payload_len         = (uint16_t)(1U + len);

    canardBroadcastObj(dronecanGetInstance(), &tx);
}

//-----------------------------------------------------------------------------
// Allocation request handler
//-----------------------------------------------------------------------------

static void handleAllocation(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    // Only anonymous transfers are allocation requests; non-anonymous Allocation
    // messages are our own echoes or another allocator's traffic — ignore them.
    if (t->source_node_id != CANARD_BROADCAST_NODE_ID) {
        return;
    }

    uint8_t preferredId = 0;
    uint8_t firstPart = 0;
    canardDecodeScalar(t, UAVCAN_DNA_OFFSET_NODE_ID,    7, false, &preferredId);
    canardDecodeScalar(t, UAVCAN_DNA_OFFSET_FIRST_PART, 1, false, &firstPart);

    // unique_id is the tail array: everything after the 1-byte header.
    uint16_t idBytes = (t->payload_len > 1U) ? (uint16_t)(t->payload_len - 1U) : 0U;
    if (idBytes > UAVCAN_DNA_MAX_BYTES_PER_REQUEST) {
        idBytes = UAVCAN_DNA_MAX_BYTES_PER_REQUEST;
    }

    if (firstPart) {
        dnaAccumLen = 0;            // stage 1 — restart accumulation
    }

    for (uint16_t i = 0; i < idBytes && dnaAccumLen < UAVCAN_DNA_UNIQUE_ID_LEN; i++) {
        uint8_t b = 0;
        canardDecodeScalar(t, UAVCAN_DNA_OFFSET_UNIQUE_ID + i * 8U, 8, false, &b);
        dnaAccum[dnaAccumLen++] = b;
    }

    dnaSessionActive = true;
    dnaLastActivityUs = micros();

    if (dnaAccumLen < UAVCAN_DNA_UNIQUE_ID_LEN) {
        // Intermediate stage — echo what we have so far with node_id 0.
        dnaRespond(0, dnaAccum, (uint8_t)dnaAccumLen);
        return;
    }

    // Full unique_id received — assign (or reuse) a node ID and finalise.
    uint8_t nodeId = dnaTableLookup(dnaAccum);
    if (nodeId == 0) {
        nodeId = dnaFindFreeNodeId(preferredId);
        if (nodeId == 0) {
            dnaSessionActive = false;   // nothing free to hand out
            return;
        }
        dnaTableStore(dnaAccum, nodeId);
    }

    dnaRespond(nodeId, dnaAccum, UAVCAN_DNA_UNIQUE_ID_LEN);
    dnaSessionActive = false;
}

//-----------------------------------------------------------------------------
// Public surface
//-----------------------------------------------------------------------------

void dronecanDnaInit(void)
{
    dnaAccumLen = 0;
    dnaSessionActive = false;
    dnaLastActivityUs = 0;
    dnaTransferId = 0;
    memset(dnaSeenNodeIds, 0, sizeof(dnaSeenNodeIds));

    if (!dronecanConfig()->dna_enabled) {
        return;
    }

    const dronecanSubscriber_t allocationSub = {
        .signature    = UAVCAN_DNA_ALLOCATION_SIGNATURE,
        .dataTypeId   = UAVCAN_DNA_ALLOCATION_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleAllocation,
    };
    (void)dronecanRegisterSubscriber(&allocationSub);

    // Every node broadcasts NodeStatus at >=1 Hz, so listening to it is enough
    // to learn which IDs are already taken on the bus.
    const dronecanSubscriber_t nodeStatusSub = {
        .signature    = UAVCAN_NODE_STATUS_SIGNATURE,
        .dataTypeId   = UAVCAN_NODE_STATUS_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleNodeStatus,
    };
    (void)dronecanRegisterSubscriber(&nodeStatusSub);
}

void dronecanDnaExpireSessions(timeUs_t currentTimeUs)
{
    if (dnaSessionActive
            && cmpTimeUs(currentTimeUs, dnaLastActivityUs) >= (timeDelta_t)(UAVCAN_DNA_FOLLOWUP_TIMEOUT_MS * 1000U)) {
        // Allocatee went quiet mid-handshake; drop the partial state so its next
        // stage-1 request starts cleanly.
        dnaSessionActive = false;
        dnaAccumLen = 0;
    }
}

#endif // ENABLE_DRONECAN_DNA
