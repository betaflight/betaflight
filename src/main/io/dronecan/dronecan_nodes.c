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

#if ENABLE_DRONECAN

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "canard.h"

#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/dronecan/dronecan.h"
#include "io/dronecan/dronecan_msg.h"
#include "io/dronecan/dronecan_nodes.h"

#define DRONECAN_MAX_NODES          16U
#define NODES_INFO_RETRY_MAX        3U
#define NODES_INFO_TIMEOUT_US       2000000

static dronecanNodeEntry_t nodes[DRONECAN_MAX_NODES];
static uint8_t nodeCount;

// GetNodeInfo client state. The 1 KiB libcanard pool only has headroom for
// one multi-frame response in flight, so requests are strictly serialised:
// pendingNodeId != 0 blocks the scheduler until the response lands or the
// timeout passes.
static uint8_t infoTransferId;
static uint8_t pendingNodeId;
static timeUs_t pendingSentUs;

static dronecanNodeEntry_t *findNode(uint8_t nodeId)
{
    for (uint8_t i = 0; i < nodeCount; i++) {
        if (nodes[i].nodeId == nodeId) {
            return &nodes[i];
        }
    }
    return NULL;
}

static dronecanNodeEntry_t *findOrAddNode(uint8_t nodeId)
{
    dronecanNodeEntry_t *node = findNode(nodeId);
    if (node) {
        return node;
    }
    if (nodeId < CANARD_MIN_NODE_ID || nodeId > CANARD_MAX_NODE_ID
            || nodeCount >= DRONECAN_MAX_NODES) {
        return NULL;
    }
    node = &nodes[nodeCount++];
    memset(node, 0, sizeof(*node));
    node->nodeId = nodeId;
    return node;
}

static void handleNodeStatus(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    dronecanNodeEntry_t *node = findOrAddNode(t->source_node_id);
    if (!node) {
        return;
    }

    uint32_t uptimeSec = 0;
    uint8_t health = 0;
    uint8_t mode = 0;
    canardDecodeScalar(t, 0, 32, false, &uptimeSec);
    canardDecodeScalar(t, 32, 2, false, &health);
    canardDecodeScalar(t, 34, 3, false, &mode);

    node->uptimeSec = uptimeSec;
    node->health = health;
    node->mode = mode;
    node->lastHeardUs = micros();
}

static void handleNodeInfoResponse(CanardInstance *ins, CanardRxTransfer *t)
{
    UNUSED(ins);

    dronecanNodeEntry_t *node = findNode(t->source_node_id);
    if (!node) {
        return;
    }
    if (t->source_node_id == pendingNodeId) {
        pendingNodeId = 0;
    }

    // Response layout (see the responder in dronecan_node.c): NodeStatus (7)
    // + SoftwareVersion (15) + HardwareVersion (18 + COA length byte). The
    // certificate_of_authenticity is a length-prefixed array, so the name's
    // start moves with it; name itself is TAO — it runs to the payload end.
    if (t->payload_len < 41U) {
        return;
    }
    uint8_t coaLen = 0;
    canardDecodeScalar(t, 40U * 8U, 8, false, &coaLen);

    const uint32_t nameOffset = 41U + coaLen;
    uint8_t nameLen = 0;
    if (t->payload_len > nameOffset) {
        nameLen = (uint8_t)(t->payload_len - nameOffset);
        if (nameLen > DRONECAN_NODE_NAME_MAX) {
            nameLen = DRONECAN_NODE_NAME_MAX;
        }
    }
    for (uint8_t i = 0; i < nameLen; i++) {
        uint8_t c = 0;
        canardDecodeScalar(t, (nameOffset + i) * 8U, 8, false, &c);
        node->name[i] = (c >= 0x20U && c < 0x7FU) ? (char)c : '?';
    }
    node->name[nameLen] = '\0';
    node->infoValid = true;
}

static void sendNodeInfoRequest(dronecanNodeEntry_t *node, timeUs_t currentTimeUs)
{
    // GetNodeInfo requests carry an empty payload; give libcanard a valid
    // pointer anyway so the zero-length copy is well defined.
    static const uint8_t emptyPayload[1] = { 0 };

    CanardTxTransfer tx;
    canardInitTxTransfer(&tx);
    tx.transfer_type       = CanardTransferTypeRequest;
    tx.data_type_signature = UAVCAN_GET_NODE_INFO_SIGNATURE;
    tx.data_type_id        = UAVCAN_GET_NODE_INFO_ID;
    tx.inout_transfer_id   = &infoTransferId;
    tx.priority            = CANARD_TRANSFER_PRIORITY_LOW;
    tx.payload             = emptyPayload;
    tx.payload_len         = 0;

    if (canardRequestOrRespondObj(dronecanGetInstance(), node->nodeId, &tx) >= 0) {
        node->infoRetries++;
        pendingNodeId = node->nodeId;
        pendingSentUs = currentTimeUs;
    }
}

void dronecanNodesInit(void)
{
    memset(nodes, 0, sizeof(nodes));
    nodeCount = 0;
    infoTransferId = 0;
    pendingNodeId = 0;
    pendingSentUs = 0;

    const dronecanSubscriber_t nodeStatusSub = {
        .signature    = UAVCAN_NODE_STATUS_SIGNATURE,
        .dataTypeId   = UAVCAN_NODE_STATUS_ID,
        .transferType = CanardTransferTypeBroadcast,
        .handler      = handleNodeStatus,
    };
    (void)dronecanRegisterSubscriber(&nodeStatusSub);

    const dronecanSubscriber_t nodeInfoSub = {
        .signature    = UAVCAN_GET_NODE_INFO_SIGNATURE,
        .dataTypeId   = UAVCAN_GET_NODE_INFO_ID,
        .transferType = CanardTransferTypeResponse,
        .handler      = handleNodeInfoResponse,
    };
    (void)dronecanRegisterSubscriber(&nodeInfoSub);
}

void dronecanNodesUpdate(timeUs_t currentTimeUs)
{
    if (pendingNodeId != 0U
            && cmpTimeUs(currentTimeUs, pendingSentUs) < NODES_INFO_TIMEOUT_US) {
        return;
    }
    pendingNodeId = 0;

    for (uint8_t i = 0; i < nodeCount; i++) {
        dronecanNodeEntry_t *node = &nodes[i];
        if (!node->infoValid && node->infoRetries < NODES_INFO_RETRY_MAX) {
            sendNodeInfoRequest(node, currentTimeUs);
            return;
        }
    }
}

void dronecanNodesNoteSensor(uint8_t nodeId, uint8_t sensorFlag)
{
    dronecanNodeEntry_t *node = findOrAddNode(nodeId);
    if (node) {
        node->sensorFlags |= sensorFlag;
        node->lastHeardUs = micros();
    }
}

uint8_t dronecanNodesCount(void)
{
    return nodeCount;
}

const dronecanNodeEntry_t *dronecanNodesGet(uint8_t index)
{
    return (index < nodeCount) ? &nodes[index] : NULL;
}

#endif // ENABLE_DRONECAN
