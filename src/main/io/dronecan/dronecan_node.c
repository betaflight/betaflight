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

#include <stdint.h>
#include <string.h>

#include "canard.h"

#include "build/version.h"

#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/dronecan/dronecan.h"
#include "io/dronecan/dronecan_msg.h"

// Name broadcast in uavcan.protocol.GetNodeInfo.Response.name. The DroneCAN
// convention is a reverse-DNS-style identifier; the wider ecosystem uses the
// vendor prefix to identify who built the firmware, not the target board.
#define DRONECAN_NODE_NAME          "org.betaflight"
#define DRONECAN_NODE_NAME_LEN      (sizeof(DRONECAN_NODE_NAME) - 1U)

// Upper bound on the encoded response:
//   NodeStatus        : 7  bytes  (UAVCAN_NODE_STATUS_PAYLOAD_LEN)
//   SoftwareVersion   : 15 bytes  (major + minor + flags + vcs_commit + image_crc)
//   HardwareVersion   : 19 bytes  (major + minor + unique_id[16] + coa length byte)
//   name              : up to 80 bytes (uavcan.protocol.GetNodeInfo.Response.name)
// Total fits 121. The stack buffer in dronecanHandleGetNodeInfoRequest is
// sized from this constant so overruns stay caught at compile time if the
// node name grows.
#define DRONECAN_GET_NODE_INFO_MAX  (UAVCAN_NODE_STATUS_PAYLOAD_LEN + 15U + 19U + 80U)

// Transfer-ID counters. DroneCAN requires a monotonically-increasing 5-bit
// counter per (node, data_type_id, transfer_type) triple. Libcanard manages
// the wrap; we just give it a persistent storage location.
static uint8_t dronecanNodeStatusTransferId;

// Monotonic uptime counter in whole seconds, incremented each time the
// periodic task publishes NodeStatus. Avoids the ~71-minute wraparound
// of micros() on 32-bit time builds.
static uint32_t dronecanNodeUptimeSec;

//-----------------------------------------------------------------------------
// Encoding helpers
//
// Fields are byte-aligned once the NodeStatus.health/mode/sub_mode byte has
// been clumped, so a direct little-endian byte layout matches what libcanard
// expects without needing the bit-level canardEncodeScalar() helper.
//-----------------------------------------------------------------------------

static void encodeU16(uint8_t *buf, uint16_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void encodeU32(uint8_t *buf, uint32_t v)
{
    buf[0] = (uint8_t)(v & 0xFF);
    buf[1] = (uint8_t)((v >> 8) & 0xFF);
    buf[2] = (uint8_t)((v >> 16) & 0xFF);
    buf[3] = (uint8_t)((v >> 24) & 0xFF);
}

// 7-byte NodeStatus payload common to both the periodic broadcast and the
// leading field of GetNodeInfo.Response. Written to the caller's buffer.
static void encodeNodeStatus(uint8_t buf[UAVCAN_NODE_STATUS_PAYLOAD_LEN],
                             uint32_t uptime_sec)
{
    encodeU32(&buf[0], uptime_sec);

    // health (2b) | mode (3b) | sub_mode (3b) packed into a single byte.
    const uint8_t health   = UAVCAN_NODE_HEALTH_OK;
    const uint8_t mode     = UAVCAN_NODE_MODE_OPERATIONAL;
    const uint8_t sub_mode = 0U;
    buf[4] = (uint8_t)((health & 0x03U)
                     | ((mode & 0x07U) << 2)
                     | ((sub_mode & 0x07U) << 5));

    // vendor_specific_status_code: no use yet — zero is a valid "no info".
    encodeU16(&buf[5], 0U);
}

// Derive a 16-byte DroneCAN unique ID from the MCU's 96-bit UID. The hw spec
// provides a fixed-size 128-bit field; we zero-pad the trailing 32 bits so
// two boards built from the same silicon lot still produce distinct IDs.
static void dronecanGetUniqueId(uint8_t out[16])
{
    encodeU32(&out[0],  U_ID_0);
    encodeU32(&out[4],  U_ID_1);
    encodeU32(&out[8],  U_ID_2);
    encodeU32(&out[12], 0U);
}

//-----------------------------------------------------------------------------
// GetNodeInfo service responder
//-----------------------------------------------------------------------------

static void dronecanHandleGetNodeInfoRequest(CanardInstance *ins,
                                             CanardRxTransfer *request)
{
    uint8_t response[DRONECAN_GET_NODE_INFO_MAX];
    size_t offset = 0;

    // NodeStatus (7 bytes). Uses the monotonic uptime counter — not
    // micros() — so the value matches what we publish periodically and
    // doesn't roll over at 32-bit micros wrap (~71 minutes).
    encodeNodeStatus(&response[offset], dronecanNodeUptimeSec);
    offset += UAVCAN_NODE_STATUS_PAYLOAD_LEN;

    // SoftwareVersion (15 bytes): major, minor, optional_flags,
    // vcs_commit (u32), image_crc (u64). No VCS / image CRC yet → flags 0.
    response[offset++] = (uint8_t)(FC_VERSION_YEAR - FC_CALVER_BASE_YEAR);
    response[offset++] = (uint8_t)FC_VERSION_MONTH;
    response[offset++] = 0U;                            // optional_field_flags
    encodeU32(&response[offset], 0U); offset += 4;      // vcs_commit
    memset(&response[offset], 0, 8); offset += 8;       // image_crc (u64)

    // HardwareVersion: major, minor, unique_id[16], certificate length.
    // Flight controllers don't carry board-level hw version — leave 0/0.
    response[offset++] = 0U;                            // hw_major
    response[offset++] = 0U;                            // hw_minor
    dronecanGetUniqueId(&response[offset]);
    offset += 16;
    response[offset++] = 0U;                            // coa length (empty)

    // name (tail array, TAO omits the length prefix).
    memcpy(&response[offset], DRONECAN_NODE_NAME, DRONECAN_NODE_NAME_LEN);
    offset += DRONECAN_NODE_NAME_LEN;

    CanardTxTransfer tx;
    canardInitTxTransfer(&tx);
    tx.transfer_type       = CanardTransferTypeResponse;
    tx.data_type_signature = UAVCAN_GET_NODE_INFO_SIGNATURE;
    tx.data_type_id        = UAVCAN_GET_NODE_INFO_ID;
    tx.inout_transfer_id   = &request->transfer_id;   // echo the request TID
    tx.priority            = request->priority;
    tx.payload             = response;
    tx.payload_len         = (uint16_t)offset;

    canardRequestOrRespondObj(ins, request->source_node_id, &tx);
}

//-----------------------------------------------------------------------------
// Public surface invoked by dronecan.c
//-----------------------------------------------------------------------------

void dronecanNodeInit(void)
{
    // Register ourselves as the GetNodeInfo server. NodeStatus is a broadcast
    // publisher — it needs no registration on this side; only peers
    // subscribing to it do.
    const dronecanSubscriber_t sub = {
        .signature    = UAVCAN_GET_NODE_INFO_SIGNATURE,
        .dataTypeId   = UAVCAN_GET_NODE_INFO_ID,
        .transferType = CanardTransferTypeRequest,
        .handler      = dronecanHandleGetNodeInfoRequest,
    };
    (void)dronecanRegisterSubscriber(&sub);

    dronecanNodeStatusTransferId = 0;
    dronecanNodeUptimeSec = 0;
}

void dronecanNodeUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Broadcast NodeStatus. Called from the dronecan.c 1 Hz boundary, so
    // incrementing a monotonic counter gives us uptime_sec without
    // relying on the 32-bit micros() clock.
    const uint32_t uptime_sec = ++dronecanNodeUptimeSec;

    uint8_t payload[UAVCAN_NODE_STATUS_PAYLOAD_LEN];
    encodeNodeStatus(payload, uptime_sec);

    CanardTxTransfer tx;
    canardInitTxTransfer(&tx);
    tx.transfer_type       = CanardTransferTypeBroadcast;
    tx.data_type_signature = UAVCAN_NODE_STATUS_SIGNATURE;
    tx.data_type_id        = UAVCAN_NODE_STATUS_ID;
    tx.inout_transfer_id   = &dronecanNodeStatusTransferId;
    tx.priority            = CANARD_TRANSFER_PRIORITY_LOW;
    tx.payload             = payload;
    tx.payload_len         = sizeof(payload);

    canardBroadcastObj(dronecanGetInstance(), &tx);
}

#endif // ENABLE_DRONECAN
