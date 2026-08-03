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

#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/can/can.h"
#include "drivers/time.h"

#include "io/dronecan/dronecan.h"

#include "pg/dronecan.h"

// Forward declarations; implemented in dronecan_node.c / dronecan_gnss.c.
void dronecanNodeInit(void);
void dronecanNodeUpdate(timeUs_t currentTimeUs);
void dronecanGnssInit(void);

//-----------------------------------------------------------------------------
// Memory pool
//
// libcanard's memory pool is a simple block allocator over a caller-supplied
// arena. Each block is CANARD_MEM_BLOCK_SIZE (32 bytes with the default build
// config). 1 KiB gives us 32 blocks — enough for NodeStatus (1 frame), a
// multi-frame GetNodeInfo response (~4 blocks per request in flight), plus
// headroom for future consumers that register subscribers.
//-----------------------------------------------------------------------------
#define DRONECAN_POOL_SIZE  1024U
static uint8_t dronecanPool[DRONECAN_POOL_SIZE];

//-----------------------------------------------------------------------------
// RX ring buffer
//
// canRegisterRxCallback() fires from the CAN ISR. libcanard's state machine
// is not re-entrant against concurrent canardBroadcast/Respond calls made
// from task context, so we can't call canardHandleRxFrame directly in the
// ISR. Instead the ISR pushes the raw frame onto a lock-free SPSC ring and
// the task drains it. Size is a power of two so the index can be masked.
//-----------------------------------------------------------------------------
#define DRONECAN_RX_RING_SIZE   32U
#define DRONECAN_RX_RING_MASK   (DRONECAN_RX_RING_SIZE - 1U)

typedef struct dronecanRxRingEntry_s {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  length;
    bool     isExtended;
} dronecanRxRingEntry_t;

static dronecanRxRingEntry_t dronecanRxRing[DRONECAN_RX_RING_SIZE];
static volatile uint8_t dronecanRxHead;     // written by ISR
static volatile uint8_t dronecanRxTail;     // written by task

//-----------------------------------------------------------------------------
// Singleton state
//-----------------------------------------------------------------------------
static CanardInstance dronecanInstance;
static bool dronecanInitialised;
static canDevice_e dronecanDevice = CANINVALID;

static dronecanSubscriber_t dronecanSubscribers[DRONECAN_MAX_SUBSCRIBERS];
static uint8_t dronecanSubscriberCount;

// 1 Hz boundary tracker. Keeping a separate counter (rather than doing
// modulo arithmetic on currentTimeUs) lets us emit NodeStatus with exactly
// the right cadence across jittery task scheduling.
static timeUs_t dronecanLastSecondUs;
static timeUs_t dronecanStartUs;

//-----------------------------------------------------------------------------
// Callbacks handed to libcanard
//-----------------------------------------------------------------------------

static bool dronecanShouldAcceptTransfer(const CanardInstance *ins,
                                         uint64_t *out_data_type_signature,
                                         uint16_t data_type_id,
                                         CanardTransferType transfer_type,
                                         uint8_t source_node_id)
{
    UNUSED(ins);
    UNUSED(source_node_id);

    for (uint8_t i = 0; i < dronecanSubscriberCount; i++) {
        const dronecanSubscriber_t *sub = &dronecanSubscribers[i];
        if (sub->dataTypeId == data_type_id
                && sub->transferType == (uint8_t)transfer_type) {
            *out_data_type_signature = sub->signature;
            return true;
        }
    }

    return false;
}

static void dronecanOnTransferReception(CanardInstance *ins,
                                        CanardRxTransfer *transfer)
{
    for (uint8_t i = 0; i < dronecanSubscriberCount; i++) {
        const dronecanSubscriber_t *sub = &dronecanSubscribers[i];
        if (sub->dataTypeId == transfer->data_type_id
                && sub->transferType == transfer->transfer_type) {
            if (sub->handler) {
                sub->handler(ins, transfer);
            }
            return;
        }
    }
}

//-----------------------------------------------------------------------------
// CAN ISR -> ring buffer bridge
//-----------------------------------------------------------------------------

static void dronecanCanRxAdapter(uint32_t identifier, bool isExtended,
                                 const uint8_t *data, uint8_t length)
{
    uint8_t head = dronecanRxHead;
    uint8_t next = (head + 1U) & DRONECAN_RX_RING_MASK;

    // Drop frames when the ring is full. The task is the only consumer; if
    // it can't keep up, the per-transfer timeouts inside libcanard will let
    // the sender retry on the next period rather than jamming the node.
    if (next == dronecanRxTail) {
        return;
    }

    dronecanRxRingEntry_t *slot = &dronecanRxRing[head];
    slot->id = identifier;
    slot->isExtended = isExtended;
    slot->length = (length > 8U) ? 8U : length;
    for (uint8_t i = 0; i < slot->length; i++) {
        slot->data[i] = data[i];
    }

    // Barrier before advancing the head so the task never observes an
    // incremented head index with a partially-written slot. Cortex-M is
    // strongly ordered at the hardware level but the C standard lets the
    // compiler reorder non-volatile stores around a volatile one, so we
    // need an explicit compiler-side fence.
    __asm volatile ("" ::: "memory");
    dronecanRxHead = next;
}

static void dronecanDrainRxRing(timeUs_t currentTimeUs)
{
    while (dronecanRxTail != dronecanRxHead) {
        // Pair the ISR-side fence on head with a consumer-side fence so
        // the compiler can't hoist slot reads above the head snapshot.
        __asm volatile ("" ::: "memory");
        const dronecanRxRingEntry_t *slot = &dronecanRxRing[dronecanRxTail];

        CanardCANFrame frame;
        memset(&frame, 0, sizeof(frame));
        frame.id = slot->id | (slot->isExtended ? CANARD_CAN_FRAME_EFF : 0U);
        frame.data_len = slot->length;
        memcpy(frame.data, slot->data, slot->length);

        // libcanard stores timestamps as uint64 and uses them for transfer
        // timeout (2s) / stale cleanup. Zero-extending a uint32 micros() is
        // accurate inside the 71-minute wraparound window; the task runs at
        // 50 Hz so every active transfer will be serviced well inside that.
        canardHandleRxFrame(&dronecanInstance, &frame, (uint64_t)currentTimeUs);

        dronecanRxTail = (dronecanRxTail + 1U) & DRONECAN_RX_RING_MASK;
    }
}

//-----------------------------------------------------------------------------
// TX drain: libcanard -> CAN driver
//-----------------------------------------------------------------------------

static void dronecanDrainTxQueue(void)
{
    for (const CanardCANFrame *tx = canardPeekTxQueue(&dronecanInstance);
         tx != NULL;
         tx = canardPeekTxQueue(&dronecanInstance)) {

        const bool isExtended = (tx->id & CANARD_CAN_FRAME_EFF) != 0U;
        const uint32_t pureId = tx->id & CANARD_CAN_EXT_ID_MASK;

        if (!canTransmit(dronecanDevice, pureId, isExtended,
                         tx->data, tx->data_len)) {
            // Hardware FIFO is full. Leave the frame at the head of the
            // queue and try again next tick — do NOT pop or we'd drop it.
            break;
        }

        canardPopTxQueue(&dronecanInstance);
    }
}

//-----------------------------------------------------------------------------
// Public API
//-----------------------------------------------------------------------------

void dronecanInit(void)
{
    if (dronecanInitialised) {
        return;
    }

    if (!dronecanConfig()->enabled) {
        return;
    }

    const uint8_t nodeId = dronecanConfig()->node_id;
    if (nodeId < CANARD_MIN_NODE_ID || nodeId > CANARD_MAX_NODE_ID) {
        // User has not assigned a node ID yet. Staying uninitialised is safer
        // than announcing as anonymous and colliding with other nodes.
        return;
    }

    const uint8_t cfgDevice = dronecanConfig()->device;
    if (cfgDevice < 1U || cfgDevice > CANDEV_COUNT) {
        return;
    }
    dronecanDevice = (canDevice_e)CAN_CFG_TO_DEV(cfgDevice);

    dronecanRxHead = 0;
    dronecanRxTail = 0;
    dronecanSubscriberCount = 0;

    canardInit(&dronecanInstance, dronecanPool, sizeof(dronecanPool),
               dronecanOnTransferReception, dronecanShouldAcceptTransfer,
               NULL);
    canardSetLocalNodeID(&dronecanInstance, nodeId);

    // Wire the node-level publishers/responders (registers the GetNodeInfo
    // subscriber against our table before the first frame can arrive).
    dronecanNodeInit();
    // Install the GNSS Fix2 subscriber so a DroneCAN GPS module's broadcasts
    // land in our cache as soon as the transport is live.
    dronecanGnssInit();

    canRegisterRxCallback(dronecanDevice, dronecanCanRxAdapter);

    dronecanStartUs = micros();
    dronecanLastSecondUs = dronecanStartUs;
    dronecanInitialised = true;
}

bool dronecanIsInitialised(void)
{
    return dronecanInitialised;
}

void dronecanUpdate(timeUs_t currentTimeUs)
{
    if (!dronecanInitialised) {
        return;
    }

    dronecanDrainRxRing(currentTimeUs);

    // 1 Hz boundary — publish NodeStatus and sweep stale RX transfers.
    if (cmpTimeUs(currentTimeUs, dronecanLastSecondUs) >= 1000000) {
        dronecanLastSecondUs = currentTimeUs;
        dronecanNodeUpdate(currentTimeUs);
        canardCleanupStaleTransfers(&dronecanInstance, (uint64_t)currentTimeUs);
    }

    dronecanDrainTxQueue();
}

bool dronecanRegisterSubscriber(const dronecanSubscriber_t *subscriber)
{
    if (dronecanSubscriberCount >= DRONECAN_MAX_SUBSCRIBERS) {
        return false;
    }
    dronecanSubscribers[dronecanSubscriberCount++] = *subscriber;
    return true;
}

CanardInstance *dronecanGetInstance(void)
{
    return &dronecanInstance;
}

#endif // ENABLE_DRONECAN
