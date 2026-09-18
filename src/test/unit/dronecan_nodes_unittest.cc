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

#include <cstdint>
#include <cstring>

extern "C" {
    #include "platform.h"

    #include "canard.h"

    #include "io/dronecan/dronecan.h"
    #include "io/dronecan/dronecan_msg.h"
    #include "io/dronecan/dronecan_nodes.h"

    // dronecanRegisterSubscriber() lives in dronecan.c, which is not linked
    // into this test. Capture the handlers the module registers so the tests
    // can feed hand-built transfers directly.
    static dronecanRxHandler capturedStatusHandler = nullptr;
    static dronecanRxHandler capturedInfoHandler = nullptr;

    bool dronecanRegisterSubscriber(const dronecanSubscriber_t *subscriber)
    {
        if (subscriber->dataTypeId == UAVCAN_NODE_STATUS_ID
                && subscriber->transferType == CanardTransferTypeBroadcast) {
            capturedStatusHandler = subscriber->handler;
        } else if (subscriber->dataTypeId == UAVCAN_GET_NODE_INFO_ID
                && subscriber->transferType == CanardTransferTypeResponse) {
            capturedInfoHandler = subscriber->handler;
        }
        return true;
    }

    // Real libcanard instance so the module's canardRequestOrRespondObj()
    // calls enqueue genuine frames the tests can count and inspect.
    static CanardInstance testInstance;
    static uint8_t testPool[2048];

    static bool testShouldAccept(const CanardInstance *, uint64_t *, uint16_t,
                                 CanardTransferType, uint8_t)
    {
        return false;
    }

    static void testOnReception(CanardInstance *, CanardRxTransfer *)
    {
    }

    CanardInstance *dronecanGetInstance(void)
    {
        return &testInstance;
    }

    // Controllable clock; the module timestamps NodeStatus receptions with it.
    static timeUs_t mockMicros = 0;

    timeUs_t micros(void)
    {
        return mockMicros;
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static void feedNodeStatus(uint8_t sourceNodeId, uint32_t uptimeSec,
                           uint8_t health, uint8_t mode)
{
    static uint8_t payload[UAVCAN_NODE_STATUS_PAYLOAD_LEN];
    const uint8_t subMode = 0;
    const uint16_t vendorCode = 0;
    memset(payload, 0, sizeof(payload));
    canardEncodeScalar(payload, 0, 32, &uptimeSec);
    canardEncodeScalar(payload, 32, 2, &health);
    canardEncodeScalar(payload, 34, 3, &mode);
    canardEncodeScalar(payload, 37, 3, &subMode);
    canardEncodeScalar(payload, 40, 16, &vendorCode);

    CanardRxTransfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.data_type_id = UAVCAN_NODE_STATUS_ID;
    transfer.source_node_id = sourceNodeId;
    transfer.payload_head = payload;
    transfer.payload_len = sizeof(payload);

    capturedStatusHandler(nullptr, &transfer);
}

// GetNodeInfo.Response: NodeStatus (7) + SoftwareVersion (15) +
// HardwareVersion (18 + COA length byte + COA) + TAO name to payload end.
static void feedNodeInfoResponse(uint8_t sourceNodeId, uint8_t coaLen,
                                 const char *name)
{
    static uint8_t payload[160];
    memset(payload, 0, sizeof(payload));
    payload[40] = coaLen;
    const size_t nameLen = strlen(name);
    memcpy(&payload[41 + coaLen], name, nameLen);

    CanardRxTransfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.transfer_type = CanardTransferTypeResponse;
    transfer.data_type_id = UAVCAN_GET_NODE_INFO_ID;
    transfer.source_node_id = sourceNodeId;
    transfer.payload_head = payload;
    transfer.payload_len = (uint16_t)(41 + coaLen + nameLen);

    capturedInfoHandler(nullptr, &transfer);
}

// Drain the instance's TX queue, returning how many frames were queued and
// the destination node id of the first one (service frames carry it in CAN
// id bits 8..14).
static int drainTxQueue(uint8_t *firstDestination = nullptr)
{
    int frames = 0;
    for (const CanardCANFrame *frame = canardPeekTxQueue(&testInstance);
         frame != NULL;
         frame = canardPeekTxQueue(&testInstance)) {
        if (frames == 0 && firstDestination) {
            *firstDestination = (uint8_t)((frame->id >> 8) & 0x7FU);
        }
        frames++;
        canardPopTxQueue(&testInstance);
    }
    return frames;
}

static const dronecanNodeEntry_t *findEntry(uint8_t nodeId)
{
    for (uint8_t i = 0; i < dronecanNodesCount(); i++) {
        const dronecanNodeEntry_t *node = dronecanNodesGet(i);
        if (node->nodeId == nodeId) {
            return node;
        }
    }
    return nullptr;
}

class DronecanNodesTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mockMicros = 0;
        capturedStatusHandler = nullptr;
        capturedInfoHandler = nullptr;
        canardInit(&testInstance, testPool, sizeof(testPool),
                   testOnReception, testShouldAccept, NULL);
        canardSetLocalNodeID(&testInstance, 10);
        dronecanNodesInit();
        ASSERT_NE(capturedStatusHandler, nullptr);
        ASSERT_NE(capturedInfoHandler, nullptr);
    }
};

TEST_F(DronecanNodesTest, EmptyUntilFirstFrame)
{
    EXPECT_EQ(0, dronecanNodesCount());
    EXPECT_EQ(nullptr, dronecanNodesGet(0));

    dronecanNodesUpdate(1000000);
    EXPECT_EQ(0, drainTxQueue());
}

TEST_F(DronecanNodesTest, NodeStatusCreatesAndDecodes)
{
    mockMicros = 5000000;
    feedNodeStatus(42, 123, UAVCAN_NODE_HEALTH_WARNING, UAVCAN_NODE_MODE_INITIALIZATION);

    ASSERT_EQ(1, dronecanNodesCount());
    const dronecanNodeEntry_t *node = findEntry(42);
    ASSERT_NE(nullptr, node);
    EXPECT_EQ(123U, node->uptimeSec);
    EXPECT_EQ(UAVCAN_NODE_HEALTH_WARNING, node->health);
    EXPECT_EQ(UAVCAN_NODE_MODE_INITIALIZATION, node->mode);
    EXPECT_EQ(5000000U, node->lastHeardUs);
    EXPECT_FALSE(node->infoValid);
}

TEST_F(DronecanNodesTest, AnonymousNodeStatusIgnored)
{
    feedNodeStatus(0, 1, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);
    EXPECT_EQ(0, dronecanNodesCount());
}

TEST_F(DronecanNodesTest, NoteSensorCreatesEntryAndMergesWithStatus)
{
    mockMicros = 2000000;
    dronecanNodesNoteSensor(50, DRONECAN_NODE_SENSOR_GPS);
    dronecanNodesNoteSensor(50, DRONECAN_NODE_SENSOR_MAG);

    ASSERT_EQ(1, dronecanNodesCount());
    const dronecanNodeEntry_t *node = findEntry(50);
    ASSERT_NE(nullptr, node);
    EXPECT_EQ(DRONECAN_NODE_SENSOR_GPS | DRONECAN_NODE_SENSOR_MAG, node->sensorFlags);
    EXPECT_EQ(2000000U, node->lastHeardUs);

    feedNodeStatus(50, 7, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);
    EXPECT_EQ(1, dronecanNodesCount());
    EXPECT_EQ(7U, node->uptimeSec);
}

TEST_F(DronecanNodesTest, SingleOutstandingInfoRequest)
{
    feedNodeStatus(20, 1, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);
    feedNodeStatus(21, 1, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);

    uint8_t destination = 0;
    dronecanNodesUpdate(1000000);
    EXPECT_EQ(1, drainTxQueue(&destination));
    EXPECT_EQ(20, destination);

    // Within the 2 s response window no further request goes out.
    dronecanNodesUpdate(2000000);
    EXPECT_EQ(0, drainTxQueue());

    // After the timeout the scheduler moves on (same node retried first).
    dronecanNodesUpdate(3100000);
    EXPECT_EQ(1, drainTxQueue(&destination));
    EXPECT_EQ(20, destination);
}

TEST_F(DronecanNodesTest, ResponseStoresNameAndFreesScheduler)
{
    feedNodeStatus(20, 1, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);
    feedNodeStatus(21, 1, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);

    dronecanNodesUpdate(1000000);
    EXPECT_EQ(1, drainTxQueue());

    feedNodeInfoResponse(20, 0, "org.test.device");
    const dronecanNodeEntry_t *node = findEntry(20);
    ASSERT_NE(nullptr, node);
    EXPECT_TRUE(node->infoValid);
    EXPECT_STREQ("org.test.device", node->name);

    // The pending slot is free, so the next update queries the other node
    // without waiting for the timeout.
    uint8_t destination = 0;
    dronecanNodesUpdate(2000000);
    EXPECT_EQ(1, drainTxQueue(&destination));
    EXPECT_EQ(21, destination);
}

TEST_F(DronecanNodesTest, ResponseNameFollowsCertificateOfAuthenticity)
{
    feedNodeStatus(20, 1, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);
    feedNodeInfoResponse(20, 4, "org.coa.device");

    const dronecanNodeEntry_t *node = findEntry(20);
    ASSERT_NE(nullptr, node);
    EXPECT_TRUE(node->infoValid);
    EXPECT_STREQ("org.coa.device", node->name);
}

TEST_F(DronecanNodesTest, LongNameTruncated)
{
    feedNodeStatus(20, 1, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);
    feedNodeInfoResponse(20, 0,
        "org.example.a-very-long-node-name-that-exceeds-the-cli-limit");

    const dronecanNodeEntry_t *node = findEntry(20);
    ASSERT_NE(nullptr, node);
    EXPECT_TRUE(node->infoValid);
    EXPECT_EQ(DRONECAN_NODE_NAME_MAX, strlen(node->name));
    EXPECT_STREQ("org.example.a-very-long-node-nam", node->name);
}

TEST_F(DronecanNodesTest, InfoRequestsStopAfterRetryLimit)
{
    feedNodeStatus(20, 1, UAVCAN_NODE_HEALTH_OK, UAVCAN_NODE_MODE_OPERATIONAL);

    timeUs_t now = 1000000;
    int requests = 0;
    for (int i = 0; i < 6; i++) {
        dronecanNodesUpdate(now);
        requests += drainTxQueue();
        now += 2500000;
    }
    EXPECT_EQ(3, requests);

    const dronecanNodeEntry_t *node = findEntry(20);
    ASSERT_NE(nullptr, node);
    EXPECT_FALSE(node->infoValid);
}
