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
    #include "drivers/can/can.h"
    #include "io/dronecan/dronecan.h"
    #include "io/dronecan/dronecan_msg.h"
    #include "pg/dronecan.h"

    dronecanConfig_t dronecanConfig_System;
    static timeUs_t mockMicros;
    static canRxCallbackPtr receiveFrame;
    static unsigned transfers;
    static uint8_t received[14];

    timeUs_t micros(void)
    {
        return mockMicros;
    }
    void dronecanNodeInit(void)
    {
    }
    void dronecanNodeUpdate(timeUs_t)
    {
    }
    void dronecanNodesInit(void)
    {
    }
    void dronecanNodesUpdate(timeUs_t)
    {
    }
    void dronecanGnssInit(void)
    {
    }
    void dronecanMagInit(void)
    {
    }
    bool canTransmit(canDevice_e, uint32_t, bool, const uint8_t *, uint8_t)
    {
        return true;
    }
    void canRegisterRxCallback(canDevice_e, canRxCallbackPtr callback)
    {
        receiveFrame = callback;
    }

    static void onStatus(CanardInstance *, CanardRxTransfer *transfer)
    {
        transfers++;
        for (unsigned i = 0; i < sizeof(received); i++) {
            canardDecodeScalar(transfer, i * 8, 8, false, &received[i]);
        }
    }
}

#include "gtest/gtest.h"

TEST(DronecanTransportTest, ReceivesMultiframeStatusAcrossRepeatedTimerWraps)
{
    dronecanConfig_System.enabled = 1;
    dronecanConfig_System.node_id = 127;
    dronecanConfig_System.device = 1;
    mockMicros = 1000;
    dronecanInit();
    ASSERT_TRUE(dronecanIsInitialised());
    ASSERT_NE(nullptr, receiveFrame);
    const dronecanSubscriber_t subscriber = {
        UAVCAN_ESC_STATUS_SIGNATURE, UAVCAN_ESC_STATUS_ID,
        CanardTransferTypeBroadcast, onStatus,
    };
    ASSERT_TRUE(dronecanRegisterSubscriber(&subscriber));
    const uint8_t payload[14] = {0, 0, 0, 0, 0x20, 0x4e, 0, 0x41, 0xb0, 0x5c, 0xd0, 0x7e, 0x64, 0x80};
    const uint8_t frames[3][8] = {
        {0xb3, 0x8e, 0, 0, 0, 0, 0x20, 0x87},
        {0x4e, 0, 0x41, 0xb0, 0x5c, 0xd0, 0x7e, 0x27},
        {0x64, 0x80, 0x47},
    };
    uint64_t now = 1000;
    auto tick = [&](uint64_t target) {
        while (now < target) {
            now += (target - now > 20000) ? 20000 : target - now;
            mockMicros = static_cast<timeUs_t>(now);
            dronecanUpdate(mockMicros);
        }
    };
    auto inject = [&](unsigned index, uint8_t tid) {
        uint8_t frame[8];
        memcpy(frame, frames[index], sizeof(frame));
        const uint8_t length = index == 2 ? 3 : 8;
        frame[length - 1] = (frame[length - 1] & 0xe0) | tid;
        receiveFrame(0x08040a2a, true, frame, length);
        dronecanUpdate(mockMicros);
    };
    for (unsigned wrap = 1; wrap <= 2; wrap++) {
        const uint64_t boundary = static_cast<uint64_t>(wrap) << 32;
        tick(boundary - 30000);
        inject(0, wrap + 6);
        tick(boundary - 10000);
        inject(1, wrap + 6);
        tick(boundary + 10000);
        inject(2, wrap + 6);
        EXPECT_EQ(wrap, transfers);
        EXPECT_EQ(0, memcmp(payload, received, sizeof(payload)));
    }
    inject(0, 9);
    EXPECT_GT(canardGetPoolAllocatorStatistics(dronecanGetInstance()).current_usage_blocks, 0);
    tick(now + 3100000);
    EXPECT_EQ(0, canardGetPoolAllocatorStatistics(dronecanGetInstance()).current_usage_blocks);
    EXPECT_EQ(2U, transfers);
}
