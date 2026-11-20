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

#include <stdint.h>
#include <string.h>

#include <vector>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"
    #include "common/time.h"
    #include "drivers/serial.h"
    #include "io/gimbal_control.h"
    #include "io/serial.h"
    #include "pg/gimbal.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "rx/rx.h"
    #include "scheduler/scheduler.h"

    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static serialPort_t gimbalPort;
static std::vector<uint8_t> rxBytes;
static size_t rxIndex;
static std::vector<uint8_t> txBytes;

extern "C" {
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e)
    {
        return &gimbalPort;
    }

    uint32_t serialRxBytesWaiting(const serialPort_t *)
    {
        return rxBytes.size() - rxIndex;
    }

    uint8_t serialRead(serialPort_t *)
    {
        return rxBytes[rxIndex++];
    }

    void serialWrite(serialPort_t *, uint8_t ch)
    {
        txBytes.push_back(ch);
    }

    void serialWriteBuf(serialPort_t *, const uint8_t *data, int count)
    {
        txBytes.insert(txBytes.end(), data, data + count);
    }

    void setTaskEnabled(taskId_e, bool) {}
}

class GimbalControlTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        gimbalTrackConfigMutable()->gimbal_uart = SERIAL_PORT_USART2;
        ASSERT_TRUE(gimbalInit());
        rxBytes.clear();
        rxIndex = 0;
        txBytes.clear();
    }
};

TEST_F(GimbalControlTest, PassesThroughAFirstHeaderByteThatIsNotFollowedByTheSecond)
{
    // 0xA5 is the first byte of the command header (0x5AA5, little endian). 0x12 is not 0x5A.
    rxBytes = { 0xA5, 0x12 };

    gimbalUpdate(1000);

    EXPECT_EQ(std::vector<uint8_t>({ 0xA5, 0x12 }), txBytes);
}

TEST_F(GimbalControlTest, PassesThroughBytesThatDoNotStartAHeader)
{
    rxBytes = { 0x12, 0x34 };

    gimbalUpdate(1000);

    EXPECT_EQ(std::vector<uint8_t>({ 0x12, 0x34 }), txBytes);
}
