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

#include <algorithm>

extern "C" {
#include "platform.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
}

#include "gtest/gtest.h"

namespace {

uint8_t *pendingPayload[I2CDEV_COUNT];

class I2cBusDeviceTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        std::fill_n(pendingPayload, I2CDEV_COUNT, nullptr);
    }
};

TEST_F(I2cBusDeviceTest, KeepsQueuedWritesOnDifferentControllersIndependent)
{
    extDevice_t first{}, second{}, third{};
    ASSERT_TRUE(i2cBusSetInstance(&first, 1));
    ASSERT_TRUE(i2cBusSetInstance(&second, 2));
    ASSERT_TRUE(i2cBusSetInstance(&third, 3));

    ASSERT_TRUE(i2cBusWriteRegisterStart(&first, 0x10, 0xA5));
    ASSERT_TRUE(i2cBusWriteRegisterStart(&second, 0x20, 0x5A));
    ASSERT_TRUE(i2cBusWriteRegisterStart(&third, 0x30, 0x97));

    // Read the payloads only after all three writes have been queued, as the IRQ would.
    EXPECT_EQ(0xA5, *pendingPayload[I2CDEV_1]);
    EXPECT_EQ(0x5A, *pendingPayload[I2CDEV_2]);
    EXPECT_EQ(0x97, *pendingPayload[I2CDEV_3]);
}

TEST_F(I2cBusDeviceTest, PreservesQueuedPayloadWhileControllerIsBusy)
{
    extDevice_t dev{};
    ASSERT_TRUE(i2cBusSetInstance(&dev, 1));
    ASSERT_TRUE(i2cBusWriteRegisterStart(&dev, 0x10, 0xA5));

    EXPECT_FALSE(i2cBusWriteRegisterStart(&dev, 0x20, 0xEE));
    EXPECT_EQ(0xA5, *pendingPayload[I2CDEV_1]);
}

TEST_F(I2cBusDeviceTest, RejectsInvalidController)
{
    busDevice_t bus{};
    extDevice_t dev{};
    dev.bus = &bus;
    bus.busType_u.i2c.device = I2CINVALID;
    EXPECT_FALSE(i2cBusWriteRegisterStart(&dev, 0x10, 0xA5));

    bus.busType_u.i2c.device = static_cast<i2cDevice_e>(I2CDEV_COUNT);
    EXPECT_FALSE(i2cBusWriteRegisterStart(&dev, 0x10, 0xA5));
}

} // namespace

extern "C" {

bool i2cBusy(i2cDevice_e device, bool *)
{
    if (static_cast<unsigned>(device) >= I2CDEV_COUNT) {
        ADD_FAILURE() << "Invalid controller passed to i2cBusy";
        return true;
    }
    return pendingPayload[device] != nullptr;
}

bool i2cWriteBuffer(i2cDevice_e device, uint8_t, uint8_t, uint8_t length, uint8_t *data)
{
    EXPECT_EQ(1, length);
    pendingPayload[device] = data;
    return true;
}

bool i2cWrite(i2cDevice_e, uint8_t, uint8_t, uint8_t)
{
    return false;
}

bool i2cRead(i2cDevice_e, uint8_t, uint8_t, uint8_t, uint8_t *)
{
    return false;
}

bool i2cReadBuffer(i2cDevice_e, uint8_t, uint8_t, uint8_t, uint8_t *)
{
    return false;
}

} // extern "C"
