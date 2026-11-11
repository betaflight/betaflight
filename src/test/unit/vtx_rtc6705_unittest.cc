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

#include <cstdint>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "drivers/bus_spi.h"
    #include "drivers/io.h"
    #include "drivers/time.h"
    #include "drivers/vtx_rtc6705.h"
    #include "drivers/vtx_rtc6705_soft_spi.h"
    #include "pg/vtx_io.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// External power pins carry a 2-bit code: pin 1 = bit 0, pin 2 = bit 1.
// 00 = lowest power, 01 = mid, 10 = max, 11 = undefined and unsafe.
// Register 0x07 values come from the Richwave RTC6705 datasheet: default 0x4FBD,
// low setting clears PA5G_PW and PA5G_BS and sets PD_Q5G, which gives 0x407D.
namespace {

constexpr ioTag_t TAG_CS = 3;
constexpr ioTag_t TAG_PIN1 = 1;
constexpr ioTag_t TAG_PIN2 = 2;
constexpr uint32_t REG_LOW = 0x407D;
constexpr uint32_t REG_DEFAULT = 0x4FBD;

bool pinState[8];
struct PinWrite {
    uint8_t tag;
    bool value;
    bool pin1After;
    bool pin2After;
};
std::vector<PinWrite> writes;
uint32_t lastReg;
int regWrites;

void recordWrite(IO_t io, bool value)
{
    const uint8_t tag = (uint8_t)(uintptr_t)io;
    pinState[tag] = value;
    writes.push_back({ tag, value, pinState[TAG_PIN1], pinState[TAG_PIN2] });
}

} // namespace

extern "C" {

IO_t IOGetByTag(ioTag_t tag) { return tag ? (IO_t)(uintptr_t)tag : NULL; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
void IOWrite(IO_t io, bool value) { recordWrite(io, value); }
void IOHi(IO_t io) { recordWrite(io, true); }
void IOLo(IO_t io) { recordWrite(io, false); }

spiResource_t *spiInstanceByDevice(spiDevice_e) { return NULL; }
bool spiSetBusInstance(extDevice_t *, uint32_t) { return false; }
uint16_t spiCalculateDivider(uint32_t) { return 0; }
void spiSetClkDivisor(const extDevice_t *, uint16_t) {}
void spiReadWriteBuf(const extDevice_t *, uint8_t *, uint8_t *, int) {}
void delayMicroseconds(timeUs_t) {}

bool rtc6705SoftSpiIOInit(const vtxIOConfig_t *, const IO_t) { return true; }
void rtc6705SoftSpiSetFrequency(uint16_t) {}
void rtc6705SoftSpiSetRFPower(uint32_t data) { lastReg = data; regWrites++; }

}

class Rtc6705DynamicPowerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(pinState, 0, sizeof(pinState));
        writes.clear();
        lastReg = 0;
        regWrites = 0;

        vtxIOConfig_t config = {};
        config.csTag = TAG_CS;
        config.exPowerTag[0] = TAG_PIN1;
        config.exPowerTag[1] = TAG_PIN2;
        ASSERT_TRUE(rtc6705IOInit(&config));
    }

    // Set a power value and check the resulting pin code and register.
    void expectLevel(uint8_t value, bool pin1, bool pin2, uint32_t reg)
    {
        rtc6705SetRFPower(value);
        EXPECT_EQ(pin1, pinState[TAG_PIN1]) << "value " << (int)value;
        EXPECT_EQ(pin2, pinState[TAG_PIN2]) << "value " << (int)value;
        EXPECT_EQ(reg, lastReg) << "value " << (int)value;
    }
};

TEST_F(Rtc6705DynamicPowerTest, PinsStartLowAtInit)
{
    EXPECT_FALSE(pinState[TAG_PIN1]);
    EXPECT_FALSE(pinState[TAG_PIN2]);
}

TEST_F(Rtc6705DynamicPowerTest, EachLevelSetsItsCodeAndRegister)
{
    expectLevel(0, false, false, REG_LOW);
    expectLevel(1, true, false, REG_LOW);
    expectLevel(2, false, true, REG_DEFAULT);
}

TEST_F(Rtc6705DynamicPowerTest, ValuesAboveTopLevelNeverGiveCode11)
{
    for (uint8_t value : { 3, 4, 100, 255 }) {
        expectLevel(value, false, true, REG_DEFAULT);
    }
}

TEST_F(Rtc6705DynamicPowerTest, LevelChangesNeverPassThroughCode11)
{
    const uint8_t values[] = { 0, 1, 2, 255 };
    for (uint8_t from : values) {
        for (uint8_t to : values) {
            writes.clear();
            rtc6705SetRFPower(from);
            rtc6705SetRFPower(to);
            for (const PinWrite &w : writes) {
                EXPECT_FALSE(w.pin1After && w.pin2After)
                    << "code 11 after a write, from " << (int)from << " to " << (int)to;
            }
        }
    }
}

TEST_F(Rtc6705DynamicPowerTest, EveryCallWritesTheRegister)
{
    rtc6705SetRFPower(1);
    rtc6705SetRFPower(2);
    rtc6705SetRFPower(2);
    EXPECT_EQ(3, regWrites);
}
