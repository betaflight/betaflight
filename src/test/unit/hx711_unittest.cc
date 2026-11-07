/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

extern "C" {

#include "platform.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/hx711.h"
#include "pg/hx711.h"

}

#include "gtest/gtest.h"

extern "C" {

uint32_t testPrimask;
hx711Config_t hx711Config_System = {};
hx711Config_t hx711Config_Copy = {};

static uint32_t now;
static uint32_t bits;
static unsigned pulses;
static unsigned reads;
static unsigned owners[3];
static bool ready;
static bool high;
static bool stuckLow;

static unsigned ioIndex(IO_t io)
{
    return static_cast<unsigned>(reinterpret_cast<uintptr_t>(io));
}

uint32_t millis(void)
{
    return now;
}

uint32_t micros(void)
{
    return now * 1000;
}

void delayMicroseconds(timeUs_t us)
{
    EXPECT_EQ(1u, us);
    if (high) {
        EXPECT_EQ(1u, testPrimask);
    }
}

IO_t IOGetByTag(ioTag_t tag)
{
    return tag ? reinterpret_cast<IO_t>(static_cast<uintptr_t>(tag)) : nullptr;
}

bool IOIsFreeOrPreinit(IO_t io)
{
    return !owners[ioIndex(io)];
}

void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
    (void)index;
    owners[ioIndex(io)] = static_cast<unsigned>(owner);
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    (void)io;
    (void)cfg;
}

void IOHi(IO_t io)
{
    EXPECT_EQ(1u, ioIndex(io));
    EXPECT_EQ(1u, testPrimask);
    high = true;
    pulses++;
}

void IOLo(IO_t io)
{
    EXPECT_EQ(1u, ioIndex(io));
    high = false;
    if (pulses == 25) {
        ready = false;
    }
}

bool IORead(IO_t io)
{
    EXPECT_EQ(2u, ioIndex(io));
    if (stuckLow) {
        return false;
    }
    if (!high) {
        return !ready;
    }
    EXPECT_LT(reads, 24u);
    return !!(bits & (1u << (23 - reads++)));
}

}

static hx711Sample_t sample(void)
{
    hx711Sample_t value;
    hx711GetSample(&value);
    return value;
}

static void feed(int32_t value)
{
    const uint32_t primask = testPrimask;
    bits = value & 0xffffff;
    ready = true;
    pulses = 0;
    reads = 0;
    hx711Update();
    EXPECT_EQ(25u, pulses);
    EXPECT_EQ(24u, reads);
    EXPECT_FALSE(high);
    EXPECT_EQ(primask, testPrimask);
    EXPECT_EQ(value, sample().raw);
}

TEST(Hx711Test, HandlesResourcesPulsesSaturationAndFreshness)
{
    hx711Init();
    EXPECT_EQ(0u, sample().flags & HX711_CONFIGURED);

    hx711Config_System.sckTag = 1;
    hx711Config_System.doutTag = 1;
    hx711Init();
    EXPECT_EQ(0u, sample().flags & HX711_CONFIGURED);

    hx711Config_System.doutTag = 2;
    owners[1] = 99;
    hx711Init();
    EXPECT_EQ(0u, sample().flags & HX711_CONFIGURED);

    owners[1] = 0;
    hx711Init();
    EXPECT_EQ(HX711_CONFIGURED, sample().flags);
    EXPECT_EQ(UINT32_MAX, sample().ageMs);

    hx711Update();
    EXPECT_EQ(0u, pulses);

    now = 100;
    feed(0);
    EXPECT_EQ(1u, sample().sequence);
    EXPECT_EQ(100000u, sample().timeUs);

    feed(-1);
    feed(-8388608);
    EXPECT_NE(0u, sample().flags & HX711_SATURATED);
    feed(8388607);
    EXPECT_NE(0u, sample().flags & HX711_SATURATED);
    feed(-42000);
    EXPECT_EQ(0u, sample().flags & HX711_SATURATED);

    const uint32_t stuckSequence = sample().sequence;
    stuckLow = true;
    ready = true;
    pulses = 0;
    reads = 0;
    hx711Update();
    stuckLow = false;
    EXPECT_EQ(25u, pulses);
    EXPECT_EQ(0u, reads);
    EXPECT_EQ(stuckSequence, sample().sequence);
    EXPECT_EQ(-42000, sample().raw);

    now += 250;
    EXPECT_NE(0u, sample().flags & HX711_FRESH);
    now++;
    EXPECT_EQ(0u, sample().flags & HX711_FRESH);

    const uint32_t sequence = sample().sequence;
    for (unsigned i = 0; i < 80; i++) {
        now += 13;
        feed(100 + static_cast<int32_t>(i));
    }
    for (unsigned i = 0; i < 10; i++) {
        now += 100;
        feed(-100 - static_cast<int32_t>(i));
    }
    EXPECT_EQ(sequence + 90, sample().sequence);

    now = UINT32_MAX - 100;
    feed(10);
    now = 149;
    EXPECT_EQ(250u, sample().ageMs);
    EXPECT_NE(0u, sample().flags & HX711_FRESH);
    now++;
    EXPECT_EQ(0u, sample().flags & HX711_FRESH);

    testPrimask = 1;
    feed(42);
    EXPECT_EQ(1u, testPrimask);
}
