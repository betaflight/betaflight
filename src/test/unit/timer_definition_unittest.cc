/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight.  If not, see <http://www.gnu.org/licenses/>.
 */

extern "C" {
    #include <target.h>
    #include <drivers/timer.h>
}

#include <bitset>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include "gtest/gtest.h"

#if !defined(USE_UNIFIED_TARGET)
extern "C" {
    extern const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT];
}

TEST(TimerDefinitionTest, Test_counterMismatch) {
    for (const timerHardware_t &t : timerHardware)
        ASSERT_EQ(&t - timerHardware, t.def_tim_counter)
            << "Counter mismatch in timerHardware (in target.c) at position "
            << &t - timerHardware << "; the array may have uninitialized "
            << "trailing elements. This happens when USABLE_TIMER_CHANNEL_COUNT"
            << " is not equal to the number of array initializers. Current "
            << "value is " << USABLE_TIMER_CHANNEL_COUNT << ", last initialized"
            << " array element appears to be " << &t - timerHardware - 1 << '.';
}

TEST(TimerDefinitionTest, Test_duplicatePin) {
    std::set<TestPinEnum> usedPins;
    for (const timerHardware_t &t : timerHardware)
        EXPECT_TRUE(usedPins.emplace(t.pin).second)
            << "Pin " << TEST_PIN_NAMES[t.pin] << " is used more than once. "
            << "This is a problem with the timerHardware array (in target.c). "
            << "Check the array for typos. Then check the size of the array "
            << "initializers; it must be USABLE_TIMER_CHANNEL_COUNT.";
    EXPECT_EQ(USABLE_TIMER_CHANNEL_COUNT, usedPins.size());
}

#if !defined(USE_TIMER_MGMT)
namespace {
std::string writeUsedTimers(const std::bitset<TEST_TIMER_SIZE> &tset) {
    std::stringstream used_timers;
    if (tset.any()) {
        unsigned int timer{0};
        for (; timer < TEST_TIMER_SIZE; ++timer)
            if (tset[timer]) {
                used_timers << "( TIM_N(" << timer << ')';
                break;
            }
        for (++timer; timer < TEST_TIMER_SIZE; ++timer)
            if (tset[timer]) used_timers << " | TIM_N(" << timer << ')';
        used_timers << " )";
    } else {
        used_timers << "(0)";
    }
    return used_timers.str();
}
}

TEST(TimerDefinitionTest, Test_usedTimers)
{
    std::bitset<TEST_TIMER_SIZE> expected;
    for (const timerHardware_t &t : timerHardware)
        expected |= TIM_N(t.timer);
    const std::bitset<TEST_TIMER_SIZE> actual{USED_TIMERS};
    EXPECT_EQ(expected, actual)
        << "Used timers mismatch. target.c says " << expected << ", but "
        << "target.h says " << actual << ". This has two possible causes: "
        << "(1) The USED_TIMERS bitmap (in target.h) is outdated and out of "
        << "sync with timerHardware (in target.c). (2) There is an "
        << "inconsistency between USABLE_TIMER_CHANNEL_COUNT and the length "
        << "of timerHardware's initializer list.";
    std::cerr
        << "USED_TIMERS definition based on timerHardware:" << std::endl
        << writeUsedTimers(expected) << std::endl;
}
#endif

// STUBS

extern "C" {
    void spiPinConfigure(int) {}
    int spiPinConfig(int) { return 0; }
    void spiInit(int) {}

    int i2cConfig(int) { return 0; }
    void i2cHardwareConfigure(int) {}
    void i2cInit(int) {}

    void bstInit(int) {}
}
#endif // USE_UNIFIED_TARGET
