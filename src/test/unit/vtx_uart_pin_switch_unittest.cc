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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_uart_pin_switch.h"
#include "io/serial.h"
#include "io/vtx.h"
#include "platform/vtx_uart_pin_switch.h"

vtxSettingsConfig_t vtxSettingsConfig_System;
}

namespace {
struct GpioEvent {
    IO_t io;
    bool alternate;
    uint8_t af;
};

std::vector<GpioEvent> events;
IO_t tx = reinterpret_cast<IO_t>(1);
IO_t uartRx = reinterpret_cast<IO_t>(2);
IO_t switchRx;
timeMs_t nowMs;
unsigned clockReads;

class VtxUartPinSwitchTest : public testing::Test {
protected:
    serialPort_t uart1 = {};
    serialPort_t uart2 = {};

    void SetUp() override
    {
        uart1.identifier = SERIAL_PORT_UART1;
        uart2.identifier = SERIAL_PORT_UART2;
        switchRx = reinterpret_cast<IO_t>(3);
        nowMs = 100;
        clockReads = 0;
        vtxSettingsConfig_System = {};
        vtxUartPinSwitchInit(&uart1, SERIAL_NOT_INVERTED, tx, uartRx, 7);
        vtxSettingsConfig_System.vtx_uart = SERIAL_PORT_UART1;
        vtxSettingsConfig_System.vtx_type = VTXDEV_SMARTAUDIO;
        events.clear();
    }

    void bind()
    {
        vtxUartPinSwitchInit(&uart1, SERIAL_BIDIR, tx, uartRx, 7);
    }

    void expectEvent(unsigned index, IO_t io, bool alternate)
    {
        ASSERT_LT(index, events.size());
        EXPECT_EQ(io, events[index].io);
        EXPECT_EQ(alternate, events[index].alternate);
        if (alternate) {
            EXPECT_EQ(7, events[index].af);
        }
    }

    void checkWindow(uint16_t delayMs)
    {
        bind();
        events.clear();
        vtxUartPinSwitchBegin(&uart1, 100, delayMs);
        ASSERT_EQ(2u, events.size());
        expectEvent(0, tx, true);
        expectEvent(1, switchRx, false);
        events.clear();
        nowMs = 100 + delayMs - 1;
        vtxUartPinSwitchUpdate();
        EXPECT_TRUE(events.empty());
        nowMs++;
        vtxUartPinSwitchUpdate();
        ASSERT_EQ(2u, events.size());
        expectEvent(0, tx, false);
        expectEvent(1, switchRx, true);
        events.clear();
        const unsigned readsAfterSwitch = clockReads;
        nowMs++;
        vtxUartPinSwitchUpdate();
        EXPECT_TRUE(events.empty());
        EXPECT_EQ(readsAfterSwitch, clockReads);
    }
};

TEST_F(VtxUartPinSwitchTest, UnboundPortsDoNotTouchGpioOrReadClock)
{
    vtxUartPinSwitchBegin(nullptr, 100, 25);
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    vtxUartPinSwitchUpdate();
    EXPECT_TRUE(events.empty());
    EXPECT_EQ(0u, clockReads);
}

TEST_F(VtxUartPinSwitchTest, InitializationPreservesFloatingPinOrder)
{
    bind();
    ASSERT_EQ(3u, events.size());
    expectEvent(0, tx, false);
    expectEvent(1, uartRx, false);
    expectEvent(2, switchRx, false);
}

TEST_F(VtxUartPinSwitchTest, SmartAudioReturnsToRxOnceAt25Ms)
{
    checkWindow(25);
}

TEST_F(VtxUartPinSwitchTest, TrampReturnsToRxOnceAt20Ms)
{
    vtxSettingsConfig_System.vtx_type = VTXDEV_TRAMP;
    checkWindow(20);
}

TEST_F(VtxUartPinSwitchTest, LatePollStillReturnsToRx)
{
    bind();
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    events.clear();
    nowMs = 200;
    vtxUartPinSwitchUpdate();
    ASSERT_EQ(2u, events.size());
    expectEvent(0, tx, false);
    expectEvent(1, switchRx, true);
}

TEST_F(VtxUartPinSwitchTest, RepeatedBeginRestartsWindow)
{
    bind();
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    vtxUartPinSwitchBegin(&uart1, 110, 25);
    events.clear();
    nowMs = 134;
    vtxUartPinSwitchUpdate();
    EXPECT_TRUE(events.empty());
    nowMs = 135;
    vtxUartPinSwitchUpdate();
    ASSERT_EQ(2u, events.size());
    expectEvent(1, switchRx, true);
}

TEST_F(VtxUartPinSwitchTest, TimeoutWorksAcrossMillisWrap)
{
    bind();
    vtxUartPinSwitchBegin(&uart1, UINT32_MAX - 15, 25);
    events.clear();
    nowMs = 8;
    vtxUartPinSwitchUpdate();
    EXPECT_TRUE(events.empty());
    nowMs = 9;
    vtxUartPinSwitchUpdate();
    ASSERT_EQ(2u, events.size());
    expectEvent(1, switchRx, true);
}

TEST_F(VtxUartPinSwitchTest, OtherBidirUartCannotOverwriteOrUseBinding)
{
    bind();
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    events.clear();
    vtxUartPinSwitchInit(&uart2, SERIAL_BIDIR, reinterpret_cast<IO_t>(4), uartRx, 8);
    vtxUartPinSwitchBegin(&uart2, 110, 20);
    EXPECT_TRUE(events.empty());
    nowMs = 125;
    vtxUartPinSwitchUpdate();
    ASSERT_EQ(2u, events.size());
    expectEvent(0, tx, false);
    expectEvent(1, switchRx, true);
}

TEST_F(VtxUartPinSwitchTest, ReopeningFullDuplexCancelsPendingSwitch)
{
    bind();
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    events.clear();
    vtxUartPinSwitchInit(&uart1, SERIAL_NOT_INVERTED, tx, uartRx, 7);
    nowMs = 200;
    vtxUartPinSwitchUpdate();
    vtxUartPinSwitchBegin(&uart1, 200, 20);
    EXPECT_TRUE(events.empty());
}

TEST_F(VtxUartPinSwitchTest, OtherVtxPortAndProtocolDoNotEnableSpecialPins)
{
    vtxSettingsConfig_System.vtx_uart = SERIAL_PORT_UART2;
    bind();
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    EXPECT_TRUE(events.empty());
    vtxSettingsConfig_System.vtx_uart = SERIAL_PORT_UART1;
    vtxSettingsConfig_System.vtx_type = VTXDEV_MSP;
    bind();
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    EXPECT_TRUE(events.empty());
}

TEST_F(VtxUartPinSwitchTest, MissingOrOverlappingPinsDoNotEnableSwitch)
{
    vtxUartPinSwitchInit(&uart1, SERIAL_BIDIR, IO_NONE, uartRx, 7);
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    switchRx = IO_NONE;
    bind();
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    switchRx = tx;
    bind();
    vtxUartPinSwitchBegin(&uart1, 100, 25);
    EXPECT_TRUE(events.empty());
}
} // namespace

extern "C" {
IO_t IOGetByTag(ioTag_t tag)
{
    return tag == IO_TAG(PB7) ? switchRx : IO_NONE;
}

void IOConfigGPIO(IO_t io, ioConfig_t)
{
    events.push_back({ io, false, 0 });
}

void IOConfigGPIOAF(IO_t io, ioConfig_t, uint8_t af)
{
    events.push_back({ io, true, af });
}

timeMs_t millis(void)
{
    clockReads++;
    return nowMs;
}
}
