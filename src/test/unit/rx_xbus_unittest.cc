/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <algorithm>
#include <array>
#include <cstdint>

extern "C" {
#include "platform.h"

#include "common/crc.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "rx/xbus.h"
#include "telemetry/telemetry.h"
}

#include "gtest/gtest.h"

static serialReceiveCallbackPtr receiveCallback;
static serialPort_t serialPort;
static uint32_t currentMicros;

bool telemetryCheckRxPortShared(serialPortIdentifier_e, SerialRXType)
{
    return false;
}

serialPort_t *telemetrySharedPort;

uint32_t micros(void)
{
    return currentMicros;
}

uint32_t microsISR(void)
{
    return currentMicros;
}

serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e,
    serialReceiveCallbackPtr callback, void *, uint32_t, portMode_e, portOptions_e)
{
    receiveCallback = callback;
    return &serialPort;
}

static uint8_t rj01Crc8(uint8_t inData, uint8_t seed)
{
    for (int bit = 0; bit < 8; bit++) {
        if ((seed ^ inData) & 1) {
            seed = (seed >> 1) ^ 0x8c;
        } else {
            seed >>= 1;
        }
        inData >>= 1;
    }
    return seed;
}

static std::array<uint8_t, 27> makeModeBFrame(uint8_t marker = 0xa1)
{
    std::array<uint8_t, 27> frame = {};
    frame[0] = marker;
    for (unsigned channel = 0; channel < 12; channel++) {
        frame[1 + channel * 2] = 0x08;
        frame[2 + channel * 2] = 0x00;
    }
    const uint16_t crc = crc16_ccitt_update(0, frame.data(), frame.size() - 2);
    frame[25] = crc >> 8;
    frame[26] = crc;
    return frame;
}

static std::array<uint8_t, 33> makeRj01Frame()
{
    std::array<uint8_t, 33> frame = {};
    const auto inner = makeModeBFrame();
    frame[0] = 0xa1;
    frame[1] = 30;
    frame[2] = 0;
    std::copy(inner.begin(), inner.end(), frame.begin() + 3);
    frame[30] = 0;
    frame[31] = 0;
    uint8_t crc = 0;
    for (unsigned i = 0; i < frame.size() - 1; i++) {
        crc = rj01Crc8(crc, frame[i]);
    }
    frame[32] = crc;
    return frame;
}

class XBusTest : public ::testing::Test {
protected:
    rxRuntimeState_t runtime = {};

    void init(SerialRXType provider)
    {
        rxConfig_t config = {};
        config.rx_uart = SERIAL_PORT_USART1;
        runtime.serialrxProvider = provider;
        receiveCallback = nullptr;
        currentMicros += 12000;
        ASSERT_TRUE(xBusInit(&config, &runtime));
        ASSERT_NE(receiveCallback, nullptr);
    }

    template <size_t N>
    void send(const std::array<uint8_t, N>& frame, size_t count = N)
    {
        for (size_t i = 0; i < count; i++) {
            currentMicros += 48;
            receiveCallback(frame[i], nullptr);
        }
    }
};

TEST_F(XBusTest, AcceptsCompleteRj01Frame)
{
    init(SERIALRX_XBUS_MODE_B_RJ01);
    send(makeRj01Frame());
    EXPECT_EQ(runtime.rcFrameStatusFn(&runtime), RX_FRAME_COMPLETE);
    for (unsigned channel = 0; channel < 12; channel++) {
        EXPECT_EQ(runtime.rcReadRawFn(&runtime, channel), 1500);
    }
}

TEST_F(XBusTest, RejectsBadInnerAndOuterCrcAndRecovers)
{
    init(SERIALRX_XBUS_MODE_B_RJ01);
    auto frame = makeRj01Frame();
    frame[28] ^= 1;
    uint8_t crc = 0;
    for (unsigned i = 0; i < frame.size() - 1; i++) {
        crc = rj01Crc8(crc, frame[i]);
    }
    frame[32] = crc;
    send(frame);
    EXPECT_EQ(runtime.rcFrameStatusFn(&runtime), RX_FRAME_PENDING);

    frame = makeRj01Frame();
    frame[32] ^= 1;
    send(frame);
    EXPECT_EQ(runtime.rcFrameStatusFn(&runtime), RX_FRAME_PENDING);

    currentMicros += 12000;
    send(makeRj01Frame());
    EXPECT_EQ(runtime.rcFrameStatusFn(&runtime), RX_FRAME_COMPLETE);
}

TEST_F(XBusTest, DoesNotReleasePartialRj01Frame)
{
    init(SERIALRX_XBUS_MODE_B_RJ01);
    const auto frame = makeRj01Frame();
    send(frame, frame.size() - 1);
    EXPECT_EQ(runtime.rcFrameStatusFn(&runtime), RX_FRAME_PENDING);
}

TEST_F(XBusTest, AcceptsNormalModeBFrames)
{
    init(SERIALRX_XBUS_MODE_B);
    send(makeModeBFrame());
    EXPECT_EQ(runtime.rcFrameStatusFn(&runtime), RX_FRAME_COMPLETE);

    std::array<uint8_t, 35> frame = {};
    frame[0] = 0xa2;
    for (unsigned channel = 0; channel < 16; channel++) {
        frame[1 + channel * 2] = 0x08;
    }
    const uint16_t crc = crc16_ccitt_update(0, frame.data(), frame.size() - 2);
    frame[33] = crc >> 8;
    frame[34] = crc;
    currentMicros += 12000;
    send(frame);
    EXPECT_EQ(runtime.rcFrameStatusFn(&runtime), RX_FRAME_COMPLETE);
}
