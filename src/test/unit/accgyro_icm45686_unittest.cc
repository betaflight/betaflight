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

#include <array>
#include <cstring>
#include <deque>

extern "C" {
#include "platform.h"
#include "drivers/bus.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro_icm456xx.h"
}

#include "gtest/gtest.h"

namespace {
using Packet = std::array<uint8_t, ICM45686_FIFO_PACKET_BYTES>;
uint64_t elapsedNs;
bool stoppedClock;

void put16(uint8_t *data, uint16_t value, bool bigEndian)
{
    data[bigEndian ? 1 : 0] = value;
    data[bigEndian ? 0 : 1] = value >> 8;
}

Packet packet(uint16_t tick, bool bigEndian = false)
{
    Packet data{};
    data[0] = 0x68;
    data[13] = 8;
    put16(data.data() + 1, static_cast<uint16_t>(-1234), bigEndian);
    put16(data.data() + 3, 32767, bigEndian);
    put16(data.data() + 5, 1024, bigEndian);
    put16(data.data() + 7, static_cast<uint16_t>(-16), bigEndian);
    put16(data.data() + 14, tick, bigEndian);
    return data;
}

struct RegisterBus {
    std::array<uint8_t, 128> registers{};
    std::deque<Packet> packets;
    uint16_t indirectAddress = 0;
    uint8_t smc = 0x14;
    uint8_t endian = 0;
    unsigned writes = 0;
    unsigned configurationWrites = 0;
    unsigned failWrite = 0;
    unsigned busyReads = 0;
    unsigned countReads = 0;
    unsigned dataReads = 0;
    int failRead = -1;
    bool busyForever = false;
    bool fullAfterRead = false;
    bool staleFirstCount = false;
    bool flushStuck = false;
    bool deferWrites = false;
    bool stallWrite = false;
    unsigned completionPolls = 0;
    uint8_t *pendingBuffer = nullptr;
    uint8_t pendingRegister = 0, pendingLength = 0;
    bool completionError = false;

    RegisterBus()
    {
        registers[0x72] = 0xe9;
        registers[0x10] = 0x0f;
        registers[0x1b] = registers[0x1c] = 3;
        registers[0x16] = 4;
        registers[0x1d] = 0x12;
        registers[0x20] = 0x20;
        registers[0x23] = 0x60;
        registers[0x7f] = 1;
    }
};

int readRegister(void *context, uint8_t reg, uint8_t *data, size_t count)
{
    auto &bus = *static_cast<RegisterBus *>(context);
    if (reg == bus.failRead) {
        return -1;
    }
    if (reg == 0x7f && count == 1) {
        if (bus.busyForever || bus.busyReads) {
            if (bus.busyReads) {
                --bus.busyReads;
            }
            *data = 0;
        } else {
            *data = 1;
        }
        return 1;
    }
    if (reg == 0x7e && count == 1) {
        *data = bus.indirectAddress == 0xa258 ? bus.smc : bus.endian;
        return 1;
    }
    if (reg == 0x19 && count == 1 && bus.fullAfterRead && bus.dataReads) {
        *data = 1;
        return 1;
    }
    if (reg == 0x12 && count == 2) {
        ++bus.countReads;
        put16(data, bus.staleFirstCount && bus.countReads == 1 ? 999 : bus.packets.size(), bus.endian & 2);
        return 1;
    }
    if (reg == 0x14) {
        ++bus.dataReads;
        if (count % 16 || count / 16 > bus.packets.size()) {
            return 0;
        }
        for (size_t i = 0; i < count / 16; ++i) {
            std::memcpy(data + 16 * i, bus.packets.front().data(), 16);
            bus.packets.pop_front();
        }
        return 1;
    }
    if (reg + count > bus.registers.size()) {
        return 0;
    }
    std::memcpy(data, bus.registers.data() + reg, count);
    return 1;
}

int writeRegister(void *context, uint8_t reg, const uint8_t *data, size_t count)
{
    auto &bus = *static_cast<RegisterBus *>(context);
    if (++bus.writes == bus.failWrite) {
        return -1;
    }
    if (!(reg == 0x7c && count == 2)) {
        ++bus.configurationWrites;
    }
    if (reg == 0x7c && (count == 2 || count == 3)) {
        bus.indirectAddress = (data[0] << 8) | data[1];
        if (count == 3) {
            bus.smc = data[2];
        }
        return 1;
    }
    if (reg + count > bus.registers.size()) {
        return 0;
    }
    std::memcpy(bus.registers.data() + reg, data, count);
    if (reg == 0x20 && count == 1 && (*data & 0x80) && !bus.flushStuck) {
        bus.registers[reg] &= 0x7f;
        bus.packets.clear();
    }
    if (reg == 0x7f && count == 1 && *data == 2) {
        bus.registers[0x19] = 0x80;
        bus.registers[0x7f] = 1;
    }
    return 1;
}

void delayUs(uint32_t duration)
{
    elapsedNs += duration * UINT64_C(1000);
}

uint64_t clockNs()
{
    return stoppedClock ? 0 : elapsedNs;
}

class Icm45686Test : public ::testing::Test {
protected:
    RegisterBus bus;
    busDevice_t nativeBus{};
    extDevice_t endpoint{};
    icm456xxDevice_t io{};
    icm45686FifoDevice_t device{};

    void SetUp() override
    {
        nativeBus.busType = BUS_TYPE_SPI;
        endpoint.bus = &nativeBus;
        endpoint.callbackArg = reinterpret_cast<uintptr_t>(&bus);
        io.dev = &endpoint;
        elapsedNs = 0;
        stoppedClock = false;
    }
};

TEST_F(Icm45686Test, InitializesTheDocumentedDirectProfile)
{
    ASSERT_TRUE(icm45686InitDirect(&io, false));
    EXPECT_EQ(3, bus.registers[0x1b]);
    EXPECT_EQ(3, bus.registers[0x1c]);
    EXPECT_EQ(0x0f, bus.registers[0x10]);
    EXPECT_EQ(4, bus.registers[0x16]);
    EXPECT_GE(elapsedNs, UINT64_C(71000000));
}

TEST_F(Icm45686Test, DoesNotAcceptNegativeTransportResults)
{
    bus.failRead = 0x72;
    EXPECT_FALSE(icm45686InitDirect(&io, false));
    bus.failRead = -1;
    bus.failWrite = 1;
    EXPECT_FALSE(icm45686InitDirect(&io, false));
}

TEST_F(Icm45686Test, ReadsOnlyFreshDirectSamplesAndPreservesOutputOnErrors)
{
    icm45686DirectSample_t sample{};
    sample.temperature = 123;
    EXPECT_EQ(ICM45686_PENDING, icm45686ReadDirect(&io, &sample));
    EXPECT_EQ(123, sample.temperature);
    bus.registers[0x19] = 4;
    bus.failRead = 0;
    EXPECT_EQ(ICM45686_TRANSPORT, icm45686ReadDirect(&io, &sample));
    EXPECT_EQ(123, sample.temperature);
    bus.failRead = -1;
    put16(bus.registers.data(), static_cast<uint16_t>(-1234), false);
    put16(bus.registers.data() + 12, static_cast<uint16_t>(-128), false);
    ASSERT_EQ(ICM45686_READY, icm45686ReadDirect(&io, &sample));
    EXPECT_EQ(-1234, sample.accel[0]);
    EXPECT_EQ(-128, sample.temperature);
}

TEST_F(Icm45686Test, DecodesBothByteOrdersAndTimestampWrap)
{
    for (bool bigEndian : {false, true}) {
        const auto first = packet(65500, bigEndian);
        const auto second = packet(120, bigEndian);
        std::array<uint8_t, 32> data{};
        std::memcpy(data.data(), first.data(), 16);
        std::memcpy(data.data() + 16, second.data(), 16);
        icm45686FifoTimeline_t timeline{};
        icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
        ASSERT_EQ(ICM45686_READY, icm45686FifoDecode(data.data(), 2, bigEndian, &timeline, frames));
        EXPECT_EQ(-1234, frames[0].accel[0]);
        EXPECT_EQ(-16, frames[0].gyro[0]);
        EXPECT_EQ(156u, frames[1].deviceElapsedUs);
        EXPECT_EQ(2u, timeline.ordinal);
    }
}

TEST_F(Icm45686Test, RejectsAnInvalidLastPacketWithoutCommittingAnyFrame)
{
    std::array<uint8_t, 32> data{};
    const auto first = packet(100);
    const auto second = packet(256);
    std::memcpy(data.data(), first.data(), 16);
    std::memcpy(data.data() + 16, second.data(), 16);
    data[16] |= 0x80;
    icm45686FifoTimeline_t timeline{};
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    frames[0].ordinal = 999;
    EXPECT_EQ(ICM45686_PACKET, icm45686FifoDecode(data.data(), 2, false, &timeline, frames));
    EXPECT_EQ(999u, frames[0].ordinal);
    EXPECT_EQ(0u, timeline.ordinal);
}

TEST_F(Icm45686Test, RejectsMissingSamplesAndInvalidAxisSentinels)
{
    auto data = packet(412);
    icm45686FifoTimeline_t timeline{};
    timeline.hasTick = true;
    timeline.previousTick = 100;
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    EXPECT_EQ(ICM45686_TIMELINE, icm45686FifoDecode(data.data(), 1, false, &timeline, frames));
    put16(data.data() + 7, 0x8000, false);
    EXPECT_EQ(ICM45686_PACKET, icm45686FifoDecode(data.data(), 1, false, &timeline, frames));
    EXPECT_EQ(100, timeline.previousTick);
}

TEST_F(Icm45686Test, RestoresAllSavedSettingsAfterFifoCapture)
{
    const auto registers = bus.registers;
    const auto smc = bus.smc;
    ASSERT_EQ(ICM45686_READY, icm45686FifoStart(&io, &device));
    EXPECT_TRUE(device.saved);
    EXPECT_TRUE(device.active);
    EXPECT_EQ(ICM45686_INVALID, icm45686FifoStart(&io, &device));
    ASSERT_EQ(ICM45686_CLOSED, icm45686FifoStop(&io, &device));
    EXPECT_FALSE(device.saved);
    EXPECT_FALSE(device.active);
    EXPECT_EQ(registers, bus.registers);
    EXPECT_EQ(smc, bus.smc);
}

TEST_F(Icm45686Test, RollsBackEveryPartiallyAppliedStart)
{
    for (unsigned failedWrite = 3; failedWrite <= 11; ++failedWrite) {
        RegisterBus candidate;
        const auto registers = candidate.registers;
        extDevice_t candidateDevice = endpoint;
        candidateDevice.callbackArg = reinterpret_cast<uintptr_t>(&candidate);
        icm456xxDevice_t transport{};
        transport.dev = &candidateDevice;
        candidate.failWrite = failedWrite;
        icm45686FifoDevice_t state{};
        EXPECT_EQ(ICM45686_CONFIGURATION, icm45686FifoStart(&transport, &state)) << failedWrite;
        EXPECT_FALSE(state.saved) << failedWrite;
        EXPECT_FALSE(state.active) << failedWrite;
        EXPECT_EQ(registers, candidate.registers) << failedWrite;
    }
}

TEST_F(Icm45686Test, WaitsForIndirectAccessAndBoundsAStuckBusyFlag)
{
    bus.busyReads = 3;
    ASSERT_EQ(ICM45686_READY, icm45686FifoStart(&io, &device));
    EXPECT_GE(elapsedNs, UINT64_C(12000));
    ASSERT_EQ(ICM45686_CLOSED, icm45686FifoStop(&io, &device));
    bus.busyForever = true;
    const auto before = elapsedNs;
    EXPECT_EQ(ICM45686_TRANSPORT, icm45686FifoStart(&io, &device));
    EXPECT_EQ(UINT64_C(5000000), elapsedNs - before);
    EXPECT_FALSE(device.saved);
}

TEST_F(Icm45686Test, RejectsIndirectBusErrorsAndMismatchedAcquisitionProfiles)
{
    bus.failRead = 0x7f;
    EXPECT_EQ(ICM45686_TRANSPORT, icm45686FifoStart(&io, &device));
    EXPECT_EQ(0u, bus.configurationWrites);
    bus.failRead = -1;
    bus.registers[0x1c] = 4;
    EXPECT_EQ(ICM45686_CONFIGURATION, icm45686FifoStart(&io, &device));
    EXPECT_EQ(0u, bus.configurationWrites);
}

TEST_F(Icm45686Test, DiscardsTheFirstCountAndLeavesTheNewestPacket)
{
    ASSERT_EQ(ICM45686_READY, icm45686FifoStart(&io, &device));
    bus.packets = {packet(100), packet(256), packet(412)};
    bus.staleFirstCount = true;
    icm45686FifoRead_t readout{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoFetch(&io, &device, &readout));
    EXPECT_EQ(2u, readout.count);
    EXPECT_EQ(3u, readout.available);
    EXPECT_EQ(1u, bus.packets.size());
    EXPECT_EQ(2u, bus.countReads);
}

TEST_F(Icm45686Test, RejectsTheWholeBurstWhenTheFifoBecomesFull)
{
    ASSERT_EQ(ICM45686_READY, icm45686FifoStart(&io, &device));
    bus.packets = {packet(100), packet(256)};
    bus.fullAfterRead = true;
    icm45686FifoRead_t readout{};
    EXPECT_EQ(ICM45686_OVERFLOW, icm45686FifoFetch(&io, &device, &readout));
    EXPECT_EQ(0u, readout.count);
}

TEST_F(Icm45686Test, BoundsFlushFailureEvenIfTheClockStops)
{
    bus.flushStuck = true;
    stoppedClock = true;
    EXPECT_EQ(ICM45686_RESTORE, icm45686FifoStart(&io, &device));
    EXPECT_TRUE(device.saved);
    EXPECT_FALSE(device.active);
    bus.flushStuck = false;
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStop(&io, &device));
}

TEST_F(Icm45686Test, ValidatesRequiredPointers)
{
    EXPECT_FALSE(icm45686InitDirect(nullptr, false));
    EXPECT_EQ(ICM45686_INVALID, icm45686ReadDirect(&io, nullptr));
    EXPECT_EQ(ICM45686_INVALID, icm45686FifoStart(&io, nullptr));
    EXPECT_EQ(ICM45686_INVALID, icm45686FifoStop(nullptr, &device));
    EXPECT_EQ(ICM45686_INVALID, icm45686FifoFetch(&io, &device, nullptr));
    const auto data = packet(100);
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    EXPECT_EQ(ICM45686_INVALID, icm45686FifoDecode(data.data(), 1, false, nullptr, frames));
}

TEST_F(Icm45686Test, StreamReadOwnsDecodingAndTheCompleteDeviceTimeline)
{
    icm45686FifoStream_t stream{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamStart(&io, &stream));
    bus.packets = {packet(65500), packet(120), packet(276)};
    icm45686FifoRead_t readout{};
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamRead(&io, &stream, frames, &readout));
    EXPECT_EQ(2u, readout.count);
    EXPECT_EQ(2u, stream.timeline.ordinal);
    EXPECT_EQ(156u, frames[1].deviceElapsedUs);
    EXPECT_EQ(-1234, frames[0].accel[0]);
    EXPECT_LE(readout.readBeginNs, readout.readCompleteNs);
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStreamStop(&io, &stream));
}

TEST_F(Icm45686Test, PendingReadsDoNotExtendTheLastFreshFrameDeadline)
{
    icm45686FifoStream_t stream{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamStart(&io, &stream));
    icm45686FifoRead_t readout{};
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    elapsedNs += 49999999;
    EXPECT_EQ(ICM45686_PENDING, icm45686FifoStreamRead(&io, &stream, frames, &readout));
    ++elapsedNs;
    EXPECT_EQ(ICM45686_TIMELINE, icm45686FifoStreamRead(&io, &stream, frames, &readout));
    EXPECT_EQ(0u, readout.count);
    EXPECT_EQ(ICM45686_TIMELINE, stream.fault);
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStreamStop(&io, &stream));
}

TEST_F(Icm45686Test, StreamRejectsBackwardHostTimeAndLatchesTheFault)
{
    icm45686FifoStream_t stream{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamStart(&io, &stream));
    icm45686FifoRead_t readout{};
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    --elapsedNs;
    EXPECT_EQ(ICM45686_TIMELINE, icm45686FifoStreamRead(&io, &stream, frames, &readout));
    elapsedNs += 1000;
    bus.packets = {packet(100), packet(256)};
    EXPECT_EQ(ICM45686_TIMELINE, icm45686FifoStreamRead(&io, &stream, frames, &readout));
    EXPECT_EQ(0u, bus.dataReads);
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStreamStop(&io, &stream));
}

TEST_F(Icm45686Test, InvalidStreamBurstsDoNotCommitFramesAndStopCanRecover)
{
    icm45686FifoStream_t stream{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamStart(&io, &stream));
    auto invalid = packet(256);
    invalid[0] |= 0x80;
    bus.packets = {packet(100), invalid, packet(412)};
    icm45686FifoRead_t readout{};
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    frames[0].ordinal = 999;
    EXPECT_EQ(ICM45686_PACKET, icm45686FifoStreamRead(&io, &stream, frames, &readout));
    EXPECT_EQ(0u, readout.count);
    EXPECT_EQ(999u, frames[0].ordinal);
    EXPECT_EQ(0u, stream.timeline.ordinal);
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStreamStop(&io, &stream));
    EXPECT_EQ(ICM45686_READY, icm45686FifoStreamStart(&io, &stream));
    EXPECT_EQ(ICM45686_READY, stream.fault);
    EXPECT_EQ(0u, stream.timeline.ordinal);
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStreamStop(&io, &stream));
}

TEST_F(Icm45686Test, KeepsIndependentStreamStateForSeparateDevices)
{
    RegisterBus second;
    extDevice_t secondDevice = endpoint;
    secondDevice.callbackArg = reinterpret_cast<uintptr_t>(&second);
    icm456xxDevice_t secondIo{};
    secondIo.dev = &secondDevice;
    icm45686FifoStream_t firstStream{}, secondStream{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamStart(&io, &firstStream));
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamStart(&secondIo, &secondStream));
    bus.packets = {packet(100), packet(256), packet(412)};
    second.packets = {packet(1000), packet(1156)};
    icm45686FifoRead_t readout{};
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamRead(&io, &firstStream, frames, &readout));
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamRead(&secondIo, &secondStream, frames, &readout));
    EXPECT_EQ(2u, firstStream.timeline.ordinal);
    EXPECT_EQ(1u, secondStream.timeline.ordinal);
    EXPECT_EQ(1156, second.packets.front()[14] | second.packets.front()[15] << 8);
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStreamStop(&io, &firstStream));
    EXPECT_TRUE(secondStream.device.active);
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStreamStop(&secondIo, &secondStream));
}
TEST_F(Icm45686Test, WaitsForBufferedI2cWriteCompletion)
{
    nativeBus.busType = BUS_TYPE_I2C;
    bus.deferWrites = true;
    bus.completionPolls = 3;
    ASSERT_TRUE(icm456xxWriteIndirect(&io, 0xa258, 0x11));
    EXPECT_EQ(0x11, bus.smc);
    EXPECT_EQ(nullptr, bus.pendingBuffer);
    EXPECT_FALSE(io.writePending);
    EXPECT_EQ(1U, bus.writes);
    EXPECT_GE(elapsedNs, UINT64_C(11000));
}

TEST_F(Icm45686Test, RetainsTimedOutWriteUntilCompletionAndRejectsReplacement)
{
    nativeBus.busType = BUS_TYPE_I2C;
    bus.deferWrites = true;
    bus.stallWrite = true;
    EXPECT_FALSE(icm456xxWriteIndirect(&io, 0xa258, 0x11));
    ASSERT_EQ(io.writeBuffer, bus.pendingBuffer);
    const auto data = std::array<uint8_t, 3>{io.writeBuffer[0], io.writeBuffer[1], io.writeBuffer[2]};
    EXPECT_TRUE(io.writePending);
    EXPECT_FALSE(icm456xxWriteIndirect(&io, 0xa258, 0x22));
    EXPECT_EQ((std::array<uint8_t, 3>{0xa2, 0x58, 0x11}), data);
    EXPECT_EQ(data, (std::array<uint8_t, 3>{io.writeBuffer[0], io.writeBuffer[1], io.writeBuffer[2]}));
    EXPECT_EQ(0U, bus.writes);
    EXPECT_EQ(0x14, bus.smc);
    bus.stallWrite = false;
    uint8_t actual = 0;
    ASSERT_TRUE(icm456xxReadIndirect(&io, 0xa258, &actual));
    EXPECT_EQ(0x11, actual);
    EXPECT_FALSE(io.writePending);
    EXPECT_EQ(nullptr, bus.pendingBuffer);
}

TEST_F(Icm45686Test, KeepsStreamTimeAcrossThe32BitMicrosBoundary)
{
    elapsedNs = (UINT64_C(0xffffffff) - 1000U) * 1000U;
    icm45686FifoStream_t stream{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamStart(&io, &stream));
    const uint64_t before = stream.lastReadNs;
    elapsedNs += UINT64_C(2000000);
    bus.packets = {packet(100), packet(256)};
    icm45686FifoRead_t readout{};
    icm45686FifoFrame_t frames[ICM45686_FIFO_BATCH_CAPACITY]{};
    ASSERT_EQ(ICM45686_READY, icm45686FifoStreamRead(&io, &stream, frames, &readout));
    EXPECT_GT(readout.readCompleteNs, UINT64_C(0xffffffff) * 1000U);
    EXPECT_GE(readout.readCompleteNs, before);
    EXPECT_EQ(elapsedNs, readout.readCompleteNs);
    EXPECT_EQ(1U, readout.count);
    EXPECT_EQ(ICM45686_CLOSED, icm45686FifoStreamStop(&io, &stream));
}

} // namespace

extern "C" {
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (reinterpret_cast<RegisterBus *>(dev->callbackArg)->pendingBuffer) {
        return false;
    }
    return readRegister(reinterpret_cast<void *>(dev->callbackArg), reg, data, length) > 0;
}
bool busWriteRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    auto &bus = *reinterpret_cast<RegisterBus *>(dev->callbackArg);
    if (bus.pendingBuffer) {
        return false;
    }
    if (bus.deferWrites) {
        bus.pendingBuffer = data;
        bus.pendingRegister = reg;
        bus.pendingLength = length;
        return true;
    }
    return writeRegister(reinterpret_cast<void *>(dev->callbackArg), reg, data, length) > 0;
}
bool busBusy(const extDevice_t *dev, bool *error)
{
    auto &bus = *reinterpret_cast<RegisterBus *>(dev->callbackArg);
    if (bus.pendingBuffer) {
        if (bus.stallWrite) {
            return true;
        }
        if (bus.completionPolls) {
            --bus.completionPolls;
            return true;
        }
        const int result = writeRegister(&bus, bus.pendingRegister, bus.pendingBuffer, bus.pendingLength);
        bus.pendingBuffer = nullptr;
        bus.completionError = result <= 0;
    }
    if (error) {
        *error = bus.completionError;
    }
    return false;
}
void delayMicroseconds(timeUs_t duration)
{
    delayUs(duration);
}
timeUs_t micros(void)
{
    return clockNs() / 1000U;
}
}
