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

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

extern "C" {

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_quadspi.h"
#include "drivers/bus_spi.h"
#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "io/flashfs.h"
#include "pg/flash.h"

}

#include "gtest/gtest.h"

namespace {

// Behavioural model of a W25Q128FV on QSPI: 16MB, 256-byte program pages, 64KB erase blocks.
constexpr uint32_t TOTAL_SIZE = 16 * 1024 * 1024;
constexpr uint32_t PAGE_SIZE = 256;
constexpr uint32_t BLOCK_SIZE = 64 * 1024;

constexpr uint8_t INSTRUCTION_RDID = 0x9F;
constexpr uint8_t INSTRUCTION_WRITE_ENABLE = 0x06;
constexpr uint8_t INSTRUCTION_RESET_DEVICE = 0x99;
constexpr uint8_t INSTRUCTION_READ_STATUS1_REG = 0x05;
constexpr uint8_t INSTRUCTION_READ_STATUS2_REG = 0x35;
constexpr uint8_t INSTRUCTION_CHIP_ERASE = 0xC7;
constexpr uint8_t INSTRUCTION_BLOCK_ERASE_64KB = 0xD8;
constexpr uint8_t INSTRUCTION_QUAD_PAGE_PROGRAM = 0x32;
constexpr uint8_t INSTRUCTION_FAST_READ_QUAD_OUTPUT = 0x6B;

constexpr uint8_t SR1_WRITE_IN_PROGRESS = 1 << 0;
constexpr uint8_t SR1_WRITE_ENABLED = 1 << 1;
constexpr uint8_t SR2_QUAD_ENABLE = 1 << 1;

std::vector<uint8_t> memory;
bool writeEnableLatch;
int busyPolls;
int busyPollsAfterProgram;
uint32_t clockTicks;

busDevice_t bus;

void resetChip()
{
    memory.assign(TOTAL_SIZE, 0x00);
    writeEnableLatch = false;
    busyPolls = 0;
    busyPollsAfterProgram = 0;
    clockTicks = 0;
    std::memset(&bus, 0, sizeof(bus));
}

uint8_t logByte(uint32_t seed, uint32_t index)
{
    return (seed * 131 + index * 7 + (index >> 8)) % 0xFF;
}

void writeLog(uint32_t seed, uint32_t length, uint32_t frameSize)
{
    for (uint32_t i = 0; i < length; i++) {
        flashfsWriteByte(logByte(seed, i));
        if ((i % frameSize) == frameSize - 1) {
            flashfsFlushAsync(false);
        }
    }

    while (!flashfsFlushAsync(true)) {
    }
    flashfsClose();
}

uint32_t countMismatches(uint32_t seed, uint32_t start, uint32_t length)
{
    uint32_t mismatches = 0;
    uint8_t chunk[256];

    for (uint32_t offset = 0; offset < length;) {
        const int len = flashfsReadAbs(start + offset, chunk, std::min<uint32_t>(sizeof(chunk), length - offset));
        if (len <= 0) {
            return mismatches + (length - offset);
        }
        for (int i = 0; i < len; i++) {
            mismatches += chunk[i] != logByte(seed, offset + i);
        }
        offset += len;
    }

    return mismatches;
}

} // namespace

extern "C" {

bool quadSpiSetBusInstance(extDevice_t *dev, uint32_t)
{
    dev->bus = &bus;
    return true;
}

void quadSpiSetDivisor(const extDevice_t *, uint16_t) {}

bool quadSpiReceive1LINE(const extDevice_t *, uint8_t instruction, uint8_t, uint8_t *in, int length)
{
    const uint8_t id[] = { 0xEF, 0x40, 0x18, 0x00 };
    std::memset(in, 0, length);

    switch (instruction) {
    case INSTRUCTION_RDID:
        std::memcpy(in, id, std::min<int>(length, sizeof(id)));
        break;
    case INSTRUCTION_READ_STATUS1_REG:
        clockTicks++;
        in[0] = (writeEnableLatch ? SR1_WRITE_ENABLED : 0) | (busyPolls > 0 ? SR1_WRITE_IN_PROGRESS : 0);
        if (busyPolls > 0) {
            busyPolls--;
        }
        break;
    case INSTRUCTION_READ_STATUS2_REG:
        in[0] = SR2_QUAD_ENABLE;
        break;
    }
    return true;
}

bool quadSpiReceive4LINES(const extDevice_t *, uint8_t, uint8_t, uint8_t *, int)
{
    return false;
}

bool quadSpiTransmit1LINE(const extDevice_t *, uint8_t instruction, uint8_t, const uint8_t *, int)
{
    switch (instruction) {
    case INSTRUCTION_WRITE_ENABLE:
        writeEnableLatch = true;
        break;
    case INSTRUCTION_RESET_DEVICE:
        writeEnableLatch = false;
        break;
    case INSTRUCTION_CHIP_ERASE:
        if (writeEnableLatch) {
            std::fill(memory.begin(), memory.end(), 0xFF);
        }
        writeEnableLatch = false;
        break;
    }
    return true;
}

bool quadSpiInstructionWithAddress1LINE(const extDevice_t *, uint8_t instruction, uint8_t, uint32_t address, uint8_t)
{
    if (instruction == INSTRUCTION_BLOCK_ERASE_64KB) {
        if (writeEnableLatch) {
            const uint32_t start = address - (address % BLOCK_SIZE);
            std::fill(memory.begin() + start, memory.begin() + start + BLOCK_SIZE, 0xFF);
        }
        writeEnableLatch = false;
    }
    return true;
}

bool quadSpiTransmitWithAddress4LINES(const extDevice_t *, uint8_t instruction, uint8_t, uint32_t address, uint8_t, const uint8_t *out, int length)
{
    if (instruction == INSTRUCTION_QUAD_PAGE_PROGRAM && writeEnableLatch) {
        const uint32_t pageStart = address - (address % PAGE_SIZE);
        for (int i = 0; i < length; i++) {
            memory[pageStart + ((address + i) % PAGE_SIZE)] &= out[i];
        }
        busyPolls = busyPollsAfterProgram;
    }
    writeEnableLatch = false;
    return true;
}

bool quadSpiReceiveWithAddress4LINES(const extDevice_t *, uint8_t instruction, uint8_t, uint32_t address, uint8_t, uint8_t *in, int length)
{
    if (instruction == INSTRUCTION_FAST_READ_QUAD_OUTPUT) {
        std::copy_n(memory.begin() + address, length, in);
    }
    return true;
}

bool quadSpiReceiveWithAddress1LINE(const extDevice_t *, uint8_t, uint8_t, uint32_t, uint8_t, uint8_t *, int) { return false; }
bool quadSpiTransmitWithAddress1LINE(const extDevice_t *, uint8_t, uint8_t, uint32_t, uint8_t, const uint8_t *, int) { return false; }

bool spiSetBusInstance(extDevice_t *, uint32_t) { return false; }
void spiReadRegBuf(const extDevice_t *, uint8_t, uint8_t *, uint8_t) {}
void spiSetClkDivisor(const extDevice_t *, uint16_t) {}
uint16_t spiCalculateDivider(uint32_t) { return 0; }
IO_t IOGetByTag(ioTag_t) { return IO_NONE; }
bool IOIsFreeOrPreinit(IO_t) { return true; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
void IOHi(IO_t) {}

uint32_t microsISR(void) { return (clockTicks++ / 64) * 1000; }
uint32_t millis(void) { return microsISR() / 1000; }
void delay(uint32_t) {}
void failureMode(failureMode_e) {}
void ioPreinitByTag(ioTag_t, ioConfig_t, ioPreinitPinState_e) {}

} // extern "C"

class W25q128fvQuadSpiFlashfsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        resetChip();

        flashConfig_t config = {};
        config.quadSpiDevice = QUADSPI_DEV_TO_CFG(QUADSPIDEV_1);
        ASSERT_TRUE(flashInit(&config));
        ASSERT_EQ(TOTAL_SIZE, flashGetGeometry()->totalSize);

        flashfsInit();
        flashfsEraseCompletely();
        while (!flashfsIsReady()) {
            flashfsEraseAsync();
        }
    }

    void writeAndVerifyConsecutiveLogs(uint32_t frameSize)
    {
        const uint32_t lengths[] = { 5037, 9001, 7777 };
        uint32_t starts[3];

        for (uint32_t log = 0; log < 3; log++) {
            starts[log] = flashfsGetOffset();
            writeLog(log + 1, lengths[log], frameSize);
        }

        for (uint32_t log = 0; log < 3; log++) {
            EXPECT_EQ(0u, countMismatches(log + 1, starts[log], lengths[log])) << "log " << log + 1;
        }

        EXPECT_GE((uint32_t)flashfsIdentifyStartOfFreeSpace(), starts[2] + lengths[2]);
    }
};

TEST_F(W25q128fvQuadSpiFlashfsTest, ConsecutiveLogsSurviveWhenDeviceAlwaysReady)
{
    writeAndVerifyConsecutiveLogs(48);
}

TEST_F(W25q128fvQuadSpiFlashfsTest, ConsecutiveLogsSurviveWhenPageProgramKeepsDeviceBusy)
{
    busyPollsAfterProgram = 5;
    writeAndVerifyConsecutiveLogs(37);
}
