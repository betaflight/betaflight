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
#include <array>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>

extern "C" {

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_quadspi.h"
#include "drivers/bus_spi.h"
#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/flash_w25n.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "io/flashfs.h"
#include "pg/flash.h"

}

#include "gtest/gtest.h"

namespace {

// Behavioural model of a W25N01GV on QSPI: 2048-byte pages plus 64 spare bytes, 64 pages per block.
constexpr uint32_t PAGE_SIZE = 2048;
constexpr uint32_t BUFFER_SIZE = PAGE_SIZE + 64;
constexpr uint32_t PAGES_PER_BLOCK = 64;

constexpr uint8_t INSTRUCTION_RDID = 0x9F;
constexpr uint8_t INSTRUCTION_DEVICE_RESET = 0xFF;
constexpr uint8_t INSTRUCTION_WRITE_ENABLE = 0x06;
constexpr uint8_t INSTRUCTION_READ_STATUS_REG = 0x05;
constexpr uint8_t INSTRUCTION_PROGRAM_DATA_LOAD = 0x02;
constexpr uint8_t INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD = 0x84;
constexpr uint8_t INSTRUCTION_PROGRAM_EXECUTE = 0x10;
constexpr uint8_t INSTRUCTION_PAGE_DATA_READ = 0x13;
constexpr uint8_t INSTRUCTION_BLOCK_ERASE = 0xD8;
constexpr uint8_t INSTRUCTION_FAST_READ_QUAD_OUTPUT = 0x6B;

constexpr uint8_t STAT_REG = 0xC0;
constexpr uint8_t STATUS_BUSY = 1 << 0;
constexpr uint8_t STATUS_WEL = 1 << 1;

std::map<uint32_t, std::vector<uint8_t>> pages;
std::array<uint8_t, BUFFER_SIZE> dataBuffer;
bool writeEnableLatch;
int busyPolls;
int busyPollsAfterProgram;
uint32_t statusPolls;

busDevice_t bus;

void resetChip()
{
    pages.clear();
    dataBuffer.fill(0xFF);
    writeEnableLatch = false;
    busyPolls = 0;
    busyPollsAfterProgram = 0;
    statusPolls = 0;
    std::memset(&bus, 0, sizeof(bus));
}

void loadBuffer(uint32_t column, const uint8_t *out, int length)
{
    for (int i = 0; i < length && column + i < BUFFER_SIZE; i++) {
        dataBuffer[column + i] = out[i];
    }
}

void programPage(uint32_t page)
{
    auto &cells = pages.emplace(page, std::vector<uint8_t>(PAGE_SIZE, 0xFF)).first->second;
    for (uint32_t i = 0; i < PAGE_SIZE; i++) {
        cells[i] &= dataBuffer[i];
    }
}

void readPageIntoBuffer(uint32_t page)
{
    dataBuffer.fill(0xFF);
    const auto it = pages.find(page);
    if (it != pages.end()) {
        std::copy(it->second.begin(), it->second.end(), dataBuffer.begin());
    }
}

void eraseBlock(uint32_t page)
{
    const uint32_t first = page - (page % PAGES_PER_BLOCK);
    pages.erase(pages.lower_bound(first), pages.lower_bound(first + PAGES_PER_BLOCK));
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
    const uint8_t id[] = { 0x00, 0xEF, 0xAA, 0x21 };
    std::memset(in, 0, length);
    if (instruction == INSTRUCTION_RDID) {
        std::memcpy(in, id, std::min<int>(length, sizeof(id)));
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
    case INSTRUCTION_DEVICE_RESET:
        writeEnableLatch = false;
        break;
    }
    return true;
}

bool quadSpiInstructionWithAddress1LINE(const extDevice_t *, uint8_t instruction, uint8_t, uint32_t address, uint8_t)
{
    const uint32_t page = address & 0xFFFF;

    switch (instruction) {
    case INSTRUCTION_PROGRAM_EXECUTE:
        if (writeEnableLatch) {
            programPage(page);
            busyPolls = busyPollsAfterProgram;
        }
        writeEnableLatch = false;
        break;
    case INSTRUCTION_BLOCK_ERASE:
        if (writeEnableLatch) {
            eraseBlock(page);
        }
        writeEnableLatch = false;
        break;
    case INSTRUCTION_PAGE_DATA_READ:
        readPageIntoBuffer(page);
        break;
    }
    return true;
}

bool quadSpiReceiveWithAddress1LINE(const extDevice_t *, uint8_t instruction, uint8_t, uint32_t address, uint8_t, uint8_t *in, int length)
{
    std::memset(in, 0, length);
    if (instruction == INSTRUCTION_READ_STATUS_REG && address == STAT_REG) {
        statusPolls++;
        in[0] = (writeEnableLatch ? STATUS_WEL : 0) | (busyPolls > 0 ? STATUS_BUSY : 0);
        if (busyPolls > 0) {
            busyPolls--;
        }
    }
    return true;
}

bool quadSpiTransmitWithAddress1LINE(const extDevice_t *, uint8_t instruction, uint8_t, uint32_t address, uint8_t, const uint8_t *out, int length)
{
    switch (instruction) {
    case INSTRUCTION_PROGRAM_DATA_LOAD:
        if (writeEnableLatch) {
            dataBuffer.fill(0xFF);
            loadBuffer(address, out, length);
        }
        break;
    case INSTRUCTION_RANDOM_PROGRAM_DATA_LOAD:
        if (writeEnableLatch) {
            loadBuffer(address, out, length);
        }
        break;
    }
    return true;
}

bool quadSpiReceiveWithAddress4LINES(const extDevice_t *, uint8_t instruction, uint8_t, uint32_t address, uint8_t, uint8_t *in, int length)
{
    if (instruction == INSTRUCTION_FAST_READ_QUAD_OUTPUT) {
        for (int i = 0; i < length; i++) {
            in[i] = address + i < BUFFER_SIZE ? dataBuffer[address + i] : 0xFF;
        }
    }
    return true;
}

void spiSequence(const extDevice_t *, busSegment_t *) {}
void spiWait(const extDevice_t *) {}
bool spiIsBusy(const extDevice_t *) { return false; }
void spiSetClkDivisor(const extDevice_t *, uint16_t) {}
uint16_t spiCalculateDivider(uint32_t) { return 0; }
bool spiSetBusInstance(extDevice_t *, uint32_t) { return false; }
void spiReadRegBuf(const extDevice_t *, uint8_t, uint8_t *, uint8_t) {}
IO_t IOGetByTag(ioTag_t) { return IO_NONE; }
bool IOIsFreeOrPreinit(IO_t) { return true; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
void IOHi(IO_t) {}

uint32_t millis(void) { return statusPolls / 64; }
uint32_t microsISR(void) { return millis() * 1000; }
void delay(uint32_t) {}
void failureMode(failureMode_e) {}
void ioPreinitByTag(ioTag_t, ioConfig_t, ioPreinitPinState_e) {}

} // extern "C"

class W25nQuadSpiFlashfsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        resetChip();

        flashConfig_t config = {};
        config.quadSpiDevice = QUADSPI_DEV_TO_CFG(QUADSPIDEV_1);
        ASSERT_TRUE(flashInit(&config));
        ASSERT_EQ(PAGE_SIZE, flashGetGeometry()->pageSize);

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

TEST_F(W25nQuadSpiFlashfsTest, ConsecutiveLogsSurviveWhenDeviceAlwaysReady)
{
    writeAndVerifyConsecutiveLogs(48);
}

TEST_F(W25nQuadSpiFlashfsTest, ConsecutiveLogsSurviveWhenProgramExecuteKeepsDeviceBusy)
{
    busyPollsAfterProgram = 5;
    writeAndVerifyConsecutiveLogs(37);
}
