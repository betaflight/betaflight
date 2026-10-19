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
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

extern "C" {

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/flash/flash.h"
#include "drivers/flash/flash_fm25v02a.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/system.h"

}

#include "gtest/gtest.h"

namespace {

constexpr uint8_t WREN = 0x06;
constexpr uint8_t RDSR = 0x05;
constexpr uint8_t WRSR = 0x01;
constexpr uint8_t READ = 0x03;
constexpr uint8_t WRITE = 0x02;

std::array<uint8_t, FM25V02A_TOTAL_SIZE> memory;
uint8_t statusRegister;
bool spiBusy;
bool writeEnableLatch;
bool statusWriteBlocked;
uint16_t lastDivider;
std::vector<std::vector<uint8_t>> transactions;
std::vector<uint8_t> readIdResponse;
size_t readIdLength;
int readIdCalls;
bool m25p16IdentifyResult;
uint32_t lastM25p16JedecId;
busSegment_t *pendingSegments;
const extDevice_t *pendingDevice;
uintptr_t callbackBytes;

busDevice_t bus;
extDevice_t dev;
flashDevice_t flashDevice;
flashVTable_t mockNorVTable;

void resetMock()
{
    memory.fill(0xA5);
    statusRegister = 0;
    spiBusy = false;
    writeEnableLatch = false;
    statusWriteBlocked = false;
    lastDivider = 0;
    transactions.clear();
    readIdResponse.clear();
    readIdLength = 0;
    readIdCalls = 0;
    m25p16IdentifyResult = false;
    lastM25p16JedecId = 0;
    pendingSegments = nullptr;
    pendingDevice = nullptr;
    callbackBytes = 0;

    std::memset(&bus, 0, sizeof(bus));
    std::memset(&dev, 0, sizeof(dev));
    std::memset(&flashDevice, 0, sizeof(flashDevice));
    std::memset(&mockNorVTable, 0, sizeof(mockNorVTable));
    dev.bus = &bus;
    dev.callbackArg = reinterpret_cast<uintptr_t>(&flashDevice);
    flashDevice.io.mode = FLASHIO_SPI;
    flashDevice.io.handle.dev = &dev;
}

uint16_t commandAddress(const uint8_t *command)
{
    return (static_cast<uint16_t>(command[1] & 0x7F) << 8) | command[2];
}

void recordSegment(const busSegment_t &segment)
{
    if (!segment.u.buffers.txData || segment.len <= 0) {
        return;
    }

    transactions.emplace_back(segment.u.buffers.txData,
        segment.u.buffers.txData + segment.len);
}

void completePendingSequence()
{
    if (!pendingSegments) {
        spiBusy = false;
        return;
    }

    busSegment_t *segments = pendingSegments;
    const extDevice_t *mockDev = pendingDevice;
    pendingSegments = nullptr;
    pendingDevice = nullptr;

    uint32_t writeAddress = 0;
    bool writing = false;
    bool reading = false;

    for (busSegment_t *segment = segments; segment->len != 0; segment++) {
        recordSegment(*segment);

        const uint8_t *tx = segment->u.buffers.txData;
        if (!writing && !reading && tx) {
            if (segment->len == 1 && tx[0] == WREN) {
                writeEnableLatch = true;
            } else if (segment->len == 2 && tx[0] == WRSR) {
                if (writeEnableLatch && !statusWriteBlocked) {
                    statusRegister = tx[1] & 0x8C;
                }
                writeEnableLatch = false;
            } else if (segment->len == 3 && tx[0] == WRITE) {
                writeAddress = commandAddress(tx);
                writing = true;
            } else if (segment->len == 3 && tx[0] == READ) {
                writeAddress = commandAddress(tx);
                reading = true;
            }
        } else if (writing && tx) {
            if (writeEnableLatch) {
                const uint32_t writable = std::min<uint32_t>(segment->len, memory.size() - writeAddress);
                std::copy(tx, tx + writable, memory.begin() + writeAddress);
                writeAddress += writable;
            }
        } else if (reading && segment->u.buffers.rxData) {
            const uint32_t readable = std::min<uint32_t>(segment->len, memory.size() - writeAddress);
            std::copy(memory.begin() + writeAddress, memory.begin() + writeAddress + readable,
                segment->u.buffers.rxData);
            writeAddress += readable;
        }

        if (segment->callback) {
            spiBusy = false;
            EXPECT_EQ(BUS_READY, segment->callback(mockDev->callbackArg));
        }
    }

    if (writing) {
        writeEnableLatch = false;
    }
    spiBusy = false;
}

flashConfig_t spiFlashConfig()
{
    flashConfig_t config = {};
    config.csTag = 1;
    config.spiDevice = SPI_DEV_TO_CFG(SPIDEV_1);
    return config;
}

bool initializeStandardFram()
{
    readIdResponse = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22, 0x08 };
    const flashConfig_t config = spiFlashConfig();
    return flashInit(&config);
}

} // namespace

extern "C" {

uint16_t spiCalculateDivider(uint32_t frequency)
{
    return frequency == 25000000 ? 8 : 0;
}

void spiSetClkDivisor(const extDevice_t *, uint16_t divider)
{
    lastDivider = divider;
}

bool spiIsBusy(const extDevice_t *)
{
    return spiBusy;
}

void spiWait(const extDevice_t *)
{
    completePendingSequence();
}

void spiReadWriteBuf(const extDevice_t *, uint8_t *txData, uint8_t *rxData, int length)
{
    transactions.emplace_back(txData, txData + length);
    if (rxData) {
        std::memset(rxData, 0, length);
        if (length == 2 && txData[0] == RDSR) {
            rxData[1] = statusRegister | (writeEnableLatch ? 0x02 : 0);
        }
    }
}

void spiReadRegBuf(const extDevice_t *, uint8_t reg, uint8_t *data, uint8_t length)
{
    EXPECT_EQ(SPIFLASH_INSTRUCTION_RDID, reg);
    readIdCalls++;
    readIdLength = length;
    std::memset(data, 0, length);
    std::copy_n(readIdResponse.begin(), std::min<size_t>(length, readIdResponse.size()), data);
}

bool spiSetBusInstance(extDevice_t *target, uint32_t)
{
    target->bus = &bus;
    return true;
}

void delay(uint32_t) {}
uint32_t microsISR(void) { return 0; }
void failureMode(failureMode_e) {}
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
void IOHi(IO_t) {}
IO_t IOGetByTag(ioTag_t tag) { return reinterpret_cast<IO_t>(static_cast<uintptr_t>(tag)); }
bool IOIsFreeOrPreinit(IO_t) { return true; }
void ioPreinitByTag(ioTag_t, ioConfig_t, ioPreinitPinState_e) {}

bool m25p16_identify(flashDevice_t *fdevice, uint32_t jedecId)
{
    lastM25p16JedecId = jedecId;
    if (!m25p16IdentifyResult) {
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.sectors = 16;
    fdevice->geometry.pagesPerSector = 16;
    fdevice->geometry.pageSize = 256;
    fdevice->geometry.sectorSize = 4096;
    fdevice->geometry.totalSize = 65536;
    fdevice->vTable = &mockNorVTable;
    return true;
}
bool w25m_identify(flashDevice_t *, uint32_t) { return false; }
bool w25n_identify(flashDevice_t *, uint32_t) { return false; }
bool mt29f_identify(flashDevice_t *, uint32_t) { return false; }

void spiSequence(const extDevice_t *mockDev, busSegment_t *segments)
{
    EXPECT_EQ(nullptr, pendingSegments);
    spiBusy = true;
    pendingDevice = mockDev;
    pendingSegments = segments;
}

} // extern "C"

class Fm25v02aTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        resetMock();
    }

    void identifyStandard()
    {
        const uint8_t id[] = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22, 0x08 };
        ASSERT_TRUE(fm25v02a_identify(&flashDevice, id, sizeof(id)));
    }
};

TEST_F(Fm25v02aTest, IdentifiesOnlyCompleteOfficialIds)
{
    const uint8_t standardId[] = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22, 0x08 };
    const uint8_t extendedId[] = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22, 0x48 };
    const uint8_t shortId[] = { 0x7F, 0x7F, 0x7F };
    const uint8_t otherPart[] = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22, 0x18 };

    EXPECT_TRUE(fm25v02a_identify(&flashDevice, standardId, sizeof(standardId)));
    EXPECT_EQ(FM25V02A_JEDEC_ID, flashDevice.geometry.jedecId);
    EXPECT_TRUE(fm25v02a_identify(&flashDevice, extendedId, sizeof(extendedId)));
    EXPECT_EQ(FM25V02A_JEDEC_ID_EXTENDED_TEMP, flashDevice.geometry.jedecId);
    EXPECT_FALSE(fm25v02a_identify(&flashDevice, shortId, sizeof(shortId)));
    EXPECT_FALSE(fm25v02a_identify(&flashDevice, otherPart, sizeof(otherPart)));
}

TEST_F(Fm25v02aTest, FlashInitUsesFourByteProbeThenFullFramId)
{
    readIdResponse = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22, 0x48 };
    const flashConfig_t config = spiFlashConfig();

    ASSERT_TRUE(flashInit(&config));
    EXPECT_EQ(2, readIdCalls);
    EXPECT_EQ(FM25V02A_DEVICE_ID_LENGTH, readIdLength);
    EXPECT_EQ(FLASH_TYPE_FRAM, flashGetGeometry()->flashType);
    EXPECT_EQ(FM25V02A_JEDEC_ID_EXTENDED_TEMP, flashGetGeometry()->jedecId);
}

TEST_F(Fm25v02aTest, FlashInitKeepsFourByteProbeAndFallsBackToNor)
{
    readIdResponse = { 0xEF, 0x40, 0x18, 0x00 };
    m25p16IdentifyResult = true;
    const flashConfig_t config = spiFlashConfig();

    EXPECT_TRUE(flashInit(&config));
    EXPECT_EQ(1, readIdCalls);
    EXPECT_EQ(4U, readIdLength);
    EXPECT_EQ(0xEF4018U, lastM25p16JedecId);
}

TEST_F(Fm25v02aTest, FailedFullFramIdStillFallsBackToNorProbe)
{
    readIdResponse = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x22, 0x18 };
    m25p16IdentifyResult = true;
    const flashConfig_t config = spiFlashConfig();

    EXPECT_TRUE(flashInit(&config));
    EXPECT_EQ(2, readIdCalls);
    EXPECT_EQ(FM25V02A_DEVICE_ID_LENGTH, readIdLength);
    EXPECT_EQ(0x7F7F7FU, lastM25p16JedecId);
}

TEST_F(Fm25v02aTest, ReportsFramLogicalGeometry)
{
    identifyStandard();

    const flashGeometry_t *geometry = flashDevice.vTable->getGeometry(&flashDevice);
    EXPECT_EQ(FLASH_TYPE_FRAM, geometry->flashType);
    EXPECT_EQ(16, geometry->sectors);
    EXPECT_EQ(256, geometry->pageSize);
    EXPECT_EQ(8, geometry->pagesPerSector);
    EXPECT_EQ(2048U, geometry->sectorSize);
    EXPECT_EQ(32768U, geometry->totalSize);
}

TEST_F(Fm25v02aTest, ConfigureClearsNonvolatileProtection)
{
    identifyStandard();
    statusRegister = 0x8C;

    flashDevice.vTable->configure(&flashDevice, 0);

    EXPECT_EQ(0, statusRegister);
    EXPECT_EQ(8, lastDivider);
    ASSERT_GE(transactions.size(), 3U);
    EXPECT_EQ(RDSR, transactions.front()[0]);
    EXPECT_EQ(WREN, transactions[1][0]);
    EXPECT_EQ(WRSR, transactions[2][0]);
}

TEST_F(Fm25v02aTest, FlashInitAcceptsWpenOnlyWhenStatusWriteIsHardwareProtected)
{
    statusRegister = 0x80;
    statusWriteBlocked = true;

    ASSERT_TRUE(initializeStandardFram());
    EXPECT_EQ(0x80, statusRegister);
    EXPECT_EQ(FM25V02A_TOTAL_SIZE, flashGetGeometry()->totalSize);

    transactions.clear();
    const uint8_t value = 0x39;
    const uint8_t *buffers[] = { &value };
    uint32_t sizes[] = { sizeof(value) };
    flashPageProgramBegin(0, nullptr);
    EXPECT_EQ(1U, flashPageProgramContinue(buffers, sizes, 1));
    EXPECT_EQ(0x39, memory[0]);
}

TEST_F(Fm25v02aTest, FlashInitRejectsUnclearableBlockProtection)
{
    statusRegister = 0x0C;
    statusWriteBlocked = true;

    EXPECT_FALSE(initializeStandardFram());
    EXPECT_EQ(0x0C, statusRegister);
    EXPECT_EQ(0U, flashGetGeometry()->totalSize);
}

extern "C" void captureWriteLength(uintptr_t bytes)
{
    callbackBytes = bytes;
}

TEST_F(Fm25v02aTest, ProgramWritesTwoBuffersAcrossLogicalPageBoundary)
{
    ASSERT_TRUE(initializeStandardFram());

    const uint8_t first[] = { 1, 2, 3 };
    const uint8_t second[] = { 4, 5, 6, 7 };
    const uint8_t *buffers[] = { first, second };
    uint32_t sizes[] = { sizeof(first), sizeof(second) };

    transactions.clear();
    flashPageProgramBegin(254, captureWriteLength);
    const uint32_t written = flashPageProgramContinue(buffers, sizes, 2);

    EXPECT_EQ(7U, written);
    EXPECT_TRUE(spiBusy);
    EXPECT_EQ(0U, callbackBytes);
    EXPECT_EQ(0xA5, memory[254]);

    completePendingSequence();

    EXPECT_EQ(7U, callbackBytes);
    const uint8_t expected[] = { 1, 2, 3, 4, 5, 6, 7 };
    EXPECT_TRUE(std::equal(std::begin(expected), std::end(expected), memory.begin() + 254));
    ASSERT_GE(transactions.size(), 4U);
    EXPECT_EQ(WREN, transactions[0][0]);
    EXPECT_EQ(WRITE, transactions[1][0]);

    const uint8_t next = 8;
    const uint8_t *nextBuffer[] = { &next };
    uint32_t nextSize[] = { sizeof(next) };
    EXPECT_EQ(1U, flashPageProgramContinue(nextBuffer, nextSize, 1));
    EXPECT_EQ(7U, callbackBytes);
    completePendingSequence();
    EXPECT_EQ(1U, callbackBytes);
    EXPECT_EQ(8, memory[261]);
}

TEST_F(Fm25v02aTest, ProgramClampsAtEndOfArray)
{
    ASSERT_TRUE(initializeStandardFram());

    const uint8_t first[] = { 1, 2 };
    const uint8_t second[] = { 3, 4, 5 };
    const uint8_t *buffers[] = { first, second };
    uint32_t sizes[] = { sizeof(first), sizeof(second) };

    flashPageProgramBegin(FM25V02A_TOTAL_SIZE - 3, captureWriteLength);
    EXPECT_EQ(3U, flashPageProgramContinue(buffers, sizes, 2));
    EXPECT_EQ(0U, callbackBytes);
    EXPECT_EQ(2U, sizes[0]);
    EXPECT_EQ(1U, sizes[1]);

    completePendingSequence();

    EXPECT_EQ(3U, callbackBytes);
    EXPECT_EQ(1, memory[FM25V02A_TOTAL_SIZE - 3]);
    EXPECT_EQ(2, memory[FM25V02A_TOTAL_SIZE - 2]);
    EXPECT_EQ(3, memory[FM25V02A_TOTAL_SIZE - 1]);

    EXPECT_EQ(0U, flashPageProgramContinue(buffers, sizes, 2));
}

TEST_F(Fm25v02aTest, ProgramWithOneBufferAndNoCallbackCompletesSynchronously)
{
    ASSERT_TRUE(initializeStandardFram());

    const uint8_t data[] = { 0x10, 0x20, 0x30 };
    const uint8_t *buffers[] = { data };
    uint32_t sizes[] = { sizeof(data) };

    flashPageProgramBegin(100, nullptr);
    EXPECT_EQ(3U, flashPageProgramContinue(buffers, sizes, 1));
    EXPECT_FALSE(spiBusy);
    EXPECT_TRUE(std::equal(std::begin(data), std::end(data), memory.begin() + 100));
}

TEST_F(Fm25v02aTest, SectorEraseWritesOnlyRequestedLogicalSectorToFF)
{
    identifyStandard();
    memory.fill(0x00);

    flashDevice.vTable->eraseSector(&flashDevice, 3072);

    EXPECT_TRUE(std::all_of(memory.begin() + 2048, memory.begin() + 4096,
        [](uint8_t byte) { return byte == 0xFF; }));
    EXPECT_EQ(0, memory[2047]);
    EXPECT_EQ(0, memory[4096]);
    ASSERT_GE(transactions.size(), 3U);
    EXPECT_EQ(WREN, transactions[0][0]);
    EXPECT_EQ(WRITE, transactions[1][0]);
}

TEST_F(Fm25v02aTest, CompleteEraseWritesEntireArrayToFF)
{
    identifyStandard();
    memory.fill(0x00);

    flashDevice.vTable->eraseCompletely(&flashDevice);

    EXPECT_TRUE(std::all_of(memory.begin(), memory.end(), [](uint8_t byte) { return byte == 0xFF; }));
    EXPECT_EQ(16U * 18U, transactions.size());
}

TEST_F(Fm25v02aTest, ReadClampsAtEndOfArray)
{
    identifyStandard();
    memory[FM25V02A_TOTAL_SIZE - 2] = 0x12;
    memory[FM25V02A_TOTAL_SIZE - 1] = 0x34;
    uint8_t buffer[4] = { 0 };

    EXPECT_EQ(2, flashDevice.vTable->readBytes(&flashDevice, FM25V02A_TOTAL_SIZE - 2, buffer, sizeof(buffer)));
    EXPECT_EQ(0x12, buffer[0]);
    EXPECT_EQ(0x34, buffer[1]);
    EXPECT_EQ(0, flashDevice.vTable->readBytes(&flashDevice, FM25V02A_TOTAL_SIZE, buffer, sizeof(buffer)));
}
