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
#include <string.h>

#include <deque>
#include <map>
#include <vector>

extern "C" {
#include "platform.h"

#include "drivers/sdcard.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/asyncfatfs/fat_standard.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

/*
 * A simulated SD card holding a small FAT32 volume, generated sector by sector on demand so no image needs to be
 * stored. Reads and writes complete asynchronously on the next sdcard_poll(), the way the real driver behaves.
 *
 * Layout (one sector per cluster, so the volume needs more than 65524 clusters to be FAT32):
 *   sector 0                      MBR, one partition starting at sector 1
 *   sector 1                      volume ID
 *   sectors 2 .. 2+2*FAT_SECTORS  two copies of the FAT
 *   from CLUSTER_START_SECTOR     cluster 2 onwards, one sector each
 */
namespace {

const uint32_t SECTOR_SIZE = 512;
const uint32_t PARTITION_START = 1;
const uint32_t RESERVED_SECTORS = 1;
const uint32_t DATA_CLUSTERS = 65600;
const uint32_t FAT_ENTRIES_PER_SECTOR = SECTOR_SIZE / sizeof(uint32_t);
const uint32_t FAT_SECTORS = (DATA_CLUSTERS + 2 + FAT_ENTRIES_PER_SECTOR - 1) / FAT_ENTRIES_PER_SECTOR;
const uint32_t FAT_START_SECTOR = PARTITION_START + RESERVED_SECTORS;
const uint32_t CLUSTER_START_SECTOR = FAT_START_SECTOR + 2 * FAT_SECTORS;
const uint32_t PARTITION_SECTORS = RESERVED_SECTORS + 2 * FAT_SECTORS + DATA_CLUSTERS;
const uint32_t ROOT_CLUSTER = 2;
const uint32_t FAT32_END_OF_CHAIN = 0x0FFFFFFF;

struct pendingOperation_t {
    sdcardBlockOperation_e operation;
    uint32_t blockIndex;
    uint8_t *buffer;
    sdcard_operationCompleteCallback_c callback;
    uint32_t callbackData;
};

// Number of clusters in the root directory's chain, or 0 for a chain whose only cluster points back at itself
uint32_t rootChainLength = 1;
// Fill the root directory so a walk never finds a terminator entry
enum rootDirectoryFill_e {
    ROOT_DIRECTORY_EMPTY,
    ROOT_DIRECTORY_FULL_OF_DELETED_ENTRIES,
    ROOT_DIRECTORY_FULL_OF_FILES
};
rootDirectoryFill_e rootDirectoryFill = ROOT_DIRECTORY_EMPTY;
std::map<uint32_t, std::vector<uint8_t> > writtenSectors;
std::deque<pendingOperation_t> pendingOperations;
uint32_t readCount = 0;

void putU16(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xFF;
    p[1] = v >> 8;
}

void putU32(uint8_t *p, uint32_t v)
{
    putU16(p, v & 0xFFFF);
    putU16(p + 2, v >> 16);
}

void generateSector(uint32_t blockIndex, uint8_t *buffer)
{
    memset(buffer, 0, SECTOR_SIZE);

    if (blockIndex == 0) {
        // MBR with a single FAT32 LBA partition
        uint8_t *partition = buffer + 446;
        partition[4] = MBR_PARTITION_TYPE_FAT32_LBA;
        putU32(partition + 8, PARTITION_START);
        putU32(partition + 12, PARTITION_SECTORS);
        buffer[510] = FAT_VOLUME_ID_SIGNATURE_1;
        buffer[511] = FAT_VOLUME_ID_SIGNATURE_2;
    } else if (blockIndex == PARTITION_START) {
        // Volume ID
        buffer[0] = 0xEB; buffer[1] = 0x58; buffer[2] = 0x90;
        memcpy(buffer + 3, "MSWIN4.1", 8);
        putU16(buffer + 11, SECTOR_SIZE);
        buffer[13] = 1;                          // sectors per cluster
        putU16(buffer + 14, RESERVED_SECTORS);
        buffer[16] = 2;                          // number of FATs
        putU16(buffer + 17, 0);                  // root entry count (always 0 on FAT32)
        putU16(buffer + 19, 0);                  // total sectors (16 bit)
        buffer[21] = 0xF8;                       // media
        putU16(buffer + 22, 0);                  // FAT size (16 bit)
        putU32(buffer + 32, PARTITION_SECTORS);  // total sectors (32 bit)
        putU32(buffer + 36, FAT_SECTORS);        // FAT size (32 bit)
        putU32(buffer + 44, ROOT_CLUSTER);
        putU16(buffer + 48, 1);                  // FSInfo sector
        buffer[66] = 0x29;                       // extended boot signature
        memcpy(buffer + 71, "NO NAME    ", 11);
        memcpy(buffer + 82, "FAT32   ", 8);
        buffer[510] = FAT_VOLUME_ID_SIGNATURE_1;
        buffer[511] = FAT_VOLUME_ID_SIGNATURE_2;
    } else if (blockIndex >= FAT_START_SECTOR && blockIndex < CLUSTER_START_SECTOR) {
        // Both FAT copies are identical: the two reserved entries, then the root directory's chain running from
        // cluster 2 upwards, which either terminates after rootChainLength clusters or points back at itself.
        const uint32_t firstEntry = ((blockIndex - FAT_START_SECTOR) % FAT_SECTORS) * FAT_ENTRIES_PER_SECTOR;
        for (uint32_t i = 0; i < FAT_ENTRIES_PER_SECTOR; i++) {
            const uint32_t cluster = firstEntry + i;
            uint32_t value = 0;

            if (cluster == 0) {
                value = 0x0FFFFFF8;
            } else if (cluster == 1) {
                value = FAT32_END_OF_CHAIN;
            } else if (rootChainLength == 0) {
                value = cluster == ROOT_CLUSTER ? ROOT_CLUSTER : 0;
            } else if (cluster < ROOT_CLUSTER + rootChainLength) {
                value = cluster + 1 == ROOT_CLUSTER + rootChainLength ? FAT32_END_OF_CHAIN : cluster + 1;
            }

            putU32(buffer + i * sizeof(uint32_t), value);
        }
    } else if (blockIndex >= CLUSTER_START_SECTOR && rootDirectoryFill != ROOT_DIRECTORY_EMPTY) {
        const uint32_t cluster = blockIndex - CLUSTER_START_SECTOR + 2;

        if (cluster < ROOT_CLUSTER + (rootChainLength == 0 ? 1 : rootChainLength)) {
            // A root directory sector with no terminator to stop a walk on: either deleted entries, which an
            // allocation can reuse, or files, which it cannot
            for (uint32_t i = 0; i < SECTOR_SIZE; i += FAT_DIRECTORY_ENTRY_SIZE) {
                if (rootDirectoryFill == ROOT_DIRECTORY_FULL_OF_DELETED_ENTRIES) {
                    buffer[i] = FAT_DELETED_FILE_MARKER;
                } else {
                    memcpy(buffer + i, "FILE    TXT", FAT_FILENAME_LENGTH);
                    buffer[i + 11] = FAT_FILE_ATTRIBUTE_ARCHIVE;
                }
            }
        }
    }
}

void resetCard(uint32_t chainLength, rootDirectoryFill_e fill)
{
    rootChainLength = chainLength;
    rootDirectoryFill = fill;
    writtenSectors.clear();
    pendingOperations.clear();
    readCount = 0;
}

// Keep polling until the filesystem leaves the initialisation state, or give up
afatfsFilesystemState_e initialiseFilesystem(unsigned maxPolls)
{
    afatfs_destroy(true);
    afatfs_init();

    for (unsigned i = 0; i < maxPolls; i++) {
        afatfs_poll();

        if (afatfs_getFilesystemState() != AFATFS_FILESYSTEM_STATE_INITIALIZATION) {
            break;
        }
    }

    return afatfs_getFilesystemState();
}

} // namespace

extern "C" {

bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    readCount++;
    pendingOperations.push_back(pendingOperation_t{ SDCARD_BLOCK_OPERATION_READ, blockIndex, buffer, callback, callbackData });
    return true;
}

sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount)
{
    UNUSED(blockIndex);
    UNUSED(blockCount);
    return SDCARD_OPERATION_SUCCESS;
}

sdcardOperationStatus_e sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    pendingOperations.push_back(pendingOperation_t{ SDCARD_BLOCK_OPERATION_WRITE, blockIndex, buffer, callback, callbackData });
    return SDCARD_OPERATION_IN_PROGRESS;
}

bool sdcard_poll(void)
{
    // Complete everything queued since the last poll, in order
    std::deque<pendingOperation_t> completing;
    completing.swap(pendingOperations);

    while (!completing.empty()) {
        pendingOperation_t op = completing.front();
        completing.pop_front();

        if (op.operation == SDCARD_BLOCK_OPERATION_READ) {
            std::map<uint32_t, std::vector<uint8_t> >::const_iterator written = writtenSectors.find(op.blockIndex);
            if (written != writtenSectors.end()) {
                memcpy(op.buffer, written->second.data(), SECTOR_SIZE);
            } else {
                generateSector(op.blockIndex, op.buffer);
            }
        } else {
            writtenSectors[op.blockIndex] = std::vector<uint8_t>(op.buffer, op.buffer + SECTOR_SIZE);
        }

        if (op.callback) {
            op.callback(op.operation, op.blockIndex, op.buffer, op.callbackData);
        }
    }

    return true;
}

void sdcard_setProfilerCallback(sdcard_profilerCallback_c callback)
{
    UNUSED(callback);
}

} // extern "C"

TEST(AsyncFatFsTest, WellFormedCardInitialises)
{
    resetCard(1, ROOT_DIRECTORY_EMPTY);

    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_READY, initialiseFilesystem(200000));
    EXPECT_EQ(AFATFS_ERROR_NONE, afatfs_getLastError());

    afatfs_destroy(true);
}

// The FAT specification allows a directory of up to 65536 entries, and this volume has one sector per cluster
const uint32_t MAX_DIRECTORY_CLUSTERS = 65536 * FAT_DIRECTORY_ENTRY_SIZE / SECTOR_SIZE;

TEST(AsyncFatFsTest, RootDirectoryAtTheSizeLimitInitialises)
{
    // Exactly 65536 entries with no terminator: the walk must run to the end-of-chain marker and stop normally
    resetCard(MAX_DIRECTORY_CLUSTERS, ROOT_DIRECTORY_FULL_OF_DELETED_ENTRIES);

    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_READY, initialiseFilesystem(200000));
    EXPECT_EQ(AFATFS_ERROR_NONE, afatfs_getLastError());

    afatfs_destroy(true);
}

TEST(AsyncFatFsTest, FullRootDirectoryAtTheSizeLimitIsNotExtended)
{
    // 65536 files and no room for the freefile's entry: the directory must not be grown past the limit, since a
    // later walk would reject the longer chain. The card is left untouched and initialisation fails instead.
    resetCard(MAX_DIRECTORY_CLUSTERS, ROOT_DIRECTORY_FULL_OF_FILES);

    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_FATAL, initialiseFilesystem(200000));
    EXPECT_EQ(AFATFS_ERROR_GENERIC, afatfs_getLastError());
    EXPECT_TRUE(writtenSectors.empty());

    afatfs_destroy(true);
}

TEST(AsyncFatFsTest, RootDirectoryBeyondTheSizeLimitFailsInitialisation)
{
    // One cluster more than the specification allows: the chain is treated as broken
    resetCard(MAX_DIRECTORY_CLUSTERS + 1, ROOT_DIRECTORY_FULL_OF_DELETED_ENTRIES);

    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_FATAL, initialiseFilesystem(200000));

    afatfs_destroy(true);
}

TEST(AsyncFatFsTest, LoopedRootDirectoryChainFailsInitialisationInsteadOfHanging)
{
    resetCard(0, ROOT_DIRECTORY_FULL_OF_DELETED_ENTRIES);

    // Without a bound on the directory walk this never returns from afatfs_poll(): the root directory's only
    // cluster and its FAT sector are both cached, so every step of the walk succeeds immediately and the walk
    // never reaches a terminator entry or an end-of-chain marker.
    EXPECT_EQ(AFATFS_FILESYSTEM_STATE_FATAL, initialiseFilesystem(200000));

    // The walk gave up after at most the specification's directory size, not after reading the whole card
    EXPECT_LT(readCount, 1000u);

    afatfs_destroy(true);
}
