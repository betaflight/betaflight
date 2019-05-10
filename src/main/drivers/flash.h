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

#pragma once

#include <stdint.h>

#include "pg/flash.h"
#include "drivers/io.h"

// Maximum page size of all supported SPI flash devices.
// Used to detect flashfs allocation size being too small.
#define FLASH_MAX_PAGE_SIZE       2048

#define SPIFLASH_INSTRUCTION_RDID 0x9F

typedef enum {
    FLASH_TYPE_NOR = 0,
    FLASH_TYPE_NAND
} flashType_e;

typedef uint16_t flashSector_t;

typedef struct flashGeometry_s {
    flashSector_t sectors; // Count of the number of erasable blocks on the device
    uint16_t pageSize; // In bytes
    uint32_t sectorSize; // This is just pagesPerSector * pageSize
    uint32_t totalSize;  // This is just sectorSize * sectors
    uint16_t pagesPerSector;
    flashType_e flashType;
} flashGeometry_t;

void flashPreInit(const flashConfig_t *flashConfig);
bool flashInit(const flashConfig_t *flashConfig);

bool flashIsReady(void);
bool flashWaitForReady(uint32_t timeoutMillis);
void flashEraseSector(uint32_t address);
void flashEraseCompletely(void);
void flashPageProgramBegin(uint32_t address);
void flashPageProgramContinue(const uint8_t *data, int length);
void flashPageProgramFinish(void);
void flashPageProgram(uint32_t address, const uint8_t *data, int length);
int flashReadBytes(uint32_t address, uint8_t *buffer, int length);
void flashFlush(void);
const flashGeometry_t *flashGetGeometry(void);

//
// flash partitioning api
//

typedef struct flashPartition_s {
    uint8_t usage;
    flashSector_t startSector;
    flashSector_t endSector;
} flashPartition_t;

#define FLASH_PARTITION_SECTOR_COUNT(partition) (partition->endSector + 1 - partition->startSector) // + 1 for inclusive, start and end sector can be the same sector.

#define FLASH_PARTITION_UNKNOWN               0
#define FLASH_PARTITION_FLASHFS               1
#define FLASH_PARTITION_BADBLOCK_MANAGEMENT   2
#define FLASH_PARTITION_FIRMWARE              3

#define FLASH_MAX_PARTITIONS 3

typedef struct flashPartitionTable_s {
    flashPartition_t partitions[FLASH_MAX_PARTITIONS];
} flashPartitionTable_t;

void flashSetPartition(uint8_t index, const flashPartition_t *partition);
const flashPartition_t *flashFindPartitionByUsage(uint8_t usage);
