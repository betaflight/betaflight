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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH_CHIP

#include "drivers/flash/flash.h"
#include "drivers/flash/flash_impl.h"
#include "drivers/flash/flash_m25p16.h"
#include "drivers/flash/flash_w25n.h"
#include "drivers/flash/flash_w25q128fv.h"
#include "drivers/flash/flash_w25m.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_quadspi.h"
#include "drivers/bus_octospi.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/system.h"

#ifdef USE_FLASH_SPI

// 20 MHz max SPI frequency
#define FLASH_MAX_SPI_CLK_HZ 20000000
// 5 MHz max SPI init frequency
#define FLASH_MAX_SPI_INIT_CLK 5000000

static extDevice_t devInstance;
static extDevice_t *dev;

#endif

static flashDevice_t flashDevice;
static flashPartitionTable_t flashPartitionTable;
static int flashPartitions = 0;

#define FLASH_INSTRUCTION_RDID 0x9F

#ifdef USE_FLASH_MEMORY_MAPPED
MMFLASH_CODE_NOINLINE void flashMemoryMappedModeDisable(void)
{
    __disable_irq();
#ifdef USE_FLASH_OCTOSPI
    octoSpiDisableMemoryMappedMode(flashDevice.io.handle.octoSpi);
#else
#error Invalid configuration - Not implemented
#endif
}

MMFLASH_CODE_NOINLINE void flashMemoryMappedModeEnable(void)
{
#ifdef USE_FLASH_OCTOSPI
    octoSpiEnableMemoryMappedMode(flashDevice.io.handle.octoSpi);
    __enable_irq();
#else
#error Invalid configuration - Not implemented
#endif
}
#endif

#ifdef USE_FLASH_OCTOSPI
MMFLASH_CODE_NOINLINE static bool flashOctoSpiInit(const flashConfig_t *flashConfig)
{
    bool detected = false;

    enum {
        TRY_1LINE = 0, TRY_4LINE, BAIL
    } phase = TRY_1LINE;

#ifdef USE_FLASH_MEMORY_MAPPED
    bool memoryMappedModeEnabledOnBoot = isMemoryMappedModeEnabledOnBoot();
#else
    bool memoryMappedModeEnabledOnBoot = false;
#endif

#ifndef USE_OCTOSPI_EXPERIMENTAL
    if (!memoryMappedModeEnabledOnBoot) {
        return false; // Not supported yet, enable USE_OCTOSPI_EXPERIMENTAL and test/update implementation as required.
    }
#endif

    OCTOSPI_TypeDef *instance = octoSpiInstanceByDevice(OCTOSPI_CFG_TO_DEV(flashConfig->octoSpiDevice));

    flashDevice.io.handle.octoSpi = instance;
    flashDevice.io.mode = FLASHIO_OCTOSPI;

    if (memoryMappedModeEnabledOnBoot) {
        flashMemoryMappedModeDisable();
    }

    do {
#ifdef USE_OCTOSPI_EXPERIMENTAL
        if (!memoryMappedMode) {
            octoSpiSetDivisor(instance, OCTOSPI_CLOCK_INITIALISATION);
        }
#endif
        // for the memory-mapped use-case, we rely on the bootloader to have already selected the correct speed for the flash chip.

        // 3 bytes for what we need, but some IC's need 8 dummy cycles after the instruction, so read 4 and make two attempts to
        // assemble the chip id from the response.
        uint8_t readIdResponse[4];

        bool status = false;
        switch (phase) {
        case TRY_1LINE:
            status = octoSpiReceive1LINE(instance, FLASH_INSTRUCTION_RDID, 0, readIdResponse, 4);
            break;
        case TRY_4LINE:
            status = octoSpiReceive4LINES(instance, FLASH_INSTRUCTION_RDID, 2, readIdResponse, 3);
            break;
        default:
            break;
        }

        if (!status) {
            phase++;
            continue;
        }

#ifdef USE_OCTOSPI_EXPERIMENTAL
        if (!memoryMappedModeEnabledOnBoot) {
            octoSpiSetDivisor(instance, OCTOSPI_CLOCK_ULTRAFAST);
        }
#endif

        for (uint8_t offset = 0; offset <= 1 && !detected; offset++) {
#if defined(USE_FLASH_W25N01G) || defined(USE_FLASH_W25N02K) || defined(USE_FLASH_W25Q128FV) || defined(USE_FLASH_W25M02G)
            uint32_t jedecID = (readIdResponse[offset + 0] << 16) | (readIdResponse[offset + 1] << 8) | (readIdResponse[offset + 2]);
#endif

            if (offset == 0) {
#if defined(USE_FLASH_W25Q128FV)
                if (!detected && w25q128fv_identify(&flashDevice, jedecID)) {
                    detected = true;
                }
#endif
            }

            if (offset == 1) {
#ifdef USE_OCTOSPI_EXPERIMENTAL
                if (!memoryMappedModeEnabledOnBoot) {
                    // These flash chips DO NOT support memory mapped mode; suitable flash read commands must be available.
#if defined(USE_FLASH_W25N01G) || defined(USE_FLASH_W25N02K)
                    if (!detected && w25n_identify(&flashDevice, jedecID)) {
                        detected = true;
                    }
#endif
#if defined(USE_FLASH_W25M02G)
                    if (!detected && w25m_identify(&flashDevice, jedecID)) {
                        detected = true;
                    }
#endif
                }
#endif
            }
        }
        phase++;
    } while (phase != BAIL && !detected);

    if (memoryMappedModeEnabledOnBoot) {
        flashMemoryMappedModeEnable();
    }
    return detected;

}
#endif // USE_FLASH_OCTOSPI

#ifdef USE_FLASH_QUADSPI
static bool flashQuadSpiInit(const flashConfig_t *flashConfig)
{
    bool detected = false;

    enum { TRY_1LINE = 0, TRY_4LINE, BAIL};
    int phase = TRY_1LINE;

    QUADSPI_TypeDef *hqspi = quadSpiInstanceByDevice(QUADSPI_CFG_TO_DEV(flashConfig->quadSpiDevice));

    flashDevice.io.handle.quadSpi = hqspi;
    flashDevice.io.mode = FLASHIO_QUADSPI;

    do {
        quadSpiSetDivisor(hqspi, QUADSPI_CLOCK_INITIALISATION);

        // 3 bytes for what we need, but some IC's need 8 dummy cycles after the instruction, so read 4 and make two attempts to
        // assemble the chip id from the response.
        uint8_t readIdResponse[4];

        bool status = false;
        switch (phase) {
        case TRY_1LINE:
            status = quadSpiReceive1LINE(hqspi, FLASH_INSTRUCTION_RDID, 0, readIdResponse, 4);
            break;
        case TRY_4LINE:
            status = quadSpiReceive4LINES(hqspi, FLASH_INSTRUCTION_RDID, 2, readIdResponse, 3);
            break;
        default:
            break;
        }

        if (!status) {
            phase++;
            continue;
        }

        quadSpiSetDivisor(hqspi, QUADSPI_CLOCK_ULTRAFAST);

        for (uint8_t offset = 0; offset <= 1 && !detected; offset++) {

            uint32_t jedecID = (readIdResponse[offset + 0] << 16) | (readIdResponse[offset + 1] << 8) | (readIdResponse[offset + 2]);

            if (offset == 0) {
#if defined(USE_FLASH_W25Q128FV)
                if (!detected && w25q128fv_identify(&flashDevice, jedecID)) {
                    detected = true;
                }
#endif

#ifdef USE_FLASH_M25P16
                if (!detected && m25p16_identify(&flashDevice, jedecID)) {
                    detected = true;
                }
#endif
            }

            if (offset == 1) {
#if defined(USE_FLASH_W25N01G)
                if (!detected && w25n_identify(&flashDevice, jedecID)) {
                    detected = true;
                }
#endif
#if defined(USE_FLASH_W25M02G)
                if (!detected && w25m_identify(&flashDevice, jedecID)) {
                    detected = true;
                }
#endif
            }

            if (detected) {
                flashDevice.geometry.jedecId = jedecID;
            }
        }
        phase++;
    } while (phase != BAIL && !detected);

    return detected;
}
#endif  // USE_FLASH_QUADSPI

#ifdef USE_FLASH_SPI
static bool flashSpiInit(const flashConfig_t *flashConfig)
{
    bool detected = false;
    // Read chip identification and send it to device detect
    dev = &devInstance;

    if (flashConfig->csTag) {
        dev->busType_u.spi.csnPin = IOGetByTag(flashConfig->csTag);
    } else {
        return false;
    }

    if (!IOIsFreeOrPreinit(dev->busType_u.spi.csnPin)) {
        return false;
    }

    if (!spiSetBusInstance(dev, flashConfig->spiDevice)) {
        return false;
    }

    // Set the callback argument when calling back to this driver for DMA completion
    dev->callbackArg = (uintptr_t)&flashDevice;

    IOInit(dev->busType_u.spi.csnPin, OWNER_FLASH_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);

    //Maximum speed for standard READ command is 20mHz, other commands tolerate 25mHz
    spiSetClkDivisor(dev, spiCalculateDivider(FLASH_MAX_SPI_INIT_CLK));

    flashDevice.io.mode = FLASHIO_SPI;
    flashDevice.io.handle.dev = dev;

    delay(50); // short delay required after initialisation of SPI device instance.

    /*
     * Some newer chips require one dummy byte to be read; we can read
     * 4 bytes for these chips while retaining backward compatibility.
     */
    uint8_t readIdResponse[4] = { 0 };

    spiReadRegBuf(dev, FLASH_INSTRUCTION_RDID, readIdResponse, sizeof(readIdResponse));

    // Manufacturer, memory type, and capacity
    uint32_t jedecID = (readIdResponse[0] << 16) | (readIdResponse[1] << 8) | (readIdResponse[2]);

#ifdef USE_FLASH_M25P16
    if (m25p16_identify(&flashDevice, jedecID)) {
        detected = true;
    }
#endif

#if defined(USE_FLASH_W25M512) || defined(USE_FLASH_W25M)
    if (!detected && w25m_identify(&flashDevice, jedecID)) {
        detected = true;
    }
#endif

    if (!detected) {
        // Newer chips
        jedecID = (readIdResponse[1] << 16) | (readIdResponse[2] << 8) | (readIdResponse[3]);
    }

#ifdef USE_FLASH_W25N
    if (!detected && w25n_identify(&flashDevice, jedecID)) {
        detected = true;
    }
#endif

#ifdef USE_FLASH_W25M02G
    if (!detected && w25m_identify(&flashDevice, jedecID)) {
        detected = true;
    }
#endif

    if (detected) {
        flashDevice.geometry.jedecId = jedecID;
        return detected;
    }

    ioPreinitByTag(flashConfig->csTag, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
    return false;
}
#endif // USE_FLASH_SPI

void flashPreinit(const flashConfig_t *flashConfig)
{
    ioPreinitByTag(flashConfig->csTag, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
}

static bool flashDeviceInit(const flashConfig_t *flashConfig)
{
    bool haveFlash = false;

#if !defined(USE_FLASH_SPI) && !defined(USE_FLASH_QUADSPI) && !defined(USE_FLASH_OCTOSPI)
    UNUSED(flashConfig);
#endif

#ifdef USE_FLASH_SPI
    bool useSpi = (SPI_CFG_TO_DEV(flashConfig->spiDevice) != SPIINVALID);

    if (useSpi) {
        haveFlash = flashSpiInit(flashConfig);
    }
#endif

#ifdef USE_FLASH_QUADSPI
    bool useQuadSpi = (QUADSPI_CFG_TO_DEV(flashConfig->quadSpiDevice) != QUADSPIINVALID);
    if (useQuadSpi) {
        haveFlash = flashQuadSpiInit(flashConfig);
    }
#endif

#ifdef USE_FLASH_OCTOSPI
    bool useOctoSpi = (OCTOSPI_CFG_TO_DEV(flashConfig->octoSpiDevice) != OCTOSPIINVALID);
    if (useOctoSpi) {
        haveFlash = flashOctoSpiInit(flashConfig);
    }
#endif

    if (haveFlash && flashDevice.vTable->configure) {
        uint32_t configurationFlags = 0;

#ifdef USE_FLASH_MEMORY_MAPPED
        if (isMemoryMappedModeEnabledOnBoot()) {
            configurationFlags |= FLASH_CF_SYSTEM_IS_MEMORY_MAPPED;
        }
#endif

        flashDevice.vTable->configure(&flashDevice, configurationFlags);
    }

    return haveFlash;
}

MMFLASH_CODE bool flashIsReady(void)
{
    return flashDevice.vTable->isReady(&flashDevice);
}

MMFLASH_CODE bool flashWaitForReady(void)
{
    return flashDevice.vTable->waitForReady(&flashDevice);
}

MMFLASH_CODE void flashEraseSector(uint32_t address)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->eraseSector(&flashDevice, address);
}

void flashEraseCompletely(void)
{
    flashDevice.callback = NULL;
    flashDevice.vTable->eraseCompletely(&flashDevice);
}

/* The callback, if provided, will receive the totoal number of bytes transfered
 * by each call to flashPageProgramContinue() once the transfer completes.
 */
MMFLASH_CODE void flashPageProgramBegin(uint32_t address, void (*callback)(uint32_t length))
{
    flashDevice.vTable->pageProgramBegin(&flashDevice, address, callback);
}

MMFLASH_CODE uint32_t flashPageProgramContinue(const uint8_t **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    uint32_t maxBytesToWrite = flashDevice.geometry.pageSize - (flashDevice.currentWriteAddress % flashDevice.geometry.pageSize);

    if (bufferCount == 0) {
        return 0;
    }

    if (bufferSizes[0] >= maxBytesToWrite) {
        bufferSizes[0] = maxBytesToWrite;
        bufferCount = 1;
    } else {
        maxBytesToWrite -= bufferSizes[0];
        if ((bufferCount == 2) && (bufferSizes[1] > maxBytesToWrite)) {
            bufferSizes[1] = maxBytesToWrite;
        }
    }

    return flashDevice.vTable->pageProgramContinue(&flashDevice, buffers, bufferSizes, bufferCount);
}

MMFLASH_CODE void flashPageProgramFinish(void)
{
    flashDevice.vTable->pageProgramFinish(&flashDevice);
}

MMFLASH_CODE void flashPageProgram(uint32_t address, const uint8_t *data, uint32_t length, void (*callback)(uint32_t length))
{
    flashDevice.vTable->pageProgram(&flashDevice, address, data, length, callback);
}

MMFLASH_CODE int flashReadBytes(uint32_t address, uint8_t *buffer, uint32_t length)
{
    flashDevice.callback = NULL;
    return flashDevice.vTable->readBytes(&flashDevice, address, buffer, length);
}

MMFLASH_CODE void flashFlush(void)
{
    if (flashDevice.vTable->flush) {
        flashDevice.vTable->flush(&flashDevice);
    }
}

static const flashGeometry_t noFlashGeometry = {
    .totalSize = 0,
};

const flashGeometry_t *flashGetGeometry(void)
{
    if (flashDevice.vTable && flashDevice.vTable->getGeometry) {
        return flashDevice.vTable->getGeometry(&flashDevice);
    }

    return &noFlashGeometry;
}

/*
 * Flash partitioning
 *
 * Partition table is not currently stored on the flash, in-memory only.
 *
 * Partitions are required so that Badblock management (inc spare blocks), FlashFS (Blackbox Logging), Configuration and Firmware can be kept separate and tracked.
 *
 * XXX FIXME
 * XXX Note that Flash FS must start at sector 0.
 * XXX There is existing blackbox/flash FS code the relies on this!!!
 * XXX This restriction can and will be fixed by creating a set of flash operation functions that take partition as an additional parameter.
 */

static void flashConfigurePartitions(void)
{
#if defined(FIRMWARE_SIZE) || defined(CONFIG_IN_EXTERNAL_FLASH) || defined(USE_FLASHFS)
    const flashGeometry_t *flashGeometry = flashGetGeometry();
    if (flashGeometry->totalSize == 0) {
        return;
    }

    flashSector_t startSector = 0;
    flashSector_t endSector = flashGeometry->sectors - 1; // 0 based index

    const flashPartition_t *badBlockPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_BADBLOCK_MANAGEMENT);
    if (badBlockPartition) {
        endSector = badBlockPartition->startSector - 1;
    }
#endif

#if defined(FIRMWARE_SIZE) && defined(USE_FIRMWARE_PARTITION)
    const uint32_t firmwareSize = (FIRMWARE_SIZE * 1024);
    flashSector_t firmwareSectors = (firmwareSize / flashGeometry->sectorSize);

    if (firmwareSize % flashGeometry->sectorSize > 0) {
        firmwareSectors++; // needs a portion of a sector.
    }

    startSector = (endSector + 1) - firmwareSectors; // + 1 for inclusive

    flashPartitionSet(FLASH_PARTITION_TYPE_FIRMWARE, startSector, endSector);

    endSector = startSector - 1;
    startSector = 0;
#endif

#if defined(CONFIG_IN_EXTERNAL_FLASH) || defined(CONFIG_IN_MEMORY_MAPPED_FLASH)
    const uint32_t configSize = EEPROM_SIZE;
    flashSector_t configSectors = (configSize / flashGeometry->sectorSize);

    if (configSize % flashGeometry->sectorSize > 0) {
        configSectors++; // needs a portion of a sector.
    }

    startSector = (endSector + 1) - configSectors; // + 1 for inclusive

    flashPartitionSet(FLASH_PARTITION_TYPE_CONFIG, startSector, endSector);

    endSector = startSector - 1;
    startSector = 0;
#endif

#ifdef USE_FLASHFS
    flashPartitionSet(FLASH_PARTITION_TYPE_FLASHFS, startSector, endSector);
#endif
}

flashPartition_t *flashPartitionFindByType(flashPartitionType_e type)
{
    for (int index = 0; index < FLASH_MAX_PARTITIONS; index++) {
        flashPartition_t *candidate = &flashPartitionTable.partitions[index];
        if (candidate->type == type) {
            return candidate;
        }
    }

    return NULL;
}

const flashPartition_t *flashPartitionFindByIndex(uint8_t index)
{
    if (index >= flashPartitions) {
        return NULL;
    }

    return &flashPartitionTable.partitions[index];
}

void flashPartitionSet(uint8_t type, uint32_t startSector, uint32_t endSector)
{
    flashPartition_t *entry = flashPartitionFindByType(type);

    if (!entry) {
        if (flashPartitions == FLASH_MAX_PARTITIONS - 1) {
            return;
        }
        entry = &flashPartitionTable.partitions[flashPartitions++];
    }

    entry->type = type;
    entry->startSector = startSector;
    entry->endSector = endSector;
}

// Must be in sync with FLASH_PARTITION_TYPE
static const char *flashPartitionNames[] = {
    "UNKNOWN  ",
    "PARTITION",
    "FLASHFS  ",
    "BBMGMT   ",
    "FIRMWARE ",
    "CONFIG   ",
};

const char *flashPartitionGetTypeName(flashPartitionType_e type)
{
    if (type < ARRAYLEN(flashPartitionNames)) {
        return flashPartitionNames[type];
    }

    return NULL;
}

bool flashInit(const flashConfig_t *flashConfig)
{
    memset(&flashPartitionTable, 0x00, sizeof(flashPartitionTable));
    flashPartitions = 0;

    bool haveFlash = flashDeviceInit(flashConfig);

    flashConfigurePartitions();

    return haveFlash;
}

int flashPartitionCount(void)
{
    return flashPartitions;
}
#endif // USE_FLASH_CHIP
