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

#include "platform.h"

#ifdef USE_SDCARD

#include "drivers/nvic.h"
#include "drivers/io.h"
#include "dma.h"
#include "dma_reqmap.h"

#include "drivers/bus_spi.h"
#include "drivers/time.h"

#include "pg/bus_spi.h"
#include "pg/sdcard.h"

#include "sdcard.h"
#include "sdcard_impl.h"
#include "sdcard_standard.h"

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    #define SDCARD_PROFILING
#endif

#define SDCARD_INIT_NUM_DUMMY_BYTES                 10
#define SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY     8
// Chosen so that CMD8 will have the same CRC as CMD0:
#define SDCARD_IF_COND_CHECK_PATTERN                0xAB

#define SDCARD_TIMEOUT_INIT_MILLIS                  200
#define SDCARD_MAX_CONSECUTIVE_FAILURES             8

/* SPI_CLOCK_INITIALIZATION (256) is the slowest (Spec calls for under 400KHz) */
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER     SPI_CLOCK_INITIALIZATION

/* Operational speed <= 25MHz */
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER         SPI_CLOCK_FAST

/* Break up 512-byte SD card sectors into chunks of this size when writing without DMA to reduce the peak overhead
 * per call to sdcard_poll().
 */
#define SDCARD_NON_DMA_CHUNK_SIZE                   256

sdcard_t sdcard;

STATIC_ASSERT(sizeof(sdcardCSD_t) == 16, sdcard_csd_bitfields_didnt_pack_properly);

static void sdcardInsertionDetectInit(const sdcardConfig_t *config)
{
    if (config->cardDetectTag) {
        sdcard.cardDetectPin = IOGetByTag(config->cardDetectTag);
        sdcard.detectionInverted = config->cardDetectInverted;
    } else {
        sdcard.cardDetectPin = IO_NONE;
        sdcard.detectionInverted = false;
    }

    if (sdcard.cardDetectPin) {
        IOInit(sdcard.cardDetectPin, OWNER_SDCARD_DETECT, 0);
        IOConfigGPIO(sdcard.cardDetectPin, IOCFG_IPU);
    }
}

/**
 * Detect if a SD card is physically present in the memory slot.
 */
bool sdcard_isInserted(void)
{
    bool result = true;
    if (sdcard.cardDetectPin) {
        result = IORead(sdcard.cardDetectPin) != 0;
        if (sdcard.detectionInverted) {
            result = !result;
        }
    }
    return result;
}

/**
 * Dispatch
 */
sdcardVTable_t *sdcardVTable;

void sdcard_preinit(const sdcardConfig_t *config)
{
#ifdef USE_SDCARD_SPI
    sdcardSpiVTable.sdcard_preinit(config);
#else
    UNUSED(config);
#endif
}

void sdcard_init(const sdcardConfig_t *config)
{
    sdcardInsertionDetectInit(config);

    switch (config->mode) {
#ifdef USE_SDCARD_SPI
    case SDCARD_MODE_SPI:
        sdcardVTable = &sdcardSpiVTable;
        break;
#endif
#ifdef USE_SDCARD_SDIO
    case SDCARD_MODE_SDIO:
        sdcardVTable = &sdcardSdioVTable;
        break;
#endif
    default:
        break;
    }

    if (sdcardVTable) {
        sdcardVTable->sdcard_init(config, spiPinConfig(0));
    }
}

bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    return sdcardVTable->sdcard_readBlock(blockIndex, buffer, callback, callbackData);
}

sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount)
{
    return sdcardVTable->sdcard_beginWriteBlocks(blockIndex, blockCount);
}

sdcardOperationStatus_e sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    return sdcardVTable->sdcard_writeBlock(blockIndex, buffer, callback, callbackData);
}

bool sdcard_poll(void)
{
    // sdcard_poll is called from taskMain() via afatfs_poll() and  for USE_SDCARD.
    if (sdcardVTable) {
        return sdcardVTable->sdcard_poll();
    } else {
        return false;
    }
}

bool sdcard_isFunctional(void)
{
    // sdcard_isFunctional is called from multiple places, including the case of hardware implementation
    // without a detect pin in which case sdcard_isInserted() always returns true.
    if (sdcardVTable) {
        return sdcardVTable->sdcard_isFunctional();
    } else {
        return false;
    }
}

bool sdcard_isInitialized(void)
{
    return sdcardVTable->sdcard_isInitialized();
}

const sdcardMetadata_t* sdcard_getMetadata(void)
{
    return sdcardVTable->sdcard_getMetadata();
}
#endif
