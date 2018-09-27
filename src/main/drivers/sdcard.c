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

#include "drivers/bus_spi.h"
#include "drivers/time.h"

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

void sdcardInsertionDetectDeinit(void)
{
    if (sdcard.cardDetectPin) {
        IOInit(sdcard.cardDetectPin, OWNER_FREE, 0);
        IOConfigGPIO(sdcard.cardDetectPin, IOCFG_IN_FLOATING);
    }
}

void sdcardInsertionDetectInit(void)
{
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
#endif
