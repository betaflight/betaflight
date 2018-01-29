/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "drivers/nvic.h"
#include "drivers/io.h"
#include "drivers/dma.h"

#include "drivers/time.h"

#include "drivers/sdcard.h"
#include "drivers/sdcard_standard.h"

#include "drivers/sdio_stdlib.h"

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    #define SDCARD_PROFILING
#endif

#define SDCARD_TIMEOUT_INIT_MILLIS      200
#define SDCARD_MAX_CONSECUTIVE_FAILURES 8

typedef enum {
    // In these states we run at the initialization 400kHz clockspeed:
    SDCARD_STATE_NOT_PRESENT = 0,
    SDCARD_STATE_RESET,
    SDCARD_STATE_CARD_INIT_IN_PROGRESS,
    SDCARD_STATE_INITIALIZATION_RECEIVE_CID,

    // In these states we run at full clock speed
    SDCARD_STATE_READY,
    SDCARD_STATE_READING,
    SDCARD_STATE_SENDING_WRITE,
    SDCARD_STATE_WAITING_FOR_WRITE,
    SDCARD_STATE_WRITING_MULTIPLE_BLOCKS,
    SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE
} sdcardState_e;

typedef struct sdcard_t {
    struct {
        uint8_t *buffer;
        uint32_t blockIndex;
        uint8_t chunkIndex;

        sdcard_operationCompleteCallback_c callback;
        uint32_t callbackData;

#ifdef SDCARD_PROFILING
        uint32_t profileStartTime;
#endif
    } pendingOperation;

    uint32_t operationStartTime;

    uint8_t failureCount;

    uint8_t version;
    bool highCapacity;

    uint32_t multiWriteNextBlock;
    uint32_t multiWriteBlocksRemain;

    sdcardState_e state;

    sdcardMetadata_t metadata;
    sdcardCSD_t csd;

#ifdef SDIO_DMA
    volatile uint8_t TxDMA_Cplt;
    volatile uint8_t RxDMA_Cplt;
#endif

#ifdef SDCARD_PROFILING
    sdcard_profilerCallback_c profiler;
#endif
} sdcard_t;

static sdcard_t sdcard;

STATIC_ASSERT(sizeof(sdcardCSD_t) == 16, sdcard_csd_bitfields_didnt_pack_properly);

void sdcardInsertionDetectDeinit(void)
{
    //Handled by the driver
    return;
}

void sdcardInsertionDetectInit(void)
{
    //Handled by the driver
    return;
}

/**
 * Detect if a SD card is physically present in the memory slot.
 */
bool sdcard_isInserted(void)
{
    return SD_Detect();
}

/**
 * Returns true if the card has already been, or is currently, initializing and hasn't encountered enough errors to
 * trip our error threshold and be disabled (i.e. our card is in and working!)
 */
bool sdcard_isFunctional(void)
{
    return sdcard.state != SDCARD_STATE_NOT_PRESENT;
}

/**
 * Handle a failure of an SD card operation by resetting the card back to its initialization phase.
 *
 * Increments the failure counter, and when the failure threshold is reached, disables the card until
 * the next call to sdcard_init().
 */
static void sdcard_reset(void)
{
    SD_DeInit();
    delay(200);
    SD_Init();

    sdcard.failureCount++;
    if (sdcard.failureCount >= SDCARD_MAX_CONSECUTIVE_FAILURES || sdcard_isInserted() == SD_NOT_PRESENT) {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
    } else {
        sdcard.operationStartTime = millis();
        sdcard.state = SDCARD_STATE_RESET;
    }
}

typedef enum {
    SDCARD_RECEIVE_SUCCESS,
    SDCARD_RECEIVE_BLOCK_IN_PROGRESS,
    SDCARD_RECEIVE_ERROR
} sdcardReceiveBlockStatus_e;

/**
 * Attempt to receive a data block from the SD card.
 *
 * Return true on success, otherwise the card has not responded yet and you should retry later.
 */
static sdcardReceiveBlockStatus_e sdcard_receiveDataBlock(uint8_t *buffer, int count)
{
    UNUSED(buffer);
    UNUSED(count);
    SD_Error ret = SD_WaitReadOperation();

    if (ret == SD_REQUEST_PENDING) {
        return SDCARD_RECEIVE_BLOCK_IN_PROGRESS;
    }

    if (SD_GetStatus() != SD_TRANSFER_OK) {
        return SDCARD_RECEIVE_ERROR;
    }

    return SDCARD_RECEIVE_SUCCESS;
}

static bool sdcard_receiveCID(void)
{
    SD_CardInfo sdinfo;
    SD_GetCardInfo(&sdinfo);

    sdcard.metadata.manufacturerID = sdinfo.SD_cid.ManufacturerID;
    sdcard.metadata.oemID = sdinfo.SD_cid.OEM_AppliID;
    sdcard.metadata.productName[0] = (sdinfo.SD_cid.ProdName1 & 0xFF000000) >> 24;
    sdcard.metadata.productName[1] = (sdinfo.SD_cid.ProdName1 & 0x00FF0000) >> 16;
    sdcard.metadata.productName[2] = (sdinfo.SD_cid.ProdName1 & 0x0000FF00) >> 8;
    sdcard.metadata.productName[3] = (sdinfo.SD_cid.ProdName1 & 0x000000FF) >> 0;
    sdcard.metadata.productName[4] = sdinfo.SD_cid.ProdName2;
    sdcard.metadata.productRevisionMajor = sdinfo.SD_cid.ProdRev >> 4;
    sdcard.metadata.productRevisionMinor = sdinfo.SD_cid.ProdRev & 0x0F;
    sdcard.metadata.productSerial = sdinfo.SD_cid.ProdSN;
    sdcard.metadata.productionYear = (((sdinfo.SD_cid.ManufactDate & 0x0F00) >> 8) | ((sdinfo.SD_cid.ManufactDate & 0xFF) >> 4)) + 2000;
    sdcard.metadata.productionMonth = sdinfo.SD_cid.ManufactDate & 0x000F;

    return true;
}

static bool sdcard_fetchCSD(void)
{
    /* The CSD command's data block should always arrive within 8 idle clock cycles (SD card spec). This is because
     * the information about card latency is stored in the CSD register itself, so we can't use that yet!
     */
    SD_CardInfo sdinfo;
    SD_GetCardInfo(&sdinfo);

    sdcard.metadata.numBlocks = sdinfo.CardCapacity / sdinfo.CardBlockSize;
    return (bool) 1;
}

/**
 * Check if the SD Card has completed its startup sequence. Must be called with sdcard.state == SDCARD_STATE_INITIALIZATION.
 *
 * Returns true if the card has finished its init process.
 */
static bool sdcard_checkInitDone(void)
{
    uint8_t ret = SD_GetStatus();

    if (ret == SD_TRANSFER_OK) {
        SD_CardInfo sdinfo;
        SD_GetCardInfo(&sdinfo);

        sdcard.version = (sdinfo.CardType) ? 2 : 1;
        sdcard.highCapacity = (sdinfo.CardType == 2) ? 1 : 0;
    }

    // When card init is complete, the idle bit in the response becomes zero.
    return (ret == SD_TRANSFER_OK) ? true : false;
}

/**
 * Begin the initialization process for the SD card. This must be called first before any other sdcard_ routine.
 */
void sdcard_init(const sdcardConfig_t *config)
{
    UNUSED(config);
    if (SD_Detect() == SD_PRESENT) {
        if (SD_Init() != SD_OK) {
            sdcard.state = SDCARD_STATE_NOT_PRESENT;
            sdcard.failureCount++;
            return;
        }
    } else {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
        sdcard.failureCount++;
        return;
    }

    sdcard.operationStartTime = millis();
    sdcard.state = SDCARD_STATE_RESET;
    sdcard.failureCount = 0;
}

/*
 * Returns true if the card is ready to accept read/write commands.
 */
static bool sdcard_isReady()
{
    return sdcard.state == SDCARD_STATE_READY || sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
}

/**
 * Send the stop-transmission token to complete a multi-block write.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - We're now waiting for that stop to complete, the card will enter
 *                                    the SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE state.
 *     SDCARD_OPERATION_SUCCESS     - The multi-block write finished immediately, the card will enter
 *                                    the SDCARD_READY state.
 *
 */
static sdcardOperationStatus_e sdcard_endWriteBlocks()
{
    sdcard.multiWriteBlocksRemain = 0;

    // 8 dummy clocks to guarantee N_WR clocks between the last card response and this token

    // Card may choose to raise a busy (non-0xFF) signal after at most N_BR (1 byte) delay
    if (SD_GetStatus() == SD_TRANSFER_OK) {
        sdcard.state = SDCARD_STATE_READY;
        return SDCARD_OPERATION_SUCCESS;
    } else {
        sdcard.state = SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE;
        sdcard.operationStartTime = millis();

        return SDCARD_OPERATION_IN_PROGRESS;
    }
}
/**
 * Call periodically for the SD card to perform in-progress transfers.
 *
 * Returns true if the card is ready to accept commands.
 */
bool sdcard_poll(void)
{
    uint8_t initStatus;
    UNUSED(initStatus);

#ifdef SDCARD_PROFILING
    bool profilingComplete;
#endif

    doMore:
    switch (sdcard.state) {
        case SDCARD_STATE_RESET:
                //HAL Takes care of voltage crap.
            sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS;
            goto doMore;
        break;

        case SDCARD_STATE_CARD_INIT_IN_PROGRESS:
            if (sdcard_checkInitDone()) {
                // Now fetch the CSD and CID registers
                if (sdcard_fetchCSD()) {
                    sdcard.state = SDCARD_STATE_INITIALIZATION_RECEIVE_CID;
                    goto doMore;
                } else {
                    sdcard_reset();
                    goto doMore;
                }
            }
        break;
        case SDCARD_STATE_INITIALIZATION_RECEIVE_CID:
            if (sdcard_receiveCID()) {

                /* The spec is a little iffy on what the default block size is for Standard Size cards (it can be changed on
                 * standard size cards) so let's just set it to 512 explicitly so we don't have a problem.
                 */
//                if (!sdcard.highCapacity && SDMMC_CmdBlockLength(_HSD.Instance, SDCARD_BLOCK_SIZE)) {
//                    sdcard_reset();
//                    goto doMore;
//                }

                sdcard.multiWriteBlocksRemain = 0;

                sdcard.state = SDCARD_STATE_READY;
                goto doMore;
            } // else keep waiting for the CID to arrive
        break;
        case SDCARD_STATE_SENDING_WRITE:
            // Have we finished sending the write yet?
            if (SD_WaitWriteOperation() == SD_OK) {

                // The SD card is now busy committing that write to the card
                sdcard.state = SDCARD_STATE_WAITING_FOR_WRITE;
                sdcard.operationStartTime = millis();

                // Since we've transmitted the buffer we can go ahead and tell the caller their operation is complete
                if (sdcard.pendingOperation.callback) {
                    sdcard.pendingOperation.callback(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, sdcard.pendingOperation.buffer, sdcard.pendingOperation.callbackData);
                }
            }
        break;
        case SDCARD_STATE_WAITING_FOR_WRITE:
            if (SD_GetStatus() == SD_TRANSFER_OK) {
#ifdef SDCARD_PROFILING
                profilingComplete = true;
#endif

                sdcard.failureCount = 0; // Assume the card is good if it can complete a write

                // Still more blocks left to write in a multi-block chain?
                if (sdcard.multiWriteBlocksRemain > 1) {
                    sdcard.multiWriteBlocksRemain--;
                    sdcard.multiWriteNextBlock++;
                    sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
                } else if (sdcard.multiWriteBlocksRemain == 1) {
                    // This function changes the sd card state for us whether immediately succesful or delayed:
                    sdcard.multiWriteBlocksRemain = 0;
                } else {
                    sdcard.state = SDCARD_STATE_READY;
                }

#ifdef SDCARD_PROFILING
                if (profilingComplete && sdcard.profiler) {
                    sdcard.profiler(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                }
#endif
            } else if (millis() > sdcard.operationStartTime + SDCARD_TIMEOUT_WRITE_MSEC) {
                /*
                 * The caller has already been told that their write has completed, so they will have discarded
                 * their buffer and have no hope of retrying the operation. But this should be very rare and it allows
                 * them to reuse their buffer milliseconds faster than they otherwise would.
                 */
                sdcard_reset();
                goto doMore;
            }
        break;
        case SDCARD_STATE_READING:
            switch (sdcard_receiveDataBlock(sdcard.pendingOperation.buffer, SDCARD_BLOCK_SIZE)) {
                case SDCARD_RECEIVE_SUCCESS:

                    sdcard.state = SDCARD_STATE_READY;
                    sdcard.failureCount = 0; // Assume the card is good if it can complete a read

#ifdef SDCARD_PROFILING
                    if (sdcard.profiler) {
                        sdcard.profiler(SDCARD_BLOCK_OPERATION_READ, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                    }
#endif

                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(
                            SDCARD_BLOCK_OPERATION_READ,
                            sdcard.pendingOperation.blockIndex,
                            sdcard.pendingOperation.buffer,
                            sdcard.pendingOperation.callbackData
                        );
                    }
                break;
                case SDCARD_RECEIVE_BLOCK_IN_PROGRESS:
                    if (millis() <= sdcard.operationStartTime + SDCARD_TIMEOUT_READ_MSEC) {
                        break; // Timeout not reached yet so keep waiting
                    }
                    // Timeout has expired, so fall through to convert to a fatal error

                case SDCARD_RECEIVE_ERROR:
                    goto doMore;
                break;
            }
        break;
        case SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE:
            if (SD_GetStatus() == SD_TRANSFER_OK) {
                sdcard.state = SDCARD_STATE_READY;

#ifdef SDCARD_PROFILING
                if (sdcard.profiler) {
                    sdcard.profiler(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                }
#endif
            } else if (millis() > sdcard.operationStartTime + SDCARD_TIMEOUT_WRITE_MSEC) {
                sdcard_reset();
                goto doMore;
            }
        break;
        case SDCARD_STATE_NOT_PRESENT:
        default:
            ;
    }

    // Is the card's initialization taking too long?
    if (sdcard.state >= SDCARD_STATE_RESET && sdcard.state < SDCARD_STATE_READY
            && millis() - sdcard.operationStartTime > SDCARD_TIMEOUT_INIT_MILLIS) {
        sdcard_reset();
    }

    return sdcard_isReady();
}

/**
 * Write the 512-byte block from the given buffer into the block with the given index.
 *
 * If the write does not complete immediately, your callback will be called later. If the write was successful, the
 * buffer pointer will be the same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - Your buffer is currently being transmitted to the card and your callback will be
 *                                    called later to report the completion. The buffer pointer must remain valid until
 *                                    that time.
 *     SDCARD_OPERATION_SUCCESS     - Your buffer has been transmitted to the card now.
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - Your write was rejected by the card, card will be reset
 */
sdcardOperationStatus_e sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{

#ifdef SDCARD_PROFILING
    sdcard.pendingOperation.profileStartTime = micros();
#endif

    doMore:
    switch (sdcard.state) {
        case SDCARD_STATE_WRITING_MULTIPLE_BLOCKS:
            // Do we need to cancel the previous multi-block write?
            if (blockIndex != sdcard.multiWriteNextBlock) {
                if (sdcard_endWriteBlocks() == SDCARD_OPERATION_SUCCESS) {
                    // Now we've entered the ready state, we can try again
                    goto doMore;
                } else {
                    return SDCARD_OPERATION_BUSY;
                }
            }

            // We're continuing a multi-block write
        break;
        case SDCARD_STATE_READY:
        break;
        default:
            return SDCARD_OPERATION_BUSY;
    }

    sdcard.pendingOperation.buffer = buffer;
    sdcard.pendingOperation.blockIndex = blockIndex;
    sdcard.pendingOperation.callback = callback;
    sdcard.pendingOperation.callbackData = callbackData;
    sdcard.pendingOperation.chunkIndex = 1; // (for non-DMA transfers) we've sent chunk #0 already
    sdcard.state = SDCARD_STATE_SENDING_WRITE;

    if (SD_WriteBlock(buffer, blockIndex * 512, 1) != SD_OK) {
        /* Our write was rejected! This could be due to a bad address but we hope not to attempt that, so assume
         * the card is broken and needs reset.
         */
        sdcard_reset();

        // Announce write failure:
        if (sdcard.pendingOperation.callback) {
            sdcard.pendingOperation.callback(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, NULL, sdcard.pendingOperation.callbackData);
        }
            return SDCARD_OPERATION_FAILURE;
    }

    return SDCARD_OPERATION_IN_PROGRESS;
}

/**
 * Begin writing a series of consecutive blocks beginning at the given block index. This will allow (but not require)
 * the SD card to pre-erase the number of blocks you specifiy, which can allow the writes to complete faster.
 *
 * Afterwards, just call sdcard_writeBlock() as normal to write those blocks consecutively.
 *
 * It's okay to abort the multi-block write at any time by writing to a non-consecutive address, or by performing a read.
 *
 * Returns:
 *     SDCARD_OPERATION_SUCCESS     - Multi-block write has been queued
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - A fatal error occured, card will be reset
 */
sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount)
{
    if (sdcard.state != SDCARD_STATE_READY) {
        if (sdcard.state == SDCARD_STATE_WRITING_MULTIPLE_BLOCKS) {
            if (blockIndex == sdcard.multiWriteNextBlock) {
                // Assume that the caller wants to continue the multi-block write they already have in progress!
                return SDCARD_OPERATION_SUCCESS;
            } else if (sdcard_endWriteBlocks() != SDCARD_OPERATION_SUCCESS) {
                return SDCARD_OPERATION_BUSY;
            } // Else we've completed the previous multi-block write and can fall through to start the new one
        } else {
            return SDCARD_OPERATION_BUSY;
        }
    }

//    if (BSP_SD_Erase(blockIndex, blockCount * 512 + blockIndex)) {
        sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
        sdcard.multiWriteBlocksRemain = blockCount;
        sdcard.multiWriteNextBlock = blockIndex;
        // Leave the card selected
        return SDCARD_OPERATION_SUCCESS;
//    } else {
//        sdcard_reset();
//        return SDCARD_OPERATION_FAILURE;
//    }
}

/**
 * Read the 512-byte block with the given index into the given 512-byte buffer.
 *
 * When the read completes, your callback will be called. If the read was successful, the buffer pointer will be the
 * same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * You must keep the pointer to the buffer valid until the operation completes!
 *
 * Returns:
 *     true - The operation was successfully queued for later completion, your callback will be called later
 *     false - The operation could not be started due to the card being busy (try again later).
 */
bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    if (sdcard.state != SDCARD_STATE_READY) {
            return false;
    }

#ifdef SDCARD_PROFILING
    sdcard.pendingOperation.profileStartTime = micros();
#endif

    // Standard size cards use byte addressing, high capacity cards use block addressing
    uint8_t status = SD_ReadBlock(buffer, blockIndex * 512, 1);

    if (status == SD_OK) {
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = blockIndex;
        sdcard.pendingOperation.callback = callback;
        sdcard.pendingOperation.callbackData = callbackData;

        sdcard.state = SDCARD_STATE_READING;

        sdcard.operationStartTime = millis();

        return true;
    } else {
        sdcard_reset();
        if (sdcard.pendingOperation.callback) {
            sdcard.pendingOperation.callback(
                SDCARD_BLOCK_OPERATION_READ,
                sdcard.pendingOperation.blockIndex,
                NULL,
                sdcard.pendingOperation.callbackData
            );
        }
        return false;
    }
}

/**
 * Returns true if the SD card has successfully completed its startup procedures.
 */
bool sdcard_isInitialized(void)
{
    return sdcard.state >= SDCARD_STATE_READY;
}

const sdcardMetadata_t* sdcard_getMetadata(void)
{
    return &sdcard.metadata;
}

#ifdef SDCARD_PROFILING

void sdcard_setProfilerCallback(sdcard_profilerCallback_c callback)
{
    sdcard.profiler = callback;
}

#endif

#endif
