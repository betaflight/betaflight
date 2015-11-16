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

#include "sdcard.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/bus_spi.h"
#include "drivers/system.h"

#include "sdcard_standard.h"

#ifdef USE_SDCARD

#define SET_CS_HIGH          GPIO_SetBits(SDCARD_SPI_CS_GPIO,   SDCARD_SPI_CS_PIN)
#define SET_CS_LOW           GPIO_ResetBits(SDCARD_SPI_CS_GPIO, SDCARD_SPI_CS_PIN)

#define DESELECT_SDCARD      SET_CS_HIGH
#define SELECT_SDCARD        SET_CS_LOW

#define SDCARD_INIT_NUM_DUMMY_BYTES 10
#define SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY 8
// Chosen so that CMD8 will have the same CRC as CMD0:
#define SDCARD_IF_COND_CHECK_PATTERN 0xAB

#define STATIC_ASSERT(condition, name ) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ]

typedef enum {
    SDCARD_STATE_NOT_PRESENT = 0,
    SDCARD_STATE_INITIALIZATION,
    SDCARD_STATE_READY,
    SDCARD_STATE_READING,
    SDCARD_STATE_WRITING,
} sdcardState_e;

typedef struct sdcard_t {
    struct {
        uint8_t *buffer;
        int error;
        uint32_t blockIndex;

        sdcard_operationCompleteCallback_c callback;
        uint32_t callbackData;
    } pendingOperation;

    uint8_t version;
    bool highCapacity;

    sdcardMetadata_t metadata;
    sdcardCSD_t csd;

    sdcardState_e state;
} sdcard_t;


static sdcard_t sdcard;

STATIC_ASSERT(sizeof(sdcardCSD_t) == 16, sdcard_csd_bitfields_didnt_pack_properly);


/**
 * The SD card spec requires 8 clock cycles to be sent by us on the bus after most commands so it can finish its
 * processing of that command. The easiest way for us to do this is to just wait for the bus to become idle before
 * we transmit a command, sending at least 8-bits onto the bus when we do so.
 */
static bool sdcard_waitForIdle(int maxBytesToWait)
{
    while (maxBytesToWait > 0) {
        uint8_t b = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
        if (b == 0xFF) {
            return true;
        }
        maxBytesToWait--;
    }

    return false;
}

/**
 * Wait for up to maxDelay 0xFF idle bytes to arrive from the card, returning the first non-idle byte found.
 *
 * Returns 0xFF on failure.
 */
static uint8_t sdcard_waitForNonIdleByte(int maxDelay)
{
    for (int i = 0; i < maxDelay + 1; i++) { // + 1 so we can wait for maxDelay '0xFF' bytes before reading a response byte afterwards
        uint8_t response = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

        if (response != 0xFF)
            return response;
    }

    return 0xFF;
}

/**
 * Waits up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes for the card to become ready, send a command to the card
 * with the given argument, waits up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes for a reply, and returns the
 * first non-0xFF byte of the reply.
 *
 * You must select the card first with SELECT_SDCARD and deselect it afterwards with DESELECT_SDCARD.
 *
 * Upon failure, 0xFF is returned.
 */
static uint8_t sdcard_sendCommand(uint8_t commandCode, uint32_t commandArgument)
{
    uint8_t command[6] = {
        0x40 | commandCode,
        commandArgument >> 24,
        commandArgument >> 16,
        commandArgument >> 8,
        commandArgument,
        0x95 /* Static CRC. This CRC is valid for CMD0 with a 0 argument, and CMD8 with 0x1AB argument, which are the only
        commands that require a CRC */
    };

    // Go ahead and send the command even if the card isn't idle if this is the reset command
    if (!sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY) && commandCode != SDCARD_COMMAND_GO_IDLE_STATE)
        return 0xFF;

    spiTransfer(SDCARD_SPI_INSTANCE, NULL, command, sizeof(command));

    /*
     * The card can take up to SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY bytes to send the response, in the meantime
     * it'll transmit 0xFF filler bytes.
     */
    return sdcard_waitForNonIdleByte(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY);
}

static uint8_t sdcard_sendAppCommand(uint8_t commandCode, uint32_t commandArgument) {
    sdcard_sendCommand(SDCARD_COMMAND_APP_CMD, 0);

    return sdcard_sendCommand(commandCode, commandArgument);
}

/**
 * Sends an IF_COND message to the card to check its version and validate its voltage requirements. Sets the global
 * sdCardVersion with the detected version (0, 1, or 2) and returns true if the card is compatbile.
 */
static bool sdcard_validateInterfaceCondition()
{
    uint8_t ifCondReply[4];

    sdcard.version = 0;

    SELECT_SDCARD;

    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SEND_IF_COND, (SDCARD_VOLTAGE_ACCEPTED_2_7_to_3_6 << 8) | SDCARD_IF_COND_CHECK_PATTERN);

    // Don't deselect the card right away, because we'll want to read the rest of its reply if it's a V2 card

    if (status == (SDCARD_R1_STATUS_BIT_ILLEGAL_COMMAND | SDCARD_R1_STATUS_BIT_IDLE)) {
        // V1 cards don't support this command
        sdcard.version = 1;
    } else if (status == SDCARD_R1_STATUS_BIT_IDLE) {
        spiTransfer(SDCARD_SPI_INSTANCE, ifCondReply, NULL, sizeof(ifCondReply));

        /*
         * We don't bother to validate the SDCard's operating voltage range since the spec requires it to accept our
         * 3.3V, but do check that it echoed back our check pattern properly.
         */
        if (ifCondReply[3] == SDCARD_IF_COND_CHECK_PATTERN) {
            sdcard.version = 2;
        }
    }

    DESELECT_SDCARD;

    return sdcard.version > 0;
}

static bool sdcard_readOCRRegister(uint32_t *result)
{
    SELECT_SDCARD;

    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_OCR, 0);

    uint8_t response[4];

    spiTransfer(SDCARD_SPI_INSTANCE, response, NULL, sizeof(response));

    if (status == 0) {
        DESELECT_SDCARD;

        *result = (response[0] << 24) | (response[1] << 16) | (response[2] << 8) | response[3];

        return true;
    } else {
        DESELECT_SDCARD;

        return false;
    }
}

/**
 * Attempt to receive a data block from the SD card.
 *
 * Return true on success, otherwise the card has not responded yet and you should retry later.
 */
static bool sdcard_receiveDataBlock(uint8_t *buffer, int count)
{
    if (sdcard_waitForNonIdleByte(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY) != SDCARD_SINGLE_BLOCK_READ_START_TOKEN) {
        return false;
    }

    spiTransfer(SDCARD_SPI_INSTANCE, buffer, NULL, count);

    // Discard trailing CRC, we don't care
    spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);
    spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

    return true;
}

/**
 * Write the buffer of `count` bytes to the SD card.
 *
 * Returns true if the card accepted the write (card will enter a busy state).
 */
static bool sdcard_sendDataBlock(uint8_t *buffer, int count)
{
    // Card wants 8 dummy clock cycles after the command response to become ready
    spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

    spiTransferByte(SDCARD_SPI_INSTANCE, SDCARD_SINGLE_BLOCK_WRITE_START_TOKEN);
    spiTransfer(SDCARD_SPI_INSTANCE, NULL, buffer, count);

    // Send a dummy CRC
    spiTransferByte(SDCARD_SPI_INSTANCE, 0x00);
    spiTransferByte(SDCARD_SPI_INSTANCE, 0x00);

    uint8_t dataResponseToken = spiTransferByte(SDCARD_SPI_INSTANCE, 0xFF);

    /*
     * Check if the card accepted the write (no CRC error / no address error)
     *
     * The lower 5 bits are structured as follows:
     * | 0 | Status  | 1 |
     * | 0 | x  x  x | 1 |
     *
     * Statuses:
     * 010 - Data accepted
     * 101 - CRC error
     * 110 - Write error
     */
    return (dataResponseToken & 0x1F) == 0x05;
}

static bool sdcard_fetchCID()
{
    uint8_t cid[16];

    SELECT_SDCARD;

    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_SEND_CID, 0);

    if (status != 0 || !sdcard_receiveDataBlock(cid, sizeof(cid))) {
        DESELECT_SDCARD;

        return false;
    }

    sdcard.metadata.manufacturerID = cid[0];
    sdcard.metadata.oemID = (cid[1] << 8) | cid[2];
    sdcard.metadata.productName[0] = cid[3];
    sdcard.metadata.productName[1] = cid[4];
    sdcard.metadata.productName[2] = cid[5];
    sdcard.metadata.productName[3] = cid[6];
    sdcard.metadata.productName[4] = cid[7];
    sdcard.metadata.productRevisionMajor = cid[8] >> 4;
    sdcard.metadata.productRevisionMinor = cid[8] & 0x0F;
    sdcard.metadata.productSerial = (cid[9] << 24) | (cid[10] << 16) | (cid[11] << 8) | cid[12];
    sdcard.metadata.productionYear = (((cid[13] & 0x0F) << 4) | (cid[14] >> 4)) + 2000;
    sdcard.metadata.productionMonth = cid[14] & 0x0F;

    DESELECT_SDCARD;

    return true;
}

static bool sdcard_fetchCSD()
{
    uint32_t readBlockLen, blockCount, blockCountMult, capacityBytes;

    SELECT_SDCARD;

    bool success =
        sdcard_sendCommand(SDCARD_COMMAND_SEND_CSD, 0) == 0
        && sdcard_receiveDataBlock((uint8_t*) &sdcard.csd, sizeof(sdcard.csd))
        && SDCARD_GET_CSD_FIELD(sdcard.csd, 1, TRAILER) == 1;

    if (success) {
        switch (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSD_STRUCTURE_VER)) {
            case SDCARD_CSD_STRUCTURE_VERSION_1:
                // Block size in bytes (doesn't have to be 512)
                readBlockLen = 1 << SDCARD_GET_CSD_FIELD(sdcard.csd, 1, READ_BLOCK_LEN);
                blockCountMult = 1 << (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSIZE_MULT) + 2);
                blockCount = (SDCARD_GET_CSD_FIELD(sdcard.csd, 1, CSIZE) + 1) * blockCountMult;
                capacityBytes = blockCount * readBlockLen;

                // Re-express that capacity (max 2GB) in our standard 512-byte block size
                sdcard.metadata.numBlocks = capacityBytes / SDCARD_BLOCK_SIZE;
            break;
            case SDCARD_CSD_STRUCTURE_VERSION_2:
                sdcard.metadata.numBlocks = (SDCARD_GET_CSD_FIELD(sdcard.csd, 2, CSIZE) + 1) * 1024;
            break;
            default:
                success = false;
        }
    }

    DESELECT_SDCARD;

    return success;
}

/**
 * Call once SDcard has finished its initialisation phase to read ID data from the card and complete our init.
 *
 * Returns true on success, false on card init failure.
 */
static bool sdcard_completeInit()
{
    if (sdcard.version == 2) {
        // Check for high capacity card
        uint32_t ocr;

        if (!sdcard_readOCRRegister(&ocr)) {
            return false;
        }

        sdcard.highCapacity = (ocr & (1 << 30)) != 0;
    } else {
        // Version 1 cards are always low-capacity
        sdcard.highCapacity = false;
    }

    if (!sdcard_fetchCID() || !sdcard_fetchCSD())
        return false;
    
    /* The spec is a little iffy on what the default block size is for Standard Size cards (it can be changed on
     * standard size cards) so let's just set it to 512 explicitly so we don't have a problem.
     */
    if (!sdcard.highCapacity) {
        SELECT_SDCARD;

        if (sdcard_sendCommand(SDCARD_COMMAND_SET_BLOCKLEN, SDCARD_BLOCK_SIZE) != 0) {
            return false;
        }

        DESELECT_SDCARD;
    }

    spiSetDivisor(SDCARD_SPI_INSTANCE, SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER);

    sdcard.state = SDCARD_STATE_READY;

    return true;
}

/**
 * Check if the SD Card has completed its startup sequence. Must be called with sdcard.state == SDCARD_STATE_INITIALIZATION.
 *
 * Changes sdcard.state to SDCARD_STATE_READY on success and returns true, returns false otherwise.
 */
static bool sdcard_checkInitDone() {
    SELECT_SDCARD;

    uint8_t status = sdcard_sendAppCommand(SDCARD_ACOMMAND_SEND_OP_COND, sdcard.version == 2 ? 1 << 30 /* We support high capacity cards */ : 0);

    DESELECT_SDCARD;

    // When card init is complete, the idle bit in the response becomes zero.
    if (status == 0x00) {
        return sdcard_completeInit();
    }

    return false;
}

bool sdcard_init()
{
    // Max frequency is initially 400kHz
    spiSetDivisor(SDCARD_SPI_INSTANCE, SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER);

    // SDCard wants 1ms minimum delay after power is applied to it
    delay(1000);

    // Transmit at least 74 dummy clock cycles with CS high so the SD card can start up
    SET_CS_HIGH;

    spiTransfer(SDCARD_SPI_INSTANCE, NULL, NULL, SDCARD_INIT_NUM_DUMMY_BYTES);

    // Wait for that transmission to finish before we enable the SDCard, so it receives the required number of cycles
    while (spiIsBusBusy(SDCARD_SPI_INSTANCE)) {
    }

    SELECT_SDCARD;

    uint8_t initStatus = sdcard_sendCommand(SDCARD_COMMAND_GO_IDLE_STATE, 0);

    DESELECT_SDCARD;

    if (initStatus != SDCARD_R1_STATUS_BIT_IDLE)
        return false;

    // Check card voltage and version
    if (!sdcard_validateInterfaceCondition())
        return false;

    uint32_t ocr;

    sdcard_readOCRRegister(&ocr);

    /*
     * Now the SD card will perform its startup, which can take hundreds of milliseconds. We won't wait for this to
     * avoid slowing down system startup. Instead we'll periodically poll with sdcard_checkInitDone() later on.
     */
    sdcard.state = SDCARD_STATE_INITIALIZATION;

    return true;
}

/**
 * Call periodically for the SD card to perform in-progress transfers.
 */
void sdcard_poll()
{
    switch (sdcard.state) {
        case SDCARD_STATE_READING:
            if (sdcard_receiveDataBlock(sdcard.pendingOperation.buffer, SDCARD_BLOCK_SIZE)) {
                DESELECT_SDCARD;

                sdcard.state = SDCARD_STATE_READY;

                if (sdcard.pendingOperation.callback) {
                    sdcard.pendingOperation.callback(
                        SDCARD_BLOCK_OPERATION_READ,
                        sdcard.pendingOperation.blockIndex,
                        sdcard.pendingOperation.buffer,
                        sdcard.pendingOperation.callbackData
                    );
                }
            }
        break;
        case SDCARD_STATE_INITIALIZATION:
            sdcard_checkInitDone();
        break;
        case SDCARD_STATE_WRITING:
            if (sdcard_waitForIdle(SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY)) {
                DESELECT_SDCARD;
                sdcard.state = SDCARD_STATE_READY;
            }

        break;
        default:
            ;
    }
}

/**
 * Write the 512-byte block from the given buffer into the block with the given index.
 *
 * Returns true if the write was successfully sent to the card, or false if the operation could
 * not be started due to the card being busy (try again later), or because the write was invalid (bad address).
 *
 * The buffer is not copied anywhere, you must keep the pointer to the buffer valid until the operation completes!
 */
bool sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer)
{
    if (sdcard.state != SDCARD_STATE_READY)
        return false;

    SELECT_SDCARD;

    // Standard size cards use byte addressing, high capacity cards use block addressing
    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_WRITE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);

    if (status == 0 && sdcard_sendDataBlock(buffer, SDCARD_BLOCK_SIZE)) {
        sdcard.state = SDCARD_STATE_WRITING;

        // Leave the card selected while the write is in progress
        return true;
    } else {
        DESELECT_SDCARD;
        return false;
    }
}

/**
 * Read the 512-byte block with the given index into the given 512-byte buffer.
 *
 * Returns true if the operation was successfully queued for later completion, or false if the operation could
 * not be started due to the card being busy (try again later).
 *
 * You must keep the pointer to the buffer valid until the operation completes!
 */
bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    if (sdcard.state != SDCARD_STATE_READY)
        return false;

    SELECT_SDCARD;

    // Standard size cards use byte addressing, high capacity cards use block addressing
    uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_SINGLE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);

    if (status == 0) {
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = blockIndex;
        sdcard.pendingOperation.callback = callback;
        sdcard.pendingOperation.callbackData = callbackData;
        
        sdcard.state = SDCARD_STATE_READING;
        // Leave the card selected for the whole transaction

        return true;
    } else {
        DESELECT_SDCARD;

        return false;
    }
}

bool sdcard_isReady() {
    return sdcard.state == SDCARD_STATE_READY;
}

#endif
