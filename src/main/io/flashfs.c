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

/**
 * This provides a stream interface to a flash chip if one is present.
 *
 * Note that bits can only be set to 0 when writing, not back to 1 from 0. You must erase sectors in order
 * to bring bits back to 1 again.
 */

#include <stdint.h>
#include <stdbool.h>

#include "drivers/flash_m25p16.h"
#include "flashfs.h"

static uint8_t flashWriteBuffer[FLASHFS_WRITE_BUFFER_SIZE];

/* The position of our head and tail in the circular flash write buffer.
 *
 * The head is the index that a byte would be inserted into on writing, while the tail is the index of the
 * oldest byte that has yet to be written to flash.
 *
 * When the circular buffer is empty, head == tail
 */
static uint8_t bufferHead = 0, bufferTail = 0;

// The position of our tail in the overall flash address space:
static uint32_t tailAddress = 0;
// The index of the tail within the flash page it is inside
static uint16_t tailIndexInPage = 0;

static bool shouldFlush = false;

static void flashfsClearBuffer()
{
    bufferTail = bufferHead = 0;
    shouldFlush = false;
}

static void flashfsSetTailAddress(uint32_t address)
{
    tailAddress = address;
    tailIndexInPage = tailAddress % m25p16_getGeometry()->pageSize;
}

void flashfsEraseCompletely()
{
    m25p16_eraseCompletely();

    flashfsClearBuffer();

    flashfsSetTailAddress(0);
}

/**
 * Start and end must lie on sector boundaries, or they will be rounded out to sector boundaries such that
 * all the bytes in the range [start...end) are erased.
 */
void flashfsEraseRange(uint32_t start, uint32_t end)
{
    const flashGeometry_t *geometry = m25p16_getGeometry();

    if (geometry->sectorSize <= 0)
        return;

    // Round the start down to a sector boundary
    int startSector = start / geometry->sectorSize;

    // And the end upward
    int endSector = end / geometry->sectorSize;
    int endRemainder = end % geometry->sectorSize;

    if (endRemainder > 0) {
        endSector++;
    }

    for (int i = startSector; i < endSector; i++) {
        m25p16_eraseSector(i * geometry->sectorSize);
    }
}

uint32_t flashfsGetSize()
{
    return m25p16_getGeometry()->totalSize;
}

const flashGeometry_t* flashfsGetGeometry()
{
    return m25p16_getGeometry();
}

static uint32_t flashfsTransmitBufferUsed()
{
    if (bufferHead >= bufferTail)
        return bufferHead - bufferTail;

    return FLASHFS_WRITE_BUFFER_SIZE - bufferTail + bufferHead;
}

static uint32_t flashfsTransmitBufferRemaining()
{
    return FLASHFS_WRITE_BUFFER_USABLE - flashfsTransmitBufferUsed();
}

/**
 * Waits for the flash device to be ready to accept writes, then write the given buffers to flash sequentially.
 *
 * Advances the address of the beginning of the supplied buffers and reduces the size of the buffers according to how
 * many bytes get written.
 *
 * bufferCount: the number of buffers provided
 * buffers: an array of pointers to the beginning of buffers
 * bufferSizes: an array of the sizes of those buffers
 */
static void flashfsWriteBuffers(uint8_t const **buffers, uint32_t *bufferSizes, int bufferCount)
{
    const flashGeometry_t *geometry = m25p16_getGeometry();

    uint32_t bytesTotalRemaining = 0;

    int i;

    for (i = 0; i < bufferCount; i++) {
        bytesTotalRemaining += bufferSizes[i];
    }

    while (bytesTotalRemaining > 0) {
        uint32_t bytesTotalThisIteration;
        uint32_t bytesRemainThisIteration;

        /*
         * Each page needs to be saved in a separate program operation, so
         * if we would cross a page boundary, only write up to the boundary in this iteration:
         */
        if (tailIndexInPage + bytesTotalRemaining > geometry->pageSize) {
            bytesTotalThisIteration = geometry->pageSize - tailIndexInPage;
        } else {
            bytesTotalThisIteration = bytesTotalRemaining;
        }

        m25p16_pageProgramBegin(tailAddress);

        bytesRemainThisIteration = bytesTotalThisIteration;

        for (i = 0; i < bufferCount; i++) {
            if (bufferSizes[i] > 0) {
                // Is buffer larger than our write limit? Write our limit out of it
                if (bufferSizes[i] >= bytesRemainThisIteration) {
                    m25p16_pageProgramContinue(buffers[i], bytesRemainThisIteration);

                    buffers[i] += bytesRemainThisIteration;
                    bufferSizes[i] -= bytesRemainThisIteration;

                    bytesRemainThisIteration = 0;
                    break;
                } else {
                    // We'll still have more to write after finishing this buffer off
                    m25p16_pageProgramContinue(buffers[i], bufferSizes[i]);

                    bytesRemainThisIteration -= bufferSizes[i];

                    buffers[i] += bufferSizes[i];
                    bufferSizes[i] = 0;
                }
            }
        }

        m25p16_pageProgramFinish();

        bytesTotalRemaining -= bytesTotalThisIteration;

        // Advance the cursor in the file system to match the bytes we wrote
        flashfsSetTailAddress(tailAddress + bytesTotalThisIteration);
    }
}

/*
 * Since the buffered data might wrap around the end of the circular buffer, we can have two segments of data to write,
 * an initial portion and a possible wrapped portion.
 *
 * This routine will fill the details of those buffers into the provided arrays, which must be at least 2 elements long.
 */
static void flashfsGetDirtyDataBuffers(uint8_t const *buffers[], uint32_t bufferSizes[])
{
    buffers[0] = flashWriteBuffer + bufferTail;
    buffers[1] = flashWriteBuffer + 0;

    if (bufferHead >= bufferTail) {
        bufferSizes[0] = bufferHead - bufferTail;
        bufferSizes[1] = 0;
    } else {
        bufferSizes[0] = FLASHFS_WRITE_BUFFER_SIZE - bufferTail;
        bufferSizes[1] = bufferHead;
    }
}

/**
 * If the flash is ready to accept writes, flush the buffer to it, otherwise schedule
 * a flush for later and return immediately.
 */
void flashfsFlushAsync()
{
    if (bufferHead == bufferTail) {
        shouldFlush = false;
        return; // Nothing to flush
    }

    if (m25p16_isReady()) {
        uint8_t const * buffers[2];
        uint32_t bufferSizes[2];

        flashfsGetDirtyDataBuffers(buffers, bufferSizes);
        flashfsWriteBuffers(buffers, bufferSizes, 2);

        flashfsClearBuffer();

        shouldFlush = false;
    } else {
        shouldFlush = true;
    }
}

/**
 * Wait for the flash to become ready and begin flushing any buffered data to flash.
 *
 * The flash will still be busy some time after this sync completes, but space will
 * be freed up to accept more writes in the write buffer.
 */
void flashfsFlushSync()
{
    if (bufferHead == bufferTail) {
        shouldFlush = false;
        return; // Nothing to write
    }

    m25p16_waitForReady(10); //TODO caller should customize timeout

    flashfsFlushAsync();
}

void flashfsSeekAbs(uint32_t offset)
{
    flashfsFlushSync();

    flashfsSetTailAddress(offset);
}

void flashfsSeekRel(int32_t offset)
{
    flashfsFlushSync();

    flashfsSetTailAddress(tailAddress + offset);
}

void flashfsWriteByte(uint8_t byte)
{
    flashWriteBuffer[bufferHead++] = byte;

    if (bufferHead >= FLASHFS_WRITE_BUFFER_SIZE) {
        bufferHead = 0;
    }

    if (shouldFlush || flashfsTransmitBufferUsed() >= FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN) {
        flashfsFlushAsync();
    }
}

void flashfsWrite(const uint8_t *data, unsigned int len)
{
    // Would writing this cause our buffer to reach the flush threshold? If so just write it now
    if (shouldFlush || len + flashfsTransmitBufferUsed() >= FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN) {
        uint8_t const * buffers[3];
        uint32_t bufferSizes[3];

        // There could be two dirty buffers to write out already:
        flashfsGetDirtyDataBuffers(buffers, bufferSizes);

        // Plus the buffer the user supplied:
        buffers[2] = data;
        bufferSizes[2] = len;

        // Write all three buffers through to the flash
        flashfsWriteBuffers(buffers, bufferSizes, 3);

        // And now our buffer is empty
        flashfsClearBuffer();

        return;
    }

    // Buffer up the data the user supplied instead of writing it right away

    // First write the portion before we wrap around the end of the circular buffer
    unsigned int bufferBytesBeforeLoop = FLASHFS_WRITE_BUFFER_SIZE - bufferHead;

    unsigned int firstPortion = len < bufferBytesBeforeLoop ? len : bufferBytesBeforeLoop;
    unsigned int remainder = firstPortion < len ? len - firstPortion : 0;

    for (unsigned int i = 0; i < firstPortion; i++) {
        flashWriteBuffer[bufferHead++] = *data;
        data++;
    }

    if (bufferHead == FLASHFS_WRITE_BUFFER_SIZE) {
        bufferHead = 0;
    }

    // Then the remainder (if any)
    for (unsigned int i = 0; i < remainder; i++) {
        flashWriteBuffer[bufferHead++] = *data;
        data++;
    }
}

/**
 * Read `len` bytes from the current cursor location into the supplied buffer.
 *
 * Returns the number of bytes actually read which may be less than that requested.
 */
int flashfsRead(uint8_t *data, unsigned int len)
{
    int result = m25p16_readBytes(tailAddress, data, len);

    flashfsSetTailAddress(tailAddress + result);

    return result;
}
