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

/**
 * This provides a stream interface to a flash chip if one is present.
 *
 * On statup, call flashfsInit() after initialising the flash chip in order to init the filesystem. This will
 * result in the file pointer being pointed at the first free block found, or at the end of the device if the
 * flash chip is full.
 *
 * Note that bits can only be set to 0 when writing, not back to 1 from 0. You must erase sectors in order
 * to bring bits back to 1 again.
 *
 * In future, we can add support for multiple different flash chips by adding a flash device driver vtable
 * and make calls through that, at the moment flashfs just calls m25p16_* routines explicitly.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#if defined(USE_FLASHFS)

#include "build/debug.h"
#include "common/maths.h"
#include "common/printf.h"
#include "drivers/flash/flash.h"
#include "drivers/light_led.h"

#include "io/flashfs.h"

typedef enum {
    FLASHFS_IDLE,
    FLASHFS_ERASING,
} flashfsState_e;

static const flashPartition_t *flashPartition = NULL;
static const flashGeometry_t *flashGeometry = NULL;
static uint32_t flashfsSize = 0;
static flashfsState_e flashfsState = FLASHFS_IDLE;
static flashSector_t eraseSectorCurrent = 0;

static DMA_DATA_ZERO_INIT uint8_t flashWriteBuffer[FLASHFS_WRITE_BUFFER_SIZE];

/* The position of our head and tail in the circular flash write buffer.
 *
 * The head is the index that a byte would be inserted into on writing, while the tail is the index of the
 * oldest byte that has yet to be written to flash.
 *
 * When the circular buffer is empty, head == tail
 *
 * The tail is advanced once a write is complete up to the location behind head. The tail is advanced
 * by a callback from the FLASH write routine. This prevents data being overwritten whilst a write is in progress.
 */
// Type-wide enough to index FLASHFS_WRITE_BUFFER_SIZE; bumped from uint8_t when the
// buffer was 128 B to uint16_t now that USE_BLACKBOX_RING_LOG can grow it to 16 KB.
// Compile-time guard so a future bump past 64 KB fails loudly instead of silently
// wrapping the indices in flashfsAdvanceTailInBuffer / flashfsWriteByte.
static uint16_t bufferHead = 0;
static volatile uint16_t bufferTail = 0;
STATIC_ASSERT(FLASHFS_WRITE_BUFFER_SIZE < 0x10000u, flashfs_write_buffer_too_large_for_uint16_index);

/* Track if there is new data to write. Until the contents of the buffer have been completely
 * written flashfsFlushAsync() will be repeatedly called. The tail pointer is only updated
 * once an asynchronous write has completed. To do so any earlier could result in data being
 * overwritten in the ring buffer. This routine checks that flashfsFlushAsync() should attempt
 * to write new data and avoids it writing old data during the race condition that occurs if
 * its called again before the previous write to FLASH has completed.
  */
static volatile bool dataWritten = true;

// The position of the buffer's tail in the overall flash address space:
static uint32_t tailAddress = 0;

// Optional ring-mode wrap. When wrapEndAddress != UINT32_MAX, sequential writes whose
// tailAddress is inside [wrapStartAddress, wrapEndAddress) will have their post-write
// tailAddress folded back to the start of the ring once it reaches wrapEndAddress.
// Linear mode leaves these at their disabled defaults so the wrap branch in the write
// callback is never taken. Set/cleared by USE_BLACKBOX_RING_LOG via the public
// flashfsSetRing()/flashfsClearRing() API.
//
// The "started inside the ring" gate matters because the same flashfs instance is also
// used to write into the header buffer area (which sits past wrapEndAddress in linear
// flash address space). Writes positioned outside the ring must advance linearly, not
// wrap.
static uint32_t wrapStartAddress = 0;
static uint32_t wrapEndAddress = UINT32_MAX;

// Captured wrap end at the moment of the first wrap fire. Once non-zero, the ring has
// looped back to its start at least once and every byte in [wrapStartAddress, this)
// holds log data. flashfsGetOffset() uses it to keep its "bytes stored" contract
// monotonic across the wrap (callers like MSP_DATAFLASH_SUMMARY would otherwise see
// the value collapse to ~0 once tailAddress folds back). Cleared on full-erase, not
// on flashfsClearRing — leaving a ring-mode session doesn't un-fill the chip.
static uint32_t ringFullSize = 0;

#ifdef USE_FLASH_TEST_PRBS
// Write an incrementing sequence of bytes instead of the requested data and verify
static DMA_DATA uint8_t checkFlashBuffer[FLASHFS_WRITE_BUFFER_SIZE];
static uint32_t checkFlashPtr = 0;
static uint32_t checkFlashLen = 0;
static uint32_t checkFlashErrors = 0;
static bool checkFlashActive = false;
static uint32_t checkFlashSeedPRBS;

static uint8_t checkFlashNextByte(void)
{
    uint32_t newLSB = ((checkFlashSeedPRBS >> 31) ^ (checkFlashSeedPRBS >> 28)) & 1;

    checkFlashSeedPRBS = (checkFlashSeedPRBS << 1) | newLSB;

    return checkFlashSeedPRBS & 0xff;
}

// Called from blackboxSetState() to start/stop writing of pseudo-random data to FLASH
void checkFlashStart(void)
{
    DEBUG_SET(DEBUG_FLASH_TEST_PRBS, 0, 1);
    checkFlashSeedPRBS = 0xdeadbeef;
    checkFlashPtr = tailAddress;
    checkFlashLen = 0;
    checkFlashActive = true;
}

void checkFlashStop(void)
{
    checkFlashSeedPRBS = 0xdeadbeef;
    checkFlashErrors = 0;

    DEBUG_SET(DEBUG_FLASH_TEST_PRBS, 6, checkFlashLen / flashGeometry->pageSize);

    // Verify the data written since flashfsSeekAbs() last called
    while (checkFlashLen) {
        uint32_t checkLen = MIN(checkFlashLen, sizeof(checkFlashBuffer));

        // Don't read over a page boundary
        checkLen = MIN(checkLen, flashGeometry->pageSize - (checkFlashPtr & (flashGeometry->pageSize - 1)));

        flashReadBytes(checkFlashPtr, checkFlashBuffer, checkLen);

        for (uint32_t i = 0; i < checkLen; i++) {
            uint8_t expected = checkFlashNextByte();
            if (checkFlashBuffer[i] != expected) {
                checkFlashErrors++; // <-- insert breakpoint here to catch errors
            }
        }

        checkFlashPtr += checkLen;
        checkFlashLen -= checkLen;
    }

    DEBUG_SET(DEBUG_FLASH_TEST_PRBS, 7, checkFlashErrors);

    checkFlashActive = false;
}
#endif

static void flashfsClearBuffer(void)
{
    bufferTail = bufferHead = 0;
}

static bool flashfsBufferIsEmpty(void)
{
    return bufferTail == bufferHead;
}

// Public predicate for callers that need to know whether a sync flash transaction
// would run quickly (no pending RAM-buffer pages to drain AND the chip is idle).
// Used by flashfs_log's non-blocking lap-marker persist path to avoid stalling
// the FC loop on a busy chip or a full write buffer.
bool flashfsIsIdle(void)
{
    // Reuse flashfsIsReady() rather than the bare flashIsReady() so an in-flight
    // async erase keeps idle == false. flashIsReady() can transiently return true
    // between sector-erase commands inside flashfsEraseAsync's loop, which would
    // otherwise let callers (notably flashfs_log's persistLappedMarkerIfReady)
    // sneak a write in mid-erase.
    return flashfsBufferIsEmpty() && flashfsIsReady();
}

static void flashfsSetTailAddress(uint32_t address)
{
    tailAddress = address;
}

void flashfsEraseCompletely(void)
{
    if (flashGeometry->sectors > 0 && flashPartitionCount() > 0) {
        // if there's a single FLASHFS partition and it uses the entire flash then do a full erase
        const bool doFullErase = (flashPartitionCount() == 1) && (FLASH_PARTITION_SECTOR_COUNT(flashPartition) == flashGeometry->sectors);
        if (doFullErase) {
            flashEraseCompletely();
        } else {
            // start asynchronous erase of all sectors
            eraseSectorCurrent = flashPartition->startSector;
            flashfsState = FLASHFS_ERASING;
        }
    }

    flashfsClearBuffer();

    flashfsSetTailAddress(0);

    // Chip is being wiped — any prior ring-wrap signal no longer reflects on-flash
    // state. Reset so flashfsGetOffset() reports 0 again.
    ringFullSize = 0;
}

/**
 * Start and end must lie on sector boundaries, or they will be rounded out to sector boundaries such that
 * all the bytes in the range [start...end) are erased.
 */
void flashfsEraseRange(uint32_t start, uint32_t end)
{
    if (flashGeometry->sectorSize <= 0)
        return;

    // Round the start down to a sector boundary
    int startSector = start / flashGeometry->sectorSize;

    // And the end upward
    int endSector = end / flashGeometry->sectorSize;
    int endRemainder = end % flashGeometry->sectorSize;

    if (endRemainder > 0) {
        endSector++;
    }

    for (int sectorIndex = startSector; sectorIndex < endSector; sectorIndex++) {
        uint32_t sectorAddress = sectorIndex * flashGeometry->sectorSize;
        flashEraseSector(sectorAddress);
    }
}

/**
 * Return true if the flash is not currently occupied with an operation.
 */
bool flashfsIsReady(void)
{
    // Check for flash chip existence first, then check if idle and ready.

    return (flashfsIsSupported() && (flashfsState == FLASHFS_IDLE) && flashIsReady());
}

bool flashfsIsEraseInProgress(void)
{
    return flashfsState == FLASHFS_ERASING;
}

bool flashfsIsSupported(void)
{
    return flashfsSize > 0;
}

uint32_t flashfsGetSize(void)
{
    return flashfsSize;
}

static uint32_t flashfsTransmitBufferUsed(void)
{
    if (bufferHead >= bufferTail)
        return bufferHead - bufferTail;

    return FLASHFS_WRITE_BUFFER_SIZE - bufferTail + bufferHead;
}

/**
 * Get the size of the largest single write that flashfs could ever accept without blocking or data loss.
 */
uint32_t flashfsGetWriteBufferSize(void)
{
    return FLASHFS_WRITE_BUFFER_USABLE;
}

/**
 * Get the number of bytes that can currently be written to flashfs without any blocking or data loss.
 */
uint32_t flashfsGetWriteBufferFreeSpace(void)
{
    return flashfsGetWriteBufferSize() - flashfsTransmitBufferUsed();
}

/**
 * Called after bytes have been written from the buffer to advance the position of the tail by the given amount.
 */
static void flashfsAdvanceTailInBuffer(uint32_t delta)
{
    bufferTail += delta;

    // Wrap tail around the end of the buffer
    if (bufferTail >= FLASHFS_WRITE_BUFFER_SIZE) {
        bufferTail -= FLASHFS_WRITE_BUFFER_SIZE;
    }
}

/**
 * Write the given buffers to flash sequentially at the current tail address, advancing the tail address after
 * each write.
 *
 * In synchronous mode, waits for the flash to become ready before writing so that every byte requested can be written.
 *
 * In asynchronous mode, if the flash is busy, then the write is aborted and the routine returns immediately.
 * In this case the returned number of bytes written will be less than the total amount requested.
 *
 * Modifies the supplied buffer pointers and sizes to reflect how many bytes remain in each of them.
 *
 * bufferCount: the number of buffers provided
 * buffers: an array of pointers to the beginning of buffers
 * bufferSizes: an array of the sizes of those buffers
 * sync: true if we should wait for the device to be idle before writes, otherwise if the device is busy the
 *       write will be aborted and this routine will return immediately.
 *
 * Returns the number of bytes written
 */
static void flashfsWriteCallback(uintptr_t arg)
{
    uint32_t newTail = tailAddress + arg;

    // Ring-mode wrap. Only applies if the write *started* inside the configured ring;
    // writes positioned past the ring (e.g. into the header buffer area) advance
    // linearly. wrapEndAddress is asserted page-aligned in flashfsSetRing(), and
    // flashPageProgramBegin bounds writes at page boundaries, so a single page write
    // can land at most at wrapEndAddress (never strictly past it). The math here
    // tolerates overshoot defensively.
    if (tailAddress >= wrapStartAddress && tailAddress < wrapEndAddress
            && newTail >= wrapEndAddress) {
        newTail = wrapStartAddress + (newTail - wrapEndAddress);
        // Capture ring capacity (NOT the absolute end address) on the first wrap,
        // so flashfsGetOffset can report a stable "bytes ever stored" once the
        // tailAddress is no longer monotonic. The capacity form (end - start) is
        // correct for any wrapStartAddress; the current ring-log caller happens
        // to pass start=0 so the two values coincide today.
        if (ringFullSize == 0) ringFullSize = wrapEndAddress - wrapStartAddress;
    }

    flashfsSetTailAddress(newTail);

    // Free bytes in the ring buffer
    flashfsAdvanceTailInBuffer(arg);

    // Mark that data has been written from the buffer
    dataWritten = true;
}

static uint32_t flashfsWriteBuffers(uint8_t const **buffers, uint32_t *bufferSizes, int bufferCount, bool sync)
{
    uint32_t bytesWritten;

    // It's OK to overwrite the buffer addresses/lengths being passed in

    // If sync is true, block until the FLASH device is ready, otherwise return 0 if the device isn't ready
    if (sync) {
        while (!flashIsReady());
    } else {
        if (!flashIsReady()) {
            return 0;
        }
    }

    // Are we at EOF already? Abort.
    if (flashfsIsEOF()) {
        return 0;
    }

    flashPageProgramBegin(tailAddress, flashfsWriteCallback);

    /* Mark that data has yet to be written. There is no race condition as the DMA engine is known
     * to be idle at this point
     */
    dataWritten = false;

    bytesWritten = flashPageProgramContinue(buffers, bufferSizes, bufferCount);

    flashPageProgramFinish();

    return bytesWritten;
}

/*
 * Since the buffered data might wrap around the end of the circular buffer, we can have two segments of data to write,
 * an initial portion and a possible wrapped portion.
 *
 * This routine will fill the details of those buffers into the provided arrays, which must be at least 2 elements long.
 */
static int flashfsGetDirtyDataBuffers(uint8_t const *buffers[], uint32_t bufferSizes[])
{
    buffers[0] = flashWriteBuffer + bufferTail;
    buffers[1] = flashWriteBuffer + 0;

    if (bufferHead > bufferTail) {
        bufferSizes[0] = bufferHead - bufferTail;
        bufferSizes[1] = 0;
        return 1;
    } else if (bufferHead < bufferTail) {
        bufferSizes[0] = FLASHFS_WRITE_BUFFER_SIZE - bufferTail;
        bufferSizes[1] = bufferHead;
        if (bufferSizes[1] == 0) {
            return 1;
        } else {
            return 2;
        }
    }

    bufferSizes[0] = 0;
    bufferSizes[1] = 0;

    return 0;
}

static bool flashfsNewData(void)
{
    return dataWritten;
}

/**
 * Get the logical used size of the volume.
 *
 * In linear mode this is the current write tail address plus any bytes still in
 * the RAM buffer — effectively a "bytes ever stored" counter that callers can
 * also treat as the next write position.
 *
 * In ring mode (USE_BLACKBOX_RING_LOG, after the writer has wrapped at least
 * once and `ringFullSize` is non-zero), the physical tail address has folded
 * back to the ring start and is no longer monotonic. To preserve the public
 * "bytes stored" contract for callers like MSP_DATAFLASH_SUMMARY and the OSD
 * storage gauge, this function returns `ringFullSize` (the captured ring
 * capacity at the moment of the first wrap). Callers must NOT use the returned
 * value as a seek offset once that branch is active — it's a usage report,
 * not a file pointer.
 */
uint32_t flashfsGetOffset(void)
{
    // Once the ring has wrapped, every byte in [wrapStartAddress, ringFullSize)
    // holds log data. tailAddress has folded back to the ring start, so falling
    // through to "tailAddress + buffered" would collapse the reported "bytes
    // stored" to ~0 and break callers like MSP_DATAFLASH_SUMMARY / OSD storage
    // gauge. Report the full ring size instead — the chip is, for practical
    // purposes, full of log data.
    if (ringFullSize != 0) {
        return ringFullSize;
    }

    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];

    // Dirty data in the buffers contributes to the offset

    flashfsGetDirtyDataBuffers(buffers, bufferSizes);

    return tailAddress + bufferSizes[0] + bufferSizes[1];
}

/**
 * If the flash is ready to accept writes, flush the buffer to it.
 *
 * Returns true if all data in the buffer has been flushed to the device, or false if
 * there is still data to be written (call flush again later).
 */
bool flashfsFlushAsync(bool force)
{
    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];
    int bufCount;

    if (flashfsBufferIsEmpty()) {
        return true; // Nothing to flush
    }

    if (!flashfsNewData()) {
        // The previous write has yet to complete
        return false;
    }

    bufCount = flashfsGetDirtyDataBuffers(buffers, bufferSizes);
    uint32_t bufferedBytes = bufferSizes[0] + bufferSizes[1];

    if (bufCount && (force || (bufferedBytes >= FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN))) {
        flashfsWriteBuffers(buffers, bufferSizes, bufCount, false);
    }

    return flashfsBufferIsEmpty();
}

/**
 * Wait for the flash to become ready and begin flushing any buffered data to flash.
 *
 * The flash will still be busy some time after this sync completes, but space will
 * be freed up to accept more writes in the write buffer.
 */
void flashfsFlushSync(void)
{
    // Each flashfsWriteBuffers() call programs at most one flash page (the underlying
    // flashPageProgramContinue truncates buffers at the next page boundary), so a
    // single flush only drains ~pageSize bytes — typically 256 B. With the ring-mode
    // build's 16 KB write buffer, that left up to 64 pages still buffered after a
    // "sync" flush, which broke callers that relied on the chip being fully drained
    // (markBufferLapped's mid-session preamble update, flashfsSetRing /
    // flashfsClearRing's wrap-config swap, etc — all could leave residual writes to
    // complete under post-call state). Loop until the buffer is empty.
    while (!flashfsBufferIsEmpty()) {
        // Block until the previous page's DMA-TC callback has fired and advanced
        // bufferTail. flashIsReady() inside flashfsWriteBuffers(sync=true) only
        // gates on the chip's BUSY flag — the write callback (which mutates
        // bufferTail / sets dataWritten) is queued at TC IRQ and may not have run
        // yet. Without this gate, the next iteration captures (buffer, size) via
        // flashfsGetDirtyDataBuffers() using a stale bufferTail and re-programs
        // bytes that were just drained. flashfsFlushAsync already uses this same
        // guard via flashfsNewData().
        while (!flashfsNewData()) { /* spin until prior write callback fired */ }

        uint8_t const * buffers[2];
        uint32_t bufferSizes[2];
        int bufCount = flashfsGetDirtyDataBuffers(buffers, bufferSizes);
        if (!bufCount) break;
        const uint32_t written = flashfsWriteBuffers(buffers, bufferSizes, bufCount, true);
        if (written == 0) break;  // chip refused (e.g. EOF in linear mode); avoid spin
    }

    while (!flashIsReady());
}

/**
 * Configure ring-mode wrap. After this, sequential writes whose tail is inside the
 * ring [startAddress, endAddress) will fold their post-write tail back to startAddress
 * once it reaches endAddress, instead of advancing past it. Writes positioned outside
 * the ring (e.g. into the header buffer area) are unaffected.
 *
 * endAddress must be page-aligned so a single page write can land at it but never
 * cross it; this is checked at runtime.
 */
void flashfsSetRing(uint32_t startAddress, uint32_t endAddress)
{
    // endAddress must be page-aligned, otherwise a page-program could span the wrap
    // boundary and overshoot into the post-ring region (e.g. the header buffer area)
    // before the wrap callback fires.
    const uint32_t pageSize = flashGeometry->pageSize;
    if ((endAddress & (pageSize - 1)) != 0) {
        return; // misconfigured caller; refuse to enable rather than corrupt later
    }
    // Range sanity: refuse to enable if the ring is empty (start >= end) or
    // would extend past the end of the partition. Either case would silently
    // mis-fold the writer's tailAddress and could land writes in the wrong
    // physical region.
    if (startAddress >= endAddress) return;
    if (endAddress > flashfsSize) return;
    // Drain any in-flight page program before swapping wrap addresses, so any
    // already-submitted write completes its callback against the OLD wrap config.
    // If we updated wrapStartAddress / wrapEndAddress while the chip was still
    // busy, the post-write tailAddress fold could mix old and new bounds.
    flashfsFlushSync();
    // Only reset ringFullSize when the ring geometry actually changes. Re-enabling
    // the same [start, end) range across sessions (the common case) must preserve
    // the prior capacity so flashfsGetOffset keeps reporting "chip is full" — once
    // the ring has wrapped at least once, every byte from start to end holds log
    // data, regardless of which session put it there. The reset only fires when a
    // future caller actually resizes the ring without an intervening full erase.
    if (wrapStartAddress != startAddress || wrapEndAddress != endAddress) {
        ringFullSize = 0;
    }
    wrapStartAddress = startAddress;
    wrapEndAddress = endAddress;
}

/**
 * Disable ring-mode wrap. Writes resume linear advancement.
 */
void flashfsClearRing(void)
{
    // Same drain rationale as flashfsSetRing — let any in-flight write finish
    // under the old (ring-enabled) wrap config before exposing the disabled state.
    flashfsFlushSync();
    wrapStartAddress = 0;
    wrapEndAddress = UINT32_MAX;
}

/**
 *  Asynchronously erase the flash: Check if ready and then erase sector.
 */
void flashfsEraseAsync(void)
{
    if (flashfsState == FLASHFS_ERASING) {
        if ((flashfsIsSupported() && flashIsReady())) {
            if (eraseSectorCurrent <= flashPartition->endSector) {
                // Erase sector
                uint32_t sectorAddress = eraseSectorCurrent * flashGeometry->sectorSize;
                flashEraseSector(sectorAddress);
                eraseSectorCurrent++;
                LED1_TOGGLE;
            } else {
                // Done erasing
                flashfsState = FLASHFS_IDLE;
                LED1_OFF;
            }
        }
    }
}

void flashfsSeekAbs(uint32_t offset)
{
    flashfsFlushSync();

    flashfsSetTailAddress(offset);
}

/**
 * Write the given byte asynchronously to the flash. If the buffer overflows, data is silently discarded.
 */
void flashfsWriteByte(uint8_t byte)
{
#ifdef USE_FLASH_TEST_PRBS
    if (debugMode == DEBUG_FLASH_TEST_PRBS) {
        debug[1]++;
    }
    if (checkFlashActive) {
        byte = checkFlashNextByte();
        checkFlashLen++;
        DEBUG_SET(DEBUG_FLASH_TEST_PRBS, 2, checkFlashLen);
    }
#endif

    flashWriteBuffer[bufferHead++] = byte;

    if (bufferHead >= FLASHFS_WRITE_BUFFER_SIZE) {
        bufferHead = 0;
    }

    if (flashfsTransmitBufferUsed() >= FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN) {
        flashfsFlushAsync(false);
    }
}

/**
 * Write the given buffer to the flash either synchronously or asynchronously depending on the 'sync' parameter.
 *
 * If writing asynchronously, data will be silently discarded if the buffer overflows.
 * If writing synchronously, the routine will block waiting for the flash to become ready so will never drop data.
 */
void flashfsWrite(const uint8_t *data, unsigned int len, bool sync)
{
    uint8_t const * buffers[2];
    uint32_t bufferSizes[2];
    int bufCount;
    uint32_t totalBufSize;

    // Buffer up the data the user supplied instead of writing it right away
    for (unsigned int i = 0; i < len; i++) {
        flashfsWriteByte(data[i]);
    }

    // There could be two dirty buffers to write out already:
    bufCount = flashfsGetDirtyDataBuffers(buffers, bufferSizes);
    totalBufSize = bufferSizes[0] + bufferSizes[1];

    /*
     * Would writing this data to our buffer cause our buffer to reach the flush threshold? If so try to write through
     * to the flash now
     */
    if (bufCount && (totalBufSize >= FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN)) {
        flashfsWriteBuffers(buffers, bufferSizes, bufCount, sync);
    }
}

/**
 * Read `len` bytes from the given address into the supplied buffer.
 *
 * Returns the number of bytes actually read which may be less than that requested.
 */
int flashfsReadAbs(uint32_t address, uint8_t *buffer, unsigned int len)
{
    int bytesRead;

    // Did caller try to read past the end of the volume?
    if (address + len > flashfsSize) {
        // Truncate their request
        len = flashfsSize - address;
    }

    // Since the read could overlap data in our dirty buffers, force a sync to clear those first
    flashfsFlushSync();

    bytesRead = flashReadBytes(address, buffer, len);

    return bytesRead;
}

/**
 * Find the offset of the start of the free space on the device (or the size of the device if it is full).
 */
int flashfsIdentifyStartOfFreeSpace(void)
{
    /* Find the start of the free space on the device by examining the beginning of blocks with a binary search,
     * looking for ones that appear to be erased. We can achieve this with good accuracy because an erased block
     * is all bits set to 1, which pretty much never appears in reasonable size substrings of blackbox logs.
     *
     * To do better we might write a volume header instead, which would mark how much free space remains. But keeping
     * a header up to date while logging would incur more writes to the flash, which would consume precious write
     * bandwidth and block more often.
     */

    enum {
        /* We can choose whatever power of 2 size we like, which determines how much wastage of free space we'll have
         * at the end of the last written data. But smaller blocksizes will require more searching.
         */
        FREE_BLOCK_SIZE = 2048, // XXX This can't be smaller than page size for underlying flash device.

        /* We don't expect valid data to ever contain this many consecutive uint32_t's of all 1 bits: */
        FREE_BLOCK_TEST_SIZE_INTS = 4, // i.e. 16 bytes
        FREE_BLOCK_TEST_SIZE_BYTES = FREE_BLOCK_TEST_SIZE_INTS * sizeof(uint32_t)
    };

    STATIC_ASSERT(FREE_BLOCK_SIZE >= FLASH_MAX_PAGE_SIZE, FREE_BLOCK_SIZE_too_small);

    STATIC_DMA_DATA_AUTO union {
        uint8_t bytes[FREE_BLOCK_TEST_SIZE_BYTES];
        uint32_t ints[FREE_BLOCK_TEST_SIZE_INTS];
    } testBuffer;

    int left = 0; // Smallest block index in the search region
    int right = flashfsSize / FREE_BLOCK_SIZE; // One past the largest block index in the search region
    int mid;
    int result = right;
    int i;
    bool blockErased;

    while (left < right) {
        mid = (left + right) / 2;

        if (flashReadBytes(mid * FREE_BLOCK_SIZE, testBuffer.bytes, FREE_BLOCK_TEST_SIZE_BYTES) < FREE_BLOCK_TEST_SIZE_BYTES) {
            // Unexpected timeout from flash, so bail early (reporting the device fuller than it really is)
            break;
        }

        // Checking the buffer 4 bytes at a time like this is probably faster than byte-by-byte, but I didn't benchmark it :)
        blockErased = true;
        for (i = 0; i < FREE_BLOCK_TEST_SIZE_INTS; i++) {
            if (testBuffer.ints[i] != 0xFFFFFFFF) {
                blockErased = false;
                break;
            }
        }

        if (blockErased) {
            /* This erased block might be the leftmost erased block in the volume, but we'll need to continue the
             * search leftwards to find out:
             */
            result = mid;

            right = mid;
        } else {
            left = mid + 1;
        }
    }

    return result * FREE_BLOCK_SIZE;
}

/**
 * Returns true if the file pointer is at the end of the device.
 */
bool flashfsIsEOF(void)
{
    return tailAddress >= flashfsSize;
}

void flashfsClose(void)
{
    switch(flashGeometry->flashType) {
    case FLASH_TYPE_NOR:
        break;

    case FLASH_TYPE_NAND:
        flashFlush();

        // Advance tailAddress to next page boundary.
        uint32_t pageSize = flashGeometry->pageSize;
        flashfsSetTailAddress((tailAddress + pageSize - 1) & ~(pageSize - 1));

        break;
    }

    // Each log is expected to start on a 2K boundary
    flashfsSetTailAddress((tailAddress + 2047) & ~(2047));
}

/**
 * Call after initializing the flash chip in order to set up the filesystem.
 */
void flashfsInit(void)
{
    flashfsSize = 0;

    flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);
    flashGeometry = flashGetGeometry();

    if (!flashPartition) {
        return;
    }

    flashfsSize = FLASH_PARTITION_SECTOR_COUNT(flashPartition) * flashGeometry->sectorSize;

    // Start the file pointer off at the beginning of free space so caller can start writing immediately
    flashfsSeekAbs(flashfsIdentifyStartOfFreeSpace());
}

#ifdef USE_FLASH_TOOLS
bool flashfsVerifyEntireFlash(void)
{
    flashfsEraseCompletely();

    uint32_t address = 0;
    flashfsSeekAbs(address);

    const int bufferSize = 32;
    char buffer[bufferSize + 1];

    const uint32_t testLimit = flashfsGetSize();

    for (address = 0; address < testLimit; address += bufferSize) {
        tfp_sprintf(buffer, "%08x >> **0123456789ABCDEF**", address);
        flashfsWrite((uint8_t*)buffer, strlen(buffer), true);
        if ((address % 0x10000) == 0) {
            LED0_TOGGLE;
        }
        // Don't overwrite the buffer if the FLASH is busy writing
        flashfsFlushSync();
    }
    flashfsClose();

    char expectedBuffer[bufferSize + 1];

    flashfsSeekAbs(0);

    int verificationFailures = 0;
    for (address = 0; address < testLimit; address += bufferSize) {
        tfp_sprintf(expectedBuffer, "%08x >> **0123456789ABCDEF**", address);

        memset(buffer, 0, sizeof(buffer));
        int bytesRead = flashfsReadAbs(address, (uint8_t *)buffer, bufferSize);

        int result = strncmp(buffer, expectedBuffer, bufferSize);
        if (result != 0 || bytesRead != bufferSize) {
            verificationFailures++;
        }
        if ((address % 0x10000) == 0) {
            LED0_TOGGLE;
        }
    }
    return verificationFailures == 0;
}
#endif // USE_FLASH_TOOLS
#endif // USE_FLASHFS
