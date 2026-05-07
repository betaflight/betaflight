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

/*
 * See flashfs_log.h for layout / lifecycle docs.
 *
 * Implementation notes for review:
 *
 * Hot path
 * --------
 * The data byte writer (flashfsLogWriteDataByte) does:
 *   if (writeHead == dataSectionEnd) wrap to 0
 *   flashfsWriteByte(b)
 *   writeHead++
 * No synchronous metadata writes during a flight. INVALIDATE is implicit (a log
 * disappears the moment its trailer is overwritten by new data). HWM is implicit
 * (the active log's start is in the header buffer; recovery scans forward from there).
 *
 * Power-loss safety
 * -----------------
 * - Buffer empty/erased  → no active log; nothing to recover.
 * - Buffer valid + uncommitted (no `commit` flag set) → power was cut mid-flight.
 *   On boot: scan forward from data_start for the trailer magic. If absent (the common
 *   case), copy the buffered header to the end-of-data position; mark committed; erase.
 *   If present (rare: power loss between trailer write and commit-flag write), just
 *   mark committed and erase.
 * - Buffer valid + committed → log was committed but the buffer erase was interrupted.
 *   Just erase the buffer.
 *
 * Mode safety
 * -----------
 * Switching between linear and ring modes requires an erase. flashfsLogDetectFormat()
 * exposes the detected format; the writer / MSC paths refuse to operate when there's
 * a mismatch between the configured mode and the on-flash content.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#ifdef USE_BLACKBOX_RING_LOG

#include "build/debug.h"
#include "common/crc.h"
#include "common/maths.h"
#include "drivers/flash/flash.h"

#include "io/flashfs.h"
#include "io/flashfs_log.h"

// =============================================================================
// Layout constants
// =============================================================================

// Active header buffer size in sectors. Must be large enough to hold the largest
// possible blackbox header (sysinfo block, field defs, GPS headers, etc.). Headers
// observed in the wild range 6–10 KB; 16 KB gives generous margin.
#define HDR_BUFFER_SECTORS 4

// Erase-ahead pool size in sectors. The writer always operates at least this many
// sectors behind the leading edge of erased space, so per-byte writes never block on
// flash erase. At log start we sync-erase POOL_TARGET_SECTORS sectors ahead; during
// flight we kick off async erases (chip background) to keep the pool full.
//
// Sustained writer rate vs erase rate determines whether the pool depletes:
//   - 1/8 sample rate (~100 KB/s)  ≤ NOR erase rate (~133 KB/s) → pool stable
//   - 1/4 sample rate (~200 KB/s)  > NOR erase rate              → pool slowly drains;
//     drop guard takes over and the log shows occasional gaps. (Validated config
//     refuses sample rates faster than 1/4 in ring mode — see blackboxValidateConfig.)
#define POOL_TARGET_SECTORS 4

// Magic numbers identifying our on-flash records.
#define BUFFER_PREAMBLE_MAGIC 0x42424C42u  // "BBLB"  (Betaflight Blackbox Log Buffer)
#define LOG_TRAILER_MAGIC     0x4C424c54u  // "TLBL"  (Trailer Log BLackbox)

// Linear-mode probe text (start of a legacy blackbox log header).
static const char LINEAR_FORMAT_PROBE[] = "H Product";
#define LINEAR_FORMAT_PROBE_LEN (sizeof(LINEAR_FORMAT_PROBE) - 1)

// =============================================================================
// On-flash record formats (16 bytes each)
// =============================================================================

// Buffer preamble. Lives at offset 0 of the active header buffer area while a log
// is in progress (or recovering from power loss). Erased after clean log close.
typedef struct {
    uint32_t magic;            // BUFFER_PREAMBLE_MAGIC
    uint16_t crc;              // CRC-16 over (logId, dataStart, headerLength, headerText)
    uint16_t reserved;
    uint32_t logId;
    uint32_t dataStart;        // physical offset in data ring where this log started
    // Followed by 4 more bytes:
    uint32_t headerLengthAndCommit;
                               // bits  0..23: header length (24-bit, plenty for ≤ 16 KB)
                               // bits 24..31: commit flag — 0xFF = uncommitted, 0x00 = committed
} __attribute__((packed)) flashfsBufferPreamble_t;

STATIC_ASSERT(sizeof(flashfsBufferPreamble_t) == 20, buffer_preamble_size);

// Sector-aligned trailer record placed in the data ring just before each log's
// header text. The MSC enumeration scan reads the first 16 bytes of each sector
// looking for this magic.
typedef struct {
    uint32_t magic;            // LOG_TRAILER_MAGIC
    uint16_t crc;              // CRC-16 over (logId, dataStart, headerLength)
    uint16_t reserved;
    uint32_t logId;
    uint32_t dataStart;
    uint32_t headerLength;     // bytes of header text immediately following this trailer
} __attribute__((packed)) flashfsLogTrailer_t;

STATIC_ASSERT(sizeof(flashfsLogTrailer_t) == 20, log_trailer_size);

// Both records are 20 bytes — they don't fit in a single 16-byte page write.
// We size them at 20 bytes for clarity (one u32 + crc/reserved + payload). Flash
// page program supports arbitrary lengths up to a page (256+ bytes typically).

// Chunk size used by the chunked I/O helpers (CRC and bulk copy). Sized to one typical
// flash page; large enough to amortize per-call overhead without using much RAM.
#define CHUNK_SIZE 256
static uint8_t chunkBuf[CHUNK_SIZE];

// =============================================================================
// In-RAM state
// =============================================================================

static struct {
    uint32_t partitionSize;
    uint32_t sectorSize;
    uint32_t pageSize;
    uint32_t dataSectionEnd;       // first byte past the data ring (== buffer area start)
    uint32_t bufferAreaStart;
    uint32_t bufferAreaSize;       // HDR_BUFFER_SECTORS * sectorSize
} geom;

// Module state.
static bool moduleInitialized = false;
static bool moduleSafeForLogging = false;
static flashfsFlashFormat_e detectedFormat = FLASHFS_FLASH_FORMAT_UNKNOWN;

// In-RAM log table (built by scanning at init/erase, refreshed when a log is closed).
static flashfsLogInfo_t logTable[FLASHFS_LOG_MAX_LOGS];
static uint32_t logTableCount = 0;

// Next-log id assignment.
static uint32_t nextLogId = 1;

// Where the next data byte will be written. Tracked in RAM during a flight; recovered
// from the data ring at boot (= sector-aligned position after the latest committed log).
static uint32_t dataWriteHead = 0;

// First non-erased byte ahead of the writer (always sector-aligned). Bytes in the data
// ring from dataWriteHead up to (but not including) eraseHead — accounting for ring wrap
// — are guaranteed to be in erased state and safe to program.
static uint32_t eraseHead = 0;

// Address of a sector currently being erased (chip is busy until flashIsReady() returns
// true). 0xFFFFFFFF means no erase in flight. Set when we kick off an erase; cleared and
// `eraseHead` advanced when we detect completion. This lets us run erases asynchronously
// in the chip's background while the CPU continues servicing the hot path.
#define PENDING_ERASE_NONE 0xFFFFFFFFu
static uint32_t pendingEraseAddr = PENDING_ERASE_NONE;

// Counter (diagnostic only) of bytes dropped because the writer outran the erase pool
// or the RAM buffer filled. Surfaces in DEBUG_BLACKBOX_OUTPUT-style logs if needed.
static uint32_t bbDataDrops = 0;

// Active log state. Valid only between flashfsLogBeginLog() and flashfsLogEndLog().
static struct {
    bool      isOpen;
    bool      headerPhase;
    uint32_t  logId;
    uint32_t  dataStart;
    uint32_t  dataWriteHead;       // mirrors module-level dataWriteHead during the flight
    uint32_t  bufferHeaderOffset;  // bytes written into header buffer so far
} active;

// =============================================================================
// CRC helpers
// =============================================================================

static uint16_t crcRecord(const void *p, size_t skipPrefix, size_t total)
{
    return crc16_ccitt_update(0, (const uint8_t *)p + skipPrefix, total - skipPrefix);
}

// =============================================================================
// Geometry
// =============================================================================

static void computeGeometry(void)
{
    memset(&geom, 0, sizeof(geom));
    geom.partitionSize = flashfsGetSize();
    if (geom.partitionSize == 0) return;
    const flashGeometry_t *fg = flashGetGeometry();
    if (!fg || fg->sectorSize == 0) return;
    geom.sectorSize = fg->sectorSize;
    geom.pageSize = fg->pageSize;
    geom.bufferAreaSize = HDR_BUFFER_SECTORS * geom.sectorSize;
    if (geom.bufferAreaSize >= geom.partitionSize) return;
    geom.bufferAreaStart = geom.partitionSize - geom.bufferAreaSize;
    geom.dataSectionEnd = geom.bufferAreaStart;
}

static uint32_t alignUpToSector(uint32_t addr)
{
    return (addr + geom.sectorSize - 1) / geom.sectorSize * geom.sectorSize;
}

// =============================================================================
// Low-level flash access (bypasses flashfs's circular write buffer)
// =============================================================================
//
// Used for the trailer record and the buffer preamble. The data writer still goes
// through flashfsWriteByte() to benefit from flashfs's existing buffering.

static void writeBytesSync(uint32_t addr, const uint8_t *data, uint32_t length)
{
    flashfsFlushSync();
    flashfsSeekAbs(addr);
    flashfsWrite(data, length, true);
    flashfsFlushSync();
}

static int readBytes(uint32_t addr, uint8_t *dest, uint32_t length)
{
    return flashfsReadAbs(addr, dest, length);
}

static void eraseSectorSync(uint32_t addr)
{
    flashfsFlushSync();
    flashEraseSector(addr);
    while (!flashIsReady());
}

static void eraseBufferArea(void)
{
    for (uint32_t i = 0; i < HDR_BUFFER_SECTORS; i++) {
        eraseSectorSync(geom.bufferAreaStart + i * geom.sectorSize);
    }
}

// =============================================================================
// Erase pool management
// =============================================================================

// Bytes between `writer` and `eraseHead` going forward in the ring. Returns the number
// of erased bytes the writer can still write into without blocking. 0 means the writer
// has caught up to the erase frontier (must drop or stall).
static uint32_t ringSpaceFreeAhead(uint32_t writer)
{
    if (eraseHead >= writer) {
        return eraseHead - writer;
    } else {
        return (geom.dataSectionEnd - writer) + eraseHead;
    }
}

// Run the async-erase progress check + new-erase trigger. Cheap: a register read for
// flashIsReady() and at most one flashEraseSector() call (which itself just sends the
// command and returns; the chip then erases for ~30-50 ms in its own background while
// we keep going).
//
// Called from the data write hot path. NEVER blocks.
static void eraseTick(void)
{
    if (pendingEraseAddr != PENDING_ERASE_NONE) {
        if (flashIsReady()) {
            // Previous erase completed.
            eraseHead = pendingEraseAddr + geom.sectorSize;
            if (eraseHead >= geom.dataSectionEnd) eraseHead = 0;
            pendingEraseAddr = PENDING_ERASE_NONE;
        } else {
            return; // chip still busy; can't start a new erase
        }
    }

    // Pool refill: trigger an erase if we're below target. flashEraseSector is non-
    // blocking from the CPU's point of view (sends the command, returns; chip is busy
    // ~30-50 ms while we continue to write into the existing pool / RAM buffer).
    // Use the active writer's position when a log is open; otherwise the module-level
    // dataWriteHead.
    uint32_t writer = active.isOpen ? active.dataWriteHead : dataWriteHead;
    uint32_t spaceAhead = ringSpaceFreeAhead(writer);
    if (spaceAhead < POOL_TARGET_SECTORS * geom.sectorSize) {
        flashEraseSector(eraseHead);
        pendingEraseAddr = eraseHead;
    }
}

// Synchronously erase one sector, advancing eraseHead. Used at log start, log close,
// and during recovery — places where blocking is acceptable.
static void eraseOneSectorSync(void)
{
    if (pendingEraseAddr != PENDING_ERASE_NONE) {
        // Wait for any background erase to finish first.
        while (!flashIsReady());
        eraseHead = pendingEraseAddr + geom.sectorSize;
        if (eraseHead >= geom.dataSectionEnd) eraseHead = 0;
        pendingEraseAddr = PENDING_ERASE_NONE;
    }
    eraseSectorSync(eraseHead);
    uint32_t newEraseHead = eraseHead + geom.sectorSize;
    if (newEraseHead >= geom.dataSectionEnd) newEraseHead = 0;
    eraseHead = newEraseHead;
}

// Ensure that at least `bytesNeeded` bytes of contiguous erased space exist starting at
// `addr`, sync-erasing additional sectors via `eraseOneSectorSync()` as required. This
// is called only from non-realtime paths (log close, recovery). Blocking is acceptable.
static void ensureErasedSpace(uint32_t addr, uint32_t bytesNeeded)
{
    // We measure how much erased space we have starting at `addr` going forward in the ring,
    // bounded by `eraseHead`. If insufficient, sync-erase one more sector at a time.
    // (eraseHead is sector-aligned and either greater than addr or wrapped before it.)
    while (true) {
        uint32_t haveErased;
        if (eraseHead >= addr) {
            haveErased = eraseHead - addr;
        } else {
            haveErased = (geom.dataSectionEnd - addr) + eraseHead;
        }
        if (haveErased >= bytesNeeded) return;
        eraseOneSectorSync();
    }
}

// =============================================================================
// Trailer record validation
// =============================================================================

static bool readTrailerAt(uint32_t addr, flashfsLogTrailer_t *out)
{
    if (readBytes(addr, (uint8_t *)out, sizeof(*out)) != (int)sizeof(*out)) {
        return false;
    }
    if (out->magic != LOG_TRAILER_MAGIC) return false;
    uint16_t expected = crcRecord(out, 8, sizeof(*out)); // skip magic + crc + reserved
    return out->crc == expected;
}

static void writeTrailer(uint32_t addr, uint32_t logId, uint32_t dataStart, uint32_t headerLength)
{
    flashfsLogTrailer_t t = {
        .magic = LOG_TRAILER_MAGIC,
        .reserved = 0,
        .logId = logId,
        .dataStart = dataStart,
        .headerLength = headerLength,
    };
    t.crc = crcRecord(&t, 8, sizeof(t));
    writeBytesSync(addr, (const uint8_t *)&t, sizeof(t));
}

// =============================================================================
// Buffer preamble
// =============================================================================

static bool bufferIsCommitted(const flashfsBufferPreamble_t *p)
{
    // Commit flag lives in the high byte of headerLengthAndCommit. 0xFF = uncommitted
    // (initial state after write), 0x00 = committed (bit-flipped post-write).
    return (p->headerLengthAndCommit & 0xFF000000u) == 0;
}

// Write the preamble at buffer offset 0, with commit flag = uncommitted (0xFF).
// Called at the end of the header phase. The header bytes are already on flash
// (they were streamed there during the header phase) — we read them back in chunks
// to compute the CRC, avoiding a large RAM scratch buffer.
static void writeBufferPreambleStreaming(uint32_t logId, uint32_t dataStart, uint32_t headerLength)
{
    flashfsBufferPreamble_t p = {
        .magic = BUFFER_PREAMBLE_MAGIC,
        .reserved = 0,
        .logId = logId,
        .dataStart = dataStart,
        .headerLengthAndCommit = (headerLength & 0x00FFFFFFu) | 0xFF000000u,
    };
    uint16_t crc = 0;
    crc = crc16_ccitt_update(crc, &p.logId, sizeof(p.logId));
    crc = crc16_ccitt_update(crc, &p.dataStart, sizeof(p.dataStart));
    crc = crc16_ccitt_update(crc, &headerLength, sizeof(headerLength));
    // Stream the header bytes back from the buffer area to extend the CRC.
    {
        uint32_t addr = geom.bufferAreaStart + sizeof(p);
        uint32_t remaining = headerLength;
        while (remaining > 0) {
            uint32_t chunk = MIN(remaining, (uint32_t)CHUNK_SIZE);
            readBytes(addr, chunkBuf, chunk);
            crc = crc16_ccitt_update(crc, chunkBuf, chunk);
            addr += chunk;
            remaining -= chunk;
        }
    }
    p.crc = crc;
    writeBytesSync(geom.bufferAreaStart, (const uint8_t *)&p, sizeof(p));
}

// Bit-flip the commit flag from 0xFF to 0x00 in the high byte of headerLengthAndCommit.
// We rewrite the entire preamble with the flag set; only that one byte changes value
// (0xFF→0x00) which is a permitted 1→0 NOR flash transition.
static void markBufferCommitted(void)
{
    flashfsBufferPreamble_t p;
    if (readBytes(geom.bufferAreaStart, (uint8_t *)&p, sizeof(p)) != (int)sizeof(p)) return;
    p.headerLengthAndCommit &= 0x00FFFFFFu; // clear the high byte to 0x00
    writeBytesSync(geom.bufferAreaStart, (const uint8_t *)&p, sizeof(p));
}

// =============================================================================
// Data ring scan (MSC enumeration)
// =============================================================================
//
// Scan every sector in the data section. Read the first sizeof(trailer) bytes of each
// sector and check for the trailer magic. Each match describes one log.

static void buildLogTableByScan(void)
{
    logTableCount = 0;
    nextLogId = 1;

    for (uint32_t addr = 0; addr + geom.sectorSize <= geom.dataSectionEnd; addr += geom.sectorSize) {
        flashfsLogTrailer_t t;
        if (!readTrailerAt(addr, &t)) continue;

        if (logTableCount >= FLASHFS_LOG_MAX_LOGS) break;
        flashfsLogInfo_t *info = &logTable[logTableCount++];
        info->logId = t.logId;
        info->headerOffset = addr + sizeof(t); // header text follows trailer
        info->headerLength = t.headerLength;
        info->dataStart = t.dataStart;
        info->dataEnd = addr;                  // data ends where trailer begins
        info->valid = true;
        info->wasOpen = false;
        // total size accounting for ring wrap (dataStart > dataEnd ⇒ wrapped)
        uint32_t dataLen;
        if (info->dataEnd >= info->dataStart) {
            dataLen = info->dataEnd - info->dataStart;
        } else {
            dataLen = (geom.dataSectionEnd - info->dataStart) + info->dataEnd;
        }
        info->totalSize = info->headerLength + dataLen;

        if (t.logId >= nextLogId) nextLogId = t.logId + 1;
    }

    // Sort by logId (ascending) so MSC presents logs in chronological order. Physical
    // sector order is meaningless after the ring has wrapped; without this sort, log
    // numbering and the combined _ALL.BBL file would come back scrambled after a reboot.
    // logTableCount is bounded by FLASHFS_LOG_MAX_LOGS=64 so a simple insertion sort is
    // fine and keeps code minimal.
    for (uint32_t i = 1; i < logTableCount; i++) {
        flashfsLogInfo_t key = logTable[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && logTable[j].logId > key.logId) {
            logTable[j + 1] = logTable[j];
            j--;
        }
        logTable[j + 1] = key;
    }
}

// Determine the next data write head from the latest log's trailer (highest logId).
// On a fresh chip with no logs, returns 0.
static uint32_t computeWriteHeadFromTable(void)
{
    uint32_t latestEnd = 0;
    uint32_t latestLogId = 0;
    bool found = false;
    for (uint32_t i = 0; i < logTableCount; i++) {
        const flashfsLogInfo_t *info = &logTable[i];
        if (!info->valid) continue;
        if (!found || info->logId > latestLogId) {
            latestLogId = info->logId;
            // After log close, write head sits at sector-aligned position past header.
            uint32_t after = info->headerOffset + info->headerLength;
            latestEnd = alignUpToSector(after);
            if (latestEnd >= geom.dataSectionEnd) latestEnd = 0;
            found = true;
        }
    }
    return found ? latestEnd : 0;
}

// =============================================================================
// Format detection
// =============================================================================

flashfsFlashFormat_e flashfsLogDetectFormat(void)
{
    return detectedFormat;
}

static flashfsFlashFormat_e probeFormat(void)
{
    uint8_t probe[16];
    if (readBytes(0, probe, sizeof(probe)) != (int)sizeof(probe)) {
        return FLASHFS_FLASH_FORMAT_UNKNOWN;
    }

    // Linear: byte 0 starts with "H Product" (legacy format always begins with the
    // first log's header text at offset 0).
    if (memcmp(probe, LINEAR_FORMAT_PROBE, LINEAR_FORMAT_PROBE_LEN) == 0) {
        return FLASHFS_FLASH_FORMAT_LINEAR;
    }

    // Empty: first 16 bytes of data section all 0xFF AND buffer area all 0xFF (sampled).
    // The bufEmpty default is `false` so that a flash read failure is conservatively
    // treated as "not empty" rather than triggering an erase-the-chip prompt.
    bool dataEmpty = true;
    for (size_t i = 0; i < sizeof(probe); i++) {
        if (probe[i] != 0xFF) { dataEmpty = false; break; }
    }
    uint8_t bufProbe[16];
    bool bufEmpty = false;
    if (readBytes(geom.bufferAreaStart, bufProbe, sizeof(bufProbe)) == (int)sizeof(bufProbe)) {
        bufEmpty = true;
        for (size_t i = 0; i < sizeof(bufProbe); i++) {
            if (bufProbe[i] != 0xFF) { bufEmpty = false; break; }
        }
    }
    if (dataEmpty && bufEmpty) return FLASHFS_FLASH_FORMAT_EMPTY;

    // Anything that's neither linear nor empty is treated as RING. False positives are
    // benign: the init flow will scan the data section for trailers; if none are found
    // (e.g. random garbage from another use), the log table starts empty and the next
    // log's writes overwrite the garbage. The user sees an empty MSC mount until they
    // log a flight.
    return FLASHFS_FLASH_FORMAT_RING;
}

// =============================================================================
// Chunked helpers (avoid large stack/static buffers)
// =============================================================================

// Copy `length` bytes from `srcAddr` to `dstAddr`, chunked. Both addresses are absolute
// flash offsets within the partition.
static void copyChunkedSync(uint32_t srcAddr, uint32_t dstAddr, uint32_t length)
{
    while (length > 0) {
        uint32_t chunk = MIN(length, (uint32_t)CHUNK_SIZE);
        readBytes(srcAddr, chunkBuf, chunk);
        writeBytesSync(dstAddr, chunkBuf, chunk);
        srcAddr += chunk;
        dstAddr += chunk;
        length -= chunk;
    }
}

// Read and CRC-validate the buffer preamble using streaming reads so we don't need
// a large RAM scratch buffer.
static bool readAndValidateBufferPreamble(flashfsBufferPreamble_t *out, uint32_t *outHeaderLen)
{
    if (readBytes(geom.bufferAreaStart, (uint8_t *)out, sizeof(*out)) != (int)sizeof(*out)) {
        return false;
    }
    if (out->magic != BUFFER_PREAMBLE_MAGIC) return false;

    uint32_t hdrLen = out->headerLengthAndCommit & 0x00FFFFFFu;
    if (hdrLen == 0 || hdrLen > geom.bufferAreaSize - sizeof(*out)) return false;

    uint16_t crc = 0;
    crc = crc16_ccitt_update(crc, &out->logId, sizeof(out->logId));
    crc = crc16_ccitt_update(crc, &out->dataStart, sizeof(out->dataStart));
    crc = crc16_ccitt_update(crc, &hdrLen, sizeof(hdrLen));
    uint32_t addr = geom.bufferAreaStart + sizeof(*out);
    uint32_t remaining = hdrLen;
    while (remaining > 0) {
        uint32_t chunk = MIN(remaining, (uint32_t)CHUNK_SIZE);
        readBytes(addr, chunkBuf, chunk);
        crc = crc16_ccitt_update(crc, chunkBuf, chunk);
        addr += chunk;
        remaining -= chunk;
    }
    if (crc != out->crc) return false;
    *outHeaderLen = hdrLen;
    return true;
}

// =============================================================================
// Power-loss recovery
// =============================================================================

// Recovery: scan the entire ring (with wrap) for an all-0xFF sector. Because the active
// log maintains an erase pool ahead of the writer at all times, at least one fully-erased
// sector exists somewhere in the ring after any in-flight power loss — even if the flight
// wrapped the ring multiple times. That sector is the marker for "where the write head
// was approximately".
//
// Once found, write the trailer there. If the buffered header is larger than one sector,
// erase additional sectors forward until there's enough contiguous erased space for it.
//
// Edge case: if scanning happens to hit a trailer with our logId (rare race between
// trailer write and commit-flag flip in flashfsLogEndLog), the log was already committed
// to data; we just need to clean up the buffer.
static void recoverFromBuffer(void)
{
    flashfsBufferPreamble_t pre;
    uint32_t headerLen = 0;
    if (!readAndValidateBufferPreamble(&pre, &headerLen)) {
        return;
    }

    if (bufferIsCommitted(&pre)) {
        eraseBufferArea();
        return;
    }

    // Scan the whole ring for the erased gap (going forward from the pre-power dataStart,
    // wrapping). Also detect "trailer already there" race.
    uint32_t addr = pre.dataStart;
    uint32_t scanned = 0;
    bool foundGap = false;
    while (scanned < geom.dataSectionEnd) {
        if (addr >= geom.dataSectionEnd) addr = 0;
        flashfsLogTrailer_t t;
        if (readTrailerAt(addr, &t) && t.logId == pre.logId) {
            markBufferCommitted();
            eraseBufferArea();
            return;
        }
        uint8_t probe[16];
        readBytes(addr, probe, sizeof(probe));
        bool isErased = true;
        for (size_t i = 0; i < sizeof(probe); i++) {
            if (probe[i] != 0xFF) { isErased = false; break; }
        }
        if (isErased) { foundGap = true; break; }
        addr += geom.sectorSize;
        scanned += geom.sectorSize;
    }

    if (!foundGap) {
        // No erased sector exists in the ring. This shouldn't happen with a correctly
        // operating erase pool — abandon recovery, the active log is unrecoverable.
        eraseBufferArea();
        return;
    }

    if (addr >= geom.dataSectionEnd) addr = 0;

    uint32_t bytesNeeded = sizeof(flashfsLogTrailer_t) + headerLen;

    // Same end-of-ring spill check as flashfsLogEndLog: the writes below are linear,
    // so if trailer + header would extend past dataSectionEnd we must wrap addr to 0
    // before writing.
    if (addr + bytesNeeded > geom.dataSectionEnd) {
        addr = 0;
    }

    // We know `addr` is sector-aligned and erased. Make sure we have enough contiguous
    // erased space for trailer + header. Erase additional sectors synchronously as needed.
    eraseHead = addr + geom.sectorSize;
    if (eraseHead >= geom.dataSectionEnd) eraseHead = 0;
    pendingEraseAddr = PENDING_ERASE_NONE;
    ensureErasedSpace(addr, bytesNeeded);

    writeTrailer(addr, pre.logId, pre.dataStart, headerLen);
    copyChunkedSync(geom.bufferAreaStart + sizeof(flashfsBufferPreamble_t),
                    addr + sizeof(flashfsLogTrailer_t),
                    headerLen);
    markBufferCommitted();
    eraseBufferArea();
}

// =============================================================================
// Init / erase
// =============================================================================

void flashfsLogInit(void)
{
    moduleInitialized = false;
    moduleSafeForLogging = false;
    detectedFormat = FLASHFS_FLASH_FORMAT_UNKNOWN;
    logTableCount = 0;
    nextLogId = 1;
    dataWriteHead = 0;
    memset(&active, 0, sizeof(active));

    if (!flashfsIsSupported()) return;
    computeGeometry();
    if (geom.partitionSize == 0 || geom.sectorSize == 0 || geom.dataSectionEnd == 0) return;

    detectedFormat = probeFormat();

    switch (detectedFormat) {
    case FLASHFS_FLASH_FORMAT_RING:
        recoverFromBuffer();    // safe no-op if buffer is empty
        buildLogTableByScan();
        dataWriteHead = computeWriteHeadFromTable();
        // We don't know the actual erased frontier from a cold boot — the ring is full
        // of arbitrary contents. Pin eraseHead to dataWriteHead so the next log start
        // sync-erases a fresh pool from the writer's position.
        eraseHead = dataWriteHead;
        moduleSafeForLogging = true;
        break;
    case FLASHFS_FLASH_FORMAT_EMPTY:
        dataWriteHead = 0;
        eraseHead = 0;
        moduleSafeForLogging = true;
        break;
    case FLASHFS_FLASH_FORMAT_LINEAR:
    case FLASHFS_FLASH_FORMAT_UNKNOWN:
    default:
        moduleSafeForLogging = false;
        break;
    }
    moduleInitialized = true;
}

bool flashfsLogIsReady(void)
{
    // Also gates against an in-progress async erase: while flashfs is busy erasing
    // sectors, neither writer nor MSC enumeration is safe.
    return moduleInitialized && moduleSafeForLogging && flashfsIsReady();
}

bool flashfsLogIsFull(void)
{
    return false; // ring mode never reports full
}

void flashfsLogEraseAll(void)
{
    flashfsEraseCompletely();
    detectedFormat = FLASHFS_FLASH_FORMAT_EMPTY;
    logTableCount = 0;
    nextLogId = 1;
    dataWriteHead = 0;
    eraseHead = 0;                          // entire ring will be erased once flash idle
    pendingEraseAddr = PENDING_ERASE_NONE;  // any prior async erase is irrelevant now
    memset(&active, 0, sizeof(active));
    moduleSafeForLogging = true;
}

// =============================================================================
// Read API (MSC)
// =============================================================================

uint32_t flashfsLogGetLogCount(void)
{
    return logTableCount;
}

const flashfsLogInfo_t* flashfsLogGetInfo(uint32_t index)
{
    if (index >= logTableCount) return NULL;
    return &logTable[index];
}

int flashfsLogRead(uint32_t index, uint32_t offsetInLog, uint8_t *dest, uint32_t length)
{
    if (index >= logTableCount) return 0;
    const flashfsLogInfo_t *info = &logTable[index];
    if (!info->valid || offsetInLog >= info->totalSize) return 0;
    if (offsetInLog + length > info->totalSize) length = info->totalSize - offsetInLog;

    int total = 0;

    // Header portion: virtual offsets 0..headerLength → physical [headerOffset..)
    if (offsetInLog < info->headerLength) {
        uint32_t fromHeader = MIN(length, info->headerLength - offsetInLog);
        readBytes(info->headerOffset + offsetInLog, dest, fromHeader);
        dest += fromHeader;
        length -= fromHeader;
        offsetInLog += fromHeader;
        total += fromHeader;
    }

    // Data portion: virtual offsets [headerLength .. totalSize) → data section, may wrap.
    if (length > 0) {
        uint32_t dataOffset = offsetInLog - info->headerLength;
        if (info->dataEnd >= info->dataStart) {
            readBytes(info->dataStart + dataOffset, dest, length);
            total += length;
        } else {
            uint32_t firstSeg = geom.dataSectionEnd - info->dataStart;
            if (dataOffset < firstSeg) {
                uint32_t fromFirst = MIN(length, firstSeg - dataOffset);
                readBytes(info->dataStart + dataOffset, dest, fromFirst);
                dest += fromFirst;
                length -= fromFirst;
                dataOffset += fromFirst;
                total += fromFirst;
            }
            if (length > 0) {
                uint32_t inSecond = dataOffset - firstSeg;
                readBytes(inSecond, dest, length);
                total += length;
            }
        }
    }
    return total;
}

// =============================================================================
// Active log lifecycle
// =============================================================================

bool flashfsLogIsActive(void)      { return active.isOpen; }
bool flashfsLogIsHeaderPhase(void) { return active.isOpen && active.headerPhase; }

bool flashfsLogBeginLog(void)
{
    if (!moduleSafeForLogging) return false;
    if (active.isOpen) return false;

    // Erase the active-log header buffer (synchronous, ~120-400 ms for 4 sectors).
    eraseBufferArea();

    // Pre-erase the data-ring pool ahead of dataWriteHead. If a previous log close left
    // extra erased space (e.g. long header overflowed into more sectors than the pool),
    // eraseHead may already be ahead — only erase the remainder. Up to POOL_TARGET_SECTORS
    // sector erases × ~30-50 ms each = ~120-200 ms log-start latency in the cold case.

    // First reconcile any background erase that may have been pending from a previous
    // flight (eraseTick in the data path schedules them; if a log closed without a
    // matching tick, the variable can be stale). This is also where the wait-for-ready
    // for that pending erase happens.
    if (pendingEraseAddr != PENDING_ERASE_NONE) {
        while (!flashIsReady());
        eraseHead = pendingEraseAddr + geom.sectorSize;
        if (eraseHead >= geom.dataSectionEnd) eraseHead = 0;
        pendingEraseAddr = PENDING_ERASE_NONE;
    }

    // We can't reliably trust eraseHead inherited from a prior flight: any heuristic
    // ("is it close enough to dataWriteHead?") risks misjudging a stale value and
    // leaving the writer to program non-erased sectors. Always pin eraseHead to the
    // writer position and let ensureErasedSpace fill the pool from scratch. The cost
    // is ~POOL_TARGET_SECTORS extra erases per log start (a few tens of ms on top of
    // the buffer-area erase that already runs here) — acceptable for safety.
    eraseHead = dataWriteHead;
    ensureErasedSpace(dataWriteHead, POOL_TARGET_SECTORS * geom.sectorSize);

    memset(&active, 0, sizeof(active));
    active.isOpen = true;
    active.headerPhase = true;
    active.logId = nextLogId++;
    active.dataStart = dataWriteHead;
    active.dataWriteHead = dataWriteHead;
    active.bufferHeaderOffset = 0;

    // Position flashfs's tail at the start of the buffer area's header-text region. From
    // here, header bytes go through the existing buffered async path (flashfsWriteByte)
    // — no per-byte sync flush.
    flashfsSeekAbs(geom.bufferAreaStart + sizeof(flashfsBufferPreamble_t));
    return true;
}

void flashfsLogWriteHeaderByte(uint8_t b)
{
    if (!active.isOpen || !active.headerPhase) return;

    uint32_t maxHdr = geom.bufferAreaSize - sizeof(flashfsBufferPreamble_t);
    if (active.bufferHeaderOffset >= maxHdr) {
        // Header overflow — drop the byte silently. Should never happen with a 16 KB
        // buffer; if it does, the resulting log will be missing some header lines.
        return;
    }
    // Goes through flashfs's existing 128-byte circular write buffer with auto-flush
    // every 64 bytes. Identical bandwidth profile to a normal blackbox header write
    // in linear mode.
    flashfsWriteByte(b);
    active.bufferHeaderOffset++;
}

void flashfsLogFinishHeader(void)
{
    if (!active.isOpen || !active.headerPhase) return;

    uint32_t headerLen = active.bufferHeaderOffset;
    if (headerLen == 0) {
        // No header was written. Treat as logBeginLog never happened.
        memset(&active, 0, sizeof(active));
        return;
    }

    // Drain the header-write buffer to flash so the bytes are stable before we CRC them.
    flashfsFlushSync();

    // Compute the preamble's CRC by streaming the just-written header bytes back through
    // chunked reads — no large RAM scratch.
    writeBufferPreambleStreaming(active.logId, active.dataStart, headerLen);

    active.headerPhase = false;
    // Position flashfs's tail at the data write head for the upcoming data writes.
    flashfsSeekAbs(active.dataWriteHead);
}

void flashfsLogWriteDataByte(uint8_t b)
{
    if (!active.isOpen || active.headerPhase) return;

    // Hot path: must NOT block. eraseTick() does at most a flashIsReady() register read
    // and (if ready) a flashEraseSector() command-submit. Both return in microseconds.
    // The actual erase happens in the chip's background; CPU continues immediately.
    eraseTick();

    // Wrap at end of data ring. flashfsSeekAbs() flushes any buffered bytes to the
    // current end of the ring before redirecting the tail to 0; this can briefly block
    // (~tens of ms in the worst case if chip is mid-erase). Wrap happens at most once
    // per ring loop (~14 MB), so the amortized cost is negligible.
    if (active.dataWriteHead >= geom.dataSectionEnd) {
        active.dataWriteHead = 0;
        flashfsSeekAbs(0);
    }

    // If the writer has caught up to the erase frontier (pool depleted), drop the byte.
    // This is the "writer faster than erase" backstop — rate validation should normally
    // prevent it, but it can happen during sustained 1/4 logging.
    if (active.dataWriteHead == eraseHead) {
        bbDataDrops++;
        return;
    }

    // RAM buffer overflow guard: while the chip is mid-erase, flushes can't drain. Drop
    // bytes rather than silently overwriting unflushed data in the ring buffer.
    if (flashfsGetWriteBufferFreeSpace() == 0) {
        bbDataDrops++;
        return;
    }

    flashfsWriteByte(b);
    active.dataWriteHead++;
}

void flashfsLogWriteData(const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) flashfsLogWriteDataByte(data[i]);
}

uint32_t flashfsLogGetWriteBufferFreeSpace(void) { return flashfsGetWriteBufferFreeSpace(); }
bool flashfsLogFlushAsync(bool force)            { return flashfsFlushAsync(force); }
void flashfsLogFlushSync(void)                   { flashfsFlushSync(); }

void flashfsLogEndLog(void)
{
    if (!active.isOpen) return;

    flashfsFlushSync();

    // Sector-align the trailer position so MSC enumeration finds it without scanning
    // every byte. The bytes between the active.dataWriteHead and the aligned position
    // are abandoned (in ring mode, possibly stale junk from earlier wraps — fine).
    uint32_t trailerAddr = alignUpToSector(active.dataWriteHead);
    if (trailerAddr >= geom.dataSectionEnd) trailerAddr = 0;

    uint32_t headerLen = active.bufferHeaderOffset;
    uint32_t bytesNeeded = sizeof(flashfsLogTrailer_t) + headerLen;

    // The trailer write and header copy below are LINEAR (no ring wrap), but the
    // trailer's chosen position may be close enough to the end of the ring that
    // trailer + header would spill into the reserved buffer area. Detect this and
    // wrap trailerAddr to 0 — the partial sector at the end of the ring is left
    // unused. (The just-closed log's data range is unaffected: it spans
    // [dataStart, trailerAddr) in ring order.)
    if (trailerAddr + bytesNeeded > geom.dataSectionEnd) {
        trailerAddr = 0;
    }

    // Make sure there's enough contiguous erased space at trailerAddr for the trailer
    // record + the full header text. The pool we keep ahead during flight covers the
    // common case; if header is bigger than the remaining pool, sync-erase the extras
    // here. Disarm is non-realtime so blocking is fine.
    ensureErasedSpace(trailerAddr, bytesNeeded);

    writeTrailer(trailerAddr, active.logId, active.dataStart, headerLen);
    // Copy buffered header from the buffer area into the data ring (after the trailer)
    // in chunks so we don't need a large RAM scratch.
    copyChunkedSync(geom.bufferAreaStart + sizeof(flashfsBufferPreamble_t),
                    trailerAddr + sizeof(flashfsLogTrailer_t),
                    headerLen);

    // Update module-level write head to sector-aligned position past the header.
    uint32_t afterHeader = trailerAddr + sizeof(flashfsLogTrailer_t) + headerLen;
    dataWriteHead = alignUpToSector(afterHeader);
    if (dataWriteHead >= geom.dataSectionEnd) dataWriteHead = 0;

    // Mark committed in the buffer (so a power loss during the buffer erase below leaves
    // the recovery path with an unambiguous "already committed" signal), then erase.
    markBufferCommitted();
    eraseBufferArea();

    // Add the just-closed log to the in-RAM table so MSC sees it without a rescan.
    if (logTableCount < FLASHFS_LOG_MAX_LOGS) {
        flashfsLogInfo_t *info = &logTable[logTableCount++];
        info->logId = active.logId;
        info->headerOffset = trailerAddr + sizeof(flashfsLogTrailer_t);
        info->headerLength = headerLen;
        info->dataStart = active.dataStart;
        info->dataEnd = trailerAddr;
        info->valid = true;
        info->wasOpen = false;
        uint32_t dataLen;
        if (info->dataEnd >= info->dataStart) {
            dataLen = info->dataEnd - info->dataStart;
        } else {
            dataLen = (geom.dataSectionEnd - info->dataStart) + info->dataEnd;
        }
        info->totalSize = info->headerLength + dataLen;
    }

    memset(&active, 0, sizeof(active));
}

#endif // USE_BLACKBOX_RING_LOG
