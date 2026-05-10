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
 * Switching between linear and ring modes requires an erase.
 * flashfsLogDetectFormatFromFlash() reports what's on the chip; the writer / MSC
 * paths refuse to operate when there's a mismatch between the configured mode and
 * the on-flash content.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_FLASHFS)

#include "drivers/flash/flash.h"

#include "io/flashfs.h"
#include "io/flashfs_log.h"

// Number of trailing sectors of the partition reserved for the active log's header
// preamble + text. Used both by the ring-mode infrastructure below and (indirectly,
// via the geometry-based detector logic) by always-compiled probes.
#define HDR_BUFFER_SECTORS 4
#define BUFFER_PREAMBLE_MAGIC 0x42424C42u  // "BBLB"  (Betaflight Blackbox Log Buffer)
#define LOG_TRAILER_MAGIC     0x4C424c54u  // "TLBL"  (Trailer Log BLackbox)

// Linear-format probe text. A linear-formatted chip always begins with the first
// log's header text at offset 0 (which always starts "H Product"). Defined here so
// the always-compiled detector can use it without the ring-mode code being present.
static const char LINEAR_FORMAT_PROBE_TEXT[] = "H Product";
#define LINEAR_FORMAT_PROBE_TEXT_LEN (sizeof(LINEAR_FORMAT_PROBE_TEXT) - 1)

// Self-contained format probe — does not depend on the rest of the flashfs_log
// module. Sequence of checks, each catches a different chip state:
//
//   1. offset 0 starts with "H Product"           → LINEAR
//   2. offset 0 contains non-erased ring content  → RING
//   3. buffer-area carries BUFFER_PREAMBLE_MAGIC   → RING (in-flight log, no data
//                                                    landed in sector 0 yet)
//   4. any sector-aligned position carries
//      LOG_TRAILER_MAGIC                          → RING (clean-closed ring where
//                                                    the erase pool happened to
//                                                    wrap past sector 0 AND the
//                                                    buffer-area was erased on
//                                                    close — neither (2) nor (3)
//                                                    catch this case)
//   5. otherwise                                  → EMPTY
//
// (4) is the durable signature for a populated ring: trailers are never erased
// outside of explicit user erase or being overwritten by a newer ring lap. The
// scan is O(partition / sectorSize) reads of 4 bytes each — ~3500 reads on a
// 14 MB chip, runs once at boot, no hot-path cost.
flashfsFlashFormat_e flashfsLogDetectFormatFromFlash(void)
{
    uint8_t probe[16];
    if (flashfsReadAbs(0, probe, sizeof(probe)) != (int)sizeof(probe)) {
        return FLASHFS_FLASH_FORMAT_UNKNOWN;
    }
    if (memcmp(probe, LINEAR_FORMAT_PROBE_TEXT, LINEAR_FORMAT_PROBE_TEXT_LEN) == 0) {
        return FLASHFS_FLASH_FORMAT_LINEAR;
    }

    // Track whether sector 0 was fully erased — only a fully-erased chip can
    // ultimately classify as EMPTY. Non-erased data without a ring signature is
    // UNKNOWN (caller decides whether that's safe to overwrite); we no longer
    // assume "any non-erased prefix" means ring, which falsely classified
    // garbage / partial linear / orphan-from-other-use chips.
    bool offset0Erased = true;
    for (size_t i = 0; i < sizeof(probe); i++) {
        if (probe[i] != 0xFF) { offset0Erased = false; break; }
    }

    const flashGeometry_t *fg = flashGetGeometry();
    if (!fg || fg->sectorSize == 0) return FLASHFS_FLASH_FORMAT_UNKNOWN;
    const uint32_t partitionSize = flashfsGetSize();
    const uint32_t bufferAreaSize = HDR_BUFFER_SECTORS * fg->sectorSize;
    if (bufferAreaSize >= partitionSize) return FLASHFS_FLASH_FORMAT_UNKNOWN;
    const uint32_t bufferAreaStart = partitionSize - bufferAreaSize;

    // Buffer-area preamble magic: catches the in-flight ring state where the
    // active log finalised its header but no data landed in sector 0 before crash.
    {
        uint32_t magic = 0;
        if (flashfsReadAbs(bufferAreaStart, (uint8_t *)&magic, sizeof(magic)) == (int)sizeof(magic)
                && magic == BUFFER_PREAMBLE_MAGIC) {
            return FLASHFS_FLASH_FORMAT_RING;
        }
    }

    // Sector-aligned trailer magic scan. Trailers persist across clean closes and
    // across erase-pool wraps, so this is the durable signature that cleanly-closed
    // ring volumes have when offset 0 and the buffer area both happen to be erased.
    // Start at addr 0: a ring wrap can place a trailer at offset 0 of the data
    // section (when the previous log's tail aligned exactly at dataSectionEnd).
    // The probe at the top already rejected LINEAR_FORMAT_PROBE_TEXT, so a
    // LOG_TRAILER_MAGIC at 0 is unambiguously ring.
    for (uint32_t addr = 0; addr + sizeof(uint32_t) <= bufferAreaStart; addr += fg->sectorSize) {
        uint32_t magic = 0;
        if (flashfsReadAbs(addr, (uint8_t *)&magic, sizeof(magic)) == (int)sizeof(magic)
                && magic == LOG_TRAILER_MAGIC) {
            return FLASHFS_FLASH_FORMAT_RING;
        }
    }

    // No ring signature found anywhere. EMPTY only if the chip looks fully erased
    // (sector 0 was 0xFF — we already checked above); otherwise UNKNOWN, which
    // signals callers to refuse destructive writes until the user erases.
    return offset0Erased ? FLASHFS_FLASH_FORMAT_EMPTY : FLASHFS_FLASH_FORMAT_UNKNOWN;
}

#ifdef USE_BLACKBOX_RING_LOG

#include "build/debug.h"
#include "common/crc.h"
#include "common/maths.h"

// =============================================================================
// Layout constants
// =============================================================================

// HDR_BUFFER_SECTORS and BUFFER_PREAMBLE_MAGIC are defined above this #ifdef so
// the always-compiled flashfsLogDetectFormatFromFlash() can use them.

// Erase-ahead pool size in sectors. The writer always operates at least this many
// sectors behind the leading edge of erased space, so per-byte writes never block on
// flash erase. At log start we sync-erase POOL_TARGET_SECTORS sectors ahead; during
// flight we kick off async erases (chip background) to keep the pool full.
//
// Sustained writer rate vs erase rate determines whether the pool depletes. NOR erase
// is ~80-133 KB/s; at ~80 B/frame the chip can sustain ~1000-1660 frames/sec. The
// validated-config Hz cap (BLACKBOX_RING_MAX_FRAME_HZ = 1000) keeps the writer
// nominally below the chip's erase rate. Worst-case erase outliers can still briefly
// drain the pool — the drop guard takes over and the log shows a small gap.
#define POOL_TARGET_SECTORS 4

// Number of erase-pool sectors held in reserve at all times for power-loss
// recovery. The writer drops bytes once `headroom <= RECOVERY_RESERVED_SECTORS *
// sectorSize` instead of at strict `writer == eraseHead`, which guarantees that
// at any moment during a flight there are RECOVERY_RESERVED_SECTORS fully-erased
// sectors immediately preceding eraseHead in ring order. Recovery's gap-scan is
// therefore guaranteed to find an erased sector to land the trailer in, even if
// the writer was at the edge of pool depletion at the moment of crash. K=1 (one
// sector, ~4 KB) is enough for the trailer + a small header; recovery's existing
// ensureErasedSpace() sync-erases additional sectors if the buffered header
// needs more space. Cost: ~4 KB of the ~14 MB ring is reserved (~0.03%), and
// drops kick in one sector earlier under transient erase-pool depletion.
#define RECOVERY_RESERVED_SECTORS 1

// Trailer/preamble magic constants are defined above the USE_BLACKBOX_RING_LOG
// guard so the always-compiled detector can use them.

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
// (Format is no longer cached at module scope: the only consumer is the boot-time
// switch in flashfsLogInit; external callers fetch it fresh via
// flashfsLogDetectFormatFromFlash, which is cheap to re-run.)

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
    bool      everWrapped;         // dataWriteHead has crossed dataSectionEnd at least once this session
    bool      lapped;              // dataWriteHead has overtaken dataStart (writer is overwriting this session's earliest bytes)
    bool      lappedPersisted;     // markBufferLapped() has flipped the preamble's lapped marker for this session
    uint32_t  logId;
    uint32_t  dataStart;
    uint32_t  dataWriteHead;       // mirrors module-level dataWriteHead during the flight
    uint32_t  bufferHeaderOffset;  // bytes written into header buffer so far
} active;

// Wrap-aware ring-range overlap test. Each range is [start, end) in a ring of size
// geom.dataSectionEnd. If end <= start the range wraps and covers
// [start, dataSectionEnd) ∪ [0, end). Returns true iff the two ranges share any byte.
static bool ringRangesOverlap(uint32_t aStart, uint32_t aEnd, uint32_t bStart, uint32_t bEnd)
{
    // Decompose each into 1-2 linear sub-ranges, then test all pairs.
    uint32_t aSubs[2][2];
    int aN = 0;
    if (aStart < aEnd) {
        aSubs[aN][0] = aStart; aSubs[aN][1] = aEnd; aN++;
    } else if (aStart > aEnd) {
        aSubs[aN][0] = aStart; aSubs[aN][1] = geom.dataSectionEnd; aN++;
        aSubs[aN][0] = 0; aSubs[aN][1] = aEnd; aN++;
    } // start == end: empty range, no sub-ranges
    uint32_t bSubs[2][2];
    int bN = 0;
    if (bStart < bEnd) {
        bSubs[bN][0] = bStart; bSubs[bN][1] = bEnd; bN++;
    } else if (bStart > bEnd) {
        bSubs[bN][0] = bStart; bSubs[bN][1] = geom.dataSectionEnd; bN++;
        bSubs[bN][0] = 0; bSubs[bN][1] = bEnd; bN++;
    }
    for (int i = 0; i < aN; i++) {
        for (int j = 0; j < bN; j++) {
            const uint32_t lo = aSubs[i][0] > bSubs[j][0] ? aSubs[i][0] : bSubs[j][0];
            const uint32_t hi = aSubs[i][1] < bSubs[j][1] ? aSubs[i][1] : bSubs[j][1];
            if (lo < hi) return true;
        }
    }
    return false;
}

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
    // Most callers (eraseBufferArea on close, ensureErasedSpace at log start) come
    // through paths that already drain via flashfsFlushSync, but a previous async
    // erase tick can still be in flight. Wait for the chip before submitting the
    // new erase command; NOR parts ignore commands while BUSY is asserted, which
    // would silently no-op the erase and leave a non-erased sector that the
    // subsequent program would land on.
    while (!flashIsReady());
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
    //
    // The flashIsReady() gate is required even on this no-pending-erase path: an
    // async page program from the data writer can still have the chip BUSY here.
    // Most NOR parts ignore commands while BUSY, so issuing flashEraseSector() now
    // would silently no-op while we still record pendingEraseAddr=eraseHead. The
    // next eraseTick() would then see flashIsReady() true (program completed),
    // declare the non-existent erase done, and advance eraseHead past a sector
    // that was never erased. Subsequent programs into that sector would land on
    // non-erased flash. Skipping this tick is safe — eraseTick() runs every data
    // byte, the next call will retry.
    uint32_t writer = active.isOpen ? active.dataWriteHead : dataWriteHead;
    uint32_t spaceAhead = ringSpaceFreeAhead(writer);
    if (spaceAhead < POOL_TARGET_SECTORS * geom.sectorSize && flashIsReady()) {
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
    if (out->crc != expected) return false;

    // Geometry sanity. A valid trailer points at a header that lives in the data
    // ring, immediately after the trailer itself. Reject anything whose claimed
    // dataStart or headerLength would put header bytes outside the ring or have
    // them collide with the trailer record. Without this guard, an unlucky bit
    // pattern that happens to match the magic AND happens to CRC clean could feed
    // garbage offsets into latestTrailerWriteHead / computeWriteHeadFromTable /
    // flashfsLogRead and corrupt the log table.
    if (out->dataStart >= geom.dataSectionEnd) return false;
    if (out->headerLength == 0 || out->headerLength > geom.bufferAreaSize - sizeof(flashfsBufferPreamble_t)) return false;
    const uint32_t headerEnd = addr + sizeof(*out) + out->headerLength;
    if (headerEnd > geom.dataSectionEnd) return false;

    return true;
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
    // .reserved is initialised to 0xFFFF rather than 0 so it can be 1→0 flipped
    // in place when the session laps (markBufferLapped) — recovery uses it to know
    // the writer has overtaken the original dataStart. NOR can only program 1 bits
    // to 0; the all-ones initial value is what makes the in-place update possible.
    flashfsBufferPreamble_t p = {
        .magic = BUFFER_PREAMBLE_MAGIC,
        .reserved = 0xFFFF,
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

// Flip the preamble's reserved field from 0xFFFF to 0x0000 in place. Called once
// per session, from the non-realtime flush path, the first time active.lapped
// transitions to true (i.e. the writer has overtaken the session's own dataStart).
// recoverFromBuffer reads the field — if it sees 0x0000, the session lapped and
// the original preamble.dataStart no longer points at intact data. The CRC in
// the preamble does NOT cover this field (it's over logId, dataStart,
// headerLength, headerText), so flipping it doesn't invalidate the commit signal.
//
// writeBytesSync seeks flashfs's tail to bufferAreaStart for the write — but the
// data writer is mid-session expecting the tail to stay in the data ring at
// active.dataWriteHead. We seek back after the preamble update so subsequent
// data bytes land where they should.
static void markBufferLapped(void)
{
    flashfsBufferPreamble_t p;
    if (readBytes(geom.bufferAreaStart, (uint8_t *)&p, sizeof(p)) != (int)sizeof(p)) return;
    if (p.reserved == 0) return; // already lapped
    p.reserved = 0;
    writeBytesSync(geom.bufferAreaStart, (const uint8_t *)&p, sizeof(p));
    if (active.isOpen && !active.headerPhase) {
        flashfsSeekAbs(active.dataWriteHead);
    }
}

// =============================================================================
// Data ring scan (MSC enumeration)
// =============================================================================
//
// Scan every sector in the data section. Read the first sizeof(trailer) bytes of each
// sector and check for the trailer magic. Each match describes one log.

static uint32_t latestTrailerWriteHead = 0;
static bool latestTrailerWriteHeadValid = false;

static void buildLogTableByScan(void)
{
    logTableCount = 0;
    nextLogId = 1;
    latestTrailerWriteHead = 0;
    latestTrailerWriteHeadValid = false;
    uint32_t latestLogId = 0;

    for (uint32_t addr = 0; addr + geom.sectorSize <= geom.dataSectionEnd; addr += geom.sectorSize) {
        flashfsLogTrailer_t t;
        if (!readTrailerAt(addr, &t)) continue;

        // Always update nextLogId from every trailer encountered, even after the
        // table is full — otherwise the next ring-wrap-eviction-aware close would
        // re-use a stale id below the on-flash maximum, breaking sort/eviction
        // assumptions ("table is sorted ascending by logId, append in order").
        //
        // Also separately track the highest-logId trailer's post-header write head,
        // because computeWriteHeadFromTable() needs to recover the writer position
        // even when there are more than FLASHFS_LOG_MAX_LOGS trailers on flash —
        // scanning logTable[] alone would pick a "latest" that has been truncated
        // out, rewinding dataWriteHead to an older log end and overlapping recent
        // logs on the next session.
        if (t.logId >= nextLogId) nextLogId = t.logId + 1;
        if (!latestTrailerWriteHeadValid || t.logId > latestLogId) {
            latestLogId = t.logId;
            uint32_t afterHeader = addr + sizeof(t) + t.headerLength;
            uint32_t writeHead = alignUpToSector(afterHeader);
            if (writeHead >= geom.dataSectionEnd) writeHead = 0;
            latestTrailerWriteHead = writeHead;
            latestTrailerWriteHeadValid = true;
        }

        // Pick the destination slot. While the table has room, append. Once at the
        // cap, replace the entry with the smallest logId only if this new trailer
        // is newer — that way we always end up with the most-recent 64 trailers
        // regardless of physical scan order. Plain `continue` would silently lose
        // newer flights and keep older ones whose ranges were already overwritten.
        flashfsLogInfo_t *info = NULL;
        if (logTableCount < FLASHFS_LOG_MAX_LOGS) {
            info = &logTable[logTableCount++];
        } else {
            uint32_t smallestIdx = 0;
            for (uint32_t i = 1; i < FLASHFS_LOG_MAX_LOGS; i++) {
                if (logTable[i].logId < logTable[smallestIdx].logId) smallestIdx = i;
            }
            if (t.logId > logTable[smallestIdx].logId) {
                info = &logTable[smallestIdx];
            } else {
                continue;
            }
        }

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

    // Replay the same wrap-aware overlap eviction that flashfsLogEndLog() runs at
    // close. Without this, an older trailer whose data/header range was clobbered
    // by a newer wrapped log would still be exposed via MSC after reboot, surfacing
    // a phantom file pointing at bytes that now belong to a different log. Walk
    // newest → oldest (table is sorted ascending by logId) and evict any older
    // entry whose footprint overlaps a newer one's footprint.
    for (uint32_t newer = logTableCount; newer-- > 1; ) {
        const uint32_t newerStart = logTable[newer].dataStart;
        // Mirror flashfsLogEndLog's footprint: the close path advances the write head
        // to the next sector boundary after the header, so any older log whose data
        // range falls inside that alignment slack was clobbered too. Use the aligned
        // post-header boundary here so boot-time eviction matches close-time eviction
        // — otherwise an older log surviving only in the slack reappears as a phantom
        // MSC entry pointing at bytes that now belong to the newer log.
        uint32_t newerEnd = alignUpToSector(logTable[newer].headerOffset + logTable[newer].headerLength);
        if (newerEnd >= geom.dataSectionEnd) newerEnd = 0;
        // If the newer log occupies the entire ring (start == aligned end after wrap),
        // every older entry is by definition inside its footprint.
        const bool newerFullRing = (newerStart == newerEnd);
        for (uint32_t older = 0; older < newer; older++) {
            if (!logTable[older].valid) continue;
            const uint32_t olderStart = logTable[older].dataStart;
            const uint32_t olderEnd   = logTable[older].headerOffset + logTable[older].headerLength;
            if (newerFullRing || ringRangesOverlap(newerStart, newerEnd, olderStart, olderEnd)) {
                logTable[older].valid = false;
            }
        }
    }
    // Compact: drop the entries we just marked invalid.
    uint32_t kept = 0;
    for (uint32_t i = 0; i < logTableCount; i++) {
        if (logTable[i].valid) {
            if (kept != i) logTable[kept] = logTable[i];
            kept++;
        }
    }
    logTableCount = kept;
}

// Determine the next data write head from the latest log's trailer (highest logId).
// On a fresh chip with no logs, returns 0.
static uint32_t computeWriteHeadFromTable(void)
{
    // buildLogTableByScan() captured the post-header write position of the
    // highest-logId trailer it observed across the WHOLE flash, regardless of
    // FLASHFS_LOG_MAX_LOGS truncation. Use that instead of re-scanning logTable[]
    // — on a wrapped chip with > 64 logs, the in-RAM table is missing the truly
    // latest entries and would rewind dataWriteHead to an older log end.
    return latestTrailerWriteHeadValid ? latestTrailerWriteHead : 0;
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

// Recovery: replay flashfsLogEndLog from the right address.
//
// The close path's trailer-first ordering means power loss leaves one of three
// on-flash states (buffer-already-committed is handled before we get here):
//
//   1. Crash BEFORE writeTrailer fired (or ensureErasedSpace was still in flight):
//      no trailer in the ring, but a pre-erased gap exists at the close target
//      address. Even mid-flight (no close attempted), the writer's drop check
//      reserves RECOVERY_RESERVED_SECTORS fully-erased sectors immediately
//      preceding eraseHead at all times, so an erased gap is guaranteed to exist
//      somewhere in the ring regardless of pool depletion at the moment of crash.
//   2. Crash mid copyChunkedSync (or after writeTrailer but before the copy started):
//      trailer is present at the close target address; the post-trailer header bytes
//      may be partial or absent. We re-do the close at the trailer's address — that
//      requires re-erasing the trailer's own sector, which is fine because we
//      immediately re-write the trailer with the same content.
//   3. Crash after copyChunkedSync but before markBufferCommitted: trailer present,
//      header complete, buffer uncommitted. Same handling as (2): re-doing is
//      idempotent.
//
// In all three cases we walk the ring forward from pre.dataStart looking for
// either a matching-logId trailer or a fully-erased sector — whichever appears
// first IS the close target address. Then we replay the close path (writeTrailer,
// copyChunkedSync, markBufferCommitted, eraseBufferArea) at that address.
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

    uint32_t addr = pre.dataStart;
    uint32_t scanned = 0;
    bool found = false;
    bool foundTrailer = false;
    while (scanned < geom.dataSectionEnd) {
        if (addr >= geom.dataSectionEnd) addr = 0;
        flashfsLogTrailer_t t;
        if (readTrailerAt(addr, &t) && t.logId == pre.logId) {
            // Case (2)/(3): close was attempted, trailer landed. Re-do the close
            // here — replays writeTrailer + header copy + commit, fully idempotent.
            // The fast path that used to commit-and-erase on a trailer sighting was
            // a corruption source: if the crashed close had only partially copied
            // the header, that fast path would erase the buffer's authoritative
            // header copy and freeze the partial header on flash forever.
            foundTrailer = true;
            found = true;
            break;
        }
        // Validate the whole sector — recovery then advances eraseHead past it on
        // the assumption that everything inside is 0xFF. A 16-byte probe was not
        // enough: a used sector that happens to start with 16 erased bytes (e.g. a
        // small log fragment) would be falsely accepted as the gap, and the next
        // trailer/header write would land on non-erased NOR.
        bool isErased = true;
        for (uint32_t off = 0; off < geom.sectorSize && isErased; off += CHUNK_SIZE) {
            const uint32_t chunk = MIN(geom.sectorSize - off, (uint32_t)CHUNK_SIZE);
            if (readBytes(addr + off, chunkBuf, chunk) != (int)chunk) {
                isErased = false;
                break;
            }
            for (uint32_t i = 0; i < chunk; i++) {
                if (chunkBuf[i] != 0xFF) {
                    isErased = false;
                    break;
                }
            }
        }
        if (isErased) {
            // Case (1): no trailer reached flash, but a pre-erased gap is here.
            found = true;
            break;
        }
        addr += geom.sectorSize;
        scanned += geom.sectorSize;
    }

    if (!found) {
        // Neither a matching trailer nor an erased sector exists in the ring.
        // Shouldn't be reachable: the writer's RECOVERY_RESERVED_SECTORS guard
        // keeps K sectors fully erased at all times during a flight. Hitting
        // this branch implies a deeper invariant violation (e.g. flash hardware
        // fault, or the chip was tampered with between sessions). Abandon
        // recovery for the active log; next session starts cleanly.
        eraseBufferArea();
        return;
    }

    if (addr >= geom.dataSectionEnd) addr = 0;

    uint32_t bytesNeeded = sizeof(flashfsLogTrailer_t) + headerLen;

    // Same end-of-ring spill check as flashfsLogEndLog: the writes below are linear,
    // so if trailer + header would extend past dataSectionEnd we must wrap addr to 0
    // before writing. (Only relevant for case (1): in cases (2)/(3) the trailer is
    // already at a known-good address that the original close vetted.)
    bool wrappedToFit = false;
    if (!foundTrailer && addr + bytesNeeded > geom.dataSectionEnd) {
        addr = 0;
        wrappedToFit = true;
    }

    // Reset the erase frontier ahead of `addr` for the upcoming linear write.
    //  - foundTrailer: the trailer's sector + following header sectors are already
    //    pre-erased OR partially programmed. Either way ensureErasedSpace must
    //    sync-erase from `addr` to guarantee the post-trailer copy lands on clean
    //    flash. (The trailer itself gets re-erased and immediately re-written.)
    //  - !foundTrailer + wrappedToFit: sector 0 is whatever stale data was there,
    //    sync-erase from scratch.
    //  - !foundTrailer + !wrappedToFit: the gap-scan verified `addr` is erased,
    //    so the pool starts one sector ahead.
    pendingEraseAddr = PENDING_ERASE_NONE;
    if (foundTrailer || wrappedToFit) {
        eraseHead = addr;
    } else {
        eraseHead = addr + geom.sectorSize;
        if (eraseHead >= geom.dataSectionEnd) eraseHead = 0;
    }
    ensureErasedSpace(addr, bytesNeeded);

    // Mirror flashfsLogEndLog's lapped-aware dataStart selection. If the in-flight
    // session lapped before the crash, pre.dataStart points at flash that the
    // writer has since overwritten — the readable range is the whole ring minus
    // the trailer + header sectors we're about to write, so set the trailer's
    // dataStart to the post-header position (sector-aligned past addr + sizeof
    // trailer + headerLen). The "lapped" signal comes from the preamble's
    // reserved field (set 0xFFFF on log start, flipped to 0 by markBufferLapped
    // in flashfsLogFlushAsync the first time the writer overtakes its own dataStart).
    uint32_t recoveredDataStart;
    if (pre.reserved == 0) {
        uint32_t afterHeader = addr + sizeof(flashfsLogTrailer_t) + headerLen;
        recoveredDataStart = alignUpToSector(afterHeader);
        if (recoveredDataStart >= geom.dataSectionEnd) recoveredDataStart = 0;
    } else {
        recoveredDataStart = pre.dataStart;
    }

    // Trailer-first, matching flashfsLogEndLog. If we crash again before the copy
    // completes, the next recovery sees the trailer and converges on the same
    // address — idempotent retry.
    writeTrailer(addr, pre.logId, recoveredDataStart, headerLen);
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
    logTableCount = 0;
    nextLogId = 1;
    dataWriteHead = 0;
    memset(&active, 0, sizeof(active));

    if (!flashfsIsSupported()) return;
    computeGeometry();
    if (geom.partitionSize == 0 || geom.sectorSize == 0 || geom.dataSectionEnd == 0) return;

    // Note: ring wrap is enabled per-session in flashfsLogBeginLog and disabled in
    // End/Abort. We deliberately do NOT enable it here because the same flashfs
    // instance is used by linear-mode blackbox in builds where USE_BLACKBOX_RING_LOG
    // is compiled in but the user has BLACKBOX_FLASH_MODE_LINEAR configured at
    // runtime — enabling the wrap globally would cause those linear writes to loop
    // back over the start of the chip when they reach dataSectionEnd, corrupting
    // earlier linear logs. Recovery writes done below use writeBytesSync at explicit
    // addresses bounded by the trailer wrap-to-0 logic, so they never need the ring
    // wrap to be active.

    const flashfsFlashFormat_e fmt = flashfsLogDetectFormatFromFlash();
    switch (fmt) {
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
    // Goes through flashfs's existing circular write buffer (sized 16 KB on
    // USE_BLACKBOX_RING_LOG builds, with FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN = 256 B
    // page-aligned auto-flush — see flashfs.h). Header writes hit the same buffer
    // as data writes; identical bandwidth profile to a linear-mode header write.
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

    // Enable ring wrap for the data writer — from here on, sequential writes that
    // hit dataSectionEnd loop back to 0 inside flashfs without us needing to seek
    // from the hot path. Disabled again in flashfsLogEndLog / flashfsLogAbortLog so
    // post-session writes (recovery, mode-switch to linear, etc.) advance linearly.
    flashfsSetRing(0, geom.dataSectionEnd);
}

void flashfsLogWriteDataByte(uint8_t b)
{
    if (!active.isOpen || active.headerPhase) return;

    // Hot path: must NOT block. eraseTick() does at most a flashIsReady() register read
    // and (if ready) a flashEraseSector() command-submit. Both return in microseconds.
    // The actual erase happens in the chip's background; CPU continues immediately.
    eraseTick();

    // Wrap at end of data ring. flashfs is configured (via flashfsSetRing in
    // flashfsLogFinishHeader) to wrap its own tailAddress automatically, so we
    // only need to mirror the wrap in our local dataWriteHead — no seek, no flush,
    // no stall. The wrap fires at most every ~14 MB so the branch is essentially
    // free under prediction.
    //
    // active.lapped means "the writer has overtaken its own session's dataStart" —
    // i.e. the byte at dataStart has been overwritten by later session data, so the
    // trailer's dataStart pointer needs to shift forward (see flashfsLogEndLog). It
    // does NOT cause the writer to drop bytes — multi-lap sessions are an explicit
    // feature (long flights with the chip acting as a sliding window over the most
    // recent ring-size-worth of data).
    //
    // For sessions that start mid-ring, the first wrap-past-dataSectionEnd is NOT
    // the lap event: writeHead progresses from 0 toward dataStart and the original
    // dataStart byte is still intact until writeHead crosses it. Track wrap separately
    // (everWrapped) and set lapped once both conditions hold.
    if (active.dataWriteHead >= geom.dataSectionEnd) {
        active.dataWriteHead -= geom.dataSectionEnd;
        active.everWrapped = true;
    }

    // Erase-pool backstop with recovery reservation. Drops kick in when the writer
    // is within RECOVERY_RESERVED_SECTORS sectors of eraseHead — leaving those K
    // sectors fully erased at all times so that recoverFromBuffer is guaranteed to
    // find an erased gap to land the trailer in after a power-loss crash. (Strict
    // `writer == eraseHead` would let the pool deplete to zero, leaving no sector
    // for recovery to use.) The Hz-based rate cap normally keeps the writer well
    // below the chip's erase rate; this guard only fires under worst-case erase
    // outliers.
    {
        const uint32_t headroom = (eraseHead >= active.dataWriteHead)
            ? (eraseHead - active.dataWriteHead)
            : (geom.dataSectionEnd - active.dataWriteHead) + eraseHead;
        if (headroom <= RECOVERY_RESERVED_SECTORS * geom.sectorSize) {
            bbDataDrops++;
            return;
        }
    }

    // RAM buffer overflow guard: while the chip is mid-erase, flushes can't drain. Drop
    // bytes rather than silently overwriting unflushed data in the ring buffer.
    if (flashfsGetWriteBufferFreeSpace() == 0) {
        bbDataDrops++;
        return;
    }

    flashfsWriteByte(b);
    active.dataWriteHead++;

    // Lap is committed only AFTER a byte successfully programs at or past dataStart
    // post-wrap (the prior byte is the one that just overwrote dataStart). Doing the
    // check after the drop checks ensures dropped bytes don't falsely mark the
    // session lapped, which would make flashfsLogEndLog compute a post-trailer
    // dataStart pointing at stale flash content from earlier flights.
    if (!active.lapped && active.everWrapped && active.dataWriteHead > active.dataStart) {
        active.lapped = true;
    }
}

void flashfsLogWriteData(const uint8_t *data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) flashfsLogWriteDataByte(data[i]);
}

uint32_t flashfsLogGetWriteBufferFreeSpace(void) { return flashfsGetWriteBufferFreeSpace(); }

// Persist the lap marker into the buffer preamble if the writer has lapped this
// session and the marker hasn't already been written. Called from BOTH async and
// sync flush entry points so a power-cut during the close-time sync drain can
// still find the lapped state in recovery — without this, a session that lapped
// after its last async flush and crashed during close would replay recovery with
// a stale pre-lap dataStart and resurrect overwritten bytes at the front of the
// log. Both call sites are non-realtime (blackbox task / disarm path) so the
// flash bookkeeping I/O does not touch the PID loop hot path.
static void persistLappedMarkerIfNeeded(void)
{
    if (active.isOpen && active.lapped && !active.lappedPersisted) {
        markBufferLapped();
        active.lappedPersisted = true;
    }
}

bool flashfsLogFlushAsync(bool force)
{
    persistLappedMarkerIfNeeded();
    return flashfsFlushAsync(force);
}

void flashfsLogFlushSync(void)
{
    persistLappedMarkerIfNeeded();
    flashfsFlushSync();
}

void flashfsLogEndLog(void)
{
    if (!active.isOpen) return;

    // Drain the in-RAM write buffer FIRST while ring wrap is still active — any bytes
    // pending from the just-closed session may need to wrap, and clearing the ring
    // before flushing would force them down the linear-overflow path (drop). After the
    // flush the buffer is empty, so the close-path's mixed-address writes (trailer +
    // header copy + buffer-area erase) can safely run linear: those writes are
    // explicitly bounded by the wrap-to-0 logic below so they never span dataSectionEnd.
    //
    // Use flashfsLogFlushSync (the wrapping variant) rather than flashfsFlushSync
    // directly, so the lapped marker is persisted to the preamble before the drain.
    // Without this, a power cut during this sync flush would replay recovery with
    // a stale pre-lap dataStart on a session that lapped after its last async flush.
    flashfsLogFlushSync();
    flashfsClearRing();

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

    // Compute the logDataStart and post-close write head BEFORE the trailer write.
    //
    // Trailer's dataStart describes where the *currently readable* range begins.
    //   - Non-lapped sessions: just the original active.dataStart. Read range is
    //     the linear span [dataStart, trailerAddr).
    //   - Lapped sessions: the original active.dataStart byte has been overwritten
    //     by later session data, so it's the wrong pointer to expose. The intact
    //     range is the whole ring minus the trailer + header sectors we're about
    //     to write. Set dataStart to the first byte past the post-close write head
    //     so the reader walks ring-wraps from there back around to trailerAddr —
    //     the last ringSize-headerLen-trailerLen bytes the writer produced.
    uint32_t afterHeader = trailerAddr + sizeof(flashfsLogTrailer_t) + headerLen;
    dataWriteHead = alignUpToSector(afterHeader);
    if (dataWriteHead >= geom.dataSectionEnd) dataWriteHead = 0;
    const uint32_t logDataStart = active.lapped ? dataWriteHead : active.dataStart;

    // Write trailer FIRST, then copy the header that the trailer points at. The
    // trailer is the on-ring "close intent" marker — its presence anywhere in the
    // ring tells recovery exactly where the in-flight close was attempting to land
    // its trailer + header. A power loss between writeTrailer() and the end of
    // copyChunkedSync() leaves a complete trailer pointing at a (possibly partial)
    // header sitting in pre-erased flash; the buffer is still uncommitted, so
    // recovery treats this as "redo the close at this trailer's address" rather
    // than fast-path-committing onto a partial header. See recoverFromBuffer().
    //
    // The previous order (header first, trailer last) was a workaround for a now-
    // removed recovery fast path that would erase the buffer on any matching trailer
    // sighting, including matches over a partially-copied header. With the recovery
    // path fixed to redo the close instead of fast-pathing, trailer-first is both
    // safer (recovery has an unambiguous landing position to resume at — no need to
    // gap-scan past partial-copy debris) and simpler.
    writeTrailer(trailerAddr, active.logId, logDataStart, headerLen);

    copyChunkedSync(geom.bufferAreaStart + sizeof(flashfsBufferPreamble_t),
                    trailerAddr + sizeof(flashfsLogTrailer_t),
                    headerLen);

    // Mark committed in the buffer (so a power loss during the buffer erase below leaves
    // the recovery path with an unambiguous "already committed" signal), then erase.
    markBufferCommitted();
    eraseBufferArea();

    // Add the just-closed log to the in-RAM table so MSC sees it without a rescan.
    //
    // The new log occupies the ring range [logDataStart, trailerAddr) (with wrap on
    // lapped sessions, where logDataStart > trailerAddr). After a ring wrap, any
    // older logs whose footprints overlap that range have had their on-flash bytes
    // clobbered — keeping them in the table would expose phantom files via MSC
    // pointing at stale or rewritten data. Evict them. Separately, if the table is
    // at its hard cap (FLASHFS_LOG_MAX_LOGS) we used to silently stop appending,
    // which made every log past the 64th invisible until reboot — drop the oldest
    // entry to keep the newest one reachable.
    // Use the sector-aligned post-close write head as the new log's footprint end:
    // the bytes between afterHeader and dataWriteHead (the alignUpToSector slack)
    // are abandoned but still inside the new log's claimed range from the next
    // session's perspective, so older entries overlapping that slack should be
    // evicted too. For lapped sessions where dataWriteHead == logDataStart (a
    // full-ring footprint after wrap), every other entry is by definition
    // overlapping — handle that explicitly.
    const uint32_t newStart = logDataStart;
    const uint32_t newEnd = dataWriteHead;
    const bool newFullRing = active.lapped && newStart == newEnd;
    uint32_t kept = 0;
    for (uint32_t i = 0; i < logTableCount; i++) {
        const uint32_t entryStart = logTable[i].dataStart;
        const uint32_t entryEnd = logTable[i].headerOffset + logTable[i].headerLength;
        const bool overlaps = newFullRing || ringRangesOverlap(newStart, newEnd, entryStart, entryEnd);
        if (!overlaps) {
            if (kept != i) logTable[kept] = logTable[i];
            kept++;
        }
    }
    logTableCount = kept;

    if (logTableCount >= FLASHFS_LOG_MAX_LOGS) {
        // Drop the oldest (logTable[0] — the table is sorted by logId ascending at boot
        // and append-in-order at runtime, so [0] is always the oldest survivor).
        memmove(&logTable[0], &logTable[1], sizeof(logTable[0]) * (FLASHFS_LOG_MAX_LOGS - 1));
        logTableCount = FLASHFS_LOG_MAX_LOGS - 1;
    }

    flashfsLogInfo_t *info = &logTable[logTableCount++];
    info->logId = active.logId;
    info->headerOffset = trailerAddr + sizeof(flashfsLogTrailer_t);
    info->headerLength = headerLen;
    info->dataStart = logDataStart;
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

    memset(&active, 0, sizeof(active));
}

// Caller asked to discard (e.g. blackboxDeviceEndLog(retainLog=false)). We do NOT
// write a trailer — without one, log enumeration ignores this slot and the orphan
// data in the ring is overwritten by the next wrap. Still need to:
//   - flush any buffered writes so the chip is quiescent before erasing the buffer
//   - advance dataWriteHead past the abandoned data so the next log doesn't program
//     into already-written sectors
//   - erase the buffer area so recoverFromBuffer() at next boot doesn't see a valid
//     uncommitted preamble and try to reconstruct this log
void flashfsLogAbortLog(void)
{
    if (!active.isOpen) return;

    // Match flashfsLogEndLog: drain pending bytes (which may need to wrap) BEFORE
    // disabling ring wrap — clearing the ring first would push wrap-needing bytes down
    // the linear-overflow drop path. Use the wrapping flashfsLogFlushSync so the lapped
    // marker is persisted before the drain — covers the narrow power-loss window before
    // markBufferCommitted runs, where a gap-scan recovery would otherwise synthesize a
    // phantom log with stale (pre-lap) dataStart.
    flashfsLogFlushSync();
    flashfsClearRing();

    // If the writer made it past the header phase, an uncommitted preamble exists at
    // bufferAreaStart. Mark it committed BEFORE erasing the buffer area: a power loss
    // in the window between the (synchronous) preamble update and the first sector
    // erase would otherwise leave recoverFromBuffer() with a valid uncommitted
    // preamble and no trailer, sending it down the gap-scan path to synthesize a
    // phantom log for the session the caller explicitly aborted. With the commit bit
    // flipped, recoverFromBuffer takes its committed-fast-path (just erase the buffer
    // area, no reconstruction) — the correct semantics for an abort.
    if (!active.headerPhase) {
        markBufferCommitted();
    }

    dataWriteHead = alignUpToSector(active.dataWriteHead);
    if (dataWriteHead >= geom.dataSectionEnd) dataWriteHead = 0;

    eraseBufferArea();

    memset(&active, 0, sizeof(active));
}

#endif // USE_BLACKBOX_RING_LOG

#endif // USE_FLASHFS
