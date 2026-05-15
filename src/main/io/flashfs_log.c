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

// Cache for hot pollers (MSP_BLACKBOX_CONFIG, OSD storage gauge, etc). The probe
// is O(partition / sectorSize) so calling it on every MSP request would saturate
// the SPI bus when the configurator is connected (~10-50 Hz polling × ~14 KB of
// SPI reads each = up to 700 KB/s of pure metadata traffic). On-flash format
// only changes at well-defined points: boot detection, log close, full erase.
// Cache the result and invalidate from those points.
static flashfsFlashFormat_e cachedFlashFormat = FLASHFS_FLASH_FORMAT_UNKNOWN;
static bool cachedFlashFormatValid = false;

void flashfsLogInvalidateCachedFormat(void)
{
    cachedFlashFormatValid = false;
}

#ifdef USE_BLACKBOX_RING_LOG
// Strict ring-format detection used on ring-enabled builds. Defined inside the
// USE_BLACKBOX_RING_LOG block below; forward-declared here so the always-compiled
// flashfsLogDetectFormatFromFlash() can call it. Returns RING on a validated
// signature, UNKNOWN otherwise.
static flashfsFlashFormat_e detectRingFormatStrict(void);
#endif

flashfsFlashFormat_e flashfsLogGetCachedFormat(void)
{
    if (!cachedFlashFormatValid) {
        // Don't populate the cache while a full-erase is in flight.
        // flashfsLogEraseAll() invalidates the cache when the erase STARTS, but
        // the erase then runs asynchronously for seconds. A call landing in
        // that window would probe the chip mid-erase and cache whatever
        // transient bytes the chip happens to return (partially-erased data,
        // 0xFF from already-erased sectors, etc.) — a wrong "format" verdict
        // that survives until the next explicit invalidation. Return UNKNOWN
        // and leave the cache invalid; the next call after erase completes
        // will probe a quiescent chip and cache the correct value.
        if (flashfsIsEraseInProgress()) {
            return FLASHFS_FLASH_FORMAT_UNKNOWN;
        }
        cachedFlashFormat = flashfsLogDetectFormatFromFlash();
        cachedFlashFormatValid = true;
    }
    return cachedFlashFormat;
}

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

#ifdef USE_BLACKBOX_RING_LOG
    // Ring-enabled build: route through full structural validators (magic +
    // bounds + CRC). A lone 4-byte magic match by chance must not classify as
    // RING and bypass the writer's erase gate. The strict path is defined inside
    // the ring-log block below; it returns RING on a validated signature,
    // UNKNOWN otherwise (and we fall through to the EMPTY/UNKNOWN logic).
    // computeGeometry() inside the strict path handles partition/buffer-area
    // sizing, so we don't need local copies here.
    {
        const flashfsFlashFormat_e ringResult = detectRingFormatStrict();
        if (ringResult == FLASHFS_FLASH_FORMAT_RING) {
            return FLASHFS_FLASH_FORMAT_RING;
        }
    }
#else
    // Linear-only build: magic-only is acceptable because the linear writer
    // can't act on a RING classification, and ring-mode MSC enumeration is
    // also USE_BLACKBOX_RING_LOG-gated. The classification is still useful as
    // a downgrade-warning signal to the configurator via MSP.
    const uint32_t partitionSize = flashfsGetSize();
    const uint32_t bufferAreaSize = HDR_BUFFER_SECTORS * fg->sectorSize;
    if (bufferAreaSize >= partitionSize) return FLASHFS_FLASH_FORMAT_UNKNOWN;
    const uint32_t bufferAreaStart = partitionSize - bufferAreaSize;
    {
        uint32_t magic = 0;
        if (flashfsReadAbs(bufferAreaStart, (uint8_t *)&magic, sizeof(magic)) == (int)sizeof(magic)
                && magic == BUFFER_PREAMBLE_MAGIC) {
            return FLASHFS_FLASH_FORMAT_RING;
        }
    }
    for (uint32_t addr = 0; addr + sizeof(uint32_t) <= bufferAreaStart; addr += fg->sectorSize) {
        uint32_t magic = 0;
        if (flashfsReadAbs(addr, (uint8_t *)&magic, sizeof(magic)) == (int)sizeof(magic)
                && magic == LOG_TRAILER_MAGIC) {
            return FLASHFS_FLASH_FORMAT_RING;
        }
    }
#endif

    // No ring signature found anywhere. EMPTY requires the WHOLE first sector
    // to be erased — the 16-byte probe at the top isn't enough on its own. A
    // chip with non-erased data later in sector 0 (e.g. a half-completed erase
    // that started from byte 0, or junk written by another use that didn't
    // touch the first 16 bytes) would otherwise be misclassified as EMPTY and
    // the linear writer would happily overwrite it. If anything later in
    // sector 0 isn't 0xFF, fall through to UNKNOWN.
    if (offset0Erased) {
        uint8_t scan[64];
        for (uint32_t addr = sizeof(probe); addr < fg->sectorSize; addr += sizeof(scan)) {
            const uint32_t remaining = fg->sectorSize - addr;
            const uint32_t chunk = remaining < sizeof(scan) ? remaining : sizeof(scan);
            if (flashfsReadAbs(addr, scan, chunk) != (int)chunk) {
                return FLASHFS_FLASH_FORMAT_UNKNOWN;
            }
            for (uint32_t i = 0; i < chunk; i++) {
                if (scan[i] != 0xFF) {
                    return FLASHFS_FLASH_FORMAT_UNKNOWN;
                }
            }
        }
        return FLASHFS_FLASH_FORMAT_EMPTY;
    }
    return FLASHFS_FLASH_FORMAT_UNKNOWN;
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
//
// IMMUTABLE after the single write in flashfsLogFinishHeader. Lap and commit
// markers no longer live inside this struct — they're stored as single-byte
// programs in distinct pages of the state sector (see HDR_BUFFER_STATE_*
// constants below). This keeps the preamble compatible with NAND chips that
// have on-chip ECC and forbid multiple programs to the same page region: each
// marker becomes a fresh 1-byte program in its own page, never overlapping
// previously programmed bytes.
typedef struct {
    uint32_t magic;            // BUFFER_PREAMBLE_MAGIC
    uint16_t crc;              // CRC-16 over (logId, dataStart, headerLength, headerText)
    uint16_t pad;              // unused (kept for 4-byte alignment of trailing fields)
    uint32_t logId;
    uint32_t dataStart;        // physical offset in data ring where this log started
    uint32_t headerLength;     // bytes of header text immediately following this preamble
} __attribute__((packed)) flashfsBufferPreamble_t;

STATIC_ASSERT(sizeof(flashfsBufferPreamble_t) == 20, buffer_preamble_size);

// Buffer area layout. HDR_BUFFER_SECTORS total sectors:
//   Sectors [0 .. HDR_BUFFER_STATE_SECTOR-1]: preamble (offset 0, 20 bytes) +
//       header text (immediately after).
//   Sector HDR_BUFFER_STATE_SECTOR (last): state sector. Holds the lap and
//       commit markers as single bytes in distinct pages, so each becomes a
//       fresh 1-byte program in its own NAND page (no in-place rewrites, no
//       same-page partial-program conflicts that NAND ECC can't tolerate).
//       Rest of the sector is unused — the cost is one sector per session
//       (~0.03% of a 128 MB chip; ~0.1% of a 16 MB chip) for portability.
#define HDR_BUFFER_STATE_SECTOR (HDR_BUFFER_SECTORS - 1)
#define HDR_BUFFER_HEADER_SECTORS (HDR_BUFFER_SECTORS - 1)

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
    bool             isOpen;
    bool             headerPhase;
    bool             everWrapped;         // dataWriteHead has crossed dataSectionEnd at least once this session
    bool             lapped;              // dataWriteHead has overtaken dataStart (writer is overwriting this session's earliest bytes)
    volatile bool    lappedPersisted;     // markBufferLapped() has flipped the preamble's lapped marker for this session.
                                          // volatile: the async path's DMA TC IRQ callback writes this from IRQ context.
    uint32_t         logId;
    uint32_t         dataStart;
    uint32_t         dataWriteHead;       // mirrors module-level dataWriteHead during the flight
    uint32_t         bufferHeaderOffset;  // bytes written into header buffer so far
} active;

// State machine for the async lap-marker DMA write:
//   1. persistLappedMarkerIfReady() kicks off flashPageProgram (sets InFlight=true).
//   2. DMA TC IRQ fires when the SPI transfer completes (clears InFlight,
//      sets DmaDone=true). On NAND this means "byte loaded into the page
//      register" — PROGRAM_EXECUTE has NOT yet run, so the marker is not
//      yet durable on flash. On NOR it means "byte programmed" — durable.
//   3. Next non-IRQ tick observes DmaDone=true, calls flashFlush() (no-op on
//      NOR; issues PROGRAM_EXECUTE on NAND), then sets lappedPersisted=true.
// All three flags written from IRQ context → volatile. The data payload (a
// single 0x00 byte) is module-scope below alongside the async helper.
static volatile bool lapMarkerInFlight = false;
static volatile bool lapMarkerDmaDone = false;

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
    // The marker layout puts commit at state-sector page 0 and lap at state-sector
    // page 1. That requires pageSize > 0 AND pageSize < sectorSize so the lap
    // marker stays inside the state sector. On any chip whose pageSize equals (or
    // exceeds) sectorSize — exotic and unsupported here — the marker offsets would
    // collide with the next region. Leave geom zeroed in that case so flashfsLogInit
    // refuses to mark itself safe-for-logging and the writer falls back to LINEAR.
    if (fg->pageSize == 0 || fg->pageSize >= fg->sectorSize) return;
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

// Forward decl: readBytes is a thin wrapper around flashfsReadAbs, defined below
// so it lives next to the other low-level read helpers. The marker helpers that
// follow need it before that point.
static int readBytes(uint32_t addr, uint8_t *dest, uint32_t length);

static void writeBytesSync(uint32_t addr, const uint8_t *data, uint32_t length)
{
    flashfsFlushSync();
    flashfsSeekAbs(addr);
    flashfsWrite(data, length, true);
    flashfsFlushSync();
}

// Absolute address of the commit marker (1 byte). Lives in page 0 of the state
// sector. Recovery reads this byte; anything != 0xFF means "log was committed".
static uint32_t commitMarkerAddr(void)
{
    return geom.bufferAreaStart + HDR_BUFFER_STATE_SECTOR * geom.sectorSize;
}

// Absolute address of the lap marker (1 byte). Lives in page 1 of the state
// sector — distinct NAND page from the commit marker so each marker is a fresh
// 1-byte program in its own page register, no partial-program collisions.
static uint32_t lappedMarkerAddr(void)
{
    return geom.bufferAreaStart + HDR_BUFFER_STATE_SECTOR * geom.sectorSize + geom.pageSize;
}

// Read one marker byte. Returns true iff the byte has been cleared (any value
// other than 0xFF). Returns false on read failure (recovery treats this as
// "marker not set" — the conservative redo-close path is idempotent).
static bool readMarkerSet(uint32_t addr)
{
    uint8_t b = 0xFF;
    if (readBytes(addr, &b, 1) != 1) return false;
    return b != 0xFF;
}

// Force a synchronous PROGRAM_EXECUTE so a just-loaded partial-page write
// becomes durable on flash. NAND drivers (W25N, MT29F) only auto-execute when
// the page register fills to a page boundary; sub-page writes (like our 1-byte
// markers) sit in the register until something else triggers execution. NOR's
// flashFlush is a no-op.
//
// Without this, a power loss between the marker write and the next non-marker
// flash operation could lose the marker — recovery would then misclassify the
// log state (e.g. "not lapped" when it was, exposing pre-lap dataStart bytes
// that the writer already overwrote).
static void flushMarkerWrite(void)
{
    flashFlush();
}

// Set a marker by writing a single 0x00 byte at the marker offset, then forcing
// a PROGRAM_EXECUTE so the write lands on flash before we return. Works on NOR
// (writes are atomic at any granularity) and on NAND (1 byte per page, separate
// pages → no ECC conflicts).
static void setMarkerSync(uint32_t addr)
{
    static const uint8_t zero = 0;
    writeBytesSync(addr, &zero, 1);
    flushMarkerWrite();
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

// Forward declaration: opportunistic lap marker persist, called from eraseTick.
// Defined further down with its sync sibling near the flush helpers.
static void persistLappedMarkerIfReady(void);

// Run the async-erase progress check + new-erase trigger. Cheap: a register read for
// flashIsReady() and at most one flashEraseSector() call (which itself just sends the
// command and returns; the chip then erases for ~30-50 ms in its own background while
// we keep going).
//
// Called from the data write hot path. NEVER blocks. The opportunistic
// lap-marker persist below kicks off via DMA and returns immediately —
// hot-path cost is ~15-20 µs (one synchronous preamble read + DMA setup),
// once per session. The actual page program runs in the chip's background
// while the FC loop continues.
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

    // Squeeze the lap marker write in between page programs. eraseTick fires
    // per data byte (~640 kHz at 8 kHz pidloop × 80 B/frame), giving us many
    // observation points per page-program cycle — we catch the brief idle
    // windows that the ~250 Hz async-flush polling misses entirely on busy
    // chips. persistLappedMarkerIfReady is itself gated on flashfsIsIdle()
    // (buffer empty AND chip ready) so we only fire when the SPI bus is free.
    // The write itself goes through markBufferLappedAsync which uses DMA +
    // callback — hot-path cost is ~15-20 µs (preamble read + DMA setup),
    // not the ~50-200 µs the sync path would block for. The chip's internal
    // page program runs in background while the FC loop continues.
    persistLappedMarkerIfReady();

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
    if (out->headerLength == 0
            || out->headerLength > HDR_BUFFER_HEADER_SECTORS * geom.sectorSize - sizeof(flashfsBufferPreamble_t)) {
        return false;
    }
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

static bool bufferIsCommitted(void)
{
    // Commit marker is a single byte in page 0 of the state sector. Initial
    // value 0xFF (erased) means "not committed yet"; anything else means a
    // commit was attempted (possibly partial-write on power loss — recovery's
    // "any non-0xFF means committed" check is robust either way).
    return readMarkerSet(commitMarkerAddr());
}

// Write the preamble at buffer offset 0. Called at the end of the header phase.
// The header bytes are already on flash (they were streamed there during the
// header phase) — we read them back in chunks to compute the CRC, avoiding a
// large RAM scratch buffer. Returns false on a readBytes failure during CRC
// computation; the caller must NOT proceed into the data phase in that case
// (no preamble on flash → recovery couldn't find the log).
//
// The preamble is written ONCE and is immutable for the lifetime of the session.
// Lap and commit signals are recorded separately as single-byte programs in the
// state sector (see commitMarkerAddr / lappedMarkerAddr) so NAND chips with
// on-chip ECC stay happy — no overlapping partial programs of the same page.
static bool writeBufferPreambleStreaming(uint32_t logId, uint32_t dataStart, uint32_t headerLength)
{
    flashfsBufferPreamble_t p = {
        .magic = BUFFER_PREAMBLE_MAGIC,
        .pad = 0xFFFF,         // erased-flash value; ignored by recovery
        .logId = logId,
        .dataStart = dataStart,
        .headerLength = headerLength,
    };
    uint16_t crc = 0;
    crc = crc16_ccitt_update(crc, &p.logId, sizeof(p.logId));
    crc = crc16_ccitt_update(crc, &p.dataStart, sizeof(p.dataStart));
    crc = crc16_ccitt_update(crc, &p.headerLength, sizeof(p.headerLength));
    // Stream the header bytes back from the buffer area to extend the CRC. Each
    // readBytes call is checked: a short read on this path would mix stale
    // chunkBuf bytes into the CRC, producing a preamble whose stored CRC doesn't
    // match the on-flash header. Recovery would then reject the preamble — safe
    // failure mode, but explicit early-return makes the abort intent obvious.
    {
        uint32_t addr = geom.bufferAreaStart + sizeof(p);
        uint32_t remaining = headerLength;
        while (remaining > 0) {
            uint32_t chunk = MIN(remaining, (uint32_t)CHUNK_SIZE);
            if (readBytes(addr, chunkBuf, chunk) != (int)chunk) {
                return false; // abort — no preamble written, caller must not enter data phase
            }
            crc = crc16_ccitt_update(crc, chunkBuf, chunk);
            addr += chunk;
            remaining -= chunk;
        }
    }
    p.crc = crc;
    writeBytesSync(geom.bufferAreaStart, (const uint8_t *)&p, sizeof(p));
    return true;
}

// Set the commit marker. One-byte program at a dedicated page in the state
// sector, followed by an explicit flush so it's durable on flash before the
// caller proceeds (typically into eraseBufferArea — which would otherwise wipe
// the marker if it were still sitting unflushed in the NAND page register).
static void markBufferCommitted(void)
{
    setMarkerSync(commitMarkerAddr());
}

// Set the lap marker. One-byte program at a different page in the state
// sector from the commit marker, so the two markers never share a NAND page.
// Returns true iff the marker is now (or was already) durable. Returns false
// only on read-back failure; the caller leaves lappedPersisted=false so the
// next flush retries.
static bool markBufferLapped(void)
{
    if (readMarkerSet(lappedMarkerAddr())) {
        return true; // already lapped (or partially programmed — recovery treats either as lapped)
    }
    setMarkerSync(lappedMarkerAddr());
    if (active.isOpen && !active.headerPhase) {
        // setMarkerSync routed through flashfs (which moved tailAddress to the
        // marker offset). Seek back so the writer's next data byte lands at
        // active.dataWriteHead, not on top of the marker sector.
        flashfsSeekAbs(active.dataWriteHead);
    }
    // Verify the marker actually landed before signalling durability. A
    // failed read-back means the write didn't take (or we can't read it
    // back), so leave the caller's lappedPersisted=false and let the next
    // tick retry.
    return readMarkerSet(lappedMarkerAddr());
}

// DMA TC IRQ callback for the async lap-marker write. Runs in interrupt context;
// touches only volatile flags. Signals "SPI transfer complete" — on NOR this
// also means the byte is durable, but on NAND the byte is in the page register
// and the next polling tick must call flashFlush() to issue PROGRAM_EXECUTE
// before we can claim durability. lappedPersisted is therefore deliberately NOT
// set here; the next persistLappedMarkerIfReady (or persistLappedMarkerBlocking)
// observes DmaDone=true, forces the flush, and only then sets persisted=true.
static void lapMarkerWriteCallback(uintptr_t arg)
{
    (void)arg;
    lapMarkerDmaDone = true;
    lapMarkerInFlight = false;
}

// Async variant of markBufferLapped used by the hot-path persist trigger
// (eraseTick → persistLappedMarkerIfReady). Kicks off the marker write via DMA
// and returns immediately — the CPU is free during the ~50-200 µs page program,
// so the FC loop sees ~15-20 µs hot-path cost (DMA setup) instead of the
// ~50-200 µs the sync path would block for.
//
// One-byte program at lappedMarkerAddr(). On NAND this loads the byte into the
// page register; the actual PROGRAM_EXECUTE that makes it durable is triggered
// by the W25N driver on the next pageProgramBegin to a different address. Data
// ring writes at ~640 kHz (one per FC loop iteration × ~80 B/frame) provide
// that trigger within ~125 µs of the lap event. The narrow window where a
// power loss between the DMA-load callback and the next data write could lose
// the marker is acceptable: recovery's redo path is idempotent, and the
// trailer (written at clean disarm) carries an independent dataStart that
// doesn't rely on the marker.
//
// Caller MUST have verified flashfsIsIdle() so the SPI bus is free and the
// chip is ready to accept commands.
//
// Returns true on successful kick-off (or "already lapped, nothing to do");
// false on a hardware read failure — caller leaves lappedPersisted=false so
// the next tick retries.
static uint8_t lapMarkerZeroByte = 0;
static bool markBufferLappedAsync(void)
{
    // flashReadBytes is the low-level driver call (bypasses flashfs's buffer);
    // since the marker is only ever written via flash* primitives (no flashfs
    // buffering), the on-chip byte is authoritative.
    uint8_t cur = 0xFF;
    if (flashReadBytes(lappedMarkerAddr(), &cur, 1) != 1) {
        return false;
    }
    if (cur != 0xFF) {
        // Already lapped (possibly partially programmed from an interrupted
        // previous attempt — recovery treats any non-0xFF as lapped).
        active.lappedPersisted = true;
        return true;
    }
    // The data buffer (single byte at module scope) must outlive this call
    // because flashPageProgram is async — DMA will continue transferring after
    // we return.
    lapMarkerInFlight = true;
    flashPageProgram(lappedMarkerAddr(),
                     &lapMarkerZeroByte,
                     1,
                     lapMarkerWriteCallback);
    return true;
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
    // fine and keeps code minimal. The insertion sort is stable, so entries with the
    // same logId stay in input (= sector-scan) order — required for the dedup pass
    // below to identify the highest-addressed survivor of each duplicate group.
    for (uint32_t i = 1; i < logTableCount; i++) {
        flashfsLogInfo_t key = logTable[i];
        int32_t j = (int32_t)i - 1;
        while (j >= 0 && logTable[j].logId > key.logId) {
            logTable[j + 1] = logTable[j];
            j--;
        }
        logTable[j + 1] = key;
    }

    // Dedupe by logId. More than one valid trailer can exist for the same session if
    // a close attempt commits its trailer but the header copy is interrupted (power
    // loss mid-copy, hardware hiccup), and a subsequent boot's recovery cannot land
    // the rewrite on the same sector and instead writes the replacement trailer
    // elsewhere in the ring. Without dedup, both trailers surface as separate phantom
    // log entries in MSC — the older one pointing at the partially-copied header,
    // confusing decoders (and the user) with a duplicate "log 1" / "log 2" pair from
    // a single flight.
    //
    // Preference rule: keep the highest physical trailer address. Recovery always
    // writes the replacement at-or-past the original (the scan in recoverFromBuffer
    // walks forward from pre.dataStart and writes at the first matching trailer or
    // erased gap it finds), so a higher address means "more recent attempt", which
    // is the only attempt with a complete header copy.
    //
    // Sector-granularity caveat: this only catches duplicates at distinct sector
    // boundaries. Two trailers in the same sector at different page offsets aren't
    // both visible to the scan above (it steps by sectorSize), so the dedup pass
    // can't help with that case — it falls to writer-side correctness (see
    // flashfsWrite / writeBytesSync).
    {
        uint32_t kept = 0;
        for (uint32_t i = 0; i < logTableCount; i++) {
            const bool nextHasSameLogId = (i + 1 < logTableCount)
                    && (logTable[i + 1].logId == logTable[i].logId);
            if (nextHasSameLogId) continue;  // a higher-addressed copy follows; skip this one
            if (kept != i) logTable[kept] = logTable[i];
            kept++;
        }
        logTableCount = kept;
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
// flash offsets within the partition. Returns true on full success, false if any
// readBytes call returned short — the caller's commit step is then skipped so a
// hardware read failure mid-copy doesn't get committed as durable data.
static bool copyChunkedSync(uint32_t srcAddr, uint32_t dstAddr, uint32_t length)
{
    while (length > 0) {
        uint32_t chunk = MIN(length, (uint32_t)CHUNK_SIZE);
        if (readBytes(srcAddr, chunkBuf, chunk) != (int)chunk) {
            return false;
        }
        writeBytesSync(dstAddr, chunkBuf, chunk);
        srcAddr += chunk;
        dstAddr += chunk;
        length -= chunk;
    }
    return true;
}

// Read and CRC-validate the buffer preamble using streaming reads so we don't need
// a large RAM scratch buffer.
static bool readAndValidateBufferPreamble(flashfsBufferPreamble_t *out, uint32_t *outHeaderLen)
{
    if (readBytes(geom.bufferAreaStart, (uint8_t *)out, sizeof(*out)) != (int)sizeof(*out)) {
        return false;
    }
    if (out->magic != BUFFER_PREAMBLE_MAGIC) return false;

    // Cap header at the usable header span (state sector reserved at the end).
    const uint32_t maxHeader = HDR_BUFFER_HEADER_SECTORS * geom.sectorSize - sizeof(*out);
    if (out->headerLength == 0 || out->headerLength > maxHeader) return false;

    uint16_t crc = 0;
    crc = crc16_ccitt_update(crc, &out->logId, sizeof(out->logId));
    crc = crc16_ccitt_update(crc, &out->dataStart, sizeof(out->dataStart));
    crc = crc16_ccitt_update(crc, &out->headerLength, sizeof(out->headerLength));
    uint32_t addr = geom.bufferAreaStart + sizeof(*out);
    uint32_t remaining = out->headerLength;
    while (remaining > 0) {
        uint32_t chunk = MIN(remaining, (uint32_t)CHUNK_SIZE);
        if (readBytes(addr, chunkBuf, chunk) != (int)chunk) {
            // Reject the preamble rather than mixing stale chunkBuf into the
            // CRC. A short read here means we can't trust the on-flash header
            // bytes either, so recovery treats this preamble as invalid and
            // falls through to the no-recovery path.
            return false;
        }
        crc = crc16_ccitt_update(crc, chunkBuf, chunk);
        addr += chunk;
        remaining -= chunk;
    }
    if (crc != out->crc) return false;
    *outHeaderLen = out->headerLength;
    return true;
}

// Strict ring-format detection. Called from the always-compiled
// flashfsLogDetectFormatFromFlash() on ring-enabled builds. Routes through the
// same validators (magic + bounds + CRC) that recovery uses so a stray 4-byte
// magic match by chance can't classify as RING and bypass the writer's erase
// gate. Caller decides what to do with UNKNOWN (typically: fall through to
// EMPTY/UNKNOWN logic).
static flashfsFlashFormat_e detectRingFormatStrict(void)
{
    // Idempotent; flashfsLogInit() also calls this and the detector may be
    // invoked before init in the cached-format path.
    computeGeometry();
    if (geom.partitionSize == 0 || geom.sectorSize == 0 || geom.dataSectionEnd == 0) {
        return FLASHFS_FLASH_FORMAT_UNKNOWN;
    }

    // Buffer-area preamble: catches the in-flight ring state where the active
    // log finalised its header but no data landed in sector 0 before crash.
    flashfsBufferPreamble_t pre;
    uint32_t headerLen;
    if (readAndValidateBufferPreamble(&pre, &headerLen)) {
        return FLASHFS_FLASH_FORMAT_RING;
    }

    // Sector-aligned trailer scan. Trailers persist across clean closes and
    // across erase-pool wraps. readTrailerAt does magic + CRC + bounds, so an
    // unlucky bit pattern that matches LOG_TRAILER_MAGIC alone is rejected.
    for (uint32_t addr = 0;
            addr + sizeof(flashfsLogTrailer_t) <= geom.dataSectionEnd;
            addr += geom.sectorSize) {
        flashfsLogTrailer_t t;
        if (readTrailerAt(addr, &t)) {
            return FLASHFS_FLASH_FORMAT_RING;
        }
    }
    return FLASHFS_FLASH_FORMAT_UNKNOWN;
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

    if (bufferIsCommitted()) {
        eraseBufferArea();
        return;
    }
    const bool sessionLapped = readMarkerSet(lappedMarkerAddr());

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
    //
    // Defensive: pendingEraseAddr is already PENDING_ERASE_NONE on cold boot
    // (static initializer), and recoverFromBuffer() only runs from the boot path
    // in flashfsLogInit(). The reset is here so the function stays correct if
    // a future caller invokes recovery from a non-init context.
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
    // trailer + headerLen). The "lapped" signal lives in a single byte at
    // lappedMarkerAddr() (page 1 of the state sector), set on the first lap
    // event by markBufferLapped[Async]. readMarkerSet treats ANY non-0xFF value
    // as lapped — a power loss mid-program could leave the byte partially
    // written (e.g. 0xF0), and the conservative interpretation is "lap
    // attempted" so we don't expose stale pre-lap bytes via the original
    // dataStart.
    uint32_t recoveredDataStart;
    if (sessionLapped) {
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
    if (!copyChunkedSync(geom.bufferAreaStart + sizeof(flashfsBufferPreamble_t),
                         addr + sizeof(flashfsLogTrailer_t),
                         headerLen)) {
        // Same handling as flashfsLogEndLog's failure path: erase the trailer
        // sectors so buildLogTableByScan (running immediately after this in
        // flashfsLogInit) doesn't surface a phantom log entry pointing at
        // non-existent header bytes. Buffer stays uncommitted so the next boot
        // can fall back to gap-scan.
        //
        // Linear erase loop is safe (no wrap needed): bytesNeeded never spans
        // past geom.dataSectionEnd because wrappedToFit was set above when
        // addr + bytesNeeded would have spilled, and addr was reset to 0 in
        // that case — so [addr, addr+bytesNeeded) stays inside the data section.
        uint32_t eraseAddr = addr;
        const uint32_t eraseEnd = addr + bytesNeeded;
        while (eraseAddr < eraseEnd) {
            eraseSectorSync(eraseAddr);
            eraseAddr += geom.sectorSize;
        }
        return;
    }
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
    // Seed the format cache so the next caller (typically MSP_BLACKBOX_CONFIG or
    // the linear-mode safety gate) doesn't pay for a second full probe. The cache
    // is invalidated from the only paths that can change the on-flash format
    // (flashfsLogEraseAll, flashfsLogEndLog, preamble write), so it stays
    // authoritative for the life of the boot.
    cachedFlashFormat = fmt;
    cachedFlashFormatValid = true;
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

    // Reset flashfs's tailAddress to the post-init data-ring write head. flashfsInit
    // (which ran before us) seeded tailAddress via flashfsIdentifyStartOfFreeSpace() —
    // a linear-mode-only heuristic that scans for the first non-LINEAR_FORMAT_PROBE_TEXT
    // sector. On a ring-formatted chip that returns garbage (the data ring is full of
    // binary frames, not the linear preamble), and on top of that recoverFromBuffer
    // above may have left tailAddress in the buffer area (physical end of chip) from
    // its markBufferCommitted + eraseBufferArea. Either path leaves flashfsGetOffset()
    // reporting the wrong value, which makes the configurator see the chip as full and
    // save-to-file dump the whole 32 MB. Anchor tailAddress to dataWriteHead so usage
    // accounting reflects the actual data-ring extent. Skip for UNKNOWN/LINEAR formats
    // (writer is disabled there, so dataWriteHead is meaningless — leave the linear-mode
    // tailAddress flashfsInit set).
    if (moduleSafeForLogging) {
        flashfsSeekAbs(dataWriteHead);
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
    flashfsLogInvalidateCachedFormat();     // chip is now empty; next probe will return EMPTY
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
    // Each readBytes is checked: a short or failed read terminates the call so MSC
    // sees the actual byte count, not the requested-but-not-delivered length.
    if (offsetInLog < info->headerLength) {
        uint32_t fromHeader = MIN(length, info->headerLength - offsetInLog);
        const int got = readBytes(info->headerOffset + offsetInLog, dest, fromHeader);
        if (got <= 0) return total;
        total += got;
        if ((uint32_t)got < fromHeader) return total;
        dest += fromHeader;
        length -= fromHeader;
        offsetInLog += fromHeader;
    }

    // Data portion: virtual offsets [headerLength .. totalSize) → data section, may wrap.
    if (length > 0) {
        uint32_t dataOffset = offsetInLog - info->headerLength;
        if (info->dataEnd >= info->dataStart) {
            const int got = readBytes(info->dataStart + dataOffset, dest, length);
            if (got > 0) total += got;
        } else {
            uint32_t firstSeg = geom.dataSectionEnd - info->dataStart;
            if (dataOffset < firstSeg) {
                uint32_t fromFirst = MIN(length, firstSeg - dataOffset);
                const int got = readBytes(info->dataStart + dataOffset, dest, fromFirst);
                if (got <= 0) return total;
                total += got;
                if ((uint32_t)got < fromFirst) return total;
                dest += fromFirst;
                length -= fromFirst;
                dataOffset += fromFirst;
            }
            if (length > 0) {
                const int got = readBytes(dataOffset - firstSeg, dest, length);
                if (got > 0) total += got;
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

    // Reset the lap-marker async-write state so a stale DmaDone from a prior
    // session can't be observed by this session's first persistLappedMarkerIfReady
    // and incorrectly mark this new session as already-persisted. (active.* fields
    // are zeroed elsewhere; the module-scope volatiles aren't.)
    lapMarkerInFlight = false;
    lapMarkerDmaDone = false;

    // Erase the active-log header buffer (synchronous, ~120-400 ms for 4 sectors).
    eraseBufferArea();

    // Pre-erase the data-ring pool ahead of dataWriteHead. If a previous log close left
    // extra erased space (e.g. long header overflowed into more sectors than the pool),
    // eraseHead may already be ahead — only erase the remainder. Up to POOL_TARGET_SECTORS
    // sector erases × ~30-50 ms each = ~120-200 ms log-start latency in the cold case.

    // Reconcile any background erase pending from a previous flight: wait for the
    // chip to finish, then clear pendingEraseAddr. We don't compute the resulting
    // eraseHead here because the next block overwrites it unconditionally —
    // inheriting eraseHead from a prior flight is unsafe (any "is it close enough
    // to dataWriteHead?" heuristic risks misjudging a stale value and letting the
    // writer program non-erased sectors).
    if (pendingEraseAddr != PENDING_ERASE_NONE) {
        while (!flashIsReady());
        pendingEraseAddr = PENDING_ERASE_NONE;
    }

    // Always pin eraseHead to the writer position and let ensureErasedSpace fill
    // the pool from scratch. Costs ~POOL_TARGET_SECTORS extra erases per log start
    // (a few tens of ms on top of the buffer-area erase that already runs here)
    // — acceptable for safety.
    eraseHead = dataWriteHead;
    ensureErasedSpace(dataWriteHead, POOL_TARGET_SECTORS * geom.sectorSize);

    memset(&active, 0, sizeof(active));
    active.isOpen = true;
    active.headerPhase = true;
    active.logId = nextLogId++;
    active.dataStart = dataWriteHead;
    active.dataWriteHead = dataWriteHead;
    active.bufferHeaderOffset = 0;

    // Reset the diagnostic drop counter so flashfsLogGetDataDrops() reports
    // drops for THIS session only — otherwise the value bleeds across sessions
    // (the static is only zeroed at boot) and the counter becomes useless for
    // tuning the rate cap on a per-flight basis.
    bbDataDrops = 0;

    // Position flashfs's tail at the start of the buffer area's header-text region. From
    // here, header bytes go through the existing buffered async path (flashfsWriteByte)
    // — no per-byte sync flush.
    flashfsSeekAbs(geom.bufferAreaStart + sizeof(flashfsBufferPreamble_t));
    return true;
}

void flashfsLogWriteHeaderByte(uint8_t b)
{
    if (!active.isOpen || !active.headerPhase) return;

    uint32_t maxHdr = HDR_BUFFER_HEADER_SECTORS * geom.sectorSize - sizeof(flashfsBufferPreamble_t);
    if (active.bufferHeaderOffset >= maxHdr) {
        // Header overflow — drop the byte silently. Should never happen with the
        // header buffer span (~12 KB on smallest NOR sectorSize, hundreds of KB
        // on NAND); if it does, the resulting log will be missing some header lines.
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
    // chunked reads — no large RAM scratch. If the streaming readback fails (rare hardware
    // hiccup), abort the session: the preamble didn't make it onto flash, so a power loss
    // during the data phase would leave recovery with no preamble to find. Treat as if
    // BeginLog never happened — clear active state, do NOT advance to data phase, do NOT
    // enable ring wrap.
    if (!writeBufferPreambleStreaming(active.logId, active.dataStart, headerLen)) {
        memset(&active, 0, sizeof(active));
        return;
    }

    // Seed the format cache directly: we just wrote a ring preamble, so RING
    // is authoritative without re-probing. Invalidating here would force the
    // next flashfsLogGetCachedFormat() to flush + chunked-read while the log
    // is active.
    cachedFlashFormat = FLASHFS_FLASH_FORMAT_RING;
    cachedFlashFormatValid = true;

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
    //
    // Throttle to ~once per page write (32 bytes — 1/8 of a 256-byte page) rather than
    // every byte. Per-byte was originally chosen to maximise observation points for
    // the lap-marker persist, but at the high data rates the 4 kHz NOR cap unlocks
    // (~120 KB/s = 30k calls/sec at 30 B/frame), the cumulative SPI status-poll cost
    // inside flashIsReady() during sector-erase windows starts dominating FC CPU
    // (observed: 95% CPU spikes correlated with logging at 4 kHz P-frame rate).
    //
    // 32 B is the sweet spot: a sector-erase window is ~30-50 ms, during which ~3500-
    // 6000 bytes pass through the writer at peak rate. Firing eraseTick every 32 B
    // gives ~100-180 observation points per erase — far more than needed for both
    // erase-completion polling and lap-marker squeeze-in. SPI bus contention drops
    // 32×.
    static uint8_t eraseTickThrottle = 0;
    if ((++eraseTickThrottle & 0x1F) == 0) {
        eraseTick();
    }

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

// Diagnostic: bytes dropped on the data hot path because either the writer caught
// the erase frontier (pool depleted under worst-case erase outliers) or the RAM
// write buffer was full (chip mid-erase, flush couldn't drain). Monotonic per
// session. Useful for tuning the BLACKBOX_RING_MAX_FRAME_HZ cap and confirming
// the erase pool is keeping up on a given chip.
uint32_t flashfsLogGetDataDrops(void) { return bbDataDrops; }

// Persist the lap marker into the buffer preamble if the writer has lapped this
// session and the marker hasn't already been written. Two variants because the
// hot-path and disarm call sites have different real-time contracts:
//
//   - persistLappedMarkerIfReady: called from eraseTick (which fires per data
//     byte — ~640 kHz under sustained writes). Gates on flashfsIsIdle()
//     (buffer empty AND chip ready) AND uses markBufferLappedAsync — kicks off
//     the marker write via DMA and returns. Hot-path cost is ~15-20 µs
//     (synchronous preamble read + DMA setup), once per session. The actual
//     ~50-200 µs page program runs in the chip's background while the FC loop
//     continues; the DMA TC IRQ callback (lapMarkerWriteCallback) sets
//     active.lappedPersisted=true. flashfs's normal flow naturally waits
//     during the in-flight window because flashIsReady() returns false.
//
//     The per-byte polling rate is essential: it catches the brief idle
//     windows between page programs (each ~50-200 µs wide) that
//     lower-frequency polling would miss on a heavily-loaded chip. During the
//     marker write the writer keeps queuing bytes into the 16 KB RAM buffer
//     which absorbs the delay without dropping data. Worst case (chip
//     genuinely never idle for a sustained period): persist is deferred to
//     disarm via the blocking variant.
//
//     The sync path's writeBytesSync would be a hard real-time violation:
//     it starts with a flashfsFlushSync (could drain ~13 ms of buffered
//     pages) and ends with `while (!flashIsReady())` (could be mid-sector-
//     erase = up to 30-50 ms).
//
//   - persistLappedMarkerBlocking: called from flashfsLogFlushSync at disarm
//     (non-realtime). Here we MUST guarantee the marker lands on flash before
//     the session closes, because the close erases the buffer preamble that
//     mid-flight recovery would otherwise use. Blocking is acceptable here.
//
// Both variants honor markBufferLapped's bool return: on read-back failure the
// persisted flag stays false so a subsequent call retries.
static void persistLappedMarkerIfReady(void)
{
    // Stage 2: DMA completed in a prior tick. Force PROGRAM_EXECUTE so the
    // marker byte lands on physical flash (no-op on NOR; required on NAND
    // for sub-page partial programs that don't auto-execute), then mark the
    // lap durable. Hot-path cost: one SPI command (~10-20 µs) once per
    // session — comparable to other eraseTick operations.
    if (lapMarkerDmaDone) {
        flashFlush();
        active.lappedPersisted = true;
        lapMarkerDmaDone = false;
        return;
    }
    // Stage 1: first lap event of the session. Kick off async DMA write.
    // If markBufferLappedAsync returns false (read-back failure) the
    // in-flight flag stays clear and a subsequent tick retries.
    if (active.isOpen && active.lapped && !active.lappedPersisted
            && !lapMarkerInFlight && flashfsIsIdle()) {
        markBufferLappedAsync();
    }
}

static void persistLappedMarkerBlocking(void)
{
    // Drain any in-flight async marker write. The DMA TC IRQ callback
    // clears lapMarkerInFlight and sets lapMarkerDmaDone atomically from
    // the FC-loop's POV.
    while (lapMarkerInFlight) { /* spin until DMA TC IRQ fires */ }

    // If the prior async hop reached the DMA-done stage, force durability
    // here just like persistLappedMarkerIfReady would.
    if (lapMarkerDmaDone) {
        flashFlush();
        active.lappedPersisted = true;
        lapMarkerDmaDone = false;
    }

    // Sync fallback: the async path never fired (e.g. chip was busy every
    // tick) or failed (read-back error). At disarm we MUST guarantee
    // durability before the buffer-area erase wipes the preamble that
    // mid-flight recovery would otherwise use.
    if (active.isOpen && active.lapped && !active.lappedPersisted) {
        if (markBufferLapped()) {
            active.lappedPersisted = true;
        }
    }
}

bool flashfsLogFlushAsync(bool force)
{
    persistLappedMarkerIfReady();   // non-blocking; FC-loop safe
    return flashfsFlushAsync(force);
}

void flashfsLogFlushSync(void)
{
    persistLappedMarkerBlocking();  // disarm path; OK to block on a busy chip
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

    if (!copyChunkedSync(geom.bufferAreaStart + sizeof(flashfsBufferPreamble_t),
                         trailerAddr + sizeof(flashfsLogTrailer_t),
                         headerLen)) {
        // Source-read of the buffered header failed mid-copy. We must NOT leave
        // the just-written trailer dangling: between this disarm and the next
        // boot, a re-arm runs flashfsLogBeginLog → eraseBufferArea, wiping the
        // preamble that recoverFromBuffer would otherwise use to find and redo
        // this close. Without the preamble, the dangling trailer survives and
        // buildLogTableByScan accepts it next boot — exposing a phantom MSC log
        // pointing at non-existent header bytes.
        //
        // Erase the trailer + (partial) header sectors so neither path can see
        // a valid trailer. The buffer stays valid + uncommitted: if recovery
        // does run before re-arm (i.e. immediate reboot), it falls back to the
        // gap-scan path — and our just-erased sector is the guaranteed gap.
        uint32_t addr = trailerAddr;
        const uint32_t end = trailerAddr + bytesNeeded;
        while (addr < end) {
            eraseSectorSync(addr);
            addr += geom.sectorSize;
        }
        memset(&active, 0, sizeof(active));
        return;
    }

    // Mark committed in the buffer (so a power loss during the buffer erase below leaves
    // the recovery path with an unambiguous "already committed" signal), then erase.
    markBufferCommitted();
    eraseBufferArea();

    // Reset flashfs's tailAddress to the post-close data-ring write head. The
    // markBufferCommitted + eraseBufferArea sequence above leaves tailAddress
    // somewhere in the buffer area (physical end of the chip), because that's
    // where the last byte was programmed. flashfsGetOffset() would then return
    // chip-end-of-buffer-area, making MSP_DATAFLASH_SUMMARY / OSD storage gauge
    // / configurator save-to-file all think the chip is full — and dump all
    // 32 MB in the save case — even after a single 2 MB log session. Pointing
    // tailAddress at dataWriteHead makes flashfsGetOffset() report the actual
    // logical extent of log data on the chip. Safe to call at this point: the
    // buffer is empty (eraseBufferArea drained it), there are no pending
    // writes, and we've already flashfsClearRing'd above so the linear-mode
    // wrap config is inactive.
    flashfsSeekAbs(dataWriteHead);

    // Just wrote a fresh trailer. Cached format may go from EMPTY → RING — invalidate
    // so the next MSP poll re-probes (cheap; this is a non-realtime path).
    flashfsLogInvalidateCachedFormat();

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

    // Buffer-area preamble was just erased / committed-then-erased. Cached
    // format may go RING → EMPTY (or to LINEAR fallback) — invalidate so the
    // next probe re-scans.
    flashfsLogInvalidateCachedFormat();

    memset(&active, 0, sizeof(active));
}

#endif // USE_BLACKBOX_RING_LOG

#endif // USE_FLASHFS
