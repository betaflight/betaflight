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
 * flashfs_log: ring-buffer blackbox log storage on top of flashfs.
 *
 * On-flash layout (ring mode only — linear mode is bit-for-bit unchanged):
 *
 *   [ data ring ............................................ ][ active hdr buffer ]
 *     0                                                         partition - HDR_BUF
 *
 * - Data ring: holds each log's frame data followed by its trailing header.
 *
 *     ... [ data of log N ][ trailer ][ header text ] [ data of log N+1 ][ trailer ][ header text ] ...
 *                          ^ sector-aligned          ^ sector-aligned
 *
 *   Each log finds its end at the trailer that follows its data. The trailer is a small
 *   16-byte record with magic + CRC + header_length + data_start, sector-aligned so the
 *   MSC enumeration scan only has to read one offset per sector. The header text follows
 *   the trailer (header at the END of the log, as you specified).
 *
 * - Active header buffer: a small reserved region (default 4 sectors / 16 KB) that holds
 *   the in-flight log's header until commit. After clean log close the buffer is erased,
 *   so on boot:
 *       buffer empty   → no active log; nothing to recover
 *       buffer valid   → power was cut mid-flight; copy header to current data end
 *                        and erase buffer.
 *
 * MSC presents each ring-mode log as a virtual file whose contents are header || data,
 * preserving the on-host appearance of the legacy blackbox file format. The trailing
 * header is read from flash and prepended at MSC read time.
 *
 * IMPORTANT for review by hard-realtime experts: the data write hot path
 * (flashfsLogWriteDataByte) is just `flashfsWriteByte(b)` plus a wrap-bound check —
 * NO synchronous metadata writes during a flight. The cost per byte is the same as
 * linear mode plus one comparison.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Maximum number of distinct logs we expose at MSC mount. Each in-RAM entry costs ~24 B.
#define FLASHFS_LOG_MAX_LOGS 64

// Detected on-flash format. Determined by flashfsLogDetectFormat() at boot.
typedef enum {
    FLASHFS_FLASH_FORMAT_EMPTY   = 0,  // Erased; either mode may be used.
    FLASHFS_FLASH_FORMAT_LINEAR  = 1,  // Contains legacy/linear-mode logs.
    FLASHFS_FLASH_FORMAT_RING    = 2,  // Contains ring-mode trailers (this module's format).
    FLASHFS_FLASH_FORMAT_UNKNOWN = 3,  // Garbage / partially erased.
} flashfsFlashFormat_e;

// Per-log info, populated by scanning the data ring at boot, exposed to MSC/emfat.
typedef struct {
    uint32_t logId;
    uint32_t headerOffset;     // physical flash offset where header text begins
    uint32_t headerLength;
    uint32_t dataStart;        // physical flash offset where this log's data begins
    uint32_t dataEnd;          // physical flash offset where this log's data ends (= headerOffset - sizeof(flashfsLogTrailer_t), currently 20)
    uint32_t totalSize;        // headerLength + (dataEnd - dataStart) accounting for ring wrap
    bool     valid;
    bool     wasOpen;          // recovered from power loss (no clean trailer; header copied from buffer)
} flashfsLogInfo_t;

#ifdef USE_BLACKBOX_RING_LOG

// ---- Init / format ----
void flashfsLogInit(void);
flashfsFlashFormat_e flashfsLogDetectFormat(void);
void flashfsLogEraseAll(void);

// ---- Read API (MSC) ----
uint32_t flashfsLogGetLogCount(void);
const flashfsLogInfo_t* flashfsLogGetInfo(uint32_t index);
// Returns bytes read. Splices header || data, handling data ring-wrap.
int flashfsLogRead(uint32_t index, uint32_t offsetInLog, uint8_t *dest, uint32_t length);

// ---- Writer lifecycle ----
bool flashfsLogBeginLog(void);              // erase buffer, prep header phase
void flashfsLogWriteHeaderByte(uint8_t b);  // write into active header buffer
void flashfsLogFinishHeader(void);          // commit preamble; switch to data phase
void flashfsLogWriteDataByte(uint8_t b);    // write data byte (handles wrap)
void flashfsLogWriteData(const uint8_t *data, uint32_t length);
uint32_t flashfsLogGetWriteBufferFreeSpace(void);
bool flashfsLogFlushAsync(bool force);
void flashfsLogFlushSync(void);
void flashfsLogEndLog(void);                // copy buffered header to end-of-data; erase buffer
void flashfsLogAbortLog(void);              // discard active log: no trailer is written, orphaned data in the ring is left to be overwritten on the next wrap

bool flashfsLogIsActive(void);
bool flashfsLogIsHeaderPhase(void);
bool flashfsLogIsReady(void);
bool flashfsLogIsFull(void);

#else // !USE_BLACKBOX_RING_LOG: inline stubs so call sites compile away cleanly.

static inline void flashfsLogInit(void) {}
static inline flashfsFlashFormat_e flashfsLogDetectFormat(void) { return FLASHFS_FLASH_FORMAT_UNKNOWN; }
static inline void flashfsLogEraseAll(void) {}
static inline uint32_t flashfsLogGetLogCount(void) { return 0; }
static inline const flashfsLogInfo_t* flashfsLogGetInfo(uint32_t i) { (void)i; return 0; }
static inline int flashfsLogRead(uint32_t i, uint32_t o, uint8_t *d, uint32_t l)
{ (void)i; (void)o; (void)d; (void)l; return 0; }
static inline bool flashfsLogBeginLog(void) { return false; }
static inline void flashfsLogWriteHeaderByte(uint8_t b) { (void)b; }
static inline void flashfsLogFinishHeader(void) {}
static inline void flashfsLogWriteDataByte(uint8_t b) { (void)b; }
static inline void flashfsLogWriteData(const uint8_t *d, uint32_t l) { (void)d; (void)l; }
static inline uint32_t flashfsLogGetWriteBufferFreeSpace(void) { return 0; }
static inline bool flashfsLogFlushAsync(bool f) { (void)f; return true; }
static inline void flashfsLogFlushSync(void) {}
static inline void flashfsLogEndLog(void) {}
static inline void flashfsLogAbortLog(void) {}
static inline bool flashfsLogIsActive(void) { return false; }
static inline bool flashfsLogIsHeaderPhase(void) { return false; }
static inline bool flashfsLogIsReady(void) { return false; }
static inline bool flashfsLogIsFull(void) { return false; }

#endif // USE_BLACKBOX_RING_LOG
