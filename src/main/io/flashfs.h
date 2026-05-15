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

#pragma once

// In ring-log mode, sustained writes are interrupted by ~30-150 ms NOR erase
// windows during which page programs can't run; the RAM buffer absorbs the
// writer's output until the chip becomes ready again. Sizing rule:
//
//   buffer >= sustained_rate × typical_erase_window_ms
//
// The sustained rate itself is capped by the chip driver (maxSustainedLogRateHz)
// so that erase-refill bandwidth equals writer-byte bandwidth. At the cap, one
// typical erase window fills approximately:
//
//   m25p16 sub-sector, modern NOR: 60 KB/s × 60 ms  = 3.6 KB
//   m25p16 sub-sector, older NOR:  37 KB/s × 100 ms = 3.7 KB
//   w25q128fv block erase fallback: ~53 KB/s × 150 ms = 8 KB  (cap is buffer-bound)
//   NAND block erase:              320 KB/s ×   2 ms = 0.6 KB
//
// 8 KB covers every NOR case at its respective cap, with a small headroom for
// frame-size variability and brief erase-time outliers. Earlier the buffer was
// 16 KB based on a 200 KB/s × 50 ms calculation that assumed an UNCAPPED writer
// rate; the per-driver cap now binds the writer to actual chip bandwidth, so
// the 16 KB sizing carried 2× headroom we don't need (and 8 KB of static RAM
// is meaningful on F411 / G4-class targets).
//
// Linear mode never sees mid-flight erases so its 128-byte buffer is sufficient.
// The bump only applies on targets compiling in ring mode.
#ifdef USE_BLACKBOX_RING_LOG
#define FLASHFS_WRITE_BUFFER_SIZE 8192
#define FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN 256
#else
#define FLASHFS_WRITE_BUFFER_SIZE 128
#define FLASHFS_WRITE_BUFFER_AUTO_FLUSH_LEN 64
#endif

#define FLASHFS_WRITE_BUFFER_USABLE (FLASHFS_WRITE_BUFFER_SIZE - 1)

void flashfsEraseCompletely(void);
void flashfsEraseRange(uint32_t start, uint32_t end);

uint32_t flashfsGetSize(void);
uint32_t flashfsGetOffset(void);
uint32_t flashfsGetWriteBufferFreeSpace(void);
uint32_t flashfsGetWriteBufferSize(void);
int flashfsIdentifyStartOfFreeSpace(void);
struct flashGeometry_s;
const struct flashGeometry_s* flashfsGetGeometry(void);

// Maximum sustained ring-mode log frame rate the underlying chip can support
// without overrunning the RAM write buffer during the worst-case sector erase
// stall. Reads the driver-supplied geometry.maxSustainedLogRateHz; returns a
// conservative fallback (FLASHFS_DEFAULT_MAX_SUSTAINED_LOG_RATE_HZ) when the
// driver hasn't populated it or the chip isn't supported. Consumers cap their
// per-session frame rate at this value to avoid drops mid-flight.
#define FLASHFS_DEFAULT_MAX_SUSTAINED_LOG_RATE_HZ 1000
uint16_t flashfsGetMaxSustainedLogRateHz(void);

void flashfsSeekAbs(uint32_t offset);

// Ring-mode wrap. When enabled, sequential writes that started inside the configured
// ring [startAddress, endAddress) loop their post-write tail back to startAddress
// once it reaches endAddress, instead of advancing linearly past it. Default is
// disabled; linear-mode callers should never enable it. endAddress must be page-aligned.
void flashfsSetRing(uint32_t startAddress, uint32_t endAddress);
void flashfsClearRing(void);

void flashfsWriteByte(uint8_t byte);
void flashfsWrite(const uint8_t *data, unsigned int len, bool sync);

int flashfsReadAbs(uint32_t offset, uint8_t *data, unsigned int len);

bool flashfsFlushAsync(bool force);
void flashfsFlushSync(void);
void flashfsEraseAsync(void);

void flashfsClose(void);
void flashfsInit(void);
bool flashfsIsSupported(void);

bool flashfsIsReady(void);
// True iff the RAM write buffer is empty AND the chip is idle — i.e. a
// synchronous flash transaction would not block on a pending page program
// or in-flight sector erase. For real-time callers that need to perform a
// sync write only when the cost can be bounded to one page program.
bool flashfsIsIdle(void);
bool flashfsIsEOF(void);
bool flashfsIsEraseInProgress(void);

bool flashfsVerifyEntireFlash(void);

