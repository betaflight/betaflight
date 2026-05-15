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

// In ring-log mode, sustained writes are interrupted by NOR erase windows
// during which page programs can't run; the RAM buffer absorbs the writer's
// output until the chip becomes ready again. Sizing rule:
//
//   buffer >= sustained_rate × typical_erase_window_ms
//
// Per-MCU tiering reflects (a) what PID-loop rate the target can actually run,
// (b) how much SRAM we can spare, and (c) which erase strategy makes sense.
//
//   H7        →  PID loop typically 8 kHz. Target P-frame rate 4 kHz (1/2
//                divider). To sustain 4 kHz × 40 B = 160 KB/s on a typical
//                W25Q256-class NOR, the chip-side bandwidth has to come from
//                64 KB BLOCK ERASE (~150 ms typical, but ~426 KB/s erase
//                refill rate vs ~67 KB/s on a 4 KB sub-sector). Hardware
//                testing on F722 + W25Q256 showed the chip's actual erase
//                windows reach ~190 ms (well within the 60-2000 ms datasheet
//                range, but above the 150 ms typical), so one outlier window
//                fills 160 KB/s × 200 ms ≈ 32 KB. 48 KB gives ~300 ms outlier
//                headroom for end-of-life / hot chips. H7 has the SRAM
//                headroom for that. Use FLASHFS_RING_USE_BLOCK_ERASE.
//
//   H7 + EXST →  H7 External-Storage builds (USE_EXST — bootloader places
//                firmware in CODE_RAM at a fixed address) have only 64 KB of
//                DMA-able RAM at 0x24000000 for all .dmaram_* + .DMA_RAM
//                sections combined. The 48 KB non-EXST H7 buffer overflows
//                this by ~17 KB (asyncfatfs cache and other DMA buffers
//                already take ~33 KB). Fall back to the F7 tier (24 KB
//                / 2 kHz) which fits. SPRACINGH7EXTREME is the main target.
//
//   F7        →  Same PID-loop rate as H7 but DTCM is much tighter (64 KB on
//                F722; the worst-case F7 part) — the H7 buffer fails to link
//                on F722. Cap at 2 kHz so the buffer-vs-erase math shrinks:
//
//                  80 KB/s × 300 ms outlier ≈ 24 KB
//
//                24 KB linked on F722 in earlier tests and gives ~300 ms
//                outlier headroom on top of the ~150 ms typical block-erase
//                window — drop-free at 2 kHz on W25Q256-class NOR even
//                under chip-end-of-life timing. Stay on block erase:
//                sub-sector can't sustain 2 kHz on slower NOR (W25Q256
//                sub-sector ~60 ms typical → ~60 KB/s refill, less than
//                the 80 KB/s writer rate).
//
//   F4 / G4   →  PID loop typically 4 kHz. Target P-frame rate 2 kHz (1/2
//                divider). At 2 kHz × 40 B = 80 KB/s we can stay on the 4 KB
//                sub-sector erase (~60 ms typical, fills ~5 KB of buffer at
//                cap) so we keep the buffer at 8 KB. This minimises DTCM
//                pressure on F4 boards where it's tightest.
//
//   NAND      →  Block erase ~2 ms; per-erase buffer fill is < 1 KB at any
//                cap we'd configure. Driver advertises 8 kHz and
//                flashfsGetMaxSustainedLogRateHz bypasses the MCU cap so
//                fast NAND chips run at full PID-loop rate where the MCU
//                can sustain it.
//
// Linear mode never sees mid-flight erases so its 128-byte buffer is sufficient.
// The bump only applies on targets compiling in ring mode.
#ifdef USE_BLACKBOX_RING_LOG
#if defined(STM32H7) && !defined(USE_EXST)
#define FLASHFS_WRITE_BUFFER_SIZE       49152    // 48 KB
#define FLASHFS_RING_USE_BLOCK_ERASE    1        // ring-mode pool refill uses 64 KB block erase
#define FLASHFS_RING_MCU_CAP_HZ         4000     // H7 NOR cap (NAND bypasses, see flashfs.c)
#elif defined(STM32H7) && defined(USE_EXST)
// H7 EXST (External Storage) builds — bootloader places firmware in CODE_RAM at
// a fixed address, leaving only 64 KB of DMA-able RAM at 0x24000000 to hold
// .dmaram_data + .dmaram_bss + .DMA_RAM + .DMA_RW_AXI combined. The 48 KB
// non-EXST H7 buffer overflows this region by ~17 KB (asyncfatfs cache and
// other DMA buffers take ~33 KB of the budget already). Fall back to the F7
// tier (24 KB / 2 kHz) which we've verified fits inside the constrained EXST
// RAM region. SPRACINGH7EXTREME is the main target affected.
#define FLASHFS_WRITE_BUFFER_SIZE       24576    // 24 KB
#define FLASHFS_RING_USE_BLOCK_ERASE    1        // block erase, same as non-EXST H7
#define FLASHFS_RING_MCU_CAP_HZ         2000     // 2 kHz — matches the F7 tier's buffer-vs-erase math
#elif defined(STM32F7)
#define FLASHFS_WRITE_BUFFER_SIZE       24576    // 24 KB (32 KB fails to link on F722's 64 KB DTCM)
#define FLASHFS_RING_USE_BLOCK_ERASE    1        // block erase still preferred — sub-sector can't sustain 2 kHz on slow NOR
#define FLASHFS_RING_MCU_CAP_HZ         2000     // F7 NOR cap — drop-free at 2 kHz with 300 ms erase outlier headroom
#elif defined(STM32F4) || defined(STM32G4)
#define FLASHFS_WRITE_BUFFER_SIZE       8192     // 8 KB
// FLASHFS_RING_USE_BLOCK_ERASE intentionally not defined — sub-sector erase
// keeps the per-erase buffer fill small, which matches the 8 KB sizing.
#define FLASHFS_RING_MCU_CAP_HZ         2000     // F4/G4 NOR cap
#else
#define FLASHFS_WRITE_BUFFER_SIZE       8192     // 8 KB (conservative for unknown MCU)
#define FLASHFS_RING_MCU_CAP_HZ         1000
#endif
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

