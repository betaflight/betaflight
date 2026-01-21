/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)

#include "dshot_pico.h"

// Maximum time to wait for telemetry reception to complete
#define DSHOT_TELEMETRY_TIMEOUT 2000

// TODO use with TELEMETRY for cli report (or not)
FAST_DATA_ZERO_INIT dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;

// Integer clock divider: PIO runs at 75 MHz (150 MHz / 2) for clean timing
// This gives exactly 125 cycles per DShot bit and 100 cycles per telemetry bit
#define DSHOT_BIDIR_BIT_PERIOD 125

// Bidirectional DShot telemetry protocol:
// - FC sends inverted DShot frame (idle HIGH, 1 = LOW pulse, 0 = HIGH pulse)
// - ESC responds ~30us later with 21-bit GCR-encoded telemetry at 5/4x bitrate
// - Telemetry contains 12-bit eRPM data + 4-bit checksum
//
// This implementation uses edge detection with 5.5x oversampling. The PIO uses
// 'wait' instructions for precise edge synchronization, then samples 128 bits
// into 4 words via autopush. Edge positions are converted to GCR bits using
// run-length decoding (based on ArduPilot's proven algorithm).

bool dshot_program_bidir_init(PIO pio, uint sm, int offset, uint pin)
{
    bprintf("dshot_program_bidir_init on pin %d", pin);
#ifdef DSHOT_DEBUG_PIO
    pio_sm_config config = dshot_600_bidir_debug_program_get_default_config(offset);
#else
    pio_sm_config config = dshot_600_bidir_program_get_default_config(offset);
#endif

    sm_config_set_set_pins(&config, pin, 1);
    sm_config_set_in_pins(&config, pin);
    sm_config_set_jmp_pin(&config, pin);
    pio_gpio_init(pio, pin);
    bprintf("dshot bidir on pin %d done init PIO->gpio for pio", pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    gpio_set_pulls(pin, true, false);  // Pull up - idle HIGH for bidir telemetry

    sm_config_set_out_shift(&config, PIO_SHIFT_LEFT, PIO_NO_AUTO_PUSHPULL, 32);
    sm_config_set_in_shift(&config, PIO_SHIFT_LEFT, true, 32);  // Left shift, autopush at 32 bits

    float clocks_per_us = clock_get_hz(clk_sys) / 1000000;
#ifdef TEST_DSHOT_SLOW
    sm_config_set_clkdiv(&config, (1.0e4f + dshotGetPeriodTiming()) / DSHOT_BIDIR_BIT_PERIOD * clocks_per_us);
#else
    sm_config_set_clkdiv(&config, dshotGetPeriodTiming() / DSHOT_BIDIR_BIT_PERIOD * clocks_per_us);
#endif

    bool ok = PICO_OK == pio_sm_init(pio, sm, offset, &config);
    if (ok) {
        // Initialize OSR to all 1s (mov osr, ~null)
        pio_sm_exec(pio, sm, 0xa0eb);
    }
    return ok;
}

// GCR decode lookup table (maps 5-bit GCR to 4-bit nibble, -1 = invalid)
static const int8_t gcrDecodeLut[32] = {
    -1, -1, -1, -1, -1, -1, -1, -1,
    -1,  9, 10, 11, -1, 13, 14, 15,
    -1, -1,  2,  3, -1,  5,  6,  7,
    -1,  0,  8,  1, -1,  4, 12, -1
};

// Edge detection decoder for 5.5x oversampled telemetry
// Algorithm:
// 1. Scan samples to find edge transitions
// 2. Convert run lengths between edges to GCR bits
// 3. Decode GCR to 16-bit value and verify checksum
//
// Note: Edge positions are NOT normalized. The first edge position varies
// based on the GCR-encoded value (0 RPM starts at ~11 samples, real RPM
// values can start at ~5-6 samples). Normalizing would add phantom bits.

// Oversampling rate: 100 PIO cycles/bit ÷ 18 cycles/sample = 5.556x
// For run-length decoding: len = round(diff / samplesPerBit) = round(diff * samplesToBits)
#define DEFAULT_SAMPLES_TO_BITS 0.20f  // Default ~5.0 samples/bit (measured from BlueJay ESCs)
#define OVERSAMPLE_WORDS 4   // 4 x 32 = 128 samples (need ~115 for 21 bits at ~5.49x)
#define MAX_EDGES 32

// Auto-calibration for ESC telemetry timing
// At 0 RPM (0xFFF0), we know the exact pattern: 15 edges spanning 18 bits
// By measuring the actual sample span, we can calculate the true samples-per-bit rate
#define CALIBRATION_FRAMES 16       // Number of 0 RPM frames to average
#define ZERO_RPM_EDGES 15           // 0 RPM pattern has exactly 15 edges
#define ZERO_RPM_BIT_SPAN 18        // 18 bits between first and last edge for 0 RPM
                                    // (edges at bit 2 and bit 20 of the RLE pattern)

static float samplesToBits = DEFAULT_SAMPLES_TO_BITS;
static bool calibrationComplete = false;
static uint32_t calibrationSum = 0;      // Sum of sample spans
static uint8_t calibrationCount = 0;     // Number of calibration frames collected

// Failure reason tracking for debug
typedef enum {
    FAIL_NONE = 0,
    FAIL_EDGE_COUNT,
    FAIL_BIT_COUNT,
    FAIL_GCR_DECODE,
    FAIL_CHECKSUM
} failReason_e;

static failReason_e lastFailReason = FAIL_NONE;

// Store edge info for calibration and debug
static uint16_t lastEdgePositions[MAX_EDGES];
static int lastEdgeCount;

#ifdef PICO_TRACE
#define dbgEdgePositions lastEdgePositions
#define dbgEdgeCount lastEdgeCount
#endif

static uint32_t decodeOversampledTelemetry(int motorIndex, const uint32_t *buffer)
{
#ifndef PICO_TRACE
    UNUSED(motorIndex);
#endif

    // Find edges in the sample stream
    // Each word has 32 samples, MSB is earliest (left shift in PIO)
    // Store in lastEdgePositions for calibration and debug
    lastEdgeCount = 0;

    uint8_t lastBit = (buffer[0] >> 31) & 1;

    for (int word = 0; word < OVERSAMPLE_WORDS && lastEdgeCount < MAX_EDGES; word++) {
        uint32_t samples = buffer[word];
        for (int bit = 31; bit >= 0 && lastEdgeCount < MAX_EDGES; bit--) {
            uint8_t currentBit = (samples >> bit) & 1;
            if (currentBit != lastBit) {
                int16_t pos = (word * 32) + (31 - bit);
                lastEdgePositions[lastEdgeCount++] = pos;
                lastBit = currentBit;
            }
        }
    }

    if (lastEdgeCount < 2) {
        lastFailReason = FAIL_EDGE_COUNT;
        return DSHOT_TELEMETRY_INVALID;
    }

    // Note: We intentionally do NOT normalize edge positions.
    // The first edge position varies based on the GCR-encoded value:
    // - 0 RPM (0xFFF) has first edge at ~11 samples (bit 2)
    // - Real RPM values can have first edge at ~5-6 samples (bit 1)
    // Normalizing would add phantom bits to the first run for non-zero RPM.

    // Convert edge positions to GCR bits using run-length decoding
    // Try different first_run values (1, 2, 3) to handle variable first edge timing
    uint32_t decodedValue = DSHOT_TELEMETRY_INVALID;

    for (int firstRun = 1; firstRun <= 3; firstRun++) {
        uint32_t gcrValue = 1U << (firstRun - 1);  // first_run bits: 1 followed by (firstRun-1) zeros
        uint32_t totalBits = firstRun;

        // Process inter-edge gaps
        for (int i = 1; i <= lastEdgeCount; i++) {
            uint32_t len;
            if (i < lastEdgeCount) {
                int32_t diff = lastEdgePositions[i] - lastEdgePositions[i - 1];
                if (totalBits >= 21U) {
                    break;
                }
                len = (uint32_t)(diff * samplesToBits + 0.5f);
            } else {
                len = 21U - totalBits;  // Pad to 21 bits
            }

            if (len == 0) {
                len = 1;
            }

            gcrValue <<= len;
            gcrValue |= 1U << (len - 1U);
            totalBits += len;
        }

        // Need exactly 21 bits (allow 22 due to timing jitter)
        if (totalBits < 21U || totalBits > 22U) {
            continue;
        }

        // Remove start bit (MSB) to get 20-bit GCR value
        uint32_t gcr20 = gcrValue & 0xFFFFF;

        // GCR decode: extract 4 nibbles from 4 x 5-bit symbols
        uint8_t n3 = gcrDecodeLut[(gcr20 >> 15) & 0x1F];
        uint8_t n2 = gcrDecodeLut[(gcr20 >> 10) & 0x1F];
        uint8_t n1 = gcrDecodeLut[(gcr20 >> 5) & 0x1F];
        uint8_t n0 = gcrDecodeLut[gcr20 & 0x1F];

        // Check if all symbols are valid
        if (n0 > 15 || n1 > 15 || n2 > 15 || n3 > 15) {
            continue;
        }

        // Assemble 16-bit value and verify checksum
        uint32_t candidateValue = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0;
        uint32_t csum = candidateValue;
        csum = csum ^ (csum >> 8);
        csum = csum ^ (csum >> 4);

        if ((csum & 0xF) == 0xF) {
            decodedValue = candidateValue;
            break;
        }
    }

    if (decodedValue == DSHOT_TELEMETRY_INVALID) {
        // Determine failure reason from last attempt
        uint32_t gcrValue = 1U << 2;  // first_run=3
        uint32_t totalBits = 3;

        for (int i = 1; i <= lastEdgeCount && totalBits < 21U; i++) {
            uint32_t len;
            if (i < lastEdgeCount) {
                int32_t diff = lastEdgePositions[i] - lastEdgePositions[i - 1];
                len = (uint32_t)(diff * samplesToBits + 0.5f);
            } else {
                len = 21U - totalBits;
            }
            if (len == 0) len = 1;
            gcrValue <<= len;
            gcrValue |= 1U << (len - 1U);
            totalBits += len;
        }

        if (totalBits < 21U || totalBits > 22U) {
            lastFailReason = FAIL_BIT_COUNT;
        } else {
            uint32_t gcr20 = gcrValue & 0xFFFFF;
            uint8_t n3 = gcrDecodeLut[(gcr20 >> 15) & 0x1F];
            uint8_t n2 = gcrDecodeLut[(gcr20 >> 10) & 0x1F];
            uint8_t n1 = gcrDecodeLut[(gcr20 >> 5) & 0x1F];
            uint8_t n0 = gcrDecodeLut[gcr20 & 0x1F];

            if (n0 > 15 || n1 > 15 || n2 > 15 || n3 > 15) {
                lastFailReason = FAIL_GCR_DECODE;
#ifdef PICO_TRACE
                static uint32_t gcrFailCount = 0;
                if (motorIndex == 0 && (++gcrFailCount % 1000) == 1) {
                    uint8_t sym3 = (gcr20 >> 15) & 0x1F;
                    uint8_t sym2 = (gcr20 >> 10) & 0x1F;
                    uint8_t sym1 = (gcr20 >> 5) & 0x1F;
                    uint8_t sym0 = gcr20 & 0x1F;
                    bprintf("M0 GCR fail: syms=%02x,%02x,%02x,%02x gcr20=%05x edges=%d",
                            sym3, sym2, sym1, sym0, gcr20, dbgEdgeCount);
                    bprintf("  pos: %d %d %d %d %d %d %d %d %d %d %d %d",
                            dbgEdgePositions[0], dbgEdgePositions[1], dbgEdgePositions[2],
                            dbgEdgePositions[3], dbgEdgePositions[4], dbgEdgePositions[5],
                            dbgEdgePositions[6], dbgEdgePositions[7], dbgEdgePositions[8],
                            dbgEdgePositions[9], dbgEdgePositions[10], dbgEdgePositions[11]);
                }
#endif
            } else {
                lastFailReason = FAIL_CHECKSUM;
            }
        }
        return DSHOT_TELEMETRY_INVALID;
    }

    lastFailReason = FAIL_NONE;

    // Auto-calibration: use 0 RPM frames to measure actual telemetry timing
    // 0 RPM (0xFFF0) has exactly 15 edges spanning 18 bits
    if (!calibrationComplete && decodedValue == 0xFFF0 && lastEdgeCount == ZERO_RPM_EDGES) {
        // Measure sample span from first to last edge
        uint32_t sampleSpan = lastEdgePositions[ZERO_RPM_EDGES - 1] - lastEdgePositions[0];

        // Sanity check: span should be roughly 19 bits * 5 samples = 95 samples (allow 70-120)
        if (sampleSpan >= 70 && sampleSpan <= 120) {
            calibrationSum += sampleSpan;
            calibrationCount++;

            if (calibrationCount >= CALIBRATION_FRAMES) {
                // Calculate average samples per bit and derive conversion factor
                float avgSpan = (float)calibrationSum / CALIBRATION_FRAMES;
                float samplesPerBit = avgSpan / ZERO_RPM_BIT_SPAN;
                samplesToBits = 1.0f / samplesPerBit;
                calibrationComplete = true;

                bprintf("Telemetry calibrated: %d frames, avgSpan=%.1f, samplesPerBit=%.2f, samplesToBits=%.4f",
                        CALIBRATION_FRAMES, (double)avgSpan, (double)samplesPerBit, (double)samplesToBits);
            }
        }
    }

#ifdef PICO_TRACE
    // Occasionally print successful decodes for motor 0
    static uint32_t successCount = 0;
    if (motorIndex == 0 && (++successCount % 5000) == 1) {
        uint16_t eRPM = (decodedValue >> 4) & 0xFFF;
        bprintf("M0 OK: eRPM=%u (raw=%04x) edges=%d cal=%d",
                eRPM, decodedValue, dbgEdgeCount, calibrationComplete);
        bprintf("  pos: %d %d %d %d %d %d %d %d %d %d %d %d",
                dbgEdgePositions[0], dbgEdgePositions[1], dbgEdgePositions[2],
                dbgEdgePositions[3], dbgEdgePositions[4], dbgEdgePositions[5],
                dbgEdgePositions[6], dbgEdgePositions[7], dbgEdgePositions[8],
                dbgEdgePositions[9], dbgEdgePositions[10], dbgEdgePositions[11]);
    }
#endif

    // Return 12-bit eRPM (remove 4-bit checksum)
    return (decodedValue >> 4) & 0xFFF;
}

bool dshotTelemetryWait(void)
{
    bool telemetryWait = false;
#ifdef USE_DSHOT_TELEMETRY
    bool telemetryPending;
    const timeUs_t startTimeUs = micros();

    do {
        telemetryPending = false;
/*
  TODO TBC this could be something like
        for (unsigned motorIndex = 0; motorIndex < dshotMotorCount && !telemetryPending; motorIndex++) {
            const motorOutput_t *motor = &dshotMotors[motorIndex];

            // call a function that (taking into account which PIO program we are running), tells
            // us on the basis of
            //    uint8_t pio_pc = pio_sm_get_pc(motor->pio, motor->pio_sm) - motor->offset;
            // whether we have finished receiving a sequence of bidir telemetry bits and it's
            // therefore safe to send out a dshot command
            telemetryPending |= SafeToSend(motor);

            or, we can keep track of state based on whether we have called
               dshotwrite
               dshotdecodetelemetry successfully
               dshotdecodetelemetry unsuccessfully
            and, if necessary, keep calling dshotdecodetelemetry (up to a timeout) until telemetry received
        }

  I think telemetryPending tracks when we are safe to transmit
  and telemetryWait is for debugging, telling us if we had to wait
  (but no code looks at the return value of this function)

 */
        telemetryWait |= telemetryPending;

        if (cmpTimeUs(micros(), startTimeUs) > DSHOT_TELEMETRY_TIMEOUT) {
            break;
        }
    } while (telemetryPending);

    if (telemetryWait) {
        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 2, debug[2] + 1);
    }

    bprintf("dshotTelemetryWait returning %d", telemetryWait);
#endif
    return telemetryWait;
}

bool dshotDecodeTelemetry(void)
{
#ifndef USE_DSHOT_TELEMETRY
    return true;
#else
    if (!useDshotTelemetry) {
        return true;
    }

#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = millis();
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
        const motorOutput_t *motor = &dshotMotors[motorIndex];

        int fifo_words = pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm);

        // Wait until we have all 4 words, or skip if not enough yet
        if (fifo_words < OVERSAMPLE_WORDS) {
            continue;
        }

        // Discard older data to get most recent frame
        while (fifo_words > OVERSAMPLE_WORDS) {
            (void)pio_sm_get(motor->pio, motor->pio_sm);
            fifo_words--;
        }

        // Read 4 words of oversampled data using NON-BLOCKING reads
        // IMPORTANT: pio_sm_get_blocking can hang forever if FIFO is empty,
        // which would trigger watchdog reset. Use non-blocking with re-check.
        uint32_t sampleBuffer[OVERSAMPLE_WORDS];
        bool readOk = true;
        for (int i = 0; i < OVERSAMPLE_WORDS; i++) {
            if (pio_sm_is_rx_fifo_empty(motor->pio, motor->pio_sm)) {
                // FIFO unexpectedly empty - skip this motor to avoid hanging
                readOk = false;
                break;
            }
            sampleBuffer[i] = pio_sm_get(motor->pio, motor->pio_sm);
        }
        if (!readOk) {
            continue;
        }

        uint32_t rawValue = decodeOversampledTelemetry(motorIndex, sampleBuffer);

        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 0, debug[0] + 1);
        dshotTelemetryState.readCount++;

        if (rawValue != DSHOT_TELEMETRY_INVALID) {
            if ((rawValue == 0x0E00) && (dshotCommandGetCurrent(motorIndex) == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE)) {
                dshotTelemetryState.motorState[motorIndex].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
            } else {
                dshotTelemetryState.motorState[motorIndex].rawValue = rawValue;
            }
        } else {
            dshotTelemetryState.invalidPacketCount++;
        }

#ifdef PICO_TRACE
        // Periodic stats with failure breakdown
        static uint32_t okCount = 0, reportCount = 0;
        static uint32_t failEdge = 0, failBits = 0, failGcr = 0, failCsum = 0;
        static uint32_t lastOk = 0, lastEdge = 0, lastBits = 0, lastGcr = 0, lastCsum = 0;
        if (rawValue != DSHOT_TELEMETRY_INVALID) {
            okCount++;
        } else {
            switch (lastFailReason) {
                case FAIL_EDGE_COUNT: failEdge++; break;
                case FAIL_BIT_COUNT:  failBits++; break;
                case FAIL_GCR_DECODE: failGcr++;  break;
                case FAIL_CHECKSUM:   failCsum++; break;
                default: break;
            }
        }
        if ((++reportCount % 40000) == 0) {
            uint32_t dOk = okCount - lastOk;
            uint32_t dEdge = failEdge - lastEdge;
            uint32_t dBits = failBits - lastBits;
            uint32_t dGcr = failGcr - lastGcr;
            uint32_t dCsum = failCsum - lastCsum;
            uint32_t dFail = dEdge + dBits + dGcr + dCsum;
            uint32_t dTotal = dOk + dFail;
            uint32_t pct = dTotal > 0 ? (dFail * 100) / dTotal : 0;
            bprintf("t=%u ok=%u err=%u%% [edge=%u bits=%u gcr=%u csum=%u]",
                    millis(), dOk, pct, dEdge, dBits, dGcr, dCsum);
            lastOk = okCount;
            lastEdge = failEdge;
            lastBits = failBits;
            lastGcr = failGcr;
            lastCsum = failCsum;
        }
#endif

#ifdef USE_DSHOT_TELEMETRY_STATS
        updateDshotTelemetryQuality(&dshotTelemetryQuality[motorIndex], rawValue != DSHOT_TELEMETRY_INVALID, currentTimeMs);
#endif
    }

    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
    return true;
#endif
}

#endif // defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
