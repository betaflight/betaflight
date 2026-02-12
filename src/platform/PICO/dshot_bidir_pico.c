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

uint32_t getCycleCounter(void);

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
//
// ERROR RATE LIMITATION:
// With spinning motors, expect ~5-8% decode errors due to hardware constraints.
// This is caused by inter-edge timing jitter where gaps fall on the boundary
// between 1-bit and 2-bit runs. With ~5x oversampling and ESC timing jitter of
// ~2 samples, borderline gaps cannot be reliably decoded. At 0 RPM (stationary),
// error rate is <1% which satisfies Configurator thresholds.

bool dshot_program_bidir_init(PIO pio, uint sm, int offset, uint pin)
{
    bprintf("dshot_program_bidir_init on pin %d", pin);
    pio_sm_config config = dshot_600_bidir_program_get_default_config(offset);

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

// GCR decode lookup table (maps 5-bit GCR to 4-bit nibble, 255 = invalid)
static const uint8_t gcrDecodeLut[32] = {
    255, 255, 255, 255, 255, 255, 255, 255,
    255,   9,  10,  11, 255,  13,  14,  15,
    255, 255,   2,   3, 255,   5,   6,   7,
    255,   0,   8,   1, 255,   4,  12, 255
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
#define OVERSAMPLE_WORDS 4   // 4 x 32 = 128 samples (need ~115 for 21 bits at ~5.49x)
#define MAX_EDGES 24 // valid samples have max certainly no more than 22 edges (21 data + final revert to high)
#define US_PER_CYCLE (18.0f / 75) // Sample every 18 PIO cycles, PIO clock is 75MHz
#define DEFAULT_SAMPLES_TO_BITS (US_PER_CYCLE / 1.33333333333f) // Default 0.18 bits / sample, 5.555.. samples/bit

// Auto-calibration for ESC telemetry timing
// At 0 RPM (0xFFF0), we know the exact pattern: 15 edges spanning 18 bits
// By measuring the actual sample span, we can calculate the true samples-per-bit rate
#define CALIBRATION_FRAMES 32       // Number of 0 RPM frames to average
#define ZERO_RPM_EDGES 15           // 0 RPM pattern has exactly 15 edges
#define ZERO_RPM_BIT_SPAN 18        // 18 bits between first and last edge for 0 RPM
                                    // (edges at bit 2 and bit 20 of the RLE pattern)

static bool calibrationComplete;

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
static uint8_t edgeDiffs[24];

// transitions 0 to 1, 1 to 2, 2 to 3, 3 to 4.
// interpret number of samples as bit length n when
// lengthTransitions[n-1] <= num samples < lengthTransitions[n]
static uint8_t lengthTransitions[4];

static void setTransitions(float bitsPerSample, bool strict)
{
    int length = 0;
    float bits = 0.5;
    int samples = 0;
    while (length < 4) {
        bits += bitsPerSample;
        samples++;

        int roundedBits = bits;
        if (roundedBits >= length + 1) {
            // transition from length to length + 1
            lengthTransitions[length] = samples;
            length++;
        }
    }

    if (!strict) {
        // Allow flexibility in decoding a long run as 3 while calibrating
        lengthTransitions[3] += 2;
    }

    bprintf("setTransitions from %.3f got %d %d %d %d", (double)bitsPerSample, lengthTransitions[0], lengthTransitions[1], lengthTransitions[2],lengthTransitions[3]);
}

static uint32_t decodeOversampledTelemetry(int motorIndex, const uint32_t *buffer)
{
#ifndef PICO_TRACE
    UNUSED(motorIndex);
#endif
    static int decodeCount;
    if (decodeCount++ == 0) {
        setTransitions(DEFAULT_SAMPLES_TO_BITS, false);
    }

    lastFailReason = FAIL_NONE;
#ifdef PICO_TRACE
    uint32_t c0 = getCycleCounter();
#endif
    bool ones = buffer[0] >> 31;
    // First sample ought to be zero, so ones starts as false
    uint32_t w0 = buffer[0];
    uint32_t w1 = buffer[1];
    uint32_t w2 = buffer[2];
    uint32_t w3 = buffer[3];
    int shiftcount = 128;
    int edgeCount = 0;

    while (shiftcount > 0 && edgeCount < MAX_EDGES) {
        uint32_t testword = ones ? ~w0 : w0;
        if (testword == 0) {
            break;  // Remaining bits are all the same polarity
        }
        int runLength = __builtin_clz(testword);
        edgeDiffs[edgeCount++] = runLength;

        int runLengthComplement = 32 - runLength;
        w0 = w0 << runLength | (w1 >> runLengthComplement);
        w1 = w1 << runLength | (w2 >> runLengthComplement);
        // Shortcut when w2 runs out - saves a bit of time
        if (w2) {
            w2 = w2 << runLength | (w3 >> runLengthComplement);
            w3 = w3 << runLength;
        }

        ones = !ones;
        shiftcount -= runLength;
    }

    // Don't want to count last run of ones. "Padding" will be deduced later.
    edgeCount--;

    if (edgeCount < 2 || edgeCount > 21) {
        lastFailReason = FAIL_EDGE_COUNT;
    }

#ifdef PICO_TRACE
    uint32_t c1 = getCycleCounter();
#endif

    // Convert run lengths to GCR bits using transition-table decoding
    uint32_t decodedValue = DSHOT_TELEMETRY_INVALID;

    // Compute core GCR pattern from inter-edge gaps
    uint32_t coreGcr = 0;
    uint32_t coreBits = 0;
    if (lastFailReason == FAIL_NONE) {
        for (int i=0; i<edgeCount; ++i) {
            uint8_t diff = edgeDiffs[i];
            int len; // The only run lengths that lead to valid GCR are 1, 2, 3.
            // Ignore lengthTransition[0], assume too short means run length == 1
            if (diff < lengthTransitions[1]) {
                len = 1;
            } else if (diff < lengthTransitions[2]) {
                len = 2;
            } else if (diff < lengthTransitions[3]) {
                len = 3;
            } else {
                // Anticipating it would fail GCR decode: no sequences of 000 in GCR stream => no runs of length >=4 here.
                lastFailReason = FAIL_GCR_DECODE;
                break;
            }

            coreGcr <<= len;
            coreGcr |= 1U << (len - 1U);
            coreBits += len;
            if (coreBits >= 21U) {
                break;
            }
        }
    }

#ifdef PICO_TRACE
    uint32_t c2 = getCycleCounter();
#endif

    int32_t paddingLen;
    if (lastFailReason == FAIL_NONE) {
        paddingLen = 21 - coreBits; // coreBits will have a leading 1, to be discarded
        if (paddingLen < 0) {
            lastFailReason = FAIL_BIT_COUNT;
        }
    }

    if (lastFailReason == FAIL_NONE) {
        // Build gcr20 directly: core shifted by padding, plus padding terminator
        uint32_t gcr20 = coreGcr << paddingLen;
        if (paddingLen > 0) {
            // This happens when the original sample ended with a number of 1 data bits, which
            // can't be separated from the following high values when ESC has finished sending.
            gcr20 |= 1U << (paddingLen - 1);
        }

        // GCR decode: extract 4 nibbles from 4 x 5-bit symbols (discard top 21st bit)
        uint8_t n3 = gcrDecodeLut[(gcr20 >> 15) & 0x1F];
        uint8_t n2 = gcrDecodeLut[(gcr20 >> 10) & 0x1F];
        uint8_t n1 = gcrDecodeLut[(gcr20 >> 5) & 0x1F];
        uint8_t n0 = gcrDecodeLut[gcr20 & 0x1F];

        // Check if all symbols are valid
        if (n0 > 15U || n1 > 15U || n2 > 15U || n3 > 15U) {
            lastFailReason = FAIL_GCR_DECODE;
        } else {
            // Assemble 16-bit value and verify checksum
            uint32_t candidateValue = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0;
            uint32_t csum = candidateValue;
            csum = csum ^ (csum >> 8);
            csum = csum ^ (csum >> 4);

            if ((csum & 0xF) != 0xF) {
                lastFailReason = FAIL_CHECKSUM;
            } else {
                decodedValue = candidateValue;
            }
        }
    }

#ifdef PICO_TRACE
    uint32_t c3 = getCycleCounter();
#endif

    // Auto-calibration: use 0 RPM frames to measure actual telemetry timing
    // 0 RPM (0xFFF0 from a successful decode) has exactly 15 edges spanning 18 bits
    if (!calibrationComplete && decodedValue == 0xFFF0 && edgeCount == ZERO_RPM_EDGES) {
        uint32_t sampleSpan = 0;
        for (int i=1; i<ZERO_RPM_EDGES; ++i) {
            sampleSpan += edgeDiffs[i];
        }

        static uint32_t totalSpan;
        static int totalFrames;
        // Sanity check: span should be roughly 18 bits * 5.5 samples = 99 samples (allow 70-125)
        if (sampleSpan >= 70 && sampleSpan <= 125) {
            totalSpan += sampleSpan;
            totalFrames++;
        }

        if (totalFrames == CALIBRATION_FRAMES) {
            // Calculate average samples per bit and derive conversion factor
            float avgSpan = (float)totalSpan / CALIBRATION_FRAMES;
            float samplesPerBit = avgSpan / ZERO_RPM_BIT_SPAN;
            float samplesToBitsf = 1.0f / samplesPerBit;

            bprintf("Telemetry calibrated: %d frames, avgSpan=%.1f, samplesPerBit=%.2f, samplesToBits=%.4f",
                    CALIBRATION_FRAMES, (double)avgSpan, (double)samplesPerBit, (double)samplesToBitsf);
            setTransitions(samplesToBitsf, true);
            calibrationComplete = true;
        }
    }

#ifdef PICO_TRACE
    uint32_t c4 = getCycleCounter();

    // Occasionally print decodes
    static uint32_t failCount;
    static uint32_t successCountM0;
    failCount += lastFailReason != FAIL_NONE;
    successCountM0 += motorIndex == 0 && lastFailReason == FAIL_NONE;
    if ((lastFailReason != FAIL_NONE && (failCount % 16384) == 1) || (motorIndex == 0 && ((successCountM0 % 65536) == 1))) {
        uint16_t eRPM = (decodedValue >> 4) & 0xFFF;

        bprintf("diffs (1->2 %d 2->3 %d) %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d  %d %d %d %d M%d %s: eRPM=%u (raw=%04x) edges=%d cal=%d",
                lengthTransitions[1], lengthTransitions[2],
                edgeDiffs[0], edgeDiffs[1], edgeDiffs[2], edgeDiffs[3], edgeDiffs[4], edgeDiffs[5], edgeDiffs[6], edgeDiffs[7], 
                edgeDiffs[8], edgeDiffs[9], edgeDiffs[10], edgeDiffs[11], edgeDiffs[12], edgeDiffs[13], edgeDiffs[14], edgeDiffs[15], 
                edgeDiffs[16], edgeDiffs[17], edgeDiffs[18], edgeDiffs[19],
                motorIndex, lastFailReason == FAIL_NONE ? "OK" : "Fail",
                eRPM, decodedValue, edgeCount, calibrationComplete);

        uint32_t c5 = getCycleCounter();
        bprintf("cycles %d %d %d %d %d",c1-c0,c2-c1,c3-c2,c4-c3,c5-c4);
    }
#endif

    // Return 12-bit eRPM (remove 4-bit checksum)
    return  lastFailReason == FAIL_NONE ? (decodedValue >> 4) & 0xFFF : DSHOT_TELEMETRY_INVALID;
}

bool dshotTelemetryWait(void)
{
    bool telemetryWait = false;
#ifdef USE_DSHOT_TELEMETRY
    bool telemetryPending;
    const timeUs_t startTimeUs = micros();

    do {
        telemetryPending = false;
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

#ifdef PICO_TRACE
    static uint32_t cacc;
    static uint32_t cnc;
    static uint32_t cncm;
    static uint32_t cne;
    uint32_t c1 = getCycleCounter();
    cnc++;
#endif

#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = micros() / 1024;
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
#ifdef PICO_TRACE
        cncm++;
#endif
        const motorOutput_t *motor = &dshotMotors[motorIndex];

        int fifo_words = pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm);

        // Wait until we have all 4 words, or skip if not enough yet
        if (fifo_words < OVERSAMPLE_WORDS) {
#ifdef PICO_TRACE
            cne++;
#endif
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
            // Drain remaining RX FIFO to maintain frame alignment
            while (!pio_sm_is_rx_fifo_empty(motor->pio, motor->pio_sm)) {
                (void)pio_sm_get(motor->pio, motor->pio_sm);
            }
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
        if ((++reportCount % 131072) == 0) {
            uint32_t dOk = okCount - lastOk;
            uint32_t dEdge = failEdge - lastEdge;
            uint32_t dBits = failBits - lastBits;
            uint32_t dGcr = failGcr - lastGcr;
            uint32_t dCsum = failCsum - lastCsum;
            uint32_t dFail = dEdge + dBits + dGcr + dCsum;
            uint32_t dTotal = dOk + dFail;
            uint32_t pct = dTotal > 0 ? (dFail * 100) / dTotal : 0;
            // Average time in dshotDecodeTelemetry, num calls, num motor-calls, timestamp, num times not enough in rx fifo,
            // num ok, %err, error breakdown
            bprintf("d = %d us, nt=%u nm=%u, t=%u ne = %u ok=%u err=%u%% [edge=%u bits=%u gcr=%u csum=%u]",
                    cacc/(150*cnc), cnc, cncm,  micros() / 1024, cne, dOk, pct, dEdge, dBits, dGcr, dCsum);
            cacc = 0;
            cne = 0;
            cnc = 0;
            cncm = 0;
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
#ifdef PICO_TRACE
    cacc += getCycleCounter() - c1;
#endif

    return true;
#endif
}

#endif // defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
