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

#define DSHOT_BIDIR_BIT_PERIOD 48

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

// Actual oversampling rate is 5.5x (38.4 PIO cycles/bit ÷ 7 cycles/sample)
// For run-length decoding: len = round(diff / 5.5) = (diff * 2 + 5) / 11
#define OVERSAMPLE_WORDS 4   // 4 x 32 = 128 samples (need 115 for 21 bits at 5.5x)
#define MAX_EDGES 32

static uint32_t decodeOversampledTelemetry(int motorIndex, const uint32_t *buffer)
{
#ifdef PICO_TRACE
    bprintf("M%d Raw: %08x %08x %08x %08x", motorIndex,
            buffer[0], buffer[1], buffer[2], buffer[3]);
#else
    UNUSED(motorIndex);
#endif

    // Find edges in the sample stream
    // Each word has 32 samples, MSB is earliest (left shift in PIO)
    uint16_t edgePositions[MAX_EDGES];
    int edgeCount = 0;

    uint8_t lastBit = (buffer[0] >> 31) & 1;

    for (int word = 0; word < OVERSAMPLE_WORDS && edgeCount < MAX_EDGES; word++) {
        uint32_t samples = buffer[word];
        for (int bit = 31; bit >= 0 && edgeCount < MAX_EDGES; bit--) {
            uint8_t currentBit = (samples >> bit) & 1;
            if (currentBit != lastBit) {
                edgePositions[edgeCount++] = (word * 32) + (31 - bit);
                lastBit = currentBit;
            }
        }
    }

    if (edgeCount < 2) {
        return DSHOT_TELEMETRY_INVALID;
    }

    // Note: We intentionally do NOT normalize edge positions.
    // The first edge position varies based on the GCR-encoded value:
    // - 0 RPM (0xFFF) has first edge at ~11 samples (bit 2)
    // - Real RPM values can have first edge at ~5-6 samples (bit 1)
    // Normalizing would add phantom bits to the first run for non-zero RPM.

    // Convert edge positions to GCR bits using run-length decoding
    // Each run of N samples at 5.5x oversampling: len = round(N / 5.5) = (N * 2 + 5) / 11
    // Encoded as: 1 at MSB followed by (len-1) zeros
    uint32_t gcrValue = 0;
    uint32_t totalBits = 0;
    uint16_t prevEdgePos = 0;  // Virtual start position

    for (int i = 0; i <= edgeCount; i++) {
        uint32_t len;
        if (i < edgeCount) {
            int32_t diff = edgePositions[i] - prevEdgePos;
            if (totalBits >= 21U) {
                break;
            }
            // len = round(diff / 5.5) using integer math: (diff * 2 + 5) / 11
            len = (diff * 2U + 5U) / 11U;
        } else {
            len = 21U - totalBits;  // Pad to 21 bits
        }

        if (len == 0) {
            len = 1;
        }

        gcrValue <<= len;
        gcrValue |= 1U << (len - 1U);
        prevEdgePos = (i < edgeCount) ? edgePositions[i] : prevEdgePos;
        totalBits += len;
    }

    if (totalBits != 21U) {
        return DSHOT_TELEMETRY_INVALID;
    }

    // Remove start bit (MSB) to get 20-bit GCR value
    uint32_t gcr20 = gcrValue & 0xFFFFF;

    // GCR decode: extract 4 nibbles from 4 x 5-bit symbols
    uint8_t n3 = gcrDecodeLut[(gcr20 >> 15) & 0x1F];
    uint8_t n2 = gcrDecodeLut[(gcr20 >> 10) & 0x1F];
    uint8_t n1 = gcrDecodeLut[(gcr20 >> 5) & 0x1F];
    uint8_t n0 = gcrDecodeLut[gcr20 & 0x1F];

    if (n0 > 15 || n1 > 15 || n2 > 15 || n3 > 15) {
#ifdef PICO_TRACE
        bprintf("Invalid GCR: %02x %02x %02x %02x (gcr20=%05x)", n3, n2, n1, n0, gcr20);
#endif
        return DSHOT_TELEMETRY_INVALID;
    }

    // Assemble 16-bit value and verify checksum
    uint32_t decodedValue = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0;
    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8);
    csum = csum ^ (csum >> 4);

    if ((csum & 0xF) != 0xF) {
#ifdef PICO_TRACE
        bprintf("Checksum fail: csum=%x (expected 0xF), value=%04x", csum & 0xF, decodedValue);
#endif
        return DSHOT_TELEMETRY_INVALID;
    }

    // Return 12-bit eRPM (remove 4-bit checksum)
    return (decodedValue >> 4) & 0xFFF;
}

bool dshotTelemetryWait(void)
{
    bool telemetryWait = false;
#ifdef USE_DSHOT_TELEMETRY
    bool telemetryPending;
    const timeUs_t startTimeUs = micros();

    bprintf("dshotTelemetryWait");
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

        // Read 4 words of oversampled data
        uint32_t sampleBuffer[OVERSAMPLE_WORDS];
        for (int i = 0; i < OVERSAMPLE_WORDS; i++) {
            sampleBuffer[i] = pio_sm_get_blocking(motor->pio, motor->pio_sm);
        }

        uint32_t rawValue = decodeOversampledTelemetry(motorIndex, sampleBuffer);

        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 0, debug[0] + 1);
        dshotTelemetryState.readCount++;

        if (rawValue != DSHOT_TELEMETRY_INVALID) {
            if ((rawValue == 0x0E00) && (dshotCommandGetCurrent(motorIndex) == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE)) {
                bprintf("\n** received dshot 0x0E00");
                dshotTelemetryState.motorState[motorIndex].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
            } else {
                dshotTelemetryState.motorState[motorIndex].rawValue = rawValue;
            }
        } else {
            dshotTelemetryState.invalidPacketCount++;
        }

#ifdef USE_DSHOT_TELEMETRY_STATS
        updateDshotTelemetryQuality(&dshotTelemetryQuality[motorIndex], rawValue != DSHOT_TELEMETRY_INVALID, currentTimeMs);
#endif
    }

    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
    return true;
#endif
}

#endif // defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
