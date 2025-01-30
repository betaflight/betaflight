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

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "platform.h"

#if defined(USE_DSHOT_TELEMETRY)

#include "common/maths.h"
#include "common/utils.h"
#include "build/debug.h"
#include "drivers/dshot.h"
#include "drivers/dshot_bitbang_decode.h"
#include "drivers/time.h"

#define MIN_VALID_BBSAMPLES ((21 - 2) * 3) // 57
#define MAX_VALID_BBSAMPLES ((21 + 2) * 3) // 69

// setting this define in dshot.h allows the cli command dshot_telemetry_info to
// display the received telemetry data in raw form which helps identify
// the root cause of packet decoding issues.

#ifdef DEBUG_BBDECODE
uint16_t bbBuffer[134];
#endif

/* Bit band SRAM definitions */
#define BITBAND_SRAM_REF   0x20000000
#define BITBAND_SRAM_BASE  0x22000000
#define BITBAND_SRAM(a,b) ((BITBAND_SRAM_BASE + (((a)-BITBAND_SRAM_REF)<<5) + ((b)<<2)))  // Convert SRAM address

// Period at which to check preamble length
#define MARGIN_CHECK_INTERVAL_US 500000

// Target 5 clock cycles of input data ahead of leading edge
#define DSHOT_TELEMETRY_START_MARGIN 5

static uint32_t minMargin = UINT32_MAX;
static timeUs_t nextMarginCheckUs = 0;

static uint8_t preambleSkip = 0;

typedef struct bitBandWord_s {
    uint32_t value;
    uint32_t junk[15];
} bitBandWord_t;

#ifdef DEBUG_BBDECODE
uint32_t sequence[MAX_GCR_EDGES];
int sequenceIndex = 0;
#endif

static uint32_t decode_bb_value(uint32_t value, uint16_t buffer[], uint32_t count, uint32_t bit)
{
#ifndef DEBUG_BBDECODE
    UNUSED(buffer);
    UNUSED(count);
    UNUSED(bit);
#endif
#define iv 0xffffffff
    // First bit is start bit so discard it.
    value &= 0xfffff;
    static const uint32_t decode[32] = {
        iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
        iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv };

    uint32_t decodedValue = decode[value & 0x1f];
    decodedValue |= decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if ((csum & 0xf) != 0xf || decodedValue > 0xffff) {
#ifdef DEBUG_BBDECODE
        memcpy(dshotTelemetryState.inputBuffer, sequence, sizeof(sequence));
        for (unsigned i = 0; i < count; i++) {
            bbBuffer[i] = !!(buffer[i] & (1 << bit));
        }
#endif
        value = DSHOT_TELEMETRY_INVALID;
    } else {
        value = decodedValue >> 4;
    }

    return value;
}

#ifdef USE_DSHOT_BITBAND
uint32_t decode_bb_bitband( uint16_t buffer[], uint32_t count, uint32_t bit)
{
    timeUs_t now = micros();
#ifdef DEBUG_BBDECODE
    memset(sequence, 0, sizeof(sequence));
    sequenceIndex = 0;
#endif
    uint32_t value = 0;

    bitBandWord_t* p = (bitBandWord_t*)BITBAND_SRAM((uint32_t)buffer, bit);
    bitBandWord_t* b = p;
    bitBandWord_t* endP = p + (count - MIN_VALID_BBSAMPLES);

    // Jump forward in the buffer to just before where we anticipate the first zero
    p += preambleSkip;

    DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 3, preambleSkip);

    // Eliminate leading high signal level by looking for first zero bit in data stream.
    // Manual loop unrolling and branch hinting to produce faster code.
    while (p < endP) {
        if (__builtin_expect((!(p++)->value), 0) ||
            __builtin_expect((!(p++)->value), 0) ||
            __builtin_expect((!(p++)->value), 0) ||
            __builtin_expect((!(p++)->value), 0)) {
            break;
        }
    }

    const uint32_t startMargin = p - b;

    if (p >= endP) {
        // not returning telemetry is ok if the esc cpu is
        // overburdened.  in that case no edge will be found and
        // BB_NOEDGE indicates the condition to caller
        if (preambleSkip > 0) {
            // Increase the start margin
            preambleSkip--;
        }
        return DSHOT_TELEMETRY_NOEDGE;
    }

    const int remaining = MIN(count - startMargin, (unsigned int)MAX_VALID_BBSAMPLES);

    bitBandWord_t* oldP = p;
    uint32_t bits = 0;
    endP = p + remaining;

#ifdef DEBUG_BBDECODE
    sequence[sequenceIndex++] = p - b;
#endif

    while (endP > p) {
        do {
            // Look for next positive edge. Manual loop unrolling and branch hinting to produce faster code.
            if(__builtin_expect((p++)->value, 0) ||
               __builtin_expect((p++)->value, 0) ||
               __builtin_expect((p++)->value, 0) ||
               __builtin_expect((p++)->value, 0)) {
                break;
            }
        } while (endP > p);

        if (endP > p) {

#ifdef DEBUG_BBDECODE
            sequence[sequenceIndex++] = p - b;
#endif
            // A level of length n gets decoded to a sequence of bits of
            // the form 1000 with a length of (n+1) / 3 to account for 3x
            // oversampling.
            const int len = MAX((p - oldP + 1) / 3, 1);
            bits += len;
            value <<= len;
            value |= 1 << (len - 1);
            oldP = p;

            // Look for next zero edge. Manual loop unrolling and branch hinting to produce faster code.
            do {
                if (__builtin_expect(!(p++)->value, 0) ||
                    __builtin_expect(!(p++)->value, 0) ||
                    __builtin_expect(!(p++)->value, 0) ||
                    __builtin_expect(!(p++)->value, 0)) {
                    break;
                }
            } while (endP > p);

            if (endP > p) {

#ifdef DEBUG_BBDECODE
                sequence[sequenceIndex++] = p - b;
#endif
                // A level of length n gets decoded to a sequence of bits of
                // the form 1000 with a length of (n+1) / 3 to account for 3x
                // oversampling.
                const int len = MAX((p - oldP + 1) / 3, 1);
                bits += len;
                value <<= len;
                value |= 1 << (len - 1);
                oldP = p;
            }
        }
    }

    // length of last sequence has to be inferred since the last bit with inverted dshot is high
    if (bits < 18) {
        return DSHOT_TELEMETRY_NOEDGE;
    }

    // length of last sequence has to be inferred since the last bit with inverted dshot is high
    const int nlen = 21 - bits;
    if (nlen < 0) {
        return DSHOT_TELEMETRY_NOEDGE;
    }

    // Data appears valid
    if (startMargin < minMargin) {
        minMargin = startMargin;
    }

    if (cmpTimeUs(now, nextMarginCheckUs) >= 0) {
        nextMarginCheckUs += MARGIN_CHECK_INTERVAL_US;

        // Handle a skipped check
        if (nextMarginCheckUs < now) {
            nextMarginCheckUs = now + DSHOT_TELEMETRY_START_MARGIN;
        }

        if (minMargin > DSHOT_TELEMETRY_START_MARGIN) {
            preambleSkip = minMargin - DSHOT_TELEMETRY_START_MARGIN;
        } else {
            preambleSkip = 0;
        }

        minMargin = UINT32_MAX;
    }

#ifdef DEBUG_BBDECODE
    sequence[sequenceIndex] = sequence[sequenceIndex] + (nlen) * 3;
    sequenceIndex++;
#endif

    // The anticipated edges were observed
    if (nlen > 0) {
        value <<= nlen;
        value |= 1 << (nlen - 1);
    }

    return decode_bb_value(value, buffer, count, bit);
}

#else // USE_DSHOT_BITBAND

FAST_CODE uint32_t decode_bb( uint16_t buffer[], uint32_t count, uint32_t bit)
{
    timeUs_t now = micros();
#ifdef DEBUG_BBDECODE
    memset(sequence, 0, sizeof(sequence));
    sequenceIndex = 0;
#endif
    uint32_t mask = 1 << bit;

#ifdef DEBUG_BBDECODE
    uint32_t sequence[MAX_GCR_EDGES];
    memset(sequence, 0, sizeof(sequence));
    int sequenceIndex = 0;
#endif
    uint16_t lastValue = 0;
    uint32_t value = 0;

    uint16_t* p = buffer;
    uint16_t* endP = p + count - MIN_VALID_BBSAMPLES;

    // Jump forward in the buffer to just before where we anticipate the first zero
    p += preambleSkip;

    DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 3, preambleSkip);

    // Eliminate leading high signal level by looking for first zero bit in data stream.
    // Manual loop unrolling and branch hinting to produce faster code.
    while (p < endP) {
        if (__builtin_expect(!(*p++ & mask), 0) ||
            __builtin_expect(!(*p++ & mask), 0) ||
            __builtin_expect(!(*p++ & mask), 0) ||
            __builtin_expect(!(*p++ & mask), 0)) {
            break;
        }
    }

    const uint32_t startMargin = p - buffer;

    if (p >= endP) {
        // not returning telemetry is ok if the esc cpu is
        // overburdened.  in that case no edge will be found and
        // BB_NOEDGE indicates the condition to caller
        // Increase the start margin
        if (preambleSkip > 0) {
            // Increase the start margin
            preambleSkip--;
        }
        return DSHOT_TELEMETRY_NOEDGE;
    }

    const int remaining = MIN(count - startMargin, (unsigned int)MAX_VALID_BBSAMPLES);
    uint16_t* oldP = p;
    uint32_t bits = 0;
    endP = p + remaining;

#ifdef DEBUG_BBDECODE
    sequence[sequenceIndex++] = p - buffer;
#endif

    while (endP > p ) {
        // Look for next edge. Manual loop unrolling and branch hinting to produce faster code.
        if (__builtin_expect((*p++ & mask) != lastValue, 0) ||
            __builtin_expect((*p++ & mask) != lastValue, 0) ||
            __builtin_expect((*p++ & mask) != lastValue, 0) ||
            __builtin_expect((*p++ & mask) != lastValue, 0)) {
            if (endP > p) {
#ifdef DEBUG_BBDECODE
                sequence[sequenceIndex++] = p - buffer;
#endif
                // A level of length n gets decoded to a sequence of bits of
                // the form 1000 with a length of (n+1) / 3 to account for 3x
                // oversampling.
                const int len = MAX((p - oldP + 1) / 3, 1);
                bits += len;
                value <<= len;
                value |= 1 << (len - 1);
                oldP = p;
                lastValue = *(p-1) & mask;
            }
        }
    }

    // length of last sequence has to be inferred since the last bit with inverted dshot is high
    if (bits < 18) {
        return DSHOT_TELEMETRY_NOEDGE;
    }

    // length of last sequence has to be inferred since the last bit with inverted dshot is high
    const int nlen = 21 - bits;
    if (nlen < 0) {
        return DSHOT_TELEMETRY_NOEDGE;
    }

    // Data appears valid
    if (startMargin < minMargin) {
        minMargin = startMargin;
    }

    if (cmpTimeUs(now, nextMarginCheckUs) >= 0) {
        nextMarginCheckUs += MARGIN_CHECK_INTERVAL_US;

        // Handle a skipped check
        if (nextMarginCheckUs < now) {
            nextMarginCheckUs = now + DSHOT_TELEMETRY_START_MARGIN;
        }

        if (minMargin > DSHOT_TELEMETRY_START_MARGIN) {
            preambleSkip = minMargin - DSHOT_TELEMETRY_START_MARGIN;
        } else {
            preambleSkip = 0;
        }

        minMargin = UINT32_MAX;
    }

#ifdef DEBUG_BBDECODE
    sequence[sequenceIndex] = sequence[sequenceIndex] + (nlen) * 3;
    sequenceIndex++;
#endif

    // The anticipated edges were observed
    if (nlen > 0) {
        value <<= nlen;
        value |= 1 << (nlen - 1);
    }

    return decode_bb_value(value, buffer, count, bit);
}
#endif // USE_DSHOT_BITBAND

#endif
