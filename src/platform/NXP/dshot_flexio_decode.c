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

// Bit level arithmetic for the FlexIO DShot driver; no registers, no
// Betaflight headers.

#include <stdbool.h>
#include <stdint.h>

#include "dshot_flexio.h"

// Same sections as FAST_CODE and FAST_RODATA in include/platform/platform.h.
#ifdef __arm__
#define DECODE_FAST_CODE __attribute__((section(".itcm_text")))
#define DECODE_FAST_RODATA __attribute__((section(".dtcm_rodata")))
#else
#define DECODE_FAST_CODE
#define DECODE_FAST_RODATA
#endif

// Timer compare value, dual 8 bit baud mode. The reference manual splits the 16
// bit compare in two: the low byte gives a shift clock of the FlexIO clock over
// (CMP[7:0] + 1) * 2, and the high byte runs the timer for (CMP[15:8] + 1) / 2
// shift clocks, so it holds twice the number of bits less one.
#define TIMCMP_BITS(bits) ((uint32_t)((((bits) * 2u) - 1u) << 8))
#define TIMCMP_DIVIDER(d) ((uint32_t)((d) & 0xffu))

// Three shift clock ticks per DShot bit, 24 ticks per shifter load.
#define DSHOT_TICKS_PER_BIT 3u
#define DSHOT_TX_WORD_BITS  24u

// The ESC replies at 5/4 of the DShot bit rate with one tick per bit, 21 bits
// in one shifter load.
#define DSHOT_RX_WORD_BITS  21u

uint32_t dshotFlexioTxTimerCompare(uint32_t flexioClockHz, uint32_t bitRateHz)
{
    const uint32_t shiftClockHz = bitRateHz * DSHOT_TICKS_PER_BIT;

    return TIMCMP_BITS(DSHOT_TX_WORD_BITS)
         | TIMCMP_DIVIDER(flexioClockHz / shiftClockHz / 2u - 1u);
}

// Low byte of the receive compare value; offset zero is one count slow.
static uint32_t dshotFlexioRxDivider(uint32_t flexioClockHz, uint32_t bitRateHz, int offset)
{
    const uint32_t shiftClockHz = bitRateHz * 5u / 4u;

    return flexioClockHz / shiftClockHz / 2u + (uint32_t)offset;
}

uint32_t dshotFlexioRxTimerCompare(uint32_t flexioClockHz, uint32_t bitRateHz, int offset)
{
    return TIMCMP_BITS(DSHOT_RX_WORD_BITS)
         | TIMCMP_DIVIDER(dshotFlexioRxDivider(flexioClockHz, bitRateHz, offset));
}

bool dshotFlexioRxDividerFits(uint32_t flexioClockHz, uint32_t bitRateHz)
{
    // At 108 MHz DShot600 needs 72, DShot300 144 and DShot150 288.
    const uint32_t divider = dshotFlexioRxDivider(flexioClockHz, bitRateHz, 0);

    return divider >= (uint32_t)-DSHOT_FLEXIO_TCMP_OFFSET_MIN
        && divider + DSHOT_FLEXIO_TCMP_OFFSET_MAX <= 0xffu;
}

// A one is ticks 0b011 and a zero 0b001; the shifter sends the low bit first,
// so each entry holds a nibble's twelve ticks with its top bit lowest.
static DECODE_FAST_RODATA const uint16_t dshotTicksByNibble[16] = {
    0x249, 0x649, 0x2c9, 0x6c9, 0x259, 0x659, 0x2d9, 0x6d9,
    0x24b, 0x64b, 0x2cb, 0x6cb, 0x25b, 0x65b, 0x2db, 0x6db,
};

DECODE_FAST_CODE uint64_t dshotFlexioExpandFrame(uint16_t packet)
{
    return (uint64_t)dshotTicksByNibble[(packet >> 12) & 0xfu]
         | ((uint64_t)dshotTicksByNibble[(packet >> 8) & 0xfu] << 12)
         | ((uint64_t)dshotTicksByNibble[(packet >> 4) & 0xfu] << 24)
         | ((uint64_t)dshotTicksByNibble[packet & 0xfu] << 36);
}

// GCR symbol to nibble; unused symbols mean the frame was misread.
#define GCR_INVALID 0xffu

static DECODE_FAST_RODATA const uint8_t gcrDecode[32] = {
    GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID,
    GCR_INVALID, GCR_INVALID, GCR_INVALID, GCR_INVALID,
    GCR_INVALID, 0x9,         0xa,         0xb,
    GCR_INVALID, 0xd,         0xe,         0xf,
    GCR_INVALID, GCR_INVALID, 0x2,         0x3,
    GCR_INVALID, 0x5,         0x6,         0x7,
    GCR_INVALID, 0x0,         0x8,         0x1,
    GCR_INVALID, 0x4,         0xc,         GCR_INVALID
};

// Undo the active low sampling and keep the 20 bits of the reply.
static uint32_t shifterWordToLine(uint32_t shifterWord)
{
    return ~shifterWord & 0xfffffu;
}

DECODE_FAST_CODE uint32_t dshotFlexioDecodeTelemetry(uint32_t shifterWord)
{
    uint32_t value = shifterWordToLine(shifterWord);

    // The reply always ends in a one; anything else did not line up.
    if ((value & 0x1u) == 0) {
        return DSHOT_FLEXIO_DECODE_INVALID;
    }

    // Undo the run length limiting the ESC applies on top of the GCR symbols.
    value ^= value >> 1;

    uint32_t data = 0;
    for (unsigned nibble = 0; nibble < 4; nibble++) {
        const uint8_t decoded = gcrDecode[(value >> (nibble * 5u)) & 0x1fu];
        if (decoded == GCR_INVALID) {
            return DSHOT_FLEXIO_DECODE_INVALID;
        }
        data |= (uint32_t)decoded << (nibble * 4u);
    }

    // The low nibble is an xor checksum over the other three, inverted.
    uint32_t checksum = data;
    checksum ^= checksum >> 8;
    checksum ^= checksum >> 4;

    if ((checksum & 0xfu) != 0xfu) {
        return DSHOT_FLEXIO_DECODE_INVALID;
    }

    // The 12 bit telemetry value; src/main/drivers/dshot.c turns it into eRPM.
    return (data >> 4) & 0xfffu;
}

// Offsets run from DSHOT_FLEXIO_TCMP_OFFSET_MIN upwards.
#define TRAIN_MASK_BIT(offset) (1u << ((offset) - DSHOT_FLEXIO_TCMP_OFFSET_MIN))

void dshotFlexioTrainReset(dshotFlexioTraining_t *training)
{
    training->passMask = 0;
    training->frames = 0;
    training->passes = 0;
    training->offset = DSHOT_FLEXIO_TCMP_OFFSET_MIN;
    training->done = false;
}

DECODE_FAST_CODE bool dshotFlexioTrainStep(dshotFlexioTraining_t *training, uint32_t shifterWord)
{
    if (dshotFlexioDecodeTelemetry(shifterWord) != DSHOT_FLEXIO_DECODE_INVALID) {
        training->passes++;
    } else if ((shifterWordToLine(shifterWord) & 0x1u) == 0) {
        // A frame not ending in a one rules this offset out, so end the round.
        training->frames = DSHOT_FLEXIO_TRAIN_FRAMES - 1;
    }

    training->frames++;

    if (training->frames < DSHOT_FLEXIO_TRAIN_FRAMES) {
        return false;
    }

    if (training->passes >= DSHOT_FLEXIO_TRAIN_PASSES) {
        training->passMask |= TRAIN_MASK_BIT(training->offset);
    }

    training->frames = 0;
    training->passes = 0;
    training->offset++;

    if (training->offset > DSHOT_FLEXIO_TCMP_OFFSET_MAX) {
        if (training->passMask == 0) {
            // Nothing worked, so sweep again from the start.
            training->offset = DSHOT_FLEXIO_TCMP_OFFSET_MIN;
        } else {
            // Settle on the middle of the range that worked.
            const int low = __builtin_ctz(training->passMask);
            const int high = 31 - __builtin_clz(training->passMask);
            training->offset = (int8_t)(((low + high) / 2) + DSHOT_FLEXIO_TCMP_OFFSET_MIN);
            training->done = true;
        }
    }

    return true;
}
