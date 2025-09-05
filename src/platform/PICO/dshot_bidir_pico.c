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

typedef struct picoDshotTelemetryBuffer_s {
    union {
        uint32_t u32[2];
        uint16_t u16[4];
    };
} picoDshotTelemetryBuffer_t;

#define DSHOT_BIDIR_BIT_PERIOD 40

// Program for bidirectional ("inverted") DSHOT,
// 1 is low, 0 is high voltage
// Outgoing frame starts with transition from 0 to 1
// then 16x bits of 1 == 1 for ~2/3 period, 0 for ~1/3
//               or 0 == 1 for ~1/3 period, 0 for ~2/3
// Bit period is DSHOT_BIDIR_BIT_PERIOD (currently 40 cycles)
// Expect the return frame at 30 us after end of outgoing frame
//  TODO 30us independent of DSHOT 150, 300, 600,
//       so need different number of cycles delay for DSHOT 600 vs 300 vs 150
// Return frame is encoded as 21 bits (simple lo = 0, hi = 1), at 5/4 x outgoing bitrate
// (so return bit period should currently be 40 * 4 / 5 = 32 cycles)

// 1st bit of return frame is always 0, "idle" is 1, so after ~25 us could start waiting for transition
// to 0 - when we finish the out frame, we set pin to input (pindirs 0). Will that respect a previous
// pullup setting?
// see https://forums.raspberrypi.com/viewtopic.php?t=311659 

/*
comment on syncing (?) for return frame
https://github.com/betaflight/betaflight/pull/8554
ledvinap commented on Jul 16, 2019
A side note: I didn't analyze it too far, but is seems that 1.5x oversampling (1.5 synchronous samples per bit period) should be enough to handle NRZ protocol including clock drift...
    
joelucid commented on Jul 17, 2019 • 
Let me specify the new rpm telemetry protocol here for you @sskaug, and others:

Dshot bidir uses inverted signal levels (idle is 1). FC to ESC uses dshot frames but the lowest 4 bits hold the complement of the other nibbles xor'd together (normal dshot does not complement the xor sum). The ESC detects based on the inversion that telemetry packets have to be sent.

30us after receiving the dshot frame the esc responds with a telemetry frame. Logically the telemetry frame is a 16 bit value and the lowest 4 bits hold the complemented xor'd nibbles again.

The upper 12 bit contain the eperiod (1/erps) in the following bitwise encoding:

e e e m m m m m m m m m
The 9 bit value M needs to shifted left E times to get the period in micro seconds. This gives a range of 1 us to 65408 us. Which translates to a min e-frequency of 15.29 hz or for 14 pole motors 3.82 hz.

This 16 bit value is then GCR encoded to a 20 bit value by applying the following map nibble-wise:

0 -> 19
1 -> 1b
2 -> 12
3 -> 13
4 -> 1d
5 -> 15
6 -> 16
7 -> 17
8 -> 1a
9 -> 09
a -> 0a
b -> 0b
c -> 1e
d -> 0d
e -> 0e
f -> 0f 
This creates a 20 bit value which has no more than two consecutive zeros. This value is mapped to a new 21 bit value by starting with a bit value of 0 and changing the bit value in the next bit if the current bit in the incoming value is a 1, but repeating the previous bit value otherwise. Example:

1 0 1 1 1 0 0 1 1 0 would become 0 1 1 0 1 0 0 0 1 0 0.

This 21 bit value is then sent uninverted at a bitrate of 5/4 * the dshot bitrate. So Dshot 3 uses 375 kbit, dshot 600 uses 750 kbit.

The esc needs to be ready after 40us + one dshot frame length to receive the next dshot packet.

See https://en.wikipedia.org/wiki/Run-length_limited#GCR:_(0,2)_RLL for more details on the GCR encoding.

*/

bool dshot_program_bidir_init(PIO pio, uint sm, int offset, uint pin)
{
    bprintf("dshot_program_bidir_init on pin %d",pin);
#ifdef DSHOT_DEBUG_PIO
    pio_sm_config config = dshot_600_bidir_debug_program_get_default_config(offset);
#else
    pio_sm_config config = dshot_600_bidir_program_get_default_config(offset);
#endif

    sm_config_set_set_pins(&config, pin, 1);
    sm_config_set_in_pins(&config, pin);
    // don't need sm_config_set_jmp_pin (&config, pin);
    pio_gpio_init(pio, pin);
    bprintf("dshot bidir on pin %d done init PIO->gpio for pio",pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true); // set pin to output

    gpio_set_pulls(pin, true, false); // Pull up - idle 1 when awaiting bidir telemetry input (PIO pindirs 0).
    sm_config_set_out_shift(&config, PIO_SHIFT_LEFT, PIO_NO_AUTO_PUSHPULL, 32);
    sm_config_set_in_shift(&config, PIO_SHIFT_RIGHT, PIO_NO_AUTO_PUSHPULL, 32);

    float clocks_per_us = clock_get_hz(clk_sys) / 1000000;
#ifdef TEST_DSHOT_SLOW
    sm_config_set_clkdiv(&config, (1.0e4f + dshotGetPeriodTiming()) / DSHOT_BIDIR_BIT_PERIOD * clocks_per_us);
#else
    sm_config_set_clkdiv(&config, dshotGetPeriodTiming() / DSHOT_BIDIR_BIT_PERIOD * clocks_per_us);
#endif

    return PICO_OK == pio_sm_init(pio, sm, offset, &config);
}

#ifdef PICO_TRACE
// Verbose tracing for telemetry decode
#define TELEM_TRACE
#endif

static uint32_t decodeTelemetryRaw(int ind, const uint32_t first, const uint32_t second)
{
    // first and second words are lo-endian, with bottom 2 bits = junk (because PIO shifts right 30 data bits). Second word has MSBs.
    // Each bit was 3x sampled, with the start zero bit ignored, so we have 20 x 3 = 60 bits, 30 in first word, 30 in second.
    const uint8_t majorityVerdict[] = {0, 1, 1, 1, 1, 1, 1, 0}; // everything except 000 and 111 is decided by two to one
    const uint8_t bitDecode[] = {0, 0, 0, 1, 0, 1, 1, 1}; // e.g. bit pattern 011 -> [3] == 1
    const int8_t gcrs[] = {
        -1, -1, -1, -1, -1, -1, -1, -1,
        -1,  9, 10, 11, -1, 13, 14, 15,
        -1, -1,  2,  3, -1,  5,  6,  7,
        -1,  0,  8,  1, -1,  4, 12, -1
    };
#ifdef TELEM_TRACE
    static int tc;
#endif
    if (first == 0 && second == 0) {
#ifdef TELEM_TRACE
        if ((tc % 625000) < 4) {
            bprintf(" ===== [%d] motor %d decodeTelemetry %08x %08x", tc, ind, first, second);
        }
        tc++;
#else
        UNUSED(ind);
#endif
        // early return
        return DSHOT_TELEMETRY_INVALID;
    }

    uint64_t bits60 = first >> 2 | (((uint64_t)second >> 2) << 30);
    uint64_t bits60_save = bits60;
    uint32_t bits21 = 0; // NB implicitly the first of the 21 bits is zero.
    int majorityCount = 0;
    for (int i=0; i<20; ++i) {
        bits21 <<= 1;
        int pattern = bits60 & 7;
        majorityCount += majorityVerdict[pattern];
        bits21 += bitDecode[pattern];
        bits60 >>= 3;
    }

    uint32_t bits21_save = bits21;
    bits21 ^= bits21 >> 1;

#ifdef TELEM_TRACE
    bool tracesome = false;
    static uint32_t last1st[4];
    static uint32_t last2nd[4];
    if ((tc % 500000) < 4 && ( last1st[ind] != first || last2nd[ind] != second)) {
        tracesome = true;
        bprintf("       [%d] motor %d decodeTelemetry (prev. %08x %08x) %08x %08x", tc, ind, last1st[ind], last2nd[ind], first, second);
        last1st[ind] = first; last2nd[ind] = second;
        bprintf("bits60: %llx", bits60_save);
        bprintf("majority count: %d", majorityCount);
        bprintf("bits21: %x", bits21_save);
        bprintf("bits21 new: %x", bits21);
    }

    tc++;
#endif
#ifndef PICO_TRACE
    UNUSED(bits21_save);
    UNUSED(bits60_save);
#endif

    // Decode GCR (map 5 bit sequences back to 4 bits).
    uint16_t gcrDecode = 0;
    uint8_t checkSum = 0;
    for (int i=0; i<4; ++i) {
        gcrDecode >>= 4;
        int8_t gcrd = gcrs[bits21 & 0x1f];
        if (gcrd < 0) {
#ifdef PICO_TRACE
            static int badgcrs;
            if (badgcrs % 50000 < 4) {
                bprintf("\ndshot telem bad gcr [%d] for %x [%llx]", badgcrs, bits21_save, bits60_save);
            }
            badgcrs++;
#endif
            return DSHOT_TELEMETRY_INVALID;
        }

        checkSum ^= gcrd;
        gcrDecode |= gcrd << 12;
        bits21 >>= 5;
    }

    if ((checkSum & 0xf) != 0xf) {
        bprintf("\ncheckSum = %x (should be 0xf), for  %x [%llx]", checkSum, bits21_save, bits60_save);
        return DSHOT_TELEMETRY_INVALID;
    }

#ifdef TELEM_TRACE
    if (tracesome || (tc % 550000 < 4)) {
        bprintf("[%d] dshot telem (incl crc) %x", tc, (uint32_t)gcrDecode);
    }
#endif

    // Discard checksum and return telemetry number.
    return gcrDecode >> 4;
}

bool dshotTelemetryWait(void)
{
    bool telemetryWait = false;
#ifdef USE_DSHOT_TELEMETRY
    // Wait for telemetry reception to complete
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

    bprintf("dshotTelemetryWait returning %d",telemetryWait);
#endif
    return telemetryWait;
}

bool dshotDecodeTelemetry(void)
{
///////    bprintf("dshotDecodeTelemtry");
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
        if (fifo_words < 2) {
#ifdef PICO_TRACE
            static int notelem;
            if ((notelem % 250000) < 4) {
                bprintf("NOTELEM [%d] dshot motor %d, sm at pc-offset = %d, rx:%d, tx:%d",
                        notelem,
                        motorIndex,
                        pio_sm_get_pc(motor->pio, motor->pio_sm) - motor->offset,
                        pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm),
                        pio_sm_get_tx_fifo_level(motor->pio, motor->pio_sm)
                       );
            }
            notelem++;
#endif
            continue;
        }

        const uint32_t rawOne = pio_sm_get_blocking(motor->pio, motor->pio_sm) & ~3u; // bottom 2 bits are junk
        const uint32_t rawTwo = pio_sm_get_blocking(motor->pio, motor->pio_sm) & ~3u; // bottom 2 bits are junk
        if (fifo_words > 2) {
            // TODO drain fifo?
            bprintf("*** fifo_words: %d", fifo_words);
        }

        uint32_t rawValue = decodeTelemetryRaw(motorIndex, rawOne, rawTwo);

        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 0, debug[0] + 1);
        dshotTelemetryState.readCount++;

        if (rawValue != DSHOT_TELEMETRY_INVALID) {
            // Check EDT enable or store raw value
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
