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

#ifdef USE_DSHOT

#include "build/debug.h"
#include "build/debug_pin.h"

#include "drivers/io.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/dshot_command.h"
#include "drivers/motor_types.h"

#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"

////////FAST_DATA_ZERO_INIT timeUs_t dshotFrameUs;
//static FAST_DATA_ZERO_INIT timeUs_t lastSendUs;

#ifdef USE_DSHOT_TELEMETRY
// Maximum time to wait for telemetry reception to complete
#define DSHOT_TELEMETRY_TIMEOUT 2000

// TODO use with TELEMETRY for cli report (or not)
FAST_DATA_ZERO_INIT dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;
#endif

typedef struct motorOutput_s {
    int offset;              // NB current code => offset same for all motors
    dshotProtocolControl_t protocolControl;
    int pinIndex;            // pinIndex of this motor output within a group that bbPort points to
    IO_t io;                 // IO_t for this output
    bool configured;
    bool enabled;
    PIO pio;
    uint16_t pio_sm;
} motorOutput_t;

typedef struct picoDshotTelemetryBuffer_s {
    union {
        uint32_t u32[2];
        uint16_t u16[4];
    };
} picoDshotTelemetryBuffer_t;

static motorProtocolTypes_e motorProtocol;
//uint8_t dshotMotorCount = 0;
static motorOutput_t dshotMotors[MAX_SUPPORTED_MOTORS];
bool useDshotTelemetry = false;

static float getPeriodTiming(void)
{
    switch(motorProtocol) {
        case MOTOR_PROTOCOL_DSHOT600:
            return 1.67f;
        case MOTOR_PROTOCOL_DSHOT300:
            return 3.33f;
        default:
        case MOTOR_PROTOCOL_DSHOT150:
            return 6.66f;
    }
}

#define DSHOT_BIT_PERIOD 40

#ifdef USE_DSHOT_TELEMETRY
// Program for bidirectional ("inverted") DSHOT,
// 1 is low, 0 is high voltage
// Outgoing frame starts with transition from 0 to 1
// then 16x bits of 1 == 1 for ~2/3 period, 0 for ~1/3
//               or 0 == 1 for ~1/3 period, 0 for ~2/3
// Bit period is DSHOT_BIT_PERIOD (currently 40 cycles)
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

static const uint16_t dshot_bidir_PIO_instructions[] = {
            //     .wrap_target
    0xff81, //  0: set    pindirs, 1             [31]      // min delay... (TODO check, and is this correct for dshot != 600?
    0xff01, //  1: set    pins, 1                [31]
    0x80a0, //  2: pull   block                            // Block until someone puts something into the TX FIFO
    0x6050, //  3: out    y, 16                            // discard top 16 bits (can lose this if 16-bit write? but then would have to count 16)
    0x00e6, //  4: jmp    !osre, 6                         // If not done output then -> 6
    0x000e, //  5: jmp    14
    0x6041, //  6: out    y, 1                             // next bit -> into y
    0x006b, //  7: jmp    !y, 11                           // 0 bit to output -> 11
    0xfd00, //  8: set    pins, 0                [29]      // 1 bit to output. Inverted Dshot => 0 for 30, 1 for 10
    0xe501, //  9: set    pins, 1                [5]
    0x0004, // 10: jmp    4
    0xee00, // 11: set    pins, 0                [14]      // 0 bit to output. Inverted Dshot => 0 for 15, 1 for 25
    0xf401, // 12: set    pins, 1                [20]
    0x0004, // 13: jmp    4
    0xe055, // 14: set    y, 21                            // after end of output frame, wait ~30us for start of input which should be transition to 0
    0x1f8f, // 15: jmp    y--, 15                [31]      // wait 32*21 = 672 cycles = 16.8 bit periods ~ 28us at DSHOT600 rate
    0xe080, // 16: set    pindirs, 0
    0xe05f, // 17: set    y, 31
    0xa142, // 18: nop                           [1]
    0x0076, // 19: jmp    !y, 22
    0x0095, // 20: jmp    y--, 21                             // TODO dead code?
    0x00d2, // 21: jmp    pin, 18                             // TODO dead code?
    0xe05e, // 22: set    y, 30
    0x4901, // 23: in     pins, 1                [9]
    0x0097, // 24: jmp    y--, 23                               // retrive 31 bits? every 11 cycles (in bits are at 40*4/5 = 32 cycles for dshot600)
    0x4801, // 25: in     pins, 1                [8]            // retrieve 32nd bit TODO we want 21 (or 20 given leading 0) - are we ~3x sampling?
////////////    0x8020, // 26: push   block
    0x8000, // 26: push   [not block]
    0xe05f, // 27: set    y, 31
    0x4a01, // 28: in     pins, 1                [10]           // retrieve another 32 bits at 11 cycles
    0x009c, // 29: jmp    y--, 28
////////////    0x8020, // 26: push   block
    0x8000, // 26: push   [not block]
//    0x8020, // 30: push   block        <---- problem if not expecting telemetry...?
    0x0000, // 31: jmp    0                                   // can remove this one with wrap
            //     .wrap
};

static const struct pio_program dshot_bidir_program = {
    .instructions = dshot_bidir_PIO_instructions,
    .length = ARRAYLEN(dshot_bidir_PIO_instructions),
    .origin = -1,
};

static void dshot_program_bidir_init(PIO pio, uint sm, int offset, uint pin)
{
    pio_sm_config config = pio_get_default_sm_config();

    // Wrap to = offset + 0, Wrap after instruction = offset + length - 1
    sm_config_set_wrap(&config, offset, offset + ARRAYLEN(dshot_bidir_PIO_instructions) - 1);

    sm_config_set_set_pins(&config, pin, 1);
    sm_config_set_in_pins(&config, pin);
    sm_config_set_jmp_pin (&config, pin);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true); // set pin to output

    gpio_set_pulls(pin, true, false); // Pull up - idle 1 when awaiting bidir telemetry input (PIO pindirs 0).


// TODO ** autopull == false?
    // TODO ** check shiftright = false for dshot program
    // recall we aren't outputting the pulled byte, we are using it for timing of transition of output level
    // from 0 back to 1
    // remember also that DSHOT is effectively hi-endian w.r.t. time (so the timeline looks lo-endian
    // in the usual way of picturing memory) -> so shift left

    // TODO pull again at 16 bits, then don't need to discard 16 bits in PIO program
    sm_config_set_out_shift(&config, false, false, 32); /////ARRAYLEN(dshot_bidir_PIO_instructions));
// TODO ** autopush == false?
    sm_config_set_in_shift(&config, false, false, ARRAYLEN(dshot_bidir_PIO_instructions));

    float clocks_per_us = clock_get_hz(clk_sys) / 1000000;
#ifdef TEST_DSHOT_SLOW
    sm_config_set_clkdiv(&config, (1.0e4f + getPeriodTiming()) / DSHOT_BIT_PERIOD * clocks_per_us);
#else
    sm_config_set_clkdiv(&config, getPeriodTiming() / DSHOT_BIT_PERIOD * clocks_per_us);
#endif

    pio_sm_init(pio, sm, offset, &config);
}

#else

static const uint16_t dshot_600_program_instructions[] = {
            //     .wrap_target
    0xff00, //  0: set    pins, 0                [31]
    0xb442, //  1: nop                           [20]
    0x80a0, //  2: pull   block
    0x6050, //  3: out    y, 16
    0x6041, //  4: out    y, 1
    0x006a, //  5: jmp    !y, 10
    0xfd01, //  6: set    pins, 1                [29]
    0xe600, //  7: set    pins, 0                [6]
    0x00e4, //  8: jmp    !osre, 4
    0x0000, //  9: jmp    0
    0xee01, // 10: set    pins, 1                [14]
    0xf400, // 11: set    pins, 0                [20]
    0x01e4, // 12: jmp    !osre, 4               [1]
            //     .wrap
};

static const struct pio_program dshot_600_program = {
    .instructions = dshot_600_program_instructions,
    .length = ARRAYLEN(dshot_600_program_instructions),
    .origin = -1,
};

// TODO DSHOT150, 300
static void dshot_program_init(PIO pio, uint sm, int offset, uint pin)
{
    pio_sm_config config = pio_get_default_sm_config();

    // Wrap to = offset + 0, Wrap after instruction = offset + length - 1
    sm_config_set_wrap(&config, offset, offset + ARRAYLEN(dshot_600_program_instructions) - 1);

    sm_config_set_set_pins(&config, pin, 1);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true); // set pin to output
    gpio_set_pulls(pin, false, true); // Pull down - idle low when awaiting next frame to output.

    // TODO pull again at 16 bits, then don't need to discard 16 bits in PIO program
    // write 16-bits -> duplicates (TBC)
    sm_config_set_out_shift(&config, false, false, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

    float clocks_per_us = clock_get_hz(clk_sys) / 1000000;
#ifdef TEST_DSHOT_SLOW
    sm_config_set_clkdiv(&config, (1.0e4f + getPeriodTiming()) / DSHOT_BIT_PERIOD * clocks_per_us);
#else
    sm_config_set_clkdiv(&config, getPeriodTiming() / DSHOT_BIT_PERIOD * clocks_per_us);
#endif

    pio_sm_init(pio, sm, offset, &config);
}

#endif


/*
TODO what actually finally fixed this? was it removing gpio_set_dir (unlikely) or
    the whole IOConfigGPIO (which needs rethinking)?
    TODO is Dshot writing async? (what about telemetry?)
*/


#ifdef USE_DSHOT_TELEMETRY
static uint32_t decodeTelemetry(const uint32_t first, const uint32_t second)
{
    UNUSED(first);
    UNUSED(second);
    return DSHOT_TELEMETRY_INVALID;
}
#endif

static bool dshotTelemetryWait(void)
{
    bool telemetryWait = false;
#ifdef USE_DSHOT_TELEMETRY
    // Wait for telemetry reception to complete
    bool telemetryPending;
    const timeUs_t startTimeUs = micros();

    do {
        telemetryPending = false;
        for (unsigned motorIndex = 0; motorIndex < dshotMotorCount && !telemetryPending; motorIndex++) {
            const motorOutput_t *motor = &dshotMotors[motorIndex];
            const int fifo_words = pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm);
            telemetryPending |= fifo_words >= 2; // TODO Current program outputs two words. Should this be <2 ?
        }

        telemetryWait |= telemetryPending;

        if (cmpTimeUs(micros(), startTimeUs) > DSHOT_TELEMETRY_TIMEOUT) {
            break;
        }
    } while (telemetryPending); // TODO logic here? supposed to block until all telemetry available?

    if (telemetryWait) {
        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 2, debug[2] + 1);
    }

#endif
    return telemetryWait; // TODO what should the return value be? (don't think it's used)
}

static void dshotUpdateInit(void)
{
    // NO OP as no buffer needed for PIO
}

static bool dshotDecodeTelemetry(void)
{
#ifdef USE_DSHOT_TELEMETRY
    if (!useDshotTelemetry) {
        return true;
    }

#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = millis();
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
        const motorOutput_t *motor = &dshotMotors[motorIndex];

        if (dshotTelemetryState.rawValueState == DSHOT_RAW_VALUE_STATE_NOT_PROCESSED) {
            int fifo_words = pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm);
            if (fifo_words < 2) {
                continue;
            }

            const uint32_t rawOne = pio_sm_get_blocking(motor->pio, motor->pio_sm);
            const uint32_t rawTwo = pio_sm_get_blocking(motor->pio, motor->pio_sm);

            uint32_t rawValue = decodeTelemetry(rawOne, rawTwo);

            DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 0, debug[0] + 1);
            dshotTelemetryState.readCount++;

            if (rawValue != DSHOT_TELEMETRY_INVALID) {
                // Check EDT enable or store raw value
                if ((rawValue == 0x0E00) && (dshotCommandGetCurrent(motorIndex) == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE)) {
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
    }
#endif

    return true;
}

static void dshotWriteInt(uint8_t motorIndex, uint16_t value)
{
    motorOutput_t *const motor = &dshotMotors[motorIndex];

    if (!motor->configured) {
        return;
    }

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(motorIndex);
        if (value) {
            // Request telemetry on other return wire (this isn't DSHOT bidir)
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;

    uint16_t packet = prepareDshotPacket(&motor->protocolControl);

// TODO what to do if TX buffer is full? (unlikely at standard scheduler loop rates?)
#ifdef TEST_DSHOT_ETC
    // for testing, can be convenient to block, not lose any writes
    pio_sm_put_blocking(motor->pio, motor->pio_sm, packet);
////    pio_sm_put(motor->pio, motor->pio_sm, packet);
#else
    pio_sm_put(motor->pio, motor->pio_sm, packet);
#endif
}

static void dshotWrite(uint8_t motorIndex, float value)
{
    dshotWriteInt(motorIndex, lrintf(value));
}

static void dshotUpdateComplete(void)
{
    // TODO: anything here?
    // aargh, dshotCommandQueueEmpty is a query, but dshotCommandOutputIsEnabled has side-effects...

    // If there is a dshot command loaded up, time it correctly with motor update
    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(dshotMotorCount)) {
            return;
        }
    }
}

static bool dshotEnableMotors(void)
{
    for (int i = 0; i < dshotMotorCount; i++) {
        const motorOutput_t *motor = &dshotMotors[i];
        if (motor->configured) {
            // TODO is this valid on a PIO pin
            // no, it only affects the pin when configured as "SIO"
            // gpio_set_dir(ioPin, (cfg & 0x01)); // 0 = in, 1 = out
            ///// not this now it inits the pin... IOConfigGPIO(motor->io, 0);

            // if we want to set the direction in a PIO mode, we do
            // pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true); // set pin to output
        }
    }
    return true;
}

static void dshotDisableMotors(void)
{
    // TODO: implement?
    return;
}

#ifdef TEST_DSHOT_ETC
void dshotTestWrites(void)
{
//    const int mnum = 0;
    const int mval = DSHOT_MIN_THROTTLE;
    for (int ii = mval; ii< DSHOT_MAX_THROTTLE; ii += 123) {
//        dshotWriteInt(mnum, ii);
        dshotWriteInt(0, ii);
        dshotWriteInt(1, ii);
        dshotWriteInt(2, ii);
        dshotWriteInt(3, ii);
    }
}
#endif

static void dshotShutdown(void)
{
    // TODO: implement?
    return;
}

static bool dshotIsMotorEnabled(unsigned index)
{
    return dshotMotors[index].enabled;
}

static void dshotPostInit(void)
{
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
        dshotMotors[motorIndex].enabled = true;
    }
}

static bool dshotIsMotorIdle(unsigned motorIndex)
{
    if (motorIndex >= ARRAYLEN(dshotMotors)) {
        return false;
    }
    return dshotMotors[motorIndex].protocolControl.value == 0;
}

static void dshotRequestTelemetry(unsigned index)
{
#ifndef USE_DSHOT_TELEMETRY
    UNUSED(index);
#else
    if (index < dshotMotorCount) {
        dshotMotors[index].protocolControl.requestTelemetry = true;
    }
#endif
}

static motorVTable_t dshotVTable = {
    .postInit = dshotPostInit,
    .enable = dshotEnableMotors,
    .disable = dshotDisableMotors,
    .isMotorEnabled = dshotIsMotorEnabled,
    .telemetryWait = dshotTelemetryWait,
    .decodeTelemetry = dshotDecodeTelemetry,
    .updateInit = dshotUpdateInit,
    .write = dshotWrite,
    .writeInt = dshotWriteInt,
    .updateComplete = dshotUpdateComplete,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .shutdown = dshotShutdown,
    .isMotorIdle = dshotIsMotorIdle,
    .requestTelemetry = dshotRequestTelemetry,
};

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    // Return false if not enough motors initialised for the mixer or a break in the motors or issue with PIO
    dbgPinLo(0);
    dbgPinLo(1);

    device->vTable = NULL; // Only set vTable if initialisation is succesful (TODO: check)
    dshotMotorCount = 0;   // Only set dshotMotorCount ble if initialisation is succesful (TODO: check)

    uint8_t motorCountProvisional = device->count;
    if (motorCountProvisional > 4) {
        // Currently support 4 motors with one PIO block, four state machines
        // TODO (possible future) support more than 4 motors
        // (by reconfiguring state machines perhaps? batching if required)
        return false;
    }

    motorProtocol = motorConfig->motorProtocol;

#ifdef USE_DSHOT_TELEMETRY
    useDshotTelemetry = motorConfig->useDshotTelemetry;
#endif

    // TODO somehow configure which PIO block to use for dshot? pio0 for now.
    const PIO pio = pio0;

#ifdef TEST_DSHOT_ETC
    pio_set_gpio_base(pio, 16);
#endif

    // Use one program for all motors.
    // NB the PIO block is limited to 32 instructions (shared across 4 state machines)
#ifdef USE_DSHOT_TELEMETRY
    int offset = pio_add_program(pio, &dshot_bidir_program);
#else
    int offset = pio_add_program(pio, &dshot_600_program);
#endif
    if (offset < 0) {
        /* error loading PIO */
        return false;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCountProvisional; motorIndex++) {
        IO_t io = IOGetByTag(motorConfig->ioTags[motorIndex]);
        if (!IOIsFreeOrPreinit(io)) {
            return false;
        }

        // TODO: might make use of pio_claim_free_sm_and_add_program_for_gpio_range
        // -> automatically sets the GPIO base if we might be using pins in 32..47
        const int pio_sm = pio_claim_unused_sm(pio, false);

        if (pio_sm < 0) {
            return false;
        }

        int pinIndex = DEFIO_TAG_PIN(motorConfig->ioTags[motorIndex]);

        // TODO: take account of motor reordering,
        // cf. versions of  pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorProtocolTypes_e pwmProtocolType, uint8_t output)
        dshotMotors[motorIndex].pinIndex = pinIndex;
        dshotMotors[motorIndex].io = io;
        dshotMotors[motorIndex].pio = pio;
        dshotMotors[motorIndex].pio_sm = pio_sm;
        dshotMotors[motorIndex].offset = offset;

        IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        // uint8_t iocfg = 0;
        // IOConfigGPIO(io, iocfg); // TODO: don't need this? assigned and dir in program init

#ifdef USE_DSHOT_TELEMETRY
        dshot_program_bidir_init(pio, pio_sm, dshotMotors[motorIndex].offset, pinIndex);
#else
        dshot_program_init(pio, pio_sm, dshotMotors[motorIndex].offset, pinIndex);
#endif

        // TODO pio_sm_set_enabled for pio_sm of dshotMotors[0..3] maybe
        // better in vTable functions dshotEnable/DisableMotors
        pio_sm_set_enabled(pio, pio_sm, true);

        dshotMotors[motorIndex].configured = true;

    }

    device->vTable = &dshotVTable;
    dshotMotorCount = motorCountProvisional;
    return true;
}

// TODO do we just undef USE_DSHOT_BITBANG ?
bool isDshotBitbangActive(const motorDevConfig_t *motorDevConfig)
{
    /* DSHOT BIT BANG not required for PICO */
    UNUSED(motorDevConfig);
    return false;
}

bool dshotBitbangDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    // TODO: not required
    UNUSED(device);
    UNUSED(motorConfig);
    return false;
}

dshotBitbangStatus_e dshotBitbangGetStatus(void)
{
    // TODO: not required
    return 0;
}

#endif // USE_DSHOT
