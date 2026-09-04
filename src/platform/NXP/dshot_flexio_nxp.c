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

/*
 * FlexIO based DShot for the NXP parts. One motor uses one shifter and the timer with
 * the same number; a 48 tick frame is sent as two 24 tick shifter loads, both
 * written by the trigger. The bit arithmetic lives in dshot_flexio_decode.c.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "build/atomic.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/io.h"
#include "drivers/motor_types.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/resource.h"
#include "drivers/time.h"

#include "pg/motor.h"

#include "platform/nxp_clock.h"

#include "dshot_flexio.h"
#include "iomux_nxp.h"
#include "iomux_signal_def.h"

// Shared DShot state that generic code reads directly.
bool useDshotTelemetry = false;

#ifdef USE_DSHOT_TELEMETRY
dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;
#endif

STATIC_ASSERT(FLEXIO_SHIFTCTL_COUNT >= FLEXIO_DSHOT_CHANNELS, flexio_has_too_few_shifters);
STATIC_ASSERT(FLEXIO_TIMCTL_COUNT >= FLEXIO_DSHOT_CHANNELS, flexio_has_too_few_timers);

// dshotReceivedMask holds one bit per motor in a uint32_t.
STATIC_ASSERT(MAX_SUPPORTED_MOTORS <= 32, too_many_motors_for_a_32_bit_mask);

// The training pass mask holds one bit per swept offset, in a uint32_t as well.
STATIC_ASSERT(DSHOT_FLEXIO_TCMP_OFFSET_MAX - DSHOT_FLEXIO_TCMP_OFFSET_MIN < 32,
              too_many_offsets_for_the_training_pass_mask);

// Register field values from the reference manual.
#define SHIFTER_MODE_RECEIVE            1u
#define SHIFTER_MODE_TRANSMIT           2u

#define SHIFTER_INPUT_FROM_PIN          0u
#define SHIFTER_TIMER_POLARITY_POSEDGE  0u

#define SHIFTER_START_DISABLED_LOAD_ON_ENABLE 0u
#define SHIFTER_START_DISABLED_LOAD_ON_SHIFT  1u
#define SHIFTER_STOP_DISABLED           0u
#define SHIFTER_STOP_LOW                2u

#define PIN_CONFIG_OUTPUT_DISABLED      0u
#define PIN_CONFIG_OUTPUT               3u
#define PIN_ACTIVE_LOW                  1u

#define TIMER_MODE_DUAL_8BIT_BAUD       1u

#define TIMER_OUT_ONE_NOT_RESET         0u
#define TIMER_OUT_ZERO_ON_RESET         3u
#define TIMER_DEC_ON_FLEXIO_CLOCK       0u
#define TIMER_RESET_NEVER               0u
#define TIMER_RESET_ON_PIN_RISING_EDGE  4u
#define TIMER_DISABLE_ON_COMPARE        2u
#define TIMER_ENABLE_ON_TRIGGER_HIGH    2u
#define TIMER_ENABLE_ON_TRIGGER_EITHER  7u
#define TIMER_STOP_DISABLED             0u
#define TIMER_STOP_ON_TIMER_DISABLE     2u
#define TIMER_START_DISABLED            0u
#define TIMER_START_ENABLED             1u

#define TIMER_TRIGGER_POLARITY_HIGH     0u
#define TIMER_TRIGGER_POLARITY_LOW      1u
#define TIMER_TRIGGER_SOURCE_INTERNAL   1u

// Internal trigger select table: shifter N status flag is 4 * N + 1, FlexIO pin
// N is 2 * N. Both are numbered inside one instance, so a shifter can only be
// triggered from its own instance.
#define TRIGGER_SHIFTER_STATUS(n)       ((4u * (n)) + 1u)
#define TRIGGER_FLEXIO_PIN(n)           (2u * (n))

// Status flags are write one to clear; flags not written keep standing.
static inline void flexioClearTimerFlags(FLEXIO_Type *base, uint32_t mask)
{
    base->TIMSTAT = mask;
}

static inline void flexioClearShifterFlags(FLEXIO_Type *base, uint32_t mask)
{
    base->SHIFTSTAT = mask;
}

// The FlexIO instances motors may use, in the order they are tried.
typedef struct flexioHardware_s {
    FLEXIO_Type *base;
    IRQn_Type irq;
    nxpClock_e clock;
} flexioHardware_t;

static const flexioHardware_t flexioHardware[] = {
    { FLEXIO1, FLEXIO1_IRQn, NXP_CLK_FLEXIO1 },
    { FLEXIO2, FLEXIO2_IRQn, NXP_CLK_FLEXIO2 },
};

#define FLEXIO_DSHOT_INSTANCES  ARRAYLEN(flexioHardware)

// FlexIO instances are numbered from one, the arrays here from zero.
#define FLEXIO_INSTANCE_NUMBER(index)   ((unsigned)(index) + 1u)

// The instances DShot may use and the root clock they share.
#define FLEXIO_DSHOT_USABLE(index)      true
#define FLEXIO_DSHOT_CLOCK              NXP_CLK_FLEXIO1

// Where a channel is in the frame.
typedef enum {
    DSHOT_FLEXIO_IDLE = 0,
    DSHOT_FLEXIO_FIRST_HALF_QUEUED,
    DSHOT_FLEXIO_SECOND_HALF_QUEUED,
    DSHOT_FLEXIO_FIRST_HALF_SENT,
    DSHOT_FLEXIO_SENT,
    DSHOT_FLEXIO_RECEIVING,
    DSHOT_FLEXIO_RECEIVED,
} dshotFlexioState_e;

typedef struct dshotFlexioMotor_s {
    IO_t io;

    // Shifter and timer number, counted from zero within the instance.
    FLEXIO_Type *base;
    const nxpPinMux_t *mux;               // the pad's FlexIO muxing, to put back after ESC passthrough
    uint8_t instanceIndex;
    uint8_t shifter;
    uint8_t flexioPin;
    uint32_t shifterBit;                    // 1 << shifter, for the status registers
    uint32_t motorBit;                      // 1 << motor index, for dshotReceivedMask
    bool configured;

    dshotProtocolControl_t protocolControl;
    uint32_t firstHalf;                     // lowest 24 bits of the expanded frame
    uint32_t secondHalf;                    // highest 24 bits, written behind the first
    bool pending;                           // a frame is waiting for the next trigger

    volatile dshotFlexioState_e state;
    volatile uint32_t rawResponse;

    dshotFlexioTraining_t training;
    uint32_t receiveCompare;                // TIMCMP value used while receiving
} dshotFlexioMotor_t;

// What one FlexIO instance is being used for; the interrupt reads it.
typedef struct flexioInstance_s {
    FLEXIO_Type *base;                      // NULL until a motor lands here
    uint32_t channelMask;                   // shifters and timers in use
    uint8_t inUse;                          // shifters handed out so far
    dshotFlexioMotor_t *shifterMotor[FLEXIO_DSHOT_CHANNELS];
} flexioInstance_t;

static dshotFlexioMotor_t dshotFlexioMotors[MAX_SUPPORTED_MOTORS];
static flexioInstance_t flexioInstances[FLEXIO_DSHOT_INSTANCES];

// The instances that ended up with motors on them.
static flexioInstance_t *flexioActive[FLEXIO_DSHOT_INSTANCES];
static uint8_t flexioActiveCount;

static volatile uint32_t dshotReceivedMask; // motors with a reply since the last trigger
static uint32_t dshotTransmitCompare;       // TIMCMP value used while transmitting
static uint32_t dshotBitRateHz;

// Frame plus reply window in DShot bit periods, the window STM32 bitbang captures.
#define DSHOT_FLEXIO_EXCHANGE_BITS 54u

static uint32_t dshotSentMask;              // motors whose frame went out on the last trigger
static timeUs_t dshotSentAtUs;              // when that trigger was
static timeDelta_t dshotExchangeUs;         // frame start to the end of the latest reply

static bool dshotFrameDeferred;             // this loop leaves the last exchange on the wire
static uint32_t dshotFramesDeferred;        // loops that sent no frame for that reason
static timeDelta_t dshotFrameDeferEarlyMaxUs; // the most such a loop was early by

// Set a shifter and its timer up to send; writing SHIFTBUF starts a frame.
// Bidirectional DShot inverts the frame.
static FAST_CODE void flexioConfigureOutput(const dshotFlexioMotor_t *motor, bool inverted)
{
    FLEXIO_Type *base = motor->base;
    const unsigned shifter = motor->shifter;

    // Disable the shifter before changing its configuration.
    base->SHIFTCTL[shifter] = 0;

    base->SHIFTCFG[shifter] = FLEXIO_SHIFTCFG_INSRC(SHIFTER_INPUT_FROM_PIN)
                            | FLEXIO_SHIFTCFG_PWIDTH(0)
                            | FLEXIO_SHIFTCFG_SSTOP(SHIFTER_STOP_LOW)
                            | FLEXIO_SHIFTCFG_SSTART(SHIFTER_START_DISABLED_LOAD_ON_ENABLE);

    base->SHIFTCTL[shifter] = FLEXIO_SHIFTCTL_TIMSEL(shifter)
                            | FLEXIO_SHIFTCTL_TIMPOL(SHIFTER_TIMER_POLARITY_POSEDGE)
                            | FLEXIO_SHIFTCTL_PINCFG(PIN_CONFIG_OUTPUT)
                            | FLEXIO_SHIFTCTL_PINSEL(motor->flexioPin)
                            | FLEXIO_SHIFTCTL_PINPOL(inverted ? PIN_ACTIVE_LOW : 0u)
                            | FLEXIO_SHIFTCTL_SMOD(SHIFTER_MODE_TRANSMIT);

    base->TIMCFG[shifter] = FLEXIO_TIMCFG_TIMOUT(TIMER_OUT_ONE_NOT_RESET)
                          | FLEXIO_TIMCFG_TIMDEC(TIMER_DEC_ON_FLEXIO_CLOCK)
                          | FLEXIO_TIMCFG_TIMRST(TIMER_RESET_NEVER)
                          | FLEXIO_TIMCFG_TIMDIS(TIMER_DISABLE_ON_COMPARE)
                          | FLEXIO_TIMCFG_TIMENA(TIMER_ENABLE_ON_TRIGGER_HIGH)
                          | FLEXIO_TIMCFG_TSTOP(TIMER_STOP_DISABLED)
                          | FLEXIO_TIMCFG_TSTART(TIMER_START_DISABLED);

    base->TIMCMP[shifter] = dshotTransmitCompare;

    // Trigger is the shifter status flag, active low, so the timer starts when
    // the shifter buffer is full and stops when it drains.
    base->TIMCTL[shifter] = FLEXIO_TIMCTL_TRGSEL(TRIGGER_SHIFTER_STATUS(shifter))
                          | FLEXIO_TIMCTL_TRGPOL(TIMER_TRIGGER_POLARITY_LOW)
                          | FLEXIO_TIMCTL_TRGSRC(TIMER_TRIGGER_SOURCE_INTERNAL)
                          | FLEXIO_TIMCTL_PINCFG(PIN_CONFIG_OUTPUT_DISABLED)
                          | FLEXIO_TIMCTL_PINSEL(0)
                          | FLEXIO_TIMCTL_PINPOL(PIN_ACTIVE_LOW)
                          | FLEXIO_TIMCTL_TIMOD(TIMER_MODE_DUAL_8BIT_BAUD);
}

// Set the same shifter and timer up to read the ESC reply, resyncing the
// timer on every falling edge of the reply.
static void flexioConfigureInput(const dshotFlexioMotor_t *motor)
{
    FLEXIO_Type *base = motor->base;
    const unsigned shifter = motor->shifter;

    // Stop the transmit timer before rewiring it.
    base->TIMCTL[shifter] = 0;

    base->SHIFTCFG[shifter] = FLEXIO_SHIFTCFG_INSRC(SHIFTER_INPUT_FROM_PIN)
                            | FLEXIO_SHIFTCFG_PWIDTH(0)
                            | FLEXIO_SHIFTCFG_SSTOP(SHIFTER_STOP_DISABLED)
                            | FLEXIO_SHIFTCFG_SSTART(SHIFTER_START_DISABLED_LOAD_ON_SHIFT);

    base->SHIFTCTL[shifter] = FLEXIO_SHIFTCTL_TIMSEL(shifter)
                            | FLEXIO_SHIFTCTL_TIMPOL(SHIFTER_TIMER_POLARITY_POSEDGE)
                            | FLEXIO_SHIFTCTL_PINCFG(PIN_CONFIG_OUTPUT_DISABLED)
                            | FLEXIO_SHIFTCTL_PINSEL(motor->flexioPin)
                            | FLEXIO_SHIFTCTL_PINPOL(PIN_ACTIVE_LOW)
                            | FLEXIO_SHIFTCTL_SMOD(SHIFTER_MODE_RECEIVE);

    // Clear the flag the transmit left set so the first interrupt is the reply.
    flexioClearShifterFlags(base, motor->shifterBit);

    // Timer output low on reset puts each sample in the middle of a bit.
    base->TIMCFG[shifter] = FLEXIO_TIMCFG_TIMOUT(TIMER_OUT_ZERO_ON_RESET)
                          | FLEXIO_TIMCFG_TIMDEC(TIMER_DEC_ON_FLEXIO_CLOCK)
                          | FLEXIO_TIMCFG_TIMRST(TIMER_RESET_ON_PIN_RISING_EDGE)
                          | FLEXIO_TIMCFG_TIMDIS(TIMER_DISABLE_ON_COMPARE)
                          | FLEXIO_TIMCFG_TIMENA(TIMER_ENABLE_ON_TRIGGER_EITHER)
                          | FLEXIO_TIMCFG_TSTOP(TIMER_STOP_ON_TIMER_DISABLE)
                          | FLEXIO_TIMCFG_TSTART(TIMER_START_ENABLED);

    base->TIMCMP[shifter] = motor->receiveCompare;

    // TIMRST acts on the timer pin (PINSEL), not the trigger, so it must be
    // this motor's pin.
    base->TIMCTL[shifter] = FLEXIO_TIMCTL_TRGSEL(TRIGGER_FLEXIO_PIN(motor->flexioPin))
                          | FLEXIO_TIMCTL_TRGPOL(TIMER_TRIGGER_POLARITY_HIGH)
                          | FLEXIO_TIMCTL_TRGSRC(TIMER_TRIGGER_SOURCE_INTERNAL)
                          | FLEXIO_TIMCTL_PINCFG(PIN_CONFIG_OUTPUT_DISABLED)
                          | FLEXIO_TIMCTL_PINSEL(motor->flexioPin)
                          | FLEXIO_TIMCTL_PINPOL(PIN_ACTIVE_LOW)
                          | FLEXIO_TIMCTL_TIMOD(TIMER_MODE_DUAL_8BIT_BAUD);
}

// Timer flag sweeps flexioInstanceIrq() makes before it gives up.
#define FLEXIO_IRQ_TIMER_PASSES 4

// Shifter flag reads to wait for the first half to load (two FlexIO clocks)
// before the second half is written regardless.
#define DSHOT_FLEXIO_LOAD_READS 8

static void flexioShifterEvent(dshotFlexioMotor_t *motor)
{
    FLEXIO_Type *base = motor->base;

    // Only a buffer read clears the flag, so mask it until the next turnaround.
    base->SHIFTSIEN &= ~motor->shifterBit;

    if (motor->state == DSHOT_FLEXIO_RECEIVING) {
        // The reply is MSB first, so read the bit swapped buffer alias.
        motor->state = DSHOT_FLEXIO_RECEIVED;
        motor->rawResponse = base->SHIFTBUFBIS[motor->shifter];
        dshotReceivedMask |= motor->motorBit;
    }
}

static void flexioTimerEvent(dshotFlexioMotor_t *motor)
{
    switch (motor->state) {
    case DSHOT_FLEXIO_SECOND_HALF_QUEUED:
        motor->state = DSHOT_FLEXIO_FIRST_HALF_SENT;
        break;

    case DSHOT_FLEXIO_FIRST_HALF_SENT:
        if (!useDshotTelemetry) {
            motor->state = DSHOT_FLEXIO_SENT;
            break;
        }

        // Frame out; the ESC answers about 30 us later.
        motor->base->SHIFTSIEN &= ~motor->shifterBit;

        // In receive mode a floating pin re-arms the timer endlessly, so drop
        // its interrupt; dshotFlexioUpdateComplete() restores it.
        motor->base->TIMIEN &= ~motor->shifterBit;

        motor->state = DSHOT_FLEXIO_RECEIVING;
        flexioConfigureInput(motor);
        motor->base->SHIFTSIEN |= motor->shifterBit;
        break;

    default:
        break;
    }
}

static FAST_IRQ_HANDLER void flexioInstanceIrq(flexioInstance_t *instance)
{
    FLEXIO_Type *base = instance->base;
    const uint32_t channelMask = instance->channelMask;

#if SERIAL_PIOUART_COUNT > 0
    // A PIOUART port can have the instance to itself.
    if (!base) {
        return;
    }
#endif

    uint32_t shifterFlags = base->SHIFTSTAT & channelMask;

    for (unsigned shifter = 0; shifterFlags != 0 && shifter < FLEXIO_DSHOT_CHANNELS; shifter++) {
        if (shifterFlags & (1u << shifter)) {
            flexioShifterEvent(instance->shifterMotor[shifter]);
        }
    }

    // Sweep until no timer flag stands; at the cap, channels still flagged (a
    // floating receive pin) have their timer interrupt switched off.
    uint32_t timerFlags;
    unsigned passes = 0;
    while ((timerFlags = base->TIMSTAT & channelMask) != 0) {
        if (++passes > FLEXIO_IRQ_TIMER_PASSES) {
            base->TIMIEN &= ~timerFlags;
            break;
        }

        for (unsigned shifter = 0; shifter < FLEXIO_DSHOT_CHANNELS; shifter++) {
            if (timerFlags & (1u << shifter)) {
                flexioClearTimerFlags(base, 1u << shifter);
                flexioTimerEvent(instance->shifterMotor[shifter]);
            }
        }
    }
}

FAST_IRQ_HANDLER void FLEXIO1_IRQHandler(void)
{
#if SERIAL_PIOUART_COUNT > 0
    flexioUartIrqHandler(FLEXIO1);
#endif
    flexioInstanceIrq(&flexioInstances[0]);
}

FAST_IRQ_HANDLER void FLEXIO2_IRQHandler(void)
{
#if SERIAL_PIOUART_COUNT > 0
    flexioUartIrqHandler(FLEXIO2);
#endif
    flexioInstanceIrq(&flexioInstances[1]);
}

#if SERIAL_PIOUART_COUNT > 0
uint32_t dshotFlexioChannelMask(const FLEXIO_Type *base)
{
    for (unsigned n = 0; n < FLEXIO_DSHOT_INSTANCES; n++) {
        if (flexioInstances[n].base == base) {
            return flexioInstances[n].channelMask;
        }
    }

    return 0;
}
#endif

static FAST_CODE void dshotFlexioUpdateInit(void)
{
    for (unsigned i = 0; i < ARRAYLEN(dshotFlexioMotors); i++) {
        dshotFlexioMotors[i].pending = false;
    }
}

static FAST_CODE void dshotFlexioWriteInt(uint8_t motorIndex, uint16_t value)
{
    // A deferred loop leaves the channel receiving.
    if (dshotFrameDeferred || motorIndex >= ARRAYLEN(dshotFlexioMotors)) {
        return;
    }

    dshotFlexioMotor_t *motor = &dshotFlexioMotors[motorIndex];
    if (!motor->configured) {
        return;
    }

    // A queued DShot command takes the place of the throttle value.
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(motorIndex);
        if (value) {
            motor->protocolControl.requestTelemetry = true;
        }
    }

    motor->protocolControl.value = value;

    const uint64_t expanded = dshotFlexioExpandFrame(prepareDshotPacket(&motor->protocolControl));
    motor->firstHalf = (uint32_t)(expanded & 0xffffffu);
    motor->secondHalf = (uint32_t)(expanded >> 24);
    motor->pending = true;

    // The interrupt touches the same state and registers.
    ATOMIC_BLOCK(DSHOT_FLEXIO_PRIORITY) {
        if (useDshotTelemetry) {
            // Put the channel back from receive to send; every register differs
            // between the two, and dshotFlexioUpdateComplete() clears the timer flags.
            FLEXIO_Type *base = motor->base;
            base->TIMCTL[motor->shifter] = 0;

            // A received reply has already masked its shifter interrupt.
            if (motor->state != DSHOT_FLEXIO_RECEIVED) {
                base->SHIFTSIEN &= ~motor->shifterBit;
            }
            flexioConfigureOutput(motor, true);
        }

        motor->state = DSHOT_FLEXIO_FIRST_HALF_QUEUED;
    }
}

static FAST_CODE void dshotFlexioWrite(uint8_t motorIndex, float value)
{
    dshotFlexioWriteInt(motorIndex, dshotFlexioLrintf(value));
}

static FAST_CODE void dshotFlexioUpdateComplete(void)
{
    // A deferred loop sends nothing and leaves the command queue where it is.
    if (dshotFrameDeferred) {
        dshotFrameDeferred = false;
        return;
    }

    // Hold the frame back until the command queue says this is a good moment.
    if (!dshotCommandQueueEmpty() && !dshotCommandOutputIsEnabled(dshotMotorCount)) {
        return;
    }

    for (unsigned i = 0; i < flexioActiveCount; i++) {
        flexioClearTimerFlags(flexioActive[i]->base, flexioActive[i]->channelMask);
    }

    dshotSentAtUs = micros();

    // The first loop starts every motor and the second queues each second
    // half behind the first; both run masked so the motors start together and
    // the second half is written before the first one finishes.
    ATOMIC_BLOCK(DSHOT_FLEXIO_PRIORITY) {
        for (unsigned i = 0; i < dshotMotorCount; i++) {
            const dshotFlexioMotor_t *motor = &dshotFlexioMotors[i];
            if (motor->pending) {
                motor->base->SHIFTBUF[motor->shifter] = motor->firstHalf;
            }
        }

        // Wait, bounded, for the shifter flags that say the first halves loaded.
        for (unsigned i = 0; i < flexioActiveCount; i++) {
            const uint32_t mask = flexioActive[i]->channelMask;
            unsigned reads = DSHOT_FLEXIO_LOAD_READS;
            while ((flexioActive[i]->base->SHIFTSTAT & mask) != mask && --reads) {
            }
        }

        uint32_t sent = 0;
        for (unsigned i = 0; i < dshotMotorCount; i++) {
            dshotFlexioMotor_t *motor = &dshotFlexioMotors[i];
            if (motor->pending) {
                motor->state = DSHOT_FLEXIO_SECOND_HALF_QUEUED;
                motor->base->SHIFTBUF[motor->shifter] = motor->secondHalf;
                sent |= motor->motorBit;
            }
        }
        dshotSentMask = sent;
    }

    // Written whole to restore channels the interrupt masked off.
    for (unsigned i = 0; i < flexioActiveCount; i++) {
        flexioActive[i]->base->TIMIEN = flexioActive[i]->channelMask;
    }
}

#ifdef USE_DSHOT_TELEMETRY
static FAST_CODE bool dshotFlexioDecodeTelemetryAll(void)
{
    if (!useDshotTelemetry) {
        return true;
    }

#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = millis();
#endif

    // An early loop sends no frame, since that would cut off replies still on the wire;
    // the ESC holds its throttle and the replies are read on the next loop.
    // A PID period shorter than the exchange (DShot600 at 16 kHz) sends every second loop.
    if ((dshotReceivedMask & dshotSentMask) != dshotSentMask) {
        const timeDelta_t elapsedUs = cmpTimeUs(micros(), dshotSentAtUs);
        if (elapsedUs >= 0 && elapsedUs < dshotExchangeUs) {
            dshotFrameDeferred = true;
            dshotFramesDeferred++;
            dshotFrameDeferEarlyMaxUs = MAX(dshotFrameDeferEarlyMaxUs, dshotExchangeUs - elapsedUs);
        }
    }
    DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 4, dshotFramesDeferred);  //!< Deferred Frames
    DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 5, dshotFrameDeferEarlyMaxUs);  //!< Max Early By [unit:us]

    if (dshotFrameDeferred) {
        return true;
    }

    const uint32_t received = dshotReceivedMask;

    for (unsigned i = 0; i < dshotMotorCount; i++) {
        dshotFlexioMotor_t *motor = &dshotFlexioMotors[i];
        uint32_t rawValue = DSHOT_TELEMETRY_INVALID;

        if (received & (1u << i)) {
            dshotTelemetryState.readCount++;

            if (!motor->training.done) {
                // Search the receive rate before trusting any reading.
                if (dshotFlexioTrainStep(&motor->training, motor->rawResponse)) {
                    const uint32_t clockHz = nxpClockHz(flexioHardware[motor->instanceIndex].clock);
                    motor->receiveCompare = dshotFlexioRxTimerCompare(clockHz,
                                                                      dshotBitRateHz,
                                                                      motor->training.offset);
                }
            } else {
                const uint32_t decoded = dshotFlexioDecodeTelemetry(motor->rawResponse);
                if (decoded != DSHOT_FLEXIO_DECODE_INVALID) {
                    rawValue = decoded;
                }
            }
        }

        if (rawValue != DSHOT_TELEMETRY_INVALID) {
            if ((rawValue == 0x0e00) && (dshotCommandGetCurrent(i) == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE)) {
                dshotTelemetryState.motorState[i].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
            } else {
                dshotTelemetryState.motorState[i].rawValue = rawValue;
            }
        } else {
            dshotTelemetryState.invalidPacketCount++;
        }

#ifdef USE_DSHOT_TELEMETRY_STATS
        updateDshotTelemetryQuality(&dshotTelemetryQuality[i], rawValue != DSHOT_TELEMETRY_INVALID, currentTimeMs);
#endif
    }

    // Drop only the replies read above; a late reply is kept for next time.
    ATOMIC_BLOCK(DSHOT_FLEXIO_PRIORITY) {
        dshotReceivedMask &= ~received;
    }

    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;

    return true;
}

#if PLATFORM_TRAIT_DSHOT_TRAINING
bool dshotGetTrainingStatus(unsigned motorIndex, dshotTrainingStatus_t *status)
{
    if (motorIndex >= ARRAYLEN(dshotFlexioMotors)) {
        return false;
    }

    const dshotFlexioMotor_t *motor = &dshotFlexioMotors[motorIndex];

    // Nothing is searched for unless the motor is set up and reading replies.
    if (!motor->configured || !useDshotTelemetry) {
        return false;
    }

    status->passMask = motor->training.passMask;
    status->offset = motor->training.offset;
    status->offsetMin = DSHOT_FLEXIO_TCMP_OFFSET_MIN;
    status->offsetMax = DSHOT_FLEXIO_TCMP_OFFSET_MAX;
    status->done = motor->training.done;

    return true;
}
#endif
#endif // USE_DSHOT_TELEMETRY

static bool dshotFlexioEnable(void)
{
    // Shutdown turns the blocks off, so bring them back before arming again.
    for (unsigned i = 0; i < flexioActiveCount; i++) {
        flexioActive[i]->base->CTRL |= FLEXIO_CTRL_FLEXEN_MASK;
    }

    // ESC passthrough leaves the pads muxed to GPIO, so mux them back.
    for (unsigned i = 0; i < ARRAYLEN(dshotFlexioMotors); i++) {
        dshotFlexioMotor_t *motor = &dshotFlexioMotors[i];
        if (motor->configured) {
            nxpIomuxConfigPin(motor->mux, IOCFG_AF_PP_UP);

            // Start the next frame from a known point.
            motor->state = DSHOT_FLEXIO_IDLE;
            motor->pending = false;
        }
    }

    return true;
}

static void dshotFlexioDisable(void)
{
    // Per motor state must survive for ESC passthrough.
}

static FAST_CODE bool dshotFlexioIsMotorEnabled(unsigned index)
{
    return index < ARRAYLEN(dshotFlexioMotors) && dshotFlexioMotors[index].configured;
}

static void dshotFlexioShutdown(void)
{
    for (unsigned i = 0; i < flexioActiveCount; i++) {
        FLEXIO_Type *base = flexioActive[i]->base;
        base->SHIFTSIEN = 0;
        base->TIMIEN = 0;
        base->CTRL &= ~FLEXIO_CTRL_FLEXEN_MASK;
    }
}

static FAST_CODE bool dshotFlexioIsMotorIdle(unsigned index)
{
    return index < ARRAYLEN(dshotFlexioMotors) && dshotFlexioMotors[index].protocolControl.value == 0;
}

// Not gated on bidirectional DShot: the ESC sensor uses it without.
static FAST_CODE void dshotFlexioRequestTelemetry(unsigned index)
{
    if (index < ARRAYLEN(dshotFlexioMotors)) {
        dshotFlexioMotors[index].protocolControl.requestTelemetry = true;
    }
}

static IO_t dshotFlexioGetMotorIO(unsigned index)
{
    return index < ARRAYLEN(dshotFlexioMotors) ? dshotFlexioMotors[index].io : IO_NONE;
}

static FAST_RODATA const motorVTable_t dshotFlexioVTable = {
    .postInit = NULL,
    .enable = dshotFlexioEnable,
    .disable = dshotFlexioDisable,
    .isMotorEnabled = dshotFlexioIsMotorEnabled,
    .telemetryWait = NULL,
#ifdef USE_DSHOT_TELEMETRY
    .decodeTelemetry = dshotFlexioDecodeTelemetryAll,
#else
    .decodeTelemetry = NULL,
#endif
    .updateInit = dshotFlexioUpdateInit,
    .write = dshotFlexioWrite,
    .writeInt = dshotFlexioWriteInt,
    .updateComplete = dshotFlexioUpdateComplete,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .shutdown = dshotFlexioShutdown,
    .isMotorIdle = dshotFlexioIsMotorIdle,
    .getMotorIO = dshotFlexioGetMotorIO,
    .requestTelemetry = dshotFlexioRequestTelemetry,
};

static uint32_t dshotProtocolBitRate(motorProtocolTypes_e protocol)
{
    switch (protocol) {
    case MOTOR_PROTOCOL_DSHOT150:
        return 150000;
    case MOTOR_PROTOCOL_DSHOT300:
        return 300000;
    case MOTOR_PROTOCOL_DSHOT600:
        return 600000;
    default:
        return 0;
    }
}

// Reset the block and leave it disabled while its channels are set up.
static void flexioInstanceInit(unsigned index)
{
    const flexioHardware_t *hw = &flexioHardware[index];
    flexioInstance_t *instance = &flexioInstances[index];

    nxpClockGateEnable(hw->clock);

    FLEXIO_Type *base = hw->base;
    base->CTRL |= FLEXIO_CTRL_SWRST_MASK;
    base->CTRL = 0;
    base->CTRL = FLEXIO_CTRL_DBGE(1) | FLEXIO_CTRL_FASTACC(1);

    instance->base = base;
    flexioActive[flexioActiveCount++] = instance;
}

// Release the motors set up before an init failure.
static void dshotFlexioReleaseMotors(void)
{
    dshotFlexioShutdown();

    for (unsigned i = 0; i < ARRAYLEN(dshotFlexioMotors); i++) {
        dshotFlexioMotor_t *motor = &dshotFlexioMotors[i];
        if (motor->configured) {
            IORelease(motor->io);
            motor->configured = false;
        }
    }

    flexioActiveCount = 0;

    // A failed init installs no vtable, so nothing may keep claiming telemetry.
    useDshotTelemetry = false;
}

bool dshotPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig)
{
    // Both are only set once the whole set of motors is up.
    device->vTable = NULL;
    dshotMotorCount = 0;

    const uint8_t motorCount = device->count;
    if (motorCount > ARRAYLEN(dshotFlexioMotors)) {
        return false;
    }

    dshotBitRateHz = dshotProtocolBitRate(motorConfig->motorProtocol);
    if (dshotBitRateHz == 0) {
        return false;
    }
    dshotExchangeUs = (timeDelta_t)(DSHOT_FLEXIO_EXCHANGE_BITS * 1000000u / dshotBitRateHz);
    dshotSentMask = 0;

    // Every FlexIO root DShot uses runs at the same rate, so one compare serves them all.
    const uint32_t flexioClockHz = nxpClockHz(FLEXIO_DSHOT_CLOCK);
    dshotTransmitCompare = dshotFlexioTxTimerCompare(flexioClockHz, dshotBitRateHz);

    // ESCs reject a bit rate more than one percent off, which a slow FlexIO root gives.
    const uint32_t sentBitRateHz = flexioClockHz / (2u * 3u * ((dshotTransmitCompare & 0xffu) + 1u));
    if (sentBitRateHz * 100u < dshotBitRateHz * 99u || sentBitRateHz * 100u > dshotBitRateHz * 101u) {
        return false;
    }

#ifdef USE_DSHOT_TELEMETRY
    useDshotTelemetry = motorConfig->useDshotTelemetry;

    // The 8 bit receive divider does not reach DShot150, so telemetry is off.
    if (useDshotTelemetry && !dshotFlexioRxDividerFits(flexioClockHz, dshotBitRateHz)) {
        useDshotTelemetry = false;
    }
#endif

    memset(flexioInstances, 0, sizeof(flexioInstances));
    flexioActiveCount = 0;

    for (unsigned i = 0; i < motorCount; i++) {
        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[i];
        const ioTag_t tag = motorConfig->ioTags[reorderedMotorIndex];

        // Take the first instance with this pad and a free shifter.
        const nxpFlexioPin_t *pad = NULL;
        unsigned instanceIndex = 0;

        for (unsigned n = 0; n < FLEXIO_DSHOT_INSTANCES; n++) {
            if (!FLEXIO_DSHOT_USABLE(n)) {
                continue;
            }
            const nxpFlexioPin_t *candidate = NXP_FLEXIO_PIN(nxpFlexioPads,
                                                                 FLEXIO_INSTANCE_NUMBER(n), tag);
            if (candidate && flexioInstances[n].inUse < FLEXIO_DSHOT_CHANNELS) {
                pad = candidate;
                instanceIndex = n;
                break;
            }
        }

        if (!pad) {
            dshotFlexioReleaseMotors();
            return false;
        }

        const IO_t io = IOGetByTag(tag);
        if (!IOIsFreeOrPreinit(io)) {
            dshotFlexioReleaseMotors();
            return false;
        }

        if (!flexioInstances[instanceIndex].base) {
            flexioInstanceInit(instanceIndex);
        }

        IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(i));

        // Pull up keeps the line defined while nobody drives it.
        nxpIomuxConfigPin(&pad->mux, IOCFG_AF_PP_UP);

        flexioInstance_t *instance = &flexioInstances[instanceIndex];
        const uint8_t shifter = instance->inUse++;

        dshotFlexioMotor_t *motor = &dshotFlexioMotors[i];
        motor->io = io;
        motor->base = instance->base;
        motor->mux = &pad->mux;
        motor->instanceIndex = instanceIndex;
        motor->shifter = shifter;
        motor->shifterBit = 1u << shifter;
        motor->motorBit = 1u << i;
        motor->flexioPin = pad->flexioPin;
        motor->configured = true;
        motor->pending = false;
        motor->state = DSHOT_FLEXIO_IDLE;

        dshotFlexioTrainReset(&motor->training);
        motor->receiveCompare = dshotFlexioRxTimerCompare(nxpClockHz(flexioHardware[instanceIndex].clock),
                                                          dshotBitRateHz,
                                                          motor->training.offset);

        flexioConfigureOutput(motor, useDshotTelemetry);

        instance->channelMask |= motor->shifterBit;
        instance->shifterMotor[shifter] = motor;
    }

    for (unsigned i = 0; i < flexioActiveCount; i++) {
        flexioActive[i]->base->CTRL |= FLEXIO_CTRL_FLEXEN_MASK;
    }

    for (unsigned n = 0; n < FLEXIO_DSHOT_INSTANCES; n++) {
        if (flexioInstances[n].base) {
            NVIC_SetPriority(flexioHardware[n].irq, NVIC_PRIORITY_LEVEL(DSHOT_FLEXIO_PRIORITY));
            NVIC_EnableIRQ(flexioHardware[n].irq);
        }
    }

    device->vTable = &dshotFlexioVTable;
    dshotMotorCount = motorCount;

    return true;
}

#endif // USE_DSHOT
