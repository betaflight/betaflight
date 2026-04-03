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
 * ELRS tick/tock timer for RP2350, using the pico-sdk hardware_alarm API.
 *
 * The RP2350 has a 64-bit 1µs free-running timer with four alarm channels.
 * Each alarm fires once at a programmed absolute time; we reschedule it on
 * every callback to reproduce the STM32 auto-reload behaviour while still
 * being able to apply per-period phase and frequency corrections.
 *
 * Replaces src/main/drivers/rx/expresslrs_driver.c on this platform
 * (that file is compiled only when !USE_EXPRESSLRS_TIMER_PICO).
 */

#include "platform.h"

#if defined(USE_RX_EXPRESSLRS) && defined(USE_EXPRESSLRS_TIMER_PICO)

#include "hardware/irq.h"
#include "hardware/timer.h"

#include "build/debug.h"
#include "build/debug_pin.h"

#include "drivers/nvic.h"
#include "drivers/timer.h"
#include "drivers/rx/expresslrs_driver.h"

#include "common/maths.h"

// ---------------------------------------------------------------------------
// Internal state – mirrors the STM32 driver layout exactly so the logic is
// easy to compare.
// ---------------------------------------------------------------------------

#define TIMER_INTERVAL_US_DEFAULT   20000u
#define TICK_TOCK_COUNT             2

typedef enum {
    TICK,
    TOCK,
} tickTock_e;

typedef struct {
    bool              running;
    volatile tickTock_e tickTock;
    uint32_t          intervalUs;
    int32_t           frequencyOffsetTicks;
    int32_t           phaseShiftUs;
} elrsTimerState_t;

typedef struct {
    int32_t min;
    int32_t max;
} elrsPhaseShiftLimits_t;

static uint                  alarmNum;
static absolute_time_t       nextAlarmTime;
static elrsPhaseShiftLimits_t phaseShiftLimits;

static elrsTimerState_t timerState = {
    .running              = false,
    .tickTock             = TOCK,   // matches STM32 driver initialisation
    .intervalUs           = TIMER_INTERVAL_US_DEFAULT,
    .frequencyOffsetTicks = 0,
    .phaseShiftUs         = 0,
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void expressLrsRecalculatePhaseShiftLimits(void)
{
    phaseShiftLimits.max =  (int32_t)(timerState.intervalUs / TICK_TOCK_COUNT);
    phaseShiftLimits.min = -phaseShiftLimits.max;
}

// ---------------------------------------------------------------------------
// Alarm callback – runs at NVIC interrupt level, equivalent to the STM32
// timer overflow ISR.
// ---------------------------------------------------------------------------

static void alarmCallback(uint alarm)
{
    UNUSED(alarm);

    const uint32_t halfPeriod = timerState.intervalUs / TICK_TOCK_COUNT;

    if (timerState.tickTock == TICK) {
        dbgPinHi(0);

        int32_t adjustedPeriod = (int32_t)halfPeriod + timerState.frequencyOffsetTicks;
        if (adjustedPeriod < 1) adjustedPeriod = 1;
        if (adjustedPeriod > (int32_t)(2 * halfPeriod)) adjustedPeriod = (int32_t)(2 * halfPeriod);

        // Advance the absolute target by the adjusted half-period.  Using an
        // absolute target (not "now + period") avoids drift accumulation.
        nextAlarmTime = delayed_by_us(nextAlarmTime, (uint64_t)adjustedPeriod);
        if (hardware_alarm_set_target(alarmNum, nextAlarmTime)) {
            // Target was already in the past; reschedule from now to recover.
            nextAlarmTime = make_timeout_time_us((uint64_t)adjustedPeriod);
            hardware_alarm_set_target(alarmNum, nextAlarmTime);
        }

        expressLrsOnTimerTickISR();

        timerState.tickTock = TOCK;
    } else {
        dbgPinLo(0);

        // Apply phase shift on the TOCK half-period only (matches STM32 driver).
        int32_t adjustedPeriod = (int32_t)halfPeriod
                               + timerState.phaseShiftUs
                               + timerState.frequencyOffsetTicks;

        // Consume the phase shift; it must not carry into the next cycle.
        timerState.phaseShiftUs = 0;

        if (adjustedPeriod < 1) adjustedPeriod = 1;
        if (adjustedPeriod > (int32_t)(2 * halfPeriod)) adjustedPeriod = (int32_t)(2 * halfPeriod);

        nextAlarmTime = delayed_by_us(nextAlarmTime, (uint64_t)adjustedPeriod);
        if (hardware_alarm_set_target(alarmNum, nextAlarmTime)) {
            nextAlarmTime = make_timeout_time_us((uint64_t)adjustedPeriod);
            hardware_alarm_set_target(alarmNum, nextAlarmTime);
        }

        expressLrsOnTimerTockISR();

        timerState.tickTock = TICK;
    }
}

// ---------------------------------------------------------------------------
// Public API – identical signatures to expresslrs_driver.c
// ---------------------------------------------------------------------------

void expressLrsTimerDebug(void)
{
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 2, timerState.frequencyOffsetTicks);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 3, timerState.phaseShiftUs);
}

void expressLrsUpdateTimerInterval(uint16_t intervalUs)
{
    timerState.intervalUs = intervalUs;
    expressLrsRecalculatePhaseShiftLimits();
    // No hardware reconfiguration needed; the new interval takes effect on
    // the next alarm reschedule inside alarmCallback.
}

void expressLrsUpdatePhaseShift(int32_t newPhaseShift)
{
    timerState.phaseShiftUs = constrain(newPhaseShift,
                                        phaseShiftLimits.min,
                                        phaseShiftLimits.max);
}

void expressLrsTimerIncreaseFrequencyOffset(void)
{
    timerState.frequencyOffsetTicks++;
}

void expressLrsTimerDecreaseFrequencyOffset(void)
{
    timerState.frequencyOffsetTicks--;
}

void expressLrsTimerResetFrequencyOffset(void)
{
    timerState.frequencyOffsetTicks = 0;
}

bool expressLrsTimerIsRunning(void)
{
    return timerState.running;
}

void expressLrsTimerStop(void)
{
    hardware_alarm_cancel(alarmNum);
    timerState.running = false;
}

void expressLrsTimerResume(void)
{
    timerState.tickTock = TOCK;
    nextAlarmTime = make_timeout_time_us(timerState.intervalUs / TICK_TOCK_COUNT);
    hardware_alarm_set_target(alarmNum, nextAlarmTime);
    timerState.running = true;
}

void expressLrsInitialiseTimer(const timerHardware_t *timHw, timerOvrHandlerRec_t *timerUpdateCb)
{
    UNUSED(timHw);        // No STM32-style timer instance on RP2350
    UNUSED(timerUpdateCb); // Callback registration is done via hardware_alarm API

    // Claim the first available hardware alarm.  Panics if none are free.
    alarmNum = hardware_alarm_claim_unused(true);

    // Register the callback; this also enables the alarm's NVIC IRQ.
    hardware_alarm_set_callback(alarmNum, alarmCallback);

    // Raise the alarm IRQ priority to match the timer priority used on STM32.
    irq_set_priority(hardware_alarm_get_irq_num(alarmNum), NVIC_PRIO_TIMER);

    expressLrsRecalculatePhaseShiftLimits();
}

void expressLrsTimerEnableIRQs(const timerHardware_t *timer)
{
    UNUSED(timer);
    // IRQ is already enabled by hardware_alarm_set_callback() in
    // expressLrsInitialiseTimer().  Nothing more to do here.
}

#endif // USE_RX_EXPRESSLRS && USE_EXPRESSLRS_TIMER_PICO
