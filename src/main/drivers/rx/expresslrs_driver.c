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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 *
 * Authors:
 * Dominic Clifton/Hydra - Timer-based timeout implementation.
 * AlessandroAU - stdperiph Timer-based timeout implementation.
 */

#include <string.h>
#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "build/debug.h"
#include "build/debug_pin.h"

#include "drivers/timer.h"
#include "drivers/nvic.h"
#include "drivers/rx/expresslrs_driver.h"

#include "common/maths.h"

#define TIMER_INTERVAL_US_DEFAULT 20000
#define TICK_TOCK_COUNT 2

TIM_TypeDef *timer;

typedef enum {
    TICK,
    TOCK
} tickTock_e;

typedef struct elrsTimerState_s {
    bool running;
    volatile tickTock_e tickTock;
    uint32_t intervalUs;
    int32_t frequencyOffsetTicks;
    int32_t phaseShiftUs;
} elrsTimerState_t;

// Use a little ram to keep the amount of CPU cycles used in the ISR lower.
typedef struct elrsPhaseShiftLimits_s {
    int32_t min;
    int32_t max;
} elrsPhaseShiftLimits_t;

elrsPhaseShiftLimits_t phaseShiftLimits;

static elrsTimerState_t timerState = {
    false,
    TOCK, // Start on TOCK (in ELRS isTick is initialised to false)
    TIMER_INTERVAL_US_DEFAULT,
    0,
    0
};

void expressLrsTimerDebug(void)
{
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 2, timerState.frequencyOffsetTicks);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 3, timerState.phaseShiftUs);
}

static void expressLrsRecalculatePhaseShiftLimits(void)
{
    phaseShiftLimits.max = (timerState.intervalUs / TICK_TOCK_COUNT);
    phaseShiftLimits.min = -phaseShiftLimits.max;
}

static uint16_t expressLrsCalculateMaximumExpectedPeriod(uint16_t intervalUs)
{
    // The timer reload register must not overflow when frequencyOffsetTicks is added to it.
    // frequencyOffsetTicks is not expected to be higher than 1/4 of the interval.
    // also, timer resolution must be as high as possible.
    const uint16_t maximumExpectedPeriod = (intervalUs / TICK_TOCK_COUNT) + (timerState.intervalUs / 4);
    return maximumExpectedPeriod;
}

void expressLrsUpdateTimerInterval(uint16_t intervalUs)
{
    timerState.intervalUs = intervalUs;
    expressLrsRecalculatePhaseShiftLimits();

    timerReconfigureTimeBase(timer, expressLrsCalculateMaximumExpectedPeriod(timerState.intervalUs), MHZ_TO_HZ(1));
    timerSetPeriod(timer, (timerState.intervalUs / TICK_TOCK_COUNT) - 1);
}

void expressLrsUpdatePhaseShift(int32_t newPhaseShift)
{
    timerState.phaseShiftUs = constrain(newPhaseShift, phaseShiftLimits.min, phaseShiftLimits.max);
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

static void expressLrsOnTimerUpdate(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
    UNUSED(cbRec);
    UNUSED(capture);

    if (timerState.tickTock == TICK) {
        dbgPinHi(0);

        uint32_t adjustedPeriod = (timerState.intervalUs / TICK_TOCK_COUNT) + timerState.frequencyOffsetTicks;

        timerSetPeriod(timer, adjustedPeriod - 1);

        expressLrsOnTimerTickISR();

        timerState.tickTock = TOCK;
    } else {
        dbgPinLo(0);

        uint32_t adjustedPeriod = (timerState.intervalUs / TICK_TOCK_COUNT) + timerState.phaseShiftUs + timerState.frequencyOffsetTicks;

        timerSetPeriod(timer, adjustedPeriod - 1);

        timerState.phaseShiftUs = 0;

        expressLrsOnTimerTockISR();

        timerState.tickTock = TICK;
    }
}

bool expressLrsTimerIsRunning(void)
{
    return timerState.running;
}

void expressLrsTimerStop(void)
{
    timerDisable(timer);
    timerSetCounter(timer, 0);
    timerState.running = false;
}

void expressLrsTimerResume(void)
{
    timerState.tickTock = TOCK;

    timerSetPeriod(timer, (timerState.intervalUs / TICK_TOCK_COUNT) - 1);
    timerSetCounter(timer, 0);

    timerEnableInterrupt(timer);
    timerState.running = true;
    timerEnable(timer);
}

void expressLrsInitialiseTimer(TIM_TypeDef *t, timerOvrHandlerRec_t *timerUpdateCb)
{
    timer = t;

    configTimeBase(timer, expressLrsCalculateMaximumExpectedPeriod(timerState.intervalUs), MHZ_TO_HZ(1));

    expressLrsUpdateTimerInterval(timerState.intervalUs);

    timerChOvrHandlerInit(timerUpdateCb, expressLrsOnTimerUpdate);

    timerConfigUpdateCallback(timer, timerUpdateCb);
}

void expressLrsTimerEnableIRQs(void)
{
    uint8_t irq = timerInputIrq(timer);

    // Use the NVIC TIMER priority for now
#ifdef USE_HAL_DRIVER
    HAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER), NVIC_PRIORITY_SUB(NVIC_PRIO_TIMER));
    HAL_NVIC_EnableIRQ(irq);
#else
    NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(NVIC_PRIO_TIMER));
    NVIC_EnableIRQ(irq);
#endif
}

#endif
