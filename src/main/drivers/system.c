/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "sound_beeper.h"
#include "drivers/nvic.h"
#include "build/atomic.h"
#include "build/build_config.h"

#include "drivers/system.h"
#include "drivers/time.h"

#ifndef EXTI_CALLBACK_HANDLER_COUNT
#define EXTI_CALLBACK_HANDLER_COUNT 1
#endif

extiCallbackHandlerConfig_t extiHandlerConfigs[EXTI_CALLBACK_HANDLER_COUNT];

void registerExtiCallbackHandler(IRQn_Type irqn, extiCallbackHandlerFunc *fn)
{
    for (int index = 0; index < EXTI_CALLBACK_HANDLER_COUNT; index++) {
        extiCallbackHandlerConfig_t *candidate = &extiHandlerConfigs[index];
        if (!candidate->fn) {
            candidate->fn = fn;
            candidate->irqn = irqn;
            return;
        }
    }
    failureMode(FAILURE_DEVELOPER); // EXTI_CALLBACK_HANDLER_COUNT is too low for the amount of handlers required.
}

// cycles per microsecond
STATIC_UNIT_TESTED timeUs_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
STATIC_UNIT_TESTED volatile timeMs_t sysTickUptime = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;

#ifndef UNIT_TEST
void cycleCounterInit(void)
{
#if defined(USE_HAL_DRIVER)
    usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
#else
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;

#endif

    // Enable DWT for precision time measurement
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
#endif // UNIT_TEST

// SysTick

static volatile int sysTickPending = 0;

void SysTick_Handler(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        sysTickUptime++;
        sysTickPending = 0;
        (void)(SysTick->CTRL);
    }
#ifdef USE_HAL_DRIVER
    // used by the HAL for some timekeeping and timeouts, should always be 1ms
    HAL_IncTick();
#endif
}

uint32_t ticks(void)
{
#ifdef UNIT_TEST
    return 0;
#else
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    return DWT->CYCCNT;
#endif
}

timeDelta_t ticks_diff_us(uint32_t begin, uint32_t end)
{
    return (end - begin) / usTicks;
}

// Return system uptime in microseconds
timeUs_t microsISR(void)
{
    register uint32_t ms, pending, cycle_cnt;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        cycle_cnt = SysTick->VAL;

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            // Update pending.
            // Record it for multiple calls within the same rollover period
            // (Will be cleared when serviced).
            // Note that multiple rollovers are not considered.

            sysTickPending = 1;

            // Read VAL again to ensure the value is read after the rollover.

            cycle_cnt = SysTick->VAL;
        }

        ms = sysTickUptime;
        pending = sysTickPending;
    }

    return ((timeUs_t)(ms + pending) * 1000LL) + (usTicks * 1000LL - (timeUs_t)cycle_cnt) / usTicks;
}

timeUs_t micros(void)
{
    register uint32_t ms, cycle_cnt;

    // Call microsISR() in interrupt and elevated (non-zero) BASEPRI context

#ifndef UNIT_TEST
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
        return microsISR();
    }
#endif

    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
        /*
         * If the SysTick timer expired during the previous instruction, we need to give it a little time for that
         * interrupt to be delivered before we can recheck sysTickUptime:
         */
        asm volatile("\tnop\n");
    } while (ms != sysTickUptime);
    return ((timeUs_t)ms * 1000LL) + (usTicks * 1000LL - (timeUs_t)cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
timeMs_t millis(void)
{
    return sysTickUptime;
}

#if 1
void delayMicroseconds(timeUs_t us)
{
    timeUs_t now = micros();
    while (micros() - now < us);
}
#else
void delayMicroseconds(timeUs_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;

    for (;;) {
        register uint32_t current_count = SysTick->VAL;
        timeUs_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
#endif

void delay(timeMs_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

#define SHORT_FLASH_DURATION 50
#define CODE_FLASH_DURATION 250

void failureMode(failureMode_e mode)
{
#ifdef UNIT_TEST
    (void)mode;
#else
    int codeRepeatsRemaining = 10;
    int codeFlashesRemaining;
    int shortFlashesRemaining;

    while (codeRepeatsRemaining--) {
        LED1_ON;
        LED0_OFF;
        shortFlashesRemaining = 5;
        codeFlashesRemaining = mode + 1;
        uint8_t flashDuration = SHORT_FLASH_DURATION;

        while (shortFlashesRemaining || codeFlashesRemaining) {
            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_ON;
            delay(flashDuration);

            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_OFF;
            delay(flashDuration);

            if (shortFlashesRemaining) {
                shortFlashesRemaining--;
                if (shortFlashesRemaining == 0) {
                    delay(500);
                    flashDuration = CODE_FLASH_DURATION;
                }
            } else {
                codeFlashesRemaining--;
            }
        }
        delay(1000);
    }

#ifdef DEBUG
    systemReset();
#else
    systemResetToBootloader();
#endif
#endif //UNIT_TEST
}
