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

#include "platform.h"

#include "build/atomic.h"

#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"

extern uint32_t SystemCoreClock;

static volatile uint32_t s_cycPrev;
static volatile uint64_t s_cycAccum;

void cycleCounterInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#ifdef DWT_LAR_UNLOCK
    DWT->LAR = 0xC5ACCE55;
#endif
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cycPrev = DWT->CYCCNT;
    s_cycAccum = 0U;
}

uint32_t getCycleCounter(void)
{
    return DWT->CYCCNT;
}

/*
 * The time base is the DWT cycle counter extended to 64 bits. CYCCNT wraps
 * every ~5.4 s at 800 MHz, so the fold runs on every read and from a 1 kHz
 * SysTick, and is done with interrupts masked since both thread and interrupt
 * context call it.
 */
static inline uint64_t navq95_cycles64(void)
{
    uint64_t cycles;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        uint32_t now = DWT->CYCCNT;
        uint32_t delta = now - s_cycPrev;
        s_cycPrev = now;
        s_cycAccum += delta;
        cycles = s_cycAccum;
    }

    return cycles;
}

void navq95_timebaseTick(void)
{
    (void)navq95_cycles64();
}

#define NAVQ95_TIMEBASE_TICK_HZ 1000U

// Call after cycleCounterInit(), with SystemCoreClock set.
void navq95_timebaseInit(void)
{
    uint32_t reload = (SystemCoreClock / NAVQ95_TIMEBASE_TICK_HZ);
    if (reload == 0U) {
        reload = 1U;
    }
    SysTick->LOAD = (reload - 1U) & SysTick_LOAD_RELOAD_Msk;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
    navq95_timebaseTick();
}

uint32_t micros(void)
{
    uint32_t cyc_per_us = SystemCoreClock / 1000000U;
    if (cyc_per_us == 0U) {
        cyc_per_us = 1U;
    }
    return (uint32_t)(navq95_cycles64() / cyc_per_us);
}

uint32_t microsISR(void)
{
    return micros();
}

uint32_t millis(void)
{
    uint32_t cyc_per_ms = SystemCoreClock / 1000U;
    if (cyc_per_ms == 0U) {
        cyc_per_ms = 1U;
    }
    return (uint32_t)(navq95_cycles64() / cyc_per_ms);
}

void delayMicroseconds(uint32_t us)
{
    uint32_t start = micros();
    while ((micros() - start) < us) {
    }
}

void delay(uint32_t ms)
{
    while (ms--) {
        delayMicroseconds(1000U);
    }
}

int32_t clockCyclesToMicros(int32_t cycles)
{
    return (int32_t)(cycles / (SystemCoreClock / 1000000U));
}

int32_t clockCyclesTo10thMicros(int32_t cycles)
{
    return (int32_t)(cycles / (SystemCoreClock / 10000000U));
}

int32_t clockCyclesTo100thMicros(int32_t cycles)
{
    return (int32_t)(cycles / (SystemCoreClock / 100000000U));
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * (SystemCoreClock / 1000000U);
}

// The status LEDs sit on an I2C I/O expander that has no driver yet, so only the beeper signals failures.
static void indicate(uint8_t count, uint16_t duration)
{
    if (count) {
        while (count--) {
            BEEP_ON;
            delay(duration);

            BEEP_OFF;
            delay(duration);
        }
    }
}

void indicateFailure(failureMode_e mode, int codeRepeatsRemaining)
{
    while (codeRepeatsRemaining--) {
        indicate(WARNING_FLASH_COUNT, WARNING_FLASH_DURATION_MS);

        delay(WARNING_PAUSE_DURATION_MS);

        indicate(mode + 1, WARNING_CODE_DURATION_LONG_MS);

        delay(1000);
    }
}

void failureMode(failureMode_e mode)
{
    indicateFailure(mode, 10);

    systemReset();

    for (;;) {
        indicateFailure(mode, 1);
    }
}
