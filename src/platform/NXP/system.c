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
 * system.c — NavQ95 (NXP i.MX 95, Cortex-M7) system services.
 *
 * Holds the time base, cycle counter, clock-cycle conversions, reset and
 * failure-handling primitives that the Betaflight common code requires, split
 * out of platform_stubs.c to mirror the per-platform system.c layout used by
 * the other targets (platform/common/stm32/system.c, platform/PICO/system.c).
 * Console, FPU and startup (SystemInit/systemInit) stay in platform_stubs.c as
 * they are tied to the LPUART console bring-up.
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

/*
 * Core clock used to scale the DWT cycle counter into micros()/millis(). The
 * real value is programmed by the System Manager/U-Boot; it is defined (with
 * SystemCoreClockUpdate()) alongside the startup/console code in
 * platform_stubs.c and referenced here.
 */
extern uint32_t SystemCoreClock;

/* 64-bit cycle accumulator state (see navq95_cycles64() below). */
static volatile uint32_t s_cycPrev;
static volatile uint64_t s_cycAccum;

/* ── Cycle counter (scheduler timebase) ─────────────────────────────────── */

void cycleCounterInit(void)
{
    /* Enable the DWT cycle counter: TRCENA then CYCCNTENA. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#ifdef DWT_LAR_UNLOCK
    DWT->LAR = 0xC5ACCE55;                      /* unlock (some cores need it) */
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
 * Microsecond/millisecond time base from the Cortex-M7 DWT cycle counter
 * (free-running at the core clock, no peripheral setup). A 64-bit accumulator
 * folds the 32-bit CYCCNT wraps so the time is monotonic and never loses a
 * wrap.
 *
 * Correctness requirements that the previous accumulator-only version missed:
 *   1. CYCCNT wraps every 2^32 cycles (~5.37 s @ 800 MHz). A single 32-bit
 *      delta can only ever capture ONE wrap, so if the fold is not performed
 *      at least once per wrap period the clock stalls. We therefore keep a
 *      64-bit cycle base and fold on every read AND from the SysTick ISR, so
 *      the maximum interval between folds is bounded well under one wrap.
 *   2. The fold is a read-modify-write of shared state and is reachable from
 *      both thread and interrupt context (micros()/microsISR()). It must be
 *      executed atomically or the delta is double-counted / dropped. We mask
 *      interrupts around the fold using a BASEPRI/PRIMASK critical section.
 */

/* Return the full 64-bit free-running cycle count, folding CYCCNT wraps.
 * Interrupt-safe: the read-modify-write runs with interrupts masked. */
static inline uint64_t navq95_cycles64(void)
{
    uint64_t cycles;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        uint32_t now = DWT->CYCCNT;
        uint32_t delta = now - s_cycPrev;      /* unsigned wrap-safe (1 wrap) */
        s_cycPrev = now;
        s_cycAccum += delta;
        cycles = s_cycAccum;
    }

    return cycles;
}

/* Called from the SysTick handler (1 kHz) so the fold runs far more often than
 * once per CYCCNT wrap period, guaranteeing no wrap is ever lost even if the
 * application does not call micros()/millis() for several seconds. */
void navq95_timebaseTick(void)
{
    (void)navq95_cycles64();
}

/*
 * Configure the Cortex-M7 SysTick to interrupt at NAVQ95_TIMEBASE_TICK_HZ (1 kHz)
 * so navq95_timebaseTick() folds the CYCCNT wrap ~1000x per wrap period
 * (~5.37 s @ 800 MHz). This removes the dependency on the application polling
 * micros()/millis() often enough to catch every wrap.
 *
 * SysTick is clocked from the core clock (CLKSOURCE = 1). RELOAD must fit in
 * 24 bits: at 800 MHz / 1 kHz = 800000 reload, well under 2^24-1 (16777215),
 * so a 1 kHz tick is representable for core clocks up to ~16.7 GHz. Call AFTER
 * cycleCounterInit() (DWT must be running) and with SystemCoreClock set.
 */
#define NAVQ95_TIMEBASE_TICK_HZ 1000U

void navq95_timebaseInit(void)
{
    uint32_t reload = (SystemCoreClock / NAVQ95_TIMEBASE_TICK_HZ);
    if (reload == 0U) {
        reload = 1U;
    }
    /* RELOAD is a 24-bit field; the value written is reload-1. */
    SysTick->LOAD = (reload - 1U) & SysTick_LOAD_RELOAD_Msk;
    SysTick->VAL  = 0U;                          /* clear current value + flag */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   /* core clock                 */
                  | SysTick_CTRL_TICKINT_Msk     /* enable SysTick interrupt   */
                  | SysTick_CTRL_ENABLE_Msk;     /* start the counter          */
}

/* Strong override of the weak SysTick_Handler alias in startup_navq95.s. Runs
 * at NAVQ95_TIMEBASE_TICK_HZ and only folds the CYCCNT wrap; the fold itself is
 * interrupt-safe (ATOMIC_BLOCK inside navq95_cycles64()). */
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

/* ── Cycle-count conversion helpers ─────────────────────────────────────── */
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

/* ── Failure indication (LED + beeper) ──────────────────────────────────────
 * Standard Betaflight blink/beep pattern, mirroring
 * platform/common/stm32/system.c. The LED backend is not yet wired on NavQ95
 * (LED_R/LED_G/LED_B hang off a PCAL6416A I/O-expander on LPI2C4, not a direct
 * SoC GPIO), so the LED_TOGGLE calls are commented out until that driver lands.
 * The beeper path (BEEP_ON/BEEP_OFF) is active once USE_BEEPER + BEEPER_PIN are
 * configured; it is a no-op otherwise. */
static void indicate(uint8_t count, uint16_t duration)
{
    if (count) {
        /* LED0_ON; */

        while (count--) {
            /* LED0_TOGGLE; */
            BEEP_ON;
            delay(duration);

            /* LED0_TOGGLE; */
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
    /* Indicate the failure code a few times, then warm-reset the core via the
     * real SCMI systemReset() (platform_stubs.c). */
    indicateFailure(mode, 10);

    systemReset();

    /* systemReset() does not return on success; keep indicating as a fallback. */
    for (;;) {
        indicateFailure(mode, 1);
    }
}

/*
 * Reset (systemReset/systemResetToBootloader) uses the real SCMI warm reset and
 * lives in platform_stubs.c. isMPUSoftReset/unusedPinsInit are still stubs there.
 */

