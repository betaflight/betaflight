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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "gpio.h"
#include "light_led.h"
#include "sound_beeper.h"
#include "nvic.h"

#include "system.h"


#ifndef EXTI15_10_CALLBACK_HANDLER_COUNT
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1
#endif

static extiCallbackHandler* exti15_10_handlers[EXTI15_10_CALLBACK_HANDLER_COUNT];

void registerExti15_10_CallbackHandler(extiCallbackHandler *fn)
{
    for (int index = 0; index < EXTI15_10_CALLBACK_HANDLER_COUNT; index++) {
        extiCallbackHandler *candidate = exti15_10_handlers[index];
        if (!candidate) {
            exti15_10_handlers[index] = fn;
            return;
        }
    }
    failureMode(15); // EXTI15_10_CALLBACK_HANDLER_COUNT is too low for the amount of handlers required.
}

void unregisterExti15_10_CallbackHandler(extiCallbackHandler *fn)
{
    for (int index = 0; index < EXTI15_10_CALLBACK_HANDLER_COUNT; index++) {
        extiCallbackHandler *candidate = exti15_10_handlers[index];
        if (candidate == fn) {
            exti15_10_handlers[index] = 0;
            return;
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    for (int index = 0; index < EXTI15_10_CALLBACK_HANDLER_COUNT; index++) {
        extiCallbackHandler *fn = exti15_10_handlers[index];
        if (!fn) {
            continue;
        }
        fn();
    }
}

// cycles per microsecond
static uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;
// cached value of RCC->CSR
uint32_t cachedRccCsrValue;

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

void systemInit(void)
{
#ifdef CC3D
    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    extern void *isr_vector_table_base;

    NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
#endif
    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);

#ifdef STM32F10X
    // Turn on clocks for stuff we use
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

    // cache RCC->CSR value to use it in isMPUSoftreset() and others
    cachedRccCsrValue = RCC->CSR;
    RCC_ClearFlag();


    enableGPIOPowerUsageAndNoiseReductions();


#ifdef STM32F10X
    // Turn off JTAG port 'cause we're using the GPIO for leds
#define AFIO_MAPR_SWJ_CFG_NO_JTAG_SW            (0x2 << 24)
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_NO_JTAG_SW;
#endif

    // Init cycle counter
    cycleCounterInit();


    memset(&exti15_10_handlers, 0x00, sizeof(exti15_10_handlers));
    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}

#if 1
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}
#else
void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;

    for (;;) {
        register uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_us;

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

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

// FIXME replace mode with an enum so usage can be tracked, currently mode is a magic number
void failureMode(uint8_t mode)
{
    uint8_t flashesRemaining = 10;

    LED1_ON;
    LED0_OFF;
    while (flashesRemaining--) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(475 * mode - 2);
        BEEP_ON;
        delay(25);
        BEEP_OFF;
    }

    systemResetToBootloader();
}
