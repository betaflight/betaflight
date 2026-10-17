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
 * Time base is TSTMR2, the 64 bit 24 MHz counter the System Manager runs for every core; it never wraps.
 * A cycle is 1/32 TSTMR2 tick, about one 800 MHz core clock. DWT is not used: a debugger clearing DEMCR stops it.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/time.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "scmi_imx95.h"

#define TSTMR_TICKS_PER_US      24u
#define CYCLES_PER_TICK_SHIFT   5
#define CYCLES_PER_US           (TSTMR_TICKS_PER_US << CYCLES_PER_TICK_SHIFT)

STATIC_ASSERT(sizeof(timeMs_t) == sizeof(uint32_t), timeMs_t_is_32_bit_failed);
STATIC_ASSERT(sizeof(timeUs_t) == sizeof(uint32_t), timeUs_t_is_32_bit_failed);

uint32_t cachedResetFlags = 0;

// TSTMR2 counts from System Manager boot, so time on this core counts from systemInit().
static uint64_t bootTicks;

// Reading L holds the count until H is read, so both words are read with interrupts off.
FAST_CODE static inline uint64_t tstmrRead(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t low = TSTMR2->L;
    const uint32_t high = TSTMR2->H;
    __set_PRIMASK(primask);
    return (((uint64_t)high << 32) | low) - bootTicks;
}

FAST_CODE timeUs_t micros(void)
{
    return (timeUs_t)(tstmrRead() / TSTMR_TICKS_PER_US);
}

FAST_CODE timeUs_t microsISR(void)
{
    return micros();
}

FAST_CODE timeMs_t millis(void)
{
    return (timeMs_t)(tstmrRead() / (TSTMR_TICKS_PER_US * 1000u));
}

void delayMicroseconds(uint32_t us)
{
    const uint32_t start = (uint32_t)tstmrRead();

    while ((uint32_t)tstmrRead() - start < us * TSTMR_TICKS_PER_US) {
    }
}

void delay(uint32_t ms)
{
    while (ms--) {
        delayMicroseconds(1000);
    }
}

FAST_CODE uint32_t getCycleCounter(void)
{
    return (uint32_t)tstmrRead() << CYCLES_PER_TICK_SHIFT;
}

FAST_CODE int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / (int32_t)CYCLES_PER_US;
}

FAST_CODE float clockCyclesToMicrosf(int32_t clockCycles)
{
    return clockCycles * (1.0f / CYCLES_PER_US);
}

FAST_CODE int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return 10 * clockCycles / (int32_t)CYCLES_PER_US;
}

FAST_CODE int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return 100 * clockCycles / (int32_t)CYCLES_PER_US;
}

FAST_CODE uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * CYCLES_PER_US;
}

static char mcuTypeName[SCMI_SI_NAME_LENGTH + 1] = "IMX95";

const char *nxpMcuTypeName(void)
{
    return mcuTypeName;
}

void systemInit(void)
{
    // tstmrRead() subtracts the old base, so this sets the base to the raw count.
    bootTicks += tstmrRead();
    persistentObjectInit();
    systemProcessResetReason();
    scmiSiNameGet(mcuTypeName);

    NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING >> 8);
}

// Bounds the wait for a TPM to take the stop into its counter clock domain.
#define TPM_STOP_POLLS 1000

// Called from the reset handler before the TCMs are seeded: stays in flash, word sized stack accesses only.
void imx95StopOwnedPeripherals(void)
{
    static TPM_Type *const tpms[] = TPM_BASE_PTRS;
    for (unsigned i = 1; i < ARRAYLEN(tpms); i++) {
        if ((NXP_TPM_INSTANCE_MASK >> i) & 1) {
            for (unsigned channel = 0; channel < TPM_CONTROLS_COUNT; channel++) {
                tpms[i]->CONTROLS[channel].CnSC = 0;
            }
            tpms[i]->SC = 0;
            for (unsigned polls = 0; (tpms[i]->SC & TPM_SC_CMOD_MASK) && polls < TPM_STOP_POLLS; polls++) {
            }
        }
    }

#ifdef USE_DSHOT
    static FLEXIO_Type *const flexios[] = FLEXIO_BASE_PTRS;
    flexios[NXP_DSHOT_FLEXIO_INSTANCE]->CTRL = 0;
#endif
}

extern const uint32_t __bootVectors[];

// On a refused SCMI reset, thread mode re-enters the reset handler; a handler cannot leave its
// exception that way, so it waits for the host to restart the core.
void systemReset(void)
{
    __disable_irq();

    // On success the System Manager resets this core; the jump below is for a refusal.
    if (scmiSystemReset() == SCMI_SUCCESS) {
        while (true) {
        }
    }

    if (__get_IPSR() == 0) {
        __set_CONTROL(0);
        __ISB();
        ((void (*)(void))__bootVectors[1])();
    }

    while (true) {
    }
}

// There is no bootloader the firmware can enter on this core.
void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    systemReset();
}

bool isMPUSoftReset(void)
{
    return false;
}

void systemProcessResetReason(void)
{
    cachedResetFlags = 0;
}

static void indicate(uint8_t count, uint16_t duration)
{
    if (count) {
        LED1_ON;
        LED0_OFF;

        while (count--) {
            LED1_TOGGLE;
            LED0_TOGGLE;
            BEEP_ON;
            delay(duration);

            LED1_TOGGLE;
            LED0_TOGGLE;
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
}

// A pad no driver claimed may belong to another core, so the sweep stays off.
void unusedPinsInit(void)
{
}
