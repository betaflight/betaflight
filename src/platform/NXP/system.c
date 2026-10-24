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
 * System timing and reset handling for i.MXRT. The time base is SysTick at 1 kHz, as a
 * GPT read crosses the AIPS bridge. DWT CYCCNT is not used: the debug power domain can
 * stay off after a debugger reset. A cycle is one core clock.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/maths.h"
#include "common/time.h"
#include "build/debug.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "platform/nxp_clock.h"

#include "flexspi_nor_imxrt.h"
#include "romapi_imxrt.h"
#include "usb_cdc.h"

// PERCLK is the 24 MHz crystal; GPT2 is divided to one count per microsecond.
#define GPT_SOURCE_CLOCK_HZ     24000000u
#define GPT_MICROS_TICK_HZ      1000000u
#define GPT_MICROS_PRESCALER    (GPT_SOURCE_CLOCK_HZ / GPT_MICROS_TICK_HZ)

// GPT_CR CLKSRC value 1 selects the peripheral clock, the PERCLK root here.
#define GPT_CLKSRC_PERIPHERAL   1

STATIC_ASSERT(sizeof(timeMs_t) == sizeof(uint32_t), timeMs_t_is_32_bit_failed);
STATIC_ASSERT(sizeof(timeUs_t) == sizeof(uint32_t), timeUs_t_is_32_bit_failed);

uint32_t cachedResetFlags = 0;

static volatile uint32_t sysTickUptime = 0;

// The handler's view of the down counter, and a reload it has not counted yet.
// Both are for the race in sysTickSample().
static volatile uint32_t sysTickValStamp = 0;
static volatile int sysTickPending = 0;

// usTicks is what a cycle means here: core clocks per microsecond.
static uint32_t usTicks = 0;
static float usTicksInv = 0.0f;
static uint32_t cyclesPerMs = 0;

// GPT2 free runs off PERCLK: FRR wraps at 0xffffffff, ENMOD restarts from zero, DBGEN counts while halted.
#define GPT_FREE_RUNNING_CR     (GPT_CR_CLKSRC(GPT_CLKSRC_PERIPHERAL) \
                                | GPT_CR_FRR_MASK \
                                | GPT_CR_ENMOD_MASK \
                                | GPT_CR_DBGEN_MASK)

static void gptInit(void)
{
    nxpClockGateEnable(NXP_CLK_GPT2);

    GPT2->CR = 0;
    GPT2->CR = GPT_CR_SWR_MASK;
    // A software reset clears everything but the clock source and clears its own
    // bit when done. No deadline: this loop is what builds the clock for one.
    while (GPT2->CR & GPT_CR_SWR_MASK) {
    }
    GPT2->IR = 0;
    GPT2->SR = GPT2->SR;                    // status bits are write-one-to-clear
    // The prescaler field holds the divider minus one.
    GPT2->PR = GPT_PR_PRESCALER(GPT_MICROS_PRESCALER - 1);
    GPT2->CR = GPT_FREE_RUNNING_CR;

    // Nothing counts its wraps: the delays subtract modulo 2^32.
    GPT2->CR |= GPT_CR_EN_MASK;

    // Delays assume PERCLK is 24 MHz; checked after the counter starts, as failureMode() needs delay().
    if (nxpClockHz(NXP_CLK_GPT2) != GPT_SOURCE_CLOCK_HZ) {
        failureMode(FAILURE_DEVELOPER);
    }
}

// Ten SysTick reloads are ten thousand GPT2 counts at the stated core rate. One
// percent is looser than the reading error and tighter than a PLL misprogram.
#define SYSTICK_CHECK_MS        10u
#define SYSTICK_CHECK_US        (SYSTICK_CHECK_MS * 1000u)
#define SYSTICK_CHECK_MARGIN_US (SYSTICK_CHECK_US / 100u)

// Wait for a SysTick reload, giving up after deadlineUs GPT2 counts from
// startUs. Reading CTRL clears COUNTFLAG, so a set flag is always one reload.
static bool sysTickWaitReload(uint32_t startUs, uint32_t deadlineUs)
{
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
        if ((GPT2->CNT - startUs) > deadlineUs) {
            return false;
        }
    }

    return true;
}

// Time SysTick against GPT2. Must run before the tick interrupt is turned on:
// the handler reads CTRL as well, and would take COUNTFLAG away from here.
static void sysTickCheckRate(void)
{
    // Drop a flag standing from before, so the wait below sees a fresh reload.
    (void)SysTick->CTRL;

    // Line up with a reload, so the window is a whole number of periods.
    if (!sysTickWaitReload(GPT2->CNT, SYSTICK_CHECK_US)) {
        failureMode(FAILURE_DEVELOPER);
    }

    const uint32_t startUs = GPT2->CNT;

    for (unsigned i = 0; i < SYSTICK_CHECK_MS; i++) {
        if (!sysTickWaitReload(startUs, 2u * SYSTICK_CHECK_US)) {
            failureMode(FAILURE_DEVELOPER);
        }
    }

    const uint32_t elapsedUs = GPT2->CNT - startUs;

    if ((elapsedUs < (SYSTICK_CHECK_US - SYSTICK_CHECK_MARGIN_US))
        || (elapsedUs > (SYSTICK_CHECK_US + SYSTICK_CHECK_MARGIN_US))) {
        failureMode(FAILURE_DEVELOPER);
    }
}

// cycleCounterInit() has to have run: the reload comes from cyclesPerMs.
static void sysTickInit(void)
{
    // 576000 and 996000 both fit the 24 bit reload; zero or a huge rate would not.
    if ((cyclesPerMs == 0) || ((cyclesPerMs - 1u) > SysTick_LOAD_RELOAD_Msk)) {
        failureMode(FAILURE_DEVELOPER);
    }

    SysTick->CTRL = 0;
    SysTick->LOAD = cyclesPerMs - 1u;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    sysTickCheckRate();

    // Writing VAL clears COUNTFLAG; sysTickValStamp starts at the reload value.
    sysTickUptime = 0;
    sysTickValStamp = SysTick->LOAD;
    sysTickPending = 0;

    SysTick->VAL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

    NVIC_SetPriority(SysTick_IRQn, NVIC_PRIORITY_LEVEL(NVIC_PRIO_TIMER));

    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// The stamp tells a reader whether its counter belongs to the millisecond it read.
// PRIMASK rather than BASEPRI, since the priority 0 gyro interrupt reads these too.
FAST_IRQ_HANDLER void SysTick_Handler(void)
{
    __disable_irq();
    sysTickUptime++;
    sysTickValStamp = SysTick->VAL;
    sysTickPending = 0;
    (void)(SysTick->CTRL);
    __enable_irq();
}

// A consistent millisecond count and down counter pair. With interrupts masked a reload
// is taken from COUNTFLAG and kept in sysTickPending, as reading CTRL clears it.
typedef struct sysTickReading_s {
    uint32_t ms;            // milliseconds the handler has counted
    uint32_t val;           // SysTick down counter, cyclesPerMs - 1 down to 0
    uint32_t pending;       // a reload the handler has not counted yet
} sysTickReading_t;

FAST_CODE static inline sysTickReading_t sysTickSampleISR(void)
{
    sysTickReading_t reading;

    // PRIMASK as in SysTick_Handler, since the priority 0 gyro interrupt also reads CTRL.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    reading.val = SysTick->VAL;

    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
        sysTickPending = 1;

        // Read again, so the counter is one from after that reload.
        reading.val = SysTick->VAL;
    }

    reading.ms = sysTickUptime;
    reading.pending = (uint32_t)sysTickPending;

    __set_PRIMASK(primask);

    return reading;
}

FAST_CODE static inline sysTickReading_t sysTickSample(void)
{
    // In a handler, or with BASEPRI or PRIMASK raised, the tick cannot be served.
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || __get_BASEPRI() || __get_PRIMASK()) {
        return sysTickSampleISR();
    }

    sysTickReading_t reading;

    do {
        reading.ms = sysTickUptime;
        reading.val = SysTick->VAL;
    } while ((reading.ms != sysTickUptime) || (reading.val > sysTickValStamp));

    // The retry above already allowed for a reload; sysTickPending would count a
    // masked reader's standing flag a second time.
    reading.pending = 0;

    return reading;
}

// The image runs XIP from FlexSPI NOR. Everything from here to the end of the
// converters is on the scheduler's hot path, so it is placed in ITCM.

// Clocks into the current millisecond: 0 at the start, cyclesPerMs - 1 at the end.
#define SYSTICK_CLOCKS_IN(val)  (cyclesPerMs - 1u - (val))

// Microseconds since boot, wrapping at 2^32, which is what cmpTimeUs() assumes.
FAST_CODE timeUs_t micros(void)
{
    const sysTickReading_t now = sysTickSample();

    return (timeUs_t)(((now.ms + now.pending) * 1000u) + (SYSTICK_CLOCKS_IN(now.val) / usTicks));
}

FAST_CODE timeUs_t microsISR(void)
{
    const sysTickReading_t now = sysTickSampleISR();

    return (timeUs_t)(((now.ms + now.pending) * 1000u) + (SYSTICK_CLOCKS_IN(now.val) / usTicks));
}

FAST_CODE timeMs_t millis(void)
{
    return (timeMs_t)sysTickUptime;
}

void delayMicroseconds(uint32_t us)
{
    const uint32_t start = GPT2->CNT;

    // GPT2 rather than SysTick, as init paths call this with interrupts masked.
    while ((GPT2->CNT - start) < us) {
    }
}

void delay(uint32_t ms)
{
    while (ms--) {
        delayMicroseconds(1000);
    }
}

// SystemCoreClock is the rate the ARM PLL was programmed for, a whole number of
// clocks per microsecond and per millisecond. Runs before sysTickInit().
void cycleCounterInit(void)
{
    usTicks = SystemCoreClock / 1000000u;
    usTicksInv = 1e6f / (float)SystemCoreClock;
    cyclesPerMs = SystemCoreClock / 1000u;
}

// Core clocks since boot, from the millisecond count and the down counter.
FAST_CODE uint32_t getCycleCounter(void)
{
    const sysTickReading_t now = sysTickSample();

    return ((now.ms + now.pending) * cyclesPerMs) + SYSTICK_CLOCKS_IN(now.val);
}

// usTicks is cast to int32_t so the division stays signed: these carry periods,
// and dividing by an unsigned turns a negative period into a large positive one.
FAST_CODE int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / (int32_t)usTicks;
}

FAST_CODE float clockCyclesToMicrosf(int32_t clockCycles)
{
    return clockCycles * usTicksInv;
}

FAST_CODE int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return 10 * clockCycles / (int32_t)usTicks;
}

FAST_CODE int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return 100 * clockCycles / (int32_t)usTicks;
}

FAST_CODE uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * usTicks;
}

// Four priority bits split two and two, so ATOMIC_BLOCK() can write the value to BASEPRI.
#define ASSERT_NVIC_PACKING(base, sub, name) \
    STATIC_ASSERT((NVIC_BUILD_PRIORITY(base, sub) >> 4) == ((((base) & 3) << 2) | ((sub) & 3)), name)

ASSERT_NVIC_PACKING(0, 0, nvic_priority_0_0_must_pack_as_stm32_does);
ASSERT_NVIC_PACKING(0, 1, nvic_priority_0_1_must_pack_as_stm32_does);
ASSERT_NVIC_PACKING(1, 1, nvic_priority_1_1_must_pack_as_stm32_does);
ASSERT_NVIC_PACKING(1, 2, nvic_priority_1_2_must_pack_as_stm32_does);
ASSERT_NVIC_PACKING(2, 0, nvic_priority_2_0_must_pack_as_stm32_does);
ASSERT_NVIC_PACKING(2, 1, nvic_priority_2_1_must_pack_as_stm32_does);
ASSERT_NVIC_PACKING(3, 0, nvic_priority_3_0_must_pack_as_stm32_does);
ASSERT_NVIC_PACKING(3, 0x0f, nvic_priority_3_0f_must_saturate_as_stm32_does);
ASSERT_NVIC_PACKING(0x0f, 0x0f, nvic_priority_0f_0f_must_saturate_as_stm32_does);

void systemInit(void)
{
    // Clocks, MPU and caches are already up: the startup file runs them first.

    // Opens the SNVS gate, so it precedes anything reading a persistent object.
    persistentObjectInit();

    // The status register latches across a warm reset, so read and clear it now.
    systemProcessResetReason();

    NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING >> 8);

    // Runs from ITCM with interrupts masked, as it reconfigures the XIP FlexSPI.
    (void)flexspiNorInit();

    // In this order: gptInit() gives the delays a clock, cycleCounterInit()
    // works out the reload, and sysTickInit() checks its rate on GPT2.
    gptInit();
    cycleCounterInit();
    sysTickInit();
}

void systemReset(void)
{
    __disable_irq();

    // Leave the USB bus before the core goes, so the host sees a disconnect
    // rather than a device that stops answering mid transfer.
    cdcUsbDetach();

    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    if (requestType == BOOTLOADER_REQUEST_ROM) {
        // The ROM puts its own device on the bus, so this one leaves first.
        cdcUsbDetach();

        // Does not come back if the ROM takes the request.
        romRunSerialDownloader();
    }

    // BOOTLOADER_REQUEST_FLASH asks for a second bootloader in the firmware's
    // own flash, which this platform has none of, so it becomes a plain reset.
    systemReset();
}

// SRC_SRSR records why the last reset happened. On 10xx a core reset request and
// a lockup share one bit and only the vendor named bits may be written back; on
// 11xx every bit is write one to clear and only the CM7 half matters here.
#if defined(IMXRT11XX)
#define IMXRT_SRSR_SOFT_RESET_MASK  (SRC_SRSR_M7_REQUEST_M7_MASK | SRC_SRSR_M7_LOCKUP_M7_MASK)
#define IMXRT_SRSR_W1C_MASK         0xffffffffu
#elif defined(IMXRT10XX)
#define IMXRT_SRSR_SOFT_RESET_MASK  SRC_SRSR_LOCKUP_SYSRESETREQ_MASK
#define IMXRT_SRSR_W1C_MASK         SRC_SRSR_W1C_BITS_MASK
#else
#error "Unsupported i.MXRT family: no SRC_SRSR reset cause bits"
#endif

// The cached copy: systemProcessResetReason() has already cleared the register.
bool isMPUSoftReset(void)
{
    return (cachedResetFlags & IMXRT_SRSR_SOFT_RESET_MASK) != 0;
}

void systemProcessResetReason(void)
{
    cachedResetFlags = SRC->SRSR;

    // The bits latch across resets; only the write one to clear bits go back.
    SRC->SRSR = cachedResetFlags & IMXRT_SRSR_W1C_MASK;
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

#ifdef DEBUG
    systemReset();
#else
    // A board that cannot run its firmware lands in the ROM serial downloader,
    // where it can be reflashed.
    systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
#endif
}

static void unusedPinInit(IO_t io)
{
    if (IOGetOwner(io) == OWNER_FREE) {
        IOConfigGPIO(io, IOCFG_IN_FLOATING);
    }
}

void unusedPinsInit(void)
{
    IOTraversePins(unusedPinInit);
}
