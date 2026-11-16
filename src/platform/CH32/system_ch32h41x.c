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
 * porting for ch32h41x by Temperslee
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/resource.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/usb_io.h"

// HCLK cycles per microsecond (SysTick1 clock source)
static uint32_t usTicks = 0;
// Current uptime for 1kHz SysTick1 timer; will rollover after 49 days.
static volatile uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;
static volatile int sysTickPending = 0;
// Cached value of RCC reset status register
uint32_t cachedResetFlags;

#define CYCLE_TICKS_PER_US (SystemCoreClock / 1000000)

void cycleCounterInit(void)
{
    usTicks = HCLKClock / 1000000;
    sysTickValStamp = HCLKClock / 1000 - 1;

    asm volatile ("csrw mucounteren, %0" : : "r" (0x5));
    asm volatile ("csrw mcycle, %0" : : "r" (0));
    asm volatile ("csrw mucounteren, %0" : : "r" (0));
}

__FAST_INTERRUPT
void SysTick1_Handler(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        sysTickUptime++;
        sysTickValStamp = SysTick1->CNT;
        sysTickPending = 0;
        SysTick0->ISR &= ~(1U << 1);
    }
}

// Return system uptime in microseconds (rollover in 70 minutes)
MMFLASH_CODE_NOINLINE uint32_t microsISR(void)
{
    volatile uint32_t ms;
    volatile uint32_t pending;
    volatile uint32_t cycleCnt;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        cycleCnt = SysTick1->CNT;
        if (SysTick0->ISR & (1U << 1)) {
            sysTickPending = 1;
            cycleCnt = SysTick1->CNT;
        }
        ms = sysTickUptime;
        pending = sysTickPending;
    }

    return ((ms + pending) * 1000) + (usTicks * 1000 - cycleCnt) / usTicks;
}

uint32_t micros(void)
{
    uint32_t ms;
    uint32_t cycleCnt;

    if ((PFIC->GISR & 0xFF) || (__get_BASEPRI()) || !ch32GlobalInterruptsEnabled()) {
        return microsISR();
    }

    do {
        ms = sysTickUptime;
        cycleCnt = SysTick1->CNT;
    } while (ms != sysTickUptime || cycleCnt > sysTickValStamp);

    return (ms * 1000) + (usTicks * 1000 - cycleCnt) / usTicks;
}

uint32_t getCycleCounter(void)
{
    uint32_t result;
    asm volatile ("csrr %0, mcycle" : "=r" (result));
    return result;
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / (int32_t)CYCLE_TICKS_PER_US;
}

float clockCyclesToMicrosf(int32_t clockCycles)
{
    return clockCycles * (1e6f / SystemCoreClock);
}

// Signed conversion used for periods rather than absolute timestamps
int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return (int32_t)((int64_t)clockCycles * 10 / (int32_t)CYCLE_TICKS_PER_US);
}

int32_t clockCyclesTo100thMicros(int32_t clockCycles)
{
    return (int32_t)((int64_t)clockCycles * 100 / (int32_t)CYCLE_TICKS_PER_US);
}

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * CYCLE_TICKS_PER_US;
}

uint32_t millis(void)
{
    return sysTickUptime;
}

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us) {
    }
}

void delay(uint32_t ms)
{
    while (ms--) {
        delayMicroseconds(1000);
    }
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

#if defined(DEBUG) || !PLATFORM_SUPPORTS_SOFTWARE_BOOTLOADER
    systemReset();
#else
    systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
#endif
}

void initialiseMemorySections(void)
{
#ifdef USE_ITCM_RAM
    /* Load fast-functions into ITCM RAM */
    extern uint8_t tcm_code_start;
    extern uint8_t tcm_code_end;
    extern uint8_t tcm_code;
    memcpy(&tcm_code_start, &tcm_code, (size_t)(&tcm_code_end - &tcm_code_start));
#endif

#ifdef USE_CCM_CODE
    /* Load functions into RAM */
    extern uint8_t ccm_code_start;
    extern uint8_t ccm_code_end;
    extern uint8_t ccm_code;
    memcpy(&ccm_code_start, &ccm_code, (size_t)(&ccm_code_end - &ccm_code_start));
#endif

#ifdef USE_FAST_DATA
    /* Load FAST_DATA variable initializers into DTCM RAM */
    extern uint8_t _sfastram_data;
    extern uint8_t _efastram_data;
    extern uint8_t _sfastram_idata;
    memcpy(&_sfastram_data, &_sfastram_idata, (size_t)(&_efastram_data - &_sfastram_data));
#endif

#ifdef USE_RAM_CODE
    /* Load slow-functions into ITCM RAM */
    extern uint8_t ram_code_start;
    extern uint8_t ram_code_end;
    extern uint8_t ram_code;
    memcpy(&ram_code_start, &ram_code, (size_t)(&ram_code_end - &ram_code_start));
#endif
}

static void unusedPinInit(IO_t io)
{
    if (IOGetOwner(io) == OWNER_FREE) {
        IOConfigGPIO(io, IOCFG_IPU);
    }
}

void unusedPinsInit(void)
{
    IOTraversePins(unusedPinInit);
}

#ifndef USE_VCP
bool usbCableIsInserted(void)
{
    return false;
}

bool usbCableIsActive(void)
{
    return false;
}
#endif

#define CH32_FLASH_RESET_WAIT_LIMIT 0x02000000U

static void ch32WaitFlashIdleBeforeReset(void)
{
#ifdef __riscv
    asm volatile ("fence iorw, iorw" ::: "memory");
#endif
    for (uint32_t timeout = CH32_FLASH_RESET_WAIT_LIMIT;
         timeout != 0U && (FLASH->STATR & (FLASH_STATR_BSY | FLASH_STATR_WRBSY)) != 0U;
         timeout--) {
    }
}

void systemReset(void)
{
    __disable_irq();
    ch32WaitFlashIdleBeforeReset();
#ifdef __riscv
    asm volatile ("fence iorw, iorw" ::: "memory");
#endif
    NVIC_SystemReset();
#ifdef __riscv
    asm volatile ("fence iorw, iorw" ::: "memory");
#endif
    while (true) {
        __NOP();
    }
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    // Protocol entry points reject this operation. Keep a normal reset as a
    // fallback for direct callers until the paired bootloader supports it.
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
#ifdef __riscv
    asm volatile ("fence iorw, iorw" ::: "memory");
#endif
    systemReset();
}

static void checkForBootLoaderRequest(void)
{
    const uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);
    if (bootloaderRequest == RESET_BOOTLOADER_REQUEST_ROM || bootloaderRequest == RESET_BOOTLOADER_REQUEST_FLASH) {
        // Ignore requests left by older firmware. WCH ROM ISP is not the
        // H415 USB DFU bootloader; never change FLASH_STATR_BOOT_MODE here.
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
    }
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    RCC_HBPeriphClockCmd(
        RCC_HBPeriph_DMA1 |
        RCC_HBPeriph_DMA2 |
        RCC_HBPeriph_PIOC,
        ENABLE
    );
#if defined(USE_VCP) || defined(USE_USB_MSC)
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);
#endif
    RCC_HB1PeriphClockCmd(
        RCC_HB1Periph_TIM2 |
        RCC_HB1Periph_TIM3 |
        RCC_HB1Periph_TIM4 |
        RCC_HB1Periph_TIM5 |
        RCC_HB1Periph_TIM6 |
        RCC_HB1Periph_TIM7 |
        RCC_HB1Periph_USART6 |
        RCC_HB1Periph_USART7 |
        RCC_HB1Periph_USART8 |
        RCC_HB1Periph_LPTIM1 |
        RCC_HB1Periph_LPTIM2 |
        RCC_HB1Periph_WWDG |
        RCC_HB1Periph_SPI2 |
        RCC_HB1Periph_SPI3 |
        RCC_HB1Periph_SPI4 |
        RCC_HB1Periph_USART2 |
        RCC_HB1Periph_USART3 |
        RCC_HB1Periph_USART4 |
        RCC_HB1Periph_USART5 |
        RCC_HB1Periph_I2C1 |
        RCC_HB1Periph_I2C2 |
        RCC_HB1Periph_CAN3 |
        RCC_HB1Periph_CAN1 |
        RCC_HB1Periph_CAN2 |
        RCC_HB1Periph_BKP |
        RCC_HB1Periph_PWR |
        RCC_HB1Periph_DAC |
        RCC_HB1Periph_I2C3 |
        RCC_HB1Periph_SWPMI,
        ENABLE
    );
    RCC_HB2PeriphClockCmd(
        RCC_HB2Periph_AFIO |
        RCC_HB2Periph_GPIOA |
        RCC_HB2Periph_GPIOB |
        RCC_HB2Periph_GPIOC |
        RCC_HB2Periph_GPIOD |
        RCC_HB2Periph_GPIOE |
        RCC_HB2Periph_GPIOF |
        RCC_HB2Periph_ADC1 |
        RCC_HB2Periph_ADC2 |
        RCC_HB2Periph_TIM1 |
        RCC_HB2Periph_SPI1 |
        RCC_HB2Periph_TIM8 |
        RCC_HB2Periph_USART1 |
        RCC_HB2Periph_I2C4 |
        RCC_HB2Periph_SDIO |
        RCC_HB2Periph_TIM9 |
        RCC_HB2Periph_TIM10 |
        RCC_HB2Periph_TIM11 |
        RCC_HB2Periph_TIM12,
        ENABLE
    );
}

bool isMPUSoftReset(void)
{
    if (cachedResetFlags & RCC_SFTRSTF) {
        return true;
    } else {
        return false;
    }
}

uint32_t SysTick_Config(uint32_t ticks)
{
    if ((ticks - 1UL) > 0xFFFFFFFFUL) {
        return 1UL;                                                     /* Reload value impossible */
    }
    SysTick0->ISR &= ~(1U << 1);                                        /* Clear flag */
    SysTick1->CMP = (uint32_t)(ticks - 1UL);                            /* Set reload register */
    SysTick1->CNT = 0UL;                                                /* Load the SysTick counter value */
    SysTick1->CTLR = (1U << 6) | (1U << 4) | (1U << 3) | (1U << 2) | (1U << 1) | (1U << 0);
    NVIC_SetPriority(SysTick1_IRQn, NVIC_BUILD_PRIORITY(3, 0));         /* Lowest preemption group */
    NVIC_EnableIRQ(SysTick1_IRQn);
    return 0UL;                                                         /* Function successful */
}

void systemInit(void)
{
    SystemAndCoreClockUpdate();

    persistentObjectInit();

    checkForBootLoaderRequest();

    // Cache RCC->RSTSCKR value to use it in isMPUSoftReset() and others
    cachedResetFlags = RCC->RSTSCKR;

    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS | RCC_HBPeriph_OTG_FS, DISABLE);

    RCC_ClearFlag();
    enableGPIOPowerUsageAndNoiseReductions();

    cycleCounterInit();
    SysTick_Config(HCLKClock / 1000);

    SWPMI->OR |= 1; // Disable SWPMI, enable GPIO input
}
