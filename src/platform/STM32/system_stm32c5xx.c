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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "stm32c5xx_hal_flash.h"
#include "stm32c5xx_ll_rcc.h"

/* Global flash handle for compat shims (HAL_FLASH_Unlock/Lock/Program/Erase) */
hal_flash_handle_t hflash_compat;

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/memprot.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

bool isMPUSoftReset(void)
{
    if (cachedResetFlags & RCC_RSR_SFTRSTF)
        return true;
    else
        return false;
}

// Initialise the .dmaram_data and .dmaram_bss linker-defined sections.
// Mirrors what initialiseDmaMemorySections()/initialiseD2MemorySections()
// do on H5/H7: the standard startup loop only covers .data/.bss, so any
// initialised data placed in .dmaram_data (DMA_DATA macro) or zero-init
// state in .dmaram_bss (DMA_DATA_ZERO_INIT) needs an explicit copy/zero.
static void initialiseDmaMemorySections(void)
{
    extern uint8_t _sdmaram_bss;
    extern uint8_t _edmaram_bss;
    extern uint8_t _sdmaram_data;
    extern uint8_t _edmaram_data;
    extern uint8_t _sdmaram_idata;
    bzero(&_sdmaram_bss, (size_t) (&_edmaram_bss - &_sdmaram_bss));
    memcpy(&_sdmaram_data, &_sdmaram_idata, (size_t) (&_edmaram_data - &_sdmaram_data));
}

// Switch the system clock from the reset-default HSIDIV3 (48 MHz) to the
// HSIS source running at the full HSI rate (144 MHz). Done before HAL_Init
// so the SysTick reload it computes from SystemCoreClock matches the
// running HCLK. We don't need PSI/PLL here: HSI is always on after reset
// and HSIS taps it directly, so the only step is to bump flash latency,
// switch SWS, and set the bus prescalers to 1.
static void systemClockTo144MHz(void)
{
    // 144 MHz needs 4 wait states on the C5 flash interface.
    HAL_FLASH_ITF_SetLatency(HAL_FLASH, HAL_FLASH_ITF_LATENCY_4);

    // Make sure HSIS is enabled; on reset HSI is on but HSIS gating depends
    // on the boot ROM. Enable + wait-ready before switching.
    if (HAL_RCC_HSIS_IsReady() != HAL_RCC_OSC_READY) {
        HAL_RCC_HSIS_Enable();
        while (HAL_RCC_HSIS_IsReady() != HAL_RCC_OSC_READY) {
        }
    }

    // HSIK feeds the peripheral kernel-clock muxes (CCIPR*.xxxSEL). It is
    // gated independently from HSI/HSIS via RCC_CR1_HSIKON and is OFF
    // after reset, so any peripheral selecting HSIK sees no clock until
    // it's enabled here. Divider stays at the reset default (DIV_1) so
    // HSIK == 144 MHz.
    if (LL_RCC_HSIK_IsReady() == 0) {
        LL_RCC_HSIK_Enable();
        while (LL_RCC_HSIK_IsReady() == 0) {
        }
    }

    HAL_RCC_SetSYSCLKSource(HAL_RCC_SYSCLK_SRC_HSIS);

    // Programming delay must follow the SYSCLK switch when running fast.
    HAL_FLASH_ITF_SetProgrammingDelay(HAL_FLASH, HAL_FLASH_ITF_PROGRAM_DELAY_2);

    // HCLK = SYSCLK, PCLK1/2/3 = HCLK so SPI1/USART1/etc. all see 144 MHz.
    hal_rcc_bus_clk_config_t busConfig = {
        .hclk_prescaler  = HAL_RCC_HCLK_PRESCALER1,
        .pclk1_prescaler = HAL_RCC_PCLK_PRESCALER1,
        .pclk2_prescaler = HAL_RCC_PCLK_PRESCALER1,
        .pclk3_prescaler = HAL_RCC_PCLK_PRESCALER1,
    };
    HAL_RCC_SetBusClockConfig(&busConfig);
}

void systemInit(void)
{
    // Copy .dmaram_data from flash and zero .dmaram_bss before anything
    // that might touch DMA_DATA-placed variables (the standard Reset_Handler
    // .data/.bss copy/clear loop doesn't cover these sections).
    initialiseDmaMemorySections();

    memProtReset();
    memProtConfigure(mpuRegions, mpuRegionCount);

    // Bump SYSCLK from HSIDIV3 (48 MHz) to HSIS (144 MHz) before HAL_Init
    // so SysTick is configured against the final HCLK.
    systemClockTo144MHz();

    // Bring up the HAL tick (SysTick at HAL_TICK_FREQ_DEFAULT = 1 kHz) so
    // micros()/delay() advance. HAL_Init also sets the SysTick clock source
    // to the CPU internal clock and configures NVIC priority grouping for
    // the HAL stack. Other STM32 platforms call this from the startup
    // SystemInit(), but on C5 .data is initialised after SystemInit so we
    // call it here instead -- by this point .data is live.
    HAL_Init();

    // ICACHE is intentionally left at reset default (disabled) on STM32C5.
    //
    // Despite the name, ICACHE on Cortex-M33 sits on the C-AHB code bus
    // and caches *anything* fetched through it -- not only instructions.
    // SRAM accessed via the code-region alias (addresses below 0x2000_0000)
    // and const data in flash both go through ICACHE; the system-bus alias
    // (0x2000_xxxx) bypasses it. ICACHE has only one maintenance op (full
    // invalidate), so per-region exclusion via MPU is the only knob.
    //
    // Enabling it here without an audit risks:
    //   - stale reads from DMA buffers reached via the code-region alias
    //     (DMA writes are not snooped by ICACHE);
    //   - stale flash reads from regions that change at runtime (option
    //     bytes after a write, OTP, anything reprogrammed by a bootloader
    //     path).
    //
    // Before turning this on a future change must (a) confirm SRAM access
    // always uses the system-bus alias, (b) add MPU non-cacheable regions
    // for OTP and any runtime-rewritten flash per ST guidance, and
    // (c) audit DMA buffer access patterns.

    // Configure NVIC preempt/priority groups (CMSIS direct, HAL2 has no wrapper)
    NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->RSR value to use it in isMPUSoftReset() and others
    cachedResetFlags = RCC->RSR;

    // Init cycle counter
    cycleCounterInit();
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetWithoutDisablingCaches(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

        break;
    }

    __disable_irq();
    NVIC_SystemReset();
}

// STM32C5 system flash (ROM bootloader) base address
#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)FLASH_SYSTEM_BASE)

typedef void *(*bootJumpPtr)(void);

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;

void systemJumpToBootloader(void)
{
    //DeInit all used peripherals
    HAL_RCC_Reset();

    //Disable all system timers and set to default values
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    //Disable all interrupts
    __disable_irq();

    // Use VTOR to point directly to the system flash vector table.
    SCB->VTOR = (uint32_t)SYSMEMBOOT_VECTOR_TABLE;

    //default bootloader call stack routine
    uint32_t bootStack = SYSMEMBOOT_VECTOR_TABLE[0];

    bootJumpPtr SysMemBootJump = (bootJumpPtr)SYSMEMBOOT_VECTOR_TABLE[1];

    __set_MSP(bootStack); //Set the main stack pointer to its default values

    SysMemBootJump();

    while (1);
}

void systemProcessResetReason(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
    case RESET_BOOTLOADER_REQUEST_ROM:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        systemJumpToBootloader();

        break;

    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        break;

    case RESET_BOOTLOADER_POST:
        // Boot loader activity magically prevents SysTick from interrupting.
        // Issue a soft reset to prevent the condition.
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FORCED);
        systemResetWithoutDisablingCaches();

        break;

    case RESET_MSC_REQUEST:
    case RESET_NONE:
    default:
        break;
    }
}
