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

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

// System core clock variable required by CMSIS and HAL
uint32_t SystemCoreClock = 64000000; // Default to HSI frequency

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = HAL_RCC_GetSysClockFreq();
}

// Minimal _init for C runtime
void _init(void) {}

void SystemClock_Config(void);

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_RSR_SFTRSTF)
        return true;
    else
        return false;
}

bool isMemoryMappedModeEnabledOnBoot(void)
{
    return false;
}

void systemInit(void)
{
    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->RSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->RSR;

    // Enable AXI SRAM clocks
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM1EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM2EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM3EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM4EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM5EN);
    SET_BIT(RCC->MEMENSR, RCC_MEMENR_AXISRAM6EN);

    // Init cycle counter
    cycleCounterInit();

    // SysTick is updated whenever HAL_RCC_ClockConfig is called.
}

void systemReset(void)
{
    SCB_DisableDCache();
    SCB_DisableICache();

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
#if defined(USE_FLASH_BOOT_LOADER)
    case BOOTLOADER_REQUEST_FLASH:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_FLASH);

        break;
#endif
    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

        break;
    }

    __disable_irq();
    NVIC_SystemReset();
}

#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x0BF00000)

typedef void *(*bootJumpPtr)(void);

static void systemJumpToBootloader(void)
{
    __SYSCFG_CLK_ENABLE();

    uint32_t bootStack =  SYSMEMBOOT_VECTOR_TABLE[0];

    bootJumpPtr SysMemBootJump = (bootJumpPtr)SYSMEMBOOT_VECTOR_TABLE[1];

    __set_MSP(bootStack); //Set the main stack pointer to its default values

    SysMemBootJump();

    while (1);
}

void systemProcessResetReason(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
#if defined(USE_FLASH_BOOT_LOADER)
    case RESET_BOOTLOADER_REQUEST_FLASH:
#endif
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
        systemResetWithoutDisablingCaches(); // observed that disabling dcache after cold boot with BOOT pin high causes segfault.

        break;

    case RESET_MSC_REQUEST:
    case RESET_NONE:
    default:
        break;

    }
}
