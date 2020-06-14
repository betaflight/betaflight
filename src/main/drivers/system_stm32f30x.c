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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/nvic.h"
#include "drivers/system.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

#define BOOTLOADER_MAGIC 0xDEADBEEF
// 40KB SRAM STM32F30X
#define BOOT_TARGET_REGISTER ((uint32_t *)0x20009FFC)

#define DEFAULT_STACK_POINTER ((uint32_t *)0x1FFFD800)
#define SYSTEM_MEMORY_RESET_VECTOR ((uint32_t *) 0x1FFFD804)

void SetSysClock();

void systemReset(void)
{
    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);
    // 1FFFF000 -> 20000200 -> SP
    // 1FFFF004 -> 1FFFF021 -> PC

    *BOOT_TARGET_REGISTER = BOOTLOADER_MAGIC;

    systemReset();
}


void enableGPIOPowerUsageAndNoiseReductions(void)
{
}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_CSR_SFTRSTF)
        return true;
    else
        return false;
}

static void checkForBootLoaderRequest(void)
{
    if (*BOOT_TARGET_REGISTER == BOOTLOADER_MAGIC) {

        *BOOT_TARGET_REGISTER = 0x0;

        __enable_irq();
        __set_MSP(*DEFAULT_STACK_POINTER);

        ((void(*)(void))(*SYSTEM_MEMORY_RESET_VECTOR))();

        while (1);
    }
}

void systemInit(void)
{
    checkForBootLoaderRequest();

    // Enable FPU
    SCB->CPACR = (0x3 << (10 * 2)) | (0x3 << (11 * 2));
    SetSysClock();

    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);

    // cache RCC->CSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->CSR;
    RCC_ClearFlag();

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}
