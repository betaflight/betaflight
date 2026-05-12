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

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

void SystemCoreClockUpdate(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    SystemCoreClock = clocks.M7ClkFreq;
}

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
    while (1) {
    }
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
    case BOOTLOADER_REQUEST_FLASH:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_FLASH);
        break;

    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);
        break;
    }

    systemReset();
}

void systemProcessResetReason(void)
{
    const uint32_t resetReason = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (resetReason) {
    case RESET_BOOTLOADER_REQUEST_FLASH:
    case RESET_BOOTLOADER_REQUEST_ROM:
        // Keep bootloader requests one-shot until the X32 ROM boot path is wired in.
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        break;

    case RESET_BOOTLOADER_POST:
    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        break;

    case RESET_MSC_REQUEST:
    case RESET_NONE:
    default:
        break;
    }
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    RCC_EnableAHB5PeriphClk1(
        RCC_AHB5_PERIPHEN_M7_GPIOA |
        RCC_AHB5_PERIPHEN_M7_GPIOB |
        RCC_AHB5_PERIPHEN_M7_GPIOC |
        RCC_AHB5_PERIPHEN_M7_GPIOD |
        RCC_AHB5_PERIPHEN_M7_GPIOE |
        RCC_AHB5_PERIPHEN_M7_GPIOF |
        RCC_AHB5_PERIPHEN_M7_GPIOG |
        RCC_AHB5_PERIPHEN_M7_GPIOH,
        ENABLE);

    RCC_EnableAHB5PeriphClk2(
        RCC_AHB5_PERIPHEN_M7_GPIOI |
        RCC_AHB5_PERIPHEN_M7_GPIOJ |
        RCC_AHB5_PERIPHEN_M7_GPIOK |
        RCC_AHB5_PERIPHEN_M7_AFIO,
        ENABLE);
}

bool isMPUSoftReset(void)
{
    return (cachedResetFlags & RCC_CTRLSTS_CM7SFTRSTF) != 0;
}


void systemInit(void)
{
    persistentObjectInit();
    systemProcessResetReason();

    RCC_ConfigHse(RCC_HSE_ENABLE);
    while (RCC_WaitHseStable() != SUCCESS);
    RCC_SetSysClkToMode0();
    
    SystemCoreClockUpdate();
    //USE for SD Card
    RCC_ConfigPll2(RCC_PLL_SRC_HSI,64000000,500000000,ENABLE);
    /* configure PLL1A is PLL1 */
    RCC_ConfigPLL2ADivider(RCC_PLLA_DIV5);

    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);

    cachedResetFlags = RCC->CTRLSTS;

#ifdef VECT_TAB_SRAM
    extern uint8_t isr_vector_table_base;
    SCB->VTOR = (uint32_t)&isr_vector_table_base;
#endif
    
    RCC_ClearResetFlag();

    enableGPIOPowerUsageAndNoiseReductions();

    cycleCounterInit();

    SysTick_Config(SystemCoreClock / 1000);
}
