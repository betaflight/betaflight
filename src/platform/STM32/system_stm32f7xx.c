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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/persistent.h"

#include "stm32f7xx_ll_cortex.h"


#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

#define DEFAULT_STACK_POINTER ((uint32_t *) 0x1FF00000)
#define SYSTEM_MEMORY_RESET_VECTOR ((uint32_t *) 0x1FF00004)

void SystemClock_Config(void);

void systemReset(void)
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

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_CSR_SFTRSTF)
        return true;
    else
        return false;
}

static void checkForBootLoaderRequest(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);

    __SYSCFG_CLK_ENABLE();
    SYSCFG->MEMRMP |= SYSCFG_MEM_BOOT_ADD0 ;

    __set_MSP(*DEFAULT_STACK_POINTER);

    ((void (*)(void))*SYSTEM_MEMORY_RESET_VECTOR)();

    while (1);
}

void systemInit(void)
{
    persistentObjectInit();

    checkForBootLoaderRequest();

    //  Mark ITCM-RAM as read-only
    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER0, 0, RAMITCM_BASE, LL_MPU_REGION_SIZE_16KB | LL_MPU_REGION_PRIV_RO_URO);
    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);

    //SystemClock_Config();

    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->CSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->CSR;

    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    //extern void *isr_vector_table_base;
    //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
    //__HAL_RCC_USB_OTG_FS_CLK_DISABLE;

    //RCC_ClearFlag();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    //SysTick_Config(SystemCoreClock / 1000);
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}
