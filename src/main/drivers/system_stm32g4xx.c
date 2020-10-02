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

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/memprot.h"
#include "drivers/persistent.h"
#include "drivers/system.h"

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_CSR_SFTRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
    memProtReset();
    memProtConfigure(mpuRegions, mpuRegionCount);

    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->RSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->CSR;

    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    //extern void *isr_vector_table_base;
    //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
    //__HAL_RCC_USB_OTG_FS_CLK_DISABLE;

    //RCC_ClearFlag();

    // Init cycle counter
    cycleCounterInit();

    // SysTick is updated whenever HAL_RCC_ClockConfig is called.

    // Disable UCPD pull-down functionality on PB4
    // AN5093 6.3.3:
    // (after device startup this pull-down on PB4 can be disabled by
    // setting UCPD1_DBDIS bit in the PWR_CR3 register).
    //
    // There is also a similar UCPD stand-by functionality control,
    // but don't fiddle with this one. In particular, disabling it by
    // calling HAL_PWREx_DisableUSBStandByModePD() will cause boot loader
    // invocation to fail.

#if defined(PWR_CR3_UCPD_DBDIS)
    HAL_PWREx_DisableUSBDeadBatteryPD();
#endif
}

void systemReset(void)
{
    // SCB_DisableDCache();
    // SCB_DisableICache();

    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);

    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

    __disable_irq();
    NVIC_SystemReset();
}

#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x1fff0000)

typedef void *(*bootJumpPtr)(void);

void systemJumpToBootloader(void)
{   
    __SYSCFG_CLK_ENABLE();
    
    uint32_t bootStack =  SYSMEMBOOT_VECTOR_TABLE[0];
    
    bootJumpPtr SysMemBootJump = (bootJumpPtr)SYSMEMBOOT_VECTOR_TABLE[1];
    
    __set_MSP(bootStack); //Set the main stack pointer to its default values
    
    SysMemBootJump();
    
    while (1);
}


void systemCheckResetReason(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
    case RESET_MSC_REQUEST:
        // RESET_REASON will be reset by MSC
    case RESET_NONE:
        return;

    case RESET_BOOTLOADER_REQUEST_ROM:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        break;;
    }

    systemJumpToBootloader();
}
