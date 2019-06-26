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

void forcedSystemResetWithoutDisablingCaches(void)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FORCED);

    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
#if defined(USE_FLASH_BOOT_LOADER)
    case BOATLOADER_REQUEST_FLASH:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FLASH_BOOTLOADER_REQUEST);

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

static uint32_t bootloaderRequest;

void systemCheckResetReason(void)
{
    bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
#if defined(USE_FLASH_BOOT_LOADER)
    case BOATLOADER_REQUEST_FLASH:
#endif
    case RESET_BOOTLOADER_REQUEST_ROM:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        break;

    case RESET_MSC_REQUEST:
        // RESET_REASON will be reset by MSC
        return;

    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        return;

    case RESET_NONE:
        if (!(RCC->CSR & RCC_CSR_SFTRSTF)) {
            // Direct hard reset case
            return;
        }
        // Soft reset; boot loader may have been active with BOOT pin pulled high.
        FALLTHROUGH;

    case RESET_BOOTLOADER_POST:
        // Boot loader activity magically prevents SysTick from interrupting.
        // Issue a soft reset to prevent the condition.
        forcedSystemResetWithoutDisablingCaches(); // observed that disabling dcache after cold boot with BOOT pin high causes segfault.
    }

    void (*SysMemBootJump)(void);
    __SYSCFG_CLK_ENABLE();

    systemJumpToBootloader();

#define SYSTEM_BOOTLOADER_VEC 0x1fff0000

    uint32_t p =  (*((uint32_t *)SYSTEM_BOOTLOADER_VEC));
    __set_MSP(p); //Set the main stack pointer to its defualt values
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(SYSTEM_BOOTLOADER_VEC + 4))); // Point the PC to the System Memory reset vector (+4)
    SysMemBootJump();
    while (1);
}

// Nucleo-G474RE board seems to come with software BOOT0 enabled.
// Call this function once from init() to honor PB8-BOOT0 pin status for boot loader invocation.
void systemBOOT0PinBootLoaderEnable(void)
{
    FLASH_OBProgramInitTypeDef OBInit;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    HAL_FLASH_OB_Unlock();

    HAL_FLASHEx_OBGetConfig(&OBInit);

    if ((OBInit.USERConfig & (OB_BOOT0_FROM_PIN|OB_BOOT1_SYSTEM)) != (OB_BOOT0_FROM_PIN|OB_BOOT1_SYSTEM)) {
        OBInit.OptionType = OPTIONBYTE_USER;
        OBInit.USERType = OB_USER_nSWBOOT0|OB_USER_nBOOT1;
        OBInit.USERConfig = OB_BOOT0_FROM_PIN|OB_BOOT1_SYSTEM;
        HAL_FLASHEx_OBProgram(&OBInit);

        HAL_FLASH_OB_Launch();
    }

    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}
