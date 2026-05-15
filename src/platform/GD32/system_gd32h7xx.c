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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/persistent.h"

void sys_clock_config(void);

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

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;

void checkForBootLoaderRequest(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);

    // rcu_periph_clock_enable(RCU_SYSCFG);
    // syscfg_bootmode_config(SYSCFG_BOOTMODE_BOOTLOADER);

    extern isrVector_t system_isr_vector_table_base;

    SCB->VTOR = (uint32_t)&system_isr_vector_table_base;
    __DSB();
    __DSB();

    __set_MSP(system_isr_vector_table_base.stackEnd);
    system_isr_vector_table_base.resetHandler();
    while (1);
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    /* enable AHB1 peripherals clock */
    rcu_periph_clock_enable(RCU_BKPSRAM);
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMA1);
    rcu_periph_clock_enable(RCU_DMAMUX);

    rcu_periph_clock_enable(RCU_SYSCFG);

    rcu_periph_clock_enable(RCU_OSPI0);
    rcu_periph_clock_enable(RCU_OSPI1);
    rcu_periph_clock_enable(RCU_EXMC);
}

void sys_clock_config(void)
{
    // The system clock has been configured in /startup/system_gd32f4xx.c
    // file. The HSE_VALUE can be 25MHz or 8MHz, and the default is 8MHz.
    // The HSE_VALUE can be set when you compile the firmware.
    // Here we just update the value SystemCoreClock.
    SystemCoreClockUpdate();
}

bool isMPUSoftReset(void)
{
    if (cachedResetFlags & RCU_RSTSCK_SWRSTF)
        return true;
    else
        return false;
}


#if defined(USE_FLASH_MEMORY_MAPPED)

/*
 * Memory mapped targets use a bootloader which enables memory mapped mode before running the firmware directly from external flash.
 * Code running from external flash, i.e. most of the firmware, must not disable peripherals or reconfigure pins used by the CPU to access the flash chip.
 *
 * If the config is also stored on the same flash chip that code is running from then VERY special care must be taken when detecting the flash chip
 * and when writing an updated config back to the flash.
 */

static bool memoryMappedModeEnabledOnBoot = false;

bool isMemoryMappedModeEnabledOnBoot(void)
{
    return memoryMappedModeEnabledOnBoot;
}

void memoryMappedModeInit(void)
{
    // Smaller MCU packages have ONE OSPI interface which supports memory mapped mode.
    memoryMappedModeEnabledOnBoot = READ_BIT(OSPI_CTL(OSPI0), OSPI_CTL_FMOD) == OSPI_CTL_FMOD;
}
#else
bool isMemoryMappedModeEnabledOnBoot(void)
{
    return false;
}
#endif

void systemInit(void)
{
    persistentObjectInit();

    checkForBootLoaderRequest();

    sys_clock_config();

    // Configure NVIC preempt/priority groups
    nvic_priority_group_set(NVIC_PRIORITY_GROUPING);

    // cache RCU RSTSCK register value to use it in isMPUSoftReset() and others
    cachedResetFlags = RCU_RSTSCK;

// #ifdef VECT_TAB_SRAM
//     extern uint8_t isr_vector_table_base;
//     nvic_vector_table_set((uint32_t)&isr_vector_table_base, 0x0);
// #endif

    rcu_periph_clock_disable(RCU_USBHS0);

    rcu_all_reset_flag_clear();

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}

extern void _init(void) {;}
