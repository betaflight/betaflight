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
    
    rcu_periph_clock_enable(RCU_SYSCFG);
    syscfg_bootmode_config(SYSCFG_BOOTMODE_BOOTLOADER);

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

    /* enable APB1 peripherals clock */
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_periph_clock_enable(RCU_TIMER4);
    rcu_periph_clock_enable(RCU_TIMER5);
    rcu_periph_clock_enable(RCU_TIMER6);
    rcu_periph_clock_enable(RCU_TIMER11);
    rcu_periph_clock_enable(RCU_TIMER12);
    rcu_periph_clock_enable(RCU_TIMER13);
    rcu_periph_clock_enable(RCU_WWDGT);
    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_SPI2);
    rcu_periph_clock_enable(RCU_USART1);
    rcu_periph_clock_enable(RCU_USART2);
    rcu_periph_clock_enable(RCU_UART3);
    rcu_periph_clock_enable(RCU_UART4);
    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_I2C1);
    rcu_periph_clock_enable(RCU_I2C2);
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_CAN1);
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_DAC);

    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER7);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_USART5);
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_ADC2);
    rcu_periph_clock_enable(RCU_SDIO);
    rcu_periph_clock_enable(RCU_SPI0);
    rcu_periph_clock_enable(RCU_SYSCFG);
    rcu_periph_clock_enable(RCU_TIMER8);
    rcu_periph_clock_enable(RCU_TIMER9);
    rcu_periph_clock_enable(RCU_TIMER10);

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
    if (cachedRccCsrValue & RCU_RSTSCK_SWRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
    persistentObjectInit();

    checkForBootLoaderRequest();

    sys_clock_config();

    // Configure NVIC preempt/priority groups
    nvic_priority_group_set(NVIC_PRIORITY_GROUPING);

    // cache RCU RSTSCK register value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCU_RSTSCK;

    // Although VTOR is already loaded with a possible vector table in RAM,
    // removing the call to NVIC_SetVectorTable causes USB not to become active,

#ifdef VECT_TAB_SRAM
    extern uint8_t isr_vector_table_base;
    nvic_vector_table_set((uint32_t)&isr_vector_table_base, 0x0);
#endif

    rcu_periph_clock_disable(RCU_USBFS);

    rcu_all_reset_flag_clear();

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}
