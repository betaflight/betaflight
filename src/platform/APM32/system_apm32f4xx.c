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

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/persistent.h"

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

// Used in the startup files for F4
static void checkForBootLoaderRequest(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);

    extern isrVector_t system_isr_vector_table_base;

    __set_MSP(system_isr_vector_table_base.stackEnd);
    system_isr_vector_table_base.resetHandler();
    while (1);
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    __DAL_RCM_BKPSRAM_CLK_ENABLE();
    __DAL_RCM_DMA1_CLK_ENABLE();
    __DAL_RCM_DMA2_CLK_ENABLE();

    __DAL_RCM_TMR2_CLK_ENABLE();
    __DAL_RCM_TMR3_CLK_ENABLE();
    __DAL_RCM_TMR4_CLK_ENABLE();
    __DAL_RCM_TMR5_CLK_ENABLE();
    __DAL_RCM_TMR6_CLK_ENABLE();
    __DAL_RCM_TMR7_CLK_ENABLE();
    __DAL_RCM_TMR12_CLK_ENABLE();
    __DAL_RCM_TMR13_CLK_ENABLE();
    __DAL_RCM_TMR14_CLK_ENABLE();
    __DAL_RCM_WWDT_CLK_ENABLE();
    __DAL_RCM_SPI2_CLK_ENABLE();
    __DAL_RCM_SPI3_CLK_ENABLE();
    __DAL_RCM_USART2_CLK_ENABLE();
    __DAL_RCM_USART3_CLK_ENABLE();
    __DAL_RCM_UART4_CLK_ENABLE();
    __DAL_RCM_UART5_CLK_ENABLE();
    __DAL_RCM_I2C1_CLK_ENABLE();
    __DAL_RCM_I2C2_CLK_ENABLE();
    __DAL_RCM_I2C3_CLK_ENABLE();
    __DAL_RCM_CAN1_CLK_ENABLE();
    __DAL_RCM_CAN2_CLK_ENABLE();
    __DAL_RCM_PMU_CLK_ENABLE();
    __DAL_RCM_DAC_CLK_ENABLE();

    __DAL_RCM_TMR1_CLK_ENABLE();
    __DAL_RCM_TMR8_CLK_ENABLE();
    __DAL_RCM_USART1_CLK_ENABLE();
    __DAL_RCM_USART6_CLK_ENABLE();
    __DAL_RCM_ADC1_CLK_ENABLE();
    __DAL_RCM_ADC2_CLK_ENABLE();
    __DAL_RCM_ADC3_CLK_ENABLE();
    __DAL_RCM_SDIO_CLK_ENABLE();
    __DAL_RCM_SPI1_CLK_ENABLE();
    __DAL_RCM_SYSCFG_CLK_ENABLE();
    __DAL_RCM_TMR9_CLK_ENABLE();
    __DAL_RCM_TMR10_CLK_ENABLE();
    __DAL_RCM_TMR11_CLK_ENABLE();
}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCM_CSTS_SWRSTFLG)
        return true;
    else
        return false;
}

void systemInit(void)
{
    checkForBootLoaderRequest();

#ifdef USE_DAL_DRIVER
    DAL_Init();
#endif

    /* Configure the System clock source, PLL1 and HCLK */
    DAL_SysClkConfig();

    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();

    // Configure NVIC preempt/priority groups
    DAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCM->CSTS value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = READ_REG(RCM->CSTS);

    // Although VTOR is already loaded with a possible vector table in RAM,
    // removing the call to NVIC_SetVectorTable causes USB not to become active,

    extern uint8_t isr_vector_table_base;
    SCB->VTOR = (uint32_t)&isr_vector_table_base | (0x0 & (uint32_t)0x1FFFFF80);

    // Disable USB OTG FS clock
    __DAL_RCM_USB_OTG_FS_CLK_DISABLE();

    // Clear reset flags
    SET_BIT(RCM->CSTS, RCM_CSTS_RSTFLGCLR);

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    //--Todo: Set systick here ? // SysTick is updated whenever DAL_RCM_ClockConfig is called.
}
