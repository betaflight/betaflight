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


#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
void SetSysClock(void);

void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(void)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST);

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

    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST) {
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

    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_GPIOA |
        RCC_AHB1Periph_GPIOB |
        RCC_AHB1Periph_GPIOC |
        RCC_AHB1Periph_GPIOD |
        RCC_AHB1Periph_GPIOE |
#ifdef STM32F40_41xxx
        RCC_AHB1Periph_GPIOF |
        RCC_AHB1Periph_GPIOG |
        RCC_AHB1Periph_GPIOH |
        RCC_AHB1Periph_GPIOI |
#endif
        RCC_AHB1Periph_CRC |
        RCC_AHB1Periph_FLITF |
        RCC_AHB1Periph_SRAM1 |
        RCC_AHB1Periph_SRAM2 |
        RCC_AHB1Periph_BKPSRAM |
        RCC_AHB1Periph_DMA1 |
        RCC_AHB1Periph_DMA2 |
        0, ENABLE
    );

    RCC_AHB2PeriphClockCmd(0, ENABLE);
#ifdef STM32F40_41xxx
    RCC_AHB3PeriphClockCmd(0, ENABLE);
#endif
    RCC_APB1PeriphClockCmd(
        RCC_APB1Periph_TIM2 |
        RCC_APB1Periph_TIM3 |
        RCC_APB1Periph_TIM4 |
        RCC_APB1Periph_TIM5 |
        RCC_APB1Periph_TIM6 |
        RCC_APB1Periph_TIM7 |
        RCC_APB1Periph_TIM12 |
        RCC_APB1Periph_TIM13 |
        RCC_APB1Periph_TIM14 |
        RCC_APB1Periph_WWDG |
        RCC_APB1Periph_SPI2 |
        RCC_APB1Periph_SPI3 |
        RCC_APB1Periph_USART2 |
        RCC_APB1Periph_USART3 |
        RCC_APB1Periph_UART4 |
        RCC_APB1Periph_UART5 |
        RCC_APB1Periph_I2C1 |
        RCC_APB1Periph_I2C2 |
        RCC_APB1Periph_I2C3 |
        RCC_APB1Periph_CAN1 |
        RCC_APB1Periph_CAN2 |
        RCC_APB1Periph_PWR |
        RCC_APB1Periph_DAC |
        0, ENABLE);

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_TIM1 |
        RCC_APB2Periph_TIM8 |
        RCC_APB2Periph_USART1 |
        RCC_APB2Periph_USART6 |
        RCC_APB2Periph_ADC |
        RCC_APB2Periph_ADC1 |
        RCC_APB2Periph_ADC2 |
        RCC_APB2Periph_ADC3 |
        RCC_APB2Periph_SDIO |
        RCC_APB2Periph_SPI1 |
        RCC_APB2Periph_SYSCFG |
        RCC_APB2Periph_TIM9 |
        RCC_APB2Periph_TIM10 |
        RCC_APB2Periph_TIM11 |
        0, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // default is un-pulled input

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Pin &= ~(GPIO_Pin_11 | GPIO_Pin_12); // leave USB D+/D- alone

    GPIO_InitStructure.GPIO_Pin &= ~(GPIO_Pin_13 | GPIO_Pin_14); // leave JTAG pins alone
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);

#ifdef STM32F40_41xxx
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_Init(GPIOH, &GPIO_InitStructure);
    GPIO_Init(GPIOI, &GPIO_InitStructure);
#endif

}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_CSR_SFTRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
    SetSysClock();

    // Configure NVIC preempt/priority groups
    NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);

    // cache RCC->CSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->CSR;

    // Although VTOR is already loaded with a possible vector table in RAM,
    // removing the call to NVIC_SetVectorTable causes USB not to become active,

    extern uint8_t isr_vector_table_base;
    NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);

    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, DISABLE);

    RCC_ClearFlag();

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}
