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
#include "drivers/system.h"


#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
void SystemClock_Config(void);

void systemReset(void)
{
    if (mpuResetFn) {
        mpuResetFn();
    }

    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(void)
{
    if (mpuResetFn) {
        mpuResetFn();
    }

    (*(__IO uint32_t *) (BKPSRAM_BASE + 4)) = 0xDEADBEEF;   // flag that will be readable after reboot

    __disable_irq();
    NVIC_SystemReset();
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{

    // AHB1
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    __HAL_RCC_DTCMRAMEN_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
#ifndef STM32F722xx
    __HAL_RCC_DMA2D_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();
#endif

    //APB1
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_LPTIM1_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_DAC_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
    __HAL_RCC_UART8_CLK_ENABLE();
#ifndef STM32F722xx
    __HAL_RCC_I2C4_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();
    __HAL_RCC_CEC_CLK_ENABLE();
#endif

    //APB2
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_TIM11_CLK_ENABLE();
    __HAL_RCC_SPI5_CLK_ENABLE();
    __HAL_RCC_SAI1_CLK_ENABLE();
    __HAL_RCC_SAI2_CLK_ENABLE();
#ifndef STM32F722xx
    __HAL_RCC_SPI6_CLK_ENABLE();
#endif
//
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_StructInit(&GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // default is un-pulled input
//
//    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
//    GPIO_InitStructure.GPIO_Pin &= ~(GPIO_Pin_11 | GPIO_Pin_12); // leave USB D+/D- alone
//
//    GPIO_InitStructure.GPIO_Pin &= ~(GPIO_Pin_13 | GPIO_Pin_14); // leave JTAG pins alone
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//    GPIO_Init(GPIOD, &GPIO_InitStructure);
//    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

bool isMPUSoftReset(void)
{
    if (RCC->CSR & RCC_CSR_SFTRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
    checkForBootLoaderRequest();

    //SystemClock_Config();

    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->CSR value to use it in isMPUSoftreset() and others
    cachedRccCsrValue = RCC->CSR;

    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    //extern void *isr_vector_table_base;
    //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
    //__HAL_RCC_USB_OTG_FS_CLK_DISABLE;

    //RCC_ClearFlag();

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    //SysTick_Config(SystemCoreClock / 1000);
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

void(*bootJump)(void);
void checkForBootLoaderRequest(void)
{
    uint32_t bt;
    __PWR_CLK_ENABLE();
    __BKPSRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    bt = (*(__IO uint32_t *) (BKPSRAM_BASE + 4)) ;
    if ( bt == 0xDEADBEEF ) {
        (*(__IO uint32_t *) (BKPSRAM_BASE + 4)) =  0xCAFEFEED; // Reset our trigger

        void (*SysMemBootJump)(void);
        __SYSCFG_CLK_ENABLE();
        SYSCFG->MEMRMP |= SYSCFG_MEM_BOOT_ADD0 ;
        uint32_t p =  (*((uint32_t *) 0x1ff00000));
        __set_MSP(p); //Set the main stack pointer to its defualt values
        SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1ff00004)); // Point the PC to the System Memory reset vector (+4)
        SysMemBootJump();
        while (1);
    }
}
