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
#include "drivers/persistent.h"
#include "drivers/system.h"


void SystemClock_Config(void);


void enablePeripherialClocks(void)
{
    __HAL_RCC_MDMA_CLK_ENABLE();
    __HAL_RCC_QSPI_CLK_ENABLE();

    // AHB1
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();
    // USB clock will be enabled by vcpXXX/usbd_conf.c
    // Note that enabling both ULPI and non-ULPI does not work.

    // AHB2
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();

    // AHB4
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();
    __HAL_RCC_BDMA_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();

    // APB3

    // APB1
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
    __HAL_RCC_DAC12_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
    __HAL_RCC_UART8_CLK_ENABLE();
    __HAL_RCC_CRS_CLK_ENABLE();

    // APB2
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_TIM15_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();
    __HAL_RCC_SPI5_CLK_ENABLE();

    // APB4
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_I2C4_CLK_ENABLE();
    __HAL_RCC_LPTIM2_CLK_ENABLE();
    __HAL_RCC_LPTIM3_CLK_ENABLE();
    __HAL_RCC_LPTIM4_CLK_ENABLE();
    __HAL_RCC_LPTIM5_CLK_ENABLE();
    __HAL_RCC_COMP12_CLK_ENABLE();
    __HAL_RCC_VREF_CLK_ENABLE();
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    // GPIO initialization, copied from drivers/system_stm32f7xx.c
    // ... It was commented out.
    // Where does F7 initializes the GPIO pins? It doesn't do it at all???

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

void configureMasterClockOutputs(void)
{
    // Initialize pins for MCO1 and MCO2 for clock testing/verification

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_RSR_SFTRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
#ifdef USE_ITCM_RAM
    //  Mark ITCM-RAM as read-only
    HAL_MPU_Disable();

    // "For Cortex®-M7, TCMs memories always behave as Non-cacheable, Non-shared normal memories, irrespectiveof the memory type attributes defined in the MPU for a memory region containing addresses held in the TCM"
    // See AN4838

    MPU_Region_InitTypeDef MPU_InitStruct;
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x00000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO_URO;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif


    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->RSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->RSR;

    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    //extern void *isr_vector_table_base;
    //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
    //__HAL_RCC_USB_OTG_FS_CLK_DISABLE;

    //RCC_ClearFlag();

    enablePeripherialClocks();

    enableGPIOPowerUsageAndNoiseReductions();

#ifdef USE_MCO_OUTPUTS
    configureMasterClockOutputs();
#endif

    // Init cycle counter
    cycleCounterInit();

    // SysTick is updated whenever HAL_RCC_ClockConfig is called.
}

void systemReset(void)
{
#if 0
#ifdef USE_GYRO
    if (mpuResetFn) {
        mpuResetFn();
    }
#endif
#endif

    SCB_DisableDCache();
    SCB_DisableICache();

    __disable_irq();
    NVIC_SystemReset();
}

void forcedSystemResetWithoutDisablingCaches(void)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FORCED);

    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(void)
{
#if 0
#ifdef USE_GYRO
    if (mpuResetFn) {
        mpuResetFn();
    }
#endif
#endif
#ifdef USE_EXST
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FLASH_BOOTLOADER_REQUEST);
#else
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST);
#endif
    __disable_irq();
    NVIC_SystemReset();
}

static uint32_t bootloaderRequest;

bool systemIsFlashBootloaderRequested(void)
{
    return (bootloaderRequest == RESET_FLASH_BOOTLOADER_REQUEST);
}

void systemCheckResetReason(void)
{
    bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
    case RESET_BOOTLOADER_REQUEST:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        break;

    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        return;

    case RESET_NONE:
        if (!(RCC->RSR & RCC_RSR_SFTRSTF)) {
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

    uint32_t p =  (*((uint32_t *) 0x1ff09800));
    __set_MSP(p); //Set the main stack pointer to its defualt values
    SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1ff09804)); // Point the PC to the System Memory reset vector (+4)
    SysMemBootJump();
    while (1);
}
