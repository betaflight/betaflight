/**
 *
 * @file        system_apm32f4xx.c
 *
 * @brief       CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
 *
 * @version     V1.0.0
 *
 * @date        2023-07-31
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup apm32f4xx_system
  * @{
  */  
  
/** @addtogroup APM32F4xx_System_Private_Includes
  * @{
  */

#include "apm32f4xx.h"

/* Value of the external oscillator in Hz */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)8000000U)
#endif /* HSE_VALUE */

/* Value of the internal oscillator in Hz */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U)
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_Defines
  * @{
  */
/* Uncomment the following line if you need to relocate your vector table in internal SRAM */
/* #define VECT_TAB_SRAM */

/* Vector table base offset field. This value must be a multiple of 0x200 */
#define VECT_TAB_OFFSET  0x00

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_Variables
  * @{
  */
uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup APM32F4xx_System_Private_Functions
  * @{
  */

/**
 * @brief     Setup the microcontroller system
 *
 * @param     None
 *
 * @retval    None
 */
void SystemInit(void)
{
    uint8_t i;

    /* Disable global interrupt */
    __disable_irq();

    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;

    for (i = 0U; i < 8U; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }

    /* FPU settings */
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
      SCB->CPACR |= ((3UL << 10U * 2U)|(3UL << 11U * 2U));  /* set CP10 and CP11 Full Access */
#endif
    /* Reset the RCM clock configuration to the default reset state */
    /* Set HSIEN bit */
    RCM->CTRL |= (uint32_t)0x00000001;

    /* Reset CFG register */
    RCM->CFG = 0x00000000;

    /* Reset HSEEN, CSSEN and PLL1EN bits */
    RCM->CTRL &= (uint32_t)0xFEF6FFFF;

    /* Reset PLL1CFG register */
    RCM->PLL1CFG = 0x24003010;

    /* Reset HSEBCFG bit */
    RCM->CTRL &= (uint32_t)0xFFFBFFFF;

    /* Disable all interrupts */
    RCM->INT = 0x00000000;

    /* Configure the Vector Table location add offset address */
#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

    /* Enable global interrupt */
    __enable_irq();
}

/**
   * @brief Update SystemCoreClock variable according to clock register values
 *          The SystemCoreClock variable contains the core clock (HCLK)
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
    uint32_t sysClock = 0, pllvco = 0, pllc, pllClock, pllb;
    
    /* Get SYSCLK source */
    sysClock = RCM->CFG & RCM_CFG_SCLKSWSTS;

    switch (sysClock)
    {
        case 0x00:  /* HSI used as system clock source */
            SystemCoreClock = HSI_VALUE;
            break;

        case 0x04:  /* HSE used as system clock source */
            SystemCoreClock = HSE_VALUE;
            break;

        case 0x08:  /* PLL used as system clock source */
            pllClock = (RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS) >> 22;
            pllb = RCM->PLL1CFG & RCM_PLL1CFG_PLLB;
            
            if (pllClock != 0)
            {
                /* HSE used as PLL clock source */
                pllvco = (HSE_VALUE / pllb) * ((RCM->PLL1CFG & RCM_PLL1CFG_PLL1A) >> 6);
            }
            else
            {
                /* HSI used as PLL clock source */
                pllvco = (HSI_VALUE / pllb) * ((RCM->PLL1CFG & RCM_PLL1CFG_PLL1A) >> 6);
            }

            pllc = (((RCM->PLL1CFG & RCM_PLL1CFG_PLL1C) >> 16) + 1 ) * 2;
            SystemCoreClock = pllvco / pllc;
            break;

        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    /* Compute HCLK frequency --------------------------------------------------*/
    /* Get HCLK prescaler */
    sysClock = AHBPrescTable[((RCM->CFG & RCM_CFG_AHBPSC) >> 4)];
    /* HCLK frequency */
    SystemCoreClock >>= sysClock;
}

/**
  * @}
  */

/**
  * @}
  */
  
/**
  * @}
  */    
