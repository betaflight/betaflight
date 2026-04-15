/**
  ******************************************************************************
  * @file    system_stm32c5xx.c
  * @brief   CMSIS Cortex-M33 Device Peripheral Access Layer System Source File
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32c5xxxx.c" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *   After each device reset the HSIDIV (48 MHz) is used as system clock source.
  *   Then SystemInit() function is called, in "startup_stm32c5xxxx.c" file, to
  *   configure the system clock before to branch to main program.
  *
  *   This file configures the system clock as follows:
  *=============================================================================
  *-----------------------------------------------------------------------------
  *        System Clock source                     | HSI
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                              | 48000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                                | 48000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                           | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB3 Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        PSI                                     | Disabled
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB,                  | Disabled
  *        SDIO and RNG clock                      |
  *-----------------------------------------------------------------------------
  *=============================================================================
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup STM32C5xx_system
  * @{
  */

/** @addtogroup STM32C5xx_System_Private_Includes
  * @{
  */

#include "stm32c5xx.h"
#include <math.h>

/**
  * @}
  */

/** @addtogroup STM32C5xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32C5xx_System_Private_Defines
  * @{
  */
#define SYSTEM_CLOCK  48000000U /*!< Reset system clock in Hz */

/**
  * @}
  */

/** @addtogroup STM32C5xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32C5xx_System_Private_Variables
  * @{
  */
extern const VECTOR_TABLE_Type __VECTOR_TABLE[];

/* The SystemCoreClock variable is updated in three ways:
    1) by calling CMSIS function SystemCoreClockUpdate()
    2) by initializing the HAL module through HAL_Init() function
    3) by calling a hal_rcc function to configure the system clock:
        - HAL_RCC_ResetSystemClock()
        - HAL_RCC_SetSysClkSource()
        - HAL_RCC_SetHCLKPrescaler()
        - HAL_RCC_SetBusClockConfig()
        - HAL_RCC_GetHCLKFreq()
       Note: If you use one of this function to configure the system clock; then there is no need to call
             the 2 first functions listed above, since SystemCoreClock variable is updated automatically.
 */
uint32_t SystemCoreClock = SYSTEM_CLOCK;

const uint8_t  AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t  APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

/**
  * @}
  */

/** @addtogroup STM32C5xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32C5xx_System_Private_Functions
  * @{
  */

/**
  * @brief  Setup the microcontroller system.
  * @param  None
  * @retval None
  */

void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB_EnableFPU();
#endif /* (__FPU_PRESENT == 1) && (__FPU_USED == 1) */

  /* Configure the Vector Table location -------------------------------------*/
  SCB_SetVTOR((uint32_t)(&__VECTOR_TABLE[0]));
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  * @param  None
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is PSI, SystemCoreClock will contain the PSI_VALUE(*)
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(**)
  *
  *           - If SYSCLK source is HSI_DIV_X, SystemCoreClock will contain the HSI_VALUE/X(**)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(***)
  *
  *         (*) PSI_VALUE is one of the value [100, 144, 160] MHz. The value of PSI can change
  *             slightly depending on the clock source reference of the PSI.
  *
  *         (**) HSI_VALUE is a constant defined in stm32c5xxxx.h file (default value
  *              144 MHz) but the real value can vary depending on the variations
  *              in voltage and temperature.
  *
  *         (***) HSE_VALUE is a constant defined in stm32_external_env.h file (default value
  *               24 MHz), user has to ensure that HSE_VALUE is same as the real
  *               frequency of the crystal used. Otherwise, this function can
  *               have wrong result.
  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch ((RCC->CFGR1 & RCC_CFGR1_SWS) >> RCC_CFGR1_SWS_Pos)
  {
#if defined(RCC_CR1_HSIDIV4ON)
    case 0x00UL:  /* HSIDIV4 used as system clock source */
      SystemCoreClock = (uint32_t)(HSI_VALUE / 4U);
      break;
#else
    case 0x00UL:  /* HSIDIV3 used as system clock source */
      SystemCoreClock = (uint32_t)(HSI_VALUE / 3U);
      break;
#endif /* RCC_CR1_HSIDIV4ON */

    case 0x01UL:  /* HSIS used as system clock source */
      SystemCoreClock = (uint32_t) HSI_VALUE;
      break;

    case 0x02UL:  /* HSE used as system clock source */
#if defined(HSE_VALUE)
      SystemCoreClock = HSE_VALUE;
#else
      while (1); /* Block; user has to define the real HSE value */
#endif /* HSE_VALUE */
      break;

    case 0x03UL:  /* PSI used as system clock source */
    {
      uint32_t psifreq;
      psifreq = ((RCC->CR2 & RCC_CR2_PSIFREQ) >> RCC_CR2_PSIFREQ_Pos);

      switch (psifreq)
      {
#if defined(RCC_CR1_HSIDIV4ON)
        case 0x00UL:
          SystemCoreClock = (uint32_t)200000000U; /* 200 MHz */
          break;

        case 0x01UL:
          SystemCoreClock = (uint32_t)144000000U; /* 144 MHz */
          break;

        case 0x02UL:
          SystemCoreClock = (uint32_t)160000000U; /* 160 MHz */
          break;

        case 0x03UL:
          SystemCoreClock = (uint32_t)192000000U; /* 192 MHz */
          break;

        default:
          SystemCoreClock = (uint32_t)200000000U; /* 200 MHz */
          break;
#else
        case 0x00UL:
          SystemCoreClock = (uint32_t)100000000U; /* 100 MHz */
          break;

        case 0x01UL:
          SystemCoreClock = (uint32_t)144000000U; /* 144 MHz */
          break;

        case 0x02UL:
        case 0x03UL:
          SystemCoreClock = (uint32_t)160000000U; /* 160 MHz */
          break;

        default:
          SystemCoreClock = (uint32_t)100000000U; /* 100 MHz */
          break;
#endif /* RCC_CR1_HSIDIV4ON */
      }
    }
    break;

    default:
#if defined(RCC_CR1_HSIDIV4ON)
      SystemCoreClock = (uint32_t)(HSI_VALUE / 4U);
#else
      SystemCoreClock = (uint32_t)(HSI_VALUE / 3U);
#endif /* RCC_CR1_HSIDIV4ON */
      break;
  }
  /* Compute HCLK clock frequency --------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR2 & RCC_CFGR2_HPRE) >> RCC_CFGR2_HPRE_Pos)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
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
