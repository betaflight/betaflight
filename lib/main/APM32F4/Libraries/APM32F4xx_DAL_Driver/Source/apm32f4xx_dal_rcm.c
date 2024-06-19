/**
  *
  * @file    apm32f4xx_dal_rcm.c
  * @brief   RCM DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Reset and Clock Control (RCM) peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *
  @verbatim
  ==============================================================================
                      ##### RCM specific features #####
  ==============================================================================
    [..]
      After reset the device is running from Internal High Speed oscillator
      (HSI 16MHz) with Flash 0 wait state, Flash prefetch buffer, D-Cache
      and I-Cache are disabled, and all peripherals are off except internal
      SRAM, Flash and JTAG.
      (+) There is no prescaler on High speed (AHB) and Low speed (APB) busses;
          all peripherals mapped on these busses are running at HSI speed.
      (+) The clock for all peripherals is switched off, except the SRAM and FLASH.
      (+) All GPIOs are in input floating state, except the JTAG pins which
          are assigned to be used for debug purpose.

    [..]
      Once the device started from reset, the user application has to:
      (+) Configure the clock source to be used to drive the System clock
          (if the application needs higher frequency/performance)
      (+) Configure the System clock frequency and Flash settings
      (+) Configure the AHB and APB busses prescalers
      (+) Enable the clock for the peripheral(s) to be used
      (+) Configure the clock source(s) for peripherals which clocks are not
          derived from the System clock (I2S, RTC, ADC, USB OTG FS/SDIO/RNG)

                      ##### RCM Limitations #####
  ==============================================================================
    [..]
      A delay between an RCM peripheral clock enable and the effective peripheral
      enabling should be taken into account in order to manage the peripheral read/write
      from/to registers.
      (+) This delay depends on the peripheral mapping.
      (+) If peripheral is mapped on AHB: the delay is 2 AHB clock cycle
          after the clock enable bit is set on the hardware register
      (+) If peripheral is mapped on APB: the delay is 2 APB clock cycle
          after the clock enable bit is set on the hardware register

    [..]
      Implemented Workaround:
      (+) For AHB & APB peripherals, a dummy read to the peripheral register has been
          inserted in each __DAL_RCM_PPP_CLK_ENABLE() macro.

  @endverbatim
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup RCM RCM
  * @brief RCM DAL module driver
  * @{
  */

#ifdef DAL_RCM_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup RCM_Private_Constants
  * @{
  */

/* Private macro -------------------------------------------------------------*/
#define __MCO1_CLK_ENABLE()   __DAL_RCM_GPIOA_CLK_ENABLE()
#define MCO1_GPIO_PORT        GPIOA
#define MCO1_PIN              GPIO_PIN_8

#define __MCO2_CLK_ENABLE()   __DAL_RCM_GPIOC_CLK_ENABLE()
#define MCO2_GPIO_PORT         GPIOC
#define MCO2_PIN               GPIO_PIN_9
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup RCM_Private_Variables RCM Private Variables
  * @{
  */
/**
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup RCM_Exported_Functions RCM Exported Functions
  *  @{
  */

/** @defgroup RCM_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
           ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
      This section provides functions allowing to configure the internal/external oscillators
      (HSE, HSI, LSE, LSI, PLL, CSS and MCO) and the System busses clocks (SYSCLK, AHB, APB1
       and APB2).

    [..] Internal/external clock and PLL configuration
         (#) HSI (high-speed internal), 16 MHz factory-trimmed RC used directly or through
             the PLL as System clock source.

         (#) LSI (low-speed internal), 32 KHz low consumption RC used as IWDT and/or RTC
             clock source.

         (#) HSE (high-speed external), 4 to 26 MHz crystal oscillator used directly or
             through the PLL as System clock source. Can be used also as RTC clock source.

         (#) LSE (low-speed external), 32 KHz oscillator used as RTC clock source.

         (#) PLL (clocked by HSI or HSE), featuring two different output clocks:
           (++) The first output is used to generate the high speed system clock (up to 168 MHz)
           (++) The second output is used to generate the clock for the USB OTG FS (48 MHz),
                the random analog generator (<=48 MHz) and the SDIO (<= 48 MHz).

         (#) CSS (Clock security system), once enable using the macro __DAL_RCM_CSS_ENABLE()
             and if a HSE clock failure occurs(HSE used directly or through PLL as System
             clock source), the System clocks automatically switched to HSI and an interrupt
             is generated if enabled. The interrupt is linked to the Cortex-M4 NMI
             (Non-Maskable Interrupt) exception vector.

         (#) MCO1 (microcontroller clock output), used to output HSI, LSE, HSE or PLL
             clock (through a configurable prescaler) on PA8 pin.

         (#) MCO2 (microcontroller clock output), used to output HSE, PLL, SYSCLK or PLLI2S
             clock (through a configurable prescaler) on PC9 pin.

    [..] System, AHB and APB busses clocks configuration
         (#) Several clock sources can be used to drive the System clock (SYSCLK): HSI,
             HSE and PLL.
             The AHB clock (HCLK) is derived from System clock through configurable
             prescaler and used to clock the CPU, memory and peripherals mapped
             on AHB bus (DMA, GPIO...). APB1 (PCLK1) and APB2 (PCLK2) clocks are derived
             from AHB clock through configurable prescalers and used to clock
             the peripherals mapped on these busses. You can use
             "DAL_RCM_GetSysClockFreq()" function to retrieve the frequencies of these clocks.

         (#) For the APM32F405xx/07xx and APM32F417xx devices, the maximum
             frequency of the SYSCLK and HCLK is 168 MHz, PCLK2 84 MHz and PCLK1 42 MHz.
             Depending on the device voltage range, the maximum frequency should
             be adapted accordingly (refer to the product datasheets for more details).

         (#) For the APM32F41xxx, the maximum frequency of the SYSCLK and HCLK is 100 MHz,
             PCLK2 100 MHz and PCLK1 50 MHz.
             Depending on the device voltage range, the maximum frequency should
             be adapted accordingly (refer to the product datasheets for more details).

@endverbatim
  * @{
  */

/**
  * @brief  Resets the RCM clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE and PLL OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 and MCO2 OFF
  *            - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks
  *            - LSI, LSE and RTC clocks
  * @retval DAL status
  */
__weak DAL_StatusTypeDef DAL_RCM_DeInit(void)
{
  return DAL_OK;
}

/**
  * @brief  Initializes the RCM Oscillators according to the specified parameters in the
  *         RCM_OscInitTypeDef.
  * @param  RCM_OscInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not
  *         supported by this API. User should request a transition to LSE Off
  *         first and then LSE On or LSE Bypass.
  * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
  *         supported by this API. User should request a transition to HSE Off
  *         first and then HSE On or HSE Bypass.
  * @retval DAL status
  */
__weak DAL_StatusTypeDef DAL_RCM_OscConfig(RCM_OscInitTypeDef  *RCM_OscInitStruct)
{
  uint32_t tickstart, pll_config;

  /* Check Null pointer */
  if(RCM_OscInitStruct == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_RCM_OSCILLATORTYPE(RCM_OscInitStruct->OscillatorType));
  /*------------------------------- HSE Configuration ------------------------*/
  if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_HSE) == RCM_OSCILLATORTYPE_HSE)
  {
    /* Check the parameters */
    ASSERT_PARAM(IS_RCM_HSE(RCM_OscInitStruct->HSEState));
    /* When the HSE is used as system clock or clock source for PLL in these cases HSE will not disabled */
    if((__DAL_RCM_GET_SYSCLK_SOURCE() == RCM_CFG_SCLKSWSTS_HSE) ||\
      ((__DAL_RCM_GET_SYSCLK_SOURCE() == RCM_CFG_SCLKSWSTS_PLL) && ((RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS) == RCM_PLL1CFG_PLL1CLKS_HSE)))
    {
      if((__DAL_RCM_GET_FLAG(RCM_FLAG_HSERDY) != RESET) && (RCM_OscInitStruct->HSEState == RCM_HSE_OFF))
      {
        return DAL_ERROR;
      }
    }
    else
    {
      /* Set the new HSE configuration ---------------------------------------*/
      __DAL_RCM_HSE_CONFIG(RCM_OscInitStruct->HSEState);

      /* Check the HSE State */
      if((RCM_OscInitStruct->HSEState) != RCM_HSE_OFF)
      {
        /* Get Start Tick */
        tickstart = DAL_GetTick();

        /* Wait till HSE is ready */
        while(__DAL_RCM_GET_FLAG(RCM_FLAG_HSERDY) == RESET)
        {
          if((DAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
          {
            return DAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Get Start Tick */
        tickstart = DAL_GetTick();

        /* Wait till HSE is bypassed or disabled */
        while(__DAL_RCM_GET_FLAG(RCM_FLAG_HSERDY) != RESET)
        {
          if((DAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
          {
            return DAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*----------------------------- HSI Configuration --------------------------*/
  if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_HSI) == RCM_OSCILLATORTYPE_HSI)
  {
    /* Check the parameters */
    ASSERT_PARAM(IS_RCM_HSI(RCM_OscInitStruct->HSIState));
    ASSERT_PARAM(IS_RCM_CALIBRATION_VALUE(RCM_OscInitStruct->HSICalibrationValue));

    /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
    if((__DAL_RCM_GET_SYSCLK_SOURCE() == RCM_CFG_SCLKSWSTS_HSI) ||\
      ((__DAL_RCM_GET_SYSCLK_SOURCE() == RCM_CFG_SCLKSWSTS_PLL) && ((RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS) == RCM_PLL1CFG_PLL1CLKS_HSI)))
    {
      /* When HSI is used as system clock it will not disabled */
      if((__DAL_RCM_GET_FLAG(RCM_FLAG_HSIRDY) != RESET) && (RCM_OscInitStruct->HSIState != RCM_HSI_ON))
      {
        return DAL_ERROR;
      }
      /* Otherwise, just the calibration is allowed */
      else
      {
        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
        __DAL_RCM_HSI_CALIBRATIONVALUE_ADJUST(RCM_OscInitStruct->HSICalibrationValue);
      }
    }
    else
    {
      /* Check the HSI State */
      if((RCM_OscInitStruct->HSIState)!= RCM_HSI_OFF)
      {
        /* Enable the Internal High Speed oscillator (HSI). */
        __DAL_RCM_HSI_ENABLE();

        /* Get Start Tick*/
        tickstart = DAL_GetTick();

        /* Wait till HSI is ready */
        while(__DAL_RCM_GET_FLAG(RCM_FLAG_HSIRDY) == RESET)
        {
          if((DAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
          {
            return DAL_TIMEOUT;
          }
        }

        /* Adjusts the Internal High Speed oscillator (HSI) calibration value. */
        __DAL_RCM_HSI_CALIBRATIONVALUE_ADJUST(RCM_OscInitStruct->HSICalibrationValue);
      }
      else
      {
        /* Disable the Internal High Speed oscillator (HSI). */
        __DAL_RCM_HSI_DISABLE();

        /* Get Start Tick*/
        tickstart = DAL_GetTick();

        /* Wait till HSI is ready */
        while(__DAL_RCM_GET_FLAG(RCM_FLAG_HSIRDY) != RESET)
        {
          if((DAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
          {
            return DAL_TIMEOUT;
          }
        }
      }
    }
  }
  /*------------------------------ LSI Configuration -------------------------*/
  if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_LSI) == RCM_OSCILLATORTYPE_LSI)
  {
    /* Check the parameters */
    ASSERT_PARAM(IS_RCM_LSI(RCM_OscInitStruct->LSIState));

    /* Check the LSI State */
    if((RCM_OscInitStruct->LSIState)!= RCM_LSI_OFF)
    {
      /* Enable the Internal Low Speed oscillator (LSI). */
      __DAL_RCM_LSI_ENABLE();

      /* Get Start Tick*/
      tickstart = DAL_GetTick();

      /* Wait till LSI is ready */
      while(__DAL_RCM_GET_FLAG(RCM_FLAG_LSIRDY) == RESET)
      {
        if((DAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
        {
          return DAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Disable the Internal Low Speed oscillator (LSI). */
      __DAL_RCM_LSI_DISABLE();

      /* Get Start Tick */
      tickstart = DAL_GetTick();

      /* Wait till LSI is ready */
      while(__DAL_RCM_GET_FLAG(RCM_FLAG_LSIRDY) != RESET)
      {
        if((DAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
        {
          return DAL_TIMEOUT;
        }
      }
    }
  }
  /*------------------------------ LSE Configuration -------------------------*/
  if(((RCM_OscInitStruct->OscillatorType) & RCM_OSCILLATORTYPE_LSE) == RCM_OSCILLATORTYPE_LSE)
  {
    FlagStatus       pwrclkchanged = RESET;

    /* Check the parameters */
    ASSERT_PARAM(IS_RCM_LSE(RCM_OscInitStruct->LSEState));

    /* Update LSE configuration in Backup Domain control register    */
    /* Requires to enable write access to Backup Domain of necessary */
    if(__DAL_RCM_PMU_IS_CLK_DISABLED())
    {
      __DAL_RCM_PMU_CLK_ENABLE();
      pwrclkchanged = SET;
    }

    if(DAL_IS_BIT_CLR(PMU->CTRL, PMU_CTRL_BPWEN))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PMU->CTRL, PMU_CTRL_BPWEN);

      /* Wait for Backup domain Write protection disable */
      tickstart = DAL_GetTick();

      while(DAL_IS_BIT_CLR(PMU->CTRL, PMU_CTRL_BPWEN))
      {
        if((DAL_GetTick() - tickstart) > RCM_DBP_TIMEOUT_VALUE)
        {
          return DAL_TIMEOUT;
        }
      }
    }

    /* Set the new LSE configuration -----------------------------------------*/
    __DAL_RCM_LSE_CONFIG(RCM_OscInitStruct->LSEState);
    /* Check the LSE State */
    if((RCM_OscInitStruct->LSEState) != RCM_LSE_OFF)
    {
      /* Get Start Tick*/
      tickstart = DAL_GetTick();

      /* Wait till LSE is ready */
      while(__DAL_RCM_GET_FLAG(RCM_FLAG_LSERDY) == RESET)
      {
        if((DAL_GetTick() - tickstart ) > RCM_LSE_TIMEOUT_VALUE)
        {
          return DAL_TIMEOUT;
        }
      }
    }
    else
    {
      /* Get Start Tick */
      tickstart = DAL_GetTick();

      /* Wait till LSE is ready */
      while(__DAL_RCM_GET_FLAG(RCM_FLAG_LSERDY) != RESET)
      {
        if((DAL_GetTick() - tickstart ) > RCM_LSE_TIMEOUT_VALUE)
        {
          return DAL_TIMEOUT;
        }
      }
    }

    /* Restore clock configuration if changed */
    if(pwrclkchanged == SET)
    {
      __DAL_RCM_PMU_CLK_DISABLE();
    }
  }
  /*-------------------------------- PLL Configuration -----------------------*/
  /* Check the parameters */
  ASSERT_PARAM(IS_RCM_PLL(RCM_OscInitStruct->PLL.PLLState));
  if ((RCM_OscInitStruct->PLL.PLLState) != RCM_PLL_NONE)
  {
    /* Check if the PLL is used as system clock or not */
    if(__DAL_RCM_GET_SYSCLK_SOURCE() != RCM_CFG_SCLKSWSTS_PLL)
    {
      if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_ON)
      {
        /* Check the parameters */
        ASSERT_PARAM(IS_RCM_PLLSOURCE(RCM_OscInitStruct->PLL.PLLSource));
        ASSERT_PARAM(IS_RCM_PLLB_VALUE(RCM_OscInitStruct->PLL.PLLB));
        ASSERT_PARAM(IS_RCM_PLL1A_VALUE(RCM_OscInitStruct->PLL.PLL1A));
        ASSERT_PARAM(IS_RCM_PLL1C_VALUE(RCM_OscInitStruct->PLL.PLL1C));
        ASSERT_PARAM(IS_RCM_PLLD_VALUE(RCM_OscInitStruct->PLL.PLLD));

        /* Disable the main PLL. */
        __DAL_RCM_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = DAL_GetTick();

        /* Wait till PLL is ready */
        while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLLRDY) != RESET)
        {
          if((DAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return DAL_TIMEOUT;
          }
        }

        /* Configure the main PLL clock source, multiplication and division factors. */
        WRITE_REG(RCM->PLL1CFG, (RCM_OscInitStruct->PLL.PLLSource                                            | \
                                 RCM_OscInitStruct->PLL.PLLB                                                 | \
                                 (RCM_OscInitStruct->PLL.PLL1A << RCM_PLL1CFG_PLL1A_Pos)             | \
                                 (((RCM_OscInitStruct->PLL.PLL1C >> 1U) - 1U) << RCM_PLL1CFG_PLL1C_Pos) | \
                                 (RCM_OscInitStruct->PLL.PLLD << RCM_PLL1CFG_PLLD_Pos)));
        /* Enable the main PLL. */
        __DAL_RCM_PLL_ENABLE();

        /* Get Start Tick */
        tickstart = DAL_GetTick();

        /* Wait till PLL is ready */
        while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLLRDY) == RESET)
        {
          if((DAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return DAL_TIMEOUT;
          }
        }
      }
      else
      {
        /* Disable the main PLL. */
        __DAL_RCM_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = DAL_GetTick();

        /* Wait till PLL is ready */
        while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLLRDY) != RESET)
        {
          if((DAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return DAL_TIMEOUT;
          }
        }
      }
    }
    else
    {
      /* Check if there is a request to disable the PLL used as System clock source */
      if((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF)
      {
        return DAL_ERROR;
      }
      else
      {
        /* Do not return DAL_ERROR if request repeats the current configuration */
        pll_config = RCM->PLL1CFG;
#if defined (RCM_PLL1CFG_PLLR)
        if (((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLL1CLKS) != RCM_OscInitStruct->PLL.PLLSource) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLLB) != (RCM_OscInitStruct->PLL.PLLB) << RCM_PLL1CFG_PLLB_Pos) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLL1A) != (RCM_OscInitStruct->PLL.PLL1A) << RCM_PLL1CFG_PLL1A_Pos) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLL1C) != (((RCM_OscInitStruct->PLL.PLL1C >> 1U) - 1U)) << RCM_PLL1CFG_PLL1C_Pos) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLLD) != (RCM_OscInitStruct->PLL.PLLD << RCM_PLL1CFG_PLLD_Pos)) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLLR) != (RCM_OscInitStruct->PLL.PLLR << RCM_PLL1CFG_PLLR_Pos)))
#else
        if (((RCM_OscInitStruct->PLL.PLLState) == RCM_PLL_OFF) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLL1CLKS) != RCM_OscInitStruct->PLL.PLLSource) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLLB) != (RCM_OscInitStruct->PLL.PLLB) << RCM_PLL1CFG_PLLB_Pos) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLL1A) != (RCM_OscInitStruct->PLL.PLL1A) << RCM_PLL1CFG_PLL1A_Pos) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLL1C) != (((RCM_OscInitStruct->PLL.PLL1C >> 1U) - 1U)) << RCM_PLL1CFG_PLL1C_Pos) ||
            (READ_BIT(pll_config, RCM_PLL1CFG_PLLD) != (RCM_OscInitStruct->PLL.PLLD << RCM_PLL1CFG_PLLD_Pos)))
#endif
        {
          return DAL_ERROR;
        }
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  Initializes the CPU, AHB and APB busses clocks according to the specified
  *         parameters in the RCM_ClkInitStruct.
  * @param  RCM_ClkInitStruct pointer to an RCM_OscInitTypeDef structure that
  *         contains the configuration information for the RCM peripheral.
  * @param  FLatency FLASH Latency, this parameter depend on device selected
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by DAL_RCM_GetHCLKFreq() function called within this function
  *
  * @note   The HSI is used (enabled by hardware) as system clock source after
  *         startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after startup delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *
  * @note   Depending on the device voltage range, the software has to set correctly
  *         HPRE[3:0] bits to ensure that HCLK not exceed the maximum allowed frequency
  *         (for more details refer to section above "Initialization/de-initialization functions")
  * @retval None
  */
DAL_StatusTypeDef DAL_RCM_ClockConfig(RCM_ClkInitTypeDef  *RCM_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart;

  /* Check Null pointer */
  if(RCM_ClkInitStruct == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_RCM_CLOCKTYPE(RCM_ClkInitStruct->ClockType));
  ASSERT_PARAM(IS_FLASH_LATENCY(FLatency));

  /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
    must be correctly programmed according to the frequency of the CPU clock
    (HCLK) and the supply voltage of the device. */

  /* Increasing the number of wait states because of higher CPU frequency */
  if(FLatency > __DAL_FLASH_GET_LATENCY())
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACCTRL register */
    __DAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACCTRL register */
    if(__DAL_FLASH_GET_LATENCY() != FLatency)
    {
      return DAL_ERROR;
    }
  }

  /*-------------------------- HCLK Configuration --------------------------*/
  if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_HCLK) == RCM_CLOCKTYPE_HCLK)
  {
    /* Set the highest APBx dividers in order to ensure that we do not go through
       a non-spec phase whatever we decrease or increase HCLK. */
    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
    {
      MODIFY_REG(RCM->CFG, RCM_CFG_APB1PSC, RCM_HCLK_DIV16);
    }

    if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK2) == RCM_CLOCKTYPE_PCLK2)
    {
      MODIFY_REG(RCM->CFG, RCM_CFG_APB2PSC, (RCM_HCLK_DIV16 << 3));
    }

    ASSERT_PARAM(IS_RCM_HCLK(RCM_ClkInitStruct->AHBCLKDivider));
    MODIFY_REG(RCM->CFG, RCM_CFG_AHBPSC, RCM_ClkInitStruct->AHBCLKDivider);
  }

  /*------------------------- SYSCLK Configuration ---------------------------*/
  if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_SYSCLK) == RCM_CLOCKTYPE_SYSCLK)
  {
    ASSERT_PARAM(IS_RCM_SYSCLKSOURCE(RCM_ClkInitStruct->SYSCLKSource));

    /* HSE is selected as System Clock Source */
    if(RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_HSE)
    {
      /* Check the HSE ready flag */
      if(__DAL_RCM_GET_FLAG(RCM_FLAG_HSERDY) == RESET)
      {
        return DAL_ERROR;
      }
    }
    /* PLL is selected as System Clock Source */
    else if((RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_PLLCLK)   ||
            (RCM_ClkInitStruct->SYSCLKSource == RCM_SYSCLKSOURCE_PLLRCLK))
    {
      /* Check the PLL ready flag */
      if(__DAL_RCM_GET_FLAG(RCM_FLAG_PLLRDY) == RESET)
      {
        return DAL_ERROR;
      }
    }
    /* HSI is selected as System Clock Source */
    else
    {
      /* Check the HSI ready flag */
      if(__DAL_RCM_GET_FLAG(RCM_FLAG_HSIRDY) == RESET)
      {
        return DAL_ERROR;
      }
    }

    __DAL_RCM_SYSCLK_CONFIG(RCM_ClkInitStruct->SYSCLKSource);

    /* Get Start Tick */
    tickstart = DAL_GetTick();

    while (__DAL_RCM_GET_SYSCLK_SOURCE() != (RCM_ClkInitStruct->SYSCLKSource << RCM_CFG_SCLKSWSTS_Pos))
    {
      if ((DAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
      {
        return DAL_TIMEOUT;
      }
    }
  }

  /* Decreasing the number of wait states because of lower CPU frequency */
  if(FLatency < __DAL_FLASH_GET_LATENCY())
  {
     /* Program the new number of wait states to the LATENCY bits in the FLASH_ACCTRL register */
    __DAL_FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACCTRL register */
    if(__DAL_FLASH_GET_LATENCY() != FLatency)
    {
      return DAL_ERROR;
    }
  }

  /*-------------------------- PCLK1 Configuration ---------------------------*/
  if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK1) == RCM_CLOCKTYPE_PCLK1)
  {
    ASSERT_PARAM(IS_RCM_PCLK(RCM_ClkInitStruct->APB1CLKDivider));
    MODIFY_REG(RCM->CFG, RCM_CFG_APB1PSC, RCM_ClkInitStruct->APB1CLKDivider);
  }

  /*-------------------------- PCLK2 Configuration ---------------------------*/
  if(((RCM_ClkInitStruct->ClockType) & RCM_CLOCKTYPE_PCLK2) == RCM_CLOCKTYPE_PCLK2)
  {
    ASSERT_PARAM(IS_RCM_PCLK(RCM_ClkInitStruct->APB2CLKDivider));
    MODIFY_REG(RCM->CFG, RCM_CFG_APB2PSC, ((RCM_ClkInitStruct->APB2CLKDivider) << 3U));
  }

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = DAL_RCM_GetSysClockFreq() >> AHBPrescTable[(RCM->CFG & RCM_CFG_AHBPSC)>> RCM_CFG_AHBPSC_Pos];

  /* Configure the source of time base considering new system clocks settings */
  DAL_InitTick (uwTickPrio);

  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup RCM_Exported_Functions_Group2 Peripheral Control functions
 *  @brief   RCM clocks control functions
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the RCM Clocks
    frequencies.

@endverbatim
  * @{
  */

/**
  * @brief  Selects the clock source to output on MCO1 pin(PA8) or on MCO2 pin(PC9).
  * @note   PA8/PC9 should be configured in alternate function mode.
  * @param  RCM_MCOx specifies the output direction for the clock source.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO1: Clock source to output on MCO1 pin(PA8).
  *            @arg RCM_MCO2: Clock source to output on MCO2 pin(PC9).
  * @param  RCM_MCOSource specifies the clock source to output.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCO1SOURCE_HSI: HSI clock selected as MCO1 source
  *            @arg RCM_MCO1SOURCE_LSE: LSE clock selected as MCO1 source
  *            @arg RCM_MCO1SOURCE_HSE: HSE clock selected as MCO1 source
  *            @arg RCM_MCO1SOURCE_PLLCLK: main PLL clock selected as MCO1 source
  *            @arg RCM_MCO2SOURCE_SYSCLK: System clock (SYSCLK) selected as MCO2 source
  *            @arg RCM_MCO2SOURCE_PLLI2SCLK: PLLI2S clock selected as MCO2 source, available for all APM32F4 devices except APM32F410xx
  *            @arg RCM_MCO2SOURCE_I2SCLK: I2SCLK clock selected as MCO2 source, available only for APM32F410Rx devices
  *            @arg RCM_MCO2SOURCE_HSE: HSE clock selected as MCO2 source
  *            @arg RCM_MCO2SOURCE_PLLCLK: main PLL clock selected as MCO2 source
  * @param  RCM_MCODiv specifies the MCOx prescaler.
  *          This parameter can be one of the following values:
  *            @arg RCM_MCODIV_1: no division applied to MCOx clock
  *            @arg RCM_MCODIV_2: division by 2 applied to MCOx clock
  *            @arg RCM_MCODIV_3: division by 3 applied to MCOx clock
  *            @arg RCM_MCODIV_4: division by 4 applied to MCOx clock
  *            @arg RCM_MCODIV_5: division by 5 applied to MCOx clock
  * @note  For APM32F410Rx devices to output I2SCLK clock on MCO2 you should have
  *        at last one of the SPI clocks enabled (SPI1, SPI2 or SPI5).
  * @retval None
  */
void DAL_RCM_MCOConfig(uint32_t RCM_MCOx, uint32_t RCM_MCOSource, uint32_t RCM_MCODiv)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  /* Check the parameters */
  ASSERT_PARAM(IS_RCM_MCO(RCM_MCOx));
  ASSERT_PARAM(IS_RCM_MCODIV(RCM_MCODiv));
  /* RCM_MCO1 */
  if(RCM_MCOx == RCM_MCO1)
  {
    ASSERT_PARAM(IS_RCM_MCO1SOURCE(RCM_MCOSource));

    /* MCO1 Clock Enable */
    __MCO1_CLK_ENABLE();

    /* Configure the MCO1 pin in alternate function mode */
    GPIO_InitStruct.Pin = MCO1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    DAL_GPIO_Init(MCO1_GPIO_PORT, &GPIO_InitStruct);

    /* Mask MCO1 and MCO1PRE[2:0] bits then Select MCO1 clock source and prescaler */
    MODIFY_REG(RCM->CFG, (RCM_CFG_MCO1SEL | RCM_CFG_MCO1PSC), (RCM_MCOSource | RCM_MCODiv));

   /* This RCM MCO1 enable feature is available only on APM32F410xx devices */
#if defined(RCM_CFG_MCO1EN)
    __DAL_RCM_MCO1_ENABLE();
#endif /* RCM_CFG_MCO1EN */
  }
#if defined(RCM_CFG_MCO2SEL)
  else
  {
    ASSERT_PARAM(IS_RCM_MCO2SOURCE(RCM_MCOSource));

    /* MCO2 Clock Enable */
    __MCO2_CLK_ENABLE();

    /* Configure the MCO2 pin in alternate function mode */
    GPIO_InitStruct.Pin = MCO2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    DAL_GPIO_Init(MCO2_GPIO_PORT, &GPIO_InitStruct);

    /* Mask MCO2 and MCO2PRE[2:0] bits then Select MCO2 clock source and prescaler */
    MODIFY_REG(RCM->CFG, (RCM_CFG_MCO2SEL | RCM_CFG_MCO2PSC), (RCM_MCOSource | (RCM_MCODiv << 3U)));

   /* This RCM MCO2 enable feature is available only on APM32F410Rx devices */
#if defined(RCM_CFG_MCO2EN)
    __DAL_RCM_MCO2_ENABLE();
#endif /* RCM_CFG_MCO2EN */
  }
#endif /* RCM_CFG_MCO2SEL */
}

/**
  * @brief  Enables the Clock Security System.
  * @note   If a failure is detected on the HSE oscillator clock, this oscillator
  *         is automatically disabled and an interrupt is generated to inform the
  *         software about the failure (Clock Security System Interrupt, CSSI),
  *         allowing the MCU to perform rescue operations. The CSSI is linked to
  *         the Cortex-M4 NMI (Non-Maskable Interrupt) exception vector.
  * @retval None
  */
void DAL_RCM_EnableCSS(void)
{
  *(__IO uint32_t *) RCM_CTRL_CSSEN_BB = (uint32_t)ENABLE;
}

/**
  * @brief  Disables the Clock Security System.
  * @retval None
  */
void DAL_RCM_DisableCSS(void)
{
  *(__IO uint32_t *) RCM_CTRL_CSSEN_BB = (uint32_t)DISABLE;
}

/**
  * @brief  Returns the SYSCLK frequency
  *
  * @note   The system frequency computed by this function is not the real
  *         frequency in the chip. It is calculated based on the predefined
  *         constant and the selected clock source:
  * @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
  * @note     If SYSCLK source is HSE, function returns values based on HSE_VALUE(**)
  * @note     If SYSCLK source is PLL, function returns values based on HSE_VALUE(**)
  *           or HSI_VALUE(*) multiplied/divided by the PLL factors.
  * @note     (*) HSI_VALUE is a constant defined in apm32f4xx_dal_cfg.h file (default value
  *               16 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature.
  * @note     (**) HSE_VALUE is a constant defined in apm32f4xx_dal_cfg.h file (default value
  *                25 MHz), user has to ensure that HSE_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                have wrong result.
  *
  * @note   The result of this function could be not correct when using fractional
  *         value for HSE crystal.
  *
  * @note   This function can be used by the user application to compute the
  *         baudrate for the communication peripherals or configure other parameters.
  *
  * @note   Each time SYSCLK changes, this function must be called to update the
  *         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
  *
  *
  * @retval SYSCLK frequency
  */
__weak uint32_t DAL_RCM_GetSysClockFreq(void)
{
  uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
  uint32_t sysclockfreq = 0U;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (RCM->CFG & RCM_CFG_SCLKSWSTS)
  {
    case RCM_CFG_SCLKSWSTS_HSI:  /* HSI used as system clock source */
    {
      sysclockfreq = HSI_VALUE;
       break;
    }
    case RCM_CFG_SCLKSWSTS_HSE:  /* HSE used as system clock  source */
    {
      sysclockfreq = HSE_VALUE;
      break;
    }
    case RCM_CFG_SCLKSWSTS_PLL:  /* PLL used as system clock  source */
    {
      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLB) * PLL1A
      SYSCLK = PLL_VCO / PLL1C */
      pllm = RCM->PLL1CFG & RCM_PLL1CFG_PLLB;
      if(__DAL_RCM_GET_PLL_OSCSOURCE() != RCM_PLLSOURCE_HSI)
      {
        /* HSE used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) HSE_VALUE * ((uint64_t) ((RCM->PLL1CFG & RCM_PLL1CFG_PLL1A) >> RCM_PLL1CFG_PLL1A_Pos)))) / (uint64_t)pllm);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) HSI_VALUE * ((uint64_t) ((RCM->PLL1CFG & RCM_PLL1CFG_PLL1A) >> RCM_PLL1CFG_PLL1A_Pos)))) / (uint64_t)pllm);
      }
      pllp = ((((RCM->PLL1CFG & RCM_PLL1CFG_PLL1C) >> RCM_PLL1CFG_PLL1C_Pos) + 1U) *2U);

      sysclockfreq = pllvco/pllp;
      break;
    }
    default:
    {
      sysclockfreq = HSI_VALUE;
      break;
    }
  }
  return sysclockfreq;
}

/**
  * @brief  Returns the HCLK frequency
  * @note   Each time HCLK changes, this function must be called to update the
  *         right HCLK value. Otherwise, any configuration based on this function will be incorrect.
  *
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated within this function
  * @retval HCLK frequency
  */
uint32_t DAL_RCM_GetHCLKFreq(void)
{
  return SystemCoreClock;
}

/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t DAL_RCM_GetPCLK1Freq(void)
{
  /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
  return (DAL_RCM_GetHCLKFreq() >> APBPrescTable[(RCM->CFG & RCM_CFG_APB1PSC)>> RCM_CFG_APB1PSC_Pos]);
}

/**
  * @brief  Returns the PCLK2 frequency
  * @note   Each time PCLK2 changes, this function must be called to update the
  *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK2 frequency
  */
uint32_t DAL_RCM_GetPCLK2Freq(void)
{
  /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
  return (DAL_RCM_GetHCLKFreq()>> APBPrescTable[(RCM->CFG & RCM_CFG_APB2PSC)>> RCM_CFG_APB2PSC_Pos]);
}

/**
  * @brief  Configures the RCM_OscInitStruct according to the internal
  * RCM configuration registers.
  * @param  RCM_OscInitStruct pointer to an RCM_OscInitTypeDef structure that
  * will be configured.
  * @retval None
  */
__weak void DAL_RCM_GetOscConfig(RCM_OscInitTypeDef  *RCM_OscInitStruct)
{
  /* Set all possible values for the Oscillator type parameter ---------------*/
  RCM_OscInitStruct->OscillatorType = RCM_OSCILLATORTYPE_HSE | RCM_OSCILLATORTYPE_HSI | RCM_OSCILLATORTYPE_LSE | RCM_OSCILLATORTYPE_LSI;

  /* Get the HSE configuration -----------------------------------------------*/
  if((RCM->CTRL &RCM_CTRL_HSEBCFG) == RCM_CTRL_HSEBCFG)
  {
    RCM_OscInitStruct->HSEState = RCM_HSE_BYPASS;
  }
  else if((RCM->CTRL &RCM_CTRL_HSEEN) == RCM_CTRL_HSEEN)
  {
    RCM_OscInitStruct->HSEState = RCM_HSE_ON;
  }
  else
  {
    RCM_OscInitStruct->HSEState = RCM_HSE_OFF;
  }

  /* Get the HSI configuration -----------------------------------------------*/
  if((RCM->CTRL &RCM_CTRL_HSIEN) == RCM_CTRL_HSIEN)
  {
    RCM_OscInitStruct->HSIState = RCM_HSI_ON;
  }
  else
  {
    RCM_OscInitStruct->HSIState = RCM_HSI_OFF;
  }

  RCM_OscInitStruct->HSICalibrationValue = (uint32_t)((RCM->CTRL &RCM_CTRL_HSITRM) >> RCM_CTRL_HSITRM_Pos);

  /* Get the LSE configuration -----------------------------------------------*/
  if((RCM->BDCTRL &RCM_BDCTRL_LSEBCFG) == RCM_BDCTRL_LSEBCFG)
  {
    RCM_OscInitStruct->LSEState = RCM_LSE_BYPASS;
  }
  else if((RCM->BDCTRL &RCM_BDCTRL_LSEEN) == RCM_BDCTRL_LSEEN)
  {
    RCM_OscInitStruct->LSEState = RCM_LSE_ON;
  }
  else
  {
    RCM_OscInitStruct->LSEState = RCM_LSE_OFF;
  }

  /* Get the LSI configuration -----------------------------------------------*/
  if((RCM->CSTS &RCM_CSTS_LSIEN) == RCM_CSTS_LSIEN)
  {
    RCM_OscInitStruct->LSIState = RCM_LSI_ON;
  }
  else
  {
    RCM_OscInitStruct->LSIState = RCM_LSI_OFF;
  }

  /* Get the PLL configuration -----------------------------------------------*/
  if((RCM->CTRL &RCM_CTRL_PLL1EN) == RCM_CTRL_PLL1EN)
  {
    RCM_OscInitStruct->PLL.PLLState = RCM_PLL_ON;
  }
  else
  {
    RCM_OscInitStruct->PLL.PLLState = RCM_PLL_OFF;
  }
  RCM_OscInitStruct->PLL.PLLSource = (uint32_t)(RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS);
  RCM_OscInitStruct->PLL.PLLB = (uint32_t)(RCM->PLL1CFG & RCM_PLL1CFG_PLLB);
  RCM_OscInitStruct->PLL.PLL1A = (uint32_t)((RCM->PLL1CFG & RCM_PLL1CFG_PLL1A) >> RCM_PLL1CFG_PLL1A_Pos);
  RCM_OscInitStruct->PLL.PLL1C = (uint32_t)((((RCM->PLL1CFG & RCM_PLL1CFG_PLL1C) + RCM_PLL1CFG_PLL1C_0) << 1U) >> RCM_PLL1CFG_PLL1C_Pos);
  RCM_OscInitStruct->PLL.PLLD = (uint32_t)((RCM->PLL1CFG & RCM_PLL1CFG_PLLD) >> RCM_PLL1CFG_PLLD_Pos);
}

/**
  * @brief  Configures the RCM_ClkInitStruct according to the internal
  * RCM configuration registers.
  * @param  RCM_ClkInitStruct pointer to an RCM_ClkInitTypeDef structure that
  * will be configured.
  * @param  pFLatency Pointer on the Flash Latency.
  * @retval None
  */
void DAL_RCM_GetClockConfig(RCM_ClkInitTypeDef  *RCM_ClkInitStruct, uint32_t *pFLatency)
{
  /* Set all possible values for the Clock type parameter --------------------*/
  RCM_ClkInitStruct->ClockType = RCM_CLOCKTYPE_SYSCLK | RCM_CLOCKTYPE_HCLK | RCM_CLOCKTYPE_PCLK1 | RCM_CLOCKTYPE_PCLK2;

  /* Get the SYSCLK configuration --------------------------------------------*/
  RCM_ClkInitStruct->SYSCLKSource = (uint32_t)(RCM->CFG & RCM_CFG_SCLKSEL);

  /* Get the HCLK configuration ----------------------------------------------*/
  RCM_ClkInitStruct->AHBCLKDivider = (uint32_t)(RCM->CFG & RCM_CFG_AHBPSC);

  /* Get the APB1 configuration ----------------------------------------------*/
  RCM_ClkInitStruct->APB1CLKDivider = (uint32_t)(RCM->CFG & RCM_CFG_APB1PSC);

  /* Get the APB2 configuration ----------------------------------------------*/
  RCM_ClkInitStruct->APB2CLKDivider = (uint32_t)((RCM->CFG & RCM_CFG_APB2PSC) >> 3U);

  /* Get the Flash Wait State (Latency) configuration ------------------------*/
  *pFLatency = (uint32_t)(FLASH->ACCTRL & FLASH_ACCTRL_WAITP);
}

/**
  * @brief This function handles the RCM CSS interrupt request.
  * @note This API should be called under the NMI_Handler().
  * @retval None
  */
void DAL_RCM_NMI_IRQHandler(void)
{
  /* Check RCM CSSF flag  */
  if(__DAL_RCM_GET_IT(RCM_IT_CSS))
  {
    /* RCM Clock Security System interrupt user callback */
    DAL_RCM_CSSCallback();

    /* Clear RCM CSS pending bit */
    __DAL_RCM_CLEAR_IT(RCM_IT_CSS);
  }
}

/**
  * @brief  RCM Clock Security System interrupt callback
  * @retval None
  */
__weak void DAL_RCM_CSSCallback(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_RCM_CSSCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_RCM_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

