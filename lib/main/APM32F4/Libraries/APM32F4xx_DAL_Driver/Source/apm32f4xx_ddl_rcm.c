/**
  *
  * @file    apm32f4xx_ddl_rcm.c
  * @brief   RCM DDL module driver.
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
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_rcm.h"
#ifdef  USE_FULL_ASSERT
  #include "apm32_assert.h"
#else
  #define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif
/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(RCM)

/** @addtogroup RCM_DDL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup RCM_DDL_Private_Macros
  * @{
  */

#if defined(SDIO)
#define IS_DDL_RCM_SDIO_CLKSOURCE(__VALUE__)  (((__VALUE__) == DDL_RCM_SDIO_CLKSOURCE))
#endif /* SDIO */

#if defined(RNG)
#define IS_DDL_RCM_RNG_CLKSOURCE(__VALUE__)    (((__VALUE__) == DDL_RCM_RNG_CLKSOURCE))
#endif /* RNG */

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
#define IS_DDL_RCM_USB_CLKSOURCE(__VALUE__)    (((__VALUE__) == DDL_RCM_USB_CLKSOURCE))
#endif /* USB_OTG_FS || USB_OTG_HS */

#if defined(RCM_DCKCFGR_I2S2SRC)
#define IS_DDL_RCM_I2S_CLKSOURCE(__VALUE__)    (((__VALUE__) == DDL_RCM_I2S1_CLKSOURCE) \
                                            || ((__VALUE__) == DDL_RCM_I2S2_CLKSOURCE))
#else
#define IS_DDL_RCM_I2S_CLKSOURCE(__VALUE__)    (((__VALUE__) == DDL_RCM_I2S1_CLKSOURCE))
#endif /* RCM_DCKCFGR_I2S2SRC */

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup RCM_DDL_Private_Functions RCM Private functions
  * @{
  */
uint32_t RCM_GetSystemClockFreq(void);
uint32_t RCM_GetHCLKClockFreq(uint32_t SYSCLK_Frequency);
uint32_t RCM_GetPCLK1ClockFreq(uint32_t HCLK_Frequency);
uint32_t RCM_GetPCLK2ClockFreq(uint32_t HCLK_Frequency);
uint32_t RCM_PLL_GetFreqDomain_SYS(uint32_t SYSCLK_Source);
uint32_t RCM_PLL_GetFreqDomain_48M(void);

#if defined(RCM_PLLI2S_SUPPORT)
uint32_t RCM_PLLI2S_GetFreqDomain_I2S(void);
#endif /* RCM_PLLI2S_SUPPORT */
/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @addtogroup RCM_DDL_Exported_Functions
  * @{
  */

/** @addtogroup RCM_DDL_EF_Init
  * @{
  */

/**
  * @brief  Reset the RCM clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *         - HSI ON and used as system clock source
  *         - HSE and PLL OFF
  *         - AHB, APB1 and APB2 prescaler set to 1.
  *         - CSS, MCO OFF
  *         - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *         - Peripheral clocks
  *         - LSI, LSE and RTC clocks
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RCM registers are de-initialized
  *          - ERROR: not applicable
  */
ErrorStatus DDL_RCM_DeInit(void)
{
  __IO uint32_t vl_mask;

  /* Set HSIEN bit */
  DDL_RCM_HSI_Enable();

  /* Wait for HSI READY bit */
  while(DDL_RCM_HSI_IsReady() != 1U)
  {}

  /* Reset CFG register */
  DDL_RCM_WriteReg(CFG, 0x00000000U);

  /* Read CTRL register */
  vl_mask = DDL_RCM_ReadReg(CTRL);

  /* Reset HSEEN, HSEBCFG, PLL1EN, CSSEN bits */
  CLEAR_BIT(vl_mask,
            (RCM_CTRL_HSEEN | RCM_CTRL_HSEBCFG | RCM_CTRL_PLL1EN | RCM_CTRL_CSSEN));

#if defined(RCM_PLLI2S_SUPPORT)
  /* Reset PLL2EN bit */
  CLEAR_BIT(vl_mask, RCM_CTRL_PLL2EN);
#endif /* RCM_PLLI2S_SUPPORT */

  /* Write new value in CTRL register */
  DDL_RCM_WriteReg(CTRL, vl_mask);

  /* Set HSITRM bits to the reset value*/
  DDL_RCM_HSI_SetCalibTrimming(0x10U);

  /* Wait for PLL READY bit to be reset */
  while(DDL_RCM_PLL_IsReady() != 0U)
  {}

  /* Reset PLL1CFG register */
  DDL_RCM_WriteReg(PLL1CFG, RCM_PLL1CFG_RST_VALUE);

#if defined(RCM_PLLI2S_SUPPORT)
  /* Reset PLL2CFG register */
  DDL_RCM_WriteReg(PLL2CFG, RCM_PLL2CFG_RST_VALUE);
#endif /* RCM_PLLI2S_SUPPORT */

  /* Disable all interrupts */
  CLEAR_BIT(RCM->INT, RCM_INT_LSIRDYEN | RCM_INT_LSERDYEN | RCM_INT_HSIRDYEN | RCM_INT_HSERDYEN | RCM_INT_PLL1RDYEN);

#if defined(RCM_INT_PLL2RDYEN)
  CLEAR_BIT(RCM->INT, RCM_INT_PLL2RDYEN);
#endif /* RCM_INT_PLL2RDYEN */

  /* Clear all interrupt flags */
  SET_BIT(RCM->INT, RCM_INT_LSIRDYCLR | RCM_INT_LSERDYCLR | RCM_INT_HSIRDYCLR | RCM_INT_HSERDYCLR | RCM_INT_PLL1RDYCLR | RCM_INT_CSSCLR);

#if defined(RCM_INT_PLL2RDYCLR)
  SET_BIT(RCM->INT, RCM_INT_PLL2RDYCLR);
#endif /* RCM_INT_PLL2RDYCLR */

  /* Clear LSIEN bit */
  CLEAR_BIT(RCM->CSTS, RCM_CSTS_LSIEN);

  /* Reset all RSTFLGCLR flags */
  SET_BIT(RCM->CSTS, RCM_CSTS_RSTFLGCLR);

  return SUCCESS;
}

/**
  * @}
  */

/** @addtogroup RCM_DDL_EF_Get_Freq
  * @brief  Return the frequencies of different on chip clocks;  System, AHB, APB1 and APB2 buses clocks
  *         and different peripheral clocks available on the device.
  * @note   If SYSCLK source is HSI, function returns values based on HSI_VALUE(**)
  * @note   If SYSCLK source is HSE, function returns values based on HSE_VALUE(***)
  * @note   If SYSCLK source is PLL, function returns values based on HSE_VALUE(***)
  *         or HSI_VALUE(**) multiplied/divided by the PLL factors.
  * @note   (**) HSI_VALUE is a constant defined in this file (default value
  *              16 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  * @note   (***) HSE_VALUE is a constant defined in this file (default value
  *               25 MHz), user has to ensure that HSE_VALUE is same as the real
  *               frequency of the crystal used. Otherwise, this function may
  *               have wrong result.
  * @note   The result of this function could be incorrect when using fractional
  *         value for HSE crystal.
  * @note   This function can be used by the user application to compute the
  *         baud-rate for the communication peripherals or configure other parameters.
  * @{
  */

/**
  * @brief  Return the frequencies of different on chip clocks;  System, AHB, APB1 and APB2 buses clocks
  * @note   Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
  *         must be called to update structure fields. Otherwise, any
  *         configuration based on this function will be incorrect.
  * @param  RCM_Clocks pointer to a @ref DDL_RCM_ClocksTypeDef structure which will hold the clocks frequencies
  * @retval None
  */
void DDL_RCM_GetSystemClocksFreq(DDL_RCM_ClocksTypeDef *RCM_Clocks)
{
  /* Get SYSCLK frequency */
  RCM_Clocks->SYSCLK_Frequency = RCM_GetSystemClockFreq();

  /* HCLK clock frequency */
  RCM_Clocks->HCLK_Frequency   = RCM_GetHCLKClockFreq(RCM_Clocks->SYSCLK_Frequency);

  /* PCLK1 clock frequency */
  RCM_Clocks->PCLK1_Frequency  = RCM_GetPCLK1ClockFreq(RCM_Clocks->HCLK_Frequency);

  /* PCLK2 clock frequency */
  RCM_Clocks->PCLK2_Frequency  = RCM_GetPCLK2ClockFreq(RCM_Clocks->HCLK_Frequency);
}

/**
  * @brief  Return I2Sx clock frequency
  * @param  I2SxSource This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_I2S1_CLKSOURCE
  *         @arg @ref DDL_RCM_I2S2_CLKSOURCE (*)
  *
  *         (*) value not defined in all devices.
  * @retval I2S clock frequency (in Hz)
  *         - @ref  DDL_RCM_PERIPH_FREQUENCY_NO indicates that oscillator is not ready
  */
uint32_t DDL_RCM_GetI2SClockFreq(uint32_t I2SxSource)
{
  uint32_t i2s_frequency = DDL_RCM_PERIPH_FREQUENCY_NO;

  /* Check parameter */
  ASSERT_PARAM(IS_DDL_RCM_I2S_CLKSOURCE(I2SxSource));

  if (I2SxSource == DDL_RCM_I2S1_CLKSOURCE)
  {
    /* I2S1 CLK clock frequency */
    switch (DDL_RCM_GetI2SClockSource(I2SxSource))
    {
#if defined(RCM_PLLI2S_SUPPORT)
      case DDL_RCM_I2S1_CLKSOURCE_PLLI2S:       /* I2S1 Clock is PLLI2S */
        if (DDL_RCM_PLLI2S_IsReady())
        {
          i2s_frequency = RCM_PLLI2S_GetFreqDomain_I2S();
        }
        break;
#endif /* RCM_PLLI2S_SUPPORT */

      case DDL_RCM_I2S1_CLKSOURCE_PIN:          /* I2S1 Clock is External clock */
      default:
        i2s_frequency = EXTERNAL_CLOCK_VALUE;
        break;
    }
  }

  return i2s_frequency;
}

#if defined(SDIO)
/**
  * @brief  Return SDIOx clock frequency
  * @param  SDIOxSource This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_SDIO_CLKSOURCE
  * @retval SDIO clock frequency (in Hz)
  *         - @ref  DDL_RCM_PERIPH_FREQUENCY_NO indicates that oscillator is not ready
  */
uint32_t DDL_RCM_GetSDIOClockFreq(uint32_t SDIOxSource)
{
  uint32_t SDIO_frequency = DDL_RCM_PERIPH_FREQUENCY_NO;

  /* Check parameter */
  ASSERT_PARAM(IS_DDL_RCM_SDIO_CLKSOURCE(SDIOxSource));

  if (SDIOxSource == DDL_RCM_SDIO_CLKSOURCE)
  {
#if defined(RCM_DCKCFGR_SDIOSEL) || defined(RCM_DCKCFGR2_SDIOSEL)
    /* SDIOCLK clock frequency */
    switch (DDL_RCM_GetSDIOClockSource(SDIOxSource))
    {
      case DDL_RCM_SDIO_CLKSOURCE_PLL48CLK:         /* PLL48M clock used as SDIO clock source */
        switch (DDL_RCM_GetCK48MClockSource(DDL_RCM_CK48M_CLKSOURCE))
        {
          case DDL_RCM_CK48M_CLKSOURCE_PLL:         /* PLL clock used as 48Mhz domain clock */
            if (DDL_RCM_PLL_IsReady())
            {
              SDIO_frequency = RCM_PLL_GetFreqDomain_48M();
            }
          break;

#if defined(RCM_PLLSAI_SUPPORT)
          case DDL_RCM_CK48M_CLKSOURCE_PLLSAI:      /* PLLSAI clock used as 48Mhz domain clock */
          default:
            if (DDL_RCM_PLLSAI_IsReady())
            {
              SDIO_frequency = RCM_PLLSAI_GetFreqDomain_48M();
            }
            break;
#endif /* RCM_PLLSAI_SUPPORT */

#if defined(RCM_PLL2CFG_PLLI2SQ) && !defined(RCM_DCKCFGR_PLLI2SDIVQ)
          case DDL_RCM_CK48M_CLKSOURCE_PLLI2S:      /* PLLI2S clock used as 48Mhz domain clock */
          default:
            if (DDL_RCM_PLLI2S_IsReady())
            {
              SDIO_frequency = RCM_PLLI2S_GetFreqDomain_48M();
            }
            break;
#endif /* RCM_PLL2CFG_PLLI2SQ && !RCM_DCKCFGR_PLLI2SDIVQ */
        }
        break;

      case DDL_RCM_SDIO_CLKSOURCE_SYSCLK:           /* PLL clock used as SDIO clock source */
      default:
      SDIO_frequency = RCM_GetSystemClockFreq();
      break;
    }
#else
    /* PLL clock used as 48Mhz domain clock */
    if (DDL_RCM_PLL_IsReady())
    {
      SDIO_frequency = RCM_PLL_GetFreqDomain_48M();
    }
#endif /* RCM_DCKCFGR_SDIOSEL || RCM_DCKCFGR2_SDIOSEL */
  }

  return SDIO_frequency;
}
#endif /* SDIO */

#if defined(RNG)
/**
  * @brief  Return RNGx clock frequency
  * @param  RNGxSource This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_RNG_CLKSOURCE
  * @retval RNG clock frequency (in Hz)
  *         - @ref  DDL_RCM_PERIPH_FREQUENCY_NO indicates that oscillator is not ready
  */
uint32_t DDL_RCM_GetRNGClockFreq(uint32_t RNGxSource)
{
  uint32_t rng_frequency = DDL_RCM_PERIPH_FREQUENCY_NO;

  /* Check parameter */
  ASSERT_PARAM(IS_DDL_RCM_RNG_CLKSOURCE(RNGxSource));

#if defined(RCM_DCKCFGR_CK48MSEL) || defined(RCM_DCKCFGR2_CK48MSEL)
  /* RNGCLK clock frequency */
  switch (DDL_RCM_GetRNGClockSource(RNGxSource))
  {
#if defined(RCM_PLL2CFG_PLLI2SQ) && !defined(RCM_DCKCFGR_PLLI2SDIVQ)
    case DDL_RCM_RNG_CLKSOURCE_PLLI2S:        /* PLLI2S clock used as RNG clock source */
      if (DDL_RCM_PLLI2S_IsReady())
      {
        rng_frequency = RCM_PLLI2S_GetFreqDomain_48M();
      }
      break;
#endif /* RCM_PLL2CFG_PLLI2SQ && !RCM_DCKCFGR_PLLI2SDIVQ */

#if defined(RCM_PLLSAI_SUPPORT)
    case DDL_RCM_RNG_CLKSOURCE_PLLSAI:        /* PLLSAI clock used as RNG clock source */
      if (DDL_RCM_PLLSAI_IsReady())
      {
        rng_frequency = RCM_PLLSAI_GetFreqDomain_48M();
      }
      break;
#endif /* RCM_PLLSAI_SUPPORT */

    case DDL_RCM_RNG_CLKSOURCE_PLL:           /* PLL clock used as RNG clock source */
    default:
      if (DDL_RCM_PLL_IsReady())
      {
        rng_frequency = RCM_PLL_GetFreqDomain_48M();
      }
      break;
  }
#else
  /* PLL clock used as RNG clock source */
  if (DDL_RCM_PLL_IsReady())
  {
    rng_frequency = RCM_PLL_GetFreqDomain_48M();
  }
#endif /* RCM_DCKCFGR_CK48MSEL || RCM_DCKCFGR2_CK48MSEL */

  return rng_frequency;
}
#endif /* RNG */

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
/**
  * @brief  Return USBx clock frequency
  * @param  USBxSource This parameter can be one of the following values:
  *         @arg @ref DDL_RCM_USB_CLKSOURCE
  * @retval USB clock frequency (in Hz)
  *         - @ref  DDL_RCM_PERIPH_FREQUENCY_NO indicates that oscillator is not ready
  */
uint32_t DDL_RCM_GetUSBClockFreq(uint32_t USBxSource)
{
  uint32_t usb_frequency = DDL_RCM_PERIPH_FREQUENCY_NO;

  /* Check parameter */
  ASSERT_PARAM(IS_DDL_RCM_USB_CLKSOURCE(USBxSource));

#if defined(RCM_DCKCFGR_CK48MSEL) || defined(RCM_DCKCFGR2_CK48MSEL)
  /* USBCLK clock frequency */
  switch (DDL_RCM_GetUSBClockSource(USBxSource))
  {
#if defined(RCM_PLL2CFG_PLLI2SQ) && !defined(RCM_DCKCFGR_PLLI2SDIVQ)
    case DDL_RCM_USB_CLKSOURCE_PLLI2S:       /* PLLI2S clock used as USB clock source */
      if (DDL_RCM_PLLI2S_IsReady())
      {
        usb_frequency = RCM_PLLI2S_GetFreqDomain_48M();
      }
      break;

#endif /* RCM_PLL2CFG_PLLI2SQ && !RCM_DCKCFGR_PLLI2SDIVQ */

#if defined(RCM_PLLSAI_SUPPORT)
    case DDL_RCM_USB_CLKSOURCE_PLLSAI:       /* PLLSAI clock used as USB clock source */
      if (DDL_RCM_PLLSAI_IsReady())
      {
        usb_frequency = RCM_PLLSAI_GetFreqDomain_48M();
      }
      break;
#endif /* RCM_PLLSAI_SUPPORT */

    case DDL_RCM_USB_CLKSOURCE_PLL:          /* PLL clock used as USB clock source */
    default:
      if (DDL_RCM_PLL_IsReady())
      {
        usb_frequency = RCM_PLL_GetFreqDomain_48M();
      }
      break;
  }
#else
  /* PLL clock used as USB clock source */
  if (DDL_RCM_PLL_IsReady())
  {
    usb_frequency = RCM_PLL_GetFreqDomain_48M();
  }
#endif /* RCM_DCKCFGR_CK48MSEL || RCM_DCKCFGR2_CK48MSEL */

  return usb_frequency;
}
#endif /* USB_OTG_FS || USB_OTG_HS */

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup RCM_DDL_Private_Functions
  * @{
  */

/**
  * @brief  Return SYSTEM clock frequency
  * @retval SYSTEM clock frequency (in Hz)
  */
uint32_t RCM_GetSystemClockFreq(void)
{
  uint32_t frequency = 0U;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (DDL_RCM_GetSysClkSource())
  {
    case DDL_RCM_SYS_CLKSOURCE_STATUS_HSI:  /* HSI used as system clock  source */
      frequency = HSI_VALUE;
      break;

    case DDL_RCM_SYS_CLKSOURCE_STATUS_HSE:  /* HSE used as system clock  source */
      frequency = HSE_VALUE;
      break;

    case DDL_RCM_SYS_CLKSOURCE_STATUS_PLL:  /* PLL used as system clock  source */
      frequency = RCM_PLL_GetFreqDomain_SYS(DDL_RCM_SYS_CLKSOURCE_STATUS_PLL);
      break;

    default:
      frequency = HSI_VALUE;
      break;
  }

  return frequency;
}

/**
  * @brief  Return HCLK clock frequency
  * @param  SYSCLK_Frequency SYSCLK clock frequency
  * @retval HCLK clock frequency (in Hz)
  */
uint32_t RCM_GetHCLKClockFreq(uint32_t SYSCLK_Frequency)
{
  /* HCLK clock frequency */
  return __DDL_RCM_CALC_HCLK_FREQ(SYSCLK_Frequency, DDL_RCM_GetAHBPrescaler());
}

/**
  * @brief  Return PCLK1 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK1 clock frequency (in Hz)
  */
uint32_t RCM_GetPCLK1ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK1 clock frequency */
  return __DDL_RCM_CALC_PCLK1_FREQ(HCLK_Frequency, DDL_RCM_GetAPB1Prescaler());
}

/**
  * @brief  Return PCLK2 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK2 clock frequency (in Hz)
  */
uint32_t RCM_GetPCLK2ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK2 clock frequency */
  return __DDL_RCM_CALC_PCLK2_FREQ(HCLK_Frequency, DDL_RCM_GetAPB2Prescaler());
}

/**
  * @brief  Return PLL clock frequency used for system domain
  * @param  SYSCLK_Source System clock source
  * @retval PLL clock frequency (in Hz)
  */
uint32_t RCM_PLL_GetFreqDomain_SYS(uint32_t SYSCLK_Source)
{
  uint32_t pllinputfreq = 0U, pllsource = 0U, plloutputfreq = 0U;

  /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLB) * PLL1A
     SYSCLK = PLL_VCO / (PLL1C or PLLR)
  */
  pllsource = DDL_RCM_PLL_GetMainSource();

  switch (pllsource)
  {
    case DDL_RCM_PLLSOURCE_HSI:  /* HSI used as PLL clock source */
      pllinputfreq = HSI_VALUE;
      break;

    case DDL_RCM_PLLSOURCE_HSE:  /* HSE used as PLL clock source */
      pllinputfreq = HSE_VALUE;
      break;

    default:
      pllinputfreq = HSI_VALUE;
      break;
  }

  if (SYSCLK_Source == DDL_RCM_SYS_CLKSOURCE_STATUS_PLL)
  {
    plloutputfreq = __DDL_RCM_CALC_PLLCLK_FREQ(pllinputfreq, DDL_RCM_PLL_GetDivider(),
                                        DDL_RCM_PLL_GetN(), DDL_RCM_PLL_GetP());
  }

  return plloutputfreq;
}

/**
  * @brief  Return PLL clock frequency used for 48 MHz domain
  * @retval PLL clock frequency (in Hz)
  */
uint32_t RCM_PLL_GetFreqDomain_48M(void)
{
  uint32_t pllinputfreq = 0U, pllsource = 0U;

  /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLB ) * PLL1A
     48M Domain clock = PLL_VCO / PLLD
  */
  pllsource = DDL_RCM_PLL_GetMainSource();

  switch (pllsource)
  {
    case DDL_RCM_PLLSOURCE_HSI:  /* HSI used as PLL clock source */
      pllinputfreq = HSI_VALUE;
      break;

    case DDL_RCM_PLLSOURCE_HSE:  /* HSE used as PLL clock source */
      pllinputfreq = HSE_VALUE;
      break;

    default:
      pllinputfreq = HSI_VALUE;
      break;
  }
  return __DDL_RCM_CALC_PLLCLK_48M_FREQ(pllinputfreq, DDL_RCM_PLL_GetDivider(),
                                        DDL_RCM_PLL_GetN(), DDL_RCM_PLL_GetQ());
}

#if defined(RCM_PLLI2S_SUPPORT)

/**
  * @brief  Return PLLI2S clock frequency used for I2S domain
  * @retval PLLI2S clock frequency (in Hz)
  */
uint32_t RCM_PLLI2S_GetFreqDomain_I2S(void)
{
  uint32_t plli2sinputfreq = 0U, plli2ssource = 0U, plli2soutputfreq = 0U;

  /* PLLI2S_VCO = (HSE_VALUE or HSI_VALUE / PLL2B) * PLL2A
     I2S Domain clock  = PLLI2S_VCO / PLL2C
  */
  plli2ssource = DDL_RCM_PLLI2S_GetMainSource();

  switch (plli2ssource)
  {
    case DDL_RCM_PLLSOURCE_HSE:     /* HSE used as PLLI2S clock source */
      plli2sinputfreq = HSE_VALUE;
      break;

    case DDL_RCM_PLLSOURCE_HSI:     /* HSI used as PLLI2S clock source */
    default:
      plli2sinputfreq = HSI_VALUE;
      break;
  }

  plli2soutputfreq = __DDL_RCM_CALC_PLLI2S_I2S_FREQ(plli2sinputfreq, DDL_RCM_PLLI2S_GetDivider(),
                                                   DDL_RCM_PLLI2S_GetN(), DDL_RCM_PLLI2S_GetR());

  return plli2soutputfreq;
}

#endif /* RCM_PLLI2S_SUPPORT */
/**
  * @}
  */

/**
  * @}
  */

#endif /* defined(RCM) */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

