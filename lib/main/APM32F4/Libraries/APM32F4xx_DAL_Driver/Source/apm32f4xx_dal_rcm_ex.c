/**
  *
  * @file    apm32f4xx_dal_rcm_ex.c
  * @brief   Extension RCM DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities RCM extension peripheral:
  *           + Extended Peripheral Control functions
  *
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

/** @defgroup RCMEx RCMEx
  * @brief RCMEx DAL module driver
  * @{
  */

#ifdef DAL_RCM_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup RCMEx_Private_Constants
  * @{
  */
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup RCMEx_Exported_Functions RCMEx Exported Functions
  *  @{
  */

/** @defgroup RCMEx_Exported_Functions_Group1 Extended Peripheral Control functions
 *  @brief  Extended Peripheral Control functions
 *
@verbatim
 ===============================================================================
                ##### Extended Peripheral Control functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the RCM Clocks
    frequencies.
    [..]
    (@) Important note: Care must be taken when DAL_RCMEx_PeriphCLKConfig() is used to
        select the RTC clock source; in this case the Backup domain will be reset in
        order to modify the RTC Clock source, as consequence RTC registers (including
        the backup registers) and RCM_BDCTRL register are set to their reset values.

@endverbatim
  * @{
  */

#if defined(APM32F405xx) || defined(APM32F407xx)|| defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)
/**
  * @brief  Initializes the RCM extended peripherals clocks according to the specified parameters in the
  *         RCM_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCM_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals clocks(I2S and RTC clocks).
  *
  * @note   A caution to be taken when DAL_RCMEx_PeriphCLKConfig() is used to select RTC clock selection, in this case
  *         the Reset of Backup domain will be applied in order to modify the RTC Clock source as consequence all backup
  *        domain (RTC and RCM_BDCTRL register expect BKPSRAM) will be reset
  *
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RCMEx_PeriphCLKConfig(RCM_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t tmpreg1 = 0U;

  /* Check the parameters */
  ASSERT_PARAM(IS_RCM_PERIPHCLOCK(PeriphClkInit->PeriphClockSelection));

#if defined(APM32F407xx) || defined(APM32F417xx)
  /*---------------------------- SDRAM configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCM_PERIPHCLK_SDRAM) == RCM_PERIPHCLK_SDRAM)
  {
    /* Check for parameters */
    ASSERT_PARAM(IS_RCM_SDRAM_DIV(PeriphClkInit->SDRAMClockDivision));

    /* Set RCM CFG parameters */
    MODIFY_REG(RCM->CFG, RCM_CFG_SDRAMPSC, \
              (PeriphClkInit->SDRAMClockDivision << RCM_CFG_SDRAMPSC_Pos));
  }
#endif /* APM32F407xx || APM32F417xx */

  /*---------------------------- I2S configuration ---------------------------*/
  if((((PeriphClkInit->PeriphClockSelection) & RCM_PERIPHCLK_I2S) == RCM_PERIPHCLK_I2S) ||
     (((PeriphClkInit->PeriphClockSelection) & RCM_PERIPHCLK_PLLI2S) == RCM_PERIPHCLK_PLLI2S))
  {
    /* check for Parameters */
    ASSERT_PARAM(IS_RCM_PLL2C_VALUE(PeriphClkInit->PLLI2S.PLL2C));
    ASSERT_PARAM(IS_RCM_PLL2A_VALUE(PeriphClkInit->PLLI2S.PLL2A));
#if defined(APM32F411xx)
    ASSERT_PARAM(IS_RCM_PLL2B_VALUE(PeriphClkInit->PLLI2S.PLL2B));
#endif /* APM32F411xx */
    /* Disable the PLLI2S */
    __DAL_RCM_PLLI2S_DISABLE();
    /* Get tick */
    tickstart = DAL_GetTick();
    /* Wait till PLLI2S is disabled */
    while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLL2RDY)  != RESET)
    {
      if((DAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return DAL_TIMEOUT;
      }
    }

#if defined(APM32F411xx)
    /* Configure the PLLI2S division factors */
    /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLL2A/PLL2B) */
    /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLL2C */
    __DAL_RCM_PLLI2S_I2SCLK_CONFIG(PeriphClkInit->PLLI2S.PLL2B, PeriphClkInit->PLLI2S.PLL2A, PeriphClkInit->PLLI2S.PLL2C);
#else
    /* Configure the PLLI2S division factors */
    /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLL2A/PLLB) */
    /* I2SCLK = f(PLLI2S clock output) = f(VCO clock) / PLL2C */
    __DAL_RCM_PLLI2S_CONFIG(PeriphClkInit->PLLI2S.PLL2A , PeriphClkInit->PLLI2S.PLL2C);
#endif /* APM32F411xx */

    /* Enable the PLLI2S */
    __DAL_RCM_PLLI2S_ENABLE();
    /* Get tick */
    tickstart = DAL_GetTick();
    /* Wait till PLLI2S is ready */
    while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLL2RDY)  == RESET)
    {
      if((DAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
      {
        /* return in case of Timeout detected */
        return DAL_TIMEOUT;
      }
    }
  }

  /*---------------------------- RTC configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCM_PERIPHCLK_RTC) == (RCM_PERIPHCLK_RTC))
  {
    /* Check for RTC Parameters used to output RTCCLK */
    ASSERT_PARAM(IS_RCM_RTCCLKSOURCE(PeriphClkInit->RTCClockSelection));

    /* Enable Power Clock*/
    __DAL_RCM_PMU_CLK_ENABLE();

    /* Enable write access to Backup domain */
    PMU->CTRL |= PMU_CTRL_BPWEN;

    /* Get tick */
    tickstart = DAL_GetTick();

    while((PMU->CTRL & PMU_CTRL_BPWEN) == RESET)
    {
      if((DAL_GetTick() - tickstart ) > RCM_DBP_TIMEOUT_VALUE)
      {
        return DAL_TIMEOUT;
      }
    }
    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    tmpreg1 = (RCM->BDCTRL & RCM_BDCTRL_RTCSRCSEL);
    if((tmpreg1 != 0x00000000U) && ((tmpreg1) != (PeriphClkInit->RTCClockSelection & RCM_BDCTRL_RTCSRCSEL)))
    {
      /* Store the content of BDCTRL register before the reset of Backup Domain */
      tmpreg1 = (RCM->BDCTRL & ~(RCM_BDCTRL_RTCSRCSEL));
      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      __DAL_RCM_BACKUPRESET_FORCE();
      __DAL_RCM_BACKUPRESET_RELEASE();
      /* Restore the Content of BDCTRL register */
      RCM->BDCTRL = tmpreg1;

      /* Wait for LSE reactivation if LSE was enable prior to Backup Domain reset */
      if(DAL_IS_BIT_SET(RCM->BDCTRL, RCM_BDCTRL_LSEEN))
      {
        /* Get tick */
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
    }
    __DAL_RCM_RTC_CONFIG(PeriphClkInit->RTCClockSelection);
  }
#if defined(APM32F411xx)
  /*---------------------------- TMR configuration ---------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCM_PERIPHCLK_TMR) == (RCM_PERIPHCLK_TMR))
  {
    __DAL_RCM_TMRCLKPRESCALER(PeriphClkInit->TMRPresSelection);
  }
#endif /* APM32F411xx */
  return DAL_OK;
}

/**
  * @brief  Configures the RCM_OscInitStruct according to the internal
  * RCM configuration registers.
  * @param  PeriphClkInit pointer to an RCM_PeriphCLKInitTypeDef structure that
  * will be configured.
  * @retval None
  */
void DAL_RCMEx_GetPeriphCLKConfig(RCM_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tempreg;

  /* Set all possible values for the extended clock type parameter------------*/
  PeriphClkInit->PeriphClockSelection = RCM_PERIPHCLK_I2S | RCM_PERIPHCLK_RTC;

  /* Get the PLLI2S Clock configuration --------------------------------------*/
  PeriphClkInit->PLLI2S.PLL2A = (uint32_t)((RCM->PLL2CFG & RCM_PLL2CFG_PLL2A) >> RCM_PLL2CFG_PLL2A_Pos);
  PeriphClkInit->PLLI2S.PLL2C = (uint32_t)((RCM->PLL2CFG & RCM_PLL2CFG_PLL2C) >> RCM_PLL2CFG_PLL2C_Pos);
#if defined(APM32F411xx)
  PeriphClkInit->PLLI2S.PLL2B = (uint32_t)(RCM->PLL2CFG & RCM_PLL2CFG_PLL2B);
#endif /* APM32F411xx */
  /* Get the RTC Clock configuration -----------------------------------------*/
  tempreg = (RCM->CFG & RCM_CFG_RTCPSC);
  PeriphClkInit->RTCClockSelection = (uint32_t)((tempreg) | (RCM->BDCTRL & RCM_BDCTRL_RTCSRCSEL));

#if defined(APM32F411xx)
  /* Get the TMR Prescaler configuration -------------------------------------*/
  if ((RCM->CFGSEL & RCM_CFGSEL_CLKPSEL) == RESET)
  {
    PeriphClkInit->TMRPresSelection = RCM_TMRPRES_DESACTIVATED;
  }
  else
  {
    PeriphClkInit->TMRPresSelection = RCM_TMRPRES_ACTIVATED;
  }
#endif /* APM32F411xx */
}

/**
  * @brief  Return the peripheral clock frequency for a given peripheral(SAI..)
  * @note   Return 0 if peripheral clock identifier not managed by this API
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg RCM_PERIPHCLK_I2S: I2S peripheral clock
  * @retval Frequency in KHz
  */
uint32_t DAL_RCMEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  /* This variable used to store the I2S clock frequency (value in Hz) */
  uint32_t frequency = 0U;
  /* This variable used to store the VCO Input (value in Hz) */
  uint32_t vcoinput = 0U;
  uint32_t srcclk = 0U;
  /* This variable used to store the VCO Output (value in Hz) */
  uint32_t vcooutput = 0U;
  switch (PeriphClk)
  {
  case RCM_PERIPHCLK_I2S:
    {
      /* Get the current I2S source */
      srcclk = __DAL_RCM_GET_I2S_SOURCE();
      switch (srcclk)
      {
      /* Check if I2S clock selection is External clock mapped on the I2S_CKIN pin used as I2S clock */
      case RCM_I2SCLKSOURCE_EXT:
        {
          /* Set the I2S clock to the external clock  value */
          frequency = EXTERNAL_CLOCK_VALUE;
          break;
        }
      /* Check if I2S clock selection is PLLI2S VCO output clock divided by PLL2C used as I2S clock */
      case RCM_I2SCLKSOURCE_PLLI2S:
        {
#if defined(APM32F411xx)
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLL2B */
          if((RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS) == RCM_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCM->PLL2CFG & RCM_PLL2CFG_PLL2B));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCM->PLL2CFG & RCM_PLL2CFG_PLL2B));
          }
#else
          /* Configure the PLLI2S division factor */
          /* PLLI2S_VCO Input  = PLL_SOURCE/PLLB */
          if((RCM->PLL1CFG & RCM_PLL1CFG_PLL1CLKS) == RCM_PLLSOURCE_HSE)
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSE_VALUE / (uint32_t)(RCM->PLL1CFG & RCM_PLL1CFG_PLLB));
          }
          else
          {
            /* Get the I2S source clock value */
            vcoinput = (uint32_t)(HSI_VALUE / (uint32_t)(RCM->PLL1CFG & RCM_PLL1CFG_PLLB));
          }
#endif /* APM32F411xx */
          /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLL2A */
          vcooutput = (uint32_t)(vcoinput * (((RCM->PLL2CFG & RCM_PLL2CFG_PLL2A) >> 6U) & (RCM_PLL2CFG_PLL2A >> 6U)));
          /* I2S_CLK = PLLI2S_VCO Output/PLL2C */
          frequency = (uint32_t)(vcooutput /(((RCM->PLL2CFG & RCM_PLL2CFG_PLL2C) >> 28U) & (RCM_PLL2CFG_PLL2C >> 28U)));
          break;
        }
        /* Clock not enabled for I2S*/
      default:
        {
          frequency = 0U;
          break;
        }
      }
      break;
    }
  }
  return frequency;
}
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F411xx */

#if defined(APM32F411xx) && defined(RCM_BDCTRL_LSEMOD)
/**
  * @brief  Select LSE mode
  *
  * @note   This mode is only available for APM32F411xx devices.
  *
  * @param  Mode specifies the LSE mode.
  *          This parameter can be one of the following values:
  *            @arg RCM_LSE_LOWPOWER_MODE:  LSE oscillator in low power mode selection
  *            @arg RCM_LSE_HIGHDRIVE_MODE: LSE oscillator in High Drive mode selection
  * @retval None
  */
void DAL_RCMEx_SelectLSEMode(uint8_t Mode)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_RCM_LSE_MODE(Mode));
  if(Mode == RCM_LSE_HIGHDRIVE_MODE)
  {
    SET_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEMOD);
  }
  else
  {
    CLEAR_BIT(RCM->BDCTRL, RCM_BDCTRL_LSEMOD);
  }
}

#endif /* APM32F411xx && RCM_BDCTRL_LSEMOD */

/** @defgroup RCMEx_Exported_Functions_Group2 Extended Clock management functions
 *  @brief  Extended Clock management functions
 *
@verbatim   
 ===============================================================================
                ##### Extended clock management functions  #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the 
    activation or deactivation of PLLI2S, PLLSAI.
@endverbatim
  * @{
  */

#if defined(RCM_PLLI2S_SUPPORT)
/**
  * @brief  Enable PLLI2S.
  * @param  PLLI2SInit  pointer to an RCM_PLLI2SInitTypeDef structure that
  *         contains the configuration information for the PLLI2S
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RCMEx_EnablePLLI2S(RCM_PLLI2SInitTypeDef  *PLLI2SInit)
{
  uint32_t tickstart;

  /* Check for parameters */
  ASSERT_PARAM(IS_RCM_PLL2A_VALUE(PLLI2SInit->PLL2A));
  ASSERT_PARAM(IS_RCM_PLL2C_VALUE(PLLI2SInit->PLL2C));
#if defined(RCM_PLL2CFG_PLL2B)
  ASSERT_PARAM(IS_RCM_PLL2B_VALUE(PLLI2SInit->PLL2B));
#endif /* RCM_PLL2CFG_PLL2B */
#if defined(RCM_PLL2CFG_PLLI2SP)
  ASSERT_PARAM(IS_RCM_PLLI2SP_VALUE(PLLI2SInit->PLLI2SP));
#endif /* RCM_PLL2CFG_PLLI2SP */
#if defined(RCM_PLL2CFG_PLLI2SQ)
  ASSERT_PARAM(IS_RCM_PLLI2SQ_VALUE(PLLI2SInit->PLLI2SQ));
#endif /* RCM_PLL2CFG_PLLI2SQ */

  /* Disable the PLLI2S */
  __DAL_RCM_PLLI2S_DISABLE();

  /* Wait till PLLI2S is disabled */
  tickstart = DAL_GetTick();
  while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLL2RDY) != RESET)
  {
    if((DAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return DAL_TIMEOUT;
    }
  }

  /* Configure the PLLI2S division factors */
#if defined(APM32F411xx)
  /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) * (PLL2A/PLL2B) */
  /* I2SRCLK = PLLI2S_VCO / PLL2C */
  __DAL_RCM_PLLI2S_I2SCLK_CONFIG(PLLI2SInit->PLL2B, PLLI2SInit->PLL2A, PLLI2SInit->PLL2C);
#else
  /* PLLI2S_VCO = f(VCO clock) = f(PLLI2S clock input) x PLL2A */
  /* I2SRCLK = PLLI2S_VCO / PLL2C */
  __DAL_RCM_PLLI2S_CONFIG(PLLI2SInit->PLL2A, PLLI2SInit->PLL2C);
#endif /* APM32F411xx */

  /* Enable the PLLI2S */
  __DAL_RCM_PLLI2S_ENABLE();

  /* Wait till PLLI2S is ready */
  tickstart = DAL_GetTick();
  while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLL2RDY) == RESET)
  {
    if((DAL_GetTick() - tickstart ) > PLLI2S_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return DAL_TIMEOUT;
    }
  }

 return DAL_OK;
}

/**
  * @brief  Disable PLLI2S.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RCMEx_DisablePLLI2S(void)
{
  uint32_t tickstart;

  /* Disable the PLLI2S */
  __DAL_RCM_PLLI2S_DISABLE();

  /* Wait till PLLI2S is disabled */
  tickstart = DAL_GetTick();
  while(READ_BIT(RCM->CTRL, RCM_CTRL_PLL2RDYFLG) != RESET)
  {
    if((DAL_GetTick() - tickstart) > PLLI2S_TIMEOUT_VALUE)
    {
      /* return in case of Timeout detected */
      return DAL_TIMEOUT;
    }
  }

  return DAL_OK;
}

#endif /* RCM_PLLI2S_SUPPORT */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  Resets the RCM clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *            - HSI ON and used as system clock source
  *            - HSE, PLL, PLLI2S and PLLSAI OFF
  *            - AHB, APB1 and APB2 prescaler set to 1.
  *            - CSS, MCO1 and MCO2 OFF
  *            - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *            - Peripheral clocks
  *            - LSI, LSE and RTC clocks
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_RCM_DeInit(void)
{
  uint32_t tickstart;

  /* Get Start Tick */
  tickstart = DAL_GetTick();

  /* Set HSIEN bit to the reset value */
  SET_BIT(RCM->CTRL, RCM_CTRL_HSIEN);

  /* Wait till HSI is ready */
  while (READ_BIT(RCM->CTRL, RCM_CTRL_HSIRDYFLG) == RESET)
  {
    if ((DAL_GetTick() - tickstart) > HSI_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    }
  }

  /* Set HSITRM[4:0] bits to the reset value */
  SET_BIT(RCM->CTRL, RCM_CTRL_HSITRM_4);

  /* Get Start Tick */
  tickstart = DAL_GetTick();

  /* Reset CFG register */
  CLEAR_REG(RCM->CFG);

  /* Wait till clock switch is ready */
  while (READ_BIT(RCM->CFG, RCM_CFG_SCLKSWSTS) != RESET)
  {
    if ((DAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    }
  }

  /* Get Start Tick */
  tickstart = DAL_GetTick();

  /* Clear HSEEN, HSEBCFG and CSSEN bits */
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_HSEEN | RCM_CTRL_HSEBCFG | RCM_CTRL_CSSEN);

  /* Wait till HSE is disabled */
  while (READ_BIT(RCM->CTRL, RCM_CTRL_HSERDYFLG) != RESET)
  {
    if ((DAL_GetTick() - tickstart) > HSE_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    }
  }

  /* Get Start Tick */
  tickstart = DAL_GetTick();

  /* Clear PLL1EN bit */
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_PLL1EN);

  /* Wait till PLL is disabled */
  while (READ_BIT(RCM->CTRL, RCM_CTRL_PLL1RDYFLG) != RESET)
  {
    if ((DAL_GetTick() - tickstart) > PLL_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    }
  }

#if defined(RCM_PLLI2S_SUPPORT)
  /* Get Start Tick */
  tickstart = DAL_GetTick();

  /* Reset PLL2EN bit */
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_PLL2EN);

  /* Wait till PLLI2S is disabled */
  while (READ_BIT(RCM->CTRL, RCM_CTRL_PLL2RDYFLG) != RESET)
  {
    if ((DAL_GetTick() - tickstart) > PLLI2S_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    }
  }
#endif /* RCM_PLLI2S_SUPPORT */

#if defined(RCM_PLLSAI_SUPPORT)
  /* Get Start Tick */
  tickstart = DAL_GetTick();

  /* Reset PLLSAI bit */
  CLEAR_BIT(RCM->CTRL, RCM_CTRL_PLLSAION);

  /* Wait till PLLSAI is disabled */
  while (READ_BIT(RCM->CTRL, RCM_CTRL_PLLSAIRDY) != RESET)
  {
    if ((DAL_GetTick() - tickstart) > PLLSAI_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    }
  }
#endif /* RCM_PLLSAI_SUPPORT */

  /* Once PLL, PLLI2S and PLLSAI are OFF, reset PLLCFGR register to default value */
  RCM->PLL1CFG = RCM_PLL1CFG_PLLB_4 | RCM_PLL1CFG_PLL1A_6 | RCM_PLL1CFG_PLL1A_7 | RCM_PLL1CFG_PLLD_2;

  /* Reset PLLI2SCFGR register to default value */
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
  RCM->PLL2CFG = RCM_PLL2CFG_PLL2A_6 | RCM_PLL2CFG_PLL2A_7 | RCM_PLL2CFG_PLL2C_1;
#elif defined(APM32F411xx)
  RCM->PLL2CFG = RCM_PLL2CFG_PLL2B_4 | RCM_PLL2CFG_PLL2A_6 | RCM_PLL2CFG_PLL2A_7 | RCM_PLL2CFG_PLL2C_1;
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */

  /* Disable all interrupts */
  CLEAR_BIT(RCM->INT, RCM_INT_LSIRDYEN | RCM_INT_LSERDYEN | RCM_INT_HSIRDYEN | RCM_INT_HSERDYEN | RCM_INT_PLL1RDYEN);

#if defined(RCM_INT_PLL2RDYEN)
  CLEAR_BIT(RCM->INT, RCM_INT_PLL2RDYEN);
#endif /* RCM_INT_PLL2RDYEN */

#if defined(RCM_INT_PLLSAIRDYIE)
  CLEAR_BIT(RCM->INT, RCM_INT_PLLSAIRDYIE);
#endif /* RCM_INT_PLLSAIRDYIE */

  /* Clear all interrupt flags */
  SET_BIT(RCM->INT, RCM_INT_LSIRDYCLR | RCM_INT_LSERDYCLR | RCM_INT_HSIRDYCLR | RCM_INT_HSERDYCLR | RCM_INT_PLL1RDYCLR | RCM_INT_CSSCLR);

#if defined(RCM_INT_PLL2RDYCLR)
  SET_BIT(RCM->INT, RCM_INT_PLL2RDYCLR);
#endif /* RCM_INT_PLL2RDYCLR */

#if defined(RCM_INT_PLLSAIRDYC)
  SET_BIT(RCM->INT, RCM_INT_PLLSAIRDYC);
#endif /* RCM_INT_PLLSAIRDYC */

  /* Clear LSIEN bit */
  CLEAR_BIT(RCM->CSTS, RCM_CSTS_LSIEN);

  /* Reset all CSTS flags */
  SET_BIT(RCM->CSTS, RCM_CSTS_RSTFLGCLR);

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = HSI_VALUE;

  /* Adapt Systick interrupt period */
  if(DAL_InitTick(uwTickPrio) != DAL_OK)
  {
    return DAL_ERROR;
  }
  else
  {
    return DAL_OK;
  }
}

#endif /* DAL_RCM_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

