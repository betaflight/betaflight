/**
  *
  * @file    apm32f4xx_dal_pmu_ex.c
  * @brief   Extended PMU DAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of PMU extension peripheral:           
  *           + Peripheral Extended features functions
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

/** @defgroup PMUEx PMUEx
  * @brief PMU DAL module driver
  * @{
  */

#ifdef DAL_PMU_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup PMUEx_Private_Constants
  * @{
  */    
#define PMU_OVERDRIVE_TIMEOUT_VALUE  1000U
#define PMU_UDERDRIVE_TIMEOUT_VALUE  1000U
#define PMU_BKPREG_TIMEOUT_VALUE     1000U
#define PMU_VOSRDY_TIMEOUT_VALUE     1000U
/**
  * @}
  */

   
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup PMUEx_Exported_Functions PMUEx Exported Functions
  *  @{
  */

/** @defgroup PMUEx_Exported_Functions_Group1 Peripheral Extended features functions 
  *  @brief Peripheral Extended features functions 
  *
@verbatim   

 ===============================================================================
                 ##### Peripheral extended features functions #####
 ===============================================================================

    *** Main and Backup Regulators configuration ***
    ================================================
    [..] 
      (+) The backup domain includes 4 Kbytes of backup SRAM accessible only from 
          the CPU, and address in 32-bit, 16-bit or 8-bit mode. Its content is 
          retained even in Standby or VBAT mode when the low power backup regulator
          is enabled. It can be considered as an internal EEPROM when VBAT is 
          always present. You can use the DAL_PMUEx_EnableBkUpReg() function to 
          enable the low power backup regulator. 

      (+) When the backup domain is supplied by VDD (analog switch connected to VDD) 
          the backup SRAM is powered from VDD which replaces the VBAT power supply to 
          save battery life.

      (+) The backup SRAM is not mass erased by a tamper event. It is read 
          protected to prevent confidential data, such as cryptographic private 
          key, from being accessed. The backup SRAM can be erased only through 
          the Flash interface when a protection level change from level 1 to 
          level 0 is requested. 
      -@- Refer to the description of Read protection (RDP) in the Flash 
          programming manual.

      (+) The main internal regulator can be configured to have a tradeoff between 
          performance and power consumption when the device does not operate at 
          the maximum frequency. This is done through __DAL_PMU_MAINREGULATORMODE_CONFIG() 
          macro which configure VOS bit in PMU_CTRL register
          
        Refer to the product datasheets for more details.

    *** FLASH Power Down configuration ****
    =======================================
    [..] 
      (+) By setting the FPDS bit in the PMU_CTRL register by using the 
          DAL_PMUEx_EnableFlashPowerDown() function, the Flash memory also enters power 
          down mode when the device enters Stop mode. When the Flash memory 
          is in power down mode, an additional startup delay is incurred when 
          waking up from Stop mode.
          
        Refer to the datasheets for more details.

@endverbatim
  * @{
  */

/**
  * @brief Enables the Backup Regulator.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PMUEx_EnableBkUpReg(void)
{
  uint32_t tickstart = 0U;

  *(__IO uint32_t *) CSTS_BKPREN_BB = (uint32_t)ENABLE;

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait till Backup regulator ready flag is set */  
  while(__DAL_PMU_GET_FLAG(PMU_FLAG_BRR) == RESET)
  {
    if((DAL_GetTick() - tickstart ) > PMU_BKPREG_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    } 
  }
  return DAL_OK;
}

/**
  * @brief Disables the Backup Regulator.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PMUEx_DisableBkUpReg(void)
{
  uint32_t tickstart = 0U;

  *(__IO uint32_t *) CSTS_BKPREN_BB = (uint32_t)DISABLE;

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait till Backup regulator ready flag is set */  
  while(__DAL_PMU_GET_FLAG(PMU_FLAG_BRR) != RESET)
  {
    if((DAL_GetTick() - tickstart ) > PMU_BKPREG_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    } 
  }
  return DAL_OK;
}

/**
  * @brief Enables the Flash Power Down in Stop mode.
  * @retval None
  */
void DAL_PMUEx_EnableFlashPowerDown(void)
{
  *(__IO uint32_t *) CTRL_FPDSM_BB = (uint32_t)ENABLE;
}

/**
  * @brief Disables the Flash Power Down in Stop mode.
  * @retval None
  */
void DAL_PMUEx_DisableFlashPowerDown(void)
{
  *(__IO uint32_t *) CTRL_FPDSM_BB = (uint32_t)DISABLE;
}

/**
  * @brief Return Voltage Scaling Range.
  * @retval The configured scale for the regulator voltage(VOS bit field).
  *         The returned value can be one of the following:
  *            - @arg PMU_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output Scale 1 mode
  *            - @arg PMU_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output Scale 2 mode
  *            - @arg PMU_REGULATOR_VOLTAGE_SCALE3: Regulator voltage output Scale 3 mode
  */  
uint32_t DAL_PMUEx_GetVoltageRange(void)
{
  return (PMU->CTRL & PMU_CTRL_VOSSEL);
}

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
/**
  * @brief Configures the main internal regulator output voltage.
  * @param  VoltageScaling specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption.
  *          This parameter can be one of the following values:
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output range 1 mode,
  *                                               the maximum value of fHCLK = 168 MHz.
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output range 2 mode,
  *                                               the maximum value of fHCLK = 144 MHz.
  * @note  When moving from Range 1 to Range 2, the system frequency must be decreased to
  *        a value below 144 MHz before calling DAL_PMUEx_ConfigVoltageScaling() API.
  *        When moving from Range 2 to Range 1, the system frequency can be increased to
  *        a value up to 168 MHz after calling DAL_PMUEx_ConfigVoltageScaling() API.
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_PMUEx_ControlVoltageScaling(uint32_t VoltageScaling)
{
  uint32_t tickstart = 0U;
  
  ASSERT_PARAM(IS_PMU_VOLTAGE_SCALING_RANGE(VoltageScaling));
  
  /* Enable PMU RCM Clock Peripheral */
  __DAL_RCM_PMU_CLK_ENABLE();
  
  /* Set Range */
  __DAL_PMU_VOLTAGESCALING_CONFIG(VoltageScaling);
  
  /* Get Start Tick*/
  tickstart = DAL_GetTick();
  while((__DAL_PMU_GET_FLAG(PMU_FLAG_VOSRDY) == RESET))
  {
    if((DAL_GetTick() - tickstart ) > PMU_VOSRDY_TIMEOUT_VALUE)
    {
      return DAL_TIMEOUT;
    } 
  }

  return DAL_OK;
}

#elif defined(APM32F411xx)
/**
  * @brief Configures the main internal regulator output voltage.
  * @param  VoltageScaling specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption.
  *          This parameter can be one of the following values:
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output range 1 mode,
  *                                               the maximum value of fHCLK is 168 MHz. It can be extended to
  *                                               180 MHz by activating the over-drive mode.
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output range 2 mode,
  *                                               the maximum value of fHCLK is 144 MHz. It can be extended to,                
  *                                               168 MHz by activating the over-drive mode.
  *            @arg PMU_REGULATOR_VOLTAGE_SCALE3: Regulator voltage output range 3 mode,
  *                                               the maximum value of fHCLK is 120 MHz.
  * @note To update the system clock frequency(SYSCLK):
  *        - Set the HSI or HSE as system clock frequency using the DAL_RCM_ClockConfig().
  *        - Call the DAL_RCM_OscConfig() to configure the PLL.
  *        - Call DAL_PMUEx_ConfigVoltageScaling() API to adjust the voltage scale.
  *        - Set the new system clock frequency using the DAL_RCM_ClockConfig().
  * @note The scale can be modified only when the HSI or HSE clock source is selected 
  *        as system clock source, otherwise the API returns DAL_ERROR.  
  * @note When the PLL is OFF, the voltage scale 3 is automatically selected and the VOS bits
  *       value in the PMU_CTRL1 register are not taken in account.
  * @note This API forces the PLL state ON to allow the possibility to configure the voltage scale 1 or 2.
  * @note The new voltage scale is active only when the PLL is ON.  
  * @retval DAL Status
  */
DAL_StatusTypeDef DAL_PMUEx_ControlVoltageScaling(uint32_t VoltageScaling)
{
  uint32_t tickstart = 0U;
  
  ASSERT_PARAM(IS_PMU_VOLTAGE_SCALING_RANGE(VoltageScaling));
  
  /* Enable PMU RCM Clock Peripheral */
  __DAL_RCM_PMU_CLK_ENABLE();
  
  /* Check if the PLL is used as system clock or not */
  if(__DAL_RCM_GET_SYSCLK_SOURCE() != RCM_CFG_SCLKSWSTS_PLL)
  {
    /* Disable the main PLL */
    __DAL_RCM_PLL_DISABLE();
    
    /* Get Start Tick */
    tickstart = DAL_GetTick();    
    /* Wait till PLL is disabled */  
    while(__DAL_RCM_GET_FLAG(RCM_FLAG_PLLRDY) != RESET)
    {
      if((DAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
      {
        return DAL_TIMEOUT;
      }
    }
    
    /* Set Range */
    __DAL_PMU_VOLTAGESCALING_CONFIG(VoltageScaling);
    
    /* Enable the main PLL */
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
    
    /* Get Start Tick */
    tickstart = DAL_GetTick();
    while((__DAL_PMU_GET_FLAG(PMU_FLAG_VOSRDY) == RESET))
    {
      if((DAL_GetTick() - tickstart ) > PMU_VOSRDY_TIMEOUT_VALUE)
      {
        return DAL_TIMEOUT;
      } 
    }
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */

#if defined(APM32F411xx)
/**
  * @brief Enables Main Regulator low voltage mode.
  * @note  This mode is only available for APM32F411xx devices.
  * @retval None
  */
void DAL_PMUEx_EnableMainRegulatorLowVoltage(void)
{
  *(__IO uint32_t *) CTRL_MRLV_BB = (uint32_t)ENABLE;
}

/**
  * @brief Disables Main Regulator low voltage mode.
  * @note  This mode is only available for APM32F411xx devices.
  * @retval None
  */
void DAL_PMUEx_DisableMainRegulatorLowVoltage(void)
{
  *(__IO uint32_t *) CTRL_MRLV_BB = (uint32_t)DISABLE;
}

/**
  * @brief Enables Low Power Regulator low voltage mode.
  * @note  This mode is only available for APM32F411xx devices.
  * @retval None
  */
void DAL_PMUEx_EnableLowRegulatorLowVoltage(void)
{
  *(__IO uint32_t *) CTRL_LPRLV_BB = (uint32_t)ENABLE;
}

/**
  * @brief Disables Low Power Regulator low voltage mode.
  * @note  This mode is only available for APM32F411xx devices.
  * @retval None
  */
void DAL_PMUEx_DisableLowRegulatorLowVoltage(void)
{
  *(__IO uint32_t *) CTRL_LPRLV_BB = (uint32_t)DISABLE;
}

#endif /* APM32F411xx */

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_PMU_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
