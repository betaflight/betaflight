/**
  *
  * @file    apm32f4xx_dal_i2c_ex.c
  * @brief   I2C Extension DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of I2C extension peripheral:
  *           + Extension features functions
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
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  @verbatim
  ==============================================================================
               ##### I2C peripheral extension features  #####
  ==============================================================================

  [..] Comparing to other previous devices, the I2C interface for APM32F427xx/437xx/
       429xx/439xx devices contains the following additional features :

       (+) Possibility to disable or enable Analog Noise Filter
       (+) Use of a configured Digital Noise Filter

                     ##### How to use this driver #####
  ==============================================================================
  [..] This driver provides functions to configure Noise Filter
    (#) Configure I2C Analog noise filter using the function DAL_I2C_AnalogFilter_Config()
    (#) Configure I2C Digital noise filter using the function DAL_I2C_DigitalFilter_Config()

  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup I2CEx I2CEx
  * @brief I2C DAL module driver
  * @{
  */

#ifdef DAL_I2C_MODULE_ENABLED

#if defined(I2C_FILTER_ANFDIS) && defined(I2C_FILTER_DNFCFG)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup I2CEx_Exported_Functions I2C Exported Functions
  * @{
  */


/** @defgroup I2CEx_Exported_Functions_Group1 Extension features functions
 *  @brief   Extension features functions
 *
@verbatim
 ===============================================================================
                      ##### Extension features functions #####
 ===============================================================================
    [..] This section provides functions allowing to:
      (+) Configure Noise Filters

@endverbatim
  * @{
  */

/**
  * @brief  Configures I2C Analog noise filter.
  * @param  hi2c pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2Cx peripheral.
  * @param  AnalogFilter new state of the Analog filter.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  ASSERT_PARAM(IS_I2C_ANALOG_FILTER(AnalogFilter));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    hi2c->State = DAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    __DAL_I2C_DISABLE(hi2c);

    /* Reset I2Cx ANFDIS bit */
    hi2c->Instance->FILTER &= ~(I2C_FILTER_ANFDIS);

    /* Disable the analog filter */
    hi2c->Instance->FILTER |= AnalogFilter;

    __DAL_I2C_ENABLE(hi2c);

    hi2c->State = DAL_I2C_STATE_READY;

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Configures I2C Digital noise filter.
  * @param  hi2c pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2Cx peripheral.
  * @param  DigitalFilter Coefficient of digital noise filter between 0x00 and 0x0F.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter)
{
  uint16_t tmpreg = 0;

  /* Check the parameters */
  ASSERT_PARAM(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  ASSERT_PARAM(IS_I2C_DIGITAL_FILTER(DigitalFilter));

  if (hi2c->State == DAL_I2C_STATE_READY)
  {
    hi2c->State = DAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    __DAL_I2C_DISABLE(hi2c);

    /* Get the old register value */
    tmpreg = hi2c->Instance->FILTER;

    /* Reset I2Cx DNFCFG bit [3:0] */
    tmpreg &= ~(I2C_FILTER_DNFCFG);

    /* Set I2Cx DNFCFG coefficient */
    tmpreg |= DigitalFilter;

    /* Store the new register value */
    hi2c->Instance->FILTER = tmpreg;

    __DAL_I2C_ENABLE(hi2c);

    hi2c->State = DAL_I2C_STATE_READY;

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @}
  */

/**
  * @}
  */
#endif /* I2C_FILTER_ANFDIS && I2C_FILTER_DNFCFG */

#endif /* DAL_I2C_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

