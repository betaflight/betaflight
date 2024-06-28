/**
  *
  * @file    apm32f4xx_dal_crc.c
  * @brief   CRC DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Cyclic Redundancy Check (CRC) peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
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
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
         (+) Enable CRC AHB clock using __DAL_RCM_CRC_CLK_ENABLE();
         (+) Initialize CRC calculator
             (++) specify generating polynomial (peripheral default or non-default one)
             (++) specify initialization value (peripheral default or non-default one)
             (++) specify input data format
             (++) specify input or output data inversion mode if any
         (+) Use DAL_CRC_Accumulate() function to compute the CRC value of the
             input data buffer starting with the previously computed CRC as
             initialization value
         (+) Use DAL_CRC_Calculate() function to compute the CRC value of the
             input data buffer starting with the defined initialization value
             (default or non-default) to initiate CRC calculation

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup CRC CRC
  * @brief CRC DAL module driver.
  * @{
  */

#ifdef DAL_CRC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/** @defgroup CRC_Exported_Functions CRC Exported Functions
  * @{
  */

/** @defgroup CRC_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions.
  *
@verbatim
 ===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the CRC according to the specified parameters
          in the CRC_InitTypeDef and create the associated handle
      (+) DeInitialize the CRC peripheral
      (+) Initialize the CRC MSP (MCU Specific Package)
      (+) DeInitialize the CRC MSP

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the CRC according to the specified
  *         parameters in the CRC_InitTypeDef and create the associated handle.
  * @param  hcrc CRC handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CRC_Init(CRC_HandleTypeDef *hcrc)
{
  /* Check the CRC handle allocation */
  if (hcrc == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_CRC_ALL_INSTANCE(hcrc->Instance));

  if (hcrc->State == DAL_CRC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcrc->Lock = DAL_UNLOCKED;
    /* Init the low level hardware */
    DAL_CRC_MspInit(hcrc);
  }

  /* Change CRC peripheral state */
  hcrc->State = DAL_CRC_STATE_READY;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  DeInitialize the CRC peripheral.
  * @param  hcrc CRC handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CRC_DeInit(CRC_HandleTypeDef *hcrc)
{
  /* Check the CRC handle allocation */
  if (hcrc == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_CRC_ALL_INSTANCE(hcrc->Instance));

  /* Check the CRC peripheral state */
  if (hcrc->State == DAL_CRC_STATE_BUSY)
  {
    return DAL_BUSY;
  }

  /* Change CRC peripheral state */
  hcrc->State = DAL_CRC_STATE_BUSY;

  /* Reset CRC calculation unit */
  __DAL_CRC_DATA_RESET(hcrc);

  /* Reset IDR register content */
  CLEAR_BIT(hcrc->Instance->INDATA, CRC_INDATA_INDATA);

  /* DeInit the low level hardware */
  DAL_CRC_MspDeInit(hcrc);

  /* Change CRC peripheral state */
  hcrc->State = DAL_CRC_STATE_RESET;

  /* Process unlocked */
  __DAL_UNLOCK(hcrc);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Initializes the CRC MSP.
  * @param  hcrc CRC handle
  * @retval None
  */
__weak void DAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcrc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_CRC_MspInit can be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the CRC MSP.
  * @param  hcrc CRC handle
  * @retval None
  */
__weak void DAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcrc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_CRC_MspDeInit can be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group2 Peripheral Control functions
  *  @brief    management functions.
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) compute the 32-bit CRC value of a 32-bit data buffer
          using combination of the previous CRC value and the new one.

       [..]  or

      (+) compute the 32-bit CRC value of a 32-bit data buffer
          independently of the previous CRC value.

@endverbatim
  * @{
  */

/**
  * @brief  Compute the 32-bit CRC value of a 32-bit data buffer
  *         starting with the previously computed CRC as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer.
  * @param  BufferLength input data buffer length (number of uint32_t words).
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t DAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength)
{
  uint32_t index;      /* CRC input data buffer index */
  uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DATA register) */

  /* Change CRC peripheral state */
  hcrc->State = DAL_CRC_STATE_BUSY;

  /* Enter Data to the CRC calculator */
  for (index = 0U; index < BufferLength; index++)
  {
    hcrc->Instance->DATA = pBuffer[index];
  }
  temp = hcrc->Instance->DATA;

  /* Change CRC peripheral state */
  hcrc->State = DAL_CRC_STATE_READY;

  /* Return the CRC computed value */
  return temp;
}

/**
  * @brief  Compute the 32-bit CRC value of a 32-bit data buffer
  *         starting with hcrc->Instance->INIT as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer.
  * @param  BufferLength input data buffer length (number of uint32_t words).
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t DAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength)
{
  uint32_t index;      /* CRC input data buffer index */
  uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DATA register) */

  /* Change CRC peripheral state */
  hcrc->State = DAL_CRC_STATE_BUSY;

  /* Reset CRC Calculation Unit (hcrc->Instance->INIT is
  *  written in hcrc->Instance->DATA) */
  __DAL_CRC_DATA_RESET(hcrc);

  /* Enter 32-bit input data to the CRC calculator */
  for (index = 0U; index < BufferLength; index++)
  {
    hcrc->Instance->DATA = pBuffer[index];
  }
  temp = hcrc->Instance->DATA;

  /* Change CRC peripheral state */
  hcrc->State = DAL_CRC_STATE_READY;

  /* Return the CRC computed value */
  return temp;
}

/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group3 Peripheral State functions
  *  @brief    Peripheral State functions.
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Return the CRC handle state.
  * @param  hcrc CRC handle
  * @retval DAL state
  */
DAL_CRC_StateTypeDef DAL_CRC_GetState(CRC_HandleTypeDef *hcrc)
{
  /* Return CRC handle state */
  return hcrc->State;
}

/**
  * @}
  */

/**
  * @}
  */


#endif /* DAL_CRC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
