/**
  *
  * @file    apm32f4xx_dal_crc.h
  * @brief   Header file of CRC DAL module.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_CRC_H
#define APM32F4xx_DAL_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup CRC
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup CRC_Exported_Types CRC Exported Types
  * @{
  */

/**
  * @brief  CRC DAL State Structure definition
  */
typedef enum
{
  DAL_CRC_STATE_RESET     = 0x00U,  /*!< CRC not yet initialized or disabled */
  DAL_CRC_STATE_READY     = 0x01U,  /*!< CRC initialized and ready for use   */
  DAL_CRC_STATE_BUSY      = 0x02U,  /*!< CRC internal process is ongoing     */
  DAL_CRC_STATE_TIMEOUT   = 0x03U,  /*!< CRC timeout state                   */
  DAL_CRC_STATE_ERROR     = 0x04U   /*!< CRC error state                     */
} DAL_CRC_StateTypeDef;


/**
  * @brief  CRC Handle Structure definition
  */
typedef struct
{
  CRC_TypeDef                 *Instance;   /*!< Register base address        */

  DAL_LockTypeDef             Lock;        /*!< CRC Locking object           */

  __IO DAL_CRC_StateTypeDef   State;       /*!< CRC communication state      */

} CRC_HandleTypeDef;
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CRC_Exported_Constants CRC Exported Constants
  * @{
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup CRC_Exported_Macros CRC Exported Macros
  * @{
  */

/** @brief Reset CRC handle state.
  * @param  __HANDLE__ CRC handle.
  * @retval None
  */
#define __DAL_CRC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_CRC_STATE_RESET)

/**
  * @brief  Reset CRC Data Register.
  * @param  __HANDLE__ CRC handle
  * @retval None
  */
#define __DAL_CRC_DATA_RESET(__HANDLE__) ((__HANDLE__)->Instance->CTRL |= CRC_CTRL_RST)

/**
  * @brief Store data in the Independent Data (ID) register.
  * @param __HANDLE__ CRC handle
  * @param __VALUE__  Value to be stored in the ID register
  * @note  Refer to the Reference Manual to get the authorized __VALUE__ length in bits
  * @retval None
  */
#define __DAL_CRC_SET_IDR(__HANDLE__, __VALUE__) (WRITE_REG((__HANDLE__)->Instance->INDATA, (__VALUE__)))

/**
  * @brief Return the data stored in the Independent Data (ID) register.
  * @param __HANDLE__ CRC handle
  * @note  Refer to the Reference Manual to get the authorized __VALUE__ length in bits
  * @retval Value of the ID register
  */
#define __DAL_CRC_GET_IDR(__HANDLE__) (((__HANDLE__)->Instance->INDATA) & CRC_INDATA_INDATA)
/**
  * @}
  */


/* Private macros --------------------------------------------------------*/
/** @defgroup  CRC_Private_Macros CRC Private Macros
  * @{
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup CRC_Exported_Functions CRC Exported Functions
  * @{
  */

/* Initialization and de-initialization functions  ****************************/
/** @defgroup CRC_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
DAL_StatusTypeDef DAL_CRC_Init(CRC_HandleTypeDef *hcrc);
DAL_StatusTypeDef DAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);
void DAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void DAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);
/**
  * @}
  */

/* Peripheral Control functions ***********************************************/
/** @defgroup CRC_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */
uint32_t DAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t DAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
/**
  * @}
  */

/* Peripheral State and Error functions ***************************************/
/** @defgroup CRC_Exported_Functions_Group3 Peripheral State functions
  * @{
  */
DAL_CRC_StateTypeDef DAL_CRC_GetState(CRC_HandleTypeDef *hcrc);
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
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_CRC_H */
