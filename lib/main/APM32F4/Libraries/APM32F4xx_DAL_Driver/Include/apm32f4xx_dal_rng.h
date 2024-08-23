/**
  *
  * @file    apm32f4xx_dal_rng.h
  * @brief   Header file of RNG DAL module.
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
#ifndef APM32F4xx_DAL_RNG_H
#define APM32F4xx_DAL_RNG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#if defined (RNG)

/** @defgroup RNG RNG
  * @brief RNG DAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup RNG_Exported_Types RNG Exported Types
  * @{
  */

/** @defgroup RNG_Exported_Types_Group1 RNG Init Structure definition
  * @{
  */

/**
  * @}
  */

/** @defgroup RNG_Exported_Types_Group2 RNG State Structure definition
  * @{
  */
typedef enum
{
  DAL_RNG_STATE_RESET     = 0x00U,  /*!< RNG not yet initialized or disabled */
  DAL_RNG_STATE_READY     = 0x01U,  /*!< RNG initialized and ready for use   */
  DAL_RNG_STATE_BUSY      = 0x02U,  /*!< RNG internal process is ongoing     */
  DAL_RNG_STATE_TIMEOUT   = 0x03U,  /*!< RNG timeout state                   */
  DAL_RNG_STATE_ERROR     = 0x04U   /*!< RNG error state                     */

} DAL_RNG_StateTypeDef;

/**
  * @}
  */

/** @defgroup RNG_Exported_Types_Group3 RNG Handle Structure definition
  * @{
  */
#if (USE_DAL_RNG_REGISTER_CALLBACKS == 1)
typedef struct  __RNG_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_RNG_REGISTER_CALLBACKS */
{
  RNG_TypeDef                 *Instance;    /*!< Register base address   */

  DAL_LockTypeDef             Lock;         /*!< RNG locking object      */

  __IO DAL_RNG_StateTypeDef   State;        /*!< RNG communication state */

  __IO  uint32_t              ErrorCode;    /*!< RNG Error code          */

  uint32_t                    RandomNumber; /*!< Last Generated RNG Data */

#if (USE_DAL_RNG_REGISTER_CALLBACKS == 1)
  void (* ReadyDataCallback)(struct __RNG_HandleTypeDef *hrng, uint32_t random32bit);  /*!< RNG Data Ready Callback    */
  void (* ErrorCallback)(struct __RNG_HandleTypeDef *hrng);                            /*!< RNG Error Callback         */

  void (* MspInitCallback)(struct __RNG_HandleTypeDef *hrng);                          /*!< RNG Msp Init callback      */
  void (* MspDeInitCallback)(struct __RNG_HandleTypeDef *hrng);                        /*!< RNG Msp DeInit callback    */
#endif  /* USE_DAL_RNG_REGISTER_CALLBACKS */

} RNG_HandleTypeDef;

#if (USE_DAL_RNG_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL RNG Callback ID enumeration definition
  */
typedef enum
{
  DAL_RNG_ERROR_CB_ID                   = 0x00U,     /*!< RNG Error Callback ID          */

  DAL_RNG_MSPINIT_CB_ID                 = 0x01U,     /*!< RNG MspInit callback ID        */
  DAL_RNG_MSPDEINIT_CB_ID               = 0x02U      /*!< RNG MspDeInit callback ID      */

} DAL_RNG_CallbackIDTypeDef;

/**
  * @brief  DAL RNG Callback pointer definition
  */
typedef  void (*pRNG_CallbackTypeDef)(RNG_HandleTypeDef *hrng);                                  /*!< pointer to a common RNG callback function */
typedef  void (*pRNG_ReadyDataCallbackTypeDef)(RNG_HandleTypeDef *hrng, uint32_t random32bit);   /*!< pointer to an RNG Data Ready specific callback function */

#endif /* USE_DAL_RNG_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RNG_Exported_Constants RNG Exported Constants
  * @{
  */

/** @defgroup RNG_Exported_Constants_Group1 RNG Interrupt definition
  * @{
  */
#define RNG_IT_DRDY  RNG_STS_DATARDY  /*!< Data Ready interrupt  */
#define RNG_IT_CEI   RNG_STS_CLKERINT  /*!< Clock error interrupt */
#define RNG_IT_SEI   RNG_STS_FSINT  /*!< Seed error interrupt  */
/**
  * @}
  */

/** @defgroup RNG_Exported_Constants_Group2 RNG Flag definition
  * @{
  */
#define RNG_FLAG_DRDY   RNG_STS_DATARDY  /*!< Data ready                 */
#define RNG_FLAG_CECS   RNG_STS_CLKERCSTS  /*!< Clock error current status */
#define RNG_FLAG_SECS   RNG_STS_FSCSTS  /*!< Seed error current status  */
/**
  * @}
  */

/** @defgroup RNG_Error_Definition   RNG Error Definition
  * @{
  */
#define  DAL_RNG_ERROR_NONE             0x00000000U    /*!< No error          */
#if (USE_DAL_RNG_REGISTER_CALLBACKS == 1)
#define  DAL_RNG_ERROR_INVALID_CALLBACK 0x00000001U    /*!< Invalid Callback error  */
#endif /* USE_DAL_RNG_REGISTER_CALLBACKS */
#define  DAL_RNG_ERROR_TIMEOUT          0x00000002U    /*!< Timeout error     */
#define  DAL_RNG_ERROR_BUSY             0x00000004U    /*!< Busy error        */
#define  DAL_RNG_ERROR_SEED             0x00000008U    /*!< Seed error        */
#define  DAL_RNG_ERROR_CLOCK            0x00000010U    /*!< Clock error       */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup RNG_Exported_Macros RNG Exported Macros
  * @{
  */

/** @brief Reset RNG handle state
  * @param  __HANDLE__ RNG Handle
  * @retval None
  */
#if (USE_DAL_RNG_REGISTER_CALLBACKS == 1)
#define __DAL_RNG_RESET_HANDLE_STATE(__HANDLE__)  do{                                                   \
                                                       (__HANDLE__)->State = DAL_RNG_STATE_RESET;       \
                                                       (__HANDLE__)->MspInitCallback = NULL;            \
                                                       (__HANDLE__)->MspDeInitCallback = NULL;          \
                                                    } while(0U)
#else
#define __DAL_RNG_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_RNG_STATE_RESET)
#endif /* USE_DAL_RNG_REGISTER_CALLBACKS */

/**
  * @brief  Enables the RNG peripheral.
  * @param  __HANDLE__ RNG Handle
  * @retval None
  */
#define __DAL_RNG_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL |=  RNG_CTRL_RNGEN)

/**
  * @brief  Disables the RNG peripheral.
  * @param  __HANDLE__ RNG Handle
  * @retval None
  */
#define __DAL_RNG_DISABLE(__HANDLE__) ((__HANDLE__)->Instance->CTRL &= ~RNG_CTRL_RNGEN)

/**
  * @brief  Check the selected RNG flag status.
  * @param  __HANDLE__ RNG Handle
  * @param  __FLAG__ RNG flag
  *          This parameter can be one of the following values:
  *            @arg RNG_FLAG_DRDY:  Data ready
  *            @arg RNG_FLAG_CECS:  Clock error current status
  *            @arg RNG_FLAG_SECS:  Seed error current status
  * @retval The new state of __FLAG__ (SET or RESET).
  */
#define __DAL_RNG_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->STS & (__FLAG__)) == (__FLAG__))

/**
  * @brief  Clears the selected RNG flag status.
  * @param  __HANDLE__ RNG handle
  * @param  __FLAG__ RNG flag to clear
  * @note   WARNING: This is a dummy macro for DAL code alignment,
  *         flags RNG_FLAG_DRDY, RNG_FLAG_CECS and RNG_FLAG_SECS are read-only.
  * @retval None
  */
#define __DAL_RNG_CLEAR_FLAG(__HANDLE__, __FLAG__)                      /* dummy  macro */

/**
  * @brief  Enables the RNG interrupts.
  * @param  __HANDLE__ RNG Handle
  * @retval None
  */
#define __DAL_RNG_ENABLE_IT(__HANDLE__) ((__HANDLE__)->Instance->CTRL |=  RNG_CTRL_INTEN)

/**
  * @brief  Disables the RNG interrupts.
  * @param  __HANDLE__ RNG Handle
  * @retval None
  */
#define __DAL_RNG_DISABLE_IT(__HANDLE__) ((__HANDLE__)->Instance->CTRL &= ~RNG_CTRL_INTEN)

/**
  * @brief  Checks whether the specified RNG interrupt has occurred or not.
  * @param  __HANDLE__ RNG Handle
  * @param  __INTERRUPT__ specifies the RNG interrupt status flag to check.
  *         This parameter can be one of the following values:
  *            @arg RNG_IT_DRDY: Data ready interrupt
  *            @arg RNG_IT_CEI: Clock error interrupt
  *            @arg RNG_IT_SEI: Seed error interrupt
  * @retval The new state of __INTERRUPT__ (SET or RESET).
  */
#define __DAL_RNG_GET_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->STS & (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @brief  Clear the RNG interrupt status flags.
  * @param  __HANDLE__ RNG Handle
  * @param  __INTERRUPT__ specifies the RNG interrupt status flag to clear.
  *          This parameter can be one of the following values:
  *            @arg RNG_IT_CEI: Clock error interrupt
  *            @arg RNG_IT_SEI: Seed error interrupt
  * @note   RNG_IT_DRDY flag is read-only, reading RNG_DATA register automatically clears RNG_IT_DRDY.
  * @retval None
  */
#define __DAL_RNG_CLEAR_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->STS) = ~(__INTERRUPT__))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup RNG_Exported_Functions RNG Exported Functions
  * @{
  */

/** @defgroup RNG_Exported_Functions_Group1 Initialization and configuration functions
  * @{
  */
DAL_StatusTypeDef DAL_RNG_Init(RNG_HandleTypeDef *hrng);
DAL_StatusTypeDef DAL_RNG_DeInit(RNG_HandleTypeDef *hrng);
void DAL_RNG_MspInit(RNG_HandleTypeDef *hrng);
void DAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_RNG_REGISTER_CALLBACKS == 1)
DAL_StatusTypeDef DAL_RNG_RegisterCallback(RNG_HandleTypeDef *hrng, DAL_RNG_CallbackIDTypeDef CallbackID,
                                           pRNG_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_RNG_UnRegisterCallback(RNG_HandleTypeDef *hrng, DAL_RNG_CallbackIDTypeDef CallbackID);

DAL_StatusTypeDef DAL_RNG_RegisterReadyDataCallback(RNG_HandleTypeDef *hrng, pRNG_ReadyDataCallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_RNG_UnRegisterReadyDataCallback(RNG_HandleTypeDef *hrng);
#endif /* USE_DAL_RNG_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */
uint32_t DAL_RNG_GetRandomNumber(RNG_HandleTypeDef
                                 *hrng);    /* Obsolete, use DAL_RNG_GenerateRandomNumber() instead    */
uint32_t DAL_RNG_GetRandomNumber_IT(RNG_HandleTypeDef
                                    *hrng); /* Obsolete, use DAL_RNG_GenerateRandomNumber_IT() instead */
DAL_StatusTypeDef DAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
DAL_StatusTypeDef DAL_RNG_GenerateRandomNumber_IT(RNG_HandleTypeDef *hrng);
uint32_t DAL_RNG_ReadLastRandomNumber(RNG_HandleTypeDef *hrng);

void DAL_RNG_IRQHandler(RNG_HandleTypeDef *hrng);
void DAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng);
void DAL_RNG_ReadyDataCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit);

/**
  * @}
  */

/** @defgroup RNG_Exported_Functions_Group3 Peripheral State functions
  * @{
  */
DAL_RNG_StateTypeDef DAL_RNG_GetState(RNG_HandleTypeDef *hrng);
uint32_t             DAL_RNG_GetError(RNG_HandleTypeDef *hrng);
/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup RNG_Private_Macros RNG Private Macros
  * @{
  */
#define IS_RNG_IT(IT) (((IT) == RNG_IT_CEI) || \
                       ((IT) == RNG_IT_SEI))

#define IS_RNG_FLAG(FLAG) (((FLAG) == RNG_FLAG_DRDY) || \
                           ((FLAG) == RNG_FLAG_CECS) || \
                           ((FLAG) == RNG_FLAG_SECS))

/**
  * @}
  */

/**
  * @}
  */

#endif /* RNG */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* APM32F4xx_DAL_RNG_H */

