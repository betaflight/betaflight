/**
  *
  * @file    apm32f4xx_dal_wwdt.h
  * @brief   Header file of WWDT DAL module.
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
#ifndef APM32F4xx_DAL_WWDT_H
#define APM32F4xx_DAL_WWDT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup WWDT
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup WWDT_Exported_Types WWDT Exported Types
  * @{
  */

/**
  * @brief  WWDT Init structure definition
  */
typedef struct
{
  uint32_t Prescaler;     /*!< Specifies the prescaler value of the WWDT.
                               This parameter can be a value of @ref WWDT_Prescaler */

  uint32_t Window;        /*!< Specifies the WWDT window value to be compared to the downcounter.
                               This parameter must be a number Min_Data = 0x40 and Max_Data = 0x7F */

  uint32_t Counter;       /*!< Specifies the WWDT free-running downcounter  value.
                               This parameter must be a number between Min_Data = 0x40 and Max_Data = 0x7F */

  uint32_t EWIMode ;      /*!< Specifies if WWDT Early Wakeup Interrupt is enable or not.
                               This parameter can be a value of @ref WWDT_EWI_Mode */

} WWDT_InitTypeDef;

/**
  * @brief  WWDT handle Structure definition
  */
#if (USE_DAL_WWDT_REGISTER_CALLBACKS == 1)
typedef struct __WWDT_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_WWDT_REGISTER_CALLBACKS */
{
  WWDT_TypeDef      *Instance;  /*!< Register base address */

  WWDT_InitTypeDef  Init;       /*!< WWDT required parameters */

#if (USE_DAL_WWDT_REGISTER_CALLBACKS == 1)
  void (* EwiCallback)(struct __WWDT_HandleTypeDef *hwwdt);                  /*!< WWDT Early WakeUp Interrupt callback */

  void (* MspInitCallback)(struct __WWDT_HandleTypeDef *hwwdt);              /*!< WWDT Msp Init callback */
#endif /* USE_DAL_WWDT_REGISTER_CALLBACKS */
} WWDT_HandleTypeDef;

#if (USE_DAL_WWDT_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL WWDT common Callback ID enumeration definition
  */
typedef enum
{
  DAL_WWDT_EWI_CB_ID          = 0x00U,    /*!< WWDT EWI callback ID */
  DAL_WWDT_MSPINIT_CB_ID      = 0x01U,    /*!< WWDT MspInit callback ID */
} DAL_WWDT_CallbackIDTypeDef;

/**
  * @brief  DAL WWDT Callback pointer definition
  */
typedef void (*pWWDT_CallbackTypeDef)(WWDT_HandleTypeDef *hppp);  /*!< pointer to a WWDT common callback functions */

#endif /* USE_DAL_WWDT_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup WWDT_Exported_Constants WWDT Exported Constants
  * @{
  */

/** @defgroup WWDT_Interrupt_definition WWDT Interrupt definition
  * @{
  */
#define WWDT_IT_EWI                         WWDT_CFR_EWIEN  /*!< Early wakeup interrupt */
/**
  * @}
  */

/** @defgroup WWDT_Flag_definition WWDT Flag definition
  * @brief WWDT Flag definition
  * @{
  */
#define WWDT_FLAG_EWIF                      WWDT_STS_EWIFLG  /*!< Early wakeup interrupt flag */
/**
  * @}
  */

/** @defgroup WWDT_Prescaler WWDT Prescaler
  * @{
  */
#define WWDT_PRESCALER_1                    0x00000000u                              /*!< WWDT counter clock = (PCLK1/4096)/1 */
#define WWDT_PRESCALER_2                    WWDT_CFR_TBPSC_0                         /*!< WWDT counter clock = (PCLK1/4096)/2 */
#define WWDT_PRESCALER_4                    WWDT_CFR_TBPSC_1                         /*!< WWDT counter clock = (PCLK1/4096)/4 */
#define WWDT_PRESCALER_8                    (WWDT_CFR_TBPSC_1 | WWDT_CFR_TBPSC_0)    /*!< WWDT counter clock = (PCLK1/4096)/8 */
/**
  * @}
  */

/** @defgroup WWDT_EWI_Mode WWDT Early Wakeup Interrupt Mode
  * @{
  */
#define WWDT_EWI_DISABLE                    0x00000000u       /*!< EWI Disable */
#define WWDT_EWI_ENABLE                     WWDT_CFR_EWIEN      /*!< EWI Enable */
/**
  * @}
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/** @defgroup WWDT_Private_Macros WWDT Private Macros
  * @{
  */
#define IS_WWDT_PRESCALER(__PRESCALER__)    (((__PRESCALER__) == WWDT_PRESCALER_1)  || \
                                             ((__PRESCALER__) == WWDT_PRESCALER_2)  || \
                                             ((__PRESCALER__) == WWDT_PRESCALER_4)  || \
                                             ((__PRESCALER__) == WWDT_PRESCALER_8))

#define IS_WWDT_WINDOW(__WINDOW__)          (((__WINDOW__) >= WWDT_CFR_WIN_6) && ((__WINDOW__) <= WWDT_CFR_WIN))

#define IS_WWDT_COUNTER(__COUNTER__)        (((__COUNTER__) >= WWDT_CTRL_CNT_6) && ((__COUNTER__) <= WWDT_CTRL_CNT))

#define IS_WWDT_EWI_MODE(__MODE__)          (((__MODE__) == WWDT_EWI_ENABLE) || \
                                             ((__MODE__) == WWDT_EWI_DISABLE))
/**
  * @}
  */


/* Exported macros ------------------------------------------------------------*/

/** @defgroup WWDT_Exported_Macros WWDT Exported Macros
  * @{
  */

/**
  * @brief  Enable the WWDT peripheral.
  * @param  __HANDLE__  WWDT handle
  * @retval None
  */
#define __DAL_WWDT_ENABLE(__HANDLE__)                         SET_BIT((__HANDLE__)->Instance->CTRL, WWDT_CTRL_WWDTEN)

/**
  * @brief  Enable the WWDT early wakeup interrupt.
  * @param  __HANDLE__     WWDT handle
  * @param  __INTERRUPT__  specifies the interrupt to enable.
  *         This parameter can be one of the following values:
  *            @arg WWDT_IT_EWI: Early wakeup interrupt
  * @note   Once enabled this interrupt cannot be disabled except by a system reset.
  * @retval None
  */
#define __DAL_WWDT_ENABLE_IT(__HANDLE__, __INTERRUPT__)       SET_BIT((__HANDLE__)->Instance->CFR, (__INTERRUPT__))

/**
  * @brief  Check whether the selected WWDT interrupt has occurred or not.
  * @param  __HANDLE__  WWDT handle
  * @param  __INTERRUPT__  specifies the it to check.
  *        This parameter can be one of the following values:
  *            @arg WWDT_FLAG_EWIF: Early wakeup interrupt IT
  * @retval The new state of WWDT_FLAG (SET or RESET).
  */
#define __DAL_WWDT_GET_IT(__HANDLE__, __INTERRUPT__)        __DAL_WWDT_GET_FLAG((__HANDLE__),(__INTERRUPT__))

/** @brief  Clear the WWDT interrupt pending bits.
  *         bits to clear the selected interrupt pending bits.
  * @param  __HANDLE__  WWDT handle
  * @param  __INTERRUPT__  specifies the interrupt pending bit to clear.
  *         This parameter can be one of the following values:
  *            @arg WWDT_FLAG_EWIF: Early wakeup interrupt flag
  */
#define __DAL_WWDT_CLEAR_IT(__HANDLE__, __INTERRUPT__)      __DAL_WWDT_CLEAR_FLAG((__HANDLE__), (__INTERRUPT__))

/**
  * @brief  Check whether the specified WWDT flag is set or not.
  * @param  __HANDLE__  WWDT handle
  * @param  __FLAG__  specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg WWDT_FLAG_EWIF: Early wakeup interrupt flag
  * @retval The new state of WWDT_FLAG (SET or RESET).
  */
#define __DAL_WWDT_GET_FLAG(__HANDLE__, __FLAG__)           (((__HANDLE__)->Instance->STS & (__FLAG__)) == (__FLAG__))

/**
  * @brief  Clear the WWDT's pending flags.
  * @param  __HANDLE__  WWDT handle
  * @param  __FLAG__  specifies the flag to clear.
  *         This parameter can be one of the following values:
  *            @arg WWDT_FLAG_EWIF: Early wakeup interrupt flag
  * @retval None
  */
#define __DAL_WWDT_CLEAR_FLAG(__HANDLE__, __FLAG__)         ((__HANDLE__)->Instance->STS = ~(__FLAG__))

/** @brief  Check whether the specified WWDT interrupt source is enabled or not.
  * @param  __HANDLE__  WWDT Handle.
  * @param  __INTERRUPT__  specifies the WWDT interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg WWDT_IT_EWI: Early Wakeup Interrupt
  * @retval state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __DAL_WWDT_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CFR\
                                                              & (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup WWDT_Exported_Functions
  * @{
  */

/** @addtogroup WWDT_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
DAL_StatusTypeDef     DAL_WWDT_Init(WWDT_HandleTypeDef *hwwdt);
void                  DAL_WWDT_MspInit(WWDT_HandleTypeDef *hwwdt);
/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_DAL_WWDT_REGISTER_CALLBACKS == 1)
DAL_StatusTypeDef     DAL_WWDT_RegisterCallback(WWDT_HandleTypeDef *hwwdt, DAL_WWDT_CallbackIDTypeDef CallbackID,
                                                pWWDT_CallbackTypeDef pCallback);
DAL_StatusTypeDef     DAL_WWDT_UnRegisterCallback(WWDT_HandleTypeDef *hwwdt, DAL_WWDT_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_WWDT_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup WWDT_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ******************************************************/
DAL_StatusTypeDef     DAL_WWDT_Refresh(WWDT_HandleTypeDef *hwwdt);
void                  DAL_WWDT_IRQHandler(WWDT_HandleTypeDef *hwwdt);
void                  DAL_WWDT_EarlyWakeupCallback(WWDT_HandleTypeDef *hwwdt);
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

#endif /* APM32F4xx_DAL_WWDT_H */
