/**
  *
  * @file    apm32f4xx_dal_comp.h
  * @brief   Header file of COMP DAL module.
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
  * Copyright (C) 2023-2024 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_COMP_H
#define APM32F4xx_DAL_COMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup COMP
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup COMP_Exported_Types COMP Exported Types
  * @{
  */

/**
 * @brief  COMP Init structure definition
 */
typedef struct
{
    uint32_t WindowMode;            /*!< Set window mode.
                                        Note: Window mode is available only for COMP1.
                                        This parameter can be a value of @ref COMP_WindowMode */

    uint32_t Mode;                  /*!< Set comparator operating mode to adjust speed.
                                        Note: Speed mode is available only for COMP2.
                                        This parameter can be a value of @ref COMP_SpeedMode */

    uint32_t NonInvertingInput;     /*!< Set comparator input plus (non-inverting input).
                                        Note: Non-inverting input is available only for COMP2.
                                        This parameter can be a value of @ref COMP_NonInvertingInput */

    uint32_t InvertingInput;        /*!< Set comparator input minus (inverting input).
                                        This parameter can be a value of @ref COMP_InvertingInput */

    uint32_t Output;                /*!< Set comparator output.
                                        This parameter can be a value of @ref COMP_Output */

    uint32_t OutputPol;             /*!< Set comparator output polarity.
                                        This parameter can be a value of @ref COMP_OutputPolarity */

    
} COMP_InitTypeDef;

/**
 * @brief  DAL COMP States definition
 */
#define COMP_STATE_LOCKED             (uint32_t)0x10U             /*!< COMP Configuration is locked */
typedef enum
{
    DAL_COMP_STATE_RESET             = 0x00U,                                           /*!< COMP not yet initialized */
    DAL_COMP_STATE_RESET_LOCKED      = (DAL_COMP_STATE_RESET | COMP_STATE_LOCKED),      /*!< COMP not yet initialized and configuration is locked */
    DAL_COMP_STATE_READY             = 0x01U,                                           /*!< COMP initialized and ready for use */
    DAL_COMP_STATE_READY_LOCKED      = (DAL_COMP_STATE_READY | COMP_STATE_LOCKED),      /*!< COMP initialized but configuration is locked */
    DAL_COMP_STATE_BUSY              = 0x02U,                                           /*!< COMP internal process is ongoing */
    DAL_COMP_STATE_BUSY_LOCKED       = (DAL_COMP_STATE_BUSY | COMP_STATE_LOCKED),       /*!< COMP internal process ongoing but configuration is locked */
} DAL_COMP_StateTypeDef;

/**
 * @brief  COMP Handle Structure definition
 */
#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
typedef struct __COMP_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */
{
    COMP_TypeDef                  *Instance;      /*!< Register base address    */

    COMP_InitTypeDef              Init;           /*!< COMP required parameters */

    __IO DAL_COMP_StateTypeDef    State;          /*!< COMP communication state */

    DAL_LockTypeDef               Lock;           /*!< Locking object           */

    __IO uint32_t                 ErrorCode;      /*!< COMP Error code          */
#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
    void (* MspInitCallback)(struct __COMP_HandleTypeDef * hcomp);           /*!< COMP Msp Init callback              */
    void (* MspDeInitCallback)(struct __COMP_HandleTypeDef * hcomp);         /*!< COMP Msp DeInit callback            */
#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */
} COMP_HandleTypeDef;

#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
/**
 * @brief  COMP Callbacks ID enumeration definition
 */
typedef enum
{
    DAL_COMP_MSPINIT_CB_ID            = 0x00U,    /*!< COMP MspInit callback ID          */
    DAL_COMP_MSPDEINIT_CB_ID          = 0x01U     /*!< COMP MspDeInit callback ID        */
} DAL_COMP_CallbackIDTypeDef;

/**
 * @brief  COMP Callback pointer definition
 */
typedef  void (*pCOMP_CallbackTypeDef)(COMP_HandleTypeDef * hcomp); /*!< pointer to a COMP callback function */
#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup COMP_Exported_Constants COMP Exported Constants
  * @{
  */

/** @defgroup COMP_Error_Code COMP Error Code
 * @brief    COMP Error Code
 * @{
 */
#define DAL_COMP_ERROR_NONE             0x00000000U    /*!< No error              */
#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
#define DAL_COMP_ERROR_INVALID_CALLBACK 0x00000001U    /*!< Invalid Callback error */
#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup COMP_WindowMode COMP Window Mode
 * @{
 */
#define COMP_WINDOWMODE_DISABLE         0x00000000U             /*!< Window mode disable: Comparators instances pair COMP1 and COMP2 are independent */
#define COMP_WINDOWMODE_ENABLE          COMP_CSTS_WMODESEL      /*!< Window mode enable: Comparators instances pair COMP1 and COMP2 have their non inverting inputs connected together */
/**
  * @}
  */

/** @defgroup COMP_NonInvertingInput COMP Non Inverting Input
 * @{
 */
#define COMP_NONINVERTING_INPUT_PC2      0x00000000U             /*!< PC2 available only for COMP2 */
/**
  * @}
  */

/** @defgroup COMP_InvertingInput COMP Inverting Input
 * @{
 */
#define COMP_INVERTING_INPUT_VREFINT    0x00000000U                                 /*!< VrefInt */
#define COMP_INVERTING_INPUT_PC1        COMP_CSTS_INMCCFG_0                         /*!< PC1 available only for COMP1 */
#define COMP_INVERTING_INPUT_PC3        COMP_CSTS_INMCCFG_0                         /*!< PC3 available only for COMP2 */
#define COMP_INVERTING_INPUT_1_4VREFINT COMP_CSTS_INMCCFG_2                         /*!< 1/4 VrefInt available only for COMP2 */
#define COMP_INVERTING_INPUT_1_2VREFINT (COMP_CSTS_INMCCFG_2 | COMP_CSTS_INMCCFG_0) /*!< 1/2 VrefInt available only for COMP2 */
#define COMP_INVERTING_INPUT_3_4VREFINT (COMP_CSTS_INMCCFG_2 | COMP_CSTS_INMCCFG_1) /*!< 3/4 VrefInt available only for COMP2 */
/**
  * @}
  */

/** @defgroup COMP_SpeedMode COMP Speed Mode
 * @{
 */
#define COMP_SPEEDMODE_LOW              0x00000000U             /*!< Low speed */
#define COMP_SPEEDMODE_HIGH             (COMP_CSTS_SPEEDM)      /*!< High speed */
/**
  * @}
  */

/** @defgroup COMP_Output COMP Output
 * @{
 */
#define COMP_OUTPUT_NONE                0x00000000U                                                                             /*!< COMP output isn't connected to other peripherals */
#define COMP_OUTPUT_TMR1BKIN            COMP_CSTS_OUTSEL_0                                                                      /*!< COMP output connected to TMR1 Break Input (BKIN) */
#define COMP_OUTPUT_TMR1IC1             COMP_CSTS_OUTSEL_1                                                                      /*!< COMP output connected to TMR1 Input Capture 1 */
#define COMP_OUTPUT_TMR1ETRF            (COMP_CSTS_OUTSEL_1 | COMP_CSTS_OUTSEL_0)                                               /*!< COMP output connected to TMR1 External Trigger Input */
#define COMP_OUTPUT_TMR8BKIN            COMP_CSTS_OUTSEL_2                                                                      /*!< COMP output connected to TMR8 Break Input (BKIN) */
#define COMP_OUTPUT_TMR8IC1             (COMP_CSTS_OUTSEL_2 | COMP_CSTS_OUTSEL_0)                                               /*!< COMP output connected to TMR8 Input Capture 1 */
#define COMP_OUTPUT_TMR8ETRF            (COMP_CSTS_OUTSEL_2 | COMP_CSTS_OUTSEL_1)                                               /*!< COMP output connected to TMR8 External Trigger Input */
#define COMP_OUTPUT_TMR2IC4             (COMP_CSTS_OUTSEL_2 | COMP_CSTS_OUTSEL_1 | COMP_CSTS_OUTSEL_0)                          /*!< COMP output connected to TMR2 Input Capture 4 */
#define COMP_OUTPUT_TMR2ETRF            (COMP_CSTS_OUTSEL_3)                                                                    /*!< COMP output connected to TMR2 External Trigger Input */
#define COMP_OUTPUT_TMR3IC1             (COMP_CSTS_OUTSEL_3 | COMP_CSTS_OUTSEL_0)                                               /*!< COMP output connected to TMR3 Input Capture 1 */
#define COMP_OUTPUT_TMR3ETRF            (COMP_CSTS_OUTSEL_3 | COMP_CSTS_OUTSEL_1)                                               /*!< COMP output connected to TMR3 External Trigger Input */
#define COMP_OUTPUT_TMR4IC1             (COMP_CSTS_OUTSEL_3 | COMP_CSTS_OUTSEL_1 | COMP_CSTS_OUTSEL_0)                          /*!< COMP output connected to TMR4 Input Capture 1 */
/**
  * @}
  */

/** @defgroup COMP_OutputPolarity COMP Output Polarity
 * @{
 */
#define COMP_OUTPUTPOL_NONINVERTED      0x00000000U             /*!< COMP output on GPIO isn't inverted */
#define COMP_OUTPUTPOL_INVERTED         COMP_CSTS_POLCFG        /*!< COMP output on GPIO is inverted */
/**
  * @}
  */

/** @defgroup COMP_OutputLevel COMP Output Level
 * @{
 */
#define COMP_OUTPUTLEVEL_LOW            (uint32_t)0x00000000U   /*!< COMP output level is low */
#define COMP_OUTPUTLEVEL_HIGH           (uint32_t)0x00000001U   /*!< COMP output level is high */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup COMP_Exported_Macros COMP Exported Macros
  * @{
  */
/** @defgroup COMP_Handle_Management COMP Handle Management
 * @{
  */
/**
 * @brief  Reset COMP handle state
 * @param  __HANDLE__ COMP handle.
 * @retval None
 */
#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
#define __DAL_COMP_RESET_HANDLE_STATE(__HANDLE__) do{                                                   \
                                                      (__HANDLE__)->State = DAL_COMP_STATE_RESET;      \
                                                      (__HANDLE__)->MspInitCallback = NULL;            \
                                                      (__HANDLE__)->MspDeInitCallback = NULL;          \
                                                    } while(0)
#else
#define __DAL_COMP_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_COMP_STATE_RESET)
#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */

/**
  * @brief Clear COMP error code (set it to no error code "DAL_COMP_ERROR_NONE").
  * @param __HANDLE__ COMP handle
  * @retval None
  */
#define COMP_CLEAR_ERRORCODE(__HANDLE__)    ((__HANDLE__)->ErrorCode = DAL_COMP_ERROR_NONE)

/**
 * @brief  Enable the specified comparator.
 * @param  __HANDLE__ COMP handle
 * @retval None
 */
#define __DAL_COMP_ENABLE(__HANDLE__)       ((__HANDLE__)->Instance->CSTS |= COMP_CSTS_EN)

/**
 * @brief  Disable the specified comparator.
 * @param  __HANDLE__ COMP handle
 * @retval None
 */
#define __DAL_COMP_DISABLE(__HANDLE__)      ((__HANDLE__)->Instance->CSTS &= ~COMP_CSTS_EN)

/**
 * @brief  Lock the specified comparator configuration.
 * @note   Using this function implies that COMP DAL status is already locked.
 *         To unlock the configuration, user will have to resort to __DAL_COMP_DISABLE_LOCK() API.
 * @param  __HANDLE__ COMP handle
 * @retval None
 */
#define __DAL_COMP_LOCK(__HANDLE__)         ((__HANDLE__)->Instance->CSTS |= COMP_CSTS_LOCK)

/**
 * @brief  Check whether the specified COMP is locked.
 * @param  __HANDLE__ COMP handle
 * @retval Value 0: COMP is not locked, Value 1: COMP is locked
 */
#define __DAL_COMP_IS_LOCKED(__HANDLE__)    (((__HANDLE__)->Instance->CSTS & COMP_CSTS_LOCK) == COMP_CSTS_LOCK)

/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup COMP_Private_Macros COMP Private Macros
  * @{
  */

/** @defgroup COMP_IS_COMP_Definitions COMP private macros to check input parameters
  * @{
  */
#define IS_COMP_WINDOWMODE(__MODE__) (((__MODE__) == COMP_WINDOWMODE_DISABLE) || \
                                      ((__MODE__) == COMP_WINDOWMODE_ENABLE))

#define IS_COMP1_INVERTINGINPUT(__INPUT__) (((__INPUT__) == COMP_INVERTING_INPUT_VREFINT) || \
                                           ((__INPUT__) == COMP_INVERTING_INPUT_PC1))

#define IS_COMP2_INVERTINGINPUT(__INPUT__) (((__INPUT__) == COMP_INVERTING_INPUT_VREFINT) || \
                                           ((__INPUT__) == COMP_INVERTING_INPUT_PC3) || \
                                           ((__INPUT__) == COMP_INVERTING_INPUT_1_4VREFINT) || \
                                           ((__INPUT__) == COMP_INVERTING_INPUT_1_2VREFINT) || \
                                           ((__INPUT__) == COMP_INVERTING_INPUT_3_4VREFINT))

#define IS_COMP_NONINVERTINGINPUT(__INPUT__) (((__INPUT__) == COMP_NONINVERTING_INPUT_PC2))

#define IS_COMP_SPEEDMODE(__MODE__) (((__MODE__) == COMP_SPEEDMODE_LOW) || \
                                     ((__MODE__) == COMP_SPEEDMODE_HIGH))

#define IS_COMP_OUTPUT(__OUTPUT__) (((__OUTPUT__) == COMP_OUTPUT_NONE) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR1BKIN) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR1IC1) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR1ETRF) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR8BKIN) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR8IC1) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR8ETRF) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR2IC4) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR2ETRF) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR3IC1) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR3ETRF) || \
                                    ((__OUTPUT__) == COMP_OUTPUT_TMR4IC1))

#define IS_COMP_OUTPUTPOL(__POLARITY__) (((__POLARITY__) == COMP_OUTPUTPOL_NONINVERTED) || \
                                         ((__POLARITY__) == COMP_OUTPUTPOL_INVERTED))

#define IS_COMP_OUTPUTLEVEL(__LEVEL__) (((__LEVEL__) == COMP_OUTPUTLEVEL_LOW) || \
                                        ((__LEVEL__) == COMP_OUTPUTLEVEL_HIGH))

/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup COMP_Exported_Functions
  * @{
  */

/** @addtogroup COMP_Exported_Functions_Group1
 * @{
 */
/* Initialization/de-initialization functions  ********************************/
DAL_StatusTypeDef DAL_COMP_Init(COMP_HandleTypeDef * hcomp);
DAL_StatusTypeDef DAL_COMP_DeInit(COMP_HandleTypeDef * hcomp);
void              DAL_COMP_MspInit(COMP_HandleTypeDef * hcomp);
void              DAL_COMP_MspDeInit(COMP_HandleTypeDef * hcomp);
#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
/* Callbacks Register/UnRegister functions  ***********************************/
DAL_StatusTypeDef DAL_COMP_RegisterCallback(COMP_HandleTypeDef * hcomp, DAL_COMP_CallbackIDTypeDef CallbackID, pCOMP_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_COMP_UnRegisterCallback(COMP_HandleTypeDef * hcomp, DAL_COMP_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @addtogroup COMP_Exported_Functions_Group2
 * @{
 */
/* I/O operation functions  ***************************************************/
DAL_StatusTypeDef DAL_COMP_Start(COMP_HandleTypeDef * hcomp);
DAL_StatusTypeDef DAL_COMP_Stop(COMP_HandleTypeDef * hcomp);
/**
  * @}
  */

/** @addtogroup COMP_Exported_Functions_Group3
 * @{
 */
/* Peripheral Control functions  ************************************************/
DAL_StatusTypeDef DAL_COMP_Lock(COMP_HandleTypeDef * hcomp);
uint32_t          DAL_COMP_GetOutputLevel(COMP_HandleTypeDef * hcomp);
/**
  * @}
  */

/** @addtogroup COMP_Exported_Functions_Group4
 * @{
 */
/* Peripheral State functions  **************************************************/
DAL_COMP_StateTypeDef DAL_COMP_GetState(COMP_HandleTypeDef * hcomp);
uint32_t              DAL_COMP_GetError(COMP_HandleTypeDef * hcomp);
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

#endif /* APM32F4xx_DAL_COMP_H */

