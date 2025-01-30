/**
  *
  * @file    apm32f4xx_dal_tmr_ex.h
  * @brief   Header file of TMR DAL Extended module.
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
#ifndef APM32F4xx_DAL_TMR_EX_H
#define APM32F4xx_DAL_TMR_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup TMREx
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup TMREx_Exported_Types TMR Extended Exported Types
  * @{
  */

/**
  * @brief  TMR Hall sensor Configuration Structure definition
  */

typedef struct
{
  uint32_t IC1Polarity;         /*!< Specifies the active edge of the input signal.
                                     This parameter can be a value of @ref TMR_Input_Capture_Polarity */

  uint32_t IC1Prescaler;        /*!< Specifies the Input Capture Prescaler.
                                     This parameter can be a value of @ref TMR_Input_Capture_Prescaler */

  uint32_t IC1Filter;           /*!< Specifies the input capture filter.
                                     This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF */

  uint32_t Commutation_Delay;   /*!< Specifies the pulse value to be loaded into the Capture Compare Register.
                                     This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */
} TMR_HallSensor_InitTypeDef;
/**
  * @}
  */
/* End of exported types -----------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup TMREx_Exported_Constants TMR Extended Exported Constants
  * @{
  */

/** @defgroup TMREx_Remap TMR Extended Remapping
  * @{
  */
#if defined (TMR2)
#if defined(TMR8)
#define TMR_TMR2_TMR8_TRGO                     0x00000000U                              /*!< TMR2 ITR1 is connected to TMR8 TRGO */
#else
#define TMR_TMR2_ETH_PTP                       TMR_OR_RMPSEL_0                        /*!< TMR2 ITR1 is connected to PTP trigger output */
#endif /*  TMR8 */
#define TMR_TMR2_USBFS_SOF                     TMR_OR_RMPSEL_1                        /*!< TMR2 ITR1 is connected to OTG FS SOF */
#define TMR_TMR2_USBHS_SOF                     (TMR_OR_RMPSEL_1 | TMR_OR_RMPSEL_0)  /*!< TMR2 ITR1 is connected to OTG HS SOF */
#endif /* TMR2 */

#define TMR_TMR5_GPIO                          0x00000000U                              /*!< TMR5 TI4 is connected to GPIO */
#define TMR_TMR5_LSI                           TMR_OR_TI4_RMPSEL_0                         /*!< TMR5 TI4 is connected to LSI */
#define TMR_TMR5_LSE                           TMR_OR_TI4_RMPSEL_1                         /*!< TMR5 TI4 is connected to LSE */
#define TMR_TMR5_RTC                           (TMR_OR_TI4_RMPSEL_1 | TMR_OR_TI4_RMPSEL_0)    /*!< TMR5 TI4 is connected to the RTC wakeup interrupt */

#define TMR_TMR11_GPIO                         0x00000000U                              /*!< TMR11 TI1 is connected to GPIO */
#define TMR_TMR11_HSE                          TMR_OR_TI1_RMPSEL_1                         /*!< TMR11 TI1 is connected to HSE_RTC clock */
#if defined(SPDIFRX)
#define TMR_TMR11_SPDIFRX                      TMR_OR_TI1_RMPSEL_0                         /*!< TMR11 TI1 is connected to SPDIFRX_FRAME_SYNC */
#endif /* SPDIFRX*/

#if defined(LPTMR_OR_TMR1_ITR2_RMP) && defined(LPTMR_OR_TMR5_ITR1_RMP) && defined(LPTMR_OR_TMR5_ITR1_RMP)
#define LPTMR_REMAP_MASK                       0x10000000U

#define TMR_TMR9_TMR3_TRGO                     LPTMR_REMAP_MASK                             /*!< TMR9 ITR1 is connected to TMR3 TRGO */
#define TMR_TMR9_LPTMR                         (LPTMR_REMAP_MASK | LPTMR_OR_TMR9_ITR1_RMP)  /*!< TMR9 ITR1 is connected to LPTMR1 output */

#define TMR_TMR5_TMR3_TRGO                     LPTMR_REMAP_MASK                             /*!< TMR5 ITR1 is connected to TMR3 TRGO */
#define TMR_TMR5_LPTMR                         (LPTMR_REMAP_MASK | LPTMR_OR_TMR5_ITR1_RMP)  /*!< TMR5 ITR1 is connected to LPTMR1 output */

#define TMR_TMR1_TMR3_TRGO                     LPTMR_REMAP_MASK                             /*!< TMR1 ITR2 is connected to TMR3 TRGO */
#define TMR_TMR1_LPTMR                         (LPTMR_REMAP_MASK | LPTMR_OR_TMR1_ITR2_RMP)  /*!< TMR1 ITR2 is connected to LPTMR1 output */
#endif /* LPTMR_OR_TMR1_ITR2_RMP &&  LPTMR_OR_TMR5_ITR1_RMP && LPTMR_OR_TMR5_ITR1_RMP */
/**
  * @}
  */

/**
  * @}
  */
/* End of exported constants -------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TMREx_Exported_Macros TMR Extended Exported Macros
  * @{
  */

/**
  * @}
  */
/* End of exported macro -----------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/** @defgroup TMREx_Private_Macros TMR Extended Private Macros
  * @{
  */
#if defined(SPDIFRX)
#define IS_TMR_REMAP(INSTANCE, TMR_REMAP)                                 \
  ((((INSTANCE) == TMR2)  && (((TMR_REMAP) == TMR_TMR2_TMR8_TRGO)      || \
                              ((TMR_REMAP) == TMR_TMR2_USBFS_SOF)      || \
                              ((TMR_REMAP) == TMR_TMR2_USBHS_SOF)))    || \
   (((INSTANCE) == TMR5)  && (((TMR_REMAP) == TMR_TMR5_GPIO)           || \
                              ((TMR_REMAP) == TMR_TMR5_LSI)            || \
                              ((TMR_REMAP) == TMR_TMR5_LSE)            || \
                              ((TMR_REMAP) == TMR_TMR5_RTC)))          || \
   (((INSTANCE) == TMR11) && (((TMR_REMAP) == TMR_TMR11_GPIO)          || \
                              ((TMR_REMAP) == TMR_TMR11_SPDIFRX)       || \
                              ((TMR_REMAP) == TMR_TMR11_HSE))))
#elif defined(TMR2)
#if defined(LPTMR_OR_TMR1_ITR2_RMP) && defined(LPTMR_OR_TMR5_ITR1_RMP) && defined(LPTMR_OR_TMR5_ITR1_RMP)
#define IS_TMR_REMAP(INSTANCE, TMR_REMAP)                                 \
  ((((INSTANCE) == TMR2)  && (((TMR_REMAP) == TMR_TMR2_TMR8_TRGO)      || \
                              ((TMR_REMAP) == TMR_TMR2_USBFS_SOF)      || \
                              ((TMR_REMAP) == TMR_TMR2_USBHS_SOF)))    || \
   (((INSTANCE) == TMR5)  && (((TMR_REMAP) == TMR_TMR5_GPIO)           || \
                              ((TMR_REMAP) == TMR_TMR5_LSI)            || \
                              ((TMR_REMAP) == TMR_TMR5_LSE)            || \
                              ((TMR_REMAP) == TMR_TMR5_RTC)))          || \
   (((INSTANCE) == TMR11) && (((TMR_REMAP) == TMR_TMR11_GPIO)          || \
                              ((TMR_REMAP) == TMR_TMR11_HSE)))         || \
   (((INSTANCE) == TMR1)  && (((TMR_REMAP) == TMR_TMR1_TMR3_TRGO)      || \
                              ((TMR_REMAP) == TMR_TMR1_LPTMR)))        || \
   (((INSTANCE) == TMR5)  && (((TMR_REMAP) == TMR_TMR5_TMR3_TRGO)      || \
                              ((TMR_REMAP) == TMR_TMR5_LPTMR)))        || \
   (((INSTANCE) == TMR9)  && (((TMR_REMAP) == TMR_TMR9_TMR3_TRGO)      || \
                              ((TMR_REMAP) == TMR_TMR9_LPTMR))))
#elif defined(TMR8)
#define IS_TMR_REMAP(INSTANCE, TMR_REMAP)                                 \
  ((((INSTANCE) == TMR2)  && (((TMR_REMAP) == TMR_TMR2_TMR8_TRGO)      || \
                              ((TMR_REMAP) == TMR_TMR2_USBFS_SOF)      || \
                              ((TMR_REMAP) == TMR_TMR2_USBHS_SOF)))    || \
   (((INSTANCE) == TMR5)  && (((TMR_REMAP) == TMR_TMR5_GPIO)           || \
                              ((TMR_REMAP) == TMR_TMR5_LSI)            || \
                              ((TMR_REMAP) == TMR_TMR5_LSE)            || \
                              ((TMR_REMAP) == TMR_TMR5_RTC)))          || \
   (((INSTANCE) == TMR11) && (((TMR_REMAP) == TMR_TMR11_GPIO)          || \
                              ((TMR_REMAP) == TMR_TMR11_HSE))))
#else
#define IS_TMR_REMAP(INSTANCE, TMR_REMAP)                                 \
  ((((INSTANCE) == TMR2)  && (((TMR_REMAP) == TMR_TMR2_ETH_PTP)        || \
                              ((TMR_REMAP) == TMR_TMR2_USBFS_SOF)      || \
                              ((TMR_REMAP) == TMR_TMR2_USBHS_SOF)))    || \
   (((INSTANCE) == TMR5)  && (((TMR_REMAP) == TMR_TMR5_GPIO)           || \
                              ((TMR_REMAP) == TMR_TMR5_LSI)            || \
                              ((TMR_REMAP) == TMR_TMR5_LSE)            || \
                              ((TMR_REMAP) == TMR_TMR5_RTC)))          || \
   (((INSTANCE) == TMR11) && (((TMR_REMAP) == TMR_TMR11_GPIO)          || \
                              ((TMR_REMAP) == TMR_TMR11_HSE))))
#endif /* LPTMR_OR_TMR1_ITR2_RMP &&  LPTMR_OR_TMR5_ITR1_RMP && LPTMR_OR_TMR5_ITR1_RMP */
#else
#define IS_TMR_REMAP(INSTANCE, TMR_REMAP)                                 \
  ((((INSTANCE) == TMR5)  && (((TMR_REMAP) == TMR_TMR5_GPIO)           || \
                              ((TMR_REMAP) == TMR_TMR5_LSI)            || \
                              ((TMR_REMAP) == TMR_TMR5_LSE)            || \
                              ((TMR_REMAP) == TMR_TMR5_RTC)))          || \
   (((INSTANCE) == TMR11) && (((TMR_REMAP) == TMR_TMR11_GPIO)          || \
                              ((TMR_REMAP) == TMR_TMR11_HSE))))
#endif /* SPDIFRX */

/**
  * @}
  */
/* End of private macro ------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup TMREx_Exported_Functions TMR Extended Exported Functions
  * @{
  */

/** @addtogroup TMREx_Exported_Functions_Group1 Extended Timer Hall Sensor functions
  *  @brief    Timer Hall Sensor functions
  * @{
  */
/*  Timer Hall Sensor functions  **********************************************/
DAL_StatusTypeDef DAL_TMREx_HallSensor_Init(TMR_HandleTypeDef *htim, TMR_HallSensor_InitTypeDef *sConfig);
DAL_StatusTypeDef DAL_TMREx_HallSensor_DeInit(TMR_HandleTypeDef *htim);

void DAL_TMREx_HallSensor_MspInit(TMR_HandleTypeDef *htim);
void DAL_TMREx_HallSensor_MspDeInit(TMR_HandleTypeDef *htim);

/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Start(TMR_HandleTypeDef *htim);
DAL_StatusTypeDef DAL_TMREx_HallSensor_Stop(TMR_HandleTypeDef *htim);
/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Start_IT(TMR_HandleTypeDef *htim);
DAL_StatusTypeDef DAL_TMREx_HallSensor_Stop_IT(TMR_HandleTypeDef *htim);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_TMREx_HallSensor_Start_DMA(TMR_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
DAL_StatusTypeDef DAL_TMREx_HallSensor_Stop_DMA(TMR_HandleTypeDef *htim);
/**
  * @}
  */

/** @addtogroup TMREx_Exported_Functions_Group2 Extended Timer Complementary Output Compare functions
  *  @brief   Timer Complementary Output Compare functions
  * @{
  */
/*  Timer Complementary Output Compare functions  *****************************/
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMREx_OCN_Start(TMR_HandleTypeDef *htim, uint32_t Channel);
DAL_StatusTypeDef DAL_TMREx_OCN_Stop(TMR_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMREx_OCN_Start_IT(TMR_HandleTypeDef *htim, uint32_t Channel);
DAL_StatusTypeDef DAL_TMREx_OCN_Stop_IT(TMR_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_TMREx_OCN_Start_DMA(TMR_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
DAL_StatusTypeDef DAL_TMREx_OCN_Stop_DMA(TMR_HandleTypeDef *htim, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TMREx_Exported_Functions_Group3 Extended Timer Complementary PWM functions
  *  @brief    Timer Complementary PWM functions
  * @{
  */
/*  Timer Complementary PWM functions  ****************************************/
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMREx_PWMN_Start(TMR_HandleTypeDef *htim, uint32_t Channel);
DAL_StatusTypeDef DAL_TMREx_PWMN_Stop(TMR_HandleTypeDef *htim, uint32_t Channel);

/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMREx_PWMN_Start_IT(TMR_HandleTypeDef *htim, uint32_t Channel);
DAL_StatusTypeDef DAL_TMREx_PWMN_Stop_IT(TMR_HandleTypeDef *htim, uint32_t Channel);
/* Non-Blocking mode: DMA */
DAL_StatusTypeDef DAL_TMREx_PWMN_Start_DMA(TMR_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
DAL_StatusTypeDef DAL_TMREx_PWMN_Stop_DMA(TMR_HandleTypeDef *htim, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup TMREx_Exported_Functions_Group4 Extended Timer Complementary One Pulse functions
  *  @brief    Timer Complementary One Pulse functions
  * @{
  */
/*  Timer Complementary One Pulse functions  **********************************/
/* Blocking mode: Polling */
DAL_StatusTypeDef DAL_TMREx_OnePulseN_Start(TMR_HandleTypeDef *htim, uint32_t OutputChannel);
DAL_StatusTypeDef DAL_TMREx_OnePulseN_Stop(TMR_HandleTypeDef *htim, uint32_t OutputChannel);

/* Non-Blocking mode: Interrupt */
DAL_StatusTypeDef DAL_TMREx_OnePulseN_Start_IT(TMR_HandleTypeDef *htim, uint32_t OutputChannel);
DAL_StatusTypeDef DAL_TMREx_OnePulseN_Stop_IT(TMR_HandleTypeDef *htim, uint32_t OutputChannel);
/**
  * @}
  */

/** @addtogroup TMREx_Exported_Functions_Group5 Extended Peripheral Control functions
  *  @brief    Peripheral Control functions
  * @{
  */
/* Extended Control functions  ************************************************/
DAL_StatusTypeDef DAL_TMREx_ConfigCommutEvent(TMR_HandleTypeDef *htim, uint32_t  InputTrigger,
                                              uint32_t  CommutationSource);
DAL_StatusTypeDef DAL_TMREx_ConfigCommutEvent_IT(TMR_HandleTypeDef *htim, uint32_t  InputTrigger,
                                                 uint32_t  CommutationSource);
DAL_StatusTypeDef DAL_TMREx_ConfigCommutEvent_DMA(TMR_HandleTypeDef *htim, uint32_t  InputTrigger,
                                                  uint32_t  CommutationSource);
DAL_StatusTypeDef DAL_TMREx_MasterConfigSynchronization(TMR_HandleTypeDef *htim,
                                                        TMR_MasterConfigTypeDef *sMasterConfig);
DAL_StatusTypeDef DAL_TMREx_ConfigBreakDeadTime(TMR_HandleTypeDef *htim,
                                                TMR_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
DAL_StatusTypeDef DAL_TMREx_RemapConfig(TMR_HandleTypeDef *htim, uint32_t Remap);
/**
  * @}
  */

/** @addtogroup TMREx_Exported_Functions_Group6 Extended Callbacks functions
  * @brief    Extended Callbacks functions
  * @{
  */
/* Extended Callback **********************************************************/
void DAL_TMREx_CommutCallback(TMR_HandleTypeDef *htim);
void DAL_TMREx_CommutHalfCpltCallback(TMR_HandleTypeDef *htim);
void DAL_TMREx_BreakCallback(TMR_HandleTypeDef *htim);
/**
  * @}
  */

/** @addtogroup TMREx_Exported_Functions_Group7 Extended Peripheral State functions
  * @brief    Extended Peripheral State functions
  * @{
  */
/* Extended Peripheral State functions  ***************************************/
DAL_TMR_StateTypeDef DAL_TMREx_HallSensor_GetState(TMR_HandleTypeDef *htim);
DAL_TMR_ChannelStateTypeDef DAL_TMREx_GetChannelNState(TMR_HandleTypeDef *htim,  uint32_t ChannelN);
/**
  * @}
  */

/**
  * @}
  */
/* End of exported functions -------------------------------------------------*/

/* Private functions----------------------------------------------------------*/
/** @addtogroup TMREx_Private_Functions TMR Extended Private Functions
  * @{
  */
void TMREx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TMREx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);
/**
  * @}
  */
/* End of private functions --------------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* APM32F4xx_DAL_TMR_EX_H */
