/**
  *
  * @file    apm32f4xx_dal_dac.h
  * @brief   Header file of DAC DAL module.
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
#ifndef APM32F4xx_DAL_DAC_H
#define APM32F4xx_DAL_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

#if defined(DAC)

/** @addtogroup DAC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Types DAC Exported Types
  * @{
  */

/**
  * @brief  DAL State structures definition
  */
typedef enum
{
  DAL_DAC_STATE_RESET             = 0x00U,  /*!< DAC not yet initialized or disabled  */
  DAL_DAC_STATE_READY             = 0x01U,  /*!< DAC initialized and ready for use    */
  DAL_DAC_STATE_BUSY              = 0x02U,  /*!< DAC internal processing is ongoing   */
  DAL_DAC_STATE_TIMEOUT           = 0x03U,  /*!< DAC timeout state                    */
  DAL_DAC_STATE_ERROR             = 0x04U   /*!< DAC error state                      */

} DAL_DAC_StateTypeDef;

/**
  * @brief  DAC handle Structure definition
  */
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
typedef struct __DAC_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */
{
  DAC_TypeDef                 *Instance;     /*!< Register base address             */

  __IO DAL_DAC_StateTypeDef   State;         /*!< DAC communication state           */

  DAL_LockTypeDef             Lock;          /*!< DAC locking object                */

  DMA_HandleTypeDef           *DMA_Handle1;  /*!< Pointer DMA handler for channel 1 */

  DMA_HandleTypeDef           *DMA_Handle2;  /*!< Pointer DMA handler for channel 2 */

  __IO uint32_t               ErrorCode;     /*!< DAC Error code                    */

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
  void (* ConvCpltCallbackCh1)            (struct __DAC_HandleTypeDef *hdac);
  void (* ConvHalfCpltCallbackCh1)        (struct __DAC_HandleTypeDef *hdac);
  void (* ErrorCallbackCh1)               (struct __DAC_HandleTypeDef *hdac);
  void (* DMAUnderrunCallbackCh1)         (struct __DAC_HandleTypeDef *hdac);
#if defined(DAC_CHANNEL2_SUPPORT)
  void (* ConvCpltCallbackCh2)            (struct __DAC_HandleTypeDef *hdac);
  void (* ConvHalfCpltCallbackCh2)        (struct __DAC_HandleTypeDef *hdac);
  void (* ErrorCallbackCh2)               (struct __DAC_HandleTypeDef *hdac);
  void (* DMAUnderrunCallbackCh2)         (struct __DAC_HandleTypeDef *hdac);
#endif /* DAC_CHANNEL2_SUPPORT */

  void (* MspInitCallback)                (struct __DAC_HandleTypeDef *hdac);
  void (* MspDeInitCallback)              (struct __DAC_HandleTypeDef *hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

} DAC_HandleTypeDef;

/**
  * @brief   DAC Configuration regular Channel structure definition
  */
typedef struct
{
  uint32_t DAC_Trigger;                  /*!< Specifies the external trigger for the selected DAC channel.
                                              This parameter can be a value of @ref DAC_trigger_selection */

  uint32_t DAC_OutputBuffer;             /*!< Specifies whether the DAC channel output buffer is enabled or disabled.
                                               This parameter can be a value of @ref DAC_output_buffer */

} DAC_ChannelConfTypeDef;

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL DAC Callback ID enumeration definition
  */
typedef enum
{
  DAL_DAC_CH1_COMPLETE_CB_ID                 = 0x00U,  /*!< DAC CH1 Complete Callback ID      */
  DAL_DAC_CH1_HALF_COMPLETE_CB_ID            = 0x01U,  /*!< DAC CH1 half Complete Callback ID */
  DAL_DAC_CH1_ERROR_ID                       = 0x02U,  /*!< DAC CH1 error Callback ID         */
  DAL_DAC_CH1_UNDERRUN_CB_ID                 = 0x03U,  /*!< DAC CH1 underrun Callback ID      */
#if defined(DAC_CHANNEL2_SUPPORT)
  DAL_DAC_CH2_COMPLETE_CB_ID                 = 0x04U,  /*!< DAC CH2 Complete Callback ID      */
  DAL_DAC_CH2_HALF_COMPLETE_CB_ID            = 0x05U,  /*!< DAC CH2 half Complete Callback ID */
  DAL_DAC_CH2_ERROR_ID                       = 0x06U,  /*!< DAC CH2 error Callback ID         */
  DAL_DAC_CH2_UNDERRUN_CB_ID                 = 0x07U,  /*!< DAC CH2 underrun Callback ID      */
#endif /* DAC_CHANNEL2_SUPPORT */
  DAL_DAC_MSPINIT_CB_ID                      = 0x08U,  /*!< DAC MspInit Callback ID           */
  DAL_DAC_MSPDEINIT_CB_ID                    = 0x09U,  /*!< DAC MspDeInit Callback ID         */
  DAL_DAC_ALL_CB_ID                          = 0x0AU   /*!< DAC All ID                        */
} DAL_DAC_CallbackIDTypeDef;

/**
  * @brief  DAL DAC Callback pointer definition
  */
typedef void (*pDAC_CallbackTypeDef)(DAC_HandleTypeDef *hdac);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup DAC_Exported_Constants DAC Exported Constants
  * @{
  */

/** @defgroup DAC_Error_Code DAC Error Code
  * @{
  */
#define  DAL_DAC_ERROR_NONE              0x00U    /*!< No error                          */
#define  DAL_DAC_ERROR_DMAUNDERRUNCH1    0x01U    /*!< DAC channel1 DMA underrun error   */
#if defined(DAC_CHANNEL2_SUPPORT)
#define  DAL_DAC_ERROR_DMAUNDERRUNCH2    0x02U    /*!< DAC channel2 DMA underrun error   */
#endif /* DAC_CHANNEL2_SUPPORT */
#define  DAL_DAC_ERROR_DMA               0x04U    /*!< DMA error                         */
#define  DAL_DAC_ERROR_TIMEOUT           0x08U    /*!< Timeout error                     */
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
#define DAL_DAC_ERROR_INVALID_CALLBACK   0x10U    /*!< Invalid callback error            */
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup DAC_trigger_selection DAC trigger selection
  * @{
  */
#define DAC_TRIGGER_NONE                0x00000000UL                                                     /*!< Conversion is automatic once the DAC1_DHRxxxx register has been loaded, and not by external trigger */
#define DAC_TRIGGER_T2_TRGO             (DAC_CTRL_TRGSELCH1_2                                   | DAC_CTRL_TRGENCH1) /*!< TIM2 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T4_TRGO             (DAC_CTRL_TRGSELCH1_2                  | DAC_CTRL_TRGSELCH1_0 | DAC_CTRL_TRGENCH1) /*!< TIM4 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T5_TRGO             (                 DAC_CTRL_TRGSELCH1_1 | DAC_CTRL_TRGSELCH1_0 | DAC_CTRL_TRGENCH1) /*!< TIM3 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T6_TRGO             (                                                   DAC_CTRL_TRGENCH1) /*!< Conversion started by software trigger for DAC channel */
#define DAC_TRIGGER_T7_TRGO             (                 DAC_CTRL_TRGSELCH1_1                  | DAC_CTRL_TRGENCH1) /*!< TIM7 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T8_TRGO             (                                  DAC_CTRL_TRGSELCH1_0 | DAC_CTRL_TRGENCH1) /*!< TIM8 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_EXT_IT9             (DAC_CTRL_TRGSELCH1_2 | DAC_CTRL_TRGSELCH1_1                  | DAC_CTRL_TRGENCH1) /*!< EXTI Line9 event selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_SOFTWARE            (DAC_CTRL_TRGSELCH1                                     | DAC_CTRL_TRGENCH1) /*!< Conversion started by software trigger for DAC channel */

/**
  * @}
  */

/** @defgroup DAC_output_buffer DAC output buffer
  * @{
  */
#define DAC_OUTPUTBUFFER_ENABLE            0x00000000U
#define DAC_OUTPUTBUFFER_DISABLE           (DAC_CTRL_BUFFDCH1)

/**
  * @}
  */

/** @defgroup DAC_Channel_selection DAC Channel selection
  * @{
  */
#define DAC_CHANNEL_1                      0x00000000U
#if defined(DAC_CHANNEL2_SUPPORT)
#define DAC_CHANNEL_2                      0x00000010U
#endif /* DAC_CHANNEL2_SUPPORT */
/**
  * @}
  */

/** @defgroup DAC_data_alignment DAC data alignment
  * @{
  */
#define DAC_ALIGN_12B_R                    0x00000000U
#define DAC_ALIGN_12B_L                    0x00000004U
#define DAC_ALIGN_8B_R                     0x00000008U

/**
  * @}
  */

/** @defgroup DAC_flags_definition DAC flags definition
  * @{
  */
#define DAC_FLAG_DMAUDR1                   (DAC_STS_DMAUDFLG1)
#if defined(DAC_CHANNEL2_SUPPORT)
#define DAC_FLAG_DMAUDR2                   (DAC_STS_DMAUDFLG2)
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @}
  */

/** @defgroup DAC_IT_definition  DAC IT definition
  * @{
  */
#define DAC_IT_DMAUDR1                   (DAC_STS_DMAUDFLG1)
#if defined(DAC_CHANNEL2_SUPPORT)
#define DAC_IT_DMAUDR2                   (DAC_STS_DMAUDFLG2)
#endif /* DAC_CHANNEL2_SUPPORT */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Macros DAC Exported Macros
  * @{
  */

/** @brief Reset DAC handle state.
  * @param  __HANDLE__ specifies the DAC handle.
  * @retval None
  */
#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
#define __DAL_DAC_RESET_HANDLE_STATE(__HANDLE__) do {                                                        \
                                                      (__HANDLE__)->State             = DAL_DAC_STATE_RESET; \
                                                      (__HANDLE__)->MspInitCallback   = NULL;                \
                                                      (__HANDLE__)->MspDeInitCallback = NULL;                \
                                                     } while(0)
#else
#define __DAL_DAC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_DAC_STATE_RESET)
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

/** @brief Enable the DAC channel.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __DAC_Channel__ specifies the DAC channel
  * @retval None
  */
#define __DAL_DAC_ENABLE(__HANDLE__, __DAC_Channel__) \
  ((__HANDLE__)->Instance->CTRL |=  (DAC_CTRL_ENCH1 << ((__DAC_Channel__) & 0x10UL)))

/** @brief Disable the DAC channel.
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __DAC_Channel__ specifies the DAC channel.
  * @retval None
  */
#define __DAL_DAC_DISABLE(__HANDLE__, __DAC_Channel__) \
  ((__HANDLE__)->Instance->CTRL &=  ~(DAC_CTRL_ENCH1 << ((__DAC_Channel__) & 0x10UL)))

/** @brief Set DHR12R1 alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DH12R1_ALIGNMENT(__ALIGNMENT__) (0x00000008UL + (__ALIGNMENT__))

#if defined(DAC_CHANNEL2_SUPPORT)
/** @brief  Set DHR12R2 alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DH12R2_ALIGNMENT(__ALIGNMENT__) (0x00000014UL + (__ALIGNMENT__))
#endif /* DAC_CHANNEL2_SUPPORT */

/** @brief  Set DHR12RD alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DH12RDUAL_ALIGNMENT(__ALIGNMENT__) (0x00000020UL + (__ALIGNMENT__))

/** @brief Enable the DAC interrupt.
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1 DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2 DAC channel 2 DMA underrun interrupt
  * @retval None
  */
#define __DAL_DAC_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL) |= (__INTERRUPT__))

/** @brief Disable the DAC interrupt.
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1 DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2 DAC channel 2 DMA underrun interrupt
  * @retval None
  */
#define __DAL_DAC_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL) &= ~(__INTERRUPT__))

/** @brief  Check whether the specified DAC interrupt source is enabled or not.
  * @param __HANDLE__ DAC handle
  * @param __INTERRUPT__ DAC interrupt source to check
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1 DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2 DAC channel 2 DMA underrun interrupt
  * @retval State of interruption (SET or RESET)
  */
#define __DAL_DAC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL\
                                                             & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Get the selected DAC's flag status.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to get.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1 DAC channel 1 DMA underrun flag
  *            @arg DAC_FLAG_DMAUDR2 DAC channel 2 DMA underrun flag
  * @retval None
  */
#define __DAL_DAC_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->STS) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the DAC's flag.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1 DAC channel 1 DMA underrun flag
  *            @arg DAC_FLAG_DMAUDR2 DAC channel 2 DMA underrun flag
  * @retval None
  */
#define __DAL_DAC_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->STS) = (__FLAG__))

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/

/** @defgroup DAC_Private_Macros DAC Private Macros
  * @{
  */
#define IS_DAC_OUTPUT_BUFFER_STATE(STATE) (((STATE) == DAC_OUTPUTBUFFER_ENABLE) || \
                                           ((STATE) == DAC_OUTPUTBUFFER_DISABLE))

#if defined(DAC_CHANNEL2_SUPPORT)
#define IS_DAC_CHANNEL(CHANNEL) (((CHANNEL) == DAC_CHANNEL_1) || \
                                 ((CHANNEL) == DAC_CHANNEL_2))
#else
#define IS_DAC_CHANNEL(CHANNEL)  ((CHANNEL) == DAC_CHANNEL_1)
#endif /* DAC_CHANNEL2_SUPPORT */

#define IS_DAC_ALIGN(ALIGN) (((ALIGN) == DAC_ALIGN_12B_R) || \
                             ((ALIGN) == DAC_ALIGN_12B_L) || \
                             ((ALIGN) == DAC_ALIGN_8B_R))

#define IS_DAC_DATA(DATA) ((DATA) <= 0xFFF0UL)

/**
  * @}
  */

/* Include DAC DAL Extended module */
#include "apm32f4xx_dal_dac_ex.h"

/* Exported functions --------------------------------------------------------*/

/** @addtogroup DAC_Exported_Functions
  * @{
  */

/** @addtogroup DAC_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
DAL_StatusTypeDef DAL_DAC_Init(DAC_HandleTypeDef *hdac);
DAL_StatusTypeDef DAL_DAC_DeInit(DAC_HandleTypeDef *hdac);
void DAL_DAC_MspInit(DAC_HandleTypeDef *hdac);
void DAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac);

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *****************************************************/
DAL_StatusTypeDef DAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel);
DAL_StatusTypeDef DAL_DAC_Stop(DAC_HandleTypeDef *hdac, uint32_t Channel);
DAL_StatusTypeDef DAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment);
DAL_StatusTypeDef DAL_DAC_Stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel);
void DAL_DAC_IRQHandler(DAC_HandleTypeDef *hdac);
DAL_StatusTypeDef DAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);

void DAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void DAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void DAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void DAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);

#if (USE_DAL_DAC_REGISTER_CALLBACKS == 1)
/* DAC callback registering/unregistering */
DAL_StatusTypeDef     DAL_DAC_RegisterCallback(DAC_HandleTypeDef *hdac, DAL_DAC_CallbackIDTypeDef CallbackID,
                                               pDAC_CallbackTypeDef pCallback);
DAL_StatusTypeDef     DAL_DAC_UnRegisterCallback(DAC_HandleTypeDef *hdac, DAL_DAC_CallbackIDTypeDef CallbackID);
#endif /* USE_DAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions ***********************************************/
uint32_t DAL_DAC_GetValue(DAC_HandleTypeDef *hdac, uint32_t Channel);
DAL_StatusTypeDef DAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac, DAC_ChannelConfTypeDef *sConfig, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State and Error functions ***************************************/
DAL_DAC_StateTypeDef DAL_DAC_GetState(DAC_HandleTypeDef *hdac);
uint32_t DAL_DAC_GetError(DAC_HandleTypeDef *hdac);

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup DAC_Private_Functions DAC Private Functions
  * @{
  */
void DAC_DMAConvCpltCh1(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh1(DMA_HandleTypeDef *hdma);
void DAC_DMAHalfConvCpltCh1(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/**
  * @}
  */

#endif /* DAC */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* APM32F4xx_DAL_DAC_H */

