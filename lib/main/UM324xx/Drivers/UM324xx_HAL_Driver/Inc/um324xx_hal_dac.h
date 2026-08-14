 /**
  ******************************************************************************
  * @file     um324xx_hal_dac.h
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-18  
  * @brief   
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UM324XX_HAL_DAC_H__
#define __UM324XX_HAL_DAC_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal_def.h"

#if defined(DAC)

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @addtogroup DAC
  * @{
  */  

/* Exported typedefs ---------------------------------------------------------*/
/** @defgroup DAC_Exported_Types DAC Exported Types
  * @{
  */

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
  HAL_DAC_STATE_RESET             = 0x00U,  /*!< DAC not yet initialized or disabled  */
  HAL_DAC_STATE_READY             = 0x01U,  /*!< DAC initialized and ready for use    */
  HAL_DAC_STATE_BUSY              = 0x02U,  /*!< DAC internal processing is ongoing   */
  HAL_DAC_STATE_TIMEOUT           = 0x03U,  /*!< DAC timeout state                    */
  HAL_DAC_STATE_ERROR             = 0x04U   /*!< DAC error state                      */

} HAL_DAC_StateTypeDef;

/**
  * @brief  DAC handle Structure definition
  */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
typedef struct __DAC_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
{
  DAC_TypeDef                 *Instance;     /*!< Register base address             */

  __IO HAL_DAC_StateTypeDef   State;         /*!< DAC communication state           */

  
    
  HAL_LockTypeDef             Lock;          /*!< DAC locking object                */

  DMA_HandleTypeDef           *DMA_Handle0;  /*!< Pointer DMA handler for channel 0 */

  DMA_HandleTypeDef           *DMA_Handle1;  /*!< Pointer DMA handler for channel 1 */

  __IO uint32_t               ErrorCode;     /*!< DAC Error code                    */

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  void (* ConvCpltCallbackCh0)            (struct __DAC_HandleTypeDef *hdac);
  void (* ConvHalfCpltCallbackCh0)        (struct __DAC_HandleTypeDef *hdac);
  void (* ErrorCallbackCh0)               (struct __DAC_HandleTypeDef *hdac);
  void (* DMAUnderrunCallbackCh0)         (struct __DAC_HandleTypeDef *hdac);
#if defined(DAC_CHANNEL1_SUPPORT)
  void (* ConvCpltCallbackCh1)            (struct __DAC_HandleTypeDef *hdac);
  void (* ConvHalfCpltCallbackCh1)        (struct __DAC_HandleTypeDef *hdac);
  void (* ErrorCallbackCh1)               (struct __DAC_HandleTypeDef *hdac);
  void (* DMAUnderrunCallbackCh1)         (struct __DAC_HandleTypeDef *hdac);
#endif /* DAC_CHANNEL1_SUPPORT */

  void (* MspInitCallback)                (struct __DAC_HandleTypeDef *hdac);
  void (* MspDeInitCallback)              (struct __DAC_HandleTypeDef *hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

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

  uint32_t    Vrefset;
} DAC_ChannelConfTypeDef;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL DAC Callback ID enumeration definition
  */
typedef enum
{
  HAL_DAC_CH0_COMPLETE_CB_ID                 = 0x00U,  /*!< DAC CH0 Complete Callback ID      */
  HAL_DAC_CH0_HALF_COMPLETE_CB_ID            = 0x01U,  /*!< DAC CH0 half Complete Callback ID */
  HAL_DAC_CH0_ERROR_ID                       = 0x02U,  /*!< DAC CH0 error Callback ID         */
  HAL_DAC_CH0_UNDERRUN_CB_ID                 = 0x03U,  /*!< DAC CH0 underrun Callback ID      */
#if defined(DAC_CHANNEL1_SUPPORT)
  HAL_DAC_CH1_COMPLETE_CB_ID                 = 0x04U,  /*!< DAC CH1 Complete Callback ID      */
  HAL_DAC_CH1_HALF_COMPLETE_CB_ID            = 0x05U,  /*!< DAC CH1 half Complete Callback ID */
  HAL_DAC_CH1_ERROR_ID                       = 0x06U,  /*!< DAC CH1 error Callback ID         */
  HAL_DAC_CH1_UNDERRUN_CB_ID                 = 0x07U,  /*!< DAC CH1 underrun Callback ID      */
#endif /* DAC_CHANNEL1_SUPPORT */
  HAL_DAC_MSPINIT_CB_ID                      = 0x08U,  /*!< DAC MspInit Callback ID           */
  HAL_DAC_MSPDEINIT_CB_ID                    = 0x09U,  /*!< DAC MspDeInit Callback ID         */
  HAL_DAC_ALL_CB_ID                          = 0x0AU   /*!< DAC All ID                        */
} HAL_DAC_CallbackIDTypeDef;

/**
  * @brief  HAL DAC Callback pointer definition
  */
typedef void (*pDAC_CallbackTypeDef)(DAC_HandleTypeDef *hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */  

/* Exported constants --------------------------------------------------------*/
/** @defgroup DAC_Exported_constants DAC Exported Constants
  * @{
  */ 

/** @defgroup DAC_Error_Code DAC Error Code
  * @{
  */
#define  HAL_DAC_ERROR_NONE              0x00U    /*!< No error                          */
#define  HAL_DAC_ERROR_DMAUNDERRUNCH0    0x01U    /*!< DAC channel0 DMA underrun error   */
#if defined(DAC_CHANNEL1_SUPPORT)
#define  HAL_DAC_ERROR_DMAUNDERRUNCH1    0x02U    /*!< DAC channel1 DMA underrun error   */
#endif /* DAC_CHANNEL1_SUPPORT */
#define  HAL_DAC_ERROR_DMA               0x04U    /*!< DMA error                         */
#define  HAL_DAC_ERROR_TIMEOUT           0x08U    /*!< Timeout error                     */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
#define HAL_DAC_ERROR_INVALID_CALLBACK   0x10U    /*!< Invalid callback error            */
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DAC_Exported_macro DAC Exported Macro
  * @{
  */ 

/** @defgroup DAC_output_buffer DAC output buffer
  * @{
  */
#define DAC_OUTPUTBUFFER_DISABLE            0x00000000U
#define DAC_OUTPUTBUFFER_ENABLE           (DAC_CTRL_BUF0)

/**
  * @}
  */

/** @defgroup DAC_Channel_selection DAC Channel selection
  * @{
  */
#define DAC_CHANNEL_0                      0x00000000U
#if defined(DAC_CHANNEL1_SUPPORT)
#define DAC_CHANNEL_1                      0x00000010U
#endif /* DAC_CHANNEL1_SUPPORT */
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
#define DAC_FLAG_DMAIT0                   (DAC_IS_DMAIT0)
#if defined(DAC_CHANNEL1_SUPPORT)
#define DAC_FLAG_DMAIT1                   (DAC_IS_DMAIT1)
#endif /* DAC_CHANNEL1_SUPPORT */

/**
  * @}
  */

/** @defgroup DAC_IT_definition  DAC IT definition
  * @{
  */
#define DAC_IT_DMAIT0                   (DAC_CTRL_DMAIE0)
#if defined(DAC_CHANNEL1_SUPPORT)
#define DAC_IT_DMAIT1                   (DAC_CTRL_DMAIE1)
#endif /* DAC_CHANNEL1_SUPPORT */

/**
  * @}
  */
  
 
/** @defgroup DAC_VREF_definition  DAC VREF definition
  * @{
  */
#define  DAC_VREF_AVDD              0x00000000UL
#define  DAC_VREF_VREFP             DAC_CTRL_DAC0_VREF_MODE_1

/**
  * @}
  */
  
/** @defgroup DAC_trigger_selection DAC trigger selection
  * @{
  */
#define DAC_TRIGGER_NONE                0x00000000UL                                                     /*!< Conversion is automatic once the DAC0_DHRxxxx register has been loaded, and not by external trigger */
#define DAC_TRIGGER_T5_TRGO             (DAC_CTRL_TEN0)                                                  /*!< TIM5 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T7_TRGO             (DAC_CTRL_TSEL0_0 | DAC_CTRL_TEN0)                               /*!< TIM7 TRGO selected as external conversion trigger for DAC channel */
#if defined(UM324xF)                                                                                     
#define DAC_TRIGGER_T6_TRGO             (DAC_CTRL_TSEL0_1 | DAC_CTRL_TEN0)                               /*!< TIM6 TRGO selected as external conversion trigger for DAC channel */
#endif                                                                                                   
#if defined(UM32x42x)                                                                                    
#define DAC_TRIGGER_T2_TRGO             (DAC_CTRL_TSEL0_1 | DAC_CTRL_TEN0)                               /*!< TIM2 TRGO selected as external conversion trigger for DAC channel */
#endif
#define DAC_TRIGGER_T4_TRGO             (DAC_CTRL_TSEL0_1 | DAC_CTRL_TSEL0_0 | DAC_CTRL_TEN0)            /*!< TIM4 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T1_TRGO             (DAC_CTRL_TSEL0_2                                       | DAC_CTRL_TEN0) /*!< TIM1 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T3_TRGO             (DAC_CTRL_TSEL0_2 |                    DAC_CTRL_TSEL0_0 | DAC_CTRL_TEN0) /*!< TIM3 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_EXT_IT              (DAC_CTRL_TSEL0_2 | DAC_CTRL_TSEL0_1                    | DAC_CTRL_TEN0) /*!< EXTI Line8 event selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_SOFTWARE            (DAC_CTRL_TSEL0_2 | DAC_CTRL_TSEL0_1 | DAC_CTRL_TSEL0_0 | DAC_CTRL_TEN0) /*!< Conversion started by software trigger for DAC channel */

/**
  * @}
  */
  
/** @defgroup DACEx_lfsrunmask_triangleamplitude DACEx lfsrunmask triangle amplitude
  * @{
  */
#define DAC_LFSRUNMASK_BIT0                0x00000000UL                                              /*!< Unmask DAC channel LFSR bit0 for noise wave generation */
#define DAC_LFSRUNMASK_BITS1_0             (DAC_CTRL_MAMP0_0)                                        /*!< Unmask DAC channel LFSR bit[1:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS2_0             (DAC_CTRL_MAMP0_1)                                        /*!< Unmask DAC channel LFSR bit[2:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS3_0             (DAC_CTRL_MAMP0_1 | DAC_CTRL_MAMP0_0)                     /*!< Unmask DAC channel LFSR bit[3:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS4_0             (DAC_CTRL_MAMP0_2)                                        /*!< Unmask DAC channel LFSR bit[4:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS5_0             (DAC_CTRL_MAMP0_2 | DAC_CTRL_MAMP0_0)                     /*!< Unmask DAC channel LFSR bit[5:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS6_0             (DAC_CTRL_MAMP0_2 | DAC_CTRL_MAMP0_1                   )  /*!< Unmask DAC channel LFSR bit[6:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS7_0             (DAC_CTRL_MAMP0_2 | DAC_CTRL_MAMP0_1 | DAC_CTRL_MAMP0_0)  /*!< Unmask DAC channel LFSR bit[7:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS8_0             (DAC_CTRL_MAMP0_3 )                                       /*!< Unmask DAC channel LFSR bit[8:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS9_0             (DAC_CTRL_MAMP0_3 | DAC_CTRL_MAMP0_0)                     /*!< Unmask DAC channel LFSR bit[9:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS10_0            (DAC_CTRL_MAMP0_3 | DAC_CTRL_MAMP0_1 )                    /*!< Unmask DAC channel LFSR bit[10:0] for noise wave generation */
#define DAC_LFSRUNMASK_BITS11_0            (DAC_CTRL_MAMP0_3 | DAC_CTRL_MAMP0_1 | DAC_CTRL_MAMP0_0)  /*!< Unmask DAC channel LFSR bit[11:0] for noise wave generation */

#define DAC_TRIANGLEAMPLITUDE_1            0x00000000UL                                              /*!< Select max triangle amplitude of 1 */
#define DAC_TRIANGLEAMPLITUDE_3            (DAC_CTRL_MAMP0_0)                                        /*!< Select max triangle amplitude of 3 */
#define DAC_TRIANGLEAMPLITUDE_7            (DAC_CTRL_MAMP0_1)                                        /*!< Select max triangle amplitude of 7 */
#define DAC_TRIANGLEAMPLITUDE_15           (DAC_CTRL_MAMP0_1 | DAC_CTRL_MAMP0_0)                     /*!< Select max triangle amplitude of 15 */
#define DAC_TRIANGLEAMPLITUDE_31           (DAC_CTRL_MAMP0_2 )                                       /*!< Select max triangle amplitude of 31 */
#define DAC_TRIANGLEAMPLITUDE_63           (DAC_CTRL_MAMP0_2 | DAC_CTRL_MAMP0_0)                     /*!< Select max triangle amplitude of 63 */
#define DAC_TRIANGLEAMPLITUDE_127          (DAC_CTRL_MAMP0_2 | DAC_CTRL_MAMP0_1)                     /*!< Select max triangle amplitude of 127 */
#define DAC_TRIANGLEAMPLITUDE_255          (DAC_CTRL_MAMP0_2 | DAC_CTRL_MAMP0_1 | DAC_CTRL_MAMP0_0)  /*!< Select max triangle amplitude of 255 */
#define DAC_TRIANGLEAMPLITUDE_511          (DAC_CTRL_MAMP0_3)                                        /*!< Select max triangle amplitude of 511 */
#define DAC_TRIANGLEAMPLITUDE_1023         (DAC_CTRL_MAMP0_3 | DAC_CTRL_MAMP0_0)                     /*!< Select max triangle amplitude of 1023 */
#define DAC_TRIANGLEAMPLITUDE_2047         (DAC_CTRL_MAMP0_3 | DAC_CTRL_MAMP0_1)                     /*!< Select max triangle amplitude of 2047 */
#define DAC_TRIANGLEAMPLITUDE_4095         (DAC_CTRL_MAMP0_3 | DAC_CTRL_MAMP0_1 | DAC_CTRL_MAMP0_0)  /*!< Select max triangle amplitude of 4095 */

/**
  * @}
  */

/** @brief Set DHR12R0 alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12R0_ALIGNMENT(__ALIGNMENT__) (0x00000008UL + (__ALIGNMENT__))

#if defined(DAC_CHANNEL1_SUPPORT)
/** @brief  Set DHR12R1 alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12R1_ALIGNMENT(__ALIGNMENT__) (0x00000014UL + (__ALIGNMENT__))
#endif /* DAC_CHANNEL1_SUPPORT */

/** @brief  Set DHR12RD alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12RD_ALIGNMENT(__ALIGNMENT__) (0x00000020UL + (__ALIGNMENT__))

/** @brief Enable the DAC channel.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __DAC_Channe0__ specifies the DAC channel
  * @retval None
  */
#define __HAL_DAC_ENABLE(__HANDLE__, __DAC_Channel__) \
  ((__HANDLE__)->Instance->CTRL |=  (DAC_CTRL_DACEN0 << ((__DAC_Channel__) & 0x10UL)))

/** @brief Disable the DAC channel.
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __DAC_Channe1__ specifies the DAC channel.
  * @retval None
  */
#define __HAL_DAC_DISABLE(__HANDLE__, __DAC_Channel__) \
  ((__HANDLE__)->Instance->CTRL &=  ~(DAC_CTRL_DACEN0 << ((__DAC_Channel__) & 0x10UL)))

/** @brief Enable the DAC interrupt.
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAIT0 DAC channel 0 DMA interrupt
  *            @arg DAC_IT_DMAIT1 DAC channel 1 DMA interrupt
  * @retval None
  */
#define __HAL_DAC_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL) |= (__INTERRUPT__))

/** @brief  Check whether the specified DAC interrupt source is enabled or not.
  * @param __HANDLE__ DAC handle
  * @param __INTERRUPT__ DAC interrupt source to check
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAIE0 DAC channel 0 DMA interrupt
  *            @arg DAC_IT_DMAIE1 DAC channel 1 DMA interrupt
  * @retval State of interruption (SET or RESET)
  */
#define __HAL_DAC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CTRL\
                                                             & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Get the selected DAC's flag status.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to get.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAIT0 DAC channel 0 DMA IT flag
  *            @arg DAC_FLAG_DMAIT1 DAC channel 1 DMA IT flag
  * @retval None
  */
#define __HAL_DAC_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->IS) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the DAC's flag.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAIT0 DAC channel 0 DMA IT flag
  *            @arg DAC_FLAG_DMAIT1 DAC channel 1 DMA IT flag
  * @retval None
  */
#define __HAL_DAC_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->IS) = (__FLAG__))

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup DAC_Exported_Functions
  * @{
  */ 

/* Private macros ------------------------------------------------------------*/
/** @defgroup DAC_Private_Macros 
  * @{
  */
  
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup DAC_Private_Functions DAC Private Functions
  * @{
  */  
/** @addtogroup DAC_Exported_Functions_Group1
  * @{
  */

/* Initialization and de-initialization functions *****************************/
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef *hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac);

/**
  * @}
  */
  
/** @addtogroup DAC_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *****************************************************/
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel);
void HAL_DAC_IRQHandler(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);
HAL_StatusTypeDef HAL_DACEx_TriangleWaveGenerate(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Amplitude);
HAL_StatusTypeDef HAL_DACEx_NoiseWaveGenerate(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Amplitude);

HAL_StatusTypeDef HAL_DACEx_DualStart(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DACEx_DualStop(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DACEx_DualSetValue(DAC_HandleTypeDef *hdac, uint32_t Alignment, uint32_t Data0, uint32_t Data1);
uint32_t HAL_DACEx_DualGetValue(DAC_HandleTypeDef *hdac);

void HAL_DAC_ConvCpltCallbackCh0(DAC_HandleTypeDef *hdac);
void HAL_DAC_ErrorCallbackCh0(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh0(DAC_HandleTypeDef *hdac);

#if defined(DAC_CHANNEL1_SUPPORT)
void HAL_DACEx_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DACEx_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DACEx_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);
#endif

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/* DAC callback registering/unregistering */
HAL_StatusTypeDef     HAL_DAC_RegisterCallback(DAC_HandleTypeDef *hdac, HAL_DAC_CallbackIDTypeDef CallbackID,
                                               pDAC_CallbackTypeDef pCallback);
HAL_StatusTypeDef     HAL_DAC_UnRegisterCallback(DAC_HandleTypeDef *hdac, HAL_DAC_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */
  
/** @addtogroup DAC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions ***********************************************/
uint32_t HAL_DAC_GetValue(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac, DAC_ChannelConfTypeDef *sConfig, uint32_t Channel);
/**
  * @}
  */
  
/** @addtogroup DAC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_DAC_StateTypeDef HAL_DAC_GetState(DAC_HandleTypeDef *hdac);
uint32_t HAL_DAC_GetError(DAC_HandleTypeDef *hdac);

/**
  * @}
  */
  
/**
  * @}
  */

/** @defgroup DAC_Private_Functions DAC Private Functions
  * @{
  */
void DAC_DMAConvCpltCh0(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh0(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/** @addtogroup DACEx_Private_Functions
  * @{
  */
#if defined(DAC_CHANNEL1_SUPPORT)
/* DAC_DMAConvCpltCh1 / DAC_DMAErrorCh1 */
/* are called by HAL_DAC_Start_DMA */
void DAC_DMAConvCpltCh1(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh1(DMA_HandleTypeDef *hdma);
#endif /* DAC_CHANNEL1_SUPPORT */
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

#endif

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
