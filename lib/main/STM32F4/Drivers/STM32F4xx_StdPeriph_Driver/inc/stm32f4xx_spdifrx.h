/**
  ******************************************************************************
  * @file    stm32f4xx_spdifrx.h
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    20-May-2016
  * @brief   This file contains all the functions prototypes for the SPDIFRX firmware 
  *          library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_SPDIFRX_H
#define __STM32F4xx_SPDIFRX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup SPDIFRX
  * @{
  */ 
#if defined(STM32F446xx)
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  SPDIFRX Init structure definition  
  */
typedef struct
{
  uint32_t SPDIFRX_InputSelection;           /*!< Specifies the SPDIFRX input selection.
                                                This parameter can be a value of @ref SPDIFRX_Input_Selection */

  uint32_t SPDIFRX_Retries;                  /*!< Specifies the Maximum allowed re-tries during synchronization phase.
                                                This parameter can be a value of @ref SPDIFRX_Max_Retries */

  uint32_t SPDIFRX_WaitForActivity;          /*!< Specifies the wait for activity on SPDIFRX selected input.
                                                This parameter can be a value of @ref SPDIFRX_Wait_For_Activity. */

  uint32_t SPDIFRX_ChannelSelection;         /*!< Specifies whether the control flow will take the channel status from channel A or B.
                                                This parameter can be a value of @ref SPDIFRX_Channel_Selection */

  uint32_t SPDIFRX_DataFormat;               /*!< Specifies the Data samples format (LSB, MSB, ...).
                                                This parameter can be a value of @ref SPDIFRX_Data_Format */
                                               
  uint32_t SPDIFRX_StereoMode;               /*!< Specifies whether the peripheral is in stereo or mono mode.
                                                This parameter can be a value of @ref SPDIFRX_Stereo_Mode */ 
}SPDIFRX_InitTypeDef;


/* Exported constants --------------------------------------------------------*/

/** @defgroup SPDIFRX_Exported_Constants
  * @{
  */
#define IS_SPDIFRX_PERIPH(PERIPH) (((PERIPH) == SPDIFRX)) 

/** @defgroup SPDIFRX_Input_Selection SPDIFRX Input Selection
  * @{
  */
#define SPDIFRX_Input_IN0               ((uint32_t)0x00000000)
#define SPDIFRX_Input_IN1               ((uint32_t)0x00010000)  
#define SPDIFRX_Input_IN2               ((uint32_t)0x00020000)
#define SPDIFRX_Input_IN3               ((uint32_t)0x00030000)
#define IS_SPDIFRX_INPUT_SELECT(INPUT)  (((INPUT) == SPDIFRX_Input_IN1) || \
                                         ((INPUT) == SPDIFRX_Input_IN2) || \
                                         ((INPUT) == SPDIFRX_Input_IN3)  || \
                                         ((INPUT) == SPDIFRX_Input_IN0))
/**
  * @}
  */

/** @defgroup SPDIFRX_Max_Retries SPDIFRX Max Retries 
  * @{
  */
#define SPDIFRX_1MAX_RETRIES               ((uint32_t)0x00000000)
#define SPDIFRX_4MAX_RETRIES               ((uint32_t)0x00001000)  
#define SPDIFRX_16MAX_RETRIES              ((uint32_t)0x00002000)
#define SPDIFRX_64MAX_RETRIES              ((uint32_t)0x00003000)
#define IS_SPDIFRX_MAX_RETRIES(RET)   (((RET) == SPDIFRX_1MAX_RETRIES) || \
                                       ((RET) == SPDIFRX_4MAX_RETRIES) || \
                                       ((RET) == SPDIFRX_16MAX_RETRIES)  || \
                                       ((RET) == SPDIFRX_64MAX_RETRIES))
/**
  * @}
  */

/** @defgroup SPDIFRX_Wait_For_Activity SPDIFRX Wait For Activity
  * @{
  */
#define SPDIFRX_WaitForActivity_Off                 ((uint32_t)0x00000000)
#define SPDIFRX_WaitForActivity_On                  ((uint32_t)SPDIFRX_CR_WFA)
#define IS_SPDIFRX_WAIT_FOR_ACTIVITY(VAL)    (((VAL) == SPDIFRX_WaitForActivity_On) || \
                                              ((VAL) == SPDIFRX_WaitForActivity_Off))
/**
  * @}
  */

/** @defgroup SPDIFRX_ChannelSelection SPDIFRX Channel Selection
  * @{
  */
#define SPDIFRX_Select_Channel_A      ((uint32_t)0x00000000)
#define SPDIFRX_Select_Channel_B      ((uint32_t)SPDIFRX_CR_CHSEL)
#define IS_SPDIFRX_CHANNEL(CHANNEL)   (((CHANNEL) == SPDIFRX_Select_Channel_A) || \
                                       ((CHANNEL) == SPDIFRX_Select_Channel_B))
/**
  * @}
  */

/** @defgroup SPDIFRX_Block_Synchronization SPDIFRX Block Synchronization
  * @{
  */
#define SPDIFRX_LSB_DataFormat                   ((uint32_t)0x00000000)
#define SPDIFRX_MSB_DataFormat                   ((uint32_t)0x00000010)
#define SPDIFRX_32BITS_DataFormat                ((uint32_t)0x00000020)
#define IS_SPDIFRX_DATA_FORMAT(FORMAT)    (((FORMAT) == SPDIFRX_LSB_DataFormat) || \
                                           ((FORMAT) == SPDIFRX_MSB_DataFormat) || \
                                           ((FORMAT) == SPDIFRX_32BITS_DataFormat))
/**
  * @}
  */ 

/** @defgroup SPDIFRX_StereoMode SPDIFRX StereoMode
  * @{
  */
#define SPDIFRX_StereoMode_Disabled          ((uint32_t)0x00000000)
#define SPDIFRX_StereoMode_Enabled           ((uint32_t)SPDIFRX_CR_RXSTEO)
#define IS_STEREO_MODE(MODE)  (((MODE) == SPDIFRX_StereoMode_Disabled) || \
                               ((MODE) == SPDIFRX_StereoMode_Enabled))
/**
  * @}
  */ 

/** @defgroup SPDIFRX_State SPDIFRX State
  * @{
  */
#define SPDIFRX_STATE_IDLE    ((uint32_t)0x00000000)
#define SPDIFRX_STATE_SYNC    ((uint32_t)0x00000001)
#define SPDIFRX_STATE_RCV     ((uint32_t)SPDIFRX_CR_SPDIFEN)
#define IS_SPDIFRX_STATE(STATE)    (((STATE) == SPDIFRX_STATE_IDLE) || \
                                    ((STATE) == SPDIFRX_STATE_SYNC) || \
                                    ((STATE) == SPDIFRX_STATE_RCV))
/**
  * @}
  */
	
/** @defgroup SPDIFRX_Interrupts_Definition SPDIFRX Interrupts Definition
  * @{
  */
#define SPDIFRX_IT_RXNE                       ((uint32_t)SPDIFRX_IMR_RXNEIE)
#define SPDIFRX_IT_CSRNE                      ((uint32_t)SPDIFRX_IMR_CSRNEIE)
#define SPDIFRX_IT_PERRIE                     ((uint32_t)SPDIFRX_IMR_PERRIE)
#define SPDIFRX_IT_OVRIE                      ((uint32_t)SPDIFRX_IMR_OVRIE)
#define SPDIFRX_IT_SBLKIE                     ((uint32_t)SPDIFRX_IMR_SBLKIE)
#define SPDIFRX_IT_SYNCDIE                    ((uint32_t)SPDIFRX_IMR_SYNCDIE)
#define SPDIFRX_IT_IFEIE                      ((uint32_t)SPDIFRX_IMR_IFEIE )
#define IS_SPDIFRX_CONFIG_IT(IT)    (((IT) == SPDIFRX_IT_RXNE)   || \
                                     ((IT) == SPDIFRX_IT_CSRNE)   || \
                                     ((IT) == SPDIFRX_IT_PERRIE)  || \
                                     ((IT) == SPDIFRX_IT_OVRIE)   || \
                                     ((IT) == SPDIFRX_IT_SBLKIE)  || \
                                     ((IT) == SPDIFRX_IT_SYNCDIE) || \
                                     ((IT) == SPDIFRX_IT_IFEIE))
/**
  * @}
  */
	
/** @defgroup SPDIFRX_Flags_Definition SPDIFRX Flags Definition
  * @{
  */
#define SPDIFRX_FLAG_RXNE                   ((uint32_t)SPDIFRX_SR_RXNE)
#define SPDIFRX_FLAG_CSRNE                  ((uint32_t)SPDIFRX_SR_CSRNE)
#define SPDIFRX_FLAG_PERR                   ((uint32_t)SPDIFRX_SR_PERR)
#define SPDIFRX_FLAG_OVR                    ((uint32_t)SPDIFRX_SR_OVR)
#define SPDIFRX_FLAG_SBD                    ((uint32_t)SPDIFRX_SR_SBD)
#define SPDIFRX_FLAG_SYNCD                  ((uint32_t)SPDIFRX_SR_SYNCD)
#define SPDIFRX_FLAG_FERR                   ((uint32_t)SPDIFRX_SR_FERR)
#define SPDIFRX_FLAG_SERR                   ((uint32_t)SPDIFRX_SR_SERR)
#define SPDIFRX_FLAG_TERR                   ((uint32_t)SPDIFRX_SR_TERR)
#define IS_SPDIFRX_FLAG(FLAG)    (((FLAG) == SPDIFRX_FLAG_RXNE) || ((FLAG) == SPDIFRX_FLAG_CSRNE) || \
                                  ((FLAG) == SPDIFRX_FLAG_PERR) || ((FLAG) == SPDIFRX_FLAG_OVR) || \
                                  ((FLAG) == SPDIFRX_SR_SBD) || ((FLAG) == SPDIFRX_SR_SYNCD) || \
                                  ((FLAG) == SPDIFRX_SR_FERR) || ((FLAG) == SPDIFRX_SR_SERR) || \
				  ((FLAG) == SPDIFRX_SR_TERR))  
#define IS_SPDIFRX_CLEAR_FLAG(FLAG)    (((FLAG) == SPDIFRX_FLAG_PERR) || ((FLAG) == SPDIFRX_FLAG_OVR)   || \
                                        ((FLAG) == SPDIFRX_SR_SBD) || ((FLAG) == SPDIFRX_SR_SYNCD))  
/**
  * @}
  */
 	
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 

/*  Function used to set the SPDIFRX configuration to the default reset state *****/ 
void SPDIFRX_DeInit(void);

/* Initialization and Configuration functions *********************************/
void SPDIFRX_Init(SPDIFRX_InitTypeDef* SPDIFRX_InitStruct);
void SPDIFRX_StructInit(SPDIFRX_InitTypeDef* SPDIFRX_InitStruct);
void SPDIFRX_Cmd(uint32_t SPDIFRX_State);
void SPDIFRX_SetPreambleTypeBit(FunctionalState NewState);
void SPDIFRX_SetUserDataChannelStatusBits(FunctionalState NewState);
void SPDIFRX_SetValidityBit(FunctionalState NewState);
void SPDIFRX_SetParityBit(FunctionalState NewState);

/* Data transfers functions ***************************************************/ 
uint32_t SPDIFRX_ReceiveData(void);

/* DMA transfers management functions *****************************************/
void SPDIFRX_RxDMACmd(FunctionalState NewState);
void SPDIFRX_CbDMACmd(FunctionalState NewState);

/* Interrupts and flags management functions **********************************/
void SPDIFRX_ITConfig(uint32_t SPDIFRX_IT, FunctionalState NewState);
FlagStatus SPDIFRX_GetFlagStatus(uint32_t SPDIFRX_FLAG);
void SPDIFRX_ClearFlag(uint32_t SPDIFRX_FLAG);
ITStatus SPDIFRX_GetITStatus(uint32_t SPDIFRX_IT);
void SPDIFRX_ClearITPendingBit(uint32_t SPDIFRX_IT);

#endif /* STM32F446xx */
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__STM32F4xx_SPDIFRX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
