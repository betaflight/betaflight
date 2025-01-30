/**
  *
  * @file    apm32f4xx_dal_sram.h
  * @brief   Header file of SRAM DAL module.
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
#ifndef APM32F4xx_DAL_SRAM_H
#define APM32F4xx_DAL_SRAM_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(SMC_Bank1)

/* Includes ------------------------------------------------------------------*/
#if defined(SMC_Bank1)
#include "apm32f4xx_ddl_smc.h"
#endif /* SMC_Bank1 */

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */
/** @addtogroup SRAM
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup SRAM_Exported_Types SRAM Exported Types
  * @{
  */
/**
  * @brief  DAL SRAM State structures definition
  */
typedef enum
{
  DAL_SRAM_STATE_RESET     = 0x00U,  /*!< SRAM not yet initialized or disabled           */
  DAL_SRAM_STATE_READY     = 0x01U,  /*!< SRAM initialized and ready for use             */
  DAL_SRAM_STATE_BUSY      = 0x02U,  /*!< SRAM internal process is ongoing               */
  DAL_SRAM_STATE_ERROR     = 0x03U,  /*!< SRAM error state                               */
  DAL_SRAM_STATE_PROTECTED = 0x04U   /*!< SRAM peripheral NORSRAM device write protected */

} DAL_SRAM_StateTypeDef;

/**
  * @brief  SRAM handle Structure definition
  */
#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
typedef struct __SRAM_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS  */
{
  FMC_NORSRAM_TypeDef           *Instance;  /*!< Register base address                        */

  FMC_NORSRAM_EXTENDED_TypeDef  *Extended;  /*!< Extended mode register base address          */

  FMC_NORSRAM_InitTypeDef       Init;       /*!< SRAM device control configuration parameters */

  DAL_LockTypeDef               Lock;       /*!< SRAM locking object                          */

  __IO DAL_SRAM_StateTypeDef    State;      /*!< SRAM device access state                     */

  DMA_HandleTypeDef             *hdma;      /*!< Pointer DMA handler                          */

#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
  void (* MspInitCallback)(struct __SRAM_HandleTypeDef *hsram);               /*!< SRAM Msp Init callback              */
  void (* MspDeInitCallback)(struct __SRAM_HandleTypeDef *hsram);             /*!< SRAM Msp DeInit callback            */
  void (* DmaXferCpltCallback)(DMA_HandleTypeDef *hdma);                      /*!< SRAM DMA Xfer Complete callback     */
  void (* DmaXferErrorCallback)(DMA_HandleTypeDef *hdma);                     /*!< SRAM DMA Xfer Error callback        */
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS  */
} SRAM_HandleTypeDef;

#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL SRAM Callback ID enumeration definition
  */
typedef enum
{
  DAL_SRAM_MSP_INIT_CB_ID       = 0x00U,  /*!< SRAM MspInit Callback ID           */
  DAL_SRAM_MSP_DEINIT_CB_ID     = 0x01U,  /*!< SRAM MspDeInit Callback ID         */
  DAL_SRAM_DMA_XFER_CPLT_CB_ID  = 0x02U,  /*!< SRAM DMA Xfer Complete Callback ID */
  DAL_SRAM_DMA_XFER_ERR_CB_ID   = 0x03U   /*!< SRAM DMA Xfer Complete Callback ID */
} DAL_SRAM_CallbackIDTypeDef;

/**
  * @brief  DAL SRAM Callback pointer definition
  */
typedef void (*pSRAM_CallbackTypeDef)(SRAM_HandleTypeDef *hsram);
typedef void (*pSRAM_DmaCallbackTypeDef)(DMA_HandleTypeDef *hdma);
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS  */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup SRAM_Exported_Macros SRAM Exported Macros
  * @{
  */

/** @brief Reset SRAM handle state
  * @param  __HANDLE__ SRAM handle
  * @retval None
  */
#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
#define __DAL_SRAM_RESET_HANDLE_STATE(__HANDLE__)         do {                                             \
                                                               (__HANDLE__)->State = DAL_SRAM_STATE_RESET; \
                                                               (__HANDLE__)->MspInitCallback = NULL;       \
                                                               (__HANDLE__)->MspDeInitCallback = NULL;     \
                                                             } while(0)
#else
#define __DAL_SRAM_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_SRAM_STATE_RESET)
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SRAM_Exported_Functions SRAM Exported Functions
  * @{
  */

/** @addtogroup SRAM_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/* Initialization/de-initialization functions  ********************************/
DAL_StatusTypeDef DAL_SRAM_Init(SRAM_HandleTypeDef *hsram, FMC_NORSRAM_TimingTypeDef *Timing,
                                FMC_NORSRAM_TimingTypeDef *ExtTiming);
DAL_StatusTypeDef DAL_SRAM_DeInit(SRAM_HandleTypeDef *hsram);
void DAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram);
void DAL_SRAM_MspDeInit(SRAM_HandleTypeDef *hsram);

/**
  * @}
  */

/** @addtogroup SRAM_Exported_Functions_Group2 Input Output and memory control functions
  * @{
  */

/* I/O operation functions  ***************************************************/
DAL_StatusTypeDef DAL_SRAM_Read_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pDstBuffer,
                                   uint32_t BufferSize);
DAL_StatusTypeDef DAL_SRAM_Write_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pSrcBuffer,
                                    uint32_t BufferSize);
DAL_StatusTypeDef DAL_SRAM_Read_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pDstBuffer,
                                    uint32_t BufferSize);
DAL_StatusTypeDef DAL_SRAM_Write_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pSrcBuffer,
                                     uint32_t BufferSize);
DAL_StatusTypeDef DAL_SRAM_Read_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                    uint32_t BufferSize);
DAL_StatusTypeDef DAL_SRAM_Write_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                     uint32_t BufferSize);
DAL_StatusTypeDef DAL_SRAM_Read_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                    uint32_t BufferSize);
DAL_StatusTypeDef DAL_SRAM_Write_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                     uint32_t BufferSize);

void DAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void DAL_SRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma);

#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
/* SRAM callback registering/unregistering */
DAL_StatusTypeDef DAL_SRAM_RegisterCallback(SRAM_HandleTypeDef *hsram, DAL_SRAM_CallbackIDTypeDef CallbackId,
                                            pSRAM_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_SRAM_UnRegisterCallback(SRAM_HandleTypeDef *hsram, DAL_SRAM_CallbackIDTypeDef CallbackId);
DAL_StatusTypeDef DAL_SRAM_RegisterDmaCallback(SRAM_HandleTypeDef *hsram, DAL_SRAM_CallbackIDTypeDef CallbackId,
                                               pSRAM_DmaCallbackTypeDef pCallback);
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS  */

/**
  * @}
  */

/** @addtogroup SRAM_Exported_Functions_Group3 Control functions
  * @{
  */

/* SRAM Control functions  ****************************************************/
DAL_StatusTypeDef DAL_SRAM_WriteOperation_Enable(SRAM_HandleTypeDef *hsram);
DAL_StatusTypeDef DAL_SRAM_WriteOperation_Disable(SRAM_HandleTypeDef *hsram);

/**
  * @}
  */

/** @addtogroup SRAM_Exported_Functions_Group4 Peripheral State functions
  * @{
  */

/* SRAM  State functions ******************************************************/
DAL_SRAM_StateTypeDef DAL_SRAM_GetState(SRAM_HandleTypeDef *hsram);

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

#endif /* SMC_Bank1 */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_SRAM_H */
