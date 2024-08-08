/**
  *
  * @file    apm32f4xx_dal_sdram.h
  * @brief   Header file of SDRAM DAL module.
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
#ifndef APM32F4xx_DAL_SDRAM_H
#define APM32F4xx_DAL_SDRAM_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(DMC)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_dmc.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup SDRAM
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup SDRAM_Exported_Types SDRAM Exported Types
  * @{
  */

/**
  * @brief  DAL SDRAM State structure definition
  */
typedef enum
{
  DAL_SDRAM_STATE_RESET             = 0x00U,  /*!< SDRAM not yet initialized or disabled */
  DAL_SDRAM_STATE_READY             = 0x01U,  /*!< SDRAM initialized and ready for use   */
  DAL_SDRAM_STATE_BUSY              = 0x02U,  /*!< SDRAM internal process is ongoing     */
  DAL_SDRAM_STATE_ERROR             = 0x03U,  /*!< SDRAM error state                     */
  DAL_SDRAM_STATE_WRITE_PROTECTED   = 0x04U,  /*!< SDRAM device write protected          */
  DAL_SDRAM_STATE_PRECHARGED        = 0x05U   /*!< SDRAM device precharged               */

} DAL_SDRAM_StateTypeDef;

/**
  * @brief  SDRAM handle Structure definition
  */
#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
typedef struct __SDRAM_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS  */
{
  DMC_SDRAM_TypeDef             *Instance;  /*!< Register base address                 */

  DMC_SDRAM_InitTypeDef         Init;       /*!< SDRAM device configuration parameters */

  __IO DAL_SDRAM_StateTypeDef   State;      /*!< SDRAM access state                    */

  DAL_LockTypeDef               Lock;       /*!< SDRAM locking object                  */

  DMA_HandleTypeDef             *hdma;      /*!< Pointer DMA handler                   */

#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
  void (* MspInitCallback)(struct __SDRAM_HandleTypeDef *hsdram);               /*!< SDRAM Msp Init callback              */
  void (* MspDeInitCallback)(struct __SDRAM_HandleTypeDef *hsdram);             /*!< SDRAM Msp DeInit callback            */
  void (* DmaXferCpltCallback)(DMA_HandleTypeDef *hdma);                        /*!< SDRAM DMA Xfer Complete callback     */
  void (* DmaXferErrorCallback)(DMA_HandleTypeDef *hdma);                       /*!< SDRAM DMA Xfer Error callback        */
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */
} SDRAM_HandleTypeDef;

#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL SDRAM Callback ID enumeration definition
  */
typedef enum
{
  DAL_SDRAM_MSP_INIT_CB_ID       = 0x00U,  /*!< SDRAM MspInit Callback ID           */
  DAL_SDRAM_MSP_DEINIT_CB_ID     = 0x01U,  /*!< SDRAM MspDeInit Callback ID         */
  DAL_SDRAM_DMA_XFER_CPLT_CB_ID  = 0x02U,  /*!< SDRAM DMA Xfer Complete Callback ID */
  DAL_SDRAM_DMA_XFER_ERR_CB_ID   = 0x03U   /*!< SDRAM DMA Xfer Error Callback ID    */
} DAL_SDRAM_CallbackIDTypeDef;

/**
  * @brief  DAL SDRAM Callback pointer definition
  */
typedef void (*pSDRAM_CallbackTypeDef)(SDRAM_HandleTypeDef *hsdram);
typedef void (*pSDRAM_DmaCallbackTypeDef)(DMA_HandleTypeDef *hdma);
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup SDRAM_Exported_Macros SDRAM Exported Macros
  * @{
  */

/** @brief Reset SDRAM handle state
  * @param  __HANDLE__ specifies the SDRAM handle.
  * @retval None
  */
#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
#define __DAL_SDRAM_RESET_HANDLE_STATE(__HANDLE__)        do {                                               \
                                                               (__HANDLE__)->State = DAL_SDRAM_STATE_RESET;  \
                                                               (__HANDLE__)->MspInitCallback = NULL;         \
                                                               (__HANDLE__)->MspDeInitCallback = NULL;       \
                                                             } while(0)
#else
#define __DAL_SDRAM_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_SDRAM_STATE_RESET)
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup SDRAM_Exported_Functions SDRAM Exported Functions
  * @{
  */

/** @addtogroup SDRAM_Exported_Functions_Group1
  * @{
  */

/* Initialization/de-initialization functions *********************************/
DAL_StatusTypeDef DAL_SDRAM_Init(SDRAM_HandleTypeDef *hsdram, DMC_SDRAM_TimingTypeDef *Timing);
DAL_StatusTypeDef DAL_SDRAM_DeInit(SDRAM_HandleTypeDef *hsdram);
void DAL_SDRAM_MspInit(SDRAM_HandleTypeDef *hsdram);
void DAL_SDRAM_MspDeInit(SDRAM_HandleTypeDef *hsdram);

void DAL_SDRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void DAL_SDRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma);

/**
  * @}
  */

/** @addtogroup SDRAM_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ****************************************************/
DAL_StatusTypeDef DAL_SDRAM_Read_8b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint8_t *pDstBuffer,
                                    uint32_t BufferSize);
DAL_StatusTypeDef DAL_SDRAM_Write_8b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint8_t *pSrcBuffer,
                                     uint32_t BufferSize);
DAL_StatusTypeDef DAL_SDRAM_Read_16b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint16_t *pDstBuffer,
                                     uint32_t BufferSize);
DAL_StatusTypeDef DAL_SDRAM_Write_16b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint16_t *pSrcBuffer,
                                      uint32_t BufferSize);
DAL_StatusTypeDef DAL_SDRAM_Read_32b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                     uint32_t BufferSize);
DAL_StatusTypeDef DAL_SDRAM_Write_32b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                      uint32_t BufferSize);

DAL_StatusTypeDef DAL_SDRAM_Read_DMA(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                     uint32_t BufferSize);
DAL_StatusTypeDef DAL_SDRAM_Write_DMA(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                      uint32_t BufferSize);

#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
/* SDRAM callback registering/unregistering */
DAL_StatusTypeDef DAL_SDRAM_RegisterCallback(SDRAM_HandleTypeDef *hsdram, DAL_SDRAM_CallbackIDTypeDef CallbackId,
                                             pSDRAM_CallbackTypeDef pCallback);
DAL_StatusTypeDef DAL_SDRAM_UnRegisterCallback(SDRAM_HandleTypeDef *hsdram, DAL_SDRAM_CallbackIDTypeDef CallbackId);
DAL_StatusTypeDef DAL_SDRAM_RegisterDmaCallback(SDRAM_HandleTypeDef *hsdram, DAL_SDRAM_CallbackIDTypeDef CallbackId,
                                                pSDRAM_DmaCallbackTypeDef pCallback);
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup SDRAM_Exported_Functions_Group3
  * @{
  */
/* SDRAM Control functions  *****************************************************/
DAL_StatusTypeDef DAL_SDRAM_ProgramRefreshPeriod(SDRAM_HandleTypeDef *hsdram, uint32_t RefreshPeriod);
DAL_StatusTypeDef DAL_SDRAM_SetOpenBankNumber(SDRAM_HandleTypeDef *hsdram, uint32_t OpenBankNumber);
uint32_t          DAL_SDRAM_GetModeStatus(SDRAM_HandleTypeDef *hsdram);

/**
  * @}
  */

/** @addtogroup SDRAM_Exported_Functions_Group4
  * @{
  */
/* SDRAM State functions ********************************************************/
DAL_SDRAM_StateTypeDef  DAL_SDRAM_GetState(SDRAM_HandleTypeDef *hsdram);
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

#endif /* DMC */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_SDRAM_H */
