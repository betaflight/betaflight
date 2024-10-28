/**
  *
  * @file    apm32f4xx_dal_pccard.h
  * @brief   Header file of PCCARD DAL module.
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
#ifndef APM32F4xx_DAL_PCCARD_H
#define APM32F4xx_DAL_PCCARD_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(SMC_Bank4)

/* Includes ------------------------------------------------------------------*/
#if defined(SMC_Bank4)
#include "apm32f4xx_ddl_smc.h"
#endif /* SMC_Bank4 */

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup PCCARD
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/** @defgroup PCCARD_Exported_Types PCCARD Exported Types
  * @{
  */

/**
  * @brief  DAL PCCARD State structures definition
  */
typedef enum
{
  DAL_PCCARD_STATE_RESET     = 0x00U,    /*!< PCCARD peripheral not yet initialized or disabled */
  DAL_PCCARD_STATE_READY     = 0x01U,    /*!< PCCARD peripheral ready                           */
  DAL_PCCARD_STATE_BUSY      = 0x02U,    /*!< PCCARD peripheral busy                            */
  DAL_PCCARD_STATE_ERROR     = 0x04U     /*!< PCCARD peripheral error                           */
} DAL_PCCARD_StateTypeDef;

typedef enum
{
  DAL_PCCARD_STATUS_SUCCESS = 0U,
  DAL_PCCARD_STATUS_ONGOING,
  DAL_PCCARD_STATUS_ERROR,
  DAL_PCCARD_STATUS_TIMEOUT
} DAL_PCCARD_StatusTypeDef;

/**
  * @brief  FMC_PCCARD handle Structure definition
  */
#if (USE_DAL_PCCARD_REGISTER_CALLBACKS == 1)
typedef struct __PCCARD_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_PCCARD_REGISTER_CALLBACKS  */
{
  FMC_PCCARD_TypeDef           *Instance;              /*!< Register base address for PCCARD device          */

  FMC_PCCARD_InitTypeDef       Init;                   /*!< PCCARD device control configuration parameters   */

  __IO DAL_PCCARD_StateTypeDef State;                  /*!< PCCARD device access state                       */

  DAL_LockTypeDef              Lock;                   /*!< PCCARD Lock                                      */

#if (USE_DAL_PCCARD_REGISTER_CALLBACKS == 1)
  void (* MspInitCallback)(struct __PCCARD_HandleTypeDef *hpccard);               /*!< PCCARD Msp Init callback              */
  void (* MspDeInitCallback)(struct __PCCARD_HandleTypeDef *hpccard);             /*!< PCCARD Msp DeInit callback            */
  void (* ItCallback)(struct __PCCARD_HandleTypeDef *hpccard);                    /*!< PCCARD IT callback                    */
#endif
} PCCARD_HandleTypeDef;

#if (USE_DAL_PCCARD_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL PCCARD Callback ID enumeration definition
  */
typedef enum
{
  DAL_PCCARD_MSP_INIT_CB_ID       = 0x00U,  /*!< PCCARD MspInit Callback ID          */
  DAL_PCCARD_MSP_DEINIT_CB_ID     = 0x01U,  /*!< PCCARD MspDeInit Callback ID        */
  DAL_PCCARD_IT_CB_ID             = 0x02U   /*!< PCCARD IT Callback ID               */
} DAL_PCCARD_CallbackIDTypeDef;

/**
  * @brief  DAL PCCARD Callback pointer definition
  */
typedef void (*pPCCARD_CallbackTypeDef)(PCCARD_HandleTypeDef *hpccard);
#endif
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/** @defgroup PCCARD_Exported_Macros PCCARD Exported Macros
  * @{
  */
/** @brief Reset PCCARD handle state
  * @param  __HANDLE__ specifies the PCCARD handle.
  * @retval None
  */
#if (USE_DAL_PCCARD_REGISTER_CALLBACKS == 1)
#define __DAL_PCCARD_RESET_HANDLE_STATE(__HANDLE__)       do {                                               \
                                                               (__HANDLE__)->State = DAL_PCCARD_STATE_RESET; \
                                                               (__HANDLE__)->MspInitCallback = NULL;         \
                                                               (__HANDLE__)->MspDeInitCallback = NULL;       \
                                                             } while(0)
#else
#define __DAL_PCCARD_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_PCCARD_STATE_RESET)
#endif
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PCCARD_Exported_Functions
  * @{
  */

/** @addtogroup PCCARD_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
DAL_StatusTypeDef  DAL_PCCARD_Init(PCCARD_HandleTypeDef *hpccard, FMC_NAND_PCC_TimingTypeDef *ComSpaceTiming,
                                   FMC_NAND_PCC_TimingTypeDef *AttSpaceTiming, FMC_NAND_PCC_TimingTypeDef *IOSpaceTiming);
DAL_StatusTypeDef  DAL_PCCARD_DeInit(PCCARD_HandleTypeDef *hpccard);
void DAL_PCCARD_MspInit(PCCARD_HandleTypeDef *hpccard);
void DAL_PCCARD_MspDeInit(PCCARD_HandleTypeDef *hpccard);
/**
  * @}
  */

/** @addtogroup PCCARD_Exported_Functions_Group2
  * @{
  */
/* IO operation functions  *****************************************************/
DAL_StatusTypeDef  DAL_PCCARD_Read_ID(PCCARD_HandleTypeDef *hpccard, uint8_t CompactFlash_ID[], uint8_t *pStatus);
DAL_StatusTypeDef  DAL_PCCARD_Write_Sector(PCCARD_HandleTypeDef *hpccard, uint16_t *pBuffer, uint16_t SectorAddress,
                                           uint8_t *pStatus);
DAL_StatusTypeDef  DAL_PCCARD_Read_Sector(PCCARD_HandleTypeDef *hpccard, uint16_t *pBuffer, uint16_t SectorAddress,
                                          uint8_t *pStatus);
DAL_StatusTypeDef  DAL_PCCARD_Erase_Sector(PCCARD_HandleTypeDef *hpccard, uint16_t SectorAddress, uint8_t *pStatus);
DAL_StatusTypeDef  DAL_PCCARD_Reset(PCCARD_HandleTypeDef *hpccard);
void               DAL_PCCARD_IRQHandler(PCCARD_HandleTypeDef *hpccard);
void               DAL_PCCARD_ITCallback(PCCARD_HandleTypeDef *hpccard);

#if (USE_DAL_PCCARD_REGISTER_CALLBACKS == 1)
/* PCCARD callback registering/unregistering */
DAL_StatusTypeDef  DAL_PCCARD_RegisterCallback(PCCARD_HandleTypeDef *hpccard, DAL_PCCARD_CallbackIDTypeDef CallbackId,
                                               pPCCARD_CallbackTypeDef pCallback);
DAL_StatusTypeDef  DAL_PCCARD_UnRegisterCallback(PCCARD_HandleTypeDef *hpccard,
                                                 DAL_PCCARD_CallbackIDTypeDef CallbackId);
#endif
/**
  * @}
  */

/** @addtogroup PCCARD_Exported_Functions_Group3
  * @{
  */
/* PCCARD State functions *******************************************************/
DAL_PCCARD_StateTypeDef  DAL_PCCARD_GetState(PCCARD_HandleTypeDef *hpccard);
DAL_PCCARD_StatusTypeDef DAL_PCCARD_GetStatus(PCCARD_HandleTypeDef *hpccard);
DAL_PCCARD_StatusTypeDef DAL_PCCARD_ReadStatus(PCCARD_HandleTypeDef *hpccard);
/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup PCCARD_Private_Constants PCCARD Private Constants
  * @{
  */
#define PCCARD_DEVICE_ADDRESS             0x90000000U
#define PCCARD_ATTRIBUTE_SPACE_ADDRESS    0x98000000U              /* Attribute space size to @0x9BFF FFFF */
#define PCCARD_COMMON_SPACE_ADDRESS       PCCARD_DEVICE_ADDRESS    /* Common space size to @0x93FF FFFF    */
#define PCCARD_IO_SPACE_ADDRESS           0x9C000000U              /* IO space size to @0x9FFF FFFF        */
#define PCCARD_IO_SPACE_PRIMARY_ADDR      0x9C0001F0U              /* IO space size to @0x9FFF FFFF        */

/* Flash-ATA registers description */
#define ATA_DATA                       ((uint8_t)0x00)    /* Data register */
#define ATA_SECTOR_COUNT               ((uint8_t)0x02)    /* Sector Count register */
#define ATA_SECTOR_NUMBER              ((uint8_t)0x03)    /* Sector Number register */
#define ATA_CYLINDER_LOW               ((uint8_t)0x04)    /* Cylinder low register */
#define ATA_CYLINDER_HIGH              ((uint8_t)0x05)    /* Cylinder high register */
#define ATA_CARD_HEAD                  ((uint8_t)0x06)    /* Card/Head register */
#define ATA_STATUS_CMD                 ((uint8_t)0x07)    /* Status(read)/Command(write) register */
#define ATA_STATUS_CMD_ALTERNATE       ((uint8_t)0x0E)    /* Alternate Status(read)/Command(write) register */
#define ATA_COMMON_DATA_AREA           ((uint16_t)0x0400) /* Start of data area (for Common access only!) */
#define ATA_CARD_CONFIGURATION         ((uint16_t)0x0202) /* Card Configuration and Status Register */

/* Flash-ATA commands */
#define ATA_READ_SECTOR_CMD            ((uint8_t)0x20)
#define ATA_WRITE_SECTOR_CMD           ((uint8_t)0x30)
#define ATA_ERASE_SECTOR_CMD           ((uint8_t)0xC0)
#define ATA_IDENTIFY_CMD               ((uint8_t)0xEC)

/* PC Card/Compact Flash status */
#define PCCARD_TIMEOUT_ERROR           ((uint8_t)0x60)
#define PCCARD_BUSY                    ((uint8_t)0x80)
#define PCCARD_PROGR                   ((uint8_t)0x01)
#define PCCARD_READY                   ((uint8_t)0x40)

#define PCCARD_SECTOR_SIZE             255U               /* In half words */

/**
  * @}
  */
/* Compact Flash redefinition */
#define DAL_CF_Init                 DAL_PCCARD_Init
#define DAL_CF_DeInit               DAL_PCCARD_DeInit
#define DAL_CF_MspInit              DAL_PCCARD_MspInit
#define DAL_CF_MspDeInit            DAL_PCCARD_MspDeInit

#define DAL_CF_Read_ID              DAL_PCCARD_Read_ID
#define DAL_CF_Write_Sector         DAL_PCCARD_Write_Sector
#define DAL_CF_Read_Sector          DAL_PCCARD_Read_Sector
#define DAL_CF_Erase_Sector         DAL_PCCARD_Erase_Sector
#define DAL_CF_Reset                DAL_PCCARD_Reset
#define DAL_CF_IRQHandler           DAL_PCCARD_IRQHandler
#define DAL_CF_ITCallback           DAL_PCCARD_ITCallback

#define DAL_CF_GetState             DAL_PCCARD_GetState
#define DAL_CF_GetStatus            DAL_PCCARD_GetStatus
#define DAL_CF_ReadStatus           DAL_PCCARD_ReadStatus

#define DAL_CF_STATUS_SUCCESS       DAL_PCCARD_STATUS_SUCCESS
#define DAL_CF_STATUS_ONGOING       DAL_PCCARD_STATUS_ONGOING
#define DAL_CF_STATUS_ERROR         DAL_PCCARD_STATUS_ERROR
#define DAL_CF_STATUS_TIMEOUT       DAL_PCCARD_STATUS_TIMEOUT
#define DAL_CF_StatusTypeDef        DAL_PCCARD_StatusTypeDef

#define CF_DEVICE_ADDRESS           PCCARD_DEVICE_ADDRESS
#define CF_ATTRIBUTE_SPACE_ADDRESS  PCCARD_ATTRIBUTE_SPACE_ADDRESS
#define CF_COMMON_SPACE_ADDRESS     PCCARD_COMMON_SPACE_ADDRESS
#define CF_IO_SPACE_ADDRESS         PCCARD_IO_SPACE_ADDRESS
#define CF_IO_SPACE_PRIMARY_ADDR    PCCARD_IO_SPACE_PRIMARY_ADDR

#define CF_TIMEOUT_ERROR            PCCARD_TIMEOUT_ERROR
#define CF_BUSY                     PCCARD_BUSY
#define CF_PROGR                    PCCARD_PROGR
#define CF_READY                    PCCARD_READY

#define CF_SECTOR_SIZE              PCCARD_SECTOR_SIZE

/* Private macros ------------------------------------------------------------*/
/**
  * @}
  */


/**
  * @}
  */

#endif /* SMC_Bank4 */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_PCCARD_H */
