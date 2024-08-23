/**
  *
  * @file    apm32f4xx_dal_nand.h
  * @brief   Header file of NAND DAL module.
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
#ifndef APM32F4xx_DAL_NAND_H
#define APM32F4xx_DAL_NAND_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(SMC_Bank2_3)

/* Includes ------------------------------------------------------------------*/
#if defined(SMC_Bank2_3)
#include "apm32f4xx_ddl_smc.h"
#endif /* SMC_Bank2_3 */

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup NAND
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** @defgroup NAND_Exported_Types NAND Exported Types
  * @{
  */

/**
  * @brief  DAL NAND State structures definition
  */
typedef enum
{
  DAL_NAND_STATE_RESET     = 0x00U,  /*!< NAND not yet initialized or disabled */
  DAL_NAND_STATE_READY     = 0x01U,  /*!< NAND initialized and ready for use   */
  DAL_NAND_STATE_BUSY      = 0x02U,  /*!< NAND internal process is ongoing     */
  DAL_NAND_STATE_ERROR     = 0x03U   /*!< NAND error state                     */
} DAL_NAND_StateTypeDef;

/**
  * @brief  NAND Memory electronic signature Structure definition
  */
typedef struct
{
  /*<! NAND memory electronic signature maker and device IDs */

  uint8_t Maker_Id;

  uint8_t Device_Id;

  uint8_t Third_Id;

  uint8_t Fourth_Id;
} NAND_IDTypeDef;

/**
  * @brief  NAND Memory address Structure definition
  */
typedef struct
{
  uint16_t Page;   /*!< NAND memory Page address  */

  uint16_t Plane;   /*!< NAND memory Zone address  */

  uint16_t Block;  /*!< NAND memory Block address */

} NAND_AddressTypeDef;

/**
  * @brief  NAND Memory info Structure definition
  */
typedef struct
{
  uint32_t        PageSize;              /*!< NAND memory page (without spare area) size measured in bytes
                                              for 8 bits addressing or words for 16 bits addressing             */

  uint32_t        SpareAreaSize;         /*!< NAND memory spare area size measured in bytes
                                              for 8 bits addressing or words for 16 bits addressing             */

  uint32_t        BlockSize;             /*!< NAND memory block size measured in number of pages               */

  uint32_t        BlockNbr;              /*!< NAND memory number of total blocks                               */

  uint32_t        PlaneNbr;              /*!< NAND memory number of planes                                     */

  uint32_t        PlaneSize;             /*!< NAND memory zone size measured in number of blocks               */

  FunctionalState ExtraCommandEnable;    /*!< NAND extra command needed for Page reading mode. This
                                              parameter is mandatory for some NAND parts after the read
                                              command (NAND_CMD_AREA_TRUE1) and before DATA reading sequence.
                                              Example: Toshiba THTH58BYG3S0HBAI6.
                                              This parameter could be ENABLE or DISABLE
                                              Please check the Read Mode sequnece in the NAND device datasheet */
} NAND_DeviceConfigTypeDef;

/**
  * @brief  NAND handle Structure definition
  */
#if (USE_DAL_NAND_REGISTER_CALLBACKS == 1)
typedef struct __NAND_HandleTypeDef
#else
typedef struct
#endif /* USE_DAL_NAND_REGISTER_CALLBACKS  */
{
  FMC_NAND_TypeDef               *Instance;  /*!< Register base address                                 */

  FMC_NAND_InitTypeDef           Init;       /*!< NAND device control configuration parameters          */

  DAL_LockTypeDef                Lock;       /*!< NAND locking object                                   */

  __IO DAL_NAND_StateTypeDef     State;      /*!< NAND device access state                              */

  NAND_DeviceConfigTypeDef       Config;     /*!< NAND phusical characteristic information structure    */

#if (USE_DAL_NAND_REGISTER_CALLBACKS == 1)
  void (* MspInitCallback)(struct __NAND_HandleTypeDef *hnand);               /*!< NAND Msp Init callback              */
  void (* MspDeInitCallback)(struct __NAND_HandleTypeDef *hnand);             /*!< NAND Msp DeInit callback            */
  void (* ItCallback)(struct __NAND_HandleTypeDef *hnand);                    /*!< NAND IT callback                    */
#endif /* USE_DAL_NAND_REGISTER_CALLBACKS */
} NAND_HandleTypeDef;

#if (USE_DAL_NAND_REGISTER_CALLBACKS == 1)
/**
  * @brief  DAL NAND Callback ID enumeration definition
  */
typedef enum
{
  DAL_NAND_MSP_INIT_CB_ID       = 0x00U,  /*!< NAND MspInit Callback ID          */
  DAL_NAND_MSP_DEINIT_CB_ID     = 0x01U,  /*!< NAND MspDeInit Callback ID        */
  DAL_NAND_IT_CB_ID             = 0x02U   /*!< NAND IT Callback ID               */
} DAL_NAND_CallbackIDTypeDef;

/**
  * @brief  DAL NAND Callback pointer definition
  */
typedef void (*pNAND_CallbackTypeDef)(NAND_HandleTypeDef *hnand);
#endif /* USE_DAL_NAND_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/** @defgroup NAND_Exported_Macros NAND Exported Macros
  * @{
  */

/** @brief Reset NAND handle state
  * @param  __HANDLE__ specifies the NAND handle.
  * @retval None
  */
#if (USE_DAL_NAND_REGISTER_CALLBACKS == 1)
#define __DAL_NAND_RESET_HANDLE_STATE(__HANDLE__)         do {                                             \
                                                               (__HANDLE__)->State = DAL_NAND_STATE_RESET; \
                                                               (__HANDLE__)->MspInitCallback = NULL;       \
                                                               (__HANDLE__)->MspDeInitCallback = NULL;     \
                                                             } while(0)
#else
#define __DAL_NAND_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = DAL_NAND_STATE_RESET)
#endif /* USE_DAL_NAND_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup NAND_Exported_Functions NAND Exported Functions
  * @{
  */

/** @addtogroup NAND_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/* Initialization/de-initialization functions  ********************************/
DAL_StatusTypeDef  DAL_NAND_Init(NAND_HandleTypeDef *hnand, FMC_NAND_PCC_TimingTypeDef *ComSpace_Timing,
                                 FMC_NAND_PCC_TimingTypeDef *AttSpace_Timing);
DAL_StatusTypeDef  DAL_NAND_DeInit(NAND_HandleTypeDef *hnand);

DAL_StatusTypeDef  DAL_NAND_ConfigDevice(NAND_HandleTypeDef *hnand, NAND_DeviceConfigTypeDef *pDeviceConfig);

DAL_StatusTypeDef  DAL_NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDTypeDef *pNAND_ID);

void               DAL_NAND_MspInit(NAND_HandleTypeDef *hnand);
void               DAL_NAND_MspDeInit(NAND_HandleTypeDef *hnand);
void               DAL_NAND_IRQHandler(NAND_HandleTypeDef *hnand);
void               DAL_NAND_ITCallback(NAND_HandleTypeDef *hnand);

/**
  * @}
  */

/** @addtogroup NAND_Exported_Functions_Group2 Input and Output functions
  * @{
  */

/* IO operation functions  ****************************************************/
DAL_StatusTypeDef  DAL_NAND_Reset(NAND_HandleTypeDef *hnand);

DAL_StatusTypeDef  DAL_NAND_Read_Page_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer,
                                         uint32_t NumPageToRead);
DAL_StatusTypeDef  DAL_NAND_Write_Page_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer,
                                          uint32_t NumPageToWrite);
DAL_StatusTypeDef  DAL_NAND_Read_SpareArea_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress,
                                              uint8_t *pBuffer, uint32_t NumSpareAreaToRead);
DAL_StatusTypeDef  DAL_NAND_Write_SpareArea_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress,
                                               uint8_t *pBuffer, uint32_t NumSpareAreaTowrite);

DAL_StatusTypeDef  DAL_NAND_Read_Page_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer,
                                          uint32_t NumPageToRead);
DAL_StatusTypeDef  DAL_NAND_Write_Page_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer,
                                           uint32_t NumPageToWrite);
DAL_StatusTypeDef  DAL_NAND_Read_SpareArea_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress,
                                               uint16_t *pBuffer, uint32_t NumSpareAreaToRead);
DAL_StatusTypeDef  DAL_NAND_Write_SpareArea_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress,
                                                uint16_t *pBuffer, uint32_t NumSpareAreaTowrite);

DAL_StatusTypeDef  DAL_NAND_Erase_Block(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress);

uint32_t           DAL_NAND_Address_Inc(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress);

#if (USE_DAL_NAND_REGISTER_CALLBACKS == 1)
/* NAND callback registering/unregistering */
DAL_StatusTypeDef  DAL_NAND_RegisterCallback(NAND_HandleTypeDef *hnand, DAL_NAND_CallbackIDTypeDef CallbackId,
                                             pNAND_CallbackTypeDef pCallback);
DAL_StatusTypeDef  DAL_NAND_UnRegisterCallback(NAND_HandleTypeDef *hnand, DAL_NAND_CallbackIDTypeDef CallbackId);
#endif /* USE_DAL_NAND_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup NAND_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */

/* NAND Control functions  ****************************************************/
DAL_StatusTypeDef  DAL_NAND_ECC_Enable(NAND_HandleTypeDef *hnand);
DAL_StatusTypeDef  DAL_NAND_ECC_Disable(NAND_HandleTypeDef *hnand);
DAL_StatusTypeDef  DAL_NAND_GetECC(NAND_HandleTypeDef *hnand, uint32_t *ECCval, uint32_t Timeout);

/**
  * @}
  */

/** @addtogroup NAND_Exported_Functions_Group4 Peripheral State functions
  * @{
  */
/* NAND State functions *******************************************************/
DAL_NAND_StateTypeDef DAL_NAND_GetState(NAND_HandleTypeDef *hnand);
uint32_t              DAL_NAND_Read_Status(NAND_HandleTypeDef *hnand);
/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup NAND_Private_Constants NAND Private Constants
  * @{
  */

#define NAND_DEVICE                0x80000000UL

#define NAND_WRITE_TIMEOUT         0x01000000UL

#define CMD_AREA                   (1UL<<16U)  /* A16 = CLE high */
#define ADDR_AREA                  (1UL<<17U)  /* A17 = ALE high */

#define NAND_CMD_AREA_A            ((uint8_t)0x00)
#define NAND_CMD_AREA_B            ((uint8_t)0x01)
#define NAND_CMD_AREA_C            ((uint8_t)0x50)
#define NAND_CMD_AREA_TRUE1        ((uint8_t)0x30)

#define NAND_CMD_WRITE0            ((uint8_t)0x80)
#define NAND_CMD_WRITE_TRUE1       ((uint8_t)0x10)
#define NAND_CMD_ERASE0            ((uint8_t)0x60)
#define NAND_CMD_ERASE1            ((uint8_t)0xD0)
#define NAND_CMD_READID            ((uint8_t)0x90)
#define NAND_CMD_STATUS            ((uint8_t)0x70)
#define NAND_CMD_LOCK_STATUS       ((uint8_t)0x7A)
#define NAND_CMD_RESET             ((uint8_t)0xFF)

/* NAND memory status */
#define NAND_VALID_ADDRESS         0x00000100UL
#define NAND_INVALID_ADDRESS       0x00000200UL
#define NAND_TIMEOUT_ERROR         0x00000400UL
#define NAND_BUSY                  0x00000000UL
#define NAND_ERROR                 0x00000001UL
#define NAND_READY                 0x00000040UL
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup NAND_Private_Macros NAND Private Macros
  * @{
  */

/**
  * @brief  NAND memory address computation.
  * @param  __ADDRESS__ NAND memory address.
  * @param  __HANDLE__  NAND handle.
  * @retval NAND Raw address value
  */
#define ARRAY_ADDRESS(__ADDRESS__ , __HANDLE__) ((__ADDRESS__)->Page + \
                                                 (((__ADDRESS__)->Block + \
                                                   (((__ADDRESS__)->Plane) * \
                                                    ((__HANDLE__)->Config.PlaneSize))) * \
                                                  ((__HANDLE__)->Config.BlockSize)))

/**
  * @brief  NAND memory Column address computation.
  * @param  __HANDLE__ NAND handle.
  * @retval NAND Raw address value
  */
#define COLUMN_ADDRESS( __HANDLE__) ((__HANDLE__)->Config.PageSize)

/**
  * @brief  NAND memory address cycling.
  * @param  __ADDRESS__ NAND memory address.
  * @retval NAND address cycling value.
  */
#define ADDR_1ST_CYCLE(__ADDRESS__)       (uint8_t)(__ADDRESS__)              /* 1st addressing cycle */
#define ADDR_2ND_CYCLE(__ADDRESS__)       (uint8_t)((__ADDRESS__) >> 8)       /* 2nd addressing cycle */
#define ADDR_3RD_CYCLE(__ADDRESS__)       (uint8_t)((__ADDRESS__) >> 16)      /* 3rd addressing cycle */
#define ADDR_4TH_CYCLE(__ADDRESS__)       (uint8_t)((__ADDRESS__) >> 24)      /* 4th addressing cycle */

/**
  * @brief  NAND memory Columns cycling.
  * @param  __ADDRESS__ NAND memory address.
  * @retval NAND Column address cycling value.
  */
#define COLUMN_1ST_CYCLE(__ADDRESS__)       (uint8_t)((__ADDRESS__) & 0xFFU)    /* 1st Column addressing cycle */
#define COLUMN_2ND_CYCLE(__ADDRESS__)       (uint8_t)((__ADDRESS__) >> 8)       /* 2nd Column addressing cycle */

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

#endif /* defined(SMC_Bank2_3) */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DAL_NAND_H */
