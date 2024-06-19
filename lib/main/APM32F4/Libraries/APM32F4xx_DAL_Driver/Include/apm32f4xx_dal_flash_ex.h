/**
  *
  * @file    apm32f4xx_dal_flash_ex.h
  * @brief   Header file of FLASH DAL Extension module.
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
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_FLASH_EX_H
#define APM32F4xx_DAL_FLASH_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup FLASHEx
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/
/** @defgroup FLASHEx_Exported_Types FLASH Exported Types
  * @{
  */

/**
  * @brief  FLASH Erase structure definition
  */
typedef struct
{
  uint32_t TypeErase;   /*!< Mass erase or sector Erase.
                             This parameter can be a value of @ref FLASHEx_Type_Erase */

  uint32_t Banks;       /*!< Select banks to erase when Mass erase is enabled.
                             This parameter must be a value of @ref FLASHEx_Banks */

  uint32_t Sector;      /*!< Initial FLASH sector to erase when Mass erase is disabled
                             This parameter must be a value of @ref FLASHEx_Sectors */

  uint32_t NbSectors;   /*!< Number of sectors to be erased.
                             This parameter must be a value between 1 and (max number of sectors - value of Initial sector)*/

  uint32_t VoltageRange;/*!< The device voltage range which defines the erase parallelism
                             This parameter must be a value of @ref FLASHEx_Voltage_Range */

} FLASH_EraseInitTypeDef;

/**
  * @brief  FLASH Option Bytes Program structure definition
  */
typedef struct
{
  uint32_t OptionType;   /*!< Option byte to be configured.
                              This parameter can be a value of @ref FLASHEx_Option_Type */

  uint32_t WRPState;     /*!< Write protection activation or deactivation.
                              This parameter can be a value of @ref FLASHEx_WRP_State */

  uint32_t WRPSector;         /*!< Specifies the sector(s) to be write protected.
                              The value of this parameter depend on device used within the same series */

  uint32_t Banks;        /*!< Select banks for WRP activation/deactivation of all sectors.
                              This parameter must be a value of @ref FLASHEx_Banks */        

  uint32_t RDPLevel;     /*!< Set the read protection level.
                              This parameter can be a value of @ref FLASHEx_Option_Bytes_Read_Protection */

  uint32_t BORLevel;     /*!< Set the BOR Level.
                              This parameter can be a value of @ref FLASHEx_BOR_Reset_Level */

  uint8_t  USERConfig;   /*!< Program the FLASH User Option Byte: IWDT_SW / RST_STOP / RST_STDBY. */

} FLASH_OBProgramInitTypeDef;

/**
  * @brief  FLASH Advanced Option Bytes Program structure definition
  */
#if defined(APM32F411xx)
typedef struct
{
  uint32_t OptionType;     /*!< Option byte to be configured for extension.
                                This parameter can be a value of @ref FLASHEx_Advanced_Option_Type */

  uint32_t PCROPState;     /*!< PCROP activation or deactivation.
                                This parameter can be a value of @ref FLASHEx_PCROP_State */

  uint16_t Sectors;        /*!< specifies the sector(s) set for PCROP.
                                This parameter can be a value of @ref FLASHEx_Option_Bytes_PC_ReadWrite_Protection */

}FLASH_AdvOBProgramInitTypeDef;
#endif /* APM32F411xx */
/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup FLASHEx_Exported_Constants FLASH Exported Constants
  * @{
  */

/** @defgroup FLASHEx_Type_Erase FLASH Type Erase
  * @{
  */ 
#define FLASH_TYPEERASE_SECTORS         0x00000000U  /*!< Sectors erase only          */
#define FLASH_TYPEERASE_MASSERASE       0x00000001U  /*!< Flash Mass erase activation */
/**
  * @}
  */
  
/** @defgroup FLASHEx_Voltage_Range FLASH Voltage Range
  * @{
  */ 
#define FLASH_VOLTAGE_RANGE_1        0x00000000U  /*!< Device operating range: 1.8V to 2.1V                */
#define FLASH_VOLTAGE_RANGE_2        0x00000001U  /*!< Device operating range: 2.1V to 2.7V                */
#define FLASH_VOLTAGE_RANGE_3        0x00000002U  /*!< Device operating range: 2.7V to 3.6V                */
#define FLASH_VOLTAGE_RANGE_4        0x00000003U  /*!< Device operating range: 2.7V to 3.6V + External Vpp */
/**
  * @}
  */
  
/** @defgroup FLASHEx_WRP_State FLASH WRP State
  * @{
  */ 
#define OB_WRPSTATE_DISABLE       0x00000000U  /*!< Disable the write protection of the desired bank 1 sectors */
#define OB_WRPSTATE_ENABLE        0x00000001U  /*!< Enable the write protection of the desired bank 1 sectors  */
/**
  * @}
  */
  
/** @defgroup FLASHEx_Option_Type FLASH Option Type
  * @{
  */ 
#define OPTIONBYTE_WRP        0x00000001U  /*!< WRP option byte configuration  */
#define OPTIONBYTE_RDP        0x00000002U  /*!< RDP option byte configuration  */
#define OPTIONBYTE_USER       0x00000004U  /*!< USER option byte configuration */
#define OPTIONBYTE_BOR        0x00000008U  /*!< BOR option byte configuration  */
/**
  * @}
  */
  
/** @defgroup FLASHEx_Option_Bytes_Read_Protection FLASH Option Bytes Read Protection
  * @{
  */
#define OB_RDP_LEVEL_0   ((uint8_t)0xAA)
#define OB_RDP_LEVEL_1   ((uint8_t)0x55)
#define OB_RDP_LEVEL_2   ((uint8_t)0xCC) /*!< Warning: When enabling read protection level 2 
                                              it s no more possible to go back to level 1 or 0 */
/**
  * @}
  */ 
  
/** @defgroup FLASHEx_Option_Bytes_IWatchdog FLASH Option Bytes IWatchdog
  * @{
  */ 
#define OB_IWDT_SW                     ((uint8_t)0x20)  /*!< Software IWDT selected */
#define OB_IWDT_HW                     ((uint8_t)0x00)  /*!< Hardware IWDT selected */
/**
  * @}
  */ 
  
/** @defgroup FLASHEx_Option_Bytes_nRST_STOP FLASH Option Bytes nRST_STOP
  * @{
  */ 
#define OB_STOP_NO_RST                 ((uint8_t)0x40) /*!< No reset generated when entering in STOP */
#define OB_STOP_RST                    ((uint8_t)0x00) /*!< Reset generated when entering in STOP    */
/**
  * @}
  */ 


/** @defgroup FLASHEx_Option_Bytes_nRST_STDBY FLASH Option Bytes nRST_STDBY
  * @{
  */ 
#define OB_STDBY_NO_RST                ((uint8_t)0x80) /*!< No reset generated when entering in STANDBY */
#define OB_STDBY_RST                   ((uint8_t)0x00) /*!< Reset generated when entering in STANDBY    */
/**
  * @}
  */    

/** @defgroup FLASHEx_BOR_Reset_Level FLASH BOR Reset Level
  * @{
  */  
#define OB_BOR_LEVEL3          ((uint8_t)0x00)  /*!< Supply voltage ranges from 2.70 to 3.60 V */
#define OB_BOR_LEVEL2          ((uint8_t)0x04)  /*!< Supply voltage ranges from 2.40 to 2.70 V */
#define OB_BOR_LEVEL1          ((uint8_t)0x08)  /*!< Supply voltage ranges from 2.10 to 2.40 V */
#define OB_BOR_OFF             ((uint8_t)0x0C)  /*!< Supply voltage ranges from 1.62 to 2.10 V */
/**
  * @}
  */

#if defined(APM32F411xx)
/** @defgroup FLASHEx_PCROP_State FLASH PCROP State
  * @{
  */ 
#define OB_PCROP_STATE_DISABLE       0x00000000U  /*!< Disable PCROP */
#define OB_PCROP_STATE_ENABLE        0x00000001U  /*!< Enable PCROP  */
/**
  * @}
  */
#endif /* APM32F411xx */

/** @defgroup FLASHEx_Advanced_Option_Type FLASH Advanced Option Type
  * @{
  */ 

#if defined(APM32F411xx)
#define OPTIONBYTE_PCROP        0x00000001U  /*!<PCROP option byte configuration */
#endif /* APM32F411xx */
/**
  * @}
  */

/** @defgroup FLASH_Latency FLASH Latency
  * @{
  */

/*-------------------------- APM32F40xxx/APM32F41xxx/APM32F411xx -----------------------*/ 
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)
     
#define FLASH_LATENCY_0                FLASH_ACCTRL_WAITP_0WS   /*!< FLASH Zero Latency cycle      */
#define FLASH_LATENCY_1                FLASH_ACCTRL_WAITP_1WS   /*!< FLASH One Latency cycle       */
#define FLASH_LATENCY_2                FLASH_ACCTRL_WAITP_2WS   /*!< FLASH Two Latency cycles      */
#define FLASH_LATENCY_3                FLASH_ACCTRL_WAITP_3WS   /*!< FLASH Three Latency cycles    */
#define FLASH_LATENCY_4                FLASH_ACCTRL_WAITP_4WS   /*!< FLASH Four Latency cycles     */
#define FLASH_LATENCY_5                FLASH_ACCTRL_WAITP_5WS   /*!< FLASH Five Latency cycles     */
#define FLASH_LATENCY_6                FLASH_ACCTRL_WAITP_6WS   /*!< FLASH Six Latency cycles      */
#define FLASH_LATENCY_7                FLASH_ACCTRL_WAITP_7WS   /*!< FLASH Seven Latency cycles    */
#endif /* APM32F40xxx || APM32F41xxx | APM32F465xx || APM32F411xx */
/*--------------------------------------------------------------------------------------------------------------*/

/**
  * @}
  */ 
  

/** @defgroup FLASHEx_Banks FLASH Banks
  * @{
  */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)
#define FLASH_BANK_1     1U /*!< Bank 1   */
#endif /* APM32F40xxx || APM32F41xxx || APM32F465xx || APM32F411xx */
/**
  * @}
  */ 
    
/** @defgroup FLASHEx_MassErase_bit FLASH Mass Erase bit
  * @{
  */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)
#define FLASH_MER_BIT     (FLASH_CTRL_MERS) /*!< only 1 MER Bit */
#endif /* APM32F40xxx || APM32F41xxx || APM32F465xx || APM32F411xx */
/**
  * @}
  */ 

/** @defgroup FLASHEx_Sectors FLASH Sectors
  * @{
  */ 

/*--------------------------------------- APM32F40xxx/APM32F41xxx/APM32F465xx -------------------------------------*/ 
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define FLASH_SECTOR_0     0U  /*!< Sector Number 0   */
#define FLASH_SECTOR_1     1U  /*!< Sector Number 1   */
#define FLASH_SECTOR_2     2U  /*!< Sector Number 2   */
#define FLASH_SECTOR_3     3U  /*!< Sector Number 3   */
#define FLASH_SECTOR_4     4U  /*!< Sector Number 4   */
#define FLASH_SECTOR_5     5U  /*!< Sector Number 5   */
#define FLASH_SECTOR_6     6U  /*!< Sector Number 6   */
#define FLASH_SECTOR_7     7U  /*!< Sector Number 7   */
#define FLASH_SECTOR_8     8U  /*!< Sector Number 8   */
#define FLASH_SECTOR_9     9U  /*!< Sector Number 9   */
#define FLASH_SECTOR_10    10U /*!< Sector Number 10  */
#define FLASH_SECTOR_11    11U /*!< Sector Number 11  */
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */
/*-----------------------------------------------------------------------------------------------------*/

/*---------------------------------- APM32F411xx ------------------------------*/
#if defined(APM32F411xx)
#define FLASH_SECTOR_0     0U /*!< Sector Number 0   */
#define FLASH_SECTOR_1     1U /*!< Sector Number 1   */
#define FLASH_SECTOR_2     2U /*!< Sector Number 2   */
#define FLASH_SECTOR_3     3U /*!< Sector Number 3   */
#define FLASH_SECTOR_4     4U /*!< Sector Number 4   */
#define FLASH_SECTOR_5     5U /*!< Sector Number 5   */
#define FLASH_SECTOR_6     6U /*!< Sector Number 6   */
#define FLASH_SECTOR_7     7U /*!< Sector Number 7   */
#endif /* APM32F411xx */
/*-----------------------------------------------------------------------------------------------------*/

/**
  * @}
  */ 

/** @defgroup FLASHEx_Option_Bytes_Write_Protection FLASH Option Bytes Write Protection
  * @{
  */
      
/*--------------------------------------- APM32F40xxx/APM32F41xxx/APM32F465xx -------------------------------------*/ 
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define OB_WRP_SECTOR_0       0x00000001U /*!< Write protection of Sector0     */
#define OB_WRP_SECTOR_1       0x00000002U /*!< Write protection of Sector1     */
#define OB_WRP_SECTOR_2       0x00000004U /*!< Write protection of Sector2     */
#define OB_WRP_SECTOR_3       0x00000008U /*!< Write protection of Sector3     */
#define OB_WRP_SECTOR_4       0x00000010U /*!< Write protection of Sector4     */
#define OB_WRP_SECTOR_5       0x00000020U /*!< Write protection of Sector5     */
#define OB_WRP_SECTOR_6       0x00000040U /*!< Write protection of Sector6     */
#define OB_WRP_SECTOR_7       0x00000080U /*!< Write protection of Sector7     */
#define OB_WRP_SECTOR_8       0x00000100U /*!< Write protection of Sector8     */
#define OB_WRP_SECTOR_9       0x00000200U /*!< Write protection of Sector9     */
#define OB_WRP_SECTOR_10      0x00000400U /*!< Write protection of Sector10    */
#define OB_WRP_SECTOR_11      0x00000800U /*!< Write protection of Sector11    */
#define OB_WRP_SECTOR_All     0x00000FFFU /*!< Write protection of all Sectors */
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */
/*-----------------------------------------------------------------------------------------------------*/

/*---------------------------------- APM32F411xx ------------------------------*/
#if defined(APM32F411xx)
#define OB_WRP_SECTOR_0       0x00000001U /*!< Write protection of Sector0     */
#define OB_WRP_SECTOR_1       0x00000002U /*!< Write protection of Sector1     */
#define OB_WRP_SECTOR_2       0x00000004U /*!< Write protection of Sector2     */
#define OB_WRP_SECTOR_3       0x00000008U /*!< Write protection of Sector3     */
#define OB_WRP_SECTOR_4       0x00000010U /*!< Write protection of Sector4     */
#define OB_WRP_SECTOR_5       0x00000020U /*!< Write protection of Sector5     */
#define OB_WRP_SECTOR_6       0x00000040U /*!< Write protection of Sector6     */
#define OB_WRP_SECTOR_7       0x00000080U /*!< Write protection of Sector7     */
#define OB_WRP_SECTOR_All     0x00000FFFU /*!< Write protection of all Sectors */
#endif /* APM32F411xx */
/*-----------------------------------------------------------------------------------------------------*/
/**
  * @}
  */
  
/** @defgroup FLASHEx_Option_Bytes_PC_ReadWrite_Protection FLASH Option Bytes PC ReadWrite Protection
  * @{
  */

/*----------------------------------------- APM32F411xx-------------------------------------------*/
#if defined(APM32F411xx)
#define OB_PCROP_SECTOR_0        0x00000001U /*!< PC Read/Write protection of Sector0      */
#define OB_PCROP_SECTOR_1        0x00000002U /*!< PC Read/Write protection of Sector1      */
#define OB_PCROP_SECTOR_2        0x00000004U /*!< PC Read/Write protection of Sector2      */
#define OB_PCROP_SECTOR_3        0x00000008U /*!< PC Read/Write protection of Sector3      */
#define OB_PCROP_SECTOR_4        0x00000010U /*!< PC Read/Write protection of Sector4      */
#define OB_PCROP_SECTOR_5        0x00000020U /*!< PC Read/Write protection of Sector5      */
#define OB_PCROP_SECTOR_6        0x00000040U /*!< PC Read/Write protection of Sector6      */
#define OB_PCROP_SECTOR_7        0x00000080U /*!< PC Read/Write protection of Sector7      */
#define OB_PCROP_SECTOR_All      0x00000FFFU /*!< PC Read/Write protection of all Sectors  */
#endif /* APM32F411xx */
/*-----------------------------------------------------------------------------------------------------*/

/**
  * @}
  */

/** @defgroup  FLASHEx_Selection_Protection_Mode FLASH Selection Protection Mode
  * @{
  */
#if defined(APM32F411xx)
#define OB_PCROP_DESELECTED     ((uint8_t)0x00) /*!< Disabled PcROP, nWPRi bits used for Write Protection on sector i */
#define OB_PCROP_SELECTED       ((uint8_t)0x80) /*!< Enable PcROP, nWPRi bits used for PCRoP Protection on sector i   */
#endif /* APM32F411xx */
/**
  * @}
  */

/**
  * @}
  */ 
  
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup FLASHEx_Exported_Functions
  * @{
  */

/** @addtogroup FLASHEx_Exported_Functions_Group1
  * @{
  */
/* Extension Program operation functions  *************************************/
DAL_StatusTypeDef DAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
DAL_StatusTypeDef DAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
DAL_StatusTypeDef DAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              DAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);

#if defined(APM32F411xx)
DAL_StatusTypeDef DAL_FLASHEx_AdvOBProgram (FLASH_AdvOBProgramInitTypeDef *pAdvOBInit);
void              DAL_FLASHEx_AdvOBGetConfig(FLASH_AdvOBProgramInitTypeDef *pAdvOBInit);
DAL_StatusTypeDef DAL_FLASHEx_OB_SelectPCROP(void);
DAL_StatusTypeDef DAL_FLASHEx_OB_DeSelectPCROP(void);
#endif /* APM32F411xx */

/**
  * @}
  */

/**
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @defgroup FLASHEx_Private_Constants FLASH Private Constants
  * @{
  */

/*--------------------------------------- APM32F40xxx/APM32F41xxx/APM32F465xx -------------------------------------*/ 
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define FLASH_SECTOR_TOTAL  12U
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */

/*--------------------------------- APM32F411xx -------------------------------------*/
#if defined(APM32F411xx)
#define FLASH_SECTOR_TOTAL  8U
#endif /* APM32F411xx */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup FLASHEx_Private_Macros FLASH Private Macros
  * @{
  */

/** @defgroup FLASHEx_IS_FLASH_Definitions FLASH Private macros to check input parameters
  * @{
  */

#define IS_FLASH_TYPEERASE(VALUE)(((VALUE) == FLASH_TYPEERASE_SECTORS) || \
                                  ((VALUE) == FLASH_TYPEERASE_MASSERASE))  

#define IS_VOLTAGERANGE(RANGE)(((RANGE) == FLASH_VOLTAGE_RANGE_1) || \
                               ((RANGE) == FLASH_VOLTAGE_RANGE_2) || \
                               ((RANGE) == FLASH_VOLTAGE_RANGE_3) || \
                               ((RANGE) == FLASH_VOLTAGE_RANGE_4))  

#define IS_WRPSTATE(VALUE)(((VALUE) == OB_WRPSTATE_DISABLE) || \
                           ((VALUE) == OB_WRPSTATE_ENABLE))  

#define IS_OPTIONBYTE(VALUE)(((VALUE) <= (OPTIONBYTE_WRP|OPTIONBYTE_RDP|OPTIONBYTE_USER|OPTIONBYTE_BOR)))

#define IS_OB_RDP_LEVEL(LEVEL) (((LEVEL) == OB_RDP_LEVEL_0) ||\
                                ((LEVEL) == OB_RDP_LEVEL_1) ||\
                                ((LEVEL) == OB_RDP_LEVEL_2))

#define IS_OB_IWDT_SOURCE(SOURCE) (((SOURCE) == OB_IWDT_SW) || ((SOURCE) == OB_IWDT_HW))

#define IS_OB_STOP_SOURCE(SOURCE) (((SOURCE) == OB_STOP_NO_RST) || ((SOURCE) == OB_STOP_RST))

#define IS_OB_STDBY_SOURCE(SOURCE) (((SOURCE) == OB_STDBY_NO_RST) || ((SOURCE) == OB_STDBY_RST))

#define IS_OB_BOR_LEVEL(LEVEL) (((LEVEL) == OB_BOR_LEVEL1) || ((LEVEL) == OB_BOR_LEVEL2) ||\
                                ((LEVEL) == OB_BOR_LEVEL3) || ((LEVEL) == OB_BOR_OFF))

#if defined(APM32F411xx)
#define IS_PCROPSTATE(VALUE)(((VALUE) == OB_PCROP_STATE_DISABLE) || \
                             ((VALUE) == OB_PCROP_STATE_ENABLE))  
#endif /* APM32F411xx */

#if defined(APM32F411xx)
#define IS_OBEX(VALUE)(((VALUE) == OPTIONBYTE_PCROP))  
#endif /* APM32F411xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)
#define IS_FLASH_LATENCY(LATENCY) (((LATENCY) == FLASH_LATENCY_0)  || \
                                   ((LATENCY) == FLASH_LATENCY_1)  || \
                                   ((LATENCY) == FLASH_LATENCY_2)  || \
                                   ((LATENCY) == FLASH_LATENCY_3)  || \
                                   ((LATENCY) == FLASH_LATENCY_4)  || \
                                   ((LATENCY) == FLASH_LATENCY_5)  || \
                                   ((LATENCY) == FLASH_LATENCY_6)  || \
                                   ((LATENCY) == FLASH_LATENCY_7))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)
#define IS_FLASH_BANK(BANK) (((BANK) == FLASH_BANK_1))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx || APM32F411xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define IS_FLASH_SECTOR(SECTOR) (((SECTOR) == FLASH_SECTOR_0)   || ((SECTOR) == FLASH_SECTOR_1)   ||\
                                 ((SECTOR) == FLASH_SECTOR_2)   || ((SECTOR) == FLASH_SECTOR_3)   ||\
                                 ((SECTOR) == FLASH_SECTOR_4)   || ((SECTOR) == FLASH_SECTOR_5)   ||\
                                 ((SECTOR) == FLASH_SECTOR_6)   || ((SECTOR) == FLASH_SECTOR_7)   ||\
                                 ((SECTOR) == FLASH_SECTOR_8)   || ((SECTOR) == FLASH_SECTOR_9)   ||\
                                 ((SECTOR) == FLASH_SECTOR_10)  || ((SECTOR) == FLASH_SECTOR_11))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */

#if defined(APM32F411xx)
#define IS_FLASH_SECTOR(SECTOR) (((SECTOR) == FLASH_SECTOR_0)   || ((SECTOR) == FLASH_SECTOR_1)   ||\
                                 ((SECTOR) == FLASH_SECTOR_2)   || ((SECTOR) == FLASH_SECTOR_3)   ||\
                                 ((SECTOR) == FLASH_SECTOR_4)   || ((SECTOR) == FLASH_SECTOR_5)   ||\
                                 ((SECTOR) == FLASH_SECTOR_6)   || ((SECTOR) == FLASH_SECTOR_7))
#endif /* APM32F411xx */

#define IS_FLASH_ADDRESS(ADDRESS) ((((ADDRESS) >= FLASH_BASE) && ((ADDRESS) <= FLASH_END)) || \
                                   (((ADDRESS) >= FLASH_OTP_BASE) && ((ADDRESS) <= FLASH_OTP_END)))

#define IS_FLASH_NBSECTORS(NBSECTORS) (((NBSECTORS) != 0) && ((NBSECTORS) <= FLASH_SECTOR_TOTAL))

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F417xx) || defined(APM32F465xx)
#define IS_OB_WRP_SECTOR(SECTOR)((((SECTOR) & 0xFFFFF000U) == 0x00000000U) && ((SECTOR) != 0x00000000U))
#endif /* APM32F405xx || APM32F407xx || APM32F417xx || APM32F465xx */

#if defined(APM32F411xx)
#define IS_OB_WRP_SECTOR(SECTOR)((((SECTOR) & 0xFFFFF000U) == 0x00000000U) && ((SECTOR) != 0x00000000U))
#endif /* APM32F411xx */

#if defined(APM32F411xx)
#define IS_OB_PCROP(SECTOR)((((SECTOR) & 0xFFFFF000U) == 0x00000000U) && ((SECTOR) != 0x00000000U))
#endif /* APM32F411xx */

#if defined(APM32F411xx)
#define IS_OB_PCROP_SELECT(PCROP) (((PCROP) == OB_PCROP_SELECTED) || ((PCROP) == OB_PCROP_DESELECTED))
#endif /* APM32F411xx */
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup FLASHEx_Private_Functions FLASH Private Functions
  * @{
  */
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);
void FLASH_FlushCaches(void);
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

#endif /* APM32F4xx_DAL_FLASH_EX_H */

