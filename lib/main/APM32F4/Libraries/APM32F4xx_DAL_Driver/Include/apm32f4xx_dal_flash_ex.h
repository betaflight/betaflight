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
  * The original code has been modified by Geehy Semiconductor.
  * Copyright (c) 2017 STMicroelectronics. Copyright (C) 2023-2025 Geehy Semiconductor.
  * All rights reserved.
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

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  uint32_t Sector;      /*!< Initial FLASH sector to erase when Mass erase is disabled
                             This parameter must be a value of @ref FLASHEx_Sectors */

  uint32_t NbSectors;   /*!< Number of sectors to be erased.
                             This parameter must be a value between 1 and (max number of sectors - value of Initial sector)*/

  uint32_t VoltageRange;/*!< The device voltage range which defines the erase parallelism
                             This parameter must be a value of @ref FLASHEx_Voltage_Range */
#else
  uint32_t PageAddress; /*!< PageAdress: Initial FLASH page address to erase when mass erase is disabled
                             This parameter must be a number between Min_Data = 0x08000000 and Max_Data = 0x08020000*/

  uint32_t NbPages;     /*!< NbPages: Number of pagess to be erased.
                             This parameter must be a value between Min_Data = 1 and Max_Data = (max number of pages - value of initial page)*/
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
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

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  uint32_t WRPSector;    /*!< Specifies the sector(s) to be write protected.
                              The value of this parameter depend on device used within the same series */
#else
  uint32_t WRPPage;      /*!< WRPPage: specifies the page(s) to be write protected
                             This parameter can be a value of @ref FLASHEx_Option_Bytes_Write_Protection */
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

  uint32_t Banks;        /*!< Select banks for WRP activation/deactivation of all sectors.
                              This parameter must be a value of @ref FLASHEx_Banks */

  uint32_t RDPLevel;     /*!< Set the read protection level.
                              This parameter can be a value of @ref FLASHEx_Option_Bytes_Read_Protection */

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
  uint32_t BORLevel;     /*!< Set the BOR Level.
                              This parameter can be a value of @ref FLASHEx_BOR_Reset_Level */
#else
  uint32_t DATAAddress;  /*!< DATAAddress: Address of the option byte DATA to be programmed
                              This parameter can be a value of @ref FLASHEx_OB_Data_Address */

  uint8_t DATAData;      /*!< DATAData: Data to be stored in the option byte DATA
                              This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF */
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

  uint8_t  USERConfig;   /*!< Program the FLASH User Option Byte: IWDT_SW / RST_STOP / RST_STDBY / ROM_SEL(APM32402/403xx). */

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

/** @defgroup  FLASHEx_Page_Size Page Size
  * @{
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define FLASH_PAGE_SIZE          0x400U
#endif /* APM32F403xx || APM32F402xx */
/**
  * @}
  */

/** @defgroup FLASHEx_Type_Erase FLASH Type Erase
  * @{
  */
#if defined(APM32F403xx) || defined(APM32F402xx)
#define FLASH_TYPEERASE_PAGES           0x00000000U  /*!< Pages erase only */
#else
#define FLASH_TYPEERASE_SECTORS         0x00000000U  /*!< Sectors erase only          */
#endif /* APM32F403xx || APM32F402xx */
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
#define OPTIONBYTE_WRP        0x00000001U   /*!< WRP option byte configuration  */
#define OPTIONBYTE_RDP        0x00000002U   /*!< RDP option byte configuration  */
#define OPTIONBYTE_USER       0x00000004U   /*!< USER option byte configuration */
#define OPTIONBYTE_BOR        0x00000008U   /*!< BOR option byte configuration  */
#define OPTIONBYTE_DATA       0x00000008U   /*!< DATA option byte configuration */
/**
  * @}
  */

/** @defgroup FLASHEx_Option_Bytes_Read_Protection FLASH Option Bytes Read Protection
  * @{
  */
#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
#define OB_RDP_LEVEL_0   ((uint8_t)0xAA)
#define OB_RDP_LEVEL_1   ((uint8_t)0x55)
#define OB_RDP_LEVEL_2   ((uint8_t)0xCC) /*!< Warning: When enabling read protection level 2
                                              it s no more possible to go back to level 1 or 0 */
#else
#define OB_RDP_LEVEL_0            ((uint8_t)0xA5)
#define OB_RDP_LEVEL_1            ((uint8_t)0x00)
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
/**
  * @}
  */

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
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

#else

/** @defgroup FLASHEx_Option_Bytes_IWatchdog FLASH Option Bytes IWatchdog
  * @{
  */
#define OB_IWDT_SW                     ((uint8_t)0x01)  /*!< Software IWDT selected */
#define OB_IWDT_HW                     ((uint8_t)0x00)  /*!< Hardware IWDT selected */
/**
  * @}
  */

/** @defgroup FLASHEx_Option_Bytes_nRST_STOP FLASH Option Bytes nRST_STOP
  * @{
  */
#define OB_STOP_NO_RST                 ((uint8_t)0x02) /*!< No reset generated when entering in STOP */
#define OB_STOP_RST                    ((uint8_t)0x00) /*!< Reset generated when entering in STOP    */
/**
  * @}
  */

/** @defgroup FLASHEx_Option_Bytes_nRST_STDBY FLASH Option Bytes nRST_STDBY
  * @{
  */
#define OB_STDBY_NO_RST                ((uint8_t)0x04) /*!< No reset generated when entering in STANDBY */
#define OB_STDBY_RST                   ((uint8_t)0x00) /*!< Reset generated when entering in STANDBY    */
/**
  * @}
  */

/** @defgroup FLASHEx_Option_Bytes_nROM_SEL FLASH Option Bytes nROM_SEL
  * @{
  */
#define OB_SEL_INFO                    ((uint8_t)0x10) /*!< 0x1FFFE400 maps to 0x00 when system memory starts up */
#define OB_SEL_ROM                     ((uint8_t)0x00) /*!< 0x1FFF6000 maps to 0x00 when system memory starts up */
/**
  * @}
  */

/** @defgroup FLASHEx_OB_Data_Address  Option Byte Data Address
  * @{
  */
#define OB_DATA_ADDRESS_DATA0     0x1FFFF804U
#define OB_DATA_ADDRESS_DATA1     0x1FFFF806U
/**
  * @}
  */
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

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
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx)

#define FLASH_LATENCY_0                FLASH_ACCTRL_WAITP_0WS   /*!< FLASH Zero Latency cycle      */
#define FLASH_LATENCY_1                FLASH_ACCTRL_WAITP_1WS   /*!< FLASH One Latency cycle       */
#define FLASH_LATENCY_2                FLASH_ACCTRL_WAITP_2WS   /*!< FLASH Two Latency cycles      */
#define FLASH_LATENCY_3                FLASH_ACCTRL_WAITP_3WS   /*!< FLASH Three Latency cycles    */
#define FLASH_LATENCY_4                FLASH_ACCTRL_WAITP_4WS   /*!< FLASH Four Latency cycles     */
#define FLASH_LATENCY_5                FLASH_ACCTRL_WAITP_5WS   /*!< FLASH Five Latency cycles     */
#define FLASH_LATENCY_6                FLASH_ACCTRL_WAITP_6WS   /*!< FLASH Six Latency cycles      */
#define FLASH_LATENCY_7                FLASH_ACCTRL_WAITP_7WS   /*!< FLASH Seven Latency cycles    */
#elif defined (APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define FLASH_LATENCY_0                FLASH_ACCTRL_WAITP_0WS
#define FLASH_LATENCY_1                FLASH_ACCTRL_WAITP_1WS
#define FLASH_LATENCY_2                FLASH_ACCTRL_WAITP_2WS
#define FLASH_LATENCY_3                FLASH_ACCTRL_WAITP_3WS
#define FLASH_LATENCY_4                FLASH_ACCTRL_WAITP_4WS
#define FLASH_LATENCY_5                FLASH_ACCTRL_WAITP_5WS
#define FLASH_LATENCY_6                FLASH_ACCTRL_WAITP_6WS
#define FLASH_LATENCY_7                FLASH_ACCTRL_WAITP_7WS
#define FLASH_LATENCY_8                FLASH_ACCTRL_WAITP_8WS
#define FLASH_LATENCY_9                FLASH_ACCTRL_WAITP_9WS
#define FLASH_LATENCY_10               FLASH_ACCTRL_WAITP_10WS
#define FLASH_LATENCY_11               FLASH_ACCTRL_WAITP_11WS
#define FLASH_LATENCY_12               FLASH_ACCTRL_WAITP_12WS
#define FLASH_LATENCY_13               FLASH_ACCTRL_WAITP_13WS
#define FLASH_LATENCY_14               FLASH_ACCTRL_WAITP_14WS
#define FLASH_LATENCY_15               FLASH_ACCTRL_WAITP_15WS
#define FLASH_LATENCY_16               FLASH_ACCTRL_WAITP_16WS
#define FLASH_LATENCY_17               FLASH_ACCTRL_WAITP_17WS
#define FLASH_LATENCY_18               FLASH_ACCTRL_WAITP_18WS
#define FLASH_LATENCY_19               FLASH_ACCTRL_WAITP_19WS
#define FLASH_LATENCY_20               FLASH_ACCTRL_WAITP_20WS
#define FLASH_LATENCY_21               FLASH_ACCTRL_WAITP_21WS
#define FLASH_LATENCY_22               FLASH_ACCTRL_WAITP_22WS
#define FLASH_LATENCY_23               FLASH_ACCTRL_WAITP_23WS
#define FLASH_LATENCY_24               FLASH_ACCTRL_WAITP_24WS
#define FLASH_LATENCY_25               FLASH_ACCTRL_WAITP_25WS
#define FLASH_LATENCY_26               FLASH_ACCTRL_WAITP_26WS
#define FLASH_LATENCY_27               FLASH_ACCTRL_WAITP_27WS
#define FLASH_LATENCY_28               FLASH_ACCTRL_WAITP_28WS
#define FLASH_LATENCY_29               FLASH_ACCTRL_WAITP_29WS
#define FLASH_LATENCY_30               FLASH_ACCTRL_WAITP_30WS
#define FLASH_LATENCY_31               FLASH_ACCTRL_WAITP_31WS
#define FLASH_LATENCY_32               FLASH_ACCTRL_WAITP_32WS
#define FLASH_LATENCY_33               FLASH_ACCTRL_WAITP_33WS
#define FLASH_LATENCY_34               FLASH_ACCTRL_WAITP_34WS
#define FLASH_LATENCY_35               FLASH_ACCTRL_WAITP_35WS
#define FLASH_LATENCY_36               FLASH_ACCTRL_WAITP_36WS
#define FLASH_LATENCY_37               FLASH_ACCTRL_WAITP_37WS
#define FLASH_LATENCY_38               FLASH_ACCTRL_WAITP_38WS
#define FLASH_LATENCY_39               FLASH_ACCTRL_WAITP_39WS
#define FLASH_LATENCY_40               FLASH_ACCTRL_WAITP_40WS
#define FLASH_LATENCY_41               FLASH_ACCTRL_WAITP_41WS
#define FLASH_LATENCY_42               FLASH_ACCTRL_WAITP_42WS
#define FLASH_LATENCY_43               FLASH_ACCTRL_WAITP_43WS
#define FLASH_LATENCY_44               FLASH_ACCTRL_WAITP_44WS
#define FLASH_LATENCY_45               FLASH_ACCTRL_WAITP_45WS
#define FLASH_LATENCY_46               FLASH_ACCTRL_WAITP_46WS
#define FLASH_LATENCY_47               FLASH_ACCTRL_WAITP_47WS
#define FLASH_LATENCY_48               FLASH_ACCTRL_WAITP_48WS
#define FLASH_LATENCY_49               FLASH_ACCTRL_WAITP_49WS
#define FLASH_LATENCY_50               FLASH_ACCTRL_WAITP_50WS
#define FLASH_LATENCY_51               FLASH_ACCTRL_WAITP_51WS
#define FLASH_LATENCY_52               FLASH_ACCTRL_WAITP_52WS
#define FLASH_LATENCY_53               FLASH_ACCTRL_WAITP_53WS
#define FLASH_LATENCY_54               FLASH_ACCTRL_WAITP_54WS
#define FLASH_LATENCY_55               FLASH_ACCTRL_WAITP_55WS
#define FLASH_LATENCY_56               FLASH_ACCTRL_WAITP_56WS
#define FLASH_LATENCY_57               FLASH_ACCTRL_WAITP_57WS
#define FLASH_LATENCY_58               FLASH_ACCTRL_WAITP_58WS
#define FLASH_LATENCY_59               FLASH_ACCTRL_WAITP_59WS
#define FLASH_LATENCY_60               FLASH_ACCTRL_WAITP_60WS
#define FLASH_LATENCY_61               FLASH_ACCTRL_WAITP_61WS
#define FLASH_LATENCY_62               FLASH_ACCTRL_WAITP_62WS
#define FLASH_LATENCY_63               FLASH_ACCTRL_WAITP_63WS
#define FLASH_LATENCY_64               FLASH_ACCTRL_WAITP_64WS
#define FLASH_LATENCY_65               FLASH_ACCTRL_WAITP_65WS
#define FLASH_LATENCY_66               FLASH_ACCTRL_WAITP_66WS
#define FLASH_LATENCY_67               FLASH_ACCTRL_WAITP_67WS
#define FLASH_LATENCY_68               FLASH_ACCTRL_WAITP_68WS
#define FLASH_LATENCY_69               FLASH_ACCTRL_WAITP_69WS
#define FLASH_LATENCY_70               FLASH_ACCTRL_WAITP_70WS
#define FLASH_LATENCY_71               FLASH_ACCTRL_WAITP_71WS
#define FLASH_LATENCY_72               FLASH_ACCTRL_WAITP_72WS
#define FLASH_LATENCY_73               FLASH_ACCTRL_WAITP_73WS
#define FLASH_LATENCY_74               FLASH_ACCTRL_WAITP_74WS
#define FLASH_LATENCY_75               FLASH_ACCTRL_WAITP_75WS
#define FLASH_LATENCY_76               FLASH_ACCTRL_WAITP_76WS
#define FLASH_LATENCY_77               FLASH_ACCTRL_WAITP_77WS
#define FLASH_LATENCY_78               FLASH_ACCTRL_WAITP_78WS
#define FLASH_LATENCY_79               FLASH_ACCTRL_WAITP_79WS
#define FLASH_LATENCY_80               FLASH_ACCTRL_WAITP_80WS
#define FLASH_LATENCY_81               FLASH_ACCTRL_WAITP_81WS
#define FLASH_LATENCY_82               FLASH_ACCTRL_WAITP_82WS
#define FLASH_LATENCY_83               FLASH_ACCTRL_WAITP_83WS
#define FLASH_LATENCY_84               FLASH_ACCTRL_WAITP_84WS
#define FLASH_LATENCY_85               FLASH_ACCTRL_WAITP_85WS
#define FLASH_LATENCY_86               FLASH_ACCTRL_WAITP_86WS
#define FLASH_LATENCY_87               FLASH_ACCTRL_WAITP_87WS
#define FLASH_LATENCY_88               FLASH_ACCTRL_WAITP_88WS
#define FLASH_LATENCY_89               FLASH_ACCTRL_WAITP_89WS
#define FLASH_LATENCY_90               FLASH_ACCTRL_WAITP_90WS
#define FLASH_LATENCY_91               FLASH_ACCTRL_WAITP_91WS
#define FLASH_LATENCY_92               FLASH_ACCTRL_WAITP_92WS
#define FLASH_LATENCY_93               FLASH_ACCTRL_WAITP_93WS
#define FLASH_LATENCY_94               FLASH_ACCTRL_WAITP_94WS
#define FLASH_LATENCY_95               FLASH_ACCTRL_WAITP_95WS
#define FLASH_LATENCY_96               FLASH_ACCTRL_WAITP_96WS
#define FLASH_LATENCY_97               FLASH_ACCTRL_WAITP_97WS
#define FLASH_LATENCY_98               FLASH_ACCTRL_WAITP_98WS
#define FLASH_LATENCY_99               FLASH_ACCTRL_WAITP_99WS
#define FLASH_LATENCY_100              FLASH_ACCTRL_WAITP_100WS
#define FLASH_LATENCY_101              FLASH_ACCTRL_WAITP_101WS
#define FLASH_LATENCY_102              FLASH_ACCTRL_WAITP_102WS
#define FLASH_LATENCY_103              FLASH_ACCTRL_WAITP_103WS
#define FLASH_LATENCY_104              FLASH_ACCTRL_WAITP_104WS
#define FLASH_LATENCY_105              FLASH_ACCTRL_WAITP_105WS
#define FLASH_LATENCY_106              FLASH_ACCTRL_WAITP_106WS
#define FLASH_LATENCY_107              FLASH_ACCTRL_WAITP_107WS
#define FLASH_LATENCY_108              FLASH_ACCTRL_WAITP_108WS
#define FLASH_LATENCY_109              FLASH_ACCTRL_WAITP_109WS
#define FLASH_LATENCY_110              FLASH_ACCTRL_WAITP_110WS
#define FLASH_LATENCY_111              FLASH_ACCTRL_WAITP_111WS
#define FLASH_LATENCY_112              FLASH_ACCTRL_WAITP_112WS
#define FLASH_LATENCY_113              FLASH_ACCTRL_WAITP_113WS
#define FLASH_LATENCY_114              FLASH_ACCTRL_WAITP_114WS
#define FLASH_LATENCY_115              FLASH_ACCTRL_WAITP_115WS
#define FLASH_LATENCY_116              FLASH_ACCTRL_WAITP_116WS
#define FLASH_LATENCY_117              FLASH_ACCTRL_WAITP_117WS
#define FLASH_LATENCY_118              FLASH_ACCTRL_WAITP_118WS
#define FLASH_LATENCY_119              FLASH_ACCTRL_WAITP_119WS
#define FLASH_LATENCY_120              FLASH_ACCTRL_WAITP_120WS
#define FLASH_LATENCY_121              FLASH_ACCTRL_WAITP_121WS
#define FLASH_LATENCY_122              FLASH_ACCTRL_WAITP_122WS
#define FLASH_LATENCY_123              FLASH_ACCTRL_WAITP_123WS
#define FLASH_LATENCY_124              FLASH_ACCTRL_WAITP_124WS
#define FLASH_LATENCY_125              FLASH_ACCTRL_WAITP_125WS
#define FLASH_LATENCY_126              FLASH_ACCTRL_WAITP_126WS
#define FLASH_LATENCY_127              FLASH_ACCTRL_WAITP_127WS
#else
#define FLASH_LATENCY_0                FLASH_CTRL_WS_0WS
#define FLASH_LATENCY_1                FLASH_CTRL_WS_1WS
#define FLASH_LATENCY_2                FLASH_CTRL_WS_2WS
#define FLASH_LATENCY_3                FLASH_CTRL_WS_3WS
#define FLASH_LATENCY_4                FLASH_CTRL_WS_4WS
#define FLASH_LATENCY_5                FLASH_CTRL_WS_5WS
#define FLASH_LATENCY_6                FLASH_CTRL_WS_6WS
#define FLASH_LATENCY_7                FLASH_CTRL_WS_7WS
#define FLASH_LATENCY_8                FLASH_CTRL_WS_8WS
#define FLASH_LATENCY_9                FLASH_CTRL_WS_9WS
#define FLASH_LATENCY_10               FLASH_CTRL_WS_10WS
#define FLASH_LATENCY_11               FLASH_CTRL_WS_11WS
#define FLASH_LATENCY_12               FLASH_CTRL_WS_12WS
#define FLASH_LATENCY_13               FLASH_CTRL_WS_13WS
#define FLASH_LATENCY_14               FLASH_CTRL_WS_14WS
#define FLASH_LATENCY_15               FLASH_CTRL_WS_15WS
#define FLASH_LATENCY_16               FLASH_CTRL_WS_16WS
#define FLASH_LATENCY_17               FLASH_CTRL_WS_17WS
#define FLASH_LATENCY_18               FLASH_CTRL_WS_18WS
#define FLASH_LATENCY_19               FLASH_CTRL_WS_19WS
#define FLASH_LATENCY_20               FLASH_CTRL_WS_20WS
#define FLASH_LATENCY_21               FLASH_CTRL_WS_21WS
#define FLASH_LATENCY_22               FLASH_CTRL_WS_22WS
#define FLASH_LATENCY_23               FLASH_CTRL_WS_23WS
#define FLASH_LATENCY_24               FLASH_CTRL_WS_24WS
#define FLASH_LATENCY_25               FLASH_CTRL_WS_25WS
#define FLASH_LATENCY_26               FLASH_CTRL_WS_26WS
#define FLASH_LATENCY_27               FLASH_CTRL_WS_27WS
#define FLASH_LATENCY_28               FLASH_CTRL_WS_28WS
#define FLASH_LATENCY_29               FLASH_CTRL_WS_29WS
#define FLASH_LATENCY_30               FLASH_CTRL_WS_30WS
#define FLASH_LATENCY_31               FLASH_CTRL_WS_31WS
#endif /* APM32F40xxx || APM32F41xxx | APM32F465xx || APM32F411xx */
/*--------------------------------------------------------------------------------------------------------------*/

/**
  * @}
  */

#if defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
/** @defgroup FLASH_Prefetch_Mode FLASH Prefetch Mode
  * @{
  */
#define FLASH_PREFETCH_MODE_DOUBLE_ACCESS    0x00000000U                          /* Prefetch initiated after cacheline accessed twice */
#define FLASH_PREFETCH_MODE_SINGLE_ACCESS    ((uint32_t)FLASH_ACCTRL_PREF_TIMING) /* Prefetch initiated after single cache/buffer access */
/**
  * @}
  */

/** @defgroup FLASH_Read_Interrupt_Time FLASH Read Interrupt Time
  * @{
  */
#define FLASH_READ_INTERRUPT_FAST    0x00000000U                            /* Fast read interrupt time */
#define FLASH_READ_INTERRUPT_SLOW    ((uint32_t)FLASH_ACCTRL_RD_INT_TIMING) /* Slow read interrupt time */
/**
  * @}
  */
#endif /* APM32F423xx || APM32F425xx || APM32F427xx */

/** @defgroup FLASHEx_Banks FLASH Banks
  * @{
  */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) || \
    defined(APM32F465xx) || defined(APM32F411xx) || defined(APM32F403xx) || defined(APM32F402xx) || \
    defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define FLASH_BANK_1     1U /*!< Bank 1   */
#endif /* APM32F40xxx || APM32F41xxx || APM32F465xx || APM32F411xx || APM32F403xx || APM32F402xx || APM32F423xx || APM32F425xx || APM32F427xx */
/**
  * @}
  */

/** @defgroup FLASHEx_MassErase_bit FLASH Mass Erase bit
  * @{
  */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) || \
    defined(APM32F465xx) || defined(APM32F411xx) || defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define FLASH_MER_BIT     (FLASH_CTRL_MERS) /*!< only 1 MER Bit */
#endif /* APM32F40xxx || APM32F41xxx || APM32F465xx || APM32F411xx || APM32F423xx || APM32F425xx || APM32F427xx */
/**
  * @}
  */

/** @defgroup FLASHEx_Sectors FLASH Sectors
  * @{
  */

/*--------------------------------------- APM32F40xxx/APM32F41xxx/APM32F465xx -------------------------------------*/
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) || defined(APM32F465xx) || \
    defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
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
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
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
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) || defined(APM32F465xx) || \
    defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
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
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */
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

/*---------------------------------- APM32F402/403xx ------------------------------*/
#if  defined(APM32F403xx) || defined(APM32F402xx)
#define OB_WRP_PAGES0TO3               0x00000001U   /*!< Write protection of page 0 to 3 */
#define OB_WRP_PAGES4TO7               0x00000002U   /*!< Write protection of page 4 to 7 */
#define OB_WRP_PAGES8TO11              0x00000004U   /*!< Write protection of page 8 to 11 */
#define OB_WRP_PAGES12TO15             0x00000008U   /*!< Write protection of page 12 to 15 */
#define OB_WRP_PAGES16TO19             0x00000010U   /*!< Write protection of page 16 to 19 */
#define OB_WRP_PAGES20TO23             0x00000020U   /*!< Write protection of page 20 to 23 */
#define OB_WRP_PAGES24TO27             0x00000040U   /*!< Write protection of page 24 to 27 */
#define OB_WRP_PAGES28TO31             0x00000080U   /*!< Write protection of page 28 to 31 */
#define OB_WRP_PAGES32TO35             0x00000100U   /*!< Write protection of page 32 to 35 */
#define OB_WRP_PAGES36TO39             0x00000200U   /*!< Write protection of page 36 to 39 */
#define OB_WRP_PAGES40TO43             0x00000400U   /*!< Write protection of page 40 to 43 */
#define OB_WRP_PAGES44TO47             0x00000800U   /*!< Write protection of page 44 to 47 */
#define OB_WRP_PAGES48TO51             0x00001000U   /*!< Write protection of page 48 to 51 */
#define OB_WRP_PAGES52TO55             0x00002000U   /*!< Write protection of page 52 to 55 */
#define OB_WRP_PAGES56TO59             0x00004000U   /*!< Write protection of page 56 to 59 */
#define OB_WRP_PAGES60TO63             0x00008000U   /*!< Write protection of page 60 to 63 */
#define OB_WRP_PAGES64TO67             0x00010000U   /*!< Write protection of page 64 to 67 */
#define OB_WRP_PAGES68TO71             0x00020000U   /*!< Write protection of page 68 to 71 */
#define OB_WRP_PAGES72TO75             0x00040000U   /*!< Write protection of page 72 to 75 */
#define OB_WRP_PAGES76TO79             0x00080000U   /*!< Write protection of page 76 to 79 */
#define OB_WRP_PAGES80TO83             0x00100000U   /*!< Write protection of page 80 to 83 */
#define OB_WRP_PAGES84TO87             0x00200000U   /*!< Write protection of page 84 to 87 */
#define OB_WRP_PAGES88TO91             0x00400000U   /*!< Write protection of page 88 to 91 */
#define OB_WRP_PAGES92TO95             0x00800000U   /*!< Write protection of page 92 to 95 */
#define OB_WRP_PAGES96TO99             0x01000000U   /*!< Write protection of page 96 to 99 */
#define OB_WRP_PAGES100TO103           0x02000000U   /*!< Write protection of page 100 to 103 */
#define OB_WRP_PAGES104TO107           0x04000000U   /*!< Write protection of page 104 to 107 */
#define OB_WRP_PAGES108TO111           0x08000000U   /*!< Write protection of page 108 to 111 */
#define OB_WRP_PAGES112TO115           0x10000000U   /*!< Write protection of page 112 to 115 */
#define OB_WRP_PAGES116TO119           0x20000000U   /*!< Write protection of page 115 to 119 */
#define OB_WRP_PAGES120TO123           0x40000000U   /*!< Write protection of page 120 to 123 */
#define OB_WRP_PAGES124TO127           0x80000000U   /*!< Write protection of page 124 to 127 */
#endif /* APM32F403xx || APM32F402xx */
/*-----------------------------------------------------------------------------------------------------*/

#if defined(APM32F403xx) || defined(APM32F402xx)
#define OB_WRP_PAGES0TO15MASK          0x000000FFU
#define OB_WRP_PAGES16TO31MASK         0x0000FF00U
#define OB_WRP_PAGES32TO47MASK         0x00FF0000U
#define OB_WRP_PAGES48TO127MASK        0xFF000000U
#endif /* APM32F403xx || APM32F402xx */
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

/** @defgroup FLASH_Exported_Macros FLASH Exported Macros
 *  @brief macros to control FLASH features
 *  @{
 */

#if defined(APM32F403xx) || defined(APM32F402xx)
/** @defgroup FLASH_Half_Cycle FLASH Half Cycle
 *  @brief macros to handle FLASH half cycle
 * @{
 */

/**
  * @brief  Enable the FLASH half cycle access.
  * @note   half cycle access can only be used with a low-frequency clock of less than
            8 MHz that can be obtained with the use of HSI or HSE but not of PLL.
  * @retval None
  */
#define __DAL_FLASH_HALF_CYCLE_ACCESS_ENABLE()  (FLASH->CTRL1 |= FLASH_CTRL1_HCAEN)

/**
  * @brief  Disable the FLASH half cycle access.
  * @note   half cycle access can only be used with a low-frequency clock of less than
            8 MHz that can be obtained with the use of HSI or HSE but not of PLL.
  * @retval None
  */
#define __DAL_FLASH_HALF_CYCLE_ACCESS_DISABLE() (FLASH->CTRL1 &= (~FLASH_CTRL1_HCAEN))

/**
  * @}
  */

#endif /* APM32F403xx || APM32F402xx */

#if defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
/**
  * @brief  Set FLASH Prefetch Mode
  * @param  __MODE__: FLASH Prefetch Mode
  *                   FLASH_PREFETCH_MODE_DOUBLE_ACCESS: Prefetch triggered after two accesses to same cacheline
  *                   FLASH_PREFETCH_MODE_SINGLE_ACCESS: Prefetch triggered after cache/buffer is accessed
  * @retval None
  */
#define __DAL_FLASH_SET_PREFETCH_MODE(__MODE__) (FLASH->ACCTRL = (FLASH->ACCTRL & (~FLASH_ACCTRL_PREF_TIMING)) | (__MODE__))

/**
  * @brief  Enable the FLASH read interrupt.
  * @retval none
  */
#define __DAL_FLASH_READ_INT_ENABLE() (FLASH->ACCTRL |= FLASH_ACCTRL_RD_INT_EN)

/**
  * @brief  Disable the FLASH read interrupt.
  * @retval none
  */
#define __DAL_FLASH_READ_INT_DISABLE() (FLASH->ACCTRL &= (~FLASH_ACCTRL_RD_INT_EN))

/**
  * @brief  Set FLASH Read Interrupt Time
  * @param  __MODE__: FLASH Read Interrupt Time
  *                   FLASH_READ_INTERRUPT_FAST: Fast read interrupt time
  *                   FLASH_READ_INTERRUPT_SLOW: Slow read interrupt time
  * @retval None
  */
#define __DAL_FLASH_SET_TRC_MODE(__MODE__) (FLASH->ACCTRL = (FLASH->ACCTRL & (~FLASH_ACCTRL_RD_INT_TIMING)) | (__MODE__))

#endif /* APM32F423xx || APM32F425xx || APM32F427xx */

/**
  * @}
  */

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

#if defined (APM32F403xx) || defined (APM32F402xx)
DAL_StatusTypeDef DAL_FLASHEx_OBErase(void);
uint32_t          DAL_FLASHEx_OBGetUserData(uint32_t DATAAdress);
#endif /* APM32F403xx || APM32F402xx */

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
#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) || defined(APM32F465xx) || \
    defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define FLASH_SECTOR_TOTAL  12U
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

/*--------------------------------- APM32F411xx -------------------------------------*/
#if defined(APM32F411xx)
#define FLASH_SECTOR_TOTAL  8U
#endif /* APM32F411xx */

/*--------------------------------- APM32F402/403xx -------------------------------------*/
#if defined(APM32F403xx) || defined(APM32F402xx)
#define FLASH_SIZE_DATA_REGISTER  0x1FFFF7E0U
#define OBR_REG_INDEX                1U
#endif /* APM32F403xx || APM32F402xx */

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

#if defined(APM32F403xx) || defined(APM32F402xx)
#define IS_FLASH_NB_PAGES(ADDRESS,NBPAGES) (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x80U) ? ((ADDRESS)+((NBPAGES)*FLASH_PAGE_SIZE)-1 <= 0x0801FFFFU) : \
                                           (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x40U) ? ((ADDRESS)+((NBPAGES)*FLASH_PAGE_SIZE)-1 <= 0x0800FFFFU) : \
                                           (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x20U) ? ((ADDRESS)+((NBPAGES)*FLASH_PAGE_SIZE)-1 <= 0x08007FFFU) : \
                                           ((ADDRESS)+((NBPAGES)*FLASH_PAGE_SIZE)-1 <= 0x08003FFFU))))
#endif /* APM32F403xx || APM32F402xx */

#if defined(APM32F403xx) || defined(APM32F402xx)
#define IS_FLASH_TYPEERASE(VALUE)(((VALUE) == FLASH_TYPEERASE_PAGES) || \
                                  ((VALUE) == FLASH_TYPEERASE_MASSERASE))
#else
#define IS_FLASH_TYPEERASE(VALUE)(((VALUE) == FLASH_TYPEERASE_SECTORS) || \
                                  ((VALUE) == FLASH_TYPEERASE_MASSERASE))
#endif /* APM32F403xx || APM32F402xx */

#define IS_VOLTAGERANGE(RANGE)(((RANGE) == FLASH_VOLTAGE_RANGE_1) || \
                               ((RANGE) == FLASH_VOLTAGE_RANGE_2) || \
                               ((RANGE) == FLASH_VOLTAGE_RANGE_3) || \
                               ((RANGE) == FLASH_VOLTAGE_RANGE_4))

#define IS_WRPSTATE(VALUE)(((VALUE) == OB_WRPSTATE_DISABLE) || \
                           ((VALUE) == OB_WRPSTATE_ENABLE))

#define IS_OPTIONBYTE(VALUE)(((VALUE) <= (OPTIONBYTE_WRP|OPTIONBYTE_RDP|OPTIONBYTE_USER|OPTIONBYTE_BOR)))

#if defined (APM32F405xx) || defined (APM32F407xx) || defined (APM32F415xx) || defined (APM32F417xx) || defined (APM32F411xx) || defined (APM32F465xx) || \
    defined (APM32F423xx) || defined (APM32F425xx) || defined (APM32F427xx)
#define IS_OB_RDP_LEVEL(LEVEL) (((LEVEL) == OB_RDP_LEVEL_0) ||\
                                ((LEVEL) == OB_RDP_LEVEL_1) ||\
                                ((LEVEL) == OB_RDP_LEVEL_2))
#else
#define IS_OB_RDP_LEVEL(LEVEL) (((LEVEL) == OB_RDP_LEVEL_0) ||\
                                ((LEVEL) == OB_RDP_LEVEL_1))
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F411xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

#define IS_OB_IWDT_SOURCE(SOURCE) (((SOURCE) == OB_IWDT_SW) || ((SOURCE) == OB_IWDT_HW))

#define IS_OB_STOP_SOURCE(SOURCE) (((SOURCE) == OB_STOP_NO_RST) || ((SOURCE) == OB_STOP_RST))

#define IS_OB_STDBY_SOURCE(SOURCE) (((SOURCE) == OB_STDBY_NO_RST) || ((SOURCE) == OB_STDBY_RST))

#if defined(APM32F403xx) || defined(APM32F402xx)
#define IS_OB_ROMSEL_SOURCE(SOURCE) (((SOURCE) == OB_SEL_INFO) || ((SOURCE) == OB_SEL_ROM))
#define IS_OB_DATA_ADDRESS(ADDRESS) (((ADDRESS) == OB_DATA_ADDRESS_DATA0) || ((ADDRESS) == OB_DATA_ADDRESS_DATA1))
#endif /* APM32F403xx || APM32F402xx */

#define IS_OB_BOR_LEVEL(LEVEL) (((LEVEL) == OB_BOR_LEVEL1) || ((LEVEL) == OB_BOR_LEVEL2) ||\
                                ((LEVEL) == OB_BOR_LEVEL3) || ((LEVEL) == OB_BOR_OFF))

#if defined(APM32F411xx)
#define IS_PCROPSTATE(VALUE)(((VALUE) == OB_PCROP_STATE_DISABLE) || \
                             ((VALUE) == OB_PCROP_STATE_ENABLE))
#endif /* APM32F411xx */

#if defined(APM32F411xx)
#define IS_OBEX(VALUE)(((VALUE) == OPTIONBYTE_PCROP))
#endif /* APM32F411xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx) || defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define IS_FLASH_LATENCY(LATENCY) (((LATENCY) == FLASH_LATENCY_0)  || \
                                   ((LATENCY) == FLASH_LATENCY_1)  || \
                                   ((LATENCY) == FLASH_LATENCY_2)  || \
                                   ((LATENCY) == FLASH_LATENCY_3)  || \
                                   ((LATENCY) == FLASH_LATENCY_4)  || \
                                   ((LATENCY) == FLASH_LATENCY_5)  || \
                                   ((LATENCY) == FLASH_LATENCY_6)  || \
                                   ((LATENCY) == FLASH_LATENCY_7))
#else
#define IS_FLASH_LATENCY(LATENCY) (((LATENCY) == FLASH_LATENCY_0)  || ((LATENCY) == FLASH_LATENCY_8)  || ((LATENCY) == FLASH_LATENCY_16) || ((LATENCY) == FLASH_LATENCY_24) || \
                                   ((LATENCY) == FLASH_LATENCY_1)  || ((LATENCY) == FLASH_LATENCY_9)  || ((LATENCY) == FLASH_LATENCY_17) || ((LATENCY) == FLASH_LATENCY_25) || \
                                   ((LATENCY) == FLASH_LATENCY_2)  || ((LATENCY) == FLASH_LATENCY_10) || ((LATENCY) == FLASH_LATENCY_18) || ((LATENCY) == FLASH_LATENCY_26) || \
                                   ((LATENCY) == FLASH_LATENCY_3)  || ((LATENCY) == FLASH_LATENCY_11) || ((LATENCY) == FLASH_LATENCY_19) || ((LATENCY) == FLASH_LATENCY_27) || \
                                   ((LATENCY) == FLASH_LATENCY_4)  || ((LATENCY) == FLASH_LATENCY_12) || ((LATENCY) == FLASH_LATENCY_20) || ((LATENCY) == FLASH_LATENCY_28) || \
                                   ((LATENCY) == FLASH_LATENCY_5)  || ((LATENCY) == FLASH_LATENCY_13) || ((LATENCY) == FLASH_LATENCY_21) || ((LATENCY) == FLASH_LATENCY_29) || \
                                   ((LATENCY) == FLASH_LATENCY_6)  || ((LATENCY) == FLASH_LATENCY_14) || ((LATENCY) == FLASH_LATENCY_22) || ((LATENCY) == FLASH_LATENCY_30) || \
                                   ((LATENCY) == FLASH_LATENCY_7)  || ((LATENCY) == FLASH_LATENCY_15) || ((LATENCY) == FLASH_LATENCY_23) || ((LATENCY) == FLASH_LATENCY_31))
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F465xx || APM32F411xx || APM32F423xx || APM32F425xx || APM32F427xx */

#if defined(APM32F403xx) || defined(APM32F402xx)
#define IS_OB_WRP(PAGE) (((PAGE) != 0x0000000U))
#endif /* APM32F403xx || APM32F402xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) ||\
    defined(APM32F465xx) || defined(APM32F411xx) || defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx) || \
    defined(APM32F403xx) || defined(APM32F402xx)
#define IS_FLASH_BANK(BANK) (((BANK) == FLASH_BANK_1))
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F465xx || APM32F411xx || APM32F423xx || APM32F425xx || APM32F427xx || APM32F403xx || APM32F402xx */

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) || defined(APM32F465xx) || \
    defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define IS_FLASH_SECTOR(SECTOR) (((SECTOR) == FLASH_SECTOR_0)   || ((SECTOR) == FLASH_SECTOR_1)   ||\
                                 ((SECTOR) == FLASH_SECTOR_2)   || ((SECTOR) == FLASH_SECTOR_3)   ||\
                                 ((SECTOR) == FLASH_SECTOR_4)   || ((SECTOR) == FLASH_SECTOR_5)   ||\
                                 ((SECTOR) == FLASH_SECTOR_6)   || ((SECTOR) == FLASH_SECTOR_7)   ||\
                                 ((SECTOR) == FLASH_SECTOR_8)   || ((SECTOR) == FLASH_SECTOR_9)   ||\
                                 ((SECTOR) == FLASH_SECTOR_10)  || ((SECTOR) == FLASH_SECTOR_11))
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

#if defined(APM32F411xx)
#define IS_FLASH_SECTOR(SECTOR) (((SECTOR) == FLASH_SECTOR_0)   || ((SECTOR) == FLASH_SECTOR_1)   ||\
                                 ((SECTOR) == FLASH_SECTOR_2)   || ((SECTOR) == FLASH_SECTOR_3)   ||\
                                 ((SECTOR) == FLASH_SECTOR_4)   || ((SECTOR) == FLASH_SECTOR_5)   ||\
                                 ((SECTOR) == FLASH_SECTOR_6)   || ((SECTOR) == FLASH_SECTOR_7))
#endif /* APM32F411xx */

#if defined (APM32F403xx) || defined (APM32F402xx)
#define IS_FLASH_PROGRAM_ADDRESS(ADDRESS) (((ADDRESS) >= FLASH_BASE) && ((ADDRESS) <= FLASH_END))
#endif /* APM32F403xx || APM32F402xx */

#define IS_FLASH_ADDRESS(ADDRESS) ((((ADDRESS) >= FLASH_BASE) && ((ADDRESS) <= FLASH_END)) || \
                                   (((ADDRESS) >= FLASH_OTP_BASE) && ((ADDRESS) <= FLASH_OTP_END)))

#define IS_FLASH_NBSECTORS(NBSECTORS) (((NBSECTORS) != 0) && ((NBSECTORS) <= FLASH_SECTOR_TOTAL))

#if defined(APM32F405xx) || defined(APM32F407xx) || defined(APM32F415xx) || defined(APM32F417xx) || defined(APM32F465xx) || \
    defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
#define IS_OB_WRP_SECTOR(SECTOR)((((SECTOR) & 0xFFFFF000U) == 0x00000000U) && ((SECTOR) != 0x00000000U))
#endif /* APM32F405xx || APM32F407xx || APM32F415xx || APM32F417xx || APM32F465xx || APM32F423xx || APM32F425xx || APM32F427xx */

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
void FLASH_PageErase(uint32_t PageAddress);
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

