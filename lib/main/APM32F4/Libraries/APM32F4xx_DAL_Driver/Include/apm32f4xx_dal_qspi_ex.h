/**
  *
  * @file    apm32f4xx_dal_qspi_ex.h
  * @brief   Header file of QSPI DAL Extension module.
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
  * Copyright (C) 2025 Geehy Semiconductor.
  * All rights reserved.
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DAL_QSPI_EX_H
#define APM32F4xx_DAL_QSPI_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal_def.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup QSPIEx
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup QSPIEx_Exported_Types QSPI Exported Types
  * @{
  */

/**
  * @brief  QSPI XIP structure definition
  */
typedef struct
{
    uint32_t Instruction;             /*!< Specifies the Instruction to be sent.
                                            This parameter can be a value between 0x00 and 0xFF */

    uint32_t WrapCode;                /*!< Specifies the Wrap transfer code.
                                            This parameter can be a value between 0x00 and 0xFF */

    uint32_t AddressSize;             /*!< Specifies the Address Size.
                                            This parameter can be a value of @ref QSPI_XIP_AddressSize */

    uint32_t InstructionMode;         /*!< Specifies the Instruction Mode.
                                            This parameter can be a value of @ref QSPI_XIP_InstructionMode */

    uint32_t InstructionSize;         /*!< Specifies the Instruction Size.
                                            This parameter can be a value of @ref QSPI_XIP_InstructionSize */

    uint32_t FrameFormat;             /*!< Specifies the frame format.
                                            This parameter can be a value of @ref QSPI_XIP_FrameFormat */

    uint32_t DummyCycles;             /*!< Specifies the Number of Dummy Cycles.
                                            This parameter can be a number between 0 and 31 */

    uint32_t Endianness;              /*!< Specifies the Endianness for external memory access.
                                            This parameter can be a value of @ref QSPI_External_Memory_Access_Format */

    FunctionalState ContinuousMode;    /*!< Specifies the Continuous Mode.
                                            This parameter can be a value of ENABLE or DISABLE */

    FunctionalState PrefetchMode;      /*!< Specifies the Prefetch Mode.
                                            This parameter can be a value of ENABLE or DISABLE */
} QSPI_XIPTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup QSPIEx_Exported_Constants QSPIEx Exported Constants
  * @{
  */

/** @defgroup QSPI_External_Memory_Access_Format QSPI External Memory Access Format
  * @{
  */
#define QSPI_XIP_MEM_ACCESS_FORMAT_BIG_ENDIAN     0x00000000U  /*!< Big Endian format for external memory access */
#define QSPI_XIP_MEM_ACCESS_FORMAT_LITTLE_ENDIAN  ((uint32_t)QSPI_SWITCH_LITTLE_ENDIAN_EN)  /*!< Little Endian format for external memory access */
/**
  * @}
  */

/** @defgroup QSPI_XIP_FrameFormat QSPI XIP Frame Format
 * @{
 */
#define QSPI_XIP_FRAME_FORMAT_DUAL    QSPI_XIP_CTRL_FRF_0     /*!< QSPI Dual frame format     */
#define QSPI_XIP_FRAME_FORMAT_QUAD    QSPI_XIP_CTRL_FRF_1     /*!< QSPI Quad frame format     */
/**
  * @}
  */

/** @defgroup QSPI_XIP_InstructionSize QSPI XIP Instruction Size
 * @{
 */
#define QSPI_XIP_INSTRUCTION_SIZE_NONE        0x00000000U                                       /*!< No instruction         */
#define QSPI_XIP_INSTRUCTION_SIZE_4_BITS      QSPI_XIP_CTRL_INST_L_0                            /*!< Instruction on 4 bits  */
#define QSPI_XIP_INSTRUCTION_SIZE_8_BITS      QSPI_XIP_CTRL_INST_L_1                            /*!< Instruction on 8 bits  */
#define QSPI_XIP_INSTRUCTION_SIZE_16_BITS     (QSPI_XIP_CTRL_INST_L_1 | QSPI_XIP_CTRL_INST_L_0) /*!< Instruction on 16 bits */
/**
  * @}
  */

/** @defgroup QSPI_XIP_InstructionMode QSPI XIP Instruction Mode
 * @{
 */
#define QSPI_XIP_INSTRUCTION_STANDARD_INS_ADDR    0x00000000U                   /*!< Send instruction and address in standard SPI mode */
#define QSPI_XIP_INSTRUCTION_STANDARD_INS         QSPI_XIP_CTRL_TRANS_TYPE_0    /*!< Send instruction only in standard SPI mode */
#define QSPI_XIP_INSTRUCTION_FRF_INS_ADDR         QSPI_XIP_CTRL_TRANS_TYPE_1    /*!< Send instruction and address in FRF mode          */
/**
  * @}
  */


/** @defgroup QSPI_XIP_AddressSize QSPI XIP Address Size
 * @{
 */
#define QSPI_XIP_ADDRESS_SIZE_NONE              0x00000000U                                         /*!< No address */
#define QSPI_XIP_ADDRESS_SIZE_4_BITS            (QSPI_XIP_CTRL_ADDR_L_0)                            /*!< Address on 4 bits */
#define QSPI_XIP_ADDRESS_SIZE_8_BITS            (QSPI_XIP_CTRL_ADDR_L_1)                            /*!< Address on 8 bits */
#define QSPI_XIP_ADDRESS_SIZE_12_BITS           (QSPI_XIP_CTRL_ADDR_L_1 | QSPI_XIP_CTRL_ADDR_L_0)   /*!< Address on 12 bits */
#define QSPI_XIP_ADDRESS_SIZE_16_BITS           (QSPI_XIP_CTRL_ADDR_L_2)                            /*!< Address on 16 bits */
#define QSPI_XIP_ADDRESS_SIZE_20_BITS           (QSPI_XIP_CTRL_ADDR_L_2 | QSPI_XIP_CTRL_ADDR_L_0)   /*!< Address on 20 bits */
#define QSPI_XIP_ADDRESS_SIZE_24_BITS           (QSPI_XIP_CTRL_ADDR_L_2 | QSPI_XIP_CTRL_ADDR_L_1)   /*!< Address on 24 bits */
#define QSPI_XIP_ADDRESS_SIZE_28_BITS           (QSPI_XIP_CTRL_ADDR_L_2 | QSPI_XIP_CTRL_ADDR_L_1 |  \
                                                QSPI_XIP_CTRL_ADDR_L_0)                             /*!< Address on 28 bits */
#define QSPI_XIP_ADDRESS_SIZE_32_BITS           (QSPI_XIP_CTRL_ADDR_L_3)                            /*!< Address on 32 bits */
#define QSPI_XIP_ADDRESS_SIZE_36_BITS           (QSPI_XIP_CTRL_ADDR_L_3 | QSPI_XIP_CTRL_ADDR_L_0)   /*!< Address on 36 bits */
#define QSPI_XIP_ADDRESS_SIZE_40_BITS           (QSPI_XIP_CTRL_ADDR_L_3 | QSPI_XIP_CTRL_ADDR_L_1)   /*!< Address on 40 bits */
#define QSPI_XIP_ADDRESS_SIZE_44_BITS           (QSPI_XIP_CTRL_ADDR_L_3 | QSPI_XIP_CTRL_ADDR_L_1 |  \
                                                QSPI_XIP_CTRL_ADDR_L_0)                             /*!< Address on 44 bits */
#define QSPI_XIP_ADDRESS_SIZE_48_BITS           (QSPI_XIP_CTRL_ADDR_L_3 | QSPI_XIP_CTRL_ADDR_L_2)   /*!< Address on 48 bits */
#define QSPI_XIP_ADDRESS_SIZE_52_BITS           (QSPI_XIP_CTRL_ADDR_L_3 | QSPI_XIP_CTRL_ADDR_L_2 |  \
                                                QSPI_XIP_CTRL_ADDR_L_0)                             /*!< Address on 52 bits */
#define QSPI_XIP_ADDRESS_SIZE_56_BITS           (QSPI_XIP_CTRL_ADDR_L_3 | QSPI_XIP_CTRL_ADDR_L_2 |  \
                                                QSPI_XIP_CTRL_ADDR_L_1)                             /*!< Address on 56 bits */
#define QSPI_XIP_ADDRESS_SIZE_60_BITS           (QSPI_XIP_CTRL_ADDR_L_3 | QSPI_XIP_CTRL_ADDR_L_2 |  \
                                                QSPI_XIP_CTRL_ADDR_L_1 | QSPI_XIP_CTRL_ADDR_L_0)    /*!< Address on 60 bits */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/

/** @defgroup QSPIEx_Exported_Macros QSPI Extended Exported Macros
  * @{
  */

#if defined(QSPI_XIP_EN_XIP_EN)
/**
 * @brief Enable the specified QSPI peripheral XIP mode.
 * @param __HANDLE__ QSPI handle.
 */
#define __DAL_QSPI_XIP_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->XIP_EN, QSPI_XIP_EN_XIP_EN)

/**
 * @brief Disable the specified QSPI peripheral XIP mode.
 * @param __HANDLE__ QSPI handle.
 */
#define __DAL_QSPI_XIP_DISABLE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->XIP_EN, QSPI_XIP_EN_XIP_EN)
#endif /* QSPI_XIP_EN_XIP_EN */

#if defined(QSPI_SWITCH_LITTLE_ENDIAN_EN)
/**
  * @brief  Set XIP external memory access format.
  * @param  __HANDLE__ QSPI handle.
  * @param  __FORMAT__ specifies the XIP external memory access format.
  *                   QSPI_XIP_MEM_ACCESS_FORMAT_BIG_ENDIAN: Big Endian format
  *                   QSPI_XIP_MEM_ACCESS_FORMAT_LITTLE_ENDIAN: Little Endian format
  * @retval None
  */
#define __DAL_QSPI_XIP_SET_MEM_ACCESS_FORMAT(__HANDLE__, __FORMAT__)  MODIFY_REG((__HANDLE__)->Instance->SWITCH, QSPI_SWITCH_LITTLE_ENDIAN_EN, (__FORMAT__))

/**
  * @brief  Get XIP external memory access format.
  * @param  __HANDLE__ QSPI handle.
  * @retval The XIP external memory access format.
  *         This can be one of the following values:
  *         - QSPI_XIP_MEM_ACCESS_FORMAT_BIG_ENDIAN: Big Endian format
  *         - QSPI_XIP_MEM_ACCESS_FORMAT_LITTLE_ENDIAN: Little Endian format
  */
#define __DAL_QSPI_XIP_GET_MEM_ACCESS_FORMAT(__HANDLE__)  ((uint32_t)((__HANDLE__)->Instance->SWITCH & QSPI_SWITCH_LITTLE_ENDIAN_EN))

#endif /* QSPI_SWITCH_LITTLE_ENDIAN_EN */

#if defined(QSPI_SWITCH_SWITCH)
/**
  * @brief  Enable XIP external memory access.
  * @param  __HANDLE__ QSPI handle.
  * @note   This macro enables the external memory (0x90000000) access in XIP mode.
  */
#define __DAL_QSPI_XIP_MEM_ACCESS_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->SWITCH, QSPI_SWITCH_SWITCH)

/**
  * @brief  Disable XIP external memory access.
  * @param  __HANDLE__ QSPI handle.
  * @note   This macro disables the external memory (0x90000000) access in XIP mode.
  */
#define __DAL_QSPI_XIP_MEM_ACCESS_DISABLE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->SWITCH, QSPI_SWITCH_SWITCH)
#endif /* QSPI_SWITCH_SWITCH */

#if defined(QSPI_XIP_INCR_INST_INCR_INST)
/**
  * @brief  Set XIP INCR transfer code
  * @param  __HANDLE__ QSPI handle.
  * @param  __XIP_INCR__ specifies the XIP INCR transfer code (0x0000 to 0xFFFF).
  * @retval None
  */
#define __DAL_QSPI_XIP_SET_INCR_CODE(__HANDLE__, __XIP_INCR__)  ((__HANDLE__)->XIP_INCR_INST = (uint16_t)(__XIP_INCR__))

/**
  * @brief  Get XIP INCR transfer code
  * @param  __HANDLE__ QSPI handle.
  * @retval The XIP INCR transfer code (0x0000 to 0xFFFF).
  */
#define __DAL_QSPI_XIP_GET_INCR_CODE(__HANDLE__)  ((__HANDLE__)->XIP_INCR_INST)
#endif /* QSPI_XIP_INCR_INST_INCR_INST */

#if defined(QSPI_XIP_WRAP_INST_WRAP_INST)
/**
  * @brief  Set XIP WRAP transfer code
  * @param  __HANDLE__ QSPI handle.
  * @param  __XIP_WRAP__ specifies the XIP WRAP transfer code (0x0000 to 0xFFFF).
  * @retval None
  */
#define __DAL_QSPI_XIP_SET_WRAP_CODE(__HANDLE__, __XIP_WRAP__)  ((__HANDLE__)->XIP_WRAP_INST = (uint16_t)(__XIP_WRAP__))

/**
  * @brief  Get XIP WRAP transfer code
  * @param  __HANDLE__ QSPI handle.
  * @retval The XIP WRAP transfer code (0x0000 to 0xFFFF).
  */
#define __DAL_QSPI_XIP_GET_WRAP_CODE(__HANDLE__)  ((__HANDLE__)->XIP_WRAP_INST)
#endif /* QSPI_XIP_WRAP_INST_WRAP_INST */

#if defined(QSPI_XIP_CTRL_INST_EN)
/**
  * @brief  Enable XIP instruction state
  * @param  __HANDLE__ QSPI handle.
  * @retval None
  */
#define __DAL_QSPI_XIP_INST_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->XIP_CTRL, QSPI_XIP_CTRL_INST_EN)

/**
  * @brief  Disable XIP instruction state
  * @param  __HANDLE__ QSPI handle.
  * @retval None
  */
#define __DAL_QSPI_XIP_INST_DISABLE(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->XIP_CTRL, QSPI_XIP_CTRL_INST_EN)
#endif /* QSPI_XIP_CTRL_INST_EN */

#if defined(QSPI_XIP_SER_SER)
/**
 * @brief  Enable the XIP slave select signal.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @retval None
 */
#define __DAL_QSPI_XIP_ENABLE_SS(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->XIP_SER, QSPI_XIP_SER_SER)

/**
 * @brief  Disable the XIP slave select signal.
 * @param  __HANDLE__ specifies the QSPI Handle.
 * @retval None
 */
#define __DAL_QSPI_XIP_DISABLE_SS(__HANDLE__)  CLEAR_BIT((__HANDLE__)->Instance->XIP_SER, QSPI_XIP_SER_SER)
#endif /* QSPI_XIP_SER_SER */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup QSPIEx_Exported_Functions
  * @{
  */

/** @addtogroup QSPIEx_Exported_Functions_Group1
 * @{
 */
#if defined(APM32F423xx) || defined(APM32F425xx) || defined(APM32F427xx)
DAL_StatusTypeDef DAL_QSPIEx_MemoryMapped(QSPI_HandleTypeDef *hqspi, QSPI_XIPTypeDef *xipConfig);
#endif /* APM32F423xx || APM32F425xx || APM32F427xx */

/**
  * @}
  */

 /**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/** @defgroup QSPIEx_Private_Macros QSPIEx Private Macros
  * @{
  */
#define IS_QSPI_XIP_INSTRUCTION(__INSTRUCTION__)  ((__INSTRUCTION__) <= 0xFFU)

#define IS_QSPI_XIP_WRAP_CODE(__WRAP_CODE__)  ((__WRAP_CODE__) <= 0xFFU)

#define IS_QSPI_XIP_ADDRESS_SIZE(__SIZE__)  ((__SIZE__) <= QSPI_XIP_ADDRESS_SIZE_60_BITS)

#define IS_QSPI_XIP_FRAME_FORMAT(__FORMAT__)  (((__FORMAT__) == QSPI_XIP_FRAME_FORMAT_DUAL) || \
                                               ((__FORMAT__) == QSPI_XIP_FRAME_FORMAT_QUAD))

#define IS_QSPI_XIP_INSTRUCTION_SIZE(__SIZE__)  (((__SIZE__) == QSPI_XIP_INSTRUCTION_SIZE_NONE) || \
                                                 ((__SIZE__) == QSPI_XIP_INSTRUCTION_SIZE_4_BITS) || \
                                                 ((__SIZE__) == QSPI_XIP_INSTRUCTION_SIZE_8_BITS) || \
                                                 ((__SIZE__) == QSPI_XIP_INSTRUCTION_SIZE_16_BITS))

#define IS_QSPI_XIP_INSTRUCTION_MODE(__MODE__)  (((__MODE__) == QSPI_XIP_INSTRUCTION_STANDARD_INS_ADDR) || \
                                                 ((__MODE__) == QSPI_XIP_INSTRUCTION_STANDARD_INS) || \
                                                 ((__MODE__) == QSPI_XIP_INSTRUCTION_FRF_INS_ADDR))

#define IS_QSPI_XIP_DUMMY_CYCLES(__CYCLES__)  ((__CYCLES__) <= 31U)

#define IS_QSPI_XIP_ENDIANNES(__ENDIANNESS__)  (((__ENDIANNESS__) == QSPI_XIP_MEM_ACCESS_FORMAT_BIG_ENDIAN) || \
                                                    ((__ENDIANNESS__) == QSPI_XIP_MEM_ACCESS_FORMAT_LITTLE_ENDIAN))

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

#endif /* APM32F4xx_DAL_QSPI_EX_H */
