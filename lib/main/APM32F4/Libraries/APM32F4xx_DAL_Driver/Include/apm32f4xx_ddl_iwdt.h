/**
  *
  * @file    apm32f4xx_ddl_iwdt.h
  * @brief   Header file of IWDT DDL module.
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
#ifndef APM32F4xx_DDL_IWDT_H
#define APM32F4xx_DDL_IWDT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined(IWDT)

/** @defgroup IWDT_DDL IWDT
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/** @defgroup IWDT_DDL_Private_Constants IWDT Private Constants
  * @{
  */
#define DDL_IWDT_KEY_RELOAD                 0x0000AAAAU               /*!< IWDT Reload Counter Enable   */
#define DDL_IWDT_KEY_ENABLE                 0x0000CCCCU               /*!< IWDT Peripheral Enable       */
#define DDL_IWDT_KEY_WR_ACCESS_ENABLE       0x00005555U               /*!< IWDT KR Write Access Enable  */
#define DDL_IWDT_KEY_WR_ACCESS_DISABLE      0x00000000U               /*!< IWDT KR Write Access Disable */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup IWDT_DDL_Exported_Constants IWDT Exported Constants
  * @{
  */

/** @defgroup IWDT_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_IWDT_ReadReg function
  * @{
  */
#define DDL_IWDT_STS_PSCUFLG                     IWDT_STS_PSCUFLG                           /*!< Watchdog prescaler value update */
#define DDL_IWDT_STS_CNTUFLG                     IWDT_STS_CNTUFLG                           /*!< Watchdog counter reload value update */
/**
  * @}
  */

/** @defgroup IWDT_DDL_EC_PRESCALER  Prescaler Divider
  * @{
  */
#define DDL_IWDT_PSC_4                0x00000000U                           /*!< Divider by 4   */
#define DDL_IWDT_PSC_8                (IWDT_PSC_PSC_0)                        /*!< Divider by 8   */
#define DDL_IWDT_PSC_16               (IWDT_PSC_PSC_1)                        /*!< Divider by 16  */
#define DDL_IWDT_PSC_32               (IWDT_PSC_PSC_1 | IWDT_PSC_PSC_0)         /*!< Divider by 32  */
#define DDL_IWDT_PSC_64               (IWDT_PSC_PSC_2)                        /*!< Divider by 64  */
#define DDL_IWDT_PSC_128              (IWDT_PSC_PSC_2 | IWDT_PSC_PSC_0)         /*!< Divider by 128 */
#define DDL_IWDT_PSC_256              (IWDT_PSC_PSC_2 | IWDT_PSC_PSC_1)         /*!< Divider by 256 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup IWDT_DDL_Exported_Macros IWDT Exported Macros
  * @{
  */

/** @defgroup IWDT_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in IWDT register
  * @param  __INSTANCE__ IWDT Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_IWDT_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in IWDT register
  * @param  __INSTANCE__ IWDT Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_IWDT_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup IWDT_DDL_Exported_Functions IWDT Exported Functions
  * @{
  */
/** @defgroup IWDT_DDL_EF_Configuration Configuration
  * @{
  */

/**
  * @brief  Start the Independent Watchdog
  * @note   Except if the hardware watchdog option is selected
  * @param  IWDTx IWDT Instance
  * @retval None
  */
__STATIC_INLINE void DDL_IWDT_Enable(IWDT_TypeDef *IWDTx)
{
  WRITE_REG(IWDTx->KEY, DDL_IWDT_KEY_ENABLE);
}

/**
  * @brief  Reloads IWDT counter with value defined in the reload register
  * @param  IWDTx IWDT Instance
  * @retval None
  */
__STATIC_INLINE void DDL_IWDT_ReloadCounter(IWDT_TypeDef *IWDTx)
{
  WRITE_REG(IWDTx->KEY, DDL_IWDT_KEY_RELOAD);
}

/**
  * @brief  Enable write access to IWDT_PR, IWDT_RLR and IWDT_WINR registers
  * @param  IWDTx IWDT Instance
  * @retval None
  */
__STATIC_INLINE void DDL_IWDT_EnableWriteAccess(IWDT_TypeDef *IWDTx)
{
  WRITE_REG(IWDTx->KEY, DDL_IWDT_KEY_WR_ACCESS_ENABLE);
}

/**
  * @brief  Disable write access to IWDT_PR, IWDT_RLR and IWDT_WINR registers
  * @param  IWDTx IWDT Instance
  * @retval None
  */
__STATIC_INLINE void DDL_IWDT_DisableWriteAccess(IWDT_TypeDef *IWDTx)
{
  WRITE_REG(IWDTx->KEY, DDL_IWDT_KEY_WR_ACCESS_DISABLE);
}

/**
  * @brief  Select the prescaler of the IWDT
  * @param  IWDTx IWDT Instance
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_IWDT_PSC_4
  *         @arg @ref DDL_IWDT_PSC_8
  *         @arg @ref DDL_IWDT_PSC_16
  *         @arg @ref DDL_IWDT_PSC_32
  *         @arg @ref DDL_IWDT_PSC_64
  *         @arg @ref DDL_IWDT_PSC_128
  *         @arg @ref DDL_IWDT_PSC_256
  * @retval None
  */
__STATIC_INLINE void DDL_IWDT_SetPrescaler(IWDT_TypeDef *IWDTx, uint32_t Prescaler)
{
  WRITE_REG(IWDTx->PSC, IWDT_PSC_PSC & Prescaler);
}

/**
  * @brief  Get the selected prescaler of the IWDT
  * @param  IWDTx IWDT Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_IWDT_PSC_4
  *         @arg @ref DDL_IWDT_PSC_8
  *         @arg @ref DDL_IWDT_PSC_16
  *         @arg @ref DDL_IWDT_PSC_32
  *         @arg @ref DDL_IWDT_PSC_64
  *         @arg @ref DDL_IWDT_PSC_128
  *         @arg @ref DDL_IWDT_PSC_256
  */
__STATIC_INLINE uint32_t DDL_IWDT_GetPrescaler(IWDT_TypeDef *IWDTx)
{
  return (READ_REG(IWDTx->PSC));
}

/**
  * @brief  Specify the IWDT down-counter reload value
  * @param  IWDTx IWDT Instance
  * @param  Counter Value between Min_Data=0 and Max_Data=0x0FFF
  * @retval None
  */
__STATIC_INLINE void DDL_IWDT_SetReloadCounter(IWDT_TypeDef *IWDTx, uint32_t Counter)
{
  WRITE_REG(IWDTx->CNTRLD, IWDT_CNTRLD_CNTRLD & Counter);
}

/**
  * @brief  Get the specified IWDT down-counter reload value
  * @param  IWDTx IWDT Instance
  * @retval Value between Min_Data=0 and Max_Data=0x0FFF
  */
__STATIC_INLINE uint32_t DDL_IWDT_GetReloadCounter(IWDT_TypeDef *IWDTx)
{
  return (READ_REG(IWDTx->CNTRLD));
}

/**
  * @}
  */

/** @defgroup IWDT_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Check if flag Prescaler Value Update is set or not
  * @param  IWDTx IWDT Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_IWDT_IsActiveFlag_PVU(IWDT_TypeDef *IWDTx)
{
  return ((READ_BIT(IWDTx->STS, IWDT_STS_PSCUFLG) == (IWDT_STS_PSCUFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Check if flag Reload Value Update is set or not
  * @param  IWDTx IWDT Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_IWDT_IsActiveFlag_RVU(IWDT_TypeDef *IWDTx)
{
  return ((READ_BIT(IWDTx->STS, IWDT_STS_CNTUFLG) == (IWDT_STS_CNTUFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Check if flags Prescaler & Reload Value Update are reset or not
  * @param  IWDTx IWDT Instance
  * @retval State of bits (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_IWDT_IsReady(IWDT_TypeDef *IWDTx)
{
  return ((READ_BIT(IWDTx->STS, IWDT_STS_PSCUFLG | IWDT_STS_CNTUFLG) == 0U) ? 1UL : 0UL);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* IWDT */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_IWDT_H */
