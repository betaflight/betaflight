/**
  *
  * @file    apm32f4xx_ddl_rng.h
  * @brief   Header file of RNG DDL module.
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
#ifndef APM32F4xx_DDL_RNG_H
#define APM32F4xx_DDL_RNG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (RNG)

/** @defgroup RNG_DDL RNG
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup RNG_DDL_Exported_Constants RNG Exported Constants
  * @{
  */


/** @defgroup RNG_DDL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with DDL_RNG_ReadReg function
  * @{
  */
#define DDL_RNG_STS_DATARDY RNG_STS_DATARDY    /*!< Register contains valid random data */
#define DDL_RNG_STS_CLKERCSTS RNG_STS_CLKERCSTS    /*!< Clock error current status */
#define DDL_RNG_STS_FSCSTS RNG_STS_FSCSTS    /*!< Seed error current status */
#define DDL_RNG_STS_CLKERINT RNG_STS_CLKERINT    /*!< Clock error interrupt status */
#define DDL_RNG_STS_FSINT RNG_STS_FSINT    /*!< Seed error interrupt status */
/**
  * @}
  */

/** @defgroup RNG_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_RNG_ReadReg and  DDL_RNG_WriteReg macros
  * @{
  */
#define DDL_RNG_CTRL_INTEN   RNG_CTRL_INTEN      /*!< RNG Interrupt enable */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup RNG_DDL_Exported_Macros RNG Exported Macros
  * @{
  */

/** @defgroup RNG_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in RNG register
  * @param  __INSTANCE__ RNG Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_RNG_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in RNG register
  * @param  __INSTANCE__ RNG Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_RNG_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup RNG_DDL_Exported_Functions RNG Exported Functions
  * @{
  */
/** @defgroup RNG_DDL_EF_Configuration RNG Configuration functions
  * @{
  */

/**
  * @brief  Enable Random Number Generation
  * @param  RNGx RNG Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RNG_Enable(RNG_TypeDef *RNGx)
{
  SET_BIT(RNGx->CTRL, RNG_CTRL_RNGEN);
}

/**
  * @brief  Disable Random Number Generation
  * @param  RNGx RNG Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RNG_Disable(RNG_TypeDef *RNGx)
{
  CLEAR_BIT(RNGx->CTRL, RNG_CTRL_RNGEN);
}

/**
  * @brief  Check if Random Number Generator is enabled
  * @param  RNGx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RNG_IsEnabled(RNG_TypeDef *RNGx)
{
  return ((READ_BIT(RNGx->CTRL, RNG_CTRL_RNGEN) == (RNG_CTRL_RNGEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup RNG_DDL_EF_FLAG_Management FLAG Management
  * @{
  */

/**
  * @brief  Indicate if the RNG Data ready Flag is set or not
  * @param  RNGx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RNG_IsActiveFlag_DRDY(RNG_TypeDef *RNGx)
{
  return ((READ_BIT(RNGx->STS, RNG_STS_DATARDY) == (RNG_STS_DATARDY)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate if the Clock Error Current Status Flag is set or not
  * @param  RNGx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RNG_IsActiveFlag_CECS(RNG_TypeDef *RNGx)
{
  return ((READ_BIT(RNGx->STS, RNG_STS_CLKERCSTS) == (RNG_STS_CLKERCSTS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate if the Seed Error Current Status Flag is set or not
  * @param  RNGx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RNG_IsActiveFlag_SECS(RNG_TypeDef *RNGx)
{
  return ((READ_BIT(RNGx->STS, RNG_STS_FSCSTS) == (RNG_STS_FSCSTS)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate if the Clock Error Interrupt Status Flag is set or not
  * @param  RNGx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RNG_IsActiveFlag_CEIS(RNG_TypeDef *RNGx)
{
  return ((READ_BIT(RNGx->STS, RNG_STS_CLKERINT) == (RNG_STS_CLKERINT)) ? 1UL : 0UL);
}

/**
  * @brief  Indicate if the Seed Error Interrupt Status Flag is set or not
  * @param  RNGx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RNG_IsActiveFlag_SEIS(RNG_TypeDef *RNGx)
{
  return ((READ_BIT(RNGx->STS, RNG_STS_FSINT) == (RNG_STS_FSINT)) ? 1UL : 0UL);
}

/**
  * @brief  Clear Clock Error interrupt Status (CEIS) Flag
  * @param  RNGx RNG Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RNG_ClearFlag_CEIS(RNG_TypeDef *RNGx)
{
  WRITE_REG(RNGx->STS, ~RNG_STS_CLKERINT);
}

/**
  * @brief  Clear Seed Error interrupt Status (SEIS) Flag
  * @param  RNGx RNG Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RNG_ClearFlag_SEIS(RNG_TypeDef *RNGx)
{
  WRITE_REG(RNGx->STS, ~RNG_STS_FSINT);
}

/**
  * @}
  */

/** @defgroup RNG_DDL_EF_IT_Management IT Management
  * @{
  */

/**
  * @brief  Enable Random Number Generator Interrupt
  *         (applies for either Seed error, Clock Error or Data ready interrupts)
  * @param  RNGx RNG Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RNG_EnableIT(RNG_TypeDef *RNGx)
{
  SET_BIT(RNGx->CTRL, RNG_CTRL_INTEN);
}

/**
  * @brief  Disable Random Number Generator Interrupt
  *         (applies for either Seed error, Clock Error or Data ready interrupts)
  * @param  RNGx RNG Instance
  * @retval None
  */
__STATIC_INLINE void DDL_RNG_DisableIT(RNG_TypeDef *RNGx)
{
  CLEAR_BIT(RNGx->CTRL, RNG_CTRL_INTEN);
}

/**
  * @brief  Check if Random Number Generator Interrupt is enabled
  *         (applies for either Seed error, Clock Error or Data ready interrupts)
  * @param  RNGx RNG Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_RNG_IsEnabledIT(RNG_TypeDef *RNGx)
{
  return ((READ_BIT(RNGx->CTRL, RNG_CTRL_INTEN) == (RNG_CTRL_INTEN)) ? 1UL : 0UL);
}

/**
  * @}
  */

/** @defgroup RNG_DDL_EF_Data_Management Data Management
  * @{
  */

/**
  * @brief  Return32-bit Random Number value
  * @param  RNGx RNG Instance
  * @retval Generated 32-bit random value
  */
__STATIC_INLINE uint32_t DDL_RNG_ReadRandData32(RNG_TypeDef *RNGx)
{
  return (uint32_t)(READ_REG(RNGx->DATA));
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup RNG_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */
ErrorStatus DDL_RNG_DeInit(RNG_TypeDef *RNGx);

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/**
  * @}
  */

/**
  * @}
  */

#endif /* RNG */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __APM32F4xx_DDL_RNG_H */

