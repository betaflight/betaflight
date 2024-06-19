/**
  *
  * @file    apm32f4xx_ddl_wwdt.h
  * @brief   Header file of WWDT DDL module.
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
#ifndef APM32F4xx_DDL_WWDT_H
#define APM32F4xx_DDL_WWDT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (WWDT)

/** @defgroup WWDT_DDL WWDT
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup WWDT_DDL_Exported_Constants WWDT Exported Constants
  * @{
  */

/** @defgroup WWDT_DDL_EC_IT IT Defines
  * @brief    IT defines which can be used with DDL_WWDT_ReadReg and  DDL_WWDT_WriteReg functions
  * @{
  */
#define DDL_WWDT_CFR_EWIEN                     WWDT_CFR_EWIEN
/**
  * @}
  */

/** @defgroup WWDT_DDL_EC_PRESCALER  PRESCALER
  * @{
  */
#define DDL_WWDT_PRESCALER_1                 0x00000000u                                               /*!< WWDT counter clock = (PCLK1/4096)/1 */
#define DDL_WWDT_PRESCALER_2                 WWDT_CFR_TBPSC_0                                          /*!< WWDT counter clock = (PCLK1/4096)/2 */
#define DDL_WWDT_PRESCALER_4                 WWDT_CFR_TBPSC_1                                          /*!< WWDT counter clock = (PCLK1/4096)/4 */
#define DDL_WWDT_PRESCALER_8                 (WWDT_CFR_TBPSC_0 | WWDT_CFR_TBPSC_1)                     /*!< WWDT counter clock = (PCLK1/4096)/8 */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup WWDT_DDL_Exported_Macros WWDT Exported Macros
  * @{
  */
/** @defgroup WWDT_DDL_EM_WRITE_READ Common Write and read registers macros
  * @{
  */
/**
  * @brief  Write a value in WWDT register
  * @param  __INSTANCE__ WWDT Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_WWDT_WriteReg(__INSTANCE__, __REG__, __VALUE__) WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in WWDT register
  * @param  __INSTANCE__ WWDT Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_WWDT_ReadReg(__INSTANCE__, __REG__) READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup WWDT_DDL_Exported_Functions WWDT Exported Functions
  * @{
  */

/** @defgroup WWDT_DDL_EF_Configuration Configuration
  * @{
  */
/**
  * @brief  Enable Window Watchdog. The watchdog is always disabled after a reset.
  * @note   It is enabled by setting the WDGA bit in the WWDT_CTRL register,
  *         then it cannot be disabled again except by a reset.
  *         This bit is set by software and only cleared by hardware after a reset.
  *         When WDGA = 1, the watchdog can generate a reset.
  * @param  WWDTx WWDT Instance
  * @retval None
  */
__STATIC_INLINE void DDL_WWDT_Enable(WWDT_TypeDef *WWDTx)
{
  SET_BIT(WWDTx->CTRL, WWDT_CTRL_WWDTEN);
}

/**
  * @brief  Checks if Window Watchdog is enabled
  * @param  WWDTx WWDT Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_WWDT_IsEnabled(WWDT_TypeDef *WWDTx)
{
  return ((READ_BIT(WWDTx->CTRL, WWDT_CTRL_WWDTEN) == (WWDT_CTRL_WWDTEN)) ? 1UL : 0UL);
}

/**
  * @brief  Set the Watchdog counter value to provided value (7-bits T[6:0])
  * @note   When writing to the WWDT_CTRL register, always write 1 in the MSB b6 to avoid generating an immediate reset
  *         This counter is decremented every (4096 x 2expWDGTB) PCLK cycles
  *         A reset is produced when it rolls over from 0x40 to 0x3F (bit T6 becomes cleared)
  *         Setting the counter lower then 0x40 causes an immediate reset (if WWDT enabled)
  * @param  WWDTx WWDT Instance
  * @param  Counter 0..0x7F (7 bit counter value)
  * @retval None
  */
__STATIC_INLINE void DDL_WWDT_SetCounter(WWDT_TypeDef *WWDTx, uint32_t Counter)
{
  MODIFY_REG(WWDTx->CTRL, WWDT_CTRL_CNT, Counter);
}

/**
  * @brief  Return current Watchdog Counter Value (7 bits counter value)
  * @param  WWDTx WWDT Instance
  * @retval 7 bit Watchdog Counter value
  */
__STATIC_INLINE uint32_t DDL_WWDT_GetCounter(WWDT_TypeDef *WWDTx)
{
  return (READ_BIT(WWDTx->CTRL, WWDT_CTRL_CNT));
}

/**
  * @brief  Set the time base of the prescaler (WDGTB).
  * @note   Prescaler is used to apply ratio on PCLK clock, so that Watchdog counter
  *         is decremented every (4096 x 2expWDGTB) PCLK cycles
  * @param  WWDTx WWDT Instance
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref DDL_WWDT_PRESCALER_1
  *         @arg @ref DDL_WWDT_PRESCALER_2
  *         @arg @ref DDL_WWDT_PRESCALER_4
  *         @arg @ref DDL_WWDT_PRESCALER_8
  * @retval None
  */
__STATIC_INLINE void DDL_WWDT_SetPrescaler(WWDT_TypeDef *WWDTx, uint32_t Prescaler)
{
  MODIFY_REG(WWDTx->CFR, WWDT_CFR_TBPSC, Prescaler);
}

/**
  * @brief  Return current Watchdog Prescaler Value
  * @param  WWDTx WWDT Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref DDL_WWDT_PRESCALER_1
  *         @arg @ref DDL_WWDT_PRESCALER_2
  *         @arg @ref DDL_WWDT_PRESCALER_4
  *         @arg @ref DDL_WWDT_PRESCALER_8
  */
__STATIC_INLINE uint32_t DDL_WWDT_GetPrescaler(WWDT_TypeDef *WWDTx)
{
  return (READ_BIT(WWDTx->CFR, WWDT_CFR_TBPSC));
}

/**
  * @brief  Set the Watchdog Window value to be compared to the downcounter (7-bits W[6:0]).
  * @note   This window value defines when write in the WWDT_CTRL register
  *         to program Watchdog counter is allowed.
  *         Watchdog counter value update must occur only when the counter value
  *         is lower than the Watchdog window register value.
  *         Otherwise, a MCU reset is generated if the 7-bit Watchdog counter value
  *         (in the control register) is refreshed before the downcounter has reached
  *         the watchdog window register value.
  *         Physically is possible to set the Window lower then 0x40 but it is not recommended.
  *         To generate an immediate reset, it is possible to set the Counter lower than 0x40.
  * @param  WWDTx WWDT Instance
  * @param  Window 0x00..0x7F (7 bit Window value)
  * @retval None
  */
__STATIC_INLINE void DDL_WWDT_SetWindow(WWDT_TypeDef *WWDTx, uint32_t Window)
{
  MODIFY_REG(WWDTx->CFR, WWDT_CFR_WIN, Window);
}

/**
  * @brief  Return current Watchdog Window Value (7 bits value)
  * @param  WWDTx WWDT Instance
  * @retval 7 bit Watchdog Window value
  */
__STATIC_INLINE uint32_t DDL_WWDT_GetWindow(WWDT_TypeDef *WWDTx)
{
  return (READ_BIT(WWDTx->CFR, WWDT_CFR_WIN));
}

/**
  * @}
  */

/** @defgroup WWDT_DDL_EF_FLAG_Management FLAG_Management
  * @{
  */
/**
  * @brief  Indicates if the WWDT Early Wakeup Interrupt Flag is set or not.
  * @note   This bit is set by hardware when the counter has reached the value 0x40.
  *         It must be cleared by software by writing 0.
  *         A write of 1 has no effect. This bit is also set if the interrupt is not enabled.
  * @param  WWDTx WWDT Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_WWDT_IsActiveFlag_EWKUP(WWDT_TypeDef *WWDTx)
{
  return ((READ_BIT(WWDTx->STS, WWDT_STS_EWIFLG) == (WWDT_STS_EWIFLG)) ? 1UL : 0UL);
}

/**
  * @brief  Clear WWDT Early Wakeup Interrupt Flag (EWIF)
  * @param  WWDTx WWDT Instance
  * @retval None
  */
__STATIC_INLINE void DDL_WWDT_ClearFlag_EWKUP(WWDT_TypeDef *WWDTx)
{
  WRITE_REG(WWDTx->STS, ~WWDT_STS_EWIFLG);
}

/**
  * @}
  */

/** @defgroup WWDT_DDL_EF_IT_Management IT_Management
  * @{
  */
/**
  * @brief  Enable the Early Wakeup Interrupt.
  * @note   When set, an interrupt occurs whenever the counter reaches value 0x40.
  *         This interrupt is only cleared by hardware after a reset
  * @param  WWDTx WWDT Instance
  * @retval None
  */
__STATIC_INLINE void DDL_WWDT_EnableIT_EWKUP(WWDT_TypeDef *WWDTx)
{
  SET_BIT(WWDTx->CFR, WWDT_CFR_EWIEN);
}

/**
  * @brief  Check if Early Wakeup Interrupt is enabled
  * @param  WWDTx WWDT Instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_WWDT_IsEnabledIT_EWKUP(WWDT_TypeDef *WWDTx)
{
  return ((READ_BIT(WWDTx->CFR, WWDT_CFR_EWIEN) == (WWDT_CFR_EWIEN)) ? 1UL : 0UL);
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

#endif /* WWDT */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_WWDT_H */
