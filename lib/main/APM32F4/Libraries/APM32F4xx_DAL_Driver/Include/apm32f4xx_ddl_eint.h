/**
  *
  * @file    apm32f4xx_ddl_eint.h
  * @brief   Header file of EINT DDL module.
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
  * If no LICENSE file comes with this software, it is provided AS-IS.Clause
  *
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APM32F4xx_DDL_EINT_H
#define APM32F4xx_DDL_EINT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

#if defined (EINT)

/** @defgroup EINT_DDL EINT
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private Macros ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup EINT_DDL_Private_Macros EINT Private Macros
  * @{
  */
/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup EINT_DDL_ES_INIT EINT Exported Init structure
  * @{
  */
typedef struct
{

  uint32_t Line_0_31;           /*!< Specifies the EINT lines to be enabled or disabled for Lines in range 0 to 31
                                     This parameter can be any combination of @ref EINT_DDL_EC_LINE */

  FunctionalState LineCommand;  /*!< Specifies the new state of the selected EINT lines.
                                     This parameter can be set either to ENABLE or DISABLE */

  uint8_t Mode;                 /*!< Specifies the mode for the EINT lines.
                                     This parameter can be a value of @ref EINT_DDL_EC_MODE. */

  uint8_t Trigger;              /*!< Specifies the trigger signal active edge for the EINT lines.
                                     This parameter can be a value of @ref EINT_DDL_EC_TRIGGER. */
} DDL_EINT_InitTypeDef;

/**
  * @}
  */
#endif /*USE_FULL_DDL_DRIVER*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup EINT_DDL_Exported_Constants EINT Exported Constants
  * @{
  */

/** @defgroup EINT_DDL_EC_LINE LINE
  * @{
  */
#define DDL_EINT_LINE_0                 EINT_IMASK_IM0           /*!< Extended line 0 */
#define DDL_EINT_LINE_1                 EINT_IMASK_IM1           /*!< Extended line 1 */
#define DDL_EINT_LINE_2                 EINT_IMASK_IM2           /*!< Extended line 2 */
#define DDL_EINT_LINE_3                 EINT_IMASK_IM3           /*!< Extended line 3 */
#define DDL_EINT_LINE_4                 EINT_IMASK_IM4           /*!< Extended line 4 */
#define DDL_EINT_LINE_5                 EINT_IMASK_IM5           /*!< Extended line 5 */
#define DDL_EINT_LINE_6                 EINT_IMASK_IM6           /*!< Extended line 6 */
#define DDL_EINT_LINE_7                 EINT_IMASK_IM7           /*!< Extended line 7 */
#define DDL_EINT_LINE_8                 EINT_IMASK_IM8           /*!< Extended line 8 */
#define DDL_EINT_LINE_9                 EINT_IMASK_IM9           /*!< Extended line 9 */
#define DDL_EINT_LINE_10                EINT_IMASK_IM10          /*!< Extended line 10 */
#define DDL_EINT_LINE_11                EINT_IMASK_IM11          /*!< Extended line 11 */
#define DDL_EINT_LINE_12                EINT_IMASK_IM12          /*!< Extended line 12 */
#define DDL_EINT_LINE_13                EINT_IMASK_IM13          /*!< Extended line 13 */
#define DDL_EINT_LINE_14                EINT_IMASK_IM14          /*!< Extended line 14 */
#define DDL_EINT_LINE_15                EINT_IMASK_IM15          /*!< Extended line 15 */
#if defined(EINT_IMASK_IM16)
#define DDL_EINT_LINE_16                EINT_IMASK_IM16          /*!< Extended line 16 */
#endif
#define DDL_EINT_LINE_17                EINT_IMASK_IM17          /*!< Extended line 17 */
#if defined(EINT_IMASK_IM18)
#define DDL_EINT_LINE_18                EINT_IMASK_IM18          /*!< Extended line 18 */
#endif
#define DDL_EINT_LINE_19                EINT_IMASK_IM19          /*!< Extended line 19 */
#if defined(EINT_IMASK_IM20)
#define DDL_EINT_LINE_20                EINT_IMASK_IM20          /*!< Extended line 20 */
#endif
#if defined(EINT_IMASK_IM21)
#define DDL_EINT_LINE_21                EINT_IMASK_IM21          /*!< Extended line 21 */
#endif
#if defined(EINT_IMASK_IM22)
#define DDL_EINT_LINE_22                EINT_IMASK_IM22          /*!< Extended line 22 */
#endif
#if defined(EINT_IMASK_IM23)
#define DDL_EINT_LINE_23                EINT_IMASK_IM23          /*!< Extended line 23 */
#endif
#if defined(EINT_IMASK_IM24)
#define DDL_EINT_LINE_24                EINT_IMASK_IM24          /*!< Extended line 24 */
#endif
#if defined(EINT_IMASK_IM25)
#define DDL_EINT_LINE_25                EINT_IMASK_IM25          /*!< Extended line 25 */
#endif
#if defined(EINT_IMASK_IM26)
#define DDL_EINT_LINE_26                EINT_IMASK_IM26          /*!< Extended line 26 */
#endif
#if defined(EINT_IMASK_IM27)
#define DDL_EINT_LINE_27                EINT_IMASK_IM27          /*!< Extended line 27 */
#endif
#if defined(EINT_IMASK_IM28)
#define DDL_EINT_LINE_28                EINT_IMASK_IM28          /*!< Extended line 28 */
#endif
#if defined(EINT_IMASK_IM29)
#define DDL_EINT_LINE_29                EINT_IMASK_IM29          /*!< Extended line 29 */
#endif
#if defined(EINT_IMASK_IM30)
#define DDL_EINT_LINE_30                EINT_IMASK_IM30          /*!< Extended line 30 */
#endif
#if defined(EINT_IMASK_IM31)
#define DDL_EINT_LINE_31                EINT_IMASK_IM31          /*!< Extended line 31 */
#endif
#define DDL_EINT_LINE_ALL_0_31          EINT_IMASK_IM            /*!< All Extended line not reserved*/


#define DDL_EINT_LINE_ALL               ((uint32_t)0xFFFFFFFFU)  /*!< All Extended line */

#if defined(USE_FULL_DDL_DRIVER)
#define DDL_EINT_LINE_NONE              ((uint32_t)0x00000000U)  /*!< None Extended line */
#endif /*USE_FULL_DDL_DRIVER*/

/**
  * @}
  */
#if defined(USE_FULL_DDL_DRIVER)

/** @defgroup EINT_DDL_EC_MODE Mode
  * @{
  */
#define DDL_EINT_MODE_IT                 ((uint8_t)0x00U) /*!< Interrupt Mode */
#define DDL_EINT_MODE_EVENT              ((uint8_t)0x01U) /*!< Event Mode */
#define DDL_EINT_MODE_IT_EVENT           ((uint8_t)0x02U) /*!< Interrupt & Event Mode */
/**
  * @}
  */

/** @defgroup EINT_DDL_EC_TRIGGER Edge Trigger
  * @{
  */
#define DDL_EINT_TRIGGER_NONE            ((uint8_t)0x00U) /*!< No Trigger Mode */
#define DDL_EINT_TRIGGER_RISING          ((uint8_t)0x01U) /*!< Trigger Rising Mode */
#define DDL_EINT_TRIGGER_FALLING         ((uint8_t)0x02U) /*!< Trigger Falling Mode */
#define DDL_EINT_TRIGGER_RISING_FALLING  ((uint8_t)0x03U) /*!< Trigger Rising & Falling Mode */

/**
  * @}
  */


#endif /*USE_FULL_DDL_DRIVER*/


/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup EINT_DDL_Exported_Macros EINT Exported Macros
  * @{
  */

/** @defgroup EINT_DDL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in EINT register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define DDL_EINT_WriteReg(__REG__, __VALUE__) WRITE_REG(EINT->__REG__, (__VALUE__))

/**
  * @brief  Read a value in EINT register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define DDL_EINT_ReadReg(__REG__) READ_REG(EINT->__REG__)
/**
  * @}
  */


/**
  * @}
  */



/* Exported functions --------------------------------------------------------*/
/** @defgroup EINT_DDL_Exported_Functions EINT Exported Functions
 * @{
 */
/** @defgroup EINT_DDL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable EintLine Interrupt request for Lines in range 0 to 31
  * @note The reset value for the direct or internal lines (see RM)
  *       is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @param  EintLine This parameter can be one of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_17
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  *         @arg @ref DDL_EINT_LINE_23(*)
  *         @arg @ref DDL_EINT_LINE_ALL_0_31
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_EnableIT_0_31(uint32_t EintLine)
{
  SET_BIT(EINT->IMASK, EintLine);
}

/**
  * @brief  Disable EintLine Interrupt request for Lines in range 0 to 31
  * @note The reset value for the direct or internal lines (see RM)
  *       is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @param  EintLine This parameter can be one of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_17
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  *         @arg @ref DDL_EINT_LINE_23(*)
  *         @arg @ref DDL_EINT_LINE_ALL_0_31
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_DisableIT_0_31(uint32_t EintLine)
{
  CLEAR_BIT(EINT->IMASK, EintLine);
}


/**
  * @brief  Indicate if EintLine Interrupt request is enabled for Lines in range 0 to 31
  * @note The reset value for the direct or internal lines (see RM)
  *       is set to 1 in order to enable the interrupt by default.
  *       Bits are set automatically at Power on.
  * @param  EintLine This parameter can be one of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_17
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  *         @arg @ref DDL_EINT_LINE_23(*)
  *         @arg @ref DDL_EINT_LINE_ALL_0_31
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_EINT_IsEnabledIT_0_31(uint32_t EintLine)
{
  return (READ_BIT(EINT->IMASK, EintLine) == (EintLine));
}


/**
  * @}
  */

/** @defgroup EINT_DDL_EF_Event_Management Event_Management
  * @{
  */

/**
  * @brief  Enable EintLine Event request for Lines in range 0 to 31
  * @param  EintLine This parameter can be one of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_17
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  *         @arg @ref DDL_EINT_LINE_23(*)
  *         @arg @ref DDL_EINT_LINE_ALL_0_31
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_EnableEvent_0_31(uint32_t EintLine)
{
  SET_BIT(EINT->EMASK, EintLine);

}


/**
  * @brief  Disable EintLine Event request for Lines in range 0 to 31
  * @param  EintLine This parameter can be one of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_17
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  *         @arg @ref DDL_EINT_LINE_23(*)
  *         @arg @ref DDL_EINT_LINE_ALL_0_31
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_DisableEvent_0_31(uint32_t EintLine)
{
  CLEAR_BIT(EINT->EMASK, EintLine);
}


/**
  * @brief  Indicate if EintLine Event request is enabled for Lines in range 0 to 31
  * @param  EintLine This parameter can be one of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_17
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  *         @arg @ref DDL_EINT_LINE_23(*)
  *         @arg @ref DDL_EINT_LINE_ALL_0_31
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_EINT_IsEnabledEvent_0_31(uint32_t EintLine)
{
  return (READ_BIT(EINT->EMASK, EintLine) == (EintLine));

}


/**
  * @}
  */

/** @defgroup EINT_DDL_EF_Rising_Trigger_Management Rising_Trigger_Management
  * @{
  */

/**
  * @brief  Enable EintLine Rising Edge Trigger for Lines in range 0 to 31
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a rising edge on a configurable interrupt
  *       line occurs during a write operation in the EINT_RTEN register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_EnableRisingTrig_0_31(uint32_t EintLine)
{
  SET_BIT(EINT->RTEN, EintLine);

}


/**
  * @brief  Disable EintLine Rising Edge Trigger for Lines in range 0 to 31
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a rising edge on a configurable interrupt
  *       line occurs during a write operation in the EINT_RTEN register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_DisableRisingTrig_0_31(uint32_t EintLine)
{
  CLEAR_BIT(EINT->RTEN, EintLine);

}


/**
  * @brief  Check if rising edge trigger is enabled for Lines in range 0 to 31
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_EINT_IsEnabledRisingTrig_0_31(uint32_t EintLine)
{
  return (READ_BIT(EINT->RTEN, EintLine) == (EintLine));
}


/**
  * @}
  */

/** @defgroup EINT_DDL_EF_Falling_Trigger_Management Falling_Trigger_Management
  * @{
  */

/**
  * @brief  Enable EintLine Falling Edge Trigger for Lines in range 0 to 31
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a falling edge on a configurable interrupt
  *       line occurs during a write operation in the EINT_FTEN register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for
  *       the same interrupt line. In this case, both generate a trigger
  *       condition.
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_EnableFallingTrig_0_31(uint32_t EintLine)
{
  SET_BIT(EINT->FTEN, EintLine);
}


/**
  * @brief  Disable EintLine Falling Edge Trigger for Lines in range 0 to 31
  * @note The configurable wakeup lines are edge-triggered. No glitch must be
  *       generated on these lines. If a Falling edge on a configurable interrupt
  *       line occurs during a write operation in the EINT_FTEN register, the
  *       pending bit is not set.
  *       Rising and falling edge triggers can be set for the same interrupt line.
  *       In this case, both generate a trigger condition.
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_DisableFallingTrig_0_31(uint32_t EintLine)
{
  CLEAR_BIT(EINT->FTEN, EintLine);
}


/**
  * @brief  Check if falling edge trigger is enabled for Lines in range 0 to 31
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_EINT_IsEnabledFallingTrig_0_31(uint32_t EintLine)
{
  return (READ_BIT(EINT->FTEN, EintLine) == (EintLine));
}


/**
  * @}
  */

/** @defgroup EINT_DDL_EF_Software_Interrupt_Management Software_Interrupt_Management
  * @{
  */

/**
  * @brief  Generate a software Interrupt Event for Lines in range 0 to 31
  * @note If the interrupt is enabled on this line in the EINT_IMASK, writing a 1 to
  *       this bit when it is at '0' sets the corresponding pending bit in EINT_IPEND
  *       resulting in an interrupt request generation.
  *       This bit is cleared by clearing the corresponding bit in the EINT_IPEND
  *       register (by writing a 1 into the bit)
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_GenerateSWI_0_31(uint32_t EintLine)
{
  SET_BIT(EINT->SWINTE, EintLine);
}


/**
  * @}
  */

/** @defgroup EINT_DDL_EF_Flag_Management Flag_Management
  * @{
  */

/**
  * @brief  Check if the ExtLine Flag is set or not for Lines in range 0 to 31
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t DDL_EINT_IsActiveFlag_0_31(uint32_t EintLine)
{
  return (READ_BIT(EINT->IPEND, EintLine) == (EintLine));
}


/**
  * @brief  Read ExtLine Combination Flag for Lines in range 0 to 31
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval @note This bit is set when the selected edge event arrives on the interrupt
  */
__STATIC_INLINE uint32_t DDL_EINT_ReadFlag_0_31(uint32_t EintLine)
{
  return (uint32_t)(READ_BIT(EINT->IPEND, EintLine));
}


/**
  * @brief  Clear ExtLine Flags  for Lines in range 0 to 31
  * @note This bit is set when the selected edge event arrives on the interrupt
  *       line. This bit is cleared by writing a 1 to the bit.
  * @param  EintLine This parameter can be a combination of the following values:
  *         @arg @ref DDL_EINT_LINE_0
  *         @arg @ref DDL_EINT_LINE_1
  *         @arg @ref DDL_EINT_LINE_2
  *         @arg @ref DDL_EINT_LINE_3
  *         @arg @ref DDL_EINT_LINE_4
  *         @arg @ref DDL_EINT_LINE_5
  *         @arg @ref DDL_EINT_LINE_6
  *         @arg @ref DDL_EINT_LINE_7
  *         @arg @ref DDL_EINT_LINE_8
  *         @arg @ref DDL_EINT_LINE_9
  *         @arg @ref DDL_EINT_LINE_10
  *         @arg @ref DDL_EINT_LINE_11
  *         @arg @ref DDL_EINT_LINE_12
  *         @arg @ref DDL_EINT_LINE_13
  *         @arg @ref DDL_EINT_LINE_14
  *         @arg @ref DDL_EINT_LINE_15
  *         @arg @ref DDL_EINT_LINE_16
  *         @arg @ref DDL_EINT_LINE_18
  *         @arg @ref DDL_EINT_LINE_19(*)
  *         @arg @ref DDL_EINT_LINE_20(*)
  *         @arg @ref DDL_EINT_LINE_21
  *         @arg @ref DDL_EINT_LINE_22
  * @note   (*): Available in some devices
  * @note   Please check each device line mapping for EINT Line availability
  * @retval None
  */
__STATIC_INLINE void DDL_EINT_ClearFlag_0_31(uint32_t EintLine)
{
  WRITE_REG(EINT->IPEND, EintLine);
}


/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup EINT_DDL_EF_Init Initialization and de-initialization functions
  * @{
  */

uint32_t DDL_EINT_Init(DDL_EINT_InitTypeDef *EINT_InitStruct);
uint32_t DDL_EINT_DeInit(void);
void DDL_EINT_StructInit(DDL_EINT_InitTypeDef *EINT_InitStruct);


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

#endif /* EINT */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* APM32F4xx_DDL_EINT_H */

