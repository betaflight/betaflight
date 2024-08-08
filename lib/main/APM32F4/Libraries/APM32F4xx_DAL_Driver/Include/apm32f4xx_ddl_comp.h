/**
  *
  * @file    apm32f4xx_ddl_comp.h
  * @brief   Header file of COMP DDL module.
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
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023-2024 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APM32F4XX_DDL_COMP_H__
#define __APM32F4XX_DDL_COMP_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx.h"

/** @addtogroup APM32F4xx_DDL_Driver
  * @{
  */

 #if defined (COMP1) || defined (COMP2)

/** @addtogroup COMP_DDL COMP
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup COMP_DDL_ES_INIT COMP Exported Init Structure
  * @{
  */

/**
 * @brief COMP Init structure definition
 */
typedef struct
{
    uint32_t Mode;                  /*!< Set comparator operating mode to adjust speed.
                                        Note: Speed mode is available only for COMP2.
                                        This parameter can be a value of @ref COMP_DDL_EC_SPEEDMODE

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP_SetSpeedMode(). */

    uint32_t InputPlus;             /*!< Set comparator input plus.
                                        Note: Input plus is available only for COMP2.
                                        This parameter can be a value of @ref COMP_DDL_EC_INPUT_PLUS

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP_SetInputPlus(). */

    uint32_t InputMinus;            /*!< Set comparator input minus.
                                        This parameter can be a value of @ref COMP_DDL_EC_INPUT_MINUS

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP_SetInputMinus(). */

    uint32_t OutputPol;             /*!< Set comparator output polarity.
                                        This parameter can be a value of @ref COMP_DDL_EC_OUTPUT_POLARITY

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP_SetOutputPolarity(). */

    uint32_t Output;                /*!< Set comparator output.
                                        This parameter can be a value of @ref COMP_DDL_EC_OUTPUT

                                        This feature can be modified afterwards using unitary function @ref DDL_COMP_SetOutput(). */
} DDL_COMP_InitTypeDef;

/**
  * @}
  */
#endif /* USE_FULL_DDL_DRIVER */

/* Exported constants --------------------------------------------------------*/
/** @defgroup COMP_DDL_Exported_Constants COMP Exported Constants
  * @{
  */

/** @defgroup COMP_DDL_EC_WINDOWMODE COMP common window mod
 * @{
 */
#define DDL_COMP_WINDOWMODE_DISABLE             (0x00000000UL)              /*!< Window mode disable: Comparator 1 and comparator 2 are independent. */
#define DDL_COMP_WINDOWMODE_ENABLE              (COMP_CSTS_WMODESEL)        /*!< Window mode enable: Comparator 1 and comparator 2 are combined. */
/**
 * @}
 */

/** @defgroup COMP_DDL_EC_SPEEDMODE COMP speed mode
 * @{
 */
#define DDL_COMP_SPEEDMODE_LOW                  (0x00000000UL)              /*!< Low speed mode: Comparator 2 is in low speed mode. */
#define DDL_COMP_SPEEDMODE_HIGH                 (COMP_CSTS_SPEEDM)          /*!< High speed mode: Comparator 2 is in high speed mode. */
/**
 * @}
 */

/** @defgroup COMP_DDL_EC_INPUT_PLUS COMP input plus
 * @{
 */
#define DDL_COMP_INPUT_PLUS_PC2                 (0x00000000UL)              /*!< Input plus: PC2. Available only for COMP2. */
/**
 * @}
 */

/** @defgroup COMP_DDL_EC_INPUT_MINUS COMP input minus
 * @{
 */
#define DDL_COMP_INPUT_MINUS_VREFINT            (0x00000000UL)                                      /*!< Input minus: Vrefint. */
#define DDL_COMP_INPUT_MINUS_PC1                (COMP_CSTS_INMCCFG_0)                               /*!< Input minus: PC1. Available only for COMP1. */
#define DDL_COMP_INPUT_MINUS_PC3                (COMP_CSTS_INMCCFG_0)                               /*!< Input minus: PC3. Available only for COMP2. */
#define DDL_COMP_INPUT_MINUS_1_4_VREFINT        (COMP_CSTS_INMCCFG_2)                               /*!< Input minus: 1/4 Vrefint. Available only for COMP2. */
#define DDL_COMP_INPUT_MINUS_1_2_VREFINT        (COMP_CSTS_INMCCFG_2 | COMP_CSTS_INMCCFG_0)         /*!< Input minus: 1/2 Vrefint. Available only for COMP2. */
#define DDL_COMP_INPUT_MINUS_3_4_VREFINT        (COMP_CSTS_INMCCFG_2 | COMP_CSTS_INMCCFG_1)         /*!< Input minus: 3/4 Vrefint. Available only for COMP2. */
/**
 * @}
 */

/** @defgroup COMP_DDL_EC_OUTPUT_POLARITY COMP output polarity
 * @{
 */
#define DDL_COMP_OUTPUTPOL_NONINVERTED          (0x00000000UL)              /*!< Comparator output is non-inverted. */
#define DDL_COMP_OUTPUTPOL_INVERTED             (COMP_CSTS_POLCFG)          /*!< Comparator output is inverted. */
/**
 * @}
 */

/** @defgroup COMP_DDL_EC_OUTPUT COMP output
 * @{
 */
#define DDL_COMP_OUTPUT_NONE                    (0x00000000UL)                                                  /*!< Comparator output is not connected. */
#define DDL_COMP_OUTPUT_TMR1BKIN                (COMP_CSTS_OUTSEL_0)                                            /*!< Comparator output is connected to TMR1BKIN. */
#define DDL_COMP_OUTPUT_TMR1IC1                 (COMP_CSTS_OUTSEL_1)                                            /*!< Comparator output is connected to TMR1IC1. */
#define DDL_COMP_OUTPUT_TMR1ETRF                (COMP_CSTS_OUTSEL_1 | COMP_CSTS_OUTSEL_0)                       /*!< Comparator output is connected to TMR1ETRF. */
#define DDL_COMP_OUTPUT_TMR8BKIN                (COMP_CSTS_OUTSEL_2)                                            /*!< Comparator output is connected to TMR8BKIN. */
#define DDL_COMP_OUTPUT_TMR8IC1                 (COMP_CSTS_OUTSEL_2 | COMP_CSTS_OUTSEL_0)                       /*!< Comparator output is connected to TMR8IC1. */
#define DDL_COMP_OUTPUT_TMR8ETRF                (COMP_CSTS_OUTSEL_2 | COMP_CSTS_OUTSEL_1)                       /*!< Comparator output is connected to TMR8ETRF. */
#define DDL_COMP_OUTPUT_TMR2IC4                 (COMP_CSTS_OUTSEL_2 | COMP_CSTS_OUTSEL_1 | COMP_CSTS_OUTSEL_0)  /*!< Comparator output is connected to TMR2IC4. */
#define DDL_COMP_OUTPUT_TMR2ETRF                (COMP_CSTS_OUTSEL_3)                                            /*!< Comparator output is connected to TMR2ETRF. */
#define DDL_COMP_OUTPUT_TMR3IC1                 (COMP_CSTS_OUTSEL_3 | COMP_CSTS_OUTSEL_0)                       /*!< Comparator output is connected to TMR3IC1. */
#define DDL_COMP_OUTPUT_TMR3ETRF                (COMP_CSTS_OUTSEL_3 | COMP_CSTS_OUTSEL_1)                       /*!< Comparator output is connected to TMR3ETRF. */
#define DDL_COMP_OUTPUT_TMR4IC1                 (COMP_CSTS_OUTSEL_3 | COMP_CSTS_OUTSEL_1 | COMP_CSTS_OUTSEL_0)  /*!< Comparator output is connected to TMR4IC1. */
/**
 * @}
 */

/** @defgroup COMP_DDL_EC_OUTPUT_LEVEL COMP output level
 * @{
 */
#define DDL_COMP_OUTPUT_LEVEL_LOW               (0x00000000UL)              /*!< Comparator output is low level. */
#define DDL_COMP_OUTPUT_LEVEL_HIGH              (0x00000001UL)              /*!< Comparator output is high level. */
/**
 * @}
 */

/** @defgroup COMP_DDL_EC_HW_DELAYS COMP hardware delay
 * @{
 */

/* Delay for comparator startup time */
#define DDL_COMP_DELAY_STARTUP_US               (80UL)                      /*!< Comparator startup time. */
/**
 * @}
 */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup COMP_DDL_Exported_Macros COMP Exported Macros
  * @{
  */
/** @defgroup COMP_DDL_EM_WRITE_READ Common write and read registers macro
  * @{
  */

/**
 * @brief  Write a value in COMP register.
 * @param  __INSTANCE__ COMP instance.
 * @param  __REG__ COMP register.
 * @param  __VALUE__ Value to be written.
 * @retval None
 */
#define DDL_COMP_WriteReg(__INSTANCE__, __REG__, __VALUE__)     WRITE_REG(__INSTANCE__->__REG__, (__VALUE__))

/**
 * @brief  Read a value in COMP register.
 * @param  __INSTANCE__ COMP instance.
 * @param  __REG__ COMP register.
 * @retval Register value
 */
#define DDL_COMP_ReadReg(__INSTANCE__, __REG__)                 READ_REG(__INSTANCE__->__REG__)
/**
  * @}
  */

/** @defgroup COMP_DDL_EM_HELPER_MACRO Helper macro
  * @{
  */

/**
 * @brief  Helper macro to select the COMP common instance
 *         to which is belonging the selected COMP instance.
 * @param  __COMPx__ COMP instance.
 * @retval COMP common instance or value "0" if there is no COMP common instance.
 * @note   COMP common register instance can be used to set parameters common to
 *         several COMP instances. Refer to functions having argument "COMPxy_COMMON" as parameter.
 */
#define __DDL_COMP_COMMON_INSTANCE(__COMPx__)               (COMP12_COMMON)

 /**
  * @}
  */

 /**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup COMP_DDL_Exported_Functions COMP Exported Functions
  * @{
  */

/** @defgroup COMP_DDL_EF_Configuration_comparator_common Configuration of comparator hierarchical scope: common to several COMP instances
  * @{
  */

/**
 * @brief  Set comparator window mode.
 * @param  COMPxy_COMMON COMP common instance.
 * @param  WindowMode This parameter can be one of the following values:
 *        @arg @ref DDL_COMP_WINDOWMODE_DISABLE
 *        @arg @ref DDL_COMP_WINDOWMODE_ENABLE
 * @retval None
 */
__STATIC_INLINE void DDL_COMP_SetWindowMode(COMP_Common_TypeDef *COMPxy_COMMON,
                                            uint32_t WindowMode)
{
    /* Note: Window mode is available only for COMP1. */
    MODIFY_REG(COMPxy_COMMON->CSTS, COMP_CSTS_WMODESEL, WindowMode);
}

/**
 * @brief Get comparator window mode.
 * @param  COMPxy_COMMON COMP common instance.
 * @retval Returned value can be one of the following values:
 *        @arg @ref DDL_COMP_WINDOWMODE_DISABLE
 *        @arg @ref DDL_COMP_WINDOWMODE_ENABLE
 */
__STATIC_INLINE uint32_t DDL_COMP_GetWindowMode(COMP_Common_TypeDef *COMPxy_COMMON)
{
    /* Note: Window mode is available only for COMP1. */
    return (uint32_t)(READ_BIT(COMPxy_COMMON->CSTS, COMP_CSTS_WMODESEL));
}

/**
  * @}
  */

/** @defgroup COMP_DDL_EF_Configuration_comparator_modes Configuration of comparator instance mode
 * @{
 */

/**
 * @brief  Set comparator operating mode to adjust speed.
 * @param  COMPx COMP instance.
 * @param  Mode This parameter can be one of the following values:
 *         @arg @ref DDL_COMP_SPEEDMODE_LOW
 *         @arg @ref DDL_COMP_SPEEDMODE_HIGH
 * @retval None
 */
__STATIC_INLINE void DDL_COMP_SetSpeedMode(COMP_TypeDef *COMPx,
                                            uint32_t Mode)
{
    /* Note: Speed mode is available only for COMP2. */
    MODIFY_REG(COMPx->CSTS, COMP_CSTS_SPEEDM, Mode);
}

/**
 * @brief Get comparator operating mode to adjust speed.
 * @param  COMPx COMP instance.
 * @retval Returned value can be one of the following values:
 *        @arg @ref DDL_COMP_SPEEDMODE_LOW
 *        @arg @ref DDL_COMP_SPEEDMODE_HIGH
 */
__STATIC_INLINE uint32_t DDL_COMP_GetSpeedMode(COMP_TypeDef *COMPx)
{
    /* Note: Speed mode is available only for COMP2. */
    return (uint32_t)(READ_BIT(COMPx->CSTS, COMP_CSTS_SPEEDM));
}

/**
  * @}
  */

/** @defgroup COMP_DDL_EF_Configuration_comparator_inputs Configuration of comparator inputs
 * @{
 */

/**
 * @brief  Set comparator input plus.
 * @param  COMPx COMP instance.
 * @param  InputPlus This parameter can be one of the following values:
 *        @arg @ref DDL_COMP_INPUT_PLUS_PC2
 * @retval None
 */
__STATIC_INLINE void DDL_COMP_SetInputPlus(COMP_TypeDef *COMPx,
                                            uint32_t InputPlus)
{
    /* Note: Input plus is available only for COMP2. */
    MODIFY_REG(COMPx->CSTS, COMP_CSTS_INPCCFG, InputPlus);
}

/**
 * @brief Get comparator input plus.
 * @param  COMPx COMP instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP_INPUT_PLUS_PC2
 */
__STATIC_INLINE uint32_t DDL_COMP_GetInputPlus(COMP_TypeDef *COMPx)
{
    /* Note: Input plus is available only for COMP2. */
    return (uint32_t)(READ_BIT(COMPx->CSTS, COMP_CSTS_INPCCFG));
}

/**
 * @brief  Set comparator input minus.
 * @param  COMPx COMP instance.
 * @param  InputMinus This parameter can be one of the following values:
 *        @arg @ref DDL_COMP_INPUT_MINUS_VREFINT
 *        @arg @ref DDL_COMP_INPUT_MINUS_PC1 (COMP1 only)
 *        @arg @ref DDL_COMP_INPUT_MINUS_PC3 (COMP2 only)
 *        @arg @ref DDL_COMP_INPUT_MINUS_1_4_VREFINT (COMP2 only)
 *        @arg @ref DDL_COMP_INPUT_MINUS_1_2_VREFINT (COMP2 only)
 *        @arg @ref DDL_COMP_INPUT_MINUS_3_4_VREFINT (COMP2 only)
 * @retval None
 */
__STATIC_INLINE void DDL_COMP_SetInputMinus(COMP_TypeDef *COMPx,
                                            uint32_t InputMinus)
{
    /* Note: Input minus is available only for COMP1 and COMP2. */
    MODIFY_REG(COMPx->CSTS, COMP_CSTS_INMCCFG, InputMinus);
}

/**
 * @brief Get comparator input minus.
 * @param  COMPx COMP instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP_INPUT_MINUS_VREFINT
 *       @arg @ref DDL_COMP_INPUT_MINUS_PC1 (COMP1 only)
 *       @arg @ref DDL_COMP_INPUT_MINUS_PC3 (COMP2 only)
 *       @arg @ref DDL_COMP_INPUT_MINUS_1_4_VREFINT (COMP2 only)
 *       @arg @ref DDL_COMP_INPUT_MINUS_1_2_VREFINT (COMP2 only)
 *       @arg @ref DDL_COMP_INPUT_MINUS_3_4_VREFINT (COMP2 only)
 */
__STATIC_INLINE uint32_t DDL_COMP_GetInputMinus(COMP_TypeDef *COMPx)
{
    return (uint32_t)(READ_BIT(COMPx->CSTS, COMP_CSTS_INMCCFG));
}

/**
  * @}
  */

/** @defgroup COMP_DDL_EF_Configuration_comparator_output Configuration of comparator output
 * @{
 */

/**
 * @brief  Set comparator output polarity.
 * @param  COMPx COMP instance.
 * @param  OutputPol This parameter can be one of the following values:
 *        @arg @ref DDL_COMP_OUTPUTPOL_NONINVERTED
 *        @arg @ref DDL_COMP_OUTPUTPOL_INVERTED
 * @retval None
 */
__STATIC_INLINE void DDL_COMP_SetOutputPolarity(COMP_TypeDef *COMPx,
                                                uint32_t OutputPol)
{
    MODIFY_REG(COMPx->CSTS, COMP_CSTS_POLCFG, OutputPol);
}

/**
 * @brief Get comparator output polarity.
 * @param  COMPx COMP instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP_OUTPUTPOL_NONINVERTED
 *       @arg @ref DDL_COMP_OUTPUTPOL_INVERTED
 */
__STATIC_INLINE uint32_t DDL_COMP_GetOutputPolarity(COMP_TypeDef *COMPx)
{
    return (uint32_t)(READ_BIT(COMPx->CSTS, COMP_CSTS_POLCFG));
}

/**
 * @brief  Set comparator output.
 * @param  COMPx COMP instance.
 * @param  Output This parameter can be one of the following values:
 *       @arg @ref DDL_COMP_OUTPUT_NONE
 *       @arg @ref DDL_COMP_OUTPUT_TMR1BKIN
 *       @arg @ref DDL_COMP_OUTPUT_TMR1IC1
 *       @arg @ref DDL_COMP_OUTPUT_TMR1ETRF
 *       @arg @ref DDL_COMP_OUTPUT_TMR8BKIN
 *       @arg @ref DDL_COMP_OUTPUT_TMR8IC1
 *       @arg @ref DDL_COMP_OUTPUT_TMR8ETRF
 *       @arg @ref DDL_COMP_OUTPUT_TMR2IC4
 *       @arg @ref DDL_COMP_OUTPUT_TMR2ETRF
 *       @arg @ref DDL_COMP_OUTPUT_TMR3IC1
 *       @arg @ref DDL_COMP_OUTPUT_TMR3ETRF
 *       @arg @ref DDL_COMP_OUTPUT_TMR4IC1
 * @retval None
 */
__STATIC_INLINE void DDL_COMP_SetOutput(COMP_TypeDef *COMPx,
                                        uint32_t Output)
{
    MODIFY_REG(COMPx->CSTS, COMP_CSTS_OUTSEL, Output);
}

/**
 * @brief Get comparator output.
 * @param  COMPx COMP instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP_OUTPUT_NONE
 *       @arg @ref DDL_COMP_OUTPUT_TMR1BKIN
 *       @arg @ref DDL_COMP_OUTPUT_TMR1IC1
 *       @arg @ref DDL_COMP_OUTPUT_TMR1ETRF
 *       @arg @ref DDL_COMP_OUTPUT_TMR8BKIN
 *       @arg @ref DDL_COMP_OUTPUT_TMR8IC1
 *       @arg @ref DDL_COMP_OUTPUT_TMR8ETRF
 *       @arg @ref DDL_COMP_OUTPUT_TMR2IC4
 *       @arg @ref DDL_COMP_OUTPUT_TMR2ETRF
 *       @arg @ref DDL_COMP_OUTPUT_TMR3IC1
 *       @arg @ref DDL_COMP_OUTPUT_TMR3ETRF
 *       @arg @ref DDL_COMP_OUTPUT_TMR4IC1
 */
__STATIC_INLINE uint32_t DDL_COMP_GetOutput(COMP_TypeDef *COMPx)
{
    return (uint32_t)(READ_BIT(COMPx->CSTS, COMP_CSTS_OUTSEL));
}

/**
  * @}
  */

 /** @defgroup COMP_DDL_EF_Operation Operation on comparator instance
  * @{
  */

/**
 * @brief  Enable comparator instance.
 * @param  COMPx COMP instance.
 * @retval None
 * @note   After enable from off state, comparator startup time is needed.
 */
__STATIC_INLINE void DDL_COMP_Enable(COMP_TypeDef *COMPx)
{
    SET_BIT(COMPx->CSTS, COMP_CSTS_EN);
}

/**
 * @brief  Disable comparator instance.
 * @param  COMPx COMP instance.
 * @retval None
 */
__STATIC_INLINE void DDL_COMP_Disable(COMP_TypeDef *COMPx)
{
    CLEAR_BIT(COMPx->CSTS, COMP_CSTS_EN);
}

/**
 * @brief  Get comparator enable state.
 * @param  COMPx COMP instance.
 * @retval State of bit (1 or 0).
 *        - 0: comparator instance is disabled.
 *        - 1: comparator instance is enabled.
 */
__STATIC_INLINE uint32_t DDL_COMP_IsEnabled(COMP_TypeDef *COMPx)
{
    return ((READ_BIT(COMPx->CSTS, COMP_CSTS_EN) == (COMP_CSTS_EN)) ? 1UL : 0UL);
}

/**
 * @brief  Lock comparator instance.
 * @param  COMPx COMP instance.
 * @retval None
 * @note   Once locked, comparator configuration can no longer be modified until next reset.
 * @note   The only way to unlock the comparator is a device hardware reset.
 */
__STATIC_INLINE void DDL_COMP_Lock(COMP_TypeDef *COMPx)
{
    SET_BIT(COMPx->CSTS, COMP_CSTS_LOCK);
}

/**
 * @brief  Get comparator lock state.
 * @param  COMPx COMP instance.
 * @retval State of bit (1 or 0).
 *       - 0: comparator instance is unlocked.
 *      - 1: comparator instance is locked.
 */
__STATIC_INLINE uint32_t DDL_COMP_IsLocked(COMP_TypeDef *COMPx)
{
    return ((READ_BIT(COMPx->CSTS, COMP_CSTS_LOCK) == (COMP_CSTS_LOCK)) ? 1UL : 0UL);
}

/**
 * @brief  Read comparator output level.
 * @param  COMPx COMP instance.
 * @retval Returned value can be one of the following values:
 *       @arg @ref DDL_COMP_OUTPUT_LEVEL_LOW
 *       @arg @ref DDL_COMP_OUTPUT_LEVEL_HIGH
 * @note   The comparator output level depends on the selected polarity.
 *         If the polarity is not inverted:
 *         - Comparator output is low level when the input plus is lower than the input minus.
 *         - Comparator output is high level when the input plus is higher than the input minus.
 *         If the polarity is inverted:
 *         - Comparator output is high level when the input plus is lower than the input minus.
 *         - Comparator output is low level when the input plus is higher than the input minus.
 */
__STATIC_INLINE uint32_t DDL_COMP_ReadOutputLevel(COMP_TypeDef *COMPx)
{
    return (uint32_t)(READ_BIT(COMPx->CSTS, COMP_CSTS_OUTVAL));
}

/**
  * @}
  */

#if defined(USE_FULL_DDL_DRIVER)
/** @defgroup COMP_DDL_EF_Init Initialization and de-initialization functions
 * @{
 */

ErrorStatus DDL_COMP_Init(COMP_TypeDef *COMPx, DDL_COMP_InitTypeDef *COMP_InitStruct);
ErrorStatus DDL_COMP_DeInit(COMP_TypeDef *COMPx);
void        DDL_COMP_StructInit(DDL_COMP_InitTypeDef *COMP_InitStruct);

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

 #endif /* COMP1 || COMP2 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __APM32F4XX_DDL_COMP_H__ */

