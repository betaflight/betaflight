/**
  *
  * @file    apm32f4xx_ddl_comp.c
  * @brief   COMP DDL module driver.
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
  * Copyright (C) 2023-2024 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_comp.h"
#include "apm32f4xx_ddl_bus.h"

#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif /* USE_FULL_ASSERT */

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

/** @defgroup COMP_DDL_Private_Macros COMP Private Macros
  * @{
  */

 /* Check of parameters for configuration of COMP hierarchical scope:          */
    /* COMP instance.                                                             */

#define IS_DDL_COMP_MODE(__MODE__)                                      \
    (   ((__MODE__) == DDL_COMP_SPEEDMODE_LOW) ||                       \
        ((__MODE__) == DDL_COMP_SPEEDMODE_HIGH)                         \
    )

#define IS_DDL_COMP_INPUT_PLUS(__COMP_INSTANCE__, __INPUT_PLUS__)       \
    ((__INPUT_PLUS__) == DDL_COMP_INPUT_PLUS_PC2)

#define IS_DDL_COMP1_INPUT_MINUS(__COMP_INSTANCE__, __INPUT_MINUS__)    \
    (   ((__INPUT_MINUS__) == DDL_COMP_INPUT_MINUS_VREFINT) ||          \
        ((__INPUT_MINUS__) == DDL_COMP_INPUT_MINUS_PC1)                 \
    )

#define IS_DDL_COMP2_INPUT_MINUS(__COMP_INSTANCE__, __INPUT_MINUS__)    \
    (   ((__INPUT_MINUS__) == DDL_COMP_INPUT_MINUS_VREFINT) ||          \
        ((__INPUT_MINUS__) == DDL_COMP_INPUT_MINUS_PC3) ||              \
        ((__INPUT_MINUS__) == DDL_COMP_INPUT_MINUS_1_4_VREFINT) ||      \
        ((__INPUT_MINUS__) == DDL_COMP_INPUT_MINUS_1_2_VREFINT) ||      \
        ((__INPUT_MINUS__) == DDL_COMP_INPUT_MINUS_3_4_VREFINT)         \
    )

#define IS_DDL_COMP_OUTPUT_POLARITY(__POLARITY__)                       \
    (   ((__POLARITY__) == DDL_COMP_OUTPUTPOL_NONINVERTED) ||           \
        ((__POLARITY__) == DDL_COMP_OUTPUTPOL_INVERTED)                 \
    )

#define IS_DDL_COMP_OUTPUT_MODE(__MODE__)                               \
    (   ((__MODE__) == DDL_COMP_OUTPUT_NONE) ||                         \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR1BKIN) ||                     \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR1IC1) ||                      \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR1ETRF) ||                     \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR8BKIN) ||                     \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR8IC1) ||                      \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR8ETRF) ||                     \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR2IC4) ||                      \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR2ETRF) ||                     \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR3IC1) ||                      \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR3ETRF) ||                     \
        ((__MODE__) == DDL_COMP_OUTPUT_TMR4IC1)                         \
    )

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup COMP_DDL_Exported_Functions
  * @{
  */

/** @addtogroup COMP_DDL_EF_Init
  * @{
  */

/**
 * @brief  Initialize COMP function.
 * @param  COMPx COMP instance
 * @param  COMP_InitStruct Pointer to a @ref DDL_COMP_InitTypeDef structure
 * @retval An ErrorStatus enumeration value:
 *          - SUCCESS: COMP registers are initialized
 *          - ERROR: COMP registers are not initialized
 */
ErrorStatus DDL_COMP_Init(COMP_TypeDef *COMPx, DDL_COMP_InitTypeDef *COMP_InitStruct)
{
    ErrorStatus status = SUCCESS;

    /* Check parameters */
    ASSERT_PARAM(IS_COMP_ALL_INSTANCE(COMPx));
    ASSERT_PARAM(IS_DDL_COMP_INPUT_PLUS(COMPx, COMP_InitStruct->InputPlus));
    ASSERT_PARAM(IS_DDL_COMP_OUTPUT_POLARITY(COMP_InitStruct->OutputPol));
    ASSERT_PARAM(IS_DDL_COMP_OUTPUT_MODE(COMP_InitStruct->Output));

    if (COMPx == COMP1)
    {
        ASSERT_PARAM(IS_DDL_COMP1_INPUT_MINUS(COMPx, COMP_InitStruct->InputMinus));
    }
    else
    {
        ASSERT_PARAM(IS_DDL_COMP_MODE(COMP_InitStruct->Mode));
        ASSERT_PARAM(IS_DDL_COMP2_INPUT_MINUS(COMPx, COMP_InitStruct->InputMinus));
    }

    /* COMP instance must not be locked */
    if (DDL_COMP_IsLocked(COMPx) == 0UL)
    {
        /* Configuration of comparator instance */
        if (COMPx == COMP1)
        {
            MODIFY_REG(COMPx->CSTS,
                        COMP_CSTS_INMCCFG |
                        COMP_CSTS_POLCFG |
                        COMP_CSTS_OUTSEL,
                        COMP_InitStruct->InputMinus |
                        COMP_InitStruct->OutputPol |
                        COMP_InitStruct->Output
            );
        }
        else
        {
            MODIFY_REG(COMPx->CSTS,
                        COMP_CSTS_SPEEDM |
                        COMP_CSTS_INMCCFG |
                        COMP_CSTS_POLCFG |
                        COMP_CSTS_OUTSEL,
                        COMP_InitStruct->Mode |
                        COMP_InitStruct->InputMinus |
                        COMP_InitStruct->OutputPol |
                        COMP_InitStruct->Output
            );
        }
    }
    else
    {
        status = ERROR;
    }

    return status;
}

/**
 * @brief  De-Initialize COMP function.
 * @param  COMPx COMP instance
 * @retval An ErrorStatus enumeration value:
 *         - SUCCESS: COMP registers are de-initialized
 *         - ERROR: COMP registers are not de-initialized
 * @note   If COMP instance is locked, de-initialization can't be performed.
 *         The only way to unlock the COMP instance is to perform a system reset.
 */
ErrorStatus DDL_COMP_DeInit(COMP_TypeDef *COMPx)
{
    ErrorStatus status = SUCCESS;

    /* Check parameters */
    ASSERT_PARAM(IS_COMP_ALL_INSTANCE(COMPx));

    /* COMP instance must not be locked */
    if (DDL_COMP_IsLocked(COMPx) == 0UL)
    {
        /* De-initialize the COMP registers to the reset values */
        DDL_COMP_WriteReg((COMPx), CSTS, 0x00000000UL);
    }
    else
    {
        /* COMP instance is locked */
        /* The only way to unlock the COMP instance is to perform a system reset */
        status = ERROR;
    }

    return status;
}

/**
 * @brief  Set the fields of structure COMP_InitStruct to default values.
 * @param  COMP_InitStruct Pointer to a @ref DDL_COMP_InitTypeDef structure
 *                          whose fields will be set to default values.
 * @retval None
 */
void DDL_COMP_StructInit(DDL_COMP_InitTypeDef *COMP_InitStruct)
{
    /* Set COMP_InitStruct fields to default values */
    COMP_InitStruct->Mode           = DDL_COMP_SPEEDMODE_LOW;
    COMP_InitStruct->InputPlus      = DDL_COMP_INPUT_PLUS_PC2;
    COMP_InitStruct->InputMinus     = DDL_COMP_INPUT_MINUS_VREFINT;
    COMP_InitStruct->OutputPol      = DDL_COMP_OUTPUTPOL_NONINVERTED;
    COMP_InitStruct->Output         = DDL_COMP_OUTPUT_NONE;
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

#endif /* COMP1 || COMP2 */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */
