/**
  *
  * @file    apm32f4xx_ddl_eint.c
  * @author  MCD Application Team
  * @brief   EINT DDL module driver.
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
#if defined(USE_FULL_DDL_DRIVER)

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_ddl_eint.h"
#ifdef  USE_FULL_ASSERT
#include "apm32_assert.h"
#else
#define ASSERT_PARAM(_PARAM_) ((void)(_PARAM_))
#endif

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
/* Private macros ------------------------------------------------------------*/
/** @addtogroup EINT_DDL_Private_Macros
  * @{
  */

#define IS_DDL_EINT_LINE_0_31(__VALUE__)              (((__VALUE__) & ~DDL_EINT_LINE_ALL_0_31) == 0x00000000U)

#define IS_DDL_EINT_MODE(__VALUE__)                   (((__VALUE__) == DDL_EINT_MODE_IT)            \
                                                   || ((__VALUE__) == DDL_EINT_MODE_EVENT)         \
                                                   || ((__VALUE__) == DDL_EINT_MODE_IT_EVENT))


#define IS_DDL_EINT_TRIGGER(__VALUE__)                (((__VALUE__) == DDL_EINT_TRIGGER_NONE)       \
                                                   || ((__VALUE__) == DDL_EINT_TRIGGER_RISING)     \
                                                   || ((__VALUE__) == DDL_EINT_TRIGGER_FALLING)    \
                                                   || ((__VALUE__) == DDL_EINT_TRIGGER_RISING_FALLING))

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup EINT_DDL_Exported_Functions
  * @{
  */

/** @addtogroup EINT_DDL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the EINT registers to their default reset values.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: EINT registers are de-initialized
  *          - ERROR: not applicable
  */
uint32_t DDL_EINT_DeInit(void)
{
  /* Interrupt mask register set to default reset values */
  DDL_EINT_WriteReg(IMASK,   0x00000000U);
  /* Event mask register set to default reset values */
  DDL_EINT_WriteReg(EMASK,   0x00000000U);
  /* Rising Trigger selection register set to default reset values */
  DDL_EINT_WriteReg(RTEN,  0x00000000U);
  /* Falling Trigger selection register set to default reset values */
  DDL_EINT_WriteReg(FTEN,  0x00000000U);
  /* Software interrupt event register set to default reset values */
  DDL_EINT_WriteReg(SWINTE, 0x00000000U);
  /* Pending register set to default reset values */
  DDL_EINT_WriteReg(IPEND,    0x00FFFFFFU);

  return SUCCESS;
}

/**
  * @brief  Initialize the EINT registers according to the specified parameters in EINT_InitStruct.
  * @param  EINT_InitStruct pointer to a @ref DDL_EINT_InitTypeDef structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: EINT registers are initialized
  *          - ERROR: not applicable
  */
uint32_t DDL_EINT_Init(DDL_EINT_InitTypeDef *EINT_InitStruct)
{
  ErrorStatus status = SUCCESS;
  /* Check the parameters */
  ASSERT_PARAM(IS_DDL_EINT_LINE_0_31(EINT_InitStruct->Line_0_31));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(EINT_InitStruct->LineCommand));
  ASSERT_PARAM(IS_DDL_EINT_MODE(EINT_InitStruct->Mode));

  /* ENABLE LineCommand */
  if (EINT_InitStruct->LineCommand != DISABLE)
  {
    ASSERT_PARAM(IS_DDL_EINT_TRIGGER(EINT_InitStruct->Trigger));

    /* Configure EINT Lines in range from 0 to 31 */
    if (EINT_InitStruct->Line_0_31 != DDL_EINT_LINE_NONE)
    {
      switch (EINT_InitStruct->Mode)
      {
        case DDL_EINT_MODE_IT:
          /* First Disable Event on provided Lines */
          DDL_EINT_DisableEvent_0_31(EINT_InitStruct->Line_0_31);
          /* Then Enable IT on provided Lines */
          DDL_EINT_EnableIT_0_31(EINT_InitStruct->Line_0_31);
          break;
        case DDL_EINT_MODE_EVENT:
          /* First Disable IT on provided Lines */
          DDL_EINT_DisableIT_0_31(EINT_InitStruct->Line_0_31);
          /* Then Enable Event on provided Lines */
          DDL_EINT_EnableEvent_0_31(EINT_InitStruct->Line_0_31);
          break;
        case DDL_EINT_MODE_IT_EVENT:
          /* Directly Enable IT & Event on provided Lines */
          DDL_EINT_EnableIT_0_31(EINT_InitStruct->Line_0_31);
          DDL_EINT_EnableEvent_0_31(EINT_InitStruct->Line_0_31);
          break;
        default:
          status = ERROR;
          break;
      }
      if (EINT_InitStruct->Trigger != DDL_EINT_TRIGGER_NONE)
      {
        switch (EINT_InitStruct->Trigger)
        {
          case DDL_EINT_TRIGGER_RISING:
            /* First Disable Falling Trigger on provided Lines */
            DDL_EINT_DisableFallingTrig_0_31(EINT_InitStruct->Line_0_31);
            /* Then Enable Rising Trigger on provided Lines */
            DDL_EINT_EnableRisingTrig_0_31(EINT_InitStruct->Line_0_31);
            break;
          case DDL_EINT_TRIGGER_FALLING:
            /* First Disable Rising Trigger on provided Lines */
            DDL_EINT_DisableRisingTrig_0_31(EINT_InitStruct->Line_0_31);
            /* Then Enable Falling Trigger on provided Lines */
            DDL_EINT_EnableFallingTrig_0_31(EINT_InitStruct->Line_0_31);
            break;
          case DDL_EINT_TRIGGER_RISING_FALLING:
            DDL_EINT_EnableRisingTrig_0_31(EINT_InitStruct->Line_0_31);
            DDL_EINT_EnableFallingTrig_0_31(EINT_InitStruct->Line_0_31);
            break;
          default:
            status = ERROR;
            break;
        }
      }
    }
  }
  /* DISABLE LineCommand */
  else
  {
    /* De-configure EINT Lines in range from 0 to 31 */
    DDL_EINT_DisableIT_0_31(EINT_InitStruct->Line_0_31);
    DDL_EINT_DisableEvent_0_31(EINT_InitStruct->Line_0_31);
  }
  return status;
}

/**
  * @brief  Set each @ref DDL_EINT_InitTypeDef field to default value.
  * @param  EINT_InitStruct Pointer to a @ref DDL_EINT_InitTypeDef structure.
  * @retval None
  */
void DDL_EINT_StructInit(DDL_EINT_InitTypeDef *EINT_InitStruct)
{
  EINT_InitStruct->Line_0_31      = DDL_EINT_LINE_NONE;
  EINT_InitStruct->LineCommand    = DISABLE;
  EINT_InitStruct->Mode           = DDL_EINT_MODE_IT;
  EINT_InitStruct->Trigger        = DDL_EINT_TRIGGER_FALLING;
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

#endif /* defined (EINT) */

/**
  * @}
  */

#endif /* USE_FULL_DDL_DRIVER */

