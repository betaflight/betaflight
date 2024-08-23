/**
  *
  * @file    apm32f4xx_dal_eint.c
  * @author  MCD Application Team
  * @brief   EINT DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Extended Interrupts and events controller (EINT) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *
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
  * Copyright (c) 2018 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  @verbatim
  ==============================================================================
                    ##### EINT Peripheral features #####
  ==============================================================================
  [..]
    (+) Each Eint line can be configured within this driver.

    (+) Eint line can be configured in 3 different modes
        (++) Interrupt
        (++) Event
        (++) Both of them

    (+) Configurable Eint lines can be configured with 3 different triggers
        (++) Rising
        (++) Falling
        (++) Both of them

    (+) When set in interrupt mode, configurable Eint lines have two different
        interrupts pending registers which allow to distinguish which transition
        occurs:
        (++) Rising edge pending interrupt
        (++) Falling

    (+) Eint lines 0 to 15 are linked to gpio pin number 0 to 15. Gpio port can
        be selected through multiplexer.

                     ##### How to use this driver #####
  ==============================================================================
  [..]

    (#) Configure the EINT line using DAL_EINT_SetConfigLine().
        (++) Choose the interrupt line number by setting "Line" member from
             EINT_ConfigTypeDef structure.
        (++) Configure the interrupt and/or event mode using "Mode" member from
             EINT_ConfigTypeDef structure.
        (++) For configurable lines, configure rising and/or falling trigger
             "Trigger" member from EINT_ConfigTypeDef structure.
        (++) For Eint lines linked to gpio, choose gpio port using "GPIOSel"
             member from GPIO_InitTypeDef structure.

    (#) Get current Eint configuration of a dedicated line using
        DAL_EINT_GetConfigLine().
        (++) Provide exiting handle as parameter.
        (++) Provide pointer on EINT_ConfigTypeDef structure as second parameter.

    (#) Clear Eint configuration of a dedicated line using DAL_EINT_GetConfigLine().
        (++) Provide exiting handle as parameter.

    (#) Register callback to treat Eint interrupts using DAL_EINT_RegisterCallback().
        (++) Provide exiting handle as first parameter.
        (++) Provide which callback will be registered using one value from
             EINT_CallbackIDTypeDef.
        (++) Provide callback function pointer.

    (#) Get interrupt pending bit using DAL_EINT_GetPending().

    (#) Clear interrupt pending bit using DAL_EINT_GetPending().

    (#) Generate software interrupt using DAL_EINT_GenerateSWI().

  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @addtogroup EINT
  * @{
  */
/** MISRA C:2012 deviation rule has been granted for following rule:
  * Rule-18.1_b - Medium: Array `EINTCR' 1st subscript interval [0,7] may be out
  * of bounds [0,3] in following API :
  * DAL_EINT_SetConfigLine
  * DAL_EINT_GetConfigLine
  * DAL_EINT_ClearConfigLine
  */

#ifdef DAL_EINT_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup EINT_Private_Constants EINT Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup EINT_Exported_Functions
  * @{
  */

/** @addtogroup EINT_Exported_Functions_Group1
  *  @brief    Configuration functions
  *
@verbatim
 ===============================================================================
              ##### Configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Set configuration of a dedicated Eint line.
  * @param  heint Eint handle.
  * @param  pEintConfig Pointer on EINT configuration to be set.
  * @retval DAL Status.
  */
DAL_StatusTypeDef DAL_EINT_SetConfigLine(EINT_HandleTypeDef *heint, EINT_ConfigTypeDef *pEintConfig)
{
  uint32_t regval;
  uint32_t linepos;
  uint32_t maskline;

  /* Check null pointer */
  if ((heint == NULL) || (pEintConfig == NULL))
  {
    return DAL_ERROR;
  }

  /* Check parameters */
  ASSERT_PARAM(IS_EINT_LINE(pEintConfig->Line));
  ASSERT_PARAM(IS_EINT_MODE(pEintConfig->Mode));

  /* Assign line number to handle */
  heint->Line = pEintConfig->Line;

  /* Compute line mask */
  linepos = (pEintConfig->Line & EINT_PIN_MASK);
  maskline = (1uL << linepos);

  /* Configure triggers for configurable lines */
  if ((pEintConfig->Line & EINT_CONFIG) != 0x00u)
  {
    ASSERT_PARAM(IS_EINT_TRIGGER(pEintConfig->Trigger));

    /* Configure rising trigger */
    /* Mask or set line */
    if ((pEintConfig->Trigger & EINT_TRIGGER_RISING) != 0x00u)
    {
      EINT->RTEN |= maskline;
    }
    else
    {
      EINT->RTEN &= ~maskline;
    }

    /* Configure falling trigger */
    /* Mask or set line */
    if ((pEintConfig->Trigger & EINT_TRIGGER_FALLING) != 0x00u)
    {
      EINT->FTEN |= maskline;
    }
    else
    {
      EINT->FTEN &= ~maskline;
    }


    /* Configure gpio port selection in case of gpio exti line */
    if ((pEintConfig->Line & EINT_GPIO) == EINT_GPIO)
    {
      ASSERT_PARAM(IS_EINT_GPIO_PORT(pEintConfig->GPIOSel));
      ASSERT_PARAM(IS_EINT_GPIO_PIN(linepos));

      regval = SYSCFG->EINTCFG[linepos >> 2u];
      regval &= ~(SYSCFG_EINTCFG1_EINT0 << (SYSCFG_EINTCFG1_EINT1_Pos * (linepos & 0x03u)));
      regval |= (pEintConfig->GPIOSel << (SYSCFG_EINTCFG1_EINT1_Pos * (linepos & 0x03u)));
      SYSCFG->EINTCFG[linepos >> 2u] = regval;
    }
  }

  /* Configure interrupt mode : read current mode */
  /* Mask or set line */
  if ((pEintConfig->Mode & EINT_MODE_INTERRUPT) != 0x00u)
  {
    EINT->IMASK |= maskline;
  }
  else
  {
    EINT->IMASK &= ~maskline;
  }

  /* Configure event mode : read current mode */
  /* Mask or set line */
  if ((pEintConfig->Mode & EINT_MODE_EVENT) != 0x00u)
  {
    EINT->EMASK |= maskline;
  }
  else
  {
    EINT->EMASK &= ~maskline;
  }

  return DAL_OK;
}

/**
  * @brief  Get configuration of a dedicated Eint line.
  * @param  heint Eint handle.
  * @param  pEintConfig Pointer on structure to store Eint configuration.
  * @retval DAL Status.
  */
DAL_StatusTypeDef DAL_EINT_GetConfigLine(EINT_HandleTypeDef *heint, EINT_ConfigTypeDef *pEintConfig)
{
  uint32_t regval;
  uint32_t linepos;
  uint32_t maskline;

  /* Check null pointer */
  if ((heint == NULL) || (pEintConfig == NULL))
  {
    return DAL_ERROR;
  }

  /* Check the parameter */
  ASSERT_PARAM(IS_EINT_LINE(heint->Line));

  /* Store handle line number to configuration structure */
  pEintConfig->Line = heint->Line;

  /* Compute line mask */
  linepos = (pEintConfig->Line & EINT_PIN_MASK);
  maskline = (1uL << linepos);

  /* 1] Get core mode : interrupt */

  /* Check if selected line is enable */
  if ((EINT->IMASK & maskline) != 0x00u)
  {
    pEintConfig->Mode = EINT_MODE_INTERRUPT;
  }
  else
  {
    pEintConfig->Mode = EINT_MODE_NONE;
  }

  /* Get event mode */
  /* Check if selected line is enable */
  if ((EINT->EMASK & maskline) != 0x00u)
  {
    pEintConfig->Mode |= EINT_MODE_EVENT;
  }

  /* Get default Trigger and GPIOSel configuration */
  pEintConfig->Trigger = EINT_TRIGGER_NONE;
  pEintConfig->GPIOSel = 0x00u;

  /* 2] Get trigger for configurable lines : rising */
  if ((pEintConfig->Line & EINT_CONFIG) != 0x00u)
  {
    /* Check if configuration of selected line is enable */
    if ((EINT->RTEN & maskline) != 0x00u)
    {
      pEintConfig->Trigger = EINT_TRIGGER_RISING;
    }

    /* Get falling configuration */
    /* Check if configuration of selected line is enable */
    if ((EINT->FTEN & maskline) != 0x00u)
    {
      pEintConfig->Trigger |= EINT_TRIGGER_FALLING;
    }

    /* Get Gpio port selection for gpio lines */
    if ((pEintConfig->Line & EINT_GPIO) == EINT_GPIO)
    {
      ASSERT_PARAM(IS_EINT_GPIO_PIN(linepos));

      regval = (SYSCFG->EINTCFG[linepos >> 2u] << 16u );
      pEintConfig->GPIOSel = ((regval << (SYSCFG_EINTCFG1_EINT1_Pos * (3uL - (linepos & 0x03u)))) >> 28u);
    }
  }

  return DAL_OK;
}

/**
  * @brief  Clear whole configuration of a dedicated Eint line.
  * @param  heint Eint handle.
  * @retval DAL Status.
  */
DAL_StatusTypeDef DAL_EINT_ClearConfigLine(EINT_HandleTypeDef *heint)
{
  uint32_t regval;
  uint32_t linepos;
  uint32_t maskline;

  /* Check null pointer */
  if (heint == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameter */
  ASSERT_PARAM(IS_EINT_LINE(heint->Line));

  /* compute line mask */
  linepos = (heint->Line & EINT_PIN_MASK);
  maskline = (1uL << linepos);

  /* 1] Clear interrupt mode */
  EINT->IMASK = (EINT->IMASK & ~maskline);

  /* 2] Clear event mode */
  EINT->EMASK = (EINT->EMASK & ~maskline);

  /* 3] Clear triggers in case of configurable lines */
  if ((heint->Line & EINT_CONFIG) != 0x00u)
  {
    EINT->RTEN = (EINT->RTEN & ~maskline);
    EINT->FTEN = (EINT->FTEN & ~maskline);

    /* Get Gpio port selection for gpio lines */
    if ((heint->Line & EINT_GPIO) == EINT_GPIO)
    {
      ASSERT_PARAM(IS_EINT_GPIO_PIN(linepos));

      regval = SYSCFG->EINTCFG[linepos >> 2u];
      regval &= ~(SYSCFG_EINTCFG1_EINT0 << (SYSCFG_EINTCFG1_EINT1_Pos * (linepos & 0x03u)));
      SYSCFG->EINTCFG[linepos >> 2u] = regval;
    }
  }

  return DAL_OK;
}

/**
  * @brief  Register callback for a dedicated Eint line.
  * @param  heint Eint handle.
  * @param  CallbackID User callback identifier.
  *         This parameter can be one of @arg @ref EINT_CallbackIDTypeDef values.
  * @param  pPendingCbfn function pointer to be stored as callback.
  * @retval DAL Status.
  */
DAL_StatusTypeDef DAL_EINT_RegisterCallback(EINT_HandleTypeDef *heint, EINT_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void))
{
  DAL_StatusTypeDef status = DAL_OK;

  switch (CallbackID)
  {
    case  DAL_EINT_COMMON_CB_ID:
      heint->PendingCallback = pPendingCbfn;
      break;

    default:
      status = DAL_ERROR;
      break;
  }

  return status;
}

/**
  * @brief  Store line number as handle private field.
  * @param  heint Eint handle.
  * @param  EintLine Eint line number.
  *         This parameter can be from 0 to @ref EINT_LINE_NB.
  * @retval DAL Status.
  */
DAL_StatusTypeDef DAL_EINT_GetHandle(EINT_HandleTypeDef *heint, uint32_t EintLine)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_EINT_LINE(EintLine));

  /* Check null pointer */
  if (heint == NULL)
  {
    return DAL_ERROR;
  }
  else
  {
    /* Store line number as handle private field */
    heint->Line = EintLine;

    return DAL_OK;
  }
}

/**
  * @}
  */

/** @addtogroup EINT_Exported_Functions_Group2
  *  @brief EINT IO functions.
  *
@verbatim
 ===============================================================================
                       ##### IO operation functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Handle EINT interrupt request.
  * @param  heint Eint handle.
  * @retval none.
  */
void DAL_EINT_IRQHandler(EINT_HandleTypeDef *heint)
{
  uint32_t regval;
  uint32_t maskline;

  /* Compute line mask */
  maskline = (1uL << (heint->Line & EINT_PIN_MASK));

  /* Get pending bit  */
  regval = (EINT->IPEND & maskline);
  if (regval != 0x00u)
  {
    /* Clear pending bit */
    EINT->IPEND = maskline;

    /* Call callback */
    if (heint->PendingCallback != NULL)
    {
      heint->PendingCallback();
    }
  }
}

/**
  * @brief  Get interrupt pending bit of a dedicated line.
  * @param  heint Eint handle.
  * @param  Edge Specify which pending edge as to be checked.
  *         This parameter can be one of the following values:
  *           @arg @ref EINT_TRIGGER_RISING_FALLING
  *         This parameter is kept for compatibility with other series.
  * @retval 1 if interrupt is pending else 0.
  */
uint32_t DAL_EINT_GetPending(EINT_HandleTypeDef *heint, uint32_t Edge)
{
  uint32_t regval;
  uint32_t linepos;
  uint32_t maskline;

  /* Check parameters */
  ASSERT_PARAM(IS_EINT_LINE(heint->Line));
  ASSERT_PARAM(IS_EINT_CONFIG_LINE(heint->Line));
  ASSERT_PARAM(IS_EINT_PENDING_EDGE(Edge));

  /* Compute line mask */
  linepos = (heint->Line & EINT_PIN_MASK);
  maskline = (1uL << linepos);

  /* return 1 if bit is set else 0 */
  regval = ((EINT->IPEND & maskline) >> linepos);
  return regval;
}

/**
  * @brief  Clear interrupt pending bit of a dedicated line.
  * @param  heint Eint handle.
  * @param  Edge Specify which pending edge as to be clear.
  *         This parameter can be one of the following values:
  *           @arg @ref EINT_TRIGGER_RISING_FALLING
  *         This parameter is kept for compatibility with other series.
  * @retval None.
  */
void DAL_EINT_ClearPending(EINT_HandleTypeDef *heint, uint32_t Edge)
{
  uint32_t maskline;

  /* Check parameters */
  ASSERT_PARAM(IS_EINT_LINE(heint->Line));
  ASSERT_PARAM(IS_EINT_CONFIG_LINE(heint->Line));
  ASSERT_PARAM(IS_EINT_PENDING_EDGE(Edge));

  /* Compute line mask */
  maskline = (1uL << (heint->Line & EINT_PIN_MASK));

  /* Clear Pending bit */
  EINT->IPEND =  maskline;
}

/**
  * @brief  Generate a software interrupt for a dedicated line.
  * @param  heint Eint handle.
  * @retval None.
  */
void DAL_EINT_GenerateSWI(EINT_HandleTypeDef *heint)
{
  uint32_t maskline;

  /* Check parameters */
  ASSERT_PARAM(IS_EINT_LINE(heint->Line));
  ASSERT_PARAM(IS_EINT_CONFIG_LINE(heint->Line));

  /* Compute line mask */
  maskline = (1uL << (heint->Line & EINT_PIN_MASK));

  /* Generate Software interrupt */
  EINT->SWINTE = maskline;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_EINT_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

