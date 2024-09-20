/**
  *
  * @file    apm32f4xx_dal_wwdt.c
  * @brief   WWDT DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Window Watchdog (WWDT) peripheral:
  *           + Initialization and Configuration functions
  *           + IO operation functions
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
  *
  @verbatim
  ==============================================================================
                      ##### WWDT Specific features #####
  ==============================================================================
  [..]
    Once enabled the WWDT generates a system reset on expiry of a programmed
    time period, unless the program refreshes the counter (T[6;0] downcounter)
    before reaching 0x3F value (i.e. a reset is generated when the counter
    value rolls down from 0x40 to 0x3F).

    (+) An MCU reset is also generated if the counter value is refreshed
        before the counter has reached the refresh window value. This
        implies that the counter must be refreshed in a limited window.
    (+) Once enabled the WWDT cannot be disabled except by a system reset.
    (+) If required by application, an Early Wakeup Interrupt can be triggered
        in order to be warned before WWDT expiration. The Early Wakeup Interrupt
        (EWI) can be used if specific safety operations or data logging must
        be performed before the actual reset is generated. When the downcounter
        reaches 0x40, interrupt occurs. This mechanism requires WWDT interrupt
        line to be enabled in NVIC. Once enabled, EWI interrupt cannot be
        disabled except by a system reset.
    (+) WWDTRST flag in RCC CSR register can be used to inform when a WWDT
        reset occurs.
    (+) The WWDT counter input clock is derived from the APB clock divided
        by a programmable prescaler.
    (+) WWDT clock (Hz) = PCLK1 / (4096 * Prescaler)
    (+) WWDT timeout (mS) = 1000 * (T[5;0] + 1) / WWDT clock (Hz)
        where T[5;0] are the lowest 6 bits of Counter.
    (+) WWDT Counter refresh is allowed between the following limits :
        (++) min time (mS) = 1000 * (Counter - Window) / WWDT clock
        (++) max time (mS) = 1000 * (Counter - 0x40) / WWDT clock
    (+) Typical values:
        (++) Counter min (T[5;0] = 0x00) at 42MHz (PCLK1) with zero prescaler:
             max timeout before reset: approximately 97.52us
        (++) Counter max (T[5;0] = 0x3F) at 42MHz (PCLK1) with prescaler
             dividing by 8:
             max timeout before reset: approximately 49.93ms

                     ##### How to use this driver #####
  ==============================================================================

    *** Common driver usage ***
    ===========================

  [..]
    (+) Enable WWDT APB1 clock using __DAL_RCM_WWDT_CLK_ENABLE().
    (+) Configure the WWDT prescaler, refresh window value, counter value and early
        interrupt status using DAL_WWDT_Init() function. This will automatically
        enable WWDT and start its downcounter. Time reference can be taken from
        function exit. Care must be taken to provide a counter value
        greater than 0x40 to prevent generation of immediate reset.
    (+) If the Early Wakeup Interrupt (EWI) feature is enabled, an interrupt is
        generated when the counter reaches 0x40. When DAL_WWDT_IRQHandler is
        triggered by the interrupt service routine, flag will be automatically
        cleared and DAL_WWDT_WakeupCallback user callback will be executed. User
        can add his own code by customization of callback DAL_WWDT_WakeupCallback.
    (+) Then the application program must refresh the WWDT counter at regular
        intervals during normal operation to prevent an MCU reset, using
        DAL_WWDT_Refresh() function. This operation must occur only when
        the counter is lower than the refresh window value already programmed.

    *** Callback registration ***
    =============================

  [..]
    The compilation define USE_DAL_WWDT_REGISTER_CALLBACKS when set to 1 allows
    the user to configure dynamically the driver callbacks. Use Functions
    DAL_WWDT_RegisterCallback() to register a user callback.

    (+) Function DAL_WWDT_RegisterCallback() allows to register following
        callbacks:
        (++) EwiCallback : callback for Early WakeUp Interrupt.
        (++) MspInitCallback : WWDT MspInit.
    This function takes as parameters the DAL peripheral handle, the Callback ID
    and a pointer to the user callback function.

    (+) Use function DAL_WWDT_UnRegisterCallback() to reset a callback to
    the default weak (surcharged) function. DAL_WWDT_UnRegisterCallback()
    takes as parameters the DAL peripheral handle and the Callback ID.
    This function allows to reset following callbacks:
        (++) EwiCallback : callback for  Early WakeUp Interrupt.
        (++) MspInitCallback : WWDT MspInit.

    [..]
    When calling DAL_WWDT_Init function, callbacks are reset to the
    corresponding legacy weak (surcharged) functions:
    DAL_WWDT_EarlyWakeupCallback() and DAL_WWDT_MspInit() only if they have
    not been registered before.

    [..]
    When compilation define USE_DAL_WWDT_REGISTER_CALLBACKS is set to 0 or
    not defined, the callback registering feature is not available
    and weak (surcharged) callbacks are used.

    *** WWDT DAL driver macros list ***
    ===================================
    [..]
      Below the list of available macros in WWDT DAL driver.
      (+) __DAL_WWDT_ENABLE: Enable the WWDT peripheral
      (+) __DAL_WWDT_GET_FLAG: Get the selected WWDT's flag status
      (+) __DAL_WWDT_CLEAR_FLAG: Clear the WWDT's pending flags
      (+) __DAL_WWDT_ENABLE_IT: Enable the WWDT early wakeup interrupt

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#ifdef DAL_WWDT_MODULE_ENABLED
/** @defgroup WWDT WWDT
  * @brief WWDT DAL module driver.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup WWDT_Exported_Functions WWDT Exported Functions
  * @{
  */

/** @defgroup WWDT_Exported_Functions_Group1 Initialization and Configuration functions
  *  @brief    Initialization and Configuration functions.
  *
@verbatim
  ==============================================================================
          ##### Initialization and Configuration functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
      (+) Initialize and start the WWDT according to the specified parameters
          in the WWDT_InitTypeDef of associated handle.
      (+) Initialize the WWDT MSP.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the WWDT according to the specified.
  *         parameters in the WWDT_InitTypeDef of  associated handle.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_WWDT_Init(WWDT_HandleTypeDef *hwwdt)
{
  /* Check the WWDT handle allocation */
  if (hwwdt == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_WWDT_ALL_INSTANCE(hwwdt->Instance));
  ASSERT_PARAM(IS_WWDT_PRESCALER(hwwdt->Init.Prescaler));
  ASSERT_PARAM(IS_WWDT_WINDOW(hwwdt->Init.Window));
  ASSERT_PARAM(IS_WWDT_COUNTER(hwwdt->Init.Counter));
  ASSERT_PARAM(IS_WWDT_EWI_MODE(hwwdt->Init.EWIMode));

#if (USE_DAL_WWDT_REGISTER_CALLBACKS == 1)
  /* Reset Callback pointers */
  if (hwwdt->EwiCallback == NULL)
  {
    hwwdt->EwiCallback = DAL_WWDT_EarlyWakeupCallback;
  }

  if (hwwdt->MspInitCallback == NULL)
  {
    hwwdt->MspInitCallback = DAL_WWDT_MspInit;
  }

  /* Init the low level hardware */
  hwwdt->MspInitCallback(hwwdt);
#else
  /* Init the low level hardware */
  DAL_WWDT_MspInit(hwwdt);
#endif /* USE_DAL_WWDT_REGISTER_CALLBACKS */

  /* Set WWDT Counter */
  WRITE_REG(hwwdt->Instance->CTRL, (WWDT_CTRL_WWDTEN | hwwdt->Init.Counter));

  /* Set WWDT Prescaler and Window */
  WRITE_REG(hwwdt->Instance->CFR, (hwwdt->Init.EWIMode | hwwdt->Init.Prescaler | hwwdt->Init.Window));

  /* Return function status */
  return DAL_OK;
}


/**
  * @brief  Initialize the WWDT MSP.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @note   When rewriting this function in user file, mechanism may be added
  *         to avoid multiple initialize when DAL_WWDT_Init function is called
  *         again to change parameters.
  * @retval None
  */
__weak void DAL_WWDT_MspInit(WWDT_HandleTypeDef *hwwdt)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hwwdt);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_WWDT_MspInit could be implemented in the user file
   */
}


#if (USE_DAL_WWDT_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User WWDT Callback
  *         To be used instead of the weak (surcharged) predefined callback
  * @param  hwwdt WWDT handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_WWDT_EWI_CB_ID Early WakeUp Interrupt Callback ID
  *           @arg @ref DAL_WWDT_MSPINIT_CB_ID MspInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval status
  */
DAL_StatusTypeDef DAL_WWDT_RegisterCallback(WWDT_HandleTypeDef *hwwdt, DAL_WWDT_CallbackIDTypeDef CallbackID,
                                            pWWDT_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    status = DAL_ERROR;
  }
  else
  {
    switch (CallbackID)
    {
      case DAL_WWDT_EWI_CB_ID:
        hwwdt->EwiCallback = pCallback;
        break;

      case DAL_WWDT_MSPINIT_CB_ID:
        hwwdt->MspInitCallback = pCallback;
        break;

      default:
        status = DAL_ERROR;
        break;
    }
  }

  return status;
}


/**
  * @brief  Unregister a WWDT Callback
  *         WWDT Callback is redirected to the weak (surcharged) predefined callback
  * @param  hwwdt WWDT handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_WWDT_EWI_CB_ID Early WakeUp Interrupt Callback ID
  *           @arg @ref DAL_WWDT_MSPINIT_CB_ID MspInit callback ID
  * @retval status
  */
DAL_StatusTypeDef DAL_WWDT_UnRegisterCallback(WWDT_HandleTypeDef *hwwdt, DAL_WWDT_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  switch (CallbackID)
  {
    case DAL_WWDT_EWI_CB_ID:
      hwwdt->EwiCallback = DAL_WWDT_EarlyWakeupCallback;
      break;

    case DAL_WWDT_MSPINIT_CB_ID:
      hwwdt->MspInitCallback = DAL_WWDT_MspInit;
      break;

    default:
      status = DAL_ERROR;
      break;
  }

  return status;
}
#endif /* USE_DAL_WWDT_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup WWDT_Exported_Functions_Group2 IO operation functions
  *  @brief    IO operation functions
  *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Refresh the WWDT.
    (+) Handle WWDT interrupt request and associated function callback.

@endverbatim
  * @{
  */

/**
  * @brief  Refresh the WWDT.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_WWDT_Refresh(WWDT_HandleTypeDef *hwwdt)
{
  /* Write to WWDT CTRL the WWDT Counter value to refresh with */
  WRITE_REG(hwwdt->Instance->CTRL, (hwwdt->Init.Counter));

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Handle WWDT interrupt request.
  * @note   The Early Wakeup Interrupt (EWI) can be used if specific safety operations
  *         or data logging must be performed before the actual reset is generated.
  *         The EWI interrupt is enabled by calling DAL_WWDT_Init function with
  *         EWIMode set to WWDT_EWI_ENABLE.
  *         When the downcounter reaches the value 0x40, and EWI interrupt is
  *         generated and the corresponding Interrupt Service Routine (ISR) can
  *         be used to trigger specific actions (such as communications or data
  *         logging), before resetting the device.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @retval None
  */
void DAL_WWDT_IRQHandler(WWDT_HandleTypeDef *hwwdt)
{
  /* Check if Early Wakeup Interrupt is enable */
  if (__DAL_WWDT_GET_IT_SOURCE(hwwdt, WWDT_IT_EWI) != RESET)
  {
    /* Check if WWDT Early Wakeup Interrupt occurred */
    if (__DAL_WWDT_GET_FLAG(hwwdt, WWDT_FLAG_EWIF) != RESET)
    {
      /* Clear the WWDT Early Wakeup flag */
      __DAL_WWDT_CLEAR_FLAG(hwwdt, WWDT_FLAG_EWIF);

#if (USE_DAL_WWDT_REGISTER_CALLBACKS == 1)
      /* Early Wakeup registered callback */
      hwwdt->EwiCallback(hwwdt);
#else
      /* Early Wakeup callback */
      DAL_WWDT_EarlyWakeupCallback(hwwdt);
#endif /* USE_DAL_WWDT_REGISTER_CALLBACKS */
    }
  }
}


/**
  * @brief  WWDT Early Wakeup callback.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @retval None
  */
__weak void DAL_WWDT_EarlyWakeupCallback(WWDT_HandleTypeDef *hwwdt)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hwwdt);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_WWDT_EarlyWakeupCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_WWDT_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
