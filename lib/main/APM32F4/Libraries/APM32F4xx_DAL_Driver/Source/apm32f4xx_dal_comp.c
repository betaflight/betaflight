/**
  *
  * @file    apm32f4xx_dal_comp.c
  * @brief   COMP DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the COMP peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
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
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023-2024 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  @verbatim
 ===============================================================================
                     ##### COMP Peripheral features #####
 ===============================================================================

  [..]
    The APM32F411xx integrates 2 analog comparators COMP1 and COMP2:
    (#) The COMP input minus (inverting input) and input plus (non inverting input)
        can be set to internal references or to GPIO pins

    (#) The COMP output is available using DAL_COMP_GetOutputLevel().

    (#) The COMP output can be redirected to embedded timers (TMR1, TMR8, TMR2, TMR3, TMR4)

    (#) Paris of comparators instances can be combined in window mode.

    ##### How to use this driver #####
===============================================================================
[..]
    This driver provides functions to configure and program the Comparators of 
    APM32F411xx devices.

    To use the comparator, perform the following steps:

    (#) Initialize the COMP low level resources by implementing the DAL_COMP_MspInit():
        (##) Enable the COMP interface clock using __DAL_RCC_SYSCFG_CLK_ENABLE() function.
        (##) Configure the GPIO connected to comparator inputs plus and minus in analog mode
             using DAL_GPIO_Init().

    (#) Configuration the COMP peripheral using DAL_COMP_Init() function:
    (##) Select the window mode (Available only on COMP1 instance)
    (##) Select the speed mode (Available only on COMP2 instance)
    (##) Select the input minus (inverting input)
    (##) Select the input plus (non-inverting input)
    (##) Select the output polarity
    (##) Select the output redirection

    (#) Enable the comparator using DAL_COMP_Start() function

    (#) Use DAL_COMP_GetOutputLevel() function to get the output level (high or low).

    (#) Disable the comparator using DAL_COMP_Stop() function

    (#) De-initialize the comparator using DAL_COMP_DeInit() function.

    (#) For safety purpose, use DAL_COMP_Lock() function to lock the whole
        configuration of the comparator unit and the CSTS registers.
        Then, no further change of the comparator unit configuration can be done
        until the next reset.

    (#) Reset the comparator only by system reset.

    (#) If needed, reinitialize the comparator using DAL_COMP_Init() function.
    
    *** Callback registration ***
    =============================================
    [..]

    The compilation define  USE_DAL_COMP_REGISTER_CALLBACKS when set to 1,
    allows the user to configure dynamically the driver callbacks.
    Use Functions @ref DAL_COMP_RegisterCallback() to register an callback.
    [..]

    Function @ref DAL_COMP_RegisterCallback() allows to register following callbacks:
      (+) MspInitCallback    : COMP MspInit.
      (+) MspDeInitCallback  : COMP MspDeInit.
    This function takes as parameters the DAL peripheral handle, the Callback ID
    and a pointer to the user callback function.
    [..]

    Use function @ref DAL_COMP_UnRegisterCallback() to reset a callback to the default
    weak function.
    [..]

    @ref DAL_COMP_UnRegisterCallback() takes as parameters the DAL peripheral handle,
    and the Callback ID.
    This function allows to reset following callbacks:
      (+) MspInitCallback    : COMP MspInit.
      (+) MspDeInitCallback  : COMP MspDeInit.
    [..]

    By default, after the @ref DAL_COMP_Init() and if the state is DAL_COMP_STATE_RESET
    all callbacks are reset to the corresponding legacy weak functions.
    Exception done for MspInit and MspDeInit functions that are respectively
    reset to the legacy weak functions in the @ref DAL_COMP_Init() and @ref DAL_COMP_DeInit() only when
    these callbacks are null (not registered beforehand).
    [..]

    If MspInit or MspDeInit are not null, the @ref DAL_COMP_Init() and @ref DAL_COMP_DeInit()
    keep and use the user MspInit/MspDeInit callbacks (registered beforehand) whatever the state.
    [..]

    Callbacks can be registered/unregistered in DAL_COMP_STATE_READY state only.
    Exception done MspInit/MspDeInit functions that can be registered/unregistered
    in DAL_COMP_STATE_READY or DAL_COMP_STATE_RESET state,
    thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
    [..]

    Then, the user first registers the MspInit/MspDeInit user callbacks
    using @ref DAL_COMP_RegisterCallback() before calling @ref DAL_COMP_DeInit()
    or @ref DAL_COMP_Init() function.
    [..]

    When the compilation define USE_DAL_COMP_REGISTER_CALLBACKS is set to 0 or
    not defined, the callback registration feature is not available and all callbacks
    are set to the corresponding legacy weak functions.

  @endverbatim
  ******************************************************************************

  Table 1. COMP inputs for APM32F411xx devices
    +---------------------------------------------------------+
    |                |                |   COMP1   |   COMP2   |
    |----------------|----------------|-----------|-----------|
    |                |                |   PC0     |   PC2     |
    | Input Plus     |  GPIO          |   PC2     |   PC2     |
    |                |                |           |           |
    |----------------|----------------|-----------|-----------|
    |                | VREFINT        | Available | Available |
    |                | PC1            | Available |           |
    |                | PC3            |           | Available |
    | Input Minus    | 1/4 VREFINT    |           | Available |
    |                | 1/2 VREFINT    |           | Available |
    |                | 3/4 VREFINT    |           | Available |
    |                |                |           |           |
    +---------------------------------------------------------+
    |                | TMR1 BK IN     | Available | Available |
    |                | TMR1 IC1       | Available | Available |
    |                | TMR1 ETRF      | Available | Available |
    |                | TMR8 BK IN     | Available | Available |
    |                | TMR8 IC1       | Available | Available |
    | Output         | TMR8 ETRF      | Available | Available |
    |                | TMR2 IC4       | Available | Available |
    |                | TMR2 ETRF      | Available | Available |
    |                | TMR3 IC1       | Available | Available |
    |                | TMR3 ETRF      | Available | Available |
    |                | TMR4 IC1       | Available | Available |
    |                |                |           |           |
    +---------------------------------------------------------+

    ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup COMP COMP
 * @brief COMP DAL driver modules
 * @{
 */

#ifdef DAL_COMP_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup COMP_Private_Constants COMP Private Constants
  * @{
  */

/* Delay for COMP startup time.                                         */
#define COMP_DELAY_STARTUP_US               (80UL)      /*!< Delay for COMP startup time. */

/**
 * @}
 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup COMP_Exported_Functions COMP Exported Functions
  * @{
  */

/** @defgroup COMP_Exported_Functions_Group1 Initialization and de-initialization functions
 * @brief    Initialization and Configuration functions
 *
 * @verbatim
 * ===============================================================================
 *             ##### Initialization and Configuration functions #####
 * ===============================================================================
 *   [..] This section provides functions allowing to:
 *     (+) Initialize and configure the COMP.
 *     (+) De-initialize the COMP.
 *
 * @endverbatim
  * @{
  */

/**
 * @brief  Initialize the COMP according to the specified parameters
 *         in the COMP_InitTypeDef and initialize the associated handle.
 * @note   If the selected comparator is locked, initialization cannot be performed.
 *         To unlock the configuration, perform a system reset.
 * @param  hcomp COMP handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_COMP_Init(COMP_HandleTypeDef *hcomp)
{
    DAL_StatusTypeDef status = DAL_OK;
    uint32_t temp;

    /* Check the COMP handle allocation */
    if (hcomp == NULL)
    {
        status = DAL_ERROR;
    }
    else if (__DAL_COMP_IS_LOCKED(hcomp))
    {
        status = DAL_ERROR;
    }
    else
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_COMP_ALL_INSTANCE(hcomp->Instance));
        if (hcomp->Instance == COMP1)
        {
            ASSERT_PARAM(IS_COMP_WINDOWMODE(hcomp->Init.WindowMode));
            ASSERT_PARAM(IS_COMP1_INVERTINGINPUT(hcomp->Init.InvertingInput));
        }
        else
        {
            ASSERT_PARAM(IS_COMP_SPEEDMODE(hcomp->Init.SpeedMode));
            ASSERT_PARAM(IS_COMP2_INVERTINGINPUT(hcomp->Init.InvertingInput));
            ASSERT_PARAM(IS_COMP_NONINVERTINGINPUT(hcomp->Init.NonInvertingInput));
        }
        ASSERT_PARAM(IS_COMP_OUTPUT(hcomp->Init.Output));
        ASSERT_PARAM(IS_COMP_OUTPUTPOL(hcomp->Init.OutputPol));

        if (hcomp->State == DAL_COMP_STATE_RESET)
        {
            /* Allocate lock resource and initialize it */
            hcomp->Lock = DAL_UNLOCKED;

            /* Set COMP error code to none */
            COMP_CLEAR_ERRORCODE(hcomp);

#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
            /* Init the COMP Callback settings */
            if (hcomp->MspInitCallback == NULL)
            {
                hcomp->MspInitCallback = DAL_COMP_MspInit; /* Legacy weak MspInit */
            }

            /* Init the low level hardware */
            hcomp->MspInitCallback(hcomp);
#else
            /* Init the low level hardware */
            DAL_COMP_MspInit(hcomp);
#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */
        }

        /* Set COMP1 parameters */
        if (hcomp->Instance == COMP1)
        {
            temp = (hcomp->Init.WindowMode | \
                    hcomp->Init.InvertingInput | \
                    hcomp->Init.OutputPol | \
                    hcomp->Init.Output);

            /* Set parameters in COMP_CSTS register */
            MODIFY_REG(hcomp->Instance->CSTS,
                       COMP_CSTS_WMODESEL | COMP_CSTS_INMCCFG | COMP_CSTS_POLCFG | COMP_CSTS_OUTSEL,
                       temp);
        }
        /* Set COMP2 parameters */
        else
        {
            temp = (hcomp->Init.Mode | \
                    hcomp->Init.InvertingInput | \
                    hcomp->Init.NonInvertingInput | \
                    hcomp->Init.OutputPol | \
                    hcomp->Init.Output);

            /* Set parameters in COMP_CSTS register */
            MODIFY_REG(hcomp->Instance->CSTS,
                       COMP_CSTS_SPEEDM | COMP_CSTS_INMCCFG | COMP_CSTS_INPCCFG | COMP_CSTS_POLCFG | COMP_CSTS_OUTSEL,
                       temp);
        }

        /* Set DAL COMP handle state */
        if (hcomp->State == DAL_COMP_STATE_RESET)
        {
            hcomp->State = DAL_COMP_STATE_READY;
        }
    }

    return status;
}

/**
 * @brief  DeInitialize the COMP peripheral.
 * @param  hcomp COMP handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_COMP_DeInit(COMP_HandleTypeDef *hcomp)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Check the COMP handle allocation */
    if (hcomp == NULL)
    {
        status = DAL_ERROR;
    }
    else if (__DAL_COMP_IS_LOCKED(hcomp))
    {
        status = DAL_ERROR;
    }
    else
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_COMP_ALL_INSTANCE(hcomp->Instance));

        /* Set COMP_CSTS register to reset value */
        WRITE_REG(hcomp->Instance->CSTS, 0x00000000UL);

#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
        if (hcomp->MspDeInitCallback == NULL)
        {
            hcomp->MspDeInitCallback = DAL_COMP_MspDeInit; /* Legacy weak MspDeInit */
        }

        /* DeInit the low level hardware */
        hcomp->MspDeInitCallback(hcomp);
#else
        /* DeInit the low level hardware */
        DAL_COMP_MspDeInit(hcomp);
#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */

        /* Set DAL COMP handle state */
        hcomp->State = DAL_COMP_STATE_RESET;

        /* Release Lock */
        __DAL_UNLOCK(hcomp);
    }

    return status;
}

/**
 * @brief  Initialize the COMP MSP.
 * @param  hcomp COMP handle
 * @retval None
 */
__WEAK void DAL_COMP_MspInit(COMP_HandleTypeDef *hcomp)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcomp);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_COMP_MspInit could be implemented in the user file
     */
}

/**
 * @brief  DeInitialize COMP MSP.
 * @param  hcomp COMP handle
 * @retval None
 */
__WEAK void DAL_COMP_MspDeInit(COMP_HandleTypeDef *hcomp)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcomp);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_COMP_MspDeInit could be implemented in the user file
     */
}

#if (USE_DAL_COMP_REGISTER_CALLBACKS == 1)
/**
 * @brief  Register a User COMP Callback
 *         To be used instead of the weak predefined callback
 * @param  hcomp COMP handle
 * @param  CallbackID ID of the callback to be registered
 *         This parameter can be one of the following values:
 *           @arg @ref DAL_COMP_MSPINIT_CB_ID COMP MspInit callback ID
 *           @arg @ref DAL_COMP_MSPDEINIT_CB_ID COMP MspDeInit callback ID
 * @param  pCallback pointer to the Callback function
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_COMP_RegisterCallback(COMP_HandleTypeDef *hcomp, DAL_COMP_CallbackIDTypeDef CallbackID, pCOMP_CallbackTypeDef pCallback)
{
    DAL_StatusTypeDef status = DAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hcomp->ErrorCode |= DAL_COMP_ERROR_INVALID_CALLBACK;

        return DAL_ERROR;
    }

    if (DAL_COMP_STATE_READY == hcomp->State)
    {
        switch (CallbackID)
        {
            case DAL_COMP_MSPINIT_CB_ID:
                hcomp->MspInitCallback = pCallback;
                break;

            case DAL_COMP_MSPDEINIT_CB_ID:
                hcomp->MspDeInitCallback = pCallback;
                break;

            default:
                /* Update the error code */
                hcomp->ErrorCode |= DAL_COMP_ERROR_INVALID_CALLBACK;

                /* Return error status */
                status = DAL_ERROR;
                break;
        }
    }
    else if (DAL_COMP_STATE_RESET == hcomp->State)
    {
        switch (CallbackID)
        {
            case DAL_COMP_MSPINIT_CB_ID:
                hcomp->MspInitCallback = pCallback;
                break;

            case DAL_COMP_MSPDEINIT_CB_ID:
                hcomp->MspDeInitCallback = pCallback;
                break;

            default:
                /* Update the error code */
                hcomp->ErrorCode |= DAL_COMP_ERROR_INVALID_CALLBACK;

                /* Return error status */
                status = DAL_ERROR;
                break;
        }
    }
    else
    {
        /* Update the error code */
        hcomp->ErrorCode |= DAL_COMP_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status = DAL_ERROR;
    }

    return status;
}

/**
 * @brief  Unregister an COMP Callback
 *         COMP callback is redirected to the weak predefined callback
 * @param  hcomp COMP handle
 * @param  CallbackID ID of the callback to be unregistered
 *         This parameter can be one of the following values:
 *           @arg @ref DAL_COMP_MSPINIT_CB_ID COMP MspInit callback ID
 *           @arg @ref DAL_COMP_MSPDEINIT_CB_ID COMP MspDeInit callback ID
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_COMP_UnRegisterCallback(COMP_HandleTypeDef *hcomp, DAL_COMP_CallbackIDTypeDef CallbackID)
{
    DAL_StatusTypeDef status = DAL_OK;

    if (DAL_COMP_STATE_READY == hcomp->State)
    {
        switch (CallbackID)
        {
            case DAL_COMP_MSPINIT_CB_ID:
                hcomp->MspInitCallback = DAL_COMP_MspInit; /* Legacy weak MspInit */
                break;

            case DAL_COMP_MSPDEINIT_CB_ID:
                hcomp->MspDeInitCallback = DAL_COMP_MspDeInit; /* Legacy weak MspDeInit */
                break;

            default:
                /* Update the error code */
                hcomp->ErrorCode |= DAL_COMP_ERROR_INVALID_CALLBACK;

                /* Return error status */
                status = DAL_ERROR;
                break;
        }
    }
    else if (DAL_COMP_STATE_RESET == hcomp->State)
    {
        switch (CallbackID)
        {
            case DAL_COMP_MSPINIT_CB_ID:
                hcomp->MspInitCallback = DAL_COMP_MspInit; /* Legacy weak MspInit */
                break;

            case DAL_COMP_MSPDEINIT_CB_ID:
                hcomp->MspDeInitCallback = DAL_COMP_MspDeInit; /* Legacy weak MspDeInit */
                break;

            default:
                /* Update the error code */
                hcomp->ErrorCode |= DAL_COMP_ERROR_INVALID_CALLBACK;

                /* Return error status */
                status = DAL_ERROR;
                break;
        }
    }
    else
    {
        /* Update the error code */
        hcomp->ErrorCode |= DAL_COMP_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status = DAL_ERROR;
    }

    return status;
}

#endif /* USE_DAL_COMP_REGISTER_CALLBACKS */
/**
 * @}
 */

/** @defgroup COMP_Exported_Functions_Group2 Start-Stop operation functions
 * @brief Start-Stop operation functions.
 *
 * @verbatim
 * ===============================================================================
 *                  ##### IO operation functions #####
 * ===============================================================================
 *  [..] This section provides functions allowing to:
 *     (+) Start the comparator.
 *     (+) Stop the comparator.
 * 
 * @endverbatim
  * @{
  */

/**
 * @brief  Start the comparator.
 * @param  hcomp COMP handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_COMP_Start(COMP_HandleTypeDef *hcomp)
{
    DAL_StatusTypeDef status = DAL_OK;
    uint32_t time_delay = 0UL;

    /* Check the COMP handle allocation */
    if (hcomp == NULL)
    {
        status = DAL_ERROR;
    }
    else if (__DAL_COMP_IS_LOCKED(hcomp))
    {
        status = DAL_ERROR;
    }
    else
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_COMP_ALL_INSTANCE(hcomp->Instance));

        if (hcomp->State == DAL_COMP_STATE_READY)
        {
            /* Enable the selected comparator */
            SET_BIT(hcomp->Instance->CSTS, COMP_CSTS_EN);

            /* Set COMP state */
            hcomp->State = DAL_COMP_STATE_BUSY;

            /* Delay for COMP startup time */
            time_delay = ((COMP_DELAY_STARTUP_US / 10UL) * ((SystemCoreClock / (1000000UL * 2UL)) + 1UL));
            while (time_delay != 0UL)
            {
                time_delay--;
            }
        }
        else
        {
            status = DAL_ERROR;
        }
    }

    return status;
}

/**
 * @brief  Stop the comparator.
 * @param  hcomp COMP handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_COMP_Stop(COMP_HandleTypeDef *hcomp)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Check the COMP handle allocation */
    if (hcomp == NULL)
    {
        status = DAL_ERROR;
    }
    else if (__DAL_COMP_IS_LOCKED(hcomp))
    {
        status = DAL_ERROR;
    }
    else
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_COMP_ALL_INSTANCE(hcomp->Instance));

        if (hcomp->State == DAL_COMP_STATE_RESET)
        {
            /* Disable the selected comparator */
            CLEAR_BIT(hcomp->Instance->CSTS, COMP_CSTS_EN);

            /* Set COMP state */
            hcomp->State = DAL_COMP_STATE_READY;
        }
        else
        {
            status = DAL_ERROR;
        }
    }

    return status;
}

/**
 * @}
 */

/** @defgroup COMP_Exported_Functions_Group3 Peripheral Control functions
 * @brief    Peripheral Control functions
 * 
 * @verbatim
 * ===============================================================================
 *                ##### Peripheral Control functions #####
 * ===============================================================================
 * [..] This section provides functions allowing to:
 *     (+) Lock the selected comparator configuration.
 * 
 * @endverbatim
  * @{
  */

/**
 * @brief  Lock the selected comparator configuration.
 * @note   A system reset is required to unlock the comparator configuration.
 * @param  hcomp COMP handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_COMP_Lock(COMP_HandleTypeDef *hcomp)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Check the COMP handle allocation */
    if (hcomp == NULL)
    {
        status = DAL_ERROR;
    }
    else if (__DAL_COMP_IS_LOCKED(hcomp))
    {
        status = DAL_ERROR;
    }
    else
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_COMP_ALL_INSTANCE(hcomp->Instance));

        /* Set lock flag on state */
        switch(hcomp->State)
        {
            case DAL_COMP_STATE_READY:
                hcomp->State = DAL_COMP_STATE_READY_LOCKED;
                break;

            case DAL_COMP_STATE_RESET:
                hcomp->State = DAL_COMP_STATE_RESET_LOCKED;
                break;

            default:
                hcomp->State = DAL_COMP_STATE_BUSY_LOCKED;
                break;
        }
    }

    if (status == DAL_OK)
    {
        /* Set lock flag on handle */
        __DAL_COMP_LOCK(hcomp);
    }

    return status;
}

/**
 * @brief Return the output level (high or low) of the selected comparator.
 * @param  hcomp COMP handle
 * @retval Returns the selected comparator output level: 
 * @note  The output level depends on the selected polarity.
 *        If the polarity is not inverted:
 *        - Comparator output is low when the non-inverting input is at a lower
 *          voltage than the inverting input
 *        - Comparator output is high when the non-inverting input is at a higher
 *          voltage than the inverting input
 *       If the polarity is inverted:
 *        - Comparator output is high when the non-inverting input is at a lower
 *          voltage than the inverting input
 *        - Comparator output is low when the non-inverting input is at a higher
 *          voltage than the inverting input
 */
uint32_t DAL_COMP_GetOutputLevel(COMP_HandleTypeDef *hcomp)
{
    /* Check the parameters */
    ASSERT_PARAM(IS_COMP_ALL_INSTANCE(hcomp->Instance));

    /* Return the selected comparator output level */
    return (uint32_t)(READ_BIT(hcomp->Instance->CSTS, COMP_CSTS_OUTVAL));
}

/**
 * @}
 */

/** @defgroup COMP_Exported_Functions_Group4 Peripheral State functions
 * @brief    Peripheral State functions
 * 
 * @verbatim
 * ===============================================================================
 *                ##### Peripheral State functions #####
 * ===============================================================================
 * [..] This section provides functions allowing to:
 * 
 * @endverbatim
  * @{
  */

/**
 * @brief  Return the COMP state.
 * @param  hcomp COMP handle
 * @retval DAL state
 */
DAL_COMP_StateTypeDef DAL_COMP_GetState(COMP_HandleTypeDef *hcomp)
{
   /* Check the COMP handle allocation */
    if (hcomp == NULL)
    {
        return DAL_COMP_STATE_RESET;
    }

    /* Check the parameter */
    ASSERT_PARAM(IS_COMP_ALL_INSTANCE(hcomp->Instance));

    /* Return DAL COMP state */
    return hcomp->State;
}

/**
 * @brief  Return the COMP error code.
 * @param  hcomp COMP handle
 * @retval COMP error code
 */
uint32_t DAL_COMP_GetError(COMP_HandleTypeDef *hcomp)
{
    /* Check the COMP handle allocation */
    if (hcomp == NULL)
    {
        return 0UL;
    }

    /* Check the parameter */
    ASSERT_PARAM(IS_COMP_ALL_INSTANCE(hcomp->Instance));

    /* Return DAL COMP error code */
    return hcomp->ErrorCode;
}

/**
 * @}
 */

/**
 * @}
 */

#endif /* DAL_COMP_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

