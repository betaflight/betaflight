/**
  ******************************************************************************
  * @file    um324xx_hal_wwdt.c
  * @author  MCU Team
  * @version V1.00
  * @date    2023-4-21
  * @brief   WWDT HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Window Watchdog Timer (WWDT) peripheral:
  *           + Initialization and Configuration functions
  *           + IO operation functions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 - 2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  @verbatim
  ==============================================================================
                      ##### WWDT Specific features #####
  ==============================================================================
  [..]
    Once enabled the WWDT generates a system reset on expiry of a programmed
    time period, unless the program refreshes the counter (CNT upcounter)
    before reaching Ncfg value

    (+) An MCU reset is also generated if the counter value is refreshed
        before the counter has reached the refresh window value, i.e. 50%*Ncfg. 
        This implies that the counter must be refreshed in a limited window.
    (+) An MCU soft reset is also generated if the counter value is refreshed
        by except 0xAC.
    (+) Once enabled the WWDT cannot be disabled except by a system reset.
    (+) If required by application, an Early Warning Interrupt can be triggered
        in order to be warned before WWDT expiration. The Early Warning Interrupt
        (EWI) can be used if specific safety operations or data logging must
        be performed before the actual reset is generated. When the upcounter
        reaches 75%*Ncfg, interrupt occurs. This mechanism requires WWDT interrupt
        line to be enabled in NVIC. Once enabled, EWI interrupt cannot be
        disabled except by a system reset.
    (+) WWDTRSTF flag in RCM RFR register can be used to inform if a WWDT
        reset occurs.
    (+) The WWDT counter input clock is derived from the APB0 clock divided
        by a programmable prescaler Ncfg.
    (+) WWDT clock (Hz) = PCLK0 / (4096 * Ncfg)
    (+) WWDT timeout (mS) = 1000 * Ncfg / WWDT clock (Hz)
        where Ncfg can be following values:
            Ncfg = 1        where WWDT_CFG[3;0] = 0000b
            Ncfg = 4        where WWDT_CFG[3;0] = 0001b
            Ncfg = 16       where WWDT_CFG[3;0] = 0010b
            Ncfg = 64       where WWDT_CFG[3;0] = 0011b
            Ncfg = 128      where WWDT_CFG[3;0] = 0100b
            Ncfg = 256      where WWDT_CFG[3;0] = 0101b
            Ncfg = 512      where WWDT_CFG[3;0] = 0110b
            Ncfg = 1024     where WWDT_CFG[3;0] = 0111b
            Ncfg = 2048     where WWDT_CFG[3;0] = 1000b
            Ncfg = 4096     where WWDT_CFG[3;0] = 1001b
            Ncfg = 8192     where WWDT_CFG[3;0] = 1010b
            Ncfg = 16384    where WWDT_CFG[3;0] = 1011b
            Ncfg = 32768    where WWDT_CFG[3;0] = 1100b
            Ncfg = 65536    where WWDT_CFG[3;0] = 1101b
            Ncfg = reserve  where WWDT_CFG[3;0] = 1110b
            Ncfg = reserve  where WWDT_CFG[3;0] = 1111b
    (+) WWDT Counter refresh is allowed between the following limits :
        (++) min time (mS) = 1000 * (Ncfg - Ncfg/2) / WWDT clock
        (++) max time (mS) = 1000 * (Ncfg) / WWDT clock
    (+) Typical values:
        (++) Counter min (WWDT_CFG[3;0] = 0000b) at 42MHz (PCLK0) with 1 prescaler:
             max timeout before reset: approximately 97.52us
        (++) Counter max (WWDT_CFG[3;0] = 1101b) at 42MHz (PCLK0) with prescaler
             dividing by 65536:
             max timeout before reset: approximately 6.391320384s

                     ##### How to use this driver #####
  ==============================================================================

    *** Common driver usage ***
    ===========================

  [..]
    (+) Enable WWDT APB0 clock using __HAL_RCM_WWDT_CLK_ENABLE().
    (+) Configure the WWDT prescaler Ncfg, refresh counter value and early warning
        interrupt status using HAL_WWDT_Init() function. This will automatically
        enable WWDT and start its upcounter. Time reference can be taken from
        function exit. 
    (+) If the Early Warning Interrupt (EWI) feature is enabled, an interrupt is
        generated when the counter reaches 75%Ncfg. When HAL_WWDT_IRQHandler is
        triggered by the interrupt service routine, flag will be automatically
        cleared and HAL_WWDT_EarlyWarningCallback user callback will be executed. User
        can add his own code by customization of callback HAL_WWDT_EarlyWarningCallback.
    (+) Then the application program must refresh the WWDT counter at regular
        intervals during normal operation to prevent an MCU reset, using
        HAL_WWDT_Refresh() function. This operation must occur only when
        the counter is higher than the 50%Ncfg value already programmed.

    *** Callback registration ***
    =============================

  [..]
    The compilation define USE_HAL_WWDT_REGISTER_CALLBACKS when set to 1 allows
    the user to configure dynamically the driver callbacks. Use Functions
    HAL_WWDT_RegisterCallback() to register a user callback.

    (+) Function HAL_WWDT_RegisterCallback() allows to register following
        callbacks:
        (++) EwiCallback : callback for Early Warning Interrupt.
        (++) MspInitCallback : WWDT MspInit.
    This function takes as parameters the HAL peripheral handle, the Callback ID
    and a pointer to the user callback function.

    (+) Use function HAL_WWDT_UnRegisterCallback() to reset a callback to
    the default weak (surcharged) function. HAL_WWDT_UnRegisterCallback()
    takes as parameters the HAL peripheral handle and the Callback ID.
    This function allows to reset following callbacks:
        (++) EwiCallback : callback for Early Warning Interrupt.
        (++) MspInitCallback : WWDT MspInit.

    [..]
    When calling HAL_WWDT_Init function, callbacks are reset to the
    corresponding legacy weak (surcharged) functions:
    HAL_WWDT_EarlyWarningCallback() and HAL_WWDT_MspInit() only if they have
    not been registered before.

    [..]
    When compilation define USE_HAL_WWDT_REGISTER_CALLBACKS is set to 0 or
    not defined, the callback registering feature is not available
    and weak (surcharged) callbacks are used.

    *** WWDT HAL driver macros list ***
    ===================================
    [..]
      Below the list of available macros in WWDT HAL driver.
      (+) __HAL_WWDT_ENABLE: Enable the WWDT peripheral
      (+) __HAL_WWDT_GET_FLAG: Get the selected WWDT's flag status
      (+) __HAL_WWDT_CLEAR_FLAG: Clear the WWDT's pending flags
      (+) __HAL_WWDT_ENABLE_IT: Enable the WWDT early warning interrupt

  @endverbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

#ifdef HAL_WWDT_MODULE_ENABLED
/** @defgroup WWDT WWDT
  * @brief WWDT HAL module driver.
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_WWDT_Init(WWDT_HandleTypeDef *hwwdt)
{
  /* Check the WWDT handle allocation */
  if (hwwdt == NULL)
  {
    return HAL_ERROR;
  }

 
#if (USE_HAL_WWDT_REGISTER_CALLBACKS == 1)
  /* Reset Callback pointers */
  if (hwwdt->EwiCallback == NULL)
  {
    hwwdt->EwiCallback = HAL_WWDT_EarlyWarningCallback;
  }

  if (hwwdt->MspInitCallback == NULL)
  {
    hwwdt->MspInitCallback = HAL_WWDT_MspInit;
  }

  /* Init the low level hardware */
  hwwdt->MspInitCallback(hwwdt);
#else
  /* Init the low level hardware */
  HAL_WWDT_MspInit(hwwdt);
#endif /* USE_HAL_WWDT_REGISTER_CALLBACKS */

  /* Set WWDT Counter */
  WRITE_REG(hwwdt->Instance->CFG, hwwdt->Init.Counter);

  /* Set WWDT Interrupt Enable */
  WRITE_REG(hwwdt->Instance->IE, hwwdt->Init.IE);

  /* Enable WWDT */
  WRITE_REG(hwwdt->Instance->CTRL, WWDT_ENABLE);

  /* Return function status */
  return HAL_OK;
}


/**
  * @brief  Initialize the WWDT MSP.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @note   When rewriting this function in user file, mechanism may be added
  *         to avoid multiple initialize when HAL_WWDT_Init function is called
  *         again to change parameters.
  * @retval None
  */
__weak void HAL_WWDT_MspInit(WWDT_HandleTypeDef *hwwdt)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hwwdt);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_WWDT_MspInit could be implemented in the user file
   */
}


#if (USE_HAL_WWDT_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User WWDT Callback
  *         To be used instead of the weak (surcharged) predefined callback
  * @param  hwwdt WWDT handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_WWDT_EWI_CB_ID Early Warning Interrupt Callback ID
  *           @arg @ref HAL_WWDT_MSPINIT_CB_ID MspInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval status
  */
HAL_StatusTypeDef HAL_WWDT_RegisterCallback(WWDT_HandleTypeDef *hwwdt, HAL_WWDT_CallbackIDTypeDef CallbackID,
                                            pWWDT_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    status = HAL_ERROR;
  }
  else
  {
    switch (CallbackID)
    {
      case HAL_WWDT_INT_CB_ID:
        hwwdt->EwiCallback = pCallback;
        break;

      case HAL_WWDT_MSPINIT_CB_ID:
        hwwdt->MspInitCallback = pCallback;
        break;

      default:
        status = HAL_ERROR;
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
  *           @arg @ref HAL_WWDT_EWI_CB_ID Early Warning Interrupt Callback ID
  *           @arg @ref HAL_WWDT_MSPINIT_CB_ID MspInit callback ID
  * @retval status
  */
HAL_StatusTypeDef HAL_WWDT_UnRegisterCallback(WWDT_HandleTypeDef *hwwdt, HAL_WWDT_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  switch (CallbackID)
  {
    case HAL_WWDT_INT_CB_ID:
      hwwdt->EwiCallback = HAL_WWDT_EarlyWarningCallback;
      break;

    case HAL_WWDT_MSPINIT_CB_ID:
      hwwdt->MspInitCallback = HAL_WWDT_MspInit;
      break;

    default:
      status = HAL_ERROR;
      break;
  }

  return status;
}
#endif /* USE_HAL_WWDT_REGISTER_CALLBACKS */

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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_WWDT_Refresh(WWDT_HandleTypeDef *hwwdt)
{
  /* Write to WWDT CR the WWDT Counter value to refresh with */
  WRITE_REG(hwwdt->Instance->CTRL, WWDT_COUNTER_CLEAR);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Software Reset the WWDT.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_WWDT_SoftReset(WWDT_HandleTypeDef *hwwdt)
{
  /* Write to WWDT CR to software reset WWDT */
  WRITE_REG(hwwdt->Instance->CTRL, WWDT_SOFT_RESET);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Handle WWDT interrupt request.
  * @note   The Early Warning Interrupt (EWI) can be used if specific safety operations
  *         or data logging must be performed before the actual reset is generated.
  *         The EWI interrupt is enabled by calling HAL_WWDT_Init function with
  *         IE_Mode set to WWDT_IE_ENABLE.
  *         When the upcounter reaches the 75%Ncfg and EWI interrupt is
  *         generated and the corresponding Interrupt Service Routine (ISR) can
  *         be used to trigger specific actions (such as communications or data
  *         logging), before resetting the device.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @retval None
  */
void HAL_WWDT_IRQHandler(WWDT_HandleTypeDef *hwwdt)
{
  /* Check if WWDT Interrupt is enable */
  if (__HAL_WWDT_GET_IT_SOURCE(hwwdt, WWDT_IT_IE) != RESET)
  {
    /* Check if WWDT Interrupt occurred */
    if (__HAL_WWDT_GET_FLAG(hwwdt, WWDT_FLAG_IF) != RESET)
    {
      /* Clear the WWDT Interrupt flag */
      __HAL_WWDT_CLEAR_FLAG(hwwdt, WWDT_FLAG_IF);

#if (USE_HAL_WWDT_REGISTER_CALLBACKS == 1)
      /* Early Warning registered callback */
      hwwdt->EwiCallback(hwwdt);
#else
      /* Early warning callback */
      HAL_WWDT_EarlyWarningCallback(hwwdt);
#endif /* USE_HAL_WWDT_REGISTER_CALLBACKS */
    }
  }
}


/**
  * @brief  WWDT  callback.
  * @param  hwwdt  pointer to a WWDT_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDT module.
  * @retval None
  */
__weak void HAL_WWDT_EarlyWarningCallback(WWDT_HandleTypeDef *hwwdt)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hwwdt);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_WWDT_EarlyWarningCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_WWDT_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
