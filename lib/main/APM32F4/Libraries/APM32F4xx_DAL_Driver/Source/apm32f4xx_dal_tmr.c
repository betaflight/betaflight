/**
  *
  * @file    apm32f4xx_dal_tmr.c
  * @brief   TMR DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Timer (TMR) peripheral:
  *           + TMR Time Base Initialization
  *           + TMR Time Base Start
  *           + TMR Time Base Start Interruption
  *           + TMR Time Base Start DMA
  *           + TMR Output Compare/PWM Initialization
  *           + TMR Output Compare/PWM Channel Configuration
  *           + TMR Output Compare/PWM  Start
  *           + TMR Output Compare/PWM  Start Interruption
  *           + TMR Output Compare/PWM Start DMA
  *           + TMR Input Capture Initialization
  *           + TMR Input Capture Channel Configuration
  *           + TMR Input Capture Start
  *           + TMR Input Capture Start Interruption
  *           + TMR Input Capture Start DMA
  *           + TMR One Pulse Initialization
  *           + TMR One Pulse Channel Configuration
  *           + TMR One Pulse Start
  *           + TMR Encoder Interface Initialization
  *           + TMR Encoder Interface Start
  *           + TMR Encoder Interface Start Interruption
  *           + TMR Encoder Interface Start DMA
  *           + Commutation Event configuration with Interruption and DMA
  *           + TMR OCRef clear configuration
  *           + TMR External Clock configuration
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
                      ##### TMRER Generic features #####
  ==============================================================================
  [..] The Timer features include:
       (#) 16-bit up, down, up/down auto-reload counter.
       (#) 16-bit programmable prescaler allowing dividing (also on the fly) the
           counter clock frequency either by any factor between 1 and 65536.
       (#) Up to 4 independent channels for:
           (++) Input Capture
           (++) Output Compare
           (++) PWM generation (Edge and Center-aligned Mode)
           (++) One-pulse mode output
       (#) Synchronization circuit to control the timer with external signals and to interconnect
            several timers together.
       (#) Supports incremental encoder for positioning purposes

            ##### How to use this driver #####
  ==============================================================================
    [..]
     (#) Initialize the TMR low level resources by implementing the following functions
         depending on the selected feature:
           (++) Time Base : DAL_TMR_Base_MspInit()
           (++) Input Capture : DAL_TMR_IC_MspInit()
           (++) Output Compare : DAL_TMR_OC_MspInit()
           (++) PWM generation : DAL_TMR_PWM_MspInit()
           (++) One-pulse mode output : DAL_TMR_OnePulse_MspInit()
           (++) Encoder mode output : DAL_TMR_Encoder_MspInit()

     (#) Initialize the TMR low level resources :
        (##) Enable the TMR interface clock using __DAL_RCM_TMRx_CLK_ENABLE();
        (##) TMR pins configuration
            (+++) Enable the clock for the TMR GPIOs using the following function:
             __DAL_RCM_GPIOx_CLK_ENABLE();
            (+++) Configure these TMR pins in Alternate function mode using DAL_GPIO_Init();

     (#) The external Clock can be configured, if needed (the default clock is the
         internal clock from the APBx), using the following function:
         DAL_TMR_ConfigClockSource, the clock configuration should be done before
         any start function.

     (#) Configure the TMR in the desired functioning mode using one of the
       Initialization function of this driver:
       (++) DAL_TMR_Base_Init: to use the Timer to generate a simple time base
       (++) DAL_TMR_OC_Init and DAL_TMR_OC_ConfigChannel: to use the Timer to generate an
            Output Compare signal.
       (++) DAL_TMR_PWM_Init and DAL_TMR_PWM_ConfigChannel: to use the Timer to generate a
            PWM signal.
       (++) DAL_TMR_IC_Init and DAL_TMR_IC_ConfigChannel: to use the Timer to measure an
            external signal.
       (++) DAL_TMR_OnePulse_Init and DAL_TMR_OnePulse_ConfigChannel: to use the Timer
            in One Pulse Mode.
       (++) DAL_TMR_Encoder_Init: to use the Timer Encoder Interface.

     (#) Activate the TMR peripheral using one of the start functions depending from the feature used:
           (++) Time Base : DAL_TMR_Base_Start(), DAL_TMR_Base_Start_DMA(), DAL_TMR_Base_Start_IT()
           (++) Input Capture :  DAL_TMR_IC_Start(), DAL_TMR_IC_Start_DMA(), DAL_TMR_IC_Start_IT()
           (++) Output Compare : DAL_TMR_OC_Start(), DAL_TMR_OC_Start_DMA(), DAL_TMR_OC_Start_IT()
           (++) PWM generation : DAL_TMR_PWM_Start(), DAL_TMR_PWM_Start_DMA(), DAL_TMR_PWM_Start_IT()
           (++) One-pulse mode output : DAL_TMR_OnePulse_Start(), DAL_TMR_OnePulse_Start_IT()
           (++) Encoder mode output : DAL_TMR_Encoder_Start(), DAL_TMR_Encoder_Start_DMA(), DAL_TMR_Encoder_Start_IT().

     (#) The DMA Burst is managed with the two following functions:
         DAL_TMR_DMABurst_WriteStart()
         DAL_TMR_DMABurst_ReadStart()

    *** Callback registration ***
  =============================================

  [..]
  The compilation define  USE_DAL_TMR_REGISTER_CALLBACKS when set to 1
  allows the user to configure dynamically the driver callbacks.

  [..]
  Use Function DAL_TMR_RegisterCallback() to register a callback.
  DAL_TMR_RegisterCallback() takes as parameters the DAL peripheral handle,
  the Callback ID and a pointer to the user callback function.

  [..]
  Use function DAL_TMR_UnRegisterCallback() to reset a callback to the default
  weak function.
  DAL_TMR_UnRegisterCallback takes as parameters the DAL peripheral handle,
  and the Callback ID.

  [..]
  These functions allow to register/unregister following callbacks:
    (+) Base_MspInitCallback              : TMR Base Msp Init Callback.
    (+) Base_MspDeInitCallback            : TMR Base Msp DeInit Callback.
    (+) IC_MspInitCallback                : TMR IC Msp Init Callback.
    (+) IC_MspDeInitCallback              : TMR IC Msp DeInit Callback.
    (+) OC_MspInitCallback                : TMR OC Msp Init Callback.
    (+) OC_MspDeInitCallback              : TMR OC Msp DeInit Callback.
    (+) PWM_MspInitCallback               : TMR PWM Msp Init Callback.
    (+) PWM_MspDeInitCallback             : TMR PWM Msp DeInit Callback.
    (+) OnePulse_MspInitCallback          : TMR One Pulse Msp Init Callback.
    (+) OnePulse_MspDeInitCallback        : TMR One Pulse Msp DeInit Callback.
    (+) Encoder_MspInitCallback           : TMR Encoder Msp Init Callback.
    (+) Encoder_MspDeInitCallback         : TMR Encoder Msp DeInit Callback.
    (+) HallSensor_MspInitCallback        : TMR Hall Sensor Msp Init Callback.
    (+) HallSensor_MspDeInitCallback      : TMR Hall Sensor Msp DeInit Callback.
    (+) PeriodElapsedCallback             : TMR Period Elapsed Callback.
    (+) PeriodElapsedHalfCpltCallback     : TMR Period Elapsed half complete Callback.
    (+) TriggerCallback                   : TMR Trigger Callback.
    (+) TriggerHalfCpltCallback           : TMR Trigger half complete Callback.
    (+) IC_CaptureCallback                : TMR Input Capture Callback.
    (+) IC_CaptureHalfCpltCallback        : TMR Input Capture half complete Callback.
    (+) OC_DelayElapsedCallback           : TMR Output Compare Delay Elapsed Callback.
    (+) PWM_PulseFinishedCallback         : TMR PWM Pulse Finished Callback.
    (+) PWM_PulseFinishedHalfCpltCallback : TMR PWM Pulse Finished half complete Callback.
    (+) ErrorCallback                     : TMR Error Callback.
    (+) CommutationCallback               : TMR Commutation Callback.
    (+) CommutationHalfCpltCallback       : TMR Commutation half complete Callback.
    (+) BreakCallback                     : TMR Break Callback.

  [..]
By default, after the Init and when the state is DAL_TMR_STATE_RESET
all interrupt callbacks are set to the corresponding weak functions:
  examples DAL_TMR_TriggerCallback(), DAL_TMR_ErrorCallback().

  [..]
  Exception done for MspInit and MspDeInit functions that are reset to the legacy weak
  functionalities in the Init / DeInit only when these callbacks are null
  (not registered beforehand). If not, MspInit or MspDeInit are not null, the Init / DeInit
    keep and use the user MspInit / MspDeInit callbacks(registered beforehand)

  [..]
    Callbacks can be registered / unregistered in DAL_TMR_STATE_READY state only.
    Exception done MspInit / MspDeInit that can be registered / unregistered
    in DAL_TMR_STATE_READY or DAL_TMR_STATE_RESET state,
    thus registered(user) MspInit / DeInit callbacks can be used during the Init / DeInit.
  In that case first register the MspInit/MspDeInit user callbacks
      using DAL_TMR_RegisterCallback() before calling DeInit or Init function.

  [..]
      When The compilation define USE_DAL_TMR_REGISTER_CALLBACKS is set to 0 or
      not defined, the callback registration feature is not available and all callbacks
      are set to the corresponding weak functions.

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup TMR TMR
  * @brief TMR DAL module driver
  * @{
  */

#ifdef DAL_TMR_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup TMR_Private_Functions
  * @{
  */
static void TMR_OC1_SetConfig(TMR_TypeDef *TMRx, TMR_OC_InitTypeDef *OC_Config);
static void TMR_OC3_SetConfig(TMR_TypeDef *TMRx, TMR_OC_InitTypeDef *OC_Config);
static void TMR_OC4_SetConfig(TMR_TypeDef *TMRx, TMR_OC_InitTypeDef *OC_Config);
static void TMR_TI1_ConfigInputStage(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICFilter);
static void TMR_TI2_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICSelection,
                              uint32_t TMR_ICFilter);
static void TMR_TI2_ConfigInputStage(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICFilter);
static void TMR_TI3_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICSelection,
                              uint32_t TMR_ICFilter);
static void TMR_TI4_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICSelection,
                              uint32_t TMR_ICFilter);
static void TMR_ITRx_SetConfig(TMR_TypeDef *TMRx, uint32_t InputTriggerSource);
static void TMR_DMAPeriodElapsedCplt(DMA_HandleTypeDef *hdma);
static void TMR_DMAPeriodElapsedHalfCplt(DMA_HandleTypeDef *hdma);
static void TMR_DMATriggerCplt(DMA_HandleTypeDef *hdma);
static void TMR_DMATriggerHalfCplt(DMA_HandleTypeDef *hdma);
static DAL_StatusTypeDef TMR_SlaveTimer_SetConfig(TMR_HandleTypeDef *htmr,
                                                  TMR_SlaveConfigTypeDef *sSlaveConfig);
/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/

/** @defgroup TMR_Exported_Functions TMR Exported Functions
  * @{
  */

/** @defgroup TMR_Exported_Functions_Group1 TMR Time Base functions
  *  @brief    Time Base functions
  *
@verbatim
  ==============================================================================
              ##### Time Base functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TMR base.
    (+) De-initialize the TMR base.
    (+) Start the Time Base.
    (+) Stop the Time Base.
    (+) Start the Time Base and enable interrupt.
    (+) Stop the Time Base and disable interrupt.
    (+) Start the Time Base and enable DMA transfer.
    (+) Stop the Time Base and disable DMA transfer.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the TMR Time base Unit according to the specified
  *         parameters in the TMR_HandleTypeDef and initialize the associated handle.
  * @note   Switching from Center Aligned counter mode to Edge counter mode (or reverse)
  *         requires a timer reset to avoid unexpected direction
  *         due to DIR bit readonly in center aligned mode.
  *         Ex: call @ref DAL_TMR_Base_DeInit() before DAL_TMR_Base_Init()
  * @param  htmr TMR Base handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Base_Init(TMR_HandleTypeDef *htmr)
{
  /* Check the TMR handle allocation */
  if (htmr == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_COUNTER_MODE(htmr->Init.CounterMode));
  ASSERT_PARAM(IS_TMR_CLOCKDIVISION_DIV(htmr->Init.ClockDivision));
  ASSERT_PARAM(IS_TMR_AUTORELOAD_PRELOAD(htmr->Init.AutoReloadPreload));

  if (htmr->State == DAL_TMR_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htmr->Lock = DAL_UNLOCKED;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
    /* Reset interrupt callbacks to legacy weak callbacks */
    TMR_ResetCallback(htmr);

    if (htmr->Base_MspInitCallback == NULL)
    {
      htmr->Base_MspInitCallback = DAL_TMR_Base_MspInit;
    }
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    htmr->Base_MspInitCallback(htmr);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    DAL_TMR_Base_MspInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Set the Time Base configuration */
  TMR_Base_SetConfig(htmr->Instance, &htmr->Init);

  /* Initialize the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;

  /* Initialize the TMR channels state */
  TMR_CHANNEL_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_READY);

  /* Initialize the TMR state*/
  htmr->State = DAL_TMR_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the TMR Base peripheral
  * @param  htmr TMR Base handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Base_DeInit(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  htmr->State = DAL_TMR_STATE_BUSY;

  /* Disable the TMR Peripheral Clock */
  __DAL_TMR_DISABLE(htmr);

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  if (htmr->Base_MspDeInitCallback == NULL)
  {
    htmr->Base_MspDeInitCallback = DAL_TMR_Base_MspDeInit;
  }
  /* DeInit the low level hardware */
  htmr->Base_MspDeInitCallback(htmr);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  DAL_TMR_Base_MspDeInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  /* Change the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_RESET;

  /* Change the TMR channels state */
  TMR_CHANNEL_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_RESET);

  /* Change TMR state */
  htmr->State = DAL_TMR_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Initializes the TMR Base MSP.
  * @param  htmr TMR Base handle
  * @retval None
  */
__weak void DAL_TMR_Base_MspInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_Base_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes TMR Base MSP.
  * @param  htmr TMR Base handle
  * @retval None
  */
__weak void DAL_TMR_Base_MspDeInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_Base_MspDeInit could be implemented in the user file
   */
}


/**
  * @brief  Starts the TMR Base generation.
  * @param  htmr TMR Base handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Base_Start(TMR_HandleTypeDef *htmr)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  /* Check the TMR state */
  if (htmr->State != DAL_TMR_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Base generation.
  * @param  htmr TMR Base handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Base_Stop(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_READY;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Base generation in interrupt mode.
  * @param  htmr TMR Base handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Base_Start_IT(TMR_HandleTypeDef *htmr)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  /* Check the TMR state */
  if (htmr->State != DAL_TMR_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Enable the TMR Update interrupt */
  __DAL_TMR_ENABLE_IT(htmr, TMR_IT_UPDATE);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Base generation in interrupt mode.
  * @param  htmr TMR Base handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Base_Stop_IT(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  /* Disable the TMR Update interrupt */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_UPDATE);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_READY;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Base generation in DMA mode.
  * @param  htmr TMR Base handle
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to peripheral.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Base_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t *pData, uint16_t Length)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_DMA_INSTANCE(htmr->Instance));

  /* Set the TMR state */
  if (htmr->State == DAL_TMR_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (htmr->State == DAL_TMR_STATE_READY)
  {
    if ((pData == NULL) && (Length > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      htmr->State = DAL_TMR_STATE_BUSY;
    }
  }
  else
  {
    return DAL_ERROR;
  }

  /* Set the DMA Period elapsed callbacks */
  htmr->hdma[TMR_DMA_ID_UPDATE]->XferCpltCallback = TMR_DMAPeriodElapsedCplt;
  htmr->hdma[TMR_DMA_ID_UPDATE]->XferHalfCpltCallback = TMR_DMAPeriodElapsedHalfCplt;

  /* Set the DMA error callback */
  htmr->hdma[TMR_DMA_ID_UPDATE]->XferErrorCallback = TMR_DMAError ;

  /* Enable the DMA stream */
  if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_UPDATE], (uint32_t)pData, (uint32_t)&htmr->Instance->AUTORLD,
                       Length) != DAL_OK)
  {
    /* Return error status */
    return DAL_ERROR;
  }

  /* Enable the TMR Update DMA request */
  __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_UPDATE);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Base generation in DMA mode.
  * @param  htmr TMR Base handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Base_Stop_DMA(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_DMA_INSTANCE(htmr->Instance));

  /* Disable the TMR Update DMA request */
  __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_UPDATE);

  (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_UPDATE]);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_READY;

  /* Return function status */
  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group2 TMR Output Compare functions
  *  @brief    TMR Output Compare functions
  *
@verbatim
  ==============================================================================
                  ##### TMR Output Compare functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TMR Output Compare.
    (+) De-initialize the TMR Output Compare.
    (+) Start the TMR Output Compare.
    (+) Stop the TMR Output Compare.
    (+) Start the TMR Output Compare and enable interrupt.
    (+) Stop the TMR Output Compare and disable interrupt.
    (+) Start the TMR Output Compare and enable DMA transfer.
    (+) Stop the TMR Output Compare and disable DMA transfer.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the TMR Output Compare according to the specified
  *         parameters in the TMR_HandleTypeDef and initializes the associated handle.
  * @note   Switching from Center Aligned counter mode to Edge counter mode (or reverse)
  *         requires a timer reset to avoid unexpected direction
  *         due to DIR bit readonly in center aligned mode.
  *         Ex: call @ref DAL_TMR_OC_DeInit() before DAL_TMR_OC_Init()
  * @param  htmr TMR Output Compare handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_Init(TMR_HandleTypeDef *htmr)
{
  /* Check the TMR handle allocation */
  if (htmr == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_COUNTER_MODE(htmr->Init.CounterMode));
  ASSERT_PARAM(IS_TMR_CLOCKDIVISION_DIV(htmr->Init.ClockDivision));
  ASSERT_PARAM(IS_TMR_AUTORELOAD_PRELOAD(htmr->Init.AutoReloadPreload));

  if (htmr->State == DAL_TMR_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htmr->Lock = DAL_UNLOCKED;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
    /* Reset interrupt callbacks to legacy weak callbacks */
    TMR_ResetCallback(htmr);

    if (htmr->OC_MspInitCallback == NULL)
    {
      htmr->OC_MspInitCallback = DAL_TMR_OC_MspInit;
    }
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    htmr->OC_MspInitCallback(htmr);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    DAL_TMR_OC_MspInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Init the base time for the Output Compare */
  TMR_Base_SetConfig(htmr->Instance,  &htmr->Init);

  /* Initialize the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;

  /* Initialize the TMR channels state */
  TMR_CHANNEL_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_READY);

  /* Initialize the TMR state*/
  htmr->State = DAL_TMR_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the TMR peripheral
  * @param  htmr TMR Output Compare handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_DeInit(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  htmr->State = DAL_TMR_STATE_BUSY;

  /* Disable the TMR Peripheral Clock */
  __DAL_TMR_DISABLE(htmr);

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  if (htmr->OC_MspDeInitCallback == NULL)
  {
    htmr->OC_MspDeInitCallback = DAL_TMR_OC_MspDeInit;
  }
  /* DeInit the low level hardware */
  htmr->OC_MspDeInitCallback(htmr);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
  DAL_TMR_OC_MspDeInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  /* Change the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_RESET;

  /* Change the TMR channels state */
  TMR_CHANNEL_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_RESET);

  /* Change TMR state */
  htmr->State = DAL_TMR_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Initializes the TMR Output Compare MSP.
  * @param  htmr TMR Output Compare handle
  * @retval None
  */
__weak void DAL_TMR_OC_MspInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_OC_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes TMR Output Compare MSP.
  * @param  htmr TMR Output Compare handle
  * @retval None
  */
__weak void DAL_TMR_OC_MspDeInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_OC_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  Starts the TMR Output Compare signal generation.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_Start(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR channel state */
  if (TMR_CHANNEL_STATE_GET(htmr, Channel) != DAL_TMR_CHANNEL_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the Output compare channel */
  TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

  if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
  {
    /* Enable the main output */
    __DAL_TMR_MOE_ENABLE(htmr);
  }

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Output Compare signal generation.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Disable the Output compare channel */
  TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

  if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
  {
    /* Disable the Main Output */
    __DAL_TMR_MOE_DISABLE(htmr);
  }

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Output Compare signal generation in interrupt mode.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR channel state */
  if (TMR_CHANNEL_STATE_GET(htmr, Channel) != DAL_TMR_CHANNEL_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Enable the TMR Capture/Compare 1 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Enable the TMR Capture/Compare 2 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Enable the TMR Capture/Compare 3 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Enable the TMR Capture/Compare 4 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the Output compare channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

    if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
    {
      /* Enable the main output */
      __DAL_TMR_MOE_ENABLE(htmr);
    }

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR Output Compare signal generation in interrupt mode.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Capture/Compare 1 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Capture/Compare 2 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Capture/Compare 3 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Disable the TMR Capture/Compare 4 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the Output compare channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

    if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
    {
      /* Disable the Main Output */
      __DAL_TMR_MOE_DISABLE(htmr);
    }

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR channel state */
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Starts the TMR Output Compare signal generation in DMA mode.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to TMR peripheral
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Set the TMR channel state */
  if (TMR_CHANNEL_STATE_GET(htmr, Channel) == DAL_TMR_CHANNEL_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (TMR_CHANNEL_STATE_GET(htmr, Channel) == DAL_TMR_CHANNEL_STATE_READY)
  {
    if ((pData == NULL) && (Length > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else
  {
    return DAL_ERROR;
  }

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htmr->Instance->CC1,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }

      /* Enable the TMR Capture/Compare 1 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htmr->Instance->CC2,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }

      /* Enable the TMR Capture/Compare 2 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC3]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC3]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC3]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htmr->Instance->CC3,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 3 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC4]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC4]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC4]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC4], (uint32_t)pData, (uint32_t)&htmr->Instance->CC4,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 4 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the Output compare channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

    if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
    {
      /* Enable the main output */
      __DAL_TMR_MOE_ENABLE(htmr);
    }

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR Output Compare signal generation in DMA mode.
  * @param  htmr TMR Output Compare handle
  * @param  Channel TMR Channel to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Capture/Compare 1 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC1);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Capture/Compare 2 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC2);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Capture/Compare 3 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC3);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC3]);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Disable the TMR Capture/Compare 4 interrupt */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC4);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC4]);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the Output compare channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

    if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
    {
      /* Disable the Main Output */
      __DAL_TMR_MOE_DISABLE(htmr);
    }

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR channel state */
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group3 TMR PWM functions
  *  @brief    TMR PWM functions
  *
@verbatim
  ==============================================================================
                          ##### TMR PWM functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TMR PWM.
    (+) De-initialize the TMR PWM.
    (+) Start the TMR PWM.
    (+) Stop the TMR PWM.
    (+) Start the TMR PWM and enable interrupt.
    (+) Stop the TMR PWM and disable interrupt.
    (+) Start the TMR PWM and enable DMA transfer.
    (+) Stop the TMR PWM and disable DMA transfer.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the TMR PWM Time Base according to the specified
  *         parameters in the TMR_HandleTypeDef and initializes the associated handle.
  * @note   Switching from Center Aligned counter mode to Edge counter mode (or reverse)
  *         requires a timer reset to avoid unexpected direction
  *         due to DIR bit readonly in center aligned mode.
  *         Ex: call @ref DAL_TMR_PWM_DeInit() before DAL_TMR_PWM_Init()
  * @param  htmr TMR PWM handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_Init(TMR_HandleTypeDef *htmr)
{
  /* Check the TMR handle allocation */
  if (htmr == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_COUNTER_MODE(htmr->Init.CounterMode));
  ASSERT_PARAM(IS_TMR_CLOCKDIVISION_DIV(htmr->Init.ClockDivision));
  ASSERT_PARAM(IS_TMR_AUTORELOAD_PRELOAD(htmr->Init.AutoReloadPreload));

  if (htmr->State == DAL_TMR_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htmr->Lock = DAL_UNLOCKED;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
    /* Reset interrupt callbacks to legacy weak callbacks */
    TMR_ResetCallback(htmr);

    if (htmr->PWM_MspInitCallback == NULL)
    {
      htmr->PWM_MspInitCallback = DAL_TMR_PWM_MspInit;
    }
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    htmr->PWM_MspInitCallback(htmr);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    DAL_TMR_PWM_MspInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Init the base time for the PWM */
  TMR_Base_SetConfig(htmr->Instance, &htmr->Init);

  /* Initialize the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;

  /* Initialize the TMR channels state */
  TMR_CHANNEL_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_READY);

  /* Initialize the TMR state*/
  htmr->State = DAL_TMR_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the TMR peripheral
  * @param  htmr TMR PWM handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_DeInit(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  htmr->State = DAL_TMR_STATE_BUSY;

  /* Disable the TMR Peripheral Clock */
  __DAL_TMR_DISABLE(htmr);

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  if (htmr->PWM_MspDeInitCallback == NULL)
  {
    htmr->PWM_MspDeInitCallback = DAL_TMR_PWM_MspDeInit;
  }
  /* DeInit the low level hardware */
  htmr->PWM_MspDeInitCallback(htmr);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
  DAL_TMR_PWM_MspDeInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  /* Change the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_RESET;

  /* Change the TMR channels state */
  TMR_CHANNEL_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_RESET);

  /* Change TMR state */
  htmr->State = DAL_TMR_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Initializes the TMR PWM MSP.
  * @param  htmr TMR PWM handle
  * @retval None
  */
__weak void DAL_TMR_PWM_MspInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_PWM_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes TMR PWM MSP.
  * @param  htmr TMR PWM handle
  * @retval None
  */
__weak void DAL_TMR_PWM_MspDeInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_PWM_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  Starts the PWM signal generation.
  * @param  htmr TMR handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_Start(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR channel state */
  if (TMR_CHANNEL_STATE_GET(htmr, Channel) != DAL_TMR_CHANNEL_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the Capture compare channel */
  TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

  if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
  {
    /* Enable the main output */
    __DAL_TMR_MOE_ENABLE(htmr);
  }

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the PWM signal generation.
  * @param  htmr TMR PWM handle
  * @param  Channel TMR Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Disable the Capture compare channel */
  TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

  if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
  {
    /* Disable the Main Output */
    __DAL_TMR_MOE_DISABLE(htmr);
  }

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the PWM signal generation in interrupt mode.
  * @param  htmr TMR PWM handle
  * @param  Channel TMR Channel to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR channel state */
  if (TMR_CHANNEL_STATE_GET(htmr, Channel) != DAL_TMR_CHANNEL_STATE_READY)
  {
    return DAL_ERROR;
  }

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Enable the TMR Capture/Compare 1 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Enable the TMR Capture/Compare 2 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Enable the TMR Capture/Compare 3 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Enable the TMR Capture/Compare 4 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the Capture compare channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

    if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
    {
      /* Enable the main output */
      __DAL_TMR_MOE_ENABLE(htmr);
    }

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the PWM signal generation in interrupt mode.
  * @param  htmr TMR PWM handle
  * @param  Channel TMR Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Capture/Compare 1 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Capture/Compare 2 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Capture/Compare 3 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Disable the TMR Capture/Compare 4 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the Capture compare channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

    if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
    {
      /* Disable the Main Output */
      __DAL_TMR_MOE_DISABLE(htmr);
    }

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR channel state */
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Starts the TMR PWM signal generation in DMA mode.
  * @param  htmr TMR PWM handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @param  pData The source Buffer address.
  * @param  Length The length of data to be transferred from memory to TMR peripheral
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Set the TMR channel state */
  if (TMR_CHANNEL_STATE_GET(htmr, Channel) == DAL_TMR_CHANNEL_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (TMR_CHANNEL_STATE_GET(htmr, Channel) == DAL_TMR_CHANNEL_STATE_READY)
  {
    if ((pData == NULL) && (Length > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else
  {
    return DAL_ERROR;
  }

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htmr->Instance->CC1,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }

      /* Enable the TMR Capture/Compare 1 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htmr->Instance->CC2,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 2 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC3]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC3]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC3]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htmr->Instance->CC3,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Output Capture/Compare 3 request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC4]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC4]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC4]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC4], (uint32_t)pData, (uint32_t)&htmr->Instance->CC4,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 4 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the Capture compare channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

    if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
    {
      /* Enable the main output */
      __DAL_TMR_MOE_ENABLE(htmr);
    }

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR PWM signal generation in DMA mode.
  * @param  htmr TMR PWM handle
  * @param  Channel TMR Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Capture/Compare 1 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC1);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Capture/Compare 2 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC2);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Capture/Compare 3 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC3);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC3]);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Disable the TMR Capture/Compare 4 interrupt */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC4);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC4]);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the Capture compare channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

    if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
    {
      /* Disable the Main Output */
      __DAL_TMR_MOE_DISABLE(htmr);
    }

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR channel state */
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group4 TMR Input Capture functions
  *  @brief    TMR Input Capture functions
  *
@verbatim
  ==============================================================================
              ##### TMR Input Capture functions #####
  ==============================================================================
 [..]
   This section provides functions allowing to:
   (+) Initialize and configure the TMR Input Capture.
   (+) De-initialize the TMR Input Capture.
   (+) Start the TMR Input Capture.
   (+) Stop the TMR Input Capture.
   (+) Start the TMR Input Capture and enable interrupt.
   (+) Stop the TMR Input Capture and disable interrupt.
   (+) Start the TMR Input Capture and enable DMA transfer.
   (+) Stop the TMR Input Capture and disable DMA transfer.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the TMR Input Capture Time base according to the specified
  *         parameters in the TMR_HandleTypeDef and initializes the associated handle.
  * @note   Switching from Center Aligned counter mode to Edge counter mode (or reverse)
  *         requires a timer reset to avoid unexpected direction
  *         due to DIR bit readonly in center aligned mode.
  *         Ex: call @ref DAL_TMR_IC_DeInit() before DAL_TMR_IC_Init()
  * @param  htmr TMR Input Capture handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_Init(TMR_HandleTypeDef *htmr)
{
  /* Check the TMR handle allocation */
  if (htmr == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_COUNTER_MODE(htmr->Init.CounterMode));
  ASSERT_PARAM(IS_TMR_CLOCKDIVISION_DIV(htmr->Init.ClockDivision));
  ASSERT_PARAM(IS_TMR_AUTORELOAD_PRELOAD(htmr->Init.AutoReloadPreload));

  if (htmr->State == DAL_TMR_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htmr->Lock = DAL_UNLOCKED;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
    /* Reset interrupt callbacks to legacy weak callbacks */
    TMR_ResetCallback(htmr);

    if (htmr->IC_MspInitCallback == NULL)
    {
      htmr->IC_MspInitCallback = DAL_TMR_IC_MspInit;
    }
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    htmr->IC_MspInitCallback(htmr);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    DAL_TMR_IC_MspInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Init the base time for the input capture */
  TMR_Base_SetConfig(htmr->Instance, &htmr->Init);

  /* Initialize the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;

  /* Initialize the TMR channels state */
  TMR_CHANNEL_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_READY);

  /* Initialize the TMR state*/
  htmr->State = DAL_TMR_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the TMR peripheral
  * @param  htmr TMR Input Capture handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_DeInit(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  htmr->State = DAL_TMR_STATE_BUSY;

  /* Disable the TMR Peripheral Clock */
  __DAL_TMR_DISABLE(htmr);

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  if (htmr->IC_MspDeInitCallback == NULL)
  {
    htmr->IC_MspDeInitCallback = DAL_TMR_IC_MspDeInit;
  }
  /* DeInit the low level hardware */
  htmr->IC_MspDeInitCallback(htmr);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
  DAL_TMR_IC_MspDeInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  /* Change the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_RESET;

  /* Change the TMR channels state */
  TMR_CHANNEL_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET_ALL(htmr, DAL_TMR_CHANNEL_STATE_RESET);

  /* Change TMR state */
  htmr->State = DAL_TMR_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Initializes the TMR Input Capture MSP.
  * @param  htmr TMR Input Capture handle
  * @retval None
  */
__weak void DAL_TMR_IC_MspInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_IC_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes TMR Input Capture MSP.
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMR_IC_MspDeInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_IC_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  Starts the TMR Input Capture measurement.
  * @param  htmr TMR Input Capture handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_Start(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  uint32_t tmpsmcr;
  DAL_TMR_ChannelStateTypeDef channel_state = TMR_CHANNEL_STATE_GET(htmr, Channel);
  DAL_TMR_ChannelStateTypeDef complementary_channel_state = TMR_CHANNEL_N_STATE_GET(htmr, Channel);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR channel state */
  if ((channel_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_state != DAL_TMR_CHANNEL_STATE_READY))
  {
    return DAL_ERROR;
  }

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the Input Capture channel */
  TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Input Capture measurement.
  * @param  htmr TMR Input Capture handle
  * @param  Channel TMR Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Disable the Input Capture channel */
  TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Input Capture measurement in interrupt mode.
  * @param  htmr TMR Input Capture handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  DAL_TMR_ChannelStateTypeDef channel_state = TMR_CHANNEL_STATE_GET(htmr, Channel);
  DAL_TMR_ChannelStateTypeDef complementary_channel_state = TMR_CHANNEL_N_STATE_GET(htmr, Channel);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  /* Check the TMR channel state */
  if ((channel_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_state != DAL_TMR_CHANNEL_STATE_READY))
  {
    return DAL_ERROR;
  }

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Enable the TMR Capture/Compare 1 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Enable the TMR Capture/Compare 2 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Enable the TMR Capture/Compare 3 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Enable the TMR Capture/Compare 4 interrupt */
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Enable the Input Capture channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

    /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
    if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
    {
      tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
      if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
      {
        __DAL_TMR_ENABLE(htmr);
      }
    }
    else
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR Input Capture measurement in interrupt mode.
  * @param  htmr TMR Input Capture handle
  * @param  Channel TMR Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Capture/Compare 1 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Capture/Compare 2 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Capture/Compare 3 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Disable the TMR Capture/Compare 4 interrupt */
      __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the Input Capture channel */
    TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR channel state */
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Starts the TMR Input Capture measurement in DMA mode.
  * @param  htmr TMR Input Capture handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @param  pData The destination Buffer address.
  * @param  Length The length of data to be transferred from TMR peripheral to memory.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  DAL_TMR_ChannelStateTypeDef channel_state = TMR_CHANNEL_STATE_GET(htmr, Channel);
  DAL_TMR_ChannelStateTypeDef complementary_channel_state = TMR_CHANNEL_N_STATE_GET(htmr, Channel);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));
  ASSERT_PARAM(IS_TMR_DMA_CC_INSTANCE(htmr->Instance));

  /* Set the TMR channel state */
  if ((channel_state == DAL_TMR_CHANNEL_STATE_BUSY)
      || (complementary_channel_state == DAL_TMR_CHANNEL_STATE_BUSY))
  {
    return DAL_BUSY;
  }
  else if ((channel_state == DAL_TMR_CHANNEL_STATE_READY)
           && (complementary_channel_state == DAL_TMR_CHANNEL_STATE_READY))
  {
    if ((pData == NULL) && (Length > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else
  {
    return DAL_ERROR;
  }

  /* Enable the Input Capture channel */
  TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_ENABLE);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)&htmr->Instance->CC1, (uint32_t)pData,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 1 DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)&htmr->Instance->CC2, (uint32_t)pData,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 2  DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC2);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC3]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC3]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC3]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC3], (uint32_t)&htmr->Instance->CC3, (uint32_t)pData,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 3  DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC3);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC4]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC4]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC4]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC4], (uint32_t)&htmr->Instance->CC4, (uint32_t)pData,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Capture/Compare 4  DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC4);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TMR_SLAVE_INSTANCE(htmr->Instance))
  {
    tmpsmcr = htmr->Instance->SMCTRL & TMR_SMCTRL_SMFSEL;
    if (!IS_TMR_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __DAL_TMR_ENABLE(htmr);
    }
  }
  else
  {
    __DAL_TMR_ENABLE(htmr);
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR Input Capture measurement in DMA mode.
  * @param  htmr TMR Input Capture handle
  * @param  Channel TMR Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));
  ASSERT_PARAM(IS_TMR_DMA_CC_INSTANCE(htmr->Instance));

  /* Disable the Input Capture channel */
  TMR_CCxChannelCmd(htmr->Instance, Channel, TMR_CCx_DISABLE);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Disable the TMR Capture/Compare 1 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC1);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Disable the TMR Capture/Compare 2 DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC2);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Disable the TMR Capture/Compare 3  DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC3);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC3]);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Disable the TMR Capture/Compare 4  DMA request */
      __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC4);
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC4]);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the Peripheral */
    __DAL_TMR_DISABLE(htmr);

    /* Set the TMR channel state */
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return status;
}
/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group5 TMR One Pulse functions
  *  @brief    TMR One Pulse functions
  *
@verbatim
  ==============================================================================
                        ##### TMR One Pulse functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TMR One Pulse.
    (+) De-initialize the TMR One Pulse.
    (+) Start the TMR One Pulse.
    (+) Stop the TMR One Pulse.
    (+) Start the TMR One Pulse and enable interrupt.
    (+) Stop the TMR One Pulse and disable interrupt.
    (+) Start the TMR One Pulse and enable DMA transfer.
    (+) Stop the TMR One Pulse and disable DMA transfer.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the TMR One Pulse Time Base according to the specified
  *         parameters in the TMR_HandleTypeDef and initializes the associated handle.
  * @note   Switching from Center Aligned counter mode to Edge counter mode (or reverse)
  *         requires a timer reset to avoid unexpected direction
  *         due to DIR bit readonly in center aligned mode.
  *         Ex: call @ref DAL_TMR_OnePulse_DeInit() before DAL_TMR_OnePulse_Init()
  * @note   When the timer instance is initialized in One Pulse mode, timer
  *         channels 1 and channel 2 are reserved and cannot be used for other
  *         purpose.
  * @param  htmr TMR One Pulse handle
  * @param  OnePulseMode Select the One pulse mode.
  *         This parameter can be one of the following values:
  *            @arg TMR_OPMODE_SINGLE: Only one pulse will be generated.
  *            @arg TMR_OPMODE_REPETITIVE: Repetitive pulses will be generated.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OnePulse_Init(TMR_HandleTypeDef *htmr, uint32_t OnePulseMode)
{
  /* Check the TMR handle allocation */
  if (htmr == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_COUNTER_MODE(htmr->Init.CounterMode));
  ASSERT_PARAM(IS_TMR_CLOCKDIVISION_DIV(htmr->Init.ClockDivision));
  ASSERT_PARAM(IS_TMR_OPM_MODE(OnePulseMode));
  ASSERT_PARAM(IS_TMR_AUTORELOAD_PRELOAD(htmr->Init.AutoReloadPreload));

  if (htmr->State == DAL_TMR_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htmr->Lock = DAL_UNLOCKED;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
    /* Reset interrupt callbacks to legacy weak callbacks */
    TMR_ResetCallback(htmr);

    if (htmr->OnePulse_MspInitCallback == NULL)
    {
      htmr->OnePulse_MspInitCallback = DAL_TMR_OnePulse_MspInit;
    }
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    htmr->OnePulse_MspInitCallback(htmr);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    DAL_TMR_OnePulse_MspInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Configure the Time base in the One Pulse Mode */
  TMR_Base_SetConfig(htmr->Instance, &htmr->Init);

  /* Reset the OPM Bit */
  htmr->Instance->CTRL1 &= ~TMR_CTRL1_SPMEN;

  /* Configure the OPM Mode */
  htmr->Instance->CTRL1 |= OnePulseMode;

  /* Initialize the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;

  /* Initialize the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Initialize the TMR state*/
  htmr->State = DAL_TMR_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the TMR One Pulse
  * @param  htmr TMR One Pulse handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OnePulse_DeInit(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  htmr->State = DAL_TMR_STATE_BUSY;

  /* Disable the TMR Peripheral Clock */
  __DAL_TMR_DISABLE(htmr);

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  if (htmr->OnePulse_MspDeInitCallback == NULL)
  {
    htmr->OnePulse_MspDeInitCallback = DAL_TMR_OnePulse_MspDeInit;
  }
  /* DeInit the low level hardware */
  htmr->OnePulse_MspDeInitCallback(htmr);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  DAL_TMR_OnePulse_MspDeInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  /* Change the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_RESET;

  /* Set the TMR channel state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_RESET);

  /* Change TMR state */
  htmr->State = DAL_TMR_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Initializes the TMR One Pulse MSP.
  * @param  htmr TMR One Pulse handle
  * @retval None
  */
__weak void DAL_TMR_OnePulse_MspInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_OnePulse_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes TMR One Pulse MSP.
  * @param  htmr TMR One Pulse handle
  * @retval None
  */
__weak void DAL_TMR_OnePulse_MspDeInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_OnePulse_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  Starts the TMR One Pulse signal generation.
  * @note Though OutputChannel parameter is deprecated and ignored by the function
  *        it has been kept to avoid DAL_TMR API compatibility break.
  * @note The pulse output channel is determined when calling
  *       @ref DAL_TMR_OnePulse_ConfigChannel().
  * @param  htmr TMR One Pulse handle
  * @param  OutputChannel See note above
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OnePulse_Start(TMR_HandleTypeDef *htmr, uint32_t OutputChannel)
{
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Prevent unused argument(s) compilation warning */
  UNUSED(OutputChannel);

  /* Check the TMR channels state */
  if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
  {
    return DAL_ERROR;
  }

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the Capture compare and the Input Capture channels
    (in the OPM Mode the two possible channels that can be used are TMR_CHANNEL_1 and TMR_CHANNEL_2)
    if TMR_CHANNEL_1 is used as output, the TMR_CHANNEL_2 will be used as input and
    if TMR_CHANNEL_1 is used as input, the TMR_CHANNEL_2 will be used as output
    whatever the combination, the TMR_CHANNEL_1 and TMR_CHANNEL_2 should be enabled together

    No need to enable the counter, it's enabled automatically by hardware
    (the counter starts in response to a stimulus and generate a pulse */

  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_ENABLE);

  if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
  {
    /* Enable the main output */
    __DAL_TMR_MOE_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR One Pulse signal generation.
  * @note Though OutputChannel parameter is deprecated and ignored by the function
  *        it has been kept to avoid DAL_TMR API compatibility break.
  * @note The pulse output channel is determined when calling
  *       @ref DAL_TMR_OnePulse_ConfigChannel().
  * @param  htmr TMR One Pulse handle
  * @param  OutputChannel See note above
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OnePulse_Stop(TMR_HandleTypeDef *htmr, uint32_t OutputChannel)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(OutputChannel);

  /* Disable the Capture compare and the Input Capture channels
  (in the OPM Mode the two possible channels that can be used are TMR_CHANNEL_1 and TMR_CHANNEL_2)
  if TMR_CHANNEL_1 is used as output, the TMR_CHANNEL_2 will be used as input and
  if TMR_CHANNEL_1 is used as input, the TMR_CHANNEL_2 will be used as output
  whatever the combination, the TMR_CHANNEL_1 and TMR_CHANNEL_2 should be disabled together */

  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_DISABLE);

  if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
  {
    /* Disable the Main Output */
    __DAL_TMR_MOE_DISABLE(htmr);
  }

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR One Pulse signal generation in interrupt mode.
  * @note Though OutputChannel parameter is deprecated and ignored by the function
  *        it has been kept to avoid DAL_TMR API compatibility break.
  * @note The pulse output channel is determined when calling
  *       @ref DAL_TMR_OnePulse_ConfigChannel().
  * @param  htmr TMR One Pulse handle
  * @param  OutputChannel See note above
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OnePulse_Start_IT(TMR_HandleTypeDef *htmr, uint32_t OutputChannel)
{
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Prevent unused argument(s) compilation warning */
  UNUSED(OutputChannel);

  /* Check the TMR channels state */
  if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
      || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
  {
    return DAL_ERROR;
  }

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);

  /* Enable the Capture compare and the Input Capture channels
    (in the OPM Mode the two possible channels that can be used are TMR_CHANNEL_1 and TMR_CHANNEL_2)
    if TMR_CHANNEL_1 is used as output, the TMR_CHANNEL_2 will be used as input and
    if TMR_CHANNEL_1 is used as input, the TMR_CHANNEL_2 will be used as output
    whatever the combination, the TMR_CHANNEL_1 and TMR_CHANNEL_2 should be enabled together

    No need to enable the counter, it's enabled automatically by hardware
    (the counter starts in response to a stimulus and generate a pulse */

  /* Enable the TMR Capture/Compare 1 interrupt */
  __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);

  /* Enable the TMR Capture/Compare 2 interrupt */
  __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);

  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_ENABLE);

  if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
  {
    /* Enable the main output */
    __DAL_TMR_MOE_ENABLE(htmr);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR One Pulse signal generation in interrupt mode.
  * @note Though OutputChannel parameter is deprecated and ignored by the function
  *        it has been kept to avoid DAL_TMR API compatibility break.
  * @note The pulse output channel is determined when calling
  *       @ref DAL_TMR_OnePulse_ConfigChannel().
  * @param  htmr TMR One Pulse handle
  * @param  OutputChannel See note above
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OnePulse_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t OutputChannel)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(OutputChannel);

  /* Disable the TMR Capture/Compare 1 interrupt */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);

  /* Disable the TMR Capture/Compare 2 interrupt */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);

  /* Disable the Capture compare and the Input Capture channels
  (in the OPM Mode the two possible channels that can be used are TMR_CHANNEL_1 and TMR_CHANNEL_2)
  if TMR_CHANNEL_1 is used as output, the TMR_CHANNEL_2 will be used as input and
  if TMR_CHANNEL_1 is used as input, the TMR_CHANNEL_2 will be used as output
  whatever the combination, the TMR_CHANNEL_1 and TMR_CHANNEL_2 should be disabled together */
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);
  TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_DISABLE);

  if (IS_TMR_BREAK_INSTANCE(htmr->Instance) != RESET)
  {
    /* Disable the Main Output */
    __DAL_TMR_MOE_DISABLE(htmr);
  }

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Return function status */
  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group6 TMR Encoder functions
  *  @brief    TMR Encoder functions
  *
@verbatim
  ==============================================================================
                          ##### TMR Encoder functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to:
    (+) Initialize and configure the TMR Encoder.
    (+) De-initialize the TMR Encoder.
    (+) Start the TMR Encoder.
    (+) Stop the TMR Encoder.
    (+) Start the TMR Encoder and enable interrupt.
    (+) Stop the TMR Encoder and disable interrupt.
    (+) Start the TMR Encoder and enable DMA transfer.
    (+) Stop the TMR Encoder and disable DMA transfer.

@endverbatim
  * @{
  */
/**
  * @brief  Initializes the TMR Encoder Interface and initialize the associated handle.
  * @note   Switching from Center Aligned counter mode to Edge counter mode (or reverse)
  *         requires a timer reset to avoid unexpected direction
  *         due to DIR bit readonly in center aligned mode.
  *         Ex: call @ref DAL_TMR_Encoder_DeInit() before DAL_TMR_Encoder_Init()
  * @note   Encoder mode and External clock mode 2 are not compatible and must not be selected together
  *         Ex: A call for @ref DAL_TMR_Encoder_Init will erase the settings of @ref DAL_TMR_ConfigClockSource
  *         using TMR_CLOCKSOURCE_ETRMODE2 and vice versa
  * @note   When the timer instance is initialized in Encoder mode, timer
  *         channels 1 and channel 2 are reserved and cannot be used for other
  *         purpose.
  * @param  htmr TMR Encoder Interface handle
  * @param  sConfig TMR Encoder Interface configuration structure
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Encoder_Init(TMR_HandleTypeDef *htmr,  TMR_Encoder_InitTypeDef *sConfig)
{
  uint32_t tmpsmcr;
  uint32_t tmpccmr1;
  uint32_t tmpccer;

  /* Check the TMR handle allocation */
  if (htmr == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_ENCODER_INTERFACE_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_COUNTER_MODE(htmr->Init.CounterMode));
  ASSERT_PARAM(IS_TMR_CLOCKDIVISION_DIV(htmr->Init.ClockDivision));
  ASSERT_PARAM(IS_TMR_AUTORELOAD_PRELOAD(htmr->Init.AutoReloadPreload));
  ASSERT_PARAM(IS_TMR_ENCODER_MODE(sConfig->EncoderMode));
  ASSERT_PARAM(IS_TMR_IC_SELECTION(sConfig->IC1Selection));
  ASSERT_PARAM(IS_TMR_IC_SELECTION(sConfig->IC2Selection));
  ASSERT_PARAM(IS_TMR_ENCODERINPUT_POLARITY(sConfig->IC1Polarity));
  ASSERT_PARAM(IS_TMR_ENCODERINPUT_POLARITY(sConfig->IC2Polarity));
  ASSERT_PARAM(IS_TMR_IC_PRESCALER(sConfig->IC1Prescaler));
  ASSERT_PARAM(IS_TMR_IC_PRESCALER(sConfig->IC2Prescaler));
  ASSERT_PARAM(IS_TMR_IC_FILTER(sConfig->IC1Filter));
  ASSERT_PARAM(IS_TMR_IC_FILTER(sConfig->IC2Filter));

  if (htmr->State == DAL_TMR_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htmr->Lock = DAL_UNLOCKED;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
    /* Reset interrupt callbacks to legacy weak callbacks */
    TMR_ResetCallback(htmr);

    if (htmr->Encoder_MspInitCallback == NULL)
    {
      htmr->Encoder_MspInitCallback = DAL_TMR_Encoder_MspInit;
    }
    /* Init the low level hardware : GPIO, CLOCK, NVIC */
    htmr->Encoder_MspInitCallback(htmr);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    DAL_TMR_Encoder_MspInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
  }

  /* Set the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Reset the SMFSEL and ECEN bits */
  htmr->Instance->SMCTRL &= ~(TMR_SMCTRL_SMFSEL | TMR_SMCTRL_ECEN);

  /* Configure the Time base in the Encoder Mode */
  TMR_Base_SetConfig(htmr->Instance, &htmr->Init);

  /* Get the TMRx SMCTRL register value */
  tmpsmcr = htmr->Instance->SMCTRL;

  /* Get the TMRx CCM1 register value */
  tmpccmr1 = htmr->Instance->CCM1;

  /* Get the TMRx CCEN register value */
  tmpccer = htmr->Instance->CCEN;

  /* Set the encoder Mode */
  tmpsmcr |= sConfig->EncoderMode;

  /* Select the Capture Compare 1 and the Capture Compare 2 as input */
  tmpccmr1 &= ~(TMR_CCM1_CC1SEL | TMR_CCM1_CC2SEL);
  tmpccmr1 |= (sConfig->IC1Selection | (sConfig->IC2Selection << 8U));

  /* Set the Capture Compare 1 and the Capture Compare 2 prescalers and filters */
  tmpccmr1 &= ~(TMR_CCM1_IC1PSC | TMR_CCM1_IC2PSC);
  tmpccmr1 &= ~(TMR_CCM1_IC1F | TMR_CCM1_IC2F);
  tmpccmr1 |= sConfig->IC1Prescaler | (sConfig->IC2Prescaler << 8U);
  tmpccmr1 |= (sConfig->IC1Filter << 4U) | (sConfig->IC2Filter << 12U);

  /* Set the TI1 and the TI2 Polarities */
  tmpccer &= ~(TMR_CCEN_CC1POL | TMR_CCEN_CC2POL);
  tmpccer &= ~(TMR_CCEN_CC1NPOL | TMR_CCEN_CC2NPOL);
  tmpccer |= sConfig->IC1Polarity | (sConfig->IC2Polarity << 4U);

  /* Write to TMRx SMCTRL */
  htmr->Instance->SMCTRL = tmpsmcr;

  /* Write to TMRx CCM1 */
  htmr->Instance->CCM1 = tmpccmr1;

  /* Write to TMRx CCEN */
  htmr->Instance->CCEN = tmpccer;

  /* Initialize the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);

  /* Initialize the TMR state*/
  htmr->State = DAL_TMR_STATE_READY;

  return DAL_OK;
}


/**
  * @brief  DeInitializes the TMR Encoder interface
  * @param  htmr TMR Encoder Interface handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Encoder_DeInit(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));

  htmr->State = DAL_TMR_STATE_BUSY;

  /* Disable the TMR Peripheral Clock */
  __DAL_TMR_DISABLE(htmr);

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  if (htmr->Encoder_MspDeInitCallback == NULL)
  {
    htmr->Encoder_MspDeInitCallback = DAL_TMR_Encoder_MspDeInit;
  }
  /* DeInit the low level hardware */
  htmr->Encoder_MspDeInitCallback(htmr);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  DAL_TMR_Encoder_MspDeInit(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  /* Change the DMA burst operation state */
  htmr->DMABurstState = DAL_DMA_BURST_STATE_RESET;

  /* Set the TMR channels state */
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_RESET);
  TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_RESET);

  /* Change TMR state */
  htmr->State = DAL_TMR_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Initializes the TMR Encoder Interface MSP.
  * @param  htmr TMR Encoder Interface handle
  * @retval None
  */
__weak void DAL_TMR_Encoder_MspInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_Encoder_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes TMR Encoder Interface MSP.
  * @param  htmr TMR Encoder Interface handle
  * @retval None
  */
__weak void DAL_TMR_Encoder_MspDeInit(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_Encoder_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  Starts the TMR Encoder Interface.
  * @param  htmr TMR Encoder Interface handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_ALL: TMR Channel 1 and TMR Channel 2 are selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Encoder_Start(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_ENCODER_INTERFACE_INSTANCE(htmr->Instance));

  /* Set the TMR channel(s) state */
  if (Channel == TMR_CHANNEL_1)
  {
    if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
        || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else if (Channel == TMR_CHANNEL_2)
  {
    if ((channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
        || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else
  {
    if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
        || (channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
        || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
        || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }

  /* Enable the encoder interface channels */
  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);
      break;
    }

    case TMR_CHANNEL_2:
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_ENABLE);
      break;
    }

    default :
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_ENABLE);
      break;
    }
  }
  /* Enable the Peripheral */
  __DAL_TMR_ENABLE(htmr);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Encoder Interface.
  * @param  htmr TMR Encoder Interface handle
  * @param  Channel TMR Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_ALL: TMR Channel 1 and TMR Channel 2 are selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Encoder_Stop(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_ENCODER_INTERFACE_INSTANCE(htmr->Instance));

  /* Disable the Input Capture channels 1 and 2
    (in the EncoderInterface the two possible channels that can be used are TMR_CHANNEL_1 and TMR_CHANNEL_2) */
  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);
      break;
    }

    case TMR_CHANNEL_2:
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_DISABLE);
      break;
    }

    default :
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_DISABLE);
      break;
    }
  }

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channel(s) state */
  if ((Channel == TMR_CHANNEL_1) || (Channel == TMR_CHANNEL_2))
  {
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }
  else
  {
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Encoder Interface in interrupt mode.
  * @param  htmr TMR Encoder Interface handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_ALL: TMR Channel 1 and TMR Channel 2 are selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Encoder_Start_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_ENCODER_INTERFACE_INSTANCE(htmr->Instance));

  /* Set the TMR channel(s) state */
  if (Channel == TMR_CHANNEL_1)
  {
    if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
        || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else if (Channel == TMR_CHANNEL_2)
  {
    if ((channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
        || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }
  else
  {
    if ((channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
        || (channel_2_state != DAL_TMR_CHANNEL_STATE_READY)
        || (complementary_channel_1_state != DAL_TMR_CHANNEL_STATE_READY)
        || (complementary_channel_2_state != DAL_TMR_CHANNEL_STATE_READY))
    {
      return DAL_ERROR;
    }
    else
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
    }
  }

  /* Enable the encoder interface channels */
  /* Enable the capture compare Interrupts 1 and/or 2 */
  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);
      break;
    }

    case TMR_CHANNEL_2:
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_ENABLE);
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);
      break;
    }

    default :
    {
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_ENABLE);
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC1);
      __DAL_TMR_ENABLE_IT(htmr, TMR_IT_CC2);
      break;
    }
  }

  /* Enable the Peripheral */
  __DAL_TMR_ENABLE(htmr);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Encoder Interface in interrupt mode.
  * @param  htmr TMR Encoder Interface handle
  * @param  Channel TMR Channels to be disabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_ALL: TMR Channel 1 and TMR Channel 2 are selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Encoder_Stop_IT(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_ENCODER_INTERFACE_INSTANCE(htmr->Instance));

  /* Disable the Input Capture channels 1 and 2
    (in the EncoderInterface the two possible channels that can be used are TMR_CHANNEL_1 and TMR_CHANNEL_2) */
  if (Channel == TMR_CHANNEL_1)
  {
    TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);

    /* Disable the capture compare Interrupts 1 */
    __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);
  }
  else if (Channel == TMR_CHANNEL_2)
  {
    TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_DISABLE);

    /* Disable the capture compare Interrupts 2 */
    __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);
  }
  else
  {
    TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);
    TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_DISABLE);

    /* Disable the capture compare Interrupts 1 and 2 */
    __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC1);
    __DAL_TMR_DISABLE_IT(htmr, TMR_IT_CC2);
  }

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channel(s) state */
  if ((Channel == TMR_CHANNEL_1) || (Channel == TMR_CHANNEL_2))
  {
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }
  else
  {
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Starts the TMR Encoder Interface in DMA mode.
  * @param  htmr TMR Encoder Interface handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_ALL: TMR Channel 1 and TMR Channel 2 are selected
  * @param  pData1 The destination Buffer address for IC1.
  * @param  pData2 The destination Buffer address for IC2.
  * @param  Length The length of data to be transferred from TMR peripheral to memory.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Encoder_Start_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel, uint32_t *pData1,
                                            uint32_t *pData2, uint16_t Length)
{
  DAL_TMR_ChannelStateTypeDef channel_1_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef channel_2_state = TMR_CHANNEL_STATE_GET(htmr, TMR_CHANNEL_2);
  DAL_TMR_ChannelStateTypeDef complementary_channel_1_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_1);
  DAL_TMR_ChannelStateTypeDef complementary_channel_2_state = TMR_CHANNEL_N_STATE_GET(htmr, TMR_CHANNEL_2);

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_ENCODER_INTERFACE_INSTANCE(htmr->Instance));

  /* Set the TMR channel(s) state */
  if (Channel == TMR_CHANNEL_1)
  {
    if ((channel_1_state == DAL_TMR_CHANNEL_STATE_BUSY)
        || (complementary_channel_1_state == DAL_TMR_CHANNEL_STATE_BUSY))
    {
      return DAL_BUSY;
    }
    else if ((channel_1_state == DAL_TMR_CHANNEL_STATE_READY)
             && (complementary_channel_1_state == DAL_TMR_CHANNEL_STATE_READY))
    {
      if ((pData1 == NULL) && (Length > 0U))
      {
        return DAL_ERROR;
      }
      else
      {
        TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
        TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
      }
    }
    else
    {
      return DAL_ERROR;
    }
  }
  else if (Channel == TMR_CHANNEL_2)
  {
    if ((channel_2_state == DAL_TMR_CHANNEL_STATE_BUSY)
        || (complementary_channel_2_state == DAL_TMR_CHANNEL_STATE_BUSY))
    {
      return DAL_BUSY;
    }
    else if ((channel_2_state == DAL_TMR_CHANNEL_STATE_READY)
             && (complementary_channel_2_state == DAL_TMR_CHANNEL_STATE_READY))
    {
      if ((pData2 == NULL) && (Length > 0U))
      {
        return DAL_ERROR;
      }
      else
      {
        TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
        TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
      }
    }
    else
    {
      return DAL_ERROR;
    }
  }
  else
  {
    if ((channel_1_state == DAL_TMR_CHANNEL_STATE_BUSY)
        || (channel_2_state == DAL_TMR_CHANNEL_STATE_BUSY)
        || (complementary_channel_1_state == DAL_TMR_CHANNEL_STATE_BUSY)
        || (complementary_channel_2_state == DAL_TMR_CHANNEL_STATE_BUSY))
    {
      return DAL_BUSY;
    }
    else if ((channel_1_state == DAL_TMR_CHANNEL_STATE_READY)
             && (channel_2_state == DAL_TMR_CHANNEL_STATE_READY)
             && (complementary_channel_1_state == DAL_TMR_CHANNEL_STATE_READY)
             && (complementary_channel_2_state == DAL_TMR_CHANNEL_STATE_READY))
    {
      if ((((pData1 == NULL) || (pData2 == NULL))) && (Length > 0U))
      {
        return DAL_ERROR;
      }
      else
      {
        TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
        TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
        TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_BUSY);
        TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_BUSY);
      }
    }
    else
    {
      return DAL_ERROR;
    }
  }

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)&htmr->Instance->CC1, (uint32_t)pData1,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Input Capture DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC1);

      /* Enable the Capture compare channel */
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);

      /* Enable the Peripheral */
      __DAL_TMR_ENABLE(htmr);

      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAError;
      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)&htmr->Instance->CC2, (uint32_t)pData2,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      /* Enable the TMR Input Capture  DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC2);

      /* Enable the Capture compare channel */
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_ENABLE);

      /* Enable the Peripheral */
      __DAL_TMR_ENABLE(htmr);

      break;
    }

    default:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)&htmr->Instance->CC1, (uint32_t)pData1,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }

      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)&htmr->Instance->CC2, (uint32_t)pData2,
                           Length) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }

      /* Enable the TMR Input Capture  DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC1);
      /* Enable the TMR Input Capture  DMA request */
      __DAL_TMR_ENABLE_DMA(htmr, TMR_DMA_CC2);

      /* Enable the Capture compare channel */
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_ENABLE);
      TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_ENABLE);

      /* Enable the Peripheral */
      __DAL_TMR_ENABLE(htmr);

      break;
    }
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Stops the TMR Encoder Interface in DMA mode.
  * @param  htmr TMR Encoder Interface handle
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_ALL: TMR Channel 1 and TMR Channel 2 are selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_Encoder_Stop_DMA(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_ENCODER_INTERFACE_INSTANCE(htmr->Instance));

  /* Disable the Input Capture channels 1 and 2
    (in the EncoderInterface the two possible channels that can be used are TMR_CHANNEL_1 and TMR_CHANNEL_2) */
  if (Channel == TMR_CHANNEL_1)
  {
    TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);

    /* Disable the capture compare DMA Request 1 */
    __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC1);
    (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
  }
  else if (Channel == TMR_CHANNEL_2)
  {
    TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_DISABLE);

    /* Disable the capture compare DMA Request 2 */
    __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC2);
    (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
  }
  else
  {
    TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_1, TMR_CCx_DISABLE);
    TMR_CCxChannelCmd(htmr->Instance, TMR_CHANNEL_2, TMR_CCx_DISABLE);

    /* Disable the capture compare DMA Request 1 and 2 */
    __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC1);
    __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_CC2);
    (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
    (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
  }

  /* Disable the Peripheral */
  __DAL_TMR_DISABLE(htmr);

  /* Set the TMR channel(s) state */
  if ((Channel == TMR_CHANNEL_1) || (Channel == TMR_CHANNEL_2))
  {
    TMR_CHANNEL_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, Channel, DAL_TMR_CHANNEL_STATE_READY);
  }
  else
  {
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  }

  /* Return function status */
  return DAL_OK;
}

/**
  * @}
  */
/** @defgroup TMR_Exported_Functions_Group7 TMR IRQ handler management
  *  @brief    TMR IRQ handler management
  *
@verbatim
  ==============================================================================
                        ##### IRQ handler management #####
  ==============================================================================
  [..]
    This section provides Timer IRQ handler function.

@endverbatim
  * @{
  */
/**
  * @brief  This function handles TMR interrupts requests.
  * @param  htmr TMR  handle
  * @retval None
  */
void DAL_TMR_IRQHandler(TMR_HandleTypeDef *htmr)
{
  /* Capture compare 1 event */
  if (__DAL_TMR_GET_FLAG(htmr, TMR_FLAG_CC1) != RESET)
  {
    if (__DAL_TMR_GET_IT_SOURCE(htmr, TMR_IT_CC1) != RESET)
    {
      {
        __DAL_TMR_CLEAR_IT(htmr, TMR_IT_CC1);
        htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_1;

        /* Input capture event */
        if ((htmr->Instance->CCM1 & TMR_CCM1_CC1SEL) != 0x00U)
        {
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
          htmr->IC_CaptureCallback(htmr);
#else
          DAL_TMR_IC_CaptureCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
        }
        /* Output compare event */
        else
        {
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
          htmr->OC_DelayElapsedCallback(htmr);
          htmr->PWM_PulseFinishedCallback(htmr);
#else
          DAL_TMR_OC_DelayElapsedCallback(htmr);
          DAL_TMR_PWM_PulseFinishedCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
        }
        htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
      }
    }
  }
  /* Capture compare 2 event */
  if (__DAL_TMR_GET_FLAG(htmr, TMR_FLAG_CC2) != RESET)
  {
    if (__DAL_TMR_GET_IT_SOURCE(htmr, TMR_IT_CC2) != RESET)
    {
      __DAL_TMR_CLEAR_IT(htmr, TMR_IT_CC2);
      htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_2;
      /* Input capture event */
      if ((htmr->Instance->CCM1 & TMR_CCM1_CC2SEL) != 0x00U)
      {
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
        htmr->IC_CaptureCallback(htmr);
#else
        DAL_TMR_IC_CaptureCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
      }
      /* Output compare event */
      else
      {
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
        htmr->OC_DelayElapsedCallback(htmr);
        htmr->PWM_PulseFinishedCallback(htmr);
#else
        DAL_TMR_OC_DelayElapsedCallback(htmr);
        DAL_TMR_PWM_PulseFinishedCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
      }
      htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
    }
  }
  /* Capture compare 3 event */
  if (__DAL_TMR_GET_FLAG(htmr, TMR_FLAG_CC3) != RESET)
  {
    if (__DAL_TMR_GET_IT_SOURCE(htmr, TMR_IT_CC3) != RESET)
    {
      __DAL_TMR_CLEAR_IT(htmr, TMR_IT_CC3);
      htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_3;
      /* Input capture event */
      if ((htmr->Instance->CCM2 & TMR_CCM2_CC3SEL) != 0x00U)
      {
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
        htmr->IC_CaptureCallback(htmr);
#else
        DAL_TMR_IC_CaptureCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
      }
      /* Output compare event */
      else
      {
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
        htmr->OC_DelayElapsedCallback(htmr);
        htmr->PWM_PulseFinishedCallback(htmr);
#else
        DAL_TMR_OC_DelayElapsedCallback(htmr);
        DAL_TMR_PWM_PulseFinishedCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
      }
      htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
    }
  }
  /* Capture compare 4 event */
  if (__DAL_TMR_GET_FLAG(htmr, TMR_FLAG_CC4) != RESET)
  {
    if (__DAL_TMR_GET_IT_SOURCE(htmr, TMR_IT_CC4) != RESET)
    {
      __DAL_TMR_CLEAR_IT(htmr, TMR_IT_CC4);
      htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_4;
      /* Input capture event */
      if ((htmr->Instance->CCM2 & TMR_CCM2_CC4SEL) != 0x00U)
      {
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
        htmr->IC_CaptureCallback(htmr);
#else
        DAL_TMR_IC_CaptureCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
      }
      /* Output compare event */
      else
      {
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
        htmr->OC_DelayElapsedCallback(htmr);
        htmr->PWM_PulseFinishedCallback(htmr);
#else
        DAL_TMR_OC_DelayElapsedCallback(htmr);
        DAL_TMR_PWM_PulseFinishedCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
      }
      htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
    }
  }
  /* TMR Update event */
  if (__DAL_TMR_GET_FLAG(htmr, TMR_FLAG_UPDATE) != RESET)
  {
    if (__DAL_TMR_GET_IT_SOURCE(htmr, TMR_IT_UPDATE) != RESET)
    {
      __DAL_TMR_CLEAR_IT(htmr, TMR_IT_UPDATE);
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
      htmr->PeriodElapsedCallback(htmr);
#else
      DAL_TMR_PeriodElapsedCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
    }
  }
  /* TMR Break input event */
  if (__DAL_TMR_GET_FLAG(htmr, TMR_FLAG_BREAK) != RESET)
  {
    if (__DAL_TMR_GET_IT_SOURCE(htmr, TMR_IT_BREAK) != RESET)
    {
      __DAL_TMR_CLEAR_IT(htmr, TMR_IT_BREAK);
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
      htmr->BreakCallback(htmr);
#else
      DAL_TMREx_BreakCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
    }
  }
  /* TMR Trigger detection event */
  if (__DAL_TMR_GET_FLAG(htmr, TMR_FLAG_TRIGGER) != RESET)
  {
    if (__DAL_TMR_GET_IT_SOURCE(htmr, TMR_IT_TRIGGER) != RESET)
    {
      __DAL_TMR_CLEAR_IT(htmr, TMR_IT_TRIGGER);
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
      htmr->TriggerCallback(htmr);
#else
      DAL_TMR_TriggerCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
    }
  }
  /* TMR commutation event */
  if (__DAL_TMR_GET_FLAG(htmr, TMR_FLAG_COM) != RESET)
  {
    if (__DAL_TMR_GET_IT_SOURCE(htmr, TMR_IT_COM) != RESET)
    {
      __DAL_TMR_CLEAR_IT(htmr, TMR_FLAG_COM);
#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
      htmr->CommutationCallback(htmr);
#else
      DAL_TMREx_CommutCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group8 TMR Peripheral Control functions
  *  @brief    TMR Peripheral Control functions
  *
@verbatim
  ==============================================================================
                   ##### Peripheral Control functions #####
  ==============================================================================
 [..]
   This section provides functions allowing to:
      (+) Configure The Input Output channels for OC, PWM, IC or One Pulse mode.
      (+) Configure External Clock source.
      (+) Configure Complementary channels, break features and dead time.
      (+) Configure Master and the Slave synchronization.
      (+) Configure the DMA Burst Mode.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the TMR Output Compare Channels according to the specified
  *         parameters in the TMR_OC_InitTypeDef.
  * @param  htmr TMR Output Compare handle
  * @param  sConfig TMR Output Compare configuration structure
  * @param  Channel TMR Channels to configure
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OC_ConfigChannel(TMR_HandleTypeDef *htmr,
                                           TMR_OC_InitTypeDef *sConfig,
                                           uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CHANNELS(Channel));
  ASSERT_PARAM(IS_TMR_OC_MODE(sConfig->OCMode));
  ASSERT_PARAM(IS_TMR_OC_POLARITY(sConfig->OCPolarity));

  /* Process Locked */
  __DAL_LOCK(htmr);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC1_INSTANCE(htmr->Instance));

      /* Configure the TMR Channel 1 in Output Compare */
      TMR_OC1_SetConfig(htmr->Instance, sConfig);
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC2_INSTANCE(htmr->Instance));

      /* Configure the TMR Channel 2 in Output Compare */
      TMR_OC2_SetConfig(htmr->Instance, sConfig);
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC3_INSTANCE(htmr->Instance));

      /* Configure the TMR Channel 3 in Output Compare */
      TMR_OC3_SetConfig(htmr->Instance, sConfig);
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC4_INSTANCE(htmr->Instance));

      /* Configure the TMR Channel 4 in Output Compare */
      TMR_OC4_SetConfig(htmr->Instance, sConfig);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  __DAL_UNLOCK(htmr);

  return status;
}

/**
  * @brief  Initializes the TMR Input Capture Channels according to the specified
  *         parameters in the TMR_IC_InitTypeDef.
  * @param  htmr TMR IC handle
  * @param  sConfig TMR Input Capture configuration structure
  * @param  Channel TMR Channel to configure
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_IC_ConfigChannel(TMR_HandleTypeDef *htmr, TMR_IC_InitTypeDef *sConfig, uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC1_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_IC_POLARITY(sConfig->ICPolarity));
  ASSERT_PARAM(IS_TMR_IC_SELECTION(sConfig->ICSelection));
  ASSERT_PARAM(IS_TMR_IC_PRESCALER(sConfig->ICPrescaler));
  ASSERT_PARAM(IS_TMR_IC_FILTER(sConfig->ICFilter));

  /* Process Locked */
  __DAL_LOCK(htmr);

  if (Channel == TMR_CHANNEL_1)
  {
    /* TI1 Configuration */
    TMR_TI1_SetConfig(htmr->Instance,
                      sConfig->ICPolarity,
                      sConfig->ICSelection,
                      sConfig->ICFilter);

    /* Reset the IC1PSC Bits */
    htmr->Instance->CCM1 &= ~TMR_CCM1_IC1PSC;

    /* Set the IC1PSC value */
    htmr->Instance->CCM1 |= sConfig->ICPrescaler;
  }
  else if (Channel == TMR_CHANNEL_2)
  {
    /* TI2 Configuration */
    ASSERT_PARAM(IS_TMR_CC2_INSTANCE(htmr->Instance));

    TMR_TI2_SetConfig(htmr->Instance,
                      sConfig->ICPolarity,
                      sConfig->ICSelection,
                      sConfig->ICFilter);

    /* Reset the IC2PSC Bits */
    htmr->Instance->CCM1 &= ~TMR_CCM1_IC2PSC;

    /* Set the IC2PSC value */
    htmr->Instance->CCM1 |= (sConfig->ICPrescaler << 8U);
  }
  else if (Channel == TMR_CHANNEL_3)
  {
    /* TI3 Configuration */
    ASSERT_PARAM(IS_TMR_CC3_INSTANCE(htmr->Instance));

    TMR_TI3_SetConfig(htmr->Instance,
                      sConfig->ICPolarity,
                      sConfig->ICSelection,
                      sConfig->ICFilter);

    /* Reset the IC3PSC Bits */
    htmr->Instance->CCM2 &= ~TMR_CCM2_IC3PSC;

    /* Set the IC3PSC value */
    htmr->Instance->CCM2 |= sConfig->ICPrescaler;
  }
  else if (Channel == TMR_CHANNEL_4)
  {
    /* TI4 Configuration */
    ASSERT_PARAM(IS_TMR_CC4_INSTANCE(htmr->Instance));

    TMR_TI4_SetConfig(htmr->Instance,
                      sConfig->ICPolarity,
                      sConfig->ICSelection,
                      sConfig->ICFilter);

    /* Reset the IC4PSC Bits */
    htmr->Instance->CCM2 &= ~TMR_CCM2_IC4PSC;

    /* Set the IC4PSC value */
    htmr->Instance->CCM2 |= (sConfig->ICPrescaler << 8U);
  }
  else
  {
    status = DAL_ERROR;
  }

  __DAL_UNLOCK(htmr);

  return status;
}

/**
  * @brief  Initializes the TMR PWM  channels according to the specified
  *         parameters in the TMR_OC_InitTypeDef.
  * @param  htmr TMR PWM handle
  * @param  sConfig TMR PWM configuration structure
  * @param  Channel TMR Channels to be configured
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_PWM_ConfigChannel(TMR_HandleTypeDef *htmr,
                                            TMR_OC_InitTypeDef *sConfig,
                                            uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CHANNELS(Channel));
  ASSERT_PARAM(IS_TMR_PWM_MODE(sConfig->OCMode));
  ASSERT_PARAM(IS_TMR_OC_POLARITY(sConfig->OCPolarity));
  ASSERT_PARAM(IS_TMR_FAST_STATE(sConfig->OCFastMode));

  /* Process Locked */
  __DAL_LOCK(htmr);

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC1_INSTANCE(htmr->Instance));

      /* Configure the Channel 1 in PWM mode */
      TMR_OC1_SetConfig(htmr->Instance, sConfig);

      /* Set the Preload enable bit for channel1 */
      htmr->Instance->CCM1 |= TMR_CCM1_OC1PEN;

      /* Configure the Output Fast mode */
      htmr->Instance->CCM1 &= ~TMR_CCM1_OC1FEN;
      htmr->Instance->CCM1 |= sConfig->OCFastMode;
      break;
    }

    case TMR_CHANNEL_2:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC2_INSTANCE(htmr->Instance));

      /* Configure the Channel 2 in PWM mode */
      TMR_OC2_SetConfig(htmr->Instance, sConfig);

      /* Set the Preload enable bit for channel2 */
      htmr->Instance->CCM1 |= TMR_CCM1_OC2PEN;

      /* Configure the Output Fast mode */
      htmr->Instance->CCM1 &= ~TMR_CCM1_OC2FEN;
      htmr->Instance->CCM1 |= sConfig->OCFastMode << 8U;
      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC3_INSTANCE(htmr->Instance));

      /* Configure the Channel 3 in PWM mode */
      TMR_OC3_SetConfig(htmr->Instance, sConfig);

      /* Set the Preload enable bit for channel3 */
      htmr->Instance->CCM2 |= TMR_CCM2_OC3PEN;

      /* Configure the Output Fast mode */
      htmr->Instance->CCM2 &= ~TMR_CCM2_OC3FEN;
      htmr->Instance->CCM2 |= sConfig->OCFastMode;
      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC4_INSTANCE(htmr->Instance));

      /* Configure the Channel 4 in PWM mode */
      TMR_OC4_SetConfig(htmr->Instance, sConfig);

      /* Set the Preload enable bit for channel4 */
      htmr->Instance->CCM2 |= TMR_CCM2_OC4PEN;

      /* Configure the Output Fast mode */
      htmr->Instance->CCM2 &= ~TMR_CCM2_OC4FEN;
      htmr->Instance->CCM2 |= sConfig->OCFastMode << 8U;
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  __DAL_UNLOCK(htmr);

  return status;
}

/**
  * @brief  Initializes the TMR One Pulse Channels according to the specified
  *         parameters in the TMR_OnePulse_InitTypeDef.
  * @param  htmr TMR One Pulse handle
  * @param  sConfig TMR One Pulse configuration structure
  * @param  OutputChannel TMR output channel to configure
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  * @param  InputChannel TMR input Channel to configure
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  * @note  To output a waveform with a minimum delay user can enable the fast
  *        mode by calling the @ref __DAL_TMR_ENABLE_OCxFAST macro. Then CCx
  *        output is forced in response to the edge detection on TIx input,
  *        without taking in account the comparison.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_OnePulse_ConfigChannel(TMR_HandleTypeDef *htmr,  TMR_OnePulse_InitTypeDef *sConfig,
                                                 uint32_t OutputChannel,  uint32_t InputChannel)
{
  DAL_StatusTypeDef status = DAL_OK;
  TMR_OC_InitTypeDef temp1;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_OPM_CHANNELS(OutputChannel));
  ASSERT_PARAM(IS_TMR_OPM_CHANNELS(InputChannel));

  if (OutputChannel != InputChannel)
  {
    /* Process Locked */
    __DAL_LOCK(htmr);

    htmr->State = DAL_TMR_STATE_BUSY;

    /* Extract the Output compare configuration from sConfig structure */
    temp1.OCMode = sConfig->OCMode;
    temp1.Pulse = sConfig->Pulse;
    temp1.OCPolarity = sConfig->OCPolarity;
    temp1.OCNPolarity = sConfig->OCNPolarity;
    temp1.OCIdleState = sConfig->OCIdleState;
    temp1.OCNIdleState = sConfig->OCNIdleState;

    switch (OutputChannel)
    {
      case TMR_CHANNEL_1:
      {
        ASSERT_PARAM(IS_TMR_CC1_INSTANCE(htmr->Instance));

        TMR_OC1_SetConfig(htmr->Instance, &temp1);
        break;
      }

      case TMR_CHANNEL_2:
      {
        ASSERT_PARAM(IS_TMR_CC2_INSTANCE(htmr->Instance));

        TMR_OC2_SetConfig(htmr->Instance, &temp1);
        break;
      }

      default:
        status = DAL_ERROR;
        break;
    }

    if (status == DAL_OK)
    {
      switch (InputChannel)
      {
        case TMR_CHANNEL_1:
        {
          ASSERT_PARAM(IS_TMR_CC1_INSTANCE(htmr->Instance));

          TMR_TI1_SetConfig(htmr->Instance, sConfig->ICPolarity,
                            sConfig->ICSelection, sConfig->ICFilter);

          /* Reset the IC1PSC Bits */
          htmr->Instance->CCM1 &= ~TMR_CCM1_IC1PSC;

          /* Select the Trigger source */
          htmr->Instance->SMCTRL &= ~TMR_SMCTRL_TRGSEL;
          htmr->Instance->SMCTRL |= TMR_TS_TI1FP1;

          /* Select the Slave Mode */
          htmr->Instance->SMCTRL &= ~TMR_SMCTRL_SMFSEL;
          htmr->Instance->SMCTRL |= TMR_SLAVEMODE_TRIGGER;
          break;
        }

        case TMR_CHANNEL_2:
        {
          ASSERT_PARAM(IS_TMR_CC2_INSTANCE(htmr->Instance));

          TMR_TI2_SetConfig(htmr->Instance, sConfig->ICPolarity,
                            sConfig->ICSelection, sConfig->ICFilter);

          /* Reset the IC2PSC Bits */
          htmr->Instance->CCM1 &= ~TMR_CCM1_IC2PSC;

          /* Select the Trigger source */
          htmr->Instance->SMCTRL &= ~TMR_SMCTRL_TRGSEL;
          htmr->Instance->SMCTRL |= TMR_TS_TI2FP2;

          /* Select the Slave Mode */
          htmr->Instance->SMCTRL &= ~TMR_SMCTRL_SMFSEL;
          htmr->Instance->SMCTRL |= TMR_SLAVEMODE_TRIGGER;
          break;
        }

        default:
          status = DAL_ERROR;
          break;
      }
    }

    htmr->State = DAL_TMR_STATE_READY;

    __DAL_UNLOCK(htmr);

    return status;
  }
  else
  {
    return DAL_ERROR;
  }
}

/**
  * @brief  Configure the DMA Burst to transfer Data from the memory to the TMR peripheral
  * @param  htmr TMR handle
  * @param  BurstBaseAddress TMR Base address from where the DMA  will start the Data write
  *         This parameter can be one of the following values:
  *            @arg TMR_DMABASE_CTRL1
  *            @arg TMR_DMABASE_CTRL2
  *            @arg TMR_DMABASE_SMCTRL
  *            @arg TMR_DMABASE_DIEN
  *            @arg TMR_DMABASE_STS
  *            @arg TMR_DMABASE_CEG
  *            @arg TMR_DMABASE_CCM1
  *            @arg TMR_DMABASE_CCM2
  *            @arg TMR_DMABASE_CCEN
  *            @arg TMR_DMABASE_CNT
  *            @arg TMR_DMABASE_PSC
  *            @arg TMR_DMABASE_AUTORLD
  *            @arg TMR_DMABASE_REPCNT
  *            @arg TMR_DMABASE_CC1
  *            @arg TMR_DMABASE_CC2
  *            @arg TMR_DMABASE_CC3
  *            @arg TMR_DMABASE_CC4
  *            @arg TMR_DMABASE_BDT
  * @param  BurstRequestSrc TMR DMA Request sources
  *         This parameter can be one of the following values:
  *            @arg TMR_DMA_UPDATE: TMR update Interrupt source
  *            @arg TMR_DMA_CC1: TMR Capture Compare 1 DMA source
  *            @arg TMR_DMA_CC2: TMR Capture Compare 2 DMA source
  *            @arg TMR_DMA_CC3: TMR Capture Compare 3 DMA source
  *            @arg TMR_DMA_CC4: TMR Capture Compare 4 DMA source
  *            @arg TMR_DMA_COM: TMR Commutation DMA source
  *            @arg TMR_DMA_TRIGGER: TMR Trigger DMA source
  * @param  BurstBuffer The Buffer address.
  * @param  BurstLength DMA Burst length. This parameter can be one value
  *         between: TMR_DMABURSTLENGTH_1TRANSFER and TMR_DMABURSTLENGTH_18TRANSFERS.
  * @note   This function should be used only when BurstLength is equal to DMA data transfer length.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_DMABurst_WriteStart(TMR_HandleTypeDef *htmr, uint32_t BurstBaseAddress,
                                              uint32_t BurstRequestSrc, uint32_t *BurstBuffer, uint32_t  BurstLength)
{
  DAL_StatusTypeDef status;

  status = DAL_TMR_DMABurst_MultiWriteStart(htmr, BurstBaseAddress, BurstRequestSrc, BurstBuffer, BurstLength,
                                            ((BurstLength) >> 8U) + 1U);



  return status;
}

/**
  * @brief  Configure the DMA Burst to transfer multiple Data from the memory to the TMR peripheral
  * @param  htmr TMR handle
  * @param  BurstBaseAddress TMR Base address from where the DMA will start the Data write
  *         This parameter can be one of the following values:
  *            @arg TMR_DMABASE_CTRL1
  *            @arg TMR_DMABASE_CTRL2
  *            @arg TMR_DMABASE_SMCTRL
  *            @arg TMR_DMABASE_DIEN
  *            @arg TMR_DMABASE_STS
  *            @arg TMR_DMABASE_CEG
  *            @arg TMR_DMABASE_CCM1
  *            @arg TMR_DMABASE_CCM2
  *            @arg TMR_DMABASE_CCEN
  *            @arg TMR_DMABASE_CNT
  *            @arg TMR_DMABASE_PSC
  *            @arg TMR_DMABASE_AUTORLD
  *            @arg TMR_DMABASE_REPCNT
  *            @arg TMR_DMABASE_CC1
  *            @arg TMR_DMABASE_CC2
  *            @arg TMR_DMABASE_CC3
  *            @arg TMR_DMABASE_CC4
  *            @arg TMR_DMABASE_BDT
  * @param  BurstRequestSrc TMR DMA Request sources
  *         This parameter can be one of the following values:
  *            @arg TMR_DMA_UPDATE: TMR update Interrupt source
  *            @arg TMR_DMA_CC1: TMR Capture Compare 1 DMA source
  *            @arg TMR_DMA_CC2: TMR Capture Compare 2 DMA source
  *            @arg TMR_DMA_CC3: TMR Capture Compare 3 DMA source
  *            @arg TMR_DMA_CC4: TMR Capture Compare 4 DMA source
  *            @arg TMR_DMA_COM: TMR Commutation DMA source
  *            @arg TMR_DMA_TRIGGER: TMR Trigger DMA source
  * @param  BurstBuffer The Buffer address.
  * @param  BurstLength DMA Burst length. This parameter can be one value
  *         between: TMR_DMABURSTLENGTH_1TRANSFER and TMR_DMABURSTLENGTH_18TRANSFERS.
  * @param  DataLength Data length. This parameter can be one value
  *         between 1 and 0xFFFF.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_DMABurst_MultiWriteStart(TMR_HandleTypeDef *htmr, uint32_t BurstBaseAddress,
                                                   uint32_t BurstRequestSrc, uint32_t *BurstBuffer,
                                                   uint32_t  BurstLength,  uint32_t  DataLength)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_DMABURST_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_DMA_BASE(BurstBaseAddress));
  ASSERT_PARAM(IS_TMR_DMA_SOURCE(BurstRequestSrc));
  ASSERT_PARAM(IS_TMR_DMA_LENGTH(BurstLength));
  ASSERT_PARAM(IS_TMR_DMA_DATA_LENGTH(DataLength));

  if (htmr->DMABurstState == DAL_DMA_BURST_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (htmr->DMABurstState == DAL_DMA_BURST_STATE_READY)
  {
    if ((BurstBuffer == NULL) && (BurstLength > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      htmr->DMABurstState = DAL_DMA_BURST_STATE_BUSY;
    }
  }
  else
  {
    /* nothing to do */
  }

  switch (BurstRequestSrc)
  {
    case TMR_DMA_UPDATE:
    {
      /* Set the DMA Period elapsed callbacks */
      htmr->hdma[TMR_DMA_ID_UPDATE]->XferCpltCallback = TMR_DMAPeriodElapsedCplt;
      htmr->hdma[TMR_DMA_ID_UPDATE]->XferHalfCpltCallback = TMR_DMAPeriodElapsedHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_UPDATE]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_UPDATE], (uint32_t)BurstBuffer,
                           (uint32_t)&htmr->Instance->DMAR, DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_CC1:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)BurstBuffer,
                           (uint32_t)&htmr->Instance->DMAR, DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_CC2:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)BurstBuffer,
                           (uint32_t)&htmr->Instance->DMAR, DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_CC3:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC3]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC3]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC3]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC3], (uint32_t)BurstBuffer,
                           (uint32_t)&htmr->Instance->DMAR, DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_CC4:
    {
      /* Set the DMA compare callbacks */
      htmr->hdma[TMR_DMA_ID_CC4]->XferCpltCallback = TMR_DMADelayPulseCplt;
      htmr->hdma[TMR_DMA_ID_CC4]->XferHalfCpltCallback = TMR_DMADelayPulseHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC4]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC4], (uint32_t)BurstBuffer,
                           (uint32_t)&htmr->Instance->DMAR, DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_COM:
    {
      /* Set the DMA commutation callbacks */
      htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferCpltCallback =  TMREx_DMACommutationCplt;
      htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferHalfCpltCallback =  TMREx_DMACommutationHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_COMMUTATION], (uint32_t)BurstBuffer,
                           (uint32_t)&htmr->Instance->DMAR, DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_TRIGGER:
    {
      /* Set the DMA trigger callbacks */
      htmr->hdma[TMR_DMA_ID_TRIGGER]->XferCpltCallback = TMR_DMATriggerCplt;
      htmr->hdma[TMR_DMA_ID_TRIGGER]->XferHalfCpltCallback = TMR_DMATriggerHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_TRIGGER]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_TRIGGER], (uint32_t)BurstBuffer,
                           (uint32_t)&htmr->Instance->DMAR, DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Configure the DMA Burst Mode */
    htmr->Instance->DCTRL = (BurstBaseAddress | BurstLength);
    /* Enable the TMR DMA Request */
    __DAL_TMR_ENABLE_DMA(htmr, BurstRequestSrc);
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stops the TMR DMA Burst mode
  * @param  htmr TMR handle
  * @param  BurstRequestSrc TMR DMA Request sources to disable
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_DMABurst_WriteStop(TMR_HandleTypeDef *htmr, uint32_t BurstRequestSrc)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_DMA_SOURCE(BurstRequestSrc));

  /* Abort the DMA transfer (at least disable the DMA stream) */
  switch (BurstRequestSrc)
  {
    case TMR_DMA_UPDATE:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_UPDATE]);
      break;
    }
    case TMR_DMA_CC1:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
      break;
    }
    case TMR_DMA_CC2:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
      break;
    }
    case TMR_DMA_CC3:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC3]);
      break;
    }
    case TMR_DMA_CC4:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC4]);
      break;
    }
    case TMR_DMA_COM:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_COMMUTATION]);
      break;
    }
    case TMR_DMA_TRIGGER:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_TRIGGER]);
      break;
    }
    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the TMR Update DMA request */
    __DAL_TMR_DISABLE_DMA(htmr, BurstRequestSrc);

    /* Change the DMA burst operation state */
    htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Configure the DMA Burst to transfer Data from the TMR peripheral to the memory
  * @param  htmr TMR handle
  * @param  BurstBaseAddress TMR Base address from where the DMA  will start the Data read
  *         This parameter can be one of the following values:
  *            @arg TMR_DMABASE_CTRL1
  *            @arg TMR_DMABASE_CTRL2
  *            @arg TMR_DMABASE_SMCTRL
  *            @arg TMR_DMABASE_DIEN
  *            @arg TMR_DMABASE_STS
  *            @arg TMR_DMABASE_CEG
  *            @arg TMR_DMABASE_CCM1
  *            @arg TMR_DMABASE_CCM2
  *            @arg TMR_DMABASE_CCEN
  *            @arg TMR_DMABASE_CNT
  *            @arg TMR_DMABASE_PSC
  *            @arg TMR_DMABASE_AUTORLD
  *            @arg TMR_DMABASE_REPCNT
  *            @arg TMR_DMABASE_CC1
  *            @arg TMR_DMABASE_CC2
  *            @arg TMR_DMABASE_CC3
  *            @arg TMR_DMABASE_CC4
  *            @arg TMR_DMABASE_BDT
  * @param  BurstRequestSrc TMR DMA Request sources
  *         This parameter can be one of the following values:
  *            @arg TMR_DMA_UPDATE: TMR update Interrupt source
  *            @arg TMR_DMA_CC1: TMR Capture Compare 1 DMA source
  *            @arg TMR_DMA_CC2: TMR Capture Compare 2 DMA source
  *            @arg TMR_DMA_CC3: TMR Capture Compare 3 DMA source
  *            @arg TMR_DMA_CC4: TMR Capture Compare 4 DMA source
  *            @arg TMR_DMA_COM: TMR Commutation DMA source
  *            @arg TMR_DMA_TRIGGER: TMR Trigger DMA source
  * @param  BurstBuffer The Buffer address.
  * @param  BurstLength DMA Burst length. This parameter can be one value
  *         between: TMR_DMABURSTLENGTH_1TRANSFER and TMR_DMABURSTLENGTH_18TRANSFERS.
  * @note   This function should be used only when BurstLength is equal to DMA data transfer length.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_DMABurst_ReadStart(TMR_HandleTypeDef *htmr, uint32_t BurstBaseAddress,
                                             uint32_t BurstRequestSrc, uint32_t  *BurstBuffer, uint32_t  BurstLength)
{
  DAL_StatusTypeDef status;

  status = DAL_TMR_DMABurst_MultiReadStart(htmr, BurstBaseAddress, BurstRequestSrc, BurstBuffer, BurstLength,
                                           ((BurstLength) >> 8U) + 1U);


  return status;
}

/**
  * @brief  Configure the DMA Burst to transfer Data from the TMR peripheral to the memory
  * @param  htmr TMR handle
  * @param  BurstBaseAddress TMR Base address from where the DMA  will start the Data read
  *         This parameter can be one of the following values:
  *            @arg TMR_DMABASE_CTRL1
  *            @arg TMR_DMABASE_CTRL2
  *            @arg TMR_DMABASE_SMCTRL
  *            @arg TMR_DMABASE_DIEN
  *            @arg TMR_DMABASE_STS
  *            @arg TMR_DMABASE_CEG
  *            @arg TMR_DMABASE_CCM1
  *            @arg TMR_DMABASE_CCM2
  *            @arg TMR_DMABASE_CCEN
  *            @arg TMR_DMABASE_CNT
  *            @arg TMR_DMABASE_PSC
  *            @arg TMR_DMABASE_AUTORLD
  *            @arg TMR_DMABASE_REPCNT
  *            @arg TMR_DMABASE_CC1
  *            @arg TMR_DMABASE_CC2
  *            @arg TMR_DMABASE_CC3
  *            @arg TMR_DMABASE_CC4
  *            @arg TMR_DMABASE_BDT
  * @param  BurstRequestSrc TMR DMA Request sources
  *         This parameter can be one of the following values:
  *            @arg TMR_DMA_UPDATE: TMR update Interrupt source
  *            @arg TMR_DMA_CC1: TMR Capture Compare 1 DMA source
  *            @arg TMR_DMA_CC2: TMR Capture Compare 2 DMA source
  *            @arg TMR_DMA_CC3: TMR Capture Compare 3 DMA source
  *            @arg TMR_DMA_CC4: TMR Capture Compare 4 DMA source
  *            @arg TMR_DMA_COM: TMR Commutation DMA source
  *            @arg TMR_DMA_TRIGGER: TMR Trigger DMA source
  * @param  BurstBuffer The Buffer address.
  * @param  BurstLength DMA Burst length. This parameter can be one value
  *         between: TMR_DMABURSTLENGTH_1TRANSFER and TMR_DMABURSTLENGTH_18TRANSFERS.
  * @param  DataLength Data length. This parameter can be one value
  *         between 1 and 0xFFFF.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_DMABurst_MultiReadStart(TMR_HandleTypeDef *htmr, uint32_t BurstBaseAddress,
                                                  uint32_t BurstRequestSrc, uint32_t  *BurstBuffer,
                                                  uint32_t  BurstLength, uint32_t  DataLength)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_DMABURST_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_DMA_BASE(BurstBaseAddress));
  ASSERT_PARAM(IS_TMR_DMA_SOURCE(BurstRequestSrc));
  ASSERT_PARAM(IS_TMR_DMA_LENGTH(BurstLength));
  ASSERT_PARAM(IS_TMR_DMA_DATA_LENGTH(DataLength));

  if (htmr->DMABurstState == DAL_DMA_BURST_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (htmr->DMABurstState == DAL_DMA_BURST_STATE_READY)
  {
    if ((BurstBuffer == NULL) && (BurstLength > 0U))
    {
      return DAL_ERROR;
    }
    else
    {
      htmr->DMABurstState = DAL_DMA_BURST_STATE_BUSY;
    }
  }
  else
  {
    /* nothing to do */
  }
  switch (BurstRequestSrc)
  {
    case TMR_DMA_UPDATE:
    {
      /* Set the DMA Period elapsed callbacks */
      htmr->hdma[TMR_DMA_ID_UPDATE]->XferCpltCallback = TMR_DMAPeriodElapsedCplt;
      htmr->hdma[TMR_DMA_ID_UPDATE]->XferHalfCpltCallback = TMR_DMAPeriodElapsedHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_UPDATE]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_UPDATE], (uint32_t)&htmr->Instance->DMAR, (uint32_t)BurstBuffer,
                           DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_CC1:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC1]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC1]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC1]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC1], (uint32_t)&htmr->Instance->DMAR, (uint32_t)BurstBuffer,
                           DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_CC2:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC2]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC2]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC2]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC2], (uint32_t)&htmr->Instance->DMAR, (uint32_t)BurstBuffer,
                           DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_CC3:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC3]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC3]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC3]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC3], (uint32_t)&htmr->Instance->DMAR, (uint32_t)BurstBuffer,
                           DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_CC4:
    {
      /* Set the DMA capture callbacks */
      htmr->hdma[TMR_DMA_ID_CC4]->XferCpltCallback = TMR_DMACaptureCplt;
      htmr->hdma[TMR_DMA_ID_CC4]->XferHalfCpltCallback = TMR_DMACaptureHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_CC4]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_CC4], (uint32_t)&htmr->Instance->DMAR, (uint32_t)BurstBuffer,
                           DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_COM:
    {
      /* Set the DMA commutation callbacks */
      htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferCpltCallback =  TMREx_DMACommutationCplt;
      htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferHalfCpltCallback =  TMREx_DMACommutationHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_COMMUTATION]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_COMMUTATION], (uint32_t)&htmr->Instance->DMAR, (uint32_t)BurstBuffer,
                           DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    case TMR_DMA_TRIGGER:
    {
      /* Set the DMA trigger callbacks */
      htmr->hdma[TMR_DMA_ID_TRIGGER]->XferCpltCallback = TMR_DMATriggerCplt;
      htmr->hdma[TMR_DMA_ID_TRIGGER]->XferHalfCpltCallback = TMR_DMATriggerHalfCplt;

      /* Set the DMA error callback */
      htmr->hdma[TMR_DMA_ID_TRIGGER]->XferErrorCallback = TMR_DMAError ;

      /* Enable the DMA stream */
      if (DAL_DMA_Start_IT(htmr->hdma[TMR_DMA_ID_TRIGGER], (uint32_t)&htmr->Instance->DMAR, (uint32_t)BurstBuffer,
                           DataLength) != DAL_OK)
      {
        /* Return error status */
        return DAL_ERROR;
      }
      break;
    }
    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Configure the DMA Burst Mode */
    htmr->Instance->DCTRL = (BurstBaseAddress | BurstLength);

    /* Enable the TMR DMA Request */
    __DAL_TMR_ENABLE_DMA(htmr, BurstRequestSrc);
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Stop the DMA burst reading
  * @param  htmr TMR handle
  * @param  BurstRequestSrc TMR DMA Request sources to disable.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_DMABurst_ReadStop(TMR_HandleTypeDef *htmr, uint32_t BurstRequestSrc)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_DMA_SOURCE(BurstRequestSrc));

  /* Abort the DMA transfer (at least disable the DMA stream) */
  switch (BurstRequestSrc)
  {
    case TMR_DMA_UPDATE:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_UPDATE]);
      break;
    }
    case TMR_DMA_CC1:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC1]);
      break;
    }
    case TMR_DMA_CC2:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC2]);
      break;
    }
    case TMR_DMA_CC3:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC3]);
      break;
    }
    case TMR_DMA_CC4:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_CC4]);
      break;
    }
    case TMR_DMA_COM:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_COMMUTATION]);
      break;
    }
    case TMR_DMA_TRIGGER:
    {
      (void)DAL_DMA_Abort_IT(htmr->hdma[TMR_DMA_ID_TRIGGER]);
      break;
    }
    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    /* Disable the TMR Update DMA request */
    __DAL_TMR_DISABLE_DMA(htmr, BurstRequestSrc);

    /* Change the DMA burst operation state */
    htmr->DMABurstState = DAL_DMA_BURST_STATE_READY;
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Generate a software event
  * @param  htmr TMR handle
  * @param  EventSource specifies the event source.
  *          This parameter can be one of the following values:
  *            @arg TMR_EVENTSOURCE_UPDATE: Timer update Event source
  *            @arg TMR_EVENTSOURCE_CC1: Timer Capture Compare 1 Event source
  *            @arg TMR_EVENTSOURCE_CC2: Timer Capture Compare 2 Event source
  *            @arg TMR_EVENTSOURCE_CC3: Timer Capture Compare 3 Event source
  *            @arg TMR_EVENTSOURCE_CC4: Timer Capture Compare 4 Event source
  *            @arg TMR_EVENTSOURCE_COM: Timer COM event source
  *            @arg TMR_EVENTSOURCE_TRIGGER: Timer Trigger Event source
  *            @arg TMR_EVENTSOURCE_BREAK: Timer Break event source
  * @note   Basic timers can only generate an update event.
  * @note   TMR_EVENTSOURCE_COM is relevant only with advanced timer instances.
  * @note   TMR_EVENTSOURCE_BREAK are relevant only for timer instances
  *         supporting a break input.
  * @retval DAL status
  */

DAL_StatusTypeDef DAL_TMR_GenerateEvent(TMR_HandleTypeDef *htmr, uint32_t EventSource)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_EVENT_SOURCE(EventSource));

  /* Process Locked */
  __DAL_LOCK(htmr);

  /* Change the TMR state */
  htmr->State = DAL_TMR_STATE_BUSY;

  /* Set the event sources */
  htmr->Instance->CEG = EventSource;

  /* Change the TMR state */
  htmr->State = DAL_TMR_STATE_READY;

  __DAL_UNLOCK(htmr);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Configures the OCRef clear feature
  * @param  htmr TMR handle
  * @param  sClearInputConfig pointer to a TMR_ClearInputConfigTypeDef structure that
  *         contains the OCREF clear feature and parameters for the TMR peripheral.
  * @param  Channel specifies the TMR Channel
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1
  *            @arg TMR_CHANNEL_2: TMR Channel 2
  *            @arg TMR_CHANNEL_3: TMR Channel 3
  *            @arg TMR_CHANNEL_4: TMR Channel 4
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_ConfigOCrefClear(TMR_HandleTypeDef *htmr,
                                           TMR_ClearInputConfigTypeDef *sClearInputConfig,
                                           uint32_t Channel)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_OCXREF_CLEAR_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_CLEARINPUT_SOURCE(sClearInputConfig->ClearInputSource));

  /* Process Locked */
  __DAL_LOCK(htmr);

  htmr->State = DAL_TMR_STATE_BUSY;

  switch (sClearInputConfig->ClearInputSource)
  {
    case TMR_CLEARINPUTSOURCE_NONE:
    {
      /* Clear the OCREF clear selection bit and the the ETR Bits */
      CLEAR_BIT(htmr->Instance->SMCTRL, (TMR_SMCTRL_ETFCFG | TMR_SMCTRL_ETPCFG | TMR_SMCTRL_ECEN | TMR_SMCTRL_ETPOL));
      break;
    }

    case TMR_CLEARINPUTSOURCE_ETR:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CLEARINPUT_POLARITY(sClearInputConfig->ClearInputPolarity));
      ASSERT_PARAM(IS_TMR_CLEARINPUT_PRESCALER(sClearInputConfig->ClearInputPrescaler));
      ASSERT_PARAM(IS_TMR_CLEARINPUT_FILTER(sClearInputConfig->ClearInputFilter));

      /* When OCRef clear feature is used with ETR source, ETR prescaler must be off */
      if (sClearInputConfig->ClearInputPrescaler != TMR_CLEARINPUTPRESCALER_DIV1)
      {
        htmr->State = DAL_TMR_STATE_READY;
        __DAL_UNLOCK(htmr);
        return DAL_ERROR;
      }

      TMR_ETR_SetConfig(htmr->Instance,
                        sClearInputConfig->ClearInputPrescaler,
                        sClearInputConfig->ClearInputPolarity,
                        sClearInputConfig->ClearInputFilter);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  if (status == DAL_OK)
  {
    switch (Channel)
    {
      case TMR_CHANNEL_1:
      {
        if (sClearInputConfig->ClearInputState != (uint32_t)DISABLE)
        {
          /* Enable the OCREF clear feature for Channel 1 */
          SET_BIT(htmr->Instance->CCM1, TMR_CCM1_OC1CEN);
        }
        else
        {
          /* Disable the OCREF clear feature for Channel 1 */
          CLEAR_BIT(htmr->Instance->CCM1, TMR_CCM1_OC1CEN);
        }
        break;
      }
      case TMR_CHANNEL_2:
      {
        if (sClearInputConfig->ClearInputState != (uint32_t)DISABLE)
        {
          /* Enable the OCREF clear feature for Channel 2 */
          SET_BIT(htmr->Instance->CCM1, TMR_CCM1_OC2CEN);
        }
        else
        {
          /* Disable the OCREF clear feature for Channel 2 */
          CLEAR_BIT(htmr->Instance->CCM1, TMR_CCM1_OC2CEN);
        }
        break;
      }
      case TMR_CHANNEL_3:
      {
        if (sClearInputConfig->ClearInputState != (uint32_t)DISABLE)
        {
          /* Enable the OCREF clear feature for Channel 3 */
          SET_BIT(htmr->Instance->CCM2, TMR_CCM2_OC3CEN);
        }
        else
        {
          /* Disable the OCREF clear feature for Channel 3 */
          CLEAR_BIT(htmr->Instance->CCM2, TMR_CCM2_OC3CEN);
        }
        break;
      }
      case TMR_CHANNEL_4:
      {
        if (sClearInputConfig->ClearInputState != (uint32_t)DISABLE)
        {
          /* Enable the OCREF clear feature for Channel 4 */
          SET_BIT(htmr->Instance->CCM2, TMR_CCM2_OC4CEN);
        }
        else
        {
          /* Disable the OCREF clear feature for Channel 4 */
          CLEAR_BIT(htmr->Instance->CCM2, TMR_CCM2_OC4CEN);
        }
        break;
      }
      default:
        break;
    }
  }

  htmr->State = DAL_TMR_STATE_READY;

  __DAL_UNLOCK(htmr);

  return status;
}

/**
  * @brief   Configures the clock source to be used
  * @param  htmr TMR handle
  * @param  sClockSourceConfig pointer to a TMR_ClockConfigTypeDef structure that
  *         contains the clock source information for the TMR peripheral.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_ConfigClockSource(TMR_HandleTypeDef *htmr, TMR_ClockConfigTypeDef *sClockSourceConfig)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;

  /* Process Locked */
  __DAL_LOCK(htmr);

  htmr->State = DAL_TMR_STATE_BUSY;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CLOCKSOURCE(sClockSourceConfig->ClockSource));

  /* Reset the SMFSEL, TRGSEL, ETFCFG, ETPCFG and ECEN bits */
  tmpsmcr = htmr->Instance->SMCTRL;
  tmpsmcr &= ~(TMR_SMCTRL_SMFSEL | TMR_SMCTRL_TRGSEL);
  tmpsmcr &= ~(TMR_SMCTRL_ETFCFG | TMR_SMCTRL_ETPCFG | TMR_SMCTRL_ECEN | TMR_SMCTRL_ETPOL);
  htmr->Instance->SMCTRL = tmpsmcr;

  switch (sClockSourceConfig->ClockSource)
  {
    case TMR_CLOCKSOURCE_INTERNAL:
    {
      ASSERT_PARAM(IS_TMR_INSTANCE(htmr->Instance));
      break;
    }

    case TMR_CLOCKSOURCE_ETRMODE1:
    {
      /* Check whether or not the timer instance supports external trigger input mode 1 (ETRF)*/
      ASSERT_PARAM(IS_TMR_CLOCKSOURCE_ETRMODE1_INSTANCE(htmr->Instance));

      /* Check ETR input conditioning related parameters */
      ASSERT_PARAM(IS_TMR_CLOCKPRESCALER(sClockSourceConfig->ClockPrescaler));
      ASSERT_PARAM(IS_TMR_CLOCKPOLARITY(sClockSourceConfig->ClockPolarity));
      ASSERT_PARAM(IS_TMR_CLOCKFILTER(sClockSourceConfig->ClockFilter));

      /* Configure the ETR Clock source */
      TMR_ETR_SetConfig(htmr->Instance,
                        sClockSourceConfig->ClockPrescaler,
                        sClockSourceConfig->ClockPolarity,
                        sClockSourceConfig->ClockFilter);

      /* Select the External clock mode1 and the ETRF trigger */
      tmpsmcr = htmr->Instance->SMCTRL;
      tmpsmcr |= (TMR_SLAVEMODE_EXTERNAL1 | TMR_CLOCKSOURCE_ETRMODE1);
      /* Write to TMRx SMCTRL */
      htmr->Instance->SMCTRL = tmpsmcr;
      break;
    }

    case TMR_CLOCKSOURCE_ETRMODE2:
    {
      /* Check whether or not the timer instance supports external trigger input mode 2 (ETRF)*/
      ASSERT_PARAM(IS_TMR_CLOCKSOURCE_ETRMODE2_INSTANCE(htmr->Instance));

      /* Check ETR input conditioning related parameters */
      ASSERT_PARAM(IS_TMR_CLOCKPRESCALER(sClockSourceConfig->ClockPrescaler));
      ASSERT_PARAM(IS_TMR_CLOCKPOLARITY(sClockSourceConfig->ClockPolarity));
      ASSERT_PARAM(IS_TMR_CLOCKFILTER(sClockSourceConfig->ClockFilter));

      /* Configure the ETR Clock source */
      TMR_ETR_SetConfig(htmr->Instance,
                        sClockSourceConfig->ClockPrescaler,
                        sClockSourceConfig->ClockPolarity,
                        sClockSourceConfig->ClockFilter);
      /* Enable the External clock mode2 */
      htmr->Instance->SMCTRL |= TMR_SMCTRL_ECEN;
      break;
    }

    case TMR_CLOCKSOURCE_TI1:
    {
      /* Check whether or not the timer instance supports external clock mode 1 */
      ASSERT_PARAM(IS_TMR_CLOCKSOURCE_TIX_INSTANCE(htmr->Instance));

      /* Check TI1 input conditioning related parameters */
      ASSERT_PARAM(IS_TMR_CLOCKPOLARITY(sClockSourceConfig->ClockPolarity));
      ASSERT_PARAM(IS_TMR_CLOCKFILTER(sClockSourceConfig->ClockFilter));

      TMR_TI1_ConfigInputStage(htmr->Instance,
                               sClockSourceConfig->ClockPolarity,
                               sClockSourceConfig->ClockFilter);
      TMR_ITRx_SetConfig(htmr->Instance, TMR_CLOCKSOURCE_TI1);
      break;
    }

    case TMR_CLOCKSOURCE_TI2:
    {
      /* Check whether or not the timer instance supports external clock mode 1 (ETRF)*/
      ASSERT_PARAM(IS_TMR_CLOCKSOURCE_TIX_INSTANCE(htmr->Instance));

      /* Check TI2 input conditioning related parameters */
      ASSERT_PARAM(IS_TMR_CLOCKPOLARITY(sClockSourceConfig->ClockPolarity));
      ASSERT_PARAM(IS_TMR_CLOCKFILTER(sClockSourceConfig->ClockFilter));

      TMR_TI2_ConfigInputStage(htmr->Instance,
                               sClockSourceConfig->ClockPolarity,
                               sClockSourceConfig->ClockFilter);
      TMR_ITRx_SetConfig(htmr->Instance, TMR_CLOCKSOURCE_TI2);
      break;
    }

    case TMR_CLOCKSOURCE_TI1ED:
    {
      /* Check whether or not the timer instance supports external clock mode 1 */
      ASSERT_PARAM(IS_TMR_CLOCKSOURCE_TIX_INSTANCE(htmr->Instance));

      /* Check TI1 input conditioning related parameters */
      ASSERT_PARAM(IS_TMR_CLOCKPOLARITY(sClockSourceConfig->ClockPolarity));
      ASSERT_PARAM(IS_TMR_CLOCKFILTER(sClockSourceConfig->ClockFilter));

      TMR_TI1_ConfigInputStage(htmr->Instance,
                               sClockSourceConfig->ClockPolarity,
                               sClockSourceConfig->ClockFilter);
      TMR_ITRx_SetConfig(htmr->Instance, TMR_CLOCKSOURCE_TI1ED);
      break;
    }

    case TMR_CLOCKSOURCE_ITR0:
    case TMR_CLOCKSOURCE_ITR1:
    case TMR_CLOCKSOURCE_ITR2:
    case TMR_CLOCKSOURCE_ITR3:
    {
      /* Check whether or not the timer instance supports internal trigger input */
      ASSERT_PARAM(IS_TMR_CLOCKSOURCE_ITRX_INSTANCE(htmr->Instance));

      TMR_ITRx_SetConfig(htmr->Instance, sClockSourceConfig->ClockSource);
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }
  htmr->State = DAL_TMR_STATE_READY;

  __DAL_UNLOCK(htmr);

  return status;
}

/**
  * @brief  Selects the signal connected to the TI1 input: direct from CH1_input
  *         or a XOR combination between CH1_input, CH2_input & CH3_input
  * @param  htmr TMR handle.
  * @param  TI1_Selection Indicate whether or not channel 1 is connected to the
  *         output of a XOR gate.
  *          This parameter can be one of the following values:
  *            @arg TMR_TI1SELECTION_CH1: The TMRx_CH1 pin is connected to TI1 input
  *            @arg TMR_TI1SELECTION_XORCOMBINATION: The TMRx_CH1, CH2 and CH3
  *            pins are connected to the TI1 input (XOR combination)
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_ConfigTI1Input(TMR_HandleTypeDef *htmr, uint32_t TI1_Selection)
{
  uint32_t tmpcr2;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_XOR_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_TI1SELECTION(TI1_Selection));

  /* Get the TMRx CTRL2 register value */
  tmpcr2 = htmr->Instance->CTRL2;

  /* Reset the TI1 selection */
  tmpcr2 &= ~TMR_CTRL2_TI1SEL;

  /* Set the TI1 selection */
  tmpcr2 |= TI1_Selection;

  /* Write to TMRx CTRL2 */
  htmr->Instance->CTRL2 = tmpcr2;

  return DAL_OK;
}

/**
  * @brief  Configures the TMR in Slave mode
  * @param  htmr TMR handle.
  * @param  sSlaveConfig pointer to a TMR_SlaveConfigTypeDef structure that
  *         contains the selected trigger (internal trigger input, filtered
  *         timer input or external trigger input) and the Slave mode
  *         (Disable, Reset, Gated, Trigger, External clock mode 1).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_SlaveConfigSynchro(TMR_HandleTypeDef *htmr, TMR_SlaveConfigTypeDef *sSlaveConfig)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_SLAVE_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_SLAVE_MODE(sSlaveConfig->SlaveMode));
  ASSERT_PARAM(IS_TMR_TRIGGER_SELECTION(sSlaveConfig->InputTrigger));

  __DAL_LOCK(htmr);

  htmr->State = DAL_TMR_STATE_BUSY;

  if (TMR_SlaveTimer_SetConfig(htmr, sSlaveConfig) != DAL_OK)
  {
    htmr->State = DAL_TMR_STATE_READY;
    __DAL_UNLOCK(htmr);
    return DAL_ERROR;
  }

  /* Disable Trigger Interrupt */
  __DAL_TMR_DISABLE_IT(htmr, TMR_IT_TRIGGER);

  /* Disable Trigger DMA request */
  __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_TRIGGER);

  htmr->State = DAL_TMR_STATE_READY;

  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Configures the TMR in Slave mode in interrupt mode
  * @param  htmr TMR handle.
  * @param  sSlaveConfig pointer to a TMR_SlaveConfigTypeDef structure that
  *         contains the selected trigger (internal trigger input, filtered
  *         timer input or external trigger input) and the Slave mode
  *         (Disable, Reset, Gated, Trigger, External clock mode 1).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_TMR_SlaveConfigSynchro_IT(TMR_HandleTypeDef *htmr,
                                                TMR_SlaveConfigTypeDef *sSlaveConfig)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_SLAVE_INSTANCE(htmr->Instance));
  ASSERT_PARAM(IS_TMR_SLAVE_MODE(sSlaveConfig->SlaveMode));
  ASSERT_PARAM(IS_TMR_TRIGGER_SELECTION(sSlaveConfig->InputTrigger));

  __DAL_LOCK(htmr);

  htmr->State = DAL_TMR_STATE_BUSY;

  if (TMR_SlaveTimer_SetConfig(htmr, sSlaveConfig) != DAL_OK)
  {
    htmr->State = DAL_TMR_STATE_READY;
    __DAL_UNLOCK(htmr);
    return DAL_ERROR;
  }

  /* Enable Trigger Interrupt */
  __DAL_TMR_ENABLE_IT(htmr, TMR_IT_TRIGGER);

  /* Disable Trigger DMA request */
  __DAL_TMR_DISABLE_DMA(htmr, TMR_DMA_TRIGGER);

  htmr->State = DAL_TMR_STATE_READY;

  __DAL_UNLOCK(htmr);

  return DAL_OK;
}

/**
  * @brief  Read the captured value from Capture Compare unit
  * @param  htmr TMR handle.
  * @param  Channel TMR Channels to be enabled
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1 selected
  *            @arg TMR_CHANNEL_2: TMR Channel 2 selected
  *            @arg TMR_CHANNEL_3: TMR Channel 3 selected
  *            @arg TMR_CHANNEL_4: TMR Channel 4 selected
  * @retval Captured value
  */
uint32_t DAL_TMR_ReadCapturedValue(TMR_HandleTypeDef *htmr, uint32_t Channel)
{
  uint32_t tmpreg = 0U;

  switch (Channel)
  {
    case TMR_CHANNEL_1:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC1_INSTANCE(htmr->Instance));

      /* Return the capture 1 value */
      tmpreg =  htmr->Instance->CC1;

      break;
    }
    case TMR_CHANNEL_2:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC2_INSTANCE(htmr->Instance));

      /* Return the capture 2 value */
      tmpreg =   htmr->Instance->CC2;

      break;
    }

    case TMR_CHANNEL_3:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC3_INSTANCE(htmr->Instance));

      /* Return the capture 3 value */
      tmpreg =   htmr->Instance->CC3;

      break;
    }

    case TMR_CHANNEL_4:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC4_INSTANCE(htmr->Instance));

      /* Return the capture 4 value */
      tmpreg =   htmr->Instance->CC4;

      break;
    }

    default:
      break;
  }

  return tmpreg;
}

/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group9 TMR Callbacks functions
  *  @brief    TMR Callbacks functions
  *
@verbatim
  ==============================================================================
                        ##### TMR Callbacks functions #####
  ==============================================================================
 [..]
   This section provides TMR callback functions:
   (+) TMR Period elapsed callback
   (+) TMR Output Compare callback
   (+) TMR Input capture callback
   (+) TMR Trigger callback
   (+) TMR Error callback

@endverbatim
  * @{
  */

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMR_PeriodElapsedCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_PeriodElapsedCallback could be implemented in the user file
   */
}

/**
  * @brief  Period elapsed half complete callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMR_PeriodElapsedHalfCpltCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_PeriodElapsedHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Output Compare callback in non-blocking mode
  * @param  htmr TMR OC handle
  * @retval None
  */
__weak void DAL_TMR_OC_DelayElapsedCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_OC_DelayElapsedCallback could be implemented in the user file
   */
}

/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htmr TMR IC handle
  * @retval None
  */
__weak void DAL_TMR_IC_CaptureCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_IC_CaptureCallback could be implemented in the user file
   */
}

/**
  * @brief  Input Capture half complete callback in non-blocking mode
  * @param  htmr TMR IC handle
  * @retval None
  */
__weak void DAL_TMR_IC_CaptureHalfCpltCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_IC_CaptureHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  PWM Pulse finished callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMR_PWM_PulseFinishedCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_PWM_PulseFinishedCallback could be implemented in the user file
   */
}

/**
  * @brief  PWM Pulse finished half complete callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMR_PWM_PulseFinishedHalfCpltCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_PWM_PulseFinishedHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Hall Trigger detection callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMR_TriggerCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_TriggerCallback could be implemented in the user file
   */
}

/**
  * @brief  Hall Trigger detection half complete callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMR_TriggerHalfCpltCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_TriggerHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Timer error callback in non-blocking mode
  * @param  htmr TMR handle
  * @retval None
  */
__weak void DAL_TMR_ErrorCallback(TMR_HandleTypeDef *htmr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htmr);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_TMR_ErrorCallback could be implemented in the user file
   */
}

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User TMR callback to be used instead of the weak predefined callback
  * @param htmr tim handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_TMR_BASE_MSPINIT_CB_ID Base MspInit Callback ID
  *          @arg @ref DAL_TMR_BASE_MSPDEINIT_CB_ID Base MspDeInit Callback ID
  *          @arg @ref DAL_TMR_IC_MSPINIT_CB_ID IC MspInit Callback ID
  *          @arg @ref DAL_TMR_IC_MSPDEINIT_CB_ID IC MspDeInit Callback ID
  *          @arg @ref DAL_TMR_OC_MSPINIT_CB_ID OC MspInit Callback ID
  *          @arg @ref DAL_TMR_OC_MSPDEINIT_CB_ID OC MspDeInit Callback ID
  *          @arg @ref DAL_TMR_PWM_MSPINIT_CB_ID PWM MspInit Callback ID
  *          @arg @ref DAL_TMR_PWM_MSPDEINIT_CB_ID PWM MspDeInit Callback ID
  *          @arg @ref DAL_TMR_ONE_PULSE_MSPINIT_CB_ID One Pulse MspInit Callback ID
  *          @arg @ref DAL_TMR_ONE_PULSE_MSPDEINIT_CB_ID One Pulse MspDeInit Callback ID
  *          @arg @ref DAL_TMR_ENCODER_MSPINIT_CB_ID Encoder MspInit Callback ID
  *          @arg @ref DAL_TMR_ENCODER_MSPDEINIT_CB_ID Encoder MspDeInit Callback ID
  *          @arg @ref DAL_TMR_HALL_SENSOR_MSPINIT_CB_ID Hall Sensor MspInit Callback ID
  *          @arg @ref DAL_TMR_HALL_SENSOR_MSPDEINIT_CB_ID Hall Sensor MspDeInit Callback ID
  *          @arg @ref DAL_TMR_PERIOD_ELAPSED_CB_ID Period Elapsed Callback ID
  *          @arg @ref DAL_TMR_PERIOD_ELAPSED_HALF_CB_ID Period Elapsed half complete Callback ID
  *          @arg @ref DAL_TMR_TRIGGER_CB_ID Trigger Callback ID
  *          @arg @ref DAL_TMR_TRIGGER_HALF_CB_ID Trigger half complete Callback ID
  *          @arg @ref DAL_TMR_IC_CAPTURE_CB_ID Input Capture Callback ID
  *          @arg @ref DAL_TMR_IC_CAPTURE_HALF_CB_ID Input Capture half complete Callback ID
  *          @arg @ref DAL_TMR_OC_DELAY_ELAPSED_CB_ID Output Compare Delay Elapsed Callback ID
  *          @arg @ref DAL_TMR_PWM_PULSE_FINISHED_CB_ID PWM Pulse Finished Callback ID
  *          @arg @ref DAL_TMR_PWM_PULSE_FINISHED_HALF_CB_ID PWM Pulse Finished half complete Callback ID
  *          @arg @ref DAL_TMR_ERROR_CB_ID Error Callback ID
  *          @arg @ref DAL_TMR_COMMUTATION_CB_ID Commutation Callback ID
  *          @arg @ref DAL_TMR_COMMUTATION_HALF_CB_ID Commutation half complete Callback ID
  *          @arg @ref DAL_TMR_BREAK_CB_ID Break Callback ID
  *          @param pCallback pointer to the callback function
  *          @retval status
  */
DAL_StatusTypeDef DAL_TMR_RegisterCallback(TMR_HandleTypeDef *htmr, DAL_TMR_CallbackIDTypeDef CallbackID,
                                           pTMR_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(htmr);

  if (htmr->State == DAL_TMR_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_TMR_BASE_MSPINIT_CB_ID :
        htmr->Base_MspInitCallback                 = pCallback;
        break;

      case DAL_TMR_BASE_MSPDEINIT_CB_ID :
        htmr->Base_MspDeInitCallback               = pCallback;
        break;

      case DAL_TMR_IC_MSPINIT_CB_ID :
        htmr->IC_MspInitCallback                   = pCallback;
        break;

      case DAL_TMR_IC_MSPDEINIT_CB_ID :
        htmr->IC_MspDeInitCallback                 = pCallback;
        break;

      case DAL_TMR_OC_MSPINIT_CB_ID :
        htmr->OC_MspInitCallback                   = pCallback;
        break;

      case DAL_TMR_OC_MSPDEINIT_CB_ID :
        htmr->OC_MspDeInitCallback                 = pCallback;
        break;

      case DAL_TMR_PWM_MSPINIT_CB_ID :
        htmr->PWM_MspInitCallback                  = pCallback;
        break;

      case DAL_TMR_PWM_MSPDEINIT_CB_ID :
        htmr->PWM_MspDeInitCallback                = pCallback;
        break;

      case DAL_TMR_ONE_PULSE_MSPINIT_CB_ID :
        htmr->OnePulse_MspInitCallback             = pCallback;
        break;

      case DAL_TMR_ONE_PULSE_MSPDEINIT_CB_ID :
        htmr->OnePulse_MspDeInitCallback           = pCallback;
        break;

      case DAL_TMR_ENCODER_MSPINIT_CB_ID :
        htmr->Encoder_MspInitCallback              = pCallback;
        break;

      case DAL_TMR_ENCODER_MSPDEINIT_CB_ID :
        htmr->Encoder_MspDeInitCallback            = pCallback;
        break;

      case DAL_TMR_HALL_SENSOR_MSPINIT_CB_ID :
        htmr->HallSensor_MspInitCallback           = pCallback;
        break;

      case DAL_TMR_HALL_SENSOR_MSPDEINIT_CB_ID :
        htmr->HallSensor_MspDeInitCallback         = pCallback;
        break;

      case DAL_TMR_PERIOD_ELAPSED_CB_ID :
        htmr->PeriodElapsedCallback                = pCallback;
        break;

      case DAL_TMR_PERIOD_ELAPSED_HALF_CB_ID :
        htmr->PeriodElapsedHalfCpltCallback        = pCallback;
        break;

      case DAL_TMR_TRIGGER_CB_ID :
        htmr->TriggerCallback                      = pCallback;
        break;

      case DAL_TMR_TRIGGER_HALF_CB_ID :
        htmr->TriggerHalfCpltCallback              = pCallback;
        break;

      case DAL_TMR_IC_CAPTURE_CB_ID :
        htmr->IC_CaptureCallback                   = pCallback;
        break;

      case DAL_TMR_IC_CAPTURE_HALF_CB_ID :
        htmr->IC_CaptureHalfCpltCallback           = pCallback;
        break;

      case DAL_TMR_OC_DELAY_ELAPSED_CB_ID :
        htmr->OC_DelayElapsedCallback              = pCallback;
        break;

      case DAL_TMR_PWM_PULSE_FINISHED_CB_ID :
        htmr->PWM_PulseFinishedCallback            = pCallback;
        break;

      case DAL_TMR_PWM_PULSE_FINISHED_HALF_CB_ID :
        htmr->PWM_PulseFinishedHalfCpltCallback    = pCallback;
        break;

      case DAL_TMR_ERROR_CB_ID :
        htmr->ErrorCallback                        = pCallback;
        break;

      case DAL_TMR_COMMUTATION_CB_ID :
        htmr->CommutationCallback                  = pCallback;
        break;

      case DAL_TMR_COMMUTATION_HALF_CB_ID :
        htmr->CommutationHalfCpltCallback          = pCallback;
        break;

      case DAL_TMR_BREAK_CB_ID :
        htmr->BreakCallback                        = pCallback;
        break;

      default :
        /* Return error status */
        status = DAL_ERROR;
        break;
    }
  }
  else if (htmr->State == DAL_TMR_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_TMR_BASE_MSPINIT_CB_ID :
        htmr->Base_MspInitCallback         = pCallback;
        break;

      case DAL_TMR_BASE_MSPDEINIT_CB_ID :
        htmr->Base_MspDeInitCallback       = pCallback;
        break;

      case DAL_TMR_IC_MSPINIT_CB_ID :
        htmr->IC_MspInitCallback           = pCallback;
        break;

      case DAL_TMR_IC_MSPDEINIT_CB_ID :
        htmr->IC_MspDeInitCallback         = pCallback;
        break;

      case DAL_TMR_OC_MSPINIT_CB_ID :
        htmr->OC_MspInitCallback           = pCallback;
        break;

      case DAL_TMR_OC_MSPDEINIT_CB_ID :
        htmr->OC_MspDeInitCallback         = pCallback;
        break;

      case DAL_TMR_PWM_MSPINIT_CB_ID :
        htmr->PWM_MspInitCallback          = pCallback;
        break;

      case DAL_TMR_PWM_MSPDEINIT_CB_ID :
        htmr->PWM_MspDeInitCallback        = pCallback;
        break;

      case DAL_TMR_ONE_PULSE_MSPINIT_CB_ID :
        htmr->OnePulse_MspInitCallback     = pCallback;
        break;

      case DAL_TMR_ONE_PULSE_MSPDEINIT_CB_ID :
        htmr->OnePulse_MspDeInitCallback   = pCallback;
        break;

      case DAL_TMR_ENCODER_MSPINIT_CB_ID :
        htmr->Encoder_MspInitCallback      = pCallback;
        break;

      case DAL_TMR_ENCODER_MSPDEINIT_CB_ID :
        htmr->Encoder_MspDeInitCallback    = pCallback;
        break;

      case DAL_TMR_HALL_SENSOR_MSPINIT_CB_ID :
        htmr->HallSensor_MspInitCallback   = pCallback;
        break;

      case DAL_TMR_HALL_SENSOR_MSPDEINIT_CB_ID :
        htmr->HallSensor_MspDeInitCallback = pCallback;
        break;

      default :
        /* Return error status */
        status = DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Return error status */
    status = DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return status;
}

/**
  * @brief  Unregister a TMR callback
  *         TMR callback is redirected to the weak predefined callback
  * @param htmr tim handle
  * @param CallbackID ID of the callback to be unregistered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_TMR_BASE_MSPINIT_CB_ID Base MspInit Callback ID
  *          @arg @ref DAL_TMR_BASE_MSPDEINIT_CB_ID Base MspDeInit Callback ID
  *          @arg @ref DAL_TMR_IC_MSPINIT_CB_ID IC MspInit Callback ID
  *          @arg @ref DAL_TMR_IC_MSPDEINIT_CB_ID IC MspDeInit Callback ID
  *          @arg @ref DAL_TMR_OC_MSPINIT_CB_ID OC MspInit Callback ID
  *          @arg @ref DAL_TMR_OC_MSPDEINIT_CB_ID OC MspDeInit Callback ID
  *          @arg @ref DAL_TMR_PWM_MSPINIT_CB_ID PWM MspInit Callback ID
  *          @arg @ref DAL_TMR_PWM_MSPDEINIT_CB_ID PWM MspDeInit Callback ID
  *          @arg @ref DAL_TMR_ONE_PULSE_MSPINIT_CB_ID One Pulse MspInit Callback ID
  *          @arg @ref DAL_TMR_ONE_PULSE_MSPDEINIT_CB_ID One Pulse MspDeInit Callback ID
  *          @arg @ref DAL_TMR_ENCODER_MSPINIT_CB_ID Encoder MspInit Callback ID
  *          @arg @ref DAL_TMR_ENCODER_MSPDEINIT_CB_ID Encoder MspDeInit Callback ID
  *          @arg @ref DAL_TMR_HALL_SENSOR_MSPINIT_CB_ID Hall Sensor MspInit Callback ID
  *          @arg @ref DAL_TMR_HALL_SENSOR_MSPDEINIT_CB_ID Hall Sensor MspDeInit Callback ID
  *          @arg @ref DAL_TMR_PERIOD_ELAPSED_CB_ID Period Elapsed Callback ID
  *          @arg @ref DAL_TMR_PERIOD_ELAPSED_HALF_CB_ID Period Elapsed half complete Callback ID
  *          @arg @ref DAL_TMR_TRIGGER_CB_ID Trigger Callback ID
  *          @arg @ref DAL_TMR_TRIGGER_HALF_CB_ID Trigger half complete Callback ID
  *          @arg @ref DAL_TMR_IC_CAPTURE_CB_ID Input Capture Callback ID
  *          @arg @ref DAL_TMR_IC_CAPTURE_HALF_CB_ID Input Capture half complete Callback ID
  *          @arg @ref DAL_TMR_OC_DELAY_ELAPSED_CB_ID Output Compare Delay Elapsed Callback ID
  *          @arg @ref DAL_TMR_PWM_PULSE_FINISHED_CB_ID PWM Pulse Finished Callback ID
  *          @arg @ref DAL_TMR_PWM_PULSE_FINISHED_HALF_CB_ID PWM Pulse Finished half complete Callback ID
  *          @arg @ref DAL_TMR_ERROR_CB_ID Error Callback ID
  *          @arg @ref DAL_TMR_COMMUTATION_CB_ID Commutation Callback ID
  *          @arg @ref DAL_TMR_COMMUTATION_HALF_CB_ID Commutation half complete Callback ID
  *          @arg @ref DAL_TMR_BREAK_CB_ID Break Callback ID
  *          @retval status
  */
DAL_StatusTypeDef DAL_TMR_UnRegisterCallback(TMR_HandleTypeDef *htmr, DAL_TMR_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(htmr);

  if (htmr->State == DAL_TMR_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_TMR_BASE_MSPINIT_CB_ID :
        /* Legacy weak Base MspInit Callback */
        htmr->Base_MspInitCallback              = DAL_TMR_Base_MspInit;
        break;

      case DAL_TMR_BASE_MSPDEINIT_CB_ID :
        /* Legacy weak Base Msp DeInit Callback */
        htmr->Base_MspDeInitCallback            = DAL_TMR_Base_MspDeInit;
        break;

      case DAL_TMR_IC_MSPINIT_CB_ID :
        /* Legacy weak IC Msp Init Callback */
        htmr->IC_MspInitCallback                = DAL_TMR_IC_MspInit;
        break;

      case DAL_TMR_IC_MSPDEINIT_CB_ID :
        /* Legacy weak IC Msp DeInit Callback */
        htmr->IC_MspDeInitCallback              = DAL_TMR_IC_MspDeInit;
        break;

      case DAL_TMR_OC_MSPINIT_CB_ID :
        /* Legacy weak OC Msp Init Callback */
        htmr->OC_MspInitCallback                = DAL_TMR_OC_MspInit;
        break;

      case DAL_TMR_OC_MSPDEINIT_CB_ID :
        /* Legacy weak OC Msp DeInit Callback */
        htmr->OC_MspDeInitCallback              = DAL_TMR_OC_MspDeInit;
        break;

      case DAL_TMR_PWM_MSPINIT_CB_ID :
        /* Legacy weak PWM Msp Init Callback */
        htmr->PWM_MspInitCallback               = DAL_TMR_PWM_MspInit;
        break;

      case DAL_TMR_PWM_MSPDEINIT_CB_ID :
        /* Legacy weak PWM Msp DeInit Callback */
        htmr->PWM_MspDeInitCallback             = DAL_TMR_PWM_MspDeInit;
        break;

      case DAL_TMR_ONE_PULSE_MSPINIT_CB_ID :
        /* Legacy weak One Pulse Msp Init Callback */
        htmr->OnePulse_MspInitCallback          = DAL_TMR_OnePulse_MspInit;
        break;

      case DAL_TMR_ONE_PULSE_MSPDEINIT_CB_ID :
        /* Legacy weak One Pulse Msp DeInit Callback */
        htmr->OnePulse_MspDeInitCallback        = DAL_TMR_OnePulse_MspDeInit;
        break;

      case DAL_TMR_ENCODER_MSPINIT_CB_ID :
        /* Legacy weak Encoder Msp Init Callback */
        htmr->Encoder_MspInitCallback           = DAL_TMR_Encoder_MspInit;
        break;

      case DAL_TMR_ENCODER_MSPDEINIT_CB_ID :
        /* Legacy weak Encoder Msp DeInit Callback */
        htmr->Encoder_MspDeInitCallback         = DAL_TMR_Encoder_MspDeInit;
        break;

      case DAL_TMR_HALL_SENSOR_MSPINIT_CB_ID :
        /* Legacy weak Hall Sensor Msp Init Callback */
        htmr->HallSensor_MspInitCallback        = DAL_TMREx_HallSensor_MspInit;
        break;

      case DAL_TMR_HALL_SENSOR_MSPDEINIT_CB_ID :
        /* Legacy weak Hall Sensor Msp DeInit Callback */
        htmr->HallSensor_MspDeInitCallback      = DAL_TMREx_HallSensor_MspDeInit;
        break;

      case DAL_TMR_PERIOD_ELAPSED_CB_ID :
        /* Legacy weak Period Elapsed Callback */
        htmr->PeriodElapsedCallback             = DAL_TMR_PeriodElapsedCallback;
        break;

      case DAL_TMR_PERIOD_ELAPSED_HALF_CB_ID :
        /* Legacy weak Period Elapsed half complete Callback */
        htmr->PeriodElapsedHalfCpltCallback     = DAL_TMR_PeriodElapsedHalfCpltCallback;
        break;

      case DAL_TMR_TRIGGER_CB_ID :
        /* Legacy weak Trigger Callback */
        htmr->TriggerCallback                   = DAL_TMR_TriggerCallback;
        break;

      case DAL_TMR_TRIGGER_HALF_CB_ID :
        /* Legacy weak Trigger half complete Callback */
        htmr->TriggerHalfCpltCallback           = DAL_TMR_TriggerHalfCpltCallback;
        break;

      case DAL_TMR_IC_CAPTURE_CB_ID :
        /* Legacy weak IC Capture Callback */
        htmr->IC_CaptureCallback                = DAL_TMR_IC_CaptureCallback;
        break;

      case DAL_TMR_IC_CAPTURE_HALF_CB_ID :
        /* Legacy weak IC Capture half complete Callback */
        htmr->IC_CaptureHalfCpltCallback        = DAL_TMR_IC_CaptureHalfCpltCallback;
        break;

      case DAL_TMR_OC_DELAY_ELAPSED_CB_ID :
        /* Legacy weak OC Delay Elapsed Callback */
        htmr->OC_DelayElapsedCallback           = DAL_TMR_OC_DelayElapsedCallback;
        break;

      case DAL_TMR_PWM_PULSE_FINISHED_CB_ID :
        /* Legacy weak PWM Pulse Finished Callback */
        htmr->PWM_PulseFinishedCallback         = DAL_TMR_PWM_PulseFinishedCallback;
        break;

      case DAL_TMR_PWM_PULSE_FINISHED_HALF_CB_ID :
        /* Legacy weak PWM Pulse Finished half complete Callback */
        htmr->PWM_PulseFinishedHalfCpltCallback = DAL_TMR_PWM_PulseFinishedHalfCpltCallback;
        break;

      case DAL_TMR_ERROR_CB_ID :
        /* Legacy weak Error Callback */
        htmr->ErrorCallback                     = DAL_TMR_ErrorCallback;
        break;

      case DAL_TMR_COMMUTATION_CB_ID :
        /* Legacy weak Commutation Callback */
        htmr->CommutationCallback               = DAL_TMREx_CommutCallback;
        break;

      case DAL_TMR_COMMUTATION_HALF_CB_ID :
        /* Legacy weak Commutation half complete Callback */
        htmr->CommutationHalfCpltCallback       = DAL_TMREx_CommutHalfCpltCallback;
        break;

      case DAL_TMR_BREAK_CB_ID :
        /* Legacy weak Break Callback */
        htmr->BreakCallback                     = DAL_TMREx_BreakCallback;
        break;

      default :
        /* Return error status */
        status = DAL_ERROR;
        break;
    }
  }
  else if (htmr->State == DAL_TMR_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_TMR_BASE_MSPINIT_CB_ID :
        /* Legacy weak Base MspInit Callback */
        htmr->Base_MspInitCallback         = DAL_TMR_Base_MspInit;
        break;

      case DAL_TMR_BASE_MSPDEINIT_CB_ID :
        /* Legacy weak Base Msp DeInit Callback */
        htmr->Base_MspDeInitCallback       = DAL_TMR_Base_MspDeInit;
        break;

      case DAL_TMR_IC_MSPINIT_CB_ID :
        /* Legacy weak IC Msp Init Callback */
        htmr->IC_MspInitCallback           = DAL_TMR_IC_MspInit;
        break;

      case DAL_TMR_IC_MSPDEINIT_CB_ID :
        /* Legacy weak IC Msp DeInit Callback */
        htmr->IC_MspDeInitCallback         = DAL_TMR_IC_MspDeInit;
        break;

      case DAL_TMR_OC_MSPINIT_CB_ID :
        /* Legacy weak OC Msp Init Callback */
        htmr->OC_MspInitCallback           = DAL_TMR_OC_MspInit;
        break;

      case DAL_TMR_OC_MSPDEINIT_CB_ID :
        /* Legacy weak OC Msp DeInit Callback */
        htmr->OC_MspDeInitCallback         = DAL_TMR_OC_MspDeInit;
        break;

      case DAL_TMR_PWM_MSPINIT_CB_ID :
        /* Legacy weak PWM Msp Init Callback */
        htmr->PWM_MspInitCallback          = DAL_TMR_PWM_MspInit;
        break;

      case DAL_TMR_PWM_MSPDEINIT_CB_ID :
        /* Legacy weak PWM Msp DeInit Callback */
        htmr->PWM_MspDeInitCallback        = DAL_TMR_PWM_MspDeInit;
        break;

      case DAL_TMR_ONE_PULSE_MSPINIT_CB_ID :
        /* Legacy weak One Pulse Msp Init Callback */
        htmr->OnePulse_MspInitCallback     = DAL_TMR_OnePulse_MspInit;
        break;

      case DAL_TMR_ONE_PULSE_MSPDEINIT_CB_ID :
        /* Legacy weak One Pulse Msp DeInit Callback */
        htmr->OnePulse_MspDeInitCallback   = DAL_TMR_OnePulse_MspDeInit;
        break;

      case DAL_TMR_ENCODER_MSPINIT_CB_ID :
        /* Legacy weak Encoder Msp Init Callback */
        htmr->Encoder_MspInitCallback      = DAL_TMR_Encoder_MspInit;
        break;

      case DAL_TMR_ENCODER_MSPDEINIT_CB_ID :
        /* Legacy weak Encoder Msp DeInit Callback */
        htmr->Encoder_MspDeInitCallback    = DAL_TMR_Encoder_MspDeInit;
        break;

      case DAL_TMR_HALL_SENSOR_MSPINIT_CB_ID :
        /* Legacy weak Hall Sensor Msp Init Callback */
        htmr->HallSensor_MspInitCallback   = DAL_TMREx_HallSensor_MspInit;
        break;

      case DAL_TMR_HALL_SENSOR_MSPDEINIT_CB_ID :
        /* Legacy weak Hall Sensor Msp DeInit Callback */
        htmr->HallSensor_MspDeInitCallback = DAL_TMREx_HallSensor_MspDeInit;
        break;

      default :
        /* Return error status */
        status = DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Return error status */
    status = DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(htmr);

  return status;
}
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup TMR_Exported_Functions_Group10 TMR Peripheral State functions
  *  @brief   TMR Peripheral State functions
  *
@verbatim
  ==============================================================================
                        ##### Peripheral State functions #####
  ==============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the TMR Base handle state.
  * @param  htmr TMR Base handle
  * @retval DAL state
  */
DAL_TMR_StateTypeDef DAL_TMR_Base_GetState(TMR_HandleTypeDef *htmr)
{
  return htmr->State;
}

/**
  * @brief  Return the TMR OC handle state.
  * @param  htmr TMR Output Compare handle
  * @retval DAL state
  */
DAL_TMR_StateTypeDef DAL_TMR_OC_GetState(TMR_HandleTypeDef *htmr)
{
  return htmr->State;
}

/**
  * @brief  Return the TMR PWM handle state.
  * @param  htmr TMR handle
  * @retval DAL state
  */
DAL_TMR_StateTypeDef DAL_TMR_PWM_GetState(TMR_HandleTypeDef *htmr)
{
  return htmr->State;
}

/**
  * @brief  Return the TMR Input Capture handle state.
  * @param  htmr TMR IC handle
  * @retval DAL state
  */
DAL_TMR_StateTypeDef DAL_TMR_IC_GetState(TMR_HandleTypeDef *htmr)
{
  return htmr->State;
}

/**
  * @brief  Return the TMR One Pulse Mode handle state.
  * @param  htmr TMR OPM handle
  * @retval DAL state
  */
DAL_TMR_StateTypeDef DAL_TMR_OnePulse_GetState(TMR_HandleTypeDef *htmr)
{
  return htmr->State;
}

/**
  * @brief  Return the TMR Encoder Mode handle state.
  * @param  htmr TMR Encoder Interface handle
  * @retval DAL state
  */
DAL_TMR_StateTypeDef DAL_TMR_Encoder_GetState(TMR_HandleTypeDef *htmr)
{
  return htmr->State;
}

/**
  * @brief  Return the TMR Encoder Mode handle state.
  * @param  htmr TMR handle
  * @retval Active channel
  */
DAL_TMR_ActiveChannel DAL_TMR_GetActiveChannel(TMR_HandleTypeDef *htmr)
{
  return htmr->Channel;
}

/**
  * @brief  Return actual state of the TMR channel.
  * @param  htmr TMR handle
  * @param  Channel TMR Channel
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1
  *            @arg TMR_CHANNEL_2: TMR Channel 2
  *            @arg TMR_CHANNEL_3: TMR Channel 3
  *            @arg TMR_CHANNEL_4: TMR Channel 4
  *            @arg TMR_CHANNEL_5: TMR Channel 5
  *            @arg TMR_CHANNEL_6: TMR Channel 6
  * @retval TMR Channel state
  */
DAL_TMR_ChannelStateTypeDef DAL_TMR_GetChannelState(TMR_HandleTypeDef *htmr,  uint32_t Channel)
{
  DAL_TMR_ChannelStateTypeDef channel_state;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CCX_INSTANCE(htmr->Instance, Channel));

  channel_state = TMR_CHANNEL_STATE_GET(htmr, Channel);

  return channel_state;
}

/**
  * @brief  Return actual state of a DMA burst operation.
  * @param  htmr TMR handle
  * @retval DMA burst state
  */
DAL_TMR_DMABurstStateTypeDef DAL_TMR_DMABurstState(TMR_HandleTypeDef *htmr)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_DMABURST_INSTANCE(htmr->Instance));

  return htmr->DMABurstState;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup TMR_Private_Functions TMR Private Functions
  * @{
  */

/**
  * @brief  TMR DMA error callback
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TMR_DMAError(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdma == htmr->hdma[TMR_DMA_ID_CC1])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_1;
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC2])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_2;
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC3])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_3;
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_3, DAL_TMR_CHANNEL_STATE_READY);
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC4])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_4;
    TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_4, DAL_TMR_CHANNEL_STATE_READY);
  }
  else
  {
    htmr->State = DAL_TMR_STATE_READY;
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->ErrorCallback(htmr);
#else
  DAL_TMR_ErrorCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  TMR DMA Delay Pulse complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TMR_DMADelayPulseCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdma == htmr->hdma[TMR_DMA_ID_CC1])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_1;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC2])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_2;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC3])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_3;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_3, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC4])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_4;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_4, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else
  {
    /* nothing to do */
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->PWM_PulseFinishedCallback(htmr);
#else
  DAL_TMR_PWM_PulseFinishedCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  TMR DMA Delay Pulse half complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TMR_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdma == htmr->hdma[TMR_DMA_ID_CC1])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_1;
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC2])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_2;
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC3])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_3;
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC4])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_4;
  }
  else
  {
    /* nothing to do */
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->PWM_PulseFinishedHalfCpltCallback(htmr);
#else
  DAL_TMR_PWM_PulseFinishedHalfCpltCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  TMR DMA Capture complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TMR_DMACaptureCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdma == htmr->hdma[TMR_DMA_ID_CC1])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_1;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_1, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC2])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_2;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_2, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC3])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_3;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_3, DAL_TMR_CHANNEL_STATE_READY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_3, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC4])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_4;

    if (hdma->Init.Mode == DMA_NORMAL)
    {
      TMR_CHANNEL_STATE_SET(htmr, TMR_CHANNEL_4, DAL_TMR_CHANNEL_STATE_READY);
      TMR_CHANNEL_N_STATE_SET(htmr, TMR_CHANNEL_4, DAL_TMR_CHANNEL_STATE_READY);
    }
  }
  else
  {
    /* nothing to do */
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->IC_CaptureCallback(htmr);
#else
  DAL_TMR_IC_CaptureCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  TMR DMA Capture half complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
void TMR_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hdma == htmr->hdma[TMR_DMA_ID_CC1])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_1;
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC2])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_2;
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC3])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_3;
  }
  else if (hdma == htmr->hdma[TMR_DMA_ID_CC4])
  {
    htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_4;
  }
  else
  {
    /* nothing to do */
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->IC_CaptureHalfCpltCallback(htmr);
#else
  DAL_TMR_IC_CaptureHalfCpltCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

  htmr->Channel = DAL_TMR_ACTIVE_CHANNEL_CLEARED;
}

/**
  * @brief  TMR DMA Period Elapse complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void TMR_DMAPeriodElapsedCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (htmr->hdma[TMR_DMA_ID_UPDATE]->Init.Mode == DMA_NORMAL)
  {
    htmr->State = DAL_TMR_STATE_READY;
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->PeriodElapsedCallback(htmr);
#else
  DAL_TMR_PeriodElapsedCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
}

/**
  * @brief  TMR DMA Period Elapse half complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void TMR_DMAPeriodElapsedHalfCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->PeriodElapsedHalfCpltCallback(htmr);
#else
  DAL_TMR_PeriodElapsedHalfCpltCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
}

/**
  * @brief  TMR DMA Trigger callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void TMR_DMATriggerCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (htmr->hdma[TMR_DMA_ID_TRIGGER]->Init.Mode == DMA_NORMAL)
  {
    htmr->State = DAL_TMR_STATE_READY;
  }

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->TriggerCallback(htmr);
#else
  DAL_TMR_TriggerCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
}

/**
  * @brief  TMR DMA Trigger half complete callback.
  * @param  hdma pointer to DMA handle.
  * @retval None
  */
static void TMR_DMATriggerHalfCplt(DMA_HandleTypeDef *hdma)
{
  TMR_HandleTypeDef *htmr = (TMR_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
  htmr->TriggerHalfCpltCallback(htmr);
#else
  DAL_TMR_TriggerHalfCpltCallback(htmr);
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */
}

/**
  * @brief  Time Base configuration
  * @param  TMRx TMR peripheral
  * @param  Structure TMR Base configuration structure
  * @retval None
  */
void TMR_Base_SetConfig(TMR_TypeDef *TMRx, TMR_Base_InitTypeDef *Structure)
{
  uint32_t tmpcr1;
  tmpcr1 = TMRx->CTRL1;

  /* Set TMR Time Base Unit parameters ---------------------------------------*/
  if (IS_TMR_COUNTER_MODE_SELECT_INSTANCE(TMRx))
  {
    /* Select the Counter Mode */
    tmpcr1 &= ~(TMR_CTRL1_CNTDIR | TMR_CTRL1_CAMSEL);
    tmpcr1 |= Structure->CounterMode;
  }

  if (IS_TMR_CLOCK_DIVISION_INSTANCE(TMRx))
  {
    /* Set the clock division */
    tmpcr1 &= ~TMR_CTRL1_CLKDIV;
    tmpcr1 |= (uint32_t)Structure->ClockDivision;
  }

  /* Set the auto-reload preload */
  MODIFY_REG(tmpcr1, TMR_CTRL1_ARPEN, Structure->AutoReloadPreload);

  TMRx->CTRL1 = tmpcr1;

  /* Set the Autoreload value */
  TMRx->AUTORLD = (uint32_t)Structure->Period ;

  /* Set the Prescaler value */
  TMRx->PSC = Structure->Prescaler;

  if (IS_TMR_REPETITION_COUNTER_INSTANCE(TMRx))
  {
    /* Set the Repetition Counter value */
    TMRx->REPCNT = Structure->RepetitionCounter;
  }

  /* Generate an update event to reload the Prescaler
     and the repetition counter (only for advanced timer) value immediately */
  TMRx->CEG = TMR_CEG_UEG;
}

/**
  * @brief  Timer Output Compare 1 configuration
  * @param  TMRx to select the TMR peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TMR_OC1_SetConfig(TMR_TypeDef *TMRx, TMR_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 1: Reset the CC1EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC1EN;

  /* Get the TMRx CCEN register value */
  tmpccer = TMRx->CCEN;
  /* Get the TMRx CTRL2 register value */
  tmpcr2 =  TMRx->CTRL2;

  /* Get the TMRx CCM1 register value */
  tmpccmrx = TMRx->CCM1;

  /* Reset the Output Compare Mode Bits */
  tmpccmrx &= ~TMR_CCM1_OC1MOD;
  tmpccmrx &= ~TMR_CCM1_CC1SEL;
  /* Select the Output Compare Mode */
  tmpccmrx |= OC_Config->OCMode;

  /* Reset the Output Polarity level */
  tmpccer &= ~TMR_CCEN_CC1POL;
  /* Set the Output Compare Polarity */
  tmpccer |= OC_Config->OCPolarity;

  if (IS_TMR_CCXN_INSTANCE(TMRx, TMR_CHANNEL_1))
  {
    /* Check parameters */
    ASSERT_PARAM(IS_TMR_OCN_POLARITY(OC_Config->OCNPolarity));

    /* Reset the Output N Polarity level */
    tmpccer &= ~TMR_CCEN_CC1NPOL;
    /* Set the Output N Polarity */
    tmpccer |= OC_Config->OCNPolarity;
    /* Reset the Output N State */
    tmpccer &= ~TMR_CCEN_CC1NEN;
  }

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    /* Check parameters */
    ASSERT_PARAM(IS_TMR_OCNIDLE_STATE(OC_Config->OCNIdleState));
    ASSERT_PARAM(IS_TMR_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare and Output Compare N IDLE State */
    tmpcr2 &= ~TMR_CTRL2_OC1OIS;
    tmpcr2 &= ~TMR_CTRL2_OC1NOIS;
    /* Set the Output Idle state */
    tmpcr2 |= OC_Config->OCIdleState;
    /* Set the Output N Idle state */
    tmpcr2 |= OC_Config->OCNIdleState;
  }

  /* Write to TMRx CTRL2 */
  TMRx->CTRL2 = tmpcr2;

  /* Write to TMRx CCM1 */
  TMRx->CCM1 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TMRx->CC1 = OC_Config->Pulse;

  /* Write to TMRx CCEN */
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Timer Output Compare 2 configuration
  * @param  TMRx to select the TMR peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
void TMR_OC2_SetConfig(TMR_TypeDef *TMRx, TMR_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 2: Reset the CC2EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC2EN;

  /* Get the TMRx CCEN register value */
  tmpccer = TMRx->CCEN;
  /* Get the TMRx CTRL2 register value */
  tmpcr2 =  TMRx->CTRL2;

  /* Get the TMRx CCM1 register value */
  tmpccmrx = TMRx->CCM1;

  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  tmpccmrx &= ~TMR_CCM1_OC2MOD;
  tmpccmrx &= ~TMR_CCM1_CC2SEL;

  /* Select the Output Compare Mode */
  tmpccmrx |= (OC_Config->OCMode << 8U);

  /* Reset the Output Polarity level */
  tmpccer &= ~TMR_CCEN_CC2POL;
  /* Set the Output Compare Polarity */
  tmpccer |= (OC_Config->OCPolarity << 4U);

  if (IS_TMR_CCXN_INSTANCE(TMRx, TMR_CHANNEL_2))
  {
    ASSERT_PARAM(IS_TMR_OCN_POLARITY(OC_Config->OCNPolarity));

    /* Reset the Output N Polarity level */
    tmpccer &= ~TMR_CCEN_CC2NPOL;
    /* Set the Output N Polarity */
    tmpccer |= (OC_Config->OCNPolarity << 4U);
    /* Reset the Output N State */
    tmpccer &= ~TMR_CCEN_CC2NEN;

  }

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    /* Check parameters */
    ASSERT_PARAM(IS_TMR_OCNIDLE_STATE(OC_Config->OCNIdleState));
    ASSERT_PARAM(IS_TMR_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare and Output Compare N IDLE State */
    tmpcr2 &= ~TMR_CTRL2_OC2OIS;
    tmpcr2 &= ~TMR_CTRL2_OC2NOIS;
    /* Set the Output Idle state */
    tmpcr2 |= (OC_Config->OCIdleState << 2U);
    /* Set the Output N Idle state */
    tmpcr2 |= (OC_Config->OCNIdleState << 2U);
  }

  /* Write to TMRx CTRL2 */
  TMRx->CTRL2 = tmpcr2;

  /* Write to TMRx CCM1 */
  TMRx->CCM1 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TMRx->CC2 = OC_Config->Pulse;

  /* Write to TMRx CCEN */
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Timer Output Compare 3 configuration
  * @param  TMRx to select the TMR peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TMR_OC3_SetConfig(TMR_TypeDef *TMRx, TMR_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 3: Reset the CC3EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC3EN;

  /* Get the TMRx CCEN register value */
  tmpccer = TMRx->CCEN;
  /* Get the TMRx CTRL2 register value */
  tmpcr2 =  TMRx->CTRL2;

  /* Get the TMRx CCM2 register value */
  tmpccmrx = TMRx->CCM2;

  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  tmpccmrx &= ~TMR_CCM2_OC3MOD;
  tmpccmrx &= ~TMR_CCM2_CC3SEL;
  /* Select the Output Compare Mode */
  tmpccmrx |= OC_Config->OCMode;

  /* Reset the Output Polarity level */
  tmpccer &= ~TMR_CCEN_CC3POL;
  /* Set the Output Compare Polarity */
  tmpccer |= (OC_Config->OCPolarity << 8U);

  if (IS_TMR_CCXN_INSTANCE(TMRx, TMR_CHANNEL_3))
  {
    ASSERT_PARAM(IS_TMR_OCN_POLARITY(OC_Config->OCNPolarity));

    /* Reset the Output N Polarity level */
    tmpccer &= ~TMR_CCEN_CC3NPOL;
    /* Set the Output N Polarity */
    tmpccer |= (OC_Config->OCNPolarity << 8U);
    /* Reset the Output N State */
    tmpccer &= ~TMR_CCEN_CC3NEN;
  }

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    /* Check parameters */
    ASSERT_PARAM(IS_TMR_OCNIDLE_STATE(OC_Config->OCNIdleState));
    ASSERT_PARAM(IS_TMR_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare and Output Compare N IDLE State */
    tmpcr2 &= ~TMR_CTRL2_OC3OIS;
    tmpcr2 &= ~TMR_CTRL2_OC3NOIS;
    /* Set the Output Idle state */
    tmpcr2 |= (OC_Config->OCIdleState << 4U);
    /* Set the Output N Idle state */
    tmpcr2 |= (OC_Config->OCNIdleState << 4U);
  }

  /* Write to TMRx CTRL2 */
  TMRx->CTRL2 = tmpcr2;

  /* Write to TMRx CCM2 */
  TMRx->CCM2 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TMRx->CC3 = OC_Config->Pulse;

  /* Write to TMRx CCEN */
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Timer Output Compare 4 configuration
  * @param  TMRx to select the TMR peripheral
  * @param  OC_Config The output configuration structure
  * @retval None
  */
static void TMR_OC4_SetConfig(TMR_TypeDef *TMRx, TMR_OC_InitTypeDef *OC_Config)
{
  uint32_t tmpccmrx;
  uint32_t tmpccer;
  uint32_t tmpcr2;

  /* Disable the Channel 4: Reset the CC4EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC4EN;

  /* Get the TMRx CCEN register value */
  tmpccer = TMRx->CCEN;
  /* Get the TMRx CTRL2 register value */
  tmpcr2 =  TMRx->CTRL2;

  /* Get the TMRx CCM2 register value */
  tmpccmrx = TMRx->CCM2;

  /* Reset the Output Compare mode and Capture/Compare selection Bits */
  tmpccmrx &= ~TMR_CCM2_OC4MOD;
  tmpccmrx &= ~TMR_CCM2_CC4SEL;

  /* Select the Output Compare Mode */
  tmpccmrx |= (OC_Config->OCMode << 8U);

  /* Reset the Output Polarity level */
  tmpccer &= ~TMR_CCEN_CC4POL;
  /* Set the Output Compare Polarity */
  tmpccer |= (OC_Config->OCPolarity << 12U);

  if (IS_TMR_BREAK_INSTANCE(TMRx))
  {
    /* Check parameters */
    ASSERT_PARAM(IS_TMR_OCIDLE_STATE(OC_Config->OCIdleState));

    /* Reset the Output Compare IDLE State */
    tmpcr2 &= ~TMR_CTRL2_OC4OIS;

    /* Set the Output Idle state */
    tmpcr2 |= (OC_Config->OCIdleState << 6U);
  }

  /* Write to TMRx CTRL2 */
  TMRx->CTRL2 = tmpcr2;

  /* Write to TMRx CCM2 */
  TMRx->CCM2 = tmpccmrx;

  /* Set the Capture Compare Register value */
  TMRx->CC4 = OC_Config->Pulse;

  /* Write to TMRx CCEN */
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Slave Timer configuration function
  * @param  htmr TMR handle
  * @param  sSlaveConfig Slave timer configuration
  * @retval None
  */
static DAL_StatusTypeDef TMR_SlaveTimer_SetConfig(TMR_HandleTypeDef *htmr,
                                                  TMR_SlaveConfigTypeDef *sSlaveConfig)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmpsmcr;
  uint32_t tmpccmr1;
  uint32_t tmpccer;

  /* Get the TMRx SMCTRL register value */
  tmpsmcr = htmr->Instance->SMCTRL;

  /* Reset the Trigger Selection Bits */
  tmpsmcr &= ~TMR_SMCTRL_TRGSEL;
  /* Set the Input Trigger source */
  tmpsmcr |= sSlaveConfig->InputTrigger;

  /* Reset the slave mode Bits */
  tmpsmcr &= ~TMR_SMCTRL_SMFSEL;
  /* Set the slave mode */
  tmpsmcr |= sSlaveConfig->SlaveMode;

  /* Write to TMRx SMCTRL */
  htmr->Instance->SMCTRL = tmpsmcr;

  /* Configure the trigger prescaler, filter, and polarity */
  switch (sSlaveConfig->InputTrigger)
  {
    case TMR_TS_ETRF:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CLOCKSOURCE_ETRMODE1_INSTANCE(htmr->Instance));
      ASSERT_PARAM(IS_TMR_TRIGGERPRESCALER(sSlaveConfig->TriggerPrescaler));
      ASSERT_PARAM(IS_TMR_TRIGGERPOLARITY(sSlaveConfig->TriggerPolarity));
      ASSERT_PARAM(IS_TMR_TRIGGERFILTER(sSlaveConfig->TriggerFilter));
      /* Configure the ETR Trigger source */
      TMR_ETR_SetConfig(htmr->Instance,
                        sSlaveConfig->TriggerPrescaler,
                        sSlaveConfig->TriggerPolarity,
                        sSlaveConfig->TriggerFilter);
      break;
    }

    case TMR_TS_TI1F_ED:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC1_INSTANCE(htmr->Instance));
      ASSERT_PARAM(IS_TMR_TRIGGERFILTER(sSlaveConfig->TriggerFilter));

      if (sSlaveConfig->SlaveMode == TMR_SLAVEMODE_GATED)
      {
        return DAL_ERROR;
      }

      /* Disable the Channel 1: Reset the CC1EN Bit */
      tmpccer = htmr->Instance->CCEN;
      htmr->Instance->CCEN &= ~TMR_CCEN_CC1EN;
      tmpccmr1 = htmr->Instance->CCM1;

      /* Set the filter */
      tmpccmr1 &= ~TMR_CCM1_IC1F;
      tmpccmr1 |= ((sSlaveConfig->TriggerFilter) << 4U);

      /* Write to TMRx CCM1 and CCEN registers */
      htmr->Instance->CCM1 = tmpccmr1;
      htmr->Instance->CCEN = tmpccer;
      break;
    }

    case TMR_TS_TI1FP1:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC1_INSTANCE(htmr->Instance));
      ASSERT_PARAM(IS_TMR_TRIGGERPOLARITY(sSlaveConfig->TriggerPolarity));
      ASSERT_PARAM(IS_TMR_TRIGGERFILTER(sSlaveConfig->TriggerFilter));

      /* Configure TI1 Filter and Polarity */
      TMR_TI1_ConfigInputStage(htmr->Instance,
                               sSlaveConfig->TriggerPolarity,
                               sSlaveConfig->TriggerFilter);
      break;
    }

    case TMR_TS_TI2FP2:
    {
      /* Check the parameters */
      ASSERT_PARAM(IS_TMR_CC2_INSTANCE(htmr->Instance));
      ASSERT_PARAM(IS_TMR_TRIGGERPOLARITY(sSlaveConfig->TriggerPolarity));
      ASSERT_PARAM(IS_TMR_TRIGGERFILTER(sSlaveConfig->TriggerFilter));

      /* Configure TI2 Filter and Polarity */
      TMR_TI2_ConfigInputStage(htmr->Instance,
                               sSlaveConfig->TriggerPolarity,
                               sSlaveConfig->TriggerFilter);
      break;
    }

    case TMR_TS_ITR0:
    case TMR_TS_ITR1:
    case TMR_TS_ITR2:
    case TMR_TS_ITR3:
    {
      /* Check the parameter */
      ASSERT_PARAM(IS_TMR_CC2_INSTANCE(htmr->Instance));
      break;
    }

    default:
      status = DAL_ERROR;
      break;
  }

  return status;
}

/**
  * @brief  Configure the TI1 as Input.
  * @param  TMRx to select the TMR peripheral.
  * @param  TMR_ICPolarity The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICPOLARITY_RISING
  *            @arg TMR_ICPOLARITY_FALLING
  *            @arg TMR_ICPOLARITY_BOTHEDGE
  * @param  TMR_ICSelection specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICSELECTION_DIRECTTI: TMR Input 1 is selected to be connected to IC1.
  *            @arg TMR_ICSELECTION_INDIRECTTI: TMR Input 1 is selected to be connected to IC2.
  *            @arg TMR_ICSELECTION_TRC: TMR Input 1 is selected to be connected to TRC.
  * @param  TMR_ICFilter Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  * @note TMR_ICFilter and TMR_ICPolarity are not used in INDIRECT mode as TI2FP1
  *       (on channel2 path) is used as the input signal. Therefore CCMR1 must be
  *        protected against un-initialized filter and polarity values.
  */
void TMR_TI1_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICSelection,
                       uint32_t TMR_ICFilter)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;

  /* Disable the Channel 1: Reset the CC1EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC1EN;
  tmpccmr1 = TMRx->CCM1;
  tmpccer = TMRx->CCEN;

  /* Select the Input */
  if (IS_TMR_CC2_INSTANCE(TMRx) != RESET)
  {
    tmpccmr1 &= ~TMR_CCM1_CC1SEL;
    tmpccmr1 |= TMR_ICSelection;
  }
  else
  {
    tmpccmr1 |= TMR_CCM1_CC1SEL_0;
  }

  /* Set the filter */
  tmpccmr1 &= ~TMR_CCM1_IC1F;
  tmpccmr1 |= ((TMR_ICFilter << 4U) & TMR_CCM1_IC1F);

  /* Select the Polarity and set the CC1EN Bit */
  tmpccer &= ~(TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL);
  tmpccer |= (TMR_ICPolarity & (TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL));

  /* Write to TMRx CCM1 and CCEN registers */
  TMRx->CCM1 = tmpccmr1;
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Configure the Polarity and Filter for TI1.
  * @param  TMRx to select the TMR peripheral.
  * @param  TMR_ICPolarity The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICPOLARITY_RISING
  *            @arg TMR_ICPOLARITY_FALLING
  *            @arg TMR_ICPOLARITY_BOTHEDGE
  * @param  TMR_ICFilter Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TMR_TI1_ConfigInputStage(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICFilter)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;

  /* Disable the Channel 1: Reset the CC1EN Bit */
  tmpccer = TMRx->CCEN;
  TMRx->CCEN &= ~TMR_CCEN_CC1EN;
  tmpccmr1 = TMRx->CCM1;

  /* Set the filter */
  tmpccmr1 &= ~TMR_CCM1_IC1F;
  tmpccmr1 |= (TMR_ICFilter << 4U);

  /* Select the Polarity and set the CC1EN Bit */
  tmpccer &= ~(TMR_CCEN_CC1POL | TMR_CCEN_CC1NPOL);
  tmpccer |= TMR_ICPolarity;

  /* Write to TMRx CCM1 and CCEN registers */
  TMRx->CCM1 = tmpccmr1;
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Configure the TI2 as Input.
  * @param  TMRx to select the TMR peripheral
  * @param  TMR_ICPolarity The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICPOLARITY_RISING
  *            @arg TMR_ICPOLARITY_FALLING
  *            @arg TMR_ICPOLARITY_BOTHEDGE
  * @param  TMR_ICSelection specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICSELECTION_DIRECTTI: TMR Input 2 is selected to be connected to IC2.
  *            @arg TMR_ICSELECTION_INDIRECTTI: TMR Input 2 is selected to be connected to IC1.
  *            @arg TMR_ICSELECTION_TRC: TMR Input 2 is selected to be connected to TRC.
  * @param  TMR_ICFilter Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  * @note TMR_ICFilter and TMR_ICPolarity are not used in INDIRECT mode as TI1FP2
  *       (on channel1 path) is used as the input signal. Therefore CCMR1 must be
  *        protected against un-initialized filter and polarity values.
  */
static void TMR_TI2_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICSelection,
                              uint32_t TMR_ICFilter)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;

  /* Disable the Channel 2: Reset the CC2EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC2EN;
  tmpccmr1 = TMRx->CCM1;
  tmpccer = TMRx->CCEN;

  /* Select the Input */
  tmpccmr1 &= ~TMR_CCM1_CC2SEL;
  tmpccmr1 |= (TMR_ICSelection << 8U);

  /* Set the filter */
  tmpccmr1 &= ~TMR_CCM1_IC2F;
  tmpccmr1 |= ((TMR_ICFilter << 12U) & TMR_CCM1_IC2F);

  /* Select the Polarity and set the CC2EN Bit */
  tmpccer &= ~(TMR_CCEN_CC2POL | TMR_CCEN_CC2NPOL);
  tmpccer |= ((TMR_ICPolarity << 4U) & (TMR_CCEN_CC2POL | TMR_CCEN_CC2NPOL));

  /* Write to TMRx CCM1 and CCEN registers */
  TMRx->CCM1 = tmpccmr1 ;
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Configure the Polarity and Filter for TI2.
  * @param  TMRx to select the TMR peripheral.
  * @param  TMR_ICPolarity The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICPOLARITY_RISING
  *            @arg TMR_ICPOLARITY_FALLING
  *            @arg TMR_ICPOLARITY_BOTHEDGE
  * @param  TMR_ICFilter Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
static void TMR_TI2_ConfigInputStage(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICFilter)
{
  uint32_t tmpccmr1;
  uint32_t tmpccer;

  /* Disable the Channel 2: Reset the CC2EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC2EN;
  tmpccmr1 = TMRx->CCM1;
  tmpccer = TMRx->CCEN;

  /* Set the filter */
  tmpccmr1 &= ~TMR_CCM1_IC2F;
  tmpccmr1 |= (TMR_ICFilter << 12U);

  /* Select the Polarity and set the CC2EN Bit */
  tmpccer &= ~(TMR_CCEN_CC2POL | TMR_CCEN_CC2NPOL);
  tmpccer |= (TMR_ICPolarity << 4U);

  /* Write to TMRx CCM1 and CCEN registers */
  TMRx->CCM1 = tmpccmr1 ;
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Configure the TI3 as Input.
  * @param  TMRx to select the TMR peripheral
  * @param  TMR_ICPolarity The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICPOLARITY_RISING
  *            @arg TMR_ICPOLARITY_FALLING
  *            @arg TMR_ICPOLARITY_BOTHEDGE
  * @param  TMR_ICSelection specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICSELECTION_DIRECTTI: TMR Input 3 is selected to be connected to IC3.
  *            @arg TMR_ICSELECTION_INDIRECTTI: TMR Input 3 is selected to be connected to IC4.
  *            @arg TMR_ICSELECTION_TRC: TMR Input 3 is selected to be connected to TRC.
  * @param  TMR_ICFilter Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  * @note TMR_ICFilter and TMR_ICPolarity are not used in INDIRECT mode as TI3FP4
  *       (on channel1 path) is used as the input signal. Therefore CCMR2 must be
  *        protected against un-initialized filter and polarity values.
  */
static void TMR_TI3_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICSelection,
                              uint32_t TMR_ICFilter)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;

  /* Disable the Channel 3: Reset the CC3EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC3EN;
  tmpccmr2 = TMRx->CCM2;
  tmpccer = TMRx->CCEN;

  /* Select the Input */
  tmpccmr2 &= ~TMR_CCM2_CC3SEL;
  tmpccmr2 |= TMR_ICSelection;

  /* Set the filter */
  tmpccmr2 &= ~TMR_CCM2_IC3F;
  tmpccmr2 |= ((TMR_ICFilter << 4U) & TMR_CCM2_IC3F);

  /* Select the Polarity and set the CC3EN Bit */
  tmpccer &= ~(TMR_CCEN_CC3POL | TMR_CCEN_CC3NPOL);
  tmpccer |= ((TMR_ICPolarity << 8U) & (TMR_CCEN_CC3POL | TMR_CCEN_CC3NPOL));

  /* Write to TMRx CCM2 and CCEN registers */
  TMRx->CCM2 = tmpccmr2;
  TMRx->CCEN = tmpccer;
}

/**
  * @brief  Configure the TI4 as Input.
  * @param  TMRx to select the TMR peripheral
  * @param  TMR_ICPolarity The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICPOLARITY_RISING
  *            @arg TMR_ICPOLARITY_FALLING
  *            @arg TMR_ICPOLARITY_BOTHEDGE
  * @param  TMR_ICSelection specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TMR_ICSELECTION_DIRECTTI: TMR Input 4 is selected to be connected to IC4.
  *            @arg TMR_ICSELECTION_INDIRECTTI: TMR Input 4 is selected to be connected to IC3.
  *            @arg TMR_ICSELECTION_TRC: TMR Input 4 is selected to be connected to TRC.
  * @param  TMR_ICFilter Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @note TMR_ICFilter and TMR_ICPolarity are not used in INDIRECT mode as TI4FP3
  *       (on channel1 path) is used as the input signal. Therefore CCMR2 must be
  *        protected against un-initialized filter and polarity values.
  * @retval None
  */
static void TMR_TI4_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ICPolarity, uint32_t TMR_ICSelection,
                              uint32_t TMR_ICFilter)
{
  uint32_t tmpccmr2;
  uint32_t tmpccer;

  /* Disable the Channel 4: Reset the CC4EN Bit */
  TMRx->CCEN &= ~TMR_CCEN_CC4EN;
  tmpccmr2 = TMRx->CCM2;
  tmpccer = TMRx->CCEN;

  /* Select the Input */
  tmpccmr2 &= ~TMR_CCM2_CC4SEL;
  tmpccmr2 |= (TMR_ICSelection << 8U);

  /* Set the filter */
  tmpccmr2 &= ~TMR_CCM2_IC4F;
  tmpccmr2 |= ((TMR_ICFilter << 12U) & TMR_CCM2_IC4F);

  /* Select the Polarity and set the CC4EN Bit */
  tmpccer &= ~(TMR_CCEN_CC4POL | TMR_CCEN_CC4NPOL);
  tmpccer |= ((TMR_ICPolarity << 12U) & (TMR_CCEN_CC4POL | TMR_CCEN_CC4NPOL));

  /* Write to TMRx CCM2 and CCEN registers */
  TMRx->CCM2 = tmpccmr2;
  TMRx->CCEN = tmpccer ;
}

/**
  * @brief  Selects the Input Trigger source
  * @param  TMRx to select the TMR peripheral
  * @param  InputTriggerSource The Input Trigger source.
  *          This parameter can be one of the following values:
  *            @arg TMR_TS_ITR0: Internal Trigger 0
  *            @arg TMR_TS_ITR1: Internal Trigger 1
  *            @arg TMR_TS_ITR2: Internal Trigger 2
  *            @arg TMR_TS_ITR3: Internal Trigger 3
  *            @arg TMR_TS_TI1F_ED: TI1 Edge Detector
  *            @arg TMR_TS_TI1FP1: Filtered Timer Input 1
  *            @arg TMR_TS_TI2FP2: Filtered Timer Input 2
  *            @arg TMR_TS_ETRF: External Trigger input
  * @retval None
  */
static void TMR_ITRx_SetConfig(TMR_TypeDef *TMRx, uint32_t InputTriggerSource)
{
  uint32_t tmpsmcr;

  /* Get the TMRx SMCTRL register value */
  tmpsmcr = TMRx->SMCTRL;
  /* Reset the TS Bits */
  tmpsmcr &= ~TMR_SMCTRL_TRGSEL;
  /* Set the Input Trigger source and the slave mode*/
  tmpsmcr |= (InputTriggerSource | TMR_SLAVEMODE_EXTERNAL1);
  /* Write to TMRx SMCTRL */
  TMRx->SMCTRL = tmpsmcr;
}
/**
  * @brief  Configures the TMRx External Trigger (ETR).
  * @param  TMRx to select the TMR peripheral
  * @param  TMR_ExtTRGPrescaler The external Trigger Prescaler.
  *          This parameter can be one of the following values:
  *            @arg TMR_ETRPRESCALER_DIV1: ETRP Prescaler OFF.
  *            @arg TMR_ETRPRESCALER_DIV2: ETRP frequency divided by 2.
  *            @arg TMR_ETRPRESCALER_DIV4: ETRP frequency divided by 4.
  *            @arg TMR_ETRPRESCALER_DIV8: ETRP frequency divided by 8.
  * @param  TMR_ExtTRGPolarity The external Trigger Polarity.
  *          This parameter can be one of the following values:
  *            @arg TMR_ETRPOLARITY_INVERTED: active low or falling edge active.
  *            @arg TMR_ETRPOLARITY_NONINVERTED: active high or rising edge active.
  * @param  ExtTRGFilter External Trigger Filter.
  *          This parameter must be a value between 0x00 and 0x0F
  * @retval None
  */
void TMR_ETR_SetConfig(TMR_TypeDef *TMRx, uint32_t TMR_ExtTRGPrescaler,
                       uint32_t TMR_ExtTRGPolarity, uint32_t ExtTRGFilter)
{
  uint32_t tmpsmcr;

  tmpsmcr = TMRx->SMCTRL;

  /* Reset the ETR Bits */
  tmpsmcr &= ~(TMR_SMCTRL_ETFCFG | TMR_SMCTRL_ETPCFG | TMR_SMCTRL_ECEN | TMR_SMCTRL_ETPOL);

  /* Set the Prescaler, the Filter value and the Polarity */
  tmpsmcr |= (uint32_t)(TMR_ExtTRGPrescaler | (TMR_ExtTRGPolarity | (ExtTRGFilter << 8U)));

  /* Write to TMRx SMCTRL */
  TMRx->SMCTRL = tmpsmcr;
}

/**
  * @brief  Enables or disables the TMR Capture Compare Channel x.
  * @param  TMRx to select the TMR peripheral
  * @param  Channel specifies the TMR Channel
  *          This parameter can be one of the following values:
  *            @arg TMR_CHANNEL_1: TMR Channel 1
  *            @arg TMR_CHANNEL_2: TMR Channel 2
  *            @arg TMR_CHANNEL_3: TMR Channel 3
  *            @arg TMR_CHANNEL_4: TMR Channel 4
  * @param  ChannelState specifies the TMR Channel CCxE bit new state.
  *          This parameter can be: TMR_CCx_ENABLE or TMR_CCx_DISABLE.
  * @retval None
  */
void TMR_CCxChannelCmd(TMR_TypeDef *TMRx, uint32_t Channel, uint32_t ChannelState)
{
  uint32_t tmp;

  /* Check the parameters */
  ASSERT_PARAM(IS_TMR_CC1_INSTANCE(TMRx));
  ASSERT_PARAM(IS_TMR_CHANNELS(Channel));

  tmp = TMR_CCEN_CC1EN << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxEN Bit */
  TMRx->CCEN &= ~tmp;

  /* Set or reset the CCxEN Bit */
  TMRx->CCEN |= (uint32_t)(ChannelState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}

#if (USE_DAL_TMR_REGISTER_CALLBACKS == 1)
/**
  * @brief  Reset interrupt callbacks to the legacy weak callbacks.
  * @param  htmr pointer to a TMR_HandleTypeDef structure that contains
  *                the configuration information for TMR module.
  * @retval None
  */
void TMR_ResetCallback(TMR_HandleTypeDef *htmr)
{
  /* Reset the TMR callback to the legacy weak callbacks */
  htmr->PeriodElapsedCallback             = DAL_TMR_PeriodElapsedCallback;
  htmr->PeriodElapsedHalfCpltCallback     = DAL_TMR_PeriodElapsedHalfCpltCallback;
  htmr->TriggerCallback                   = DAL_TMR_TriggerCallback;
  htmr->TriggerHalfCpltCallback           = DAL_TMR_TriggerHalfCpltCallback;
  htmr->IC_CaptureCallback                = DAL_TMR_IC_CaptureCallback;
  htmr->IC_CaptureHalfCpltCallback        = DAL_TMR_IC_CaptureHalfCpltCallback;
  htmr->OC_DelayElapsedCallback           = DAL_TMR_OC_DelayElapsedCallback;
  htmr->PWM_PulseFinishedCallback         = DAL_TMR_PWM_PulseFinishedCallback;
  htmr->PWM_PulseFinishedHalfCpltCallback = DAL_TMR_PWM_PulseFinishedHalfCpltCallback;
  htmr->ErrorCallback                     = DAL_TMR_ErrorCallback;
  htmr->CommutationCallback               = DAL_TMREx_CommutCallback;
  htmr->CommutationHalfCpltCallback       = DAL_TMREx_CommutHalfCpltCallback;
  htmr->BreakCallback                     = DAL_TMREx_BreakCallback;
}
#endif /* USE_DAL_TMR_REGISTER_CALLBACKS */

/**
  * @}
  */

#endif /* DAL_TMR_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
