/**
  *
  * @file    apm32f4xx_dal_can.c
  * @brief   CAN DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Controller Area Network (CAN) peripheral:
  *           + Initialization and de-initialization functions
  *           + Configuration functions
  *           + Control functions
  *           + Interrupts management
  *           + Callbacks functions
  *           + Peripheral State and Error functions
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
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) Initialize the CAN low level resources by implementing the
          DAL_CAN_MspInit():
         (++) Enable the CAN interface clock using __DAL_RCM_CANx_CLK_ENABLE()
         (++) Configure CAN pins
             (+++) Enable the clock for the CAN GPIOs
             (+++) Configure CAN pins as alternate function open-drain
         (++) In case of using interrupts (e.g. DAL_CAN_ActivateNotification())
             (+++) Configure the CAN interrupt priority using
                   DAL_NVIC_SetPriority()
             (+++) Enable the CAN IRQ handler using DAL_NVIC_EnableIRQ()
             (+++) In CAN IRQ handler, call DAL_CAN_IRQHandler()

      (#) Initialize the CAN peripheral using DAL_CAN_Init() function. This
          function resorts to DAL_CAN_MspInit() for low-level initialization.

      (#) Configure the reception filters using the following configuration
          functions:
            (++) DAL_CAN_ConfigFilter()

      (#) Start the CAN module using DAL_CAN_Start() function. At this level
          the node is active on the bus: it receive messages, and can send
          messages.

      (#) To manage messages transmission, the following Tx control functions
          can be used:
            (++) DAL_CAN_AddTxMessage() to request transmission of a new
                 message.
            (++) DAL_CAN_AbortTxRequest() to abort transmission of a pending
                 message.
            (++) DAL_CAN_GetTxMailboxesFreeLevel() to get the number of free Tx
                 mailboxes.
            (++) DAL_CAN_IsTxMessagePending() to check if a message is pending
                 in a Tx mailbox.
            (++) DAL_CAN_GetTxTimestamp() to get the timestamp of Tx message
                 sent, if time triggered communication mode is enabled.

      (#) When a message is received into the CAN Rx FIFOs, it can be retrieved
          using the DAL_CAN_GetRxMessage() function. The function
          DAL_CAN_GetRxFifoFillLevel() allows to know how many Rx message are
          stored in the Rx Fifo.

      (#) Calling the DAL_CAN_Stop() function stops the CAN module.

      (#) The deinitialization is achieved with DAL_CAN_DeInit() function.


      *** Polling mode operation ***
      ==============================
    [..]
      (#) Reception:
            (++) Monitor reception of message using DAL_CAN_GetRxFifoFillLevel()
                 until at least one message is received.
            (++) Then get the message using DAL_CAN_GetRxMessage().

      (#) Transmission:
            (++) Monitor the Tx mailboxes availability until at least one Tx
                 mailbox is free, using DAL_CAN_GetTxMailboxesFreeLevel().
            (++) Then request transmission of a message using
                 DAL_CAN_AddTxMessage().


      *** Interrupt mode operation ***
      ================================
    [..]
      (#) Notifications are activated using DAL_CAN_ActivateNotification()
          function. Then, the process can be controlled through the
          available user callbacks: DAL_CAN_xxxCallback(), using same APIs
          DAL_CAN_GetRxMessage() and DAL_CAN_AddTxMessage().

      (#) Notifications can be deactivated using
          DAL_CAN_DeactivateNotification() function.

      (#) Special care should be taken for CAN_IT_RX_FIFO0_MSG_PENDING and
          CAN_IT_RX_FIFO1_MSG_PENDING notifications. These notifications trig
          the callbacks DAL_CAN_RxFIFO0MsgPendingCallback() and
          DAL_CAN_RxFIFO1MsgPendingCallback(). User has two possible options
          here.
            (++) Directly get the Rx message in the callback, using
                 DAL_CAN_GetRxMessage().
            (++) Or deactivate the notification in the callback without
                 getting the Rx message. The Rx message can then be got later
                 using DAL_CAN_GetRxMessage(). Once the Rx message have been
                 read, the notification can be activated again.


      *** Sleep mode ***
      ==================
    [..]
      (#) The CAN peripheral can be put in sleep mode (low power), using
          DAL_CAN_RequestSleep(). The sleep mode will be entered as soon as the
          current CAN activity (transmission or reception of a CAN frame) will
          be completed.

      (#) A notification can be activated to be informed when the sleep mode
          will be entered.

      (#) It can be checked if the sleep mode is entered using
          DAL_CAN_IsSleepActive().
          Note that the CAN state (accessible from the API DAL_CAN_GetState())
          is DAL_CAN_STATE_SLEEP_PENDING as soon as the sleep mode request is
          submitted (the sleep mode is not yet entered), and become
          DAL_CAN_STATE_SLEEP_ACTIVE when the sleep mode is effective.

      (#) The wake-up from sleep mode can be triggered by two ways:
            (++) Using DAL_CAN_WakeUp(). When returning from this function,
                 the sleep mode is exited (if return status is DAL_OK).
            (++) When a start of Rx CAN frame is detected by the CAN peripheral,
                 if automatic wake up mode is enabled.

  *** Callback registration ***
  =============================================

  The compilation define  USE_DAL_CAN_REGISTER_CALLBACKS when set to 1
  allows the user to configure dynamically the driver callbacks.
  Use Function DAL_CAN_RegisterCallback() to register an interrupt callback.

  Function DAL_CAN_RegisterCallback() allows to register following callbacks:
    (+) TxMailbox0CompleteCallback   : Tx Mailbox 0 Complete Callback.
    (+) TxMailbox1CompleteCallback   : Tx Mailbox 1 Complete Callback.
    (+) TxMailbox2CompleteCallback   : Tx Mailbox 2 Complete Callback.
    (+) TxMailbox0AbortCallback      : Tx Mailbox 0 Abort Callback.
    (+) TxMailbox1AbortCallback      : Tx Mailbox 1 Abort Callback.
    (+) TxMailbox2AbortCallback      : Tx Mailbox 2 Abort Callback.
    (+) RxFifo0MsgPendingCallback    : Rx Fifo 0 Message Pending Callback.
    (+) RxFifo0FullCallback          : Rx Fifo 0 Full Callback.
    (+) RxFifo1MsgPendingCallback    : Rx Fifo 1 Message Pending Callback.
    (+) RxFifo1FullCallback          : Rx Fifo 1 Full Callback.
    (+) SleepCallback                : Sleep Callback.
    (+) WakeUpFromRxMsgCallback      : Wake Up From Rx Message Callback.
    (+) ErrorCallback                : Error Callback.
    (+) MspInitCallback              : CAN MspInit.
    (+) MspDeInitCallback            : CAN MspDeInit.
  This function takes as parameters the DAL peripheral handle, the Callback ID
  and a pointer to the user callback function.

  Use function DAL_CAN_UnRegisterCallback() to reset a callback to the default
  weak function.
  DAL_CAN_UnRegisterCallback takes as parameters the DAL peripheral handle,
  and the Callback ID.
  This function allows to reset following callbacks:
    (+) TxMailbox0CompleteCallback   : Tx Mailbox 0 Complete Callback.
    (+) TxMailbox1CompleteCallback   : Tx Mailbox 1 Complete Callback.
    (+) TxMailbox2CompleteCallback   : Tx Mailbox 2 Complete Callback.
    (+) TxMailbox0AbortCallback      : Tx Mailbox 0 Abort Callback.
    (+) TxMailbox1AbortCallback      : Tx Mailbox 1 Abort Callback.
    (+) TxMailbox2AbortCallback      : Tx Mailbox 2 Abort Callback.
    (+) RxFifo0MsgPendingCallback    : Rx Fifo 0 Message Pending Callback.
    (+) RxFifo0FullCallback          : Rx Fifo 0 Full Callback.
    (+) RxFifo1MsgPendingCallback    : Rx Fifo 1 Message Pending Callback.
    (+) RxFifo1FullCallback          : Rx Fifo 1 Full Callback.
    (+) SleepCallback                : Sleep Callback.
    (+) WakeUpFromRxMsgCallback      : Wake Up From Rx Message Callback.
    (+) ErrorCallback                : Error Callback.
    (+) MspInitCallback              : CAN MspInit.
    (+) MspDeInitCallback            : CAN MspDeInit.

  By default, after the DAL_CAN_Init() and when the state is DAL_CAN_STATE_RESET,
  all callbacks are set to the corresponding weak functions:
  example DAL_CAN_ErrorCallback().
  Exception done for MspInit and MspDeInit functions that are
  reset to the legacy weak function in the DAL_CAN_Init()/ DAL_CAN_DeInit() only when
  these callbacks are null (not registered beforehand).
  if not, MspInit or MspDeInit are not null, the DAL_CAN_Init()/ DAL_CAN_DeInit()
  keep and use the user MspInit/MspDeInit callbacks (registered beforehand)

  Callbacks can be registered/unregistered in DAL_CAN_STATE_READY state only.
  Exception done MspInit/MspDeInit that can be registered/unregistered
  in DAL_CAN_STATE_READY or DAL_CAN_STATE_RESET state,
  thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
  In that case first register the MspInit/MspDeInit user callbacks
  using DAL_CAN_RegisterCallback() before calling DAL_CAN_DeInit()
  or DAL_CAN_Init() function.

  When The compilation define USE_DAL_CAN_REGISTER_CALLBACKS is set to 0 or
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

#if defined(CAN1)

/** @defgroup CAN CAN
  * @brief CAN driver modules
  * @{
  */

#ifdef DAL_CAN_MODULE_ENABLED

#ifdef DAL_CAN_LEGACY_MODULE_ENABLED
  #error "The CAN driver cannot be used with its legacy, Please enable only one CAN module at once"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup CAN_Private_Constants CAN Private Constants
  * @{
  */
#define CAN_TIMEOUT_VALUE 10U
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup CAN_Exported_Functions CAN Exported Functions
  * @{
  */

/** @defgroup CAN_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
  ==============================================================================
              ##### Initialization and de-initialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) DAL_CAN_Init                       : Initialize and configure the CAN.
      (+) DAL_CAN_DeInit                     : De-initialize the CAN.
      (+) DAL_CAN_MspInit                    : Initialize the CAN MSP.
      (+) DAL_CAN_MspDeInit                  : DeInitialize the CAN MSP.

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the CAN peripheral according to the specified
  *         parameters in the CAN_InitStruct.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_Init(CAN_HandleTypeDef *hcan)
{
  uint32_t tickstart;

  /* Check CAN handle */
  if (hcan == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_CAN_ALL_INSTANCE(hcan->Instance));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(hcan->Init.TimeTriggeredMode));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(hcan->Init.AutoBusOff));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(hcan->Init.AutoWakeUp));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(hcan->Init.AutoRetransmission));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(hcan->Init.ReceiveFifoLocked));
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(hcan->Init.TransmitFifoPriority));
  ASSERT_PARAM(IS_CAN_MODE(hcan->Init.Mode));
  ASSERT_PARAM(IS_CAN_SJW(hcan->Init.SyncJumpWidth));
  ASSERT_PARAM(IS_CAN_BS1(hcan->Init.TimeSeg1));
  ASSERT_PARAM(IS_CAN_BS2(hcan->Init.TimeSeg2));
  ASSERT_PARAM(IS_CAN_PRESCALER(hcan->Init.Prescaler));

#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
  if (hcan->State == DAL_CAN_STATE_RESET)
  {
    /* Reset callbacks to legacy functions */
    hcan->RxFifo0MsgPendingCallback  =  DAL_CAN_RxFifo0MsgPendingCallback;  /* Legacy weak RxFifo0MsgPendingCallback  */
    hcan->RxFifo0FullCallback        =  DAL_CAN_RxFifo0FullCallback;        /* Legacy weak RxFifo0FullCallback        */
    hcan->RxFifo1MsgPendingCallback  =  DAL_CAN_RxFifo1MsgPendingCallback;  /* Legacy weak RxFifo1MsgPendingCallback  */
    hcan->RxFifo1FullCallback        =  DAL_CAN_RxFifo1FullCallback;        /* Legacy weak RxFifo1FullCallback        */
    hcan->TxMailbox0CompleteCallback =  DAL_CAN_TxMailbox0CompleteCallback; /* Legacy weak TxMailbox0CompleteCallback */
    hcan->TxMailbox1CompleteCallback =  DAL_CAN_TxMailbox1CompleteCallback; /* Legacy weak TxMailbox1CompleteCallback */
    hcan->TxMailbox2CompleteCallback =  DAL_CAN_TxMailbox2CompleteCallback; /* Legacy weak TxMailbox2CompleteCallback */
    hcan->TxMailbox0AbortCallback    =  DAL_CAN_TxMailbox0AbortCallback;    /* Legacy weak TxMailbox0AbortCallback    */
    hcan->TxMailbox1AbortCallback    =  DAL_CAN_TxMailbox1AbortCallback;    /* Legacy weak TxMailbox1AbortCallback    */
    hcan->TxMailbox2AbortCallback    =  DAL_CAN_TxMailbox2AbortCallback;    /* Legacy weak TxMailbox2AbortCallback    */
    hcan->SleepCallback              =  DAL_CAN_SleepCallback;              /* Legacy weak SleepCallback              */
    hcan->WakeUpFromRxMsgCallback    =  DAL_CAN_WakeUpFromRxMsgCallback;    /* Legacy weak WakeUpFromRxMsgCallback    */
    hcan->ErrorCallback              =  DAL_CAN_ErrorCallback;              /* Legacy weak ErrorCallback              */

    if (hcan->MspInitCallback == NULL)
    {
      hcan->MspInitCallback = DAL_CAN_MspInit; /* Legacy weak MspInit */
    }

    /* Init the low level hardware: CLOCK, NVIC */
    hcan->MspInitCallback(hcan);
  }

#else
  if (hcan->State == DAL_CAN_STATE_RESET)
  {
    /* Init the low level hardware: CLOCK, NVIC */
    DAL_CAN_MspInit(hcan);
  }
#endif /* (USE_DAL_CAN_REGISTER_CALLBACKS) */

  /* Request initialisation */
  SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_INITREQ);

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait initialisation acknowledge */
  while ((hcan->Instance->MSTS & CAN_MSTS_INITFLG) == 0U)
  {
    if ((DAL_GetTick() - tickstart) > CAN_TIMEOUT_VALUE)
    {
      /* Update error code */
      hcan->ErrorCode |= DAL_CAN_ERROR_TIMEOUT;

      /* Change CAN state */
      hcan->State = DAL_CAN_STATE_ERROR;

      return DAL_ERROR;
    }
  }

  /* Exit from sleep mode */
  CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_SLEEPREQ);

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Check Sleep mode leave acknowledge */
  while ((hcan->Instance->MSTS & CAN_MSTS_SLEEPFLG) != 0U)
  {
    if ((DAL_GetTick() - tickstart) > CAN_TIMEOUT_VALUE)
    {
      /* Update error code */
      hcan->ErrorCode |= DAL_CAN_ERROR_TIMEOUT;

      /* Change CAN state */
      hcan->State = DAL_CAN_STATE_ERROR;

      return DAL_ERROR;
    }
  }

  /* Set the time triggered communication mode */
  if (hcan->Init.TimeTriggeredMode == ENABLE)
  {
    SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_TTCM);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_TTCM);
  }

  /* Set the automatic bus-off management */
  if (hcan->Init.AutoBusOff == ENABLE)
  {
    SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_ALBOFFM);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_ALBOFFM);
  }

  /* Set the automatic wake-up mode */
  if (hcan->Init.AutoWakeUp == ENABLE)
  {
    SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_AWUPCFG);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_AWUPCFG);
  }

  /* Set the automatic retransmission */
  if (hcan->Init.AutoRetransmission == ENABLE)
  {
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_ARTXMD);
  }
  else
  {
    SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_ARTXMD);
  }

  /* Set the receive FIFO locked mode */
  if (hcan->Init.ReceiveFifoLocked == ENABLE)
  {
    SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_RXFLOCK);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_RXFLOCK);
  }

  /* Set the transmit FIFO priority */
  if (hcan->Init.TransmitFifoPriority == ENABLE)
  {
    SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_TXFPCFG);
  }
  else
  {
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_TXFPCFG);
  }

  /* Set the bit timing register */
  WRITE_REG(hcan->Instance->BITTIM, (uint32_t)(hcan->Init.Mode           |
                                            hcan->Init.SyncJumpWidth  |
                                            hcan->Init.TimeSeg1       |
                                            hcan->Init.TimeSeg2       |
                                            (hcan->Init.Prescaler - 1U)));

  /* Initialize the error code */
  hcan->ErrorCode = DAL_CAN_ERROR_NONE;

  /* Initialize the CAN state */
  hcan->State = DAL_CAN_STATE_READY;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Deinitializes the CAN peripheral registers to their default
  *         reset values.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_DeInit(CAN_HandleTypeDef *hcan)
{
  /* Check CAN handle */
  if (hcan == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_CAN_ALL_INSTANCE(hcan->Instance));

  /* Stop the CAN module */
  (void)DAL_CAN_Stop(hcan);

#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
  if (hcan->MspDeInitCallback == NULL)
  {
    hcan->MspDeInitCallback = DAL_CAN_MspDeInit; /* Legacy weak MspDeInit */
  }

  /* DeInit the low level hardware: CLOCK, NVIC */
  hcan->MspDeInitCallback(hcan);

#else
  /* DeInit the low level hardware: CLOCK, NVIC */
  DAL_CAN_MspDeInit(hcan);
#endif /* (USE_DAL_CAN_REGISTER_CALLBACKS) */

  /* Reset the CAN peripheral */
  SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_SWRST);

  /* Reset the CAN ErrorCode */
  hcan->ErrorCode = DAL_CAN_ERROR_NONE;

  /* Change CAN state */
  hcan->State = DAL_CAN_STATE_RESET;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Initializes the CAN MSP.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes the CAN MSP.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_MspDeInit could be implemented in the user file
   */
}

#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
/**
  * @brief  Register a CAN CallBack.
  *         To be used instead of the weak predefined callback
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for CAN module
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID Tx Mailbox 0 Complete callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID Tx Mailbox 1 Complete callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID Tx Mailbox 2 Complete callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX0_ABORT_CB_ID Tx Mailbox 0 Abort callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX1_ABORT_CB_ID Tx Mailbox 1 Abort callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX2_ABORT_CB_ID Tx Mailbox 2 Abort callback ID
  *           @arg @ref DAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID Rx Fifo 0 message pending callback ID
  *           @arg @ref DAL_CAN_RX_FIFO0_FULL_CB_ID Rx Fifo 0 full callback ID
  *           @arg @ref DAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID Rx Fifo 1 message pending callback ID
  *           @arg @ref DAL_CAN_RX_FIFO1_FULL_CB_ID Rx Fifo 1 full callback ID
  *           @arg @ref DAL_CAN_SLEEP_CB_ID Sleep callback ID
  *           @arg @ref DAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID Wake Up from Rx message callback ID
  *           @arg @ref DAL_CAN_ERROR_CB_ID Error callback ID
  *           @arg @ref DAL_CAN_MSPINIT_CB_ID MspInit callback ID
  *           @arg @ref DAL_CAN_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_RegisterCallback(CAN_HandleTypeDef *hcan, DAL_CAN_CallbackIDTypeDef CallbackID, void (* pCallback)(CAN_HandleTypeDef *_hcan))
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  if (hcan->State == DAL_CAN_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID :
        hcan->TxMailbox0CompleteCallback = pCallback;
        break;

      case DAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID :
        hcan->TxMailbox1CompleteCallback = pCallback;
        break;

      case DAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID :
        hcan->TxMailbox2CompleteCallback = pCallback;
        break;

      case DAL_CAN_TX_MAILBOX0_ABORT_CB_ID :
        hcan->TxMailbox0AbortCallback = pCallback;
        break;

      case DAL_CAN_TX_MAILBOX1_ABORT_CB_ID :
        hcan->TxMailbox1AbortCallback = pCallback;
        break;

      case DAL_CAN_TX_MAILBOX2_ABORT_CB_ID :
        hcan->TxMailbox2AbortCallback = pCallback;
        break;

      case DAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID :
        hcan->RxFifo0MsgPendingCallback = pCallback;
        break;

      case DAL_CAN_RX_FIFO0_FULL_CB_ID :
        hcan->RxFifo0FullCallback = pCallback;
        break;

      case DAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID :
        hcan->RxFifo1MsgPendingCallback = pCallback;
        break;

      case DAL_CAN_RX_FIFO1_FULL_CB_ID :
        hcan->RxFifo1FullCallback = pCallback;
        break;

      case DAL_CAN_SLEEP_CB_ID :
        hcan->SleepCallback = pCallback;
        break;

      case DAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID :
        hcan->WakeUpFromRxMsgCallback = pCallback;
        break;

      case DAL_CAN_ERROR_CB_ID :
        hcan->ErrorCallback = pCallback;
        break;

      case DAL_CAN_MSPINIT_CB_ID :
        hcan->MspInitCallback = pCallback;
        break;

      case DAL_CAN_MSPDEINIT_CB_ID :
        hcan->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hcan->State == DAL_CAN_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_CAN_MSPINIT_CB_ID :
        hcan->MspInitCallback = pCallback;
        break;

      case DAL_CAN_MSPDEINIT_CB_ID :
        hcan->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  return status;
}

/**
  * @brief  Unregister a CAN CallBack.
  *         CAN callback is redirected to the weak predefined callback
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for CAN module
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID Tx Mailbox 0 Complete callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID Tx Mailbox 1 Complete callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID Tx Mailbox 2 Complete callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX0_ABORT_CB_ID Tx Mailbox 0 Abort callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX1_ABORT_CB_ID Tx Mailbox 1 Abort callback ID
  *           @arg @ref DAL_CAN_TX_MAILBOX2_ABORT_CB_ID Tx Mailbox 2 Abort callback ID
  *           @arg @ref DAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID Rx Fifo 0 message pending callback ID
  *           @arg @ref DAL_CAN_RX_FIFO0_FULL_CB_ID Rx Fifo 0 full callback ID
  *           @arg @ref DAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID Rx Fifo 1 message pending callback ID
  *           @arg @ref DAL_CAN_RX_FIFO1_FULL_CB_ID Rx Fifo 1 full callback ID
  *           @arg @ref DAL_CAN_SLEEP_CB_ID Sleep callback ID
  *           @arg @ref DAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID Wake Up from Rx message callback ID
  *           @arg @ref DAL_CAN_ERROR_CB_ID Error callback ID
  *           @arg @ref DAL_CAN_MSPINIT_CB_ID MspInit callback ID
  *           @arg @ref DAL_CAN_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_UnRegisterCallback(CAN_HandleTypeDef *hcan, DAL_CAN_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (hcan->State == DAL_CAN_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID :
        hcan->TxMailbox0CompleteCallback = DAL_CAN_TxMailbox0CompleteCallback;
        break;

      case DAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID :
        hcan->TxMailbox1CompleteCallback = DAL_CAN_TxMailbox1CompleteCallback;
        break;

      case DAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID :
        hcan->TxMailbox2CompleteCallback = DAL_CAN_TxMailbox2CompleteCallback;
        break;

      case DAL_CAN_TX_MAILBOX0_ABORT_CB_ID :
        hcan->TxMailbox0AbortCallback = DAL_CAN_TxMailbox0AbortCallback;
        break;

      case DAL_CAN_TX_MAILBOX1_ABORT_CB_ID :
        hcan->TxMailbox1AbortCallback = DAL_CAN_TxMailbox1AbortCallback;
        break;

      case DAL_CAN_TX_MAILBOX2_ABORT_CB_ID :
        hcan->TxMailbox2AbortCallback = DAL_CAN_TxMailbox2AbortCallback;
        break;

      case DAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID :
        hcan->RxFifo0MsgPendingCallback = DAL_CAN_RxFifo0MsgPendingCallback;
        break;

      case DAL_CAN_RX_FIFO0_FULL_CB_ID :
        hcan->RxFifo0FullCallback = DAL_CAN_RxFifo0FullCallback;
        break;

      case DAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID :
        hcan->RxFifo1MsgPendingCallback = DAL_CAN_RxFifo1MsgPendingCallback;
        break;

      case DAL_CAN_RX_FIFO1_FULL_CB_ID :
        hcan->RxFifo1FullCallback = DAL_CAN_RxFifo1FullCallback;
        break;

      case DAL_CAN_SLEEP_CB_ID :
        hcan->SleepCallback = DAL_CAN_SleepCallback;
        break;

      case DAL_CAN_WAKEUP_FROM_RX_MSG_CB_ID :
        hcan->WakeUpFromRxMsgCallback = DAL_CAN_WakeUpFromRxMsgCallback;
        break;

      case DAL_CAN_ERROR_CB_ID :
        hcan->ErrorCallback = DAL_CAN_ErrorCallback;
        break;

      case DAL_CAN_MSPINIT_CB_ID :
        hcan->MspInitCallback = DAL_CAN_MspInit;
        break;

      case DAL_CAN_MSPDEINIT_CB_ID :
        hcan->MspDeInitCallback = DAL_CAN_MspDeInit;
        break;

      default :
        /* Update the error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hcan->State == DAL_CAN_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_CAN_MSPINIT_CB_ID :
        hcan->MspInitCallback = DAL_CAN_MspInit;
        break;

      case DAL_CAN_MSPDEINIT_CB_ID :
        hcan->MspDeInitCallback = DAL_CAN_MspDeInit;
        break;

      default :
        /* Update the error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  return status;
}
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group2 Configuration functions
 *  @brief    Configuration functions.
 *
@verbatim
  ==============================================================================
              ##### Configuration functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) DAL_CAN_ConfigFilter            : Configure the CAN reception filters

@endverbatim
  * @{
  */

/**
  * @brief  Configures the CAN reception filter according to the specified
  *         parameters in the CAN_FilterInitStruct.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  sFilterConfig pointer to a CAN_FilterTypeDef structure that
  *         contains the filter configuration information.
  * @retval None
  */
DAL_StatusTypeDef DAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig)
{
  uint32_t filternbrbitpos;
  CAN_TypeDef *can_ip = hcan->Instance;
  DAL_CAN_StateTypeDef state = hcan->State;

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Check the parameters */
    ASSERT_PARAM(IS_CAN_FILTER_ID_HALFWORD(sFilterConfig->FilterIdHigh));
    ASSERT_PARAM(IS_CAN_FILTER_ID_HALFWORD(sFilterConfig->FilterIdLow));
    ASSERT_PARAM(IS_CAN_FILTER_ID_HALFWORD(sFilterConfig->FilterMaskIdHigh));
    ASSERT_PARAM(IS_CAN_FILTER_ID_HALFWORD(sFilterConfig->FilterMaskIdLow));
    ASSERT_PARAM(IS_CAN_FILTER_MODE(sFilterConfig->FilterMode));
    ASSERT_PARAM(IS_CAN_FILTER_SCALE(sFilterConfig->FilterScale));
    ASSERT_PARAM(IS_CAN_FILTER_FIFO(sFilterConfig->FilterFIFOAssignment));
    ASSERT_PARAM(IS_CAN_FILTER_ACTIVATION(sFilterConfig->FilterActivation));

#if defined(CAN3)
    /* Check the CAN instance */
    if (hcan->Instance == CAN3)
    {
      /* CAN3 is single instance with 14 dedicated filters banks */

      /* Check the parameters */
      ASSERT_PARAM(IS_CAN_FILTER_BANK_SINGLE(sFilterConfig->FilterBank));
    }
    else
    {
      /* CAN1 and CAN2 are dual instances with 28 common filters banks */
      /* Select master instance to access the filter banks */
      can_ip = CAN1;

      /* Check the parameters */
      ASSERT_PARAM(IS_CAN_FILTER_BANK_DUAL(sFilterConfig->FilterBank));
      ASSERT_PARAM(IS_CAN_FILTER_BANK_DUAL(sFilterConfig->SlaveStartFilterBank));
    }
#elif defined(CAN2)
    /* CAN1 and CAN2 are dual instances with 28 common filters banks */
    /* Select master instance to access the filter banks */
    can_ip = CAN1;

    /* Check the parameters */
    ASSERT_PARAM(IS_CAN_FILTER_BANK_DUAL(sFilterConfig->FilterBank));
    ASSERT_PARAM(IS_CAN_FILTER_BANK_DUAL(sFilterConfig->SlaveStartFilterBank));
#else
    /* CAN1 is single instance with 14 dedicated filters banks */

    /* Check the parameters */
    ASSERT_PARAM(IS_CAN_FILTER_BANK_SINGLE(sFilterConfig->FilterBank));
#endif

    /* Initialisation mode for the filter */
    SET_BIT(can_ip->FCTRL, CAN_FCTRL_FINIT);

#if defined(CAN3)
    /* Check the CAN instance */
    if (can_ip == CAN1)
    {
      /* Select the start filter number of CAN2 slave instance */
      CLEAR_BIT(can_ip->FCTRL, CAN_FCTRL_CAN2SB);
      SET_BIT(can_ip->FCTRL, sFilterConfig->SlaveStartFilterBank << CAN_FCTRL_CAN2SB_Pos);
    }

#elif defined(CAN2)
    /* Select the start filter number of CAN2 slave instance */
    CLEAR_BIT(can_ip->FCTRL, CAN_FCTRL_CAN2SB);
    SET_BIT(can_ip->FCTRL, sFilterConfig->SlaveStartFilterBank << CAN_FCTRL_CAN2SB_Pos);

#endif
    /* Convert filter number into bit position */
    filternbrbitpos = (uint32_t)1 << (sFilterConfig->FilterBank & 0x1FU);

    /* Filter Deactivation */
    CLEAR_BIT(can_ip->FACT, filternbrbitpos);

    /* Filter Scale */
    if (sFilterConfig->FilterScale == CAN_FILTERSCALE_16BIT)
    {
      /* 16-bit scale for the filter */
      CLEAR_BIT(can_ip->FSCFG, filternbrbitpos);

      /* First 16-bit identifier and First 16-bit mask */
      /* Or First 16-bit identifier and Second 16-bit identifier */
      can_ip->sFilterRegister[sFilterConfig->FilterBank].FBANK1 =
        ((0x0000FFFFU & (uint32_t)sFilterConfig->FilterMaskIdLow) << 16U) |
        (0x0000FFFFU & (uint32_t)sFilterConfig->FilterIdLow);

      /* Second 16-bit identifier and Second 16-bit mask */
      /* Or Third 16-bit identifier and Fourth 16-bit identifier */
      can_ip->sFilterRegister[sFilterConfig->FilterBank].FBANK2 =
        ((0x0000FFFFU & (uint32_t)sFilterConfig->FilterMaskIdHigh) << 16U) |
        (0x0000FFFFU & (uint32_t)sFilterConfig->FilterIdHigh);
    }

    if (sFilterConfig->FilterScale == CAN_FILTERSCALE_32BIT)
    {
      /* 32-bit scale for the filter */
      SET_BIT(can_ip->FSCFG, filternbrbitpos);

      /* 32-bit identifier or First 32-bit identifier */
      can_ip->sFilterRegister[sFilterConfig->FilterBank].FBANK1 =
        ((0x0000FFFFU & (uint32_t)sFilterConfig->FilterIdHigh) << 16U) |
        (0x0000FFFFU & (uint32_t)sFilterConfig->FilterIdLow);

      /* 32-bit mask or Second 32-bit identifier */
      can_ip->sFilterRegister[sFilterConfig->FilterBank].FBANK2 =
        ((0x0000FFFFU & (uint32_t)sFilterConfig->FilterMaskIdHigh) << 16U) |
        (0x0000FFFFU & (uint32_t)sFilterConfig->FilterMaskIdLow);
    }

    /* Filter Mode */
    if (sFilterConfig->FilterMode == CAN_FILTERMODE_IDMASK)
    {
      /* Id/Mask mode for the filter*/
      CLEAR_BIT(can_ip->FMCFG, filternbrbitpos);
    }
    else /* CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdList */
    {
      /* Identifier list mode for the filter*/
      SET_BIT(can_ip->FMCFG, filternbrbitpos);
    }

    /* Filter FIFO assignment */
    if (sFilterConfig->FilterFIFOAssignment == CAN_FILTER_FIFO0)
    {
      /* FIFO 0 assignation for the filter */
      CLEAR_BIT(can_ip->FFASS, filternbrbitpos);
    }
    else
    {
      /* FIFO 1 assignation for the filter */
      SET_BIT(can_ip->FFASS, filternbrbitpos);
    }

    /* Filter activation */
    if (sFilterConfig->FilterActivation == CAN_FILTER_ENABLE)
    {
      SET_BIT(can_ip->FACT, filternbrbitpos);
    }

    /* Leave the initialisation mode for the filter */
    CLEAR_BIT(can_ip->FCTRL, CAN_FCTRL_FINIT);

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    return DAL_ERROR;
  }
}

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group3 Control functions
 *  @brief    Control functions
 *
@verbatim
  ==============================================================================
                      ##### Control functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) DAL_CAN_Start                    : Start the CAN module
      (+) DAL_CAN_Stop                     : Stop the CAN module
      (+) DAL_CAN_RequestSleep             : Request sleep mode entry.
      (+) DAL_CAN_WakeUp                   : Wake up from sleep mode.
      (+) DAL_CAN_IsSleepActive            : Check is sleep mode is active.
      (+) DAL_CAN_AddTxMessage             : Add a message to the Tx mailboxes
                                             and activate the corresponding
                                             transmission request
      (+) DAL_CAN_AbortTxRequest           : Abort transmission request
      (+) DAL_CAN_GetTxMailboxesFreeLevel  : Return Tx mailboxes free level
      (+) DAL_CAN_IsTxMessagePending       : Check if a transmission request is
                                             pending on the selected Tx mailbox
      (+) DAL_CAN_GetRxMessage             : Get a CAN frame from the Rx FIFO
      (+) DAL_CAN_GetRxFifoFillLevel       : Return Rx FIFO fill level

@endverbatim
  * @{
  */

/**
  * @brief  Start the CAN module.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_Start(CAN_HandleTypeDef *hcan)
{
  uint32_t tickstart;

  if (hcan->State == DAL_CAN_STATE_READY)
  {
    /* Change CAN peripheral state */
    hcan->State = DAL_CAN_STATE_LISTENING;

    /* Request leave initialisation */
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_INITREQ);

    /* Get tick */
    tickstart = DAL_GetTick();

    /* Wait the acknowledge */
    while ((hcan->Instance->MSTS & CAN_MSTS_INITFLG) != 0U)
    {
      /* Check for the Timeout */
      if ((DAL_GetTick() - tickstart) > CAN_TIMEOUT_VALUE)
      {
        /* Update error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_TIMEOUT;

        /* Change CAN state */
        hcan->State = DAL_CAN_STATE_ERROR;

        return DAL_ERROR;
      }
    }

    /* Reset the CAN ErrorCode */
    hcan->ErrorCode = DAL_CAN_ERROR_NONE;

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_READY;

    return DAL_ERROR;
  }
}

/**
  * @brief  Stop the CAN module and enable access to configuration registers.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_Stop(CAN_HandleTypeDef *hcan)
{
  uint32_t tickstart;

  if (hcan->State == DAL_CAN_STATE_LISTENING)
  {
    /* Request initialisation */
    SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_INITREQ);

    /* Get tick */
    tickstart = DAL_GetTick();

    /* Wait the acknowledge */
    while ((hcan->Instance->MSTS & CAN_MSTS_INITFLG) == 0U)
    {
      /* Check for the Timeout */
      if ((DAL_GetTick() - tickstart) > CAN_TIMEOUT_VALUE)
      {
        /* Update error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_TIMEOUT;

        /* Change CAN state */
        hcan->State = DAL_CAN_STATE_ERROR;

        return DAL_ERROR;
      }
    }

    /* Exit from sleep mode */
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_SLEEPREQ);

    /* Change CAN peripheral state */
    hcan->State = DAL_CAN_STATE_READY;

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_STARTED;

    return DAL_ERROR;
  }
}

/**
  * @brief  Request the sleep mode (low power) entry.
  *         When returning from this function, Sleep mode will be entered
  *         as soon as the current CAN activity (transmission or reception
  *         of a CAN frame) has been completed.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval DAL status.
  */
DAL_StatusTypeDef DAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan)
{
  DAL_CAN_StateTypeDef state = hcan->State;

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Request Sleep mode */
    SET_BIT(hcan->Instance->MCTRL, CAN_MCTRL_SLEEPREQ);

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    /* Return function status */
    return DAL_ERROR;
  }
}

/**
  * @brief  Wake up from sleep mode.
  *         When returning with DAL_OK status from this function, Sleep mode
  *         is exited.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval DAL status.
  */
DAL_StatusTypeDef DAL_CAN_WakeUp(CAN_HandleTypeDef *hcan)
{
  __IO uint32_t count = 0;
  uint32_t timeout = 1000000U;
  DAL_CAN_StateTypeDef state = hcan->State;

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Wake up request */
    CLEAR_BIT(hcan->Instance->MCTRL, CAN_MCTRL_SLEEPREQ);

    /* Wait sleep mode is exited */
    do
    {
      /* Increment counter */
      count++;

      /* Check if timeout is reached */
      if (count > timeout)
      {
        /* Update error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_TIMEOUT;

        return DAL_ERROR;
      }
    }
    while ((hcan->Instance->MSTS & CAN_MSTS_SLEEPFLG) != 0U);

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    return DAL_ERROR;
  }
}

/**
  * @brief  Check is sleep mode is active.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval Status
  *          - 0 : Sleep mode is not active.
  *          - 1 : Sleep mode is active.
  */
uint32_t DAL_CAN_IsSleepActive(CAN_HandleTypeDef *hcan)
{
  uint32_t status = 0U;
  DAL_CAN_StateTypeDef state = hcan->State;

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Check Sleep mode */
    if ((hcan->Instance->MSTS & CAN_MSTS_SLEEPFLG) != 0U)
    {
      status = 1U;
    }
  }

  /* Return function status */
  return status;
}

/**
  * @brief  Add a message to the first free Tx mailbox and activate the
  *         corresponding transmission request.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  pHeader pointer to a CAN_TxHeaderTypeDef structure.
  * @param  aData array containing the payload of the Tx frame.
  * @param  pTxMailbox pointer to a variable where the function will return
  *         the TxMailbox used to store the Tx message.
  *         This parameter can be a value of @arg CAN_Tx_Mailboxes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
  uint32_t transmitmailbox;
  DAL_CAN_StateTypeDef state = hcan->State;
  uint32_t TXSTS = READ_REG(hcan->Instance->TXSTS);

  /* Check the parameters */
  ASSERT_PARAM(IS_CAN_IDTYPE(pHeader->IDE));
  ASSERT_PARAM(IS_CAN_RTR(pHeader->RTR));
  ASSERT_PARAM(IS_CAN_DLC(pHeader->DLC));
  if (pHeader->IDE == CAN_ID_STD)
  {
    ASSERT_PARAM(IS_CAN_STDID(pHeader->StdId));
  }
  else
  {
    ASSERT_PARAM(IS_CAN_EXTID(pHeader->ExtId));
  }
  ASSERT_PARAM(IS_FUNCTIONAL_STATE(pHeader->TransmitGlobalTime));

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Check that all the Tx mailboxes are not full */
    if (((TXSTS & CAN_TXSTS_TXMEFLG0) != 0U) ||
        ((TXSTS & CAN_TXSTS_TXMEFLG1) != 0U) ||
        ((TXSTS & CAN_TXSTS_TXMEFLG2) != 0U))
    {
      /* Select an empty transmit mailbox */
      transmitmailbox = (TXSTS & CAN_TXSTS_EMNUM) >> CAN_TXSTS_EMNUM_Pos;

      /* Check transmit mailbox value */
      if (transmitmailbox > 2U)
      {
        /* Update error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_INTERNAL;

        return DAL_ERROR;
      }

      /* Store the Tx mailbox */
      *pTxMailbox = (uint32_t)1 << transmitmailbox;

      /* Set up the Id */
      if (pHeader->IDE == CAN_ID_STD)
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TXMID = ((pHeader->StdId << CAN_TXMID0_STDID_Pos) |
                                                           pHeader->RTR);
      }
      else
      {
        hcan->Instance->sTxMailBox[transmitmailbox].TXMID = ((pHeader->ExtId << CAN_TXMID0_EXTID_Pos) |
                                                           pHeader->IDE |
                                                           pHeader->RTR);
      }

      /* Set up the DLC */
      hcan->Instance->sTxMailBox[transmitmailbox].TXDLEN = (pHeader->DLC);

      /* Set up the Transmit Global Time mode */
      if (pHeader->TransmitGlobalTime == ENABLE)
      {
        SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TXDLEN, CAN_TXDLEN0_TXTS);
      }

      /* Set up the data field */
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TXMDH,
                ((uint32_t)aData[7] << CAN_TXMDH0_DATABYTE7_Pos) |
                ((uint32_t)aData[6] << CAN_TXMDH0_DATABYTE6_Pos) |
                ((uint32_t)aData[5] << CAN_TXMDH0_DATABYTE5_Pos) |
                ((uint32_t)aData[4] << CAN_TXMDH0_DATABYTE4_Pos));
      WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TXMDL,
                ((uint32_t)aData[3] << CAN_TXMDL0_DATABYTE3_Pos) |
                ((uint32_t)aData[2] << CAN_TXMDL0_DATABYTE2_Pos) |
                ((uint32_t)aData[1] << CAN_TXMDL0_DATABYTE1_Pos) |
                ((uint32_t)aData[0] << CAN_TXMDL0_DATABYTE0_Pos));

      /* Request transmission */
      SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TXMID, CAN_TXMID0_TXMREQ);

      /* Return function status */
      return DAL_OK;
    }
    else
    {
      /* Update error code */
      hcan->ErrorCode |= DAL_CAN_ERROR_PARAM;

      return DAL_ERROR;
    }
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    return DAL_ERROR;
  }
}

/**
  * @brief  Abort transmission requests
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  TxMailboxes List of the Tx Mailboxes to abort.
  *         This parameter can be any combination of @arg CAN_Tx_Mailboxes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes)
{
  DAL_CAN_StateTypeDef state = hcan->State;

  /* Check function parameters */
  ASSERT_PARAM(IS_CAN_TX_MAILBOX_LIST(TxMailboxes));

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Check Tx Mailbox 0 */
    if ((TxMailboxes & CAN_TX_MAILBOX0) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 0 */
      SET_BIT(hcan->Instance->TXSTS, CAN_TXSTS_ABREQFLG0);
    }

    /* Check Tx Mailbox 1 */
    if ((TxMailboxes & CAN_TX_MAILBOX1) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 1 */
      SET_BIT(hcan->Instance->TXSTS, CAN_TXSTS_ABREQFLG1);
    }

    /* Check Tx Mailbox 2 */
    if ((TxMailboxes & CAN_TX_MAILBOX2) != 0U)
    {
      /* Add cancellation request for Tx Mailbox 2 */
      SET_BIT(hcan->Instance->TXSTS, CAN_TXSTS_ABREQFLG2);
    }

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    return DAL_ERROR;
  }
}

/**
  * @brief  Return Tx Mailboxes free level: number of free Tx Mailboxes.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval Number of free Tx Mailboxes.
  */
uint32_t DAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)
{
  uint32_t freelevel = 0U;
  DAL_CAN_StateTypeDef state = hcan->State;

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Check Tx Mailbox 0 status */
    if ((hcan->Instance->TXSTS & CAN_TXSTS_TXMEFLG0) != 0U)
    {
      freelevel++;
    }

    /* Check Tx Mailbox 1 status */
    if ((hcan->Instance->TXSTS & CAN_TXSTS_TXMEFLG1) != 0U)
    {
      freelevel++;
    }

    /* Check Tx Mailbox 2 status */
    if ((hcan->Instance->TXSTS & CAN_TXSTS_TXMEFLG2) != 0U)
    {
      freelevel++;
    }
  }

  /* Return Tx Mailboxes free level */
  return freelevel;
}

/**
  * @brief  Check if a transmission request is pending on the selected Tx
  *         Mailboxes.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  TxMailboxes List of Tx Mailboxes to check.
  *         This parameter can be any combination of @arg CAN_Tx_Mailboxes.
  * @retval Status
  *          - 0 : No pending transmission request on any selected Tx Mailboxes.
  *          - 1 : Pending transmission request on at least one of the selected
  *                Tx Mailbox.
  */
uint32_t DAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes)
{
  uint32_t status = 0U;
  DAL_CAN_StateTypeDef state = hcan->State;

  /* Check function parameters */
  ASSERT_PARAM(IS_CAN_TX_MAILBOX_LIST(TxMailboxes));

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Check pending transmission request on the selected Tx Mailboxes */
    if ((hcan->Instance->TXSTS & (TxMailboxes << CAN_TXSTS_TXMEFLG0_Pos)) != (TxMailboxes << CAN_TXSTS_TXMEFLG0_Pos))
    {
      status = 1U;
    }
  }

  /* Return status */
  return status;
}

/**
  * @brief  Return timestamp of Tx message sent, if time triggered communication
            mode is enabled.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  TxMailbox Tx Mailbox where the timestamp of message sent will be
  *         read.
  *         This parameter can be one value of @arg CAN_Tx_Mailboxes.
  * @retval Timestamp of message sent from Tx Mailbox.
  */
uint32_t DAL_CAN_GetTxTimestamp(CAN_HandleTypeDef *hcan, uint32_t TxMailbox)
{
  uint32_t timestamp = 0U;
  uint32_t transmitmailbox;
  DAL_CAN_StateTypeDef state = hcan->State;

  /* Check function parameters */
  ASSERT_PARAM(IS_CAN_TX_MAILBOX(TxMailbox));

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Select the Tx mailbox */
    transmitmailbox = POSITION_VAL(TxMailbox);

    /* Get timestamp */
    timestamp = (hcan->Instance->sTxMailBox[transmitmailbox].TXDLEN & CAN_TXDLEN0_MTS) >> CAN_TXDLEN0_MTS_Pos;
  }

  /* Return the timestamp */
  return timestamp;
}

/**
  * @brief  Get an CAN frame from the Rx FIFO zone into the message RAM.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  RxFifo Fifo number of the received message to be read.
  *         This parameter can be a value of @arg CAN_receive_FIFO_number.
  * @param  pHeader pointer to a CAN_RxHeaderTypeDef structure where the header
  *         of the Rx frame will be stored.
  * @param  aData array where the payload of the Rx frame will be stored.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
  DAL_CAN_StateTypeDef state = hcan->State;

  ASSERT_PARAM(IS_CAN_RX_FIFO(RxFifo));

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Check the Rx FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Check that the Rx FIFO 0 is not empty */
      if ((hcan->Instance->RXF0 & CAN_RXF0_FMNUM0) == 0U)
      {
        /* Update error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_PARAM;

        return DAL_ERROR;
      }
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Check that the Rx FIFO 1 is not empty */
      if ((hcan->Instance->RXF1 & CAN_RXF1_FMNUM1) == 0U)
      {
        /* Update error code */
        hcan->ErrorCode |= DAL_CAN_ERROR_PARAM;

        return DAL_ERROR;
      }
    }

    /* Get the header */
    pHeader->IDE = CAN_RXMID0_IDTYPESEL & hcan->Instance->sFIFOMailBox[RxFifo].RXMID;
    if (pHeader->IDE == CAN_ID_STD)
    {
      pHeader->StdId = (CAN_RXMID0_STDID & hcan->Instance->sFIFOMailBox[RxFifo].RXMID) >> CAN_TXMID0_STDID_Pos;
    }
    else
    {
      pHeader->ExtId = ((CAN_RXMID0_EXTID | CAN_RXMID0_STDID) & hcan->Instance->sFIFOMailBox[RxFifo].RXMID) >> CAN_RXMID0_EXTID_Pos;
    }
    pHeader->RTR = (CAN_RXMID0_RFTXREQ & hcan->Instance->sFIFOMailBox[RxFifo].RXMID);
    pHeader->DLC = (CAN_RXDLEN0_DLCODE & hcan->Instance->sFIFOMailBox[RxFifo].RXDLEN) >> CAN_RXDLEN0_DLCODE_Pos;
    pHeader->FilterMatchIndex = (CAN_RXDLEN0_FMIDX & hcan->Instance->sFIFOMailBox[RxFifo].RXDLEN) >> CAN_RXDLEN0_FMIDX_Pos;
    pHeader->Timestamp = (CAN_RXDLEN0_MTS & hcan->Instance->sFIFOMailBox[RxFifo].RXDLEN) >> CAN_RXDLEN0_MTS_Pos;

    /* Get the data */
    aData[0] = (uint8_t)((CAN_RXMDL0_DATABYTE0 & hcan->Instance->sFIFOMailBox[RxFifo].RXMDL) >> CAN_RXMDL0_DATABYTE0_Pos);
    aData[1] = (uint8_t)((CAN_RXMDL0_DATABYTE1 & hcan->Instance->sFIFOMailBox[RxFifo].RXMDL) >> CAN_RXMDL0_DATABYTE1_Pos);
    aData[2] = (uint8_t)((CAN_RXMDL0_DATABYTE2 & hcan->Instance->sFIFOMailBox[RxFifo].RXMDL) >> CAN_RXMDL0_DATABYTE2_Pos);
    aData[3] = (uint8_t)((CAN_RXMDL0_DATABYTE3 & hcan->Instance->sFIFOMailBox[RxFifo].RXMDL) >> CAN_RXMDL0_DATABYTE3_Pos);
    aData[4] = (uint8_t)((CAN_RXMDH0_DATABYTE4 & hcan->Instance->sFIFOMailBox[RxFifo].RXMDH) >> CAN_RXMDH0_DATABYTE4_Pos);
    aData[5] = (uint8_t)((CAN_RXMDH0_DATABYTE5 & hcan->Instance->sFIFOMailBox[RxFifo].RXMDH) >> CAN_RXMDH0_DATABYTE5_Pos);
    aData[6] = (uint8_t)((CAN_RXMDH0_DATABYTE6 & hcan->Instance->sFIFOMailBox[RxFifo].RXMDH) >> CAN_RXMDH0_DATABYTE6_Pos);
    aData[7] = (uint8_t)((CAN_RXMDH0_DATABYTE7 & hcan->Instance->sFIFOMailBox[RxFifo].RXMDH) >> CAN_RXMDH0_DATABYTE7_Pos);

    /* Release the FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Release RX FIFO 0 */
      SET_BIT(hcan->Instance->RXF0, CAN_RXF0_RFOM0);
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Release RX FIFO 1 */
      SET_BIT(hcan->Instance->RXF1, CAN_RXF1_RFOM1);
    }

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    return DAL_ERROR;
  }
}

/**
  * @brief  Return Rx FIFO fill level.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  RxFifo Rx FIFO.
  *         This parameter can be a value of @arg CAN_receive_FIFO_number.
  * @retval Number of messages available in Rx FIFO.
  */
uint32_t DAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
  uint32_t filllevel = 0U;
  DAL_CAN_StateTypeDef state = hcan->State;

  /* Check function parameters */
  ASSERT_PARAM(IS_CAN_RX_FIFO(RxFifo));

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    if (RxFifo == CAN_RX_FIFO0)
    {
      filllevel = hcan->Instance->RXF0 & CAN_RXF0_FMNUM0;
    }
    else /* RxFifo == CAN_RX_FIFO1 */
    {
      filllevel = hcan->Instance->RXF1 & CAN_RXF1_FMNUM1;
    }
  }

  /* Return Rx FIFO fill level */
  return filllevel;
}

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group4 Interrupts management
 *  @brief    Interrupts management
 *
@verbatim
  ==============================================================================
                       ##### Interrupts management #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) DAL_CAN_ActivateNotification      : Enable interrupts
      (+) DAL_CAN_DeactivateNotification    : Disable interrupts
      (+) DAL_CAN_IRQHandler                : Handles CAN interrupt request

@endverbatim
  * @{
  */

/**
  * @brief  Enable interrupts.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  ActiveITs indicates which interrupts will be enabled.
  *         This parameter can be any combination of @arg CAN_Interrupts.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
{
  DAL_CAN_StateTypeDef state = hcan->State;

  /* Check function parameters */
  ASSERT_PARAM(IS_CAN_IT(ActiveITs));

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Enable the selected interrupts */
    __DAL_CAN_ENABLE_IT(hcan, ActiveITs);

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    return DAL_ERROR;
  }
}

/**
  * @brief  Disable interrupts.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  InactiveITs indicates which interrupts will be disabled.
  *         This parameter can be any combination of @arg CAN_Interrupts.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs)
{
  DAL_CAN_StateTypeDef state = hcan->State;

  /* Check function parameters */
  ASSERT_PARAM(IS_CAN_IT(InactiveITs));

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Disable the selected interrupts */
    __DAL_CAN_DISABLE_IT(hcan, InactiveITs);

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    return DAL_ERROR;
  }
}

/**
  * @brief  Handles CAN interrupt request
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void DAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan)
{
  uint32_t errorcode = DAL_CAN_ERROR_NONE;
  uint32_t interrupts = READ_REG(hcan->Instance->INTEN);
  uint32_t MSTSflags = READ_REG(hcan->Instance->MSTS);
  uint32_t TXSTSflags = READ_REG(hcan->Instance->TXSTS);
  uint32_t RXF0flags = READ_REG(hcan->Instance->RXF0);
  uint32_t RXF1flags = READ_REG(hcan->Instance->RXF1);
  uint32_t esrflags = READ_REG(hcan->Instance->ERRSTS);

  /* Transmit Mailbox empty interrupt management *****************************/
  if ((interrupts & CAN_IT_TX_MAILBOX_EMPTY) != 0U)
  {
    /* Transmit Mailbox 0 management *****************************************/
    if ((TXSTSflags & CAN_TXSTS_REQCFLG0) != 0U)
    {
      /* Clear the Transmission Complete flag (and TXOK0,ALST0,TERR0 bits) */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP0);

      if ((TXSTSflags & CAN_TXSTS_TXSUSFLG0) != 0U)
      {
        /* Transmission Mailbox 0 complete callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
        /* Call registered callback*/
        hcan->TxMailbox0CompleteCallback(hcan);
#else
        /* Call weak (surcharged) callback */
        DAL_CAN_TxMailbox0CompleteCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
      }
      else
      {
        if ((TXSTSflags & CAN_TXSTS_ARBLSTFLG0) != 0U)
        {
          /* Update error code */
          errorcode |= DAL_CAN_ERROR_TX_ALST0;
        }
        else if ((TXSTSflags & CAN_TXSTS_TXERRFLG0) != 0U)
        {
          /* Update error code */
          errorcode |= DAL_CAN_ERROR_TX_TERR0;
        }
        else
        {
          /* Transmission Mailbox 0 abort callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
          /* Call registered callback*/
          hcan->TxMailbox0AbortCallback(hcan);
#else
          /* Call weak (surcharged) callback */
          DAL_CAN_TxMailbox0AbortCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
        }
      }
    }

    /* Transmit Mailbox 1 management *****************************************/
    if ((TXSTSflags & CAN_TXSTS_REQCFLG1) != 0U)
    {
      /* Clear the Transmission Complete flag (and TXOK1,ALST1,TERR1 bits) */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP1);

      if ((TXSTSflags & CAN_TXSTS_TXSUSFLG1) != 0U)
      {
        /* Transmission Mailbox 1 complete callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
        /* Call registered callback*/
        hcan->TxMailbox1CompleteCallback(hcan);
#else
        /* Call weak (surcharged) callback */
        DAL_CAN_TxMailbox1CompleteCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
      }
      else
      {
        if ((TXSTSflags & CAN_TXSTS_ARBLSTFLG1) != 0U)
        {
          /* Update error code */
          errorcode |= DAL_CAN_ERROR_TX_ALST1;
        }
        else if ((TXSTSflags & CAN_TXSTS_TXERRFLG1) != 0U)
        {
          /* Update error code */
          errorcode |= DAL_CAN_ERROR_TX_TERR1;
        }
        else
        {
          /* Transmission Mailbox 1 abort callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
          /* Call registered callback*/
          hcan->TxMailbox1AbortCallback(hcan);
#else
          /* Call weak (surcharged) callback */
          DAL_CAN_TxMailbox1AbortCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
        }
      }
    }

    /* Transmit Mailbox 2 management *****************************************/
    if ((TXSTSflags & CAN_TXSTS_REQCFLG2) != 0U)
    {
      /* Clear the Transmission Complete flag (and TXOK2,ALST2,TERR2 bits) */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP2);

      if ((TXSTSflags & CAN_TXSTS_TXSUSFLG2) != 0U)
      {
        /* Transmission Mailbox 2 complete callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
        /* Call registered callback*/
        hcan->TxMailbox2CompleteCallback(hcan);
#else
        /* Call weak (surcharged) callback */
        DAL_CAN_TxMailbox2CompleteCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
      }
      else
      {
        if ((TXSTSflags & CAN_TXSTS_ARBLSTFLG2) != 0U)
        {
          /* Update error code */
          errorcode |= DAL_CAN_ERROR_TX_ALST2;
        }
        else if ((TXSTSflags & CAN_TXSTS_TXERRFLG2) != 0U)
        {
          /* Update error code */
          errorcode |= DAL_CAN_ERROR_TX_TERR2;
        }
        else
        {
          /* Transmission Mailbox 2 abort callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
          /* Call registered callback*/
          hcan->TxMailbox2AbortCallback(hcan);
#else
          /* Call weak (surcharged) callback */
          DAL_CAN_TxMailbox2AbortCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
        }
      }
    }
  }

  /* Receive FIFO 0 overrun interrupt management *****************************/
  if ((interrupts & CAN_IT_RX_FIFO0_OVERRUN) != 0U)
  {
    if ((RXF0flags & CAN_RXF0_FOVRFLG0) != 0U)
    {
      /* Set CAN error code to Rx Fifo 0 overrun error */
      errorcode |= DAL_CAN_ERROR_RX_FOV0;

      /* Clear FIFO0 Overrun Flag */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0);
    }
  }

  /* Receive FIFO 0 full interrupt management ********************************/
  if ((interrupts & CAN_IT_RX_FIFO0_FULL) != 0U)
  {
    if ((RXF0flags & CAN_RXF0_FFULLFLG0) != 0U)
    {
      /* Clear FIFO 0 full Flag */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FF0);

      /* Receive FIFO 0 full Callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
      /* Call registered callback*/
      hcan->RxFifo0FullCallback(hcan);
#else
      /* Call weak (surcharged) callback */
      DAL_CAN_RxFifo0FullCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
    }
  }

  /* Receive FIFO 0 message pending interrupt management *********************/
  if ((interrupts & CAN_IT_RX_FIFO0_MSG_PENDING) != 0U)
  {
    /* Check if message is still pending */
    if ((hcan->Instance->RXF0 & CAN_RXF0_FMNUM0) != 0U)
    {
      /* Receive FIFO 0 message pending Callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
      /* Call registered callback*/
      hcan->RxFifo0MsgPendingCallback(hcan);
#else
      /* Call weak (surcharged) callback */
      DAL_CAN_RxFifo0MsgPendingCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
    }
  }

  /* Receive FIFO 1 overrun interrupt management *****************************/
  if ((interrupts & CAN_IT_RX_FIFO1_OVERRUN) != 0U)
  {
    if ((RXF1flags & CAN_RXF1_FOVRFLG1) != 0U)
    {
      /* Set CAN error code to Rx Fifo 1 overrun error */
      errorcode |= DAL_CAN_ERROR_RX_FOV1;

      /* Clear FIFO1 Overrun Flag */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV1);
    }
  }

  /* Receive FIFO 1 full interrupt management ********************************/
  if ((interrupts & CAN_IT_RX_FIFO1_FULL) != 0U)
  {
    if ((RXF1flags & CAN_RXF1_FFULLFLG1) != 0U)
    {
      /* Clear FIFO 1 full Flag */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FF1);

      /* Receive FIFO 1 full Callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
      /* Call registered callback*/
      hcan->RxFifo1FullCallback(hcan);
#else
      /* Call weak (surcharged) callback */
      DAL_CAN_RxFifo1FullCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
    }
  }

  /* Receive FIFO 1 message pending interrupt management *********************/
  if ((interrupts & CAN_IT_RX_FIFO1_MSG_PENDING) != 0U)
  {
    /* Check if message is still pending */
    if ((hcan->Instance->RXF1 & CAN_RXF1_FMNUM1) != 0U)
    {
      /* Receive FIFO 1 message pending Callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
      /* Call registered callback*/
      hcan->RxFifo1MsgPendingCallback(hcan);
#else
      /* Call weak (surcharged) callback */
      DAL_CAN_RxFifo1MsgPendingCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
    }
  }

  /* Sleep interrupt management *********************************************/
  if ((interrupts & CAN_IT_SLEEP_ACK) != 0U)
  {
    if ((MSTSflags & CAN_MSTS_SLEEPIFLG) != 0U)
    {
      /* Clear Sleep interrupt Flag */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_SLAKI);

      /* Sleep Callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
      /* Call registered callback*/
      hcan->SleepCallback(hcan);
#else
      /* Call weak (surcharged) callback */
      DAL_CAN_SleepCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
    }
  }

  /* WakeUp interrupt management *********************************************/
  if ((interrupts & CAN_IT_WAKEUP) != 0U)
  {
    if ((MSTSflags & CAN_MSTS_WUPIFLG) != 0U)
    {
      /* Clear WakeUp Flag */
      __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_WKU);

      /* WakeUp Callback */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
      /* Call registered callback*/
      hcan->WakeUpFromRxMsgCallback(hcan);
#else
      /* Call weak (surcharged) callback */
      DAL_CAN_WakeUpFromRxMsgCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
    }
  }

  /* Error interrupts management *********************************************/
  if ((interrupts & CAN_IT_ERROR) != 0U)
  {
    if ((MSTSflags & CAN_MSTS_ERRIFLG) != 0U)
    {
      /* Check Error Warning Flag */
      if (((interrupts & CAN_IT_ERROR_WARNING) != 0U) &&
          ((esrflags & CAN_ERRSTS_ERRWFLG) != 0U))
      {
        /* Set CAN error code to Error Warning */
        errorcode |= DAL_CAN_ERROR_EWG;

        /* No need for clear of Error Warning Flag as read-only */
      }

      /* Check Error Passive Flag */
      if (((interrupts & CAN_IT_ERROR_PASSIVE) != 0U) &&
          ((esrflags & CAN_ERRSTS_ERRPFLG) != 0U))
      {
        /* Set CAN error code to Error Passive */
        errorcode |= DAL_CAN_ERROR_EPV;

        /* No need for clear of Error Passive Flag as read-only */
      }

      /* Check Bus-off Flag */
      if (((interrupts & CAN_IT_BUSOFF) != 0U) &&
          ((esrflags & CAN_ERRSTS_BOFLG) != 0U))
      {
        /* Set CAN error code to Bus-Off */
        errorcode |= DAL_CAN_ERROR_BOF;

        /* No need for clear of Error Bus-Off as read-only */
      }

      /* Check Last Error Code Flag */
      if (((interrupts & CAN_IT_LAST_ERROR_CODE) != 0U) &&
          ((esrflags & CAN_ERRSTS_LERRC) != 0U))
      {
        switch (esrflags & CAN_ERRSTS_LERRC)
        {
          case (CAN_ERRSTS_LERRC_0):
            /* Set CAN error code to Stuff error */
            errorcode |= DAL_CAN_ERROR_STF;
            break;
          case (CAN_ERRSTS_LERRC_1):
            /* Set CAN error code to Form error */
            errorcode |= DAL_CAN_ERROR_FOR;
            break;
          case (CAN_ERRSTS_LERRC_1 | CAN_ERRSTS_LERRC_0):
            /* Set CAN error code to Acknowledgement error */
            errorcode |= DAL_CAN_ERROR_ACK;
            break;
          case (CAN_ERRSTS_LERRC_2):
            /* Set CAN error code to Bit recessive error */
            errorcode |= DAL_CAN_ERROR_BR;
            break;
          case (CAN_ERRSTS_LERRC_2 | CAN_ERRSTS_LERRC_0):
            /* Set CAN error code to Bit Dominant error */
            errorcode |= DAL_CAN_ERROR_BD;
            break;
          case (CAN_ERRSTS_LERRC_2 | CAN_ERRSTS_LERRC_1):
            /* Set CAN error code to CRC error */
            errorcode |= DAL_CAN_ERROR_CRC;
            break;
          default:
            break;
        }

        /* Clear Last error code Flag */
        CLEAR_BIT(hcan->Instance->ERRSTS, CAN_ERRSTS_LERRC);
      }
    }

    /* Clear ERRI Flag */
    __DAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_ERRI);
  }

  /* Call the Error call Back in case of Errors */
  if (errorcode != DAL_CAN_ERROR_NONE)
  {
    /* Update error code in handle */
    hcan->ErrorCode |= errorcode;

    /* Call Error callback function */
#if USE_DAL_CAN_REGISTER_CALLBACKS == 1
    /* Call registered callback*/
    hcan->ErrorCallback(hcan);
#else
    /* Call weak (surcharged) callback */
    DAL_CAN_ErrorCallback(hcan);
#endif /* USE_DAL_CAN_REGISTER_CALLBACKS */
  }
}

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group5 Callback functions
 *  @brief   CAN Callback functions
 *
@verbatim
  ==============================================================================
                          ##### Callback functions #####
  ==============================================================================
    [..]
    This subsection provides the following callback functions:
      (+) DAL_CAN_TxMailbox0CompleteCallback
      (+) DAL_CAN_TxMailbox1CompleteCallback
      (+) DAL_CAN_TxMailbox2CompleteCallback
      (+) DAL_CAN_TxMailbox0AbortCallback
      (+) DAL_CAN_TxMailbox1AbortCallback
      (+) DAL_CAN_TxMailbox2AbortCallback
      (+) DAL_CAN_RxFifo0MsgPendingCallback
      (+) DAL_CAN_RxFifo0FullCallback
      (+) DAL_CAN_RxFifo1MsgPendingCallback
      (+) DAL_CAN_RxFifo1FullCallback
      (+) DAL_CAN_SleepCallback
      (+) DAL_CAN_WakeUpFromRxMsgCallback
      (+) DAL_CAN_ErrorCallback

@endverbatim
  * @{
  */

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_TxMailbox0CompleteCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Transmission Mailbox 1 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_TxMailbox1CompleteCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Transmission Mailbox 2 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_TxMailbox2CompleteCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Transmission Mailbox 0 Cancellation callback.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_TxMailbox0AbortCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Transmission Mailbox 1 Cancellation callback.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_TxMailbox1AbortCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Transmission Mailbox 2 Cancellation callback.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_TxMailbox2AbortCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Rx FIFO 0 full callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_RxFifo0FullCallback could be implemented in the user
            file
   */
}

/**
  * @brief  Rx FIFO 1 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_RxFifo1MsgPendingCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Rx FIFO 1 full callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_RxFifo1FullCallback could be implemented in the user
            file
   */
}

/**
  * @brief  Sleep callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_SleepCallback could be implemented in the user file
   */
}

/**
  * @brief  WakeUp from Rx message callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_WakeUpFromRxMsgCallback could be implemented in the
            user file
   */
}

/**
  * @brief  Error CAN callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void DAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_CAN_ErrorCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup CAN_Exported_Functions_Group6 Peripheral State and Error functions
 *  @brief   CAN Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral State and Error functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) DAL_CAN_GetState()  : Return the CAN state.
      (+) DAL_CAN_GetError()  : Return the CAN error codes if any.
      (+) DAL_CAN_ResetError(): Reset the CAN error codes if any.

@endverbatim
  * @{
  */

/**
  * @brief  Return the CAN state.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval DAL state
  */
DAL_CAN_StateTypeDef DAL_CAN_GetState(CAN_HandleTypeDef *hcan)
{
  DAL_CAN_StateTypeDef state = hcan->State;

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Check sleep mode acknowledge flag */
    if ((hcan->Instance->MSTS & CAN_MSTS_SLEEPFLG) != 0U)
    {
      /* Sleep mode is active */
      state = DAL_CAN_STATE_SLEEP_ACTIVE;
    }
    /* Check sleep mode request flag */
    else if ((hcan->Instance->MCTRL & CAN_MCTRL_SLEEPREQ) != 0U)
    {
      /* Sleep mode request is pending */
      state = DAL_CAN_STATE_SLEEP_PENDING;
    }
    else
    {
      /* Neither sleep mode request nor sleep mode acknowledge */
    }
  }

  /* Return CAN state */
  return state;
}

/**
  * @brief  Return the CAN error code.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval CAN Error Code
  */
uint32_t DAL_CAN_GetError(CAN_HandleTypeDef *hcan)
{
  /* Return CAN error code */
  return hcan->ErrorCode;
}

/**
  * @brief  Reset the CAN error code.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_CAN_ResetError(CAN_HandleTypeDef *hcan)
{
  DAL_StatusTypeDef status = DAL_OK;
  DAL_CAN_StateTypeDef state = hcan->State;

  if ((state == DAL_CAN_STATE_READY) ||
      (state == DAL_CAN_STATE_LISTENING))
  {
    /* Reset CAN error code */
    hcan->ErrorCode = 0U;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= DAL_CAN_ERROR_NOT_INITIALIZED;

    status = DAL_ERROR;
  }

  /* Return the status */
  return status;
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_CAN_MODULE_ENABLED */

/**
  * @}
  */

#endif /* CAN1 */

/**
  * @}
  */
