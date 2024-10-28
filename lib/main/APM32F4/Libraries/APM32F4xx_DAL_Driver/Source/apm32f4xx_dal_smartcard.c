/**
  *
  * @file    apm32f4xx_dal_smartcard.c
  * @brief   SMARTCARD DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the SMARTCARD peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
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
      The SMARTCARD DAL driver can be used as follows:

    (#) Declare a SMARTCARD_HandleTypeDef handle structure.
    (#) Initialize the SMARTCARD low level resources by implementing the DAL_SMARTCARD_MspInit() API:
        (##) Enable the interface clock of the USARTx associated to the SMARTCARD.
        (##) SMARTCARD pins configuration:
            (+++) Enable the clock for the SMARTCARD GPIOs.
            (+++) Configure SMARTCARD pins as alternate function pull-up.
        (##) NVIC configuration if you need to use interrupt process (DAL_SMARTCARD_Transmit_IT()
             and DAL_SMARTCARD_Receive_IT() APIs):
            (+++) Configure the USARTx interrupt priority.
            (+++) Enable the NVIC USART IRQ handle.
        (##) DMA Configuration if you need to use DMA process (DAL_SMARTCARD_Transmit_DMA()
             and DAL_SMARTCARD_Receive_DMA() APIs):
            (+++) Declare a DMA handle structure for the Tx/Rx stream.
            (+++) Enable the DMAx interface clock.
            (+++) Configure the declared DMA handle structure with the required Tx/Rx parameters.
            (+++) Configure the DMA Tx/Rx stream.
            (+++) Associate the initialized DMA handle to the SMARTCARD DMA Tx/Rx handle.
            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA Tx/Rx stream.
            (+++) Configure the USARTx interrupt priority and enable the NVIC USART IRQ handle
                  (used for last byte sending completion detection in DMA non circular mode)

    (#) Program the Baud Rate, Word Length , Stop Bit, Parity, Hardware
        flow control and Mode(Receiver/Transmitter) in the SMARTCARD Init structure.

    (#) Initialize the SMARTCARD registers by calling the DAL_SMARTCARD_Init() API:
        (++) These APIs configure also the low level Hardware GPIO, CLOCK, CORTEX...etc)
             by calling the customized DAL_SMARTCARD_MspInit() API.
    [..]
    (@) The specific SMARTCARD interrupts (Transmission complete interrupt,
        RXNE interrupt and Error Interrupts) will be managed using the macros
        __DAL_SMARTCARD_ENABLE_IT() and __DAL_SMARTCARD_DISABLE_IT() inside the transmit and receive process.

    [..]
    Three operation modes are available within this driver :

    *** Polling mode IO operation ***
    =================================
    [..]
      (+) Send an amount of data in blocking mode using DAL_SMARTCARD_Transmit()
      (+) Receive an amount of data in blocking mode using DAL_SMARTCARD_Receive()

    *** Interrupt mode IO operation ***
    ===================================
    [..]
      (+) Send an amount of data in non blocking mode using DAL_SMARTCARD_Transmit_IT()
      (+) At transmission end of transfer DAL_SMARTCARD_TxCpltCallback is executed and user can
          add his own code by customization of function pointer DAL_SMARTCARD_TxCpltCallback
      (+) Receive an amount of data in non blocking mode using DAL_SMARTCARD_Receive_IT()
      (+) At reception end of transfer DAL_SMARTCARD_RxCpltCallback is executed and user can
          add his own code by customization of function pointer DAL_SMARTCARD_RxCpltCallback
      (+) In case of transfer Error, DAL_SMARTCARD_ErrorCallback() function is executed and user can
          add his own code by customization of function pointer DAL_SMARTCARD_ErrorCallback

    *** DMA mode IO operation ***
    ==============================
    [..]
      (+) Send an amount of data in non blocking mode (DMA) using DAL_SMARTCARD_Transmit_DMA()
      (+) At transmission end of transfer DAL_SMARTCARD_TxCpltCallback is executed and user can
          add his own code by customization of function pointer DAL_SMARTCARD_TxCpltCallback
      (+) Receive an amount of data in non blocking mode (DMA) using DAL_SMARTCARD_Receive_DMA()
      (+) At reception end of transfer DAL_SMARTCARD_RxCpltCallback is executed and user can
          add his own code by customization of function pointer DAL_SMARTCARD_RxCpltCallback
      (+) In case of transfer Error, DAL_SMARTCARD_ErrorCallback() function is executed and user can
          add his own code by customization of function pointer DAL_SMARTCARD_ErrorCallback

    *** SMARTCARD DAL driver macros list ***
    ========================================
    [..]
      Below the list of most used macros in SMARTCARD DAL driver.

      (+) __DAL_SMARTCARD_ENABLE: Enable the SMARTCARD peripheral
      (+) __DAL_SMARTCARD_DISABLE: Disable the SMARTCARD peripheral
      (+) __DAL_SMARTCARD_GET_FLAG : Check whether the specified SMARTCARD flag is set or not
      (+) __DAL_SMARTCARD_CLEAR_FLAG : Clear the specified SMARTCARD pending flag
      (+) __DAL_SMARTCARD_ENABLE_IT: Enable the specified SMARTCARD interrupt
      (+) __DAL_SMARTCARD_DISABLE_IT: Disable the specified SMARTCARD interrupt

    [..]
      (@) You can refer to the SMARTCARD DAL driver header file for more useful macros

    ##### Callback registration #####
    ==================================

    [..]
    The compilation define USE_DAL_SMARTCARD_REGISTER_CALLBACKS when set to 1
    allows the user to configure dynamically the driver callbacks.

    [..]
    Use Function DAL_SMARTCARD_RegisterCallback() to register a user callback.
    Function DAL_SMARTCARD_RegisterCallback() allows to register following callbacks:
    (+) TxCpltCallback            : Tx Complete Callback.
    (+) RxCpltCallback            : Rx Complete Callback.
    (+) ErrorCallback             : Error Callback.
    (+) AbortCpltCallback         : Abort Complete Callback.
    (+) AbortTransmitCpltCallback : Abort Transmit Complete Callback.
    (+) AbortReceiveCpltCallback  : Abort Receive Complete Callback.
    (+) MspInitCallback           : SMARTCARD MspInit.
    (+) MspDeInitCallback         : SMARTCARD MspDeInit.
    This function takes as parameters the DAL peripheral handle, the Callback ID
    and a pointer to the user callback function.

    [..]
    Use function DAL_SMARTCARD_UnRegisterCallback() to reset a callback to the default
    weak (surcharged) function.
    DAL_SMARTCARD_UnRegisterCallback() takes as parameters the DAL peripheral handle,
    and the Callback ID.
    This function allows to reset following callbacks:
    (+) TxCpltCallback            : Tx Complete Callback.
    (+) RxCpltCallback            : Rx Complete Callback.
    (+) ErrorCallback             : Error Callback.
    (+) AbortCpltCallback         : Abort Complete Callback.
    (+) AbortTransmitCpltCallback : Abort Transmit Complete Callback.
    (+) AbortReceiveCpltCallback  : Abort Receive Complete Callback.
    (+) MspInitCallback           : SMARTCARD MspInit.
    (+) MspDeInitCallback         : SMARTCARD MspDeInit.

    [..]
    By default, after the DAL_SMARTCARD_Init() and when the state is DAL_SMARTCARD_STATE_RESET
    all callbacks are set to the corresponding weak (surcharged) functions:
    examples DAL_SMARTCARD_TxCpltCallback(), DAL_SMARTCARD_RxCpltCallback().
    Exception done for MspInit and MspDeInit functions that are respectively
    reset to the legacy weak (surcharged) functions in the DAL_SMARTCARD_Init()
    and DAL_SMARTCARD_DeInit() only when these callbacks are null (not registered beforehand).
    If not, MspInit or MspDeInit are not null, the DAL_SMARTCARD_Init() and DAL_SMARTCARD_DeInit()
    keep and use the user MspInit/MspDeInit callbacks (registered beforehand).

    [..]
    Callbacks can be registered/unregistered in DAL_SMARTCARD_STATE_READY state only.
    Exception done MspInit/MspDeInit that can be registered/unregistered
    in DAL_SMARTCARD_STATE_READY or DAL_SMARTCARD_STATE_RESET state, thus registered (user)
    MspInit/DeInit callbacks can be used during the Init/DeInit.
    In that case first register the MspInit/MspDeInit user callbacks
    using DAL_SMARTCARD_RegisterCallback() before calling DAL_SMARTCARD_DeInit()
    or DAL_SMARTCARD_Init() function.

    [..]
    When The compilation define USE_DAL_SMARTCARD_REGISTER_CALLBACKS is set to 0 or
    not defined, the callback registration feature is not available
    and weak (surcharged) callbacks are used.

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup SMARTCARD SMARTCARD
  * @brief DAL SMARTCARD module driver
  * @{
  */
#ifdef DAL_SMARTCARD_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup SMARTCARD_Private_Constants
  * @{
  */
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup SMARTCARD_Private_Functions
  * @{
  */
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
void SMARTCARD_InitCallbacksToDefault(SMARTCARD_HandleTypeDef *hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */
static void SMARTCARD_EndTxTransfer(SMARTCARD_HandleTypeDef *hsc);
static void SMARTCARD_EndRxTransfer(SMARTCARD_HandleTypeDef *hsc);
static void SMARTCARD_SetConfig (SMARTCARD_HandleTypeDef *hsc);
static DAL_StatusTypeDef SMARTCARD_Transmit_IT(SMARTCARD_HandleTypeDef *hsc);
static DAL_StatusTypeDef SMARTCARD_EndTransmit_IT(SMARTCARD_HandleTypeDef *hsc);
static DAL_StatusTypeDef SMARTCARD_Receive_IT(SMARTCARD_HandleTypeDef *hsc);
static void SMARTCARD_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SMARTCARD_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void SMARTCARD_DMAError(DMA_HandleTypeDef *hdma);
static void SMARTCARD_DMAAbortOnError(DMA_HandleTypeDef *hdma);
static void SMARTCARD_DMATxAbortCallback(DMA_HandleTypeDef *hdma);
static void SMARTCARD_DMARxAbortCallback(DMA_HandleTypeDef *hdma);
static void SMARTCARD_DMATxOnlyAbortCallback(DMA_HandleTypeDef *hdma);
static void SMARTCARD_DMARxOnlyAbortCallback(DMA_HandleTypeDef *hdma);
static DAL_StatusTypeDef SMARTCARD_WaitOnFlagUntilTimeout(SMARTCARD_HandleTypeDef *hsc, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SMARTCARD_Exported_Functions SMARTCARD Exported Functions
  * @{
  */

/** @defgroup SMARTCARD_Exported_Functions_Group1 SmartCard Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
  ==============================================================================
              ##### Initialization and Configuration functions #####
  ==============================================================================
  [..]
  This subsection provides a set of functions allowing to initialize the USART
  in Smartcard mode.
  [..]
  The Smartcard interface is designed to support asynchronous protocol Smartcards as
  defined in the ISO 7816-3 standard.
  [..]
  The USART can provide a clock to the smartcard through the SCLK output.
  In smartcard mode, SCLK is not associated to the communication but is simply derived
  from the internal peripheral input clock through a 5-bit prescaler.
  [..]
  (+) For the Smartcard mode only these parameters can be configured:
      (++) Baud Rate
      (++) Word Length => Should be 9 bits (8 bits + parity)
      (++) Stop Bit
      (++) Parity: => Should be enabled
      (++) USART polarity
      (++) USART phase
      (++) USART LastBit
      (++) Receiver/transmitter modes
      (++) Prescaler
      (++) GuardTime
      (++) NACKState: The Smartcard NACK state

     (+) Recommended SmartCard interface configuration to get the Answer to Reset from the Card:
        (++) Word Length = 9 Bits
        (++) 1.5 Stop Bit
        (++) Even parity
        (++) BaudRate = 12096 baud
        (++) Tx and Rx enabled
  [..]
  Please refer to the ISO 7816-3 specification for more details.

  [..]
   (@) It is also possible to choose 0.5 stop bit for receiving but it is recommended
       to use 1.5 stop bits for both transmitting and receiving to avoid switching
       between the two configurations.
  [..]
    The DAL_SMARTCARD_Init() function follows the USART  SmartCard configuration
    procedures (details for the procedures are available in reference manual).

@endverbatim

  The SMARTCARD frame format is given in the following table:
       +-------------------------------------------------------------+
       |   M bit |  PCE bit  |        SMARTCARD frame                |
       |---------------------|---------------------------------------|
       |    1    |    1      |    | SB | 8 bit data | PB | STB |     |
       +-------------------------------------------------------------+
  * @{
  */

/**
  * @brief  Initializes the SmartCard mode according to the specified
  *         parameters in the SMARTCARD_InitTypeDef and create the associated handle.
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_Init(SMARTCARD_HandleTypeDef *hsc)
{
  /* Check the SMARTCARD handle allocation */
  if(hsc == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_SMARTCARD_INSTANCE(hsc->Instance));
  ASSERT_PARAM(IS_SMARTCARD_NACK_STATE(hsc->Init.NACKState));

  if(hsc->gState == DAL_SMARTCARD_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hsc->Lock = DAL_UNLOCKED;

#if USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1
    SMARTCARD_InitCallbacksToDefault(hsc);

    if (hsc->MspInitCallback == NULL)
    {
      hsc->MspInitCallback = DAL_SMARTCARD_MspInit;
    }

    /* Init the low level hardware */
    hsc->MspInitCallback(hsc);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    DAL_SMARTCARD_MspInit(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */
  }

  hsc->gState = DAL_SMARTCARD_STATE_BUSY;

  /* Set the Prescaler */
  MODIFY_REG(hsc->Instance->GTPSC, USART_GTPSC_PSC, hsc->Init.Prescaler);

  /* Set the Guard Time */
  MODIFY_REG(hsc->Instance->GTPSC, USART_GTPSC_GRDT, ((hsc->Init.GuardTime)<<8U));

  /* Set the Smartcard Communication parameters */
  SMARTCARD_SetConfig(hsc);

  /* In SmartCard mode, the following bits must be kept cleared:
  - LINEN bit in the USART_CTRL2 register
  - HDSEL and IREN bits in the USART_CTRL3 register.*/
  CLEAR_BIT(hsc->Instance->CTRL2, USART_CTRL2_LINMEN);
  CLEAR_BIT(hsc->Instance->CTRL3, (USART_CTRL3_IREN | USART_CTRL3_HDEN));

  /* Enable the SMARTCARD Parity Error Interrupt */
  SET_BIT(hsc->Instance->CTRL1, USART_CTRL1_PEIEN);

  /* Enable the SMARTCARD Framing Error Interrupt */
  SET_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Enable the Peripheral */
  __DAL_SMARTCARD_ENABLE(hsc);

  /* Configure the Smartcard NACK state */
  MODIFY_REG(hsc->Instance->CTRL3, USART_CTRL3_SCNACKEN, hsc->Init.NACKState);

  /* Enable the SC mode by setting the SCEN bit in the CTRL3 register */
  hsc->Instance->CTRL3 |= (USART_CTRL3_SCEN);

  /* Initialize the SMARTCARD state*/
  hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
  hsc->gState= DAL_SMARTCARD_STATE_READY;
  hsc->RxState= DAL_SMARTCARD_STATE_READY;

  return DAL_OK;
}

/**
  * @brief DeInitializes the USART SmartCard peripheral
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_DeInit(SMARTCARD_HandleTypeDef *hsc)
{
  /* Check the SMARTCARD handle allocation */
  if(hsc == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_SMARTCARD_INSTANCE(hsc->Instance));

  hsc->gState = DAL_SMARTCARD_STATE_BUSY;

  /* Disable the Peripheral */
  __DAL_SMARTCARD_DISABLE(hsc);

  /* DeInit the low level hardware */
#if USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1
  if (hsc->MspDeInitCallback == NULL)
  {
    hsc->MspDeInitCallback = DAL_SMARTCARD_MspDeInit;
  }
  /* DeInit the low level hardware */
  hsc->MspDeInitCallback(hsc);
#else
  DAL_SMARTCARD_MspDeInit(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */

  hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
  hsc->gState = DAL_SMARTCARD_STATE_RESET;
  hsc->RxState = DAL_SMARTCARD_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hsc);

  return DAL_OK;
}

/**
  * @brief  SMARTCARD MSP Init
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
__weak void DAL_SMARTCARD_MspInit(SMARTCARD_HandleTypeDef *hsc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMARTCARD_MspInit can be implemented in the user file
   */
}

/**
  * @brief SMARTCARD MSP DeInit
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
__weak void DAL_SMARTCARD_MspDeInit(SMARTCARD_HandleTypeDef *hsc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMARTCARD_MspDeInit can be implemented in the user file
   */
}

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User SMARTCARD Callback
  *         To be used instead of the weak predefined callback
  * @param  hsc smartcard handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_SMARTCARD_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_ERROR_CB_ID Error Callback ID
  *           @arg @ref DAL_SMARTCARD_ABORT_COMPLETE_CB_ID Abort Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_ABORT_TRANSMIT_COMPLETE_CB_ID Abort Transmit Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_ABORT_RECEIVE_COMPLETE_CB_ID Abort Receive Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref DAL_SMARTCARD_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_RegisterCallback(SMARTCARD_HandleTypeDef *hsc, DAL_SMARTCARD_CallbackIDTypeDef CallbackID, pSMARTCARD_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hsc->ErrorCode |= DAL_SMARTCARD_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hsc);

  if (hsc->gState == DAL_SMARTCARD_STATE_READY)
  {
    switch (CallbackID)
    {

      case DAL_SMARTCARD_TX_COMPLETE_CB_ID :
        hsc->TxCpltCallback = pCallback;
        break;

      case DAL_SMARTCARD_RX_COMPLETE_CB_ID :
        hsc->RxCpltCallback = pCallback;
        break;

      case DAL_SMARTCARD_ERROR_CB_ID :
        hsc->ErrorCallback = pCallback;
        break;

      case DAL_SMARTCARD_ABORT_COMPLETE_CB_ID :
        hsc->AbortCpltCallback = pCallback;
        break;

      case DAL_SMARTCARD_ABORT_TRANSMIT_COMPLETE_CB_ID :
        hsc->AbortTransmitCpltCallback = pCallback;
        break;

      case DAL_SMARTCARD_ABORT_RECEIVE_COMPLETE_CB_ID :
        hsc->AbortReceiveCpltCallback = pCallback;
        break;


      case DAL_SMARTCARD_MSPINIT_CB_ID :
        hsc->MspInitCallback = pCallback;
        break;

      case DAL_SMARTCARD_MSPDEINIT_CB_ID :
        hsc->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hsc->ErrorCode |= DAL_SMARTCARD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hsc->gState == DAL_SMARTCARD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_SMARTCARD_MSPINIT_CB_ID :
        hsc->MspInitCallback = pCallback;
        break;

      case DAL_SMARTCARD_MSPDEINIT_CB_ID :
        hsc->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hsc->ErrorCode |= DAL_SMARTCARD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hsc->ErrorCode |= DAL_SMARTCARD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsc);

  return status;
}

/**
  * @brief  Unregister an SMARTCARD callback
  *         SMARTCARD callback is redirected to the weak predefined callback
  * @param  hsc smartcard handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_SMARTCARD_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_ERROR_CB_ID Error Callback ID
  *           @arg @ref DAL_SMARTCARD_ABORT_COMPLETE_CB_ID Abort Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_ABORT_TRANSMIT_COMPLETE_CB_ID Abort Transmit Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_ABORT_RECEIVE_COMPLETE_CB_ID Abort Receive Complete Callback ID
  *           @arg @ref DAL_SMARTCARD_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref DAL_SMARTCARD_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_UnRegisterCallback(SMARTCARD_HandleTypeDef *hsc, DAL_SMARTCARD_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hsc);

  if (DAL_SMARTCARD_STATE_READY == hsc->gState)
  {
    switch (CallbackID)
    {
      case DAL_SMARTCARD_TX_COMPLETE_CB_ID :
        hsc->TxCpltCallback = DAL_SMARTCARD_TxCpltCallback;                       /* Legacy weak TxCpltCallback            */
        break;

      case DAL_SMARTCARD_RX_COMPLETE_CB_ID :
        hsc->RxCpltCallback = DAL_SMARTCARD_RxCpltCallback;                       /* Legacy weak RxCpltCallback            */
        break;

      case DAL_SMARTCARD_ERROR_CB_ID :
        hsc->ErrorCallback = DAL_SMARTCARD_ErrorCallback;                         /* Legacy weak ErrorCallback             */
        break;

      case DAL_SMARTCARD_ABORT_COMPLETE_CB_ID :
        hsc->AbortCpltCallback = DAL_SMARTCARD_AbortCpltCallback;                 /* Legacy weak AbortCpltCallback         */
        break;

      case DAL_SMARTCARD_ABORT_TRANSMIT_COMPLETE_CB_ID :
        hsc->AbortTransmitCpltCallback = DAL_SMARTCARD_AbortTransmitCpltCallback; /* Legacy weak AbortTransmitCpltCallback */
        break;

      case DAL_SMARTCARD_ABORT_RECEIVE_COMPLETE_CB_ID :
        hsc->AbortReceiveCpltCallback = DAL_SMARTCARD_AbortReceiveCpltCallback;   /* Legacy weak AbortReceiveCpltCallback  */
        break;


      case DAL_SMARTCARD_MSPINIT_CB_ID :
        hsc->MspInitCallback = DAL_SMARTCARD_MspInit;                             /* Legacy weak MspInitCallback           */
        break;

      case DAL_SMARTCARD_MSPDEINIT_CB_ID :
        hsc->MspDeInitCallback = DAL_SMARTCARD_MspDeInit;                         /* Legacy weak MspDeInitCallback         */
        break;

      default :
        /* Update the error code */
        hsc->ErrorCode |= DAL_SMARTCARD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_SMARTCARD_STATE_RESET == hsc->gState)
  {
    switch (CallbackID)
    {
      case DAL_SMARTCARD_MSPINIT_CB_ID :
        hsc->MspInitCallback = DAL_SMARTCARD_MspInit;
        break;

      case DAL_SMARTCARD_MSPDEINIT_CB_ID :
        hsc->MspDeInitCallback = DAL_SMARTCARD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hsc->ErrorCode |= DAL_SMARTCARD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hsc->ErrorCode |= DAL_SMARTCARD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsc);

  return status;
}
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group2 IO operation functions
  * @brief    SMARTCARD Transmit and Receive functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
   This subsection provides a set of functions allowing to manage the SMARTCARD data transfers.

 [..]
    (#) Smartcard is a single wire half duplex communication protocol.
    The Smartcard interface is designed to support asynchronous protocol Smartcards as
    defined in the ISO 7816-3 standard.
    (#) The USART should be configured as:
       (++) 8 bits plus parity: where M=1 and PCE=1 in the USART_CTRL1 register
       (++) 1.5 stop bits when transmitting and receiving: where STOP=11 in the USART_CTRL2 register.

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The DAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) Non Blocking mode: The communication is performed using Interrupts
           or DMA, These APIs return the DAL status.
           The end of the data processing will be indicated through the
           dedicated SMARTCARD IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The DAL_SMARTCARD_TxCpltCallback(), DAL_SMARTCARD_RxCpltCallback() user callbacks
           will be executed respectively at the end of the Transmit or Receive process
           The DAL_SMARTCARD_ErrorCallback() user callback will be executed when a communication error is detected

    (#) Blocking mode APIs are :
        (++) DAL_SMARTCARD_Transmit()
        (++) DAL_SMARTCARD_Receive()

    (#) Non Blocking mode APIs with Interrupt are :
        (++) DAL_SMARTCARD_Transmit_IT()
        (++) DAL_SMARTCARD_Receive_IT()
        (++) DAL_SMARTCARD_IRQHandler()

    (#) Non Blocking mode functions with DMA are :
        (++) DAL_SMARTCARD_Transmit_DMA()
        (++) DAL_SMARTCARD_Receive_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) DAL_SMARTCARD_TxCpltCallback()
        (++) DAL_SMARTCARD_RxCpltCallback()
        (++) DAL_SMARTCARD_ErrorCallback()

    (#) Non-Blocking mode transfers could be aborted using Abort API's :
        (+) DAL_SMARTCARD_Abort()
        (+) DAL_SMARTCARD_AbortTransmit()
        (+) DAL_SMARTCARD_AbortReceive()
        (+) DAL_SMARTCARD_Abort_IT()
        (+) DAL_SMARTCARD_AbortTransmit_IT()
        (+) DAL_SMARTCARD_AbortReceive_IT()

    (#) For Abort services based on interrupts (DAL_SMARTCARD_Abortxxx_IT), a set of Abort Complete Callbacks are provided:
        (+) DAL_SMARTCARD_AbortCpltCallback()
        (+) DAL_SMARTCARD_AbortTransmitCpltCallback()
        (+) DAL_SMARTCARD_AbortReceiveCpltCallback()

    (#) In Non-Blocking mode transfers, possible errors are split into 2 categories.
        Errors are handled as follows :
       (+) Error is considered as Recoverable and non blocking : Transfer could go till end, but error severity is
           to be evaluated by user : this concerns Frame Error, Parity Error or Noise Error in Interrupt mode reception .
           Received character is then retrieved and stored in Rx buffer, Error code is set to allow user to identify error type,
           and DAL_SMARTCARD_ErrorCallback() user callback is executed. Transfer is kept ongoing on SMARTCARD side.
           If user wants to abort it, Abort services should be called by user.
       (+) Error is considered as Blocking : Transfer could not be completed properly and is aborted.
           This concerns Frame Error in Interrupt mode transmission, Overrun Error in Interrupt mode reception and all errors in DMA mode.
           Error code is set to allow user to identify error type, and DAL_SMARTCARD_ErrorCallback() user callback is executed.

@endverbatim
  * @{
  */

/**
  * @brief Send an amount of data in blocking mode
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @param  pData  Pointer to data buffer
  * @param  Size   Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_Transmit(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  const uint8_t *tmp = pData;
  uint32_t tickstart = 0U;

  if(hsc->gState == DAL_SMARTCARD_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hsc);

    hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
    hsc->gState = DAL_SMARTCARD_STATE_BUSY_TX;

    /* Init tickstart for timeout management */
    tickstart = DAL_GetTick();

    hsc->TxXferSize = Size;
    hsc->TxXferCount = Size;
    while(hsc->TxXferCount > 0U)
    {
      hsc->TxXferCount--;
      if(SMARTCARD_WaitOnFlagUntilTimeout(hsc, SMARTCARD_FLAG_TXE, RESET, tickstart, Timeout) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }
      hsc->Instance->DATA = (uint8_t)(*tmp & 0xFFU);
      tmp++;
    }

    if(SMARTCARD_WaitOnFlagUntilTimeout(hsc, SMARTCARD_FLAG_TC, RESET, tickstart, Timeout) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

	/* At end of Tx process, restore hsc->gState to Ready */
    hsc->gState = DAL_SMARTCARD_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hsc);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data in blocking mode
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @param  pData  Pointer to data buffer
  * @param  Size   Amount of data to be received
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_Receive(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint8_t  *tmp = pData;
  uint32_t tickstart = 0U;

  if(hsc->RxState == DAL_SMARTCARD_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hsc);

    hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
    hsc->RxState = DAL_SMARTCARD_STATE_BUSY_RX;

    /* Init tickstart for timeout management */
    tickstart = DAL_GetTick();

    hsc->RxXferSize = Size;
    hsc->RxXferCount = Size;

    /* Check the remain data to be received */
    while(hsc->RxXferCount > 0U)
    {
      hsc->RxXferCount--;
      if(SMARTCARD_WaitOnFlagUntilTimeout(hsc, SMARTCARD_FLAG_RXNE, RESET, tickstart, Timeout) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }
      *tmp = (uint8_t)(hsc->Instance->DATA & (uint8_t)0xFFU);
      tmp++;
    }

    /* At end of Rx process, restore hsc->RxState to Ready */
    hsc->RxState = DAL_SMARTCARD_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hsc);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Send an amount of data in non blocking mode
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @param  pData  Pointer to data buffer
  * @param  Size   Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_Transmit_IT(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size)
{
  /* Check that a Tx process is not already ongoing */
  if(hsc->gState == DAL_SMARTCARD_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hsc);

    hsc->pTxBuffPtr = pData;
    hsc->TxXferSize = Size;
    hsc->TxXferCount = Size;

    hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
    hsc->gState = DAL_SMARTCARD_STATE_BUSY_TX;

    /* Process Unlocked */
    __DAL_UNLOCK(hsc);

    /* Enable the SMARTCARD Parity Error Interrupt */
    SET_BIT(hsc->Instance->CTRL1, USART_CTRL1_PEIEN);

    /* Disable the SMARTCARD Error Interrupt: (Frame error, noise error, overrun error) */
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Enable the SMARTCARD Transmit data register empty Interrupt */
    SET_BIT(hsc->Instance->CTRL1, USART_CTRL1_TXBEIEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data in non blocking mode
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @param  pData  Pointer to data buffer
  * @param  Size   Amount of data to be received
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_Receive_IT(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size)
{
  /* Check that a Rx process is not already ongoing */
  if(hsc->RxState == DAL_SMARTCARD_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hsc);

    hsc->pRxBuffPtr = pData;
    hsc->RxXferSize = Size;
    hsc->RxXferCount = Size;

    hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
    hsc->RxState = DAL_SMARTCARD_STATE_BUSY_RX;

    /* Process Unlocked */
    __DAL_UNLOCK(hsc);

    /* Enable the SMARTCARD Parity Error and Data Register not empty Interrupts */
    SET_BIT(hsc->Instance->CTRL1, USART_CTRL1_PEIEN| USART_CTRL1_RXBNEIEN);

    /* Enable the SMARTCARD Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Send an amount of data in non blocking mode
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @param  pData  Pointer to data buffer
  * @param  Size   Amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_Transmit_DMA(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size)
{
  const uint32_t *tmp;

  /* Check that a Tx process is not already ongoing */
  if(hsc->gState == DAL_SMARTCARD_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hsc);

    hsc->pTxBuffPtr = pData;
    hsc->TxXferSize = Size;
    hsc->TxXferCount = Size;

    hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
    hsc->gState = DAL_SMARTCARD_STATE_BUSY_TX;

    /* Set the SMARTCARD DMA transfer complete callback */
    hsc->hdmatx->XferCpltCallback = SMARTCARD_DMATransmitCplt;

    /* Set the DMA error callback */
    hsc->hdmatx->XferErrorCallback = SMARTCARD_DMAError;

    /* Set the DMA abort callback */
    hsc->hdmatx->XferAbortCallback = NULL;

    /* Enable the SMARTCARD transmit DMA stream */
    tmp = (const uint32_t*)&pData;
    DAL_DMA_Start_IT(hsc->hdmatx, *(const uint32_t*)tmp, (uint32_t)&hsc->Instance->DATA, Size);

     /* Clear the TC flag in the STS register by writing 0 to it */
    __DAL_SMARTCARD_CLEAR_FLAG(hsc, SMARTCARD_FLAG_TC);

    /* Process Unlocked */
    __DAL_UNLOCK(hsc);

    /* Enable the DMA transfer for transmit request by setting the DMAT bit
    in the SMARTCARD CTRL3 register */
    SET_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data in non blocking mode
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @param  pData  Pointer to data buffer
  * @param  Size   Amount of data to be received
  * @note   When the SMARTCARD parity is enabled (PCE = 1) the data received contain the parity bit.s
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SMARTCARD_Receive_DMA(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size)
{
  uint32_t *tmp;

  /* Check that a Rx process is not already ongoing */
  if(hsc->RxState == DAL_SMARTCARD_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hsc);

    hsc->pRxBuffPtr = pData;
    hsc->RxXferSize = Size;

    hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
    hsc->RxState = DAL_SMARTCARD_STATE_BUSY_RX;

    /* Set the SMARTCARD DMA transfer complete callback */
    hsc->hdmarx->XferCpltCallback = SMARTCARD_DMAReceiveCplt;

    /* Set the DMA error callback */
    hsc->hdmarx->XferErrorCallback = SMARTCARD_DMAError;

    /* Set the DMA abort callback */
    hsc->hdmatx->XferAbortCallback = NULL;

    /* Enable the DMA stream */
    tmp = (uint32_t*)&pData;
    DAL_DMA_Start_IT(hsc->hdmarx, (uint32_t)&hsc->Instance->DATA, *(uint32_t*)tmp, Size);

    /* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
    __DAL_SMARTCARD_CLEAR_OREFLAG(hsc);

    /* Process Unlocked */
    __DAL_UNLOCK(hsc);

    /* Enable the SMARTCARD Parity Error Interrupt */
    SET_BIT(hsc->Instance->CTRL1, USART_CTRL1_PEIEN);

    /* Enable the SMARTCARD Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the SMARTCARD CTRL3 register */
    SET_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Abort ongoing transfers (blocking mode).
  * @param  hsc SMARTCARD handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_SMARTCARD_Abort(SMARTCARD_HandleTypeDef *hsc)
{
  /* Disable TXBEIEN, TXCIEN, RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Disable the SMARTCARD DMA Tx request if enabled */
  if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the SMARTCARD DMA Tx channel : use blocking DMA Abort API (no callback) */
    if(hsc->hdmatx != NULL)
    {
      /* Set the SMARTCARD DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hsc->hdmatx->XferAbortCallback = NULL;

      DAL_DMA_Abort(hsc->hdmatx);
    }
  }

  /* Disable the SMARTCARD DMA Rx request if enabled */
  if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the SMARTCARD DMA Rx channel : use blocking DMA Abort API (no callback) */
    if(hsc->hdmarx != NULL)
    {
      /* Set the SMARTCARD DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hsc->hdmarx->XferAbortCallback = NULL;

      DAL_DMA_Abort(hsc->hdmarx);
    }
  }

  /* Reset Tx and Rx transfer counters */
  hsc->TxXferCount = 0x00U;
  hsc->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;

  /* Restore hsc->RxState and hsc->gState to Ready */
  hsc->RxState = DAL_SMARTCARD_STATE_READY;
  hsc->gState = DAL_SMARTCARD_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (blocking mode).
  * @param  hsc SMARTCARD handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SMARTCARD Interrupts (Tx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_SMARTCARD_AbortTransmit(SMARTCARD_HandleTypeDef *hsc)
{
  /* Disable TXEIE and TCIE interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* Disable the SMARTCARD DMA Tx request if enabled */
  if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the SMARTCARD DMA Tx channel : use blocking DMA Abort API (no callback) */
    if(hsc->hdmatx != NULL)
    {
      /* Set the SMARTCARD DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hsc->hdmatx->XferAbortCallback = NULL;

      DAL_DMA_Abort(hsc->hdmatx);
    }
  }

  /* Reset Tx transfer counter */
  hsc->TxXferCount = 0x00U;

  /* Restore hsc->gState to Ready */
  hsc->gState = DAL_SMARTCARD_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (blocking mode).
  * @param  hsc SMARTCARD handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_SMARTCARD_AbortReceive(SMARTCARD_HandleTypeDef *hsc)
{
  /* Disable RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Disable the SMARTCARD DMA Rx request if enabled */
  if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the SMARTCARD DMA Rx channel : use blocking DMA Abort API (no callback) */
    if(hsc->hdmarx != NULL)
    {
      /* Set the SMARTCARD DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hsc->hdmarx->XferAbortCallback = NULL;

      DAL_DMA_Abort(hsc->hdmarx);
    }
  }

  /* Reset Rx transfer counter */
  hsc->RxXferCount = 0x00U;

  /* Restore hsc->RxState to Ready */
  hsc->RxState = DAL_SMARTCARD_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing transfers (Interrupt mode).
  * @param  hsc SMARTCARD handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_SMARTCARD_Abort_IT(SMARTCARD_HandleTypeDef *hsc)
{
  uint32_t AbortCplt = 0x01U;

  /* Disable TXBEIEN, TXCIEN, RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* If DMA Tx and/or DMA Rx Handles are associated to SMARTCARD Handle, DMA Abort complete callbacks should be initialised
     before any call to DMA Abort functions */
  /* DMA Tx Handle is valid */
  if(hsc->hdmatx != NULL)
  {
    /* Set DMA Abort Complete callback if SMARTCARD DMA Tx request if enabled.
       Otherwise, set it to NULL */
    if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN))
    {
      hsc->hdmatx->XferAbortCallback = SMARTCARD_DMATxAbortCallback;
    }
    else
    {
      hsc->hdmatx->XferAbortCallback = NULL;
    }
  }
  /* DMA Rx Handle is valid */
  if(hsc->hdmarx != NULL)
  {
    /* Set DMA Abort Complete callback if SMARTCARD DMA Rx request if enabled.
       Otherwise, set it to NULL */
    if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN))
    {
      hsc->hdmarx->XferAbortCallback = SMARTCARD_DMARxAbortCallback;
    }
    else
    {
      hsc->hdmarx->XferAbortCallback = NULL;
    }
  }

  /* Disable the SMARTCARD DMA Tx request if enabled */
  if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    /* Disable DMA Tx at SMARTCARD level */
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the SMARTCARD DMA Tx channel : use non blocking DMA Abort API (callback) */
    if(hsc->hdmatx != NULL)
    {
      /* SMARTCARD Tx DMA Abort callback has already been initialised :
         will lead to call DAL_SMARTCARD_AbortCpltCallback() at end of DMA abort procedure */

      /* Abort DMA TX */
      if(DAL_DMA_Abort_IT(hsc->hdmatx) != DAL_OK)
      {
        hsc->hdmatx->XferAbortCallback = NULL;
      }
      else
      {
        AbortCplt = 0x00U;
      }
    }
  }

  /* Disable the SMARTCARD DMA Rx request if enabled */
  if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the SMARTCARD DMA Rx channel : use non blocking DMA Abort API (callback) */
    if(hsc->hdmarx != NULL)
    {
      /* SMARTCARD Rx DMA Abort callback has already been initialised :
         will lead to call DAL_SMARTCARD_AbortCpltCallback() at end of DMA abort procedure */

      /* Abort DMA RX */
      if(DAL_DMA_Abort_IT(hsc->hdmarx) != DAL_OK)
      {
        hsc->hdmarx->XferAbortCallback = NULL;
        AbortCplt = 0x01U;
      }
      else
      {
        AbortCplt = 0x00U;
      }
    }
  }

  /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
  if(AbortCplt == 0x01U)
  {
    /* Reset Tx and Rx transfer counters */
    hsc->TxXferCount = 0x00U;
    hsc->RxXferCount = 0x00U;

    /* Reset ErrorCode */
    hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;

    /* Restore hsc->gState and hsc->RxState to Ready */
    hsc->gState  = DAL_SMARTCARD_STATE_READY;
    hsc->RxState = DAL_SMARTCARD_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
    /* Call registered Abort complete callback */
    hsc->AbortCpltCallback(hsc);
#else
    /* Call legacy weak Abort complete callback */
    DAL_SMARTCARD_AbortCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
  }
  return DAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (Interrupt mode).
  * @param  hsc SMARTCARD handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SMARTCARD Interrupts (Tx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_SMARTCARD_AbortTransmit_IT(SMARTCARD_HandleTypeDef *hsc)
{
  /* Disable TXEIE and TCIE interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* Disable the SMARTCARD DMA Tx request if enabled */
  if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the SMARTCARD DMA Tx channel : use blocking DMA Abort API (no callback) */
    if(hsc->hdmatx != NULL)
    {
      /* Set the SMARTCARD DMA Abort callback :
         will lead to call DAL_SMARTCARD_AbortCpltCallback() at end of DMA abort procedure */
      hsc->hdmatx->XferAbortCallback = SMARTCARD_DMATxOnlyAbortCallback;

      /* Abort DMA TX */
      if(DAL_DMA_Abort_IT(hsc->hdmatx) != DAL_OK)
      {
        /* Call Directly hsc->hdmatx->XferAbortCallback function in case of error */
        hsc->hdmatx->XferAbortCallback(hsc->hdmatx);
      }
    }
    else
    {
      /* Reset Tx transfer counter */
      hsc->TxXferCount = 0x00U;

      /* Restore hsc->gState to Ready */
      hsc->gState = DAL_SMARTCARD_STATE_READY;

      /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
      /* Call registered Abort Transmit Complete Callback */
      hsc->AbortTransmitCpltCallback(hsc);
#else
      /* Call legacy weak Abort Transmit Complete Callback */
      DAL_SMARTCARD_AbortTransmitCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
    }
  }
  else
  {
    /* Reset Tx transfer counter */
    hsc->TxXferCount = 0x00U;

    /* Restore hsc->gState to Ready */
    hsc->gState = DAL_SMARTCARD_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
    /* Call registered Abort Transmit Complete Callback */
    hsc->AbortTransmitCpltCallback(hsc);
#else
    /* Call legacy weak Abort Transmit Complete Callback */
    DAL_SMARTCARD_AbortTransmitCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
  }

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (Interrupt mode).
  * @param  hsc SMARTCARD handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SMARTCARD Interrupts (Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_SMARTCARD_AbortReceive_IT(SMARTCARD_HandleTypeDef *hsc)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Disable the SMARTCARD DMA Rx request if enabled */
  if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the SMARTCARD DMA Rx channel : use blocking DMA Abort API (no callback) */
    if(hsc->hdmarx != NULL)
    {
      /* Set the SMARTCARD DMA Abort callback :
         will lead to call DAL_SMARTCARD_AbortCpltCallback() at end of DMA abort procedure */
      hsc->hdmarx->XferAbortCallback = SMARTCARD_DMARxOnlyAbortCallback;

      /* Abort DMA RX */
      if(DAL_DMA_Abort_IT(hsc->hdmarx) != DAL_OK)
      {
        /* Call Directly hsc->hdmarx->XferAbortCallback function in case of error */
        hsc->hdmarx->XferAbortCallback(hsc->hdmarx);
      }
    }
    else
    {
      /* Reset Rx transfer counter */
      hsc->RxXferCount = 0x00U;

      /* Restore hsc->RxState to Ready */
      hsc->RxState = DAL_SMARTCARD_STATE_READY;

      /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
      /* Call registered Abort Receive Complete Callback */
      hsc->AbortReceiveCpltCallback(hsc);
#else
      /* Call legacy weak Abort Receive Complete Callback */
      DAL_SMARTCARD_AbortReceiveCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
    }
  }
  else
  {
    /* Reset Rx transfer counter */
    hsc->RxXferCount = 0x00U;

    /* Restore hsc->RxState to Ready */
    hsc->RxState = DAL_SMARTCARD_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
    /* Call registered Abort Receive Complete Callback */
    hsc->AbortReceiveCpltCallback(hsc);
#else
    /* Call legacy weak Abort Receive Complete Callback */
    DAL_SMARTCARD_AbortReceiveCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
  }

  return DAL_OK;
}

/**
  * @brief This function handles SMARTCARD interrupt request.
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
void DAL_SMARTCARD_IRQHandler(SMARTCARD_HandleTypeDef *hsc)
{
  uint32_t isrflags   = READ_REG(hsc->Instance->STS);
  uint32_t cr1its     = READ_REG(hsc->Instance->CTRL1);
  uint32_t cr3its     = READ_REG(hsc->Instance->CTRL3);
  uint32_t dmarequest = 0x00U;
  uint32_t errorflags = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_STS_PEFLG | USART_STS_FEFLG | USART_STS_OVREFLG | USART_STS_NEFLG));
  if(errorflags == RESET)
  {
    /* SMARTCARD in mode Receiver -------------------------------------------------*/
    if(((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))
    {
      SMARTCARD_Receive_IT(hsc);
      return;
    }
  }

  /* If some errors occur */
  if((errorflags != RESET) && (((cr3its & USART_CTRL3_ERRIEN) != RESET) || ((cr1its & (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN)) != RESET)))
  {
    /* SMARTCARD parity error interrupt occurred ---------------------------*/
    if(((isrflags & SMARTCARD_FLAG_PE) != RESET) && ((cr1its & USART_CTRL1_PEIEN) != RESET))
    {
      hsc->ErrorCode |= DAL_SMARTCARD_ERROR_PE;
    }

    /* SMARTCARD frame error interrupt occurred ----------------------------*/
    if(((isrflags & SMARTCARD_FLAG_FE) != RESET) && ((cr3its & USART_CTRL3_ERRIEN) != RESET))
    {
      hsc->ErrorCode |= DAL_SMARTCARD_ERROR_FE;
    }

    /* SMARTCARD noise error interrupt occurred ----------------------------*/
    if(((isrflags & SMARTCARD_FLAG_NE) != RESET) && ((cr3its & USART_CTRL3_ERRIEN) != RESET))
    {
      hsc->ErrorCode |= DAL_SMARTCARD_ERROR_NE;
    }

    /* SMARTCARD Over-Run interrupt occurred -------------------------------*/
    if(((isrflags & SMARTCARD_FLAG_ORE) != RESET) && (((cr1its & USART_CTRL1_RXBNEIEN) != RESET) || ((cr3its & USART_CTRL3_ERRIEN) != RESET)))
    {
      hsc->ErrorCode |= DAL_SMARTCARD_ERROR_ORE;
    }
    /* Call the Error call Back in case of Errors --------------------------*/
    if(hsc->ErrorCode != DAL_SMARTCARD_ERROR_NONE)
    {
      /* SMARTCARD in mode Receiver ----------------------------------------*/
      if(((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))
      {
        SMARTCARD_Receive_IT(hsc);
      }

      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
         consider error as blocking */
      dmarequest = DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);
      if(((hsc->ErrorCode & DAL_SMARTCARD_ERROR_ORE) != RESET) || dmarequest)
      {
        /* Blocking error : transfer is aborted
          Set the SMARTCARD state ready to be able to start again the process,
          Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        SMARTCARD_EndRxTransfer(hsc);
        /* Disable the SMARTCARD DMA Rx request if enabled */
        if(DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN))
        {
          CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);

          /* Abort the SMARTCARD DMA Rx channel */
          if(hsc->hdmarx != NULL)
          {
            /* Set the SMARTCARD DMA Abort callback :
              will lead to call DAL_SMARTCARD_ErrorCallback() at end of DMA abort procedure */
            hsc->hdmarx->XferAbortCallback = SMARTCARD_DMAAbortOnError;

           if(DAL_DMA_Abort_IT(hsc->hdmarx) != DAL_OK)
            {
              /* Call Directly XferAbortCallback function in case of error */
              hsc->hdmarx->XferAbortCallback(hsc->hdmarx);
            }
          }
          else
          {
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
            /* Call registered user error callback */
            hsc->ErrorCallback(hsc);
#else
            /* Call legacy weak user error callback */
            DAL_SMARTCARD_ErrorCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
          }
        }
        else
        {
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
          /* Call registered user error callback */
          hsc->ErrorCallback(hsc);
#else
          /* Call legacy weak user error callback */
          DAL_SMARTCARD_ErrorCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
        }
      }
      else
      {
        /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
        /* Call registered user error callback */
        hsc->ErrorCallback(hsc);
#else
        /* Call legacy weak user error callback */
        DAL_SMARTCARD_ErrorCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
        hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;
      }
    }
    return;
  } /* End if some error occurs */

  /* SMARTCARD in mode Transmitter ------------------------------------------*/
  if(((isrflags & SMARTCARD_FLAG_TXE) != RESET) && ((cr1its & USART_CTRL1_TXBEIEN) != RESET))
  {
    SMARTCARD_Transmit_IT(hsc);
    return;
  }

  /* SMARTCARD in mode Transmitter (transmission end) -----------------------*/
  if(((isrflags & SMARTCARD_FLAG_TC) != RESET) && ((cr1its & USART_CTRL1_TXCIEN) != RESET))
  {
    SMARTCARD_EndTransmit_IT(hsc);
    return;
  }
}

/**
  * @brief Tx Transfer completed callbacks
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
__weak void DAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef *hsc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMARTCARD_TxCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief Rx Transfer completed callback
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
__weak void DAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef *hsc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMARTCARD_RxCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief SMARTCARD error callback
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
__weak void DAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMARTCARD_ErrorCallback can be implemented in the user file.
   */
}

/**
  * @brief  SMARTCARD Abort Complete callback.
  * @param  hsc SMARTCARD handle.
  * @retval None
  */
__weak void DAL_SMARTCARD_AbortCpltCallback (SMARTCARD_HandleTypeDef *hsc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMARTCARD_AbortCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  SMARTCARD Abort Transmit Complete callback.
  * @param  hsc SMARTCARD handle.
  * @retval None
  */
__weak void DAL_SMARTCARD_AbortTransmitCpltCallback (SMARTCARD_HandleTypeDef *hsc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hsc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMARTCARD_AbortTransmitCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  SMARTCARD Abort Receive Complete callback.
  * @param  hsc SMARTCARD handle.
  * @retval None
  */
__weak void DAL_SMARTCARD_AbortReceiveCpltCallback (SMARTCARD_HandleTypeDef *hsc)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hsc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SMARTCARD_AbortReceiveCpltCallback can be implemented in the user file.
   */
}

/**
  * @}
  */

/** @defgroup SMARTCARD_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   SMARTCARD State and Errors functions
  *
@verbatim
 ===============================================================================
                ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SmartCard.
     (+) DAL_SMARTCARD_GetState() API can be helpful to check in run-time the state of the SmartCard peripheral.
     (+) DAL_SMARTCARD_GetError() check in run-time errors that could be occurred during communication.
@endverbatim
  * @{
  */

/**
  * @brief Return the SMARTCARD handle state
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval DAL state
  */
DAL_SMARTCARD_StateTypeDef DAL_SMARTCARD_GetState(SMARTCARD_HandleTypeDef *hsc)
{
  uint32_t temp1= 0x00U, temp2 = 0x00U;
  temp1 = hsc->gState;
  temp2 = hsc->RxState;

  return (DAL_SMARTCARD_StateTypeDef)(temp1 | temp2);
}

/**
  * @brief  Return the SMARTCARD error code
  * @param  hsc  Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *              the configuration information for the specified SMARTCARD.
  * @retval SMARTCARD Error Code
  */
uint32_t DAL_SMARTCARD_GetError(SMARTCARD_HandleTypeDef *hsc)
{
  return hsc->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup SMARTCARD_Private_Functions SMARTCARD Private Functions
  * @{
  */

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
/**
  * @brief  Initialize the callbacks to their default values.
  * @param  hsc SMARTCARD handle.
  * @retval none
  */
void SMARTCARD_InitCallbacksToDefault(SMARTCARD_HandleTypeDef *hsc)
{
  /* Init the SMARTCARD Callback settings */
  hsc->TxCpltCallback            = DAL_SMARTCARD_TxCpltCallback;            /* Legacy weak TxCpltCallback            */
  hsc->RxCpltCallback            = DAL_SMARTCARD_RxCpltCallback;            /* Legacy weak RxCpltCallback            */
  hsc->ErrorCallback             = DAL_SMARTCARD_ErrorCallback;             /* Legacy weak ErrorCallback             */
  hsc->AbortCpltCallback         = DAL_SMARTCARD_AbortCpltCallback;         /* Legacy weak AbortCpltCallback         */
  hsc->AbortTransmitCpltCallback = DAL_SMARTCARD_AbortTransmitCpltCallback; /* Legacy weak AbortTransmitCpltCallback */
  hsc->AbortReceiveCpltCallback  = DAL_SMARTCARD_AbortReceiveCpltCallback;  /* Legacy weak AbortReceiveCpltCallback  */

}
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACKS */

/**
  * @brief DMA SMARTCARD transmit process complete callback
  * @param  hdma   Pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void SMARTCARD_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  SMARTCARD_HandleTypeDef* hsc = ( SMARTCARD_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  hsc->TxXferCount = 0U;

  /* Disable the DMA transfer for transmit request by setting the DMAT bit
     in the USART CTRL3 register */
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN);

  /* Enable the SMARTCARD Transmit Complete Interrupt */
  SET_BIT(hsc->Instance->CTRL1, USART_CTRL1_TXCIEN);
}

/**
  * @brief DMA SMARTCARD receive process complete callback
  * @param  hdma   Pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void SMARTCARD_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SMARTCARD_HandleTypeDef* hsc = ( SMARTCARD_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  hsc->RxXferCount = 0U;

  /* Disable RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Disable the DMA transfer for the receiver request by setting the DMAR bit
     in the USART CTRL3 register */
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);

  /* At end of Rx process, restore hsc->RxState to Ready */
  hsc->RxState = DAL_SMARTCARD_STATE_READY;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /* Call registered Rx complete callback */
  hsc->RxCpltCallback(hsc);
#else
  /* Call legacy weak Rx complete callback */
  DAL_SMARTCARD_RxCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
}

/**
  * @brief DMA SMARTCARD communication error callback
  * @param  hdma   Pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void SMARTCARD_DMAError(DMA_HandleTypeDef *hdma)
{
  uint32_t dmarequest = 0x00U;
  SMARTCARD_HandleTypeDef* hsc = ( SMARTCARD_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hsc->RxXferCount = 0U;
  hsc->TxXferCount = 0U;
  hsc->ErrorCode = DAL_SMARTCARD_ERROR_DMA;

  /* Stop SMARTCARD DMA Tx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if((hsc->gState == DAL_SMARTCARD_STATE_BUSY_TX) && dmarequest)
  {
    SMARTCARD_EndTxTransfer(hsc);
  }

  /* Stop SMARTCARD DMA Rx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(hsc->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if((hsc->RxState == DAL_SMARTCARD_STATE_BUSY_RX) && dmarequest)
  {
    SMARTCARD_EndRxTransfer(hsc);
  }

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /* Call registered user error callback */
  hsc->ErrorCallback(hsc);
#else
  /* Call legacy weak user error callback */
  DAL_SMARTCARD_ErrorCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
}

/**
  * @brief  This function handles SMARTCARD Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @param  Flag   Specifies the SMARTCARD flag to check.
  * @param  Status The actual Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef SMARTCARD_WaitOnFlagUntilTimeout(SMARTCARD_HandleTypeDef *hsc, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while((__DAL_SMARTCARD_GET_FLAG(hsc, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if(Timeout != DAL_MAX_DELAY)
    {
      if((Timeout == 0U)||((DAL_GetTick() - Tickstart ) > Timeout))
      {
        /* Disable TXBEIEN and RXBNEIEN interrupts for the interrupt process */
        CLEAR_BIT(hsc->Instance->CTRL1, USART_CTRL1_TXBEIEN);
        CLEAR_BIT(hsc->Instance->CTRL1, USART_CTRL1_RXBNEIEN);

        hsc->gState= DAL_SMARTCARD_STATE_READY;
        hsc->RxState= DAL_SMARTCARD_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(hsc);

        return DAL_TIMEOUT;
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  End ongoing Tx transfer on SMARTCARD peripheral (following error detection or Transmit completion).
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
static void SMARTCARD_EndTxTransfer(SMARTCARD_HandleTypeDef *hsc)
{
  /* At end of Tx process, restore hsc->gState to Ready */
  hsc->gState = DAL_SMARTCARD_STATE_READY;

  /* Disable TXBEIEN and TXCIEN interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
}


/**
  * @brief  End ongoing Rx transfer on SMARTCARD peripheral (following error detection or Reception completion).
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
static void SMARTCARD_EndRxTransfer(SMARTCARD_HandleTypeDef *hsc)
{
  /* At end of Rx process, restore hsc->RxState to Ready */
  hsc->RxState = DAL_SMARTCARD_STATE_READY;

  /* Disable RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);
}

/**
  * @brief Send an amount of data in non blocking mode
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval DAL status
  */
static DAL_StatusTypeDef SMARTCARD_Transmit_IT(SMARTCARD_HandleTypeDef *hsc)
{

  /* Check that a Tx process is ongoing */
  if(hsc->gState == DAL_SMARTCARD_STATE_BUSY_TX)
  {
    hsc->Instance->DATA = (uint8_t)(*hsc->pTxBuffPtr & 0xFFU);
    hsc->pTxBuffPtr++;

    if(--hsc->TxXferCount == 0U)
    {
      /* Disable the SMARTCARD Transmit data register empty Interrupt */
      CLEAR_BIT(hsc->Instance->CTRL1, USART_CTRL1_TXBEIEN);

      /* Enable the SMARTCARD Transmit Complete Interrupt */
      SET_BIT(hsc->Instance->CTRL1, USART_CTRL1_TXCIEN);
    }

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Wraps up transmission in non blocking mode.
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for the specified SMARTCARD module.
  * @retval DAL status
  */
static DAL_StatusTypeDef SMARTCARD_EndTransmit_IT(SMARTCARD_HandleTypeDef *hsc)
{
  /* Disable the SMARTCARD Transmit Complete Interrupt */
  CLEAR_BIT(hsc->Instance->CTRL1, USART_CTRL1_TXCIEN);

  /* Disable the SMARTCARD Error Interrupt: (Frame error, noise error, overrun error) */
  CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Tx process is ended, restore hsc->gState to Ready */
  hsc->gState = DAL_SMARTCARD_STATE_READY;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /* Call registered Tx complete callback */
  hsc->TxCpltCallback(hsc);
#else
  /* Call legacy weak Tx complete callback */
  DAL_SMARTCARD_TxCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */

  return DAL_OK;
}

/**
  * @brief Receive an amount of data in non blocking mode
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval DAL status
  */
static DAL_StatusTypeDef SMARTCARD_Receive_IT(SMARTCARD_HandleTypeDef *hsc)
{

  /* Check that a Rx process is ongoing */
  if(hsc->RxState == DAL_SMARTCARD_STATE_BUSY_RX)
  {
    *hsc->pRxBuffPtr = (uint8_t)(hsc->Instance->DATA & (uint8_t)0xFFU);
    hsc->pRxBuffPtr++;

    if(--hsc->RxXferCount == 0U)
    {
      CLEAR_BIT(hsc->Instance->CTRL1, USART_CTRL1_RXBNEIEN);

      /* Disable the SMARTCARD Parity Error Interrupt */
      CLEAR_BIT(hsc->Instance->CTRL1, USART_CTRL1_PEIEN);

      /* Disable the SMARTCARD Error Interrupt: (Frame error, noise error, overrun error) */
      CLEAR_BIT(hsc->Instance->CTRL3, USART_CTRL3_ERRIEN);

      /* Rx process is completed, restore hsc->RxState to Ready */
      hsc->RxState = DAL_SMARTCARD_STATE_READY;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
      /* Call registered Rx complete callback */
      hsc->RxCpltCallback(hsc);
#else
      /* Call legacy weak Rx complete callback */
      DAL_SMARTCARD_RxCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */

      return DAL_OK;
    }
    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  DMA SMARTCARD communication abort callback, when initiated by DAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  * @retval None
  */
static void SMARTCARD_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  SMARTCARD_HandleTypeDef* hsc = (SMARTCARD_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  hsc->RxXferCount = 0x00U;
  hsc->TxXferCount = 0x00U;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /* Call registered user error callback */
  hsc->ErrorCallback(hsc);
#else
  /* Call legacy weak user error callback */
  DAL_SMARTCARD_ErrorCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
}

/**
  * @brief  DMA SMARTCARD Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SMARTCARD_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
{
  SMARTCARD_HandleTypeDef* hsc = ( SMARTCARD_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  hsc->hdmatx->XferAbortCallback = NULL;

  /* Check if an Abort process is still ongoing */
  if(hsc->hdmarx != NULL)
  {
    if(hsc->hdmarx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hsc->TxXferCount = 0x00U;
  hsc->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;

  /* Restore hsc->gState and hsc->RxState to Ready */
  hsc->gState  = DAL_SMARTCARD_STATE_READY;
  hsc->RxState = DAL_SMARTCARD_STATE_READY;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /* Call registered Abort complete callback */
  hsc->AbortCpltCallback(hsc);
#else
  /* Call legacy weak Abort complete callback */
  DAL_SMARTCARD_AbortCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
}

/**
  * @brief  DMA SMARTCARD Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SMARTCARD_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
{
  SMARTCARD_HandleTypeDef* hsc = ( SMARTCARD_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  hsc->hdmarx->XferAbortCallback = NULL;

  /* Check if an Abort process is still ongoing */
  if(hsc->hdmatx != NULL)
  {
    if(hsc->hdmatx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hsc->TxXferCount = 0x00U;
  hsc->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  hsc->ErrorCode = DAL_SMARTCARD_ERROR_NONE;

  /* Restore hsc->gState and hsc->RxState to Ready */
  hsc->gState  = DAL_SMARTCARD_STATE_READY;
  hsc->RxState = DAL_SMARTCARD_STATE_READY;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /* Call registered Abort complete callback */
  hsc->AbortCpltCallback(hsc);
#else
  /* Call legacy weak Abort complete callback */
  DAL_SMARTCARD_AbortCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
}

/**
  * @brief  DMA SMARTCARD Tx communication abort callback, when initiated by user by a call to
  *         DAL_SMARTCARD_AbortTransmit_IT API (Abort only Tx transfer)
  *         (This callback is executed at end of DMA Tx Abort procedure following user abort request,
  *         and leads to user Tx Abort Complete callback execution).
  * @param  hdma DMA handle.
  * @retval None
  */
static void SMARTCARD_DMATxOnlyAbortCallback(DMA_HandleTypeDef *hdma)
{
  SMARTCARD_HandleTypeDef* hsc = ( SMARTCARD_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  hsc->TxXferCount = 0x00U;

  /* Restore hsc->gState to Ready */
  hsc->gState = DAL_SMARTCARD_STATE_READY;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Transmit Complete Callback */
  hsc->AbortTransmitCpltCallback(hsc);
#else
  /* Call legacy weak Abort Transmit Complete Callback */
  DAL_SMARTCARD_AbortTransmitCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
}

/**
  * @brief  DMA SMARTCARD Rx communication abort callback, when initiated by user by a call to
  *         DAL_SMARTCARD_AbortReceive_IT API (Abort only Rx transfer)
  *         (This callback is executed at end of DMA Rx Abort procedure following user abort request,
  *         and leads to user Rx Abort Complete callback execution).
  * @param  hdma DMA handle.
  * @retval None
  */
static void SMARTCARD_DMARxOnlyAbortCallback(DMA_HandleTypeDef *hdma)
{
  SMARTCARD_HandleTypeDef* hsc = ( SMARTCARD_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  hsc->RxXferCount = 0x00U;

  /* Restore hsc->RxState to Ready */
  hsc->RxState = DAL_SMARTCARD_STATE_READY;

#if (USE_DAL_SMARTCARD_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Receive Complete Callback */
  hsc->AbortReceiveCpltCallback(hsc);
#else
  /* Call legacy weak Abort Receive Complete Callback */
  DAL_SMARTCARD_AbortReceiveCpltCallback(hsc);
#endif /* USE_DAL_SMARTCARD_REGISTER_CALLBACK */
}

/**
  * @brief Configure the SMARTCARD peripheral
  * @param  hsc    Pointer to a SMARTCARD_HandleTypeDef structure that contains
  *                the configuration information for SMARTCARD module.
  * @retval None
  */
static void SMARTCARD_SetConfig(SMARTCARD_HandleTypeDef *hsc)
{
  uint32_t tmpreg = 0x00U;
  uint32_t pclk;

  /* Check the parameters */
  ASSERT_PARAM(IS_SMARTCARD_INSTANCE(hsc->Instance));
  ASSERT_PARAM(IS_SMARTCARD_POLARITY(hsc->Init.CLKPolarity));
  ASSERT_PARAM(IS_SMARTCARD_PHASE(hsc->Init.CLKPhase));
  ASSERT_PARAM(IS_SMARTCARD_LASTBIT(hsc->Init.CLKLastBit));
  ASSERT_PARAM(IS_SMARTCARD_BAUDRATE(hsc->Init.BaudRate));
  ASSERT_PARAM(IS_SMARTCARD_WORD_LENGTH(hsc->Init.WordLength));
  ASSERT_PARAM(IS_SMARTCARD_STOPBITS(hsc->Init.StopBits));
  ASSERT_PARAM(IS_SMARTCARD_PARITY(hsc->Init.Parity));
  ASSERT_PARAM(IS_SMARTCARD_MODE(hsc->Init.Mode));
  ASSERT_PARAM(IS_SMARTCARD_NACK_STATE(hsc->Init.NACKState));

  /* The LBCL, CPOL and CPHA bits have to be selected when both the transmitter and the
     receiver are disabled (TE=RE=0) to ensure that the clock pulses function correctly. */
  CLEAR_BIT(hsc->Instance->CTRL1, (USART_CTRL1_TXEN | USART_CTRL1_RXEN));

  /*---------------------------- USART CTRL2 Configuration ---------------------*/
  tmpreg = hsc->Instance->CTRL2;
  /* Clear CLKEN, CPOL, CPHA and LBCPOEN bits */
  tmpreg &= (uint32_t)~((uint32_t)(USART_CTRL2_CPHA | USART_CTRL2_CPOL | USART_CTRL2_CLKEN | USART_CTRL2_LBCPOEN));
  /* Configure the SMARTCARD Clock, CPOL, CPHA and LastBit -----------------------*/
  /* Set CPOL bit according to hsc->Init.CLKPolarity value */
  /* Set CPHA bit according to hsc->Init.CLKPhase value */
  /* Set LBCL bit according to hsc->Init.CLKLastBit value */
  /* Set Stop Bits: Set STOP[13:12] bits according to hsc->Init.StopBits value */
  tmpreg |= (uint32_t)(USART_CTRL2_CLKEN | hsc->Init.CLKPolarity |
                      hsc->Init.CLKPhase| hsc->Init.CLKLastBit | hsc->Init.StopBits);
  /* Write to USART CTRL2 */
  WRITE_REG(hsc->Instance->CTRL2, (uint32_t)tmpreg);

  tmpreg = hsc->Instance->CTRL2;

  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)USART_CTRL2_STOPCFG);

  /* Set Stop Bits: Set STOP[13:12] bits according to hsc->Init.StopBits value */
  tmpreg |= (uint32_t)(hsc->Init.StopBits);

  /* Write to USART CTRL2 */
  WRITE_REG(hsc->Instance->CTRL2, (uint32_t)tmpreg);

  /*-------------------------- USART CTRL1 Configuration -----------------------*/
  tmpreg = hsc->Instance->CTRL1;

  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)(USART_CTRL1_DBLCFG | USART_CTRL1_PCEN | USART_CTRL1_PCFG | USART_CTRL1_TXEN | \
                                   USART_CTRL1_RXEN));

  /* Configure the SMARTCARD Word Length, Parity and mode:
     Set the M bits according to hsc->Init.WordLength value
     Set PCE and PS bits according to hsc->Init.Parity value
     Set TE and RE bits according to hsc->Init.Mode value */
  tmpreg |= (uint32_t)hsc->Init.WordLength | hsc->Init.Parity | hsc->Init.Mode;

  /* Write to USART CTRL1 */
  WRITE_REG(hsc->Instance->CTRL1, (uint32_t)tmpreg);

  /*-------------------------- USART CTRL3 Configuration -----------------------*/
  /* Clear CTSEN and RTSEN bits */
  CLEAR_BIT(hsc->Instance->CTRL3, (USART_CTRL3_RTSEN | USART_CTRL3_CTSEN));

  /*-------------------------- USART BR Configuration -----------------------*/
#if defined(USART6)
  if((hsc->Instance == USART1) || (hsc->Instance == USART6))
  {
    pclk = DAL_RCM_GetPCLK2Freq();
    hsc->Instance->BR = SMARTCARD_BR(pclk, hsc->Init.BaudRate);
  }
#else
  if(hsc->Instance == USART1)
  {
    pclk = DAL_RCM_GetPCLK2Freq();
    hsc->Instance->BR = SMARTCARD_BR(pclk, hsc->Init.BaudRate);
  }
#endif /* USART6 */
  else
  {
    pclk = DAL_RCM_GetPCLK1Freq();
    hsc->Instance->BR = SMARTCARD_BR(pclk, hsc->Init.BaudRate);
  }
}

/**
  * @}
  */

#endif /* DAL_SMARTCARD_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */


