/**
  *
  * @file    apm32f4xx_dal_usart.c
  * @brief   USART DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Universal Synchronous/Asynchronous Receiver Transmitter
  *          Peripheral (USART).
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
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
    The USART DAL driver can be used as follows:

    (#) Declare a USART_HandleTypeDef handle structure (eg. USART_HandleTypeDef husart).
    (#) Initialize the USART low level resources by implementing the DAL_USART_MspInit() API:
        (##) Enable the USARTx interface clock.
        (##) USART pins configuration:
             (+++) Enable the clock for the USART GPIOs.
             (+++) Configure the USART pins as alternate function pull-up.
        (##) NVIC configuration if you need to use interrupt process (DAL_USART_Transmit_IT(),
             DAL_USART_Receive_IT() and DAL_USART_TransmitReceive_IT() APIs):
             (+++) Configure the USARTx interrupt priority.
             (+++) Enable the NVIC USART IRQ handle.
        (##) DMA Configuration if you need to use DMA process (DAL_USART_Transmit_DMA()
             DAL_USART_Receive_DMA() and DAL_USART_TransmitReceive_DMA() APIs):
             (+++) Declare a DMA handle structure for the Tx/Rx stream.
             (+++) Enable the DMAx interface clock.
             (+++) Configure the declared DMA handle structure with the required Tx/Rx parameters.
             (+++) Configure the DMA Tx/Rx stream.
             (+++) Associate the initialized DMA handle to the USART DMA Tx/Rx handle.
             (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA Tx/Rx stream.
             (+++) Configure the USARTx interrupt priority and enable the NVIC USART IRQ handle
                   (used for last byte sending completion detection in DMA non circular mode)

    (#) Program the Baud Rate, Word Length, Stop Bit, Parity, Hardware
        flow control and Mode(Receiver/Transmitter) in the husart Init structure.

    (#) Initialize the USART registers by calling the DAL_USART_Init() API:
        (++) These APIs configures also the low level Hardware GPIO, CLOCK, CORTEX...etc)
             by calling the customized DAL_USART_MspInit(&husart) API.

        -@@- The specific USART interrupts (Transmission complete interrupt,
             RXNE interrupt and Error Interrupts) will be managed using the macros
             __DAL_USART_ENABLE_IT() and __DAL_USART_DISABLE_IT() inside the transmit and receive process.

    (#) Three operation modes are available within this driver :

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Send an amount of data in blocking mode using DAL_USART_Transmit()
       (+) Receive an amount of data in blocking mode using DAL_USART_Receive()

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Send an amount of data in non blocking mode using DAL_USART_Transmit_IT()
       (+) At transmission end of transfer DAL_USART_TxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_USART_TxCpltCallback
       (+) Receive an amount of data in non blocking mode using DAL_USART_Receive_IT()
       (+) At reception end of transfer DAL_USART_RxCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_USART_RxCpltCallback
       (+) In case of transfer Error, DAL_USART_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer DAL_USART_ErrorCallback

     *** DMA mode IO operation ***
     ==============================
     [..]
       (+) Send an amount of data in non blocking mode (DMA) using DAL_USART_Transmit_DMA()
       (+) At transmission end of half transfer DAL_USART_TxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_USART_TxHalfCpltCallback
       (+) At transmission end of transfer DAL_USART_TxCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_USART_TxCpltCallback
       (+) Receive an amount of data in non blocking mode (DMA) using DAL_USART_Receive_DMA()
       (+) At reception end of half transfer DAL_USART_RxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_USART_RxHalfCpltCallback
       (+) At reception end of transfer DAL_USART_RxCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_USART_RxCpltCallback
       (+) In case of transfer Error, DAL_USART_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer DAL_USART_ErrorCallback
       (+) Pause the DMA Transfer using DAL_USART_DMAPause()
       (+) Resume the DMA Transfer using DAL_USART_DMAResume()
       (+) Stop the DMA Transfer using DAL_USART_DMAStop()

     *** USART DAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in USART DAL driver.

       (+) __DAL_USART_ENABLE: Enable the USART peripheral
       (+) __DAL_USART_DISABLE: Disable the USART peripheral
       (+) __DAL_USART_GET_FLAG : Check whether the specified USART flag is set or not
       (+) __DAL_USART_CLEAR_FLAG : Clear the specified USART pending flag
       (+) __DAL_USART_ENABLE_IT: Enable the specified USART interrupt
       (+) __DAL_USART_DISABLE_IT: Disable the specified USART interrupt

     [..]
       (@) You can refer to the USART DAL driver header file for more useful macros

    ##### Callback registration #####
    ==================================

    [..]
    The compilation define USE_DAL_USART_REGISTER_CALLBACKS when set to 1
    allows the user to configure dynamically the driver callbacks.

    [..]
    Use Function @ref DAL_USART_RegisterCallback() to register a user callback.
    Function @ref DAL_USART_RegisterCallback() allows to register following callbacks:
    (+) TxHalfCpltCallback        : Tx Half Complete Callback.
    (+) TxCpltCallback            : Tx Complete Callback.
    (+) RxHalfCpltCallback        : Rx Half Complete Callback.
    (+) RxCpltCallback            : Rx Complete Callback.
    (+) TxRxCpltCallback          : Tx Rx Complete Callback.
    (+) ErrorCallback             : Error Callback.
    (+) AbortCpltCallback         : Abort Complete Callback.
    (+) MspInitCallback           : USART MspInit.
    (+) MspDeInitCallback         : USART MspDeInit.
    This function takes as parameters the DAL peripheral handle, the Callback ID
    and a pointer to the user callback function.

    [..]
    Use function @ref DAL_USART_UnRegisterCallback() to reset a callback to the default
    weak (surcharged) function.
    @ref DAL_USART_UnRegisterCallback() takes as parameters the DAL peripheral handle,
    and the Callback ID.
    This function allows to reset following callbacks:
    (+) TxHalfCpltCallback        : Tx Half Complete Callback.
    (+) TxCpltCallback            : Tx Complete Callback.
    (+) RxHalfCpltCallback        : Rx Half Complete Callback.
    (+) RxCpltCallback            : Rx Complete Callback.
    (+) TxRxCpltCallback          : Tx Rx Complete Callback.
    (+) ErrorCallback             : Error Callback.
    (+) AbortCpltCallback         : Abort Complete Callback.
    (+) MspInitCallback           : USART MspInit.
    (+) MspDeInitCallback         : USART MspDeInit.

    [..]
    By default, after the @ref DAL_USART_Init() and when the state is DAL_USART_STATE_RESET
    all callbacks are set to the corresponding weak (surcharged) functions:
    examples @ref DAL_USART_TxCpltCallback(), @ref DAL_USART_RxHalfCpltCallback().
    Exception done for MspInit and MspDeInit functions that are respectively
    reset to the legacy weak (surcharged) functions in the @ref DAL_USART_Init()
    and @ref DAL_USART_DeInit() only when these callbacks are null (not registered beforehand).
    If not, MspInit or MspDeInit are not null, the @ref DAL_USART_Init() and @ref DAL_USART_DeInit()
    keep and use the user MspInit/MspDeInit callbacks (registered beforehand).

    [..]
    Callbacks can be registered/unregistered in DAL_USART_STATE_READY state only.
    Exception done MspInit/MspDeInit that can be registered/unregistered
    in DAL_USART_STATE_READY or DAL_USART_STATE_RESET state, thus registered (user)
    MspInit/DeInit callbacks can be used during the Init/DeInit.
    In that case first register the MspInit/MspDeInit user callbacks
    using @ref DAL_USART_RegisterCallback() before calling @ref DAL_USART_DeInit()
    or @ref DAL_USART_Init() function.

    [..]
    When The compilation define USE_DAL_USART_REGISTER_CALLBACKS is set to 0 or
    not defined, the callback registration feature is not available
    and weak (surcharged) callbacks are used.

  @endverbatim
     [..]
       (@) Additional remark: If the parity is enabled, then the MSB bit of the data written
           in the data register is transmitted but is changed by the parity bit.
           Depending on the frame length defined by the M bit (8-bits or 9-bits),
           the possible USART frame formats are as listed in the following table:
    +-------------------------------------------------------------+
    |   M bit |  PCE bit  |            USART frame                 |
    |---------------------|---------------------------------------|
    |    0    |    0      |    | SB | 8 bit data | STB |          |
    |---------|-----------|---------------------------------------|
    |    0    |    1      |    | SB | 7 bit data | PB | STB |     |
    |---------|-----------|---------------------------------------|
    |    1    |    0      |    | SB | 9 bit data | STB |          |
    |---------|-----------|---------------------------------------|
    |    1    |    1      |    | SB | 8 bit data | PB | STB |     |
    +-------------------------------------------------------------+
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup USART USART
  * @brief DAL USART Synchronous module driver
  * @{
  */
#ifdef DAL_USART_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup USART_Private_Constants
  * @{
  */
#define DUMMY_DATA           0xFFFFU
#define USART_TIMEOUT_VALUE  22000U
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup USART_Private_Functions
  * @{
  */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
void USART_InitCallbacksToDefault(USART_HandleTypeDef *husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
static void USART_EndTxTransfer(USART_HandleTypeDef *husart);
static void USART_EndRxTransfer(USART_HandleTypeDef *husart);
static DAL_StatusTypeDef USART_Transmit_IT(USART_HandleTypeDef *husart);
static DAL_StatusTypeDef USART_EndTransmit_IT(USART_HandleTypeDef *husart);
static DAL_StatusTypeDef USART_Receive_IT(USART_HandleTypeDef *husart);
static DAL_StatusTypeDef USART_TransmitReceive_IT(USART_HandleTypeDef *husart);
static void USART_SetConfig(USART_HandleTypeDef *husart);
static void USART_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void USART_DMATxHalfCplt(DMA_HandleTypeDef *hdma);
static void USART_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void USART_DMARxHalfCplt(DMA_HandleTypeDef *hdma);
static void USART_DMAError(DMA_HandleTypeDef *hdma);
static void USART_DMAAbortOnError(DMA_HandleTypeDef *hdma);
static void USART_DMATxAbortCallback(DMA_HandleTypeDef *hdma);
static void USART_DMARxAbortCallback(DMA_HandleTypeDef *hdma);

static DAL_StatusTypeDef USART_WaitOnFlagUntilTimeout(USART_HandleTypeDef *husart, uint32_t Flag, FlagStatus Status,
                                                      uint32_t Tickstart, uint32_t Timeout);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup USART_Exported_Functions USART Exported Functions
  * @{
  */

/** @defgroup USART_Exported_Functions_Group1 USART Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
  ==============================================================================
              ##### Initialization and Configuration functions #####
  ==============================================================================
  [..]
  This subsection provides a set of functions allowing to initialize the USART
  in asynchronous and in synchronous modes.
  (+) For the asynchronous mode only these parameters can be configured:
      (++) Baud Rate
      (++) Word Length
      (++) Stop Bit
      (++) Parity: If the parity is enabled, then the MSB bit of the data written
           in the data register is transmitted but is changed by the parity bit.
           Depending on the frame length defined by the M bit (8-bits or 9-bits),
           please refer to Reference manual for possible USART frame formats.
      (++) USART polarity
      (++) USART phase
      (++) USART LastBit
      (++) Receiver/transmitter modes

  [..]
    The DAL_USART_Init() function follows the USART  synchronous configuration
    procedures (details for the procedures are available in reference manual).

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the USART mode according to the specified
  *         parameters in the USART_InitTypeDef and initialize the associated handle.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_Init(USART_HandleTypeDef *husart)
{
  /* Check the USART handle allocation */
  if (husart == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_USART_INSTANCE(husart->Instance));

  if (husart->State == DAL_USART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    husart->Lock = DAL_UNLOCKED;

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
    USART_InitCallbacksToDefault(husart);

    if (husart->MspInitCallback == NULL)
    {
      husart->MspInitCallback = DAL_USART_MspInit;
    }

    /* Init the low level hardware */
    husart->MspInitCallback(husart);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    DAL_USART_MspInit(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
  }

  husart->State = DAL_USART_STATE_BUSY;

  /* Set the USART Communication parameters */
  USART_SetConfig(husart);

  /* In USART mode, the following bits must be kept cleared:
     - LINEN bit in the USART_CTRL2 register
     - HDSEL, SCEN and IREN bits in the USART_CTRL3 register */
  CLEAR_BIT(husart->Instance->CTRL2, USART_CTRL2_LINMEN);
  CLEAR_BIT(husart->Instance->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_HDEN | USART_CTRL3_IREN));

  /* Enable the Peripheral */
  __DAL_USART_ENABLE(husart);

  /* Initialize the USART state */
  husart->ErrorCode = DAL_USART_ERROR_NONE;
  husart->State = DAL_USART_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the USART peripheral.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_DeInit(USART_HandleTypeDef *husart)
{
  /* Check the USART handle allocation */
  if (husart == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_USART_INSTANCE(husart->Instance));

  husart->State = DAL_USART_STATE_BUSY;

  /* Disable the Peripheral */
  __DAL_USART_DISABLE(husart);

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  if (husart->MspDeInitCallback == NULL)
  {
    husart->MspDeInitCallback = DAL_USART_MspDeInit;
  }
  /* DeInit the low level hardware */
  husart->MspDeInitCallback(husart);
#else
  /* DeInit the low level hardware */
  DAL_USART_MspDeInit(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

  husart->ErrorCode = DAL_USART_ERROR_NONE;
  husart->State = DAL_USART_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(husart);

  return DAL_OK;
}

/**
  * @brief  USART MSP Init.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_USART_MspInit(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_USART_MspInit could be implemented in the user file
   */
}

/**
  * @brief  USART MSP DeInit.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_USART_MspDeInit(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_USART_MspDeInit could be implemented in the user file
   */
}

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User USART Callback
  *         To be used instead of the weak predefined callback
  * @param  husart usart handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_USART_TX_HALFCOMPLETE_CB_ID Tx Half Complete Callback ID
  *           @arg @ref DAL_USART_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref DAL_USART_RX_HALFCOMPLETE_CB_ID Rx Half Complete Callback ID
  *           @arg @ref DAL_USART_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_USART_TX_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_USART_ERROR_CB_ID Error Callback ID
  *           @arg @ref DAL_USART_ABORT_COMPLETE_CB_ID Abort Complete Callback ID
  *           @arg @ref DAL_USART_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref DAL_USART_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
+  */
DAL_StatusTypeDef DAL_USART_RegisterCallback(USART_HandleTypeDef *husart, DAL_USART_CallbackIDTypeDef CallbackID,
                                             pUSART_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    husart->ErrorCode |= DAL_USART_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(husart);

  if (husart->State == DAL_USART_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_USART_TX_HALFCOMPLETE_CB_ID :
        husart->TxHalfCpltCallback = pCallback;
        break;

      case DAL_USART_TX_COMPLETE_CB_ID :
        husart->TxCpltCallback = pCallback;
        break;

      case DAL_USART_RX_HALFCOMPLETE_CB_ID :
        husart->RxHalfCpltCallback = pCallback;
        break;

      case DAL_USART_RX_COMPLETE_CB_ID :
        husart->RxCpltCallback = pCallback;
        break;

      case DAL_USART_TX_RX_COMPLETE_CB_ID :
        husart->TxRxCpltCallback = pCallback;
        break;

      case DAL_USART_ERROR_CB_ID :
        husart->ErrorCallback = pCallback;
        break;

      case DAL_USART_ABORT_COMPLETE_CB_ID :
        husart->AbortCpltCallback = pCallback;
        break;

      case DAL_USART_MSPINIT_CB_ID :
        husart->MspInitCallback = pCallback;
        break;

      case DAL_USART_MSPDEINIT_CB_ID :
        husart->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        husart->ErrorCode |= DAL_USART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (husart->State == DAL_USART_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_USART_MSPINIT_CB_ID :
        husart->MspInitCallback = pCallback;
        break;

      case DAL_USART_MSPDEINIT_CB_ID :
        husart->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        husart->ErrorCode |= DAL_USART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    husart->ErrorCode |= DAL_USART_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(husart);

  return status;
}

/**
  * @brief  Unregister an USART Callback
  *         USART callaback is redirected to the weak predefined callback
  * @param  husart usart handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_USART_TX_HALFCOMPLETE_CB_ID Tx Half Complete Callback ID
  *           @arg @ref DAL_USART_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref DAL_USART_RX_HALFCOMPLETE_CB_ID Rx Half Complete Callback ID
  *           @arg @ref DAL_USART_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_USART_TX_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_USART_ERROR_CB_ID Error Callback ID
  *           @arg @ref DAL_USART_ABORT_COMPLETE_CB_ID Abort Complete Callback ID
  *           @arg @ref DAL_USART_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref DAL_USART_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_UnRegisterCallback(USART_HandleTypeDef *husart, DAL_USART_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(husart);

  if (husart->State == DAL_USART_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_USART_TX_HALFCOMPLETE_CB_ID :
        husart->TxHalfCpltCallback = DAL_USART_TxHalfCpltCallback;               /* Legacy weak  TxHalfCpltCallback       */
        break;

      case DAL_USART_TX_COMPLETE_CB_ID :
        husart->TxCpltCallback = DAL_USART_TxCpltCallback;                       /* Legacy weak TxCpltCallback            */
        break;

      case DAL_USART_RX_HALFCOMPLETE_CB_ID :
        husart->RxHalfCpltCallback = DAL_USART_RxHalfCpltCallback;               /* Legacy weak RxHalfCpltCallback        */
        break;

      case DAL_USART_RX_COMPLETE_CB_ID :
        husart->RxCpltCallback = DAL_USART_RxCpltCallback;                       /* Legacy weak RxCpltCallback            */
        break;

      case DAL_USART_TX_RX_COMPLETE_CB_ID :
        husart->TxRxCpltCallback = DAL_USART_TxRxCpltCallback;                   /* Legacy weak TxRxCpltCallback            */
        break;

      case DAL_USART_ERROR_CB_ID :
        husart->ErrorCallback = DAL_USART_ErrorCallback;                         /* Legacy weak ErrorCallback             */
        break;

      case DAL_USART_ABORT_COMPLETE_CB_ID :
        husart->AbortCpltCallback = DAL_USART_AbortCpltCallback;                 /* Legacy weak AbortCpltCallback         */
        break;

      case DAL_USART_MSPINIT_CB_ID :
        husart->MspInitCallback = DAL_USART_MspInit;                             /* Legacy weak MspInitCallback           */
        break;

      case DAL_USART_MSPDEINIT_CB_ID :
        husart->MspDeInitCallback = DAL_USART_MspDeInit;                         /* Legacy weak MspDeInitCallback         */
        break;

      default :
        /* Update the error code */
        husart->ErrorCode |= DAL_USART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (husart->State == DAL_USART_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_USART_MSPINIT_CB_ID :
        husart->MspInitCallback = DAL_USART_MspInit;
        break;

      case DAL_USART_MSPDEINIT_CB_ID :
        husart->MspDeInitCallback = DAL_USART_MspDeInit;
        break;

      default :
        /* Update the error code */
        husart->ErrorCode |= DAL_USART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    husart->ErrorCode |= DAL_USART_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(husart);

  return status;
}
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup USART_Exported_Functions_Group2 IO operation functions
  *  @brief   USART Transmit and Receive functions
  *
@verbatim
  ==============================================================================
                         ##### IO operation functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to manage the USART synchronous
    data transfers.

  [..]
    The USART supports master mode only: it cannot receive or send data related to an input
    clock (SCLK is always an output).

    (#) There are two modes of transfer:
        (++) Blocking mode: The communication is performed in polling mode.
             The DAL status of all data processing is returned by the same function
             after finishing transfer.
        (++) No-Blocking mode: The communication is performed using Interrupts
             or DMA, These API's return the DAL status.
             The end of the data processing will be indicated through the
             dedicated USART IRQ when using Interrupt mode or the DMA IRQ when
             using DMA mode.
             The DAL_USART_TxCpltCallback(), DAL_USART_RxCpltCallback() and DAL_USART_TxRxCpltCallback()
              user callbacks
             will be executed respectively at the end of the transmit or Receive process
             The DAL_USART_ErrorCallback() user callback will be executed when a communication
             error is detected

    (#) Blocking mode APIs are :
        (++) DAL_USART_Transmit() in simplex mode
        (++) DAL_USART_Receive() in full duplex receive only
        (++) DAL_USART_TransmitReceive() in full duplex mode

    (#) Non Blocking mode APIs with Interrupt are :
        (++) DAL_USART_Transmit_IT()in simplex mode
        (++) DAL_USART_Receive_IT() in full duplex receive only
        (++) DAL_USART_TransmitReceive_IT() in full duplex mode
        (++) DAL_USART_IRQHandler()

    (#) Non Blocking mode functions with DMA are :
        (++) DAL_USART_Transmit_DMA()in simplex mode
        (++) DAL_USART_Receive_DMA() in full duplex receive only
        (++) DAL_USART_TransmitReceive_DMA() in full duplex mode
        (++) DAL_USART_DMAPause()
        (++) DAL_USART_DMAResume()
        (++) DAL_USART_DMAStop()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) DAL_USART_TxHalfCpltCallback()
        (++) DAL_USART_TxCpltCallback()
        (++) DAL_USART_RxHalfCpltCallback()
        (++) DAL_USART_RxCpltCallback()
        (++) DAL_USART_ErrorCallback()
        (++) DAL_USART_TxRxCpltCallback()

    (#) Non-Blocking mode transfers could be aborted using Abort API's :
        (++) DAL_USART_Abort()
        (++) DAL_USART_Abort_IT()

    (#) For Abort services based on interrupts (DAL_USART_Abort_IT), a Abort Complete Callbacks is provided:
        (++) DAL_USART_AbortCpltCallback()

    (#) In Non-Blocking mode transfers, possible errors are split into 2 categories.
        Errors are handled as follows :
        (++) Error is considered as Recoverable and non blocking : Transfer could go till end, but error severity is
             to be evaluated by user : this concerns Frame Error, Parity Error or Noise Error in Interrupt mode reception .
             Received character is then retrieved and stored in Rx buffer, Error code is set to allow user to identify error type,
             and DAL_USART_ErrorCallback() user callback is executed. Transfer is kept ongoing on USART side.
             If user wants to abort it, Abort services should be called by user.
        (++) Error is considered as Blocking : Transfer could not be completed properly and is aborted.
             This concerns Overrun Error In Interrupt mode reception and all errors in DMA mode.
             Error code is set to allow user to identify error type, and DAL_USART_ErrorCallback() user callback is executed.

@endverbatim
  * @{
  */

/**
  * @brief  Simplex Send an amount of data in blocking mode.
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pTxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pTxData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be sent.
  * @param  Timeout Timeout duration.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_Transmit(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size, uint32_t Timeout)
{
  const uint8_t  *ptxdata8bits;
  const uint16_t *ptxdata16bits;
  uint32_t tickstart;

  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pTxData == NULL) || (Size == 0))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(husart);

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_TX;

    /* Init tickstart for timeout management */
    tickstart = DAL_GetTick();

    husart->TxXferSize = Size;
    husart->TxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pTxData needs to be handled as a uint16_t pointer */
    if ((husart->Init.WordLength == USART_WORDLENGTH_9B) && (husart->Init.Parity == USART_PARITY_NONE))
    {
      ptxdata8bits  = NULL;
      ptxdata16bits = (const uint16_t *) pTxData;
    }
    else
    {
      ptxdata8bits  = pTxData;
      ptxdata16bits = NULL;
    }

    while (husart->TxXferCount > 0U)
    {
      /* Wait for TXE flag in order to write data in DATA */
      if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXE, RESET, tickstart, Timeout) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }
      if (ptxdata8bits == NULL)
      {
        husart->Instance->DATA = (uint16_t)(*ptxdata16bits & (uint16_t)0x01FF);
        ptxdata16bits++;
      }
      else
      {
        husart->Instance->DATA = (uint8_t)(*ptxdata8bits & (uint8_t)0xFF);
        ptxdata8bits++;
      }

      husart->TxXferCount--;
    }

    if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TC, RESET, tickstart, Timeout) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    husart->State = DAL_USART_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Full-Duplex Receive an amount of data in blocking mode.
  * @note   To receive synchronous data, dummy data are simultaneously transmitted.
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pRxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pRxData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be received.
  * @param  Timeout Timeout duration.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
  uint8_t  *prxdata8bits;
  uint16_t *prxdata16bits;
  uint32_t tickstart;

  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pRxData == NULL) || (Size == 0))
    {
      return  DAL_ERROR;
    }
    /* Process Locked */
    __DAL_LOCK(husart);

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_RX;

    /* Init tickstart for timeout management */
    tickstart = DAL_GetTick();

    husart->RxXferSize = Size;
    husart->RxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pRxData needs to be handled as a uint16_t pointer */
    if ((husart->Init.WordLength == USART_WORDLENGTH_9B) && (husart->Init.Parity == USART_PARITY_NONE))
    {
      prxdata8bits  = NULL;
      prxdata16bits = (uint16_t *) pRxData;
    }
    else
    {
      prxdata8bits  = pRxData;
      prxdata16bits = NULL;
    }

    /* Check the remain data to be received */
    while (husart->RxXferCount > 0U)
    {
      /* Wait until TXE flag is set to send dummy byte in order to generate the
      * clock for the slave to send data.
      * Whatever the frame length (7, 8 or 9-bit long), the same dummy value
      * can be written for all the cases. */
      if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXE, RESET, tickstart, Timeout) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }
      husart->Instance->DATA = (DUMMY_DATA & (uint16_t)0x0FF);

      /* Wait until RXNE flag is set to receive the byte */
      if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_RXNE, RESET, tickstart, Timeout) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }

      if (prxdata8bits == NULL)
      {
        *prxdata16bits = (uint16_t)(husart->Instance->DATA & (uint16_t)0x01FF);
        prxdata16bits++;
      }
      else
      {
        if ((husart->Init.WordLength == USART_WORDLENGTH_9B) || ((husart->Init.WordLength == USART_WORDLENGTH_8B) && (husart->Init.Parity == USART_PARITY_NONE)))
        {
          *prxdata8bits = (uint8_t)(husart->Instance->DATA & (uint8_t)0x0FF);
        }
        else
        {
          *prxdata8bits = (uint8_t)(husart->Instance->DATA & (uint8_t)0x07F);
        }
        prxdata8bits++;
      }
      husart->RxXferCount--;
    }

    husart->State = DAL_USART_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Full-Duplex Send and Receive an amount of data in full-duplex mode (blocking mode).
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data and the received data are handled as sets of u16. In this case, Size must indicate the number
  *         of u16 available through pTxData and through pRxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pTxData Pointer to TX data buffer (u8 or u16 data elements).
  * @param  pRxData Pointer to RX data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be sent (same amount to be received).
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_TransmitReceive(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                            uint16_t Size, uint32_t Timeout)
{
  uint8_t  *prxdata8bits;
  uint16_t *prxdata16bits;
  const uint8_t  *ptxdata8bits;
  const uint16_t *ptxdata16bits;
  uint16_t rxdatacount;
  uint32_t tickstart;

  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0))
    {
      return  DAL_ERROR;
    }

    /* In case of 9bits/No Parity transfer, pTxData and pRxData buffers provided as input parameter
       should be aligned on a u16 frontier, as data to be filled into TDATA/retrieved from RDATA will be
       handled through a u16 cast. */
    if ((husart->Init.WordLength == USART_WORDLENGTH_9B) && (husart->Init.Parity == USART_PARITY_NONE))
    {
      if (((((uint32_t)pTxData) & 1U) != 0U) || ((((uint32_t)pRxData) & 1U) != 0U))
      {
        return  DAL_ERROR;
      }
    }
    /* Process Locked */
    __DAL_LOCK(husart);

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_RX;

    /* Init tickstart for timeout management */
    tickstart = DAL_GetTick();

    husart->RxXferSize = Size;
    husart->TxXferSize = Size;
    husart->TxXferCount = Size;
    husart->RxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pRxData needs to be handled as a uint16_t pointer */
    if ((husart->Init.WordLength == USART_WORDLENGTH_9B) && (husart->Init.Parity == USART_PARITY_NONE))
    {
      prxdata8bits  = NULL;
      ptxdata8bits  = NULL;
      ptxdata16bits = (const uint16_t *) pTxData;
      prxdata16bits = (uint16_t *) pRxData;
    }
    else
    {
      prxdata8bits  = pRxData;
      ptxdata8bits  = pTxData;
      ptxdata16bits = NULL;
      prxdata16bits = NULL;
    }

    /* Check the remain data to be received */
    /* rxdatacount is a temporary variable for MISRAC2012-Rule-13.5 */
    rxdatacount = husart->RxXferCount;
    while ((husart->TxXferCount > 0U) || (rxdatacount > 0U))
    {
      if (husart->TxXferCount > 0U)
      {
        /* Wait for TXE flag in order to write data in DATA */
        if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXE, RESET, tickstart, Timeout) != DAL_OK)
        {
          return DAL_TIMEOUT;
        }

        if (ptxdata8bits == NULL)
        {
          husart->Instance->DATA = (uint16_t)(*ptxdata16bits & (uint16_t)0x01FF);
          ptxdata16bits++;
        }
        else
        {
          husart->Instance->DATA = (uint8_t)(*ptxdata8bits & (uint8_t)0xFF);
          ptxdata8bits++;
        }

        husart->TxXferCount--;
      }

      if (husart->RxXferCount > 0U)
      {
        /* Wait for RXNE Flag */
        if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_RXNE, RESET, tickstart, Timeout) != DAL_OK)
        {
          return DAL_TIMEOUT;
        }
        if (prxdata8bits == NULL)
        {
          *prxdata16bits = (uint16_t)(husart->Instance->DATA & (uint16_t)0x01FF);
          prxdata16bits++;
        }
        else
        {
          if ((husart->Init.WordLength == USART_WORDLENGTH_9B) || ((husart->Init.WordLength == USART_WORDLENGTH_8B) && (husart->Init.Parity == USART_PARITY_NONE)))
          {
            *prxdata8bits = (uint8_t)(husart->Instance->DATA & (uint8_t)0x0FF);
          }
          else
          {
            *prxdata8bits = (uint8_t)(husart->Instance->DATA & (uint8_t)0x07F);
          }

          prxdata8bits++;
        }

        husart->RxXferCount--;
      }
      rxdatacount = husart->RxXferCount;
    }

    husart->State = DAL_USART_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Simplex Send an amount of data in non-blocking mode.
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pTxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pTxData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be sent.
  * @retval DAL status
  * @note   The USART errors are not managed to avoid the overrun error.
  */
DAL_StatusTypeDef DAL_USART_Transmit_IT(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size)
{
  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pTxData == NULL) || (Size == 0))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(husart);

    husart->pTxBuffPtr = pTxData;
    husart->TxXferSize = Size;
    husart->TxXferCount = Size;

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_TX;

    /* The USART Error Interrupts: (Frame error, Noise error, Overrun error)
       are not managed by the USART transmit process to avoid the overrun interrupt
       when the USART mode is configured for transmit and receive "USART_MODE_TX_RX"
       to benefit for the frame error and noise interrupts the USART mode should be
       configured only for transmit "USART_MODE_TX"
       The __DAL_USART_ENABLE_IT(husart, USART_IT_ERR) can be used to enable the Frame error,
       Noise error interrupt */

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    /* Enable the USART Transmit Data Register Empty Interrupt */
    SET_BIT(husart->Instance->CTRL1, USART_CTRL1_TXBEIEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Simplex Receive an amount of data in non-blocking mode.
  * @note   To receive synchronous data, dummy data are simultaneously transmitted.
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pRxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pRxData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be received.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size)
{
  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pRxData == NULL) || (Size == 0))
    {
      return DAL_ERROR;
    }
    /* Process Locked */
    __DAL_LOCK(husart);

    husart->pRxBuffPtr = pRxData;
    husart->RxXferSize = Size;
    husart->RxXferCount = Size;

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_RX;

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    if (husart->Init.Parity != USART_PARITY_NONE)
    {
      /* Enable the USART Parity Error and Data Register not empty Interrupts */
      SET_BIT(husart->Instance->CTRL1, USART_CTRL1_PEIEN | USART_CTRL1_RXBNEIEN);
    }
    else
    {
      /* Enable the USART Data Register not empty Interrupts */
      SET_BIT(husart->Instance->CTRL1, USART_CTRL1_RXBNEIEN);
    }

    /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Send dummy byte in order to generate the clock for the slave to send data */
    husart->Instance->DATA = (DUMMY_DATA & (uint16_t)0x01FF);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Full-Duplex Send and Receive an amount of data in full-duplex mode (non-blocking).
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data and the received data are handled as sets of u16. In this case, Size must indicate the number
  *         of u16 available through pTxData and through pRxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pTxData Pointer to TX data buffer (u8 or u16 data elements).
  * @param  pRxData Pointer to RX data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be sent (same amount to be received).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_TransmitReceive_IT(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                               uint16_t Size)
{
  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0))
    {
      return DAL_ERROR;
    }
    /* Process Locked */
    __DAL_LOCK(husart);

    husart->pRxBuffPtr = pRxData;
    husart->RxXferSize = Size;
    husart->RxXferCount = Size;
    husart->pTxBuffPtr = pTxData;
    husart->TxXferSize = Size;
    husart->TxXferCount = Size;

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_TX_RX;

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    /* Enable the USART Data Register not empty Interrupt */
    SET_BIT(husart->Instance->CTRL1, USART_CTRL1_RXBNEIEN);

    if (husart->Init.Parity != USART_PARITY_NONE)
    {
      /* Enable the USART Parity Error Interrupt */
      SET_BIT(husart->Instance->CTRL1, USART_CTRL1_PEIEN);
    }

    /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Enable the USART Transmit Data Register Empty Interrupt */
    SET_BIT(husart->Instance->CTRL1, USART_CTRL1_TXBEIEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Simplex Send an amount of data in DMA mode.
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pTxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pTxData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be sent.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size)
{
  const uint32_t *tmp;

  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pTxData == NULL) || (Size == 0))
    {
      return DAL_ERROR;
    }
    /* Process Locked */
    __DAL_LOCK(husart);

    husart->pTxBuffPtr = pTxData;
    husart->TxXferSize = Size;
    husart->TxXferCount = Size;

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_TX;

    /* Set the USART DMA transfer complete callback */
    husart->hdmatx->XferCpltCallback = USART_DMATransmitCplt;

    /* Set the USART DMA Half transfer complete callback */
    husart->hdmatx->XferHalfCpltCallback = USART_DMATxHalfCplt;

    /* Set the DMA error callback */
    husart->hdmatx->XferErrorCallback = USART_DMAError;

    /* Set the DMA abort callback */
    husart->hdmatx->XferAbortCallback = NULL;

    /* Enable the USART transmit DMA stream */
    tmp = (const uint32_t *)&pTxData;
    DAL_DMA_Start_IT(husart->hdmatx, *(const uint32_t *)tmp, (uint32_t)&husart->Instance->DATA, Size);

    /* Clear the TC flag in the STS register by writing 0 to it */
    __DAL_USART_CLEAR_FLAG(husart, USART_FLAG_TC);

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    /* Enable the DMA transfer for transmit request by setting the DMAT bit
    in the USART CTRL3 register */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Full-Duplex Receive an amount of data in DMA mode.
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pRxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pRxData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be received.
  * @retval DAL status
  * @note   The USART DMA transmit stream must be configured in order to generate the clock for the slave.
  * @note   When the USART parity is enabled (PCE = 1) the data received contain the parity bit.
  */
DAL_StatusTypeDef DAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size)
{
  uint32_t *tmp;

  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pRxData == NULL) || (Size == 0))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(husart);

    husart->pRxBuffPtr = pRxData;
    husart->RxXferSize = Size;
    husart->pTxBuffPtr = pRxData;
    husart->TxXferSize = Size;

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_RX;

    /* Set the USART DMA Rx transfer complete callback */
    husart->hdmarx->XferCpltCallback = USART_DMAReceiveCplt;

    /* Set the USART DMA Half transfer complete callback */
    husart->hdmarx->XferHalfCpltCallback = USART_DMARxHalfCplt;

    /* Set the USART DMA Rx transfer error callback */
    husart->hdmarx->XferErrorCallback = USART_DMAError;

    /* Set the DMA abort callback */
    husart->hdmarx->XferAbortCallback = NULL;

    /* Set the USART Tx DMA transfer complete callback as NULL because the communication closing
    is performed in DMA reception complete callback  */
    husart->hdmatx->XferHalfCpltCallback = NULL;
    husart->hdmatx->XferCpltCallback = NULL;

    /* Set the DMA error callback */
    husart->hdmatx->XferErrorCallback = USART_DMAError;

    /* Set the DMA AbortCpltCallback */
    husart->hdmatx->XferAbortCallback = NULL;

    /* Enable the USART receive DMA stream */
    tmp = (uint32_t *)&pRxData;
    DAL_DMA_Start_IT(husart->hdmarx, (uint32_t)&husart->Instance->DATA, *(uint32_t *)tmp, Size);

    /* Enable the USART transmit DMA stream: the transmit stream is used in order
       to generate in the non-blocking mode the clock to the slave device,
       this mode isn't a simplex receive mode but a full-duplex receive one */
    DAL_DMA_Start_IT(husart->hdmatx, *(uint32_t *)tmp, (uint32_t)&husart->Instance->DATA, Size);

    /* Clear the Overrun flag just before enabling the DMA Rx request: mandatory for the second transfer */
    __DAL_USART_CLEAR_OREFLAG(husart);

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    if (husart->Init.Parity != USART_PARITY_NONE)
    {
      /* Enable the USART Parity Error Interrupt */
      SET_BIT(husart->Instance->CTRL1, USART_CTRL1_PEIEN);
    }

    /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
       in the USART CTRL3 register */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Enable the DMA transfer for transmit request by setting the DMAT bit
       in the USART CTRL3 register */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Full-Duplex Transmit Receive an amount of data in DMA mode.
  * @note   When USART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data and the received data are handled as sets of u16. In this case, Size must indicate the number
  *         of u16 available through pTxData and through pRxData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pTxData Pointer to TX data buffer (u8 or u16 data elements).
  * @param  pRxData Pointer to RX data buffer (u8 or u16 data elements).
  * @param  Size    Amount of data elements (u8 or u16) to be received/sent.
  * @note   When the USART parity is enabled (PCE = 1) the data received contain the parity bit.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_TransmitReceive_DMA(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                                uint16_t Size)
{
  const uint32_t *tmp;

  if (husart->State == DAL_USART_STATE_READY)
  {
    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0))
    {
      return DAL_ERROR;
    }
    /* Process Locked */
    __DAL_LOCK(husart);

    husart->pRxBuffPtr = pRxData;
    husart->RxXferSize = Size;
    husart->pTxBuffPtr = pTxData;
    husart->TxXferSize = Size;

    husart->ErrorCode = DAL_USART_ERROR_NONE;
    husart->State = DAL_USART_STATE_BUSY_TX_RX;

    /* Set the USART DMA Rx transfer complete callback */
    husart->hdmarx->XferCpltCallback = USART_DMAReceiveCplt;

    /* Set the USART DMA Half transfer complete callback */
    husart->hdmarx->XferHalfCpltCallback = USART_DMARxHalfCplt;

    /* Set the USART DMA Tx transfer complete callback */
    husart->hdmatx->XferCpltCallback = USART_DMATransmitCplt;

    /* Set the USART DMA Half transfer complete callback */
    husart->hdmatx->XferHalfCpltCallback = USART_DMATxHalfCplt;

    /* Set the USART DMA Tx transfer error callback */
    husart->hdmatx->XferErrorCallback = USART_DMAError;

    /* Set the USART DMA Rx transfer error callback */
    husart->hdmarx->XferErrorCallback = USART_DMAError;

    /* Set the DMA abort callback */
    husart->hdmarx->XferAbortCallback = NULL;

    /* Enable the USART receive DMA stream */
    tmp = (uint32_t *)&pRxData;
    DAL_DMA_Start_IT(husart->hdmarx, (uint32_t)&husart->Instance->DATA, *(const uint32_t *)tmp, Size);

    /* Enable the USART transmit DMA stream */
    tmp = (const uint32_t *)&pTxData;
    DAL_DMA_Start_IT(husart->hdmatx, *(const uint32_t *)tmp, (uint32_t)&husart->Instance->DATA, Size);

    /* Clear the TC flag in the STS register by writing 0 to it */
    __DAL_USART_CLEAR_FLAG(husart, USART_FLAG_TC);

    /* Clear the Overrun flag: mandatory for the second transfer in circular mode */
    __DAL_USART_CLEAR_OREFLAG(husart);

    /* Process Unlocked */
    __DAL_UNLOCK(husart);

    if (husart->Init.Parity != USART_PARITY_NONE)
    {
      /* Enable the USART Parity Error Interrupt */
      SET_BIT(husart->Instance->CTRL1, USART_CTRL1_PEIEN);
    }

    /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
       in the USART CTRL3 register */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Enable the DMA transfer for transmit request by setting the DMAT bit
       in the USART CTRL3 register */
    SET_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Pauses the DMA Transfer.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_DMAPause(USART_HandleTypeDef *husart)
{
  /* Process Locked */
  __DAL_LOCK(husart);

  /* Disable the USART DMA Tx request */
  CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

  /* Process Unlocked */
  __DAL_UNLOCK(husart);

  return DAL_OK;
}

/**
  * @brief  Resumes the DMA Transfer.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_DMAResume(USART_HandleTypeDef *husart)
{
  /* Process Locked */
  __DAL_LOCK(husart);

  /* Enable the USART DMA Tx request */
  SET_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

  /* Process Unlocked */
  __DAL_UNLOCK(husart);

  return DAL_OK;
}

/**
  * @brief  Stops the DMA Transfer.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_DMAStop(USART_HandleTypeDef *husart)
{
  uint32_t dmarequest = 0x00U;
  /* The Lock is not implemented on this API to allow the user application
     to call the DAL USART API under callbacks DAL_USART_TxCpltCallback() / DAL_USART_RxCpltCallback():
     when calling DAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
     and the correspond call back is executed DAL_USART_TxCpltCallback() / DAL_USART_RxCpltCallback()
     */

  /* Stop USART DMA Tx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if ((husart->State == DAL_USART_STATE_BUSY_TX) && dmarequest)
  {
    USART_EndTxTransfer(husart);

    /* Abort the USART DMA Tx channel */
    if (husart->hdmatx != NULL)
    {
      DAL_DMA_Abort(husart->hdmatx);
    }

    /* Disable the USART Tx DMA request */
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);
  }

  /* Stop USART DMA Rx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if ((husart->State == DAL_USART_STATE_BUSY_RX) && dmarequest)
  {
    USART_EndRxTransfer(husart);

    /* Abort the USART DMA Rx channel */
    if (husart->hdmarx != NULL)
    {
      DAL_DMA_Abort(husart->hdmarx);
    }

    /* Disable the USART Rx DMA request */
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);
  }

  return DAL_OK;
}

/**
  * @brief  Abort ongoing transfer (blocking mode).
  * @param  husart USART handle.
  * @note   This procedure could be used for aborting any ongoing transfer (either Tx or Rx,
  *         as described by TransferType parameter) started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_Abort(USART_HandleTypeDef *husart)
{
  /* Disable TXEIE, TCIE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(husart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
  CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Disable the USART DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the USART DMA Tx channel : use blocking DMA Abort API (no callback) */
    if (husart->hdmatx != NULL)
    {
      /* Set the USART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      husart->hdmatx->XferAbortCallback = NULL;

      DAL_DMA_Abort(husart->hdmatx);
    }
  }

  /* Disable the USART DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the USART DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (husart->hdmarx != NULL)
    {
      /* Set the USART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      husart->hdmarx->XferAbortCallback = NULL;

      DAL_DMA_Abort(husart->hdmarx);
    }
  }

  /* Reset Tx and Rx transfer counters */
  husart->TxXferCount = 0x00U;
  husart->RxXferCount = 0x00U;

  /* Restore husart->State to Ready */
  husart->State  = DAL_USART_STATE_READY;

  /* Reset Handle ErrorCode to No Error */
  husart->ErrorCode = DAL_USART_ERROR_NONE;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing transfer (Interrupt mode).
  * @param  husart USART handle.
  * @note   This procedure could be used for aborting any ongoing transfer (either Tx or Rx,
  *         as described by TransferType parameter) started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_USART_Abort_IT(USART_HandleTypeDef *husart)
{
  uint32_t AbortCplt = 0x01U;

  /* Disable TXBEIEN, TXCIEN, RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(husart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
  CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* If DMA Tx and/or DMA Rx Handles are associated to USART Handle, DMA Abort complete callbacks should be initialised
     before any call to DMA Abort functions */
  /* DMA Tx Handle is valid */
  if (husart->hdmatx != NULL)
  {
    /* Set DMA Abort Complete callback if USART DMA Tx request if enabled.
       Otherwise, set it to NULL */
    if (DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMATXEN))
    {
      husart->hdmatx->XferAbortCallback = USART_DMATxAbortCallback;
    }
    else
    {
      husart->hdmatx->XferAbortCallback = NULL;
    }
  }
  /* DMA Rx Handle is valid */
  if (husart->hdmarx != NULL)
  {
    /* Set DMA Abort Complete callback if USART DMA Rx request if enabled.
       Otherwise, set it to NULL */
    if (DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMARXEN))
    {
      husart->hdmarx->XferAbortCallback = USART_DMARxAbortCallback;
    }
    else
    {
      husart->hdmarx->XferAbortCallback = NULL;
    }
  }

  /* Disable the USART DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    /* Disable DMA Tx at USART level */
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the USART DMA Tx channel : use non blocking DMA Abort API (callback) */
    if (husart->hdmatx != NULL)
    {
      /* USART Tx DMA Abort callback has already been initialised :
         will lead to call DAL_USART_AbortCpltCallback() at end of DMA abort procedure */

      /* Abort DMA TX */
      if (DAL_DMA_Abort_IT(husart->hdmatx) != DAL_OK)
      {
        husart->hdmatx->XferAbortCallback = NULL;
      }
      else
      {
        AbortCplt = 0x00U;
      }
    }
  }

  /* Disable the USART DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the USART DMA Rx channel : use non blocking DMA Abort API (callback) */
    if (husart->hdmarx != NULL)
    {
      /* USART Rx DMA Abort callback has already been initialised :
         will lead to call DAL_USART_AbortCpltCallback() at end of DMA abort procedure */

      /* Abort DMA RX */
      if (DAL_DMA_Abort_IT(husart->hdmarx) != DAL_OK)
      {
        husart->hdmarx->XferAbortCallback = NULL;
        AbortCplt = 0x01U;
      }
      else
      {
        AbortCplt = 0x00U;
      }
    }
  }

  /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
  if (AbortCplt  == 0x01U)
  {
    /* Reset Tx and Rx transfer counters */
    husart->TxXferCount = 0x00U;
    husart->RxXferCount = 0x00U;

    /* Reset errorCode */
    husart->ErrorCode = DAL_USART_ERROR_NONE;

    /* Restore husart->State to Ready */
    husart->State  = DAL_USART_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
    /* Call registered Abort Complete Callback */
    husart->AbortCpltCallback(husart);
#else
    /* Call legacy weak Abort Complete Callback */
    DAL_USART_AbortCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
  }

  return DAL_OK;
}

/**
  * @brief  This function handles USART interrupt request.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
void DAL_USART_IRQHandler(USART_HandleTypeDef *husart)
{
  uint32_t isrflags = READ_REG(husart->Instance->STS);
  uint32_t cr1its   = READ_REG(husart->Instance->CTRL1);
  uint32_t cr3its   = READ_REG(husart->Instance->CTRL3);
  uint32_t errorflags = 0x00U;
  uint32_t dmarequest = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_STS_PEFLG | USART_STS_FEFLG | USART_STS_OVREFLG | USART_STS_NEFLG));
  if (errorflags == RESET)
  {
    /* USART in mode Receiver -------------------------------------------------*/
    if (((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))
    {
      if (husart->State == DAL_USART_STATE_BUSY_RX)
      {
        USART_Receive_IT(husart);
      }
      else
      {
        USART_TransmitReceive_IT(husart);
      }
      return;
    }
  }
  /* If some errors occur */
  if ((errorflags != RESET) && (((cr3its & USART_CTRL3_ERRIEN) != RESET) || ((cr1its & (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN)) != RESET)))
  {
    /* USART parity error interrupt occurred ----------------------------------*/
    if (((isrflags & USART_STS_PEFLG) != RESET) && ((cr1its & USART_CTRL1_PEIEN) != RESET))
    {
      husart->ErrorCode |= DAL_USART_ERROR_PE;
    }

    /* USART noise error interrupt occurred --------------------------------*/
    if (((isrflags & USART_STS_NEFLG) != RESET) && ((cr3its & USART_CTRL3_ERRIEN) != RESET))
    {
      husart->ErrorCode |= DAL_USART_ERROR_NE;
    }

    /* USART frame error interrupt occurred --------------------------------*/
    if (((isrflags & USART_STS_FEFLG) != RESET) && ((cr3its & USART_CTRL3_ERRIEN) != RESET))
    {
      husart->ErrorCode |= DAL_USART_ERROR_FE;
    }

    /* USART Over-Run interrupt occurred -----------------------------------*/
    if (((isrflags & USART_STS_OVREFLG) != RESET) && (((cr1its & USART_CTRL1_RXBNEIEN) != RESET) || ((cr3its & USART_CTRL3_ERRIEN) != RESET)))
    {
      husart->ErrorCode |= DAL_USART_ERROR_ORE;
    }

    if (husart->ErrorCode != DAL_USART_ERROR_NONE)
    {
      /* USART in mode Receiver -----------------------------------------------*/
      if (((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))
      {
        if (husart->State == DAL_USART_STATE_BUSY_RX)
        {
          USART_Receive_IT(husart);
        }
        else
        {
          USART_TransmitReceive_IT(husart);
        }
      }
      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
      consider error as blocking */
      dmarequest = DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);
      if (((husart->ErrorCode & DAL_USART_ERROR_ORE) != RESET) || dmarequest)
      {
        /* Set the USART state ready to be able to start again the process,
        Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        USART_EndRxTransfer(husart);

        /* Disable the USART DMA Rx request if enabled */
        if (DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMARXEN))
        {
          CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);

          /* Abort the USART DMA Rx channel */
          if (husart->hdmarx != NULL)
          {
            /* Set the USART DMA Abort callback :
            will lead to call DAL_USART_ErrorCallback() at end of DMA abort procedure */
            husart->hdmarx->XferAbortCallback = USART_DMAAbortOnError;

            if (DAL_DMA_Abort_IT(husart->hdmarx) != DAL_OK)
            {
              /* Call Directly XferAbortCallback function in case of error */
              husart->hdmarx->XferAbortCallback(husart->hdmarx);
            }
          }
          else
          {
            /* Call user error callback */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
            /* Call registered Error Callback */
            husart->ErrorCallback(husart);
#else
            /* Call legacy weak Error Callback */
            DAL_USART_ErrorCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
          }
        }
        else
        {
          /* Call user error callback */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
          /* Call registered Error Callback */
          husart->ErrorCallback(husart);
#else
          /* Call legacy weak Error Callback */
          DAL_USART_ErrorCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
        }
      }
      else
      {
        /* Call user error callback */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
        /* Call registered Error Callback */
        husart->ErrorCallback(husart);
#else
        /* Call legacy weak Error Callback */
        DAL_USART_ErrorCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
        husart->ErrorCode = DAL_USART_ERROR_NONE;
      }
    }
    return;
  }

  /* USART in mode Transmitter -----------------------------------------------*/
  if (((isrflags & USART_STS_TXBEFLG) != RESET) && ((cr1its & USART_CTRL1_TXBEIEN) != RESET))
  {
    if (husart->State == DAL_USART_STATE_BUSY_TX)
    {
      USART_Transmit_IT(husart);
    }
    else
    {
      USART_TransmitReceive_IT(husart);
    }
    return;
  }

  /* USART in mode Transmitter (transmission end) ----------------------------*/
  if (((isrflags & USART_STS_TXCFLG) != RESET) && ((cr1its & USART_CTRL1_TXCIEN) != RESET))
  {
    USART_EndTransmit_IT(husart);
    return;
  }
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_USART_TxCpltCallback(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_USART_TxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_USART_TxHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_USART_RxCpltCallback(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_USART_RxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Rx Half Transfer completed callbacks.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_USART_RxHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Tx/Rx Transfers completed callback for the non-blocking process.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_USART_TxRxCpltCallback(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_USART_TxRxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  USART error callbacks.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_USART_ErrorCallback(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_USART_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  USART Abort Complete callback.
  * @param  husart USART handle.
  * @retval None
  */
__weak void DAL_USART_AbortCpltCallback(USART_HandleTypeDef *husart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(husart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_USART_AbortCpltCallback can be implemented in the user file.
   */
}

/**
  * @}
  */

/** @defgroup USART_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   USART State and Errors functions
  *
@verbatim
  ==============================================================================
                  ##### Peripheral State and Errors functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to return the State of
    USART communication
    process, return Peripheral Errors occurred during communication process
     (+) DAL_USART_GetState() API can be helpful to check in run-time the state
         of the USART peripheral.
     (+) DAL_USART_GetError() check in run-time errors that could be occurred during
         communication.
@endverbatim
  * @{
  */

/**
  * @brief  Returns the USART state.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL state
  */
DAL_USART_StateTypeDef DAL_USART_GetState(USART_HandleTypeDef *husart)
{
  return husart->State;
}

/**
  * @brief  Return the USART error code
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART.
  * @retval USART Error Code
  */
uint32_t DAL_USART_GetError(USART_HandleTypeDef *husart)
{
  return husart->ErrorCode;
}

/**
  * @}
  */

/** @defgroup USART_Private_Functions USART Private Functions
 * @{
 */

/**
  * @brief  Initialize the callbacks to their default values.
  * @param  husart USART handle.
  * @retval none
  */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
void USART_InitCallbacksToDefault(USART_HandleTypeDef *husart)
{
  /* Init the USART Callback settings */
  husart->TxHalfCpltCallback        = DAL_USART_TxHalfCpltCallback;        /* Legacy weak TxHalfCpltCallback        */
  husart->TxCpltCallback            = DAL_USART_TxCpltCallback;            /* Legacy weak TxCpltCallback            */
  husart->RxHalfCpltCallback        = DAL_USART_RxHalfCpltCallback;        /* Legacy weak RxHalfCpltCallback        */
  husart->RxCpltCallback            = DAL_USART_RxCpltCallback;            /* Legacy weak RxCpltCallback            */
  husart->TxRxCpltCallback          = DAL_USART_TxRxCpltCallback;          /* Legacy weak TxRxCpltCallback          */
  husart->ErrorCallback             = DAL_USART_ErrorCallback;             /* Legacy weak ErrorCallback             */
  husart->AbortCpltCallback         = DAL_USART_AbortCpltCallback;         /* Legacy weak AbortCpltCallback         */
}
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

/**
  * @brief  DMA USART transmit process complete callback.
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA module.
  * @retval None
  */
static void USART_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal mode */
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == 0U)
  {
    husart->TxXferCount = 0U;
    if (husart->State == DAL_USART_STATE_BUSY_TX)
    {
      /* Disable the DMA transfer for transmit request by resetting the DMAT bit
         in the USART CTRL3 register */
      CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

      /* Enable the USART Transmit Complete Interrupt */
      SET_BIT(husart->Instance->CTRL1, USART_CTRL1_TXCIEN);
    }
  }
  /* DMA Circular mode */
  else
  {
    if (husart->State == DAL_USART_STATE_BUSY_TX)
    {
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Tx Complete Callback */
      husart->TxCpltCallback(husart);
#else
      /* Call legacy weak Tx Complete Callback */
      DAL_USART_TxCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  DMA USART transmit process half complete callback
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA module.
  * @retval None
  */
static void USART_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
{
  USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Tx Half Complete Callback */
  husart->TxHalfCpltCallback(husart);
#else
  /* Call legacy weak Tx Half Complete Callback */
  DAL_USART_TxHalfCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART receive process complete callback.
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA module.
  * @retval None
  */
static void USART_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal mode */
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == 0U)
  {
    husart->RxXferCount = 0x00U;

    /* Disable RXNE, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
    CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_PEIEN);
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Disable the DMA transfer for the Transmit/receiver request by clearing the DMAT/DMAR bit
         in the USART CTRL3 register */
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);
    CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* The USART state is DAL_USART_STATE_BUSY_RX */
    if (husart->State == DAL_USART_STATE_BUSY_RX)
    {
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Rx Complete Callback */
      husart->RxCpltCallback(husart);
#else
      /* Call legacy weak Rx Complete Callback */
      DAL_USART_RxCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
    }
    /* The USART state is DAL_USART_STATE_BUSY_TX_RX */
    else
    {
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Tx Rx Complete Callback */
      husart->TxRxCpltCallback(husart);
#else
      /* Call legacy weak Tx Rx Complete Callback */
      DAL_USART_TxRxCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
    }
    husart->State = DAL_USART_STATE_READY;
  }
  /* DMA circular mode */
  else
  {
    if (husart->State == DAL_USART_STATE_BUSY_RX)
    {
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Rx Complete Callback */
      husart->RxCpltCallback(husart);
#else
      /* Call legacy weak Rx Complete Callback */
      DAL_USART_RxCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
    }
    /* The USART state is DAL_USART_STATE_BUSY_TX_RX */
    else
    {
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Tx Rx Complete Callback */
      husart->TxRxCpltCallback(husart);
#else
      /* Call legacy weak Tx Rx Complete Callback */
      DAL_USART_TxRxCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  DMA USART receive process half complete callback
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA module.
  * @retval None
  */
static void USART_DMARxHalfCplt(DMA_HandleTypeDef *hdma)
{
  USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Rx Half Complete Callback */
  husart->RxHalfCpltCallback(husart);
#else
  /* Call legacy weak Rx Half Complete Callback */
  DAL_USART_RxHalfCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART communication error callback.
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA module.
  * @retval None
  */
static void USART_DMAError(DMA_HandleTypeDef *hdma)
{
  uint32_t dmarequest = 0x00U;
  USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  husart->RxXferCount = 0x00U;
  husart->TxXferCount = 0x00U;

  /* Stop USART DMA Tx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if ((husart->State == DAL_USART_STATE_BUSY_TX) && dmarequest)
  {
    USART_EndTxTransfer(husart);
  }

  /* Stop USART DMA Rx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(husart->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if ((husart->State == DAL_USART_STATE_BUSY_RX) && dmarequest)
  {
    USART_EndRxTransfer(husart);
  }

  husart->ErrorCode |= DAL_USART_ERROR_DMA;
  husart->State = DAL_USART_STATE_READY;

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Error Callback */
  husart->ErrorCallback(husart);
#else
  /* Call legacy weak Error Callback */
  DAL_USART_ErrorCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  This function handles USART Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @param  Flag specifies the USART flag to check.
  * @param  Status The actual Flag status (SET or RESET).
  * @param  Tickstart Tick start value.
  * @param  Timeout Timeout duration.
  * @retval DAL status
  */
static DAL_StatusTypeDef USART_WaitOnFlagUntilTimeout(USART_HandleTypeDef *husart, uint32_t Flag, FlagStatus Status,
                                                      uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while ((__DAL_USART_GET_FLAG(husart, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != DAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((DAL_GetTick() - Tickstart) > Timeout))
      {
        /* Disable the USART Transmit Complete Interrupt */
        CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_TXBEIEN);

        /* Disable the USART RXNE Interrupt */
        CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_RXBNEIEN);

        /* Disable the USART Parity Error Interrupt */
        CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_PEIEN);

        /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

        husart->State = DAL_USART_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(husart);

        return DAL_TIMEOUT;
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  End ongoing Tx transfer on USART peripheral (following error detection or Transmit completion).
  * @param  husart USART handle.
  * @retval None
  */
static void USART_EndTxTransfer(USART_HandleTypeDef *husart)
{
  /* Disable TXBEIEN and TXCIEN interrupts */
  CLEAR_BIT(husart->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* At end of Tx process, restore husart->State to Ready */
  husart->State = DAL_USART_STATE_READY;
}

/**
  * @brief  End ongoing Rx transfer on USART peripheral (following error detection or Reception completion).
  * @param  husart USART handle.
  * @retval None
  */
static void USART_EndRxTransfer(USART_HandleTypeDef *husart)
{
  /* Disable RXNE, PE and ERR interrupts */
  CLEAR_BIT(husart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* At end of Rx process, restore husart->State to Ready */
  husart->State = DAL_USART_STATE_READY;
}

/**
  * @brief  DMA USART communication abort callback, when initiated by DAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  * @retval None
  */
static void USART_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  husart->RxXferCount = 0x00U;
  husart->TxXferCount = 0x00U;

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Error Callback */
  husart->ErrorCallback(husart);
#else
  /* Call legacy weak Error Callback */
  DAL_USART_ErrorCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void USART_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
{
  USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  husart->hdmatx->XferAbortCallback = NULL;

  /* Check if an Abort process is still ongoing */
  if (husart->hdmarx != NULL)
  {
    if (husart->hdmarx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  husart->TxXferCount = 0x00U;
  husart->RxXferCount = 0x00U;

  /* Reset errorCode */
  husart->ErrorCode = DAL_USART_ERROR_NONE;

  /* Restore husart->State to Ready */
  husart->State  = DAL_USART_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Complete Callback */
  husart->AbortCpltCallback(husart);
#else
  /* Call legacy weak Abort Complete Callback */
  DAL_USART_AbortCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA USART Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void USART_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
{
  USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  husart->hdmarx->XferAbortCallback = NULL;

  /* Check if an Abort process is still ongoing */
  if (husart->hdmatx != NULL)
  {
    if (husart->hdmatx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  husart->TxXferCount = 0x00U;
  husart->RxXferCount = 0x00U;

  /* Reset errorCode */
  husart->ErrorCode = DAL_USART_ERROR_NONE;

  /* Restore husart->State to Ready */
  husart->State  = DAL_USART_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Complete Callback */
  husart->AbortCpltCallback(husart);
#else
  /* Call legacy weak Abort Complete Callback */
  DAL_USART_AbortCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */
}

/**
  * @brief  Simplex Send an amount of data in non-blocking mode.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  * @note   The USART errors are not managed to avoid the overrun error.
  */
static DAL_StatusTypeDef USART_Transmit_IT(USART_HandleTypeDef *husart)
{
  const uint16_t *tmp;

  if (husart->State == DAL_USART_STATE_BUSY_TX)
  {
    if ((husart->Init.WordLength == USART_WORDLENGTH_9B) && (husart->Init.Parity == USART_PARITY_NONE))
    {
      tmp = (const uint16_t *) husart->pTxBuffPtr;
      husart->Instance->DATA = (uint16_t)(*tmp & (uint16_t)0x01FF);
      husart->pTxBuffPtr += 2U;
    }
    else
    {
      husart->Instance->DATA = (uint8_t)(*husart->pTxBuffPtr++ & (uint8_t)0x00FF);
    }

    if (--husart->TxXferCount == 0U)
    {
      /* Disable the USART Transmit data register empty Interrupt */
      CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_TXBEIEN);

      /* Enable the USART Transmit Complete Interrupt */
      SET_BIT(husart->Instance->CTRL1, USART_CTRL1_TXCIEN);
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
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  */
static DAL_StatusTypeDef USART_EndTransmit_IT(USART_HandleTypeDef *husart)
{
  /* Disable the USART Transmit Complete Interrupt */
  CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_TXCIEN);

  /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
  CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  husart->State = DAL_USART_STATE_READY;

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
  /* Call registered Tx Complete Callback */
  husart->TxCpltCallback(husart);
#else
  /* Call legacy weak Tx Complete Callback */
  DAL_USART_TxCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

  return DAL_OK;
}

/**
  * @brief  Simplex Receive an amount of data in non-blocking mode.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  */
static DAL_StatusTypeDef USART_Receive_IT(USART_HandleTypeDef *husart)
{
  uint8_t  *pdata8bits;
  uint16_t *pdata16bits;

  if (husart->State == DAL_USART_STATE_BUSY_RX)
  {
    if ((husart->Init.WordLength == USART_WORDLENGTH_9B) && (husart->Init.Parity == USART_PARITY_NONE))
    {
      pdata8bits  = NULL;
      pdata16bits = (uint16_t *) husart->pRxBuffPtr;
      *pdata16bits = (uint16_t)(husart->Instance->DATA & (uint16_t)0x01FF);
      husart->pRxBuffPtr += 2U;
    }
    else
    {
      pdata8bits = (uint8_t *) husart->pRxBuffPtr;
      pdata16bits  = NULL;

      if ((husart->Init.WordLength == USART_WORDLENGTH_9B) || ((husart->Init.WordLength == USART_WORDLENGTH_8B) && (husart->Init.Parity == USART_PARITY_NONE)))
      {
        *pdata8bits = (uint8_t)(husart->Instance->DATA & (uint8_t)0x00FF);
      }
      else
      {
        *pdata8bits = (uint8_t)(husart->Instance->DATA & (uint8_t)0x007F);
      }

      husart->pRxBuffPtr += 1U;
    }

    husart->RxXferCount--;

    if (husart->RxXferCount == 0U)
    {
      /* Disable the USART RXNE Interrupt */
      CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_RXBNEIEN);

      /* Disable the USART Parity Error Interrupt */
      CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_PEIEN);

      /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
      CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

      husart->State = DAL_USART_STATE_READY;
#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Rx Complete Callback */
      husart->RxCpltCallback(husart);
#else
      /* Call legacy weak Rx Complete Callback */
      DAL_USART_RxCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

      return DAL_OK;
    }
    else
    {
      /* Send dummy byte in order to generate the clock for the slave to send the next data.
      * Whatever the frame length (7, 8 or 9-bit long), the same dummy value
      * can be written for all the cases. */
      husart->Instance->DATA = (DUMMY_DATA & (uint16_t)0x0FF);
    }
    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Full-Duplex Send receive an amount of data in full-duplex mode (non-blocking).
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval DAL status
  */
static DAL_StatusTypeDef USART_TransmitReceive_IT(USART_HandleTypeDef *husart)
{
  const uint16_t *pdatatx16bits;
  uint16_t *pdatarx16bits;

  if (husart->State == DAL_USART_STATE_BUSY_TX_RX)
  {
    if (husart->TxXferCount != 0x00U)
    {
      if (__DAL_USART_GET_FLAG(husart, USART_FLAG_TXE) != RESET)
      {
        if ((husart->Init.WordLength == USART_WORDLENGTH_9B) && (husart->Init.Parity == USART_PARITY_NONE))
        {
          pdatatx16bits = (const uint16_t *) husart->pTxBuffPtr;
          husart->Instance->DATA = (uint16_t)(*pdatatx16bits & (uint16_t)0x01FF);
          husart->pTxBuffPtr += 2U;
        }
        else
        {
          husart->Instance->DATA = (uint8_t)(*husart->pTxBuffPtr++ & (uint8_t)0x00FF);
        }

        husart->TxXferCount--;

        /* Check the latest data transmitted */
        if (husart->TxXferCount == 0U)
        {
          CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_TXBEIEN);
        }
      }
    }

    if (husart->RxXferCount != 0x00U)
    {
      if (__DAL_USART_GET_FLAG(husart, USART_FLAG_RXNE) != RESET)
      {
        if ((husart->Init.WordLength == USART_WORDLENGTH_9B) && (husart->Init.Parity == USART_PARITY_NONE))
        {
          pdatarx16bits = (uint16_t *) husart->pRxBuffPtr;
          *pdatarx16bits = (uint16_t)(husart->Instance->DATA & (uint16_t)0x01FF);
          husart->pRxBuffPtr += 2U;
        }
        else
        {
          if ((husart->Init.WordLength == USART_WORDLENGTH_9B) || ((husart->Init.WordLength == USART_WORDLENGTH_8B) && (husart->Init.Parity == USART_PARITY_NONE)))
          {
            *husart->pRxBuffPtr = (uint8_t)(husart->Instance->DATA & (uint8_t)0x00FF);
          }
          else
          {
            *husart->pRxBuffPtr = (uint8_t)(husart->Instance->DATA & (uint8_t)0x007F);
          }
          husart->pRxBuffPtr += 1U;
        }

        husart->RxXferCount--;
      }
    }

    /* Check the latest data received */
    if (husart->RxXferCount == 0U)
    {
      /* Disable the USART RXBNEIEN Interrupt */
      CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_RXBNEIEN);

      /* Disable the USART Parity Error Interrupt */
      CLEAR_BIT(husart->Instance->CTRL1, USART_CTRL1_PEIEN);

      /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
      CLEAR_BIT(husart->Instance->CTRL3, USART_CTRL3_ERRIEN);

      husart->State = DAL_USART_STATE_READY;

#if (USE_DAL_USART_REGISTER_CALLBACKS == 1)
      /* Call registered Tx Rx Complete Callback */
      husart->TxRxCpltCallback(husart);
#else
      /* Call legacy weak Tx Rx Complete Callback */
      DAL_USART_TxRxCpltCallback(husart);
#endif /* USE_DAL_USART_REGISTER_CALLBACKS */

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
  * @brief  Configures the USART peripheral.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
static void USART_SetConfig(USART_HandleTypeDef *husart)
{
  uint32_t tmpreg = 0x00U;
  uint32_t pclk;

  /* Check the parameters */
  ASSERT_PARAM(IS_USART_INSTANCE(husart->Instance));
  ASSERT_PARAM(IS_USART_POLARITY(husart->Init.CLKPolarity));
  ASSERT_PARAM(IS_USART_PHASE(husart->Init.CLKPhase));
  ASSERT_PARAM(IS_USART_LASTBIT(husart->Init.CLKLastBit));
  ASSERT_PARAM(IS_USART_BAUDRATE(husart->Init.BaudRate));
  ASSERT_PARAM(IS_USART_WORD_LENGTH(husart->Init.WordLength));
  ASSERT_PARAM(IS_USART_STOPBITS(husart->Init.StopBits));
  ASSERT_PARAM(IS_USART_PARITY(husart->Init.Parity));
  ASSERT_PARAM(IS_USART_MODE(husart->Init.Mode));

  /* The LBCL, CPOL and CPHA bits have to be selected when both the transmitter and the
     receiver are disabled (TE=RE=0) to ensure that the clock pulses function correctly. */
  CLEAR_BIT(husart->Instance->CTRL1, (USART_CTRL1_TXEN | USART_CTRL1_RXEN));

  /*---------------------------- USART CTRL2 Configuration ---------------------*/
  tmpreg = husart->Instance->CTRL2;
  /* Clear CLKEN, CPOL, CPHA and LBCL bits */
  tmpreg &= (uint32_t)~((uint32_t)(USART_CTRL2_CPHA | USART_CTRL2_CPOL | USART_CTRL2_CLKEN | USART_CTRL2_LBCPOEN | USART_CTRL2_STOPCFG));
  /* Configure the USART Clock, CPOL, CPHA and LastBit -----------------------*/
  /* Set CPOL bit according to husart->Init.CLKPolarity value */
  /* Set CPHA bit according to husart->Init.CLKPhase value */
  /* Set LBCL bit according to husart->Init.CLKLastBit value */
  /* Set Stop Bits: Set STOP[13:12] bits according to husart->Init.StopBits value */
  tmpreg |= (uint32_t)(USART_CLOCK_ENABLE | husart->Init.CLKPolarity |
                       husart->Init.CLKPhase | husart->Init.CLKLastBit | husart->Init.StopBits);
  /* Write to USART CTRL2 */
  WRITE_REG(husart->Instance->CTRL2, (uint32_t)tmpreg);

  /*-------------------------- USART CTRL1 Configuration -----------------------*/
  tmpreg = husart->Instance->CTRL1;

  /* Clear M, PCE, PS, TE, RE and OVER8 bits */
  tmpreg &= (uint32_t)~((uint32_t)(USART_CTRL1_DBLCFG | USART_CTRL1_PCEN | USART_CTRL1_PCFG | USART_CTRL1_TXEN | \
                                   USART_CTRL1_RXEN | USART_CTRL1_OSMCFG));

  /* Configure the USART Word Length, Parity and mode:
     Set the M bits according to husart->Init.WordLength value
     Set PCE and PS bits according to husart->Init.Parity value
     Set TE and RE bits according to husart->Init.Mode value
     Force OVER8 bit to 1 in order to reach the max USART frequencies */
  tmpreg |= (uint32_t)husart->Init.WordLength | husart->Init.Parity | husart->Init.Mode | USART_CTRL1_OSMCFG;

  /* Write to USART CTRL1 */
  WRITE_REG(husart->Instance->CTRL1, (uint32_t)tmpreg);

  /*-------------------------- USART CTRL3 Configuration -----------------------*/
  /* Clear CTSEN and RTSEN bits */
  CLEAR_BIT(husart->Instance->CTRL3, (USART_CTRL3_RTSEN | USART_CTRL3_CTSEN));

  /*-------------------------- USART BR Configuration -----------------------*/
#if defined(USART6) && defined(UART9) && defined(UART10)
   if ((husart->Instance == USART1) || (husart->Instance == USART6) || (husart->Instance == UART9) || (husart->Instance == UART10))
   {
    pclk = DAL_RCM_GetPCLK2Freq();
    husart->Instance->BR = USART_BR(pclk, husart->Init.BaudRate);
   }
#elif defined(USART6)
  if((husart->Instance == USART1) || (husart->Instance == USART6))
  {
    pclk = DAL_RCM_GetPCLK2Freq();
    husart->Instance->BR = USART_BR(pclk, husart->Init.BaudRate);
  }
#else
  if(husart->Instance == USART1)
  {
    pclk = DAL_RCM_GetPCLK2Freq();
    husart->Instance->BR = USART_BR(pclk, husart->Init.BaudRate);
  }
#endif /* USART6 || UART9 || UART10 */	
  else
  {
    pclk = DAL_RCM_GetPCLK1Freq();
    husart->Instance->BR = USART_BR(pclk, husart->Init.BaudRate);
  }
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_USART_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

