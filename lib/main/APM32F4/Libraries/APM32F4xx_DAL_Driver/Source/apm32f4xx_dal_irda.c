/**
  *
  * @file    apm32f4xx_dal_irda.c
  * @brief   IRDA DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the IrDA SIR ENDEC block (IrDA):
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Errors functions
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
    The IRDA DAL driver can be used as follows:

    (#) Declare a IRDA_HandleTypeDef handle structure (eg. IRDA_HandleTypeDef hirda).
    (#) Initialize the IRDA low level resources by implementing the DAL_IRDA_MspInit() API:
        (##) Enable the USARTx interface clock.
        (##) IRDA pins configuration:
            (+++) Enable the clock for the IRDA GPIOs.
            (+++) Configure IRDA pins as alternate function pull-up.
        (##) NVIC configuration if you need to use interrupt process (DAL_IRDA_Transmit_IT()
             and DAL_IRDA_Receive_IT() APIs):
            (+++) Configure the USARTx interrupt priority.
            (+++) Enable the NVIC USART IRQ handle.
        (##) DMA Configuration if you need to use DMA process (DAL_IRDA_Transmit_DMA()
             and DAL_IRDA_Receive_DMA() APIs):
            (+++) Declare a DMA handle structure for the Tx/Rx stream.
            (+++) Enable the DMAx interface clock.
            (+++) Configure the declared DMA handle structure with the required Tx/Rx parameters.
            (+++) Configure the DMA Tx/Rx stream.
            (+++) Associate the initialized DMA handle to the IRDA DMA Tx/Rx handle.
            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA Tx/Rx stream.
            (+++) Configure the IRDAx interrupt priority and enable the NVIC USART IRQ handle
                  (used for last byte sending completion detection in DMA non circular mode)

    (#) Program the Baud Rate, Word Length, Parity, IrDA Mode, Prescaler
        and Mode(Receiver/Transmitter) in the hirda Init structure.

    (#) Initialize the IRDA registers by calling the DAL_IRDA_Init() API:
        (++) This API configures also the low level Hardware GPIO, CLOCK, CORTEX...etc)
             by calling the customized DAL_IRDA_MspInit() API.

         -@@- The specific IRDA interrupts (Transmission complete interrupt,
             RXNE interrupt and Error Interrupts) will be managed using the macros
             __DAL_IRDA_ENABLE_IT() and __DAL_IRDA_DISABLE_IT() inside the transmit and receive process.

    (#) Three operation modes are available within this driver :

    *** Polling mode IO operation ***
    =================================
    [..]
      (+) Send an amount of data in blocking mode using DAL_IRDA_Transmit()
      (+) Receive an amount of data in blocking mode using DAL_IRDA_Receive()

    *** Interrupt mode IO operation ***
    ===================================
    [..]
      (+) Send an amount of data in non blocking mode using DAL_IRDA_Transmit_IT()
      (+) At transmission end of transfer DAL_IRDA_TxCpltCallback is executed and user can
           add his own code by customization of function pointer DAL_IRDA_TxCpltCallback
      (+) Receive an amount of data in non blocking mode using DAL_IRDA_Receive_IT()
      (+) At reception end of transfer DAL_IRDA_RxCpltCallback is executed and user can
           add his own code by customization of function pointer DAL_IRDA_RxCpltCallback
      (+) In case of transfer Error, DAL_IRDA_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer DAL_IRDA_ErrorCallback

    *** DMA mode IO operation ***
    =============================
    [..]
      (+) Send an amount of data in non blocking mode (DMA) using DAL_IRDA_Transmit_DMA()
      (+) At transmission end of half transfer DAL_IRDA_TxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_IRDA_TxHalfCpltCallback
      (+) At transmission end of transfer DAL_IRDA_TxCpltCallback is executed and user can
           add his own code by customization of function pointer DAL_IRDA_TxCpltCallback
      (+) Receive an amount of data in non blocking mode (DMA) using DAL_IRDA_Receive_DMA()
      (+) At reception end of half transfer DAL_IRDA_RxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_IRDA_RxHalfCpltCallback
      (+) At reception end of transfer DAL_IRDA_RxCpltCallback is executed and user can
           add his own code by customization of function pointer DAL_IRDA_RxCpltCallback
      (+) In case of transfer Error, DAL_IRDA_ErrorCallback() function is executed and user can
           add his own code by customization of function pointer DAL_IRDA_ErrorCallback
      (+) Pause the DMA Transfer using DAL_IRDA_DMAPause()
      (+) Resume the DMA Transfer using DAL_IRDA_DMAResume()
      (+) Stop the DMA Transfer using DAL_IRDA_DMAStop()

    *** IRDA DAL driver macros list ***
    ===================================
    [..]
      Below the list of most used macros in IRDA DAL driver.

       (+) __DAL_IRDA_ENABLE: Enable the IRDA peripheral
       (+) __DAL_IRDA_DISABLE: Disable the IRDA peripheral
       (+) __DAL_IRDA_GET_FLAG : Check whether the specified IRDA flag is set or not
       (+) __DAL_IRDA_CLEAR_FLAG : Clear the specified IRDA pending flag
       (+) __DAL_IRDA_ENABLE_IT: Enable the specified IRDA interrupt
       (+) __DAL_IRDA_DISABLE_IT: Disable the specified IRDA interrupt
       (+) __DAL_IRDA_GET_IT_SOURCE: Check whether the specified IRDA interrupt has occurred or not

    [..]
     (@) You can refer to the IRDA DAL driver header file for more useful macros

    ##### Callback registration #####
    ==================================

    [..]
      The compilation define USE_DAL_IRDA_REGISTER_CALLBACKS when set to 1
      allows the user to configure dynamically the driver callbacks.

    [..]
      Use Function DAL_IRDA_RegisterCallback() to register a user callback.
      Function DAL_IRDA_RegisterCallback() allows to register following callbacks:
       (+) TxHalfCpltCallback        : Tx Half Complete Callback.
       (+) TxCpltCallback            : Tx Complete Callback.
       (+) RxHalfCpltCallback        : Rx Half Complete Callback.
       (+) RxCpltCallback            : Rx Complete Callback.
       (+) ErrorCallback             : Error Callback.
       (+) AbortCpltCallback         : Abort Complete Callback.
       (+) AbortTransmitCpltCallback : Abort Transmit Complete Callback.
       (+) AbortReceiveCpltCallback  : Abort Receive Complete Callback.
       (+) MspInitCallback           : IRDA MspInit.
       (+) MspDeInitCallback         : IRDA MspDeInit.
      This function takes as parameters the DAL peripheral handle, the Callback ID
      and a pointer to the user callback function.

    [..]
      Use function DAL_IRDA_UnRegisterCallback() to reset a callback to the default
      weak (surcharged) function.
      DAL_IRDA_UnRegisterCallback() takes as parameters the DAL peripheral handle,
      and the Callback ID.
      This function allows to reset following callbacks:
       (+) TxHalfCpltCallback        : Tx Half Complete Callback.
       (+) TxCpltCallback            : Tx Complete Callback.
       (+) RxHalfCpltCallback        : Rx Half Complete Callback.
       (+) RxCpltCallback            : Rx Complete Callback.
       (+) ErrorCallback             : Error Callback.
       (+) AbortCpltCallback         : Abort Complete Callback.
       (+) AbortTransmitCpltCallback : Abort Transmit Complete Callback.
       (+) AbortReceiveCpltCallback  : Abort Receive Complete Callback.
       (+) MspInitCallback           : IRDA MspInit.
       (+) MspDeInitCallback         : IRDA MspDeInit.

    [..]
      By default, after the DAL_IRDA_Init() and when the state is DAL_IRDA_STATE_RESET
      all callbacks are set to the corresponding weak (surcharged) functions:
      examples DAL_IRDA_TxCpltCallback(), DAL_IRDA_RxHalfCpltCallback().
      Exception done for MspInit and MspDeInit functions that are respectively
      reset to the legacy weak (surcharged) functions in the DAL_IRDA_Init()
      and DAL_IRDA_DeInit() only when these callbacks are null (not registered beforehand).
      If not, MspInit or MspDeInit are not null, the DAL_IRDA_Init() and DAL_IRDA_DeInit()
      keep and use the user MspInit/MspDeInit callbacks (registered beforehand).

    [..]
      Callbacks can be registered/unregistered in DAL_IRDA_STATE_READY state only.
      Exception done MspInit/MspDeInit that can be registered/unregistered
      in DAL_IRDA_STATE_READY or DAL_IRDA_STATE_RESET state, thus registered (user)
      MspInit/DeInit callbacks can be used during the Init/DeInit.
      In that case first register the MspInit/MspDeInit user callbacks
      using DAL_IRDA_RegisterCallback() before calling DAL_IRDA_DeInit()
      or DAL_IRDA_Init() function.

    [..]
      When The compilation define USE_DAL_IRDA_REGISTER_CALLBACKS is set to 0 or
      not defined, the callback registration feature is not available
      and weak (surcharged) callbacks are used.

  @endverbatim
     [..]
       (@) Additional remark: If the parity is enabled, then the MSB bit of the data written
           in the data register is transmitted but is changed by the parity bit.
           Depending on the frame length defined by the M bit (8-bits or 9-bits),
           the possible IRDA frame formats are as listed in the following table:
    +-------------------------------------------------------------+
    |   M bit |  PCE bit  |            IRDA frame                 |
    |---------------------|---------------------------------------|
    |    0    |    0      |    | SB | 8 bit data | 1 STB |        |
    |---------|-----------|---------------------------------------|
    |    0    |    1      |    | SB | 7 bit data | PB | 1 STB |   |
    |---------|-----------|---------------------------------------|
    |    1    |    0      |    | SB | 9 bit data | 1 STB |        |
    |---------|-----------|---------------------------------------|
    |    1    |    1      |    | SB | 8 bit data | PB | 1 STB |   |
    +-------------------------------------------------------------+
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup IRDA IRDA
  * @brief DAL IRDA module driver
  * @{
  */

#ifdef DAL_IRDA_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup IRDA_Private_Functions
  * @{
  */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
void IRDA_InitCallbacksToDefault(IRDA_HandleTypeDef *hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACKS */
static void IRDA_SetConfig(IRDA_HandleTypeDef *hirda);
static DAL_StatusTypeDef IRDA_Transmit_IT(IRDA_HandleTypeDef *hirda);
static DAL_StatusTypeDef IRDA_EndTransmit_IT(IRDA_HandleTypeDef *hirda);
static DAL_StatusTypeDef IRDA_Receive_IT(IRDA_HandleTypeDef *hirda);
static void IRDA_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void IRDA_DMATransmitHalfCplt(DMA_HandleTypeDef *hdma);
static void IRDA_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void IRDA_DMAReceiveHalfCplt(DMA_HandleTypeDef *hdma);
static void IRDA_DMAError(DMA_HandleTypeDef *hdma);
static void IRDA_DMAAbortOnError(DMA_HandleTypeDef *hdma);
static void IRDA_DMATxAbortCallback(DMA_HandleTypeDef *hdma);
static void IRDA_DMARxAbortCallback(DMA_HandleTypeDef *hdma);
static void IRDA_DMATxOnlyAbortCallback(DMA_HandleTypeDef *hdma);
static void IRDA_DMARxOnlyAbortCallback(DMA_HandleTypeDef *hdma);
static DAL_StatusTypeDef IRDA_WaitOnFlagUntilTimeout(IRDA_HandleTypeDef *hirda, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout);
static void IRDA_EndTxTransfer(IRDA_HandleTypeDef *hirda);
static void IRDA_EndRxTransfer(IRDA_HandleTypeDef *hirda);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup IRDA_Exported_Functions IrDA Exported Functions
  * @{
  */

/** @defgroup IRDA_Exported_Functions_Group1 IrDA Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim

  ==============================================================================
            ##### Initialization and Configuration functions #####
  ==============================================================================
    [..]
    This subsection provides a set of functions allowing to initialize the USARTx or the UARTy
    in asynchronous IrDA mode.
      (+) For the asynchronous mode only these parameters can be configured:
        (++) BaudRate
        (++) WordLength
        (++) Parity: If the parity is enabled, then the MSB bit of the data written
             in the data register is transmitted but is changed by the parity bit.
             Depending on the frame length defined by the M bit (8-bits or 9-bits),
             please refer to Reference manual for possible IRDA frame formats.
        (++) Prescaler: A pulse of width less than two and greater than one PSC period(s) may or may
             not be rejected. The receiver set up time should be managed by software. The IrDA physical layer
             specification specifies a minimum of 10 ms delay between transmission and
             reception (IrDA is a half duplex protocol).
        (++) Mode: Receiver/transmitter modes
        (++) IrDAMode: the IrDA can operate in the Normal mode or in the Low power mode.
    [..]
    The DAL_IRDA_Init() API follows IRDA configuration procedures (details for the procedures
    are available in reference manual).

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the IRDA mode according to the specified
  *         parameters in the IRDA_InitTypeDef and create the associated handle.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_Init(IRDA_HandleTypeDef *hirda)
{
  /* Check the IRDA handle allocation */
  if (hirda == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the IRDA instance parameters */
  ASSERT_PARAM(IS_IRDA_INSTANCE(hirda->Instance));
  /* Check the IRDA mode parameter in the IRDA handle */
  ASSERT_PARAM(IS_IRDA_POWERMODE(hirda->Init.IrDAMode));

  if (hirda->gState == DAL_IRDA_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hirda->Lock = DAL_UNLOCKED;

#if USE_DAL_IRDA_REGISTER_CALLBACKS == 1
    IRDA_InitCallbacksToDefault(hirda);

    if (hirda->MspInitCallback == NULL)
    {
      hirda->MspInitCallback = DAL_IRDA_MspInit;
    }

    /* Init the low level hardware */
    hirda->MspInitCallback(hirda);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    DAL_IRDA_MspInit(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACKS */
  }

  hirda->gState = DAL_IRDA_STATE_BUSY;

  /* Disable the IRDA peripheral */
  __DAL_IRDA_DISABLE(hirda);

  /* Set the IRDA communication parameters */
  IRDA_SetConfig(hirda);

  /* In IrDA mode, the following bits must be kept cleared:
  - LINEN, STOP and CLKEN bits in the USART_CTRL2 register,
  - SCEN and HDSEL bits in the USART_CTRL3 register.*/
  CLEAR_BIT(hirda->Instance->CTRL2, (USART_CTRL2_LINMEN | USART_CTRL2_STOPCFG | USART_CTRL2_CLKEN));
  CLEAR_BIT(hirda->Instance->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_HDEN));

  /* Enable the IRDA peripheral */
  __DAL_IRDA_ENABLE(hirda);

  /* Set the prescaler */
  MODIFY_REG(hirda->Instance->GTPSC, USART_GTPSC_PSC, hirda->Init.Prescaler);

  /* Configure the IrDA mode */
  MODIFY_REG(hirda->Instance->CTRL3, USART_CTRL3_IRLPEN, hirda->Init.IrDAMode);

  /* Enable the IrDA mode by setting the IREN bit in the CTRL3 register */
  SET_BIT(hirda->Instance->CTRL3, USART_CTRL3_IREN);

  /* Initialize the IRDA state*/
  hirda->ErrorCode = DAL_IRDA_ERROR_NONE;
  hirda->gState = DAL_IRDA_STATE_READY;
  hirda->RxState = DAL_IRDA_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the IRDA peripheral
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_DeInit(IRDA_HandleTypeDef *hirda)
{
  /* Check the IRDA handle allocation */
  if (hirda == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_IRDA_INSTANCE(hirda->Instance));

  hirda->gState = DAL_IRDA_STATE_BUSY;

  /* Disable the Peripheral */
  __DAL_IRDA_DISABLE(hirda);

  /* DeInit the low level hardware */
#if USE_DAL_IRDA_REGISTER_CALLBACKS == 1
  if (hirda->MspDeInitCallback == NULL)
  {
    hirda->MspDeInitCallback = DAL_IRDA_MspDeInit;
  }
  /* DeInit the low level hardware */
  hirda->MspDeInitCallback(hirda);
#else
  DAL_IRDA_MspDeInit(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACKS */

  hirda->ErrorCode = DAL_IRDA_ERROR_NONE;

  hirda->gState = DAL_IRDA_STATE_RESET;
  hirda->RxState = DAL_IRDA_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hirda);

  return DAL_OK;
}

/**
  * @brief  IRDA MSP Init.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_MspInit(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_IRDA_MspInit can be implemented in the user file
   */
}

/**
  * @brief  IRDA MSP DeInit.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_MspDeInit(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_IRDA_MspDeInit can be implemented in the user file
   */
}

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User IRDA Callback
  *         To be used instead of the weak predefined callback
  * @param  hirda irda handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_IRDA_TX_HALFCOMPLETE_CB_ID Tx Half Complete Callback ID
  *           @arg @ref DAL_IRDA_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref DAL_IRDA_RX_HALFCOMPLETE_CB_ID Rx Half Complete Callback ID
  *           @arg @ref DAL_IRDA_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_IRDA_ERROR_CB_ID Error Callback ID
  *           @arg @ref DAL_IRDA_ABORT_COMPLETE_CB_ID Abort Complete Callback ID
  *           @arg @ref DAL_IRDA_ABORT_TRANSMIT_COMPLETE_CB_ID Abort Transmit Complete Callback ID
  *           @arg @ref DAL_IRDA_ABORT_RECEIVE_COMPLETE_CB_ID Abort Receive Complete Callback ID
  *           @arg @ref DAL_IRDA_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref DAL_IRDA_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_RegisterCallback(IRDA_HandleTypeDef *hirda, DAL_IRDA_CallbackIDTypeDef CallbackID, pIRDA_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hirda->ErrorCode |= DAL_IRDA_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hirda);

  if (hirda->gState == DAL_IRDA_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_IRDA_TX_HALFCOMPLETE_CB_ID :
        hirda->TxHalfCpltCallback = pCallback;
        break;

      case DAL_IRDA_TX_COMPLETE_CB_ID :
        hirda->TxCpltCallback = pCallback;
        break;

      case DAL_IRDA_RX_HALFCOMPLETE_CB_ID :
        hirda->RxHalfCpltCallback = pCallback;
        break;

      case DAL_IRDA_RX_COMPLETE_CB_ID :
        hirda->RxCpltCallback = pCallback;
        break;

      case DAL_IRDA_ERROR_CB_ID :
        hirda->ErrorCallback = pCallback;
        break;

      case DAL_IRDA_ABORT_COMPLETE_CB_ID :
        hirda->AbortCpltCallback = pCallback;
        break;

      case DAL_IRDA_ABORT_TRANSMIT_COMPLETE_CB_ID :
        hirda->AbortTransmitCpltCallback = pCallback;
        break;

      case DAL_IRDA_ABORT_RECEIVE_COMPLETE_CB_ID :
        hirda->AbortReceiveCpltCallback = pCallback;
        break;

      case DAL_IRDA_MSPINIT_CB_ID :
        hirda->MspInitCallback = pCallback;
        break;

      case DAL_IRDA_MSPDEINIT_CB_ID :
        hirda->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hirda->ErrorCode |= DAL_IRDA_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hirda->gState == DAL_IRDA_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_IRDA_MSPINIT_CB_ID :
        hirda->MspInitCallback = pCallback;
        break;

      case DAL_IRDA_MSPDEINIT_CB_ID :
        hirda->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hirda->ErrorCode |= DAL_IRDA_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hirda->ErrorCode |= DAL_IRDA_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hirda);

  return status;
}

/**
  * @brief  Unregister an IRDA callback
  *         IRDA callback is redirected to the weak predefined callback
  * @param  hirda irda handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_IRDA_TX_HALFCOMPLETE_CB_ID Tx Half Complete Callback ID
  *           @arg @ref DAL_IRDA_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref DAL_IRDA_RX_HALFCOMPLETE_CB_ID Rx Half Complete Callback ID
  *           @arg @ref DAL_IRDA_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_IRDA_ERROR_CB_ID Error Callback ID
  *           @arg @ref DAL_IRDA_ABORT_COMPLETE_CB_ID Abort Complete Callback ID
  *           @arg @ref DAL_IRDA_ABORT_TRANSMIT_COMPLETE_CB_ID Abort Transmit Complete Callback ID
  *           @arg @ref DAL_IRDA_ABORT_RECEIVE_COMPLETE_CB_ID Abort Receive Complete Callback ID
  *           @arg @ref DAL_IRDA_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref DAL_IRDA_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_UnRegisterCallback(IRDA_HandleTypeDef *hirda, DAL_IRDA_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hirda);

  if (DAL_IRDA_STATE_READY == hirda->gState)
  {
    switch (CallbackID)
    {
      case DAL_IRDA_TX_HALFCOMPLETE_CB_ID :
        hirda->TxHalfCpltCallback = DAL_IRDA_TxHalfCpltCallback;               /* Legacy weak  TxHalfCpltCallback       */
        break;

      case DAL_IRDA_TX_COMPLETE_CB_ID :
        hirda->TxCpltCallback = DAL_IRDA_TxCpltCallback;                       /* Legacy weak TxCpltCallback            */
        break;

      case DAL_IRDA_RX_HALFCOMPLETE_CB_ID :
        hirda->RxHalfCpltCallback = DAL_IRDA_RxHalfCpltCallback;               /* Legacy weak RxHalfCpltCallback        */
        break;

      case DAL_IRDA_RX_COMPLETE_CB_ID :
        hirda->RxCpltCallback = DAL_IRDA_RxCpltCallback;                       /* Legacy weak RxCpltCallback            */
        break;

      case DAL_IRDA_ERROR_CB_ID :
        hirda->ErrorCallback = DAL_IRDA_ErrorCallback;                         /* Legacy weak ErrorCallback             */
        break;

      case DAL_IRDA_ABORT_COMPLETE_CB_ID :
        hirda->AbortCpltCallback = DAL_IRDA_AbortCpltCallback;                 /* Legacy weak AbortCpltCallback         */
        break;

      case DAL_IRDA_ABORT_TRANSMIT_COMPLETE_CB_ID :
        hirda->AbortTransmitCpltCallback = DAL_IRDA_AbortTransmitCpltCallback; /* Legacy weak AbortTransmitCpltCallback */
        break;

      case DAL_IRDA_ABORT_RECEIVE_COMPLETE_CB_ID :
        hirda->AbortReceiveCpltCallback = DAL_IRDA_AbortReceiveCpltCallback;   /* Legacy weak AbortReceiveCpltCallback  */
        break;

      case DAL_IRDA_MSPINIT_CB_ID :
        hirda->MspInitCallback = DAL_IRDA_MspInit;                             /* Legacy weak MspInitCallback           */
        break;

      case DAL_IRDA_MSPDEINIT_CB_ID :
        hirda->MspDeInitCallback = DAL_IRDA_MspDeInit;                         /* Legacy weak MspDeInitCallback         */
        break;

      default :
        /* Update the error code */
        hirda->ErrorCode |= DAL_IRDA_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_IRDA_STATE_RESET == hirda->gState)
  {
    switch (CallbackID)
    {
      case DAL_IRDA_MSPINIT_CB_ID :
        hirda->MspInitCallback = DAL_IRDA_MspInit;
        break;

      case DAL_IRDA_MSPDEINIT_CB_ID :
        hirda->MspDeInitCallback = DAL_IRDA_MspDeInit;
        break;

      default :
        /* Update the error code */
        hirda->ErrorCode |= DAL_IRDA_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hirda->ErrorCode |= DAL_IRDA_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hirda);

  return status;
}
#endif /* USE_DAL_IRDA_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup IRDA_Exported_Functions_Group2 IO operation functions
  *  @brief   IRDA Transmit and Receive functions
  *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
  ==============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the IRDA data transfers.
    IrDA is a half duplex communication protocol. If the Transmitter is busy, any data
    on the IrDA receive line will be ignored by the IrDA decoder and if the Receiver
    is busy, data on the TX from the USART to IrDA will not be encoded by IrDA.
    While receiving data, transmission should be avoided as the data to be transmitted
    could be corrupted.

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The DAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) Non-Blocking mode: The communication is performed using Interrupts
           or DMA, these API's return the DAL status.
           The end of the data processing will be indicated through the
           dedicated IRDA IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The DAL_IRDA_TxCpltCallback(), DAL_IRDA_RxCpltCallback() user callbacks
           will be executed respectively at the end of the Transmit or Receive process
           The DAL_IRDA_ErrorCallback() user callback will be executed when a communication error is detected

    (#) Blocking mode APIs are :
        (++) DAL_IRDA_Transmit()
        (++) DAL_IRDA_Receive()

    (#) Non Blocking mode APIs with Interrupt are :
        (++) DAL_IRDA_Transmit_IT()
        (++) DAL_IRDA_Receive_IT()
        (++) DAL_IRDA_IRQHandler()

    (#) Non Blocking mode functions with DMA are :
        (++) DAL_IRDA_Transmit_DMA()
        (++) DAL_IRDA_Receive_DMA()
        (++) DAL_IRDA_DMAPause()
        (++) DAL_IRDA_DMAResume()
        (++) DAL_IRDA_DMAStop()

    (#) A set of Transfer Complete Callbacks are provided in Non Blocking mode:
        (++) DAL_IRDA_TxHalfCpltCallback()
        (++) DAL_IRDA_TxCpltCallback()
        (++) DAL_IRDA_RxHalfCpltCallback()
        (++) DAL_IRDA_RxCpltCallback()
        (++) DAL_IRDA_ErrorCallback()

    (#) Non-Blocking mode transfers could be aborted using Abort API's :
        (+) DAL_IRDA_Abort()
        (+) DAL_IRDA_AbortTransmit()
        (+) DAL_IRDA_AbortReceive()
        (+) DAL_IRDA_Abort_IT()
        (+) DAL_IRDA_AbortTransmit_IT()
        (+) DAL_IRDA_AbortReceive_IT()

    (#) For Abort services based on interrupts (DAL_IRDA_Abortxxx_IT), a set of Abort Complete Callbacks are provided:
        (+) DAL_IRDA_AbortCpltCallback()
        (+) DAL_IRDA_AbortTransmitCpltCallback()
        (+) DAL_IRDA_AbortReceiveCpltCallback()

    (#) In Non-Blocking mode transfers, possible errors are split into 2 categories.
        Errors are handled as follows :
        (+) Error is considered as Recoverable and non blocking : Transfer could go till end, but error severity is
            to be evaluated by user : this concerns Frame Error, Parity Error or Noise Error in Interrupt mode reception .
            Received character is then retrieved and stored in Rx buffer, Error code is set to allow user to identify error type,
            and DAL_IRDA_ErrorCallback() user callback is executed. Transfer is kept ongoing on IRDA side.
            If user wants to abort it, Abort services should be called by user.
        (+) Error is considered as Blocking : Transfer could not be completed properly and is aborted.
            This concerns Overrun Error In Interrupt mode reception and all errors in DMA mode.
            Error code is set to allow user to identify error type, and DAL_IRDA_ErrorCallback() user callback is executed.

@endverbatim
  * @{
  */

/**
  * @brief Sends an amount of data in blocking mode.
  * @note  When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *        the sent data is handled as a set of u16. In this case, Size must reflect the number
  *        of u16 available through pData.
  * @param hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *              the configuration information for the specified IRDA module.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be sent.
  * @param Timeout Specify timeout value.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_Transmit(IRDA_HandleTypeDef *hirda, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  const uint16_t *tmp;
  uint32_t tickstart = 0U;

  /* Check that a Tx process is not already ongoing */
  if (hirda->gState == DAL_IRDA_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hirda);

    hirda->ErrorCode = DAL_IRDA_ERROR_NONE;
    hirda->gState = DAL_IRDA_STATE_BUSY_TX;

    /* Init tickstart for timeout management*/
    tickstart = DAL_GetTick();

    hirda->TxXferSize = Size;
    hirda->TxXferCount = Size;
    while (hirda->TxXferCount > 0U)
    {
      hirda->TxXferCount--;
      if (hirda->Init.WordLength == IRDA_WORDLENGTH_9B)
      {
        if (IRDA_WaitOnFlagUntilTimeout(hirda, IRDA_FLAG_TXE, RESET, tickstart, Timeout) != DAL_OK)
        {
          return DAL_TIMEOUT;
        }
        tmp = (const uint16_t *) pData;
        hirda->Instance->DATA = (*tmp & (uint16_t)0x01FF);
        if (hirda->Init.Parity == IRDA_PARITY_NONE)
        {
          pData += 2U;
        }
        else
        {
          pData += 1U;
        }
      }
      else
      {
        if (IRDA_WaitOnFlagUntilTimeout(hirda, IRDA_FLAG_TXE, RESET, tickstart, Timeout) != DAL_OK)
        {
          return DAL_TIMEOUT;
        }
        hirda->Instance->DATA = (*pData++ & (uint8_t)0xFF);
      }
    }

    if (IRDA_WaitOnFlagUntilTimeout(hirda, IRDA_FLAG_TC, RESET, tickstart, Timeout) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    /* At end of Tx process, restore hirda->gState to Ready */
    hirda->gState = DAL_IRDA_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hirda);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data in blocking mode.
  * @note  When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *        the received data is handled as a set of u16. In this case, Size must reflect the number
  *        of u16 available through pData.
  * @param hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *              the configuration information for the specified IRDA module.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be received.
  * @param Timeout Specify timeout value
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_Receive(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint16_t *tmp;
  uint32_t tickstart = 0U;

  /* Check that a Rx process is not already ongoing */
  if (hirda->RxState == DAL_IRDA_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hirda);

    hirda->ErrorCode = DAL_IRDA_ERROR_NONE;
    hirda->RxState = DAL_IRDA_STATE_BUSY_RX;

    /* Init tickstart for timeout management*/
    tickstart = DAL_GetTick();

    hirda->RxXferSize = Size;
    hirda->RxXferCount = Size;

    /* Check the remain data to be received */
    while (hirda->RxXferCount > 0U)
    {
      hirda->RxXferCount--;

      if (hirda->Init.WordLength == IRDA_WORDLENGTH_9B)
      {
        if (IRDA_WaitOnFlagUntilTimeout(hirda, IRDA_FLAG_RXNE, RESET, tickstart, Timeout) != DAL_OK)
        {
          return DAL_TIMEOUT;
        }
        tmp = (uint16_t *) pData ;
        if (hirda->Init.Parity == IRDA_PARITY_NONE)
        {
          *tmp = (uint16_t)(hirda->Instance->DATA & (uint16_t)0x01FF);
          pData += 2U;
        }
        else
        {
          *tmp = (uint16_t)(hirda->Instance->DATA & (uint16_t)0x00FF);
          pData += 1U;
        }
      }
      else
      {
        if (IRDA_WaitOnFlagUntilTimeout(hirda, IRDA_FLAG_RXNE, RESET, tickstart, Timeout) != DAL_OK)
        {
          return DAL_TIMEOUT;
        }
        if (hirda->Init.Parity == IRDA_PARITY_NONE)
        {
          *pData++ = (uint8_t)(hirda->Instance->DATA & (uint8_t)0x00FF);
        }
        else
        {
          *pData++ = (uint8_t)(hirda->Instance->DATA & (uint8_t)0x007F);
        }
      }
    }

    /* At end of Rx process, restore hirda->RxState to Ready */
    hirda->RxState = DAL_IRDA_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hirda);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Send an amount of data in non blocking mode.
  * @note  When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *        the sent data is handled as a set of u16. In this case, Size must reflect the number
  *        of u16 available through pData.
  * @param hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *              the configuration information for the specified IRDA module.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be sent.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_Transmit_IT(IRDA_HandleTypeDef *hirda, const uint8_t *pData, uint16_t Size)
{
  /* Check that a Tx process is not already ongoing */
  if (hirda->gState == DAL_IRDA_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hirda);

    hirda->pTxBuffPtr = pData;
    hirda->TxXferSize = Size;
    hirda->TxXferCount = Size;

    hirda->ErrorCode = DAL_IRDA_ERROR_NONE;
    hirda->gState = DAL_IRDA_STATE_BUSY_TX;

    /* Process Unlocked */
    __DAL_UNLOCK(hirda);

    /* Enable the IRDA Transmit Data Register Empty Interrupt */
    SET_BIT(hirda->Instance->CTRL1, USART_CTRL1_TXBEIEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data in non blocking mode.
  * @note  When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *        the received data is handled as a set of u16. In this case, Size must reflect the number
  *        of u16 available through pData.
  * @param hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *              the configuration information for the specified IRDA module.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be received.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_Receive_IT(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size)
{
  /* Check that a Rx process is not already ongoing */
  if (hirda->RxState == DAL_IRDA_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hirda);

    hirda->pRxBuffPtr = pData;
    hirda->RxXferSize = Size;
    hirda->RxXferCount = Size;

    hirda->ErrorCode = DAL_IRDA_ERROR_NONE;
    hirda->RxState = DAL_IRDA_STATE_BUSY_RX;

    /* Process Unlocked */
    __DAL_UNLOCK(hirda);

    if (hirda->Init.Parity != IRDA_PARITY_NONE)
    {
      /* Enable the IRDA Parity Error and Data Register Not Empty Interrupts */
      SET_BIT(hirda->Instance->CTRL1, USART_CTRL1_PEIEN | USART_CTRL1_RXBNEIEN);
    }
    else
    {
      /* Enable the IRDA Data Register Not Empty Interrupts */
       SET_BIT(hirda->Instance->CTRL1, USART_CTRL1_RXBNEIEN); 
    }

    /* Enable the IRDA Error Interrupt: (Frame error, Noise error, Overrun error) */
    SET_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Send an amount of data in DMA mode.
  * @note  When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *        the sent data is handled as a set of u16. In this case, Size must reflect the number
  *        of u16 available through pData.
  * @param hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *              the configuration information for the specified IRDA module.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be sent.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_Transmit_DMA(IRDA_HandleTypeDef *hirda, const uint8_t *pData, uint16_t Size)
{
  const uint32_t *tmp;

  /* Check that a Tx process is not already ongoing */
  if (hirda->gState == DAL_IRDA_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hirda);

    hirda->pTxBuffPtr = pData;
    hirda->TxXferSize = Size;
    hirda->TxXferCount = Size;

    hirda->ErrorCode = DAL_IRDA_ERROR_NONE;
    hirda->gState = DAL_IRDA_STATE_BUSY_TX;

    /* Set the IRDA DMA transfer complete callback */
    hirda->hdmatx->XferCpltCallback = IRDA_DMATransmitCplt;

    /* Set the IRDA DMA half transfer complete callback */
    hirda->hdmatx->XferHalfCpltCallback = IRDA_DMATransmitHalfCplt;

    /* Set the DMA error callback */
    hirda->hdmatx->XferErrorCallback = IRDA_DMAError;

    /* Set the DMA abort callback */
    hirda->hdmatx->XferAbortCallback = NULL;

    /* Enable the IRDA transmit DMA stream */
    tmp = (const uint32_t *)&pData;
    DAL_DMA_Start_IT(hirda->hdmatx, *(const uint32_t *)tmp, (uint32_t)&hirda->Instance->DATA, Size);

    /* Clear the TC flag in the SR register by writing 0 to it */
    __DAL_IRDA_CLEAR_FLAG(hirda, IRDA_FLAG_TC);

    /* Process Unlocked */
    __DAL_UNLOCK(hirda);

    /* Enable the DMA transfer for transmit request by setting the DMAT bit
    in the USART CTRL3 register */
    SET_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Receives an amount of data in DMA mode.
  * @note  When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *        the received data is handled as a set of u16. In this case, Size must reflect the number
  *        of u16 available through pData.
  * @param hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *              the configuration information for the specified IRDA module.
  * @param pData Pointer to data buffer (u8 or u16 data elements).
  * @param Size  Amount of data elements (u8 or u16) to be received.
  * @note   When the IRDA parity is enabled (PCE = 1) the data received contain the parity bit.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_Receive_DMA(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size)
{
  uint32_t *tmp;

  /* Check that a Rx process is not already ongoing */
  if (hirda->RxState == DAL_IRDA_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hirda);

    hirda->pRxBuffPtr = pData;
    hirda->RxXferSize = Size;

    hirda->ErrorCode = DAL_IRDA_ERROR_NONE;
    hirda->RxState = DAL_IRDA_STATE_BUSY_RX;

    /* Set the IRDA DMA transfer complete callback */
    hirda->hdmarx->XferCpltCallback = IRDA_DMAReceiveCplt;

    /* Set the IRDA DMA half transfer complete callback */
    hirda->hdmarx->XferHalfCpltCallback = IRDA_DMAReceiveHalfCplt;

    /* Set the DMA error callback */
    hirda->hdmarx->XferErrorCallback = IRDA_DMAError;

    /* Set the DMA abort callback */
    hirda->hdmarx->XferAbortCallback = NULL;

    /* Enable the DMA stream */
    tmp = (uint32_t *)&pData;
    DAL_DMA_Start_IT(hirda->hdmarx, (uint32_t)&hirda->Instance->DATA, *(uint32_t *)tmp, Size);

    /* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
    __DAL_IRDA_CLEAR_OREFLAG(hirda);

    /* Process Unlocked */
    __DAL_UNLOCK(hirda);

    if (hirda->Init.Parity != IRDA_PARITY_NONE)
    {
      /* Enable the IRDA Parity Error Interrupt */
      SET_BIT(hirda->Instance->CTRL1, USART_CTRL1_PEIEN);
    }

    /* Enable the IRDA Error Interrupt: (Frame error, Noise error, Overrun error) */
    SET_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the USART CTRL3 register */
    SET_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Pauses the DMA Transfer.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_DMAPause(IRDA_HandleTypeDef *hirda)
{
  uint32_t dmarequest = 0x00U;

  /* Process Locked */
  __DAL_LOCK(hirda);

  dmarequest = DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if ((hirda->gState == DAL_IRDA_STATE_BUSY_TX) && dmarequest)
  {
    /* Disable the IRDA DMA Tx request */
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);
  }

  dmarequest = DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if ((hirda->RxState == DAL_IRDA_STATE_BUSY_RX) && dmarequest)
  {
    /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
    CLEAR_BIT(hirda->Instance->CTRL1, USART_CTRL1_PEIEN);
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Disable the IRDA DMA Rx request */
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);
  }

  /* Process Unlocked */
  __DAL_UNLOCK(hirda);

  return DAL_OK;
}

/**
  * @brief Resumes the DMA Transfer.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_DMAResume(IRDA_HandleTypeDef *hirda)
{
  /* Process Locked */
  __DAL_LOCK(hirda);

  if (hirda->gState == DAL_IRDA_STATE_BUSY_TX)
  {
    /* Enable the IRDA DMA Tx request */
    SET_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);
  }

  if (hirda->RxState == DAL_IRDA_STATE_BUSY_RX)
  {
    /* Clear the Overrun flag before resuming the Rx transfer */
    __DAL_IRDA_CLEAR_OREFLAG(hirda);

    /* Re-enable PE and ERR (Frame error, noise error, overrun error) interrupts */
    if (hirda->Init.Parity != IRDA_PARITY_NONE)
    {    
      SET_BIT(hirda->Instance->CTRL1, USART_CTRL1_PEIEN);
    }
    SET_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Enable the IRDA DMA Rx request */
    SET_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);
  }

  /* Process Unlocked */
  __DAL_UNLOCK(hirda);

  return DAL_OK;
}

/**
  * @brief Stops the DMA Transfer.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_IRDA_DMAStop(IRDA_HandleTypeDef *hirda)
{
  uint32_t dmarequest = 0x00U;
  /* The Lock is not implemented on this API to allow the user application
     to call the DAL IRDA API under callbacks DAL_IRDA_TxCpltCallback() / DAL_IRDA_RxCpltCallback():
     when calling DAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
     and the correspond call back is executed DAL_IRDA_TxCpltCallback() / DAL_IRDA_RxCpltCallback()
  */

  /* Stop IRDA DMA Tx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if ((hirda->gState == DAL_IRDA_STATE_BUSY_TX) && dmarequest)
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the IRDA DMA Tx channel */
    if (hirda->hdmatx != NULL)
    {
      DAL_DMA_Abort(hirda->hdmatx);
    }
    IRDA_EndTxTransfer(hirda);
  }

  /* Stop IRDA DMA Rx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if ((hirda->RxState == DAL_IRDA_STATE_BUSY_RX) && dmarequest)
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the IRDA DMA Rx channel */
    if (hirda->hdmarx != NULL)
    {
      DAL_DMA_Abort(hirda->hdmarx);
    }
    IRDA_EndRxTransfer(hirda);
  }

  return DAL_OK;
}

/**
  * @brief  Abort ongoing transfers (blocking mode).
  * @param  hirda IRDA handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_IRDA_Abort(IRDA_HandleTypeDef *hirda)
{
  /* Disable TXEIE, TCIE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
  CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Disable the IRDA DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the IRDA DMA Tx channel : use blocking DMA Abort API (no callback) */
    if (hirda->hdmatx != NULL)
    {
      /* Set the IRDA DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hirda->hdmatx->XferAbortCallback = NULL;

      DAL_DMA_Abort(hirda->hdmatx);
    }
  }

  /* Disable the IRDA DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the IRDA DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (hirda->hdmarx != NULL)
    {
      /* Set the IRDA DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hirda->hdmarx->XferAbortCallback = NULL;

      DAL_DMA_Abort(hirda->hdmarx);
    }
  }

  /* Reset Tx and Rx transfer counters */
  hirda->TxXferCount = 0x00U;
  hirda->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  hirda->ErrorCode = DAL_IRDA_ERROR_NONE;

  /* Restore hirda->RxState and hirda->gState to Ready */
  hirda->RxState = DAL_IRDA_STATE_READY;
  hirda->gState = DAL_IRDA_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (blocking mode).
  * @param  hirda IRDA handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_IRDA_AbortTransmit(IRDA_HandleTypeDef *hirda)
{
  /* Disable TXEIE and TCIE interrupts */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* Disable the IRDA DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the IRDA DMA Tx channel : use blocking DMA Abort API (no callback) */
    if (hirda->hdmatx != NULL)
    {
      /* Set the IRDA DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hirda->hdmatx->XferAbortCallback = NULL;

      DAL_DMA_Abort(hirda->hdmatx);
    }
  }

  /* Reset Tx transfer counter */
  hirda->TxXferCount = 0x00U;

  /* Restore hirda->gState to Ready */
  hirda->gState = DAL_IRDA_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (blocking mode).
  * @param  hirda IRDA handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable PPP Interrupts
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_IRDA_AbortReceive(IRDA_HandleTypeDef *hirda)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Disable the IRDA DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the IRDA DMA Rx channel : use blocking DMA Abort API (no callback) */
    if (hirda->hdmarx != NULL)
    {
      /* Set the IRDA DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      hirda->hdmarx->XferAbortCallback = NULL;

      DAL_DMA_Abort(hirda->hdmarx);
    }
  }

  /* Reset Rx transfer counter */
  hirda->RxXferCount = 0x00U;

  /* Restore hirda->RxState to Ready */
  hirda->RxState = DAL_IRDA_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing transfers (Interrupt mode).
  * @param  hirda IRDA handle.
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
DAL_StatusTypeDef DAL_IRDA_Abort_IT(IRDA_HandleTypeDef *hirda)
{
  uint32_t AbortCplt = 0x01U;

  /* Disable TXEIE, TCIE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
  CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* If DMA Tx and/or DMA Rx Handles are associated to IRDA Handle, DMA Abort complete callbacks should be initialised
     before any call to DMA Abort functions */
  /* DMA Tx Handle is valid */
  if (hirda->hdmatx != NULL)
  {
    /* Set DMA Abort Complete callback if IRDA DMA Tx request if enabled.
       Otherwise, set it to NULL */
    if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN))
    {
      hirda->hdmatx->XferAbortCallback = IRDA_DMATxAbortCallback;
    }
    else
    {
      hirda->hdmatx->XferAbortCallback = NULL;
    }
  }
  /* DMA Rx Handle is valid */
  if (hirda->hdmarx != NULL)
  {
    /* Set DMA Abort Complete callback if IRDA DMA Rx request if enabled.
       Otherwise, set it to NULL */
    if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN))
    {
      hirda->hdmarx->XferAbortCallback = IRDA_DMARxAbortCallback;
    }
    else
    {
      hirda->hdmarx->XferAbortCallback = NULL;
    }
  }

  /* Disable the IRDA DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    /* Disable DMA Tx at IRDA level */
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the IRDA DMA Tx channel : use non blocking DMA Abort API (callback) */
    if (hirda->hdmatx != NULL)
    {
      /* IRDA Tx DMA Abort callback has already been initialised :
         will lead to call DAL_IRDA_AbortCpltCallback() at end of DMA abort procedure */

      /* Abort DMA TX */
      if (DAL_DMA_Abort_IT(hirda->hdmatx) != DAL_OK)
      {
        hirda->hdmatx->XferAbortCallback = NULL;
      }
      else
      {
        AbortCplt = 0x00U;
      }
    }
  }

  /* Disable the IRDA DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the IRDA DMA Rx channel : use non blocking DMA Abort API (callback) */
    if (hirda->hdmarx != NULL)
    {
      /* IRDA Rx DMA Abort callback has already been initialised :
         will lead to call DAL_IRDA_AbortCpltCallback() at end of DMA abort procedure */

      /* Abort DMA RX */
      if (DAL_DMA_Abort_IT(hirda->hdmarx) != DAL_OK)
      {
        hirda->hdmarx->XferAbortCallback = NULL;
        AbortCplt = 0x01U;
      }
      else
      {
        AbortCplt = 0x00U;
      }
    }
  }

  /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
  if (AbortCplt == 0x01U)
  {
    /* Reset Tx and Rx transfer counters */
    hirda->TxXferCount = 0x00U;
    hirda->RxXferCount = 0x00U;

    /* Reset ErrorCode */
    hirda->ErrorCode = DAL_IRDA_ERROR_NONE;

    /* Restore hirda->gState and hirda->RxState to Ready */
    hirda->gState  = DAL_IRDA_STATE_READY;
    hirda->RxState = DAL_IRDA_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
    /* Call registered Abort complete callback */
    hirda->AbortCpltCallback(hirda);
#else
    /* Call legacy weak Abort complete callback */
    DAL_IRDA_AbortCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
  }

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (Interrupt mode).
  * @param  hirda IRDA handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable IRDA Interrupts (Tx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
*/
DAL_StatusTypeDef DAL_IRDA_AbortTransmit_IT(IRDA_HandleTypeDef *hirda)
{
  /* Disable TXEIE and TCIE interrupts */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* Disable the IRDA DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the IRDA DMA Tx channel : use non blocking DMA Abort API (callback) */
    if (hirda->hdmatx != NULL)
    {
      /* Set the IRDA DMA Abort callback :
         will lead to call DAL_IRDA_AbortCpltCallback() at end of DMA abort procedure */
      hirda->hdmatx->XferAbortCallback = IRDA_DMATxOnlyAbortCallback;

      /* Abort DMA TX */
      if (DAL_DMA_Abort_IT(hirda->hdmatx) != DAL_OK)
      {
        /* Call Directly hirda->hdmatx->XferAbortCallback function in case of error */
        hirda->hdmatx->XferAbortCallback(hirda->hdmatx);
      }
    }
    else
    {
      /* Reset Tx transfer counter */
      hirda->TxXferCount = 0x00U;

      /* Restore hirda->gState to Ready */
      hirda->gState = DAL_IRDA_STATE_READY;

      /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
      /* Call registered Abort Transmit Complete Callback */
      hirda->AbortTransmitCpltCallback(hirda);
#else
      /* Call legacy weak Abort Transmit Complete Callback */
      DAL_IRDA_AbortTransmitCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
    }
  }
  else
  {
    /* Reset Tx transfer counter */
    hirda->TxXferCount = 0x00U;

    /* Restore hirda->gState to Ready */
    hirda->gState = DAL_IRDA_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
    /* Call registered Abort Transmit Complete Callback */
    hirda->AbortTransmitCpltCallback(hirda);
#else
    /* Call legacy weak Abort Transmit Complete Callback */
    DAL_IRDA_AbortTransmitCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
  }

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (Interrupt mode).
  * @param  hirda IRDA handle.
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
DAL_StatusTypeDef DAL_IRDA_AbortReceive_IT(IRDA_HandleTypeDef *hirda)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Disable the IRDA DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the IRDA DMA Rx channel : use non blocking DMA Abort API (callback) */
    if (hirda->hdmarx != NULL)
    {
      /* Set the IRDA DMA Abort callback :
         will lead to call DAL_IRDA_AbortCpltCallback() at end of DMA abort procedure */
      hirda->hdmarx->XferAbortCallback = IRDA_DMARxOnlyAbortCallback;

      /* Abort DMA RX */
      if (DAL_DMA_Abort_IT(hirda->hdmarx) != DAL_OK)
      {
        /* Call Directly hirda->hdmarx->XferAbortCallback function in case of error */
        hirda->hdmarx->XferAbortCallback(hirda->hdmarx);
      }
    }
    else
    {
      /* Reset Rx transfer counter */
      hirda->RxXferCount = 0x00U;

      /* Restore hirda->RxState to Ready */
      hirda->RxState = DAL_IRDA_STATE_READY;

      /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
      /* Call registered Abort Receive Complete Callback */
      hirda->AbortReceiveCpltCallback(hirda);
#else
      /* Call legacy weak Abort Receive Complete Callback */
      DAL_IRDA_AbortReceiveCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
    }
  }
  else
  {
    /* Reset Rx transfer counter */
    hirda->RxXferCount = 0x00U;

    /* Restore hirda->RxState to Ready */
    hirda->RxState = DAL_IRDA_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
    /* Call registered Abort Receive Complete Callback */
    hirda->AbortReceiveCpltCallback(hirda);
#else
    /* Call legacy weak Abort Receive Complete Callback */
    DAL_IRDA_AbortReceiveCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
  }

  return DAL_OK;
}

/**
  * @brief  This function handles IRDA interrupt request.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
void DAL_IRDA_IRQHandler(IRDA_HandleTypeDef *hirda)
{
  uint32_t isrflags   = READ_REG(hirda->Instance->STS);
  uint32_t cr1its     = READ_REG(hirda->Instance->CTRL1);
  uint32_t cr3its     = READ_REG(hirda->Instance->CTRL3);
  uint32_t errorflags = 0x00U;
  uint32_t dmarequest = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_STS_PEFLG | USART_STS_FEFLG | USART_STS_OVREFLG | USART_STS_NEFLG));
  if (errorflags == RESET)
  {
    /* IRDA in mode Receiver -----------------------------------------------*/
    if (((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))
    {
      IRDA_Receive_IT(hirda);
      return;
    }
  }

  /* If some errors occur */
  if ((errorflags != RESET) && (((cr3its & USART_CTRL3_ERRIEN) != RESET) || ((cr1its & (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN)) != RESET)))
  {
    /* IRDA parity error interrupt occurred -------------------------------*/
    if (((isrflags & USART_STS_PEFLG) != RESET) && ((cr1its & USART_CTRL1_PEIEN) != RESET))
    {
      hirda->ErrorCode |= DAL_IRDA_ERROR_PE;
    }

    /* IRDA noise error interrupt occurred --------------------------------*/
    if (((isrflags & USART_STS_NEFLG) != RESET) && ((cr3its & USART_CTRL3_ERRIEN) != RESET))
    {
      hirda->ErrorCode |= DAL_IRDA_ERROR_NE;
    }

    /* IRDA frame error interrupt occurred --------------------------------*/
    if (((isrflags & USART_STS_FEFLG) != RESET) && ((cr3its & USART_CTRL3_ERRIEN) != RESET))
    {
      hirda->ErrorCode |= DAL_IRDA_ERROR_FE;
    }

    /* IRDA Over-Run interrupt occurred -----------------------------------*/
    if (((isrflags & USART_STS_OVREFLG) != RESET) && (((cr1its & USART_CTRL1_RXBNEIEN) != RESET) || ((cr3its & USART_CTRL3_ERRIEN) != RESET)))
    {
      hirda->ErrorCode |= DAL_IRDA_ERROR_ORE;
    }
    /* Call IRDA Error Call back function if need be -----------------------*/
    if (hirda->ErrorCode != DAL_IRDA_ERROR_NONE)
    {
      /* IRDA in mode Receiver ---------------------------------------------*/
      if (((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))
      {
        IRDA_Receive_IT(hirda);
      }

      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
         consider error as blocking */
      dmarequest = DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);
      if (((hirda->ErrorCode & DAL_IRDA_ERROR_ORE) != RESET) || dmarequest)
      {
        /* Blocking error : transfer is aborted
           Set the IRDA state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        IRDA_EndRxTransfer(hirda);

        /* Disable the IRDA DMA Rx request if enabled */
        if (DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN))
        {
          CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);

          /* Abort the IRDA DMA Rx channel */
          if (hirda->hdmarx != NULL)
          {
            /* Set the IRDA DMA Abort callback :
            will lead to call DAL_IRDA_ErrorCallback() at end of DMA abort procedure */
            hirda->hdmarx->XferAbortCallback = IRDA_DMAAbortOnError;

            /* Abort DMA RX */
            if (DAL_DMA_Abort_IT(hirda->hdmarx) != DAL_OK)
            {
              /* Call Directly XferAbortCallback function in case of error */
              hirda->hdmarx->XferAbortCallback(hirda->hdmarx);
            }
          }
          else
          {
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
            /* Call registered user error callback */
            hirda->ErrorCallback(hirda);
#else
            /* Call legacy weak user error callback */
            DAL_IRDA_ErrorCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
          }
        }
        else
        {
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
          /* Call registered user error callback */
          hirda->ErrorCallback(hirda);
#else
          /* Call legacy weak user error callback */
          DAL_IRDA_ErrorCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
        }
      }
      else
      {
        /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
        /* Call registered user error callback */
        hirda->ErrorCallback(hirda);
#else
        /* Call legacy weak user error callback */
        DAL_IRDA_ErrorCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */

        hirda->ErrorCode = DAL_IRDA_ERROR_NONE;
      }
    }
    return;
  } /* End if some error occurs */

  /* IRDA in mode Transmitter ------------------------------------------------*/
  if (((isrflags & USART_STS_TXBEFLG) != RESET) && ((cr1its & USART_CTRL1_TXBEIEN) != RESET))
  {
    IRDA_Transmit_IT(hirda);
    return;
  }

  /* IRDA in mode Transmitter end --------------------------------------------*/
  if (((isrflags & USART_STS_TXCFLG) != RESET) && ((cr1its & USART_CTRL1_TXCIEN) != RESET))
  {
    IRDA_EndTransmit_IT(hirda);
    return;
  }
}

/**
  * @brief  Tx Transfer complete callback.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_TxCpltCallback(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_IRDA_TxCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  Tx Half Transfer completed callback.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void DAL_IRDA_TxHalfCpltCallback(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_IRDA_TxHalfCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  Rx Transfer complete callback.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_IRDA_RxCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  Rx Half Transfer complete callback.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_RxHalfCpltCallback(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_IRDA_RxHalfCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  IRDA error callback.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_ErrorCallback(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_IRDA_ErrorCallback can be implemented in the user file.
   */
}

/**
  * @brief  IRDA Abort Complete callback.
  * @param  hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_AbortCpltCallback(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_IRDA_AbortCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  IRDA Abort Transmit Complete callback.
  * @param  hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_AbortTransmitCpltCallback(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_IRDA_AbortTransmitCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  IRDA Abort Receive Complete callback.
  * @param  hirda Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
__weak void DAL_IRDA_AbortReceiveCpltCallback(IRDA_HandleTypeDef *hirda)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hirda);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_IRDA_AbortReceiveCpltCallback can be implemented in the user file.
   */
}

/**
  * @}
  */

/** @defgroup IRDA_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   IRDA State and Errors functions
  *
@verbatim
  ==============================================================================
                  ##### Peripheral State and Errors functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to return the State of IrDA
    communication process and also return Peripheral Errors occurred during communication process
     (+) DAL_IRDA_GetState() API can be helpful to check in run-time the state of the IrDA peripheral.
     (+) DAL_IRDA_GetError() check in run-time errors that could be occurred during communication.

@endverbatim
  * @{
  */

/**
  * @brief  Return the IRDA state.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA.
  * @retval DAL state
  */
DAL_IRDA_StateTypeDef DAL_IRDA_GetState(IRDA_HandleTypeDef *hirda)
{
  uint32_t temp1 = 0x00U, temp2 = 0x00U;
  temp1 = hirda->gState;
  temp2 = hirda->RxState;

  return (DAL_IRDA_StateTypeDef)(temp1 | temp2);
}

/**
  * @brief  Return the IRDA error code
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *              the configuration information for the specified IRDA.
  * @retval IRDA Error Code
  */
uint32_t DAL_IRDA_GetError(IRDA_HandleTypeDef *hirda)
{
  return hirda->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup IRDA_Private_Functions IRDA Private Functions
  * @{
  */

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
/**
  * @brief  Initialize the callbacks to their default values.
  * @param  hirda IRDA handle.
  * @retval none
  */
void IRDA_InitCallbacksToDefault(IRDA_HandleTypeDef *hirda)
{
  /* Init the IRDA Callback settings */
  hirda->TxHalfCpltCallback        = DAL_IRDA_TxHalfCpltCallback;        /* Legacy weak TxHalfCpltCallback        */
  hirda->TxCpltCallback            = DAL_IRDA_TxCpltCallback;            /* Legacy weak TxCpltCallback            */
  hirda->RxHalfCpltCallback        = DAL_IRDA_RxHalfCpltCallback;        /* Legacy weak RxHalfCpltCallback        */
  hirda->RxCpltCallback            = DAL_IRDA_RxCpltCallback;            /* Legacy weak RxCpltCallback            */
  hirda->ErrorCallback             = DAL_IRDA_ErrorCallback;             /* Legacy weak ErrorCallback             */
  hirda->AbortCpltCallback         = DAL_IRDA_AbortCpltCallback;         /* Legacy weak AbortCpltCallback         */
  hirda->AbortTransmitCpltCallback = DAL_IRDA_AbortTransmitCpltCallback; /* Legacy weak AbortTransmitCpltCallback */
  hirda->AbortReceiveCpltCallback  = DAL_IRDA_AbortReceiveCpltCallback;  /* Legacy weak AbortReceiveCpltCallback  */

}
#endif /* USE_DAL_IRDA_REGISTER_CALLBACKS */

/**
  * @brief  DMA IRDA transmit process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA.
  * @retval None
  */
static void IRDA_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal mode */
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == 0U)
  {
    hirda->TxXferCount = 0U;

    /* Disable the DMA transfer for transmit request by resetting the DMAT bit
       in the IRDA CTRL3 register */
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Enable the IRDA Transmit Complete Interrupt */
    SET_BIT(hirda->Instance->CTRL1, USART_CTRL1_TXCIEN);
  }
  /* DMA Circular mode */
  else
  {
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
    /* Call registered Tx complete callback */
    hirda->TxCpltCallback(hirda);
#else
    /* Call legacy weak Tx complete callback */
    DAL_IRDA_TxCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
  }
}

/**
  * @brief DMA IRDA receive process half complete callback
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA.
  * @retval None
  */
static void IRDA_DMATransmitHalfCplt(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered Tx Half complete callback */
  hirda->TxHalfCpltCallback(hirda);
#else
  /* Call legacy weak Tx complete callback */
  DAL_IRDA_TxHalfCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
}

/**
  * @brief  DMA IRDA receive process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA.
  * @retval None
  */
static void IRDA_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* DMA Normal mode */
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == 0U)
  {
    hirda->RxXferCount = 0U;

    /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
    CLEAR_BIT(hirda->Instance->CTRL1, USART_CTRL1_PEIEN);
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
       in the IRDA CTRL3 register */
    CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* At end of Rx process, restore hirda->RxState to Ready */
    hirda->RxState = DAL_IRDA_STATE_READY;
  }

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered Rx complete callback */
  hirda->RxCpltCallback(hirda);
#else
  /* Call legacy weak Rx complete callback */
  DAL_IRDA_RxCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACKS */
}

/**
  * @brief DMA IRDA receive process half complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA.
  * @retval None
  */
static void IRDA_DMAReceiveHalfCplt(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /*Call registered Rx Half complete callback*/
  hirda->RxHalfCpltCallback(hirda);
#else
  /* Call legacy weak Rx Half complete callback */
  DAL_IRDA_RxHalfCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
}

/**
  * @brief  DMA IRDA communication error callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA.
  * @retval None
  */
static void IRDA_DMAError(DMA_HandleTypeDef *hdma)
{
  uint32_t dmarequest = 0x00U;
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Stop IRDA DMA Tx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if ((hirda->gState == DAL_IRDA_STATE_BUSY_TX) && dmarequest)
  {
    hirda->TxXferCount = 0U;
    IRDA_EndTxTransfer(hirda);
  }

  /* Stop IRDA DMA Rx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(hirda->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if ((hirda->RxState == DAL_IRDA_STATE_BUSY_RX) && dmarequest)
  {
    hirda->RxXferCount = 0U;
    IRDA_EndRxTransfer(hirda);
  }

  hirda->ErrorCode |= DAL_IRDA_ERROR_DMA;

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered user error callback */
  hirda->ErrorCallback(hirda);
#else
  /* Call legacy weak user error callback */
  DAL_IRDA_ErrorCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
}

/**
  * @brief  This function handles IRDA Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA.
  * @param  Flag specifies the IRDA flag to check.
  * @param  Status The actual Flag status (SET or RESET).
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
static DAL_StatusTypeDef IRDA_WaitOnFlagUntilTimeout(IRDA_HandleTypeDef *hirda, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while ((__DAL_IRDA_GET_FLAG(hirda, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != DAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((DAL_GetTick() - Tickstart) > Timeout))
      {
        /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
        CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN));
        CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

        hirda->gState  = DAL_IRDA_STATE_READY;
        hirda->RxState = DAL_IRDA_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(hirda);

        return DAL_TIMEOUT;
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  End ongoing Tx transfer on IRDA peripheral (following error detection or Transmit completion).
  * @param  hirda IRDA handle.
  * @retval None
  */
static void IRDA_EndTxTransfer(IRDA_HandleTypeDef *hirda)
{
  /* Disable TXEIE and TCIE interrupts */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* At end of Tx process, restore hirda->gState to Ready */
  hirda->gState = DAL_IRDA_STATE_READY;
}

/**
  * @brief  End ongoing Rx transfer on IRDA peripheral (following error detection or Reception completion).
  * @param  hirda IRDA handle.
  * @retval None
  */
static void IRDA_EndRxTransfer(IRDA_HandleTypeDef *hirda)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* At end of Rx process, restore hirda->RxState to Ready */
  hirda->RxState = DAL_IRDA_STATE_READY;
}

/**
  * @brief  DMA IRDA communication abort callback, when initiated by DAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  * @retval None
  */
static void IRDA_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  hirda->RxXferCount = 0x00U;
  hirda->TxXferCount = 0x00U;

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered user error callback */
  hirda->ErrorCallback(hirda);
#else
  /* Call legacy weak user error callback */
  DAL_IRDA_ErrorCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
}

/**
  * @brief  DMA IRDA Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void IRDA_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  hirda->hdmatx->XferAbortCallback = NULL;

  /* Check if an Abort process is still ongoing */
  if (hirda->hdmarx != NULL)
  {
    if (hirda->hdmarx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hirda->TxXferCount = 0x00U;
  hirda->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  hirda->ErrorCode = DAL_IRDA_ERROR_NONE;

  /* Restore hirda->gState and hirda->RxState to Ready */
  hirda->gState  = DAL_IRDA_STATE_READY;
  hirda->RxState = DAL_IRDA_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered Abort complete callback */
  hirda->AbortCpltCallback(hirda);
#else
  /* Call legacy weak Abort complete callback */
  DAL_IRDA_AbortCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
}

/**
  * @brief  DMA IRDA Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void IRDA_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  hirda->hdmarx->XferAbortCallback = NULL;

  /* Check if an Abort process is still ongoing */
  if (hirda->hdmatx != NULL)
  {
    if (hirda->hdmatx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  hirda->TxXferCount = 0x00U;
  hirda->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  hirda->ErrorCode = DAL_IRDA_ERROR_NONE;

  /* Restore hirda->gState and hirda->RxState to Ready */
  hirda->gState  = DAL_IRDA_STATE_READY;
  hirda->RxState = DAL_IRDA_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered Abort complete callback */
  hirda->AbortCpltCallback(hirda);
#else
  /* Call legacy weak Abort complete callback */
  DAL_IRDA_AbortCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
}

/**
  * @brief  DMA IRDA Tx communication abort callback, when initiated by user by a call to
  *         DAL_IRDA_AbortTransmit_IT API (Abort only Tx transfer)
  *         (This callback is executed at end of DMA Tx Abort procedure following user abort request,
  *         and leads to user Tx Abort Complete callback execution).
  * @param  hdma DMA handle.
  * @retval None
  */
static void IRDA_DMATxOnlyAbortCallback(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  hirda->TxXferCount = 0x00U;

  /* Restore hirda->gState to Ready */
  hirda->gState = DAL_IRDA_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Transmit Complete Callback */
  hirda->AbortTransmitCpltCallback(hirda);
#else
  /* Call legacy weak Abort Transmit Complete Callback */
  DAL_IRDA_AbortTransmitCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
}

/**
  * @brief  DMA IRDA Rx communication abort callback, when initiated by user by a call to
  *         DAL_IRDA_AbortReceive_IT API (Abort only Rx transfer)
  *         (This callback is executed at end of DMA Rx Abort procedure following user abort request,
  *         and leads to user Rx Abort Complete callback execution).
  * @param  hdma DMA handle.
  * @retval None
  */
static void IRDA_DMARxOnlyAbortCallback(DMA_HandleTypeDef *hdma)
{
  IRDA_HandleTypeDef *hirda = (IRDA_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  hirda->RxXferCount = 0x00U;

  /* Restore hirda->RxState to Ready */
  hirda->RxState = DAL_IRDA_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Receive Complete Callback */
  hirda->AbortReceiveCpltCallback(hirda);
#else
  /* Call legacy weak Abort Receive Complete Callback */
  DAL_IRDA_AbortReceiveCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */
}

/**
 * @brief  Send an amount of data in non blocking mode.
 * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
 *                the configuration information for the specified IRDA module.
 * @retval DAL status
 */
static DAL_StatusTypeDef IRDA_Transmit_IT(IRDA_HandleTypeDef *hirda)
{
  const uint16_t *tmp;

  /* Check that a Tx process is ongoing */
  if (hirda->gState == DAL_IRDA_STATE_BUSY_TX)
  {
    if (hirda->Init.WordLength == IRDA_WORDLENGTH_9B)
    {
      tmp = (const uint16_t *) hirda->pTxBuffPtr;
      hirda->Instance->DATA = (uint16_t)(*tmp & (uint16_t)0x01FF);
      if (hirda->Init.Parity == IRDA_PARITY_NONE)
      {
        hirda->pTxBuffPtr += 2U;
      }
      else
      {
        hirda->pTxBuffPtr += 1U;
      }
    }
    else
    {
      hirda->Instance->DATA = (uint8_t)(*hirda->pTxBuffPtr++ & (uint8_t)0x00FF);
    }

    if (--hirda->TxXferCount == 0U)
    {
      /* Disable the IRDA Transmit Data Register Empty Interrupt */
      CLEAR_BIT(hirda->Instance->CTRL1, USART_CTRL1_TXBEIEN);

      /* Enable the IRDA Transmit Complete Interrupt */
      SET_BIT(hirda->Instance->CTRL1, USART_CTRL1_TXCIEN);
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
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval DAL status
  */
static DAL_StatusTypeDef IRDA_EndTransmit_IT(IRDA_HandleTypeDef *hirda)
{
  /* Disable the IRDA Transmit Complete Interrupt */
  CLEAR_BIT(hirda->Instance->CTRL1, USART_CTRL1_TXCIEN);

  /* Disable the IRDA Error Interrupt: (Frame error, noise error, overrun error) */
  CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Tx process is ended, restore hirda->gState to Ready */
  hirda->gState = DAL_IRDA_STATE_READY;

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
  /* Call registered Tx complete callback */
  hirda->TxCpltCallback(hirda);
#else
  /* Call legacy weak Tx complete callback */
  DAL_IRDA_TxCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACK */

  return DAL_OK;
}

/**
  * @brief  Receives an amount of data in non blocking mode.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval DAL status
  */
static DAL_StatusTypeDef IRDA_Receive_IT(IRDA_HandleTypeDef *hirda)
{
  uint16_t *tmp;
  uint16_t  uhdata;

  /* Check that a Rx process is ongoing */
  if (hirda->RxState == DAL_IRDA_STATE_BUSY_RX)
  {
    uhdata = (uint16_t) READ_REG(hirda->Instance->DATA);
    if (hirda->Init.WordLength == IRDA_WORDLENGTH_9B)
    {
      tmp = (uint16_t *) hirda->pRxBuffPtr;
      if (hirda->Init.Parity == IRDA_PARITY_NONE)
      {
        *tmp = (uint16_t)(uhdata & (uint16_t)0x01FF);
        hirda->pRxBuffPtr += 2U;
      }
      else
      {
        *tmp = (uint16_t)(uhdata & (uint16_t)0x00FF);
        hirda->pRxBuffPtr += 1U;
      }
    }
    else
    {
      if (hirda->Init.Parity == IRDA_PARITY_NONE)
      {
        *hirda->pRxBuffPtr++ = (uint8_t)(uhdata & (uint8_t)0x00FF);
      }
      else
      {
        *hirda->pRxBuffPtr++ = (uint8_t)(uhdata & (uint8_t)0x007F);
      }
    }

    if (--hirda->RxXferCount == 0U)
    {
      /* Disable the IRDA Data Register not empty Interrupt */
      CLEAR_BIT(hirda->Instance->CTRL1, USART_CTRL1_RXBNEIEN);

      /* Disable the IRDA Parity Error Interrupt */
      CLEAR_BIT(hirda->Instance->CTRL1, USART_CTRL1_PEIEN);

      /* Disable the IRDA Error Interrupt: (Frame error, noise error, overrun error) */
      CLEAR_BIT(hirda->Instance->CTRL3, USART_CTRL3_ERRIEN);

      /* Rx process is completed, restore hirda->RxState to Ready */
      hirda->RxState = DAL_IRDA_STATE_READY;

#if (USE_DAL_IRDA_REGISTER_CALLBACKS == 1)
      /* Call registered Rx complete callback */
      hirda->RxCpltCallback(hirda);
#else
      /* Call legacy weak Rx complete callback */
      DAL_IRDA_RxCpltCallback(hirda);
#endif /* USE_DAL_IRDA_REGISTER_CALLBACKS */

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
  * @brief  Configures the IRDA peripheral.
  * @param  hirda  Pointer to a IRDA_HandleTypeDef structure that contains
  *                the configuration information for the specified IRDA module.
  * @retval None
  */
static void IRDA_SetConfig(IRDA_HandleTypeDef *hirda)
{
  uint32_t pclk;

  /* Check the parameters */
  ASSERT_PARAM(IS_IRDA_INSTANCE(hirda->Instance));
  ASSERT_PARAM(IS_IRDA_BAUDRATE(hirda->Init.BaudRate));
  ASSERT_PARAM(IS_IRDA_WORD_LENGTH(hirda->Init.WordLength));
  ASSERT_PARAM(IS_IRDA_PARITY(hirda->Init.Parity));
  ASSERT_PARAM(IS_IRDA_MODE(hirda->Init.Mode));
  ASSERT_PARAM(IS_IRDA_POWERMODE(hirda->Init.IrDAMode));

  /*-------------------------- USART CTRL2 Configuration ------------------------*/
  /* Clear STOP[13:12] bits */
  CLEAR_BIT(hirda->Instance->CTRL2, USART_CTRL2_STOPCFG);

  /*-------------------------- USART CTRL1 Configuration -----------------------*/
  /* Clear M, PCE, PS, TE and RE bits */
  CLEAR_BIT(hirda->Instance->CTRL1, (USART_CTRL1_DBLCFG | USART_CTRL1_PCEN | USART_CTRL1_PCFG | USART_CTRL1_TXEN | USART_CTRL1_RXEN));

  /* Configure the USART Word Length, Parity and mode:
     Set the M bits according to hirda->Init.WordLength value
     Set PCE and PS bits according to hirda->Init.Parity value
     Set TE and RE bits according to hirda->Init.Mode value */
  /* Write to USART CTRL1 */
  SET_BIT(hirda->Instance->CTRL1, (hirda->Init.WordLength | hirda->Init.Parity | hirda->Init.Mode));

  /*-------------------------- USART CTRL3 Configuration -----------------------*/
  /* Clear CTSE and RTSE bits */
  CLEAR_BIT(hirda->Instance->CTRL3, (USART_CTRL3_RTSEN | USART_CTRL3_CTSEN));

  /*-------------------------- USART BR Configuration -----------------------*/
#if defined(USART6) && defined(UART9) && defined(UART10)
   if ((hirda->Instance == USART1) || (hirda->Instance == USART6) || (hirda->Instance == UART9) || (hirda->Instance == UART10))
   {
    pclk = DAL_RCM_GetPCLK2Freq();
    SET_BIT(hirda->Instance->BR, IRDA_BR(pclk, hirda->Init.BaudRate));
   }
#elif defined(USART6)
  if((hirda->Instance == USART1) || (hirda->Instance == USART6))
  {
    pclk = DAL_RCM_GetPCLK2Freq();
    SET_BIT(hirda->Instance->BR, IRDA_BR(pclk, hirda->Init.BaudRate));
  }
#else
  if(hirda->Instance == USART1)
  {
    pclk = DAL_RCM_GetPCLK2Freq();
    SET_BIT(hirda->Instance->BR, IRDA_BR(pclk, hirda->Init.BaudRate));
  }
#endif /* USART6 */
  else
  {
    pclk = DAL_RCM_GetPCLK1Freq();
    SET_BIT(hirda->Instance->BR, IRDA_BR(pclk, hirda->Init.BaudRate));
  }
}

/**
  * @}
  */

#endif /* DAL_IRDA_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

