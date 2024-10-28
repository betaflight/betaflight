/**
  *
  * @file    apm32f4xx_dal_uart.c
  * @brief   UART DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Universal Asynchronous Receiver Transmitter Peripheral (UART).
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
    The UART DAL driver can be used as follows:

    (#) Declare a UART_HandleTypeDef handle structure (eg. UART_HandleTypeDef huart).
    (#) Initialize the UART low level resources by implementing the DAL_UART_MspInit() API:
        (##) Enable the USARTx interface clock.
        (##) UART pins configuration:
            (+++) Enable the clock for the UART GPIOs.
            (+++) Configure the UART TX/RX pins as alternate function pull-up.
        (##) NVIC configuration if you need to use interrupt process (DAL_UART_Transmit_IT()
             and DAL_UART_Receive_IT() APIs):
            (+++) Configure the USARTx interrupt priority.
            (+++) Enable the NVIC USART IRQ handle.
        (##) DMA Configuration if you need to use DMA process (DAL_UART_Transmit_DMA()
             and DAL_UART_Receive_DMA() APIs):
            (+++) Declare a DMA handle structure for the Tx/Rx stream.
            (+++) Enable the DMAx interface clock.
            (+++) Configure the declared DMA handle structure with the required
                  Tx/Rx parameters.
            (+++) Configure the DMA Tx/Rx stream.
            (+++) Associate the initialized DMA handle to the UART DMA Tx/Rx handle.
            (+++) Configure the priority and enable the NVIC for the transfer complete
                  interrupt on the DMA Tx/Rx stream.
            (+++) Configure the USARTx interrupt priority and enable the NVIC USART IRQ handle
                  (used for last byte sending completion detection in DMA non circular mode)

    (#) Program the Baud Rate, Word Length, Stop Bit, Parity, Hardware
        flow control and Mode(Receiver/Transmitter) in the huart Init structure.

    (#) For the UART asynchronous mode, initialize the UART registers by calling
        the DAL_UART_Init() API.

    (#) For the UART Half duplex mode, initialize the UART registers by calling
        the DAL_HalfDuplex_Init() API.

    (#) For the LIN mode, initialize the UART registers by calling the DAL_LIN_Init() API.

    (#) For the Multi-Processor mode, initialize the UART registers by calling
        the DAL_MultiProcessor_Init() API.

     [..]
       (@) The specific UART interrupts (Transmission complete interrupt,
            RXNE interrupt and Error Interrupts) will be managed using the macros
            __DAL_UART_ENABLE_IT() and __DAL_UART_DISABLE_IT() inside the transmit
            and receive process.

     [..]
       (@) These APIs (DAL_UART_Init() and DAL_HalfDuplex_Init()) configure also the
            low level Hardware GPIO, CLOCK, CORTEX...etc) by calling the customized
            DAL_UART_MspInit() API.

    ##### Callback registration #####
    ==================================

    [..]
    The compilation define USE_DAL_UART_REGISTER_CALLBACKS when set to 1
    allows the user to configure dynamically the driver callbacks.

    [..]
    Use Function DAL_UART_RegisterCallback() to register a user callback.
    Function DAL_UART_RegisterCallback() allows to register following callbacks:
    (+) TxHalfCpltCallback        : Tx Half Complete Callback.
    (+) TxCpltCallback            : Tx Complete Callback.
    (+) RxHalfCpltCallback        : Rx Half Complete Callback.
    (+) RxCpltCallback            : Rx Complete Callback.
    (+) ErrorCallback             : Error Callback.
    (+) AbortCpltCallback         : Abort Complete Callback.
    (+) AbortTransmitCpltCallback : Abort Transmit Complete Callback.
    (+) AbortReceiveCpltCallback  : Abort Receive Complete Callback.
    (+) MspInitCallback           : UART MspInit.
    (+) MspDeInitCallback         : UART MspDeInit.
    This function takes as parameters the DAL peripheral handle, the Callback ID
    and a pointer to the user callback function.

    [..]
    Use function DAL_UART_UnRegisterCallback() to reset a callback to the default
    weak (surcharged) function.
    DAL_UART_UnRegisterCallback() takes as parameters the DAL peripheral handle,
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
    (+) MspInitCallback           : UART MspInit.
    (+) MspDeInitCallback         : UART MspDeInit.

    [..]
    For specific callback RxEventCallback, use dedicated registration/reset functions:
    respectively DAL_UART_RegisterRxEventCallback() , DAL_UART_UnRegisterRxEventCallback().

    [..]
    By default, after the DAL_UART_Init() and when the state is DAL_UART_STATE_RESET
    all callbacks are set to the corresponding weak (surcharged) functions:
    examples DAL_UART_TxCpltCallback(), DAL_UART_RxHalfCpltCallback().
    Exception done for MspInit and MspDeInit functions that are respectively
    reset to the legacy weak (surcharged) functions in the DAL_UART_Init()
    and DAL_UART_DeInit() only when these callbacks are null (not registered beforehand).
    If not, MspInit or MspDeInit are not null, the DAL_UART_Init() and DAL_UART_DeInit()
    keep and use the user MspInit/MspDeInit callbacks (registered beforehand).

    [..]
    Callbacks can be registered/unregistered in DAL_UART_STATE_READY state only.
    Exception done MspInit/MspDeInit that can be registered/unregistered
    in DAL_UART_STATE_READY or DAL_UART_STATE_RESET state, thus registered (user)
    MspInit/DeInit callbacks can be used during the Init/DeInit.
    In that case first register the MspInit/MspDeInit user callbacks
    using DAL_UART_RegisterCallback() before calling DAL_UART_DeInit()
    or DAL_UART_Init() function.

    [..]
    When The compilation define USE_DAL_UART_REGISTER_CALLBACKS is set to 0 or
    not defined, the callback registration feature is not available
    and weak (surcharged) callbacks are used.

     [..]
        Three operation modes are available within this driver :

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Send an amount of data in blocking mode using DAL_UART_Transmit()
       (+) Receive an amount of data in blocking mode using DAL_UART_Receive()

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Send an amount of data in non blocking mode using DAL_UART_Transmit_IT()
       (+) At transmission end of transfer DAL_UART_TxCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_UART_TxCpltCallback
       (+) Receive an amount of data in non blocking mode using DAL_UART_Receive_IT()
       (+) At reception end of transfer DAL_UART_RxCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_UART_RxCpltCallback
       (+) In case of transfer Error, DAL_UART_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer DAL_UART_ErrorCallback

     *** DMA mode IO operation ***
     ==============================
     [..]
       (+) Send an amount of data in non blocking mode (DMA) using DAL_UART_Transmit_DMA()
       (+) At transmission end of half transfer DAL_UART_TxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_UART_TxHalfCpltCallback
       (+) At transmission end of transfer DAL_UART_TxCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_UART_TxCpltCallback
       (+) Receive an amount of data in non blocking mode (DMA) using DAL_UART_Receive_DMA()
       (+) At reception end of half transfer DAL_UART_RxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_UART_RxHalfCpltCallback
       (+) At reception end of transfer DAL_UART_RxCpltCallback is executed and user can
            add his own code by customization of function pointer DAL_UART_RxCpltCallback
       (+) In case of transfer Error, DAL_UART_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer DAL_UART_ErrorCallback
       (+) Pause the DMA Transfer using DAL_UART_DMAPause()
       (+) Resume the DMA Transfer using DAL_UART_DMAResume()
       (+) Stop the DMA Transfer using DAL_UART_DMAStop()


    [..] This subsection also provides a set of additional functions providing enhanced reception
    services to user. (For example, these functions allow application to handle use cases
    where number of data to be received is unknown).

    (#) Compared to standard reception services which only consider number of received
        data elements as reception completion criteria, these functions also consider additional events
        as triggers for updating reception status to caller :
       (+) Detection of inactivity period (RX line has not been active for a given period).
          (++) RX inactivity detected by IDLE event, i.e. RX line has been in idle state (normally high state)
               for 1 frame time, after last received byte.

    (#) There are two mode of transfer:
       (+) Blocking mode: The reception is performed in polling mode, until either expected number of data is received,
           or till IDLE event occurs. Reception is handled only during function execution.
           When function exits, no data reception could occur. DAL status and number of actually received data elements,
           are returned by function after finishing transfer.
       (+) Non-Blocking mode: The reception is performed using Interrupts or DMA.
           These API's return the DAL status.
           The end of the data processing will be indicated through the
           dedicated UART IRQ when using Interrupt mode or the DMA IRQ when using DMA mode.
           The DAL_UARTEx_RxEventCallback() user callback will be executed during Receive process
           The DAL_UART_ErrorCallback()user callback will be executed when a reception error is detected.

    (#) Blocking mode API:
        (+) DAL_UARTEx_ReceiveToIdle()

    (#) Non-Blocking mode API with Interrupt:
        (+) DAL_UARTEx_ReceiveToIdle_IT()

    (#) Non-Blocking mode API with DMA:
        (+) DAL_UARTEx_ReceiveToIdle_DMA()


     *** UART DAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in UART DAL driver.

      (+) __DAL_UART_ENABLE: Enable the UART peripheral
      (+) __DAL_UART_DISABLE: Disable the UART peripheral
      (+) __DAL_UART_GET_FLAG : Check whether the specified UART flag is set or not
      (+) __DAL_UART_CLEAR_FLAG : Clear the specified UART pending flag
      (+) __DAL_UART_ENABLE_IT: Enable the specified UART interrupt
      (+) __DAL_UART_DISABLE_IT: Disable the specified UART interrupt
      (+) __DAL_UART_GET_IT_SOURCE: Check whether the specified UART interrupt has occurred or not

     [..]
       (@) You can refer to the UART DAL driver header file for more useful macros

  @endverbatim
     [..]
       (@) Additional remark: If the parity is enabled, then the MSB bit of the data written
           in the data register is transmitted but is changed by the parity bit.
           Depending on the frame length defined by the M bit (8-bits or 9-bits),
           the possible UART frame formats are as listed in the following table:
    +-------------------------------------------------------------+
    |   M bit |  PCE bit  |            UART frame                 |
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

/** @defgroup UART UART
  * @brief DAL UART module driver
  * @{
  */
#ifdef DAL_UART_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup UART_Private_Constants
  * @{
  */
/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup UART_Private_Functions  UART Private Functions
  * @{
  */

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
void UART_InitCallbacksToDefault(UART_HandleTypeDef *huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
static void UART_EndTxTransfer(UART_HandleTypeDef *huart);
static void UART_EndRxTransfer(UART_HandleTypeDef *huart);
static void UART_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void UART_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void UART_DMATxHalfCplt(DMA_HandleTypeDef *hdma);
static void UART_DMARxHalfCplt(DMA_HandleTypeDef *hdma);
static void UART_DMAError(DMA_HandleTypeDef *hdma);
static void UART_DMAAbortOnError(DMA_HandleTypeDef *hdma);
static void UART_DMATxAbortCallback(DMA_HandleTypeDef *hdma);
static void UART_DMARxAbortCallback(DMA_HandleTypeDef *hdma);
static void UART_DMATxOnlyAbortCallback(DMA_HandleTypeDef *hdma);
static void UART_DMARxOnlyAbortCallback(DMA_HandleTypeDef *hdma);
static DAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart);
static DAL_StatusTypeDef UART_EndTransmit_IT(UART_HandleTypeDef *huart);
static DAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart);
static DAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                                     uint32_t Tickstart, uint32_t Timeout);
static void UART_SetConfig(UART_HandleTypeDef *huart);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup UART_Exported_Functions UART Exported Functions
  * @{
  */

/** @defgroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to initialize the USARTx or the UARTy
    in asynchronous mode.
      (+) For the asynchronous mode only these parameters can be configured:
        (++) Baud Rate
        (++) Word Length
        (++) Stop Bit
        (++) Parity: If the parity is enabled, then the MSB bit of the data written
             in the data register is transmitted but is changed by the parity bit.
             Depending on the frame length defined by the M bit (8-bits or 9-bits),
             please refer to Reference manual for possible UART frame formats.
        (++) Hardware flow control
        (++) Receiver/transmitter modes
        (++) Over Sampling Method
    [..]
    The DAL_UART_Init(), DAL_HalfDuplex_Init(), DAL_LIN_Init() and DAL_MultiProcessor_Init() APIs
    follow respectively the UART asynchronous, UART Half duplex, LIN and Multi-Processor configuration
    procedures (details for the procedures are available in reference manual).

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the UART mode according to the specified parameters in
  *         the UART_InitTypeDef and create the associated handle.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Init(UART_HandleTypeDef *huart)
{
  /* Check the UART handle allocation */
  if (huart == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  if (huart->Init.HwFlowCtl != UART_HWCONTROL_NONE)
  {
    /* The hardware flow control is available only for USART1, USART2, USART3 and USART6.
       Except for APM32F446xx devices, that is available for USART1, USART2, USART3, USART6, UART4 and UART5.
    */
    ASSERT_PARAM(IS_UART_HWFLOW_INSTANCE(huart->Instance));
    ASSERT_PARAM(IS_UART_HARDWARE_FLOW_CONTROL(huart->Init.HwFlowCtl));
  }
  else
  {
    ASSERT_PARAM(IS_UART_INSTANCE(huart->Instance));
  }
  ASSERT_PARAM(IS_UART_WORD_LENGTH(huart->Init.WordLength));
  ASSERT_PARAM(IS_UART_OVERSAMPLING(huart->Init.OverSampling));

  if (huart->gState == DAL_UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = DAL_UNLOCKED;

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    UART_InitCallbacksToDefault(huart);

    if (huart->MspInitCallback == NULL)
    {
      huart->MspInitCallback = DAL_UART_MspInit;
    }

    /* Init the low level hardware */
    huart->MspInitCallback(huart);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    DAL_UART_MspInit(huart);
#endif /* (USE_DAL_UART_REGISTER_CALLBACKS) */
  }

  huart->gState = DAL_UART_STATE_BUSY;

  /* Disable the peripheral */
  __DAL_UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  UART_SetConfig(huart);

  /* In asynchronous mode, the following bits must be kept cleared:
     - LINEN and CLKEN bits in the USART_CTRL2 register,
     - SCEN, HDSEL and IREN  bits in the USART_CTRL3 register.*/
  CLEAR_BIT(huart->Instance->CTRL2, (USART_CTRL2_LINMEN | USART_CTRL2_CLKEN));
  CLEAR_BIT(huart->Instance->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_HDEN | USART_CTRL3_IREN));

  /* Enable the peripheral */
  __DAL_UART_ENABLE(huart);

  /* Initialize the UART state */
  huart->ErrorCode = DAL_UART_ERROR_NONE;
  huart->gState = DAL_UART_STATE_READY;
  huart->RxState = DAL_UART_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Initializes the half-duplex mode according to the specified
  *         parameters in the UART_InitTypeDef and create the associated handle.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HalfDuplex_Init(UART_HandleTypeDef *huart)
{
  /* Check the UART handle allocation */
  if (huart == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_UART_HALFDUPLEX_INSTANCE(huart->Instance));
  ASSERT_PARAM(IS_UART_WORD_LENGTH(huart->Init.WordLength));
  ASSERT_PARAM(IS_UART_OVERSAMPLING(huart->Init.OverSampling));

  if (huart->gState == DAL_UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = DAL_UNLOCKED;

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    UART_InitCallbacksToDefault(huart);

    if (huart->MspInitCallback == NULL)
    {
      huart->MspInitCallback = DAL_UART_MspInit;
    }

    /* Init the low level hardware */
    huart->MspInitCallback(huart);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    DAL_UART_MspInit(huart);
#endif /* (USE_DAL_UART_REGISTER_CALLBACKS) */
  }

  huart->gState = DAL_UART_STATE_BUSY;

  /* Disable the peripheral */
  __DAL_UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  UART_SetConfig(huart);

  /* In half-duplex mode, the following bits must be kept cleared:
     - LINEN and CLKEN bits in the USART_CTRL2 register,
     - SCEN and IREN bits in the USART_CTRL3 register.*/
  CLEAR_BIT(huart->Instance->CTRL2, (USART_CTRL2_LINMEN | USART_CTRL2_CLKEN));
  CLEAR_BIT(huart->Instance->CTRL3, (USART_CTRL3_IREN | USART_CTRL3_SCEN));

  /* Enable the Half-Duplex mode by setting the HDSEL bit in the CTRL3 register */
  SET_BIT(huart->Instance->CTRL3, USART_CTRL3_HDEN);

  /* Enable the peripheral */
  __DAL_UART_ENABLE(huart);

  /* Initialize the UART state*/
  huart->ErrorCode = DAL_UART_ERROR_NONE;
  huart->gState = DAL_UART_STATE_READY;
  huart->RxState = DAL_UART_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Initializes the LIN mode according to the specified
  *         parameters in the UART_InitTypeDef and create the associated handle.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  BreakDetectLength Specifies the LIN break detection length.
  *         This parameter can be one of the following values:
  *            @arg UART_LINBREAKDETECTLENGTH_10B: 10-bit break detection
  *            @arg UART_LINBREAKDETECTLENGTH_11B: 11-bit break detection
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength)
{
  /* Check the UART handle allocation */
  if (huart == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the LIN UART instance */
  ASSERT_PARAM(IS_UART_LIN_INSTANCE(huart->Instance));

  /* Check the Break detection length parameter */
  ASSERT_PARAM(IS_UART_LIN_BREAK_DETECT_LENGTH(BreakDetectLength));
  ASSERT_PARAM(IS_UART_LIN_WORD_LENGTH(huart->Init.WordLength));
  ASSERT_PARAM(IS_UART_LIN_OVERSAMPLING(huart->Init.OverSampling));

  if (huart->gState == DAL_UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = DAL_UNLOCKED;

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    UART_InitCallbacksToDefault(huart);

    if (huart->MspInitCallback == NULL)
    {
      huart->MspInitCallback = DAL_UART_MspInit;
    }

    /* Init the low level hardware */
    huart->MspInitCallback(huart);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    DAL_UART_MspInit(huart);
#endif /* (USE_DAL_UART_REGISTER_CALLBACKS) */
  }

  huart->gState = DAL_UART_STATE_BUSY;

  /* Disable the peripheral */
  __DAL_UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  UART_SetConfig(huart);

  /* In LIN mode, the following bits must be kept cleared:
     - CLKEN bits in the USART_CTRL2 register,
     - SCEN, HDSEL and IREN bits in the USART_CTRL3 register.*/
  CLEAR_BIT(huart->Instance->CTRL2, (USART_CTRL2_CLKEN));
  CLEAR_BIT(huart->Instance->CTRL3, (USART_CTRL3_HDEN | USART_CTRL3_IREN | USART_CTRL3_SCEN));

  /* Enable the LIN mode by setting the LINEN bit in the CTRL2 register */
  SET_BIT(huart->Instance->CTRL2, USART_CTRL2_LINMEN);

  /* Set the USART LIN Break detection length. */
  CLEAR_BIT(huart->Instance->CTRL2, USART_CTRL2_LBDLCFG);
  SET_BIT(huart->Instance->CTRL2, BreakDetectLength);

  /* Enable the peripheral */
  __DAL_UART_ENABLE(huart);

  /* Initialize the UART state*/
  huart->ErrorCode = DAL_UART_ERROR_NONE;
  huart->gState = DAL_UART_STATE_READY;
  huart->RxState = DAL_UART_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Initializes the Multi-Processor mode according to the specified
  *         parameters in the UART_InitTypeDef and create the associated handle.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  Address USART address
  * @param  WakeUpMethod specifies the USART wake-up method.
  *         This parameter can be one of the following values:
  *            @arg UART_WAKEUPMETHOD_IDLELINE: Wake-up by an idle line detection
  *            @arg UART_WAKEUPMETHOD_ADDRESSMARK: Wake-up by an address mark
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod)
{
  /* Check the UART handle allocation */
  if (huart == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_UART_INSTANCE(huart->Instance));

  /* Check the Address & wake up method parameters */
  ASSERT_PARAM(IS_UART_WAKEUPMETHOD(WakeUpMethod));
  ASSERT_PARAM(IS_UART_ADDRESS(Address));
  ASSERT_PARAM(IS_UART_WORD_LENGTH(huart->Init.WordLength));
  ASSERT_PARAM(IS_UART_OVERSAMPLING(huart->Init.OverSampling));

  if (huart->gState == DAL_UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = DAL_UNLOCKED;

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    UART_InitCallbacksToDefault(huart);

    if (huart->MspInitCallback == NULL)
    {
      huart->MspInitCallback = DAL_UART_MspInit;
    }

    /* Init the low level hardware */
    huart->MspInitCallback(huart);
#else
    /* Init the low level hardware : GPIO, CLOCK */
    DAL_UART_MspInit(huart);
#endif /* (USE_DAL_UART_REGISTER_CALLBACKS) */
  }

  huart->gState = DAL_UART_STATE_BUSY;

  /* Disable the peripheral */
  __DAL_UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  UART_SetConfig(huart);

  /* In Multi-Processor mode, the following bits must be kept cleared:
     - LINEN and CLKEN bits in the USART_CTRL2 register,
     - SCEN, HDSEL and IREN  bits in the USART_CTRL3 register */
  CLEAR_BIT(huart->Instance->CTRL2, (USART_CTRL2_LINMEN | USART_CTRL2_CLKEN));
  CLEAR_BIT(huart->Instance->CTRL3, (USART_CTRL3_SCEN | USART_CTRL3_HDEN | USART_CTRL3_IREN));

  /* Set the USART address node */
  CLEAR_BIT(huart->Instance->CTRL2, USART_CTRL2_ADDR);
  SET_BIT(huart->Instance->CTRL2, Address);

  /* Set the wake up method by setting the WAKE bit in the CTRL1 register */
  CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_WUPMCFG);
  SET_BIT(huart->Instance->CTRL1, WakeUpMethod);

  /* Enable the peripheral */
  __DAL_UART_ENABLE(huart);

  /* Initialize the UART state */
  huart->ErrorCode = DAL_UART_ERROR_NONE;
  huart->gState = DAL_UART_STATE_READY;
  huart->RxState = DAL_UART_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the UART peripheral.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_DeInit(UART_HandleTypeDef *huart)
{
  /* Check the UART handle allocation */
  if (huart == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_UART_INSTANCE(huart->Instance));

  huart->gState = DAL_UART_STATE_BUSY;

  /* Disable the Peripheral */
  __DAL_UART_DISABLE(huart);

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  if (huart->MspDeInitCallback == NULL)
  {
    huart->MspDeInitCallback = DAL_UART_MspDeInit;
  }
  /* DeInit the low level hardware */
  huart->MspDeInitCallback(huart);
#else
  /* DeInit the low level hardware */
  DAL_UART_MspDeInit(huart);
#endif /* (USE_DAL_UART_REGISTER_CALLBACKS) */

  huart->ErrorCode = DAL_UART_ERROR_NONE;
  huart->gState = DAL_UART_STATE_RESET;
  huart->RxState = DAL_UART_STATE_RESET;
  huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

  /* Process Unlock */
  __DAL_UNLOCK(huart);

  return DAL_OK;
}

/**
  * @brief  UART MSP Init.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void DAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_UART_MspInit could be implemented in the user file
   */
}

/**
  * @brief  UART MSP DeInit.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void DAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_UART_MspDeInit could be implemented in the user file
   */
}

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User UART Callback
  *         To be used instead of the weak predefined callback
  * @param  huart uart handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_UART_TX_HALFCOMPLETE_CB_ID Tx Half Complete Callback ID
  *           @arg @ref DAL_UART_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref DAL_UART_RX_HALFCOMPLETE_CB_ID Rx Half Complete Callback ID
  *           @arg @ref DAL_UART_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_UART_ERROR_CB_ID Error Callback ID
  *           @arg @ref DAL_UART_ABORT_COMPLETE_CB_ID Abort Complete Callback ID
  *           @arg @ref DAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID Abort Transmit Complete Callback ID
  *           @arg @ref DAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID Abort Receive Complete Callback ID
  *           @arg @ref DAL_UART_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref DAL_UART_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_RegisterCallback(UART_HandleTypeDef *huart, DAL_UART_CallbackIDTypeDef CallbackID,
                                            pUART_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(huart);

  if (huart->gState == DAL_UART_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_UART_TX_HALFCOMPLETE_CB_ID :
        huart->TxHalfCpltCallback = pCallback;
        break;

      case DAL_UART_TX_COMPLETE_CB_ID :
        huart->TxCpltCallback = pCallback;
        break;

      case DAL_UART_RX_HALFCOMPLETE_CB_ID :
        huart->RxHalfCpltCallback = pCallback;
        break;

      case DAL_UART_RX_COMPLETE_CB_ID :
        huart->RxCpltCallback = pCallback;
        break;

      case DAL_UART_ERROR_CB_ID :
        huart->ErrorCallback = pCallback;
        break;

      case DAL_UART_ABORT_COMPLETE_CB_ID :
        huart->AbortCpltCallback = pCallback;
        break;

      case DAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID :
        huart->AbortTransmitCpltCallback = pCallback;
        break;

      case DAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID :
        huart->AbortReceiveCpltCallback = pCallback;
        break;

      case DAL_UART_MSPINIT_CB_ID :
        huart->MspInitCallback = pCallback;
        break;

      case DAL_UART_MSPDEINIT_CB_ID :
        huart->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (huart->gState == DAL_UART_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_UART_MSPINIT_CB_ID :
        huart->MspInitCallback = pCallback;
        break;

      case DAL_UART_MSPDEINIT_CB_ID :
        huart->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(huart);

  return status;
}

/**
  * @brief  Unregister an UART Callback
  *         UART callaback is redirected to the weak predefined callback
  * @param  huart uart handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *           @arg @ref DAL_UART_TX_HALFCOMPLETE_CB_ID Tx Half Complete Callback ID
  *           @arg @ref DAL_UART_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref DAL_UART_RX_HALFCOMPLETE_CB_ID Rx Half Complete Callback ID
  *           @arg @ref DAL_UART_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref DAL_UART_ERROR_CB_ID Error Callback ID
  *           @arg @ref DAL_UART_ABORT_COMPLETE_CB_ID Abort Complete Callback ID
  *           @arg @ref DAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID Abort Transmit Complete Callback ID
  *           @arg @ref DAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID Abort Receive Complete Callback ID
  *           @arg @ref DAL_UART_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref DAL_UART_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_UnRegisterCallback(UART_HandleTypeDef *huart, DAL_UART_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(huart);

  if (DAL_UART_STATE_READY == huart->gState)
  {
    switch (CallbackID)
    {
      case DAL_UART_TX_HALFCOMPLETE_CB_ID :
        huart->TxHalfCpltCallback = DAL_UART_TxHalfCpltCallback;               /* Legacy weak  TxHalfCpltCallback       */
        break;

      case DAL_UART_TX_COMPLETE_CB_ID :
        huart->TxCpltCallback = DAL_UART_TxCpltCallback;                       /* Legacy weak TxCpltCallback            */
        break;

      case DAL_UART_RX_HALFCOMPLETE_CB_ID :
        huart->RxHalfCpltCallback = DAL_UART_RxHalfCpltCallback;               /* Legacy weak RxHalfCpltCallback        */
        break;

      case DAL_UART_RX_COMPLETE_CB_ID :
        huart->RxCpltCallback = DAL_UART_RxCpltCallback;                       /* Legacy weak RxCpltCallback            */
        break;

      case DAL_UART_ERROR_CB_ID :
        huart->ErrorCallback = DAL_UART_ErrorCallback;                         /* Legacy weak ErrorCallback             */
        break;

      case DAL_UART_ABORT_COMPLETE_CB_ID :
        huart->AbortCpltCallback = DAL_UART_AbortCpltCallback;                 /* Legacy weak AbortCpltCallback         */
        break;

      case DAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID :
        huart->AbortTransmitCpltCallback = DAL_UART_AbortTransmitCpltCallback; /* Legacy weak AbortTransmitCpltCallback */
        break;

      case DAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID :
        huart->AbortReceiveCpltCallback = DAL_UART_AbortReceiveCpltCallback;   /* Legacy weak AbortReceiveCpltCallback  */
        break;

      case DAL_UART_MSPINIT_CB_ID :
        huart->MspInitCallback = DAL_UART_MspInit;                             /* Legacy weak MspInitCallback           */
        break;

      case DAL_UART_MSPDEINIT_CB_ID :
        huart->MspDeInitCallback = DAL_UART_MspDeInit;                         /* Legacy weak MspDeInitCallback         */
        break;

      default :
        /* Update the error code */
        huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_UART_STATE_RESET == huart->gState)
  {
    switch (CallbackID)
    {
      case DAL_UART_MSPINIT_CB_ID :
        huart->MspInitCallback = DAL_UART_MspInit;
        break;

      case DAL_UART_MSPDEINIT_CB_ID :
        huart->MspDeInitCallback = DAL_UART_MspDeInit;
        break;

      default :
        /* Update the error code */
        huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(huart);

  return status;
}

/**
  * @brief  Register a User UART Rx Event Callback
  *         To be used instead of the weak predefined callback
  * @param  huart     Uart handle
  * @param  pCallback Pointer to the Rx Event Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_RegisterRxEventCallback(UART_HandleTypeDef *huart, pUART_RxEventCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(huart);

  if (huart->gState == DAL_UART_STATE_READY)
  {
    huart->RxEventCallback = pCallback;
  }
  else
  {
    huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(huart);

  return status;
}

/**
  * @brief  UnRegister the UART Rx Event Callback
  *         UART Rx Event Callback is redirected to the weak DAL_UARTEx_RxEventCallback() predefined callback
  * @param  huart     Uart handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_UnRegisterRxEventCallback(UART_HandleTypeDef *huart)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(huart);

  if (huart->gState == DAL_UART_STATE_READY)
  {
    huart->RxEventCallback = DAL_UARTEx_RxEventCallback; /* Legacy weak UART Rx Event Callback  */
  }
  else
  {
    huart->ErrorCode |= DAL_UART_ERROR_INVALID_CALLBACK;

    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(huart);
  return status;
}
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group2 IO operation functions
  *  @brief UART Transmit and Receive functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    This subsection provides a set of functions allowing to manage the UART asynchronous
    and Half duplex data transfers.

    (#) There are two modes of transfer:
       (+) Blocking mode: The communication is performed in polling mode.
           The DAL status of all data processing is returned by the same function
           after finishing transfer.
       (+) Non-Blocking mode: The communication is performed using Interrupts
           or DMA, these API's return the DAL status.
           The end of the data processing will be indicated through the
           dedicated UART IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The DAL_UART_TxCpltCallback(), DAL_UART_RxCpltCallback() user callbacks
           will be executed respectively at the end of the transmit or receive process
           The DAL_UART_ErrorCallback()user callback will be executed when a communication error is detected.

    (#) Blocking mode API's are :
        (+) DAL_UART_Transmit()
        (+) DAL_UART_Receive()

    (#) Non-Blocking mode API's with Interrupt are :
        (+) DAL_UART_Transmit_IT()
        (+) DAL_UART_Receive_IT()
        (+) DAL_UART_IRQHandler()

    (#) Non-Blocking mode API's with DMA are :
        (+) DAL_UART_Transmit_DMA()
        (+) DAL_UART_Receive_DMA()
        (+) DAL_UART_DMAPause()
        (+) DAL_UART_DMAResume()
        (+) DAL_UART_DMAStop()

    (#) A set of Transfer Complete Callbacks are provided in Non_Blocking mode:
        (+) DAL_UART_TxHalfCpltCallback()
        (+) DAL_UART_TxCpltCallback()
        (+) DAL_UART_RxHalfCpltCallback()
        (+) DAL_UART_RxCpltCallback()
        (+) DAL_UART_ErrorCallback()

    (#) Non-Blocking mode transfers could be aborted using Abort API's :
        (+) DAL_UART_Abort()
        (+) DAL_UART_AbortTransmit()
        (+) DAL_UART_AbortReceive()
        (+) DAL_UART_Abort_IT()
        (+) DAL_UART_AbortTransmit_IT()
        (+) DAL_UART_AbortReceive_IT()

    (#) For Abort services based on interrupts (DAL_UART_Abortxxx_IT), a set of Abort Complete Callbacks are provided:
        (+) DAL_UART_AbortCpltCallback()
        (+) DAL_UART_AbortTransmitCpltCallback()
        (+) DAL_UART_AbortReceiveCpltCallback()

    (#) A Rx Event Reception Callback (Rx event notification) is available for Non_Blocking modes of enhanced reception services:
        (+) DAL_UARTEx_RxEventCallback()

    (#) In Non-Blocking mode transfers, possible errors are split into 2 categories.
        Errors are handled as follows :
       (+) Error is considered as Recoverable and non blocking : Transfer could go till end, but error severity is
           to be evaluated by user : this concerns Frame Error, Parity Error or Noise Error in Interrupt mode reception .
           Received character is then retrieved and stored in Rx buffer, Error code is set to allow user to identify error type,
           and DAL_UART_ErrorCallback() user callback is executed. Transfer is kept ongoing on UART side.
           If user wants to abort it, Abort services should be called by user.
       (+) Error is considered as Blocking : Transfer could not be completed properly and is aborted.
           This concerns Overrun Error In Interrupt mode reception and all errors in DMA mode.
           Error code is set to allow user to identify error type, and DAL_UART_ErrorCallback() user callback is executed.

    -@- In the Half duplex communication, it is forbidden to run the transmit
        and receive process in parallel, the UART state DAL_UART_STATE_BUSY_TX_RX can't be useful.

@endverbatim
  * @{
  */

/**
  * @brief  Sends an amount of data in blocking mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  const uint8_t  *pdata8bits;
  const uint16_t *pdata16bits;
  uint32_t tickstart = 0U;

  /* Check that a Tx process is not already ongoing */
  if (huart->gState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(huart);

    huart->ErrorCode = DAL_UART_ERROR_NONE;
    huart->gState = DAL_UART_STATE_BUSY_TX;

    /* Init tickstart for timeout management */
    tickstart = DAL_GetTick();

    huart->TxXferSize = Size;
    huart->TxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pData needs to be handled as a uint16_t pointer */
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      pdata8bits  = NULL;
      pdata16bits = (const uint16_t *) pData;
    }
    else
    {
      pdata8bits  = pData;
      pdata16bits = NULL;
    }

    /* Process Unlocked */
    __DAL_UNLOCK(huart);

    while (huart->TxXferCount > 0U)
    {
      if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }
      if (pdata8bits == NULL)
      {
        huart->Instance->DATA = (uint16_t)(*pdata16bits & 0x01FFU);
        pdata16bits++;
      }
      else
      {
        huart->Instance->DATA = (uint8_t)(*pdata8bits & 0xFFU);
        pdata8bits++;
      }
      huart->TxXferCount--;
    }

    if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TC, RESET, tickstart, Timeout) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    /* At end of Tx process, restore huart->gState to Ready */
    huart->gState = DAL_UART_STATE_READY;

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Receives an amount of data in blocking mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint8_t  *pdata8bits;
  uint16_t *pdata16bits;
  uint32_t tickstart = 0U;

  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(huart);

    huart->ErrorCode = DAL_UART_ERROR_NONE;
    huart->RxState = DAL_UART_STATE_BUSY_RX;
    huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

    /* Init tickstart for timeout management */
    tickstart = DAL_GetTick();

    huart->RxXferSize = Size;
    huart->RxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pRxData needs to be handled as a uint16_t pointer */
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      pdata8bits  = NULL;
      pdata16bits = (uint16_t *) pData;
    }
    else
    {
      pdata8bits  = pData;
      pdata16bits = NULL;
    }

    /* Process Unlocked */
    __DAL_UNLOCK(huart);

    /* Check the remain data to be received */
    while (huart->RxXferCount > 0U)
    {
      if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_RXNE, RESET, tickstart, Timeout) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }
      if (pdata8bits == NULL)
      {
        *pdata16bits = (uint16_t)(huart->Instance->DATA & 0x01FF);
        pdata16bits++;
      }
      else
      {
        if ((huart->Init.WordLength == UART_WORDLENGTH_9B) || ((huart->Init.WordLength == UART_WORDLENGTH_8B) && (huart->Init.Parity == UART_PARITY_NONE)))
        {
          *pdata8bits = (uint8_t)(huart->Instance->DATA & (uint8_t)0x00FF);
        }
        else
        {
          *pdata8bits = (uint8_t)(huart->Instance->DATA & (uint8_t)0x007F);
        }
        pdata8bits++;
      }
      huart->RxXferCount--;
    }

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = DAL_UART_STATE_READY;

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  /* Check that a Tx process is not already ongoing */
  if (huart->gState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(huart);

    huart->pTxBuffPtr = pData;
    huart->TxXferSize = Size;
    huart->TxXferCount = Size;

    huart->ErrorCode = DAL_UART_ERROR_NONE;
    huart->gState = DAL_UART_STATE_BUSY_TX;

    /* Process Unlocked */
    __DAL_UNLOCK(huart);

    /* Enable the UART Transmit data register empty Interrupt */
    __DAL_UART_ENABLE_IT(huart, UART_IT_TXE);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Receives an amount of data in non blocking mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(huart);

    /* Set Reception type to Standard reception */
    huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

    return (UART_Start_Receive_IT(huart, pData, Size));
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Sends an amount of data in DMA mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  const uint32_t *tmp;

  /* Check that a Tx process is not already ongoing */
  if (huart->gState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(huart);

    huart->pTxBuffPtr = pData;
    huart->TxXferSize = Size;
    huart->TxXferCount = Size;

    huart->ErrorCode = DAL_UART_ERROR_NONE;
    huart->gState = DAL_UART_STATE_BUSY_TX;

    /* Set the UART DMA transfer complete callback */
    huart->hdmatx->XferCpltCallback = UART_DMATransmitCplt;

    /* Set the UART DMA Half transfer complete callback */
    huart->hdmatx->XferHalfCpltCallback = UART_DMATxHalfCplt;

    /* Set the DMA error callback */
    huart->hdmatx->XferErrorCallback = UART_DMAError;

    /* Set the DMA abort callback */
    huart->hdmatx->XferAbortCallback = NULL;

    /* Enable the UART transmit DMA stream */
    tmp = (const uint32_t *)&pData;
    DAL_DMA_Start_IT(huart->hdmatx, *(const uint32_t *)tmp, (uint32_t)&huart->Instance->DATA, Size);

    /* Clear the TC flag in the STS register by writing 0 to it */
    __DAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);

    /* Process Unlocked */
    __DAL_UNLOCK(huart);

    /* Enable the DMA transfer for transmit request by setting the DMAT bit
       in the UART CTRL3 register */
    ATOMIC_SET_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Receives an amount of data in DMA mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @note   When the UART parity is enabled (PCE = 1) the received data contains the parity bit.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(huart);

    /* Set Reception type to Standard reception */
    huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

    return (UART_Start_Receive_DMA(huart, pData, Size));
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Pauses the DMA Transfer.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_DMAPause(UART_HandleTypeDef *huart)
{
  uint32_t dmarequest = 0x00U;

  /* Process Locked */
  __DAL_LOCK(huart);

  dmarequest = DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if ((huart->gState == DAL_UART_STATE_BUSY_TX) && dmarequest)
  {
    /* Disable the UART DMA Tx request */
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);
  }

  dmarequest = DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if ((huart->RxState == DAL_UART_STATE_BUSY_RX) && dmarequest)
  {
    /* Disable RXNE, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_PEIEN);
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Disable the UART DMA Rx request */
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);
  }

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  return DAL_OK;
}

/**
  * @brief Resumes the DMA Transfer.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_DMAResume(UART_HandleTypeDef *huart)
{
  /* Process Locked */
  __DAL_LOCK(huart);

  if (huart->gState == DAL_UART_STATE_BUSY_TX)
  {
    /* Enable the UART DMA Tx request */
    ATOMIC_SET_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);
  }

  if (huart->RxState == DAL_UART_STATE_BUSY_RX)
  {
    /* Clear the Overrun flag before resuming the Rx transfer*/
    __DAL_UART_CLEAR_OREFLAG(huart);

    /* Re-enable PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
    if (huart->Init.Parity != UART_PARITY_NONE)
    {
      ATOMIC_SET_BIT(huart->Instance->CTRL1, USART_CTRL1_PEIEN);
    }
    ATOMIC_SET_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Enable the UART DMA Rx request */
    ATOMIC_SET_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);
  }

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  return DAL_OK;
}

/**
  * @brief Stops the DMA Transfer.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_DMAStop(UART_HandleTypeDef *huart)
{
  uint32_t dmarequest = 0x00U;
  /* The Lock is not implemented on this API to allow the user application
     to call the DAL UART API under callbacks DAL_UART_TxCpltCallback() / DAL_UART_RxCpltCallback():
     when calling DAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
     and the correspond call back is executed DAL_UART_TxCpltCallback() / DAL_UART_RxCpltCallback()
     */

  /* Stop UART DMA Tx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if ((huart->gState == DAL_UART_STATE_BUSY_TX) && dmarequest)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the UART DMA Tx stream */
    if (huart->hdmatx != NULL)
    {
      DAL_DMA_Abort(huart->hdmatx);
    }
    UART_EndTxTransfer(huart);
  }

  /* Stop UART DMA Rx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if ((huart->RxState == DAL_UART_STATE_BUSY_RX) && dmarequest)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the UART DMA Rx stream */
    if (huart->hdmarx != NULL)
    {
      DAL_DMA_Abort(huart->hdmarx);
    }
    UART_EndRxTransfer(huart);
  }

  return DAL_OK;
}

/**
  * @brief Receive an amount of data in blocking mode till either the expected number of data is received or an IDLE event occurs.
  * @note   DAL_OK is returned if reception is completed (expected number of data has been received)
  *         or if reception is stopped after IDLE event (less than the expected number of data has been received)
  *         In this case, RxLen output parameter indicates number of data available in reception buffer.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M = 01),
  *         the received data is handled as a set of uint16_t. In this case, Size must indicate the number
  *         of uint16_t available through pData.
  * @param huart   UART handle.
  * @param pData   Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param Size    Amount of data elements (uint8_t or uint16_t) to be received.
  * @param RxLen   Number of data elements finally received (could be lower than Size, in case reception ends on IDLE event)
  * @param Timeout Timeout duration expressed in ms (covers the whole reception sequence).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UARTEx_ReceiveToIdle(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint16_t *RxLen,
                                           uint32_t Timeout)
{
  uint8_t  *pdata8bits;
  uint16_t *pdata16bits;
  uint32_t tickstart;

  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  DAL_ERROR;
    }

    __DAL_LOCK(huart);

    huart->ErrorCode = DAL_UART_ERROR_NONE;
    huart->RxState = DAL_UART_STATE_BUSY_RX;
    huart->ReceptionType = DAL_UART_RECEPTION_TOIDLE;

    /* Init tickstart for timeout management */
    tickstart = DAL_GetTick();

    huart->RxXferSize  = Size;
    huart->RxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pRxData needs to be handled as a uint16_t pointer */
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      pdata8bits  = NULL;
      pdata16bits = (uint16_t *) pData;
    }
    else
    {
      pdata8bits  = pData;
      pdata16bits = NULL;
    }

    __DAL_UNLOCK(huart);

    /* Initialize output number of received elements */
    *RxLen = 0U;

    /* as long as data have to be received */
    while (huart->RxXferCount > 0U)
    {
      /* Check if IDLE flag is set */
      if (__DAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
      {
        /* Clear IDLE flag in ISR */
        __DAL_UART_CLEAR_IDLEFLAG(huart);

        /* If Set, but no data ever received, clear flag without exiting loop */
        /* If Set, and data has already been received, this means Idle Event is valid : End reception */
        if (*RxLen > 0U)
        {
          huart->RxState = DAL_UART_STATE_READY;

          return DAL_OK;
        }
      }

      /* Check if RXNE flag is set */
      if (__DAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
      {
        if (pdata8bits == NULL)
        {
          *pdata16bits = (uint16_t)(huart->Instance->DATA & (uint16_t)0x01FF);
          pdata16bits++;
        }
        else
        {
          if ((huart->Init.WordLength == UART_WORDLENGTH_9B) || ((huart->Init.WordLength == UART_WORDLENGTH_8B) && (huart->Init.Parity == UART_PARITY_NONE)))
          {
            *pdata8bits = (uint8_t)(huart->Instance->DATA & (uint8_t)0x00FF);
          }
          else
          {
            *pdata8bits = (uint8_t)(huart->Instance->DATA & (uint8_t)0x007F);
          }

          pdata8bits++;
        }
        /* Increment number of received elements */
        *RxLen += 1U;
        huart->RxXferCount--;
      }

      /* Check for the Timeout */
      if (Timeout != DAL_MAX_DELAY)
      {
        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          huart->RxState = DAL_UART_STATE_READY;

          return DAL_TIMEOUT;
        }
      }
    }

    /* Set number of received elements in output parameter : RxLen */
    *RxLen = huart->RxXferSize - huart->RxXferCount;
    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = DAL_UART_STATE_READY;

    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data in interrupt mode till either the expected number of data is received or an IDLE event occurs.
  * @note   Reception is initiated by this function call. Further progress of reception is achieved thanks
  *         to UART interrupts raised by RXNE and IDLE events. Callback is called at end of reception indicating
  *         number of received data elements.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M = 01),
  *         the received data is handled as a set of uint16_t. In this case, Size must indicate the number
  *         of uint16_t available through pData.
  * @param huart UART handle.
  * @param pData Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param Size  Amount of data elements (uint8_t or uint16_t) to be received.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  DAL_StatusTypeDef status;

  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    __DAL_LOCK(huart);

    /* Set Reception type to reception till IDLE Event*/
    huart->ReceptionType = DAL_UART_RECEPTION_TOIDLE;

    status =  UART_Start_Receive_IT(huart, pData, Size);

    /* Check Rx process has been successfully started */
    if (status == DAL_OK)
    {
      if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
      {
        __DAL_UART_CLEAR_IDLEFLAG(huart);
        ATOMIC_SET_BIT(huart->Instance->CTRL1, USART_CTRL1_IDLEIEN);
      }
      else
      {
        /* In case of errors already pending when reception is started,
           Interrupts may have already been raised and lead to reception abortion.
           (Overrun error for instance).
           In such case Reception Type has been reset to DAL_UART_RECEPTION_STANDARD. */
        status = DAL_ERROR;
      }
    }

    return status;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief Receive an amount of data in DMA mode till either the expected number of data is received or an IDLE event occurs.
  * @note   Reception is initiated by this function call. Further progress of reception is achieved thanks
  *         to DMA services, transferring automatically received data elements in user reception buffer and
  *         calling registered callbacks at half/end of reception. UART IDLE events are also used to consider
  *         reception phase as ended. In all cases, callback execution will indicate number of received data elements.
  * @note   When the UART parity is enabled (PCE = 1), the received data contain
  *         the parity bit (MSB position).
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M = 01),
  *         the received data is handled as a set of uint16_t. In this case, Size must indicate the number
  *         of uint16_t available through pData.
  * @param huart UART handle.
  * @param pData Pointer to data buffer (uint8_t or uint16_t data elements).
  * @param Size  Amount of data elements (uint8_t or uint16_t) to be received.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  DAL_StatusTypeDef status;

  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == DAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return DAL_ERROR;
    }

    __DAL_LOCK(huart);

    /* Set Reception type to reception till IDLE Event*/
    huart->ReceptionType = DAL_UART_RECEPTION_TOIDLE;

    status =  UART_Start_Receive_DMA(huart, pData, Size);

    /* Check Rx process has been successfully started */
    if (status == DAL_OK)
    {
      if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
      {
        __DAL_UART_CLEAR_IDLEFLAG(huart);
        ATOMIC_SET_BIT(huart->Instance->CTRL1, USART_CTRL1_IDLEIEN);
      }
      else
      {
        /* In case of errors already pending when reception is started,
           Interrupts may have already been raised and lead to reception abortion.
           (Overrun error for instance).
           In such case Reception Type has been reset to DAL_UART_RECEPTION_STANDARD. */
        status = DAL_ERROR;
      }
    }

    return status;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Abort ongoing transfers (blocking mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Tx and Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Abort(UART_HandleTypeDef *huart)
{
  /* Disable TXBEIEN, TXCIEN, RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* If Reception till IDLE event was ongoing, disable IDLEIEN interrupt */
  if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_IDLEIEN));
  }

  /* Disable the UART DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the UART DMA Tx stream: use blocking DMA Abort API (no callback) */
    if (huart->hdmatx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmatx->XferAbortCallback = NULL;

      if (DAL_DMA_Abort(huart->hdmatx) != DAL_OK)
      {
        if (DAL_DMA_GetError(huart->hdmatx) == DAL_DMA_ERROR_TIMEOUT)
        {
          /* Set error code to DMA */
          huart->ErrorCode = DAL_UART_ERROR_DMA;

          return DAL_TIMEOUT;
        }
      }
    }
  }

  /* Disable the UART DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the UART DMA Rx stream: use blocking DMA Abort API (no callback) */
    if (huart->hdmarx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmarx->XferAbortCallback = NULL;

      if (DAL_DMA_Abort(huart->hdmarx) != DAL_OK)
      {
        if (DAL_DMA_GetError(huart->hdmarx) == DAL_DMA_ERROR_TIMEOUT)
        {
          /* Set error code to DMA */
          huart->ErrorCode = DAL_UART_ERROR_DMA;

          return DAL_TIMEOUT;
        }
      }
    }
  }

  /* Reset Tx and Rx transfer counters */
  huart->TxXferCount = 0x00U;
  huart->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  huart->ErrorCode = DAL_UART_ERROR_NONE;

  /* Restore huart->RxState and huart->gState to Ready */
  huart->RxState = DAL_UART_STATE_READY;
  huart->gState = DAL_UART_STATE_READY;
  huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (blocking mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing Tx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Tx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
{
  /* Disable TXBEIEN and TXCIEN interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* Disable the UART DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the UART DMA Tx stream : use blocking DMA Abort API (no callback) */
    if (huart->hdmatx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmatx->XferAbortCallback = NULL;

      if (DAL_DMA_Abort(huart->hdmatx) != DAL_OK)
      {
        if (DAL_DMA_GetError(huart->hdmatx) == DAL_DMA_ERROR_TIMEOUT)
        {
          /* Set error code to DMA */
          huart->ErrorCode = DAL_UART_ERROR_DMA;

          return DAL_TIMEOUT;
        }
      }
    }
  }

  /* Reset Tx transfer counter */
  huart->TxXferCount = 0x00U;

  /* Restore huart->gState to Ready */
  huart->gState = DAL_UART_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (blocking mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing Rx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
  /* Disable RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* If Reception till IDLE event was ongoing, disable IDLEIEN interrupt */
  if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_IDLEIEN));
  }

  /* Disable the UART DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the UART DMA Rx stream : use blocking DMA Abort API (no callback) */
    if (huart->hdmarx != NULL)
    {
      /* Set the UART DMA Abort callback to Null.
         No call back execution at end of DMA abort procedure */
      huart->hdmarx->XferAbortCallback = NULL;

      if (DAL_DMA_Abort(huart->hdmarx) != DAL_OK)
      {
        if (DAL_DMA_GetError(huart->hdmarx) == DAL_DMA_ERROR_TIMEOUT)
        {
          /* Set error code to DMA */
          huart->ErrorCode = DAL_UART_ERROR_DMA;

          return DAL_TIMEOUT;
        }
      }
    }
  }

  /* Reset Rx transfer counter */
  huart->RxXferCount = 0x00U;

  /* Restore huart->RxState to Ready */
  huart->RxState = DAL_UART_STATE_READY;
  huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

  return DAL_OK;
}

/**
  * @brief  Abort ongoing transfers (Interrupt mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Tx and Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_Abort_IT(UART_HandleTypeDef *huart)
{
  uint32_t AbortCplt = 0x01U;

  /* Disable TXBEIEN, TXCIEN, RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* If Reception till IDLE event was ongoing, disable IDLEIEN interrupt */
  if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_IDLEIEN));
  }

  /* If DMA Tx and/or DMA Rx Handles are associated to UART Handle, DMA Abort complete callbacks should be initialised
     before any call to DMA Abort functions */
  /* DMA Tx Handle is valid */
  if (huart->hdmatx != NULL)
  {
    /* Set DMA Abort Complete callback if UART DMA Tx request if enabled.
       Otherwise, set it to NULL */
    if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMATXEN))
    {
      huart->hdmatx->XferAbortCallback = UART_DMATxAbortCallback;
    }
    else
    {
      huart->hdmatx->XferAbortCallback = NULL;
    }
  }
  /* DMA Rx Handle is valid */
  if (huart->hdmarx != NULL)
  {
    /* Set DMA Abort Complete callback if UART DMA Rx request if enabled.
       Otherwise, set it to NULL */
    if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN))
    {
      huart->hdmarx->XferAbortCallback = UART_DMARxAbortCallback;
    }
    else
    {
      huart->hdmarx->XferAbortCallback = NULL;
    }
  }

  /* Disable the UART DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    /* Disable DMA Tx at UART level */
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the UART DMA Tx stream : use non blocking DMA Abort API (callback) */
    if (huart->hdmatx != NULL)
    {
      /* UART Tx DMA Abort callback has already been initialised :
         will lead to call DAL_UART_AbortCpltCallback() at end of DMA abort procedure */

      /* Abort DMA TX */
      if (DAL_DMA_Abort_IT(huart->hdmatx) != DAL_OK)
      {
        huart->hdmatx->XferAbortCallback = NULL;
      }
      else
      {
        AbortCplt = 0x00U;
      }
    }
  }

  /* Disable the UART DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the UART DMA Rx stream : use non blocking DMA Abort API (callback) */
    if (huart->hdmarx != NULL)
    {
      /* UART Rx DMA Abort callback has already been initialised :
         will lead to call DAL_UART_AbortCpltCallback() at end of DMA abort procedure */

      /* Abort DMA RX */
      if (DAL_DMA_Abort_IT(huart->hdmarx) != DAL_OK)
      {
        huart->hdmarx->XferAbortCallback = NULL;
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
    huart->TxXferCount = 0x00U;
    huart->RxXferCount = 0x00U;

    /* Reset ErrorCode */
    huart->ErrorCode = DAL_UART_ERROR_NONE;

    /* Restore huart->gState and huart->RxState to Ready */
    huart->gState  = DAL_UART_STATE_READY;
    huart->RxState = DAL_UART_STATE_READY;
    huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    /* Call registered Abort complete callback */
    huart->AbortCpltCallback(huart);
#else
    /* Call legacy weak Abort complete callback */
    DAL_UART_AbortCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
  }

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (Interrupt mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing Tx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Tx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart)
{
  /* Disable TXBEIEN and TXCIEN interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* Disable the UART DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMATXEN))
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Abort the UART DMA Tx stream : use blocking DMA Abort API (no callback) */
    if (huart->hdmatx != NULL)
    {
      /* Set the UART DMA Abort callback :
         will lead to call DAL_UART_AbortCpltCallback() at end of DMA abort procedure */
      huart->hdmatx->XferAbortCallback = UART_DMATxOnlyAbortCallback;

      /* Abort DMA TX */
      if (DAL_DMA_Abort_IT(huart->hdmatx) != DAL_OK)
      {
        /* Call Directly huart->hdmatx->XferAbortCallback function in case of error */
        huart->hdmatx->XferAbortCallback(huart->hdmatx);
      }
    }
    else
    {
      /* Reset Tx transfer counter */
      huart->TxXferCount = 0x00U;

      /* Restore huart->gState to Ready */
      huart->gState = DAL_UART_STATE_READY;

      /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
      /* Call registered Abort Transmit Complete Callback */
      huart->AbortTransmitCpltCallback(huart);
#else
      /* Call legacy weak Abort Transmit Complete Callback */
      DAL_UART_AbortTransmitCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
    }
  }
  else
  {
    /* Reset Tx transfer counter */
    huart->TxXferCount = 0x00U;

    /* Restore huart->gState to Ready */
    huart->gState = DAL_UART_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    /* Call registered Abort Transmit Complete Callback */
    huart->AbortTransmitCpltCallback(huart);
#else
    /* Call legacy weak Abort Transmit Complete Callback */
    DAL_UART_AbortTransmitCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
  }

  return DAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (Interrupt mode).
  * @param  huart UART handle.
  * @note   This procedure could be used for aborting any ongoing Rx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART Interrupts (Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart)
{
  /* Disable RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* If Reception till IDLE event was ongoing, disable IDLEIEN interrupt */
  if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_IDLEIEN));
  }

  /* Disable the UART DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN))
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* Abort the UART DMA Rx stream : use blocking DMA Abort API (no callback) */
    if (huart->hdmarx != NULL)
    {
      /* Set the UART DMA Abort callback :
         will lead to call DAL_UART_AbortCpltCallback() at end of DMA abort procedure */
      huart->hdmarx->XferAbortCallback = UART_DMARxOnlyAbortCallback;

      /* Abort DMA RX */
      if (DAL_DMA_Abort_IT(huart->hdmarx) != DAL_OK)
      {
        /* Call Directly huart->hdmarx->XferAbortCallback function in case of error */
        huart->hdmarx->XferAbortCallback(huart->hdmarx);
      }
    }
    else
    {
      /* Reset Rx transfer counter */
      huart->RxXferCount = 0x00U;

      /* Restore huart->RxState to Ready */
      huart->RxState = DAL_UART_STATE_READY;
      huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

      /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
      /* Call registered Abort Receive Complete Callback */
      huart->AbortReceiveCpltCallback(huart);
#else
      /* Call legacy weak Abort Receive Complete Callback */
      DAL_UART_AbortReceiveCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
    }
  }
  else
  {
    /* Reset Rx transfer counter */
    huart->RxXferCount = 0x00U;

    /* Restore huart->RxState to Ready */
    huart->RxState = DAL_UART_STATE_READY;
    huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    /* Call registered Abort Receive Complete Callback */
    huart->AbortReceiveCpltCallback(huart);
#else
    /* Call legacy weak Abort Receive Complete Callback */
    DAL_UART_AbortReceiveCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
  }

  return DAL_OK;
}

/**
  * @brief  This function handles UART interrupt request.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void DAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->STS);
  uint32_t cr1its     = READ_REG(huart->Instance->CTRL1);
  uint32_t cr3its     = READ_REG(huart->Instance->CTRL3);
  uint32_t errorflags = 0x00U;
  uint32_t dmarequest = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_STS_PEFLG | USART_STS_FEFLG | USART_STS_OVREFLG | USART_STS_NEFLG));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver -------------------------------------------------*/
    if (((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))
    {
      UART_Receive_IT(huart);
      return;
    }
  }

  /* If some errors occur */
  if ((errorflags != RESET) && (((cr3its & USART_CTRL3_ERRIEN) != RESET)
                                || ((cr1its & (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN)) != RESET)))
  {
    /* UART parity error interrupt occurred ----------------------------------*/
    if (((isrflags & USART_STS_PEFLG) != RESET) && ((cr1its & USART_CTRL1_PEIEN) != RESET))
    {
      huart->ErrorCode |= DAL_UART_ERROR_PE;
    }

    /* UART noise error interrupt occurred -----------------------------------*/
    if (((isrflags & USART_STS_NEFLG) != RESET) && ((cr3its & USART_CTRL3_ERRIEN) != RESET))
    {
      huart->ErrorCode |= DAL_UART_ERROR_NE;
    }

    /* UART frame error interrupt occurred -----------------------------------*/
    if (((isrflags & USART_STS_FEFLG) != RESET) && ((cr3its & USART_CTRL3_ERRIEN) != RESET))
    {
      huart->ErrorCode |= DAL_UART_ERROR_FE;
    }

    /* UART Over-Run interrupt occurred --------------------------------------*/
    if (((isrflags & USART_STS_OVREFLG) != RESET) && (((cr1its & USART_CTRL1_RXBNEIEN) != RESET)
                                                 || ((cr3its & USART_CTRL3_ERRIEN) != RESET)))
    {
      huart->ErrorCode |= DAL_UART_ERROR_ORE;
    }

    /* Call UART Error Call back function if need be --------------------------*/
    if (huart->ErrorCode != DAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver -----------------------------------------------*/
      if (((isrflags & USART_STS_RXBNEFLG) != RESET) && ((cr1its & USART_CTRL1_RXBNEIEN) != RESET))
      {
        UART_Receive_IT(huart);
      }

      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
         consider error as blocking */
      dmarequest = DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);
      if (((huart->ErrorCode & DAL_UART_ERROR_ORE) != RESET) || dmarequest)
      {
        /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
        UART_EndRxTransfer(huart);

        /* Disable the UART DMA Rx request if enabled */
        if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN))
        {
          ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

          /* Abort the UART DMA Rx stream */
          if (huart->hdmarx != NULL)
          {
            /* Set the UART DMA Abort callback :
               will lead to call DAL_UART_ErrorCallback() at end of DMA abort procedure */
            huart->hdmarx->XferAbortCallback = UART_DMAAbortOnError;
            if (DAL_DMA_Abort_IT(huart->hdmarx) != DAL_OK)
            {
              /* Call Directly XferAbortCallback function in case of error */
              huart->hdmarx->XferAbortCallback(huart->hdmarx);
            }
          }
          else
          {
            /* Call user error callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
            /*Call registered error callback*/
            huart->ErrorCallback(huart);
#else
            /*Call legacy weak error callback*/
            DAL_UART_ErrorCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
          }
        }
        else
        {
          /* Call user error callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
          /*Call registered error callback*/
          huart->ErrorCallback(huart);
#else
          /*Call legacy weak error callback*/
          DAL_UART_ErrorCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
        }
      }
      else
      {
        /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered error callback*/
        huart->ErrorCallback(huart);
#else
        /*Call legacy weak error callback*/
        DAL_UART_ErrorCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */

        huart->ErrorCode = DAL_UART_ERROR_NONE;
      }
    }
    return;
  } /* End if some error occurs */

  /* Check current reception Mode :
     If Reception till IDLE event has been selected : */
  if ((huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
      && ((isrflags & USART_STS_IDLEFLG) != 0U)
      && ((cr1its & USART_STS_IDLEFLG) != 0U))
  {
    __DAL_UART_CLEAR_IDLEFLAG(huart);

    /* Check if DMA mode is enabled in UART */
    if (DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN))
    {
      /* DMA mode enabled */
      /* Check received length : If all expected data are received, do nothing,
         (DMA cplt callback will be called).
         Otherwise, if at least one data has already been received, IDLE event is to be notified to user */
      uint16_t nb_remaining_rx_data = (uint16_t) __DAL_DMA_GET_COUNTER(huart->hdmarx);
      if ((nb_remaining_rx_data > 0U)
          && (nb_remaining_rx_data < huart->RxXferSize))
      {
        /* Reception is not complete */
        huart->RxXferCount = nb_remaining_rx_data;

        /* In Normal mode, end DMA xfer and DAL UART Rx process*/
        if (huart->hdmarx->Init.Mode != DMA_CIRCULAR)
        {
          /* Disable PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
          ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_PEIEN);
          ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

          /* Disable the DMA transfer for the receiver request by resetting the DMAR bit
             in the UART CTRL3 register */
          ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

          /* At end of Rx process, restore huart->RxState to Ready */
          huart->RxState = DAL_UART_STATE_READY;
          huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

          ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_IDLEIEN);

          /* Last bytes received, so no need as the abort is immediate */
          (void)DAL_DMA_Abort(huart->hdmarx);
        }
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx Event callback*/
        huart->RxEventCallback(huart, (huart->RxXferSize - huart->RxXferCount));
#else
        /*Call legacy weak Rx Event callback*/
        DAL_UARTEx_RxEventCallback(huart, (huart->RxXferSize - huart->RxXferCount));
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
      }
      return;
    }
    else
    {
      /* DMA mode not enabled */
      /* Check received length : If all expected data are received, do nothing.
         Otherwise, if at least one data has already been received, IDLE event is to be notified to user */
      uint16_t nb_rx_data = huart->RxXferSize - huart->RxXferCount;
      if ((huart->RxXferCount > 0U)
          && (nb_rx_data > 0U))
      {
        /* Disable the UART Parity Error Interrupt and RXBNEIEN interrupts */
        ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

        /* Rx process is completed, restore huart->RxState to Ready */
        huart->RxState = DAL_UART_STATE_READY;
        huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

        ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_IDLEIEN);
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx complete callback*/
        huart->RxEventCallback(huart, nb_rx_data);
#else
        /*Call legacy weak Rx Event callback*/
        DAL_UARTEx_RxEventCallback(huart, nb_rx_data);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
      }
      return;
    }
  }

  /* UART in mode Transmitter ------------------------------------------------*/
  if (((isrflags & USART_STS_TXBEFLG) != RESET) && ((cr1its & USART_CTRL1_TXBEIEN) != RESET))
  {
    UART_Transmit_IT(huart);
    return;
  }

  /* UART in mode Transmitter end --------------------------------------------*/
  if (((isrflags & USART_STS_TXCFLG) != RESET) && ((cr1its & USART_CTRL1_TXCIEN) != RESET))
  {
    UART_EndTransmit_IT(huart);
    return;
  }
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void DAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_UART_TxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Tx Half Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void DAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_UART_TxHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void DAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_UART_RxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Rx Half Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void DAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_UART_RxHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  UART error callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void DAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the DAL_UART_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  UART Abort Complete callback.
  * @param  huart UART handle.
  * @retval None
  */
__weak void DAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_UART_AbortCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  UART Abort Complete callback.
  * @param  huart UART handle.
  * @retval None
  */
__weak void DAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_UART_AbortTransmitCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  UART Abort Receive Complete callback.
  * @param  huart UART handle.
  * @retval None
  */
__weak void DAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_UART_AbortReceiveCpltCallback can be implemented in the user file.
   */
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
__weak void DAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  UNUSED(Size);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
}

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   UART control functions
  *
@verbatim
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control the UART:
    (+) DAL_LIN_SendBreak() API can be helpful to transmit the break character.
    (+) DAL_MultiProcessor_EnterMuteMode() API can be helpful to enter the UART in mute mode.
    (+) DAL_MultiProcessor_ExitMuteMode() API can be helpful to exit the UART mute mode by software.
    (+) DAL_HalfDuplex_EnableTransmitter() API to enable the UART transmitter and disables the UART receiver in Half Duplex mode
    (+) DAL_HalfDuplex_EnableReceiver() API to enable the UART receiver and disables the UART transmitter in Half Duplex mode

@endverbatim
  * @{
  */

/**
  * @brief  Transmits break characters.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_LIN_SendBreak(UART_HandleTypeDef *huart)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_UART_INSTANCE(huart->Instance));

  /* Process Locked */
  __DAL_LOCK(huart);

  huart->gState = DAL_UART_STATE_BUSY;

  /* Send break characters */
  ATOMIC_SET_BIT(huart->Instance->CTRL1, USART_CTRL1_TXBF);

  huart->gState = DAL_UART_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  return DAL_OK;
}

/**
  * @brief  Enters the UART in mute mode.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_UART_INSTANCE(huart->Instance));

  /* Process Locked */
  __DAL_LOCK(huart);

  huart->gState = DAL_UART_STATE_BUSY;

  /* Enable the USART mute mode  by setting the RWU bit in the CTRL1 register */
  ATOMIC_SET_BIT(huart->Instance->CTRL1, USART_CTRL1_RXMUTEEN);

  huart->gState = DAL_UART_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  return DAL_OK;
}

/**
  * @brief  Exits the UART mute mode: wake up software.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart)
{
  /* Check the parameters */
  ASSERT_PARAM(IS_UART_INSTANCE(huart->Instance));

  /* Process Locked */
  __DAL_LOCK(huart);

  huart->gState = DAL_UART_STATE_BUSY;

  /* Disable the USART mute mode by clearing the RWU bit in the CTRL1 register */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_RXMUTEEN);

  huart->gState = DAL_UART_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  return DAL_OK;
}

/**
  * @brief  Enables the UART transmitter and disables the UART receiver.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart)
{
  uint32_t tmpreg = 0x00U;

  /* Process Locked */
  __DAL_LOCK(huart);

  huart->gState = DAL_UART_STATE_BUSY;

  /*-------------------------- USART CTRL1 Configuration -----------------------*/
  tmpreg = huart->Instance->CTRL1;

  /* Clear TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)(USART_CTRL1_TXEN | USART_CTRL1_RXEN));

  /* Enable the USART's transmit interface by setting the TE bit in the USART CTRL1 register */
  tmpreg |= (uint32_t)USART_CTRL1_TXEN;

  /* Write to USART CTRL1 */
  WRITE_REG(huart->Instance->CTRL1, (uint32_t)tmpreg);

  huart->gState = DAL_UART_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  return DAL_OK;
}

/**
  * @brief  Enables the UART receiver and disables the UART transmitter.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart)
{
  uint32_t tmpreg = 0x00U;

  /* Process Locked */
  __DAL_LOCK(huart);

  huart->gState = DAL_UART_STATE_BUSY;

  /*-------------------------- USART CTRL1 Configuration -----------------------*/
  tmpreg = huart->Instance->CTRL1;

  /* Clear TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)(USART_CTRL1_TXEN | USART_CTRL1_RXEN));

  /* Enable the USART's receive interface by setting the RE bit in the USART CTRL1 register */
  tmpreg |= (uint32_t)USART_CTRL1_RXEN;

  /* Write to USART CTRL1 */
  WRITE_REG(huart->Instance->CTRL1, (uint32_t)tmpreg);

  huart->gState = DAL_UART_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup UART_Exported_Functions_Group4 Peripheral State and Errors functions
  *  @brief   UART State and Errors functions
  *
@verbatim
  ==============================================================================
                 ##### Peripheral State and Errors functions #####
  ==============================================================================
 [..]
   This subsection provides a set of functions allowing to return the State of
   UART communication process, return Peripheral Errors occurred during communication
   process
   (+) DAL_UART_GetState() API can be helpful to check in run-time the state of the UART peripheral.
   (+) DAL_UART_GetError() check in run-time errors that could be occurred during communication.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the UART state.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL state
  */
DAL_UART_StateTypeDef DAL_UART_GetState(UART_HandleTypeDef *huart)
{
  uint32_t temp1 = 0x00U, temp2 = 0x00U;
  temp1 = huart->gState;
  temp2 = huart->RxState;

  return (DAL_UART_StateTypeDef)(temp1 | temp2);
}

/**
  * @brief  Return the UART error code
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART.
  * @retval UART Error Code
  */
uint32_t DAL_UART_GetError(UART_HandleTypeDef *huart)
{
  return huart->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup UART_Private_Functions UART Private Functions
  * @{
  */

/**
  * @brief  Initialize the callbacks to their default values.
  * @param  huart UART handle.
  * @retval none
  */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
void UART_InitCallbacksToDefault(UART_HandleTypeDef *huart)
{
  /* Init the UART Callback settings */
  huart->TxHalfCpltCallback        = DAL_UART_TxHalfCpltCallback;        /* Legacy weak TxHalfCpltCallback        */
  huart->TxCpltCallback            = DAL_UART_TxCpltCallback;            /* Legacy weak TxCpltCallback            */
  huart->RxHalfCpltCallback        = DAL_UART_RxHalfCpltCallback;        /* Legacy weak RxHalfCpltCallback        */
  huart->RxCpltCallback            = DAL_UART_RxCpltCallback;            /* Legacy weak RxCpltCallback            */
  huart->ErrorCallback             = DAL_UART_ErrorCallback;             /* Legacy weak ErrorCallback             */
  huart->AbortCpltCallback         = DAL_UART_AbortCpltCallback;         /* Legacy weak AbortCpltCallback         */
  huart->AbortTransmitCpltCallback = DAL_UART_AbortTransmitCpltCallback; /* Legacy weak AbortTransmitCpltCallback */
  huart->AbortReceiveCpltCallback  = DAL_UART_AbortReceiveCpltCallback;  /* Legacy weak AbortReceiveCpltCallback  */
  huart->RxEventCallback           = DAL_UARTEx_RxEventCallback;         /* Legacy weak RxEventCallback           */

}
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */

/**
  * @brief  DMA UART transmit process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal mode*/
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == 0U)
  {
    huart->TxXferCount = 0x00U;

    /* Disable the DMA transfer for transmit request by setting the DMAT bit
       in the UART CTRL3 register */
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);

    /* Enable the UART Transmit Complete Interrupt */
    ATOMIC_SET_BIT(huart->Instance->CTRL1, USART_CTRL1_TXCIEN);

  }
  /* DMA Circular mode */
  else
  {
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Tx complete callback*/
    huart->TxCpltCallback(huart);
#else
    /*Call legacy weak Tx complete callback*/
    DAL_UART_TxCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
  }
}

/**
  * @brief DMA UART transmit process half complete callback
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  /*Call registered Tx complete callback*/
  huart->TxHalfCpltCallback(huart);
#else
  /*Call legacy weak Tx complete callback*/
  DAL_UART_TxHalfCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA UART receive process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  /* DMA Normal mode*/
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == 0U)
  {
    huart->RxXferCount = 0U;

    /* Disable RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_PEIEN);
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

    /* Disable the DMA transfer for the receiver request by setting the DMAR bit
       in the UART CTRL3 register */
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = DAL_UART_STATE_READY;

    /* If Reception till IDLE event has been selected, Disable IDLE Interrupt */
    if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
    {
      ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_IDLEIEN);
    }
  }

  /* Check current reception Mode :
     If Reception till IDLE event has been selected : use Rx Event callback */
  if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
  {
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx Event callback*/
    huart->RxEventCallback(huart, huart->RxXferSize);
#else
    /*Call legacy weak Rx Event callback*/
    DAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
  }
  else
  {
    /* In other cases : use Rx Complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx complete callback*/
    huart->RxCpltCallback(huart);
#else
    /*Call legacy weak Rx complete callback*/
    DAL_UART_RxCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
  }
}

/**
  * @brief DMA UART receive process half complete callback
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMARxHalfCplt(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Check current reception Mode :
     If Reception till IDLE event has been selected : use Rx Event callback */
  if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
  {
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx Event callback*/
    huart->RxEventCallback(huart, huart->RxXferSize / 2U);
#else
    /*Call legacy weak Rx Event callback*/
    DAL_UARTEx_RxEventCallback(huart, huart->RxXferSize / 2U);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
  }
  else
  {
    /* In other cases : use Rx Half Complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx Half complete callback*/
    huart->RxHalfCpltCallback(huart);
#else
    /*Call legacy weak Rx Half complete callback*/
    DAL_UART_RxHalfCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  DMA UART communication error callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMAError(DMA_HandleTypeDef *hdma)
{
  uint32_t dmarequest = 0x00U;
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Stop UART DMA Tx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMATXEN);
  if ((huart->gState == DAL_UART_STATE_BUSY_TX) && dmarequest)
  {
    huart->TxXferCount = 0x00U;
    UART_EndTxTransfer(huart);
  }

  /* Stop UART DMA Rx request if ongoing */
  dmarequest = DAL_IS_BIT_SET(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);
  if ((huart->RxState == DAL_UART_STATE_BUSY_RX) && dmarequest)
  {
    huart->RxXferCount = 0x00U;
    UART_EndRxTransfer(huart);
  }

  huart->ErrorCode |= DAL_UART_ERROR_DMA;
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  /*Call registered error callback*/
  huart->ErrorCallback(huart);
#else
  /*Call legacy weak error callback*/
  DAL_UART_ErrorCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief  This function handles UART Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  Flag specifies the UART flag to check.
  * @param  Status The actual Flag status (SET or RESET).
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
static DAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                                     uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while ((__DAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != DAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((DAL_GetTick() - Tickstart) > Timeout))
      {
        /* Disable TXBEIEN, RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts for the interrupt process */
        ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN | USART_CTRL1_TXBEIEN));
        ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

        huart->gState  = DAL_UART_STATE_READY;
        huart->RxState = DAL_UART_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(huart);

        return DAL_TIMEOUT;
      }
    }
  }
  return DAL_OK;
}

/**
  * @brief  Start Receive operation in interrupt mode.
  * @note   This function could be called by all DAL UART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @param  huart UART handle.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval DAL status
  */
DAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  huart->pRxBuffPtr = pData;
  huart->RxXferSize = Size;
  huart->RxXferCount = Size;

  huart->ErrorCode = DAL_UART_ERROR_NONE;
  huart->RxState = DAL_UART_STATE_BUSY_RX;

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  if (huart->Init.Parity != UART_PARITY_NONE)
  {
    /* Enable the UART Parity Error Interrupt */
    __DAL_UART_ENABLE_IT(huart, UART_IT_PE);
  }

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __DAL_UART_ENABLE_IT(huart, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __DAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

  return DAL_OK;
}

/**
  * @brief  Start Receive operation in DMA mode.
  * @note   This function could be called by all DAL UART API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @param  huart UART handle.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval DAL status
  */
DAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  uint32_t *tmp;

  huart->pRxBuffPtr = pData;
  huart->RxXferSize = Size;

  huart->ErrorCode = DAL_UART_ERROR_NONE;
  huart->RxState = DAL_UART_STATE_BUSY_RX;

  /* Set the UART DMA transfer complete callback */
  huart->hdmarx->XferCpltCallback = UART_DMAReceiveCplt;

  /* Set the UART DMA Half transfer complete callback */
  huart->hdmarx->XferHalfCpltCallback = UART_DMARxHalfCplt;

  /* Set the DMA error callback */
  huart->hdmarx->XferErrorCallback = UART_DMAError;

  /* Set the DMA abort callback */
  huart->hdmarx->XferAbortCallback = NULL;

  /* Enable the DMA stream */
  tmp = (uint32_t *)&pData;
  DAL_DMA_Start_IT(huart->hdmarx, (uint32_t)&huart->Instance->DATA, *(uint32_t *)tmp, Size);

  /* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
  __DAL_UART_CLEAR_OREFLAG(huart);

  /* Process Unlocked */
  __DAL_UNLOCK(huart);

  if (huart->Init.Parity != UART_PARITY_NONE)
  {
    /* Enable the UART Parity Error Interrupt */
    ATOMIC_SET_BIT(huart->Instance->CTRL1, USART_CTRL1_PEIEN);
  }

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  ATOMIC_SET_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* Enable the DMA transfer for the receiver request by setting the DMAR bit
  in the UART CTRL3 register */
  ATOMIC_SET_BIT(huart->Instance->CTRL3, USART_CTRL3_DMARXEN);

  return DAL_OK;
}

/**
  * @brief  End ongoing Tx transfer on UART peripheral (following error detection or Transmit completion).
  * @param  huart UART handle.
  * @retval None
  */
static void UART_EndTxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable TXBEIEN and TXCIEN interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_TXBEIEN | USART_CTRL1_TXCIEN));

  /* At end of Tx process, restore huart->gState to Ready */
  huart->gState = DAL_UART_STATE_READY;
}

/**
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param  huart UART handle.
  * @retval None
  */
static void UART_EndRxTransfer(UART_HandleTypeDef *huart)
{
  /* Disable RXBNEIEN, PEIEN and ERRIEN (Frame error, noise error, overrun error) interrupts */
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, (USART_CTRL1_RXBNEIEN | USART_CTRL1_PEIEN));
  ATOMIC_CLEAR_BIT(huart->Instance->CTRL3, USART_CTRL3_ERRIEN);

  /* In case of reception waiting for IDLE event, disable also the IDLE IE interrupt source */
  if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
  {
    ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_IDLEIEN);
  }

  /* At end of Rx process, restore huart->RxState to Ready */
  huart->RxState = DAL_UART_STATE_READY;
  huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;
}

/**
  * @brief  DMA UART communication abort callback, when initiated by DAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  huart->RxXferCount = 0x00U;
  huart->TxXferCount = 0x00U;

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  /*Call registered error callback*/
  huart->ErrorCallback(huart);
#else
  /*Call legacy weak error callback*/
  DAL_UART_ErrorCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA UART Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  huart->hdmatx->XferAbortCallback = NULL;

  /* Check if an Abort process is still ongoing */
  if (huart->hdmarx != NULL)
  {
    if (huart->hdmarx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  huart->TxXferCount = 0x00U;
  huart->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  huart->ErrorCode = DAL_UART_ERROR_NONE;

  /* Restore huart->gState and huart->RxState to Ready */
  huart->gState  = DAL_UART_STATE_READY;
  huart->RxState = DAL_UART_STATE_READY;
  huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

  /* Call user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  /* Call registered Abort complete callback */
  huart->AbortCpltCallback(huart);
#else
  /* Call legacy weak Abort complete callback */
  DAL_UART_AbortCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA UART Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  huart->hdmarx->XferAbortCallback = NULL;

  /* Check if an Abort process is still ongoing */
  if (huart->hdmatx != NULL)
  {
    if (huart->hdmatx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
  huart->TxXferCount = 0x00U;
  huart->RxXferCount = 0x00U;

  /* Reset ErrorCode */
  huart->ErrorCode = DAL_UART_ERROR_NONE;

  /* Restore huart->gState and huart->RxState to Ready */
  huart->gState  = DAL_UART_STATE_READY;
  huart->RxState = DAL_UART_STATE_READY;
  huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

  /* Call user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  /* Call registered Abort complete callback */
  huart->AbortCpltCallback(huart);
#else
  /* Call legacy weak Abort complete callback */
  DAL_UART_AbortCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA UART Tx communication abort callback, when initiated by user by a call to
  *         DAL_UART_AbortTransmit_IT API (Abort only Tx transfer)
  *         (This callback is executed at end of DMA Tx Abort procedure following user abort request,
  *         and leads to user Tx Abort Complete callback execution).
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMATxOnlyAbortCallback(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  huart->TxXferCount = 0x00U;

  /* Restore huart->gState to Ready */
  huart->gState = DAL_UART_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Transmit Complete Callback */
  huart->AbortTransmitCpltCallback(huart);
#else
  /* Call legacy weak Abort Transmit Complete Callback */
  DAL_UART_AbortTransmitCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA UART Rx communication abort callback, when initiated by user by a call to
  *         DAL_UART_AbortReceive_IT API (Abort only Rx transfer)
  *         (This callback is executed at end of DMA Rx Abort procedure following user abort request,
  *         and leads to user Rx Abort Complete callback execution).
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_DMARxOnlyAbortCallback(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  huart->RxXferCount = 0x00U;

  /* Restore huart->RxState to Ready */
  huart->RxState = DAL_UART_STATE_READY;
  huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

  /* Call user Abort complete callback */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  /* Call registered Abort Receive Complete Callback */
  huart->AbortReceiveCpltCallback(huart);
#else
  /* Call legacy weak Abort Receive Complete Callback */
  DAL_UART_AbortReceiveCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
static DAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart)
{
  const uint16_t *tmp;

  /* Check that a Tx process is ongoing */
  if (huart->gState == DAL_UART_STATE_BUSY_TX)
  {
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      tmp = (const uint16_t *) huart->pTxBuffPtr;
      huart->Instance->DATA = (uint16_t)(*tmp & (uint16_t)0x01FF);
      huart->pTxBuffPtr += 2U;
    }
    else
    {
      huart->Instance->DATA = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);
    }

    if (--huart->TxXferCount == 0U)
    {
      /* Disable the UART Transmit Data Register Empty Interrupt */
      __DAL_UART_DISABLE_IT(huart, UART_IT_TXE);

      /* Enable the UART Transmit Complete Interrupt */
      __DAL_UART_ENABLE_IT(huart, UART_IT_TC);
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
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
static DAL_StatusTypeDef UART_EndTransmit_IT(UART_HandleTypeDef *huart)
{
  /* Disable the UART Transmit Complete Interrupt */
  __DAL_UART_DISABLE_IT(huart, UART_IT_TC);

  /* Tx process is ended, restore huart->gState to Ready */
  huart->gState = DAL_UART_STATE_READY;

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
  /*Call registered Tx complete callback*/
  huart->TxCpltCallback(huart);
#else
  /*Call legacy weak Tx complete callback*/
  DAL_UART_TxCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */

  return DAL_OK;
}

/**
  * @brief  Receives an amount of data in non blocking mode
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval DAL status
  */
static DAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart)
{
  uint8_t  *pdata8bits;
  uint16_t *pdata16bits;

  /* Check that a Rx process is ongoing */
  if (huart->RxState == DAL_UART_STATE_BUSY_RX)
  {
    if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE))
    {
      pdata8bits  = NULL;
      pdata16bits = (uint16_t *) huart->pRxBuffPtr;
      *pdata16bits = (uint16_t)(huart->Instance->DATA & (uint16_t)0x01FF);
      huart->pRxBuffPtr += 2U;
    }
    else
    {
      pdata8bits = (uint8_t *) huart->pRxBuffPtr;
      pdata16bits  = NULL;

      if ((huart->Init.WordLength == UART_WORDLENGTH_9B) || ((huart->Init.WordLength == UART_WORDLENGTH_8B) && (huart->Init.Parity == UART_PARITY_NONE)))
      {
        *pdata8bits = (uint8_t)(huart->Instance->DATA & (uint8_t)0x00FF);
      }
      else
      {
        *pdata8bits = (uint8_t)(huart->Instance->DATA & (uint8_t)0x007F);
      }
      huart->pRxBuffPtr += 1U;
    }

    if (--huart->RxXferCount == 0U)
    {
      /* Disable the UART Data Register not empty Interrupt */
      __DAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

      /* Disable the UART Parity Error Interrupt */
      __DAL_UART_DISABLE_IT(huart, UART_IT_PE);

      /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
      __DAL_UART_DISABLE_IT(huart, UART_IT_ERR);

      /* Rx process is completed, restore huart->RxState to Ready */
      huart->RxState = DAL_UART_STATE_READY;

      /* Check current reception Mode :
         If Reception till IDLE event has been selected : */
      if (huart->ReceptionType == DAL_UART_RECEPTION_TOIDLE)
      {
        /* Set reception type to Standard */
        huart->ReceptionType = DAL_UART_RECEPTION_STANDARD;

        /* Disable IDLE interrupt */
        ATOMIC_CLEAR_BIT(huart->Instance->CTRL1, USART_CTRL1_IDLEIEN);

        /* Check if IDLE flag is set */
        if (__DAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
        {
          /* Clear IDLE flag in ISR */
          __DAL_UART_CLEAR_IDLEFLAG(huart);
        }

#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx Event callback*/
        huart->RxEventCallback(huart, huart->RxXferSize);
#else
        /*Call legacy weak Rx Event callback*/
        DAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
      }
      else
      {
        /* Standard reception API called */
#if (USE_DAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Rx complete callback*/
        huart->RxCpltCallback(huart);
#else
        /*Call legacy weak Rx complete callback*/
        DAL_UART_RxCpltCallback(huart);
#endif /* USE_DAL_UART_REGISTER_CALLBACKS */
      }

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
  * @brief  Configures the UART peripheral.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
static void UART_SetConfig(UART_HandleTypeDef *huart)
{
  uint32_t tmpreg;
  uint32_t pclk;

  /* Check the parameters */
  ASSERT_PARAM(IS_UART_BAUDRATE(huart->Init.BaudRate));
  ASSERT_PARAM(IS_UART_STOPBITS(huart->Init.StopBits));
  ASSERT_PARAM(IS_UART_PARITY(huart->Init.Parity));
  ASSERT_PARAM(IS_UART_MODE(huart->Init.Mode));

  /*-------------------------- USART CTRL2 Configuration -----------------------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits
     according to huart->Init.StopBits value */
  MODIFY_REG(huart->Instance->CTRL2, USART_CTRL2_STOPCFG, huart->Init.StopBits);

  /*-------------------------- USART CTRL1 Configuration -----------------------*/
  /* Configure the UART Word Length, Parity and mode:
     Set the M bits according to huart->Init.WordLength value
     Set PCE and PS bits according to huart->Init.Parity value
     Set TE and RE bits according to huart->Init.Mode value
     Set OVER8 bit according to huart->Init.OverSampling value */

  tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling;
  MODIFY_REG(huart->Instance->CTRL1,
             (uint32_t)(USART_CTRL1_DBLCFG | USART_CTRL1_PCEN | USART_CTRL1_PCFG | USART_CTRL1_TXEN | USART_CTRL1_RXEN | USART_CTRL1_OSMCFG),
             tmpreg);

  /*-------------------------- USART CTRL3 Configuration -----------------------*/
  /* Configure the UART HFC: Set CTSE and RTSE bits according to huart->Init.HwFlowCtl value */
  MODIFY_REG(huart->Instance->CTRL3, (USART_CTRL3_RTSEN | USART_CTRL3_CTSEN), huart->Init.HwFlowCtl);


#if defined(USART6) && defined(UART9) && defined(UART10)
    if ((huart->Instance == USART1) || (huart->Instance == USART6) || (huart->Instance == UART9) || (huart->Instance == UART10))
    {
      pclk = DAL_RCM_GetPCLK2Freq();
    }
#elif defined(USART6)
    if ((huart->Instance == USART1) || (huart->Instance == USART6))
    {
      pclk = DAL_RCM_GetPCLK2Freq();
    }
#else
    if (huart->Instance == USART1)
    {
      pclk = DAL_RCM_GetPCLK2Freq();
    }
#endif /* USART6 */
    else
    {
      pclk = DAL_RCM_GetPCLK1Freq();
    }
  /*-------------------------- USART BR Configuration ---------------------*/
  if (huart->Init.OverSampling == UART_OVERSAMPLING_8)
  {
    huart->Instance->BR = UART_BR_SAMPLING8(pclk, huart->Init.BaudRate);
  }
  else
  {
    huart->Instance->BR = UART_BR_SAMPLING16(pclk, huart->Init.BaudRate);
  }
}

/**
  * @}
  */

#endif /* DAL_UART_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

