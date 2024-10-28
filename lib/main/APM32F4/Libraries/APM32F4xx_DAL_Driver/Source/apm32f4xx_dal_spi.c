/**
  *
  * @file    apm32f4xx_dal_spi.c
  * @brief   SPI DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Serial Peripheral Interface (SPI) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
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
      The SPI DAL driver can be used as follows:

      (#) Declare a SPI_HandleTypeDef handle structure, for example:
          SPI_HandleTypeDef  hspi;

      (#)Initialize the SPI low level resources by implementing the DAL_SPI_MspInit() API:
          (##) Enable the SPIx interface clock
          (##) SPI pins configuration
              (+++) Enable the clock for the SPI GPIOs
              (+++) Configure these SPI pins as alternate function push-pull
          (##) NVIC configuration if you need to use interrupt process
              (+++) Configure the SPIx interrupt priority
              (+++) Enable the NVIC SPI IRQ handle
          (##) DMA Configuration if you need to use DMA process
              (+++) Declare a DMA_HandleTypeDef handle structure for the transmit or receive Stream/Channel
              (+++) Enable the DMAx clock
              (+++) Configure the DMA handle parameters
              (+++) Configure the DMA Tx or Rx Stream/Channel
              (+++) Associate the initialized hdma_tx(or _rx)  handle to the hspi DMA Tx or Rx handle
              (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the DMA Tx or Rx Stream/Channel

      (#) Program the Mode, BidirectionalMode , Data size, Baudrate Prescaler, NSS
          management, Clock polarity and phase, FirstBit and CRC configuration in the hspi Init structure.

      (#) Initialize the SPI registers by calling the DAL_SPI_Init() API:
          (++) This API configures also the low level Hardware GPIO, CLOCK, CORTEX...etc)
              by calling the customized DAL_SPI_MspInit() API.
     [..]
       Circular mode restriction:
      (#) The DMA circular mode cannot be used when the SPI is configured in these modes:
          (##) Master 2Lines RxOnly
          (##) Master 1Line Rx
      (#) The CRC feature is not managed when the DMA circular mode is enabled
      (#) When the SPI DMA Pause/Stop features are used, we must use the following APIs
          the DAL_SPI_DMAPause()/ DAL_SPI_DMAStop() only under the SPI callbacks
     [..]
       Master Receive mode restriction:
      (#) In Master unidirectional receive-only mode (MSTR =1, BIDIMODE=0, RXONLY=1) or
          bidirectional receive mode (MSTR=1, BIDIMODE=1, BIDIOE=0), to ensure that the SPI
          does not initiate a new transfer the following procedure has to be respected:
          (##) DAL_SPI_DeInit()
          (##) DAL_SPI_Init()
     [..]
       Callback registration:

      (#) The compilation flag USE_DAL_SPI_REGISTER_CALLBACKS when set to 1U
          allows the user to configure dynamically the driver callbacks.
          Use Functions DAL_SPI_RegisterCallback() to register an interrupt callback.

          Function DAL_SPI_RegisterCallback() allows to register following callbacks:
            (++) TxCpltCallback        : SPI Tx Completed callback
            (++) RxCpltCallback        : SPI Rx Completed callback
            (++) TxRxCpltCallback      : SPI TxRx Completed callback
            (++) TxHalfCpltCallback    : SPI Tx Half Completed callback
            (++) RxHalfCpltCallback    : SPI Rx Half Completed callback
            (++) TxRxHalfCpltCallback  : SPI TxRx Half Completed callback
            (++) ErrorCallback         : SPI Error callback
            (++) AbortCpltCallback     : SPI Abort callback
            (++) MspInitCallback       : SPI Msp Init callback
            (++) MspDeInitCallback     : SPI Msp DeInit callback
          This function takes as parameters the DAL peripheral handle, the Callback ID
          and a pointer to the user callback function.


      (#) Use function DAL_SPI_UnRegisterCallback to reset a callback to the default
          weak function.
          DAL_SPI_UnRegisterCallback takes as parameters the DAL peripheral handle,
          and the Callback ID.
          This function allows to reset following callbacks:
            (++) TxCpltCallback        : SPI Tx Completed callback
            (++) RxCpltCallback        : SPI Rx Completed callback
            (++) TxRxCpltCallback      : SPI TxRx Completed callback
            (++) TxHalfCpltCallback    : SPI Tx Half Completed callback
            (++) RxHalfCpltCallback    : SPI Rx Half Completed callback
            (++) TxRxHalfCpltCallback  : SPI TxRx Half Completed callback
            (++) ErrorCallback         : SPI Error callback
            (++) AbortCpltCallback     : SPI Abort callback
            (++) MspInitCallback       : SPI Msp Init callback
            (++) MspDeInitCallback     : SPI Msp DeInit callback

       [..]
       By default, after the DAL_SPI_Init() and when the state is DAL_SPI_STATE_RESET
       all callbacks are set to the corresponding weak functions:
       examples DAL_SPI_MasterTxCpltCallback(), DAL_SPI_MasterRxCpltCallback().
       Exception done for MspInit and MspDeInit functions that are
       reset to the legacy weak functions in the DAL_SPI_Init()/ DAL_SPI_DeInit() only when
       these callbacks are null (not registered beforehand).
       If MspInit or MspDeInit are not null, the DAL_SPI_Init()/ DAL_SPI_DeInit()
       keep and use the user MspInit/MspDeInit callbacks (registered beforehand) whatever the state.

       [..]
       Callbacks can be registered/unregistered in DAL_SPI_STATE_READY state only.
       Exception done MspInit/MspDeInit functions that can be registered/unregistered
       in DAL_SPI_STATE_READY or DAL_SPI_STATE_RESET state,
       thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
       Then, the user first registers the MspInit/MspDeInit user callbacks
       using DAL_SPI_RegisterCallback() before calling DAL_SPI_DeInit()
       or DAL_SPI_Init() function.

       [..]
       When the compilation define USE_DAL_PPP_REGISTER_CALLBACKS is set to 0 or
       not defined, the callback registering feature is not available
       and weak (surcharged) callbacks are used.

     [..]
       Using the DAL it is not possible to reach all supported SPI frequency with the different SPI Modes,
       the following table resume the max SPI frequency reached with data size 8bits/16bits,
         according to frequency of the APBx Peripheral Clock (fPCLK) used by the SPI instance.

  @endverbatim

  Additional table :

       DataSize = SPI_DATASIZE_8BIT:
       +----------------------------------------------------------------------------------------------+
       |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
       | Process | Transfer mode  |---------------------|----------------------|----------------------|
       |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
       |==============================================================================================|
       |    T    |     Polling    | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    /    |     Interrupt  | Fpclk/4  | Fpclk/8  |    NA     |    NA    |    NA     |   NA     |
       |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    X    |       DMA      | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    R    |     Interrupt  | Fpclk/8  | Fpclk/8  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |         |       DMA      | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/128 | Fpclk/2  |
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    | Fpclk/2  | Fpclk/4  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    T    |     Interrupt  | Fpclk/2  | Fpclk/4  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |         |       DMA      | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/128|
       +----------------------------------------------------------------------------------------------+

       DataSize = SPI_DATASIZE_16BIT:
       +----------------------------------------------------------------------------------------------+
       |         |                | 2Lines Fullduplex   |     2Lines RxOnly    |         1Line        |
       | Process | Transfer mode  |---------------------|----------------------|----------------------|
       |         |                |  Master  |  Slave   |  Master   |  Slave   |  Master   |  Slave   |
       |==============================================================================================|
       |    T    |     Polling    | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    /    |     Interrupt  | Fpclk/4  | Fpclk/4  |    NA     |    NA    |    NA     |   NA     |
       |    R    |----------------|----------|----------|-----------|----------|-----------|----------|
       |    X    |       DMA      | Fpclk/2  | Fpclk/2  |    NA     |    NA    |    NA     |   NA     |
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/32  | Fpclk/2  |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    R    |     Interrupt  | Fpclk/4  | Fpclk/4  | Fpclk/64  | Fpclk/2  | Fpclk/64  | Fpclk/2  |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |         |       DMA      | Fpclk/2  | Fpclk/2  | Fpclk/64  | Fpclk/2  | Fpclk/128 | Fpclk/2  |
       |=========|================|==========|==========|===========|==========|===========|==========|
       |         |     Polling    | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/32 |
       |         |----------------|----------|----------|-----------|----------|-----------|----------|
       |    T    |     Interrupt  | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/64 |
       |    X    |----------------|----------|----------|-----------|----------|-----------|----------|
       |         |       DMA      | Fpclk/2  | Fpclk/2  |     NA    |    NA    | Fpclk/2   | Fpclk/128|
       +----------------------------------------------------------------------------------------------+
       @note The max SPI frequency depend on SPI data size (8bits, 16bits),
             SPI mode(2 Lines fullduplex, 2 lines RxOnly, 1 line TX/RX) and Process mode (Polling, IT, DMA).
       @note
            (#) TX/RX processes are DAL_SPI_TransmitReceive(), DAL_SPI_TransmitReceive_IT() and DAL_SPI_TransmitReceive_DMA()
            (#) RX processes are DAL_SPI_Receive(), DAL_SPI_Receive_IT() and DAL_SPI_Receive_DMA()
            (#) TX processes are DAL_SPI_Transmit(), DAL_SPI_Transmit_IT() and DAL_SPI_Transmit_DMA()

  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup SPI SPI
  * @brief SPI DAL module driver
  * @{
  */
#ifdef DAL_SPI_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup SPI_Private_Constants SPI Private Constants
  * @{
  */
#define SPI_DEFAULT_TIMEOUT 100U
#define SPI_BSY_FLAG_WORKAROUND_TIMEOUT 1000U /*!< Timeout 1000 ��s             */
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup SPI_Private_Functions SPI Private Functions
  * @{
  */
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAHalfTransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_DMAError(DMA_HandleTypeDef *hdma);
static void SPI_DMAAbortOnError(DMA_HandleTypeDef *hdma);
static void SPI_DMATxAbortCallback(DMA_HandleTypeDef *hdma);
static void SPI_DMARxAbortCallback(DMA_HandleTypeDef *hdma);
static DAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus State,
                                                       uint32_t Timeout, uint32_t Tickstart);
static void SPI_TxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_TxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_RxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_RxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesTxISR_8BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesTxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_16BIT(struct __SPI_HandleTypeDef *hspi);
#if (USE_SPI_CRC != 0U)
static void SPI_RxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi);
static void SPI_RxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi);
static void SPI_2linesRxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi);
#endif /* USE_SPI_CRC */
static void SPI_AbortRx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_AbortTx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseRxTx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseRx_ISR(SPI_HandleTypeDef *hspi);
static void SPI_CloseTx_ISR(SPI_HandleTypeDef *hspi);
static DAL_StatusTypeDef SPI_EndRxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart);
static DAL_StatusTypeDef SPI_EndRxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */

/** @defgroup SPI_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SPIx peripheral:

      (+) User must implement DAL_SPI_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function DAL_SPI_Init() to configure the selected device with
          the selected configuration:
        (++) Mode
        (++) Direction
        (++) Data Size
        (++) Clock Polarity and Phase
        (++) NSS Management
        (++) BaudRate Prescaler
        (++) FirstBit
        (++) TIMode
        (++) CRC Calculation
        (++) CRC Polynomial if CRC enabled

      (+) Call the function DAL_SPI_DeInit() to restore the default configuration
          of the selected SPIx peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Init(SPI_HandleTypeDef *hspi)
{
  /* Check the SPI handle allocation */
  if (hspi == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_SPI_ALL_INSTANCE(hspi->Instance));
  ASSERT_PARAM(IS_SPI_MODE(hspi->Init.Mode));
  ASSERT_PARAM(IS_SPI_DIRECTION(hspi->Init.Direction));
  ASSERT_PARAM(IS_SPI_DATASIZE(hspi->Init.DataSize));
  ASSERT_PARAM(IS_SPI_NSS(hspi->Init.NSS));
  ASSERT_PARAM(IS_SPI_BAUDRATE_PRESCALER(hspi->Init.BaudRatePrescaler));
  ASSERT_PARAM(IS_SPI_FIRST_BIT(hspi->Init.FirstBit));
  ASSERT_PARAM(IS_SPI_TIMODE(hspi->Init.TIMode));
  if (hspi->Init.TIMode == SPI_TIMODE_DISABLE)
  {
    ASSERT_PARAM(IS_SPI_CPOL(hspi->Init.CLKPolarity));
    ASSERT_PARAM(IS_SPI_CPHA(hspi->Init.CLKPhase));

    if (hspi->Init.Mode == SPI_MODE_MASTER)
    {
      ASSERT_PARAM(IS_SPI_BAUDRATE_PRESCALER(hspi->Init.BaudRatePrescaler));
    }
    else
    {
      /* Baudrate prescaler not use in Motoraola Slave mode. force to default value */
      hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    }
  }
  else
  {
    ASSERT_PARAM(IS_SPI_BAUDRATE_PRESCALER(hspi->Init.BaudRatePrescaler));

    /* Force polarity and phase to TI protocaol requirements */
    hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi->Init.CLKPhase    = SPI_PHASE_1EDGE;
  }
#if (USE_SPI_CRC != 0U)
  ASSERT_PARAM(IS_SPI_CRC_CALCULATION(hspi->Init.CRCCalculation));
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    ASSERT_PARAM(IS_SPI_CRC_POLYNOMIAL(hspi->Init.CRCPolynomial));
  }
#else
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
#endif /* USE_SPI_CRC */

  if (hspi->State == DAL_SPI_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hspi->Lock = DAL_UNLOCKED;

#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
    /* Init the SPI Callback settings */
    hspi->TxCpltCallback       = DAL_SPI_TxCpltCallback;       /* Legacy weak TxCpltCallback       */
    hspi->RxCpltCallback       = DAL_SPI_RxCpltCallback;       /* Legacy weak RxCpltCallback       */
    hspi->TxRxCpltCallback     = DAL_SPI_TxRxCpltCallback;     /* Legacy weak TxRxCpltCallback     */
    hspi->TxHalfCpltCallback   = DAL_SPI_TxHalfCpltCallback;   /* Legacy weak TxHalfCpltCallback   */
    hspi->RxHalfCpltCallback   = DAL_SPI_RxHalfCpltCallback;   /* Legacy weak RxHalfCpltCallback   */
    hspi->TxRxHalfCpltCallback = DAL_SPI_TxRxHalfCpltCallback; /* Legacy weak TxRxHalfCpltCallback */
    hspi->ErrorCallback        = DAL_SPI_ErrorCallback;        /* Legacy weak ErrorCallback        */
    hspi->AbortCpltCallback    = DAL_SPI_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */

    if (hspi->MspInitCallback == NULL)
    {
      hspi->MspInitCallback = DAL_SPI_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    hspi->MspInitCallback(hspi);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    DAL_SPI_MspInit(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
  }

  hspi->State = DAL_SPI_STATE_BUSY;

  /* Disable the selected SPI peripheral */
  __DAL_SPI_DISABLE(hspi);

  /*----------------------- SPIx CTRL1 & CTRL2 Configuration ---------------------*/
  /* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
  Communication speed, First bit and CRC calculation state */
  WRITE_REG(hspi->Instance->CTRL1, ((hspi->Init.Mode & (SPI_CTRL1_MSMCFG | SPI_CTRL1_ISSEL)) |
                                  (hspi->Init.Direction & (SPI_CTRL1_RXOMEN | SPI_CTRL1_BMEN)) |
                                  (hspi->Init.DataSize & SPI_CTRL1_DFLSEL) |
                                  (hspi->Init.CLKPolarity & SPI_CTRL1_CPOL) |
                                  (hspi->Init.CLKPhase & SPI_CTRL1_CPHA) |
                                  (hspi->Init.NSS & SPI_CTRL1_SSEN) |
                                  (hspi->Init.BaudRatePrescaler & SPI_CTRL1_BRSEL_Msk) |
                                  (hspi->Init.FirstBit  & SPI_CTRL1_LSBSEL) |
                                  (hspi->Init.CRCCalculation & SPI_CTRL1_CRCEN)));

  /* Configure : NSS management, TI Mode */
  WRITE_REG(hspi->Instance->CTRL2, (((hspi->Init.NSS >> 16U) & SPI_CTRL2_SSOEN) | (hspi->Init.TIMode & SPI_CTRL2_FRFCFG)));

#if (USE_SPI_CRC != 0U)
  /*---------------------------- SPIx CRCPOLY Configuration ------------------*/
  /* Configure : CRC Polynomial */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    WRITE_REG(hspi->Instance->CRCPOLY, (hspi->Init.CRCPolynomial & SPI_CRCPOLY_CRCPOLY_Msk));
  }
#endif /* USE_SPI_CRC */

#if defined(SPI_I2SCFG_MODESEL)
  /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
  CLEAR_BIT(hspi->Instance->I2SCFG, SPI_I2SCFG_MODESEL);
#endif /* SPI_I2SCFG_MODESEL */

  hspi->ErrorCode = DAL_SPI_ERROR_NONE;
  hspi->State     = DAL_SPI_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  De-Initialize the SPI peripheral.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_DeInit(SPI_HandleTypeDef *hspi)
{
  /* Check the SPI handle allocation */
  if (hspi == NULL)
  {
    return DAL_ERROR;
  }

  /* Check SPI Instance parameter */
  ASSERT_PARAM(IS_SPI_ALL_INSTANCE(hspi->Instance));

  hspi->State = DAL_SPI_STATE_BUSY;

  /* Disable the SPI Peripheral Clock */
  __DAL_SPI_DISABLE(hspi);

#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  if (hspi->MspDeInitCallback == NULL)
  {
    hspi->MspDeInitCallback = DAL_SPI_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
  hspi->MspDeInitCallback(hspi);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
  DAL_SPI_MspDeInit(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */

  hspi->ErrorCode = DAL_SPI_ERROR_NONE;
  hspi->State = DAL_SPI_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hspi);

  return DAL_OK;
}

/**
  * @brief  Initialize the SPI MSP.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_MspInit should be implemented in the user file
   */
}

/**
  * @brief  De-Initialize the SPI MSP.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_MspDeInit should be implemented in the user file
   */
}

#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User SPI Callback
  *         To be used instead of the weak predefined callback
  * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI.
  * @param  CallbackID ID of the callback to be registered
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_RegisterCallback(SPI_HandleTypeDef *hspi, DAL_SPI_CallbackIDTypeDef CallbackID,
                                           pSPI_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hspi->ErrorCode |= DAL_SPI_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hspi);

  if (DAL_SPI_STATE_READY == hspi->State)
  {
    switch (CallbackID)
    {
      case DAL_SPI_TX_COMPLETE_CB_ID :
        hspi->TxCpltCallback = pCallback;
        break;

      case DAL_SPI_RX_COMPLETE_CB_ID :
        hspi->RxCpltCallback = pCallback;
        break;

      case DAL_SPI_TX_RX_COMPLETE_CB_ID :
        hspi->TxRxCpltCallback = pCallback;
        break;

      case DAL_SPI_TX_HALF_COMPLETE_CB_ID :
        hspi->TxHalfCpltCallback = pCallback;
        break;

      case DAL_SPI_RX_HALF_COMPLETE_CB_ID :
        hspi->RxHalfCpltCallback = pCallback;
        break;

      case DAL_SPI_TX_RX_HALF_COMPLETE_CB_ID :
        hspi->TxRxHalfCpltCallback = pCallback;
        break;

      case DAL_SPI_ERROR_CB_ID :
        hspi->ErrorCallback = pCallback;
        break;

      case DAL_SPI_ABORT_CB_ID :
        hspi->AbortCpltCallback = pCallback;
        break;

      case DAL_SPI_MSPINIT_CB_ID :
        hspi->MspInitCallback = pCallback;
        break;

      case DAL_SPI_MSPDEINIT_CB_ID :
        hspi->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_SPI_STATE_RESET == hspi->State)
  {
    switch (CallbackID)
    {
      case DAL_SPI_MSPINIT_CB_ID :
        hspi->MspInitCallback = pCallback;
        break;

      case DAL_SPI_MSPDEINIT_CB_ID :
        hspi->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_INVALID_CALLBACK);

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hspi);
  return status;
}

/**
  * @brief  Unregister an SPI Callback
  *         SPI callback is redirected to the weak predefined callback
  * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI.
  * @param  CallbackID ID of the callback to be unregistered
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_UnRegisterCallback(SPI_HandleTypeDef *hspi, DAL_SPI_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hspi);

  if (DAL_SPI_STATE_READY == hspi->State)
  {
    switch (CallbackID)
    {
      case DAL_SPI_TX_COMPLETE_CB_ID :
        hspi->TxCpltCallback = DAL_SPI_TxCpltCallback;             /* Legacy weak TxCpltCallback       */
        break;

      case DAL_SPI_RX_COMPLETE_CB_ID :
        hspi->RxCpltCallback = DAL_SPI_RxCpltCallback;             /* Legacy weak RxCpltCallback       */
        break;

      case DAL_SPI_TX_RX_COMPLETE_CB_ID :
        hspi->TxRxCpltCallback = DAL_SPI_TxRxCpltCallback;         /* Legacy weak TxRxCpltCallback     */
        break;

      case DAL_SPI_TX_HALF_COMPLETE_CB_ID :
        hspi->TxHalfCpltCallback = DAL_SPI_TxHalfCpltCallback;     /* Legacy weak TxHalfCpltCallback   */
        break;

      case DAL_SPI_RX_HALF_COMPLETE_CB_ID :
        hspi->RxHalfCpltCallback = DAL_SPI_RxHalfCpltCallback;     /* Legacy weak RxHalfCpltCallback   */
        break;

      case DAL_SPI_TX_RX_HALF_COMPLETE_CB_ID :
        hspi->TxRxHalfCpltCallback = DAL_SPI_TxRxHalfCpltCallback; /* Legacy weak TxRxHalfCpltCallback */
        break;

      case DAL_SPI_ERROR_CB_ID :
        hspi->ErrorCallback = DAL_SPI_ErrorCallback;               /* Legacy weak ErrorCallback        */
        break;

      case DAL_SPI_ABORT_CB_ID :
        hspi->AbortCpltCallback = DAL_SPI_AbortCpltCallback;       /* Legacy weak AbortCpltCallback    */
        break;

      case DAL_SPI_MSPINIT_CB_ID :
        hspi->MspInitCallback = DAL_SPI_MspInit;                   /* Legacy weak MspInit              */
        break;

      case DAL_SPI_MSPDEINIT_CB_ID :
        hspi->MspDeInitCallback = DAL_SPI_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_SPI_STATE_RESET == hspi->State)
  {
    switch (CallbackID)
    {
      case DAL_SPI_MSPINIT_CB_ID :
        hspi->MspInitCallback = DAL_SPI_MspInit;                   /* Legacy weak MspInit              */
        break;

      case DAL_SPI_MSPDEINIT_CB_ID :
        hspi->MspDeInitCallback = DAL_SPI_MspDeInit;               /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_INVALID_CALLBACK);

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hspi);
  return status;
}
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group2 IO operation functions
  *  @brief   Data transfers functions
  *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the SPI
    data transfers.

    [..] The SPI supports master and slave mode :

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The DAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the DAL status.
            The end of the data processing will be indicated through the
            dedicated SPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The DAL_SPI_TxCpltCallback(), DAL_SPI_RxCpltCallback() and DAL_SPI_TxRxCpltCallback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The DAL_SPI_ErrorCallback()user callback will be executed when a communication error is detected

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1Line (simplex) and 2Lines (full duplex) modes.

@endverbatim
  * @{
  */

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;
  DAL_StatusTypeDef errorcode = DAL_OK;
  uint16_t initial_TxXferCount;

  /* Check Direction parameter */
  ASSERT_PARAM(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));

  /* Process Locked */
  __DAL_LOCK(hspi);

  /* Init tickstart for timeout management*/
  tickstart = DAL_GetTick();
  initial_TxXferCount = Size;

  if (hspi->State != DAL_SPI_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  /* Set the transaction information */
  hspi->State       = DAL_SPI_STATE_BUSY_TX;
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;

  /*Init field not used in handle to zero */
  hspi->pRxBuffPtr  = (uint8_t *)NULL;
  hspi->RxXferSize  = 0U;
  hspi->RxXferCount = 0U;
  hspi->TxISR       = NULL;
  hspi->RxISR       = NULL;

  /* Configure communication direction : 1Line */
  if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
  {
    /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
    __DAL_SPI_DISABLE(hspi);
    SPI_1LINE_TX(hspi);
  }

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }

  /* Transmit data in 16 Bit mode */
  if (hspi->Init.DataSize == SPI_DATASIZE_16BIT)
  {
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
    {
      hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
      hspi->pTxBuffPtr += sizeof(uint16_t);
      hspi->TxXferCount--;
    }
    /* Transmit data in 16 Bit mode */
    while (hspi->TxXferCount > 0U)
    {
      /* Wait until TXE flag is set to send data */
      if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
      {
        hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr += sizeof(uint16_t);
        hspi->TxXferCount--;
      }
      else
      {
        /* Timeout management */
        if ((((DAL_GetTick() - tickstart) >=  Timeout) && (Timeout != DAL_MAX_DELAY)) || (Timeout == 0U))
        {
          errorcode = DAL_TIMEOUT;
          goto error;
        }
      }
    }
  }
  /* Transmit data in 8 Bit mode */
  else
  {
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
    {
      *((__IO uint8_t *)&hspi->Instance->DATA) = (*hspi->pTxBuffPtr);
      hspi->pTxBuffPtr += sizeof(uint8_t);
      hspi->TxXferCount--;
    }
    while (hspi->TxXferCount > 0U)
    {
      /* Wait until TXE flag is set to send data */
      if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
      {
        *((__IO uint8_t *)&hspi->Instance->DATA) = (*hspi->pTxBuffPtr);
        hspi->pTxBuffPtr += sizeof(uint8_t);
        hspi->TxXferCount--;
      }
      else
      {
        /* Timeout management */
        if ((((DAL_GetTick() - tickstart) >=  Timeout) && (Timeout != DAL_MAX_DELAY)) || (Timeout == 0U))
        {
          errorcode = DAL_TIMEOUT;
          goto error;
        }
      }
    }
  }
#if (USE_SPI_CRC != 0U)
  /* Enable CRC Transmission */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
  }
#endif /* USE_SPI_CRC */

  /* Check the end of the transaction */
  if (SPI_EndRxTxTransaction(hspi, Timeout, tickstart) != DAL_OK)
  {
    hspi->ErrorCode = DAL_SPI_ERROR_FLAG;
  }

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __DAL_SPI_CLEAR_OVRFLAG(hspi);
  }

  if (hspi->ErrorCode != DAL_SPI_ERROR_NONE)
  {
    errorcode = DAL_ERROR;
  }

error:
  hspi->State = DAL_SPI_STATE_READY;
  /* Process Unlocked */
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be received
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
#if (USE_SPI_CRC != 0U)
  __IO uint32_t tmpreg = 0U;
#endif /* USE_SPI_CRC */
  uint32_t tickstart;
  DAL_StatusTypeDef errorcode = DAL_OK;

  if ((hspi->Init.Mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES))
  {
    hspi->State = DAL_SPI_STATE_BUSY_RX;
    /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
    return DAL_SPI_TransmitReceive(hspi, pData, pData, Size, Timeout);
  }

  /* Process Locked */
  __DAL_LOCK(hspi);

  /* Init tickstart for timeout management*/
  tickstart = DAL_GetTick();

  if (hspi->State != DAL_SPI_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  /* Set the transaction information */
  hspi->State       = DAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /*Init field not used in handle to zero */
  hspi->pTxBuffPtr  = (uint8_t *)NULL;
  hspi->TxXferSize  = 0U;
  hspi->TxXferCount = 0U;
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
    /* this is done to handle the CRCNEXT before the latest data */
    hspi->RxXferCount--;
  }
#endif /* USE_SPI_CRC */

  /* Configure communication direction: 1Line */
  if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
  {
    /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
    __DAL_SPI_DISABLE(hspi);
    SPI_1LINE_RX(hspi);
  }

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }

  /* Receive data in 8 Bit mode */
  if (hspi->Init.DataSize == SPI_DATASIZE_8BIT)
  {
    /* Transfer loop */
    while (hspi->RxXferCount > 0U)
    {
      /* Check the RXNE flag */
      if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
      {
        /* read the received data */
        (* (uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DATA;
        hspi->pRxBuffPtr += sizeof(uint8_t);
        hspi->RxXferCount--;
      }
      else
      {
        /* Timeout management */
        if ((((DAL_GetTick() - tickstart) >=  Timeout) && (Timeout != DAL_MAX_DELAY)) || (Timeout == 0U))
        {
          errorcode = DAL_TIMEOUT;
          goto error;
        }
      }
    }
  }
  else
  {
    /* Transfer loop */
    while (hspi->RxXferCount > 0U)
    {
      /* Check the RXNE flag */
      if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE))
      {
        *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DATA;
        hspi->pRxBuffPtr += sizeof(uint16_t);
        hspi->RxXferCount--;
      }
      else
      {
        /* Timeout management */
        if ((((DAL_GetTick() - tickstart) >=  Timeout) && (Timeout != DAL_MAX_DELAY)) || (Timeout == 0U))
        {
          errorcode = DAL_TIMEOUT;
          goto error;
        }
      }
    }
  }

#if (USE_SPI_CRC != 0U)
  /* Handle the CRC Transmission */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    /* freeze the CRC before the latest data */
    SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);

    /* Read the latest data */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != DAL_OK)
    {
      /* the latest data has not been received */
      errorcode = DAL_TIMEOUT;
      goto error;
    }

    /* Receive last data in 16 Bit mode */
    if (hspi->Init.DataSize == SPI_DATASIZE_16BIT)
    {
      *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DATA;
    }
    /* Receive last data in 8 Bit mode */
    else
    {
      (*(uint8_t *)hspi->pRxBuffPtr) = *(__IO uint8_t *)&hspi->Instance->DATA;
    }

    /* Wait the CRC data */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != DAL_OK)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
      errorcode = DAL_TIMEOUT;
      goto error;
    }

    /* Read CRC to Flush DATA and RXNE flag */
    tmpreg = READ_REG(hspi->Instance->DATA);
    /* To avoid GCC warning */
    UNUSED(tmpreg);
  }
#endif /* USE_SPI_CRC */

  /* Check the end of the transaction */
  if (SPI_EndRxTransaction(hspi, Timeout, tickstart) != DAL_OK)
  {
    hspi->ErrorCode = DAL_SPI_ERROR_FLAG;
  }

#if (USE_SPI_CRC != 0U)
  /* Check if CRC error occurred */
  if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
  {
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
    __DAL_SPI_CLEAR_CRCERRFLAG(hspi);
  }
#endif /* USE_SPI_CRC */

  if (hspi->ErrorCode != DAL_SPI_ERROR_NONE)
  {
    errorcode = DAL_ERROR;
  }

error :
  hspi->State = DAL_SPI_STATE_READY;
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @param  Timeout Timeout duration
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout)
{
  uint16_t             initial_TxXferCount;
  uint32_t             tmp_mode;
  DAL_SPI_StateTypeDef tmp_state;
  uint32_t             tickstart;
#if (USE_SPI_CRC != 0U)
  __IO uint32_t tmpreg = 0U;
#endif /* USE_SPI_CRC */

  /* Variable used to alternate Rx and Tx during transfer */
  uint32_t             txallowed = 1U;
  DAL_StatusTypeDef    errorcode = DAL_OK;

  /* Check Direction parameter */
  ASSERT_PARAM(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

  /* Process Locked */
  __DAL_LOCK(hspi);

  /* Init tickstart for timeout management*/
  tickstart = DAL_GetTick();

  /* Init temporary variables */
  tmp_state           = hspi->State;
  tmp_mode            = hspi->Init.Mode;
  initial_TxXferCount = Size;

  if (!((tmp_state == DAL_SPI_STATE_READY) || \
        ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == DAL_SPI_STATE_BUSY_RX))))
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  /* Don't overwrite in case of DAL_SPI_STATE_BUSY_RX */
  if (hspi->State != DAL_SPI_STATE_BUSY_RX)
  {
    hspi->State = DAL_SPI_STATE_BUSY_TX_RX;
  }

  /* Set the transaction information */
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferCount = Size;
  hspi->RxXferSize  = Size;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->TxXferCount = Size;
  hspi->TxXferSize  = Size;

  /*Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }

  /* Transmit and Receive data in 16 Bit mode */
  if (hspi->Init.DataSize == SPI_DATASIZE_16BIT)
  {
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
    {
      hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
      hspi->pTxBuffPtr += sizeof(uint16_t);
      hspi->TxXferCount--;
    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U))
    {
      /* Check TXE flag */
      if ((__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) && (hspi->TxXferCount > 0U) && (txallowed == 1U))
      {
        hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
        hspi->pTxBuffPtr += sizeof(uint16_t);
        hspi->TxXferCount--;
        /* Next Data is a reception (Rx). Tx not allowed */
        txallowed = 0U;

#if (USE_SPI_CRC != 0U)
        /* Enable CRC Transmission */
        if ((hspi->TxXferCount == 0U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
        {
          SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
        }
#endif /* USE_SPI_CRC */
      }

      /* Check RXNE flag */
      if ((__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) && (hspi->RxXferCount > 0U))
      {
        *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DATA;
        hspi->pRxBuffPtr += sizeof(uint16_t);
        hspi->RxXferCount--;
        /* Next Data is a Transmission (Tx). Tx is allowed */
        txallowed = 1U;
      }
      if (((DAL_GetTick() - tickstart) >=  Timeout) && (Timeout != DAL_MAX_DELAY))
      {
        errorcode = DAL_TIMEOUT;
        goto error;
      }
    }
  }
  /* Transmit and Receive data in 8 Bit mode */
  else
  {
    if ((hspi->Init.Mode == SPI_MODE_SLAVE) || (initial_TxXferCount == 0x01U))
    {
      *((__IO uint8_t *)&hspi->Instance->DATA) = (*hspi->pTxBuffPtr);
      hspi->pTxBuffPtr += sizeof(uint8_t);
      hspi->TxXferCount--;
    }
    while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U))
    {
      /* Check TXE flag */
      if ((__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) && (hspi->TxXferCount > 0U) && (txallowed == 1U))
      {
        *(__IO uint8_t *)&hspi->Instance->DATA = (*hspi->pTxBuffPtr);
        hspi->pTxBuffPtr++;
        hspi->TxXferCount--;
        /* Next Data is a reception (Rx). Tx not allowed */
        txallowed = 0U;

#if (USE_SPI_CRC != 0U)
        /* Enable CRC Transmission */
        if ((hspi->TxXferCount == 0U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
        {
          SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
        }
#endif /* USE_SPI_CRC */
      }

      /* Wait until RXNE flag is reset */
      if ((__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) && (hspi->RxXferCount > 0U))
      {
        (*(uint8_t *)hspi->pRxBuffPtr) = hspi->Instance->DATA;
        hspi->pRxBuffPtr++;
        hspi->RxXferCount--;
        /* Next Data is a Transmission (Tx). Tx is allowed */
        txallowed = 1U;
      }
      if ((((DAL_GetTick() - tickstart) >=  Timeout) && ((Timeout != DAL_MAX_DELAY))) || (Timeout == 0U))
      {
        errorcode = DAL_TIMEOUT;
        goto error;
      }
    }
  }

#if (USE_SPI_CRC != 0U)
  /* Read CRC from DATA to close CRC calculation process */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    /* Wait until TXE flag */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, Timeout, tickstart) != DAL_OK)
    {
      /* Error on the CRC reception */
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
      errorcode = DAL_TIMEOUT;
      goto error;
    }
    /* Read CRC */
    tmpreg = READ_REG(hspi->Instance->DATA);
    /* To avoid GCC warning */
    UNUSED(tmpreg);
  }

  /* Check if CRC error occurred */
  if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
  {
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
    /* Clear CRC Flag */
    __DAL_SPI_CLEAR_CRCERRFLAG(hspi);

    errorcode = DAL_ERROR;
  }
#endif /* USE_SPI_CRC */

  /* Check the end of the transaction */
  if (SPI_EndRxTxTransaction(hspi, Timeout, tickstart) != DAL_OK)
  {
    errorcode = DAL_ERROR;
    hspi->ErrorCode = DAL_SPI_ERROR_FLAG;
    goto error;
  }

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __DAL_SPI_CLEAR_OVRFLAG(hspi);
  }

error :
  hspi->State = DAL_SPI_STATE_READY;
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  DAL_StatusTypeDef errorcode = DAL_OK;

  /* Check Direction parameter */
  ASSERT_PARAM(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));

  /* Process Locked */
  __DAL_LOCK(hspi);

  if ((pData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  if (hspi->State != DAL_SPI_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  /* Set the transaction information */
  hspi->State       = DAL_SPI_STATE_BUSY_TX;
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->pRxBuffPtr  = (uint8_t *)NULL;
  hspi->RxXferSize  = 0U;
  hspi->RxXferCount = 0U;
  hspi->RxISR       = NULL;

  /* Set the function for IT treatment */
  if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
  {
    hspi->TxISR = SPI_TxISR_16BIT;
  }
  else
  {
    hspi->TxISR = SPI_TxISR_8BIT;
  }

  /* Configure communication direction : 1Line */
  if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
  {
    /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
    __DAL_SPI_DISABLE(hspi);
    SPI_1LINE_TX(hspi);
  }

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

  /* Enable TXE and ERR interrupt */
  __DAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_ERR));


  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }

error :
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  DAL_StatusTypeDef errorcode = DAL_OK;

  if ((hspi->Init.Direction == SPI_DIRECTION_2LINES) && (hspi->Init.Mode == SPI_MODE_MASTER))
  {
    hspi->State = DAL_SPI_STATE_BUSY_RX;
    /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
    return DAL_SPI_TransmitReceive_IT(hspi, pData, pData, Size);
  }

  /* Process Locked */
  __DAL_LOCK(hspi);

  if (hspi->State != DAL_SPI_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  /* Set the transaction information */
  hspi->State       = DAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->pTxBuffPtr  = (uint8_t *)NULL;
  hspi->TxXferSize  = 0U;
  hspi->TxXferCount = 0U;
  hspi->TxISR       = NULL;

  /* Set the function for IT treatment */
  if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
  {
    hspi->RxISR = SPI_RxISR_16BIT;
  }
  else
  {
    hspi->RxISR = SPI_RxISR_8BIT;
  }

  /* Configure communication direction : 1Line */
  if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
  {
    /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
    __DAL_SPI_DISABLE(hspi);
    SPI_1LINE_RX(hspi);
  }

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

  /* Enable TXE and ERR interrupt */
  __DAL_SPI_ENABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

  /* Note : The SPI must be enabled after unlocking current process
            to avoid the risk of SPI interrupt handle execution before current
            process unlock */

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }

error :
  /* Process Unlocked */
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
  uint32_t             tmp_mode;
  DAL_SPI_StateTypeDef tmp_state;
  DAL_StatusTypeDef    errorcode = DAL_OK;

  /* Check Direction parameter */
  ASSERT_PARAM(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

  /* Process locked */
  __DAL_LOCK(hspi);

  /* Init temporary variables */
  tmp_state           = hspi->State;
  tmp_mode            = hspi->Init.Mode;

  if (!((tmp_state == DAL_SPI_STATE_READY) || \
        ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == DAL_SPI_STATE_BUSY_RX))))
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  /* Don't overwrite in case of DAL_SPI_STATE_BUSY_RX */
  if (hspi->State != DAL_SPI_STATE_BUSY_RX)
  {
    hspi->State = DAL_SPI_STATE_BUSY_TX_RX;
  }

  /* Set the transaction information */
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /* Set the function for IT treatment */
  if (hspi->Init.DataSize > SPI_DATASIZE_8BIT)
  {
    hspi->RxISR     = SPI_2linesRxISR_16BIT;
    hspi->TxISR     = SPI_2linesTxISR_16BIT;
  }
  else
  {
    hspi->RxISR     = SPI_2linesRxISR_8BIT;
    hspi->TxISR     = SPI_2linesTxISR_8BIT;
  }

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

  /* Enable TXE, RXNE and ERR interrupt */
  __DAL_SPI_ENABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }

error :
  /* Process Unlocked */
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Transmit an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  DAL_StatusTypeDef errorcode = DAL_OK;

  /* Check tx dma handle */
  ASSERT_PARAM(IS_SPI_DMA_HANDLE(hspi->hdmatx));

  /* Check Direction parameter */
  ASSERT_PARAM(IS_SPI_DIRECTION_2LINES_OR_1LINE(hspi->Init.Direction));

  /* Process Locked */
  __DAL_LOCK(hspi);

  if (hspi->State != DAL_SPI_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  /* Set the transaction information */
  hspi->State       = DAL_SPI_STATE_BUSY_TX;
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->pRxBuffPtr  = (uint8_t *)NULL;
  hspi->TxISR       = NULL;
  hspi->RxISR       = NULL;
  hspi->RxXferSize  = 0U;
  hspi->RxXferCount = 0U;

  /* Configure communication direction : 1Line */
  if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
  {
    /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
    __DAL_SPI_DISABLE(hspi);
    SPI_1LINE_TX(hspi);
  }

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

  /* Set the SPI TxDMA Half transfer complete callback */
  hspi->hdmatx->XferHalfCpltCallback = SPI_DMAHalfTransmitCplt;

  /* Set the SPI TxDMA transfer complete callback */
  hspi->hdmatx->XferCpltCallback = SPI_DMATransmitCplt;

  /* Set the DMA error callback */
  hspi->hdmatx->XferErrorCallback = SPI_DMAError;

  /* Set the DMA AbortCpltCallback */
  hspi->hdmatx->XferAbortCallback = NULL;

  /* Enable the Tx DMA Stream/Channel */
  if (DAL_OK != DAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DATA,
                                 hspi->TxXferCount))
  {
    /* Update SPI error code */
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_DMA);
    errorcode = DAL_ERROR;

    hspi->State = DAL_SPI_STATE_READY;
    goto error;
  }

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }

  /* Enable the SPI Error Interrupt Bit */
  __DAL_SPI_ENABLE_IT(hspi, (SPI_IT_ERR));

  /* Enable Tx DMA Request */
  SET_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN);

error :
  /* Process Unlocked */
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Receive an amount of data in non-blocking mode with DMA.
  * @note   In case of MASTER mode and SPI_DIRECTION_2LINES direction, hdmatx shall be defined.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @note   When the CRC feature is enabled the pData Length must be Size + 1.
  * @param  Size amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
  DAL_StatusTypeDef errorcode = DAL_OK;

  /* Check rx dma handle */
  ASSERT_PARAM(IS_SPI_DMA_HANDLE(hspi->hdmarx));

  if ((hspi->Init.Direction == SPI_DIRECTION_2LINES) && (hspi->Init.Mode == SPI_MODE_MASTER))
  {
    hspi->State = DAL_SPI_STATE_BUSY_RX;

    /* Check tx dma handle */
    ASSERT_PARAM(IS_SPI_DMA_HANDLE(hspi->hdmatx));

    /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
    return DAL_SPI_TransmitReceive_DMA(hspi, pData, pData, Size);
  }

  /* Process Locked */
  __DAL_LOCK(hspi);

  if (hspi->State != DAL_SPI_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  /* Set the transaction information */
  hspi->State       = DAL_SPI_STATE_BUSY_RX;
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pRxBuffPtr  = (uint8_t *)pData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /*Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;
  hspi->TxXferSize  = 0U;
  hspi->TxXferCount = 0U;

  /* Configure communication direction : 1Line */
  if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
  {
    /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
    __DAL_SPI_DISABLE(hspi);
    SPI_1LINE_RX(hspi);
  }

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

  /* Set the SPI RxDMA Half transfer complete callback */
  hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;

  /* Set the SPI Rx DMA transfer complete callback */
  hspi->hdmarx->XferCpltCallback = SPI_DMAReceiveCplt;

  /* Set the DMA error callback */
  hspi->hdmarx->XferErrorCallback = SPI_DMAError;

  /* Set the DMA AbortCpltCallback */
  hspi->hdmarx->XferAbortCallback = NULL;

  /* Enable the Rx DMA Stream/Channel  */
  if (DAL_OK != DAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DATA, (uint32_t)hspi->pRxBuffPtr,
                                 hspi->RxXferCount))
  {
    /* Update SPI error code */
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_DMA);
    errorcode = DAL_ERROR;

    hspi->State = DAL_SPI_STATE_READY;
    goto error;
  }

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }

  /* Enable the SPI Error Interrupt Bit */
  __DAL_SPI_ENABLE_IT(hspi, (SPI_IT_ERR));

  /* Enable Rx DMA Request */
  SET_BIT(hspi->Instance->CTRL2, SPI_CTRL2_RXDEN);

error:
  /* Process Unlocked */
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @note   When the CRC feature is enabled the pRxData Length must be Size + 1
  * @param  Size amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size)
{
  uint32_t             tmp_mode;
  DAL_SPI_StateTypeDef tmp_state;
  DAL_StatusTypeDef errorcode = DAL_OK;

  /* Check rx & tx dma handles */
  ASSERT_PARAM(IS_SPI_DMA_HANDLE(hspi->hdmarx));
  ASSERT_PARAM(IS_SPI_DMA_HANDLE(hspi->hdmatx));

  /* Check Direction parameter */
  ASSERT_PARAM(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

  /* Process locked */
  __DAL_LOCK(hspi);

  /* Init temporary variables */
  tmp_state           = hspi->State;
  tmp_mode            = hspi->Init.Mode;

  if (!((tmp_state == DAL_SPI_STATE_READY) ||
        ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == DAL_SPI_STATE_BUSY_RX))))
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
  {
    errorcode = DAL_ERROR;
    goto error;
  }

  /* Don't overwrite in case of DAL_SPI_STATE_BUSY_RX */
  if (hspi->State != DAL_SPI_STATE_BUSY_RX)
  {
    hspi->State = DAL_SPI_STATE_BUSY_TX_RX;
  }

  /* Set the transaction information */
  hspi->ErrorCode   = DAL_SPI_ERROR_NONE;
  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
  hspi->TxXferSize  = Size;
  hspi->TxXferCount = Size;
  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
  hspi->RxXferSize  = Size;
  hspi->RxXferCount = Size;

  /* Init field not used in handle to zero */
  hspi->RxISR       = NULL;
  hspi->TxISR       = NULL;

#if (USE_SPI_CRC != 0U)
  /* Reset CRC Calculation */
  if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
  {
    SPI_RESET_CRC(hspi);
  }
#endif /* USE_SPI_CRC */

  /* Check if we are in Rx only or in Rx/Tx Mode and configure the DMA transfer complete callback */
  if (hspi->State == DAL_SPI_STATE_BUSY_RX)
  {
    /* Set the SPI Rx DMA Half transfer complete callback */
    hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfReceiveCplt;
    hspi->hdmarx->XferCpltCallback     = SPI_DMAReceiveCplt;
  }
  else
  {
    /* Set the SPI Tx/Rx DMA Half transfer complete callback */
    hspi->hdmarx->XferHalfCpltCallback = SPI_DMAHalfTransmitReceiveCplt;
    hspi->hdmarx->XferCpltCallback     = SPI_DMATransmitReceiveCplt;
  }

  /* Set the DMA error callback */
  hspi->hdmarx->XferErrorCallback = SPI_DMAError;

  /* Set the DMA AbortCpltCallback */
  hspi->hdmarx->XferAbortCallback = NULL;

  /* Enable the Rx DMA Stream/Channel  */
  if (DAL_OK != DAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->DATA, (uint32_t)hspi->pRxBuffPtr,
                                 hspi->RxXferCount))
  {
    /* Update SPI error code */
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_DMA);
    errorcode = DAL_ERROR;

    hspi->State = DAL_SPI_STATE_READY;
    goto error;
  }

  /* Enable Rx DMA Request */
  SET_BIT(hspi->Instance->CTRL2, SPI_CTRL2_RXDEN);

  /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  is performed in DMA reception complete callback  */
  hspi->hdmatx->XferHalfCpltCallback = NULL;
  hspi->hdmatx->XferCpltCallback     = NULL;
  hspi->hdmatx->XferErrorCallback    = NULL;
  hspi->hdmatx->XferAbortCallback    = NULL;

  /* Enable the Tx DMA Stream/Channel  */
  if (DAL_OK != DAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->DATA,
                                 hspi->TxXferCount))
  {
    /* Update SPI error code */
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_DMA);
    errorcode = DAL_ERROR;

    hspi->State = DAL_SPI_STATE_READY;
    goto error;
  }

  /* Check if the SPI is already enabled */
  if ((hspi->Instance->CTRL1 & SPI_CTRL1_SPIEN) != SPI_CTRL1_SPIEN)
  {
    /* Enable SPI peripheral */
    __DAL_SPI_ENABLE(hspi);
  }
  /* Enable the SPI Error Interrupt Bit */
  __DAL_SPI_ENABLE_IT(hspi, (SPI_IT_ERR));

  /* Enable Tx DMA Request */
  SET_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN);

error :
  /* Process Unlocked */
  __DAL_UNLOCK(hspi);
  return errorcode;
}

/**
  * @brief  Abort ongoing transfer (blocking mode).
  * @param  hspi SPI handle.
  * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *         started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SPI Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Abort(SPI_HandleTypeDef *hspi)
{
  DAL_StatusTypeDef errorcode;
  __IO uint32_t count;
  __IO uint32_t resetcount;

  /* Initialized local variable  */
  errorcode = DAL_OK;
  resetcount = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);
  count = resetcount;

  /* Clear ERRIEN interrupt to avoid error interrupts generation during Abort procedure */
  CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_ERRIEN);

  /* Disable TXBEIEN, RXBNEIEN and ERRIEN(mode fault event, overrun error, TI frame error) interrupts */
  if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_TXBEIEN))
  {
    hspi->TxISR = SPI_AbortTx_ISR;
    /* Wait DAL_SPI_STATE_ABORT state */
    do
    {
      if (count == 0U)
      {
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
        break;
      }
      count--;
    } while (hspi->State != DAL_SPI_STATE_ABORT);
    /* Reset Timeout Counter */
    count = resetcount;
  }

  if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_RXBNEIEN))
  {
    hspi->RxISR = SPI_AbortRx_ISR;
    /* Wait DAL_SPI_STATE_ABORT state */
    do
    {
      if (count == 0U)
      {
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
        break;
      }
      count--;
    } while (hspi->State != DAL_SPI_STATE_ABORT);
    /* Reset Timeout Counter */
    count = resetcount;
  }

  /* Disable the SPI DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN))
  {
    /* Abort the SPI DMA Tx Stream/Channel : use blocking DMA Abort API (no callback) */
    if (hspi->hdmatx != NULL)
    {
      /* Set the SPI DMA Abort callback :
      will lead to call DAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
      hspi->hdmatx->XferAbortCallback = NULL;

      /* Abort DMA Tx Handle linked to SPI Peripheral */
      if (DAL_DMA_Abort(hspi->hdmatx) != DAL_OK)
      {
        hspi->ErrorCode = DAL_SPI_ERROR_ABORT;
      }

      /* Disable Tx DMA Request */
      CLEAR_BIT(hspi->Instance->CTRL2, (SPI_CTRL2_TXDEN));

      /* Wait until TXE flag is set */
      do
      {
        if (count == 0U)
        {
          SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
          break;
        }
        count--;
      } while ((hspi->Instance->STS & SPI_FLAG_TXE) == RESET);
    }
  }

  /* Disable the SPI DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_RXDEN))
  {
    /* Abort the SPI DMA Rx Stream/Channel : use blocking DMA Abort API (no callback) */
    if (hspi->hdmarx != NULL)
    {
      /* Set the SPI DMA Abort callback :
      will lead to call DAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
      hspi->hdmarx->XferAbortCallback = NULL;

      /* Abort DMA Rx Handle linked to SPI Peripheral */
      if (DAL_DMA_Abort(hspi->hdmarx) != DAL_OK)
      {
        hspi->ErrorCode = DAL_SPI_ERROR_ABORT;
      }

      /* Disable peripheral */
      __DAL_SPI_DISABLE(hspi);

      /* Disable Rx DMA Request */
      CLEAR_BIT(hspi->Instance->CTRL2, (SPI_CTRL2_RXDEN));
    }
  }
  /* Reset Tx and Rx transfer counters */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  /* Check error during Abort procedure */
  if (hspi->ErrorCode == DAL_SPI_ERROR_ABORT)
  {
    /* return DAL_Error in case of error during Abort procedure */
    errorcode = DAL_ERROR;
  }
  else
  {
    /* Reset errorCode */
    hspi->ErrorCode = DAL_SPI_ERROR_NONE;
  }

  /* Clear the Error flags in the SR register */
  __DAL_SPI_CLEAR_OVRFLAG(hspi);
  __DAL_SPI_CLEAR_FREFLAG(hspi);

  /* Restore hspi->state to ready */
  hspi->State = DAL_SPI_STATE_READY;

  return errorcode;
}

/**
  * @brief  Abort ongoing transfer (Interrupt mode).
  * @param  hspi SPI handle.
  * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *         started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SPI Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling DAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi)
{
  DAL_StatusTypeDef errorcode;
  uint32_t abortcplt ;
  __IO uint32_t count;
  __IO uint32_t resetcount;

  /* Initialized local variable  */
  errorcode = DAL_OK;
  abortcplt = 1U;
  resetcount = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);
  count = resetcount;

  /* Clear ERRIE interrupt to avoid error interrupts generation during Abort procedure */
  CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_ERRIEN);

  /* Change Rx and Tx Irq Handler to Disable TXEIE, RXNEIE and ERRIE interrupts */
  if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_TXBEIEN))
  {
    hspi->TxISR = SPI_AbortTx_ISR;
    /* Wait DAL_SPI_STATE_ABORT state */
    do
    {
      if (count == 0U)
      {
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
        break;
      }
      count--;
    } while (hspi->State != DAL_SPI_STATE_ABORT);
    /* Reset Timeout Counter */
    count = resetcount;
  }

  if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_RXBNEIEN))
  {
    hspi->RxISR = SPI_AbortRx_ISR;
    /* Wait DAL_SPI_STATE_ABORT state */
    do
    {
      if (count == 0U)
      {
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
        break;
      }
      count--;
    } while (hspi->State != DAL_SPI_STATE_ABORT);
    /* Reset Timeout Counter */
    count = resetcount;
  }

  /* If DMA Tx and/or DMA Rx Handles are associated to SPI Handle, DMA Abort complete callbacks should be initialised
     before any call to DMA Abort functions */
  /* DMA Tx Handle is valid */
  if (hspi->hdmatx != NULL)
  {
    /* Set DMA Abort Complete callback if UART DMA Tx request if enabled.
       Otherwise, set it to NULL */
    if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN))
    {
      hspi->hdmatx->XferAbortCallback = SPI_DMATxAbortCallback;
    }
    else
    {
      hspi->hdmatx->XferAbortCallback = NULL;
    }
  }
  /* DMA Rx Handle is valid */
  if (hspi->hdmarx != NULL)
  {
    /* Set DMA Abort Complete callback if UART DMA Rx request if enabled.
       Otherwise, set it to NULL */
    if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_RXDEN))
    {
      hspi->hdmarx->XferAbortCallback = SPI_DMARxAbortCallback;
    }
    else
    {
      hspi->hdmarx->XferAbortCallback = NULL;
    }
  }

  /* Disable the SPI DMA Tx request if enabled */
  if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN))
  {
    /* Abort the SPI DMA Tx Stream/Channel */
    if (hspi->hdmatx != NULL)
    {
      /* Abort DMA Tx Handle linked to SPI Peripheral */
      if (DAL_DMA_Abort_IT(hspi->hdmatx) != DAL_OK)
      {
        hspi->hdmatx->XferAbortCallback = NULL;
        hspi->ErrorCode = DAL_SPI_ERROR_ABORT;
      }
      else
      {
        abortcplt = 0U;
      }
    }
  }
  /* Disable the SPI DMA Rx request if enabled */
  if (DAL_IS_BIT_SET(hspi->Instance->CTRL2, SPI_CTRL2_RXDEN))
  {
    /* Abort the SPI DMA Rx Stream/Channel */
    if (hspi->hdmarx != NULL)
    {
      /* Abort DMA Rx Handle linked to SPI Peripheral */
      if (DAL_DMA_Abort_IT(hspi->hdmarx) !=  DAL_OK)
      {
        hspi->hdmarx->XferAbortCallback = NULL;
        hspi->ErrorCode = DAL_SPI_ERROR_ABORT;
      }
      else
      {
        abortcplt = 0U;
      }
    }
  }

  if (abortcplt == 1U)
  {
    /* Reset Tx and Rx transfer counters */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Check error during Abort procedure */
    if (hspi->ErrorCode == DAL_SPI_ERROR_ABORT)
    {
      /* return DAL_Error in case of error during Abort procedure */
      errorcode = DAL_ERROR;
    }
    else
    {
      /* Reset errorCode */
      hspi->ErrorCode = DAL_SPI_ERROR_NONE;
    }

    /* Clear the Error flags in the SR register */
    __DAL_SPI_CLEAR_OVRFLAG(hspi);
    __DAL_SPI_CLEAR_FREFLAG(hspi);

    /* Restore hspi->State to Ready */
    hspi->State = DAL_SPI_STATE_READY;

    /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->AbortCpltCallback(hspi);
#else
    DAL_SPI_AbortCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
  }

  return errorcode;
}

/**
  * @brief  Pause the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_DMAPause(SPI_HandleTypeDef *hspi)
{
  /* Process Locked */
  __DAL_LOCK(hspi);

  /* Disable the SPI DMA Tx & Rx requests */
  CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN);

  /* Process Unlocked */
  __DAL_UNLOCK(hspi);

  return DAL_OK;
}

/**
  * @brief  Resume the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_DMAResume(SPI_HandleTypeDef *hspi)
{
  /* Process Locked */
  __DAL_LOCK(hspi);

  /* Enable the SPI DMA Tx & Rx requests */
  SET_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN);

  /* Process Unlocked */
  __DAL_UNLOCK(hspi);

  return DAL_OK;
}

/**
  * @brief  Stop the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SPI_DMAStop(SPI_HandleTypeDef *hspi)
{
  DAL_StatusTypeDef errorcode = DAL_OK;
  /* The Lock is not implemented on this API to allow the user application
     to call the DAL SPI API under callbacks DAL_SPI_TxCpltCallback() or DAL_SPI_RxCpltCallback() or DAL_SPI_TxRxCpltCallback():
     when calling DAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
     and the correspond call back is executed DAL_SPI_TxCpltCallback() or DAL_SPI_RxCpltCallback() or DAL_SPI_TxRxCpltCallback()
     */

  /* Abort the SPI DMA tx Stream/Channel  */
  if (hspi->hdmatx != NULL)
  {
    if (DAL_OK != DAL_DMA_Abort(hspi->hdmatx))
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_DMA);
      errorcode = DAL_ERROR;
    }
  }
  /* Abort the SPI DMA rx Stream/Channel  */
  if (hspi->hdmarx != NULL)
  {
    if (DAL_OK != DAL_DMA_Abort(hspi->hdmarx))
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_DMA);
      errorcode = DAL_ERROR;
    }
  }

  /* Disable the SPI DMA Tx & Rx requests */
  CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN);
  hspi->State = DAL_SPI_STATE_READY;
  return errorcode;
}

/**
  * @brief  Handle SPI interrupt request.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval None
  */
void DAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi)
{
  uint32_t itsource = hspi->Instance->CTRL2;
  uint32_t itflag   = hspi->Instance->STS;

  /* SPI in mode Receiver ----------------------------------------------------*/
  if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_OVR) == RESET) &&
      (SPI_CHECK_FLAG(itflag, SPI_FLAG_RXNE) != RESET) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_RXNE) != RESET))
  {
    hspi->RxISR(hspi);
    return;
  }

  /* SPI in mode Transmitter -------------------------------------------------*/
  if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_TXE) != RESET) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_TXE) != RESET))
  {
    hspi->TxISR(hspi);
    return;
  }

  /* SPI in Error Treatment --------------------------------------------------*/
  if (((SPI_CHECK_FLAG(itflag, SPI_FLAG_MODF) != RESET) || (SPI_CHECK_FLAG(itflag, SPI_FLAG_OVR) != RESET)
       || (SPI_CHECK_FLAG(itflag, SPI_FLAG_FRE) != RESET)) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_ERR) != RESET))
  {
    /* SPI Overrun error interrupt occurred ----------------------------------*/
    if (SPI_CHECK_FLAG(itflag, SPI_FLAG_OVR) != RESET)
    {
      if (hspi->State != DAL_SPI_STATE_BUSY_TX)
      {
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_OVR);
        __DAL_SPI_CLEAR_OVRFLAG(hspi);
      }
      else
      {
        __DAL_SPI_CLEAR_OVRFLAG(hspi);
        return;
      }
    }

    /* SPI Mode Fault error interrupt occurred -------------------------------*/
    if (SPI_CHECK_FLAG(itflag, SPI_FLAG_MODF) != RESET)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_MODF);
      __DAL_SPI_CLEAR_MODFFLAG(hspi);
    }

    /* SPI Frame error interrupt occurred ------------------------------------*/
    if (SPI_CHECK_FLAG(itflag, SPI_FLAG_FRE) != RESET)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FRE);
      __DAL_SPI_CLEAR_FREFLAG(hspi);
    }

    if (hspi->ErrorCode != DAL_SPI_ERROR_NONE)
    {
      /* Disable all interrupts */
      __DAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);

      hspi->State = DAL_SPI_STATE_READY;
      /* Disable the SPI DMA requests if enabled */
      if ((DAL_IS_BIT_SET(itsource, SPI_CTRL2_TXDEN)) || (DAL_IS_BIT_SET(itsource, SPI_CTRL2_RXDEN)))
      {
        CLEAR_BIT(hspi->Instance->CTRL2, (SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN));

        /* Abort the SPI DMA Rx channel */
        if (hspi->hdmarx != NULL)
        {
          /* Set the SPI DMA Abort callback :
          will lead to call DAL_SPI_ErrorCallback() at end of DMA abort procedure */
          hspi->hdmarx->XferAbortCallback = SPI_DMAAbortOnError;
          if (DAL_OK != DAL_DMA_Abort_IT(hspi->hdmarx))
          {
            SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
          }
        }
        /* Abort the SPI DMA Tx channel */
        if (hspi->hdmatx != NULL)
        {
          /* Set the SPI DMA Abort callback :
          will lead to call DAL_SPI_ErrorCallback() at end of DMA abort procedure */
          hspi->hdmatx->XferAbortCallback = SPI_DMAAbortOnError;
          if (DAL_OK != DAL_DMA_Abort_IT(hspi->hdmatx))
          {
            SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
          }
        }
      }
      else
      {
        /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->ErrorCallback(hspi);
#else
        DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
      }
    }
    return;
  }
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_TxCpltCallback should be implemented in the user file
   */
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_RxCpltCallback should be implemented in the user file
   */
}

/**
  * @brief  Tx and Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_TxRxCpltCallback should be implemented in the user file
   */
}

/**
  * @brief  Tx Half Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_TxHalfCpltCallback should be implemented in the user file
   */
}

/**
  * @brief  Rx Half Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_RxHalfCpltCallback() should be implemented in the user file
   */
}

/**
  * @brief  Tx and Rx Half Transfer callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_TxRxHalfCpltCallback() should be implemented in the user file
   */
}

/**
  * @brief  SPI error callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void DAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_ErrorCallback should be implemented in the user file
   */
  /* NOTE : The ErrorCode parameter in the hspi handle is updated by the SPI processes
            and user can use DAL_SPI_GetError() API to check the latest error occurred
   */
}

/**
  * @brief  SPI Abort Complete callback.
  * @param  hspi SPI handle.
  * @retval None
  */
__weak void DAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_SPI_AbortCpltCallback can be implemented in the user file.
   */
}

/**
  * @}
  */

/** @defgroup SPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   SPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SPI.
     (+) DAL_SPI_GetState() API can be helpful to check in run-time the state of the SPI peripheral
     (+) DAL_SPI_GetError() check in run-time Errors occurring during communication
@endverbatim
  * @{
  */

/**
  * @brief  Return the SPI handle state.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI state
  */
DAL_SPI_StateTypeDef DAL_SPI_GetState(SPI_HandleTypeDef *hspi)
{
  /* Return SPI handle state */
  return hspi->State;
}

/**
  * @brief  Return the SPI error code.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI error code in bitmap format
  */
uint32_t DAL_SPI_GetError(SPI_HandleTypeDef *hspi)
{
  /* Return SPI ErrorCode */
  return hspi->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup SPI_Private_Functions
  * @brief   Private functions
  * @{
  */

/**
  * @brief  DMA SPI transmit process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
  uint32_t tickstart;

  /* Init tickstart for timeout management*/
  tickstart = DAL_GetTick();

  /* DMA Normal Mode */
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) != DMA_SCFGx_CIRCMEN)
  {
    /* Disable ERR interrupt */
    __DAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

    /* Disable Tx DMA Request */
    CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN);

    /* Check the end of the transaction */
    if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != DAL_OK)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
    }

    /* Clear overrun flag in 2 Lines communication mode because received data is not read */
    if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
    {
      __DAL_SPI_CLEAR_OVRFLAG(hspi);
    }

    hspi->TxXferCount = 0U;
    hspi->State = DAL_SPI_STATE_READY;

    if (hspi->ErrorCode != DAL_SPI_ERROR_NONE)
    {
      /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
      hspi->ErrorCallback(hspi);
#else
      DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
      return;
    }
  }
  /* Call user Tx complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->TxCpltCallback(hspi);
#else
  DAL_SPI_TxCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
  uint32_t tickstart;
#if (USE_SPI_CRC != 0U)
  __IO uint32_t tmpreg = 0U;
#endif /* USE_SPI_CRC */

  /* Init tickstart for timeout management*/
  tickstart = DAL_GetTick();

  /* DMA Normal Mode */
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) != DMA_SCFGx_CIRCMEN)
  {
    /* Disable ERR interrupt */
    __DAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

#if (USE_SPI_CRC != 0U)
    /* CRC handling */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait until RXNE flag */
      if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, SPI_DEFAULT_TIMEOUT, tickstart) != DAL_OK)
      {
        /* Error on the CRC reception */
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
      }
      /* Read CRC */
      tmpreg = READ_REG(hspi->Instance->DATA);
      /* To avoid GCC warning */
      UNUSED(tmpreg);
    }
#endif /* USE_SPI_CRC */

    /* Check if we are in Master RX 2 line mode */
    if ((hspi->Init.Direction == SPI_DIRECTION_2LINES) && (hspi->Init.Mode == SPI_MODE_MASTER))
    {
      /* Disable Rx/Tx DMA Request (done by default to handle the case master rx direction 2 lines) */
      CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN);
    }
    else
    {
      /* Normal case */
      CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_RXDEN);
    }

    /* Check the end of the transaction */
    if (SPI_EndRxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != DAL_OK)
    {
      hspi->ErrorCode = DAL_SPI_ERROR_FLAG;
    }

    hspi->RxXferCount = 0U;
    hspi->State = DAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
      __DAL_SPI_CLEAR_CRCERRFLAG(hspi);
    }
#endif /* USE_SPI_CRC */

    if (hspi->ErrorCode != DAL_SPI_ERROR_NONE)
    {
      /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
      hspi->ErrorCallback(hspi);
#else
      DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
      return;
    }
  }
  /* Call user Rx complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->RxCpltCallback(hspi);
#else
  DAL_SPI_RxCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI transmit receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
  uint32_t tickstart;
#if (USE_SPI_CRC != 0U)
  __IO uint32_t tmpreg = 0U;
#endif /* USE_SPI_CRC */

  /* Init tickstart for timeout management*/
  tickstart = DAL_GetTick();

  /* DMA Normal Mode */
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) != DMA_SCFGx_CIRCMEN)
  {
    /* Disable ERR interrupt */
    __DAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

#if (USE_SPI_CRC != 0U)
    /* CRC handling */
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Wait the CRC data */
      if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, SET, SPI_DEFAULT_TIMEOUT, tickstart) != DAL_OK)
      {
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
      }
      /* Read CRC to Flush DATA and RXNE flag */
      tmpreg = READ_REG(hspi->Instance->DATA);
      /* To avoid GCC warning */
      UNUSED(tmpreg);
    }
#endif /* USE_SPI_CRC */

    /* Check the end of the transaction */
    if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != DAL_OK)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
    }

    /* Disable Rx/Tx DMA Request */
    CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN);

    hspi->TxXferCount = 0U;
    hspi->RxXferCount = 0U;
    hspi->State = DAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
    /* Check if CRC error occurred */
    if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR))
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
      __DAL_SPI_CLEAR_CRCERRFLAG(hspi);
    }
#endif /* USE_SPI_CRC */

    if (hspi->ErrorCode != DAL_SPI_ERROR_NONE)
    {
      /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
      hspi->ErrorCallback(hspi);
#else
      DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
      return;
    }
  }
  /* Call user TxRx complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->TxRxCpltCallback(hspi);
#else
  DAL_SPI_TxRxCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI half transmit process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfTransmitCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

  /* Call user Tx half complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->TxHalfCpltCallback(hspi);
#else
  DAL_SPI_TxHalfCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI half receive process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

  /* Call user Rx half complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->RxHalfCpltCallback(hspi);
#else
  DAL_SPI_RxHalfCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI half transmit receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAHalfTransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

  /* Call user TxRx half complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->TxRxHalfCpltCallback(hspi);
#else
  DAL_SPI_TxRxHalfCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI communication error callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_DMAError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

  /* Stop the disable DMA transfer on SPI side */
  CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN);

  SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_DMA);
  hspi->State = DAL_SPI_STATE_READY;
  /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->ErrorCallback(hspi);
#else
  DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI communication abort callback, when initiated by DAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->ErrorCallback(hspi);
#else
  DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
  __IO uint32_t count;

  hspi->hdmatx->XferAbortCallback = NULL;
  count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  /* Disable Tx DMA Request */
  CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_TXDEN);

  /* Wait until TXE flag is set */
  do
  {
    if (count == 0U)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
      break;
    }
    count--;
  } while ((hspi->Instance->STS & SPI_FLAG_TXE) == RESET);

  /* Check if an Abort process is still ongoing */
  if (hspi->hdmarx != NULL)
  {
    if (hspi->hdmarx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA Stream/Channel are aborted, call user Abort Complete callback */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  /* Check no error during Abort procedure */
  if (hspi->ErrorCode != DAL_SPI_ERROR_ABORT)
  {
    /* Reset errorCode */
    hspi->ErrorCode = DAL_SPI_ERROR_NONE;
  }

  /* Clear the Error flags in the STS register */
  __DAL_SPI_CLEAR_OVRFLAG(hspi);
  __DAL_SPI_CLEAR_FREFLAG(hspi);

  /* Restore hspi->State to Ready */
  hspi->State  = DAL_SPI_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->AbortCpltCallback(hspi);
#else
  DAL_SPI_AbortCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
{
  SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

  /* Disable SPI Peripheral */
  __DAL_SPI_DISABLE(hspi);

  hspi->hdmarx->XferAbortCallback = NULL;

  /* Disable Rx DMA Request */
  CLEAR_BIT(hspi->Instance->CTRL2, SPI_CTRL2_RXDEN);

  /* Check Busy flag */
  if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, DAL_GetTick()) != DAL_OK)
  {
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
  }

  /* Check if an Abort process is still ongoing */
  if (hspi->hdmatx != NULL)
  {
    if (hspi->hdmatx->XferAbortCallback != NULL)
    {
      return;
    }
  }

  /* No Abort process still ongoing : All DMA Stream/Channel are aborted, call user Abort Complete callback */
  hspi->RxXferCount = 0U;
  hspi->TxXferCount = 0U;

  /* Check no error during Abort procedure */
  if (hspi->ErrorCode != DAL_SPI_ERROR_ABORT)
  {
    /* Reset errorCode */
    hspi->ErrorCode = DAL_SPI_ERROR_NONE;
  }

  /* Clear the Error flags in the STS register */
  __DAL_SPI_CLEAR_OVRFLAG(hspi);
  __DAL_SPI_CLEAR_FREFLAG(hspi);

  /* Restore hspi->State to Ready */
  hspi->State  = DAL_SPI_STATE_READY;

  /* Call user Abort complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
  hspi->AbortCpltCallback(hspi);
#else
  DAL_SPI_AbortCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  Rx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
  /* Receive data in 8bit mode */
  *hspi->pRxBuffPtr = *((__IO uint8_t *)&hspi->Instance->DATA);
  hspi->pRxBuffPtr++;
  hspi->RxXferCount--;

  /* Check end of the reception */
  if (hspi->RxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      hspi->RxISR =  SPI_2linesRxISR_8BITCRC;
      return;
    }
#endif /* USE_SPI_CRC */

    /* Disable RXNE  and ERR interrupt */
    __DAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

    if (hspi->TxXferCount == 0U)
    {
      SPI_CloseRxTx_ISR(hspi);
    }
  }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Rx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi)
{
  __IO uint8_t  *ptmpreg8;
  __IO uint8_t  tmpreg8 = 0;

  /* Initialize the 8bit temporary pointer */
  ptmpreg8 = (__IO uint8_t *)&hspi->Instance->DATA;
  /* Read 8bit CRC to flush Data Register */
  tmpreg8 = *ptmpreg8;
  /* To avoid GCC warning */
  UNUSED(tmpreg8);

  /* Disable RXNE and ERR interrupt */
  __DAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

  if (hspi->TxXferCount == 0U)
  {
    SPI_CloseRxTx_ISR(hspi);
  }
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Tx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesTxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
  *(__IO uint8_t *)&hspi->Instance->DATA = (*hspi->pTxBuffPtr);
  hspi->pTxBuffPtr++;
  hspi->TxXferCount--;

  /* Check the end of the transmission */
  if (hspi->TxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Set CRC Next Bit to send CRC */
      SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
      /* Disable TXE interrupt */
      __DAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);
      return;
    }
#endif /* USE_SPI_CRC */

    /* Disable TXE interrupt */
    __DAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

    if (hspi->RxXferCount == 0U)
    {
      SPI_CloseRxTx_ISR(hspi);
    }
  }
}

/**
  * @brief  Rx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
  /* Receive data in 16 Bit mode */
  *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)(hspi->Instance->DATA);
  hspi->pRxBuffPtr += sizeof(uint16_t);
  hspi->RxXferCount--;

  if (hspi->RxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      hspi->RxISR =  SPI_2linesRxISR_16BITCRC;
      return;
    }
#endif /* USE_SPI_CRC */

    /* Disable RXNE interrupt */
    __DAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE);

    if (hspi->TxXferCount == 0U)
    {
      SPI_CloseRxTx_ISR(hspi);
    }
  }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 16-bit receive for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesRxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi)
{
  __IO uint32_t tmpreg = 0U;

  /* Read 16bit CRC to flush Data Register */
  tmpreg = READ_REG(hspi->Instance->DATA);
  /* To avoid GCC warning */
  UNUSED(tmpreg);

  /* Disable RXNE interrupt */
  __DAL_SPI_DISABLE_IT(hspi, SPI_IT_RXNE);

  SPI_CloseRxTx_ISR(hspi);
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Tx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_2linesTxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
  /* Transmit data in 16 Bit mode */
  hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
  hspi->pTxBuffPtr += sizeof(uint16_t);
  hspi->TxXferCount--;

  /* Enable CRC Transmission */
  if (hspi->TxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Set CRC Next Bit to send CRC */
      SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
      /* Disable TXE interrupt */
      __DAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);
      return;
    }
#endif /* USE_SPI_CRC */

    /* Disable TXE interrupt */
    __DAL_SPI_DISABLE_IT(hspi, SPI_IT_TXE);

    if (hspi->RxXferCount == 0U)
    {
      SPI_CloseRxTx_ISR(hspi);
    }
  }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 8-bit receive in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_8BITCRC(struct __SPI_HandleTypeDef *hspi)
{
  __IO uint8_t  *ptmpreg8;
  __IO uint8_t  tmpreg8 = 0;

  /* Initialize the 8bit temporary pointer */
  ptmpreg8 = (__IO uint8_t *)&hspi->Instance->DATA;
  /* Read 8bit CRC to flush Data Register */
  tmpreg8 = *ptmpreg8;
  /* To avoid GCC warning */
  UNUSED(tmpreg8);

  SPI_CloseRx_ISR(hspi);
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Manage the receive 8-bit in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
  *hspi->pRxBuffPtr = (*(__IO uint8_t *)&hspi->Instance->DATA);
  hspi->pRxBuffPtr++;
  hspi->RxXferCount--;

#if (USE_SPI_CRC != 0U)
  /* Enable CRC Transmission */
  if ((hspi->RxXferCount == 1U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
  {
    SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
  }
#endif /* USE_SPI_CRC */

  if (hspi->RxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      hspi->RxISR =  SPI_RxISR_8BITCRC;
      return;
    }
#endif /* USE_SPI_CRC */
    SPI_CloseRx_ISR(hspi);
  }
}

#if (USE_SPI_CRC != 0U)
/**
  * @brief  Manage the CRC 16-bit receive in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_16BITCRC(struct __SPI_HandleTypeDef *hspi)
{
  __IO uint32_t tmpreg = 0U;

  /* Read 16bit CRC to flush Data Register */
  tmpreg = READ_REG(hspi->Instance->DATA);
  /* To avoid GCC warning */
  UNUSED(tmpreg);

  /* Disable RXNE and ERR interrupt */
  __DAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

  SPI_CloseRx_ISR(hspi);
}
#endif /* USE_SPI_CRC */

/**
  * @brief  Manage the 16-bit receive in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_RxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
  *((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)(hspi->Instance->DATA);
  hspi->pRxBuffPtr += sizeof(uint16_t);
  hspi->RxXferCount--;

#if (USE_SPI_CRC != 0U)
  /* Enable CRC Transmission */
  if ((hspi->RxXferCount == 1U) && (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE))
  {
    SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
  }
#endif /* USE_SPI_CRC */

  if (hspi->RxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      hspi->RxISR = SPI_RxISR_16BITCRC;
      return;
    }
#endif /* USE_SPI_CRC */
    SPI_CloseRx_ISR(hspi);
  }
}

/**
  * @brief  Handle the data 8-bit transmit in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_TxISR_8BIT(struct __SPI_HandleTypeDef *hspi)
{
  *(__IO uint8_t *)&hspi->Instance->DATA = (*hspi->pTxBuffPtr);
  hspi->pTxBuffPtr++;
  hspi->TxXferCount--;

  if (hspi->TxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Enable CRC Transmission */
      SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
    }
#endif /* USE_SPI_CRC */
    SPI_CloseTx_ISR(hspi);
  }
}

/**
  * @brief  Handle the data 16-bit transmit in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_TxISR_16BIT(struct __SPI_HandleTypeDef *hspi)
{
  /* Transmit data in 16 Bit mode */
  hspi->Instance->DATA = *((uint16_t *)hspi->pTxBuffPtr);
  hspi->pTxBuffPtr += sizeof(uint16_t);
  hspi->TxXferCount--;

  if (hspi->TxXferCount == 0U)
  {
#if (USE_SPI_CRC != 0U)
    if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
    {
      /* Enable CRC Transmission */
      SET_BIT(hspi->Instance->CTRL1, SPI_CTRL1_CRCNXT);
    }
#endif /* USE_SPI_CRC */
    SPI_CloseTx_ISR(hspi);
  }
}

/**
  * @brief  Handle SPI Communication Timeout.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param  Flag SPI flag to check
  * @param  State flag state to check
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus State,
                                                       uint32_t Timeout, uint32_t Tickstart)
{
  __IO uint32_t count;
  uint32_t tmp_timeout;
  uint32_t tmp_tickstart;

  /* Adjust Timeout value  in case of end of transfer */
  tmp_timeout   = Timeout - (DAL_GetTick() - Tickstart);
  tmp_tickstart = DAL_GetTick();

  /* Calculate Timeout based on a software loop to avoid blocking issue if Systick is disabled */
  count = tmp_timeout * ((SystemCoreClock * 32U) >> 20U);

  while ((__DAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) != State)
  {
    if (Timeout != DAL_MAX_DELAY)
    {
      if (((DAL_GetTick() - tmp_tickstart) >= tmp_timeout) || (tmp_timeout == 0U))
      {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
           on both master and slave sides in order to resynchronize the master
           and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __DAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                                                     || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
        {
          /* Disable SPI peripheral */
          __DAL_SPI_DISABLE(hspi);
        }

        /* Reset CRC Calculation */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SPI_RESET_CRC(hspi);
        }

        hspi->State = DAL_SPI_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(hspi);

        return DAL_TIMEOUT;
      }
      /* If Systick is disabled or not incremented, deactivate timeout to go in disable loop procedure */
      if (count == 0U)
      {
        tmp_timeout = 0U;
      }
      count--;
    }
  }

  return DAL_OK;
}

/**
  * @brief  Handle the check of the RX transaction complete.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef SPI_EndRxTransaction(SPI_HandleTypeDef *hspi,  uint32_t Timeout, uint32_t Tickstart)
{
  if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                                               || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
  {
    /* Disable SPI peripheral */
    __DAL_SPI_DISABLE(hspi);
  }

  /* Erratasheet: BSY bit may stay high at the end of a data transfer in Slave mode */
  if (hspi->Init.Mode == SPI_MODE_MASTER)
  {
    if (hspi->Init.Direction != SPI_DIRECTION_2LINES_RXONLY)
    {
      /* Control the BSY flag */
      if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != DAL_OK)
      {
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
        return DAL_TIMEOUT;
      }
    }
    else
    {
      /* Wait the RXNE reset */
      if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout, Tickstart) != DAL_OK)
      {
        SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
        return DAL_TIMEOUT;
      }
    }
  }
  else
  {
    /* Wait the RXNE reset */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_RXNE, RESET, Timeout, Tickstart) != DAL_OK)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
      return DAL_TIMEOUT;
    }
  }
  return DAL_OK;
}

/**
  * @brief  Handle the check of the RXTX or TX transaction complete.
  * @param  hspi SPI handle
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval DAL status
  */
static DAL_StatusTypeDef SPI_EndRxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart)
{
  /* Timeout in ��s */
  __IO uint32_t count = SPI_BSY_FLAG_WORKAROUND_TIMEOUT * (SystemCoreClock / 24U / 1000000U);
  /* Erratasheet: BSY bit may stay high at the end of a data transfer in Slave mode */
  if (hspi->Init.Mode == SPI_MODE_MASTER)
  {
    /* Control the BSY flag */
    if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, Timeout, Tickstart) != DAL_OK)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
      return DAL_TIMEOUT;
    }
  }
  else
  {
    /* Wait BSY flag during 1 Byte time transfer in case of Full-Duplex and Tx transfer
    * If Timeout is reached, the transfer is considered as finish.
    * User have to calculate the timeout value to fit with the time of 1 byte transfer.
    * This time is directly link with the SPI clock from Master device.
    */
    do
    {
      if (count == 0U)
      {
        break;
      }
      count--;
    } while (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY) != RESET);
  }

  return DAL_OK;
}

/**
  * @brief  Handle the end of the RXTX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseRxTx_ISR(SPI_HandleTypeDef *hspi)
{
  uint32_t tickstart;
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  /* Init tickstart for timeout management */
  tickstart = DAL_GetTick();

  /* Disable ERR interrupt */
  __DAL_SPI_DISABLE_IT(hspi, SPI_IT_ERR);

  /* Wait until TXE flag is set */
  do
  {
    if (count == 0U)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
      break;
    }
    count--;
  } while ((hspi->Instance->STS & SPI_FLAG_TXE) == RESET);

  /* Check the end of the transaction */
  if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != DAL_OK)
  {
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
  }

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __DAL_SPI_CLEAR_OVRFLAG(hspi);
  }

#if (USE_SPI_CRC != 0U)
  /* Check if CRC error occurred */
  if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET)
  {
    hspi->State = DAL_SPI_STATE_READY;
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
    __DAL_SPI_CLEAR_CRCERRFLAG(hspi);
    /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->ErrorCallback(hspi);
#else
    DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
  }
  else
  {
#endif /* USE_SPI_CRC */
    if (hspi->ErrorCode == DAL_SPI_ERROR_NONE)
    {
      if (hspi->State == DAL_SPI_STATE_BUSY_RX)
      {
        hspi->State = DAL_SPI_STATE_READY;
        /* Call user Rx complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->RxCpltCallback(hspi);
#else
        DAL_SPI_RxCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
      }
      else
      {
        hspi->State = DAL_SPI_STATE_READY;
        /* Call user TxRx complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->TxRxCpltCallback(hspi);
#else
        DAL_SPI_TxRxCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
      }
    }
    else
    {
      hspi->State = DAL_SPI_STATE_READY;
      /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
      hspi->ErrorCallback(hspi);
#else
      DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
    }
#if (USE_SPI_CRC != 0U)
  }
#endif /* USE_SPI_CRC */
}

/**
  * @brief  Handle the end of the RX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseRx_ISR(SPI_HandleTypeDef *hspi)
{
  /* Disable RXNE and ERR interrupt */
  __DAL_SPI_DISABLE_IT(hspi, (SPI_IT_RXNE | SPI_IT_ERR));

  /* Check the end of the transaction */
  if (SPI_EndRxTransaction(hspi, SPI_DEFAULT_TIMEOUT, DAL_GetTick()) != DAL_OK)
  {
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
  }

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __DAL_SPI_CLEAR_OVRFLAG(hspi);
  }
  hspi->State = DAL_SPI_STATE_READY;

#if (USE_SPI_CRC != 0U)
  /* Check if CRC error occurred */
  if (__DAL_SPI_GET_FLAG(hspi, SPI_FLAG_CRCERR) != RESET)
  {
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_CRC);
    __DAL_SPI_CLEAR_CRCERRFLAG(hspi);
    /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->ErrorCallback(hspi);
#else
    DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
  }
  else
  {
#endif /* USE_SPI_CRC */
    if (hspi->ErrorCode == DAL_SPI_ERROR_NONE)
    {
      /* Call user Rx complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
      hspi->RxCpltCallback(hspi);
#else
      DAL_SPI_RxCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
    }
    else
    {
      /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
      hspi->ErrorCallback(hspi);
#else
      DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
    }
#if (USE_SPI_CRC != 0U)
  }
#endif /* USE_SPI_CRC */
}

/**
  * @brief  Handle the end of the TX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_CloseTx_ISR(SPI_HandleTypeDef *hspi)
{
  uint32_t tickstart;
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  /* Init tickstart for timeout management*/
  tickstart = DAL_GetTick();

  /* Wait until TXE flag is set */
  do
  {
    if (count == 0U)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
      break;
    }
    count--;
  } while ((hspi->Instance->STS & SPI_FLAG_TXE) == RESET);

  /* Disable TXE and ERR interrupt */
  __DAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_ERR));

  /* Check the end of the transaction */
  if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != DAL_OK)
  {
    SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_FLAG);
  }

  /* Clear overrun flag in 2 Lines communication mode because received is not read */
  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
  {
    __DAL_SPI_CLEAR_OVRFLAG(hspi);
  }

  hspi->State = DAL_SPI_STATE_READY;
  if (hspi->ErrorCode != DAL_SPI_ERROR_NONE)
  {
    /* Call user error callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->ErrorCallback(hspi);
#else
    DAL_SPI_ErrorCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
  }
  else
  {
    /* Call user Rx complete callback */
#if (USE_DAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->TxCpltCallback(hspi);
#else
    DAL_SPI_TxCpltCallback(hspi);
#endif /* USE_DAL_SPI_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  Handle abort a Rx transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_AbortRx_ISR(SPI_HandleTypeDef *hspi)
{
  __IO uint32_t tmpreg = 0U;
  __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

  /* Wait until TXE flag is set */
  do
  {
    if (count == 0U)
    {
      SET_BIT(hspi->ErrorCode, DAL_SPI_ERROR_ABORT);
      break;
    }
    count--;
  } while ((hspi->Instance->STS & SPI_FLAG_TXE) == RESET);

  /* Disable SPI Peripheral */
  __DAL_SPI_DISABLE(hspi);

  /* Disable TXBEIEN, RXBNEIEN and ERRIEN(mode fault event, overrun error, TI frame error) interrupts */
  CLEAR_BIT(hspi->Instance->CTRL2, (SPI_CTRL2_TXBEIEN | SPI_CTRL2_RXBNEIEN | SPI_CTRL2_ERRIEN));

  /* Flush Data Register by a blank read */
  tmpreg = READ_REG(hspi->Instance->DATA);
  /* To avoid GCC warning */
  UNUSED(tmpreg);

  hspi->State = DAL_SPI_STATE_ABORT;
}

/**
  * @brief  Handle abort a Tx or Rx/Tx transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_AbortTx_ISR(SPI_HandleTypeDef *hspi)
{
  /* Disable TXBEIEN interrupt */
  CLEAR_BIT(hspi->Instance->CTRL2, (SPI_CTRL2_TXBEIEN));

  /* Disable SPI Peripheral */
  __DAL_SPI_DISABLE(hspi);

  hspi->State = DAL_SPI_STATE_ABORT;
}

/**
  * @}
  */

#endif /* DAL_SPI_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

