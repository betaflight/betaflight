/**
  *
  * @file    apm32f4xx_dal_i2s.c
  * @brief   I2S DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Integrated Interchip Sound (I2S) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and Errors functions
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
 ===============================================================================
                  ##### How to use this driver #####
 ===============================================================================
 [..]
    The I2S DAL driver can be used as follow:

    (#) Declare a I2S_HandleTypeDef handle structure.
    (#) Initialize the I2S low level resources by implement the DAL_I2S_MspInit() API:
        (##) Enable the SPIx interface clock.
        (##) I2S pins configuration:
            (+++) Enable the clock for the I2S GPIOs.
            (+++) Configure these I2S pins as alternate function pull-up.
        (##) NVIC configuration if you need to use interrupt process (DAL_I2S_Transmit_IT()
             and DAL_I2S_Receive_IT() APIs).
            (+++) Configure the I2Sx interrupt priority.
            (+++) Enable the NVIC I2S IRQ handle.
        (##) DMA Configuration if you need to use DMA process (DAL_I2S_Transmit_DMA()
             and DAL_I2S_Receive_DMA() APIs:
            (+++) Declare a DMA handle structure for the Tx/Rx Stream/Channel.
            (+++) Enable the DMAx interface clock.
            (+++) Configure the declared DMA handle structure with the required Tx/Rx parameters.
            (+++) Configure the DMA Tx/Rx Stream/Channel.
            (+++) Associate the initialized DMA handle to the I2S DMA Tx/Rx handle.
            (+++) Configure the priority and enable the NVIC for the transfer complete interrupt on the
                  DMA Tx/Rx Stream/Channel.

   (#) Program the Mode, Standard, Data Format, MCLK Output, Audio frequency and Polarity
       using DAL_I2S_Init() function.

   -@- The specific I2S interrupts (Transmission complete interrupt,
       RXNE interrupt and Error Interrupts) will be managed using the macros
       __DAL_I2S_ENABLE_IT() and __DAL_I2S_DISABLE_IT() inside the transmit and receive process.
   -@- Make sure that either:
        (+@) I2S PLL clock is configured or
        (+@) External clock source is configured after setting correctly
             the define constant EXTERNAL_CLOCK_VALUE in the apm32f4xx_dal_cfg.h file.

    (#) Three mode of operations are available within this driver :

   *** Polling mode IO operation ***
   =================================
   [..]
     (+) Send an amount of data in blocking mode using DAL_I2S_Transmit()
     (+) Receive an amount of data in blocking mode using DAL_I2S_Receive()

   *** Interrupt mode IO operation ***
   ===================================
   [..]
     (+) Send an amount of data in non blocking mode using DAL_I2S_Transmit_IT()
     (+) At transmission end of half transfer DAL_I2S_TxHalfCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_TxHalfCpltCallback
     (+) At transmission end of transfer DAL_I2S_TxCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_TxCpltCallback
     (+) Receive an amount of data in non blocking mode using DAL_I2S_Receive_IT()
     (+) At reception end of half transfer DAL_I2S_RxHalfCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_RxHalfCpltCallback
     (+) At reception end of transfer DAL_I2S_RxCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_RxCpltCallback
     (+) In case of transfer Error, DAL_I2S_ErrorCallback() function is executed and user can
         add his own code by customization of function pointer DAL_I2S_ErrorCallback

   *** DMA mode IO operation ***
   ==============================
   [..]
     (+) Send an amount of data in non blocking mode (DMA) using DAL_I2S_Transmit_DMA()
     (+) At transmission end of half transfer DAL_I2S_TxHalfCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_TxHalfCpltCallback
     (+) At transmission end of transfer DAL_I2S_TxCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_TxCpltCallback
     (+) Receive an amount of data in non blocking mode (DMA) using DAL_I2S_Receive_DMA()
     (+) At reception end of half transfer DAL_I2S_RxHalfCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_RxHalfCpltCallback
     (+) At reception end of transfer DAL_I2S_RxCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_RxCpltCallback
     (+) In case of transfer Error, DAL_I2S_ErrorCallback() function is executed and user can
         add his own code by customization of function pointer DAL_I2S_ErrorCallback
     (+) Pause the DMA Transfer using DAL_I2S_DMAPause()
     (+) Resume the DMA Transfer using DAL_I2S_DMAResume()
     (+) Stop the DMA Transfer using DAL_I2S_DMAStop()
         In Slave mode, if DAL_I2S_DMAStop is used to stop the communication, an error
         DAL_I2S_ERROR_BUSY_LINE_RX is raised as the master continue to transmit data.
         In this case __DAL_I2S_FLUSH_RX_DR macro must be used to flush the remaining data
         inside DATA register and avoid using DeInit/Init process for the next transfer.

   *** I2S DAL driver macros list ***
   ===================================
   [..]
     Below the list of most used macros in I2S DAL driver.

      (+) __DAL_I2S_ENABLE: Enable the specified SPI peripheral (in I2S mode)
      (+) __DAL_I2S_DISABLE: Disable the specified SPI peripheral (in I2S mode)
      (+) __DAL_I2S_ENABLE_IT : Enable the specified I2S interrupts
      (+) __DAL_I2S_DISABLE_IT : Disable the specified I2S interrupts
      (+) __DAL_I2S_GET_FLAG: Check whether the specified I2S flag is set or not
      (+) __DAL_I2S_FLUSH_RX_DR: Read DATA Register to Flush RX Data

    [..]
      (@) You can refer to the I2S DAL driver header file for more useful macros

   *** I2S DAL driver macros list ***
   ===================================
   [..]
       Callback registration:

      (#) The compilation flag USE_DAL_I2S_REGISTER_CALLBACKS when set to 1U
          allows the user to configure dynamically the driver callbacks.
          Use Functions DAL_I2S_RegisterCallback() to register an interrupt callback.

          Function DAL_I2S_RegisterCallback() allows to register following callbacks:
            (++) TxCpltCallback        : I2S Tx Completed callback
            (++) RxCpltCallback        : I2S Rx Completed callback
            (++) TxRxCpltCallback      : I2S TxRx Completed callback
            (++) TxHalfCpltCallback    : I2S Tx Half Completed callback
            (++) RxHalfCpltCallback    : I2S Rx Half Completed callback
            (++) ErrorCallback         : I2S Error callback
            (++) MspInitCallback       : I2S Msp Init callback
            (++) MspDeInitCallback     : I2S Msp DeInit callback
          This function takes as parameters the DAL peripheral handle, the Callback ID
          and a pointer to the user callback function.


      (#) Use function DAL_I2S_UnRegisterCallback to reset a callback to the default
          weak function.
          DAL_I2S_UnRegisterCallback takes as parameters the DAL peripheral handle,
          and the Callback ID.
          This function allows to reset following callbacks:
            (++) TxCpltCallback        : I2S Tx Completed callback
            (++) RxCpltCallback        : I2S Rx Completed callback
            (++) TxRxCpltCallback      : I2S TxRx Completed callback
            (++) TxHalfCpltCallback    : I2S Tx Half Completed callback
            (++) RxHalfCpltCallback    : I2S Rx Half Completed callback
            (++) ErrorCallback         : I2S Error callback
            (++) MspInitCallback       : I2S Msp Init callback
            (++) MspDeInitCallback     : I2S Msp DeInit callback

       [..]
       By default, after the DAL_I2S_Init() and when the state is DAL_I2S_STATE_RESET
       all callbacks are set to the corresponding weak functions:
       examples DAL_I2S_MasterTxCpltCallback(), DAL_I2S_MasterRxCpltCallback().
       Exception done for MspInit and MspDeInit functions that are
       reset to the legacy weak functions in the DAL_I2S_Init()/ DAL_I2S_DeInit() only when
       these callbacks are null (not registered beforehand).
       If MspInit or MspDeInit are not null, the DAL_I2S_Init()/ DAL_I2S_DeInit()
       keep and use the user MspInit/MspDeInit callbacks (registered beforehand) whatever the state.

       [..]
       Callbacks can be registered/unregistered in DAL_I2S_STATE_READY state only.
       Exception done MspInit/MspDeInit functions that can be registered/unregistered
       in DAL_I2S_STATE_READY or DAL_I2S_STATE_RESET state,
       thus registered (user) MspInit/DeInit callbacks can be used during the Init/DeInit.
       Then, the user first registers the MspInit/MspDeInit user callbacks
       using DAL_I2S_RegisterCallback() before calling DAL_I2S_DeInit()
       or DAL_I2S_Init() function.

       [..]
       When the compilation define USE_DAL_I2S_REGISTER_CALLBACKS is set to 0 or
       not defined, the callback registering feature is not available
       and weak (surcharged) callbacks are used.

  @endverbatim

  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

#ifdef DAL_I2S_MODULE_ENABLED

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup I2S I2S
  * @brief I2S DAL module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2S_TIMEOUT_FLAG          100U         /*!< Timeout 100 ms            */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup I2S_Private_Functions I2S Private Functions
  * @{
  */
static void               I2S_DMATxCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMATxHalfCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMARxCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMARxHalfCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMAError(DMA_HandleTypeDef *hdma);
static void               I2S_Transmit_IT(I2S_HandleTypeDef *hi2s);
static void               I2S_Receive_IT(I2S_HandleTypeDef *hi2s);
static void               I2S_IRQHandler(I2S_HandleTypeDef *hi2s);
static DAL_StatusTypeDef  I2S_WaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s, uint32_t Flag, FlagStatus State,
                                                        uint32_t Timeout);
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup I2S_Exported_Functions I2S Exported Functions
  * @{
  */

/** @defgroup  I2S_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the I2Sx peripheral in simplex mode:

      (+) User must Implement DAL_I2S_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function DAL_I2S_Init() to configure the selected device with
          the selected configuration:
        (++) Mode
        (++) Standard
        (++) Data Format
        (++) MCLK Output
        (++) Audio frequency
        (++) Polarity
        (++) Full duplex mode

     (+) Call the function DAL_I2S_DeInit() to restore the default configuration
          of the selected I2Sx peripheral.
  @endverbatim
  * @{
  */

/**
  * @brief  Initializes the I2S according to the specified parameters
  *         in the I2S_InitTypeDef and create the associated handle.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_Init(I2S_HandleTypeDef *hi2s)
{
  uint32_t i2sdiv;
  uint32_t i2sodd;
  uint32_t packetlength;
  uint32_t tmp;
  uint32_t i2sclk;
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
  uint16_t tmpreg;
#endif

  /* Check the I2S handle allocation */
  if (hi2s == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the I2S parameters */
  ASSERT_PARAM(IS_I2S_ALL_INSTANCE(hi2s->Instance));
  ASSERT_PARAM(IS_I2S_MODE(hi2s->Init.Mode));
  ASSERT_PARAM(IS_I2S_STANDARD(hi2s->Init.Standard));
  ASSERT_PARAM(IS_I2S_DATA_FORMAT(hi2s->Init.DataFormat));
  ASSERT_PARAM(IS_I2S_MCLK_OUTPUT(hi2s->Init.MCLKOutput));
  ASSERT_PARAM(IS_I2S_AUDIO_FREQ(hi2s->Init.AudioFreq));
  ASSERT_PARAM(IS_I2S_CPOL(hi2s->Init.CPOL));
  ASSERT_PARAM(IS_I2S_CLOCKSOURCE(hi2s->Init.ClockSource));

  if (hi2s->State == DAL_I2S_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hi2s->Lock = DAL_UNLOCKED;

    /* Initialize Default I2S IrqHandler ISR */
    hi2s->IrqHandlerISR = I2S_IRQHandler;

#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
    /* Init the I2S Callback settings */
    hi2s->TxCpltCallback       = DAL_I2S_TxCpltCallback;          /* Legacy weak TxCpltCallback       */
    hi2s->RxCpltCallback       = DAL_I2S_RxCpltCallback;          /* Legacy weak RxCpltCallback       */
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
    hi2s->TxRxCpltCallback     = DAL_I2SEx_TxRxCpltCallback;      /* Legacy weak TxRxCpltCallback     */
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
    hi2s->TxHalfCpltCallback   = DAL_I2S_TxHalfCpltCallback;      /* Legacy weak TxHalfCpltCallback   */
    hi2s->RxHalfCpltCallback   = DAL_I2S_RxHalfCpltCallback;      /* Legacy weak RxHalfCpltCallback   */
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
    hi2s->TxRxHalfCpltCallback = DAL_I2SEx_TxRxHalfCpltCallback;  /* Legacy weak TxRxHalfCpltCallback */
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
    hi2s->ErrorCallback        = DAL_I2S_ErrorCallback;           /* Legacy weak ErrorCallback        */

    if (hi2s->MspInitCallback == NULL)
    {
      hi2s->MspInitCallback = DAL_I2S_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    hi2s->MspInitCallback(hi2s);
#else
    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    DAL_I2S_MspInit(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
  }

  hi2s->State = DAL_I2S_STATE_BUSY;

  /*----------------------- SPIx I2SCFGR & I2SPR Configuration ----------------*/
  /* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
  CLEAR_BIT(hi2s->Instance->I2SCFG, (SPI_I2SCFG_CHLEN | SPI_I2SCFG_DATALEN | SPI_I2SCFG_CPOL | \
                                      SPI_I2SCFG_I2SSSEL | SPI_I2SCFG_PFSSEL | SPI_I2SCFG_I2SMOD | \
                                      SPI_I2SCFG_I2SEN | SPI_I2SCFG_MODESEL));
  hi2s->Instance->I2SPSC = 0x0002U;

  /*----------------------- I2SPR: I2SDIV and ODD Calculation -----------------*/
  /* If the requested audio frequency is not the default, compute the prescaler */
  if (hi2s->Init.AudioFreq != I2S_AUDIOFREQ_DEFAULT)
  {
    /* Check the frame length (For the Prescaler computing) ********************/
    if (hi2s->Init.DataFormat == I2S_DATAFORMAT_16B)
    {
      /* Packet length is 16 bits */
      packetlength = 16U;
    }
    else
    {
      /* Packet length is 32 bits */
      packetlength = 32U;
    }

    /* I2S standard */
    if (hi2s->Init.Standard <= I2S_STANDARD_LSB)
    {
      /* In I2S standard packet length is multiplied by 2 */
      packetlength = packetlength * 2U;
    }

    /* Get the source clock value **********************************************/
#if defined(I2S_APB1_APB2_FEATURE)
    if (IS_I2S_APB1_INSTANCE(hi2s->Instance))
    {
      i2sclk = DAL_RCMEx_GetPeriphCLKFreq(RCM_PERIPHCLK_I2S_APB1);
    }
    else
    {
      i2sclk = DAL_RCMEx_GetPeriphCLKFreq(RCM_PERIPHCLK_I2S_APB2);
    }
#else
    i2sclk = DAL_RCMEx_GetPeriphCLKFreq(RCM_PERIPHCLK_I2S);
#endif /* I2S_APB1_APB2_FEATURE */

    /* Compute the Real divider depending on the MCLK output state, with a floating point */
    if (hi2s->Init.MCLKOutput == I2S_MCLKOUTPUT_ENABLE)
    {
      /* MCLK output is enabled */
      if (hi2s->Init.DataFormat != I2S_DATAFORMAT_16B)
      {
        tmp = (uint32_t)(((((i2sclk / (packetlength * 4U)) * 10U) / hi2s->Init.AudioFreq)) + 5U);
      }
      else
      {
        tmp = (uint32_t)(((((i2sclk / (packetlength * 8U)) * 10U) / hi2s->Init.AudioFreq)) + 5U);
      }
    }
    else
    {
      /* MCLK output is disabled */
      tmp = (uint32_t)(((((i2sclk / packetlength) * 10U) / hi2s->Init.AudioFreq)) + 5U);
    }

    /* Remove the flatting point */
    tmp = tmp / 10U;

    /* Check the parity of the divider */
    i2sodd = (uint32_t)(tmp & (uint32_t)1U);

    /* Compute the i2sdiv prescaler */
    i2sdiv = (uint32_t)((tmp - i2sodd) / 2U);

    /* Get the Mask for the Odd bit (SPI_I2SPSC[8]) register */
    i2sodd = (uint32_t)(i2sodd << 8U);
  }
  else
  {
    /* Set the default values */
    i2sdiv = 2U;
    i2sodd = 0U;
  }

  /* Test if the divider is 1 or 0 or greater than 0xFF */
  if ((i2sdiv < 2U) || (i2sdiv > 0xFFU))
  {
    /* Set the error code and execute error callback*/
    SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_PRESCALER);
    return  DAL_ERROR;
  }

  /*----------------------- SPIx I2SCFG & I2SPSC Configuration ----------------*/

  /* Write to SPIx I2SPSC register the computed value */
  hi2s->Instance->I2SPSC = (uint32_t)((uint32_t)i2sdiv | (uint32_t)(i2sodd | (uint32_t)hi2s->Init.MCLKOutput));

  /* Clear CHLEN, DATALEN, CPOL, I2SSSEL, PFSSEL, I2SMOD, I2SEN and MODESEL bits */
  /* And configure the I2S with the I2S_InitStruct values                      */
  MODIFY_REG(hi2s->Instance->I2SCFG, (SPI_I2SCFG_CHLEN | SPI_I2SCFG_DATALEN | \
                                       SPI_I2SCFG_CPOL | SPI_I2SCFG_I2SSSEL | \
                                       SPI_I2SCFG_PFSSEL | SPI_I2SCFG_I2SMOD | \
                                       SPI_I2SCFG_I2SEN  | SPI_I2SCFG_MODESEL), \
             (SPI_I2SCFG_MODESEL | hi2s->Init.Mode | \
              hi2s->Init.Standard | hi2s->Init.DataFormat | \
              hi2s->Init.CPOL));

#if defined(SPI_I2SCFG_ASTRTEN)
  if ((hi2s->Init.Standard == I2S_STANDARD_PCM_SHORT) || ((hi2s->Init.Standard == I2S_STANDARD_PCM_LONG)))
  {
    /* Write to SPIx I2SCFG */
    SET_BIT(hi2s->Instance->I2SCFG, SPI_I2SCFG_ASTRTEN);
  }
#endif /* SPI_I2SCFG_ASTRTEN */

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)

  /* Configure the I2S extended if the full duplex mode is enabled */
  ASSERT_PARAM(IS_I2S_FULLDUPLEX_MODE(hi2s->Init.FullDuplexMode));

  if (hi2s->Init.FullDuplexMode == I2S_FULLDUPLEXMODE_ENABLE)
  {
    /* Set FullDuplex I2S IrqHandler ISR if FULLDUPLEXMODE is enabled */
    hi2s->IrqHandlerISR = DAL_I2SEx_FullDuplex_IRQHandler;

    /* Clear CHLEN, DATALEN, CPOL, I2SSSEL, PFSSEL, I2SMOD, I2SEN and MODESEL bits */
    CLEAR_BIT(I2SxEXT(hi2s->Instance)->I2SCFG, (SPI_I2SCFG_CHLEN | SPI_I2SCFG_DATALEN | SPI_I2SCFG_CPOL | \
                                                 SPI_I2SCFG_I2SSSEL | SPI_I2SCFG_PFSSEL | SPI_I2SCFG_I2SMOD | \
                                                 SPI_I2SCFG_I2SEN | SPI_I2SCFG_MODESEL));
    I2SxEXT(hi2s->Instance)->I2SPSC = 2U;

    /* Get the I2SCFG register value */
    tmpreg = I2SxEXT(hi2s->Instance)->I2SCFG;

    /* Get the mode to be configured for the extended I2S */
    if ((hi2s->Init.Mode == I2S_MODE_MASTER_TX) || (hi2s->Init.Mode == I2S_MODE_SLAVE_TX))
    {
      tmp = I2S_MODE_SLAVE_RX;
    }
    else /* I2S_MODE_MASTER_RX ||  I2S_MODE_SLAVE_RX */
    {
      tmp = I2S_MODE_SLAVE_TX;
    }

    /* Configure the I2S Slave with the I2S Master parameter values */
    tmpreg |= (uint16_t)((uint16_t)SPI_I2SCFG_MODESEL    | \
                         (uint16_t)tmp                   | \
                         (uint16_t)hi2s->Init.Standard   | \
                         (uint16_t)hi2s->Init.DataFormat | \
                         (uint16_t)hi2s->Init.CPOL);

    /* Write to SPIx I2SCFG */
    WRITE_REG(I2SxEXT(hi2s->Instance)->I2SCFG, tmpreg);
  }
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->State     = DAL_I2S_STATE_READY;

  return DAL_OK;
}

/**
  * @brief DeInitializes the I2S peripheral
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_DeInit(I2S_HandleTypeDef *hi2s)
{
  /* Check the I2S handle allocation */
  if (hi2s == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_I2S_ALL_INSTANCE(hi2s->Instance));

  hi2s->State = DAL_I2S_STATE_BUSY;

  /* Disable the I2S Peripheral Clock */
  __DAL_I2S_DISABLE(hi2s);

#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  if (hi2s->MspDeInitCallback == NULL)
  {
    hi2s->MspDeInitCallback = DAL_I2S_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
  hi2s->MspDeInitCallback(hi2s);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
  DAL_I2S_MspDeInit(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */

  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->State     = DAL_I2S_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hi2s);

  return DAL_OK;
}

/**
  * @brief I2S MSP Init
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void DAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_I2S_MspInit could be implemented in the user file
   */
}

/**
  * @brief I2S MSP DeInit
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void DAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_I2S_MspDeInit could be implemented in the user file
   */
}

#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User I2S Callback
  *         To be used instead of the weak predefined callback
  * @param  hi2s Pointer to a I2S_HandleTypeDef structure that contains
  *                the configuration information for the specified I2S.
  * @param  CallbackID ID of the callback to be registered
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_RegisterCallback(I2S_HandleTypeDef *hi2s, DAL_I2S_CallbackIDTypeDef CallbackID,
                                           pI2S_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hi2s->ErrorCode |= DAL_I2S_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hi2s);

  if (DAL_I2S_STATE_READY == hi2s->State)
  {
    switch (CallbackID)
    {
      case DAL_I2S_TX_COMPLETE_CB_ID :
        hi2s->TxCpltCallback = pCallback;
        break;

      case DAL_I2S_RX_COMPLETE_CB_ID :
        hi2s->RxCpltCallback = pCallback;
        break;

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
      case DAL_I2S_TX_RX_COMPLETE_CB_ID :
        hi2s->TxRxCpltCallback = pCallback;
        break;
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

      case DAL_I2S_TX_HALF_COMPLETE_CB_ID :
        hi2s->TxHalfCpltCallback = pCallback;
        break;

      case DAL_I2S_RX_HALF_COMPLETE_CB_ID :
        hi2s->RxHalfCpltCallback = pCallback;
        break;

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
      case DAL_I2S_TX_RX_HALF_COMPLETE_CB_ID :
        hi2s->TxRxHalfCpltCallback = pCallback;
        break;
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

      case DAL_I2S_ERROR_CB_ID :
        hi2s->ErrorCallback = pCallback;
        break;

      case DAL_I2S_MSPINIT_CB_ID :
        hi2s->MspInitCallback = pCallback;
        break;

      case DAL_I2S_MSPDEINIT_CB_ID :
        hi2s->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_I2S_STATE_RESET == hi2s->State)
  {
    switch (CallbackID)
    {
      case DAL_I2S_MSPINIT_CB_ID :
        hi2s->MspInitCallback = pCallback;
        break;

      case DAL_I2S_MSPDEINIT_CB_ID :
        hi2s->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_INVALID_CALLBACK);

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hi2s);
  return status;
}

/**
  * @brief  Unregister an I2S Callback
  *         I2S callback is redirected to the weak predefined callback
  * @param  hi2s Pointer to a I2S_HandleTypeDef structure that contains
  *                the configuration information for the specified I2S.
  * @param  CallbackID ID of the callback to be unregistered
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_UnRegisterCallback(I2S_HandleTypeDef *hi2s, DAL_I2S_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hi2s);

  if (DAL_I2S_STATE_READY == hi2s->State)
  {
    switch (CallbackID)
    {
      case DAL_I2S_TX_COMPLETE_CB_ID :
        hi2s->TxCpltCallback = DAL_I2S_TxCpltCallback;                /* Legacy weak TxCpltCallback       */
        break;

      case DAL_I2S_RX_COMPLETE_CB_ID :
        hi2s->RxCpltCallback = DAL_I2S_RxCpltCallback;                /* Legacy weak RxCpltCallback       */
        break;

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
      case DAL_I2S_TX_RX_COMPLETE_CB_ID :
        hi2s->TxRxCpltCallback = DAL_I2SEx_TxRxCpltCallback;          /* Legacy weak TxRxCpltCallback     */
        break;
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

      case DAL_I2S_TX_HALF_COMPLETE_CB_ID :
        hi2s->TxHalfCpltCallback = DAL_I2S_TxHalfCpltCallback;        /* Legacy weak TxHalfCpltCallback   */
        break;

      case DAL_I2S_RX_HALF_COMPLETE_CB_ID :
        hi2s->RxHalfCpltCallback = DAL_I2S_RxHalfCpltCallback;        /* Legacy weak RxHalfCpltCallback   */
        break;

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
      case DAL_I2S_TX_RX_HALF_COMPLETE_CB_ID :
        hi2s->TxRxHalfCpltCallback = DAL_I2SEx_TxRxHalfCpltCallback;  /* Legacy weak TxRxHalfCpltCallback */
        break;
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

      case DAL_I2S_ERROR_CB_ID :
        hi2s->ErrorCallback = DAL_I2S_ErrorCallback;                  /* Legacy weak ErrorCallback        */
        break;

      case DAL_I2S_MSPINIT_CB_ID :
        hi2s->MspInitCallback = DAL_I2S_MspInit;                      /* Legacy weak MspInit              */
        break;

      case DAL_I2S_MSPDEINIT_CB_ID :
        hi2s->MspDeInitCallback = DAL_I2S_MspDeInit;                  /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_I2S_STATE_RESET == hi2s->State)
  {
    switch (CallbackID)
    {
      case DAL_I2S_MSPINIT_CB_ID :
        hi2s->MspInitCallback = DAL_I2S_MspInit;                      /* Legacy weak MspInit              */
        break;

      case DAL_I2S_MSPDEINIT_CB_ID :
        hi2s->MspDeInitCallback = DAL_I2S_MspDeInit;                  /* Legacy weak MspDeInit            */
        break;

      default :
        /* Update the error code */
        SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_INVALID_CALLBACK);

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hi2s);
  return status;
}
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group2 IO operation functions
  *  @brief Data transfers functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2S data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2S IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) DAL_I2S_Transmit()
        (++) DAL_I2S_Receive()

    (#) No-Blocking mode functions with Interrupt are :
        (++) DAL_I2S_Transmit_IT()
        (++) DAL_I2S_Receive_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) DAL_I2S_Transmit_DMA()
        (++) DAL_I2S_Receive_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) DAL_I2S_TxCpltCallback()
        (++) DAL_I2S_RxCpltCallback()
        (++) DAL_I2S_ErrorCallback()

@endverbatim
  * @{
  */

/**
  * @brief  Transmit an amount of data in blocking mode
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pData a 16-bit pointer to data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 24-bit or 32-bit data length.
  * @param  Timeout Timeout duration
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tmpreg_cfgr;

  if ((pData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    __DAL_UNLOCK(hi2s);
    return DAL_BUSY;
  }

  /* Set state and reset error code */
  hi2s->State = DAL_I2S_STATE_BUSY_TX;
  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->pTxBuffPtr = pData;

  tmpreg_cfgr = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);

  if ((tmpreg_cfgr == I2S_DATAFORMAT_24B) || (tmpreg_cfgr == I2S_DATAFORMAT_32B))
  {
    hi2s->TxXferSize = (Size << 1U);
    hi2s->TxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->TxXferSize = Size;
    hi2s->TxXferCount = Size;
  }

  tmpreg_cfgr = hi2s->Instance->I2SCFG;

  /* Check if the I2S is already enabled */
  if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SEN) != SPI_I2SCFG_I2SEN)
  {
    /* Enable I2S peripheral */
    __DAL_I2S_ENABLE(hi2s);
  }

  /* Wait until TXE flag is set */
  if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_TXE, SET, Timeout) != DAL_OK)
  {
    /* Set the error code */
    SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
    hi2s->State = DAL_I2S_STATE_READY;
    __DAL_UNLOCK(hi2s);
    return DAL_ERROR;
  }

  while (hi2s->TxXferCount > 0U)
  {
    hi2s->Instance->DATA = (*hi2s->pTxBuffPtr);
    hi2s->pTxBuffPtr++;
    hi2s->TxXferCount--;

    /* Wait until TXE flag is set */
    if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_TXE, SET, Timeout) != DAL_OK)
    {
      /* Set the error code */
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
      hi2s->State = DAL_I2S_STATE_READY;
      __DAL_UNLOCK(hi2s);
      return DAL_ERROR;
    }

    /* Check if an underrun occurs */
    if (__DAL_I2S_GET_FLAG(hi2s, I2S_FLAG_UDR) == SET)
    {
      /* Clear underrun flag */
      __DAL_I2S_CLEAR_UDRFLAG(hi2s);

      /* Set the error code */
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_UDR);
    }
  }

  /* Check if Slave mode is selected */
  if (((tmpreg_cfgr & SPI_I2SCFG_I2SMOD) == I2S_MODE_SLAVE_TX)
      || ((tmpreg_cfgr & SPI_I2SCFG_I2SMOD) == I2S_MODE_SLAVE_RX))
  {
    /* Wait until Busy flag is reset */
    if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_BSY, RESET, Timeout) != DAL_OK)
    {
      /* Set the error code */
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
      hi2s->State = DAL_I2S_STATE_READY;
      __DAL_UNLOCK(hi2s);
      return DAL_ERROR;
    }
  }

  hi2s->State = DAL_I2S_STATE_READY;
  __DAL_UNLOCK(hi2s);
  return DAL_OK;
}

/**
  * @brief  Receive an amount of data in blocking mode
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pData a 16-bit pointer to data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 24-bit or 32-bit data length.
  * @param  Timeout Timeout duration
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @note   In I2S Master Receiver mode, just after enabling the peripheral the clock will be generate
  *         in continuous way and as the I2S is not disabled at the end of the I2S transaction.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tmpreg_cfgr;

  if ((pData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    __DAL_UNLOCK(hi2s);
    return DAL_BUSY;
  }

  /* Set state and reset error code */
  hi2s->State = DAL_I2S_STATE_BUSY_RX;
  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->pRxBuffPtr = pData;

  tmpreg_cfgr = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);

  if ((tmpreg_cfgr == I2S_DATAFORMAT_24B) || (tmpreg_cfgr == I2S_DATAFORMAT_32B))
  {
    hi2s->RxXferSize = (Size << 1U);
    hi2s->RxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->RxXferSize = Size;
    hi2s->RxXferCount = Size;
  }

  /* Check if the I2S is already enabled */
  if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SEN) != SPI_I2SCFG_I2SEN)
  {
    /* Enable I2S peripheral */
    __DAL_I2S_ENABLE(hi2s);
  }

  /* Check if Master Receiver mode is selected */
  if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD) == I2S_MODE_MASTER_RX)
  {
    /* Clear the Overrun Flag by a read operation on the SPI_DATA register followed by a read
    access to the SPI_STS register. */
    __DAL_I2S_CLEAR_OVRFLAG(hi2s);
  }

  /* Receive data */
  while (hi2s->RxXferCount > 0U)
  {
    /* Wait until RXNE flag is set */
    if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_RXNE, SET, Timeout) != DAL_OK)
    {
      /* Set the error code */
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
      hi2s->State = DAL_I2S_STATE_READY;
      __DAL_UNLOCK(hi2s);
      return DAL_ERROR;
    }

    (*hi2s->pRxBuffPtr) = (uint16_t)hi2s->Instance->DATA;
    hi2s->pRxBuffPtr++;
    hi2s->RxXferCount--;

    /* Check if an overrun occurs */
    if (__DAL_I2S_GET_FLAG(hi2s, I2S_FLAG_OVR) == SET)
    {
      /* Clear overrun flag */
      __DAL_I2S_CLEAR_OVRFLAG(hi2s);

      /* Set the error code */
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_OVR);
    }
  }

  hi2s->State = DAL_I2S_STATE_READY;
  __DAL_UNLOCK(hi2s);
  return DAL_OK;
}

/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pData a 16-bit pointer to data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 24-bit or 32-bit data length.
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size)
{
  uint32_t tmpreg_cfgr;

  if ((pData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    __DAL_UNLOCK(hi2s);
    return DAL_BUSY;
  }

  /* Set state and reset error code */
  hi2s->State = DAL_I2S_STATE_BUSY_TX;
  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->pTxBuffPtr = pData;

  tmpreg_cfgr = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);

  if ((tmpreg_cfgr == I2S_DATAFORMAT_24B) || (tmpreg_cfgr == I2S_DATAFORMAT_32B))
  {
    hi2s->TxXferSize = (Size << 1U);
    hi2s->TxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->TxXferSize = Size;
    hi2s->TxXferCount = Size;
  }

  /* Enable TXE and ERR interrupt */
  __DAL_I2S_ENABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

  /* Check if the I2S is already enabled */
  if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SEN) != SPI_I2SCFG_I2SEN)
  {
    /* Enable I2S peripheral */
    __DAL_I2S_ENABLE(hi2s);
  }

  __DAL_UNLOCK(hi2s);
  return DAL_OK;
}

/**
  * @brief  Receive an amount of data in non-blocking mode with Interrupt
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pData a 16-bit pointer to the Receive data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 24-bit or 32-bit data length.
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @note   It is recommended to use DMA for the I2S receiver to avoid de-synchronization
  * between Master and Slave otherwise the I2S interrupt should be optimized.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size)
{
  uint32_t tmpreg_cfgr;

  if ((pData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    __DAL_UNLOCK(hi2s);
    return DAL_BUSY;
  }

  /* Set state and reset error code */
  hi2s->State = DAL_I2S_STATE_BUSY_RX;
  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->pRxBuffPtr = pData;

  tmpreg_cfgr = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);

  if ((tmpreg_cfgr == I2S_DATAFORMAT_24B) || (tmpreg_cfgr == I2S_DATAFORMAT_32B))
  {
    hi2s->RxXferSize = (Size << 1U);
    hi2s->RxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->RxXferSize = Size;
    hi2s->RxXferCount = Size;
  }

  /* Enable RXNE and ERR interrupt */
  __DAL_I2S_ENABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

  /* Check if the I2S is already enabled */
  if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SEN) != SPI_I2SCFG_I2SEN)
  {
    /* Enable I2S peripheral */
    __DAL_I2S_ENABLE(hi2s);
  }

  __DAL_UNLOCK(hi2s);
  return DAL_OK;
}

/**
  * @brief  Transmit an amount of data in non-blocking mode with DMA
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pData a 16-bit pointer to the Transmit data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 24-bit or 32-bit data length.
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size)
{
  uint32_t tmpreg_cfgr;

  if ((pData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    __DAL_UNLOCK(hi2s);
    return DAL_BUSY;
  }

  /* Set state and reset error code */
  hi2s->State = DAL_I2S_STATE_BUSY_TX;
  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->pTxBuffPtr = pData;

  tmpreg_cfgr = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);

  if ((tmpreg_cfgr == I2S_DATAFORMAT_24B) || (tmpreg_cfgr == I2S_DATAFORMAT_32B))
  {
    hi2s->TxXferSize = (Size << 1U);
    hi2s->TxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->TxXferSize = Size;
    hi2s->TxXferCount = Size;
  }

  /* Set the I2S Tx DMA Half transfer complete callback */
  hi2s->hdmatx->XferHalfCpltCallback = I2S_DMATxHalfCplt;

  /* Set the I2S Tx DMA transfer complete callback */
  hi2s->hdmatx->XferCpltCallback = I2S_DMATxCplt;

  /* Set the DMA error callback */
  hi2s->hdmatx->XferErrorCallback = I2S_DMAError;

  /* Enable the Tx DMA Stream/Channel */
  if (DAL_OK != DAL_DMA_Start_IT(hi2s->hdmatx,
                                 (uint32_t)hi2s->pTxBuffPtr,
                                 (uint32_t)&hi2s->Instance->DATA,
                                 hi2s->TxXferSize))
  {
    /* Update SPI error code */
    SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_DMA);
    hi2s->State = DAL_I2S_STATE_READY;

    __DAL_UNLOCK(hi2s);
    return DAL_ERROR;
  }

  /* Check if the I2S is already enabled */
  if (DAL_IS_BIT_CLR(hi2s->Instance->I2SCFG, SPI_I2SCFG_I2SEN))
  {
    /* Enable I2S peripheral */
    __DAL_I2S_ENABLE(hi2s);
  }

  /* Check if the I2S Tx request is already enabled */
  if (DAL_IS_BIT_CLR(hi2s->Instance->CTRL2, SPI_CTRL2_TXDEN))
  {
    /* Enable Tx DMA Request */
    SET_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_TXDEN);
  }

  __DAL_UNLOCK(hi2s);
  return DAL_OK;
}

/**
  * @brief  Receive an amount of data in non-blocking mode with DMA
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pData a 16-bit pointer to the Receive data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 24-bit or 32-bit data length.
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size)
{
  uint32_t tmpreg_cfgr;

  if ((pData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    __DAL_UNLOCK(hi2s);
    return DAL_BUSY;
  }

  /* Set state and reset error code */
  hi2s->State = DAL_I2S_STATE_BUSY_RX;
  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->pRxBuffPtr = pData;

  tmpreg_cfgr = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);

  if ((tmpreg_cfgr == I2S_DATAFORMAT_24B) || (tmpreg_cfgr == I2S_DATAFORMAT_32B))
  {
    hi2s->RxXferSize = (Size << 1U);
    hi2s->RxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->RxXferSize = Size;
    hi2s->RxXferCount = Size;
  }

  /* Set the I2S Rx DMA Half transfer complete callback */
  hi2s->hdmarx->XferHalfCpltCallback = I2S_DMARxHalfCplt;

  /* Set the I2S Rx DMA transfer complete callback */
  hi2s->hdmarx->XferCpltCallback = I2S_DMARxCplt;

  /* Set the DMA error callback */
  hi2s->hdmarx->XferErrorCallback = I2S_DMAError;

  /* Check if Master Receiver mode is selected */
  if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD) == I2S_MODE_MASTER_RX)
  {
    /* Clear the Overrun Flag by a read operation to the SPI_DATA register followed by a read
    access to the SPI_STS register. */
    __DAL_I2S_CLEAR_OVRFLAG(hi2s);
  }

  /* Enable the Rx DMA Stream/Channel */
  if (DAL_OK != DAL_DMA_Start_IT(hi2s->hdmarx, (uint32_t)&hi2s->Instance->DATA, (uint32_t)hi2s->pRxBuffPtr,
                                 hi2s->RxXferSize))
  {
    /* Update SPI error code */
    SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_DMA);
    hi2s->State = DAL_I2S_STATE_READY;

    __DAL_UNLOCK(hi2s);
    return DAL_ERROR;
  }

  /* Check if the I2S is already enabled */
  if (DAL_IS_BIT_CLR(hi2s->Instance->I2SCFG, SPI_I2SCFG_I2SEN))
  {
    /* Enable I2S peripheral */
    __DAL_I2S_ENABLE(hi2s);
  }

  /* Check if the I2S Rx request is already enabled */
  if (DAL_IS_BIT_CLR(hi2s->Instance->CTRL2, SPI_CTRL2_RXDEN))
  {
    /* Enable Rx DMA Request */
    SET_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_RXDEN);
  }

  __DAL_UNLOCK(hi2s);
  return DAL_OK;
}

/**
  * @brief  Pauses the audio DMA Stream/Channel playing from the Media.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s)
{
  /* Process Locked */
  __DAL_LOCK(hi2s);

  if (hi2s->State == DAL_I2S_STATE_BUSY_TX)
  {
    /* Disable the I2S DMA Tx request */
    CLEAR_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_TXDEN);
  }
  else if (hi2s->State == DAL_I2S_STATE_BUSY_RX)
  {
    /* Disable the I2S DMA Rx request */
    CLEAR_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_RXDEN);
  }
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
  else if (hi2s->State == DAL_I2S_STATE_BUSY_TX_RX)
  {
    /* Pause the audio file playing by disabling the I2S DMA request */
    CLEAR_BIT(hi2s->Instance->CTRL2, (SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN));
    CLEAR_BIT(I2SxEXT(hi2s->Instance)->CTRL2, (SPI_CTRL2_TXDEN | SPI_CTRL2_RXDEN));
  }
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
  else
  {
    /* nothing to do */
  }

  /* Process Unlocked */
  __DAL_UNLOCK(hi2s);

  return DAL_OK;
}

/**
  * @brief  Resumes the audio DMA Stream/Channel playing from the Media.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s)
{
  /* Process Locked */
  __DAL_LOCK(hi2s);

  if (hi2s->State == DAL_I2S_STATE_BUSY_TX)
  {
    /* Enable the I2S DMA Tx request */
    SET_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_TXDEN);
  }
  else if (hi2s->State == DAL_I2S_STATE_BUSY_RX)
  {
    /* Enable the I2S DMA Rx request */
    SET_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_RXDEN);
  }
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
  else if (hi2s->State == DAL_I2S_STATE_BUSY_TX_RX)
  {
    /* Pause the audio file playing by disabling the I2S DMA request */
    SET_BIT(hi2s->Instance->CTRL2, (SPI_CTRL2_RXDEN | SPI_CTRL2_TXDEN));
    SET_BIT(I2SxEXT(hi2s->Instance)->CTRL2, (SPI_CTRL2_RXDEN | SPI_CTRL2_TXDEN));

    /* If the I2Sext peripheral is still not enabled, enable it */
    if ((I2SxEXT(hi2s->Instance)->I2SCFG & SPI_I2SCFG_I2SEN) == 0U)
    {
      /* Enable I2Sext peripheral */
      __DAL_I2SEXT_ENABLE(hi2s);
    }
  }
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
  else
  {
    /* nothing to do */
  }

  /* If the I2S peripheral is still not enabled, enable it */
  if (DAL_IS_BIT_CLR(hi2s->Instance->I2SCFG, SPI_I2SCFG_I2SEN))
  {
    /* Enable I2S peripheral */
    __DAL_I2S_ENABLE(hi2s);
  }

  /* Process Unlocked */
  __DAL_UNLOCK(hi2s);

  return DAL_OK;
}

/**
  * @brief  Stops the audio DMA Stream/Channel playing from the Media.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s)
{
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)
  uint32_t tickstart;
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
  DAL_StatusTypeDef errorcode = DAL_OK;
  /* The Lock is not implemented on this API to allow the user application
     to call the DAL SPI API under callbacks DAL_I2S_TxCpltCallback() or DAL_I2S_RxCpltCallback()
     when calling DAL_DMA_Abort() API the DMA TX or RX Transfer complete interrupt is generated
     and the correspond call back is executed DAL_I2S_TxCpltCallback() or DAL_I2S_RxCpltCallback()
     */

  if ((hi2s->Init.Mode == I2S_MODE_MASTER_TX) || (hi2s->Init.Mode == I2S_MODE_SLAVE_TX))
  {
    /* Abort the I2S DMA tx Stream/Channel */
    if (hi2s->hdmatx != NULL)
    {
      /* Disable the I2S DMA tx Stream/Channel */
      if (DAL_OK != DAL_DMA_Abort(hi2s->hdmatx))
      {
        SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_DMA);
        errorcode = DAL_ERROR;
      }
    }

    /* Wait until TXE flag is set */
    if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_TXE, SET, I2S_TIMEOUT_FLAG) != DAL_OK)
    {
      /* Set the error code */
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
      hi2s->State = DAL_I2S_STATE_READY;
      errorcode   = DAL_ERROR;
    }

    /* Wait until BSY flag is Reset */
    if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_BSY, RESET, I2S_TIMEOUT_FLAG) != DAL_OK)
    {
      /* Set the error code */
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
      hi2s->State = DAL_I2S_STATE_READY;
      errorcode   = DAL_ERROR;
    }

    /* Disable I2S peripheral */
    __DAL_I2S_DISABLE(hi2s);

    /* Clear UDR flag */
    __DAL_I2S_CLEAR_UDRFLAG(hi2s);

    /* Disable the I2S Tx DMA requests */
    CLEAR_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_TXDEN);

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)

    if (hi2s->State == DAL_I2S_STATE_BUSY_TX_RX)
    {
      /* Abort the I2S DMA rx Stream/Channel */
      if (hi2s->hdmarx != NULL)
      {
        /* Disable the I2S DMA rx Stream/Channel */
        if (DAL_OK != DAL_DMA_Abort(hi2s->hdmarx))
        {
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_DMA);
          errorcode = DAL_ERROR;
        }
      }

      /* Disable I2Sext peripheral */
      __DAL_I2SEXT_DISABLE(hi2s);

      /* Clear OVR flag */
      __DAL_I2SEXT_CLEAR_OVRFLAG(hi2s);

      /* Disable the I2SxEXT DMA request */
      CLEAR_BIT(I2SxEXT(hi2s->Instance)->CTRL2, SPI_CTRL2_RXDEN);

      if (hi2s->Init.Mode == I2S_MODE_SLAVE_TX)
      {
        /* Set the error code */
        SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_BUSY_LINE_RX);

        /* Set the I2S State ready */
        hi2s->State = DAL_I2S_STATE_READY;
        errorcode = DAL_ERROR;
      }
      else
      {
        /* Read DATA to Flush RX Data */
        READ_REG(I2SxEXT(hi2s->Instance)->DATA);
      }
    }
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */
  }

  else if ((hi2s->Init.Mode == I2S_MODE_MASTER_RX) || (hi2s->Init.Mode == I2S_MODE_SLAVE_RX))
  {
    /* Abort the I2S DMA rx Stream/Channel */
    if (hi2s->hdmarx != NULL)
    {
      /* Disable the I2S DMA rx Stream/Channel */
      if (DAL_OK != DAL_DMA_Abort(hi2s->hdmarx))
      {
        SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_DMA);
        errorcode = DAL_ERROR;
      }
    }
#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)

    if (hi2s->State == DAL_I2S_STATE_BUSY_TX_RX)
    {
      /* Abort the I2S DMA tx Stream/Channel */
      if (hi2s->hdmatx != NULL)
      {
        /* Disable the I2S DMA tx Stream/Channel */
        if (DAL_OK != DAL_DMA_Abort(hi2s->hdmatx))
        {
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_DMA);
          errorcode = DAL_ERROR;
        }
      }

      tickstart = DAL_GetTick();

      /* Wait until TXE flag is set */
      while (__DAL_I2SEXT_GET_FLAG(hi2s, I2S_FLAG_TXE) != SET)
      {
        if (((DAL_GetTick() - tickstart) > I2S_TIMEOUT_FLAG))
        {
          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);

          /* Set the I2S State ready */
          hi2s->State = DAL_I2S_STATE_READY;
          errorcode   = DAL_ERROR;
        }
      }

      /* Wait until BSY flag is Reset */
      while (__DAL_I2SEXT_GET_FLAG(hi2s, I2S_FLAG_BSY) != RESET)
      {
        if (((DAL_GetTick() - tickstart) > I2S_TIMEOUT_FLAG))
        {
          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);

          /* Set the I2S State ready */
          hi2s->State = DAL_I2S_STATE_READY;
          errorcode   = DAL_ERROR;
        }
      }

      /* Disable I2Sext peripheral */
      __DAL_I2SEXT_DISABLE(hi2s);

      /* Clear UDR flag */
      __DAL_I2SEXT_CLEAR_UDRFLAG(hi2s);

      /* Disable the I2SxEXT DMA request */
      CLEAR_BIT(I2SxEXT(hi2s->Instance)->CTRL2, SPI_CTRL2_TXDEN);
    }
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

    /* Disable I2S peripheral */
    __DAL_I2S_DISABLE(hi2s);

    /* Clear OVR flag */
    __DAL_I2S_CLEAR_OVRFLAG(hi2s);

    /* Disable the I2S Rx DMA request */
    CLEAR_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_RXDEN);

    if (hi2s->Init.Mode == I2S_MODE_SLAVE_RX)
    {
      /* Set the error code */
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_BUSY_LINE_RX);

      /* Set the I2S State ready */
      hi2s->State = DAL_I2S_STATE_READY;
      errorcode = DAL_ERROR;
    }
    else
    {
      /* Read DATA to Flush RX Data */
      READ_REG((hi2s->Instance)->DATA);
    }
  }

  hi2s->State = DAL_I2S_STATE_READY;

  return errorcode;
}

/**
  * @brief  This function handles I2S interrupt request.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void DAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s)
{
  /* Call the IrqHandler ISR set during DAL_I2S_INIT */
  hi2s->IrqHandlerISR(hi2s);
}

/**
  * @brief  Tx Transfer Half completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void DAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_I2S_TxHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Tx Transfer completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void DAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_I2S_TxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Rx Transfer half completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void DAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_I2S_RxHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Rx Transfer completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void DAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_I2S_RxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  I2S error callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void DAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_I2S_ErrorCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup I2S_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   Peripheral State functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the I2S state
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval DAL state
  */
DAL_I2S_StateTypeDef DAL_I2S_GetState(I2S_HandleTypeDef *hi2s)
{
  return hi2s->State;
}

/**
  * @brief  Return the I2S error code
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval I2S Error Code
  */
uint32_t DAL_I2S_GetError(I2S_HandleTypeDef *hi2s)
{
  return hi2s->ErrorCode;
}
/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup I2S_Private_Functions I2S Private Functions
  * @{
  */
/**
  * @brief  DMA I2S transmit process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMATxCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* if DMA is configured in DMA_NORMAL Mode */
  if (hdma->Init.Mode == DMA_NORMAL)
  {
    /* Disable Tx DMA Request */
    CLEAR_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_TXDEN);

    hi2s->TxXferCount = 0U;
    hi2s->State = DAL_I2S_STATE_READY;
  }
  /* Call user Tx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  hi2s->TxCpltCallback(hi2s);
#else
  DAL_I2S_TxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S transmit process half complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* Call user Tx half complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  hi2s->TxHalfCpltCallback(hi2s);
#else
  DAL_I2S_TxHalfCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S receive process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMARxCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* if DMA is configured in DMA_NORMAL Mode */
  if (hdma->Init.Mode == DMA_NORMAL)
  {
    /* Disable Rx DMA Request */
    CLEAR_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_RXDEN);
    hi2s->RxXferCount = 0U;
    hi2s->State = DAL_I2S_STATE_READY;
  }
  /* Call user Rx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  hi2s->RxCpltCallback(hi2s);
#else
  DAL_I2S_RxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S receive process half complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMARxHalfCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* Call user Rx half complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  hi2s->RxHalfCpltCallback(hi2s);
#else
  DAL_I2S_RxHalfCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S communication error callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMAError(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* Disable Rx and Tx DMA Request */
  CLEAR_BIT(hi2s->Instance->CTRL2, (SPI_CTRL2_RXDEN | SPI_CTRL2_TXDEN));
  hi2s->TxXferCount = 0U;
  hi2s->RxXferCount = 0U;

  hi2s->State = DAL_I2S_STATE_READY;

  /* Set the error code and execute error callback*/
  SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_DMA);
  /* Call user error callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  hi2s->ErrorCallback(hi2s);
#else
  DAL_I2S_ErrorCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
static void I2S_Transmit_IT(I2S_HandleTypeDef *hi2s)
{
  /* Transmit data */
  hi2s->Instance->DATA = (*hi2s->pTxBuffPtr);
  hi2s->pTxBuffPtr++;
  hi2s->TxXferCount--;

  if (hi2s->TxXferCount == 0U)
  {
    /* Disable TXE and ERR interrupt */
    __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

    hi2s->State = DAL_I2S_STATE_READY;
    /* Call user Tx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
    hi2s->TxCpltCallback(hi2s);
#else
    DAL_I2S_TxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  Receive an amount of data in non-blocking mode with Interrupt
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
static void I2S_Receive_IT(I2S_HandleTypeDef *hi2s)
{
  /* Receive data */
  (*hi2s->pRxBuffPtr) = (uint16_t)hi2s->Instance->DATA;
  hi2s->pRxBuffPtr++;
  hi2s->RxXferCount--;

  if (hi2s->RxXferCount == 0U)
  {
    /* Disable RXNE and ERR interrupt */
    __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

    hi2s->State = DAL_I2S_STATE_READY;
    /* Call user Rx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
    hi2s->RxCpltCallback(hi2s);
#else
    DAL_I2S_RxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  This function handles I2S interrupt request.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
static void I2S_IRQHandler(I2S_HandleTypeDef *hi2s)
{
  __IO uint32_t i2ssr = hi2s->Instance->STS;

  if (hi2s->State == DAL_I2S_STATE_BUSY_RX)
  {
    /* I2S in mode Receiver ------------------------------------------------*/
    if (((i2ssr & I2S_FLAG_RXNE) == I2S_FLAG_RXNE) && (__DAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_RXNE) != RESET))
    {
      I2S_Receive_IT(hi2s);
    }

    /* I2S Overrun error interrupt occurred -------------------------------------*/
    if (((i2ssr & I2S_FLAG_OVR) == I2S_FLAG_OVR) && (__DAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_ERR) != RESET))
    {
      /* Disable RXNE and ERR interrupt */
      __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

      /* Clear Overrun flag */
      __DAL_I2S_CLEAR_OVRFLAG(hi2s);

      /* Set the I2S State ready */
      hi2s->State = DAL_I2S_STATE_READY;


      /* Set the error code and execute error callback*/
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_OVR);
      /* Call user error callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
      hi2s->ErrorCallback(hi2s);
#else
      DAL_I2S_ErrorCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
    }
  }

  if (hi2s->State == DAL_I2S_STATE_BUSY_TX)
  {
    /* I2S in mode Transmitter -----------------------------------------------*/
    if (((i2ssr & I2S_FLAG_TXE) == I2S_FLAG_TXE) && (__DAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_TXE) != RESET))
    {
      I2S_Transmit_IT(hi2s);
    }

    /* I2S Underrun error interrupt occurred --------------------------------*/
    if (((i2ssr & I2S_FLAG_UDR) == I2S_FLAG_UDR) && (__DAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_ERR) != RESET))
    {
      /* Disable TXE and ERR interrupt */
      __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

      /* Clear Underrun flag */
      __DAL_I2S_CLEAR_UDRFLAG(hi2s);

      /* Set the I2S State ready */
      hi2s->State = DAL_I2S_STATE_READY;

      /* Set the error code and execute error callback*/
      SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_UDR);
      /* Call user error callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
      hi2s->ErrorCallback(hi2s);
#else
      DAL_I2S_ErrorCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  This function handles I2S Communication Timeout.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  Flag Flag checked
  * @param  State Value of the flag expected
  * @param  Timeout Duration of the timeout
  * @retval DAL status
  */
static DAL_StatusTypeDef I2S_WaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s, uint32_t Flag, FlagStatus State,
                                                       uint32_t Timeout)
{
  uint32_t tickstart;

  /* Get tick */
  tickstart = DAL_GetTick();

  /* Wait until flag is set to status*/
  while (((__DAL_I2S_GET_FLAG(hi2s, Flag)) ? SET : RESET) != State)
  {
    if (Timeout != DAL_MAX_DELAY)
    {
      if (((DAL_GetTick() - tickstart) >= Timeout) || (Timeout == 0U))
      {
        /* Set the I2S State ready */
        hi2s->State = DAL_I2S_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(hi2s);

        return DAL_TIMEOUT;
      }
    }
  }
  return DAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* DAL_I2S_MODULE_ENABLED */

