/**
  *
  * @file    apm32f4xx_dal_i2s_ex.c
  * @brief   I2S DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of I2S extension peripheral:
  *           + Extension features Functions
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
                    ##### I2S Extension features #####
  ==============================================================================
  [..]
     (#) In I2S full duplex mode, each SPI peripheral is able to manage sending and receiving
         data simultaneously using two data lines. Each SPI peripheral has an extended block
         called I2Sxext (i.e I2S2ext for SPI2 and I2S3ext for SPI3).
     (#) The extension block is not a full SPI IP, it is used only as I2S slave to
         implement full duplex mode. The extension block uses the same clock sources
         as its master.

     (#) Both I2Sx and I2Sx_ext can be configured as transmitters or receivers.

     [..]
       (@) Only I2Sx can deliver SCK and WS to I2Sx_ext in full duplex mode, where
         I2Sx can be I2S2 or I2S3.

                  ##### How to use this driver #####
 ===============================================================================
 [..]
   Three operation modes are available within this driver :

   *** Polling mode IO operation ***
   =================================
   [..]
     (+) Send and receive in the same time an amount of data in blocking mode using DAL_I2SEx_TransmitReceive()

   *** Interrupt mode IO operation ***
   ===================================
   [..]
     (+) Send and receive in the same time an amount of data in non blocking mode using DAL_I2SEx_TransmitReceive_IT()
     (+) At transmission/reception end of transfer DAL_I2SEx_TxRxCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2SEx_TxRxCpltCallback
     (+) In case of transfer Error, DAL_I2S_ErrorCallback() function is executed and user can
         add his own code by customization of function pointer DAL_I2S_ErrorCallback

   *** DMA mode IO operation ***
   ==============================
   [..]
     (+) Send and receive an amount of data in non blocking mode (DMA) using DAL_I2SEx_TransmitReceive_DMA()
     (+) At transmission/reception end of transfer DAL_I2SEx_TxRxCpltCallback is executed and user can
         add his own code by customization of function pointer DAL_I2S_TxRxCpltCallback
     (+) In case of transfer Error, DAL_I2S_ErrorCallback() function is executed and user can
         add his own code by customization of function pointer DAL_I2S_ErrorCallback
     (+) __DAL_I2SEXT_FLUSH_RX_DR: In Full-Duplex Slave mode, if DAL_I2S_DMAStop is used to stop the
         communication, an error DAL_I2S_ERROR_BUSY_LINE_RX is raised as the master continue to transmit data.
         In this case __DAL_I2SEXT_FLUSH_RX_DR macro must be used to flush the remaining data
         inside I2Sx and I2Sx_ext DATA registers and avoid using DeInit/Init process for the next transfer.
  @endverbatim

 Additional Figure: The Extended block uses the same clock sources as its master.

                +-----------------------+
    I2Sx_SCK    |                       |
 ----------+-->|          I2Sx         |------------------->I2Sx_SD(in/out)
         +--|-->|                       |
        |   |   +-----------------------+
        |   |
 I2S_WS |   |
 ------>|   |
        |   |   +-----------------------+
        |   +-->|                       |
        |       |       I2Sx_ext        |------------------->I2Sx_extSD(in/out)
         +----->|                       |
                +-----------------------+
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#ifdef DAL_I2S_MODULE_ENABLED

/** @defgroup I2SEx I2SEx
  * @brief I2S Extended DAL module driver
  * @{
  */

#if defined (SPI_I2S_FULLDUPLEX_SUPPORT)

/* Private typedef -----------------------------------------------------------*/
/** @defgroup I2SEx_Private_Typedef I2S Extended Private Typedef
  * @{
  */
typedef enum
{
  I2S_USE_I2S      = 0x00U,   /*!< I2Sx should be used      */
  I2S_USE_I2SEXT   = 0x01U,   /*!< I2Sx_ext should be used  */
} I2S_UseTypeDef;
/**
  * @}
  */
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup I2SEx_Private_Functions I2S Extended Private Functions
  * @{
  */
static void I2SEx_TxRxDMAHalfCplt(DMA_HandleTypeDef *hdma);
static void I2SEx_TxRxDMACplt(DMA_HandleTypeDef *hdma);
static void I2SEx_TxRxDMAError(DMA_HandleTypeDef *hdma);
static void I2SEx_RxISR_I2S(I2S_HandleTypeDef *hi2s);
static void I2SEx_RxISR_I2SExt(I2S_HandleTypeDef *hi2s);
static void I2SEx_TxISR_I2S(I2S_HandleTypeDef *hi2s);
static void I2SEx_TxISR_I2SExt(I2S_HandleTypeDef *hi2s);
static DAL_StatusTypeDef I2SEx_FullDuplexWaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s,
                                                                   uint32_t Flag,
                                                                   uint32_t State,
                                                                   uint32_t Timeout,
                                                                   I2S_UseTypeDef i2sUsed);
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @addtogroup I2SEx I2SEx
  * @{
  */

/** @addtogroup I2SEx_Exported_Functions I2S Extended Exported Functions
  * @{
  */

/** @defgroup I2SEx_Exported_Functions_Group1 I2S Extended IO operation functions
  *  @brief   I2SEx IO operation functions
  *
@verbatim
 ===============================================================================
                       ##### IO operation functions#####
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
        (++) DAL_I2SEx_TransmitReceive()

    (#) No-Blocking mode functions with Interrupt are :
        (++) DAL_I2SEx_TransmitReceive_IT()
        (++) DAL_I2SEx_FullDuplex_IRQHandler()

    (#) No-Blocking mode functions with DMA are :
        (++) DAL_I2SEx_TransmitReceive_DMA()

    (#) A set of Transfer Complete Callback are provided in non Blocking mode:
        (++) DAL_I2SEx_TxRxCpltCallback()
@endverbatim
  * @{
  */
/**
  * @brief  Full-Duplex Transmit/Receive data in blocking mode.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pTxData a 16-bit pointer to the Transmit data buffer.
  * @param  pRxData a 16-bit pointer to the Receive data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 16-bit data length.
  * @param  Timeout Timeout duration
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2SEx_TransmitReceive(I2S_HandleTypeDef *hi2s,
                                            uint16_t *pTxData,
                                            uint16_t *pRxData,
                                            uint16_t Size,
                                            uint32_t Timeout)
{
  uint32_t tmp1 = 0U;
  DAL_StatusTypeDef errorcode = DAL_OK;

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  tmp1 = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);
  /* Check the Data format: When a 16-bit data frame or a 16-bit data frame extended
     is selected during the I2S configuration phase, the Size parameter means the number
     of 16-bit data length in the transaction and when a 24-bit data frame or a 32-bit data
     frame is selected the Size parameter means the number of 16-bit data length. */
  if ((tmp1 == I2S_DATAFORMAT_24B) || (tmp1 == I2S_DATAFORMAT_32B))
  {
    hi2s->TxXferSize  = (Size << 1U);
    hi2s->TxXferCount = (Size << 1U);
    hi2s->RxXferSize  = (Size << 1U);
    hi2s->RxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->TxXferSize  = Size;
    hi2s->TxXferCount = Size;
    hi2s->RxXferSize  = Size;
    hi2s->RxXferCount = Size;
  }

  /* Set state and reset error code */
  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->State = DAL_I2S_STATE_BUSY_TX_RX;

  tmp1 = hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD;
  /* Check if the I2S_MODE_MASTER_TX or I2S_MODE_SLAVE_TX Mode is selected */
  if ((tmp1 == I2S_MODE_MASTER_TX) || (tmp1 == I2S_MODE_SLAVE_TX))
  {
    /* Prepare the First Data before enabling the I2S */
    hi2s->Instance->DATA = (*pTxData++);
    hi2s->TxXferCount--;

    /* Enable I2Sext(receiver) before enabling I2Sx peripheral */
    __DAL_I2SEXT_ENABLE(hi2s);

    /* Enable I2Sx peripheral */
    __DAL_I2S_ENABLE(hi2s);

    /* Check if Master Receiver mode is selected */
    if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD) == I2S_MODE_MASTER_TX)
    {
      /* Clear the Overrun Flag by a read operation on the SPI_DATA register followed by a read
      access to the SPI_STS register. */
      __DAL_I2SEXT_CLEAR_OVRFLAG(hi2s);
    }

    while ((hi2s->RxXferCount > 0U) || (hi2s->TxXferCount > 0U))
    {
      if (hi2s->TxXferCount > 0U)
      {
        /* Wait until TXE flag is set */
        if (I2SEx_FullDuplexWaitFlagStateUntilTimeout(hi2s, I2S_FLAG_TXE, SET, Timeout, I2S_USE_I2S) != DAL_OK)
        {
          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
          errorcode = DAL_ERROR;
          goto error;
        }
        /* Write Data on DATA register */
        hi2s->Instance->DATA = (*pTxData++);
        hi2s->TxXferCount--;

        /* Check if an underrun occurs */
        if ((__DAL_I2S_GET_FLAG(hi2s, I2S_FLAG_UDR) == SET) && (tmp1 == I2S_MODE_SLAVE_TX))
        {
          /* Clear Underrun flag */
          __DAL_I2S_CLEAR_UDRFLAG(hi2s);

          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_UDR);
        }
      }
      if (hi2s->RxXferCount > 0U)
      {
        /* Wait until RXNE flag is set */
        if (I2SEx_FullDuplexWaitFlagStateUntilTimeout(hi2s, I2S_FLAG_RXNE, SET, Timeout, I2S_USE_I2SEXT) != DAL_OK)
        {
          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
          errorcode = DAL_ERROR;
          goto error;
        }
        /* Read Data from DATA register */
        (*pRxData++) = I2SxEXT(hi2s->Instance)->DATA;
        hi2s->RxXferCount--;

        /* Check if an overrun occurs */
        if (__DAL_I2SEXT_GET_FLAG(hi2s, I2S_FLAG_OVR) == SET)
        {
          /* Clear Overrun flag */
          __DAL_I2S_CLEAR_OVRFLAG(hi2s);

          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_OVR);
        }
      }
    }
  }
  /* The I2S_MODE_MASTER_RX or I2S_MODE_SLAVE_RX Mode is selected */
  else
  {
    /* Prepare the First Data before enabling the I2S */
    I2SxEXT(hi2s->Instance)->DATA = (*pTxData++);
    hi2s->TxXferCount--;

    /* Enable I2Sext(transmitter) after enabling I2Sx peripheral */
    __DAL_I2SEXT_ENABLE(hi2s);

    /* Enable I2S peripheral before the I2Sext*/
    __DAL_I2S_ENABLE(hi2s);

    /* Check if Master Receiver mode is selected */
    if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD) == I2S_MODE_MASTER_RX)
    {
      /* Clear the Overrun Flag by a read operation on the SPI_DATA register followed by a read
      access to the SPI_STS register. */
      __DAL_I2S_CLEAR_OVRFLAG(hi2s);
    }

    while ((hi2s->RxXferCount > 0U) || (hi2s->TxXferCount > 0U))
    {
      if (hi2s->TxXferCount > 0U)
      {
        /* Wait until TXE flag is set */
        if (I2SEx_FullDuplexWaitFlagStateUntilTimeout(hi2s, I2S_FLAG_TXE, SET, Timeout, I2S_USE_I2SEXT) != DAL_OK)
        {
          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
          errorcode = DAL_ERROR;
          goto error;
        }
        /* Write Data on DATA register */
        I2SxEXT(hi2s->Instance)->DATA = (*pTxData++);
        hi2s->TxXferCount--;

        /* Check if an underrun occurs */
        if ((__DAL_I2SEXT_GET_FLAG(hi2s, I2S_FLAG_UDR) == SET) && (tmp1 == I2S_MODE_SLAVE_RX))
        {
          /* Clear Underrun flag */
          __DAL_I2S_CLEAR_UDRFLAG(hi2s);

          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_UDR);
        }
      }
      if (hi2s->RxXferCount > 0U)
      {
        /* Wait until RXNE flag is set */
        if (I2SEx_FullDuplexWaitFlagStateUntilTimeout(hi2s, I2S_FLAG_RXNE, SET, Timeout, I2S_USE_I2S) != DAL_OK)
        {
          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_TIMEOUT);
          errorcode = DAL_ERROR;
          goto error;
        }
        /* Read Data from DATA register */
        (*pRxData++) = hi2s->Instance->DATA;
        hi2s->RxXferCount--;

        /* Check if an overrun occurs */
        if (__DAL_I2S_GET_FLAG(hi2s, I2S_FLAG_OVR) == SET)
        {
          /* Clear Overrun flag */
          __DAL_I2S_CLEAR_OVRFLAG(hi2s);

          /* Set the error code */
          SET_BIT(hi2s->ErrorCode, DAL_I2S_ERROR_OVR);
        }
      }
    }
  }

  if (hi2s->ErrorCode != DAL_I2S_ERROR_NONE)
  {
    errorcode = DAL_ERROR;
  }

error :
  hi2s->State = DAL_I2S_STATE_READY;
  __DAL_UNLOCK(hi2s);
  return errorcode;
}

/**
  * @brief  Full-Duplex Transmit/Receive data in non-blocking mode using Interrupt
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pTxData a 16-bit pointer to the Transmit data buffer.
  * @param  pRxData a 16-bit pointer to the Receive data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 16-bit data length.
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2SEx_TransmitReceive_IT(I2S_HandleTypeDef *hi2s,
                                               uint16_t *pTxData,
                                               uint16_t *pRxData,
                                               uint16_t Size)
{
  uint32_t tmp1 = 0U;
  DAL_StatusTypeDef errorcode = DAL_OK;

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  hi2s->pTxBuffPtr = pTxData;
  hi2s->pRxBuffPtr = pRxData;

  tmp1 = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);
  /* Check the Data format: When a 16-bit data frame or a 16-bit data frame extended
  is selected during the I2S configuration phase, the Size parameter means the number
  of 16-bit data length in the transaction and when a 24-bit data frame or a 32-bit data
  frame is selected the Size parameter means the number of 16-bit data length. */
  if ((tmp1 == I2S_DATAFORMAT_24B) || (tmp1 == I2S_DATAFORMAT_32B))
  {
    hi2s->TxXferSize  = (Size << 1U);
    hi2s->TxXferCount = (Size << 1U);
    hi2s->RxXferSize  = (Size << 1U);
    hi2s->RxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->TxXferSize  = Size;
    hi2s->TxXferCount = Size;
    hi2s->RxXferSize  = Size;
    hi2s->RxXferCount = Size;
  }

  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->State     = DAL_I2S_STATE_BUSY_TX_RX;

  /* Set the function for IT treatment */
  if ((hi2s->Init.Mode == I2S_MODE_MASTER_TX) || (hi2s->Init.Mode == I2S_MODE_SLAVE_TX))
  {
    /* Enable I2Sext RXNE and ERR interrupts */
    __DAL_I2SEXT_ENABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

    /* Enable I2Sx TXE and ERR interrupts */
    __DAL_I2S_ENABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

    /* Transmit First data */
    hi2s->Instance->DATA = (*hi2s->pTxBuffPtr++);
    hi2s->TxXferCount--;

    if (hi2s->TxXferCount == 0U)
    {
      /* Disable TXE and ERR interrupt */
      __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));
    }
  }
  else  /* The I2S_MODE_MASTER_RX or I2S_MODE_SLAVE_RX Mode is selected */
  {
    /* Enable I2Sext TXE and ERR interrupts */
    __DAL_I2SEXT_ENABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

    /* Enable I2Sext RXNE and ERR interrupts */
    __DAL_I2S_ENABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

    /* Transmit First data */
    I2SxEXT(hi2s->Instance)->DATA = (*hi2s->pTxBuffPtr++);
    hi2s->TxXferCount--;

    if (hi2s->TxXferCount == 0U)
    {
      /* Disable I2Sext TXE and ERR interrupt */
      __DAL_I2SEXT_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));
    }
  }

  /* Enable I2Sext peripheral */
  __DAL_I2SEXT_ENABLE(hi2s);

  /* Enable I2S peripheral */
  __DAL_I2S_ENABLE(hi2s);

error :
  __DAL_UNLOCK(hi2s);
  return errorcode;
}

/**
  * @brief  Full-Duplex Transmit/Receive data in non-blocking mode using DMA
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @param  pTxData a 16-bit pointer to the Transmit data buffer.
  * @param  pRxData a 16-bit pointer to the Receive data buffer.
  * @param  Size number of data sample to be sent:
  * @note   When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  *         configuration phase, the Size parameter means the number of 16-bit data length
  *         in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  *         the Size parameter means the number of 16-bit data length.
  * @note   The I2S is kept enabled at the end of transaction to avoid the clock de-synchronization
  *         between Master and Slave(example: audio streaming).
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_I2SEx_TransmitReceive_DMA(I2S_HandleTypeDef *hi2s,
                                                uint16_t *pTxData,
                                                uint16_t *pRxData,
                                                uint16_t Size)
{
  uint32_t *tmp = NULL;
  uint32_t tmp1 = 0U;
  DAL_StatusTypeDef errorcode = DAL_OK;

  if (hi2s->State != DAL_I2S_STATE_READY)
  {
    errorcode = DAL_BUSY;
    goto error;
  }

  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
  {
    return  DAL_ERROR;
  }

  /* Process Locked */
  __DAL_LOCK(hi2s);

  hi2s->pTxBuffPtr = pTxData;
  hi2s->pRxBuffPtr = pRxData;

  tmp1 = hi2s->Instance->I2SCFG & (SPI_I2SCFG_DATALEN | SPI_I2SCFG_CHLEN);
  /* Check the Data format: When a 16-bit data frame or a 16-bit data frame extended
  is selected during the I2S configuration phase, the Size parameter means the number
  of 16-bit data length in the transaction and when a 24-bit data frame or a 32-bit data
  frame is selected the Size parameter means the number of 16-bit data length. */
  if ((tmp1 == I2S_DATAFORMAT_24B) || (tmp1 == I2S_DATAFORMAT_32B))
  {
    hi2s->TxXferSize  = (Size << 1U);
    hi2s->TxXferCount = (Size << 1U);
    hi2s->RxXferSize  = (Size << 1U);
    hi2s->RxXferCount = (Size << 1U);
  }
  else
  {
    hi2s->TxXferSize  = Size;
    hi2s->TxXferCount = Size;
    hi2s->RxXferSize  = Size;
    hi2s->RxXferCount = Size;
  }

  hi2s->ErrorCode = DAL_I2S_ERROR_NONE;
  hi2s->State     = DAL_I2S_STATE_BUSY_TX_RX;

  /* Set the I2S Rx DMA Half transfer complete callback */
  hi2s->hdmarx->XferHalfCpltCallback = I2SEx_TxRxDMAHalfCplt;

  /* Set the I2S Rx DMA transfer complete callback */
  hi2s->hdmarx->XferCpltCallback  = I2SEx_TxRxDMACplt;

  /* Set the I2S Rx DMA error callback */
  hi2s->hdmarx->XferErrorCallback = I2SEx_TxRxDMAError;

  /* Set the I2S Tx DMA Half transfer complete callback as NULL */
  hi2s->hdmatx->XferHalfCpltCallback  = NULL;

  /* Set the I2S Tx DMA transfer complete callback as NULL */
  hi2s->hdmatx->XferCpltCallback  = NULL;

  /* Set the I2S Tx DMA error callback */
  hi2s->hdmatx->XferErrorCallback = I2SEx_TxRxDMAError;

  tmp1 = hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD;
  /* Check if the I2S_MODE_MASTER_TX or I2S_MODE_SLAVE_TX Mode is selected */
  if ((tmp1 == I2S_MODE_MASTER_TX) || (tmp1 == I2S_MODE_SLAVE_TX))
  {
    /* Enable the Rx DMA Stream */
    tmp = (uint32_t *)&pRxData;
    DAL_DMA_Start_IT(hi2s->hdmarx, (uint32_t)&I2SxEXT(hi2s->Instance)->DATA, *(uint32_t *)tmp, hi2s->RxXferSize);

    /* Enable Rx DMA Request */
    SET_BIT(I2SxEXT(hi2s->Instance)->CTRL2, SPI_CTRL2_RXDEN);

    /* Enable the Tx DMA Stream */
    tmp = (uint32_t *)&pTxData;
    DAL_DMA_Start_IT(hi2s->hdmatx, *(uint32_t *)tmp, (uint32_t)&hi2s->Instance->DATA, hi2s->TxXferSize);

    /* Enable Tx DMA Request */
    SET_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_TXDEN);

    /* Check if the I2S is already enabled */
    if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SEN) != SPI_I2SCFG_I2SEN)
    {
      /* Enable I2Sext(receiver) before enabling I2Sx peripheral */
      __DAL_I2SEXT_ENABLE(hi2s);

      /* Enable I2S peripheral after the I2Sext */
      __DAL_I2S_ENABLE(hi2s);
    }
  }
  else
  {
    /* Check if Master Receiver mode is selected */
    if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD) == I2S_MODE_MASTER_RX)
    {
      /* Clear the Overrun Flag by a read operation on the SPI_DATA register followed by a read
      access to the SPI_STS register. */
      __DAL_I2S_CLEAR_OVRFLAG(hi2s);
    }
    /* Enable the Tx DMA Stream */
    tmp = (uint32_t *)&pTxData;
    DAL_DMA_Start_IT(hi2s->hdmatx, *(uint32_t *)tmp, (uint32_t)&I2SxEXT(hi2s->Instance)->DATA, hi2s->TxXferSize);

    /* Enable Tx DMA Request */
    SET_BIT(I2SxEXT(hi2s->Instance)->CTRL2, SPI_CTRL2_TXDEN);

    /* Enable the Rx DMA Stream */
    tmp = (uint32_t *)&pRxData;
    DAL_DMA_Start_IT(hi2s->hdmarx, (uint32_t)&hi2s->Instance->DATA, *(uint32_t *)tmp, hi2s->RxXferSize);

    /* Enable Rx DMA Request */
    SET_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_RXDEN);

    /* Check if the I2S is already enabled */
    if ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SEN) != SPI_I2SCFG_I2SEN)
    {
      /* Enable I2Sext(transmitter) before enabling I2Sx peripheral */
      __DAL_I2SEXT_ENABLE(hi2s);
      /* Enable I2S peripheral before the I2Sext */
      __DAL_I2S_ENABLE(hi2s);
    }
  }

error :
  __DAL_UNLOCK(hi2s);
  return errorcode;
}

/**
  * @brief  This function handles I2S/I2Sext interrupt requests in full-duplex mode.
  * @param  hi2s I2S handle
  * @retval DAL status
  */
void DAL_I2SEx_FullDuplex_IRQHandler(I2S_HandleTypeDef *hi2s)
{
  __IO uint32_t i2ssr     = hi2s->Instance->STS;
  __IO uint32_t i2sextsr  = I2SxEXT(hi2s->Instance)->STS;
  __IO uint32_t i2scr2    = hi2s->Instance->CTRL2;
  __IO uint32_t i2sextcr2 = I2SxEXT(hi2s->Instance)->CTRL2;

  /* Check if the I2S_MODE_MASTER_TX or I2S_MODE_SLAVE_TX Mode is selected */
  if ((hi2s->Init.Mode == I2S_MODE_MASTER_TX) || (hi2s->Init.Mode == I2S_MODE_SLAVE_TX))
  {
    /* I2S in mode Transmitter -------------------------------------------------*/
    if (((i2ssr & I2S_FLAG_TXE) == I2S_FLAG_TXE) && ((i2scr2 & I2S_IT_TXE) != RESET))
    {
      /* When the I2S mode is configured as I2S_MODE_MASTER_TX or I2S_MODE_SLAVE_TX,
      the I2S TXE interrupt will be generated to manage the full-duplex transmit phase. */
      I2SEx_TxISR_I2S(hi2s);
    }

    /* I2Sext in mode Receiver -----------------------------------------------*/
    if (((i2sextsr & I2S_FLAG_RXNE) == I2S_FLAG_RXNE) && ((i2sextcr2 & I2S_IT_RXNE) != RESET))
    {
      /* When the I2S mode is configured as I2S_MODE_MASTER_TX or I2S_MODE_SLAVE_TX,
      the I2Sext RXNE interrupt will be generated to manage the full-duplex receive phase. */
      I2SEx_RxISR_I2SExt(hi2s);
    }

    /* I2Sext Overrun error interrupt occurred --------------------------------*/
    if (((i2sextsr & I2S_FLAG_OVR) == I2S_FLAG_OVR) && ((i2sextcr2 & I2S_IT_ERR) != RESET))
    {
      /* Disable RXNE and ERR interrupt */
      __DAL_I2SEXT_DISABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

      /* Disable TXE and ERR interrupt */
      __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

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

    /* I2S Underrun error interrupt occurred ----------------------------------*/
    if (((i2ssr & I2S_FLAG_UDR) == I2S_FLAG_UDR) && ((i2scr2 & I2S_IT_ERR) != RESET))
    {
      /* Disable TXE and ERR interrupt */
      __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

      /* Disable RXNE and ERR interrupt */
      __DAL_I2SEXT_DISABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

      /* Clear underrun flag */
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
  /* The I2S_MODE_MASTER_RX or I2S_MODE_SLAVE_RX Mode is selected */
  else
  {
    /* I2Sext in mode Transmitter ----------------------------------------------*/
    if (((i2sextsr & I2S_FLAG_TXE) == I2S_FLAG_TXE) && ((i2sextcr2 & I2S_IT_TXE) != RESET))
    {
      /* When the I2S mode is configured as I2S_MODE_MASTER_RX or I2S_MODE_SLAVE_RX,
      the I2Sext TXE interrupt will be generated to manage the full-duplex transmit phase. */
      I2SEx_TxISR_I2SExt(hi2s);
    }

    /* I2S in mode Receiver --------------------------------------------------*/
    if (((i2ssr & I2S_FLAG_RXNE) == I2S_FLAG_RXNE) && ((i2scr2 & I2S_IT_RXNE) != RESET))
    {
      /* When the I2S mode is configured as I2S_MODE_MASTER_RX or I2S_MODE_SLAVE_RX,
      the I2S RXNE interrupt will be generated to manage the full-duplex receive phase. */
      I2SEx_RxISR_I2S(hi2s);
    }

    /* I2S Overrun error interrupt occurred -------------------------------------*/
    if (((i2ssr & I2S_FLAG_OVR) == I2S_FLAG_OVR) && ((i2scr2 & I2S_IT_ERR) != RESET))
    {
      /* Disable RXNE and ERR interrupt */
      __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

      /* Disable TXE and ERR interrupt */
      __DAL_I2SEXT_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

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

    /* I2Sext Underrun error interrupt occurred -------------------------------*/
    if (((i2sextsr & I2S_FLAG_UDR) == I2S_FLAG_UDR) && ((i2sextcr2 & I2S_IT_ERR) != RESET))
    {
      /* Disable TXE and ERR interrupt */
      __DAL_I2SEXT_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

      /* Disable RXNE and ERR interrupt */
      __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

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
  * @brief  Tx and Rx Transfer half completed callback
  * @param  hi2s I2S handle
  * @retval None
  */
__weak void DAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_I2SEx_TxRxHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Tx and Rx Transfer completed callback
  * @param  hi2s I2S handle
  * @retval None
  */
__weak void DAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2s);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_I2SEx_TxRxCpltCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup I2SEx_Private_Functions I2S Extended Private Functions
  * @{
  */

/**
  * @brief  DMA I2S transmit receive process half complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void I2SEx_TxRxDMAHalfCplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Call user TxRx Half complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  hi2s->TxRxHalfCpltCallback(hi2s);
#else
  DAL_I2SEx_TxRxHalfCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S transmit receive process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void I2SEx_TxRxDMACplt(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* If DMA is configured in DMA_NORMAL mode */
  if (hdma->Init.Mode == DMA_NORMAL)
  {
    if (((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD) == I2S_MODE_MASTER_TX) || \
        ((hi2s->Instance->I2SCFG & SPI_I2SCFG_I2SMOD) == I2S_MODE_SLAVE_TX))
    /* Disable Tx & Rx DMA Requests */
    {
      CLEAR_BIT(I2SxEXT(hi2s->Instance)->CTRL2, SPI_CTRL2_RXDEN);
      CLEAR_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_TXDEN);
    }
    else
    {
      CLEAR_BIT(hi2s->Instance->CTRL2, SPI_CTRL2_RXDEN);
      CLEAR_BIT(I2SxEXT(hi2s->Instance)->CTRL2, SPI_CTRL2_TXDEN);
    }

    hi2s->RxXferCount = 0U;
    hi2s->TxXferCount = 0U;

    hi2s->State = DAL_I2S_STATE_READY;
  }

  /* Call user TxRx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
  hi2s->TxRxCpltCallback(hi2s);
#else
  DAL_I2SEx_TxRxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA I2S communication error callback
  * @param  hdma DMA handle
  * @retval None
  */
static void I2SEx_TxRxDMAError(DMA_HandleTypeDef *hdma)
{
  I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Disable Rx and Tx DMA Request */
  CLEAR_BIT(hi2s->Instance->CTRL2, (SPI_CTRL2_RXDEN | SPI_CTRL2_TXDEN));
  CLEAR_BIT(I2SxEXT(hi2s->Instance)->CTRL2, (SPI_CTRL2_RXDEN | SPI_CTRL2_TXDEN));

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
  * @brief  I2S Full-Duplex IT handler transmit function
  * @param  hi2s I2S handle
  * @retval None
  */
static void I2SEx_TxISR_I2S(I2S_HandleTypeDef *hi2s)
{
  /* Write Data on DATA register */
  hi2s->Instance->DATA = (*hi2s->pTxBuffPtr++);
  hi2s->TxXferCount--;

  if (hi2s->TxXferCount == 0U)
  {
    /* Disable TXE and ERR interrupt */
    __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

    if (hi2s->RxXferCount == 0U)
    {
      hi2s->State = DAL_I2S_STATE_READY;
      /* Call user TxRx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
      hi2s->TxRxCpltCallback(hi2s);
#else
      DAL_I2SEx_TxRxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  I2SExt Full-Duplex IT handler transmit function
  * @param  hi2s I2S handle
  * @retval None
  */
static void I2SEx_TxISR_I2SExt(I2S_HandleTypeDef *hi2s)
{
  /* Write Data on DATA register */
  I2SxEXT(hi2s->Instance)->DATA = (*hi2s->pTxBuffPtr++);
  hi2s->TxXferCount--;

  if (hi2s->TxXferCount == 0U)
  {
    /* Disable I2Sext TXE and ERR interrupt */
    __DAL_I2SEXT_DISABLE_IT(hi2s, (I2S_IT_TXE | I2S_IT_ERR));

    if (hi2s->RxXferCount == 0U)
    {
      hi2s->State = DAL_I2S_STATE_READY;
      /* Call user TxRx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
      hi2s->TxRxCpltCallback(hi2s);
#else
      DAL_I2SEx_TxRxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  I2S Full-Duplex IT handler receive function
  * @param  hi2s I2S handle
  * @retval None
  */
static void I2SEx_RxISR_I2S(I2S_HandleTypeDef *hi2s)
{
  /* Read Data from DATA register */
  (*hi2s->pRxBuffPtr++) = hi2s->Instance->DATA;
  hi2s->RxXferCount--;

  if (hi2s->RxXferCount == 0U)
  {
    /* Disable RXNE and ERR interrupt */
    __DAL_I2S_DISABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

    if (hi2s->TxXferCount == 0U)
    {
      hi2s->State = DAL_I2S_STATE_READY;
      /* Call user TxRx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
      hi2s->TxRxCpltCallback(hi2s);
#else
      DAL_I2SEx_TxRxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief  I2SExt Full-Duplex IT handler receive function
  * @param  hi2s I2S handle
  * @retval None
  */
static void I2SEx_RxISR_I2SExt(I2S_HandleTypeDef *hi2s)
{
  /* Read Data from DATA register */
  (*hi2s->pRxBuffPtr++) = I2SxEXT(hi2s->Instance)->DATA;
  hi2s->RxXferCount--;

  if (hi2s->RxXferCount == 0U)
  {
    /* Disable I2Sext RXNE and ERR interrupt */
    __DAL_I2SEXT_DISABLE_IT(hi2s, (I2S_IT_RXNE | I2S_IT_ERR));

    if (hi2s->TxXferCount == 0U)
    {
      hi2s->State = DAL_I2S_STATE_READY;
      /* Call user TxRx complete callback */
#if (USE_DAL_I2S_REGISTER_CALLBACKS == 1U)
      hi2s->TxRxCpltCallback(hi2s);
#else
      DAL_I2SEx_TxRxCpltCallback(hi2s);
#endif /* USE_DAL_I2S_REGISTER_CALLBACKS */
    }
  }
}

/**
  * @brief This function handles I2S Communication Timeout.
  * @param hi2s I2S handle
  * @param Flag Flag checked
  * @param State Value of the flag expected
  * @param Timeout Duration of the timeout
  * @param i2sUsed I2S instance reference
  * @retval DAL status
  */
static DAL_StatusTypeDef I2SEx_FullDuplexWaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s,
                                                                   uint32_t Flag,
                                                                   uint32_t State,
                                                                   uint32_t Timeout,
                                                                   I2S_UseTypeDef i2sUsed)
{
  uint32_t tickstart = DAL_GetTick();

  if (i2sUsed == I2S_USE_I2S)
  {
    /* Wait until flag is reset */
    while (((__DAL_I2S_GET_FLAG(hi2s, Flag)) ? SET : RESET) != State)
    {
      if (Timeout != DAL_MAX_DELAY)
      {
        if ((Timeout == 0U) || ((DAL_GetTick() - tickstart) > Timeout))
        {
          /* Set the I2S State ready */
          hi2s->State = DAL_I2S_STATE_READY;

          /* Process Unlocked */
          __DAL_UNLOCK(hi2s);

          return DAL_TIMEOUT;
        }
      }
    }
  }
  else /* i2sUsed == I2S_USE_I2SEXT */
  {
    /* Wait until flag is reset */
    while (((__DAL_I2SEXT_GET_FLAG(hi2s, Flag)) ? SET : RESET) != State)
    {
      if (Timeout != DAL_MAX_DELAY)
      {
        if ((Timeout == 0U) || ((DAL_GetTick() - tickstart) > Timeout))
        {
          /* Set the I2S State ready */
          hi2s->State = DAL_I2S_STATE_READY;

          /* Process Unlocked */
          __DAL_UNLOCK(hi2s);

          return DAL_TIMEOUT;
        }
      }
    }
  }
  return DAL_OK;
}

/**
  * @}
  */
#endif /* SPI_I2S_FULLDUPLEX_SUPPORT */

/**
  * @}
  */
#endif /* DAL_I2S_MODULE_ENABLED */

/**
  * @}
  */

