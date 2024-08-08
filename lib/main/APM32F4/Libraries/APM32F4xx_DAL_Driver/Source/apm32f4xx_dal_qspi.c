/**
  *
  * @file    apm32f4xx_dal_qspi.c
  * @brief   QSPI DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the QSPI peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
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
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023-2024 Geehy Semiconductor.
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
    *** Initialization ***
    ======================
    [..]
      (#) As prerequisite, fill in the DAL_QSPI_MspInit() :
        (++) Enable the QSPI interface clock using __DAL_RCM_QSPI_CLK_ENABLE()
        (++) Reset QSPI peripheral with __DAL_RCM_QSPI_FORCE_RESET() and 
             __DAL_RCM_QSPI_RELEASE_RESET().
        (++) Enable the clocks for the QSPI GPIOS with __DAL_GPIO_QSPI_CLK_ENABLE().
        (++) Configure these QSPI pins in alternate function mode using __DAL_GPIO_Init().
        (++) If interrupt mode is used, enable and configure QSPI global
             interrupt with DAL_NVIC_SetPriority() and DAL_NVIC_EnableIRQ().
        (++) If DMA mode is used, enable the clocks for the QSPI DMA channel
             with __DAL_DMA_QSPI_CLK_ENABLE(), configure DMA with
             DAL_DMA_Init() function and link it with QSPI handle using
             __DAL_LINKDMA(), enable and configure DMA global interrupt with
             DAL_NVIC_SetPriority() and DAL_NVIC_EnableIRQ().
        (#) Configure the baudrate, the clock phase, the clock polarity, the clock stretch,
            the data frame size, the frame format, the FIFO threshold and the sample shifting
            using the DAL_QSPI_Init() function.

    *** Errors management and abort functionality ***
    ================================================
    [..]
      (#) DAL_QSPI_GetError() function gives the error raised during the last operation.
      (#) DAL_QSPI_Abort() and DAL_QSPI_Abort_IT() functions aborts any on-going operation and
          flushes the fifo :
        (++) In polling mode, the output of the function is done when the transfer
        complete bit is set and the busy bit cleared.
        (++) In interrupt mode, DAL_QSPI_AbortCpltCallback() will be called when 
        the transfer complete bit is set.

    *** Control functions ***
    =========================
    [..]
      (#) DAL_QSPI_GetState() function gives the current state of the DAL QSPI driver.
      (#) DAL_QSPI_SetTimeout() function configures the timeout value used in the driver.
      (#) DAL_QSPI_SetFifoThreshold() function configures the threshold on the FIFO of the QSPI IP.
      (#) DAL_QSPI_GetFifoThreshold() function gives the current of the Fifo's threshold.

    *** Callback registration ***
    =============================
    [..]
      The compilation define USE_DAL_QSPI_REGISTER_CALLBACKS when set to 1
      allows the user to configure dynamically the driver callbacks.

      Use Functions @ref DAL_QSPI_RegisterCallback() to register a user callback,
      it allows to register following callbacks:
        (+) ErrorCallback            : callback when error occurs.
        (+) AbortCpltCallback        : callback when abort is completed.
        (+) CmdCpltCallback          : callback when command is completed.
        (+) RxCpltCallback           : callback when receive is completed.
        (+) TxCpltCallback           : callback when transmit is completed.
        (+) TxRxCpltCallback         : callback when transmit and receive are completed.
        (+) MspInitCallback          : QSPI MspInit.
        (+) MspDeInitCallback        : QSPI MspDeInit.
      This function takes as parameters the HAL peripheral handle, the Callback ID
        and a pointer to the user callback function.

        Use function @ref DAL_QSPI_UnRegisterCallback() to reset a callback to the default
        weak (surcharged) function. It allows to reset following callbacks:
          (+) ErrorCallback            : callback when error occurs.
          (+) AbortCpltCallback        : callback when abort is completed.
          (+) CmdCpltCallback          : callback when command is completed.
          (+) RxCpltCallback           : callback when receive is completed.
          (+) TxCpltCallback           : callback when transmit is completed.
          (+) TxRxCpltCallback         : callback when transmit and receive are completed.
          (+) MspInitCallback          : QSPI MspInit.
          (+) MspDeInitCallback        : QSPI MspDeInit.
        This function) takes as parameters the DAL peripheral handle and the Callback ID.

        By default, after the DAL_QSPI_Init() and if the state is DAL_QSPI_STATE_RESET
        all callbacks are reset to the corresponding legacy weak (surcharged) functions.
        Exception done for MspInit and MspDeInit callbacks that are respectively
        reset to the legacy weak (surcharged) functions in the DAL_QSPI_Init()
        and DAL_QSPI_DeInit() only when these callbacks are null (not registered beforehand).
        If not, MspInit or MspDeInit are not null, the DAL_QSPI_Init() and DAL_QSPI_DeInit()
        keep and use the user MspInit/MspDeInit callbacks (registered beforehand).

        Callbacks can be registered/unregistered in DAL_QSPI_STATE_READY state only.
        Exception done MspInit/MspDeInit callbacks that can be registered/unregistered
        in DAL_QSPI_STATE_READY or DAL_QSPI_STATE_RESET state, thus registered (user)
        MspInit/DeInit callbacks can be used during the DAL_QSPI_Init()/DAL_QSPI_DeInit().
        In that case first register the MspInit/MspDeInit user callbacks
        using DAL_QSPI_RegisterCallback() before calling DAL_QSPI_DeInit()
        or DAL_QSPI_Init() function.

        When the compilation define USE_DAL_QSPI_REGISTER_CALLBACKS is set to 0 or
        not defined, the callback registration feature is not available and all callbacks
        are set to the corresponding legacy weak (surcharged) functions.
  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

#if defined(QSPI)

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup QSPI QSPI
 * @brief QSPI DAL module driver
  * @{
  */

#ifdef DAL_QSPI_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup QSPI_Private_Constants QSPI Private Constants
  * @{
  */

/* TFTL register clear mask */
#define QSPI_TFTL_CLEAR_MASK            ((uint32_t)(QSPI_TFTL_TFTH | QSPI_TFTL_TFT))

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void QSPI_DMATxCplt(DMA_HandleTypeDef *hdma);
static void QSPI_DMARxCplt(DMA_HandleTypeDef *hdma);
static void QSPI_DMAError(DMA_HandleTypeDef *hdma);
static void QSPI_DMAAbortCplt(DMA_HandleTypeDef *hdma);
static DAL_StatusTypeDef DAL_QSPI_WaitFlagStateUntilTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Flag, FlagStatus State, uint32_t TickStart, uint32_t Timeout);
static void QSPI_Config(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd);

/* Exported functions --------------------------------------------------------*/
/** @defgroup QSPI_Exported_Functions QSPI Exported Functions
  * @{
  */

/** @defgroup QSPI_Exported_Functions_Group1 Initialization and de-initialization functions
 * @brief    Initialization and Configuration functions
 * 
 * @verbatim
 * ===============================================================================
 *             ##### Initialization and de-initialization functions #####
 * ===============================================================================
 *   [..]
 *    This section provides functions allowing to:
 *     (+) Initialize the QSPI.
 *     (+) De-initialize the QSPI.
 * 
 * @endverbatim
  * @{
  */

/**
 * @brief  Initializes the QSPI according to the specified parameters
 *        in the DAL_QSPI_InitTypeDef and create the associated handle.
 * @param  hqspi: QSPI handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Init(QSPI_HandleTypeDef *hqspi)
{
    DAL_StatusTypeDef status = DAL_OK;
    uint32_t tickstart = DAL_GetTick();

    /* Check the QSPI handle allocation */
    if (hqspi == NULL)
    {
        return DAL_ERROR;
    }

    /* Check the parameters */
    ASSERT_PARAM(IS_QSPI_ALL_INSTANCE(hqspi->Instance));
    ASSERT_PARAM(IS_QSPI_CLOCK_PRESCALER(hqspi->Init.ClockPrescaler));
    ASSERT_PARAM(IS_QSPI_CLOCK_PHASE(hqspi->Init.ClockPhase));
    ASSERT_PARAM(IS_QSPI_CLOCK_POLARITY(hqspi->Init.ClockPolarity));
    ASSERT_PARAM(IS_QSPI_CLOCK_STRETCH(hqspi->Init.ClockStretch));
    ASSERT_PARAM(IS_QSPI_TX_FIFO_THRESHOLD(hqspi->Init.TxFifoThreshold));
    ASSERT_PARAM(IS_QSPI_TX_FIFO_LEVEL(hqspi->Init.TxFifoLevel));
    ASSERT_PARAM(IS_QSPI_RX_FIFO_THRESHOLD(hqspi->Init.RxFifoThreshold));
    ASSERT_PARAM(IS_QSPI_CHIP_SELECT_TOGGLE(hqspi->Init.ChipSelectToggle));

    if (hqspi->State == DAL_QSPI_STATE_RESET)
    {

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
        /* Reset callback pointers to the legacy weak (surcharged) functions */
        hqspi->ErrorCallback = DAL_QSPI_ErrorCallback;
        hqspi->AbortCpltCallback = DAL_QSPI_AbortCpltCallback;
        hqspi->CmdCpltCallback = DAL_QSPI_CmdCpltCallback;
        hqspi->RxCpltCallback = DAL_QSPI_RxCpltCallback;
        hqspi->TxCpltCallback = DAL_QSPI_TxCpltCallback;
        hqspi->TxRxCpltCallback = DAL_QSPI_TxRxCpltCallback;

        if (hqspi->MspInitCallback == NULL)
        {
            hqspi->MspInitCallback = DAL_QSPI_MspInit;
        }

        /* Init the low level hardware */
        hqspi->MspInitCallback(hqspi);
#else
        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        DAL_QSPI_MspInit(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */

        /* Configure the default timeout for the QSPI memory access */
        DAL_QSPI_SetTimeout(hqspi, DAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    }

    /* Configure QSPI FIFO Threshold */
    MODIFY_REG(hqspi->Instance->TFTL, QSPI_TFTL_CLEAR_MASK, \
                ((hqspi->Init.TxFifoThreshold << QSPI_TFTL_TFT_Pos) |
                 (hqspi->Init.TxFifoLevel << QSPI_TFTL_TFTH_Pos)));

    MODIFY_REG(hqspi->Instance->RFTL, QSPI_RFTL_RFT, hqspi->Init.RxFifoThreshold << QSPI_RFTL_RFT_Pos);

    /* Wait till BUSY flag reset */
    status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, hqspi->Timeout);

    if (status == DAL_OK)
    {
        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);

        /* Disable the Slave Select signal */
        __DAL_QSPI_DISABLE_SS(hqspi);

        /* Configure QSPI Parameters */
        MODIFY_REG(hqspi->Instance->CTRL1, \
                    (QSPI_CTRL1_CPHA | QSPI_CTRL1_CPOL | QSPI_CTRL1_SSTEN), \
                    (hqspi->Init.ClockPhase | hqspi->Init.ClockPolarity | hqspi->Init.ChipSelectToggle));

        /* Configure QSPI Clock Prescaler */
        MODIFY_REG(hqspi->Instance->BR, QSPI_BR_CLKDIV, hqspi->Init.ClockPrescaler << QSPI_BR_CLKDIV_Pos);

        /* Configure QSPI Clock Stretcher */
        MODIFY_REG(hqspi->Instance->CTRL3, QSPI_CTRL3_CSEN, hqspi->Init.ClockStretch << QSPI_CTRL3_CSEN_Pos);

        /* Enable the QSPI peripheral */
        __DAL_QSPI_ENABLE(hqspi);

        /* Set QSPI error code to none */
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        /* Initialize the QSPI state */
        hqspi->State = DAL_QSPI_STATE_READY;
    }

    /* Return function status */
    return status;
}

/**
 * @brief  De-Initializes the QSPI peripheral
 * @param  hqspi: QSPI handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_DeInit(QSPI_HandleTypeDef *hqspi)
{
    /* Check the QSPI handle allocation */
    if (hqspi == NULL)
    {
        return DAL_ERROR;
    }

    /* Check the parameters */
    ASSERT_PARAM(IS_QSPI_ALL_INSTANCE(hqspi->Instance));

    /* Disable the QSPI Peripheral */
    __DAL_QSPI_DISABLE(hqspi);

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
    if (hqspi->MspDeInitCallback == NULL)
    {
        hqspi->MspDeInitCallback = DAL_QSPI_MspDeInit;
    }

    /* DeInit the low level hardware */
    hqspi->MspDeInitCallback(hqspi);
#else
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    DAL_QSPI_MspDeInit(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */

    /* Set QSPI error code to none */
    hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

    /* Initialize the QSPI state */
    hqspi->State = DAL_QSPI_STATE_RESET;

    /* Return function status */
    return DAL_OK;
}

/**
 * @brief  Initializes the QSPI MSP.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_MspInit(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_MspInit could be implemented in the user file
     */
}

/**
 * @brief  DeInitializes QSPI MSP.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_MspDeInit could be implemented in the user file
     */
}

/**
  * @}
  */

/** @defgroup QSPI_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions
 * 
 * @verbatim
 * ===============================================================================
 *                ##### Input and Output operation functions #####
 * ===============================================================================
 *   [..]
 *   This subsection provides a set of functions allowing to :
 *      (+) Handle the interrupts.
 *      (+) Handle the command sequence.
 *      (+) Handle the data transmission.
 *      (+) Handle the data reception.
 * 
 * @endverbatim
  * @{
  */

/**
 * @brief  Handle QSPI interrupt request.
 * @param  hqspi: QSPI handle
 * @retval None
 */
void DAL_QSPI_IRQHandler(QSPI_HandleTypeDef *hqspi)
{
    __IO uint32_t *data_reg;
    uint32_t flag = READ_REG(hqspi->Instance->STS);
    uint32_t isrflags = READ_REG(hqspi->Instance->ISTS);
    uint32_t itsource = READ_REG(hqspi->Instance->INTEN);
    uint32_t errorFlags = 0x00U;

    errorFlags = (isrflags & (uint32_t)(QSPI_ISTS_TFOIF | QSPI_ISTS_RFUIF | QSPI_ISTS_RFOIF | QSPI_ISTS_MSTIF));

    /* If some errors occur */
    if (errorFlags != RESET)
    {
        /* Fifo TX overflow interrupt occurred --------------------------------*/
        if (((isrflags & QSPI_IT_TFO) != RESET) && ((itsource & QSPI_IT_TFO) != RESET))
        {
            /* Clear interrupt */
            __DAL_QSPI_CLEAR_FLAG(hqspi, QSPI_IT_TFO);

            /* Set QSPI error code to FIFO Threshold Overrun error */
            hqspi->ErrorCode |= DAL_QSPI_ERROR_TX_OVR;
        }

        /* Fifo RX underflow interrupt occurred -------------------------------*/
        if (((isrflags & QSPI_IT_RFU) != RESET) && ((itsource & QSPI_IT_RFU) != RESET))
        {
            /* Clear interrupt */
            __DAL_QSPI_CLEAR_FLAG(hqspi, QSPI_IT_RFU);

            /* Set QSPI error code to FIFO Threshold Underrun error */
            hqspi->ErrorCode |= DAL_QSPI_ERROR_RX_UDR;
        }

        /* Fifo RX overflow interrupt occurred --------------------------------*/
        if (((isrflags & QSPI_IT_RFO) != RESET) && ((itsource & QSPI_IT_RFO) != RESET))
        {
            /* Clear interrupt */
            __DAL_QSPI_CLEAR_FLAG(hqspi, QSPI_IT_RFO);

            /* Set QSPI error code to FIFO Threshold Overrun error */
            hqspi->ErrorCode |= DAL_QSPI_ERROR_RX_OVR;
        }

        /* Master complete interrupt occurred ---------------------------------*/
        if (((isrflags & QSPI_IT_MST) != RESET) && ((itsource & QSPI_IT_MST) != RESET))
        {
            /* Clear interrupt */
            __DAL_QSPI_CLEAR_FLAG(hqspi, QSPI_IT_MST);

            /* Set QSPI error code to Master error */
            hqspi->ErrorCode |= DAL_QSPI_ERROR_MST;
        }

        /* Call QSPI Error call back function if need be ----------------------*/
        if (hqspi->ErrorCode != DAL_QSPI_ERROR_NONE)
        {
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            /* Call error callback */
            hqspi->ErrorCallback(hqspi);
#else
            /* Call legacy weak error callback */
            DAL_QSPI_ErrorCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */

            hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;
        }
        return;
    }

    /* Fifo tx empty interrupt or rx full interrupt occurred ------------------*/
    if ((((flag & QSPI_FLAG_TFN) != RESET) && ((itsource & QSPI_IT_TFE) != RESET)) || \
        (((isrflags & QSPI_FLAG_RFF) != RESET) && ((itsource & QSPI_IT_RFF) != RESET)))
    {
        data_reg = &hqspi->Instance->DATA;

        if (hqspi->State == DAL_QSPI_STATE_BUSY_TX)
        {
            /* Transmission process */
            while (hqspi->TxXferCount > 0U)
            {
                while (__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_TFN) == RESET);

                /* Write data to TX FIFO */
                *((__IO uint8_t *)data_reg) = (*hqspi->pTxBuffPtr);
                hqspi->pTxBuffPtr++;
                hqspi->TxXferCount--;

                if (hqspi->TxXferCount == 0)
                {
                    /* Disable the QSPI FIFO Threshold interrupt */
                    __DAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_TFE);
                    break;
                }
            }

            /* Set QSPI state to Ready */
            hqspi->State = DAL_QSPI_STATE_READY;

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            /* Call Tx complete callback */
            hqspi->TxCpltCallback(hqspi);
#else
            /* Call legacy weak Tx complete callback */
            DAL_QSPI_TxCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
        }
        else if (hqspi->State == DAL_QSPI_STATE_BUSY_RX)
        {
            /* Receiving process */
            while (hqspi->RxXferCount > 0U)
            {
                while(__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_RFNE) == RESET);

                /* Read data from RX FIFO */
                *hqspi->pRxBuffPtr = *((__IO uint8_t *)data_reg);
                hqspi->pRxBuffPtr++;
                hqspi->RxXferCount--;

                if (hqspi->RxXferCount == 0)
                {
                    /* Disable the QSPI FIFO Threshold interrupt */
                    __DAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_RFF);
                    break;
                }
            }

            /* Set QSPI state to Ready */
            hqspi->State = DAL_QSPI_STATE_READY;

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            /* Call Rx complete callback */
            hqspi->RxCpltCallback(hqspi);
#else
            /* Call legacy weak Rx complete callback */
            DAL_QSPI_RxCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
        }
        else if (hqspi->State == DAL_QSPI_STATE_BUSY_TX_RX)
        {
            /* Transmission and receiving process */
            while (__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_TFN) != RESET)
            {
                if (hqspi->TxXferCount > 0U)
                {
                    /* Send the byte */
                    *((__IO uint8_t *)data_reg) = (*hqspi->pTxBuffPtr);
                    hqspi->pTxBuffPtr++;
                    hqspi->TxXferCount--;

                    while((__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_RFNE) != SET) || \
                            (__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_BUSY) != RESET));

                    /* Receive the byte */
                    *hqspi->pRxBuffPtr = *((__IO uint8_t *)data_reg);
                    hqspi->pRxBuffPtr++;
                    hqspi->RxXferCount--;
                }
                else
                {
                    /* No more data available for the transfer */
                    /* Disable the QSPI FIFO Threshold interrupt */
                    __DAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_TFE);
                    break;
                }
            }

            /* Set QSPI state to Ready */
            hqspi->State = DAL_QSPI_STATE_READY;

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            /* Call Tx and Rx complete callback */
            hqspi->TxRxCpltCallback(hqspi);
#else
            /* Call legacy weak Tx and Rx complete callback */
            DAL_QSPI_TxRxCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
        }
        else if(hqspi->State == DAL_QSPI_STATE_BUSY)
        {
            /* Set QSPI state to Ready */
            hqspi->State = DAL_QSPI_STATE_READY;

            /* Disable the QSPI FIFO Threshold interrupt */
            __DAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_TFE);

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            /* Call Command complete callback */
            hqspi->CmdCpltCallback(hqspi);
#else
            /* Call legacy weak Command complete callback */
            DAL_QSPI_CmdCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
        }
        else
        {
            /* Nothing to do */
        }

        /* Wait till BUSY flag reset */
        while (__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_BUSY) != RESET);
    }
}

/**
 * @brief  Set the command configuration.
 * @param  hqspi: QSPI handle
 * @param  cmd: structure that contains the command configuration information
 * @param  Timeout: Timeout duration
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Command(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, uint32_t Timeout)
{
    DAL_StatusTypeDef status;
    uint32_t tickstart = DAL_GetTick();

    /* Check the parameters */
    ASSERT_PARAM(IS_QSPI_INSTRUCTION_MODE(cmd->InstructionMode));
    ASSERT_PARAM(IS_QSPI_INSTRUCTION_SIZE(cmd->InstructionSize));
    ASSERT_PARAM(IS_QSPI_INSTRUCTION(cmd->Instruction));
    ASSERT_PARAM(IS_QSPI_ADDRESS_SIZE(cmd->AddressSize));
    ASSERT_PARAM(IS_QSPI_ADDRESS(cmd->Address));
    ASSERT_PARAM(IS_QSPI_TRANSFER_MODE(cmd->TransferMode));
    ASSERT_PARAM(IS_QSPI_FRAME_FORMAT(cmd->FrameFormat));
    ASSERT_PARAM(IS_QSPI_DATA_FRAME_SIZE(cmd->DataFrameSize));

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        hqspi->State = DAL_QSPI_STATE_BUSY;

        status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, Timeout);

        if (status == DAL_OK)
        {
            /* Disable the QSPI peripheral */
            __DAL_QSPI_DISABLE(hqspi);

            /* Call the configuration function */
            QSPI_Config(hqspi, cmd);

            /* Enable the Slave Select signal */
            __DAL_QSPI_ENABLE_SS(hqspi);

            /* Enable the QSPI peripheral */
            __DAL_QSPI_ENABLE(hqspi);

            /* Send the command */
            if (cmd->InstructionSize != QSPI_INSTRUCTION_SIZE_NONE)
            {
                WRITE_REG(hqspi->Instance->DATA, cmd->Instruction);
            }

            /* Send the address */
            if (cmd->AddressSize != QSPI_ADDRESS_SIZE_NONE)
            {
                WRITE_REG(hqspi->Instance->DATA, cmd->Address);
            }

            if (cmd->NbData == 0U)
            {
                if (status == DAL_OK)
                {
                    status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, hqspi->Timeout);
                }
            }

            if (status == DAL_OK)
            {
                /* Update QSPI state */
                hqspi->State = DAL_QSPI_STATE_READY;
            }
        }
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    /* Return function status */
    return status;
}

/**
 * @brief  Set the command configuration in interrupt mode.
 * @param  hqspi: QSPI handle
 * @param  cmd: structure that contains the command configuration information
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Command_IT(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd)
{
    DAL_StatusTypeDef status = DAL_OK;
    uint32_t tickstart = DAL_GetTick();

    /* Check the parameters */

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        hqspi->State = DAL_QSPI_STATE_BUSY;

        status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, hqspi->Timeout);

        if (status == DAL_OK)
        {
            /* Disable the QSPI peripheral */
            __DAL_QSPI_DISABLE(hqspi);

            /* Call the configuration function */
            QSPI_Config(hqspi, cmd);

            /* Enable the Slave Select signal */
            __DAL_QSPI_ENABLE_SS(hqspi);

            /* Enable the QSPI peripheral */
            __DAL_QSPI_ENABLE(hqspi);

            /* Send the command */
            if (cmd->InstructionSize != QSPI_INSTRUCTION_SIZE_NONE)
            {
                WRITE_REG(hqspi->Instance->DATA, cmd->Instruction);
            }

            /* Send the address */
            if (cmd->AddressSize != QSPI_ADDRESS_SIZE_NONE)
            {
                WRITE_REG(hqspi->Instance->DATA, cmd->Address);
            }

            if (cmd->NbData == 0U)
            {
                /* Process unlocked */
                __DAL_UNLOCK(hqspi);

                /* Enable the QSPI interrupt */
                __DAL_QSPI_ENABLE_IT(hqspi, (QSPI_IT_TFE));
            }
            else
            {
                /* Update QSPI state */
                hqspi->State = DAL_QSPI_STATE_READY;

                /* Process unlocked */
                __DAL_UNLOCK(hqspi);
            }
        }
        else
        {
            /* Process unlocked */
            __DAL_UNLOCK(hqspi);
        }
    }
    else
    {
        status = DAL_BUSY;

        /* Process unlocked */
        __DAL_UNLOCK(hqspi);
    }

    /* Return function status */
    return status;
}

/**
 * @brief  Transmit and Receive a byte in blocking mode.
 * @param  hqspi: QSPI handle
 * @param  data_in: received data
 * @param  data_out: transmitted data
 * @param  Size: number of bytes to transmit and receive
 * @param  Timeout: Timeout duration
 * @retval DAL status
 * @note   This function is used only in case of standard communication mode.
 */
DAL_StatusTypeDef DAL_QSPI_TransmitReceive(QSPI_HandleTypeDef *hqspi, uint8_t *data_out, uint8_t *data_in, uint32_t Size, uint32_t Timeout)
{
    DAL_StatusTypeDef status = DAL_OK;
    uint32_t tickstart = DAL_GetTick();
    __IO uint32_t *data_reg = &hqspi->Instance->DATA;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        if ((data_in != NULL) && (data_out != NULL))
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_BUSY_TX_RX;

            hqspi->pTxBuffPtr = data_out;
            hqspi->TxXferSize = Size;
            hqspi->TxXferCount = Size;

            hqspi->pRxBuffPtr = data_in;
            hqspi->RxXferSize = Size;
            hqspi->RxXferCount = Size;

            /* Disable the QSPI peripheral */
            __DAL_QSPI_DISABLE(hqspi);

            /* Configure CTRL1 */
            MODIFY_REG(hqspi->Instance->CTRL1, \
                        (QSPI_CTRL1_DFS | QSPI_CTRL1_FRF | QSPI_CTRL1_TXMODE), \
                        (QSPI_DATA_FRAME_SIZE_8BITS | QSPI_FRAME_FORMAT_STANDARD | QSPI_TRANSFER_MODE_TX_RX));

            /* Enable the Slave Select signal */
            __DAL_QSPI_ENABLE_SS(hqspi);

            /* Enable the QSPI peripheral */
            __DAL_QSPI_ENABLE(hqspi);

            /* Transmit and Receive data */
            while (hqspi->TxXferCount > 0U)
            {
                /* Wait till TFN flag set */
                status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_TFN, SET, tickstart, Timeout);

                if (status != DAL_OK)
                {
                    break;
                }
                
                /* Send the byte */
                *((__IO uint8_t *)data_reg) = (*hqspi->pTxBuffPtr);
                hqspi->pTxBuffPtr++;
                hqspi->TxXferCount--;

                /* Wait till RFNE flag set or BUSY flag reset */
                while (((__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_RFNE)) != SET) || \
                        ((__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_BUSY)) != RESET))
                {
                    /* Check for the Timeout */
                    if (Timeout != DAL_MAX_DELAY)
                    {
                        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
                        {
                            hqspi->State = DAL_QSPI_STATE_ERROR;
                            hqspi->ErrorCode |= DAL_QSPI_ERROR_TIMEOUT;

                            status = DAL_ERROR;
                        }
                    }
                }

                if (status != DAL_OK)
                {
                    break;
                }

                /* Receive the byte */
                *hqspi->pRxBuffPtr = *((__IO uint8_t *)data_reg);
                hqspi->pRxBuffPtr++;
                hqspi->RxXferCount--;
            }

            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
 * @param  hqspi: QSPI handle
 * @param  data_in: received data
 * @param  data_out: transmitted data
 * @param  Size: number of bytes to transmit and receive
 * @retval DAL status
 * @note   This function is used only in case of standard communication mode.
 */
DAL_StatusTypeDef DAL_QSPI_TransmitReceive_IT(QSPI_HandleTypeDef *hqspi, uint8_t *data_out, uint8_t *data_in, uint32_t Size)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        if ((data_in != NULL) && (data_out != NULL))
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_BUSY_TX_RX;

            hqspi->pTxBuffPtr = data_out;
            hqspi->TxXferSize = Size;
            hqspi->TxXferCount = Size;

            hqspi->pRxBuffPtr = data_in;
            hqspi->RxXferSize = Size;
            hqspi->RxXferCount = Size;

            /* Disable the QSPI peripheral */
            __DAL_QSPI_DISABLE(hqspi);

            /* Configure CTRL1 */
            MODIFY_REG(hqspi->Instance->CTRL1, \
                        (QSPI_CTRL1_DFS | QSPI_CTRL1_FRF | QSPI_CTRL1_TXMODE), \
                        (QSPI_DATA_FRAME_SIZE_8BITS | QSPI_FRAME_FORMAT_STANDARD | QSPI_TRANSFER_MODE_TX_RX));

            /* Process unlocked */
            __DAL_UNLOCK(hqspi);

            /* Enable the QSPI interrupt */
            __DAL_QSPI_ENABLE_IT(hqspi, (QSPI_IT_TFE));

            /* Enable the Slave Select signal */
            __DAL_QSPI_ENABLE_SS(hqspi);

            /* Enable the QSPI peripheral */
            __DAL_QSPI_ENABLE(hqspi);

        }
        else
        {
            hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_PARAM;
            status = DAL_ERROR;

            /* Process unlocked */
            __DAL_UNLOCK(hqspi);
        }
    }
    else
    {
        status = DAL_BUSY;

        /* Process unlocked */
        __DAL_UNLOCK(hqspi);
    }

    return status;
}

/**
 * @brief  Transmit an amount of data in blocking mode.
 * @param  hqspi: QSPI handle
 * @param  pData: pointer to data buffer
 * @param  Timeout: Timeout duration
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Transmit(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout)
{
    DAL_StatusTypeDef status = DAL_OK;
    uint32_t tickstart = DAL_GetTick();
    __IO uint32_t *data_reg = &hqspi->Instance->DATA;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        if (pData != NULL)
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_BUSY_TX;

            hqspi->pTxBuffPtr = pData;
            hqspi->TxXferSize = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;
            hqspi->TxXferCount = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;

            /* Transmit and Receive data */
            while (hqspi->TxXferCount > 0U)
            {
                /* Wait till TFN flag set */
                status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_TFN, SET, tickstart, Timeout);

                if (status != DAL_OK)
                {
                    break;
                }
                
                /* Send the byte */
                *((__IO uint8_t *)data_reg) = (*hqspi->pTxBuffPtr);
                hqspi->pTxBuffPtr++;
                hqspi->TxXferCount--;
            }

            if (status == DAL_OK)
            {
                /* Wait till BUSY flag reset */
                status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, Timeout);
            }

            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_READY;
        }
        else
        {
            hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_PARAM;
            status = DAL_ERROR;
        }
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
 * @param  hqspi: QSPI handle
 * @param  pData: pointer to data buffer
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Transmit_IT(QSPI_HandleTypeDef *hqspi, uint8_t *pData)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        if (pData != NULL)
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_BUSY_TX;

            hqspi->pTxBuffPtr = pData;
            hqspi->TxXferSize = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;
            hqspi->TxXferCount = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;

            /* Clear interrupt */
            __DAL_QSPI_CLEAR_FLAG(hqspi, QSPI_IT_TFE);

            /* Process unlocked */
            __DAL_UNLOCK(hqspi);

            /* Enable the QSPI interrupt */
            __DAL_QSPI_ENABLE_IT(hqspi, (QSPI_IT_TFE));
        }
        else
        {
            hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_PARAM;
            status = DAL_ERROR;

            /* Process unlocked */
            __DAL_UNLOCK(hqspi);
        }
    }
    else
    {
        status = DAL_BUSY;

        /* Process unlocked */
        __DAL_UNLOCK(hqspi);
    }

    return status;
}

/**
 * @brief  Transmit an amount of data in non-blocking mode with DMA.
 * @param  hqspi: QSPI handle
 * @param  pData: pointer to data buffer
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Transmit_DMA(QSPI_HandleTypeDef *hqspi, uint8_t *pData)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Clear the error code */
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        if (pData != NULL)
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_BUSY_TX;

            hqspi->pTxBuffPtr = pData;
            hqspi->TxXferSize = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;
            hqspi->TxXferCount = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;

            /* Set the QSPI DMA transfer complete callback */
            hqspi->hdmatx->XferCpltCallback = QSPI_DMATxCplt;

            /* Set the QSPI DMA error callback */
            hqspi->hdmatx->XferErrorCallback = QSPI_DMAError;

            /* Clear the DMA abort callback */
            hqspi->hdmatx->XferAbortCallback = NULL;

            /* Enable the DMA stream */
            if (DAL_DMA_Start_IT(hqspi->hdmatx, (uint32_t)hqspi->pTxBuffPtr, (uint32_t)&hqspi->Instance->DATA, hqspi->TxXferCount) == DAL_OK)
            {
                /* Clear interrupt */
                __DAL_QSPI_CLEAR_FLAG(hqspi, QSPI_IT_TFE);

                /* Process unlocked */
                __DAL_UNLOCK(hqspi);

                /* Enable the QSPI DMA transfer for transmit request by setting the TDMAEN bit
                in the QSPI DMACTRL register */
                SET_BIT(hqspi->Instance->DMACTRL, QSPI_DMACTRL_TDMAEN);
            }
            else
            {
                status = DAL_ERROR;
                hqspi->ErrorCode |= DAL_QSPI_ERROR_DMA;
                hqspi->State = DAL_QSPI_STATE_READY;

                /* Process unlocked */
                __DAL_UNLOCK(hqspi);
            }
        }
        else
        {
            hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_PARAM;
            status = DAL_ERROR;

            /* Process unlocked */
            __DAL_UNLOCK(hqspi);
        }
    }
    else
    {
        status = DAL_BUSY;

        /* Process unlocked */
        __DAL_UNLOCK(hqspi);
    }

    return status;
}

/**
 * @brief  Receive an amount of data in blocking mode.
 * @param  hqspi: QSPI handle
 * @param  pData: pointer to data buffer
 * @param  Timeout: Timeout duration
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Receive(QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout)
{
    DAL_StatusTypeDef status;
    uint32_t tickstart = DAL_GetTick();
    __IO uint32_t *data_reg = &hqspi->Instance->DATA;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        if (pData != NULL)
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_BUSY_RX;

            hqspi->pRxBuffPtr = pData;
            hqspi->RxXferSize = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;
            hqspi->RxXferCount = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;

            while (hqspi->RxXferCount > 0U)
            {
                /* Wait till RFNE flag set */
                status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_RFNE, SET, tickstart, Timeout);

                if (status != DAL_OK)
                {
                    break;
                }

                /* Receive the byte */
                *hqspi->pRxBuffPtr = *((__IO uint8_t *)data_reg);
                hqspi->pRxBuffPtr++;
                hqspi->RxXferCount--;
            }

            if (status == DAL_OK)
            {
                /* Wait till BUSY flag reset */
                status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, Timeout);
            }

            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_READY;
        }
        else
        {
            hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_PARAM;
            status = DAL_ERROR;
        }
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Receive an amount of data in non-blocking mode with Interrupt.
 * @param  hqspi: QSPI handle
 * @param  pData: pointer to data buffer
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Receive_IT(QSPI_HandleTypeDef *hqspi, uint8_t *pData)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        if (pData != NULL)
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_BUSY_RX;

            hqspi->pRxBuffPtr = pData;
            hqspi->RxXferSize = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;
            hqspi->RxXferCount = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;

            /* Clear interrupt */
            __DAL_QSPI_CLEAR_FLAG(hqspi, QSPI_IT_RFF);

            /* Process unlocked */
            __DAL_UNLOCK(hqspi);

            /* Enable the QSPI interrupt */
            __DAL_QSPI_ENABLE_IT(hqspi, (QSPI_IT_RFF));
        }
        else
        {
            hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_PARAM;
            status = DAL_ERROR;

            /* Process unlocked */
            __DAL_UNLOCK(hqspi);
        }
    }
    else
    {
        status = DAL_BUSY;

        /* Process unlocked */
        __DAL_UNLOCK(hqspi);
    }

    return status;
}

/**
 * @brief  Receive an amount of data in non-blocking mode with DMA.
 * @param  hqspi: QSPI handle
 * @param  pData: pointer to data buffer
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Receive_DMA(QSPI_HandleTypeDef *hqspi, uint8_t *pData)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Clear the error code */
        hqspi->ErrorCode = DAL_QSPI_ERROR_NONE;

        if (pData != NULL)
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_BUSY_RX;

            hqspi->pRxBuffPtr = pData;
            hqspi->RxXferSize = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;
            hqspi->RxXferCount = READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) + 1U;

            /* Set the QSPI DMA transfer complete callback */
            hqspi->hdmarx->XferCpltCallback = QSPI_DMARxCplt;

            /* Set the QSPI DMA error callback */
            hqspi->hdmarx->XferErrorCallback = QSPI_DMAError;

            /* Clear the DMA abort callback */
            hqspi->hdmarx->XferAbortCallback = NULL;

            /* Enable the DMA stream */
            if (DAL_DMA_Start_IT(hqspi->hdmarx, (uint32_t)&hqspi->Instance->DATA, (uint32_t)hqspi->pRxBuffPtr, hqspi->RxXferCount) == DAL_OK)
            {
                /* Clear interrupt */
                __DAL_QSPI_CLEAR_FLAG(hqspi, QSPI_IT_RFF);

                /* Process unlocked */
                __DAL_UNLOCK(hqspi);

                /* Enable the QSPI DMA transfer for receive request by setting the RDMAEN bit
                in the QSPI DMACTRL register */
                SET_BIT(hqspi->Instance->DMACTRL, QSPI_DMACTRL_RDMAEN);
            }
            else
            {
                status = DAL_ERROR;
                hqspi->ErrorCode |= DAL_QSPI_ERROR_DMA;
                hqspi->State = DAL_QSPI_STATE_READY;

                /* Process unlocked */
                __DAL_UNLOCK(hqspi);
            }
        }
        else
        {
            hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_PARAM;
            status = DAL_ERROR;

            /* Process unlocked */
            __DAL_UNLOCK(hqspi);
        }
    }
    else
    {
        status = DAL_BUSY;

        /* Process unlocked */
        __DAL_UNLOCK(hqspi);
    }

    return status;
}

/**
 * @brief  Transfer error callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_ErrorCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_ErrorCallback could be implemented in the user file
     */
}

/**
 * @brief  Abort completed callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_AbortCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_AbortCpltCallback could be implemented in the user file
     */
}

/**
 * @brief  FIFO threshold callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_FifoThresholdCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_FifoThresholdCallback could be implemented in the user file
     */
}

/**
 * @brief  Command completed callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_CmdCpltCallback could be implemented in the user file
     */
}

/**
 * @brief  Rx Transfer completed callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_RxCpltCallback could be implemented in the user file
     */
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_TxCpltCallback could be implemented in the user file
     */
}

/**
 * @brief  Tx and Rx Transfer completed callback.
 * @param  hqspi: QSPI handle
 * @retval None
 */
__weak void DAL_QSPI_TxRxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hqspi);

    /* NOTE : This function should not be modified, when the callback is needed,
              the DAL_QSPI_TxRxCpltCallback could be implemented in the user file
     */
}

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
/**
 * @brief  Register a User QSPI Callback
 *         To be used instead of the weak predefined callback
 * @param  hqspi: QSPI handle
 * @param  CallbackID: ID of the callback to be registered
 *                    This parameter can be one of the following values:
 *                    @arg @ref DAL_QSPI_ERROR_CB_ID          QSPI Error Callback ID
 *                    @arg @ref DAL_QSPI_ABORT_CB_ID          QSPI Abort Callback ID
 *                    @arg @ref DAL_QSPI_CMD_CPLT_CB_ID       QSPI Command Complete Callback ID
 *                    @arg @ref DAL_QSPI_RX_CPLT_CB_ID        QSPI Rx Complete Callback ID
 *                    @arg @ref DAL_QSPI_TX_CPLT_CB_ID        QSPI Tx Complete Callback ID
 *                    @arg @ref DAL_QSPI_TX_RX_CPLT_CB_ID     QSPI Tx Rx Complete Callback ID
 *                    @arg @ref DAL_QSPI_MSP_INIT_CB_ID       QSPI MspInit Callback ID
 *                    @arg @ref DAL_QSPI_MSP_DEINIT_CB_ID     QSPI MspDeInit Callback ID
 * @param  pCallback: pointer to the Callback function
 * @retval status
 */
DAL_StatusTypeDef DAL_QSPI_RegisterCallback(QSPI_HandleTypeDef *hqspi, DAL_QSPI_CallbackIDTypeDef CallbackID, pQSPI_CallbackTypeDef pCallback)
{
    DAL_StatusTypeDef status = DAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_CALLBACK;

        return DAL_ERROR;
    }

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        switch (CallbackID)
        {
            case DAL_QSPI_ERROR_CB_ID :
                hqspi->ErrorCallback = pCallback;
                break;

            case DAL_QSPI_ABORT_CB_ID :
                hqspi->AbortCpltCallback = pCallback;
                break;

            case DAL_QSPI_CMD_CPLT_CB_ID :
                hqspi->CmdCpltCallback = pCallback;
                break;

            case DAL_QSPI_RX_CPLT_CB_ID :
                hqspi->RxCpltCallback = pCallback;
                break;

            case DAL_QSPI_TX_CPLT_CB_ID :
                hqspi->TxCpltCallback = pCallback;
                break;

            case DAL_QSPI_TX_RX_CPLT_CB_ID :
                hqspi->TxRxCpltCallback = pCallback;
                break;

            case DAL_QSPI_MSP_INIT_CB_ID :
                hqspi->MspInitCallback = pCallback;
                break;

            case DAL_QSPI_MSP_DEINIT_CB_ID :
                hqspi->MspDeInitCallback = pCallback;
                break;

            default :
                /* Update the error code */
                hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_CALLBACK;

                /* Return error status */
                status =  DAL_ERROR;
                break;
        }
    }
    else if (hqspi->State == DAL_QSPI_STATE_RESET)
    {
        switch (CallbackID)
        {
            case DAL_QSPI_MSP_INIT_CB_ID :
                hqspi->MspInitCallback = pCallback;
                break;

            case DAL_QSPI_MSP_DEINIT_CB_ID :
                hqspi->MspDeInitCallback = pCallback;
                break;

            default :
                /* Update the error code */
                hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_CALLBACK;

                /* Return error status */
                status =  DAL_ERROR;
                break;
        }
    }
    else
    {
        /* Update the error code */
        hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_CALLBACK;

        /* Update return status */
        status = DAL_ERROR;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Unregister a User QSPI Callback
 *         QSPI Callback is redirected to the weak predefined callback
 * @param  hqspi: QSPI handle
 * @param  CallbackID: ID of the callback to be unregistered
 *                    This parameter can be one of the following values:
 *                    @arg @ref DAL_QSPI_ERROR_CB_ID          QSPI Error Callback ID
 *                    @arg @ref DAL_QSPI_ABORT_CB_ID          QSPI Abort Callback ID
 *                    @arg @ref DAL_QSPI_CMD_CPLT_CB_ID       QSPI Command Complete Callback ID
 *                    @arg @ref DAL_QSPI_RX_CPLT_CB_ID        QSPI Rx Complete Callback ID
 *                    @arg @ref DAL_QSPI_TX_CPLT_CB_ID        QSPI Tx Complete Callback ID
 *                    @arg @ref DAL_QSPI_TX_RX_CPLT_CB_ID     QSPI Tx Rx Complete Callback ID
 *                    @arg @ref DAL_QSPI_MSP_INIT_CB_ID       QSPI MspInit Callback ID
 *                    @arg @ref DAL_QSPI_MSP_DEINIT_CB_ID     QSPI MspDeInit Callback ID
 * @retval status
 */
DAL_StatusTypeDef DAL_QSPI_UnRegisterCallback(QSPI_HandleTypeDef *hqspi, DAL_QSPI_CallbackIDTypeDef CallbackID)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        switch (CallbackID)
        {
            case DAL_QSPI_ERROR_CB_ID :
                hqspi->ErrorCallback = DAL_QSPI_ErrorCallback; /* Legacy weak (surcharged) callback */
                break;

            case DAL_QSPI_ABORT_CB_ID :
                hqspi->AbortCpltCallback = DAL_QSPI_AbortCpltCallback; /* Legacy weak (surcharged) callback */
                break;

            case DAL_QSPI_CMD_CPLT_CB_ID :
                hqspi->CmdCpltCallback = DAL_QSPI_CmdCpltCallback; /* Legacy weak (surcharged) callback */
                break;

            case DAL_QSPI_RX_CPLT_CB_ID :
                hqspi->RxCpltCallback = DAL_QSPI_RxCpltCallback; /* Legacy weak (surcharged) callback */
                break;

            case DAL_QSPI_TX_CPLT_CB_ID :
                hqspi->TxCpltCallback = DAL_QSPI_TxCpltCallback; /* Legacy weak (surcharged) callback */
                break;

            case DAL_QSPI_TX_RX_CPLT_CB_ID :
                hqspi->TxRxCpltCallback = DAL_QSPI_TxRxCpltCallback; /* Legacy weak (surcharged) callback */
                break;

            case DAL_QSPI_MSP_INIT_CB_ID :
                hqspi->MspInitCallback = DAL_QSPI_MspInit; /* Legacy weak (surcharged) callback */
                break;

            case DAL_QSPI_MSP_DEINIT_CB_ID :
                hqspi->MspDeInitCallback = DAL_QSPI_MspDeInit; /* Legacy weak (surcharged) callback */
                break;

            default :
                /* Update the error code */
                hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_CALLBACK;

                /* Return error status */
                status =  DAL_ERROR;
                break;
        }
    }
    else if (hqspi->State == DAL_QSPI_STATE_RESET)
    {
        switch (CallbackID)
        {
            case DAL_QSPI_MSP_INIT_CB_ID :
                hqspi->MspInitCallback = DAL_QSPI_MspInit; /* Legacy weak (surcharged) callback */
                break;

            case DAL_QSPI_MSP_DEINIT_CB_ID :
                hqspi->MspDeInitCallback = DAL_QSPI_MspDeInit; /* Legacy weak (surcharged) callback */
                break;

            default :
                /* Update the error code */
                hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_CALLBACK;

                /* Return error status */
                status =  DAL_ERROR;
                break;
        }
    }
    else
    {
        /* Update the error code */
        hqspi->ErrorCode |= DAL_QSPI_ERROR_INVALID_CALLBACK;

        /* Update return status */
        status = DAL_ERROR;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup QSPI_Exported_Functions_Group3 Peripheral Control functions
 *   @brief   QSPI control and state functions
 * 
 * @verbatim
 * ===============================================================================
 *                 ##### Peripheral Control and State functions #####
 * ===============================================================================
 *  [..]
 *  This subsection provides a set of functions allowing to :
 *    (+) Check in run-time the state of the driver. 
 *    (+) Check the error code set during last operation.
 *    (+) Abort any operation.
 * 
 * @endverbatim
  * @{
  */

/**
 * @brief  Reture the QSPI handle state.
 * @param  hqspi QSPI handle
 * @retval DAL state
 */
DAL_QSPI_StateTypeDef DAL_QSPI_GetState(QSPI_HandleTypeDef *hqspi)
{
    /* Return QSPI handle state */
    return hqspi->State;
}

/**
 * @brief  Return the QSPI error code.
 * @param  hqspi QSPI handle
 * @retval DAL error code
 */
uint32_t DAL_QSPI_GetError(QSPI_HandleTypeDef *hqspi)
{
    /* Return QSPI error code */
    return hqspi->ErrorCode;
}

/**
 * @brief  Abort the current transmission.
 * @param  hqspi QSPI handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Abort(QSPI_HandleTypeDef *hqspi)
{
    DAL_StatusTypeDef status = DAL_OK;
    uint32_t tickstart = DAL_GetTick();

    /* Check if the state is in one of the busy states */
    if (((uint32_t)hqspi->State & 0x02) != 0U)
    {
        /* Process locked */
        __DAL_LOCK(hqspi);

        if ((hqspi->Instance->DMACTRL & QSPI_DMACTRL_TDMAEN) != 0U)
        {
            /* Disable the QSPI DMA Tx request */
            CLEAR_BIT(hqspi->Instance->DMACTRL, QSPI_DMACTRL_TDMAEN);

            /* Abort the QSPI DMA Tx channel */
            if (DAL_DMA_Abort(hqspi->hdmatx) != DAL_OK)
            {
                hqspi->ErrorCode |= DAL_QSPI_ERROR_DMA;
            }
        }

        if ((hqspi->Instance->DMACTRL & QSPI_DMACTRL_RDMAEN) != 0U)
        {
            /* Disable the QSPI DMA Rx request */
            CLEAR_BIT(hqspi->Instance->DMACTRL, QSPI_DMACTRL_RDMAEN);

            /* Abort the QSPI DMA Rx channel */
            if (DAL_DMA_Abort(hqspi->hdmarx) != DAL_OK)
            {
                hqspi->ErrorCode |= DAL_QSPI_ERROR_DMA;
            }
        }

        if (__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_BUSY) != RESET)
        {
            /* Disable slave select */
            __DAL_QSPI_DISABLE_SS(hqspi);

            /* Disable the QSPI peripheral */
            __DAL_QSPI_DISABLE(hqspi);

            /* Wait till BUSY flag reset */
            status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, hqspi->Timeout);
        }
        else
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_READY;
        }
    }

    return status;
}

/**
 * @brief  Abort the current transmission (non-blocking mode).
 * @param  hqspi QSPI handle
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_Abort_IT(QSPI_HandleTypeDef *hqspi)
{
    DAL_StatusTypeDef status = DAL_OK;
    uint32_t tickstart = DAL_GetTick();

    /* Check if the state is in one of the busy states */
    if (((uint32_t)hqspi->State & 0x02) != 0U)
    {
        /* Process locked */
        __DAL_LOCK(hqspi);

        /* Update QSPI state */
        hqspi->State = DAL_QSPI_STATE_ABORT;

        /* Disable QSPI interrupts */
        __DAL_QSPI_DISABLE_IT(hqspi, QSPI_IT_TFE \
                                     | QSPI_IT_TFO \
                                     | QSPI_IT_RFU \
                                     | QSPI_IT_RFO \
                                     | QSPI_IT_RFF \
                                     | QSPI_IT_MST);
        if (hqspi->hdmatx != NULL)
        {
            if ((hqspi->Instance->DMACTRL & QSPI_DMACTRL_TDMAEN) != 0U)
            {
                /* Disable the QSPI DMA Tx request */
                CLEAR_BIT(hqspi->Instance->DMACTRL, QSPI_DMACTRL_TDMAEN);

                /* Abort the QSPI DMA Tx channel */
                hqspi->hdmatx->XferAbortCallback = QSPI_DMAAbortCplt;
                if (DAL_DMA_Abort_IT(hqspi->hdmatx) != DAL_OK)
                {
                    /* Update QSPI state */
                    hqspi->State = DAL_QSPI_STATE_READY;

                    /* Abort completed callback */
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
                    hqspi->AbortCpltCallback(hqspi);
#else
                    DAL_QSPI_AbortCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
                }
            }
            else
            {
                hqspi->hdmatx->XferAbortCallback = NULL;
            }
        }

        if (hqspi->hdmarx != NULL)
        {
            if ((hqspi->Instance->DMACTRL & QSPI_DMACTRL_RDMAEN) != 0U)
            {
                /* Disable the QSPI DMA Rx request */
                CLEAR_BIT(hqspi->Instance->DMACTRL, QSPI_DMACTRL_RDMAEN);

                /* Abort the QSPI DMA Rx channel */
                hqspi->hdmarx->XferAbortCallback = QSPI_DMAAbortCplt;
                if (DAL_DMA_Abort_IT(hqspi->hdmarx) != DAL_OK)
                {
                    /* Update QSPI state */
                    hqspi->State = DAL_QSPI_STATE_READY;

                    /* Abort completed callback */
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
                    hqspi->AbortCpltCallback(hqspi);
#else
                    DAL_QSPI_AbortCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
                }
            }
            else
            {
                hqspi->hdmarx->XferAbortCallback = NULL;
            }
        }

        if (__DAL_QSPI_GET_FLAG(hqspi, QSPI_FLAG_BUSY) != RESET)
        {
            /* Disable slave select */
            __DAL_QSPI_DISABLE_SS(hqspi);

            /* Disable the QSPI peripheral */
            __DAL_QSPI_DISABLE(hqspi);

            /* Wait till BUSY flag reset */
            status = DAL_QSPI_WaitFlagStateUntilTimeout(hqspi, QSPI_FLAG_BUSY, RESET, tickstart, hqspi->Timeout);
        }
        else
        {
            /* Update QSPI state */
            hqspi->State = DAL_QSPI_STATE_READY;
        }
    }

    return status;
}

/**
 * @brief  Set QSPI timeout.
 * @param  hqspi QSPI handle
 * @param  Timeout Timeout for the QSPI memory access
 * @retval None
 */
void DAL_QSPI_SetTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Timeout)
{
    hqspi->Timeout = Timeout;
}

/**
 * @brief  Set QSPI TX Fifo threshold.
 * @param  hqspi QSPI handle
 * @param  Threshold threshold for the QSPI Fifo
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_SetTxFifoThreshold(QSPI_HandleTypeDef *hqspi, uint32_t Threshold)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_QSPI_TX_FIFO_THRESHOLD(Threshold));

        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);

        /* Configure FIFO threshold */
        MODIFY_REG(hqspi->Instance->TFTL, QSPI_TFTL_TFT, Threshold);

        /* Enable the QSPI peripheral */
        __DAL_QSPI_ENABLE(hqspi);
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Set QSPI RX Fifo threshold.
 * @param  hqspi QSPI handle
 * @param  Threshold threshold for the QSPI Fifo
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_SetRxFifoThreshold(QSPI_HandleTypeDef *hqspi, uint32_t Threshold)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_QSPI_RX_FIFO_THRESHOLD(Threshold));

        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);

        /* Configure FIFO threshold */
        MODIFY_REG(hqspi->Instance->RFTL, QSPI_RFTL_RFT, Threshold);

        /* Enable the QSPI peripheral */
        __DAL_QSPI_ENABLE(hqspi);
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @berief  Set QSPI Tx Fifo level.
 * @param  hqspi QSPI handle
 * @param  Level level for the QSPI Fifo
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_SetTxFifoLevel(QSPI_HandleTypeDef *hqspi, uint32_t Level)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_QSPI_TX_FIFO_LEVEL(Level));

        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);

        /* Configure FIFO level */
        MODIFY_REG(hqspi->Instance->TFTL, QSPI_TFTL_TFTH, Level);

        /* Enable the QSPI peripheral */
        __DAL_QSPI_ENABLE(hqspi);
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Get QSPI Tx Fifo threshold.
 * @param  hqspi QSPI handle
 * @retval Threshold threshold for the QSPI Fifo
 */
uint32_t DAL_QSPI_GetTxFifoThreshold(QSPI_HandleTypeDef *hqspi)
{
    /* Return the FIFO threshold */
    return READ_BIT(hqspi->Instance->TFTL, QSPI_TFTL_TFT);
}

/**
 * @brief  Get QSPI Rx Fifo threshold.
 * @param  hqspi QSPI handle
 * @retval Threshold threshold for the QSPI Fifo
 */
uint32_t DAL_QSPI_GetRxFifoThreshold(QSPI_HandleTypeDef *hqspi)
{
    /* Return the FIFO threshold */
    return READ_BIT(hqspi->Instance->RFTL, QSPI_RFTL_RFT);
}

/**
 * @brief  Get QSPI Tx Fifo level.
 * @param  hqspi QSPI handle
 * @retval Level level for the QSPI Fifo
 */
uint32_t DAL_QSPI_GetTxFifoLevel(QSPI_HandleTypeDef *hqspi)
{
    /* Return the FIFO level */
    return READ_BIT(hqspi->Instance->TFTL, QSPI_TFTL_TFTH);
}

/**
 * @brief  Set QSPI frame format.
 * @param  hqspi QSPI handle
 * @param  FrameFormat frame format
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_SetFrameFormat(QSPI_HandleTypeDef *hqspi, uint32_t FrameFormat)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_QSPI_FRAME_FORMAT(FrameFormat));

        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);

        /* Configure frame format */
        MODIFY_REG(hqspi->Instance->CTRL1, QSPI_CTRL1_FRF, FrameFormat);

        /* Enable the QSPI peripheral */
        __DAL_QSPI_ENABLE(hqspi);
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Get QSPI frame format.
 * @param  hqspi QSPI handle
 * @retval FrameFormat frame format
 */
uint32_t DAL_QSPI_GetFrameFormat(QSPI_HandleTypeDef *hqspi)
{
    /* Return the frame format */
    return READ_BIT(hqspi->Instance->CTRL1, QSPI_CTRL1_FRF);
}

/**
 * @brief  Set QSPI frame data size.
 * @param  hqspi QSPI handle
 * @param  DataFrameSize frame data size
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_SetDataFrameSize(QSPI_HandleTypeDef *hqspi, uint32_t DataFrameSize)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_QSPI_DATA_FRAME_SIZE(DataFrameSize));

        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);

        /* Configure frame data size */
        MODIFY_REG(hqspi->Instance->CTRL1, QSPI_CTRL1_DFS, DataFrameSize);

        /* Enable the QSPI peripheral */
        __DAL_QSPI_ENABLE(hqspi);
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Get QSPI frame data size.
 * @param  hqspi QSPI handle
 * @retval DataFrameSize frame data size
 */
uint32_t DAL_QSPI_GetDataFrameSize(QSPI_HandleTypeDef *hqspi)
{
    /* Return the frame data size */
    return READ_BIT(hqspi->Instance->CTRL1, QSPI_CTRL1_DFS);
}

/**
 * @brief  Set QSPI transfer mode.
 * @param  hqspi QSPI handle
 * @param  TransferMode transfer mode
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_SetTransferMode(QSPI_HandleTypeDef *hqspi, uint32_t TransferMode)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_QSPI_TRANSFER_MODE(TransferMode));

        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);

        /* Configure transfer mode */
        MODIFY_REG(hqspi->Instance->CTRL1, QSPI_CTRL1_TXMODE, TransferMode);

        /* Enable the QSPI peripheral */
        __DAL_QSPI_ENABLE(hqspi);
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Get QSPI transfer mode.
 * @param  hqspi QSPI handle
 * @retval TransferMode transfer mode
 */
uint32_t DAL_QSPI_GetTransferMode(QSPI_HandleTypeDef *hqspi)
{
    /* Return the transfer mode */
    return READ_BIT(hqspi->Instance->CTRL1, QSPI_CTRL1_TXMODE);
}

/**
 * @brief  Set QSPI number of data frames.
 * @param  hqspi QSPI handle
 * @param  NbData number of data frames
 * @retval DAL status
 */
DAL_StatusTypeDef DAL_QSPI_SetFrameNbData(QSPI_HandleTypeDef *hqspi, uint32_t NbData)
{
    DAL_StatusTypeDef status = DAL_OK;

    /* Process locked */
    __DAL_LOCK(hqspi);

    if (hqspi->State == DAL_QSPI_STATE_READY)
    {
        /* Check the parameters */
        ASSERT_PARAM(IS_QSPI_NB_DATA(NbData));

        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);

        /* Configure number of data frames */
        MODIFY_REG(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF, (NbData - 1) << QSPI_CTRL2_NDF_Pos);

        /* Enable the QSPI peripheral */
        __DAL_QSPI_ENABLE(hqspi);
    }
    else
    {
        status = DAL_BUSY;
    }

    /* Process unlocked */
    __DAL_UNLOCK(hqspi);

    return status;
}

/**
 * @brief  Get QSPI number of data frames.
 * @param  hqspi QSPI handle
 * @retval NbData number of data frames
 */
uint32_t DAL_QSPI_GetFrameNbData(QSPI_HandleTypeDef *hqspi)
{
    /* Return the number of data frames */
    return (READ_BIT(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF) >> QSPI_CTRL2_NDF_Pos) + 1U;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup QSPI_Private_Functions QSPI Private Functions
  * @{
  */

/**
 * @brief  DMA QSPI transmit process complete callback.
 * @param  hdma DMA handle
 * @retval None
 */
static void QSPI_DMATxCplt(DMA_HandleTypeDef *hdma)
{
    QSPI_HandleTypeDef *hqspi = (QSPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* DMA Normal mode */
    if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == 0U)
    {
        hqspi->TxXferCount = 0U;

        if (hqspi->State == DAL_QSPI_STATE_BUSY_TX)
        {
            /* Disable the QSPI DMA Tx request */
            CLEAR_BIT(hqspi->Instance->DMACTRL, QSPI_DMACTRL_TDMAEN);

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            hqspi->TxCpltCallback(hqspi);
#else
            DAL_QSPI_TxCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
        }

        /* Update QSPI state */
        hqspi->State = DAL_QSPI_STATE_READY;
    }
    /* DMA Circular mode */
    else
    {
        if (hqspi->State == DAL_QSPI_STATE_BUSY_TX)
        {
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            hqspi->TxCpltCallback(hqspi);
#else
            DAL_QSPI_TxCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
        }
    }
}

/**
 * @brief  DMA QSPI receive process complete callback.
 * @param  hdma DMA handle
 * @retval None
 */
static void QSPI_DMARxCplt(DMA_HandleTypeDef *hdma)
{
    QSPI_HandleTypeDef *hqspi = (QSPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* DMA Normal mode */
    if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == 0U)
    {
        hqspi->RxXferCount = 0U;

        if (hqspi->State == DAL_QSPI_STATE_BUSY_RX)
        {
            /* Disable the QSPI DMA Rx request */
            CLEAR_BIT(hqspi->Instance->DMACTRL, QSPI_DMACTRL_RDMAEN);

#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            hqspi->RxCpltCallback(hqspi);
#else
            DAL_QSPI_RxCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
        }

        /* Update QSPI state */
        hqspi->State = DAL_QSPI_STATE_READY;
    }
    /* DMA Circular mode */
    else
    {
        if (hqspi->State == DAL_QSPI_STATE_BUSY_RX)
        {
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
            hqspi->RxCpltCallback(hqspi);
#else
            DAL_QSPI_RxCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
        }
    }
}

/**
 * @brief  DMA QSPI communication error callback.
 * @param  hdma DMA handle
 * @retval None
 */
static void QSPI_DMAError(DMA_HandleTypeDef *hdma)
{
    QSPI_HandleTypeDef *hqspi = (QSPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    hqspi->RxXferCount = 0U;
    hqspi->TxXferCount = 0U;
    hqspi->ErrorCode |= DAL_QSPI_ERROR_DMA;

    /* Disable using DMA by clearing TDMAEN and RDMAEN bit in the QSPI DMACTRL register */
    CLEAR_BIT(hqspi->Instance->DMACTRL, (QSPI_DMACTRL_RDMAEN | QSPI_DMACTRL_TDMAEN));

    /* Abort the QSPI */
    (void)DAL_QSPI_Abort_IT(hqspi);
}

/**
 * @brief  DMA QSPI communication abort callback.
 * @param  hdma DMA handle
 * @retval None
 */
static void QSPI_DMAAbortCplt(DMA_HandleTypeDef *hdma)
{
    QSPI_HandleTypeDef *hqspi = (QSPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    hqspi->RxXferCount = 0U;
    hqspi->TxXferCount = 0U;

    if (hqspi->State == DAL_QSPI_STATE_ABORT)
    {
        /* Disable slave select */
        __DAL_QSPI_DISABLE_SS(hqspi);

        /* Disable the QSPI peripheral */
        __DAL_QSPI_DISABLE(hqspi);
    }
    else
    {
        /* Update QSPI state */
        hqspi->State = DAL_QSPI_STATE_READY;

        /* Error callback */
#if (USE_DAL_QSPI_REGISTER_CALLBACKS == 1)
        hqspi->AbortCpltCallback(hqspi);
#else
        DAL_QSPI_AbortCpltCallback(hqspi);
#endif /* USE_DAL_QSPI_REGISTER_CALLBACKS */
    }
}

/**
 * @brief  Wait for a flag state until timeout
 * @param  hqspi: QSPI handle
 * @param  Flag: Flag checked
 * @param  State: Value of the flag expected
 * @param  TickStart: Tick start value
 * @param  Timeout: Timeout duration
 * @retval DAL status
 */
static DAL_StatusTypeDef DAL_QSPI_WaitFlagStateUntilTimeout(QSPI_HandleTypeDef *hqspi, uint32_t Flag, FlagStatus State, uint32_t TickStart, uint32_t Timeout)
{
    /* Wait until flag is in expected state */
    while ((__DAL_QSPI_GET_FLAG(hqspi, Flag)) != State)
    {
        /* Check for the Timeout */
        if (Timeout != DAL_MAX_DELAY)
        {
            if (((DAL_GetTick() - TickStart) > Timeout) || (Timeout == 0U))
            {
                hqspi->State = DAL_QSPI_STATE_ERROR;
                hqspi->ErrorCode |= DAL_QSPI_ERROR_TIMEOUT;

                return DAL_ERROR;
            }
        }
    }

    return DAL_OK;
}

/**
 * @brief  Configure the communication registers.
 * @param  hqspi: QSPI handle
 * @param  cmd: structure that contains the command configuration information
 * @retval None
 */
static void QSPI_Config(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd)
{
    if (cmd->FrameFormat == QSPI_FRAME_FORMAT_STANDARD)
    {
        /* Configure CTRL1 */
        MODIFY_REG(hqspi->Instance->CTRL1, \
                    (QSPI_CTRL1_FRF | QSPI_CTRL1_TXMODE), \
                    (cmd->FrameFormat | cmd->TransferMode));

        if (cmd->TransferMode == QSPI_TRANSFER_MODE_EEPROM_READ)
        {
            /* Configure CTRL2 */
            MODIFY_REG(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF, (cmd->NbData) << QSPI_CTRL2_NDF_Pos);
        }
        else
        {
            /* Configure CTRL2 */
            MODIFY_REG(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF, (cmd->NbData - 1) << QSPI_CTRL2_NDF_Pos);
        }
    }
    else if (cmd->FrameFormat == QSPI_FRAME_FORMAT_DUAL)
    {
        if ((cmd->TransferMode == QSPI_TRANSFER_MODE_TX) || (cmd->TransferMode == QSPI_TRANSFER_MODE_RX))
        {
            /* Configure CTRL1 */
            MODIFY_REG(hqspi->Instance->CTRL1, \
                        (QSPI_CTRL1_FRF | QSPI_CTRL1_TXMODE), \
                        (cmd->FrameFormat | cmd->TransferMode));

            /* Configure CTRL2 */
            if (cmd->NbData != 0)
            {
                MODIFY_REG(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF, (cmd->NbData - 1) << QSPI_CTRL2_NDF_Pos);
            }
            else
            {
                MODIFY_REG(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF, 0);
            }

            if (cmd->TransferMode == QSPI_TRANSFER_MODE_TX)
            {
                /* Configure CTRL3 */
                MODIFY_REG(hqspi->Instance->CTRL3, \
                            (QSPI_CTRL3_IAT | QSPI_CTRL3_ADDRLEN | QSPI_CTRL3_INSLEN), \
                            ((cmd->InstructionMode) | (cmd->AddressSize) | (cmd->InstructionSize)));
            }
            else
            {
                /* Configure CTRL3 */
                MODIFY_REG(hqspi->Instance->CTRL3, \
                            (QSPI_CTRL3_IAT | QSPI_CTRL3_ADDRLEN | QSPI_CTRL3_INSLEN | QSPI_CTRL3_WAITCYC), \
                            ((cmd->InstructionMode) | (cmd->AddressSize) | (cmd->InstructionSize) | (cmd->DummyCycles << QSPI_CTRL3_WAITCYC_Pos)));
            }
        }
    }
    else if (cmd->FrameFormat == QSPI_FRAME_FORMAT_QUAD)
    {
        if ((cmd->TransferMode == QSPI_TRANSFER_MODE_TX) || (cmd->TransferMode == QSPI_TRANSFER_MODE_RX))
        {
            /* Configure CTRL1 */
            MODIFY_REG(hqspi->Instance->CTRL1, \
                        (QSPI_CTRL1_FRF | QSPI_CTRL1_TXMODE), \
                        (cmd->FrameFormat | cmd->TransferMode));

            /* Configure CTRL2 */
            if (cmd->NbData != 0)
            {
                MODIFY_REG(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF, (cmd->NbData - 1) << QSPI_CTRL2_NDF_Pos);
            }
            else
            {
                MODIFY_REG(hqspi->Instance->CTRL2, QSPI_CTRL2_NDF, 0);
            }

            if (cmd->TransferMode == QSPI_TRANSFER_MODE_TX)
            {
                /* Configure CTRL3 */
                MODIFY_REG(hqspi->Instance->CTRL3, \
                            (QSPI_CTRL3_IAT | QSPI_CTRL3_ADDRLEN | QSPI_CTRL3_INSLEN), \
                            ((cmd->InstructionMode) | (cmd->AddressSize) | (cmd->InstructionSize)));
            }
            else
            {
                /* Configure CTRL3 */
                MODIFY_REG(hqspi->Instance->CTRL3, \
                            (QSPI_CTRL3_IAT | QSPI_CTRL3_ADDRLEN | QSPI_CTRL3_INSLEN | QSPI_CTRL3_WAITCYC), \
                            ((cmd->InstructionMode) | (cmd->AddressSize) | (cmd->InstructionSize) | (cmd->DummyCycles << QSPI_CTRL3_WAITCYC_Pos)));
            }
        }
    }
    else
    {
        /* Nothing to do */
    }
}

/**
  * @}
  */

#endif /* DAL_QSPI_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

#endif /* QSPI */
