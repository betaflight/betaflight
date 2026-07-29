/**
  ******************************************************************************
  * @file     um324xx_hal_uart_ex.c
  * @author   MCU Team
  * @version  V1.00
  * @date     2023-03-21
  * @brief
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023. Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */

/** @defgroup UART_EX UART_EX
  * @brief HAL UART_EX module driver
  * @{
  */
#ifdef HAL_UART_EX_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @addtogroup UART_EX_Private_Functions  UART_EX Private Functions
  * @{
  */

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
void UART_EX_InitCallbacksToDefault(UART_EX_HandleTypeDef *huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */
static void UART_EX_EndTxTransfer(UART_EX_HandleTypeDef *huart);
static void UART_EX_EndRxTransfer(UART_EX_HandleTypeDef *huart);
static void UART_EX_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void UART_EX_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void UART_EX_DMAError(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef UART_EX_Transmit_IT(UART_EX_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_EX_EndTransmit_IT(UART_EX_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_EX_Receive_IT(UART_EX_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_EX_WaitOnFlagUntilTimeout(UART_EX_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
        uint32_t Tickstart, uint32_t Timeout);
static void UART_EX_SetConfig(UART_EX_HandleTypeDef *huart);

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup UART_EX_Exported_Functions UART Exported Functions
  * @{
  */

/**
  * @brief  Initializes the UART_EX mode according to the specified parameters in
  *         the UART_EX_InitTypeDef and create the associated handle.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Init(UART_EX_HandleTypeDef *huart)
{
    /* Check the UART_EX handle allocation */
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    if (huart->gState == HAL_UART_EX_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        huart->Lock = HAL_UNLOCKED;

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
        UART_EX_InitCallbacksToDefault(huart);

        if (huart->MspInitCallback == NULL)
        {
            huart->MspInitCallback = HAL_UART_EX_MspInit;
        }

        /* Init the low level hardware */
        huart->MspInitCallback(huart);
#else
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_UART_EX_MspInit(huart);
#endif /* (USE_HAL_UART_EX_REGISTER_CALLBACKS) */
    }

    huart->gState = HAL_UART_EX_STATE_BUSY;

    /* Set the UART Communication parameters */
    UART_EX_SetConfig(huart);

    /* Initialize the UART_EX state */
    huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
    huart->gState = HAL_UART_EX_STATE_READY;
    huart->RxState = HAL_UART_EX_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  DeInitializes the UART_EX peripheral.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_DeInit(UART_EX_HandleTypeDef *huart)
{
    /* Check the UART handle allocation */
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    huart->gState = HAL_UART_EX_STATE_BUSY;

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
    if (huart->MspDeInitCallback == NULL)
    {
        huart->MspDeInitCallback = HAL_UART_EX_MspDeInit;
    }
    /* DeInit the low level hardware */
    huart->MspDeInitCallback(huart);
#else
    /* DeInit the low level hardware */
    HAL_UART_EX_MspDeInit(huart);
#endif /* (USE_HAL_UART_EX_REGISTER_CALLBACKS) */

    huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
    huart->gState = HAL_UART_EX_STATE_RESET;
    huart->RxState = HAL_UART_EX_STATE_RESET;

    /* Process Unlock */
    __HAL_UNLOCK(huart);

    return HAL_OK;
}

/**
  * @brief  UART_EX MSP Init.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_EX_MspInit(UART_EX_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_EX_MspInit could be implemented in the user file
     */
}

/**
  * @brief  UART_EX MSP DeInit.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_EX_MspDeInit(UART_EX_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_EX_MspDeInit could be implemented in the user file
     */
}

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User UART_EX Callback
  *         To be used instead of the weak predefined callback
  * @param  huart UART_EX_HandleTypeDef handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_UART_EX_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref HAL_UART_EX_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref HAL_UART_EX_ERROR_CB_ID Error Callback ID
  *           @arg @ref HAL_UART_EX_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref HAL_UART_EX_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_RegisterCallback(UART_EX_HandleTypeDef *huart, HAL_UART_EX_CallbackIDTypeDef CallbackID,
        pUART_EX_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        huart->ErrorCode |= HAL_UART_EX_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(huart);

    if (huart->gState == HAL_UART_EX_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_UART_EX_TX_COMPLETE_CB_ID :
            huart->TxCpltCallback = pCallback;
            break;

        case HAL_UART_EX_RX_COMPLETE_CB_ID :
            huart->RxCpltCallback = pCallback;
            break;

        case HAL_UART_EX_ERROR_CB_ID :
            huart->ErrorCallback = pCallback;
            break;

        case HAL_UART_EX_MSPINIT_CB_ID :
            huart->MspInitCallback = pCallback;
            break;

        case HAL_UART_EX_MSPDEINIT_CB_ID :
            huart->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            huart->ErrorCode |= HAL_UART_EX_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (huart->gState == HAL_UART_EX_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_UART_EX_MSPINIT_CB_ID :
            huart->MspInitCallback = pCallback;
            break;

        case HAL_UART_EX_MSPDEINIT_CB_ID :
            huart->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            huart->ErrorCode |= HAL_UART_EX_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        huart->ErrorCode |= HAL_UART_EX_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(huart);

    return status;
}

/**
  * @brief  Unregister an UART_EX Callback
  *         UART callaback is redirected to the weak predefined callback
  * @param  huart UART_EX_HandleTypeDef handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_UART_EX_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref HAL_UART_EX_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref HAL_UART_EX_ERROR_CB_ID Error Callback ID
  *           @arg @ref HAL_UART_EX_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref HAL_UART_EX_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_UnRegisterCallback(UART_EX_HandleTypeDef *huart, HAL_UART_EX_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(huart);

    if (HAL_UART_EX_STATE_READY == huart->gState)
    {
        switch (CallbackID)
        {

        case HAL_UART_EX_TX_COMPLETE_CB_ID :
            huart->TxCpltCallback = HAL_UART_EX_TxCpltCallback;                       /* Legacy weak TxCpltCallback            */
            break;

        case HAL_UART_EX_RX_COMPLETE_CB_ID :
            huart->RxCpltCallback = HAL_UART_EX_RxCpltCallback;                       /* Legacy weak RxCpltCallback            */
            break;

        case HAL_UART_EX_ERROR_CB_ID :
            huart->ErrorCallback = HAL_UART_EX_ErrorCallback;                         /* Legacy weak ErrorCallback             */
            break;

        case HAL_UART_EX_MSPINIT_CB_ID :
            huart->MspInitCallback = HAL_UART_EX_MspInit;                             /* Legacy weak MspInitCallback           */
            break;

        case HAL_UART_EX_MSPDEINIT_CB_ID :
            huart->MspDeInitCallback = HAL_UART_EX_MspDeInit;                         /* Legacy weak MspDeInitCallback         */
            break;

        default :
            /* Update the error code */
            huart->ErrorCode |= HAL_UART_EX_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_UART_EX_STATE_RESET == huart->gState)
    {
        switch (CallbackID)
        {
        case HAL_UART_EX_MSPINIT_CB_ID :
            huart->MspInitCallback = HAL_UART_EX_MspInit;
            break;

        case HAL_UART_EX_MSPDEINIT_CB_ID :
            huart->MspDeInitCallback = HAL_UART_EX_MspDeInit;
            break;

        default :
            /* Update the error code */
            huart->ErrorCode |= HAL_UART_EX_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        huart->ErrorCode |= HAL_UART_EX_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(huart);

    return status;
}
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

/** @defgroup UART_EX_Exported_Functions_Group2 IO operation functions
  *  @brief UART Transmit and Receive functions
  * @{
  */

/**
  * @brief  Sends an amount of data in blocking mode.
  * @note   When UART_EX parity is not enabled (PEN = 0), and Word Length is configured to 9 bits (DLS_E = 1),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  huart Pointer to a UART_EX_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Transmit(UART_EX_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    const uint8_t  *pdata8bits;
    const uint16_t *pdata16bits;
    uint32_t tickstart = 0U;

    /* Check that a Tx process is not already ongoing */
    if (huart->gState == HAL_UART_EX_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        //__HAL_LOCK(huart);

        huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
        huart->gState = HAL_UART_EX_STATE_BUSY_TX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        huart->TxXferSize = Size;
        huart->TxXferCount = Size;

        /* In case of 9bits, pData needs to be handled as a uint16_t pointer */
        if (huart->Init.WordLength == UART_EX_WORDLENGTH_9B)
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
        __HAL_UNLOCK(huart);

        while (huart->TxXferCount > 0U)
        {
            if (UART_EX_WaitOnFlagUntilTimeout(huart, UART_EX_FLAG_TEMT, RESET, tickstart, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }
            if (pdata8bits == NULL)
            {
                huart->Instance->RBR = (uint16_t)(*pdata16bits & 0x01FFU);
                pdata16bits++;
            }
            else
            {
                huart->Instance->RBR = (uint8_t)(*pdata8bits & 0xFFU);
                pdata8bits++;
            }
            huart->TxXferCount--;
        }

        if (UART_EX_WaitOnFlagUntilTimeout(huart, UART_EX_FLAG_BUSY, RESET, tickstart, Timeout) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

        /* At end of Tx process, End UART_EX transfer */
        UART_EX_EndTxTransfer(huart);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in blocking mode.
  * @note   When UART_EX parity is not enabled (PEN = 0), and Word Length is configured to 9 bits (DLS_E = 1),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  huart Pointer to a UART_EX_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Receive(UART_EX_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t  *pdata8bits;
    uint16_t *pdata16bits;
    uint32_t tickstart = 0U;

    /* Check that a Rx process is not already ongoing */
    if (huart->RxState == HAL_UART_EX_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        // __HAL_LOCK(huart);

        huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
        huart->RxState = HAL_UART_EX_STATE_BUSY_RX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        huart->RxXferSize = Size;
        huart->RxXferCount = Size;

        /* In case of 9bits, pRxData needs to be handled as a uint16_t pointer */
        if (huart->Init.WordLength == UART_EX_WORDLENGTH_9B)
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
        __HAL_UNLOCK(huart);

        /* Check the remain data to be received */
        while (huart->RxXferCount > 0U)
        {
            if (UART_EX_WaitOnFlagUntilTimeout(huart, UART_EX_FLAG_DR, RESET, tickstart, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }
            if (pdata8bits == NULL)
            {
                *pdata16bits = (uint16_t)(huart->Instance->RBR & 0x01FF);
                pdata16bits++;
            }
            else
            {
                *pdata8bits = (uint8_t)(huart->Instance->RBR & (uint8_t)0x00FF);
                pdata8bits++;
            }
            huart->RxXferCount--;
        }

        /* At end of Rx process, restore huart->RxState to Ready */
        huart->RxState = HAL_UART_EX_STATE_READY;

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @note   When UART_EX parity is not enabled (PEN = 0), and Word Length is configured to 9 bits (DLS_E = 1),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  huart Pointer to a UART_EX_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Transmit_IT(UART_EX_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
    /* Check that a Tx process is not already ongoing */
    if (huart->gState == HAL_UART_EX_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        //  __HAL_LOCK(huart);

        huart->pTxBuffPtr = pData;
        huart->TxXferSize = Size;
        huart->TxXferCount = Size;

        huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
        huart->gState = HAL_UART_EX_STATE_BUSY_TX;

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        /* Enable the UART_EX Transmit data register empty Interrupt */
        __HAL_UART_EX_ENABLE_IT(huart, UART_EX_IT_ETBEI);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in non blocking mode.
  * @note   When UART_EX parity is not enabled (PEN = 0), and Word Length is configured to 9 bits (DLS_E = 1),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  huart Pointer to a UART_EX_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Receive_IT(UART_EX_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    /* Check that a Rx process is not already ongoing */
    if (huart->RxState == HAL_UART_EX_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        // __HAL_LOCK(huart);

        return (UART_EX_Start_Receive_IT(huart, pData, Size));
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sends an amount of data in DMA mode.
  * @note   When UART_EX parity is not enabled (PEN = 0), and Word Length is configured to 9 bits (DLS_E = 1),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval HAL status
  */
extern DMA_HandleTypeDef hdma_uart1_tx;
HAL_StatusTypeDef HAL_UART_EX_Transmit_DMA(UART_EX_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
    const uint32_t *tmp;

    /* Check that a Tx process is not already ongoing */
    if (huart->gState == HAL_UART_EX_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        //  __HAL_LOCK(huart);

        huart->pTxBuffPtr = pData;
        huart->TxXferSize = Size;
        huart->TxXferCount = Size;

        huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
        huart->gState = HAL_UART_EX_STATE_BUSY_TX;

        /* Set the UART_EX DMA Block transfer complete callback */
        huart->hdmatx->XferBlockCallback = UART_EX_DMATransmitCplt;

        /* Set the DMA error callback */
        huart->hdmatx->XferErrorCallback = UART_EX_DMAError;

        /* Enable the UART_EX transmit DMA stream */
        tmp = (const uint32_t *)&pData;
        HAL_DMA_Start_IT(huart->hdmatx, *(const uint32_t *)tmp, (uint32_t)&huart->Instance->THR, Size);

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        /* Enable the DMA transfer for transmit request by setting the DMAE bit
           in the UART_EX MCR register */
        ATOMIC_SET_BIT(huart->Instance->MCR, UART_EX_MCR_DMAE);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in DMA mode.
  * @note   When UART_EX parity is not enabled (PEN = 0), and Word Length is configured to 9 bits (DLS_E = 1),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  huart Pointer to a UART_EX_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Receive_DMA(UART_EX_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    /* Check that a Rx process is not already ongoing */
    if (huart->RxState == HAL_UART_EX_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        // __HAL_LOCK(huart);

        return (UART_EX_Start_Receive_DMA(huart, pData, Size));
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Abort ongoing transfers (blocking mode).
  * @param  huart UART_EX_HandleTypeDef handle.
  * @note   This procedure could be used for aborting any ongoing transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART_EX Interrupts (Tx and Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_Abort(UART_EX_HandleTypeDef *huart)
{
    /* Disable ERBFI, ETBEI and ELSI interrupts */
    ATOMIC_CLEAR_BIT(huart->Instance->IER, (UART_EX_IER_ERBFI | UART_EX_IER_ETBEI | UART_EX_IER_ELSI));

    /* Disable the UART_EX DMA request if enabled */
    if (HAL_IS_BIT_SET(huart->Instance->MCR, UART_EX_MCR_DMAE))
    {
        ATOMIC_CLEAR_BIT(huart->Instance->MCR, UART_EX_MCR_DMAE);

        /* Abort the UART_EX DMA Tx stream: use blocking DMA Abort API (no callback) */
        if (huart->hdmatx != NULL)
        {
            if (HAL_DMA_Abort(huart->hdmatx) != HAL_OK)
            {
                if (HAL_DMA_GetError(huart->hdmatx) == HAL_DMA_ERROR_TIMEOUT)
                {
                    /* Set error code to DMA */
                    huart->ErrorCode = HAL_UART_EX_ERROR_DMA;

                    return HAL_TIMEOUT;
                }
            }
        }

        /* Abort the UART_EX DMA Rx stream: use blocking DMA Abort API (no callback) */
        if (huart->hdmarx != NULL)
        {
            if (HAL_DMA_Abort(huart->hdmarx) != HAL_OK)
            {
                if (HAL_DMA_GetError(huart->hdmarx) == HAL_DMA_ERROR_TIMEOUT)
                {
                    /* Set error code to DMA */
                    huart->ErrorCode = HAL_UART_EX_ERROR_DMA;

                    return HAL_TIMEOUT;
                }
            }
        }

    }

    /* Reset Tx and Rx transfer counters */
    huart->TxXferCount = 0x00U;
    huart->RxXferCount = 0x00U;

    /* Reset ErrorCode */
    huart->ErrorCode = HAL_UART_EX_ERROR_NONE;

    /* Restore huart->RxState and huart->gState to Ready */
    huart->RxState = HAL_UART_EX_STATE_READY;
    huart->gState = HAL_UART_EX_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  Abort ongoing Transmit transfer (blocking mode).
  * @param  huart UART_EX_HandleTypeDef handle.
  * @note   This procedure could be used for aborting any ongoing Tx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART_EX Interrupts (Tx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_AbortTransmit(UART_EX_HandleTypeDef *huart)
{
    /* Disable ETBEI interrupts */
    ATOMIC_CLEAR_BIT(huart->Instance->IER, UART_EX_IER_ETBEI);

    /* Disable the UART_EX DMA Tx request if enabled */
    if (HAL_IS_BIT_SET(huart->Instance->MCR, UART_EX_MCR_DMAE))
    {
        ATOMIC_CLEAR_BIT(huart->Instance->MCR, UART_EX_MCR_DMAE);

        /* Abort the UART_EX DMA Tx stream : use blocking DMA Abort API (no callback) */
        if (huart->hdmatx != NULL)
        {
            if (HAL_DMA_Abort(huart->hdmatx) != HAL_OK)
            {
                if (HAL_DMA_GetError(huart->hdmatx) == HAL_DMA_ERROR_TIMEOUT)
                {
                    /* Set error code to DMA */
                    huart->ErrorCode = HAL_UART_EX_ERROR_DMA;

                    return HAL_TIMEOUT;
                }
            }
        }
    }

    /* Reset Tx transfer counter */
    huart->TxXferCount = 0x00U;

    /* Restore huart->gState to Ready */
    huart->gState = HAL_UART_EX_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  Abort ongoing Receive transfer (blocking mode).
  * @param  huart UART_EX_HandleTypeDef handle.
  * @note   This procedure could be used for aborting any ongoing Rx transfer started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable UART_EX Interrupts (Rx)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_EX_AbortReceive(UART_EX_HandleTypeDef *huart)
{
    /* Disable ERBFI, ELSI interrupts */
    ATOMIC_CLEAR_BIT(huart->Instance->IER, (UART_EX_IER_ERBFI | UART_EX_IER_ELSI));

    /* Disable the UART_EX DMA Rx request if enabled */
    if (HAL_IS_BIT_SET(huart->Instance->MCR, UART_EX_MCR_DMAE))
    {
        ATOMIC_CLEAR_BIT(huart->Instance->MCR, UART_EX_MCR_DMAE);

        /* Abort the UART_EX DMA Rx stream : use blocking DMA Abort API (no callback) */
        if (huart->hdmarx != NULL)
        {
            if (HAL_DMA_Abort(huart->hdmarx) != HAL_OK)
            {
                if (HAL_DMA_GetError(huart->hdmarx) == HAL_DMA_ERROR_TIMEOUT)
                {
                    /* Set error code to DMA */
                    huart->ErrorCode = HAL_UART_EX_ERROR_DMA;

                    return HAL_TIMEOUT;
                }
            }
        }
    }

    /* Reset Rx transfer counter */
    huart->RxXferCount = 0x00U;

    /* Restore huart->RxState to Ready */
    huart->RxState = HAL_UART_EX_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  This function handles UART_EX interrupt request.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_EX_IRQHandler(UART_EX_HandleTypeDef *huart)
{
	uint32_t usrflags   = READ_REG(huart->Instance->USR);
    uint32_t lsrflags   = READ_REG(huart->Instance->LSR);
    uint32_t ierits     = READ_REG(huart->Instance->IER);
    uint32_t errorflags = 0x00U;
    uint32_t dmarequest = 0x00U;

    /* If no error occurs */
    errorflags = (lsrflags & (uint32_t)(UART_EX_LSR_OE | UART_EX_LSR_PE | UART_EX_LSR_FE));
    if (errorflags == RESET)
    {
        /* UART_EX in mode Receiver -------------------------------------------------*/
		if (((usrflags & UART_EX_USR_RFNE) != RESET) && ((ierits & UART_EX_IER_ERBFI) != RESET))
        {
            UART_EX_Receive_IT(huart);
            return;
        }
    }

    /* If some errors occur */
    if ((errorflags != RESET) && (((ierits & ((uint32_t) UART_EX_IER_ELSI | UART_EX_IER_ERBFI )) != RESET)))
    {
        /* UART_EX parity error interrupt occurred ----------------------------------*/
        if (((lsrflags & UART_EX_LSR_PE) != RESET) && ((ierits & UART_EX_IER_ELSI) != RESET))
        {
            huart->ErrorCode |= HAL_UART_EX_ERROR_PE;
        }

        /* UART_EX frame error interrupt occurred -----------------------------------*/
        if (((lsrflags & UART_EX_LSR_FE) != RESET) && ((ierits & UART_EX_IER_ELSI) != RESET))
        {
            huart->ErrorCode |= HAL_UART_EX_ERROR_FE;
        }

        /* UART_EX Over-Run interrupt occurred --------------------------------------*/
        if (((lsrflags & UART_EX_LSR_OE) != RESET) && ((ierits & UART_EX_IER_ELSI) != RESET))
        {
            huart->ErrorCode |= HAL_UART_EX_ERROR_ORE;
        }

        /* Call UART_EX Error Call back function if need be --------------------------*/
        if (huart->ErrorCode != HAL_UART_EX_ERROR_NONE)
        {
            /* UART_EX in mode Receiver -----------------------------------------------*/
            if (((lsrflags & UART_EX_LSR_DR) != RESET) && ((ierits & UART_EX_IER_ERBFI) != RESET))
            {
                UART_EX_Receive_IT(huart);
            }

            /* If Overrun error occurs, or if any error occurs in DMA mode reception,
            consider error as blocking */
            dmarequest = HAL_IS_BIT_SET(huart->Instance->MCR, UART_EX_MCR_DMAE);
            if (((huart->ErrorCode & HAL_UART_EX_ERROR_ORE) != RESET) || dmarequest)
            {
                /* Blocking error : transfer is aborted
                	Set the UART_EX state ready to be able to start again the process,
                	Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
                UART_EX_EndRxTransfer(huart);

                /* Disable the UART_EX DMA Rx request if enabled */
                if (HAL_IS_BIT_SET(huart->Instance->MCR, UART_EX_MCR_DMAE))
                {
                    ATOMIC_CLEAR_BIT(huart->Instance->MCR, UART_EX_MCR_DMAE);

                    /* Call user error callback */
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
                    /*Call registered error callback*/
                    huart->ErrorCallback(huart);
#else
                    /*Call legacy weak error callback*/
                    HAL_UART_EX_ErrorCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */
                }
            }
            else
            {
                /* Call user error callback */
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
                /*Call registered error callback*/
                huart->ErrorCallback(huart);
#else
                /*Call legacy weak error callback*/
                HAL_UART_EX_ErrorCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */
            }
        }
        else
        {
            /* Non Blocking error : transfer could go on.
            	Error is notified to user through user error callback */
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
            /*Call registered error callback*/
            huart->ErrorCallback(huart);
#else
            /*Call legacy weak error callback*/
            HAL_UART_EX_ErrorCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

            huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
        }

        /* Parity error occurred */
        if (((lsrflags & UART_EX_LSR_PE) != RESET) && ((ierits & UART_EX_IER_ELSI) != RESET))
        {
            UART_EX_EndRxTransfer(huart);

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
            /*Call registered error callback*/
            huart->ErrorCallback(huart);
#else
            /*Call legacy weak error callback*/
            HAL_UART_EX_ErrorCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */
        }

        /* Framing error occurred */
        if (((lsrflags & UART_EX_LSR_FE) != RESET) && ((ierits & UART_EX_IER_ELSI) != RESET))
        {
            UART_EX_EndRxTransfer(huart);

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
            /*Call registered error callback*/
            huart->ErrorCallback(huart);
#else
            /*Call legacy weak error callback*/
            HAL_UART_EX_ErrorCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */
        }

        return;
    } /* End if some error occurs */

    /* UART_EX in mode Transmitter ------------------------------------------------*/
	if (((usrflags & UART_EX_USR_TFE) != RESET) && ((ierits & UART_EX_IER_ETBEI) != RESET))
    {
        if((huart->Instance->MCR & UART_EX_MCR_DMAE) == UART_EX_MCR_DMAE)
        {
            //  CLEAR_BIT(huart->Instance->MCR, UART_EX_MCR_DMAE);
            UART_EX_EndTransmit_IT(huart);
            return;
        }
        else
        {
            UART_EX_Transmit_IT(huart);
            return;
        }
    }
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_EX_TxCpltCallback(UART_EX_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_EX_TxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_EX_RxCpltCallback(UART_EX_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_EX_RxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  UART_EX error callbacks.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_EX_ErrorCallback(UART_EX_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_EX_ErrorCallback could be implemented in the user file
     */
}

/**
  * @}
  */

/**
  * @brief  End ongoing Tx transfer on UART_EX peripheral (following Transmit completion).
  * @param  huart UART_EX_HandleTypeDef handle.
  * @retval None
  */
static void UART_EX_EndTxTransfer(UART_EX_HandleTypeDef *huart)
{
    /* Disable the UART_EX Transmit Data Register Empty Interrupt */
    __HAL_UART_EX_DISABLE_IT(huart, UART_EX_IT_ETBEI);
    /* At end of Tx process, restore huart->gState to Ready */
    huart->gState = HAL_UART_EX_STATE_READY;
}

/**
  * @brief  End ongoing Rx transfer on UART_EX peripheral (following Reception completion).
  * @param  huart UART_EX_HandleTypeDef handle.
  * @retval None
  */
static void UART_EX_EndRxTransfer(UART_EX_HandleTypeDef *huart)
{
    /* Disable ELSI, ERBFI() interrupts */
    ATOMIC_CLEAR_BIT(huart->Instance->IER, (UART_EX_IER_ELSI | UART_EX_IER_ERBFI));

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_EX_STATE_READY;
}

/**
  * @brief  DMA UART_EX transmit process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_EX_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
    UART_EX_HandleTypeDef *huart = (UART_EX_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    /* DMA Normal mode*/

    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == 0U)
    {
        huart->TxXferCount = 0x00U;

        /* Enable the UART_EX Transmit Complete Interrupt */
        ATOMIC_SET_BIT(huart->Instance->IER, UART_EX_IER_ETBEI);

    }
    /* DMA Circular mode */
    else
    {
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
        /*Call registered Tx complete callback*/
        huart->TxCpltCallback(huart);
#else
        /*Call legacy weak Tx complete callback*/
        HAL_UART_EX_TxCpltCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */
    }
}

/**
  * @brief  DMA UART_EX receive process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_EX_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
    UART_EX_HandleTypeDef *huart = (UART_EX_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    /* DMA Normal mode*/

    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == 0U)
    {
        huart->RxXferCount = 0U;

        /* Disable ERBFI, ELSI interrupts */
        ATOMIC_CLEAR_BIT(huart->Instance->IER, UART_EX_IT_ELSI);
        ATOMIC_CLEAR_BIT(huart->Instance->IER, UART_EX_IT_ERBFI);

        /* Disable the DMA transfer for transmit request by setting the DMAE bit
           in the UART_EX MCR register */
        ATOMIC_CLEAR_BIT(huart->Instance->MCR, UART_EX_MCR_DMAE);

        /* At end of Rx process, restore huart->RxState to Ready */
        huart->RxState = HAL_UART_EX_STATE_READY;
    }

    /* In other cases : use Rx Complete callback */
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
    /*Call registered Rx complete callback*/
    huart->RxCpltCallback(huart);
#else
    /*Call legacy weak Rx complete callback*/
    HAL_UART_EX_RxCpltCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

}

/**
  * @brief  DMA UART_EX communication error callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void UART_EX_DMAError(DMA_HandleTypeDef *hdma)
{
    uint32_t dmarequest = 0x00U;
    UART_EX_HandleTypeDef *huart = (UART_EX_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Stop UART_EX DMA Tx request if ongoing */
    dmarequest = HAL_IS_BIT_SET(huart->Instance->MCR, UART_EX_MCR_DMAE);
    if ((huart->gState == HAL_UART_EX_STATE_BUSY_TX) && dmarequest)
    {
        huart->TxXferCount = 0x00U;
        UART_EX_EndTxTransfer(huart);
    }

    /* Stop UART_EX DMA Rx request if ongoing */
    dmarequest = HAL_IS_BIT_SET(huart->Instance->MCR, UART_EX_MCR_DMAE);
    if ((huart->RxState == HAL_UART_EX_STATE_BUSY_RX) && dmarequest)
    {
        huart->RxXferCount = 0x00U;
        UART_EX_EndRxTransfer(huart);
    }

    huart->ErrorCode |= HAL_UART_EX_ERROR_DMA;
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
    /*Call registered error callback*/
    huart->ErrorCallback(huart);
#else
    /*Call legacy weak error callback*/
    HAL_UART_EX_ErrorCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_EX_Transmit_IT(UART_EX_HandleTypeDef *huart)
{
    const uint16_t *tmp;

    /* Check that a Tx process is ongoing */
    if (huart->gState == HAL_UART_EX_STATE_BUSY_TX)
    {
        if (huart->Init.WordLength == UART_EX_WORDLENGTH_9B)
        {
            tmp = (const uint16_t *) huart->pTxBuffPtr;
            huart->Instance->RBR = (uint16_t)(*tmp & (uint16_t)0x01FF);
            huart->pTxBuffPtr += 2U;
        }
        else
        {
            huart->Instance->RBR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);
        }

        if (--huart->TxXferCount == 0U)
        {
            /* UART_EX in mode Transmitter end */
            UART_EX_EndTransmit_IT(huart);
        }
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Wraps up transmission in non blocking mode.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_EX_EndTransmit_IT(UART_EX_HandleTypeDef *huart)
{
    /* Disable the UART_EX Transmit Data Register Empty Interrupt */
    __HAL_UART_EX_DISABLE_IT(huart, UART_EX_IT_ETBEI);

    /* Tx process is ended, restore huart->gState to Ready */
    huart->gState = HAL_UART_EX_STATE_READY;

#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
    /*Call registered Tx complete callback*/
    huart->TxCpltCallback(huart);
#else
    /*Call legacy weak Tx complete callback*/
    HAL_UART_EX_TxCpltCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

    return HAL_OK;
}

/**
  * @brief  Receives an amount of data in non blocking mode
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_EX_Receive_IT(UART_EX_HandleTypeDef *huart)
{
    uint8_t  *pdata8bits;
    uint16_t *pdata16bits;

    /* Check that a Rx process is ongoing */
    if (huart->RxState == HAL_UART_EX_STATE_BUSY_RX)
    {
        if (huart->Init.WordLength == UART_EX_WORDLENGTH_9B)
        {
            pdata8bits  = NULL;
            pdata16bits = (uint16_t *) huart->pRxBuffPtr;
            *pdata16bits = (uint16_t)(huart->Instance->RBR & (uint16_t)0x01FF);
            huart->pRxBuffPtr += 2U;
        }
        else
        {
            pdata8bits = (uint8_t *) huart->pRxBuffPtr;
            pdata16bits  = NULL;
            *pdata8bits = (uint8_t)(huart->Instance->RBR & (uint8_t)0x00FF);
            huart->pRxBuffPtr += 1U;
        }

        if (--huart->RxXferCount == 0U)
        {
            /* Disable the UART_EX Data Register not empty Interrupt */
            __HAL_UART_EX_DISABLE_IT(huart, UART_EX_IT_ERBFI);

            /* Disable the UART_EX Receiver Line Status Interrupt */
            __HAL_UART_EX_DISABLE_IT(huart, UART_EX_IT_ELSI);

            /* Rx process is completed, restore huart->RxState to Ready */
            huart->RxState = HAL_UART_EX_STATE_READY;

            /* Standard reception API called */
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
            /*Call registered Rx complete callback*/
            huart->RxCpltCallback(huart);
#else
            /*Call legacy weak Rx complete callback*/
            HAL_UART_EX_RxCpltCallback(huart);
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

            return HAL_OK;
        }
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Configures the UART_EX peripheral.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
static void UART_EX_SetConfig(UART_EX_HandleTypeDef *huart)
{
    uint32_t tmpreg;
    uint32_t pclk;

    /*-------------------------- UART_EX MCR Configuration ------------------------*/
    /* Configure the UART_EX RTS: Set RTS bits
     according to huart->Init.HwFlowCtl value */
    MODIFY_REG(huart->Instance->MCR, UART_EX_MCR_RTS, huart->Init.HwFlowCtl);
#if defined(UM324xF)
    if (huart->Instance == UART2)
    {
        pclk = HAL_RCM_GetPCLK0Freq();
    }
    else if(huart->Instance == UART1)
    {
        pclk = HAL_RCM_GetPCLK2Freq();
    }
    else
    {
        pclk = HAL_RCM_GetPCLK1Freq();
    }
#endif


#if defined(UM32x42x) || defined(UM32x41x)
    pclk = HAL_RCM_GetPCLK1Freq();//XG
#endif

#if defined(UM324xH)
    pclk = HAL_RCM_GetPCLK1Freq();//XH
#endif

    /*-------------------------- UART_EX Baud Rate Configuration ------------------*/
    MODIFY_REG(huart->Instance->DLF, UART_EX_DLF_DLF, UART_EX_DLF_DIV(pclk, huart->Init.BaudRate));

    huart->Instance->LCR |= UART_EX_LCR_DLAB;
    MODIFY_REG(huart->Instance->DLH, UART_EX_DLH_DLH, UART_EX_DLH_DIV(pclk, huart->Init.BaudRate));
    MODIFY_REG(huart->Instance->DLL, UART_EX_DLL_DLL, UART_EX_DLL_DIV(pclk, huart->Init.BaudRate));
    huart->Instance->LCR &= ~UART_EX_LCR_DLAB;

    /*-------------------------- UART_EX LCR Configuration ------------------------*/
    /* Configure the UART_EX Word Length, Parity and Stop Bits:
     Set DLS bits according to huart->Init.WordLength value
       Set PEN and EPS bits according to huart->Init.Parity value
     Set STOP bits according to huart->Init.StopBits value */

    if(((uint32_t)huart->Init.WordLength) == UART_EX_WORDLENGTH_9B)
    {
        tmpreg = (uint32_t)huart->Init.Parity | huart->Init.StopBits;
        MODIFY_REG(huart->Instance->LCRE, (uint32_t)UART_EX_LCRE_DLS_E | UART_EX_LCRE_TRANSMIT_MODE, UART_EX_WORDLENGTH_9B);
        MODIFY_REG(huart->Instance->LCR, (uint32_t)UART_EX_LCR_PEN | UART_EX_LCR_EPS | UART_EX_LCR_STOP, tmpreg);
    }
    else
    {
        tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.StopBits;
        MODIFY_REG(huart->Instance->LCR,
                   (uint32_t)UART_EX_LCR_DLS | UART_EX_LCR_PEN | UART_EX_LCR_EPS | UART_EX_LCR_STOP, tmpreg);
    }


    /*-------------------------- UART_EX LCR Configuration ------------------------*/
    /* Set FIFOE bits enable UART_EX FIFO */
    huart->Instance->FCR |= UART_EX_FCR_FIFOE;
}

/** @defgroup UART_EX_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   UART State and Errors functions
  *
@verbatim
  ==============================================================================
                 ##### Peripheral State and Errors functions #####
  ==============================================================================
 [..]
   This subsection provides a set of functions allowing to return the State of
   UART_EX communication process, return Peripheral Errors occurred during communication
   process
   (+) HAL_UART_EX_GetState() API can be helpful to check in run-time the state of the UART peripheral.
   (+) HAL_UART_EX_GetError() check in run-time errors that could be occurred during communication.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the UART_EX state.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL state
  */
HAL_UART_EX_StateTypeDef HAL_UART_EX_GetState(UART_EX_HandleTypeDef *huart)
{
    uint32_t temp1 = 0x00U, temp2 = 0x00U;
    temp1 = huart->gState;
    temp2 = huart->RxState;

    return (HAL_UART_EX_StateTypeDef)(temp1 | temp2);
}

/**
  * @brief  Return the UART_EX error code
  * @param  huart Pointer to a UART_EX_HandleTypeDef structure that contains
  *               the configuration information for the specified UART.
  * @retval UART Error Code
  */
uint32_t HAL_UART_EX_GetError(UART_EX_HandleTypeDef *huart)
{
    return huart->ErrorCode;
}

/**
  * @}
  */

/**
  * @brief  Initialize the callbacks to their default values.
  * @param  huart UART_EX_HandleTypeDef handle.
  * @retval none
  */
#if (USE_HAL_UART_EX_REGISTER_CALLBACKS == 1)
void UART_EX_InitCallbacksToDefault(UART_EX_HandleTypeDef *huart)
{
    /* Init the UART_EX Callback settings */
    huart->TxCpltCallback            = HAL_UART_EX_TxCpltCallback;            /* Legacy weak TxCpltCallback            */
    huart->RxCpltCallback            = HAL_UART_EX_RxCpltCallback;            /* Legacy weak RxCpltCallback            */
    huart->ErrorCallback             = HAL_UART_EX_ErrorCallback;             /* Legacy weak ErrorCallback             */
}
#endif /* USE_HAL_UART_EX_REGISTER_CALLBACKS */

/**
  * @brief  This function handles UART Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  huart  Pointer to a UART_EX_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  Flag specifies the UART flag to check.
  * @param  Status The actual Flag status (SET or RESET).
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_EX_WaitOnFlagUntilTimeout(UART_EX_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
        uint32_t Tickstart, uint32_t Timeout)
{
    /* Wait until flag is set */
    while ((__HAL_UART_EX_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
            {
                /* Disable ERBFI, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
                ATOMIC_CLEAR_BIT(huart->Instance->IER, UART_EX_IER_ERBFI | UART_EX_IER_ETBEI);
                huart->gState  = HAL_UART_EX_STATE_READY;
                huart->RxState = HAL_UART_EX_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(huart);

                return HAL_TIMEOUT;
            }
        }
    }
    return HAL_OK;
}

/**
  * @brief  Start Receive operation in interrupt mode.
  * @note   This function could be called by all HAL UART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @param  huart UART_EX_HandleTypeDef handle.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_EX_Start_Receive_IT(UART_EX_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->RxXferCount = Size;

    huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
    huart->RxState = HAL_UART_EX_STATE_BUSY_RX;

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Enable the UART_EX Receiver Line Status Interrupt */
    __HAL_UART_EX_ENABLE_IT(huart, UART_EX_IT_ELSI);

    /* Enable the UART_EX Data Register not empty Interrupt */
    __HAL_UART_EX_ENABLE_IT(huart, UART_EX_IT_ERBFI);

    return HAL_OK;
}

/**
  * @brief  Start Receive operation in DMA mode.
  * @note   This function could be called by all HAL UART API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @param  huart UART_EX_HandleTypeDef handle.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */

HAL_StatusTypeDef UART_EX_Start_Receive_DMA(UART_EX_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    uint32_t *tmp;

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;

    huart->ErrorCode = HAL_UART_EX_ERROR_NONE;
    huart->RxState = HAL_UART_EX_STATE_BUSY_RX;

    /* Set the UART_EX DMA transfer complete callback */
    huart->hdmarx->XferBlockCallback = UART_EX_DMAReceiveCplt;

    /* Set the DMA error callback */
    huart->hdmarx->XferErrorCallback = UART_EX_DMAError;

    /* Enable the DMA stream */
    tmp = (uint32_t *)&pData;
    HAL_DMA_Start_IT(huart->hdmarx, (uint32_t)&huart->Instance->RBR, *(uint32_t *)tmp,Size);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Enable the UART_EX LINE Interrupt */
    ATOMIC_SET_BIT(huart->Instance->IER, UART_EX_IER_ELSI);

    /* Enable the DMA transfer for the receiver request by setting the DMAE bit
    in the UART MCR register */
    ATOMIC_SET_BIT(huart->Instance->MCR, UART_EX_MCR_DMAE);

    return HAL_OK;
}

/**
  * @}
  */

#endif /* HAL_UART_EX_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/

