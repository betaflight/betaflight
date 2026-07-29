/**
  ******************************************************************************
  * @file     um324xx_hal_lpuart.c
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

/** @defgroup LPUART LPUART
  * @brief HAL LPUART module driver
  * @{
  */
#ifdef HAL_LPUART_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

const uint16_t LPUARTMODUTable[8] = {0x952, 0xEFB, 0x6DB, 0x412, 0x6D6, 0x842, 0x842, 0x842};

/** @addtogroup LPUART_Private_Functions  LPUART Private Functions
  * @{
  */

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
void LPUART_InitCallbacksToDefault(LPUART_HandleTypeDef *hlpuart);
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */
static void LPUART_EndTxTransfer(LPUART_HandleTypeDef *hlpuart);
static void LPUART_EndRxTransfer(LPUART_HandleTypeDef *hlpuart);
static HAL_StatusTypeDef LPUART_Transmit_IT(LPUART_HandleTypeDef *hlpuart);
static HAL_StatusTypeDef LPUART_EndTransmit_IT(LPUART_HandleTypeDef *hlpuart);
static HAL_StatusTypeDef LPUART_Receive_IT(LPUART_HandleTypeDef *hlpuart);
static HAL_StatusTypeDef LPUART_WaitOnFlagUntilTimeout(LPUART_HandleTypeDef *hlpuart, uint32_t Flag, FlagStatus Status,
        uint32_t Tickstart, uint32_t Timeout);
static void LPUART_SetConfig(LPUART_HandleTypeDef *hlpuart);

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup LPUART_Exported_Functions LPUART Exported Functions
  * @{
  */

/**
  * @brief  Initializes the LPUART mode according to the specified parameters in
  *         the LPUART_InitTypeDef and create the associated handle.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPUART_Init(LPUART_HandleTypeDef *hlpuart)
{
    /* Check the LPUART handle allocation */
    if (hlpuart == NULL)
    {
        return HAL_ERROR;
    }

    if (hlpuart->gState == HAL_LPUART_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hlpuart->Lock = HAL_UNLOCKED;

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
        LPUART_InitCallbacksToDefault(hlpuart);

        if (hlpuart->MspInitCallback == NULL)
        {
            hlpuart->MspInitCallback = HAL_LPUART_MspInit;
        }

        /* Init the low level hardware */
        hlpuart->MspInitCallback(hlpuart);
#else
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_LPUART_MspInit(hlpuart);
#endif /* (USE_HAL_LPUART_REGISTER_CALLBACKS) */
    }

    hlpuart->gState = HAL_LPUART_STATE_BUSY;

    /* Set the LPUART Communication parameters */
    LPUART_SetConfig(hlpuart);

    /* Initialize the LPUART state */
    hlpuart->ErrorCode = HAL_LPUART_ERROR_NONE;
    hlpuart->gState = HAL_LPUART_STATE_READY;
    hlpuart->RxState = HAL_LPUART_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  DeInitializes the LPUART peripheral.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPUART_DeInit(LPUART_HandleTypeDef *hlpuart)
{
    /* Check the LPUART handle allocation */
    if (hlpuart == NULL)
    {
        return HAL_ERROR;
    }

    hlpuart->gState = HAL_LPUART_STATE_BUSY;

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
    if (hlpuart->MspDeInitCallback == NULL)
    {
        hlpuart->MspDeInitCallback = HAL_LPUART_MspDeInit;
    }
    /* DeInit the low level hardware */
    hlpuart->MspDeInitCallback(hlpuart);
#else
    /* DeInit the low level hardware */
    HAL_LPUART_MspDeInit(hlpuart);
#endif /* (USE_HAL_LPUART_REGISTER_CALLBACKS) */

    hlpuart->ErrorCode = HAL_LPUART_ERROR_NONE;
    hlpuart->gState = HAL_LPUART_STATE_RESET;
    hlpuart->RxState = HAL_LPUART_STATE_RESET;

    /* Process Unlock */
    __HAL_UNLOCK(hlpuart);

    return HAL_OK;
}

/**
  * @brief  LPUART MSP Init.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval None
  */
__weak void HAL_LPUART_MspInit(LPUART_HandleTypeDef *hlpuart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlpuart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_LPUART_MspInit could be implemented in the user file
     */
}

/**
  * @brief  LPUART MSP DeInit.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval None
  */
__weak void HAL_LPUART_MspDeInit(LPUART_HandleTypeDef *hlpuart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlpuart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_LPUART_MspDeInit could be implemented in the user file
     */
}

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User LPUART Callback
  *         To be used instead of the weak predefined callback
  * @param  hlpuart LPUART handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_LPUART_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref HAL_LPUART_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref HAL_LPUART_ERROR_CB_ID Error Callback ID
  *           @arg @ref HAL_LPUART_MATCH_CB_ID Error Callback ID
  *           @arg @ref HAL_LPUART_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref HAL_LPUART_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPUART_RegisterCallback(LPUART_HandleTypeDef *hlpuart, HAL_LPUART_CallbackIDTypeDef CallbackID,
        pLPUART_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hlpuart->ErrorCode |= HAL_LPUART_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(hlpuart);

    if (hlpuart->gState == HAL_LPUART_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_LPUART_TX_COMPLETE_CB_ID :
            hlpuart->TxCpltCallback = pCallback;
            break;

        case HAL_LPUART_RX_COMPLETE_CB_ID :
            hlpuart->RxCpltCallback = pCallback;
            break;

        case HAL_LPUART_ERROR_CB_ID :
            hlpuart->ErrorCallback = pCallback;
            break;

        case HAL_LPUART_MATCH_CB_ID :
            hlpuart->MatchCallback = pCallback;
            break;

        case HAL_LPUART_MSPINIT_CB_ID :
            hlpuart->MspInitCallback = pCallback;
            break;

        case HAL_LPUART_MSPDEINIT_CB_ID :
            hlpuart->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hlpuart->ErrorCode |= HAL_LPUART_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (hlpuart->gState == HAL_LPUART_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_LPUART_MSPINIT_CB_ID :
            hlpuart->MspInitCallback = pCallback;
            break;

        case HAL_LPUART_MSPDEINIT_CB_ID :
            hlpuart->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            hlpuart->ErrorCode |= HAL_LPUART_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hlpuart->ErrorCode |= HAL_LPUART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hlpuart);

    return status;
}

/**
  * @brief  Unregister an LPUART Callback
  *         LPUART callaback is redirected to the weak predefined callback
  * @param  hlpuart LPUART handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *           @arg @ref HAL_LPUART_TX_COMPLETE_CB_ID Tx Complete Callback ID
  *           @arg @ref HAL_LPUART_RX_COMPLETE_CB_ID Rx Complete Callback ID
  *           @arg @ref HAL_LPUART_ERROR_CB_ID Error Callback ID
  *           @arg @ref HAL_LPUART_MATCH_CB_ID Error Callback ID
  *           @arg @ref HAL_LPUART_MSPINIT_CB_ID MspInit Callback ID
  *           @arg @ref HAL_LPUART_MSPDEINIT_CB_ID MspDeInit Callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPUART_UnRegisterCallback(LPUART_HandleTypeDef *hlpuart, HAL_LPUART_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hlpuart);

    if (HAL_LPUART_STATE_READY == hlpuart->gState)
    {
        switch (CallbackID)
        {

        case HAL_LPUART_TX_COMPLETE_CB_ID :
            hlpuart->TxCpltCallback = HAL_LPUART_TxCpltCallback;                       /* Legacy weak TxCpltCallback            */
            break;

        case HAL_LPUART_RX_COMPLETE_CB_ID :
            hlpuart->RxCpltCallback = HAL_LPUART_RxCpltCallback;                       /* Legacy weak RxCpltCallback            */
            break;

        case HAL_LPUART_ERROR_CB_ID :
            hlpuart->ErrorCallback = HAL_LPUART_ErrorCallback;                         /* Legacy weak ErrorCallback             */
            break;

        case HAL_LPUART_MATCH_CB_ID :
            hlpuart->MatchCallback = HAL_LPUART_ErrorCallback;                         /* Legacy weak MatchCallback             */
            break;

        case HAL_LPUART_MSPINIT_CB_ID :
            hlpuart->MspInitCallback = HAL_LPUART_MspInit;                             /* Legacy weak MspInitCallback           */
            break;

        case HAL_LPUART_MSPDEINIT_CB_ID :
            hlpuart->MspDeInitCallback = HAL_LPUART_MspDeInit;                         /* Legacy weak MspDeInitCallback         */
            break;

        default :
            /* Update the error code */
            hlpuart->ErrorCode |= HAL_LPUART_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_LPUART_STATE_RESET == hlpuart->gState)
    {
        switch (CallbackID)
        {
        case HAL_LPUART_MSPINIT_CB_ID :
            hlpuart->MspInitCallback = HAL_LPUART_MspInit;
            break;

        case HAL_LPUART_MSPDEINIT_CB_ID :
            hlpuart->MspDeInitCallback = HAL_LPUART_MspDeInit;
            break;

        default :
            /* Update the error code */
            hlpuart->ErrorCode |= HAL_LPUART_ERROR_INVALID_CALLBACK;

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        hlpuart->ErrorCode |= HAL_LPUART_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hlpuart);

    return status;
}
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */

/** @defgroup LPUART_Exported_Functions_Group2 IO operation functions
  *  @brief LPUART Transmit and Receive functions
  * @{
  */

/**
  * @brief  Sends an amount of data in blocking mode.
  * @param  hlpuart Pointer to a LPUART_HandleTypeDef structure that contains
  *                 the configuration information for the specified LPUART module.
  * @param  pData Pointer to data buffer.
  * @param  Size  Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPUART_Transmit(LPUART_HandleTypeDef *hlpuart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = 0U;

    /* Check that a Tx process is not already ongoing */
    if (hlpuart->gState == HAL_LPUART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(hlpuart);

        hlpuart->ErrorCode = HAL_LPUART_ERROR_NONE;
        hlpuart->gState = HAL_LPUART_STATE_BUSY_TX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        hlpuart->TxXferSize = Size;
        hlpuart->TxXferCount = Size;

        /* Process Unlocked */
        __HAL_UNLOCK(hlpuart);

        while (hlpuart->TxXferCount > 0U)
        {
            if (LPUART_WaitOnFlagUntilTimeout(hlpuart, LPUART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }

            hlpuart->Instance->TXD = (uint8_t)(*pData & 0xFFU);
            pData++;
            hlpuart->TxXferCount--;
        }

        if (LPUART_WaitOnFlagUntilTimeout(hlpuart, LPUART_FLAG_TC, RESET, tickstart, Timeout) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

        /* At end of Tx process, End LPUART transfer */
        LPUART_EndTxTransfer(hlpuart);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in blocking mode.
  * @param  hlpuart Pointer to a LPUART_HandleTypeDef structure that contains
  *                 the configuration information for the specified LPUART module.
  * @param  pData Pointer to data buffer.
  * @param  Size  Amount of data to be received.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPUART_Receive(LPUART_HandleTypeDef *hlpuart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = 0U;

    /* Check that a Rx process is not already ongoing */
    if (hlpuart->RxState == HAL_LPUART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(hlpuart);

        hlpuart->ErrorCode = HAL_LPUART_ERROR_NONE;
        hlpuart->RxState = HAL_LPUART_STATE_BUSY_RX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        hlpuart->RxXferSize = Size;
        hlpuart->RxXferCount = Size;

        /* Process Unlocked */
        __HAL_UNLOCK(hlpuart);

        /* Check the remain data to be received */
        while (hlpuart->RxXferCount > 0U)
        {
            if (LPUART_WaitOnFlagUntilTimeout(hlpuart, LPUART_FLAG_RXF, RESET, tickstart, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }

            *pData = (uint8_t)(hlpuart->Instance->RXD & (uint8_t)0x00FF);
            pData++;

            hlpuart->RxXferCount--;
        }

        /* At end of Rx process, restore hlpuart->RxState to Ready */
        hlpuart->RxState = HAL_LPUART_STATE_READY;

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  hlpuart Pointer to a LPUART_HandleTypeDef structure that contains
  *                 the configuration information for the specified LPUART module.
  * @param  pData Pointer to data buffer.
  * @param  Size  Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPUART_Transmit_IT(LPUART_HandleTypeDef *hlpuart, const uint8_t *pData, uint16_t Size)
{
    /* Check that a Tx process is not already ongoing */
    if (hlpuart->gState == HAL_LPUART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(hlpuart);

        hlpuart->pTxBuffPtr = pData;
        hlpuart->TxXferSize = Size;
        hlpuart->TxXferCount = Size;

        hlpuart->ErrorCode = HAL_LPUART_ERROR_NONE;
        hlpuart->gState = HAL_LPUART_STATE_BUSY_TX;

        /* Process Unlocked */
        __HAL_UNLOCK(hlpuart);

        /* Enable the LPUART Transmit empty Interrupt */
        SET_BIT(hlpuart->Instance->CON, LPUART_CON_TXIE);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in non blocking mode.
  * @param  hlpuart Pointer to a LPUART_HandleTypeDef structure that contains
  *                 the configuration information for the specified LPUART module.
  * @param  pData Pointer to data buffer.
  * @param  Size  Amount of data to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_LPUART_Receive_IT(LPUART_HandleTypeDef *hlpuart, uint8_t *pData, uint16_t Size)
{
    /* Check that a Rx process is not already ongoing */
    if (hlpuart->RxState == HAL_LPUART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(hlpuart);

        return (LPUART_Start_Receive_IT(hlpuart, pData, Size));
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  This function handles LPUART interrupt request.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval None
  */
void HAL_LPUART_IRQHandler(LPUART_HandleTypeDef *hlpuart)
{
    uint32_t staflags   = READ_REG(hlpuart->Instance->STA);
    uint32_t conits     = READ_REG(hlpuart->Instance->CON);
    uint32_t itflags    = READ_REG(hlpuart->Instance->IF);
    uint32_t errorflags = 0x00U;

    /* If no error occurs */
    errorflags = (staflags & (uint32_t)(LPUART_STA_RXOV | LPUART_STA_FERR | LPUART_STA_PERR));
    if (errorflags == RESET)
    {
        /* LPUART in mode Receiver -------------------------------------------------*/
        if (((itflags & LPUART_IF_RX_IF) != RESET) && ((conits & LPUART_CON_RXIE) != RESET))
        {
            if(((conits & LPUART_CON_RXEV_0) != RESET))
            {
                LPUART_Receive_IT(hlpuart);
                return;
            }
            else if(((conits & LPUART_CON_RXEV_1) != RESET) && ((staflags & LPUART_STA_MATCH) != RESET))
            {
                /* Clear MATCH bit */
                WRITE_REG(hlpuart->Instance->STA, LPUART_STA_MATCH);

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
                /*Call registered match callback*/
                hlpuart->MatchCallback(hlpuart);
#else
                /*Call legacy weak error callback*/
                HAL_LPUART_MatchCallback(hlpuart);
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */
            }
            if(((staflags & LPUART_STA_START) != RESET))
            {
                /* Clear START bit */
                WRITE_REG(hlpuart->Instance->STA, LPUART_STA_START);
            }
        }
    }

    /* If some errors occur */
    if ((errorflags != RESET) && (((conits & LPUART_CON_ERRIE) != RESET)))
    {
        /* LPUART parity error interrupt occurred ----------------------------------*/
        if ((staflags & LPUART_STA_PERR) != RESET)
        {
            hlpuart->ErrorCode |= HAL_LPUART_ERROR_PERR;
        }

        /* LPUART frame error interrupt occurred -----------------------------------*/
        if ((staflags & LPUART_STA_FERR) != RESET)
        {
            hlpuart->ErrorCode |= HAL_LPUART_ERROR_FERR;
        }

        /* LPUART Over-Run interrupt occurred --------------------------------------*/
        if ((staflags & LPUART_STA_RXOV) != RESET)
        {
            hlpuart->ErrorCode |= HAL_LPUART_ERROR_RXOV;
        }

        /* Call LPUART Error Call back function if need be --------------------------*/
        if (hlpuart->ErrorCode != HAL_LPUART_ERROR_NONE)
        {
            /* LPUART in mode Receiver -----------------------------------------------*/
            if ((staflags & LPUART_STA_RXOV) != RESET)
            {
                LPUART_Receive_IT(hlpuart);
            }

            /* Parity error occurred */
            if ((staflags & LPUART_STA_PERR) != RESET)
            {
                LPUART_EndRxTransfer(hlpuart);

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
                /*Call registered error callback*/
                hlpuart->ErrorCallback(hlpuart);
#else
                /*Call legacy weak error callback*/
                HAL_LPUART_ErrorCallback(hlpuart);
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */
            }

            /* Framing error occurred */
            if ((staflags & LPUART_STA_FERR) != RESET)
            {
                LPUART_EndRxTransfer(hlpuart);

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
                /*Call registered error callback*/
                hlpuart->ErrorCallback(hlpuart);
#else
                /*Call legacy weak error callback*/
                HAL_LPUART_ErrorCallback(hlpuart);
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */
            }
        }
        return;
    } /* End if some error occurs */

    /* LPUART in mode Transmitter ------------------------------------------------*/
    if (((staflags & LPUART_STA_TXE) != RESET) && ((conits & LPUART_CON_TXIE) != RESET))
    {
        LPUART_Transmit_IT(hlpuart);
        return;
    }
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval None
  */
__weak void HAL_LPUART_TxCpltCallback(LPUART_HandleTypeDef *hlpuart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlpuart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_LPUART_TxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval None
  */
__weak void HAL_LPUART_RxCpltCallback(LPUART_HandleTypeDef *hlpuart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlpuart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_LPUART_RxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  LPUART error callbacks.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval None
  */
__weak void HAL_LPUART_ErrorCallback(LPUART_HandleTypeDef *hlpuart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlpuart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_LPUART_ErrorCallback could be implemented in the user file
     */
}

/**
  * @brief  LPUART match callbacks.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval None
  */
__weak void HAL_LPUART_MatchCallback(LPUART_HandleTypeDef *hlpuart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hlpuart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_LPUART_ErrorCallback could be implemented in the user file
     */
}

/**
  * @}
  */

/**
  * @brief  End ongoing Tx transfer on LPUART peripheral (following Transmit completion).
  * @param  hlpuart LPUART handle.
  * @retval None
  */
static void LPUART_EndTxTransfer(LPUART_HandleTypeDef *hlpuart)
{
    /* At end of Tx process, restore hlpuart->gState to Ready */
    hlpuart->gState = HAL_LPUART_STATE_READY;
}

/**
  * @brief  End ongoing Rx transfer on LPUART peripheral (following Reception completion).
  * @param  hlpuart LPUART handle.
  * @retval None
  */
static void LPUART_EndRxTransfer(LPUART_HandleTypeDef *hlpuart)
{
    /* Disable RXIE, TXIE, TCIE and ERRIE (Frame error,  overrun error) interrupts for the interrupt process */
    ATOMIC_CLEAR_BIT(hlpuart->Instance->CON, LPUART_CON_RXIE | LPUART_CON_TXIE | LPUART_CON_TCIE | LPUART_CON_ERRIE);

    /* At end of Rx process, restore hlpuart->RxState to Ready */
    hlpuart->RxState = HAL_LPUART_STATE_READY;
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef LPUART_Transmit_IT(LPUART_HandleTypeDef *hlpuart)
{
    /* Check that a Tx process is ongoing */
    if (hlpuart->gState == HAL_LPUART_STATE_BUSY_TX)
    {
        /* Clear TX_IF Flag */
        WRITE_REG(hlpuart->Instance->IF, LPUART_IF_TX_IF);

        hlpuart->Instance->TXD = (uint8_t)(*hlpuart->pTxBuffPtr++ & 0xFF);

        if (--hlpuart->TxXferCount == 0U)
        {
            /* LPUART in mode Transmitter end */
            LPUART_EndTransmit_IT(hlpuart);
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
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef LPUART_EndTransmit_IT(LPUART_HandleTypeDef *hlpuart)
{
    /* Clear TX_IF and RC_IF Flag */
    WRITE_REG(hlpuart->Instance->IF, LPUART_IF_TX_IF | LPUART_IF_TC_IF);

    /* Disable the LPUART Transmit Empty Interrupt */
    CLEAR_BIT(hlpuart->Instance->CON, LPUART_CON_TXIE);

    /* Tx process is ended, restore hlpuart->gState to Ready */
    hlpuart->gState = HAL_LPUART_STATE_READY;

#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
    /*Call registered Tx complete callback*/
    hlpuart->TxCpltCallback(hlpuart);
#else
    /*Call legacy weak Tx complete callback*/
    HAL_LPUART_TxCpltCallback(hlpuart);
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */

    return HAL_OK;
}

/**
  * @brief  Receives an amount of data in non blocking mode
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef LPUART_Receive_IT(LPUART_HandleTypeDef *hlpuart)
{
    /* Check that a Rx process is ongoing */
    if (hlpuart->RxState == HAL_LPUART_STATE_BUSY_RX)
    {
        *hlpuart->pRxBuffPtr = (uint8_t)(hlpuart->Instance->RXD & 0xFF);
        hlpuart->pRxBuffPtr++;
        /* Clear RX_IF Flag */
        WRITE_REG(hlpuart->Instance->IF, LPUART_IF_RX_IF);

        if (--hlpuart->RxXferCount == 0U)
        {
            /* Disable the LPUART Data Error Interrupt */
            __HAL_LPUART_DISABLE_IT(hlpuart, LPUART_IT_ERR);

            /* Disable the LPUART Receiver Interrupt */
            __HAL_LPUART_DISABLE_IT(hlpuart, LPUART_IT_RX);

            /* Rx process is completed, restore hlpuart->RxState to Ready */
            hlpuart->RxState = HAL_LPUART_STATE_READY;

            /* Standard reception API called */
#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
            /*Call registered Rx complete callback*/
            hlpuart->RxCpltCallback(hlpuart);
#else
            /*Call legacy weak Rx complete callback*/
            HAL_LPUART_RxCpltCallback(hlpuart);
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */

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
  * @brief  Configures the LPUART peripheral.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval None
  */
static void LPUART_SetConfig(LPUART_HandleTypeDef *hlpuart)
{
    uint32_t tmpreg;

    /*-------------------------- LPUART Baud Rate Configuration ------------------*/
    hlpuart->Instance->BAUD = hlpuart->Init.BaudRate;
    hlpuart->Instance->MODU = LPUARTMODUTable[hlpuart->Init.BaudRate];

    /*-------------------------- LPUART CON Configuration ------------------------*/
    /* Configure the LPUART Word Length, Parity and Stop Bits:
     Set DL bits according to hlpuart->Init.WordLength value
     Set PTYP and PAREN bits according to hlpuart->Init.Parity value
     Set RXEV bits according to hlpuart->Init.ReceiveITType
     Set SL bits according to hlpuart->Init.StopBits value */

    tmpreg = (uint32_t)hlpuart->Init.WordLength | hlpuart->Init.Parity | hlpuart->Init.StopBits | hlpuart->Init.ReceiveITType;
    MODIFY_REG(hlpuart->Instance->CON,
               (uint32_t)LPUART_CON_DL | LPUART_CON_SL | LPUART_CON_PTYP | LPUART_CON_PAREN | LPUART_CON_RXEV, tmpreg);

    /*-------------------------- LPUART EN Configuration -------------------------*/
    /* Configure the LPUART Word Length, Parity and Stop Bits:
     Set TXEN and RXEN bits according to hlpuart->Init.Mode value */
    hlpuart->Instance->EN |= hlpuart->Init.Mode;
    while(((hlpuart->Instance->EN) & (hlpuart->Init.Mode)) != hlpuart->Init.Mode);

}

/** @defgroup LPUART_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   LPUART State and Errors functions
  *
@verbatim
  ==============================================================================
                 ##### Peripheral State and Errors functions #####
  ==============================================================================
 [..]
   This subsection provides a set of functions allowing to return the State of
   LPUART communication process, return Peripheral Errors occurred during communication
   process
   (+) HAL_LPUART_GetState() API can be helpful to check in run-time the state of the LPUART peripheral.
   (+) HAL_LPUART_GetError() check in run-time errors that could be occurred during communication.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the LPUART state.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @retval HAL state
  */
HAL_LPUART_StateTypeDef HAL_LPUART_GetState(LPUART_HandleTypeDef *hlpuart)
{
    uint32_t temp1 = 0x00U, temp2 = 0x00U;
    temp1 = hlpuart->gState;
    temp2 = hlpuart->RxState;

    return (HAL_LPUART_StateTypeDef)(temp1 | temp2);
}

/**
  * @brief  Return the LPUART error code
  * @param  hlpuart Pointer to a LPUART_HandleTypeDef structure that contains
  *                 the configuration information for the specified LPUART.
  * @retval LPUART Error Code
  */
uint32_t HAL_LPUART_GetError(LPUART_HandleTypeDef *hlpuart)
{
    return hlpuart->ErrorCode;
}

/**
  * @}
  */

/**
  * @brief  Initialize the callbacks to their default values.
  * @param  hlpuart LPUART handle.
  * @retval none
  */
#if (USE_HAL_LPUART_REGISTER_CALLBACKS == 1)
void LPUART_InitCallbacksToDefault(LPUART_HandleTypeDef *hlpuart)
{
    /* Init the LPUART Callback settings */
    hlpuart->TxCpltCallback            = HAL_LPUART_TxCpltCallback;            /* Legacy weak TxCpltCallback            */
    hlpuart->RxCpltCallback            = HAL_LPUART_RxCpltCallback;            /* Legacy weak RxCpltCallback            */
    hlpuart->ErrorCallback             = HAL_LPUART_ErrorCallback;             /* Legacy weak ErrorCallback             */
}
#endif /* USE_HAL_LPUART_REGISTER_CALLBACKS */

/**
  * @brief  This function handles LPUART Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  hlpuart  Pointer to a LPUART_HandleTypeDef structure that contains
  *                  the configuration information for the specified LPUART module.
  * @param  Flag specifies the LPUART flag to check.
  * @param  Status The actual Flag status (SET or RESET).
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef LPUART_WaitOnFlagUntilTimeout(LPUART_HandleTypeDef *hlpuart, uint32_t Flag, FlagStatus Status,
        uint32_t Tickstart, uint32_t Timeout)
{
    /* Wait until flag is set */
    while ((__HAL_LPUART_GET_FLAG(hlpuart, Flag) ? SET : RESET) == Status)
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
            {
                /* Disable RXIE, TXIE, TCIE and ERRIE (Frame error,  overrun error) interrupts for the interrupt process */
                ATOMIC_CLEAR_BIT(hlpuart->Instance->CON, LPUART_CON_RXIE | LPUART_CON_TXIE | LPUART_CON_TCIE | LPUART_CON_ERRIE);
                hlpuart->gState  = HAL_LPUART_STATE_READY;
                hlpuart->RxState = HAL_LPUART_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hlpuart);

                return HAL_TIMEOUT;
            }
        }
    }
    return HAL_OK;
}

/**
  * @brief  Start Receive operation in interrupt mode.
  * @note   This function could be called by all HAL LPUART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         LPUART Handle is assumed as Locked.
  * @param  hlpuart LPUART handle.
  * @param  pData Pointer to data buffer.
  * @param  Size  Amount of data to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef LPUART_Start_Receive_IT(LPUART_HandleTypeDef *hlpuart, uint8_t *pData, uint16_t Size)
{
    hlpuart->pRxBuffPtr = pData;
    hlpuart->RxXferSize = Size;
    hlpuart->RxXferCount = Size;

    hlpuart->ErrorCode = HAL_LPUART_ERROR_NONE;
    hlpuart->RxState = HAL_LPUART_STATE_BUSY_RX;

    /* Process Unlocked */
    __HAL_UNLOCK(hlpuart);

    /* Enable the LPUART Receiver Interrupt */
    __HAL_LPUART_ENABLE_IT(hlpuart, LPUART_IT_RX);

    /* Enable the LPUART Error Interrupt */
    __HAL_LPUART_ENABLE_IT(hlpuart, LPUART_IT_ERR);

    return HAL_OK;
}

/**
  * @}
  */

#endif /* HAL_LPUART_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
