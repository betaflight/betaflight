/**
  ******************************************************************************
  * @file     um324xx_hal_usart.c 
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-05-04  
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

/** @defgroup USART USART
  * @brief HAL USART module driver
  * @{
  */
#ifdef HAL_USART_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @addtogroup USART_Private_Functions  USART Private Functions
  * @{
  */

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
void USART_InitCallbacksToDefault(USART_HandleTypeDef *husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
static void USART_EndTxTransfer(USART_HandleTypeDef *hsuart);
static void USART_EndRxTransfer(USART_HandleTypeDef *hsuart);
static void USART_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void USART_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void USART_DMAError(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef USART_Transmit_IT(USART_HandleTypeDef *hsuart);
static HAL_StatusTypeDef USART_EndTransmit_IT(USART_HandleTypeDef *hsuart);
static HAL_StatusTypeDef USART_Receive_IT(USART_HandleTypeDef *hsuart);
static HAL_StatusTypeDef USART_WaitOnFlagUntilTimeout(USART_HandleTypeDef *hsuart, uint32_t Flag, FlagStatus Status,
        uint32_t Tickstart, uint32_t Timeout);
static void USART_SetConfig(USART_HandleTypeDef *hsuart);
static void USART_SPI_SetConfig(USART_HandleTypeDef *husart);
static void USART_LIN_SetConfig(USART_HandleTypeDef *hsuart);

/**
  * @}
  */

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initializes the USART mode according to the specified parameters in
  *         the USART_InitTypeDef and create the associated handle.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_Init(USART_HandleTypeDef *husart)
{
    /* Check the USART handle allocation */
    if (husart == NULL)
    {
        return HAL_ERROR;
    }

    if (husart->gState == HAL_USART_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        husart->Lock = HAL_UNLOCKED;

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
        USART_InitCallbacksToDefault(husart);

        if (husart->MspInitCallback == NULL)
        {
            husart->MspInitCallback = HAL_USART_MspInit;
        }

        /* Init the low level hardware */
        husart->MspInitCallback(husart);
#else
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_USART_MspInit(husart);
#endif /* (USE_HAL_USART_REGISTER_CALLBACKS) */
    }

    husart->gState = HAL_USART_STATE_BUSY;

    /* Set the USART Communication parameters */
    USART_SetConfig(husart);

    /* Initialize the USART state */
    husart->ErrorCode = HAL_USART_ERROR_NONE;
    husart->gState = HAL_USART_STATE_READY;
    husart->RxState = HAL_USART_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  Initializes the USART SPI mode according to the specified parameters in
  *         the USART_SPI_InitTypeDef and create the associated handle.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_SPI_Init(USART_HandleTypeDef *husart)
{
    /* Check the USART handle allocation */
    if (husart == NULL)
    {
        return HAL_ERROR;
    }

    if (husart->gState == HAL_USART_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        husart->Lock = HAL_UNLOCKED;

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
        USART_InitCallbacksToDefault(husart);

        if (husart->MspInitCallback == NULL)
        {
            husart->MspInitCallback = HAL_USART_MspInit;
        }

        /* Init the low level hardware */
        husart->MspInitCallback(husart);
#else
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_USART_MspInit(husart);
#endif /* (USE_HAL_USART_REGISTER_CALLBACKS) */
    }

    husart->gState = HAL_USART_STATE_BUSY;

    /* Set the USART SPI Communication parameters */
    USART_SPI_SetConfig(husart);

    /* Initialize the USART state */
    husart->ErrorCode = HAL_USART_ERROR_NONE;
    husart->gState = HAL_USART_STATE_READY;
    husart->RxState = HAL_USART_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  Initializes the LIN mode according to the specified
  *         parameters in the USART_InitTypeDef and create the associated handle.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_LIN_Init(USART_HandleTypeDef *husart)
{
	/* Check the USART handle allocation */
    if (husart == NULL)
    {
        return HAL_ERROR;
    }

    if (husart->gState == HAL_USART_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        husart->Lock = HAL_UNLOCKED;

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
        USART_InitCallbacksToDefault(husart);

        if (husart->MspInitCallback == NULL)
        {
            husart->MspInitCallback = HAL_USART_MspInit;
        }

        /* Init the low level hardware */
        husart->MspInitCallback(husart);
#else
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_USART_MspInit(husart);
#endif /* (USE_HAL_USART_REGISTER_CALLBACKS) */
    }

    husart->gState = HAL_USART_STATE_BUSY;

    /* Set the USART LIN Communication parameters */
    USART_LIN_SetConfig(husart);

    /* Initialize the USART state */
    husart->ErrorCode = HAL_USART_ERROR_NONE;
    husart->gState = HAL_USART_STATE_READY;
    husart->RxState = HAL_USART_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  DeInitializes the USART peripheral.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_DeInit(USART_HandleTypeDef *husart)
{
    /* Check the USART handle allocation */
    if (husart == NULL)
    {
        return HAL_ERROR;
    }

    husart->gState = HAL_USART_STATE_BUSY;

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    if (husart->MspDeInitCallback == NULL)
    {
        husart->MspDeInitCallback = HAL_USART_MspDeInit;
    }
    /* DeInit the low level hardware */
    husart->MspDeInitCallback(husart);
#else
    /* DeInit the low level hardware */
    HAL_USART_MspDeInit(husart);
#endif /* (USE_HAL_USART_REGISTER_CALLBACKS) */

    husart->ErrorCode = HAL_USART_ERROR_NONE;
    husart->gState = HAL_USART_STATE_RESET;
    husart->RxState = HAL_USART_STATE_RESET;

    /* Process Unlock */
    __HAL_UNLOCK(husart);

    return HAL_OK;
}

/**
  * @brief  USART MSP Init.
  * @param  huart  Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void HAL_USART_MspInit(USART_HandleTypeDef *husart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(husart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_USART_MspInit could be implemented in the user file
     */
}

/**
  * @brief  USART MSP DeInit.
  * @param  huart  Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void HAL_USART_MspDeInit(USART_HandleTypeDef *husart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(husart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_USART_MspDeInit could be implemented in the user file
     */
}

/**
  * @brief  Sends an amount of data in blocking mode.
  * @note   When USART Word Length is configured to 9 bits (MODE9 = 1),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    const uint8_t  *pdata8bits;
    const uint16_t *pdata16bits;
    uint32_t tickstart = 0U;

    /* Check that a Tx process is not already ongoing */
    if (husart->gState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        husart->ErrorCode = HAL_USART_ERROR_NONE;
        husart->gState = HAL_USART_STATE_BUSY_TX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        husart->TxXferSize = Size;
        husart->TxXferCount = Size;

        /* In case of 9bits, pData needs to be handled as a uint16_t pointer */
        if (husart->Init.WordLength == USART_WORDLENGTH_9B)
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
        __HAL_UNLOCK(husart);

        while (husart->TxXferCount > 0U)
        {
            if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXEMPTY, RESET, tickstart, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }
            if (pdata8bits == NULL)
            {
                husart->Instance->THR = (uint16_t)(*pdata16bits & 0x01FFU);
                pdata16bits++;
            }
            else
            {
                husart->Instance->THR = (uint8_t)(*pdata8bits & 0xFFU);
                pdata8bits++;
            }
            husart->TxXferCount--;
        }

        if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXRDY, RESET, tickstart, Timeout) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

        /* At end of Tx process, End USART transfer */
        USART_EndTxTransfer(husart);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in blocking mode.
  * @note   When USART Word Length is configured to 9 bits (MODE9 = 1),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint8_t  *pdata8bits;
    uint16_t *pdata16bits;
    uint32_t tickstart = 0U;

    /* Check that a Rx process is not already ongoing */
    if (husart->RxState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        husart->ErrorCode = HAL_USART_ERROR_NONE;
        husart->RxState = HAL_USART_STATE_BUSY_RX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        husart->RxXferSize = Size;
        husart->RxXferCount = Size;

        /* In case of 9bits, pRxData needs to be handled as a uint16_t pointer */
        if (husart->Init.WordLength == USART_WORDLENGTH_9B)
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
        __HAL_UNLOCK(husart);

        /* Check the remain data to be received */
        while (husart->RxXferCount > 0U)
        {
            if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_RXRDY, RESET, tickstart, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }
            if (pdata8bits == NULL)
            {
                *pdata16bits = (uint16_t)(husart->Instance->RHR & 0x01FF);
                pdata16bits++;
            }
            else
            {
                *pdata8bits = (uint8_t)(husart->Instance->RHR & (uint8_t)0x00FF);
                pdata8bits++;
            }
            husart->RxXferCount--;
        }

        /* At end of Rx process, restore husart->RxState to Ready */
        husart->RxState = HAL_USART_STATE_READY;

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @note   When USART Word Length is configured to 9 bits (MODE9 = 1),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, const uint8_t *pData, uint16_t Size)
{
    /* Check that a Tx process is not already ongoing */
    if (husart->gState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        husart->pTxBuffPtr = pData;
        husart->TxXferSize = Size;
        husart->TxXferCount = Size;

        husart->ErrorCode = HAL_USART_ERROR_NONE;
        husart->gState = HAL_USART_STATE_BUSY_TX;

        /* Process Unlocked */
        __HAL_UNLOCK(husart);

        /* Enable the USART Transmit data register empty Interrupt */
        __HAL_USART_ENABLE_IT(husart, USART_IT_TXRDY);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in non blocking mode.
  * @note   When USART Word Length is configured to 9 bits (MODE9 = 1),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size)
{
    /* Check that a Rx process is not already ongoing */
    if (husart->RxState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        return (USART_Start_Receive_IT(husart, pData, Size));
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Sends an amount of data in DMA mode.
  * @note   When USART Word Length is configured to 9 bits (MODE9 = 1),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, const uint8_t *pData, uint16_t Size)
{
    const uint32_t *tmp;

    /* Check that a Tx process is not already ongoing */
    if (husart->gState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        husart->pTxBuffPtr = pData;
        husart->TxXferSize = Size;
        husart->TxXferCount = Size;

        husart->ErrorCode = HAL_USART_ERROR_NONE;
        husart->gState = HAL_USART_STATE_BUSY_TX;

        /* Set the USART DMA Block transfer complete callback */
        husart->hdmatx->XferBlockCallback = USART_DMATransmitCplt;

        /* Set the DMA error callback */
        husart->hdmatx->XferErrorCallback = USART_DMAError;

        /* Enable the USART transmit DMA stream */
        tmp = (const uint32_t *)&pData;
        HAL_DMA_Start_IT(husart->hdmatx, *(const uint32_t *)tmp, (uint32_t)&husart->Instance->THR, Size);

        /* Process Unlocked */
        __HAL_UNLOCK(husart);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in DMA mode.
  * @note   When UART Word Length is configured to 9 bits (MODE9 = 1),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size)
{
    /* Check that a Rx process is not already ongoing */
    if (husart->RxState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        return (USART_Start_Receive_DMA(husart, pData, Size));
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Transmit an amount of data in blocking mode.
  * @note	This function only use in master mode.
  * @param  husart  pointer to a USART_HandleTypeDef structure that contains
  *                     the configuration information for USART module.
  * @param  pData   pointer to data buffer
  * @param  Size    amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_SPI_Transmit(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = 0U;

    /* Check that a Tx process is not already ongoing */
    if (husart->gState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        husart->ErrorCode = HAL_USART_ERROR_NONE;
        husart->gState = HAL_USART_STATE_BUSY_TX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        husart->TxXferSize = Size;
        husart->TxXferCount = Size;

		/* Check if the USART SPI TX is already enabled */
		if ((husart->Instance->CR & USART_CR_SPI_TXEN) != USART_CR_SPI_TXEN)
		{
			/* Enable USART SPI TX */
			USART_SPI_TX_ENABLE(husart);
		}
		
        /* Process Unlocked */
        __HAL_UNLOCK(husart);

        while (husart->TxXferCount > 0U)
        {
            husart->Instance->THR = (uint8_t)(*pData & 0xFFU);
			pData++;
            husart->TxXferCount--;
			
			if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXEMPTY, RESET, tickstart, Timeout) != HAL_OK)
			{
				return HAL_TIMEOUT;
			}
        }

        if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXRDY, RESET, tickstart, Timeout) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

        /* At end of Tx process, End USART transfer */
        USART_EndTxTransfer(husart);

		/* Disable the USART SPI TX */
		USART_SPI_TX_DISABLE(husart);
		
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in blocking mode.
  * @note	This function only use in slave mode.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @param  pData Pointer to data buffer.
  * @param  Size  Amount of data to be received.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_SPI_Receive(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = 0U;

    /* Check that a Rx process is not already ongoing */
    if (husart->RxState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        husart->ErrorCode = HAL_USART_ERROR_NONE;
        husart->RxState = HAL_USART_STATE_BUSY_RX;

		/* Check if the USART SPI RX is already enabled */
		if ((husart->Instance->CR & USART_CR_SPI_RXEN) != USART_CR_SPI_RXEN)
		{
			/* Enable USART SPI RX */
			USART_SPI_RX_ENABLE(husart);
		}
		
        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        husart->RxXferSize = Size;
        husart->RxXferCount = Size;

        /* Process Unlocked */
        __HAL_UNLOCK(husart);

        /* Check the remain data to be received */
        while (husart->RxXferCount > 0U)
        {
            if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_RXRDY, RESET, tickstart, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }

			*pData = (uint8_t)(husart->Instance->RHR & (uint8_t)0xFF);
			pData++;
            husart->RxXferCount--;
        }

		/* Disable USART SPI RX */
		USART_SPI_RX_DISABLE(husart);
		
        /* At end of Rx process, restore husart->RxState to Ready */
        husart->RxState = HAL_USART_STATE_READY;

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  husart Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size    amount of data to be sent and received
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_SPI_TransmitReceive(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout)
{
    uint32_t tickstart = 0U;
	
	/* Variable used to alternate Rx and Tx during transfer */
	HAL_StatusTypeDef    errorcode = HAL_OK;
	
	/* Process Locked */
	__HAL_LOCK(husart);	
	
	if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
	{
		errorcode = HAL_ERROR;
		goto error;
	} 

	/* Don't overwrite in case of HAL_USART_STATE_BUSY_RX */
    if (husart->gState != HAL_USART_STATE_BUSY_RX)
    {
        husart->gState = HAL_USART_STATE_BUSY_TX_RX;
    }

	/* Enable USART SPI TX and RX */
	USART_SPI_TX_ENABLE(husart);
	USART_SPI_RX_ENABLE(husart);
	
	/* Set the transaction information */
    husart->ErrorCode   = HAL_USART_ERROR_NONE;
    husart->RxXferCount = Size;
    husart->RxXferSize  = Size;
    husart->TxXferCount = Size;
    husart->TxXferSize  = Size;
	
	/* Init tickstart for timeout management */
	tickstart = HAL_GetTick();
    
    __HAL_USART_SPI_NSS_ENABLE(husart);
    
	/* Check the remain data to be received */
	while ((husart->RxXferCount > 0U) || (husart->TxXferCount > 0U))
	{
		/* Wait until TX data register is empty */
		if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXRDY, RESET, tickstart, Timeout) != HAL_OK)
		{
			errorcode = HAL_TIMEOUT;
			goto error;
		}
		
		/* Send data using TX data register */
		husart->Instance->THR = (uint8_t)(*pTxData & 0xFFU);
		pTxData++;
		husart->TxXferCount--;
		
		/* Wait until transmission is complete */
		if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXEMPTY, RESET, tickstart, Timeout) != HAL_OK)
		{
			errorcode = HAL_TIMEOUT;
			goto error;
		}
	
		/* Wait until RX data is ready to be read */
		if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_RXRDY, RESET, tickstart, Timeout) != HAL_OK)
		{
			errorcode = HAL_TIMEOUT;
			goto error;
		}
		
		/* Read data from RX data register */
		*pRxData = husart->Instance->RHR;
		pRxData++;
		husart->RxXferCount--;
			
		/* Wait until RX data is completely received */
		if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_RXRDY, SET, tickstart, Timeout) != HAL_OK)
		{
			errorcode = HAL_TIMEOUT;
			goto error;
		}
	}

	/* Disable USART SPI RX and TX */
	USART_SPI_RX_DISABLE(husart);
	USART_SPI_TX_DISABLE(husart);
	
	/* At end of Rx process, restore husart->gState to Ready */
	husart->gState = HAL_USART_STATE_READY;
	
	__HAL_UNLOCK(husart);
    
	__HAL_USART_SPI_NSS_DISABLE(husart);
    
	return HAL_OK;

error :
    husart->gState = HAL_USART_STATE_READY;
    __HAL_UNLOCK(husart);
    return errorcode; 
}

/**
  * @brief  Publish an amount of data in blocking mode.
  * @param  husart  pointer to a USART_HandleTypeDef structure that contains
  *                     the configuration information for USART module.
  * @param  ID	    pointer to ID to be send or receive
  * @param  pData   pointer to data buffer
  * @param  Size    amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_LIN_Publish(USART_HandleTypeDef *husart, uint8_t *ID, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = 0U;

    /* Check that a Tx process is not already ongoing */
    if (husart->gState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        husart->ErrorCode = HAL_USART_ERROR_NONE;
        husart->gState = HAL_USART_STATE_BUSY_TX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        husart->TxXferSize = Size;
        husart->TxXferCount = Size;
		
        /* Process Unlocked */
        __HAL_UNLOCK(husart);

		/* set transmission data length */
		MODIFY_REG(husart->Instance->LINMR, USART_LINMR_DLC, USART_LIN_DLC_LENGTH(Size));

		if((husart->Instance->MR & USART_MR_USART_MODE) == USART_LIN_MODE_MASTER)
		{
			/* If current mode is LIN master, send ID */
			if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXRDY, RESET, tickstart, Timeout) != HAL_OK)
			{
				return HAL_TIMEOUT;
			}
			
			husart->Instance->LINIR = *ID;
		}
		else if((husart->Instance->MR & USART_MR_USART_MODE) == USART_LIN_MODE_SLAVE)
		{
			/* If current mode is LIN slave, receive ID */
			if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_LINID, RESET, tickstart, Timeout) != HAL_OK)
			{
				return HAL_TIMEOUT;
			}
			
			*ID  = (uint8_t)(husart->Instance->LINIR & 0x3fU);
		}
		else
		{
			/* Unsupported USART LIN mode */
			return HAL_ERROR;
		}
		
		/* set LIN work mode */
		MODIFY_REG(husart->Instance->LINMR, USART_LINMR_NACT, USART_LIN_WORK_MODE_PUBLISH);
		
		/* Send LIN data  */
        while (husart->TxXferCount > 0U)
        {
            if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXRDY, RESET, tickstart, Timeout) != HAL_OK)
			{
				return HAL_TIMEOUT;
			}
			
			husart->Instance->THR = (uint8_t)(*pData & 0xFFU);
			pData++;
            husart->TxXferCount--;
        }

		/* Wait until transmission of last data byte is complete */
        if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_LINTC, RESET, tickstart, Timeout) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

		/* Reset USART LIN status */ 
		SET_BIT(husart->Instance->CR, USART_CR_RSTSTA);
		
        /* At end of Tx process, End USART transfer */
        USART_EndTxTransfer(husart);
		
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Subscribe an amount of data in blocking mode.
  * @param  husart  pointer to a USART_HandleTypeDef structure that contains
  *                     the configuration information for USART module.
  * @param  ID	    pointer to ID to be send or receive
  * @param  pData   pointer to data buffer
  * @param  Size    amount of data to be receive
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_USART_LIN_Subscribe(USART_HandleTypeDef *husart, uint8_t *ID, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = 0U;

    /* Check that a Tx process is not already ongoing */
    if (husart->gState == HAL_USART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(husart);

        husart->ErrorCode = HAL_USART_ERROR_NONE;
        husart->RxState = HAL_USART_STATE_BUSY_RX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        husart->RxXferSize = Size;
        husart->RxXferCount = Size;
		
        /* Process Unlocked */
        __HAL_UNLOCK(husart);

		/* set transmission data length */
		MODIFY_REG(husart->Instance->LINMR, USART_LINMR_DLC, USART_LIN_DLC_LENGTH(Size));
		
		/* set LIN work mode */
		MODIFY_REG(husart->Instance->LINMR, USART_LINMR_NACT, USART_LIN_WORK_MODE_SUBSCRIBE);
		
		if((husart->Instance->MR & USART_MR_USART_MODE) == USART_LIN_MODE_MASTER)
		{
			/* If current mode is LIN master, send ID */
			if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_TXRDY, RESET, tickstart, Timeout) != HAL_OK)
			{
				return HAL_TIMEOUT;
			}
			
			husart->Instance->LINIR = (uint8_t)*ID;
		}
		else if((husart->Instance->MR & USART_MR_USART_MODE) == USART_LIN_MODE_SLAVE)
		{
			/* If current mode is LIN slave, receive ID */
			if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_LINID, RESET, tickstart, Timeout) != HAL_OK)
			{
				return HAL_TIMEOUT;
			}
			
			*ID  = (uint8_t)(husart->Instance->LINIR & 0x3fU);
		}
		else
		{
			/* Unsupported USART LIN mode */
			return HAL_ERROR;
		}
		
		/* Receive LIN data */  
        while (husart->RxXferCount > 0U)
        {
            if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_RXRDY, RESET, tickstart, Timeout) != HAL_OK)
			{
				return HAL_TIMEOUT;
			}
			
			*pData = (uint8_t)husart->Instance->RHR;
			pData++;
            husart->RxXferCount--;
        }

		/* Wait until reception of last data byte is complete */
        if (USART_WaitOnFlagUntilTimeout(husart, USART_FLAG_LINTC, RESET, tickstart, Timeout) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

		/* Reset USART LIN status */ 
		SET_BIT(husart->Instance->CR, USART_CR_RSTSTA);
		
        /* At end of Rx process, restore husart->RxState to Ready */
        husart->RxState = HAL_USART_STATE_READY;
		
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  End ongoing Tx transfer on USART peripheral (following Transmit completion).
  * @param  husart USART handle.
  * @retval None
  */
static void USART_EndTxTransfer(USART_HandleTypeDef *husart)
{
    /* Disable the USART TXRDY Interrupt */
    __HAL_USART_DISABLE_IT(husart, USART_IT_TXRDY);
    /* At end of Tx process, restore huart->gState to Ready */
    husart->gState = HAL_USART_STATE_READY;
}

/**
  * @brief  End ongoing Rx transfer on USART peripheral (following Reception completion).
  * @param  huart USART handle.
  * @retval None
  */
static void USART_EndRxTransfer(USART_HandleTypeDef *husart)
{
    /* Disable USART RXRDY, PARE, FRAME, OVRE, TIMEOUT interrupts */
    __HAL_USART_DISABLE_IT(husart, (USART_IDR_RXRDY | USART_IDR_PARE | USART_IDR_FRAME | USART_IDR_OVRE | USART_IDR_TIMEOUT));

    /* At end of Rx process, restore husart->RxState to Ready */
    husart->RxState = HAL_USART_STATE_READY;
}

/**
  * @brief  DMA USART transmit process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void USART_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
    USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    
    
    /* DMA Normal mode*/
   
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == 0U)
    {
        husart->TxXferCount = 0x00U;
		
        HAL_USART_TxCpltCallback(husart);
        
		husart->gState = HAL_USART_STATE_READY;
    }
    /* DMA Circular mode */
    else
    {
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Tx complete callback*/
        husart->TxCpltCallback(husart);
#else
        /*Call legacy weak Tx complete callback*/
        HAL_USART_TxCpltCallback(husart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
    }
}

/**
  * @brief  DMA USART receive process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void USART_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
    USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    /* DMA Normal mode*/
  
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == 0U)
    {
        husart->RxXferCount = 0U;
        
       
        /* At end of Rx process, restore huart->RxState to Ready */
        husart->RxState = HAL_USART_STATE_READY;
    }

    /* In other cases : use Rx Complete callback */
#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    /*Call registered Rx complete callback*/
    husart->RxCpltCallback(husart);
#else
    /*Call legacy weak Rx complete callback*/
    HAL_USART_RxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

}

/**
  * @brief  Initialize the callbacks to their default values.
  * @param  husart USART handle.
  * @retval none
  */
#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
void USART_InitCallbacksToDefault(USART_HandleTypeDef *husart)
{
    /* Init the USART Callback settings */
    husart->TxCpltCallback            = HAL_USART_TxCpltCallback;            /* Legacy weak TxCpltCallback            */
    husart->RxCpltCallback            = HAL_USART_RxCpltCallback;            /* Legacy weak RxCpltCallback            */
    husart->ErrorCallback             = HAL_USART_ErrorCallback;             /* Legacy weak ErrorCallback             */
}
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

/**
  * @brief  This function handles USART Communication Timeout. It waits
  *         until a flag is no longer in the specified status.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @param  Flag specifies the USART flag to check.
  * @param  Status The actual Flag status (SET or RESET).
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef USART_WaitOnFlagUntilTimeout(USART_HandleTypeDef *husart, uint32_t Flag, FlagStatus Status,
        uint32_t Tickstart, uint32_t Timeout)
{
    /* Wait until flag is set */
    while ((__HAL_USART_GET_FLAG(husart, Flag) ? SET : RESET) == Status)
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
            {
                /* Disable TXEMPTY, RXRDY, FRAME and PARE interrupts for the interrupt process */
                ATOMIC_CLEAR_BIT(husart->Instance->IER, USART_IER_TXEMPTY | USART_IER_RXRDY | USART_IER_FRAME | USART_IER_PARE);
                husart->gState  = HAL_USART_STATE_READY;
                husart->RxState = HAL_USART_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(husart);

                return HAL_TIMEOUT;
            }
        }
    }
    return HAL_OK;
}

/**
  * @brief  DMA USART communication error callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void USART_DMAError(DMA_HandleTypeDef *hdma)
{
    USART_HandleTypeDef *husart = (USART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

    /* Stop USART DMA Tx request if ongoing */
    if (husart->gState == HAL_USART_STATE_BUSY_TX)
    {
        husart->TxXferCount = 0x00U;
        USART_EndTxTransfer(husart);
    }

    /* Stop USART DMA Rx request if ongoing */
    if (husart->RxState == HAL_USART_STATE_BUSY_RX)
    {
        husart->RxXferCount = 0x00U;
        USART_EndRxTransfer(husart);
    }

    husart->ErrorCode |= HAL_USART_ERROR_DMA;
#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    /* Call registered error callback */
    husart->ErrorCallback(husart);
#else
    /* Call legacy weak error callback */
    HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef USART_Transmit_IT(USART_HandleTypeDef *husart)
{
    const uint16_t *tmp;

    /* Check that a Tx process is ongoing */
    if (husart->gState == HAL_USART_STATE_BUSY_TX)
    {
        if (husart->Init.WordLength == USART_WORDLENGTH_9B)
        {
            tmp = (const uint16_t *) husart->pTxBuffPtr;
            husart->Instance->THR = (uint16_t)(*tmp & (uint16_t)0x01FF);
            husart->pTxBuffPtr += 2U;
        }
        else
        {
            husart->Instance->THR = (uint8_t)(*husart->pTxBuffPtr++ & (uint8_t)0x00FF);
        }

        if (--husart->TxXferCount == 0U)
        {
            /* USART in mode Transmitter end */
            USART_EndTransmit_IT(husart);
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
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef USART_EndTransmit_IT(USART_HandleTypeDef *husart)
{
    /* Disable the USART Transmit Data Register Empty Interrupt */
    __HAL_USART_DISABLE_IT(husart, USART_IT_TXRDY);

    /* Tx process is ended, restore huart->gState to Ready */
    husart->gState = HAL_USART_STATE_READY;

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
    /*Call registered Tx complete callback*/
    husart->TxCpltCallback(husart);
#else
    /*Call legacy weak Tx complete callback*/
    HAL_USART_TxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

    return HAL_OK;
}

/**
  * @brief  Receives an amount of data in non blocking mode
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef USART_Receive_IT(USART_HandleTypeDef *husart)
{
    uint8_t  *pdata8bits;
    uint16_t *pdata16bits;

    /* Check that a Rx process is ongoing */
    if (husart->RxState == HAL_USART_STATE_BUSY_RX)
    {
        if (husart->Init.WordLength == USART_WORDLENGTH_9B)
        {
            pdata8bits  = NULL;
            pdata16bits = (uint16_t *) husart->pRxBuffPtr;
            *pdata16bits = (uint16_t)(husart->Instance->RHR & (uint16_t)0x01FF);
            husart->pRxBuffPtr += 2U;
        }
        else
        {
            pdata8bits = (uint8_t *) husart->pRxBuffPtr;
            pdata16bits  = NULL;
            *pdata8bits = (uint8_t)(husart->Instance->RHR & (uint8_t)0x00FF);
            husart->pRxBuffPtr += 1U;
        }

        if (--husart->RxXferCount == 0U)
        {
            /* Disable the USART RXRDY Interrupt */
            __HAL_USART_DISABLE_IT(husart, USART_IT_RXRDY);

            /* Disable the USART PARE, FRAME, OVRE, TIMEOUT Interrupt */
			__HAL_USART_DISABLE_IT(husart, USART_IT_PARE | USART_IT_FRAME | USART_IT_OVRE | USART_IT_TIMEOUT);

            /* Rx process is completed, restore husart->RxState to Ready */
            husart->RxState = HAL_USART_STATE_READY;

            /* Standard reception API called */
#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
            /*Call registered Rx complete callback*/
            husart->RxCpltCallback(husart);
#else
            /*Call legacy weak Rx complete callback*/
            HAL_USART_RxCpltCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */

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
  * @brief  Configures the USART peripheral.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval None
  */
static void USART_SetConfig(USART_HandleTypeDef *husart)
{
	uint8_t over = 0;
	uint32_t integer,fraction;
    uint32_t tmpreg;
    uint32_t pclk;
	
	/* USART UNLOCK REGISTER */
	__HAL_USART_UNLOCK_REGISTER(husart);
	
    /*-------------------------- USART MR Configuration ------------------------*/
    /* Configure the USART HwFlowCtl: Set MODE Bits
     according to huart->Init.HwFlowCtl value */
	if(husart->Init.HwFlowCtl != USART_HWCONTROL_NONE)
	{
		MODIFY_REG(husart->Instance->MR, USART_MR_USART_MODE, husart->Init.HwFlowCtl);
	}

	/* Configure the USART Word Length, Parity and Stop Bits:
	   Set MODE9 and CHRL bits according to husart->Init.WordLength value
       Set PAR bits according to husart->Init.Parity value
       Set NBSTOP bits according to husart->Init.StopBits value */
	tmpreg = husart->Init.WordLength | husart->Init.Parity | husart->Init.StopBits | husart->Init.OverSampling;
	MODIFY_REG(husart->Instance->MR, USART_MR_CHRL | USART_MR_MODE9 | USART_MR_PAR | USART_MR_NBSTOP | USART_MR_OVER, tmpreg);
	
    if (husart->Instance == USART6)
    {
		__HAL_RCM_USART6_CLK_SEL_PCLK0();
        pclk = HAL_RCM_GetPCLK0Freq();
    }
    else
    {
      #if defined(UM32x42x)
		__HAL_RCM_USART7_CLK_SEL_PCLK1();
        pclk = HAL_RCM_GetPCLK1Freq();
        #endif 
        #if defined(UM324xF)
        __HAL_RCM_USART7_CLK_SEL_PCLK2();
        pclk = HAL_RCM_GetPCLK2Freq();
        #endif
    }
	
    /*-------------------------- USART Baud Rate Configuration ------------------*/
	if(husart->Init.OverSampling != USART_OVERSAMPLING_8)
	{
		over = 0;
	}
	else
	{
		over = 1;
	}
	
	integer = (uint32_t)(pclk)/(uint32_t)(8 *(2-over)*husart->Init.BaudRate);                          //CD = PCLK/(8*(2-Over)*baud_rate)
	fraction = (uint32_t)((8 * (pclk - 8*husart->Init.BaudRate*integer*(2-over)) \
	            + husart->Init.BaudRate* 8* (2-over) / 2)/(husart->Init.BaudRate* 8* (2-over)));           //FP = 8*(PCLK - 8*baud_rate*CD*(2-over))/(baud_rate*8*(2-over))
	WRITE_REG(husart->Instance->BRGR, integer | (fraction<<16));
	
	/*-------------------------- USART MR Configuration ------------------------*/
	/* Configure the USART MODE:
	   Set RXEN and TXEN bits according to husart->Init.Mode value */
	MODIFY_REG(husart->Instance->CR, USART_CR_RXEN | USART_CR_TXEN, husart->Init.Mode);
}

/**
  * @brief  Configures the USART peripheral.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval None
  */
static void USART_SPI_SetConfig(USART_HandleTypeDef *husart)
{
	/* USART UNLOCK REGISTER */
	__HAL_USART_UNLOCK_REGISTER(husart);
	
	if(husart->SpiInit.Mode == USART_SPI_MODE_MASTER)
	{
		/*-------------------------- USART MR Configuration ------------------------*/
		/* Configure the USART MODE:
		Set SPI Mode according to husart->SpiInit.Mode value 
		Set CHRL Bits to 8bits
		Set CLKO Bit enable USART drive SCK
		*/
		WRITE_REG(husart->Instance->MR, USART_MR_SPI_CLKO |  USART_MR_SPI_CHRL | husart->SpiInit.Mode);
		
		/*-------------------------- USART BRGR Configuration ----------------------*/
		WRITE_REG(husart->Instance->BRGR, husart->SpiInit.BaudRatePrescaler);
	}
	else
	{
		/*-------------------------- USART MR Configuration ------------------------*/
		WRITE_REG(husart->Instance->MR,  USART_MR_SPI_USCLKS | USART_MR_SPI_CHRL | husart->SpiInit.Mode);
	}
	
	/* Configure the USART SPI clock polarity and clock phase:
	   Set CPOL bits according to husart->SpiInit.CLKPolarity
       Set CPHA bits according to husart->CLKPhase */
	MODIFY_REG(husart->Instance->MR, USART_MR_SPI_CPOL | USART_MR_SPI_CPHA, husart->SpiInit.CLKPolarity | husart->SpiInit.CLKPhase);
}

/**
  * @brief  Configures the USART LIN peripheral.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval None
  */
static void USART_LIN_SetConfig(USART_HandleTypeDef *husart)
{
	uint8_t over = 0;
	uint32_t integer,fraction;
    uint32_t pclk;
	
	/* USART UNLOCK REGISTER */
	__HAL_USART_UNLOCK_REGISTER(husart);
	
	/* Reset transitter and receiver */
	SET_BIT(husart->Instance->CR, USART_CR_RSTRX | USART_CR_RSTTX);
	
    /*-------------------------- USART MR Configuration ------------------------*/
    /* Configure the USART Lin Mode: Set MODE Bits
     according to huart->LinInit.Mode value */
	MODIFY_REG(husart->Instance->MR, USART_MR_USART_MODE, husart->LinInit.Mode);
	
	/* Configure the USART Lin OverSampling:
	   Set OVER bit according to husart->Init.OverSampling value */
	MODIFY_REG(husart->Instance->MR, USART_MR_OVER, husart->Init.OverSampling);
	
	/* Enable USART TX and RX */
	SET_BIT(husart->Instance->CR, USART_CR_TXEN | USART_CR_RXEN);
	
    if (husart->Instance == USART6)
    {
      
		__HAL_RCM_USART6_CLK_SEL_PCLK0();
        pclk = HAL_RCM_GetPCLK0Freq();
    }
    else
    {
	    #if defined(UM32x42x)
		__HAL_RCM_USART7_CLK_SEL_PCLK1();
        pclk = HAL_RCM_GetPCLK1Freq();
        #endif 
        #if defined(UM324xF)
        __HAL_RCM_USART7_CLK_SEL_PCLK2();
        pclk = HAL_RCM_GetPCLK2Freq();
        #endif
    }
	
    /*-------------------------- USART Lin Baud Rate Configuration ------------------*/
	if(husart->Init.OverSampling != USART_OVERSAMPLING_8)
	{
		over = 0;
	}
	else
	{
		over = 1;
	}
	
	integer = (uint32_t)(pclk)/(uint32_t)(8 *(2-over)*husart->LinInit.BaudRate);                          //CD = PCLK/(8*(2-Over)*baud_rate)
	fraction = (uint32_t)((8 * (pclk - 8*husart->LinInit.BaudRate*integer*(2-over)) \
	            + husart->LinInit.BaudRate* 8* (2-over) / 2)/(husart->LinInit.BaudRate* 8* (2-over)));           //FP = 8*(PCLK - 8*baud_rate*CD*(2-over))/(baud_rate*8*(2-over))
	WRITE_REG(husart->Instance->BRGR, integer | (fraction<<16));
}

/**
  * @brief  This function handles USART interrupt request.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval None
  */
void HAL_USART_IRQHandler(USART_HandleTypeDef *husart)
{
	uint32_t csrflags   = READ_REG(husart->Instance->CSR);
    uint32_t imrits     = READ_REG(husart->Instance->IMR);
    uint32_t errorflags = 0x00U;
	
	/* If no error occurs */
	errorflags = (csrflags & (uint32_t)(USART_CSR_OVRE | USART_CSR_FRAME | USART_CSR_PARE | USART_CSR_TIMEOUT));
	
	if (errorflags == RESET)
	{
		/* USART in mode Receiver -------------------------------------------------*/
		if (((csrflags & USART_CSR_RXRDY) != RESET) && ((imrits & USART_IMR_RXRDY) != RESET))
		{
			USART_Receive_IT(husart);
			return;
		}
	}
	
	/* If some errors occur */
    if ((errorflags != RESET) && (((imrits & ((uint32_t) USART_IMR_OVRE | USART_IMR_PARE | USART_IMR_FRAME | USART_IMR_TIMEOUT)) != RESET)))
    {
        /* USART parity error interrupt occurred ----------------------------------*/
        if (((csrflags & USART_CSR_PARE) != RESET) && ((imrits & USART_IMR_PARE) != RESET))
        {
            husart->ErrorCode |= HAL_USART_ERROR_PARE;
        }

        /* USART frame error interrupt occurred -----------------------------------*/
        if (((csrflags & USART_CSR_FRAME) != RESET) && ((imrits & USART_IMR_FRAME) != RESET))
        {
            husart->ErrorCode |= HAL_USART_ERROR_FRAME;
        }

        /* USART Over-Run interrupt occurred --------------------------------------*/
        if (((csrflags & USART_CSR_OVRE) != RESET) && ((imrits & USART_IMR_OVRE) != RESET))
        {
            husart->ErrorCode |= HAL_USART_ERROR_OVRE;
        }
		
		/* USART Timeout interrupt occurred --------------------------------------*/
        if (((csrflags & USART_CSR_TIMEOUT) != RESET) && ((imrits & USART_IMR_TIMEOUT) != RESET))
        {
            husart->ErrorCode |= HAL_USART_ERROR_TIMEOUT;
        }
		
		/* Call USART Error Call back function if need be --------------------------*/
        if (husart->ErrorCode != HAL_USART_ERROR_NONE)
        {
            /* USART in mode Receiver -----------------------------------------------*/
            if (((csrflags & USART_CSR_TXRDY) != RESET) && ((imrits & USART_IMR_TXRDY) != RESET))
            {
                USART_Receive_IT(husart);
            }

			/* Parity error occurred */
			if (((csrflags & USART_CSR_PARE) != RESET) && ((imrits & USART_IMR_PARE) != RESET))
			{
				USART_EndRxTransfer(husart);

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
				/*Call registered error callback*/
				husart->ErrorCallback(husart);
#else
				/*Call legacy weak error callback*/
				HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
			}
			
			/* Frame error occurred */
			if (((csrflags & USART_CSR_FRAME) != RESET) && ((imrits & USART_IMR_FRAME) != RESET))
			{
				USART_EndRxTransfer(husart);

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
				/*Call registered error callback*/
				husart->ErrorCallback(husart);
#else
				/*Call legacy weak error callback*/
				HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
			}
			
			/* Over-Run error occurred */
			if (((csrflags & USART_CSR_OVRE) != RESET) && ((imrits & USART_IMR_OVRE) != RESET))
			{
				USART_EndRxTransfer(husart);

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
				/*Call registered error callback*/
				husart->ErrorCallback(husart);
#else
				/*Call legacy weak error callback*/
				HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
			}
			
			/* Timeout occurred */
			if (((csrflags & USART_CSR_TIMEOUT) != RESET) && ((imrits & USART_IMR_TIMEOUT) != RESET))
			{
				USART_EndRxTransfer(husart);

#if (USE_HAL_USART_REGISTER_CALLBACKS == 1)
				/*Call registered error callback*/
				husart->ErrorCallback(husart);
#else
				/*Call legacy weak error callback*/
				HAL_USART_ErrorCallback(husart);
#endif /* USE_HAL_USART_REGISTER_CALLBACKS */
			}
        }
		
        return;
    } /* End if some error occurs */
	
    /* USART in mode Transmitter ------------------------------------------------*/
    if (((csrflags & USART_CSR_TXRDY) != RESET) && ((imrits & USART_IMR_TXRDY) != RESET))
    {
		USART_Transmit_IT(husart);
		return;
    }
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval None
  */
__weak void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(husart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_USART_TxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval None
  */
__weak void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(husart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_USART_RxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  USART error callbacks.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval None
  */
__weak void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(husart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_USART_ErrorCallback could be implemented in the user file
     */
}

/** @defgroup USART_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   USART State and Errors functions
  *
@verbatim
  ==============================================================================
                 ##### Peripheral State and Errors functions #####
  ==============================================================================
 [..]
   This subsection provides a set of functions allowing to return the State of
   UART communication process, return Peripheral Errors occurred during communication
   process
   (+) HAL_USART_GetState() API can be helpful to check in run-time the state of the USART peripheral.
   (+) HAL_USART_GetError() check in run-time errors that could be occurred during communication.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the USART state.
  * @param  husart  Pointer to a USART_HandleTypeDef structure that contains
  *                 the configuration information for the specified USART module.
  * @retval HAL state
  */
HAL_USART_StateTypeDef HAL_USART_GetState(USART_HandleTypeDef *husart)
{
    uint32_t temp1 = 0x00U, temp2 = 0x00U;
    temp1 = husart->gState;
    temp2 = husart->RxState;

    return (HAL_USART_StateTypeDef)(temp1 | temp2);
}

/**
  * @brief  Return the USART error code
  * @param  husart Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART.
  * @retval UART Error Code
  */
uint32_t HAL_USART_GetError(USART_HandleTypeDef *husart)
{
    return husart->ErrorCode;
}

/**
  * @brief  Start Receive operation in interrupt mode.
  * @note   This function could be called by all HAL USART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         USART Handle is assumed as Locked.
  * @param  husart USART handle.
  * @param  pData  Pointer to data buffer (u8 or u16 data elements).
  * @param  Size   Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef USART_Start_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size)
{
    husart->pRxBuffPtr = pData;
    husart->RxXferSize = Size;
    husart->RxXferCount = Size;

    husart->ErrorCode = HAL_USART_ERROR_NONE;
    husart->RxState = HAL_USART_STATE_BUSY_RX;

    /* Process Unlocked */
    __HAL_UNLOCK(husart);

    /* Enable the USART RXRDY, PARE, FRAME, OVRE, TIMEOUT Interrupt */
    __HAL_USART_ENABLE_IT(husart, USART_IT_RXRDY | USART_IT_PARE | USART_IT_FRAME | USART_IT_OVRE | USART_IT_TIMEOUT);

    return HAL_OK;
}

/**
  * @brief  Start Receive operation in DMA mode.
  * @note   This function could be called by all HAL UART API providing reception in DMA mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         USART Handle is assumed as Locked.
  * @param  husart USART handle.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef USART_Start_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pData, uint16_t Size)
{
    uint32_t *tmp;

    husart->pRxBuffPtr = pData;
    husart->RxXferSize = Size;

    husart->ErrorCode = HAL_USART_ERROR_NONE;
    husart->RxState = HAL_USART_STATE_BUSY_RX;

    /* Set the USART DMA transfer complete callback */
    husart->hdmarx->XferBlockCallback = USART_DMAReceiveCplt;

    /* Set the DMA error callback */
    husart->hdmarx->XferErrorCallback = USART_DMAError;

    /* Enable the DMA stream */
    tmp = (uint32_t *)&pData;
    HAL_DMA_Start_IT(husart->hdmarx, (uint32_t)&husart->Instance->RHR, *(uint32_t *)tmp, Size);

    /* Process Unlocked */
    __HAL_UNLOCK(husart);

    return HAL_OK;
}

/**
  * @}
  */

#endif /* HAL_USART_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
