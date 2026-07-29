/**
  ******************************************************************************
  * @file     um324xx_hal_uart.c 
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-04-11
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

/** @defgroup UART UART
  * @brief HAL UART module driver
  * @{
  */
#ifdef HAL_UART_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup UART_Private_Functions  UART Private Functions
  * @{
  */
static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                                     uint32_t Tickstart, uint32_t Timeout);
static void UART_SetConfig(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart);  
/**
  * @}
  */
  
/* Exported functions ---------------------------------------------------------*/
/** @defgroup UART_Exported_Functions UART Exported Functions
  * @{
  */

/**
  * @brief  UART Init.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
{
      /* Check Null pointer */
    if((huart == NULL)||(huart->Instance == NULL))
    {
        return HAL_ERROR;
    }

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    if (huart->MspDeInitCallback == NULL)
    {
        huart->MspDeInitCallback = HAL_UART_MspInit;
    }
    /* DeInit the low level hardware */
    huart->MspDeInitCallback(huart);
#else
    /* DeInit the low level hardware */
    HAL_UART_MspInit(huart);
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS) */

    huart->gState = HAL_UART_STATE_BUSY;

    /* Set the UART Communication parameters */
    UART_SetConfig(huart);
    
    /* Initialize the UART state */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_READY;
    huart->RxState = HAL_UART_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  UART  Config.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @retval None
  */
static void UART_SetConfig(UART_HandleTypeDef *huart)
{
    uint32_t tmpreg;
    uint32_t sysclk;
    uint8_t MSbyte, LSbyte;
    uint16_t temp;
  
    tmpreg = (uint32_t)(huart->Init.Mode | huart->Init.Parity);

    MODIFY_REG(huart->Instance->CR, (uint32_t)(UART_CR_UART_LB | UART_CR_UART_PD|\
                     UART_CR_FLUSH | UART_CR_TRS | UART_CR_ODD_EN), tmpreg);
    #if defined(UM324xF) ||  defined(UM32x42x)
    if(huart->Instance != UART2)
    {
        /* Obtain the APB1 clock */
        sysclk = HAL_RCM_GetPCLK1Freq();
    }
    else
    {
        /* Obtain the APB0 clock */
        sysclk = HAL_RCM_GetPCLK0Freq();        
    }
    #endif
    #if defined(UM32x41x)
     sysclk = HAL_RCM_GetPCLK1Freq();
    #endif
    
    temp = (sysclk + (huart->Init.BaudRate/2)) / huart->Init.BaudRate;

    MSbyte = temp>>8;
    LSbyte = temp;
    
    /* Writes the value to the baud rate register */
    huart->Instance->BRPH = MSbyte;
    huart->Instance->BRPL = LSbyte;
    
    /* Clears the operation to receive the fifo */
    SET_BIT(huart->Instance->CR,UART_FIFO_CLEAR);

    CLEAR_BIT(huart->Instance->CR,UART_FIFO_CLEAR);
    
    /* Clear status registe */
    CLEAR_REG(huart->Instance->ISR);   
}

/**
  * @brief  Sends an amount of data in blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer u8 data elements.
  * @param  Size  Amount of data elements u8 to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Check that a Tx process is not already ongoing */
  if (huart->gState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }
    
    /* Process Locked */
    __HAL_LOCK(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_BUSY_TX;

    /* Init tickstart for timeout management */
    tickstart = HAL_GetTick();

    huart->TxXferSize = Size;
    huart->TxXferCount = Size;

    /* Process Unlocked */
    __HAL_UNLOCK(huart);
    
//     __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TXEND);
    
    while (huart->TxXferCount > 0U)
    {
        huart->Instance->TDR = *pData;
        /* Wait flag bit timed out */
        if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXEND, RESET, tickstart, Timeout) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

        pData++;
        huart->TxXferCount--;

        /* The software needs to clear the send completed flag */
        CLEAR_BIT(huart->Instance->ISR,UART_FLAG_TXEND);
    }

    /* At end of Tx process, restore huart->gState to Ready */
    huart->gState = HAL_UART_STATE_READY;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }

}

/**
  * @brief  Receives an amount of data in blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be received.
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart = 0U;
    
    /* Check that a Rx process is not already ongoing */
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        if ((pData == NULL) || (Size == 0U))
        {
        return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(huart);
    
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_BUSY_RX;

        /* Init tickstart for timeout management */
        tickstart = HAL_GetTick();

        huart->RxXferSize = Size;
        huart->RxXferCount = Size;
    
        /* Process Unlocked */
        __HAL_UNLOCK(huart);
    
        /* Check the remain data to be received */
        while (huart->RxXferCount > 0U)
        {
            if (UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_FIFO_NO_EMPTY, RESET, tickstart, Timeout) != HAL_OK)
            {
                return HAL_TIMEOUT;
            }
            
            *pData = huart->Instance->RDR;

            pData++;
            huart->RxXferCount--;
        }
    
        /* At end of Rx process, restore huart->RxState to Ready */
        huart->RxState = HAL_UART_STATE_READY;
    
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }   
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
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
                                                     uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
      {
        /* Disable interrupts for the interrupt process */
        CLEAR_BIT(huart->Instance->ISR, (UART_IT_FIFO_NO_EMPTY | UART_IT_FIFO_HALF_FULL | UART_IT_FIFO_FULL |\
                                        UART_IT_FIFO_ORF | UART_IT_TXEND | UART_IT_PARITY_ERR));

        huart->gState  = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
  /* Check that a Tx process is not already ongoing */
  if (huart->gState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    /* Enable the UART Completion of sending Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_TXEND);
    __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TXEND);

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pTxBuffPtr = pData;
    huart->TxXferSize = Size;
    huart->TxXferCount = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState = HAL_UART_STATE_BUSY_TX;

    UART_Transmit_IT(huart);

     /* Process Unlocked */
    __HAL_UNLOCK(huart);
    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Receives an amount of data in non blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    return (UART_Start_Receive_IT(huart, pData, Size));
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Wraps up transmission in non blocking mode.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart)
{
    /* Check that a Tx process is ongoing */
    if (huart->gState == HAL_UART_STATE_BUSY_TX)
    {
        if (huart->TxXferCount == 0U)
        { 
            huart->TxXferCount = 0;
            huart->TxXferSize = 0;

            /* Tx process is ended, restore huart->gState to Ready */
            huart->gState = HAL_UART_STATE_READY;
            
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXEND);
            __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TXEND);
            return HAL_OK;
        }
        else
        {            
             huart->Instance->TDR = (uint8_t)(*huart->pTxBuffPtr);
            
             huart->TxXferCount -= 1;
             huart->pTxBuffPtr++;
        }
        
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
        /*Call registered Tx complete callback*/
        huart->TxCpltCallback(huart);
#else
        /*Call legacy weak Tx complete callback*/
        HAL_UART_TxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

        __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TXEND);
        /* Tx process is ended, restore huart->gState to Ready */

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

/**
  * @brief  Receives an amount of data in non blocking mode
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart)
{
    uint8_t  *pdata8bits;
    
    /* Check that a Rx process is ongoing */
    if (huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        pdata8bits = (uint8_t *) huart->pRxBuffPtr;
        *pdata8bits = (uint8_t)(huart->Instance->RDR & (uint8_t)0x00FF);
        huart->pRxBuffPtr += 1U;

        if (--huart->RxXferCount == 0U)
        {
            /* Disable the UART Data Register not empty Interrupt */
            __HAL_UART_DISABLE_IT(huart, UART_IT_FIFO_NO_EMPTY);

            /* Disable the UART FIFO halt full Interrupt */
            __HAL_UART_DISABLE_IT(huart, UART_IT_FIFO_HALF_FULL);

            /* Disable the UART FIFO full Interrupt */
            __HAL_UART_DISABLE_IT(huart, UART_IT_FIFO_FULL);

            /* Disable the UART parity error Interrupt*/
            __HAL_UART_DISABLE_IT(huart, UART_IT_PARITY_ERR);
            
            /* Disable the UART Data overflow */
            __HAL_UART_DISABLE_IT(huart, UART_IT_FIFO_ORF);

            /* Rx process is completed, restore huart->RxState to Ready */
            huart->RxState = HAL_UART_STATE_READY;
            
/* Standard reception API called */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
                /*Call registered Rx complete callback*/
                huart->RxCpltCallback(huart);
#else
                /*Call legacy weak Rx complete callback*/
                HAL_UART_RxCpltCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */
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
  * @brief  Returns the UART state.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL state
  */
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart)
{
    uint32_t temp1 = 0x00U, temp2 = 0x00U;
    temp1 = huart->gState;
    temp2 = huart->RxState;

    return (HAL_UART_StateTypeDef)(temp1 | temp2);
}

/**
  * @brief  Return the UART error code
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains
  *               the configuration information for the specified UART.
  * @retval UART Error Code
  */
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart)
{
    return huart->ErrorCode;
}

/**
  * @brief  Start Receive operation in interrupt mode.
  * @note   This function could be called by all HAL UART API providing reception in Interrupt mode.
  * @note   When calling this function, parameters validity is considered as already checked,
  *         i.e. Rx State, buffer address, ...
  *         UART Handle is assumed as Locked.
  * @param  huart UART handle.
  * @param  pData Pointer to data buffer (u8 data elements).
  * @param  Size  Amount of data elements u8 to be received.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->RxXferCount = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    if (huart->Init.Parity != UART_PARITY_NONE)
    {
        /* Enable the UART Parity Error Interrupt */
        __HAL_UART_ENABLE_IT(huart, UART_IT_PARITY_ERR);
    }
    /* Enable the UART Data overflow Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_FIFO_ORF);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_FIFO_NO_EMPTY);

    /* Enable the UART Data fifo halt full Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_FIFO_HALF_FULL);

    /* Enable the UART Data fifo full Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_FIFO_FULL);
   
  return HAL_OK;
}

/**
  * @brief  This function handles UART interrupt request.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    uint32_t isrflags   = READ_REG(huart->Instance->ISR);
    uint32_t cr1its     = READ_REG(huart->Instance->IER);
    uint32_t errorflags = 0x00U;

    /* If no error occurs */
    errorflags = (isrflags & (uint32_t)(UART_FLAG_PARITY_ERR | UART_FLAG_RXFIFO_ORF_ERR));
    if (errorflags == RESET)
    {
        /* UART in mode Receiver -------------------------------------------------*/
        if (((isrflags & UART_FLAG_FIFO_NO_EMPTY) != RESET) && ((cr1its & UART_IT_FIFO_NO_EMPTY) != RESET))
        {
            UART_Receive_IT(huart);
            return;
        }

    }
    /* If some errors occur */
    if ((errorflags != RESET) && ((cr1its & (UART_IT_FIFO_ORF | UART_IT_PARITY_ERR)) != RESET))
    {
         /* UART parity error interrupt occurred ----------------------------------*/
        if (((isrflags & UART_FLAG_PARITY_ERR) != RESET) && ((cr1its & UART_IT_PARITY_ERR) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_PE;
        }
        
        /* UART fifo overflow interrupt occurred --------------------------------------*/
        if (((isrflags & UART_FLAG_RXFIFO_ORF_ERR) != RESET) && (((cr1its & UART_IT_FIFO_ORF) != RESET)
                                                     || ((cr1its & UART_IT_FIFO_NO_EMPTY) != RESET)))
        {
            huart->ErrorCode |= HAL_UART_ERROR_ORF;
        }
        
        /* Call UART Error Call back function if need be --------------------------*/
        if (huart->ErrorCode != HAL_UART_ERROR_NONE)
        {
            /* UART in mode Receiver -----------------------------------------------*/
            if (((isrflags & UART_FLAG_FIFO_NO_EMPTY) != RESET) && ((cr1its & UART_IT_FIFO_NO_EMPTY) != RESET))
            {
                UART_Receive_IT(huart);
            }
            /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
            /*Call registered error callback*/
            huart->ErrorCallback(huart);
#else
            /*Call legacy weak error callback*/
            HAL_UART_ErrorCallback(huart);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

            huart->ErrorCode = HAL_UART_ERROR_NONE;
        }
        return;
    } /* End if some error occurs */

    /* UART in mode Transmitter end --------------------------------------------*/
    if (((isrflags & UART_FLAG_TXEND) != RESET) && ((cr1its & UART_IT_TXEND) != RESET))
    {
        UART_Transmit_IT(huart);
        return;
    }
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  UART error callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  UART MSP Init.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
__weak void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_MspInit could be implemented in the user file
   */
}


/**
  * @}
  */

#endif /* HAL_UART_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
