/**
  ******************************************************************************
  * @file     um324xx_hal_i2s.c 
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
 
 
/** @defgroup I2S I2S
  * @brief I2S HAL module driver
  * @{
  */
#ifdef HAL_I2S_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/  
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup I2S_Private_Functions I2S Private Functions
  * @{
  */
static void               I2S_DMATxCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMARxCplt(DMA_HandleTypeDef *hdma);
static void               I2S_DMAError(DMA_HandleTypeDef *hdma);
static void               I2S_Transmit_IT(I2S_HandleTypeDef *hi2s);
static void               I2S_Receive_IT(I2S_HandleTypeDef *hi2s);
static void               I2S_ISRHandler(I2S_HandleTypeDef *hi2s);
static HAL_StatusTypeDef  I2S_WaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s, uint32_t Flag, FlagStatus State,
                                                        uint32_t Timeout);
/**
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/** @defgroup I2S_Exported_Functions I2S Exported Functions
  * @{
  */
  

/** @defgroup I2S_Exported_Functions_Group1 Initialization/de-initialization functions 
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the I2S according to the specified parameters
  *         in the I2S_InitTypeDef and create the associated handle.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s)
{

    /* Check the I2S handle allocation */
    if (hi2s == NULL)
    {
        return HAL_ERROR;
    }


    if (hi2s->State == HAL_I2S_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hi2s->Lock = HAL_UNLOCKED;

        /* Initialize Default I2S IrqHandler ISR */
        hi2s->IrqHandlerISR = I2S_ISRHandler;

    #if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
        /* Init the I2S Callback settings */
        hi2s->TxFifoEnoughCallback       = HAL_I2S_TxFifoEnoughCallback;          /* Legacy weak TxCpltCallback       */
        hi2s->RxFifoEnoughCallback       = HAL_I2S_RxFifoEnoughCallback;          /* Legacy weak RxCpltCallback       */
        hi2s->RxFifoOvfCallback   = HAL_I2S_RxFifoOvfCallback;      /* Legacy weak TxHalfCpltCallback   */
        hi2s->TxFifoUdrCallback   = HAL_I2S_TxFifoUdrCallback;      /* Legacy weak RxHalfCpltCallback   */

        hi2s->FrameErrCallback        = HAL_I2S_FrameErrCallback;           /* Legacy weak ErrorCallback        */

        if (hi2s->MspInitCallback == NULL)
        {
          hi2s->MspInitCallback = HAL_I2S_MspInit; /* Legacy weak MspInit  */
        }

        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        hi2s->MspInitCallback(hi2s);
    #else
        /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        HAL_I2S_MspInit(hi2s);
    #endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
    }

    hi2s->State = HAL_I2S_STATE_BUSY;
  
    __HAL_RCM_UNLOCK_REGISTER();
    
    /* Set I2S clock division */
	  MODIFY_REG(RCM->CFGR0, RCM_CFGR0_I2S_DIV_Msk, hi2s->Init.MclkDiv << RCM_CFGR0_I2S_DIV_Pos);
    
#if defined(UM324xF)
    /* Enable PLL1 */
	  SET_BIT(RCM->CR0, RCM_CR0_PLL1EN);
			/* I2S clk source select PLL1 */
	  CLEAR_BIT(RCM->CFGR0, RCM_CFGR0_I2S_SRC);
	
	  /* Get Start Tick */
    uint32_t tickstart = HAL_GetTick();
	
	__HAL_RCM_LOCK_REGISTER();
#endif    
		
    uint32_t tmpreg = 0U;
    tmpreg = (((uint32_t)(hi2s->Init.PcmSync))|hi2s->Init.MonoDataSel|hi2s->Init.LeftRightFirst |     \
             hi2s->Init.SckEdgeSel|hi2s->Init.WsPolarity|hi2s->Init.ChannelSel|    \
             hi2s->Init.Channellen|hi2s->Init.DataFormat|hi2s->Init.Standard);            
    
    hi2s->Instance->DFR = tmpreg;
  
    tmpreg = (((uint32_t)(hi2s->Init.AccessFiFoMode))|hi2s->Init.IrqEn|hi2s->Init.FillDateSel |     \
             hi2s->Init.TxfifoWtmk|hi2s->Init.RxfifoWtmk|hi2s->Init.Mode);   
  
    hi2s->Instance->GCR = tmpreg;

    __HAL_I2S_SD_DIR_OUTPUT(hi2s);
    __HAL_I2S_ENABLE(hi2s);
  
    __HAL_I2S_TX_ENABLE(hi2s);
    __HAL_I2S_RX_ENABLE(hi2s);
 

    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->State     = HAL_I2S_STATE_READY;

    return HAL_OK;
}

/**
  * @brief DeInitializes the I2S peripheral
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_DeInit(I2S_HandleTypeDef *hi2s)
{
    /* Check the I2S handle allocation */
    if (hi2s == NULL)
    {
        return HAL_ERROR;
    }

    hi2s->State = HAL_I2S_STATE_BUSY;

    /* Disable the I2S Peripheral Clock */
    __HAL_I2S_DISABLE(hi2s);

  #if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
    if (hi2s->MspDeInitCallback == NULL)
    {
        hi2s->MspDeInitCallback = HAL_I2S_MspDeInit; /* Legacy weak MspDeInit  */
    }

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    hi2s->MspDeInitCallback(hi2s);
  #else
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    HAL_I2S_MspDeInit(hi2s);
  #endif /* USE_HAL_I2S_REGISTER_CALLBACKS */

    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->State     = HAL_I2S_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hi2s);

    return HAL_OK;
}

/**
  * @brief I2S MSP Init
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_MspInit could be implemented in the user file
     */
}

/**
  * @brief I2S MSP DeInit
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_MspDeInit could be implemented in the user file
     */
}

#if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User I2S Callback
  *         To be used instead of the weak predefined callback
  * @param  hi2s Pointer to a I2S_HandleTypeDef structure that contains
  *                the configuration information for the specified I2S.
  * @param  CallbackID ID of the callback to be registered
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_RegisterCallback(I2S_HandleTypeDef *hi2s, HAL_I2S_CallbackIDTypeDef CallbackID,
                                           pI2S_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hi2s->ErrorCode |= HAL_I2S_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }
    /* Process locked */
    __HAL_LOCK(hi2s);

    if (HAL_I2S_STATE_READY == hi2s->State)
    {
        switch (CallbackID)
        {
            case HAL_I2S_TXFIFO_ENOUGH_CB_ID :
                hi2s->TxFifoEnoughCallback = pCallback;
                break;

            case HAL_I2S_RXFIFO_ENOUGH_CB_ID :
                hi2s->RxFifoEnoughCallback = pCallback;
                break;

            case HAL_I2S_RXFIFO_OVERFLOW_CB_ID :
                hi2s->RxFifoOvfCallback = pCallback;
                break;

            case HAL_I2S_TXFIFO_UDR_ID :
                hi2s->TxFifoUdrCallback = pCallback;
                break;

            case HAL_I2S_FRAME_ERR_CB_ID :
                hi2s->FrameErrCallback = pCallback;
                break;

            case HAL_I2S_MSPINIT_CB_ID :
                hi2s->MspInitCallback = pCallback;
                break;

            case HAL_I2S_MSPDEINIT_CB_ID :
                hi2s->MspDeInitCallback = pCallback;
                break;

            default :
                /* Update the error code */
                SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_INVALID_CALLBACK);

                /* Return error status */
                status =  HAL_ERROR;
                break;
        }
    }
    else if (HAL_I2S_STATE_RESET == hi2s->State)
    {
        switch (CallbackID)
        {
            case HAL_I2S_MSPINIT_CB_ID :
              hi2s->MspInitCallback = pCallback;
              break;

          case HAL_I2S_MSPDEINIT_CB_ID :
              hi2s->MspDeInitCallback = pCallback;
              break;

          default :
              /* Update the error code */
              SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_INVALID_CALLBACK);

              /* Return error status */
              status =  HAL_ERROR;
              break;
        }
    }
    else
    {
        /* Update the error code */
        SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hi2s);
    return status;
}

/**
  * @brief  Unregister an I2S Callback
  *         I2S callback is redirected to the weak predefined callback
  * @param  hi2s Pointer to a I2S_HandleTypeDef structure that contains
  *                the configuration information for the specified I2S.
  * @param  CallbackID ID of the callback to be unregistered
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_UnRegisterCallback(I2S_HandleTypeDef *hi2s, HAL_I2S_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hi2s);

    if (HAL_I2S_STATE_READY == hi2s->State)
    {
        switch (CallbackID)
        {
            case HAL_I2S_TXFIFO_ENOUGH_CB_ID :
                hi2s->TxFifoEnoughCallback = HAL_I2S_TxFifoEnoughCallback;                /* Legacy weak TxCpltCallback       */
                break;

            case HAL_I2S_RXFIFO_ENOUGH_CB_ID :
                hi2s->RxFifoEnoughCallback = HAL_I2S_RxFifoEnoughCallback;                /* Legacy weak RxCpltCallback       */
                break;

            case HAL_I2S_RXFIFO_OVERFLOW_CB_ID :
                hi2s->RxFifoOvfCallback = HAL_I2S_RxFifoOvfCallback;        /* Legacy weak TxHalfCpltCallback   */
                break;

            case HAL_I2S_TXFIFO_UDR_ID :
                hi2s->TxFifoUdrCallback = HAL_I2S_TxFifoUdrCallback;        /* Legacy weak RxHalfCpltCallback   */
                break;

            case HAL_I2S_FRAME_ERR_CB_ID :
                hi2s->FrameErrCallback = HAL_I2S_FrameErrCallback;                  /* Legacy weak ErrorCallback        */
                break;

            case HAL_I2S_MSPINIT_CB_ID :
                hi2s->MspInitCallback = HAL_I2S_MspInit;                      /* Legacy weak MspInit              */
                break;

            case HAL_I2S_MSPDEINIT_CB_ID :
                hi2s->MspDeInitCallback = HAL_I2S_MspDeInit;                  /* Legacy weak MspDeInit            */
                break;

            default :
                /* Update the error code */
                SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_INVALID_CALLBACK);

                /* Return error status */
                status =  HAL_ERROR;
                break;
      }
    }
    else if (HAL_I2S_STATE_RESET == hi2s->State)
    {
        switch (CallbackID)
        {
            case HAL_I2S_MSPINIT_CB_ID :
                hi2s->MspInitCallback = HAL_I2S_MspInit;                      /* Legacy weak MspInit              */
                break;

            case HAL_I2S_MSPDEINIT_CB_ID :
                hi2s->MspDeInitCallback = HAL_I2S_MspDeInit;                  /* Legacy weak MspDeInit            */
                break;

            default :
                /* Update the error code */
                SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_INVALID_CALLBACK);

                /* Return error status */
                status =  HAL_ERROR;
                break;
        }
    }
    else
    {
        /* Update the error code */
        SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hi2s);
    return status;
}
#endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
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
        (++) HAL_I2S_Transmit()
        (++) HAL_I2S_Receive()

    (#) No-Blocking mode functions with Interrupt are :
        (++) HAL_I2S_Transmit_IT()
        (++) HAL_I2S_Receive_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_I2S_Transmit_DMA()
        (++) HAL_I2S_Receive_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) HAL_I2S_TxCpltCallback()
        (++) HAL_I2S_RxCpltCallback()
        (++) HAL_I2S_ErrorCallback()

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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size, uint32_t Timeout)
{
    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

  /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State != HAL_I2S_STATE_READY)
    {
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }

    /* Set state and reset error code */
    hi2s->State = HAL_I2S_STATE_BUSY_TX;
    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->pTxBuffPtr = pData;


    hi2s->TxXferSize = Size;
    hi2s->TxXferCount = Size;

    /* Wait until TXE flag is set */
    if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_TXE, SET, Timeout) != HAL_OK)
    {
        /* Set the error code */
        SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_TIMEOUT);
        hi2s->State = HAL_I2S_STATE_READY;
        __HAL_UNLOCK(hi2s);
        return HAL_ERROR;
    }

    while (hi2s->TxXferCount > 0U)
    {
        hi2s->Instance->WR = (*hi2s->pTxBuffPtr);
        hi2s->pTxBuffPtr++;
        hi2s->TxXferCount--;
        /* Wait until TXE flag is set */
        if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_TXE, SET, Timeout) != HAL_OK)
        {
            /* Set the error code */
            SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_TIMEOUT);
            hi2s->State = HAL_I2S_STATE_READY;
            __HAL_UNLOCK(hi2s);
            return HAL_ERROR;
        }

        /* Check if an underrun occurs */
        if (__HAL_I2S_GET_ISR_FLAG(hi2s, I2S_FLAG_UDR) == SET)
        {
            /* Set the error code */
            SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_UDR);
        }
    }

    hi2s->State = HAL_I2S_STATE_READY;
    __HAL_UNLOCK(hi2s);
    return HAL_OK;
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size, uint32_t Timeout)
{

    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State != HAL_I2S_STATE_READY)
    {
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }

    /* Set state and reset error code */
    hi2s->State = HAL_I2S_STATE_BUSY_RX;
    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->pRxBuffPtr = pData;
    
    hi2s->RxXferSize = Size;
    hi2s->RxXferCount = Size;


    /* Receive data */
    while (hi2s->RxXferCount > 0U)
    {
        /* Wait until RXNE flag is set */
        if (I2S_WaitFlagStateUntilTimeout(hi2s, I2S_FLAG_RXE, RESET, Timeout) != HAL_OK)
        {
            /* Set the error code */
            SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_TIMEOUT);
            hi2s->State = HAL_I2S_STATE_READY;
            __HAL_UNLOCK(hi2s);
            return HAL_ERROR;
        }

        (*hi2s->pRxBuffPtr) = hi2s->Instance->RD;
        hi2s->pRxBuffPtr++;
        hi2s->RxXferCount--;


    }

    hi2s->State = HAL_I2S_STATE_READY;
    __HAL_UNLOCK(hi2s);
    return HAL_OK;
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size)
{

    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State != HAL_I2S_STATE_READY)
    {
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }

    /* Set state and reset error code */
    hi2s->State = HAL_I2S_STATE_BUSY_TX;
    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->pTxBuffPtr = pData;

    hi2s->TxXferSize = Size;
    hi2s->TxXferCount = Size;

    
    /* Enable TXE and ERR interrupt */
    __HAL_I2S_ENABLE_IT(hi2s, (I2S_IT_TXSPACE_ENOUGH));

		__HAL_I2S_ENABLE_GCR_IT(hi2s);
    __HAL_UNLOCK(hi2s);
    
    return HAL_OK;
    
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size)
{

    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

    if (hi2s->State != HAL_I2S_STATE_READY)
    {
        __HAL_UNLOCK(hi2s);
        return HAL_BUSY;
    }

    /* Set state and reset error code */
    hi2s->State = HAL_I2S_STATE_BUSY_RX;
    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->pRxBuffPtr = pData;


    hi2s->RxXferSize = Size;
    hi2s->RxXferCount = Size;


    /* Enable RXNE and ERR interrupt */
    __HAL_I2S_ENABLE_IT(hi2s, (I2S_IT_RXFIFO_ENOUGH));

    __HAL_I2S_ENABLE_GCR_IT(hi2s);
    __HAL_UNLOCK(hi2s);
    
    return HAL_OK;
    
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size)
{

    
    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

//    if (hi2s->State != HAL_I2S_STATE_READY)
//    {
//        __HAL_UNLOCK(hi2s);
//        return HAL_BUSY;
//    }

    /* Set state and reset error code */
    hi2s->State = HAL_I2S_STATE_BUSY_TX;
    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->pTxBuffPtr = pData;


    hi2s->TxXferSize = Size;
    hi2s->TxXferCount = Size;

    /* Set the I2S Tx DMA transfer complete callback */
    hi2s->hdmatx->XferTfrCallback = I2S_DMATxCplt;

    /* Set the DMA error callback */
    hi2s->hdmatx->XferErrorCallback = I2S_DMAError;

    /* Enable the Tx DMA Stream/Channel */
    if (HAL_OK != HAL_DMA_Start_IT(hi2s->hdmatx,
                                   (uint32_t)hi2s->pTxBuffPtr,
                                   (uint32_t)&hi2s->Instance->WR,
                                   hi2s->TxXferSize))
    {
        /* Update SPI error code */
        SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_DMA);
        hi2s->State = HAL_I2S_STATE_READY;

        __HAL_UNLOCK(hi2s);
        return HAL_ERROR;
    }

    __HAL_UNLOCK(hi2s);
    
    return HAL_OK;
    
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint32_t *pData, uint16_t Size)
{


    if ((pData == NULL) || (Size == 0U))
    {
        return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2s);

//    if (hi2s->State != HAL_I2S_STATE_READY)
//    {
//        __HAL_UNLOCK(hi2s);
//        return HAL_BUSY;
//    }

    /* Set state and reset error code */
    hi2s->State = HAL_I2S_STATE_BUSY_RX;
    hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
    hi2s->pRxBuffPtr = pData;

    hi2s->RxXferSize = Size;
    hi2s->RxXferCount = Size;
  
    /* Set the I2S Rx DMA transfer complete callback */
    hi2s->hdmarx->XferTfrCallback = I2S_DMARxCplt;

    /* Set the DMA error callback */
    hi2s->hdmarx->XferErrorCallback = I2S_DMAError;

    /* Enable the Rx DMA Stream/Channel */
    if (HAL_OK != HAL_DMA_Start_IT(hi2s->hdmarx, (uint32_t)&hi2s->Instance->RD, (uint32_t)hi2s->pRxBuffPtr,
                                   hi2s->RxXferSize))
    {
        /* Update SPI error code */
        SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_DMA);
        hi2s->State = HAL_I2S_STATE_READY;

        __HAL_UNLOCK(hi2s);
        return HAL_ERROR;
    }

    __HAL_UNLOCK(hi2s);
    
    return HAL_OK;
    
    
}


/**
  * @brief  Get tx fifo level
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval tx fifo level
  */
uint8_t HAL_I2S_GetTxFifoLevel(I2S_HandleTypeDef *hi2s)
{

    return ((hi2s->Instance->CSR & I2S_CSR_TXFIFO_LEVEL_Msk)>>I2S_CSR_TXFIFO_LEVEL_Pos);

}

/**
  * @brief  Get rx fifo level
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval rx fifo level
  */
uint8_t HAL_I2S_GetRxFifoLevel(I2S_HandleTypeDef *hi2s)
{

    return ((hi2s->Instance->CSR & I2S_CSR_RXFIFO_LEVEL_Msk)>>I2S_CSR_RXFIFO_LEVEL_Pos);

}

/**
  * @brief  DMA I2S transmit process complete callback
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void I2S_DMATxCplt(DMA_HandleTypeDef *hdma)
{
    I2S_HandleTypeDef *hi2s = (I2S_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

    hi2s->TxXferCount = 0U;
    hi2s->State = HAL_I2S_STATE_READY;
    /* Change the DMA state */
    hdma->State = HAL_DMA_STATE_READY;
    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    HAL_I2S_TxCpltCallback(hi2s);

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

    hi2s->RxXferCount = 0U;
    hi2s->State = HAL_I2S_STATE_READY;
    /* Change the DMA state */
    hdma->State = HAL_DMA_STATE_READY;
    /* Process Unlocked */
    __HAL_UNLOCK(hdma);
    
    HAL_I2S_RxCpltCallback(hi2s);

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

    hi2s->TxXferCount = 0U;
    hi2s->RxXferCount = 0U;

    hi2s->State = HAL_I2S_STATE_READY;

    /* Set the error code and execute error callback*/
    SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_DMA);

    HAL_I2S_ErrorCallback(hi2s);

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
    hi2s->Instance->WR = (*hi2s->pTxBuffPtr);
    hi2s->pTxBuffPtr++;
    hi2s->TxXferCount--;

    if (hi2s->TxXferCount == 0U)
    {
        /* Disable TXE and ERR interrupt */
        __HAL_I2S_DISABLE_IT(hi2s, (I2S_IT_TXSPACE_ENOUGH));

        hi2s->State = HAL_I2S_STATE_READY;
        /* Call user Tx complete callback */
    #if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
        hi2s->TxFifoEnoughCallback(hi2s);
    #else
        HAL_I2S_TxFifoEnoughCallback(hi2s);
    #endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
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
    (*hi2s->pRxBuffPtr) = (uint16_t)hi2s->Instance->RD;
    hi2s->pRxBuffPtr++;
    hi2s->RxXferCount--;

    if (hi2s->RxXferCount == 0U)
    {
        /* Disable RXNE and ERR interrupt */
        __HAL_I2S_DISABLE_IT(hi2s, (I2S_IT_RXFIFO_ENOUGH));

        hi2s->State = HAL_I2S_STATE_READY;
        /* Call user Rx complete callback */
    #if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
        hi2s->RxFifoEnoughCallback(hi2s);
    #else
        HAL_I2S_RxFifoEnoughCallback(hi2s);
    #endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
    }
}
/**
  * @brief  This function handles I2S interrupt request.
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s)
{
    /* Call the IrqHandler ISR set during HAL_I2S_INIT */
    hi2s->IrqHandlerISR(hi2s);
}



/**
  * @brief  Tx Transfer Half completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_TxHalfCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Tx Transfer completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_TxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Rx Transfer half completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_RxHalfCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Rx Transfer completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_RxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  I2S error callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_ErrorCallback could be implemented in the user file
     */
}

/**
  * @brief  Tx Transfer Half completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_TxFifoEnoughCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_TxHalfCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Tx Transfer completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_RxFifoEnoughCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_TxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Rx Transfer half completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_RxFifoOvfCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_RxHalfCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  Rx Transfer completed callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_TxFifoUdrCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_RxCpltCallback could be implemented in the user file
     */
}

/**
  * @brief  I2S error callbacks
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
__weak void HAL_I2S_FrameErrCallback(I2S_HandleTypeDef *hi2s)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hi2s);

    /* NOTE : This function Should not be modified, when the callback is needed,
              the HAL_I2S_ErrorCallback could be implemented in the user file
     */
}



/**
  * @brief  This function handles I2S interrupt request.
  * @param  hi2s: pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval None
  */
static void I2S_ISRHandler(I2S_HandleTypeDef *hi2s)
{
    __IO uint32_t i2ssr = hi2s->Instance->ISR;


    if (hi2s->State == HAL_I2S_STATE_BUSY_RX)
    {
        /* I2S in mode Receiver ------------------------------------------------*/
        if (((i2ssr & I2S_FLAG_RXFIFO_ENOUGH) == I2S_FLAG_RXFIFO_ENOUGH) && (__HAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_RXFIFO_ENOUGH) != RESET))
        {
            __HAL_I2S_CLR_IT(hi2s, RX_INTCLR);
            
            I2S_Receive_IT(hi2s);
        }

        /* I2S Overrun error interrupt occurred -------------------------------------*/
        if (((i2ssr & I2S_FLAG_RX_OVF) == I2S_FLAG_RX_OVF) && (__HAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_RX_OVF) != RESET))
        {
            /* Disable RXNE and ERR interrupt */
            __HAL_I2S_DISABLE_IT(hi2s, (I2S_IT_RXFIFO_ENOUGH | I2S_IT_RX_OVF));

            /* Set the I2S State ready */
            hi2s->State = HAL_I2S_STATE_READY;


            /* Set the error code and execute error callback*/
            SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_OVR);
            /* Call user error callback */
      #if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
            hi2s->RxFifoOvfCallback(hi2s);
      #else
            HAL_I2S_RxFifoOvfCallback(hi2s);
      #endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
        }
    }

    if (hi2s->State == HAL_I2S_STATE_BUSY_TX)
    {
        /* I2S in mode Transmitter -----------------------------------------------*/
        if (((i2ssr & I2S_FLAG_TXE) == I2S_FLAG_TXSPACE_ENOUGH) && (__HAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_TXSPACE_ENOUGH) != RESET))
        {
          __HAL_I2S_CLR_IT(hi2s, TX_INTCLR);  
            
          I2S_Transmit_IT(hi2s);
        }

        /* I2S Underrun error interrupt occurred --------------------------------*/
        if (((i2ssr & I2S_FLAG_UDR) == I2S_FLAG_UDR) && (__HAL_I2S_GET_IT_SOURCE(hi2s, I2S_IT_UDR) != RESET))
        {
            /* Disable TXE and ERR interrupt */
            __HAL_I2S_DISABLE_IT(hi2s, (I2S_IT_TXSPACE_ENOUGH | I2S_IT_UDR));

            /* Set the I2S State ready */
            hi2s->State = HAL_I2S_STATE_READY;

            /* Set the error code and execute error callback*/
            SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_UDR);
            /* Call user error callback */
      #if (USE_HAL_I2S_REGISTER_CALLBACKS == 1U)
            hi2s->TxFifoUdrCallback(hi2s);
      #else
            HAL_I2S_TxFifoUdrCallback(hi2s);
      #endif /* USE_HAL_I2S_REGISTER_CALLBACKS */
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
  * @retval HAL status
  */
static HAL_StatusTypeDef I2S_WaitFlagStateUntilTimeout(I2S_HandleTypeDef *hi2s, uint32_t Flag, FlagStatus State,
                                                       uint32_t Timeout)
{
    uint32_t tickstart;

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Wait until flag is set to status*/
    while (((__HAL_I2S_GET_CSR_FLAG(hi2s, Flag)) ? SET : RESET) != State)
    {
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tickstart) >= Timeout) || (Timeout == 0U))
            {
                /* Set the I2S State ready */
                hi2s->State = HAL_I2S_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hi2s);

                return HAL_TIMEOUT;
            }
        }
    }
    return HAL_OK;
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
  * @retval HAL state
  */
HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s)
{
  return hi2s->State;
}

/**
  * @brief  Return the I2S error code
  * @param  hi2s pointer to a I2S_HandleTypeDef structure that contains
  *         the configuration information for I2S module
  * @retval I2S Error Code
  */
uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s)
{
  return hi2s->ErrorCode;
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
  

#endif 

/**
  * @}
  */ 
  
  
  
  
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
