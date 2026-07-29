/**
  ******************************************************************************
  * @file    um324xx_hal_spi2.c
  * @author  MCU Team
  * @version V1.00
  * @date    10-February-2023
  * @brief   SPI2 HAL module driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2023 Unicmicro Co.,Ltd.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup UM324xx_HAL_Driver
  * @{
  */


/** @defgroup SPI2 SPI2
  * @brief SPI2 HAL module driver
  * @{
  */
#ifdef HAL_SPI_EX_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define SPI_DEFAULT_TIMEOUT 100U
#define SPI_BSY_FLAG_WORKAROUND_TIMEOUT 1000U                           /*!< Timeout 1000 ��s  */

/** @defgroup SPI_Private_Defines SPI Private Defines
  * @{
  */


/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/** @defgroup SPI_Private_Macros SPI Private Macros
  * @{
  */
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
uint32_t temp_data;
/* Private function prototypes -----------------------------------------------*/

static void SPI_EX_DMATransmitCplt(DMA_HandleTypeDef *hdma);
static void SPI_EX_DMAReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_EX_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma);
static void SPI_EX_DMAError(DMA_HandleTypeDef *hdma);
static void SPI_EX_DMATxAbortCallback(DMA_HandleTypeDef *hdma);
static void SPI_EX_DMARxAbortCallback(DMA_HandleTypeDef *hdma);
static void SPI_EX_DMAAbortOnError(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef SPI_EX_WaitFlagStateUntilTimeout(SPI_EX_HandleTypeDef *hspi, uint32_t Flag, FlagStatus State,
        uint32_t Timeout, uint32_t Tickstart);

static void SPI_EX_TxISR_8BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_TxISR_16BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_TxISR_32BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_RxISR_8BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_RxISR_16BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_RxISR_32BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_2linesRxISR_8BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_2linesTxISR_8BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_2linesTxISR_16BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_2linesRxISR_16BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_2linesTxISR_32BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_2linesRxISR_32BIT(struct __SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_AbortRx_ISR(SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_AbortTx_ISR(SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_CloseRxTx_ISR(SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_CloseRx_ISR(SPI_EX_HandleTypeDef *hspi);
static void SPI_EX_CloseTx_ISR(SPI_EX_HandleTypeDef *hspi);
static HAL_StatusTypeDef SPI_EX_EndRxTransaction(SPI_EX_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef SPI_EX_EndRxTxTransaction(SPI_EX_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart);

/* Exported functions --------------------------------------------------------*/
#if defined ( __GNUC__ )
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
/**
  * @brief
  *
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
HAL_StatusTypeDef HAL_SPI_EX_Init(SPI_EX_HandleTypeDef *hspi)
{
    __IO uint32_t tmpregister = 0U;

    /* Check the SPI handle allocation */
    if (hspi == NULL)
    {
        return HAL_ERROR;
    }


    if (hspi->State == HAL_SPI_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        hspi->Lock = HAL_UNLOCKED;

#if (USE_HAL_SPI_EX_REGISTER_CALLBACKS == 1U)
        /* Init the SPI Callback settings */
        hspi->TxCpltCallback       = HAL_SPI_EX_TxCpltCallback;       /* Legacy weak TxCpltCallback       */
        hspi->RxCpltCallback       = HAL_SPI_EX_RxCpltCallback;       /* Legacy weak RxCpltCallback       */
        hspi->TxRxCpltCallback     = HAL_SPI_EX_TxRxCpltCallback;     /* Legacy weak TxRxCpltCallback     */
        hspi->ErrorCallback        = HAL_SPI_EX_ErrorCallback;        /* Legacy weak ErrorCallback        */
        hspi->AbortCpltCallback    = HAL_SPI_EX_AbortCpltCallback;    /* Legacy weak AbortCpltCallback    */

        if (hspi->MspInitCallback == NULL)
        {
            hspi->MspInitCallback = HAL_SPI_EX_MspInit;                 /* Legacy weak MspInit  */
        }

        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        hspi->MspInitCallback(hspi);
#else
        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        HAL_SPI_EX_MspInit(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }

    hspi->State = HAL_SPI_STATE_BUSY;

    /* Disable the selected SPI peripheral */
    __HAL_SPI_EX_DISABLE(hspi);

    /*----------------------- SPIx GCTL & CCTL Configuration ---------------------*/
    /* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
    Communication speed, First bit, Data transmission phase, Data splicing, Length of SPI character,
    Direction for the half mode, Chip selection signal controller, DMA access mode,
     Number of vacancies that can trigger the TXFIFO /RXFIFO,  Enable receive/send data and Spi interrupt master switch*/

    /* Baudrate prescaler not use in Slave mode.*/
    if (hspi->Init.Mode == SPI_EX_MODE_MASTER)
    {
        if(hspi->Init.BaudRatePrescaler != 0U)
        {
            hspi->Instance->SPBRG = hspi->Init.BaudRatePrescaler;
        }
        else
        {
            /* Restore default values */
            hspi->Instance->SPBRG = SPI_EX_SPBRG_SPBRG_DEFAULT;
        }
    }

    /* Check whether half duplex is turned on */
    if((hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE))
    {
        tmpregister |= SPI_EX_GCTL_BIDIRMODE;
    }

    /* Check whether the receive enable is turned on */
    if((hspi->Init.Direction_Mode & SPI_EX_GCTL_RXEN) && ((hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE) ==0))
    {
        tmpregister |=SPI_EX_GCTL_RXEN;
    }

    if(((hspi->Init.Mode ==SPI_EX_MODE_MASTER) && ((hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_RECEIVE))))
    {
        tmpregister &=~SPI_EX_GCTL_RXEN;
    }
    /* Check whether the send enable is turned on */
    if(((hspi->Init.Direction_Mode & SPI_EX_GCTL_TXEN) && ((hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE) ==0)) || \
            ((hspi->Init.Direction_Mode & SPI_EX_GCTL_TXEN) && hspi->Init.Mode == SPI_EX_MODE_MASTER))
    {
        tmpregister |=SPI_EX_GCTL_TXEN;
    }

    tmpregister = tmpregister | ((uint32_t)((hspi->Init.Mode & (SPI_EX_GCTL_MM)) |(hspi->Init.TXTLF & SPI_EX_GCTL_TXTLF) \
                                            | (hspi->Init.RXTLF & SPI_EX_GCTL_RXTLF) | (hspi->Init.NSS & SPI_EX_GCTL_CSN_SEL) \
                                            | (hspi->Init.DMA_Mode & SPI_EX_GCTL_DMAMODE)));

    /* Configurated the default values */
    hspi->Instance->CCTL = 0x700U;
    hspi->Instance->GCTL = 0x00U;

    /* Clear interrupt status */
    hspi->Instance->INTCLR = 0xFFU;

    WRITE_REG(hspi->Instance->GCTL,tmpregister);

    WRITE_REG(hspi->Instance->CCTL, ((hspi->Init.RX_Stitch & (SPI_EX_CCTL_RX_STITCH)) |
                                     (hspi->Init.TX_Stitch & (SPI_EX_CCTL_TX_STITCH)) |
                                     (hspi->Init.FirstBit & (SPI_EX_CCTL_LSBFE)) |
                                     (hspi->Init.SPI_Len & (SPI_EX_CCTL_SPILEN)) |
                                     (hspi->Init.TXEDGE & (SPI_EX_CCTL_TXEDGE)) |
                                     (hspi->Init.CLKPhase & (SPI_EX_CCTL_CKPH)) |
                                     (hspi->Init.CLKPolarity & (SPI_EX_CCTL_CKPL)) |
                                     (hspi->Init.TIMode & SPI_EX_CCTL_TI_MOD) |
                                     (hspi->Init.RXEDGE & SPI_EX_CCTL_RXEDGE)));


    /* Initialize the SPI state */
    hspi->State     = HAL_SPI_STATE_READY ;
    hspi->ErrorCode = HAL_SPI_EX_ERROR_NONE;


    return HAL_OK;
}



/**
  * @}
  */

/**
  * @brief  De-Initialize the SPI peripheral.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_DeInit(SPI_EX_HandleTypeDef *hspi)
{
    /* Check the SPI handle allocation */
    if (hspi == NULL)
    {
        return HAL_ERROR;
    }

    hspi->State = HAL_SPI_STATE_BUSY;

    /* Disable the SPI Peripheral Clock */
    __HAL_SPI_EX_DISABLE(hspi);

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    if (hspi->MspDeInitCallback == NULL)
    {
        hspi->MspDeInitCallback = HAL_SPI_MspDeInit; /* Legacy weak MspDeInit  */
    }

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    hspi->MspDeInitCallback(hspi);
#else
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    HAL_SPI_EX_MspDeInit(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

    hspi->ErrorCode = HAL_SPI_EX_ERROR_NONE;
    hspi->State = HAL_SPI_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

/**
  * @brief  Initialize the SPI MSP.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_EX_MspInit(SPI_EX_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_SPI_MspInit should be implemented in the user file
    */
}


/**
  * @brief  De-Initialize the SPI MSP.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_EX_MspDeInit(SPI_EX_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_SPI_MspDeInit should be implemented in the user file
    */
}


#if (USE_HAL_SPI_EX_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User SPI Callback
  *         To be used instead of the weak predefined callback
  * @param  hspi        Pointer to a SPI_HandleTypeDef structure that contains
  *                         the configuration information for the specified SPI.
  * @param  CallbackID  ID of the callback to be registered
  * @param  pCallback   pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_RegisterCallback(SPI_EX_HandleTypeDef *hspi, HAL_SPI_CallbackIDTypeDef CallbackID,
        pSPI_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        /* Update the error code */
        hspi->ErrorCode |= HAL_SPI_ERROR_INVALID_CALLBACK;

        return HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hspi);

    if (HAL_SPI_STATE_READY == hspi->State)
    {
        switch (CallbackID)
        {
        case HAL_SPI_TX_COMPLETE_CB_ID :
            hspi->TxCpltCallback = pCallback;
            break;

        case HAL_SPI_RX_COMPLETE_CB_ID :
            hspi->RxCpltCallback = pCallback;
            break;

        case HAL_SPI_TX_RX_COMPLETE_CB_ID :
            hspi->TxRxCpltCallback = pCallback;
            break;

        case HAL_SPI_ERROR_CB_ID :
            hspi->ErrorCallback = pCallback;
            break;

        case HAL_SPI_ABORT_CB_ID :
            hspi->AbortCpltCallback = pCallback;
            break;

        case HAL_SPI_MSPINIT_CB_ID :
            hspi->MspInitCallback = pCallback;
            break;

        case HAL_SPI_MSPDEINIT_CB_ID :
            hspi->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_INVALID_CALLBACK);

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (HAL_SPI_STATE_RESET == hspi->State)
    {
        switch (CallbackID)
        {
        case HAL_SPI_MSPINIT_CB_ID :
            hspi->MspInitCallback = pCallback;
            break;

        case HAL_SPI_MSPDEINIT_CB_ID :
            hspi->MspDeInitCallback = pCallback;
            break;

        default :
            /* Update the error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_INVALID_CALLBACK);

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Update the error code */
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hspi);

    return status;
}


/**
  * @brief  Unregister an SPI Callback
  *         SPI callback is redirected to the weak predefined callback
  * @param  hspi        Pointer to a SPI_HandleTypeDef structure that contains
  *                         the configuration information for the specified SPI.
  * @param  CallbackID  ID of the callback to be unregistered
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_UnRegisterCallback(SPI_EX_HandleTypeDef *hspi, HAL_SPI_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hspi);

    if (HAL_SPI_STATE_READY == hspi->State)
    {
        switch (CallbackID)
        {
        case HAL_SPI_TX_COMPLETE_CB_ID :
            hspi->TxCpltCallback = HAL_SPI_TxCpltCallback;             /* Legacy weak TxCpltCallback       */
            break;

        case HAL_SPI_RX_COMPLETE_CB_ID :
            hspi->RxCpltCallback = HAL_SPI_RxCpltCallback;             /* Legacy weak RxCpltCallback       */
            break;

        case HAL_SPI_TX_RX_COMPLETE_CB_ID :
            hspi->TxRxCpltCallback = HAL_SPI_EX_TxRxCpltCallback;         /* Legacy weak TxRxCpltCallback     */
            break;

        case HAL_SPI_ERROR_CB_ID :
            hspi->ErrorCallback = HAL_SPI_ErrorCallback;               /* Legacy weak ErrorCallback        */
            break;

        case HAL_SPI_ABORT_CB_ID :
            hspi->AbortCpltCallback = HAL_SPI_AbortCpltCallback;       /* Legacy weak AbortCpltCallback    */
            break;

        case HAL_SPI_MSPINIT_CB_ID :
            hspi->MspInitCallback = HAL_SPI_MspInit;                   /* Legacy weak MspInit              */
            break;

        case HAL_SPI_MSPDEINIT_CB_ID :
            hspi->MspDeInitCallback = HAL_SPI_MspDeInit;               /* Legacy weak MspDeInit            */
            break;

        default :
            /* Update the error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_INVALID_CALLBACK);

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }

    else if (HAL_SPI_STATE_RESET == hspi->State)
    {
        switch (CallbackID)
        {
        case HAL_SPI_MSPINIT_CB_ID :
            hspi->MspInitCallback = HAL_SPI_MspInit;                   /* Legacy weak MspInit              */
            break;

        case HAL_SPI_MSPDEINIT_CB_ID :
            hspi->MspDeInitCallback = HAL_SPI_MspDeInit;               /* Legacy weak MspDeInit            */
            break;

        default :
            /* Update the error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_INVALID_CALLBACK);

            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }

    else
    {
        /* Update the error code */
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_INVALID_CALLBACK);

        /* Return error status */
        status =  HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(hspi);

    return status;
}

#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */




/** @defgroup SPI_Exported_Functions_Group2 IO operation functions
 *  @brief SPI Read, Write, Toggle, Lock and EXTI management functions.
 *
@verbatim
 ===============================================================================
                       ##### IO operation functions #####
 ===============================================================================

@endverbatim
  * @{
  */



/**
  * @brief    Send an amount of data in blocking mode.
  *
  * @param hspi       A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @param pData      Pointer to the address to send data
  * @param Size       The amount of data to be sent
  * @param Timeout    Timeout period for waiting for sending to complete
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT transmit timeout
  *
  */
HAL_StatusTypeDef HAL_SPI_EX_Transmit(SPI_EX_HandleTypeDef *hspi, uint32_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart;
    HAL_StatusTypeDef errorcode = HAL_OK;
    uint16_t initial_TxXferCount;

    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    initial_TxXferCount = Size;
    initial_TxXferCount =initial_TxXferCount;

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    /* If the data is empty or the data size is 0, an error is returned*/
    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->pRxBuffPtr  = (uint8_t *)NULL;
    hspi->RxXferSize  = 0U;
    hspi->RxXferCount = 0U;
    hspi->TxISR       = NULL;
    hspi->RxISR       = NULL;

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    /* Transmit data */
    while (hspi->TxXferCount > 0U)
    {
        if((hspi->Init.Mode == SPI_EX_MODE_SLAVE) && (hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_SEND))
        {
            __HAL_SPI_EX_SETTXDNR(hspi, hspi->TxXferSize);
        }

        /* Configure communication direction : 1Line in HALF-DUPLEX MODE */
        if (hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE)
        {
            /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
            __HAL_SPI_EX_DISABLE(hspi);
            SPI_EX_1LINE_TX(hspi);
            SPI_EX_TX_ENABLE(hspi);
            SPI_EX_RX_DISABLE(hspi);
            /* Enable SPI peripheral */
            __HAL_SPI_EX_ENABLE(hspi);
        }

        /* If it is necessary for the slave to send simplex. */
        if((hspi->Init.Mode == SPI_EX_MODE_SLAVE) && (hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_SEND))
        {
            while(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_CSTAT_TXFULL) == SET)
            {
                /* Timeout management */
                if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
                {
                    errorcode = HAL_TIMEOUT;
                    goto error;
                }
            }
        }

        /* Wait until TXE flag is set to send data */
        if(hspi->Init.DataSize == SPI_EX_DATASIZE_32BIT)
        {
            if(hspi->Init.Mode == SPI_EX_MODE_MASTER)
            {
                if (!(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXFULL)))
                {
                    hspi->Instance->TXREG = *((uint32_t *)hspi->pTxBuffPtr);
                    hspi->pTxBuffPtr += sizeof(uint32_t);
                    hspi->TxXferCount--;
                }
            }
            else
            {
                hspi->Instance->TXREG = *((uint32_t *)hspi->pTxBuffPtr);
                hspi->pTxBuffPtr += sizeof(uint32_t);
                hspi->TxXferCount--;
            }

        }
        else if(hspi->Init.DataSize == SPI_EX_DATASIZE_16BIT)
        {
            if(hspi->Init.Mode == SPI_EX_MODE_MASTER)
            {
                if (!(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXFULL)))
                {
                    hspi->Instance->TXREG = *((uint16_t *)hspi->pTxBuffPtr);
                    hspi->pTxBuffPtr += sizeof(uint16_t);
                    hspi->TxXferCount--;
                }
            }
            else
            {
                hspi->Instance->TXREG = *((uint16_t *)hspi->pTxBuffPtr);
                hspi->pTxBuffPtr += sizeof(uint16_t);
                hspi->TxXferCount--;
            }
        }
        else
        {
            if(hspi->Init.Mode == SPI_EX_MODE_MASTER)
            {
                while((__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXFULL)))
                {
                    /* Timeout management */
                    if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
                    {
                        errorcode = HAL_TIMEOUT;
                        goto error;
                    }
                }
                hspi->Instance->TXREG = *((uint8_t *)hspi->pTxBuffPtr);
                hspi->pTxBuffPtr += sizeof(uint8_t);
                hspi->TxXferCount--;
            }
            else
            {

                hspi->Instance->TXREG = *((uint8_t *)hspi->pTxBuffPtr);
                hspi->pTxBuffPtr += sizeof(uint8_t);
                hspi->TxXferCount--;
            }
        }

        if((hspi->Init.Mode !=SPI_EX_MODE_SLAVE) || (hspi->Init.Direction_Mode !=SPI_EX_SINGLE_DUPLEX_SEND))
        {
            /* Wait until Transmitter FIFO and "transmit shift register" are empty. */
            while(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXEPT) == RESET)
            {
                /* Timeout management */
                if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
                {
                    errorcode = HAL_TIMEOUT;
                    goto error;
                }
            }
        }


        if((hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE) && (hspi->Init.Mode == SPI_EX_MODE_SLAVE))
        {
            SPI_EX_TX_DISABLE(hspi);
        }
    }

    /* Check the end of the transaction */
    if (SPI_EX_EndRxTxTransaction(hspi, Timeout, tickstart) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_EX_ERROR_FLAG;
    }

    /* Clear overrun flag*/
    __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_RXOERR);

    if (hspi->ErrorCode != HAL_SPI_EX_ERROR_NONE)
    {
        errorcode = HAL_ERROR;
    }

error:
    hspi->State = HAL_SPI_STATE_READY;
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;

}


/**
  * @brief    Receive an amount of data in blocking mode.
  *
  * @param hspi      A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @param pData     A pointer to the address to receive data
  * @param Size      The amount of data to be received
  * @param Timeout   Wait to receive timeout
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT receive timeout
  */
HAL_StatusTypeDef HAL_SPI_EX_Receive(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t tickstart;
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_RX;
    hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->pTxBuffPtr  = (uint8_t *)NULL;
    hspi->TxXferSize  = 0U;
    hspi->TxXferCount = 0U;
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;

    /* When in the host simplex receiving mode */
    if((hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_RECEIVE) && (hspi->Init.Mode == SPI_EX_MODE_MASTER))
    {
        hspi->Instance->GCTL &=~(SPI_EX_GCTL_TXEN);
        hspi->Instance->GCTL |= SPI_EX_GCTL_RXEN;
    }

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    /* Transfer loop */
    while (hspi->RxXferCount > 0U)
    {
        if((hspi->Init.Mode == SPI_EX_MODE_MASTER) && (hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_RECEIVE))
        {
            __HAL_SPI_EX_SETRXDNR(hspi,hspi->RxXferSize);
            hspi->Instance->GCTL |= SPI_EX_GCTL_RXEN;
        }

        if((hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE))
        {

            SPI_EX_1LINE_RX(hspi);
            SPI_EX_TX_DISABLE(hspi);
            SPI_EX_RX_ENABLE(hspi);
            /* Enable SPI peripheral */
            __HAL_SPI_EX_ENABLE(hspi);
        }
        if(hspi->Init.Mode == SPI_EX_MODE_MASTER && hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX)
        {
            /* it's used to provided clock */
            SPI_EX_TX_DISABLE(hspi);
            hspi->Instance->TXREG =0xFF;
        }

        /* Waiting for the receive buffer to be not empty, RXBF event */
        while(!(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_RXAVL)))
        {
            /* Timeout management */
            if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
            {
                errorcode = HAL_TIMEOUT;
                goto error;
            }
        }

        if(hspi->Init.DataSize == SPI_EX_DATASIZE_32BIT)
        {
            *((uint32_t *)hspi->pRxBuffPtr) = hspi->Instance->RXREG;
            hspi->pRxBuffPtr += sizeof(uint32_t);
            hspi->RxXferCount--;
        }
        else if(hspi->Init.DataSize == SPI_EX_DATASIZE_16BIT)
        {
            *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->RXREG;
            hspi->pRxBuffPtr += sizeof(uint16_t);
            hspi->RxXferCount--;
        }
        else
        {
            /* read the received data */
            (* (uint8_t *)hspi->pRxBuffPtr) = hspi->Instance->RXREG;
            hspi->pRxBuffPtr += sizeof(uint8_t);
            hspi->RxXferCount--;
        }

        if((hspi->Init.Mode == SPI_EX_MODE_MASTER) && (hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE))
        {
            SPI_EX_RX_DISABLE(hspi);
        }
        if(hspi->Init.Mode == SPI_EX_MODE_MASTER)
        {
            SPI_EX_TX_ENABLE(hspi);
        }

    }

    if((hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_RECEIVE) && (hspi->Init.Mode == SPI_EX_MODE_MASTER))
    {
        hspi->Instance->GCTL &=~(SPI_EX_GCTL_RXEN);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_DISABLE(hspi);

    }
    /* Check the end of the transaction */
    if (SPI_EX_EndRxTransaction(hspi, Timeout, tickstart) != HAL_OK)
    {
        hspi->ErrorCode = HAL_SPI_EX_ERROR_FLAG;
    }

    if (hspi->ErrorCode != HAL_SPI_EX_ERROR_NONE)
    {
        errorcode = HAL_ERROR;
    }


error :
    hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(hspi);
    return errorcode;

}




/**
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi    pointer to a SPI_HandleTypeDef structure that contains
  *                     the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size    amount of data to be sent and received
  * @param  Timeout Timeout duration
  * @note   This function is only available in FULL DUPLEX mode
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_TransmitReceive(SPI_EX_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
        uint32_t Timeout)
{
    uint16_t             initial_TxXferCount;
    uint32_t             tmp_mode;
    HAL_SPI_EX_StateTypeDef tmp_state;
    uint32_t             tickstart;

    /* Variable used to alternate Rx and Tx during transfer */
    uint32_t             txallowed = 1U;
    HAL_StatusTypeDef    errorcode = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hspi);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* Init temporary variables */
    tmp_state           = hspi->State;
    tmp_mode            = hspi->Init.Mode;
    initial_TxXferCount = Size;
    initial_TxXferCount = initial_TxXferCount;

    if (!((tmp_state == HAL_SPI_STATE_READY) || ((tmp_mode == SPI_EX_MODE_MASTER) && (tmp_state == HAL_SPI_STATE_BUSY_RX))))
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX)
    {
        hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

    /* Set the transaction information */
    hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferCount = Size;
    hspi->RxXferSize  = Size;
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferCount = Size;
    hspi->TxXferSize  = Size;

    /*Init field not used in handle to zero */
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }
    /* Simplex mode and Half duplex mode is not suitable for this function */
    if((hspi->Init.Direction_Mode & 0x10000000U) || (hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE))
    {
        errorcode = HAL_ERROR;
        goto error;
    }
    else
    {
        while((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U))
        {
            __HAL_SPI_EX_SETTXDNR(hspi,0x1);
            __HAL_SPI_EX_SETRXDNR(hspi,0x1);
            /* Check whether the sending fifo is not full	 */
            while((!(__HAL_SPI_EX_CURRENT_STATE(hspi,SPI_EX_STATE_TXFULL))) && (hspi->TxXferCount > 0U) && (txallowed == 1U))
            {
                if(hspi->Init.DataSize == SPI_EX_DATASIZE_32BIT)
                {
                    hspi->Instance->TXREG = *((uint32_t *)hspi->pTxBuffPtr);
                    hspi->pTxBuffPtr += sizeof(uint32_t);
                }
                else if(hspi->Init.DataSize == SPI_EX_DATASIZE_16BIT)
                {
                    hspi->Instance->TXREG = *((uint16_t *)hspi->pTxBuffPtr);
                    hspi->pTxBuffPtr += sizeof(uint16_t);
                }
                else
                {
                    hspi->Instance->TXREG = *((uint8_t *)hspi->pTxBuffPtr);
                    hspi->pTxBuffPtr += sizeof(uint8_t);
                }
                hspi->TxXferCount--;
                /* Next Data is a reception (Rx). Tx not allowed */
                txallowed = 0U;
            }
            /* Wait until Transmitter FIFO and "transmit shift register" are empty */
            while(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXEPT) == RESET)
            {
                /* Timeout management */
                if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
                {
                    errorcode = HAL_TIMEOUT;
                    goto error;
                }
            }

            /* Wait for the receiving FIFO to receive data */
            while ((__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_RXAVL)) && (hspi->RxXferCount > 0U))
            {
                if(hspi->Init.DataSize == SPI_EX_DATASIZE_32BIT)
                {
                    *((uint32_t *)hspi->pRxBuffPtr) = hspi->Instance->RXREG;
                    hspi->pRxBuffPtr += sizeof(uint32_t);
                }
                else if(hspi->Init.DataSize == SPI_EX_DATASIZE_16BIT)
                {
                    *((uint16_t *)hspi->pRxBuffPtr) = hspi->Instance->RXREG;
                    hspi->pRxBuffPtr += sizeof(uint16_t);
                }
                else
                {
                    *((uint8_t *)hspi->pRxBuffPtr) = hspi->Instance->RXREG;
                    hspi->pRxBuffPtr += sizeof(uint8_t);
                }

                hspi->RxXferCount--;
                /* Next Data is a Transmission (Tx). Tx is allowed */
                txallowed = 1U;
            }

            if (((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY))
            {
                errorcode = HAL_TIMEOUT;
                goto error;
            }
        }
    }
    /* Check the end of the transaction */
    if (SPI_EX_EndRxTxTransaction(hspi, Timeout, tickstart) != HAL_OK)
    {
        errorcode = HAL_ERROR;
        hspi->ErrorCode = HAL_SPI_EX_ERROR_FLAG;
        goto error;
    }

    /* Clear overrun flag in 2 Lines communication mode because received is not read */
    __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_RXOERR);

error:
    hspi->State = HAL_SPI_STATE_READY;
    __HAL_UNLOCK(hspi);
    return errorcode;
}


/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_Transmit_IT(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hspi);

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;

    /* Init field not used in handle to zero */
    hspi->pRxBuffPtr  = (uint8_t *)NULL;
    hspi->RxXferSize  = 0U;
    hspi->RxXferCount = 0U;
    hspi->RxISR       = NULL;

    /* Set the function for IT treatment */
    if (hspi->Init.DataSize == SPI_EX_DATASIZE_32BIT)
    {
        hspi->TxISR = SPI_EX_TxISR_32BIT;
    }
    else if (hspi->Init.DataSize == SPI_EX_DATASIZE_16BIT)
    {
        hspi->TxISR = SPI_EX_TxISR_16BIT;
    }
    else
    {
        hspi->TxISR = SPI_EX_TxISR_8BIT;
    }

    /* Enable the interrupt */
    __HAL_SPI_EX_ENABLE_IT(hspi, SPI_EX_IT_TX | SPI_EX_IT_TXEPT);

    if((hspi->Init.Mode == SPI_EX_MODE_SLAVE) && (hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_SEND))
    {
        __HAL_SPI_EX_SETTXDNR(hspi,Size);
    }

    /* Check if the SPI  interrupt is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_INT_EN) != SPI_EX_GCTL_INT_EN)
    {
        /* Enable SPI interrupt */
        hspi->Instance->GCTL |= SPI_EX_GCTL_INT_EN;
    }

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    /* Configure communication direction : 1Line in HALF-DUPLEX MODE */
    if (hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE)
    {
        /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
        __HAL_SPI_EX_DISABLE(hspi);
        SPI_EX_1LINE_TX(hspi);
        SPI_EX_TX_ENABLE(hspi);
        SPI_EX_RX_DISABLE(hspi);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }


error:
    __HAL_UNLOCK(hspi);
    return errorcode;
}
/**
  * @brief  Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_Receive_IT(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hspi);

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_RX;
    hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /* Init field not used in handle to zero */
    hspi->pTxBuffPtr  = (uint8_t *)NULL;
    hspi->TxXferSize  = 0U;
    hspi->TxXferCount = 0U;
    hspi->TxISR       = NULL;

    /* Set the function for IT treatment */
    if (hspi->Init.DataSize == SPI_EX_DATASIZE_32BIT)
    {
        hspi->RxISR = SPI_EX_RxISR_32BIT;
    }
    else if (hspi->Init.DataSize == SPI_EX_DATASIZE_16BIT)
    {
        hspi->RxISR = SPI_EX_RxISR_16BIT;
    }
    else
    {
        hspi->RxISR = SPI_EX_RxISR_8BIT;
    }
    /* When in the host simplex receiving mode */
    if((hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_RECEIVE) && (hspi->Init.Mode == SPI_EX_MODE_MASTER))
    {
        hspi->Instance->GCTL &=~(SPI_EX_GCTL_TXEN);
        hspi->Instance->GCTL |= SPI_EX_GCTL_RXEN;
    }
    /* Enable  interrupt */
    __HAL_SPI_EX_ENABLE_IT(hspi, (SPI_EX_IT_RXFIFO_FULL | SPI_EX_IT_RX));

    /* Note : The SPI must be enabled after unlocking current process
                to avoid the risk of SPI interrupt handle execution before current
                process unlock */

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    if((hspi->Init.Direction_Mode & SPI_EX_GCTL_BIDIRMODE))
    {
        __HAL_SPI_EX_DISABLE(hspi);
        SPI_EX_1LINE_RX(hspi);
        SPI_EX_TX_DISABLE(hspi);
        SPI_EX_RX_ENABLE(hspi);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    if(hspi->Init.Mode == SPI_EX_MODE_MASTER && (hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX) \
            &&(hspi->Init.Direction_Mode != SPI_EX_SINGLE_DUPLEX_RECEIVE) )
    {
        /* it's used to provided clock */
        SPI_EX_TX_DISABLE(hspi);
        hspi->Instance->TXREG =0xFF;
    }

error:
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent and received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_TransmitReceive_IT(SPI_EX_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    uint32_t             tmp_mode;
    HAL_SPI_EX_StateTypeDef tmp_state;
    HAL_StatusTypeDef    errorcode = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hspi);

    /* Init temporary variables */
    tmp_state           = hspi->State;
    tmp_mode            = hspi->Init.Mode;

    if (!((tmp_state == HAL_SPI_STATE_READY) || \
            ((tmp_mode == SPI_EX_MODE_MASTER) && (tmp_state == HAL_SPI_STATE_BUSY_RX))))
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX)
    {
        hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

    /* Set the transaction information */
    hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;
    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /* Set the function for IT treatment */
    if (hspi->Init.DataSize == SPI_EX_DATASIZE_32BIT)
    {
        hspi->RxISR     = SPI_EX_2linesRxISR_32BIT;
        hspi->TxISR     = SPI_EX_2linesTxISR_32BIT;
    }
    else if (hspi->Init.DataSize == SPI_EX_DATASIZE_16BIT)
    {
        hspi->RxISR     = SPI_EX_2linesRxISR_16BIT;
        hspi->TxISR     = SPI_EX_2linesTxISR_16BIT;
    }
    else
    {
        hspi->RxISR     = SPI_EX_2linesRxISR_8BIT;
        hspi->TxISR     = SPI_EX_2linesTxISR_8BIT;
    }

    /* Enable  interrupt */
    __HAL_SPI_EX_ENABLE_IT(hspi,  SPI_EX_IT_TX | SPI_EX_IT_RX | SPI_EX_IT_UNDERRUNN | SPI_EX_IT_RXOERR);

    /* Check if the SPI  interrupt is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_INT_EN) != SPI_EX_GCTL_INT_EN)
    {
        /* Enable SPI interrupt */
        hspi->Instance->GCTL |=(0x1U<<SPI_EX_GCTL_INT_EN_Pos);
    }

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

error:
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;

}


/**
  * @brief  Transmit an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData pointer to data buffer
  * @param  Size amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_Transmit_DMA(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hspi);

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;

    /* Init field not used in handle to zero */
    hspi->pRxBuffPtr  = (uint8_t *)NULL;
    hspi->TxISR       = NULL;
    hspi->RxISR       = NULL;
    hspi->RxXferSize  = 0U;
    hspi->RxXferCount = 0U;

    /* Set the SPI TxDMA transfer complete callback */
    hspi->hdmatx->XferBlockCallback = SPI_EX_DMATransmitCplt;

    /* Set the DMA error callback */
    hspi->hdmatx->XferErrorCallback = SPI_EX_DMAError;

    if(hspi->Init.TXTLF == SPI_EX_TXTLF_1)
    {
        MODIFY_REG(hspi->Instance->GCTL,SPI_EX_GCTL_TXTLF,SPI_EX_GCTL_TXTLF_0);
    }

    /* Enable the Tx DMA Stream/Channel */
    if (HAL_OK != HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->TXREG,
                                   hspi->TxXferCount))
    {
        /* Update SPI error code */
        SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_DMA);
        errorcode = HAL_ERROR;

        hspi->State = HAL_SPI_STATE_READY;
        goto error;
    }
    /* Check if the SPI is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    /* Enable the SPI Error Interrupt Bit */
    __HAL_SPI_EX_ENABLE_IT(hspi, (SPI_EX_IT_UNDERRUNN));

    /* Enable Tx DMA Request */
    SET_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

error:
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
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
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_Receive_DMA(SPI_EX_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hspi);

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Set the transaction information */
    hspi->State       = HAL_SPI_STATE_BUSY_RX;
    hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /*Init field not used in handle to zero */
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;
    hspi->TxXferSize  = 0U;
    hspi->TxXferCount = 0U;

    if(hspi->Init.Direction_Mode == SPI_EX_SINGLE_DUPLEX_RECEIVE)
    {
        __HAL_SPI_EX_SETRXDNR(hspi,Size);
    }

    /* Set the SPI Rx DMA transfer complete callback */
    hspi->hdmarx->XferBlockCallback = SPI_EX_DMAReceiveCplt;

    /* Set the DMA error callback */
    hspi->hdmarx->XferErrorCallback = SPI_EX_DMAError;

    /* Enable the Rx DMA Stream/Channel  */
    if (HAL_OK != HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->RXREG, (uint32_t)hspi->pRxBuffPtr,
                                   hspi->RxXferCount))
    {
        /* Update SPI error code */
        SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_DMA);
        errorcode = HAL_ERROR;

        hspi->State = HAL_SPI_STATE_READY;
        goto error;
    }

    /* Enable the SPI Error Interrupt Bit */
    __HAL_SPI_EX_ENABLE_IT(hspi, (SPI_EX_IT_RXOERR | SPI_EX_IT_UNDERRUNN));

    /* Enable SPI DMA Request */
    SET_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

    /* Check if the SPI is already enabled */
    if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
    {
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

error:
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;

}

/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData pointer to transmission data buffer
  * @param  pRxData pointer to reception data buffer
  * @param  Size amount of data to be sent
  * @note   This function is only used in Full-Duplex Mode for SPI
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_TransmitReceive_DMA(SPI_EX_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
        uint16_t Size)
{
    uint32_t             tmp_mode;
    HAL_SPI_EX_StateTypeDef tmp_state;
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hspi);

    /* Init temporary variables */
    tmp_state           = hspi->State;
    tmp_mode            = hspi->Init.Mode;

    if (!((tmp_state == HAL_SPI_STATE_READY) ||
            ((tmp_mode == SPI_EX_MODE_MASTER) && (tmp_state == HAL_SPI_STATE_BUSY_RX))))
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX)
    {
        hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    }

    /* Clear the flag to recycle DMA */
    if (hspi->hdmarx->State == HAL_DMA_STATE_BUSY)
    {
        hspi->hdmarx->State = HAL_DMA_STATE_READY;
    }

    /* Clear the flag to recycle DMA */
    if (hspi->hdmatx->State == HAL_DMA_STATE_BUSY)
    {
        hspi->hdmatx->State = HAL_DMA_STATE_READY;
    }

    if(hspi->Init.TXTLF == SPI_EX_TXTLF_1)
    {
        MODIFY_REG(hspi->Instance->GCTL,SPI_EX_GCTL_TXTLF,SPI_EX_GCTL_TXTLF_0);
    }

    /* This function is only suitable for FULL DUPLEX MODE */
    if(hspi->Init.Direction_Mode != SPI_EX_FULL_DUPLEX)
    {
        errorcode = HAL_MODE_ERROR;
        goto error;
    }
    else
    {

        /* Set the transaction information */
        hspi->ErrorCode   = HAL_SPI_EX_ERROR_NONE;
        hspi->pTxBuffPtr  = (uint8_t *)pTxData;
        hspi->TxXferSize  = Size;
        hspi->TxXferCount = Size;
        hspi->pRxBuffPtr  = (uint8_t *)pRxData;
        hspi->RxXferSize  = Size;
        hspi->RxXferCount = Size;

        /* Init field not used in handle to zero */
        hspi->RxISR       = NULL;
        hspi->TxISR       = NULL;

        /* Check if we are in Rx only or in Rx/Tx Mode and configure the DMA transfer complete callback */
        if (hspi->State == HAL_SPI_STATE_BUSY_RX)
        {
            /* Configure the DMA transfer complete callback */
            hspi->hdmarx->XferBlockCallback     = SPI_EX_DMAReceiveCplt;

        }
        else
        {
            /* Configure the DMA transfer complete callback */
            hspi->hdmarx->XferBlockCallback     = SPI_EX_DMATransmitReceiveCplt;
        }

        /* Set the DMA error callback */
        hspi->hdmarx->XferErrorCallback = SPI_EX_DMAError;

        /* Enable the Rx DMA Stream/Channel  */
        if (HAL_OK != HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->RXREG, (uint32_t)hspi->pRxBuffPtr,
                                       hspi->RxXferCount))
        {
            /* Update SPI error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_DMA);
            errorcode = HAL_ERROR;

            hspi->State = HAL_SPI_STATE_READY;
            goto error;
        }

        /* Enable DMA Request */
        SET_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

        /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
        is performed in DMA reception complete callback  */
        hspi->hdmatx->XferTfrCallback = NULL;
        hspi->hdmatx->XferBlockCallback     = NULL;
        hspi->hdmatx->XferSrcTranCallback    = NULL;
        hspi->hdmatx->XferDstTranCallback    = NULL;
        hspi->hdmatx->XferErrorCallback     = NULL;
        /* Enable the Tx DMA Stream/Channel  */
        if (HAL_OK != HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->TXREG,
                                       hspi->TxXferCount))
        {
            /* Update SPI error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_DMA);
            errorcode = HAL_ERROR;

            hspi->State = HAL_SPI_STATE_READY;
            goto error;
        }


        /* Enable the SPI Error Interrupt Bit */
        __HAL_SPI_EX_ENABLE_IT(hspi, (SPI_EX_IT_UNDERRUNN));

        /* Enable DMA Request */
        SET_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);

        /* Check if the SPI is already enabled */
        if ((hspi->Instance->GCTL & SPI_EX_GCTL_EN) != SPI_EX_GCTL_EN)
        {
            /* Enable SPI peripheral */
            __HAL_SPI_EX_ENABLE(hspi);
        }
    }
error:
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
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
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_Abort(SPI_EX_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef errorcode;
    __IO uint32_t count;
    __IO uint32_t resetcount;

    /* Initialized local variable  */
    errorcode = HAL_OK;
    resetcount = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);
    count = resetcount;

    /* Clear ERRIE interrupt to avoid error interrupts generation during Abort procedure */
    WRITE_REG(hspi->Instance->INTCLR, 0xff);  //SPI_EX_INTCLR_All_Msk

    /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    if (HAL_IS_BIT_SET(hspi->Instance->INTEN, SPI_EX_INTEN_TXIEN) || HAL_IS_BIT_SET(hspi->Instance->INTEN, SPI_EX_INTEN_TXEPT_IEN) \
            ||  HAL_IS_BIT_SET(hspi->Instance->INTEN, SPI_EX_INTEN_TXMATCHEN))
    {
        hspi->TxISR = SPI_EX_AbortTx_ISR;
        /* Wait HAL_SPI_STATE_ABORT state */
        do
        {
            if (count == 0U)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
                break;
            }
            count--;
        } while (hspi->State != HAL_SPI_STATE_ABORT);
        /* Reset Timeout Counter */
        count = resetcount;
    }

    if (HAL_IS_BIT_SET(hspi->Instance->INTEN, SPI_EX_INTEN_RXIEN) || HAL_IS_BIT_SET(hspi->Instance->INTEN, SPI_EX_INTEN_RXMATCHEN) \
            || HAL_IS_BIT_SET(hspi->Instance->INTEN, SPI_EX_INTEN_RXFIFO_FULL_IEN))
    {
        hspi->RxISR = SPI_EX_AbortRx_ISR;
        /* Wait HAL_SPI_STATE_ABORT state */
        do
        {
            if (count == 0U)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
                break;
            }
            count--;
        } while (hspi->State != HAL_SPI_STATE_ABORT);
        /* Reset Timeout Counter */
        count = resetcount;
    }

    /* Disable the SPI DMA Tx request if enabled */
    if (HAL_IS_BIT_SET(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE))
    {
        /* Abort the SPI DMA Tx Stream/Channel : use blocking DMA Abort API (no callback) */
        if (hspi->hdmatx != NULL)
        {
            /* Set the SPI DMA Abort callback :
            will lead to call HAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
            hspi->hdmatx->XferTfrCallback = NULL;

            /* Abort DMA Tx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort(hspi->hdmatx) != HAL_OK)
            {
                hspi->ErrorCode = HAL_SPI_EX_ERROR_ABORT;
            }

            /* Disable Tx DMA Request */
            CLEAR_BIT(hspi->Instance->GCTL, (SPI_EX_GCTL_DMAMODE));

            /* Wait until TXE flag is set */
            do
            {
                if (count == 0U)
                {
                    SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
                    break;
                }
                count--;
            } while ((hspi->Instance->INTSTAT & SPI_EX_INTSTAT_TX_INTF) == RESET);
        }
    }

    /* Reset Tx and Rx transfer counters */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Check error during Abort procedure */
    if (hspi->ErrorCode == HAL_SPI_EX_ERROR_ABORT)
    {
        /* return HAL_Error in case of error during Abort procedure */
        errorcode = HAL_ERROR;
    }
    else
    {
        /* Reset errorCode */
        hspi->ErrorCode = HAL_SPI_EX_ERROR_NONE;
    }

    /* Clear the Error flags in the SR register */
    __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_MASK);

    /* Restore hspi->state to ready */
    hspi->State = HAL_SPI_STATE_READY;

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
  *           - Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_Abort_IT(SPI_EX_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef errorcode;
    uint32_t abortcplt ;
    __IO uint32_t count;
    __IO uint32_t resetcount;

    /* Initialized local variable  */
    errorcode = HAL_OK;
    abortcplt = 1U;
    resetcount = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);
    count = resetcount;

    /* Clear ERRIE interrupt to avoid error interrupts generation during Abort procedure */
    SET_BIT(hspi->Instance->INTSTAT, (SPI_EX_INTSTAT_RXOERR_INTF | SPI_EX_INTSTAT_UNDERRUN_INTF));

    /* Change Rx and Tx Irq Handler to Disable TXEIE, RXNEIE and ERRIE interrupts */
    if (HAL_IS_BIT_SET(hspi->Instance->INTEN, SPI_EX_INTEN_TXEPT_IEN | SPI_EX_INTEN_TXIEN | SPI_EX_INTEN_TXMATCHEN))
    {
        hspi->TxISR = SPI_EX_AbortTx_ISR;
        /* Wait HAL_SPI_STATE_ABORT state */
        do
        {
            if (count == 0U)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
                break;
            }
            count--;
        } while (hspi->State != HAL_SPI_STATE_ABORT);
        /* Reset Timeout Counter */
        count = resetcount;
    }

    if (HAL_IS_BIT_SET(hspi->Instance->INTEN, SPI_EX_INTEN_RXIEN | SPI_EX_INTEN_RXFIFO_FULL_IEN | SPI_EX_INTEN_RXMATCHEN))
    {
        hspi->RxISR = SPI_EX_AbortRx_ISR;
        /* Wait HAL_SPI_STATE_ABORT state */
        do
        {
            if (count == 0U)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
                break;
            }
            count--;
        } while (hspi->State != HAL_SPI_STATE_ABORT);
        /* Reset Timeout Counter */
        count = resetcount;
    }

    /* If DMA Handles are associated to SPI Handle, DMA Abort complete callbacks should be initialised
        before any call to DMA Abort functions */
    /* DMA Tx Handle is valid */
    if (hspi->hdmatx != NULL)
    {
        /* Set DMA Abort Complete callback if UART DMA Tx request if enabled.
        Otherwise, set it to NULL */
        if (HAL_IS_BIT_SET(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE))
        {
            hspi->hdmatx->XferBlockCallback = SPI_EX_DMATxAbortCallback;
        }
        else
        {
            hspi->hdmatx->XferBlockCallback = NULL;
        }
    }
    /* DMA Rx Handle is valid */
    if (hspi->hdmarx != NULL)
    {
        /* Set DMA Abort Complete callback if UART DMA Rx request if enabled.
        Otherwise, set it to NULL */
        if (HAL_IS_BIT_SET(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE))
        {
            hspi->hdmarx->XferBlockCallback = SPI_EX_DMARxAbortCallback;
        }
        else
        {
            hspi->hdmarx->XferBlockCallback = NULL;
        }
    }

    /* Disable the SPI DMA Tx request if enabled */
    if (HAL_IS_BIT_SET(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE))
    {
        /* Abort the SPI DMA Tx Stream/Channel */
        if (hspi->hdmatx != NULL)
        {
            /* Abort DMA Tx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort_IT(hspi->hdmatx) != HAL_OK)
            {
                hspi->hdmatx->XferBlockCallback = NULL;
                hspi->ErrorCode = HAL_SPI_EX_ERROR_ABORT;
            }
            else
            {
                abortcplt = 0U;
            }
        }
    }

    /* Disable the SPI DMA Rx request if enabled */
    if (HAL_IS_BIT_SET(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE))
    {
        /* Abort the SPI DMA Rx Stream/Channel */
        if (hspi->hdmarx != NULL)
        {
            /* Abort DMA Rx Handle linked to SPI Peripheral */
            if (HAL_DMA_Abort_IT(hspi->hdmarx) !=  HAL_OK)
            {
                hspi->hdmarx->XferBlockCallback = NULL;
                hspi->ErrorCode = HAL_SPI_EX_ERROR_ABORT;
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
        if (hspi->ErrorCode == HAL_SPI_EX_ERROR_ABORT)
        {
            /* return HAL_Error in case of error during Abort procedure */
            errorcode = HAL_ERROR;
        }
        else
        {
            /* Reset errorCode */
            hspi->ErrorCode = HAL_SPI_EX_ERROR_NONE;
        }

        /* Clear the Error flags in the INTSTAT register */
        __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_UNDERRUN | SPI_EX_FLAG_RXOERR );

        /* Restore hspi->State to Ready */
        hspi->State = HAL_SPI_STATE_READY;

        /* As no DMA to be aborted, call directly user Abort complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->AbortCpltCallback(hspi);
#else
        HAL_SPI_EX_AbortCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }

    return errorcode;

}

/**
  * @brief  Pause the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_DMAPause(SPI_EX_HandleTypeDef *hspi)
{
    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Disable the SPI DMA Tx & Rx requests */
    CLEAR_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

/**
  * @brief  Resume the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_DMAResume(SPI_EX_HandleTypeDef *hspi)
{
    /* Process Locked */
    __HAL_LOCK(hspi);

    /* Enable the SPI DMA Tx & Rx requests */
    SET_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

    /* Process Unlocked */
    __HAL_UNLOCK(hspi);

    return HAL_OK;
}

/**
  * @brief  Stop the DMA Transfer.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SPI_EX_DMAStop(SPI_EX_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef errorcode = HAL_OK;
    /* The Lock is not implemented on this API to allow the user application
        to call the HAL SPI API under callbacks HAL_SPI_EX_TxCpltCallback() or HAL_SPI_EX_RxCpltCallback() or HAL_SPI_EX_TxRxCpltCallback():
        when calling HAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
        and the correspond call back is executed HAL_SPI_EX_TxCpltCallback() or HAL_SPI_EX_RxCpltCallback() or HAL_SPI_EX_TxRxCpltCallback()
        */

    /* Abort the SPI DMA tx Stream/Channel  */
    if (hspi->hdmatx != NULL)
    {
        if (HAL_OK != HAL_DMA_Abort(hspi->hdmatx))
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_DMA);
            errorcode = HAL_ERROR;
        }
    }
    /* Abort the SPI DMA rx Stream/Channel  */
    if (hspi->hdmarx != NULL)
    {
        if (HAL_OK != HAL_DMA_Abort(hspi->hdmarx))
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_DMA);
            errorcode = HAL_ERROR;
        }
    }

    /* Disable the SPI DMA Tx & Rx requests */
    CLEAR_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);
    hspi->State = HAL_SPI_STATE_READY;
    return errorcode;
}

/**
  * @brief  Handle SPI interrupt request.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval None
  */
void HAL_SPI_EX_IRQHandler(SPI_EX_HandleTypeDef *hspi)
{
    uint32_t itsource = hspi->Instance->INTEN;
    uint32_t itflag   = hspi->Instance->MINTSTAT;

    /* SPI in mode Receiver ----------------------------------------------------*/
    if (((SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_RXFIFO_FULL) != RESET) && (SPI_EX_CHECK_IT_SOURCE(itsource, SPI_EX_IT_RXFIFO_FULL) != RESET)) || \
            ((SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_RXMATCH) != RESET) && (SPI_EX_CHECK_IT_SOURCE(itsource, SPI_EX_IT_RXMATCH) != RESET)) || \
            ((SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_RX) != RESET) && (SPI_EX_CHECK_IT_SOURCE(itsource, SPI_EX_IT_RX) != RESET)))
    {
        hspi->RxISR(hspi);
        __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_RX); //FullDuplex_ComIT ��Ҫ�����flag
        return;
    }

    /* SPI in mode Transmitter -------------------------------------------------*/
    if (((SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_TX) != RESET) && (SPI_EX_CHECK_IT_SOURCE(itsource, SPI_EX_IT_TX) != RESET)) || \
            ((SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_TXMATCH) != RESET) && (SPI_EX_CHECK_IT_SOURCE(itsource, SPI_EX_IT_TXMATCH) != RESET)) || \
            ((SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_TXEPT) != RESET) && (SPI_EX_CHECK_IT_SOURCE(itsource, SPI_EX_IT_TXEPT) != RESET)))
    {
        hspi->TxISR(hspi);
        return;
    }

    /* SPI in Error Treatment --------------------------------------------------*/
    if (((SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_UNDERRUN) != RESET) || (SPI_EX_CHECK_IT_SOURCE(itflag, SPI_EX_IT_UNDERRUNN) != RESET)) || \
            ((SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_RXOERR) != RESET) || (SPI_EX_CHECK_IT_SOURCE(itflag, SPI_EX_IT_RXOERR) != RESET)))
    {

        /* SPI slave transmitter underload interrupt occurred ----------------------------------*/
        if (SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_UNDERRUN) != RESET)
        {
            if (hspi->State != HAL_SPI_STATE_BUSY_TX)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
                __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_UNDERRUN);
            }
            else
            {
                __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_UNDERRUN);
                return;
            }

        }

        /* SPI receiver overflow error interrupt occurred ----------------------------------*/
        if (SPI_EX_CHECK_MINTFLAG(itflag, SPI_EX_MINTFLAG_RXOERR) != RESET)
        {
            if (hspi->State != HAL_SPI_STATE_BUSY_TX)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
                __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_RXOERR);
            }
            else
            {
                __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_RXOERR);
                return;
            }

        }

        if (hspi->ErrorCode != HAL_SPI_EX_ERROR_NONE)
        {
            /* Disable all interrupts */
            __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_MASK);

            hspi->State = HAL_SPI_STATE_READY;
            /* Disable the SPI DMA requests if enabled */
            if ((HAL_IS_BIT_SET(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE)))
            {
                CLEAR_BIT(hspi->Instance->GCTL, (SPI_EX_GCTL_DMAMODE));

                /* Abort the SPI DMA Rx channel */
                if (hspi->hdmarx != NULL)
                {
                    /* Set the SPI DMA Abort callback :
                    will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
                    //hspi->hdmarx->BlockCallback = SPI_DMAAbortOnError;
                    hspi->hdmarx->XferBlockCallback = SPI_EX_DMAAbortOnError;
                    if (HAL_OK != HAL_DMA_Abort_IT(hspi->hdmarx))
                    {
                        SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
                    }
                }

                /* Abort the SPI DMA Tx channel */
                if (hspi->hdmatx != NULL)
                {
                    /* Set the SPI DMA Abort callback :
                    will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
                    //hspi->hdmatx->BlockCallback = SPI_DMAAbortOnError;
                    hspi->hdmatx->XferBlockCallback = SPI_EX_DMAAbortOnError;
                    if (HAL_OK != HAL_DMA_Abort_IT(hspi->hdmatx))
                    {
                        SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
                    }
                }

            }
            else
            {
                /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
                hspi->ErrorCallback(hspi);
#else
                HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
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
__weak void HAL_SPI_EX_TxCpltCallback(SPI_EX_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_SPI_TxCpltCallback should be implemented in the user file
    */
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_EX_RxCpltCallback(SPI_EX_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_SPI_RxCpltCallback should be implemented in the user file
    */
}

/**
  * @brief  Tx and Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_EX_TxRxCpltCallback(SPI_EX_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_SPI_EX_TxRxCpltCallback should be implemented in the user file
    */
}

/**
  * @brief  SPI error callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_EX_ErrorCallback(SPI_EX_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_SPI_ErrorCallback should be implemented in the user file
    */
    /* NOTE : The ErrorCode parameter in the hspi handle is updated by the SPI processes
                and user can use HAL_SPI_GetError() API to check the latest error occurred
    */
}


/**
  * @brief  SPI Abort Complete callback.
  * @param  hspi SPI handle.
  * @retval None
  */
__weak void HAL_SPI_EX_AbortCpltCallback(SPI_EX_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_SPI_AbortCpltCallback can be implemented in the user file.
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
     (+) HAL_SPI_GetState() API can be helpful to check in run-time the state of the SPI peripheral
     (+) HAL_SPI_GetError() check in run-time Errors occurring during communication
@endverbatim
  * @{
  */

/**
  * @brief  Return the SPI handle state.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI state
  */
HAL_SPI_EX_StateTypeDef HAL_SPI_GetState(SPI_EX_HandleTypeDef *hspi)
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
uint32_t HAL_SPI_EX_GetError(SPI_EX_HandleTypeDef *hspi)
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
static void SPI_EX_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
    SPI_EX_HandleTypeDef *hspi = (SPI_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
    uint32_t tickstart;

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* DMA Normal Mode */

#if defined(UM32x42x) || defined(UM32x41x) 
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == RESET)
#else
	if ((hdma->Instance1->CFGL & DMA_CFG0_RELOAD_DST) == RESET)
#endif
    {
        /* Disable ERR interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_UNDERRUNN | SPI_EX_IT_RXOERR);

        /* Disable DMA Request */
        CLEAR_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

        /* Check the end of the transaction */
        if (SPI_EX_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
        }

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_UNDERRUN | SPI_EX_FLAG_RXOERR);

        hspi->TxXferCount = 0U;
        hspi->State = HAL_SPI_STATE_READY;

        if (hspi->ErrorCode != HAL_SPI_EX_ERROR_NONE)
        {
            /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->ErrorCallback(hspi);
#else
            HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
            return;
        }
    }
    /* Call user Tx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->TxCpltCallback(hspi);
#else
    HAL_SPI_EX_TxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_EX_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
{
    SPI_EX_HandleTypeDef *hspi = (SPI_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
    uint32_t tickstart;

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* DMA Normal Mode */
#if defined(UM32x42x) || defined(UM32x41x)
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == RESET)
#else
	if ((hdma->Instance1->CFGL & DMA_CFG0_RELOAD_DST) == RESET)
#endif
    {
        /* Disable ERR interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_UNDERRUNN | SPI_EX_IT_RXOERR);

        /* Disable DMA Request */
        CLEAR_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

        /* Check the end of the transaction */
        if (SPI_EX_EndRxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
        {
            hspi->ErrorCode = HAL_SPI_EX_ERROR_FLAG;
        }

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_UNDERRUN | SPI_EX_FLAG_RXOERR);

        hspi->RxXferCount = 0U;
        hspi->State = HAL_SPI_STATE_READY;

        if (hspi->ErrorCode != HAL_SPI_EX_ERROR_NONE)
        {
            /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->ErrorCallback(hspi);
#else
            HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
            return;
        }
    }

    /* Call user Rx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->RxCpltCallback(hspi);
#else
    HAL_SPI_EX_RxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

}

/**
  * @brief  DMA SPI transmit receive process complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_EX_DMATransmitReceiveCplt(DMA_HandleTypeDef *hdma)
{
    SPI_EX_HandleTypeDef *hspi = (SPI_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
    uint32_t tickstart;
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* DMA Normal Mode */
#if defined(UM32x42x) || defined(UM32x41x)
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == RESET)
#else
    if ((hdma->Instance1->CFGL & DMA_CFG0_RELOAD_DST) == RESET)
#endif
    {
        /* Disable ERR interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_RXOERR | SPI_EX_IT_UNDERRUNN);

        /* Check the end of the transaction */
        if (SPI_EX_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
        }
        /* Disable Rx/Tx DMA Request */
        CLEAR_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_MASK);

        hspi->TxXferCount = 0U;
        hspi->RxXferCount = 0U;
        hspi->State = HAL_SPI_STATE_READY;

        if (hspi->ErrorCode != HAL_SPI_EX_ERROR_NONE)
        {
            /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->ErrorCallback(hspi);
#else
            HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
            return;
        }
    }
    /* Call user TxRx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->TxRxCpltCallback(hspi);
#else
    HAL_SPI_EX_TxRxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}


/**
  * @brief  DMA SPI communication abort callback, when initiated by HAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_EX_DMAAbortOnError(DMA_HandleTypeDef *hdma)
{
    SPI_EX_HandleTypeDef *hspi = (SPI_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent);
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->ErrorCallback(hspi);
#else
    HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI communication error callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void SPI_EX_DMAError(DMA_HandleTypeDef *hdma)
{
    SPI_EX_HandleTypeDef *hspi = (SPI_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

    /* Stop the disable DMA transfer on SPI side */
    CLEAR_BIT(hspi->Instance->GCTL,  SPI_EX_GCTL_DMAMODE);

    SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_DMA);
    hspi->State = HAL_SPI_STATE_READY;
    /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->ErrorCallback(hspi);
#else
    HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_EX_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
{
    SPI_EX_HandleTypeDef *hspi = (SPI_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */
    __IO uint32_t count;

    hspi->hdmatx->XferBlockCallback = NULL;
    count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

    /* Disable Tx DMA Request */
    CLEAR_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

    /* Wait until TXE flag is set */
    do
    {
        if (count == 0U)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
            break;
        }
        count--;
    } while ((hspi->Instance->CSTAT & SPI_EX_STATE_TXEPT) == RESET);

    /* Check if an Abort process is still ongoing */
    if (hspi->hdmarx != NULL)
    {
        if (hspi->hdmarx->XferBlockCallback != NULL)
        {
            return;
        }
    }

    /* No Abort process still ongoing : All DMA Stream/Channel are aborted, call user Abort Complete callback */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Check no error during Abort procedure */
    if (hspi->ErrorCode != HAL_SPI_EX_ERROR_ABORT)
    {
        /* Reset errorCode */
        hspi->ErrorCode = HAL_SPI_EX_ERROR_NONE;
    }

    /* Clear the Error flags in the INTSTAT register */
    __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_MASK);

    /* Restore hspi->State to Ready */
    hspi->State  = HAL_SPI_STATE_READY;

    /* Call user Abort complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->AbortCpltCallback(hspi);
#else
    HAL_SPI_EX_AbortCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SPI Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
static void SPI_EX_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
{
    SPI_EX_HandleTypeDef *hspi = (SPI_EX_HandleTypeDef *)(((DMA_HandleTypeDef *)hdma)->Parent); /* Derogation MISRAC2012-Rule-11.5 */

    /* Disable SPI Peripheral */
    __HAL_SPI_EX_DISABLE(hspi);

    hspi->hdmarx->XferBlockCallback = NULL;

    /* Disable DMA Request */
    CLEAR_BIT(hspi->Instance->GCTL, SPI_EX_GCTL_DMAMODE);

    /* Check Busy flag */
    if (SPI_EX_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, HAL_GetTick()) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
    }

    /* Check if an Abort process is still ongoing */
    if (hspi->hdmatx != NULL)
    {
        if (hspi->hdmatx->XferBlockCallback != NULL)
        {
            return;
        }
    }

    /* No Abort process still ongoing : All DMA Stream/Channel are aborted, call user Abort Complete callback */
    hspi->RxXferCount = 0U;
    hspi->TxXferCount = 0U;

    /* Check no error during Abort procedure */
    if (hspi->ErrorCode != HAL_SPI_EX_ERROR_ABORT)
    {
        /* Reset errorCode */
        hspi->ErrorCode = HAL_SPI_EX_ERROR_NONE;
    }

    /* Clear the Error flags in the SR register */
    __HAL_SPI_EX_CLEAR_FLAG(hspi,SPI_EX_FLAG_MASK);

    /* Restore hspi->State to Ready */
    hspi->State  = HAL_SPI_STATE_READY;

    /* Call user Abort complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->AbortCpltCallback(hspi);
#else
    HAL_SPI_EX_AbortCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
}

/**
  * @brief  Rx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_2linesRxISR_8BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    /* Receive data in 8bit mode */
    *((uint8_t *)hspi->pRxBuffPtr) = *((__IO uint8_t *)&hspi->Instance->RXREG);
    hspi->pRxBuffPtr++;
    hspi->RxXferCount--;
    /* Check end of the reception */
    if (hspi->RxXferCount == 0U)
    {
        /* Disable RXNE  and ERR interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, (SPI_EX_IT_RX));
        if (hspi->TxXferCount == 0U)
        {
            SPI_EX_CloseRxTx_ISR(hspi);
        }

    }
}

/**
  * @brief  Tx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_2linesTxISR_8BIT(struct __SPI_EX_HandleTypeDef *hspi)
{

    while((__HAL_SPI_EX_CURRENT_STATE(hspi,SPI_EX_STATE_TXFULL)) == SET);
    hspi->Instance->TXREG = (*hspi->pTxBuffPtr);
    hspi->pTxBuffPtr++;
    hspi->TxXferCount--;

    /* Check the end of the transmission */
    if (hspi->TxXferCount == 0U)
    {
        /* Disable TXE interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_TX);

        if (hspi->RxXferCount == 0U)
        {
            SPI_EX_CloseRxTx_ISR(hspi);
        }
    }

}
/**
  * @brief  Rx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_2linesRxISR_16BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    /* Receive data in 16 Bit mode */
    *((uint16_t *)hspi->pRxBuffPtr) = *((__IO uint16_t *)&hspi->Instance->RXREG);
    hspi->pRxBuffPtr += sizeof(uint16_t);
    hspi->RxXferCount--;
    if (hspi->RxXferCount == 0U)
    {
        /* Disable RXNE interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_RX);

        if (hspi->TxXferCount == 0U)
        {
            SPI_EX_CloseRxTx_ISR(hspi);
        }
    }

}

/**
  * @brief  Tx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_2linesTxISR_16BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    while((__HAL_SPI_EX_CURRENT_STATE(hspi,SPI_EX_STATE_TXFULL)) == SET);
    /* Transmit data in 16 Bit mode */
    hspi->Instance->TXREG = *((uint16_t *)hspi->pTxBuffPtr);
    hspi->pTxBuffPtr += sizeof(uint16_t);
    hspi->TxXferCount--;

    /* Enable CRC Transmission */
    if (hspi->TxXferCount == 0U)
    {
        /* Disable TXE interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_TX);

        if (hspi->RxXferCount == 0U)
        {
            SPI_EX_CloseRxTx_ISR(hspi);
        }
    }
}


/**
  * @brief  Rx 32-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_2linesRxISR_32BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    /* Receive data in 32 Bit mode */
    *((uint32_t *)hspi->pRxBuffPtr) = *((__IO uint32_t *)&hspi->Instance->RXREG);
    hspi->pRxBuffPtr += sizeof(uint32_t);
    hspi->RxXferCount--;

    if (hspi->RxXferCount == 0U)
    {
        /* Disable RXNE interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_RX  | SPI_EX_IT_RXOERR);

        if (hspi->TxXferCount == 0U)
        {
            SPI_EX_CloseRxTx_ISR(hspi);
        }
    }

}

/**
  * @brief  Tx 32-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_2linesTxISR_32BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    while((__HAL_SPI_EX_CURRENT_STATE(hspi,SPI_EX_STATE_TXFULL)) == SET);
    /* Transmit data in 32 Bit mode */
    hspi->Instance->TXREG = *((uint32_t *)hspi->pTxBuffPtr);
    hspi->pTxBuffPtr += sizeof(uint32_t);
    hspi->TxXferCount--;

    /* Enable CRC Transmission */
    if (hspi->TxXferCount == 0U)
    {
        /* Disable TXE interrupt */
        __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_TX);

        if (hspi->RxXferCount == 0U)
        {
            SPI_EX_CloseRxTx_ISR(hspi);
        }
    }
}

/**
  * @brief  Manage the receive 8-bit in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_RxISR_8BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX)
    {
        __HAL_SPI_EX_DISABLE(hspi);
        SPI_EX_1LINE_RX(hspi);
        SPI_EX_TX_DISABLE(hspi);
        SPI_EX_RX_ENABLE(hspi);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }
    /* it's set to provide clock */
    if(hspi->Init.Mode == SPI_EX_MODE_MASTER && (hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX))
    {
        SPI_EX_TX_DISABLE(hspi);
        hspi->Instance->TXREG =0xFF;
    }
    /* Wait to receive a complete word */
    while(!(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_RXAVL)));

    *((uint8_t *)hspi->pRxBuffPtr) = (hspi->Instance->RXREG);
    hspi->pRxBuffPtr += sizeof(uint8_t);
    hspi->RxXferCount--;

    if(hspi->Init.Mode == SPI_EX_MODE_MASTER)
    {
        SPI_EX_TX_ENABLE(hspi);
        SPI_EX_RX_DISABLE(hspi);
    }

    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX && (hspi->Init.Mode == SPI_EX_MODE_MASTER))
    {
        SPI_EX_RX_DISABLE(hspi);
    }

    if (hspi->RxXferCount == 0U)
    {
        SPI_EX_CloseRx_ISR(hspi);
    }
}

/**
  * @brief  Manage the 16-bit receive in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_RxISR_16BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX)
    {
        __HAL_SPI_EX_DISABLE(hspi);
        SPI_EX_1LINE_RX(hspi);
        SPI_EX_TX_DISABLE(hspi);
        SPI_EX_RX_ENABLE(hspi);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }
    /* it's set to provide clock */
    if(hspi->Init.Mode == SPI_EX_MODE_MASTER && (hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX))
    {
        SPI_EX_TX_DISABLE(hspi);
        hspi->Instance->TXREG =0xFF;
    }

    while(!(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_RXAVL)));
    *((uint16_t *)hspi->pRxBuffPtr) = (hspi->Instance->RXREG);
    hspi->pRxBuffPtr += sizeof(uint16_t);
    hspi->RxXferCount--;

    if(hspi->Init.Mode == SPI_EX_MODE_MASTER)
    {
        SPI_EX_TX_ENABLE(hspi);
    }

    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX && (hspi->Init.Mode == SPI_EX_MODE_MASTER))
    {
        SPI_EX_RX_DISABLE(hspi);
    }

    if (hspi->RxXferCount == 0U)
    {
        SPI_EX_CloseRx_ISR(hspi);
    }
}

/**
  * @brief  Manage the 32-bit receive in Interrupt context.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_RxISR_32BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX)
    {
        __HAL_SPI_EX_DISABLE(hspi);
        SPI_EX_1LINE_RX(hspi);
        SPI_EX_TX_DISABLE(hspi);
        SPI_EX_RX_ENABLE(hspi);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    /* it's set to provide clock */
    if(hspi->Init.Mode == SPI_EX_MODE_MASTER && (hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX))
    {
        SPI_EX_TX_DISABLE(hspi);
        hspi->Instance->TXREG =0xFF;
    }

    while(!(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_RXAVL)));
    *((uint32_t *)hspi->pRxBuffPtr) = (hspi->Instance->RXREG);
    hspi->pRxBuffPtr += sizeof(uint32_t);
    hspi->RxXferCount--;

    if(hspi->Init.Mode == SPI_EX_MODE_MASTER)
    {
        SPI_EX_TX_ENABLE(hspi);
    }

    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX && (hspi->Init.Mode == SPI_EX_MODE_MASTER))
    {
        SPI_EX_RX_DISABLE(hspi);
    }

    if (hspi->RxXferCount == 0U)
    {
        SPI_EX_CloseRx_ISR(hspi);
    }
}

/**
  * @brief  Handle the data 8-bit transmit in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_TxISR_8BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    if((hspi->Init.Mode == SPI_EX_MODE_SLAVE) && (hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX))
    {
        uint32_t timecount = 0xFFFFFFFFU;
        while(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXFULL) == SET)
        {
            timecount = timecount- 1;
            if(timecount==0) {
                break;
            }
        }
    }

    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX)
    {

        SPI_EX_1LINE_TX(hspi);
        SPI_EX_TX_ENABLE(hspi);
        SPI_EX_RX_DISABLE(hspi);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    /* Transmit data in 8 Bit mode */
    hspi->Instance->TXREG = *((uint8_t *)hspi->pTxBuffPtr);
    hspi->pTxBuffPtr += sizeof(uint8_t);
    hspi->TxXferCount--;

    if(hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX)
    {
        while(__HAL_SPI_EX_CURRENT_STATE(hspi,SPI_EX_STATE_TXEPT) == SET);
    }
    else
    {
        /* Wait until Transmitter FIFO and "transmit shift register" are empty. */
        while(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXEPT) == RESET)
        {
            uint32_t timecount = 0xFFFFFFFU;
            while(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXFULL) == SET)
            {
                timecount = timecount- 1;
                if(timecount==0) {
                    break;
                }
            }
        }
    }
    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX && (hspi->Init.Mode == SPI_EX_MODE_SLAVE))
    {
        SPI_EX_TX_DISABLE(hspi);
    }

    if (hspi->TxXferCount == 0U)
    {
        SPI_EX_CloseTx_ISR(hspi);
    }

}


/**
  * @brief  Handle the data 16-bit transmit in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_TxISR_16BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    if((hspi->Init.Mode == SPI_EX_MODE_SLAVE) && (hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX))
    {
        uint32_t timecount = 0xFFFFFFFFU;
        while(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXFULL) == SET)
        {
            timecount = timecount- 1;
            if(timecount==0) {
                break;
            }
        }
    }
    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX)
    {
        /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
        __HAL_SPI_EX_DISABLE(hspi);
        SPI_EX_1LINE_TX(hspi);
        SPI_EX_TX_ENABLE(hspi);
        SPI_EX_RX_DISABLE(hspi);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    /* Transmit data in 16 Bit mode */
    hspi->Instance->TXREG = *((uint16_t *)hspi->pTxBuffPtr);
    hspi->pTxBuffPtr += sizeof(uint16_t);
    hspi->TxXferCount--;

    while(__HAL_SPI_EX_CURRENT_STATE(hspi,SPI_EX_STATE_TXEPT) == SET);

    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX && (hspi->Init.Mode == SPI_EX_MODE_SLAVE))
    {
        SPI_EX_TX_DISABLE(hspi);
    }

    if (hspi->TxXferCount == 0U)
    {
        SPI_EX_CloseTx_ISR(hspi);
    }
}

/**
  * @brief  Handle the data 32-bit transmit in Interrupt mode.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_TxISR_32BIT(struct __SPI_EX_HandleTypeDef *hspi)
{
    if((hspi->Init.Mode == SPI_EX_MODE_SLAVE) && (hspi->Init.Direction_Mode != SPI_EX_HALF_DUPLEX))
    {
        uint32_t timecount = 0xFFFFFFFFU;
        while(__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXFULL) == SET)
        {
            timecount = timecount- 1;
            if(timecount==0) {
                break;
            }
        }
    }
    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX)
    {
        /* Disable SPI Peripheral before set 1Line direction (BIDIOE bit) */
        __HAL_SPI_EX_DISABLE(hspi);
        SPI_EX_1LINE_TX(hspi);
        SPI_EX_TX_ENABLE(hspi);
        SPI_EX_RX_DISABLE(hspi);
        /* Enable SPI peripheral */
        __HAL_SPI_EX_ENABLE(hspi);
    }

    /* Transmit data in 32 Bit mode */
    hspi->Instance->TXREG = *((uint32_t *)hspi->pTxBuffPtr);
    hspi->pTxBuffPtr += sizeof(uint32_t);
    hspi->TxXferCount--;

    while(__HAL_SPI_EX_CURRENT_STATE(hspi,SPI_EX_STATE_TXEPT) == SET);

    if(hspi->Init.Direction_Mode == SPI_EX_HALF_DUPLEX && (hspi->Init.Mode == SPI_EX_MODE_SLAVE))
    {
        SPI_EX_TX_DISABLE(hspi);
    }

    if (hspi->TxXferCount == 0U)
    {
        SPI_EX_CloseTx_ISR(hspi);
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
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_EX_WaitFlagStateUntilTimeout(SPI_EX_HandleTypeDef *hspi, uint32_t Flag, FlagStatus State,
        uint32_t Timeout, uint32_t Tickstart)
{
    __IO uint32_t count;
    uint32_t tmp_timeout;
    uint32_t tmp_tickstart;

    /* Adjust Timeout value  in case of end of transfer */
    tmp_timeout   = Timeout - (HAL_GetTick() - Tickstart);
    tmp_tickstart = HAL_GetTick();

    /* Calculate Timeout based on a software loop to avoid blocking issue if Systick is disabled */
    count = tmp_timeout * ((SystemCoreClock * 32U) >> 20U);

    while ((__HAL_SPI_EX_CURRENT_STATE(hspi, Flag) ? SET : RESET) != State)
    {
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tmp_tickstart) >= tmp_timeout) || (tmp_timeout == 0U))
            {
                /* Disable interrupts for the interrupt process */
                __HAL_SPI_EX_DISABLE_IT(hspi, (SPI_EX_IT_MASK));

                /* Disable SPI peripheral */
                __HAL_SPI_EX_DISABLE(hspi);

                hspi->State = HAL_SPI_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(hspi);

                return HAL_TIMEOUT;
            }

            /* If Systick is disabled or not incremented, deactivate timeout to go in disable loop procedure */
            if (count == 0U)
            {
                tmp_timeout = 0U;
            }
            count--;
        }
    }


    return HAL_OK;
}

/**
  * @brief  Handle the check of the RX transaction complete.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_EX_EndRxTransaction(SPI_EX_HandleTypeDef *hspi,  uint32_t Timeout, uint32_t Tickstart)
{
    if (hspi->Init.Mode == SPI_EX_MODE_MASTER)
    {
        /* Disable SPI peripheral */
        __HAL_SPI_EX_DISABLE(hspi);
    }

    /* Erratasheet: BSY bit may stay high at the end of a data transfer in Slave mode */
    if (hspi->Init.Mode == SPI_EX_MODE_MASTER)
    {
        /* Wait the RXNE reset */
        if (SPI_EX_WaitFlagStateUntilTimeout(hspi, SPI_EX_STATE_RXAVL, RESET, Timeout, Tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}


/**
  * @brief  Handle the check of the RXTX or TX transaction complete.
  * @param  hspi SPI handle
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_EX_EndRxTxTransaction(SPI_EX_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart)
{
    /* Timeout in ��s */
    __IO uint32_t count = SPI_BSY_FLAG_WORKAROUND_TIMEOUT * (SystemCoreClock / 24U / 1000000U);
    /* Erratasheet: BSY bit may stay high at the end of a data transfer in Slave mode */
    if (hspi->Init.Mode == SPI_EX_MODE_MASTER)
    {
        /* Control the BSY flag */
        if (SPI_EX_WaitFlagStateUntilTimeout(hspi, SPI_EX_STATE_TXEPT, SET, Timeout, Tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
            return HAL_TIMEOUT;
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
        } while (__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXEPT) != RESET);
    }

    return HAL_OK;
}

/**
  * @brief  Handle the end of the RXTX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_CloseRxTx_ISR(SPI_EX_HandleTypeDef *hspi)
{
    uint32_t tickstart;
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

    /* Init tickstart for timeout management */
    tickstart = HAL_GetTick();

    /* Disable ERR interrupt */
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_RXOERR);
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_UNDERRUNN);

    /* Wait until TXE flag is set */
    do
    {
        if (count == 0U)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
            break;
        }
        count--;
    } while ((hspi->Instance->CSTAT & SPI_EX_STATE_TXEPT) == RESET);

    /* Check the end of the transaction */
    if (SPI_EX_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
    }

    if (hspi->ErrorCode == HAL_SPI_EX_ERROR_NONE)
    {
        if (hspi->State == HAL_SPI_STATE_BUSY_RX)
        {
            hspi->State = HAL_SPI_STATE_READY;
            /* Call user Rx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->RxCpltCallback(hspi);
#else
            HAL_SPI_EX_RxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
        }
        else
        {
            hspi->State = HAL_SPI_STATE_READY;
            /* Call user TxRx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->TxRxCpltCallback(hspi);
#else
            HAL_SPI_EX_TxRxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
        }
    }
    else
    {
        hspi->State = HAL_SPI_STATE_READY;
        /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->ErrorCallback(hspi);
#else
        HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }
}

/**
  * @brief  Handle the end of the RX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_CloseRx_ISR(SPI_EX_HandleTypeDef *hspi)
{
    uint32_t tickstart;
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

    /* Init tickstart for timeout management */
    tickstart = HAL_GetTick();

    /* Disable ERR interrupt */
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_RX);
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_RXMATCH);
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_RXFIFO_FULL);
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_RXOERR);
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_UNDERRUNN);

    /* Wait until TXE flag is set */
    do
    {
        if (count == 0U)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
            break;
        }
        count--;
    } while ((hspi->Instance->CSTAT & SPI_EX_STATE_RXAVL) == RESET);

    /* Check the end of the transaction */
    if (SPI_EX_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
    }

    if (hspi->ErrorCode == HAL_SPI_EX_ERROR_NONE)
    {
        if (hspi->State == HAL_SPI_STATE_BUSY_RX)
        {
            hspi->State = HAL_SPI_STATE_READY;
            /* Call user Rx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->RxCpltCallback(hspi);
#else
            HAL_SPI_EX_RxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
        }
        else
        {
            hspi->State = HAL_SPI_STATE_READY;
            /* Call user TxRx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->TxRxCpltCallback(hspi);
#else
            HAL_SPI_EX_TxRxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
        }
    }
    else
    {
        hspi->State = HAL_SPI_STATE_READY;
        /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->ErrorCallback(hspi);
#else
        HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }

}

/**
  * @brief  Handle the end of the TX transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_CloseTx_ISR(SPI_EX_HandleTypeDef *hspi)
{
    uint32_t tickstart;
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* Wait until TXE flag is set */
    do
    {
        if (count == 0U)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
            break;
        }
        count--;
    } while ((hspi->Instance->CSTAT & SPI_EX_STATE_TXEPT) == RESET);

    /* Disable  interrupt */
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_TX);
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_TXEPT);
    __HAL_SPI_EX_DISABLE_IT(hspi, SPI_EX_IT_TXMATCH);

    /* Check the end of the transaction */
    if (SPI_EX_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_FLAG);
    }

    hspi->State = HAL_SPI_STATE_READY;
    if (hspi->ErrorCode != HAL_SPI_EX_ERROR_NONE)
    {
        /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->ErrorCallback(hspi);
#else
        HAL_SPI_EX_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }
    else
    {
        /* Call user Rx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
        hspi->TxCpltCallback(hspi);
#else
        HAL_SPI_EX_TxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
    }

}

/**
  * @brief  Handle abort a Rx transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_AbortRx_ISR(SPI_EX_HandleTypeDef *hspi)
{
    __IO uint32_t count = SPI_DEFAULT_TIMEOUT * (SystemCoreClock / 24U / 1000U);

    /* Wait until TXE flag is set */
    do
    {
        if (count == 0U)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_EX_ERROR_ABORT);
            break;
        }
        count--;
    } while ((hspi->Instance->CSTAT & SPI_EX_STATE_RXAVL) == RESET);

    /* Disable SPI Peripheral */
    __HAL_SPI_EX_DISABLE(hspi);

    /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    WRITE_REG(hspi->Instance->INTEN,(~(SPI_EX_IT_MASK)));
    hspi->State = HAL_SPI_STATE_ABORT;
}

/**
  * @brief  Handle abort a Tx or Rx/Tx transaction.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
static void SPI_EX_AbortTx_ISR(SPI_EX_HandleTypeDef *hspi)
{
    /* Disable TXEIE interrupt */
    CLEAR_BIT(hspi->Instance->INTEN, SPI_EX_IT_TX );
    CLEAR_BIT(hspi->Instance->INTEN, SPI_EX_IT_TXEPT);
    CLEAR_BIT(hspi->Instance->INTEN, SPI_EX_IT_UNDERRUNN);
    CLEAR_BIT(hspi->Instance->INTEN, SPI_EX_IT_TXMATCH);

    /* Disable SPI Peripheral */
    __HAL_SPI_EX_DISABLE(hspi);

    hspi->State = HAL_SPI_STATE_ABORT;
}

#if defined ( __GNUC__ )
  #pragma GCC diagnostic pop
#endif

/**
  * @}
  */

#endif

