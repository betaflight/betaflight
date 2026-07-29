/**
  ******************************************************************************
  * @file    um324xx_hal_SPI.c
  * @author  MCU Team
  * @version V1.00
  * @date    10-February-2023
  * @brief   SPI HAL module driver.
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


/** @defgroup SPI SPI
  * @brief SPI HAL module driver
  * @{
  */
#ifdef HAL_SPI_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup SPI_Private_Defines SPI Private Defines
  * @{
  */
#define SPI_DEFAULT_TIMEOUT 100U
#define SPI_BSY_FLAG_WORKAROUND_TIMEOUT 1000U                           /*!< Timeout 1000 μs  */


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
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */


/** @defgroup SPI_Exported_Functions_Group1 Initialization/de-initialization functions
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
  * @brief  irq config
  *
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
HAL_StatusTypeDef SPI_IrqConfig(SPI_HandleTypeDef *hspi)
{
    uint32_t tmpreg;

    /* Check Null pointer */
    if((hspi == NULL)||(hspi->Instance == NULL))
    {
        return HAL_ERROR;
    }

    tmpreg = hspi->Init.IrqType;
    if(tmpreg != SPI_IRQ_NONE)
    {
        /*enable irq*/
        MODIFY_REG(hspi->Instance->IE, SPI_IE_FIELDS, tmpreg);

        if(hspi->Instance == SPI0)
        {
            NVIC_ClearPendingIRQ(SPI0_IRQn);
            NVIC_EnableIRQ(SPI0_IRQn);
        }
        else if(hspi->Instance == SPI1)
        {
            NVIC_ClearPendingIRQ(SPI1_IRQn);
            NVIC_EnableIRQ(SPI1_IRQn);
        }
        else {}

    }
    else
    {
        /**/

    }
    return HAL_OK;
}



/**
  * @brief     set config,write register
  *
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
HAL_StatusTypeDef HAL_SPI_SetConfig(SPI_HandleTypeDef *hspi)
{
    uint32_t tmpreg;

    /* Check the SPI handle allocation */
    if ((hspi == NULL)||(hspi->Instance == NULL))
    {
        return HAL_ERROR;
    }

    /* Configure : SPI Mode, Communication Mode, Clock polarity and phase, NSS management,
    Communication speed, First bit and CRC calculation state */
    tmpreg = (uint32_t)(hspi->Init.Mode | hspi->Init.DMATxEn |hspi->Init.DMARxEn|hspi->Init.FLTEN | hspi->Init.SSNM | hspi->Init.TxoAc
                        |hspi->Init.TXO|  hspi->Init.MSPA | hspi->Init.SSPA |hspi->Init.Wait | hspi->Init.TRI0Mode|  hspi->Init.SSNSEN |hspi->Init.SPI_EN);



    MODIFY_REG(hspi->Instance->CR,SPI_CR_FIELDS,tmpreg);


    /* Configure : SPI Mode, Communication Mode, Clock polarity and phase, NSS management,
    Communication speed, First bit and CRC calculation state */
    tmpreg = (uint32_t)(hspi->Init.SSN0 | hspi->Init.FirstBit | hspi->Init.CLKPhase | hspi->Init.CLKPolarity | hspi->Init.BaudRatePrescaler);


    MODIFY_REG(hspi->Instance->CS0,SPI_CS0_FIELDS,tmpreg);

    tmpreg = (uint32_t)(hspi->Init.SSN1 | hspi->Init.CS1FirstBit | hspi->Init.CS1CLKPhase | hspi->Init.CS1CLKPolarity | hspi->Init.CS1BaudRatePrescaler);

    MODIFY_REG(hspi->Instance->CS1,SPI_CS1_FIELDS,tmpreg);

    /* Configure :DMA */
    SET_BIT(hspi->Instance->DMARXLEV,hspi->Init.DMARxLev);

    SET_BIT(hspi->Instance->DMATXLEV,hspi->Init.DMATxLev);

    return HAL_OK;
}

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
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi)
{
    /* Check the SPI handle allocation */
    if ((hspi == NULL)||(hspi->Instance == NULL))
    {
        return HAL_ERROR;
    }

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
    if (hspi->MspInitCallback == NULL)
    {
        hspi->MspInitCallback = HAL_SPI_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    hspi->MspInitCallback(hspi);
#else
    HAL_SPI_MspInit(hspi);
#endif
    /* SPI init config*/
    HAL_SPI_SetConfig(hspi);

    /* SPI irq config*/
    SPI_IrqConfig(hspi);
    /* SPI enable*/
    HAL_SPIEN_ENABLE(hspi);
    /*clear rx fifo*/
    SPI_Clear_RxBuffer(hspi);
    SPI_Clear_TxBuffer(hspi);

    hspi->state = HAL_SPI_STATE_READY;
    return HAL_OK;
}



/**
  * @}
  */



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
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t temp = Timeout;
    if (hspi->state == HAL_SPI_STATE_READY)
    {
        if((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }


        /* Check if the SPI is already enabled */
        if ((hspi->Instance->CR & SPI_CR_SPIEN) != SPI_CR_SPIEN)
        {
            /* Enable SPI peripheral */
            HAL_SPIEN_ENABLE(hspi);
        }

        hspi->TxXferSize = Size;
        hspi->TxXferCount = Size;
        hspi->state =HAL_SPI_STATE_BUSY;
        HAL_SPI_TXO_OPEN(hspi);     //open tx_only

        while(hspi->TxXferCount > 0U)
        {
            /*txfifo no full*/
            while(HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TNF) == 0)
            {
                if((Timeout--)==0)
                {
                    return HAL_TIMEOUT;
                }
            }
            Timeout = temp;
            /*put data in tx buf*/
            hspi->Instance->TXBUF = *pData;

            pData++;
            hspi->TxXferCount -- ;

        }


        while(HAL_SPI_GET_FLAG(hspi, SPI_FLAG_IDLE)== 0)
        {
            if((Timeout--)==0)
            {
                return HAL_TIMEOUT;
            }
        }

        HAL_SPI_TXO_CLOSE(hspi); //close tx_only

        hspi->state = HAL_SPI_STATE_READY;
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
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
uint32_t  STATE;
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t temp ;
    if (hspi->state == HAL_SPI_STATE_READY)
    {
        if((pData == NULL) || (Size == 0U))
        {
            return HAL_ERROR;
        }



        hspi->RxXferSize = Size;
        hspi->RxXferCount = Size;
        temp = Timeout;
        hspi->state =HAL_SPI_STATE_BUSY;
        while(hspi->RxXferCount > 0U)
        {

            if(((hspi->Instance->CR)&SPI_CR_MM) == SPI_CR_MODE_Master)
            {
                /*txfifo no full*/
                while(HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TNF)== 0)
                {
                    if((Timeout--)==0)
                    {
                        return HAL_TIMEOUT;
                    }
                }
                /*put data in tx buf*/
                hspi->Instance->TXBUF = 0xAA;

                Timeout = temp;
            }

            /*rx fifo no empty*/
            while((HAL_SPI_GET_FLAG(hspi, SPI_IF_RXBF)==0))

            {
                if((Timeout--)==0)
                {
                    return HAL_TIMEOUT;
                }
            }
            *pData++ = hspi->Instance->RXBUF;

            hspi->RxXferCount -- ;
            Timeout = temp;
        }

        hspi->RxXferCount = Size;

        hspi->state = HAL_SPI_STATE_READY;
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}


/**
  * @brief Transmit and Receive an amount of data in blocking mode.
  *
  * @param hspi
  * @param pTxData
  * @param pRxData
  * @param Size
  * @param Timeout
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *     @retval  HAL_TIMEOUT transmit or receive timeout
  *
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,uint32_t Timeout)
{
    uint32_t temp = Timeout;
    if (hspi->state == HAL_SPI_STATE_READY)
    {
        if((pTxData == NULL) || (pRxData == NULL) || (Size == 0U) )
        {
            return HAL_ERROR;
        }

        hspi->TxXferSize = Size;
        hspi->TxXferCount = Size;
        hspi->RxXferSize = Size;
        hspi->RxXferCount = Size;

        hspi->state =HAL_SPI_STATE_BUSY;
        while((hspi->TxXferCount > 0U) && (hspi->RxXferCount > 0U))
        {
            /*txfifo no full*/
            while((HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TNF)) == 0)
            {
                if((Timeout--)==0)
                {
                    return HAL_TIMEOUT;
                }
            }
            /*put data in tx buf*/
            hspi->Instance->TXBUF = *pTxData++;

            Timeout = temp;

            hspi->TxXferCount--;


            /*rx fifo no empty*/
            while((HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXBF)) == 0)
            {
                if((Timeout--)==0)
                {
                    return HAL_TIMEOUT;
                }
            }
            Timeout = temp;
            *pRxData++ = hspi->Instance->RXBUF;

            hspi->RxXferCount-- ;

        }
        /*wait spi idle*/
        while((HAL_SPI_GET_FLAG(hspi, SPI_FLAG_IDLE))== 0)
        {
            if((Timeout--)==0)
            {
                return HAL_TIMEOUT;
            }
        }
        hspi->state = HAL_SPI_STATE_READY;



        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}




/**
  * @brief  rx callback - full duplex
  *
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
static HAL_StatusTypeDef SPI_Full_RxISR(SPI_HandleTypeDef *hspi)
{
    *hspi->pRxBuffPtr = hspi->Instance->RXBUF;
    hspi->pRxBuffPtr++;
    hspi->RxXferCount--;

    if(hspi->RxXferCount == 0)
        HAL_SPI_TxRxCpltCallback(hspi);////融合平台 add

    return HAL_OK;

}



/**
  * @brief  rx callback - half duplex
  *
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
static HAL_StatusTypeDef SPI_Half_RxISR(SPI_HandleTypeDef *hspi)
{
    *hspi->pRxBuffPtr = hspi->Instance->RXBUF;
    hspi->pRxBuffPtr++;
    hspi->RxXferCount--;

    if(hspi->RxXferCount == 0)  //recv completed
    {
        hspi->pRxBuffPtr = NULL;
        hspi->RxISR = NULL;
        HAL_SPI_DISABLE_IT(hspi,SPI_IE_RXBFIE);
        return HAL_OK;
    }
    else
    {   /**recv not completed */
        if(((hspi->Instance->CR)&SPI_CR_MM) == SPI_CR_MODE_Master)
        {
            hspi->Instance->TXBUF = 0xff; //send useless data  produce clk
        }
    }

    return HAL_OK;

}


/**
  * @brief       Receive an amount of data in non-blocking mode with Interrupt.
  *
  * @param hspi  pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param pData  pointer to data buffer
  * @param Size  amount of data to be sent
  * @param recv_callback  receive irq callback
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size,HAL_StatusTypeDef (*recv_callback)())
{
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hspi);

    if (hspi->state != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }

    if((pData == NULL) || (Size == 0U) )
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    hspi->state       = HAL_SPI_STATE_BUSY_RX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pRxBuffPtr  = (uint8_t *)pData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;

    /* Init field not used in handle to zero */
    hspi->pTxBuffPtr  = (uint8_t *)NULL;
    hspi->TxXferSize  = 0U;
    hspi->TxXferCount = 0U;
    hspi->TxISR       = NULL;

    if(recv_callback != NULL)
    {
        hspi->RxISR = recv_callback;
    }
    else
    {
        hspi->RxISR = SPI_Half_RxISR;		   // register recv callback
    }

    HAL_SPI_ENABLE_IT(hspi,SPI_IE_RXBFIE);  //enable rx irq

    hspi->Instance->TXBUF = 0xff; ////send useless data  produce clk for recv data

error:
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;

}




/**
  * @brief  rx callback - half duplex
  *
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
static HAL_StatusTypeDef SPI_Full_TxISR(SPI_HandleTypeDef *hspi)
{
    hspi->Instance->TXBUF = *hspi->pTxBuffPtr;
    hspi->pTxBuffPtr++;
    hspi->TxXferCount--;

    if(hspi->TxXferCount == 0)  //tx completed
    {
        hspi->pTxBuffPtr = NULL;
        hspi->TxISR = NULL;
        HAL_SPI_DISABLE_IT(hspi, SPI_IRQ_TNF); //tx completed,disable irq

        return HAL_OK;
    }

    return HAL_OK;
}


/**
  * @brief  rx callback - half duplex
  *
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
static HAL_StatusTypeDef SPI_Half_TxISR(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TXO_OPEN(hspi);  //open tx_only

    hspi->Instance->TXBUF = *hspi->pTxBuffPtr;
    hspi->pTxBuffPtr++;
    hspi->TxXferCount--;

    if(hspi->TxXferCount == 0)  //tx completed
    {
        hspi->pTxBuffPtr = NULL;
        hspi->TxISR = NULL;
        while(((hspi->Instance->IF)&SPI_FLAG_IDLE) == 0); //wait tx success flag

        HAL_SPI_DISABLE_IT(hspi, SPI_IRQ_TNF); //tx completed,disable irq
        HAL_SPI_TXO_CLOSE(hspi);  //open tx_only


        return HAL_OK;
    }


    return HAL_OK;
}




/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  *
  * @param hspi  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param pData  pointer to data buffer
  * @param Size   amount of data to be sent
  * @param recv_callback  transmit irq callback
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, HAL_StatusTypeDef (*recv_callback)())
{
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process Locked */
    __HAL_LOCK(hspi);

    if((pData == NULL) || (Size == 0U) )
    {
        errorcode = HAL_ERROR;
        goto error;
    }

    if (hspi->state != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        goto error;
    }


    hspi->state       = HAL_SPI_STATE_BUSY_TX;
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
    hspi->pTxBuffPtr  = (uint8_t *)pData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;


    /* Init field not used in handle to zero */
    hspi->pRxBuffPtr  = (uint8_t *)NULL;
    hspi->RxXferSize  = 0U;
    hspi->RxXferCount = 0U;
    hspi->RxISR       = NULL;

    if(recv_callback != NULL)
    {
        hspi->TxISR = recv_callback;
    }
    else
    {
        hspi->TxISR = SPI_Half_TxISR;		//register tx callback
    }

    HAL_SPI_ENABLE_IT(hspi, SPI_IRQ_TNF ); // enable tx fifo not full irq

error:
    __HAL_UNLOCK(hspi);
    return errorcode;

}


/**
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  *
  * @param hspi   hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param pTxData  pointer to transmission data buffer
  * @param pRxData  pointer to reception data buffer
  * @param Size     amount of data to be sent and received
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
        uint16_t Size)
{

    if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }

    SPI_Clear_RxBuffer(hspi);
    SPI_Clear_TxBuffer(hspi);
    hspi->pTxBuffPtr  = (uint8_t *)pTxData;
    hspi->TxXferSize  = Size;
    hspi->TxXferCount = Size;
    hspi->pRxBuffPtr  = (uint8_t *)pRxData;
    hspi->RxXferSize  = Size;
    hspi->RxXferCount = Size;


    hspi->RxISR     = SPI_Full_RxISR;     //register rx callback
    hspi->TxISR     = SPI_Full_TxISR;     //register tx callback

    HAL_SPI_ENABLE_IT(hspi, SPI_IRQ_TNF | SPI_IRQ_RXBF);  //enable tx interrupt

    return HAL_OK;

}


/**
  * @brief  SPI irq function
  *
  * @param hspi  A pointer to a SPI_HandleTypeDef structure that contains configuration information for
  *				  		  the specified SPI module
  * @return HAL_StatusTypeDef
  *     @retval  HAL_OK    nothing wrong
  *     @retval  HAL_ERROR something wrong
  *
  */
void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi)
{
    uint32_t itsource = hspi->Instance->IE;
    uint32_t itflag   = hspi->Instance->IF;

    /*no error occur*/
    if((SPI_CHECK_FLAG(itflag, SPI_FLAG_TXCOL) == 0) && (SPI_CHECK_FLAG(itflag, SPI_IRQ_RXCOL) == 0)
            && (SPI_CHECK_FLAG(itflag, SPI_FLAG_SERR) == 0) && (SPI_CHECK_FLAG(itflag, SPI_IRQ_MERR) == 0))

    {
        /* -------------------------SPI Receiver ----------------------------------------------------*/
        if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_RXBF) != 0) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IRQ_RXBF) != 0))
        {
            if (hspi->RxISR != NULL)
            {
                hspi->RxISR(hspi);

            }
            return;
        }
        /* --------------------------------SPI Tx ----------------------------------------------------*/
        else  if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_TXBE) != 0) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IRQ_TXBE) != 0))
        {
            if (hspi->TxISR != NULL)
            {

                hspi->TxISR(hspi);
            }
            return;
        }
        else  if ((SPI_CHECK_FLAG(itflag, SPI_FLAG_TNF) != 0) && (SPI_CHECK_IT_SOURCE(itsource, SPI_IRQ_TNF) != 0))
        {
            if (hspi->TxISR != NULL)
            {

                hspi->TxISR(hspi);
            }
            return;
        }
    }
    else   //other irq occur
    {

        /*1、tx fifo overflow*/
        if((SPI_CHECK_FLAG(itflag, SPI_FLAG_TXCOL) != 0)&&(SPI_CHECK_FLAG(itsource, SPI_IRQ_TXCOL) != 0))
        {
            /*customize*/
            /*{

            }*/
            SET_BIT(hspi->Instance->IF,SPI_FLAG_TXCOL); //clear flag
        }
        /*2、rx fifo overflow*/
        else if((SPI_CHECK_FLAG(itflag, SPI_FLAG_RXCOL) != 0)&&(SPI_CHECK_FLAG(itsource, SPI_IRQ_RXCOL) != 0))
        {

            /*customize*/
            /*{

            }*/

            SET_BIT(hspi->Instance->IF,SPI_FLAG_RXCOL);  //clear flag
        }
        /*3、slave error*/
        else if((SPI_CHECK_FLAG(itflag, SPI_FLAG_SERR) != 0)&&(SPI_CHECK_FLAG(itsource, SPI_IRQ_SERR) != 0))
        {
            /*customize*/
            /*{

            }*/
            SET_BIT(hspi->Instance->OPCR,SPI_OPCR_SERRC);  //clear flag
        }
        /*4、master error*/
        else if((SPI_CHECK_FLAG(itflag, SPI_FLAG_MERR) != 0)&&(SPI_CHECK_FLAG(itsource, SPI_IRQ_MERR) != 0))
        {
            /*customize*/
            /*{

            }*/
            SET_BIT(hspi->Instance->OPCR,SPI_OPCR_MERRC); //clear flag
        }


        else //ignore error
        {

            SET_BIT(hspi->Instance->IF,SPI_FLAG_TXCOL); //clear flag
            SET_BIT(hspi->Instance->IF,SPI_FLAG_RXCOL); //clear flag
            SET_BIT(hspi->Instance->OPCR,SPI_OPCR_SERRC); //clear flag
            SET_BIT(hspi->Instance->OPCR,SPI_OPCR_MERRC); //clear flag
        }

    }

}


/**
  * @brief  SPI MSP Init.
  * @param  hlpuart  Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI module.
  * @retval None
  */
__weak void HAL_SPI_MspInit(SPI_HandleTypeDef *spi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(spi);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_SPI_MspInit could be implemented in the user file
     */
}

/**
  * @brief  SPI MSP DeInit.
  * @param  hlpuart  Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI module.
  * @retval None
  */
__weak void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(spi);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_SPI_MspInit could be implemented in the user file
     */
}


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
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi)
{
    /* Check the SPI handle allocation */
    if ((hspi == NULL)||(hspi->Instance == NULL))
    {
        return HAL_ERROR;
    }

#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1)
    if (hspi->MspInitCallback == NULL)
    {
        hspi->MspInitCallback = HAL_SPI_MspDeInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    hspi->MspInitCallback(hspi);
#else
    HAL_SPI_MspDeInit(hspi);
#endif

    hspi->state = HAL_SPI_STATE_RESET;
    hspi->Mode = HAL_SPI_MODE_NONE;
    /* Release Lock */
    __HAL_UNLOCK(hspi);
    return HAL_OK;
}


/*****************************************************************************************
******************************************************************************************/

/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hspi);

    /* NOTE : This function should not be modified, when the callback is needed,
                the HAL_SPI_RxCpltCallback should be implemented in the user file
    */
}


/**
  * @brief  SPI error callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
__weak void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
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
  * @brief  Handle SPI Communication Timeout.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param  Flag SPI flag to check
  * @param  State flag state to check
  * @param  Timeout Timeout duration
  * @param  Tickstart tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus State,
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

    while ((HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) != State)
    {
        if (Timeout != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tmp_tickstart) >= tmp_timeout) || (tmp_timeout == 0U))
            {
                /* Disable interrupts for the interrupt process */
                HAL_SPI_DISABLE_IT(hspi, SPI_IRQ_MASK);

                /* Disable SPI peripheral */
                HAL_SPIEN_DISABLE(hspi);

                hspi->state = HAL_SPI_STATE_READY;

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
static HAL_StatusTypeDef SPI_EndRxTransaction(SPI_HandleTypeDef *hspi,  uint32_t Timeout, uint32_t Tickstart)
{
    if (hspi->Init.Mode == SPI_MODE_MASTER)
    {
        /* Disable SPI peripheral */
        HAL_SPIEN_DISABLE(hspi);
    }

    /* Erratasheet: BSY bit may stay high at the end of a data transfer in Slave mode */
    if (hspi->Init.Mode == SPI_MODE_MASTER)
    {
        /* Wait the RXNE reset */
        if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_IF_RXBF, RESET, Timeout, Tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
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
static HAL_StatusTypeDef SPI_EndRxTxTransaction(SPI_HandleTypeDef *hspi, uint32_t Timeout, uint32_t Tickstart)
{
    /* Timeout in μs */
    __IO uint32_t count = SPI_BSY_FLAG_WORKAROUND_TIMEOUT * (SystemCoreClock / 24U / 1000000U);
    /* Erratasheet: BSY bit may stay high at the end of a data transfer in Slave mode */
    if (hspi->Init.Mode == SPI_MODE_MASTER)
    {
        /* Control the BSY flag */
        if (SPI_WaitFlagStateUntilTimeout(hspi, SPI_IF_TXBE, SET, Timeout, Tickstart) != HAL_OK)  //发送空标志
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
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
        } while (HAL_SPI_GET_FLAG(hspi, SPI_IF_TXBE) != RESET); //while (__HAL_SPI_EX_CURRENT_STATE(hspi, SPI_EX_STATE_TXEPT) != RESET);
    }

    return HAL_OK;
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

    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* DMA Normal Mode */
#ifdef UM32x42x
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == RESET)
#else
    if ((hdma->Instance->CFGL & DMA_CFGx_RELOAD_DST) == RESET)
#endif
    {
        /* Disable ERR interrupt */
        HAL_SPI_DISABLE_IT(hspi, SPI_IRQ_SERR | SPI_IRQ_RXCOL);

        /* Disable DMA Request */
        CLEAR_BIT(hspi->Instance->CR, (SPI_CR_DMA_RX_EN&&SPI_CR_DMA_TX_EN));

        /* Check the end of the transaction */
        if (SPI_EndRxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
        {
            hspi->ErrorCode = HAL_SPI_ERROR_FLAG;
        }

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        HAL_SPI_CLEAR_CRCERRFLAG(hspi,SPI_FLAG_SERR | SPI_FLAG_RXCOL);

        hspi->RxXferCount = 0U;
        hspi->state = HAL_SPI_STATE_READY;

        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        {
            /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->ErrorCallback(hspi);
#else
            HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
            return;
        }
    }

    /* Call user Rx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->RxCpltCallback(hspi);
#else
    HAL_SPI_RxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */

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
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();

    /* DMA Normal Mode */
#ifdef UM32x42x
    if ((hdma->Instance->CFG0 & DMA_CFG0_RELOAD_DST) == RESET)
#else
    if ((hdma->Instance->CFGL & DMA_CFGx_RELOAD_DST) == RESET)
#endif
    {
        /* Disable ERR interrupt */
        HAL_SPI_DISABLE_IT(hspi, SPI_IRQ_SERR | SPI_IRQ_RXCOL);

        /* Check the end of the transaction */
        if (SPI_EndRxTxTransaction(hspi, SPI_DEFAULT_TIMEOUT, tickstart) != HAL_OK)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
        }
        /* Disable Rx/Tx DMA Request */
        CLEAR_BIT(hspi->Instance->CR, SPI_CR_DMA_TX_EN&&SPI_CR_DMA_RX_EN);

        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        HAL_SPI_CLEAR_CRCERRFLAG(hspi,SPI_FLAG_MASK);

        hspi->TxXferCount = 0U;
        hspi->RxXferCount = 0U;
        hspi->state = HAL_SPI_STATE_READY;

        if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        {
            /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
            hspi->ErrorCallback(hspi);
#else
            HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
            return;
        }
    }
    /* Call user TxRx complete callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->TxRxCpltCallback(hspi);
#else
    HAL_SPI_TxRxCpltCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
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
    CLEAR_BIT(hspi->Instance->CR,  SPI_CR_DMA_TX_EN&&SPI_CR_DMA_RX_EN);

    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);

    hspi->state = HAL_SPI_STATE_READY;
    /* Call user error callback */
#if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
    hspi->ErrorCallback(hspi);
#else
    HAL_SPI_ErrorCallback(hspi);
#endif /* USE_HAL_SPI_REGISTER_CALLBACKS */
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
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
        uint16_t Size)
{
    uint32_t             tmp_mode;
    HAL_SPI_StateTypeDef tmp_state;
    HAL_StatusTypeDef errorcode = HAL_OK;

    /* Process locked */
    __HAL_LOCK(hspi);

    /* Init temporary variables */
    tmp_state           = hspi->state;
    tmp_mode            = hspi->Init.Mode;

    if (!((tmp_state == HAL_SPI_STATE_READY) ||
            ((tmp_mode == SPI_MODE_MASTER) && (tmp_state == HAL_SPI_STATE_BUSY_RX))))
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
    if (hspi->state != HAL_SPI_STATE_BUSY_RX)
    {
        hspi->state = HAL_SPI_STATE_BUSY_TX_RX;
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


    /* This function is only suitable for FULL DUPLEX MODE */
    if((hspi->Init.Mode != 0x0) && (hspi->Init.Mode != 0x20))//not master and slave mode
    {
        errorcode = HAL_MODE_ERROR;
        goto error;
    }
    else
    {

        /* Set the transaction information */
        hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
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
        if (hspi->state == HAL_SPI_STATE_BUSY_RX)
        {
            /* Configure the DMA transfer complete callback */
            hspi->hdmarx->XferBlockCallback     = SPI_DMAReceiveCplt;

        }
        else
        {
            /* Configure the DMA transfer complete callback */
            hspi->hdmarx->XferBlockCallback     = SPI_DMATransmitReceiveCplt;
        }

        /* Set the DMA error callback */
        hspi->hdmarx->XferErrorCallback = SPI_DMAError;

        /* Enable the Rx DMA Stream/Channel  */
        if (HAL_OK != HAL_DMA_Start_IT(hspi->hdmarx, (uint32_t)&hspi->Instance->RXBUF, (uint32_t)hspi->pRxBuffPtr,
                                       hspi->RxXferCount))
        {
            /* Update SPI error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
            errorcode = HAL_ERROR;

            hspi->state = HAL_SPI_STATE_READY;
            goto error;
        }

        /* Enable DMA Request */
        SET_BIT(hspi->Instance->CR, (SPI_CR_DMA_RX_EN&&SPI_CR_DMA_TX_EN));

        /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
        is performed in DMA reception complete callback  */
        hspi->hdmatx->XferTfrCallback = NULL;
        hspi->hdmatx->XferBlockCallback     = NULL;
        hspi->hdmatx->XferSrcTranCallback    = NULL;
        hspi->hdmatx->XferDstTranCallback    = NULL;
        hspi->hdmatx->XferErrorCallback     = NULL;
        /* Enable the Tx DMA Stream/Channel  */
        if (HAL_OK != HAL_DMA_Start_IT(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->TXBUF,
                                       hspi->TxXferCount))
        {
            /* Update SPI error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
            errorcode = HAL_ERROR;

            hspi->state = HAL_SPI_STATE_READY;
            goto error;
        }


        /* Enable the SPI Error Interrupt Bit */
        HAL_SPI_ENABLE_IT(hspi, (SPI_IRQ_SERR));

        /* Enable DMA Request */
        SET_BIT(hspi->Instance->CR, (SPI_CR_DMA_RX_EN&&SPI_CR_DMA_TX_EN));

        /* Enable SPI peripheral */
        HAL_SPIEN_ENABLE(hspi);

        /* Check if the SPI is already enabled */
        if ((hspi->Instance->CR & SPI_CR_SPIEN) != SPI_CR_SPIEN)
        {
            /* Enable SPI peripheral */
            HAL_SPIEN_ENABLE(hspi);
        }
    }
error:
    /* Process Unlocked */
    __HAL_UNLOCK(hspi);
    return errorcode;
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_SPI_MODULE_ENABLED */
/**
  * @}
  */


/**
  * @}
  */



