/**
  ******************************************************************************
  * @file     um324xF_hal_can.c 
  * @author   MCU Team
  * @version  V1.00 
  * @date     2023-05-08  
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

/** @addtogroup UM324xF_HAL_Driver
  * @{
  */

/** @defgroup CAN CAN
  * @brief HAL CAN module driver
  * @{
  */
#ifdef HAL_CAN_MODULE_ENABLED


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

const uint8_t CAN_BuffStdNum[8]={0, 1, 1, 1, 1, 2, 2, 2};
const uint8_t CAN_BuffExtdNum[8]={1, 1, 1, 2, 2, 2, 2, 3};

/* Exported functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */


/**
  * @brief  Initializes the CAN peripheral according to the specified
  *         parameters in the CAN_InitStruct.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan)
{
    /* Check CAN handle */
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    if (hcan->State == HAL_CAN_STATE_RESET)
    {
    /* Init the low level hardware: CLOCK */
        HAL_CAN_MspInit(hcan);
    }

    /* Reset mode */
    __HAL_CAN_RESET_ENABLE(hcan);

   
    /* Set the bit timing register */ 
    WRITE_REG(hcan->Instance->CONFIG1, (uint32_t)( hcan->Init.TimeSeg2 | hcan->Init.TimeSeg1 | hcan->Init.SyncJumpWidth  
              | ((hcan->Init.Prescaler - 1U) << CAN_CONFIG1_BRP_Pos)));

    /* Initialize the error code */
    hcan->ErrorCode = HAL_CAN_ERROR_NONE;

    /* Release reset mode */
    __HAL_CAN_RESET_DISABLE(hcan);
    
    /* Initialize the CAN state */
    hcan->State = HAL_CAN_STATE_READY;

    
    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  Configures the CAN reception filter according to the specified
  *         parameters in the CAN_FilterInitStruct.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  sFilterConfig pointer to a CAN_FilterTypeDef structure that
  *         contains the filter configuration information.
  * @retval None
  */
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig)
{
    uint32_t tempacr=0;
    HAL_CAN_StateTypeDef state = hcan->State;

    if(hcan == NULL)
    {
        return HAL_ERROR;  
    }
    /* Reset mode */
    __HAL_CAN_RESET_ENABLE(hcan);
    
    if ((state == HAL_CAN_STATE_READY) ||
        (state == HAL_CAN_STATE_LISTENING))
    {
        if(sFilterConfig->FilterMode == HAL_CAN_FilterMode_SINGLE)
        {
            /*Use single filter*/
            __HAL_CAN_SINGLE_FILTER(hcan);

            if(sFilterConfig->FilterIde == CAN_ID_STD)
            {
                tempacr = (uint32_t)((((sFilterConfig->FilterStdId1)&0x7f8) >> 3U) | (((sFilterConfig->FilterStdId1)&0x7) << 13U)
                            | (((sFilterConfig->FilterRtr)&0x01) << 12U) | (((sFilterConfig->FilterData1)&0xFF) << 16U)
                            | (((sFilterConfig->FilterData2)&0xFF) << 24U));

                
            }
            else
            {
                tempacr = (uint32_t)((((sFilterConfig->FilterExtId1)&0x1FE00000) >> 21) | (((sFilterConfig->FilterExtId1)&0x1FE000) >> 5U)
                            | (((sFilterConfig->FilterExtId1)&0x1FE0) << 11U) | (((sFilterConfig->FilterExtId1)&0x1F) << 27U)
                            | (((sFilterConfig->FilterRtr)&0x01) << 26U));    


            }
            WRITE_REG(hcan->Instance->ACR,tempacr);
            WRITE_REG(hcan->Instance->AMR,sFilterConfig->FilterMaskType);
        }
        else if(sFilterConfig->FilterMode == HAL_CAN_FilterMode_DOUBLE)
        {
            /*Use double filter*/
            __HAL_CAN_DOUBLE_FILTER(hcan);

            if(sFilterConfig->FilterIde == CAN_ID_STD)
            {
             /*filter1*/
                tempacr = (((sFilterConfig->FilterStdId1)&0x7f8) >> 3U) | (((sFilterConfig->FilterStdId1)&0x7) << 13U)
                            | (((sFilterConfig->FilterRtr)&0x01) << 12U) | ((((sFilterConfig->FilterData1)&0xf0) >> 4U) << 8U) 
                            | (((sFilterConfig->FilterData1)&0xf) << 24U) ;

                /*filter2*/         
                tempacr |= (((sFilterConfig->FilterStdId2)&0x7f8) << 13U) | (((sFilterConfig->FilterStdId2)&0x7) << 29U)
                            | (((sFilterConfig->FilterRtr)&0x01) << 28U);     
            }

            if(sFilterConfig->FilterIde == CAN_ID_EXT)
            {
                /*filter1*/
                tempacr = (((sFilterConfig->FilterExtId1)&0x1FE00000) >> 21U) | (((sFilterConfig->FilterExtId1)&0x1FE000) >> 5U);
                /*filter2*/    
                tempacr |= (((sFilterConfig->FilterExtId2)&0x1FE00000) >> 5U) | (((sFilterConfig->FilterExtId2)&0x1FE000) << 11U);   


            }
            
            WRITE_REG(hcan->Instance->ACR,tempacr);
            WRITE_REG(hcan->Instance->AMR,sFilterConfig->FilterMaskType);
        }
        else
        {
            hcan->Instance->AMR = CAN_Filter_MASK_ALL;  /*!< No comparison, no filtering */
        }

        __HAL_CAN_NORMAL_MODE(hcan);           /*!< CAN enter normal mode */
        __HAL_CAN_RESET_DISABLE(hcan);           /*!< CAN exit reset mode */

        return HAL_OK;

    }  
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        return HAL_ERROR;
    }    
}

/**@brief    Interrupts management
 *
@verbatim
  ==============================================================================
                       ##### Interrupts management #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) HAL_CAN_ActivateNotification      : Enable interrupts
      (+) HAL_CAN_DeactivateNotification    : Disable interrupts
      (+) HAL_CAN_IRQHandler                : Handles CAN interrupt request

@endverbatim
  * @{
  */
/**
  * @brief  Enable interrupts.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  ActiveITs indicates which interrupts will be enabled.
  *         This parameter can be any combination of @arg CAN_Interrupts.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
{
    HAL_CAN_StateTypeDef state = hcan->State;

    /* Check CAN handle */
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }
    
    if ((state == HAL_CAN_STATE_READY) ||
      (state == HAL_CAN_STATE_LISTENING))
    {
        /* Enable the selected interrupts */
        __HAL_CAN_ENABLE_IT(hcan, ActiveITs);

        /* Return function status */
        return HAL_OK;
    }
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        return HAL_ERROR;
    }
}
/**
  * @brief  Disable interrupts.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  InactiveITs indicates which interrupts will be disabled.
  *         This parameter can be any combination of @arg CAN_Interrupts.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs)
{
    HAL_CAN_StateTypeDef state = hcan->State;

    /* Check CAN handle */
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    if ((state == HAL_CAN_STATE_READY) ||
      (state == HAL_CAN_STATE_LISTENING))
    {
    /* Disable the selected interrupts */
    __HAL_CAN_DISABLE_IT(hcan, InactiveITs);

    /* Return function status */
    return HAL_OK;
    }
    else
    {
    /* Update error code */
    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

    return HAL_ERROR;
  }
}

/**
  * @brief  Handles CAN interrupt request
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan)
{
    uint32_t errorcode = HAL_CAN_ERROR_NONE;
    uint32_t interrupts = READ_REG(hcan->Instance->CONFIG0) & 0xFF000000;
    uint32_t itmask = READ_REG(hcan->Instance->CONFIG1) & 0xFF;
    uint32_t srflags = ((READ_REG(hcan->Instance->CONFIG0))>>16) & 0xFF;
    uint32_t eccflags = READ_REG(hcan->Instance->ERRCR) & 0xFF;

  
    /* CAN Receive Interrupt -------------------------------------------------*/
    if (((itmask & CAN_IT_MASK_RECEIVE) != RESET) && ((interrupts & CAN_IT_RECEIVE) != RESET))
    {

        HAL_CAN_RxCallback(hcan);

        __HAL_CAN_CLEAR_ITFLAG(hcan,CAN_IT_RECEIVE);  

        return;
    }    
    
    /* CAN Transmit Interrupt -------------------------------------------------*/
    if (((itmask & CAN_IT_MASK_TRANSMIT) != RESET) && ((interrupts & CAN_IT_TRANSMIT) != RESET))
    {

        HAL_CAN_TxCallback(hcan);

        __HAL_CAN_CLEAR_ITFLAG(hcan,CAN_IT_TRANSMIT);  

        return;
    }         

    /* Error interrupts management  */
    if((((itmask) & (uint32_t) (CAN_IT_MASK_ARBITRATION_LOSS | CAN_IT_MASK_ERROR_WARNING | CAN_IT_MASK_ERROR_PASSIVE | CAN_IT_MASK_BUSS_ERROR | CAN_IT_MASK_RECEIVE_OVERFLOW)) != RESET) 
        && (((interrupts) & (uint32_t)(CAN_IT_ARBITRATION_LOSS | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSS_ERROR | CAN_IT_MASK_RECEIVE_OVERFLOW)) != RESET))
    {
         /* Arbitration loss interrupt */
        if (((itmask & CAN_IT_MASK_ARBITRATION_LOSS) != RESET) && ((interrupts & CAN_IT_ARBITRATION_LOSS) != RESET))
        {
            errorcode |= HAL_CAN_ERROR_AL;
            __HAL_CAN_CLEAR_ITFLAG(hcan,CAN_IT_ARBITRATION_LOSS);                 
        }
        /* Protocol Error Warning interrupt */
        if (((itmask & CAN_IT_MASK_ERROR_WARNING) != RESET) && ((interrupts & CAN_IT_ERROR_WARNING) != RESET))
        {
            errorcode |= HAL_CAN_ERROR_EW;
            __HAL_CAN_CLEAR_ITFLAG(hcan,CAN_IT_ERROR_WARNING);                
        }       
        /* Error passive interrupt */
        if (((itmask & CAN_IT_MASK_ERROR_PASSIVE) != RESET) && ((interrupts & CAN_IT_ERROR_PASSIVE) != RESET))
        {
            errorcode |= HAL_CAN_ERROR_EP;
            __HAL_CAN_CLEAR_ITFLAG(hcan,CAN_IT_ERROR_PASSIVE);                
        }  
        /* Bus error interrupt */
        if (((itmask & CAN_IT_MASK_BUSS_ERROR) != RESET) && ((interrupts & CAN_IT_BUSS_ERROR) != RESET))
        {
            errorcode |= HAL_CAN_ERROR_BE;
            
             /* Check Last Error Code Flag */
            switch (eccflags & CAN_ERRCR_ECC)
            {
                case (CAN_ERRCR_BER):
                        /* Set CAN error code to Bit error  */
                       errorcode |= HAL_CAN_ERROR_BER;
                       break;
                case (CAN_ERRCR_STFER):
                        /* Set CAN error code to Stuff error  */
                        errorcode |= HAL_CAN_ERROR_STFER;
                        break;
                case (CAN_ERRCR_CRCER):
                        /* Set CAN error code to CRC error  */
                        errorcode |= HAL_CAN_ERROR_CRCER;
                        break;
                case (CAN_ERRCR_FRMER):
                        /* Set CAN error code to Form error  */
                        errorcode |= HAL_CAN_ERROR_FRMER;
                        break;
                case (CAN_ERRCR_ACKER):
                        /* Set CAN error code to ACK error  */
                        errorcode |= HAL_CAN_ERROR_ACKER;
                        break;
                case (CAN_ERRCR_EDIR):
                        /* Indicates Receive while error occurred  */
                        errorcode |= HAL_CAN_ERROR_EDIR;
                        break;
                case (CAN_ERRCR_TXWRN):
                        /* Set CAN error code to TXWRN  */
                        errorcode |= HAL_CAN_ERROR_TXWRN;
                        break;
                case (CAN_ERRCR_RXWRN):
                        /* Set CAN error code to RXWRN  */
                        errorcode |= HAL_CAN_ERROR_RXWRN;
                        break;
                default:
                        break;                
            
            } 
            /* Clear Bus error Flag */            
            __HAL_CAN_CLEAR_ITFLAG(hcan,CAN_IT_BUSS_ERROR);                
        }  

        /* Received data overflow interrupt */
        if (((itmask & CAN_IT_MASK_RECEIVE_OVERFLOW) != RESET) && ((interrupts & CAN_IT_RECEIVE_OVERFLOW) != RESET))
        {
            errorcode |= HAL_CAN_ERROR_RX_DO;
            __HAL_CAN_CLEAR_ITFLAG(hcan,CAN_IT_RECEIVE_OVERFLOW);                
        } 
    }    

    /* Call the Error call Back in case of Errors */
    if (errorcode != HAL_CAN_ERROR_NONE)
    {
        /* Update error code in handle */
        hcan->ErrorCode |= errorcode;
        /* Call weak (surcharged) callback */
        HAL_CAN_ErrorCallback(hcan);

    }    
      
}

/**
  * @brief  Error CAN callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_ErrorCallback could be implemented in the user file
   */
}

 /**
  * @brief  Rx callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_RxCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_RxCallback could be implemented in the
            user file
    */
}   
 /**
  * @brief  Tx callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
__weak void HAL_CAN_TxCallback(CAN_HandleTypeDef *hcan)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(hcan);

    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CAN_TxCallback could be implemented in the
            user file
    */
} 
/**
  * @brief  Get an CAN frame from the Rx FIFO zone into the message RAM.
  * @param  hcan pointer to an CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  pHeader pointer to a CAN_RxHeaderTypeDef structure where the header
  *         of the Rx frame will be stored.
  * @param  aData array where the payload of the Rx frame will be stored.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
    HAL_CAN_StateTypeDef state = hcan->State;
    
    /* Check CAN handle */
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }

    if ((state == HAL_CAN_STATE_READY) ||(state == HAL_CAN_STATE_LISTENING))
    {
        pHeader->pRxBuffer[0] = hcan->Instance->RXBUF;
       
        /* Get the IDE */
        pHeader->IDE = (pHeader->pRxBuffer[0] & 0x80) >> 7U;
        
        /* Get the RTR */
        pHeader->RTR = (pHeader->pRxBuffer[0] & 0x40) >> 6U;
        
        /* Get the DLC */
        pHeader->DLC = (pHeader->pRxBuffer[0] & 0xF);    
     
        if(pHeader->IDE == CAN_ID_STD)
        {
            /* Get the ID */
            pHeader->StdId = (uint16_t)((((pHeader->pRxBuffer[0] >> 8U) & 0xFF) << 3U ) | (((pHeader->pRxBuffer[0] >> 21U) & 0x7)));
            
            /*Get data[1]~ data[pHeader->DLC - 1]*/
            for(uint8_t i = 1 ;i <= CAN_BuffStdNum[pHeader->DLC - 1] ; i++)
            {     
                pHeader->pRxBuffer[i] = hcan->Instance->RXBUF;
            }   
            
            aData[0] = (uint8_t)((pHeader->pRxBuffer[0] >> 24U) & 0xFF);
            aData[1] = (uint8_t)(pHeader->pRxBuffer[1] & 0xFF);
            aData[2] = (uint8_t)((pHeader->pRxBuffer[1] >> 8U) & 0xFF);
            aData[3] = (uint8_t)((pHeader->pRxBuffer[1] >> 16U) & 0xFF);
            aData[4] = (uint8_t)((pHeader->pRxBuffer[1] >> 24U) & 0xFF);
            aData[5] = (uint8_t)(pHeader->pRxBuffer[2] & 0xFF);
            aData[6] = (uint8_t)((pHeader->pRxBuffer[2] >> 8U) & 0xFF);
            aData[7] = (uint8_t)((pHeader->pRxBuffer[2] >> 16U) & 0xFF); 

        }
        else
        {
            /* Get the ID */
//            pHeader->StdId = (uint16_t)((((pHeader->pRxBuffer[0] >> 8U) & 0xFF) << 21U ) | (((pHeader->pRxBuffer[0] >> 16U) & 0xFF) << 13U)
//                              | (((pHeader->pRxBuffer[0] >> 24U) & 0xFF) << 5U) | ((pHeader->pRxBuffer[0] >> 3U) & 0x1F));        
            
            /*Get data[1]~ data[pHeader->DLC - 1]*/
            for(uint8_t i = 1 ;i <= CAN_BuffExtdNum[pHeader->DLC - 1] ; i++)
            {     
                pHeader->pRxBuffer[i] = hcan->Instance->RXBUF;
            }  
            
//            aData[0] = (uint8_t)((pHeader->pRxBuffer[1] >> 8U) & 0xFF);
//            aData[1] = (uint8_t)((pHeader->pRxBuffer[1] >> 16U) & 0xFF);
//            aData[2] = (uint8_t)((pHeader->pRxBuffer[1] >> 24U) & 0xFF);
//            aData[3] = (uint8_t)(pHeader->pRxBuffer[2] & 0xFF);
//            aData[4] = (uint8_t)((pHeader->pRxBuffer[2] >> 8U) & 0xFF);
//            aData[5] = (uint8_t)((pHeader->pRxBuffer[2] >> 16U) & 0xFF);
//            aData[6] = (uint8_t)((pHeader->pRxBuffer[2] >> 24U) & 0xFF);
//            aData[7] = (uint8_t)(pHeader->pRxBuffer[3] & 0xFF); 
           pHeader->ExtId =(( pHeader->pRxBuffer[1] >>3U) &0X1F)          |
                           ((( pHeader->pRxBuffer[0] >>24U) & 0XFF)<<5U)  |
                           ((( pHeader->pRxBuffer[0] >>16U) & 0XFF)<<13U) |
                           ((( pHeader->pRxBuffer[0] >>8U) & 0XFF)<<21U) ;
        }
        /* Return function status */
        return HAL_OK;
    }
    else
    {
        return HAL_ERROR;
    }
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  Add a message to the pTxBuffer. 
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @param  pHeader pointer to a CAN_TxHeaderTypeDef structure.
  * @param  aData array containing the payload of the Tx frame.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
{
    uint32_t datatemp = 0;
    
    HAL_CAN_StateTypeDef state = hcan->State;

    /* Check CAN handle */
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }
    
    if ((state == HAL_CAN_STATE_READY) ||
    (state == HAL_CAN_STATE_LISTENING))
    {
        /* Set up the IDE */
        datatemp |= (pHeader->IDE << 7);
        /* Set up the DLC */
        datatemp |= pHeader->DLC;          
        
        if (pHeader->IDE == CAN_ID_STD)
        {
             /* Set RTR */
            datatemp |= (pHeader->RTR << 6) | (pHeader->RTR << 20) ;   
            /* Set ID */
            datatemp |= (((pHeader->StdId & 0x7F8)<< 5) | ((pHeader->StdId & 0x7)<< 21));   
 
            /* Set DATA Frame */ 
            if(pHeader->RTR == CAN_RTR_DATA)
            {
                /* DATA Frame */
                datatemp |= (aData[0] << 24);

                pHeader->pTxBuffer[0] = (uint32_t)datatemp;
                pHeader->pTxBuffer[1] = (uint32_t)((aData[1]) | (aData[2]<<8) | (aData[3]<<16) | (aData[4] <<24));
                pHeader->pTxBuffer[2] = (uint32_t)((aData[5]) | (aData[6]<<8) | (aData[7]<<16));          
            }
             /* Set Remote Frame */ 
            else
            {
                pHeader->pTxBuffer[0] = (uint32_t)datatemp;
            }
            
            for(uint8_t i = 0 ; i <= CAN_BuffStdNum[pHeader->DLC - 1]; i++)
            {
                /*Write data into the send fifo*/
                WRITE_REG(hcan->Instance->TXBUF,pHeader->pTxBuffer[i]);
            }  
        }
        /* EXT Frame */ 
        else
        {
             /* Set RTR */
            datatemp |= (pHeader->RTR << 6);   
            /* Set ID */
            datatemp |= ((pHeader->ExtId & 0x1FE00000)>> 13) | ((pHeader->ExtId & 0x1FE000)<< 3) | ((pHeader->ExtId & 0x1FE0)<< 19);
            
            pHeader->pTxBuffer[0] = (uint32_t)datatemp;
            
            /* Set DATA Frame */ 
            if(pHeader->RTR == CAN_RTR_DATA)
            {
                /* DATA Frame */
                pHeader->pTxBuffer[1] = (uint32_t)((pHeader->RTR << 2) | ((pHeader->ExtId & 0x1F) << 3) | (aData[0]<< 8) | (aData[1]<< 16) | (aData[2]<< 24));
                pHeader->pTxBuffer[2] = (uint32_t)((aData[3]) | (aData[4]<< 8) | (aData[5]<< 16) | (aData[6]<< 24));
                pHeader->pTxBuffer[3] = (uint32_t)(aData[7]);                
            }
             /* Set Remote Frame */ 
            else
            {
                pHeader->pTxBuffer[1] = (uint32_t)(pHeader->RTR << 2) | ((pHeader->ExtId & 0x1F) << 3);
            }
            
            for(uint8_t i = 0 ;i <= CAN_BuffExtdNum[pHeader->DLC -1]; i++)
            {
                /*Write data into the send fifo*/
                WRITE_REG(hcan->Instance->TXBUF,pHeader->pTxBuffer[i]);
            }    
        }
     
    return HAL_OK;      
        
    }
    else
    {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

        return HAL_ERROR;
    }
}

/**
  * @brief  Start CAN Transmit. 
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CAN_Tx_Start(CAN_HandleTypeDef *hcan)
{
    /* Check CAN handle */
    if (hcan == NULL)
    {
        return HAL_ERROR;
    }  

    __HAL_CAN_TRANSMIT_ENABLE(hcan);   
    
    /* Return function status */
    return HAL_OK;        

}

#endif /* HAL_CAN_MODULE_ENABLED */
/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
