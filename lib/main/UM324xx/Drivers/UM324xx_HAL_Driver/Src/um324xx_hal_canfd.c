/**
  ******************************************************************************
  * @file     um32x42x_hal_can.c 
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

/** @defgroup CANFD CANFD
  * @brief HAL CANFD module driver
  * @{
  */
#ifdef HAL_CANFD_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef HAL_CANFD_BaudConfig(CANFD_HandleTypeDef *hcan);
static HAL_StatusTypeDef CANFD_Receive_IT(CANFD_HandleTypeDef *hcan);

#ifdef HAL_DMA_MODULE_ENABLED
static void CANFD_DMATransmitCplt(DMA_HandleTypeDef *hdma);
#endif

const uint8_t CANFD_BuffStdNum[16]={0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 4, 5, 6, 8, 12, 16};
const uint8_t CANFD_BuffExtdNum[16]={1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 13, 17};
/** @addtogroup CANFD_Private_Functions  CANFD Private Functions
  * @{
  */
  
/**
  * @}
  */
  
/* Private functions ---------------------------------------------------------*/
/** @defgroup CANFD_Exported_Functions CANFD Exported Functions
  * @{
  */
HAL_StatusTypeDef HAL_CANFD_Init(CANFD_HandleTypeDef *hcan)
{
    /* Check Null pointer */
    if(hcan == NULL)
    {
        return HAL_ERROR;
    }

#if USE_HAL_CANFD_REGISTER_CALLBACKS == 1
  if (hcan->State == HAL_CANFD_STATE_RESET)
  {
    /* Reset callbacks to legacy functions */
    hcan->RxCpltCallback  =  HAL_CANFD_RxCpltCallback;        /* Legacy weak RxCpltCallback   */
    hcan->TxCpltCallback  =  HAL_CANFD_TxCpltCallback;        /* Legacy weak TxCpltCallback   */
    hcan->ErrorCallback   =  HAL_CANFD_ErrorCallback;         /* Legacy weak ErrorCallback    */

    if (hcan->MspInitCallback == NULL)
    {
      hcan->MspInitCallback = HAL_CANFD_MspInit; /* Legacy weak MspInit */
    }

    /* Init the low level hardware: CLOCK, NVIC */
    hcan->MspInitCallback(hcan);
  }

#else
  if (hcan->State == HAL_CANFD_STATE_RESET)
  {
    /* Init the low level hardware: CLOCK, NVIC */
    HAL_CANFD_MspInit(hcan);
  }
#endif /* (USE_HAL_CANFD_REGISTER_CALLBACKS) */

    __HAL_CANFD_RESET_ENABLE(hcan);

    /* Configuring baud rate */
    HAL_CANFD_BaudConfig(hcan);

    if(hcan->Init.CanLoopBack == ENABLE)
    {
        /* Configure the frame format, quorum baud rate, and data baud rate formats for CANFD*/
        MODIFY_REG(hcan->Instance->WUPTEST,CANFD_WUPTEST_TXC_Msk|CANFD_WUPTEST_LBEN_Msk, \
                                    hcan->Init.CanLoopBack|hcan->Init.TxdState);
    }
  
    if(hcan->Init.CanFrameFormat == CANFD_FRAME_FORMAT_FD)
    {
        /* Configure the frame format, quorum baud rate, and data baud rate formats for CANFD*/
        MODIFY_REG(hcan->Instance->CONFIG2,CANFD_CONFIG2_BRSEN_Msk|CANFD_CONFIG2_EXTBT_Msk|CANFD_CONFIG2_ISO_Msk, \
        hcan->Fd.CanfdFormatSelect|hcan->Fd.CanfdExtbtSelect|hcan->Fd.CanfdBaudRateSwitch);
    }

    /* Select the frame format of can*/
    MODIFY_REG(hcan->Instance->CONFIG2,CANFD_CONFIG2_FDEN_Msk,hcan->Init.CanFrameFormat);

    __HAL_CANFD_RXF_CLR(hcan);

    /* Clear flag bit */
    __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_Msk);
    
    WRITE_REG(hcan->Instance->AMR,CANFD_AMR_AMR3_0_Msk);
    
    MODIFY_REG(hcan->Instance->CONFIG0,CANFD_CONFIG0_LOM_Msk,hcan->Init.Mode);

    __HAL_CANFD_RESET_DISABLE(hcan);

    return HAL_OK;


}



 /**
  * @brief  Baud rate configuration of Can
  * @param  hcan Pointer to a CANFD_HandleTypeDef structure that contains
  *                the configuration information for the specified CANFD.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef HAL_CANFD_BaudConfig(CANFD_HandleTypeDef *hcan)
{
  /* Check Null pointer */
  if((hcan == NULL) || (hcan->Instance == NULL))
  {
    return HAL_ERROR;
  }
  if(((hcan->Init.CanFrameFormat == CANFD_FRAME_FORMAT_FD)&(hcan->Fd.CanfdExtbtSelect == CANFD_EXTBT_SELECT_BTR)) \
      || (hcan->Init.CanFrameFormat == CANFD_FRAME_FORMAT_NORMAL))
  {
    MODIFY_REG(hcan->Instance->CONFIG1,CANFD_CONFIG1_TSEG2_Msk|CANFD_CONFIG1_TSEG1_Msk|CANFD_CONFIG1_SJW_Msk|CANFD_CONFIG1_BRP_Msk,\
              ((uint32_t)(((hcan->Init.Prescaler - 1U) << CANFD_CONFIG1_BRP_Pos) | (hcan->Init.SyncJumpWidth) |\
                        (hcan->Init.TimeSeg1) | (hcan->Init.TimeSeg2))));
  }
  else
  { 
    /* Configure arbitration segment baud rate*/
    MODIFY_REG(hcan->Instance->NBT,CANFD_NBT_NBRP_Msk|CANFD_NBT_NSEG1_Msk|CANFD_NBT_NSEG2_Msk|CANFD_NBT_NSJM_Msk,\
              (((hcan->Fd.CanfdNbrp - 1) << CANFD_NBT_NBRP_Pos)|(hcan->Fd.CanfdNseg1 << CANFD_NBT_NSEG1_Pos)|\
                (hcan->Fd.CanfdNseg2 << CANFD_NBT_NSEG2_Pos)|(hcan->Fd.CanfdNsjm << CANFD_NBT_NSJM_Pos)));
      
    /* Configure data segment baud rate*/
    MODIFY_REG(hcan->Instance->DBTCR,CANFD_DBTCR_DBRP_Msk|CANFD_DBTCR_DSEG1_Msk|CANFD_DBTCR_DSEG2_Msk|CANFD_DBTCR_NSJM_Msk,\
              (((hcan->Fd.CanfdDbrp - 1U) << CANFD_DBTCR_DBRP_Pos)|\
                (hcan->Fd.CanfdDseg1 << CANFD_DBTCR_DSEG1_Pos)|\
                (hcan->Fd.CanfdDseg2 << CANFD_DBTCR_DSEG2_Pos)|\
                (hcan->Fd.CanfdDsjm << CANFD_DBTCR_NSJM_Pos)));   
  }

  return HAL_OK;

}


/**
  * @brief  loading the tx message into the can txfifo.  
  * @param  hcan pointer to a CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  pData array containing the payload of the Tx frame.
  *       
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK      nothing wrong
  *     @retval HAL_ERROR   something wrong
  */
HAL_StatusTypeDef HAL_CANFD_AddTxMessage(CANFD_HandleTypeDef *hcan, uint8_t pData[])
{
  uint32_t can_tx_temp = 0;

  /* Check Null pointer */
  if((hcan == NULL) || (hcan->Instance == NULL))
  {
    return HAL_ERROR;
  }
  hcan->TxHeader.TxData = pData;
  
  if(hcan->Init.CanFrameFormat != CANFD_FRAME_FORMAT_FD)
  {     
        /* Obtain DLC data length */ 
        hcan->TxHeader.DATALEN = hcan->TxHeader.DLC;
      
       /*set the parameter of  Dlc Ide Rtr*/
       can_tx_temp |= ((hcan->TxHeader.DLC) | (hcan->TxHeader.IDE << 7) | (hcan->TxHeader.RTR << 6) | (hcan->TxHeader.RTR << 20));
      
       /*Standard frame*/
      if (hcan->TxHeader.IDE == CANFD_ID_STD)
      {
         /*set the parameter of StdId*/
         can_tx_temp |= (((hcan->TxHeader.StdId & 0x7F8) << 5) | ((hcan->TxHeader.StdId & 0x7) << 21));
         /*Load data frame*/
         if(hcan->TxHeader.RTR == CANFD_RTR_DATA) 
         {
             hcan->TxHeader.pTxBuffer[0] = can_tx_temp | (pData[0] << 24);
             hcan->TxHeader.pTxBuffer[1] = ((pData[1]) | (pData[2] << 8) | (pData[3] << 16) | (pData[4] << 24));
             hcan->TxHeader.pTxBuffer[2] = ((pData[5]) | (pData[6] << 8) | (pData[7] << 16));          
         }
         /*Remote control frame*/
         else   
         {
             hcan->TxHeader.pTxBuffer[0] = can_tx_temp;
         }
      }
      /*Extended  frame*/
      else
      {
         /*set the parameter of StdId*/
         can_tx_temp |= (((hcan->TxHeader.ExtId & 0x1FE00000) >> 13) | ((hcan->TxHeader.ExtId & 0x1FE000) << 3) 
          | (((hcan->TxHeader.ExtId  & 0x1FE0) << 19)));
          /*Load data frame*/
         if(hcan->TxHeader.RTR == CANFD_RTR_DATA)
         {
            hcan->TxHeader.pTxBuffer[0] = can_tx_temp;
            hcan->TxHeader.pTxBuffer[1] = ((hcan->TxHeader.RTR << 2) | ((hcan->TxHeader.ExtId & 0x1F) << 3) | (pData[0] << 8) 
             | (pData[1] << 16)  | (pData[2] << 24));
             
            hcan->TxHeader.pTxBuffer[2] = ((pData[3]) | (pData[4] << 8) | (pData[5] << 16) | (pData[6] << 24));
            hcan->TxHeader.pTxBuffer[3] = (pData[7]);
         }
         /*Remote control frame*/
         else 
         {
            hcan->TxHeader.pTxBuffer[0] = can_tx_temp;
            hcan->TxHeader.pTxBuffer[1] = ((hcan->TxHeader.RTR << 2) | ((hcan->TxHeader.ExtId & 0x1F) << 3));

         }
      }
  }
  else   /* FD frame format packaging*/
  {
      hcan->TxHeader.FDF = CANFD_FRAME_FORMAT_FD;
      hcan->TxHeader.BRS = (hcan->Fd.CanfdBaudRateSwitch >> 1);  
      
      /*set the parameter of  Dlc Ide Rtr*/
      can_tx_temp |= ((hcan->TxHeader.DLC) | (hcan->TxHeader.IDE << 7) | (CANFD_FRAME_FORMAT_FD  << 5) |\
      ((hcan->Fd.CanfdBaudRateSwitch >> CANFD_CONFIG2_BRSEN_Pos) << 4));

      /*Standard frame*/
      if (hcan->TxHeader.IDE == CANFD_ID_STD)
      {
         if(hcan->TxHeader.DLC < 8)
            hcan->TxHeader.DATALEN = hcan->TxHeader.DLC;
         else
            hcan->TxHeader.DATALEN = CANFD_BuffStdNum[hcan->TxHeader.DLC]*4;

         /*set the parameter of StdId*/
         can_tx_temp |= (((hcan->TxHeader.StdId & 0x7F8) << 5) | ((hcan->TxHeader.StdId & 0x7) << 21));
         
         /*Load data frame*/
         if(hcan->TxHeader.DLC != 0)
         {
            hcan->TxHeader.pTxBuffer[0] = can_tx_temp | (pData[0] << 24);
             
            for(uint8_t i=0; i < CANFD_BuffStdNum[hcan->TxHeader.DLC] ; i++)
            {
                hcan->TxHeader.pTxBuffer[i+1] = ((pData[(4*i)+1]) | (pData[(4*i)+2] << 8) | (pData[(4*i)+3] << 16) | (pData[(4*i)+4] << 24));                    
            }   
         }
         else
         {
            hcan->TxHeader.pTxBuffer[0] = can_tx_temp;  
         }
      }
      /*Extended  frame*/
      else
      {
         if(hcan->TxHeader.DLC < 8)
            hcan->TxHeader.DATALEN = hcan->TxHeader.DLC;
         else
            hcan->TxHeader.DATALEN = (CANFD_BuffExtdNum[hcan->TxHeader.DLC]*4)-4;
         
         /*set the parameter of StdId*/
         can_tx_temp |= (((hcan->TxHeader.ExtId & 0x1FE00000) >> 13) | ((hcan->TxHeader.ExtId & 0x1FE000) << 3) 
          | (((hcan->TxHeader.ExtId  & 0x1FE0) << 19)));
          /*Load data frame*/
         if(hcan->TxHeader.DLC != 0)
         {
            hcan->TxHeader.pTxBuffer[0] = can_tx_temp;
            hcan->TxHeader.pTxBuffer[1] = ((hcan->TxHeader.RTR << 2) | ((hcan->TxHeader.ExtId & 0x1F) << 3) | (pData[0] << 8) 
             | (pData[1] << 16)  | (pData[2] << 24));

             for(uint8_t i=1; i < CANFD_BuffExtdNum[hcan->TxHeader.DLC]; i++)
             {
                  hcan->TxHeader.pTxBuffer[i+1] = ((pData[(4*i)-1]) | (pData[(4*i)] << 8) | (pData[(4*i)+1] << 16) | (pData[(4*i)+2] << 24));                    
             }
         }
         /*Remote control frame*/
         else
         {
            hcan->TxHeader.pTxBuffer[0] = can_tx_temp;
            hcan->TxHeader.pTxBuffer[1] = ((hcan->TxHeader.RTR << 2) | ((hcan->TxHeader.ExtId & 0x1F) << 3));

         }
      }
  }

  return HAL_OK;
}

/**
  * @brief  Write the CANFD frame to be sent through the CANFD network.  
  * @param  hcan pointer to a CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  Timeout Timeout duration.
  *       
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  *     @retval    HAL_TIMEOUT something wrong
  */
HAL_StatusTypeDef HAL_CANFD_Transmit(CANFD_HandleTypeDef *hcan, uint32_t Timeout)
{
    if((hcan == NULL) || (hcan->Instance == NULL))
    {
        return HAL_ERROR;
    }

    __HAL_LOCK(hcan);

    if(!__HAL_CANFD_GET_FLAG(hcan,CANFD_CONFIG0_TBS_Msk))
    {
        return HAL_ERROR;
    }

    __HAL_UNLOCK(hcan);   
    /*Select the buffer to send */
    MODIFY_REG(hcan->Instance->RTCONFIG,CANFD_RTCONFIG_SELTXSEL_Msk,hcan->TxHeader.SELTX);

    if(hcan->TxHeader.IDE == CANFD_ID_STD)
    {
        for(uint8_t i = 0 ;i<=CANFD_BuffStdNum[hcan->TxHeader.DLC]; i++)
        {
            /*Write data into the send fifo*/
            WRITE_REG(hcan->Instance->TXBUF,hcan->TxHeader.pTxBuffer[i]);
        }
    }
    else
    {
        for(uint8_t i = 0 ;i<=CANFD_BuffExtdNum[hcan->TxHeader.DLC] ; i++)
        {
            /*Write data into the send fifo*/
            WRITE_REG(hcan->Instance->TXBUF,hcan->TxHeader.pTxBuffer[i]);
        }
    }

#ifdef UM32x42x
    __disable_irq(); //diable_irq ÖÐ¶Ï

   __HAL_CANFD_RESET_ENABLE(hcan); //CANFD->CONFIG0 |= (0X1<<2) ; //RM =1
   __HAL_CANFD_RESET_DISABLE(hcan);// CANFD->CONFIG0 &= ~(0X1<<2) ; //RM =0

    /*Corresponding buffer abort transmission*/
    HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_TRAMSMIT_REQ);

    __enable_irq(); //enable_irq  ÖÐ¶Ï

#else
    /*Corresponding buffer abort transmission*/
    HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_TRAMSMIT_REQ);
#endif

    __HAL_LOCK(hcan);

    while(!__HAL_CANFD_GET_FLAG(hcan,CANFD_CONFIG0_TS))     
    {
        /**< Timeout wait exit*/
        if((Timeout--)==0)
        {
            __HAL_UNLOCK(hcan);
            /*Corresponding buffer abort transmission*/
            HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_ABORT_TRAMSMIT);

            return HAL_TIMEOUT;
        }
    }

    /*Corresponding buffer abort transmission*/
    HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_ABORT_TRAMSMIT);  
    
    __HAL_UNLOCK(hcan);  
    
    return HAL_OK;  

}

/**
  * @brief  CANFD TX buffer send command.  
  * @param  hcan pointer to a CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  CMD Send command.
  * @return None

  */
void HAL_CANFD_TXCMD(CANFD_HandleTypeDef *hcan, uint32_t CMD)
{
    uint32_t temp;
    
    temp = ((READ_REG(hcan->Instance->RTCONFIG) & CANFD_RTCONFIG_SELTXSEL_Msk) >> CANFD_RTCONFIG_SELTXSEL_Pos);
    
    switch (temp)
    {
    	case 0:
            MODIFY_REG(hcan->Instance->CONFIG0,CANFD_TXB0, (CMD<<CANFD_CONFIG0_CMD0_Pos));   
    		break;
    	case 1:
            MODIFY_REG(hcan->Instance->CONFIG0,CANFD_TXB1, (CMD<<CANFD_CONFIG0_CMD1_Pos));  
    		break;
    	default:
            MODIFY_REG(hcan->Instance->CONFIG0,CANFD_TXB2, (CMD<<CANFD_CONFIG0_CMD2_Pos));  
    		break;
    }
}


/**
  * @brief  Configures the CANFD reception filter according to the specified
  *         parameters in the CANFD_FilterInitStruct.
  * @param  hcan pointer to a CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  sFilterConfig pointer to a CANFD_FilterTypeDef structure that
  *         contains the filter configuration information.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_CANFD_ConfigFilter(CANFD_HandleTypeDef *hcan, CANFD_FilterTypeDef *sFilterConfig)
{
  uint32_t tempacr=0;

  if((hcan == NULL) || (hcan->Instance == NULL))
  {
    return HAL_ERROR;
      
  }

   /*CANFD operation reset mode*/    
  __HAL_CANFD_RESET_ENABLE(hcan);
  
  MODIFY_REG(hcan->Instance->RTCONFIG,CANFD_RTCONFIG_SEL,sFilterConfig->CHANNEL);   ///////        
  
  
  if(sFilterConfig->FilterMode == HAL_CANFD_FilterMode_SINGLE)
  {
      /*Use single filter*/
     __HAL_CANFD_SINGLE_FILTER(hcan);
      
     if(sFilterConfig->FilterIde == CANFD_ID_STD)
     {
        tempacr = (((sFilterConfig->FilterStdId1)&0x7f8) >> 3U) | (((sFilterConfig->FilterStdId1)&0x7) << 13U)
         | (((sFilterConfig->FilterRtr)) << 12U) | (((sFilterConfig->FilterData1)&0xFF) << 16U)
         | (((sFilterConfig->FilterData2)&0xFF) << 24U);
         
         WRITE_REG(hcan->Instance->ACR,tempacr);
     }
     
     if(sFilterConfig->FilterIde == CANFD_ID_EXT)
     {
        tempacr = (((sFilterConfig->FilterExtId1)&0x1FE00000) >> 21) | (((sFilterConfig->FilterExtId1)&0x1FE000) >> 5U)
         | (((sFilterConfig->FilterExtId1)&0x1FE0) << 11U) | (((sFilterConfig->FilterExtId1)&0x1F) << 27U)
         | (((sFilterConfig->FilterRtr)) << 26U) ;    
         
          WRITE_REG(hcan->Instance->ACR,tempacr);

     }
      WRITE_REG(hcan->Instance->AMR,sFilterConfig->FilterMaskType);
  }
  else if(sFilterConfig->FilterMode == HAL_CANFD_FilterMode_DOUBLE)
  {
     /*Use double filter*/
    __HAL_CANFD_DOUBLE_FILTER(hcan);
      
    if(sFilterConfig->FilterIde == CANFD_ID_STD)
     {
         /*filter1*/
        tempacr = (((sFilterConfig->FilterStdId1)&0x7f8) >> 3) | (((sFilterConfig->FilterStdId1)&0x7) << 13)
         | (((sFilterConfig->FilterRtr)) << 12) | (((sFilterConfig->FilterData1)&0xf0) << 4) 
         | (((sFilterConfig->FilterData1)&0xf) << 24) ;

        /*filter2*/         
        tempacr |= (((sFilterConfig->FilterStdId2)&0x7f8) << 13) | (((sFilterConfig->FilterStdId2)&0x7) << 29)
         | (((sFilterConfig->FilterRtr)) << 28);     
   
         WRITE_REG(hcan->Instance->ACR,tempacr);
     }
     if(sFilterConfig->FilterIde == CANFD_ID_EXT)
     {
        /*filter1*/
        tempacr = (((sFilterConfig->FilterExtId1)&0x1FE00000) >> 21) | (((sFilterConfig->FilterExtId1)&0x1FE000) >> 5);
        /*filter2*/    
        tempacr |= (((sFilterConfig->FilterExtId2)&0x1FE00000) >> 5) | (((sFilterConfig->FilterExtId2)&0x1FE000) << 11);   
         
        WRITE_REG(hcan->Instance->ACR,tempacr);

     }
     WRITE_REG(hcan->Instance->AMR,sFilterConfig->FilterMaskType);
  }
  else
  {
     hcan->Instance->AMR = CANFD_Filter_MASK_ALL;  /*!< No comparison, no filtering */
  }

  __HAL_CANFD_NORMAL_MODE(hcan);           /*!< CANFD enter normal mode */
  __HAL_CANFD_RESET_DISABLE(hcan);           /*!< CANFD exit normal mode */

  return HAL_OK;                      
}



/**
  * @brief  Get an CANFD frame from the Rx FIFO zone into the message RAM.
  * @param  hcan pointer to an CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  Timeout Timeout duration.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK      nothing wrong
  *     @retval HAL_ERROR   something wrong
  *     @retval HAL_TIMEOUT timeout
  */
HAL_StatusTypeDef HAL_CANFD_Receive(CANFD_HandleTypeDef *hcan, uint32_t Timeout)
{
    uint8_t tempdlc;
    uint8_t tempide;  
    
    if((hcan == NULL) || (hcan->Instance == NULL))
    {
        return HAL_ERROR;
    }

   
   while(!__HAL_CANFD_GET_FLAG(hcan,CANFD_CONFIG0_RI))
    {
        /**< Timeout wait exit*/
        if((Timeout--)==0)   
        {
            return HAL_TIMEOUT;
        }
    }
    __HAL_CANFD_CLEAR_ITFLAG(hcan, CANFD_CONFIG0_RI);
    
    /*Receive the first rxdata*/
    hcan->RxHeader.pRxBuffer[0] = hcan->Instance->RXBUF;

    tempdlc = hcan->RxHeader.pRxBuffer[0] & 0xF;
    hcan->RxHeader.DLC = tempdlc;
    
    tempide = hcan->RxHeader.pRxBuffer[0] & 0x80;
    hcan->RxHeader.IDE = (tempide >> 0x7);   
    
   if(hcan->RxHeader.RTR ==  CANFD_RTR_DATA)////
   {
       if(hcan->RxHeader.IDE)
    {
        for(uint8_t i = 1 ;i <= CANFD_BuffExtdNum[tempdlc] ; i++)
        {
            /*Write data into the send fifo*/
            hcan->RxHeader.pRxBuffer[i] = hcan->Instance->RXBUF;          
        }    
    }
    else
    {
        for(uint8_t i = 1 ;i <= CANFD_BuffStdNum[tempdlc] ; i++)
        {
            /*Write data into the send fifo*/
            hcan->RxHeader.pRxBuffer[i] = hcan->Instance->RXBUF;
        }        
    }    
  }

  return HAL_OK;  
}


/**
  * @brief  Receives an amount of data in non blocking mode.
  * @param  hcan Pointer to a CANFD_HandleTypeDef structure that contains
  *               the configuration information for the specified CANFD module.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK      nothing wrong
  *     @retval HAL_ERROR   something wrong
  */
HAL_StatusTypeDef HAL_CANFD_Receive_IT(CANFD_HandleTypeDef *hcan)
{
    if((hcan == NULL) || (hcan->Instance == NULL))
    {
        return HAL_ERROR;
    }
    
    __HAL_CANFD_ENABLE_IT(hcan,CANFD_IT_ARBITRATION_LOST);
    
    __HAL_CANFD_ENABLE_IT(hcan,CANFD_IT_ERROR_WARNING);        
    
    __HAL_CANFD_ENABLE_IT(hcan,CANFD_IT_ERROR_PASSIVE);   

    __HAL_CANFD_ENABLE_IT(hcan,CANFD_IT_RECEIVE);

    __HAL_CANFD_ENABLE_IT(hcan,CANFD_IT_BUS_ERROR);   
    
    __HAL_CANFD_ENABLE_IT(hcan,CANFD_IT_RECEIVE_OVF);     

    
    return HAL_OK;

}


/**
  * @brief  Receives an amount of data in non blocking mode
  * @param  hcan  Pointer to a CANFD_HandleTypeDef structure that contains
  *                the configuration information for the specified CANFD module.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK      nothing wrong
  *     @retval HAL_ERROR   something wrong
  */
static HAL_StatusTypeDef CANFD_Receive_IT(CANFD_HandleTypeDef *hcan)
{
    uint8_t tempdlc;
    uint8_t tempide;  

    hcan->RxHeader.pRxBuffer[0] = hcan->Instance->RXBUF;

    tempdlc = hcan->RxHeader.pRxBuffer[0] & 0xF;
    hcan->RxHeader.DLC = tempdlc;

    tempide = hcan->RxHeader.pRxBuffer[0] & 0x80;
    hcan->RxHeader.IDE = (tempide >> 0x7);
    
    if(hcan->RxHeader.IDE)
    {
        for(uint8_t i = 1 ;i <= CANFD_BuffExtdNum[tempdlc] ; i++)
        {
            /*Write data into the send fifo*/
            hcan->RxHeader.pRxBuffer[i] = hcan->Instance->RXBUF;    
        }    
    }
    else
    {
        for(uint8_t i = 1 ;i <= CANFD_BuffStdNum[tempdlc] ; i++)
        {
            /*Write data into the send fifo*/
            hcan->RxHeader.pRxBuffer[i] = hcan->Instance->RXBUF;
        }        
    }    
/* Standard reception API called */
#if (USE_HAL_CANFD_REGISTER_CALLBACKS == 1)
                /*Call registered Rx complete callback*/
                hcan->RxCpltCallback(hcan);
#else
                /*Call legacy weak Rx complete callback*/
                HAL_CANFD_RxCpltCallback(hcan);
#endif /* USE_HAL_CANFD_REGISTER_CALLBACKS */   
    
        __HAL_CANFD_DISABLE_IT(hcan,CANFD_IT_ARBITRATION_LOST);
        
        __HAL_CANFD_DISABLE_IT(hcan,CANFD_IT_ERROR_WARNING);        
        
        __HAL_CANFD_DISABLE_IT(hcan,CANFD_IT_ERROR_PASSIVE);   

        __HAL_CANFD_DISABLE_IT(hcan,CANFD_IT_RECEIVE);   

        __HAL_CANFD_DISABLE_IT(hcan,CANFD_IT_BUS_ERROR);   
        
        __HAL_CANFD_DISABLE_IT(hcan,CANFD_IT_RECEIVE_OVF);
            
        return HAL_OK;
}


/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  hcan Pointer to a CANFD_HandleTypeDef structure that contains
  *               the configuration information for the specified CANFD module.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK      nothing wrong
  *     @retval HAL_ERROR   something wrong
  */
HAL_StatusTypeDef HAL_CANFD_Transmit_IT(CANFD_HandleTypeDef *hcan)
{
    if((hcan == NULL) || (hcan->Instance == NULL))
    {
        return HAL_ERROR;
    }
  
    if(!__HAL_CANFD_GET_FLAG(hcan,CANFD_CONFIG0_TBS_Msk))
    {
        return HAL_ERROR;
    }

    /*Select the buffer to send */
    MODIFY_REG(hcan->Instance->RTCONFIG,CANFD_RTCONFIG_SELTXSEL_Msk,hcan->TxHeader.SELTX);

    /* Enable the CANFD Completion of sending Interrupt */    
    __HAL_CANFD_ENABLE_IT(hcan, CANFD_IT_TRANSFER);
    __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_TRANSFER);   

    if(hcan->TxHeader.IDE == CANFD_ID_STD)
    {
        for(uint8_t i = 0 ;i<=CANFD_BuffStdNum[hcan->TxHeader.DLC]; i++)
        {
            /*Write data into the send fifo*/
            WRITE_REG(hcan->Instance->TXBUF,hcan->TxHeader.pTxBuffer[i]);
        }    
    }
    else
    {
        for(uint8_t i = 0 ;i<=CANFD_BuffExtdNum[hcan->TxHeader.DLC] ; i++)
        {
            /*Write data into the send fifo*/
            WRITE_REG(hcan->Instance->TXBUF,hcan->TxHeader.pTxBuffer[i]);
        }              
    }

    /*Corresponding buffer abort transmission*/
    HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_SINGLE_TRAMSMIT_REQ);

    return HAL_OK;
 

}


/**
  * @brief  Get an CANFD frame from the Rx FIFO zone into the message RAM.
  * @param  hcan pointer to an CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @param  pData array where the payload of the Rx frame will be stored.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK      nothing wrong
  *     @retval HAL_ERROR   something wrong
  */
HAL_StatusTypeDef HAL_CANFD_GetRxMessage(CANFD_HandleTypeDef *hcan, uint8_t* pData)
{
  uint8_t Length = 0;
   
  if((hcan == NULL) || (hcan->Instance == NULL))
  {
    return HAL_ERROR;
  }

  hcan->RxHeader.RxData = pData;

  if(hcan->Init.CanFrameFormat != CANFD_FRAME_FORMAT_FD)
  {
      /* Obtain DLC data length */
      hcan->RxHeader.DATALEN = hcan->RxHeader.DLC;

      if(hcan->RxHeader.IDE == CANFD_ID_STD) 
      {
          hcan->RxHeader.RTR = (uint32_t)((hcan->RxHeader.pRxBuffer[0] & 0x40)>>6);

           /*get ID value*/
          hcan->RxHeader.StdId = (((hcan->RxHeader.pRxBuffer[0])&0xFF00)>>5) | (((hcan->RxHeader.pRxBuffer[0])&0xE00000)>>21);     
          pData[0] = (uint8_t)(((hcan->RxHeader.pRxBuffer[0])&0xFF000000) >> 24);
     
          if(hcan->RxHeader.RTR ==  CANFD_RTR_DATA)////
          {
              /*get the data value*/
          if(hcan->RxHeader.DLC >= 6)
          {
               pData[1] = (uint8_t)((hcan->RxHeader.pRxBuffer[1])&0xFF);
               pData[2] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1]) >> 8)&0xFF);
               pData[3] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1]) >> 16)&0xFF);
               pData[4] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1]) >> 24)&0xFF);

               pData[5] = (uint8_t)((hcan->RxHeader.pRxBuffer[2])&0xFF);
               pData[6] = (uint8_t)(((hcan->RxHeader.pRxBuffer[2]) >> 8)&0xFF);
               pData[7] = (uint8_t)(((hcan->RxHeader.pRxBuffer[2]) >> 16)&0xFF);
          }
          else
          {
               pData[1] = (uint8_t)((hcan->RxHeader.pRxBuffer[1])&0xFF);
               pData[2] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1]) >> 8)&0xFF);
               pData[3] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1]) >> 16)&0xFF);
               pData[4] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1]) >> 24)&0xFF);

          }
         }
      }
      /*Extended  frame*/
      else
      {        
          hcan->RxHeader.RTR = (uint32_t)((hcan->RxHeader.pRxBuffer[1] & 0x4)>>2);  
          
           /*get ID value*/
          hcan->RxHeader.ExtId = ((((hcan->RxHeader.pRxBuffer[0])&0xFF00) << 13) | (((hcan->RxHeader.pRxBuffer[0])&0xFF0000) >> 3) 
          | (((hcan->RxHeader.pRxBuffer[0]) & 0xFF000000) >> 19) | (((hcan->RxHeader.pRxBuffer[1]) & 0xF8) >> 3));    
       if(hcan->RxHeader.RTR ==  CANFD_RTR_DATA)////
          {
           /*get the data value*/
          pData[0] = (uint8_t)((hcan->RxHeader.pRxBuffer[1] >> 8)&0xFF);
          pData[1] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1]) >> 16) & 0xFF);
          pData[2] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1]) >> 24) & 0xFF);
          
          if(hcan->RxHeader.DLC >= 8)
          {

               pData[3] = (uint8_t)((hcan->RxHeader.pRxBuffer[2])&0xFF);
               pData[4] = (uint8_t)(((hcan->RxHeader.pRxBuffer[2]) >> 8)&0xFF);
               pData[5] = (uint8_t)(((hcan->RxHeader.pRxBuffer[2]) >> 16)&0xFF);
               pData[6] = (uint8_t)(((hcan->RxHeader.pRxBuffer[2]) >> 24)&0xFF);

               pData[7] = (uint8_t)((hcan->RxHeader.pRxBuffer[3])&0xFF);

          }
          else if(hcan->RxHeader.DLC >= 4)
          {
               pData[3] = (uint8_t)((hcan->RxHeader.pRxBuffer[2])&0xFF);
               pData[4] = (uint8_t)(((hcan->RxHeader.pRxBuffer[2]) >> 8)&0xFF);
               pData[5] = (uint8_t)(((hcan->RxHeader.pRxBuffer[2]) >> 16)&0xFF);
               pData[6] = (uint8_t)(((hcan->RxHeader.pRxBuffer[2]) >> 24)&0xFF);
          }
         }
      }
  }
  else  /* Obtain CANFD frame format data*/
  {
      hcan->RxHeader.FDF = ((hcan->RxHeader.pRxBuffer[0] & 0x20)>> 5);

      if(hcan->RxHeader.IDE == CANFD_ID_STD) 
      {
            if(hcan->RxHeader.DLC<8)
            hcan->RxHeader.DATALEN = hcan->RxHeader.DLC;
            else
            hcan->RxHeader.DATALEN = (CANFD_BuffStdNum[hcan->RxHeader.DLC]*4);
  
               /*get ID value*/
            hcan->RxHeader.StdId = (((hcan->RxHeader.pRxBuffer[0])&0xFF00)>>5) | (((hcan->RxHeader.pRxBuffer[0])&0xE00000)>>21); 
            pData[0] = (uint8_t)(((hcan->RxHeader.pRxBuffer[0])&0xFF000000) >> 24);

            Length = CANFD_BuffStdNum[hcan->RxHeader.DLC];
            for(uint8_t i=1; i<=Length; i++)
            {
                 for(uint8_t j=1; j<=4; j++)
                 pData[j+((i-1)*4)] = (uint8_t)(((hcan->RxHeader.pRxBuffer[i]) >> (8*(j-1))) & 0xFF);
            }
      }
      /*Extended  frame*/
      else
      {
            if(hcan->RxHeader.DLC<8)
            hcan->RxHeader.DATALEN = hcan->RxHeader.DLC;
            else
            hcan->RxHeader.DATALEN = ((CANFD_BuffExtdNum[hcan->RxHeader.DLC]*4)-4);
           
            /*get ID value*/
            hcan->RxHeader.ExtId = ((((hcan->RxHeader.pRxBuffer[0])&0xFF00) << 13) | (((hcan->RxHeader.pRxBuffer[0])&0xFF0000) >> 3) 
            | (((hcan->RxHeader.pRxBuffer[0]) & 0xFF000000) >> 19) | (((hcan->RxHeader.pRxBuffer[1]) & 0xF8) >> 3));    
        
            pData[0] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1])&0xFF00) >> 8);
            pData[1] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1])&0xFF0000) >> 16);
            pData[2] = (uint8_t)(((hcan->RxHeader.pRxBuffer[1])&0xFF000000) >> 24);
     
            Length = CANFD_BuffExtdNum[hcan->RxHeader.DLC];
            
            for(uint8_t i=2; i<=Length; i++)
            {
                for(uint8_t j=3; j<=6; j++)
                {
                    pData[j+((i-2)*4)] = (uint8_t)(((hcan->RxHeader.pRxBuffer[i]) >> (8*(j-3))) & 0xFF);
                }
            }
      }
  }
  return HAL_OK;
}

#ifdef HAL_DMA_MODULE_ENABLED
/**
  * @brief  Sends an amount of data in DMA mode.
  * @param  hcan Pointer to a CANFD_HandleTypeDef structure that contains
  *              the configuration information for the specified CANFD module.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CANFD_Transmit_DMA(CANFD_HandleTypeDef *hcan)
{
    if((hcan == NULL) || (hcan->Instance == NULL))
    {
        return HAL_ERROR;
    }

    __HAL_LOCK(hcan);

    if(!__HAL_CANFD_GET_FLAG(hcan,CANFD_CONFIG0_TBS_Msk))
    {
        return HAL_ERROR;
    }
    /* Set the CANFD DMA transfer complete callback */
    hcan->hdmatx->XferTfrCallback = CANFD_DMATransmitCplt;
    hcan->hdmatx->XferSrcTranCallback = NULL;
    hcan->hdmatx->XferBlockCallback = NULL;
    hcan->hdmatx->XferDstTranCallback = NULL;
    hcan->hdmatx->XferErrorCallback = NULL;

    __HAL_UNLOCK(hcan);

    __HAL_CANFD_RESET_ENABLE(hcan);
    /* Enable DMA mode */
    SET_BIT(hcan->Instance->CONFIG0,CANFD_CONFIG0_DMA);

    __HAL_CANFD_RESET_DISABLE(hcan);
    
    /* Select the buffer to send */
    MODIFY_REG(hcan->Instance->RTCONFIG,CANFD_RTCONFIG_SELTXSEL_Msk,hcan->TxHeader.SELTX);

    if(hcan->TxHeader.IDE == CANFD_ID_STD)
    {
        HAL_DMA_Start_IT(hcan->hdmatx,(uint32_t)&(hcan->TxHeader.pTxBuffer),(uint32_t)&hcan->Instance->TXBUF,\
            CANFD_BuffStdNum[hcan->TxHeader.DLC]+1);
    }
    else
    {
        HAL_DMA_Start_IT(hcan->hdmatx,(uint32_t)&(hcan->TxHeader.pTxBuffer),(uint32_t)&hcan->Instance->TXBUF,\
            CANFD_BuffExtdNum[hcan->TxHeader.DLC]+1);
    }
    
    /* Single transmission command*/
    HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_SINGLE_TRAMSMIT_REQ);
    
    return HAL_OK;

}



/**
  * @brief  DMA CANFD transmit process complete callback.
  * @param  hdma  Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void CANFD_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
  CANFD_HandleTypeDef *hcan = (CANFD_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  __HAL_DMA_DISABLE_DSTTRAN_IT(hdma,hdma->DmaChannelSel);
  __HAL_DMA_CLEAR_DSTTRAN_FLAG(hdma, hdma->DmaChannelSel);

  /* DMA Normal mode*/
  if ((*(&(hdma->Instance->CFG0)+ CHANNEL_OFFSET*hdma->DmaChannelSel) & DMA_CFG0_RELOAD_DST) == 0U)
  {    
    __HAL_CANFD_RESET_ENABLE(hcan);
    /* Disable DMA mode */
    CLEAR_BIT(hcan->Instance->CONFIG0,CANFD_CONFIG0_DMA);

    __HAL_CANFD_RESET_DISABLE(hcan);

    /*Corresponding buffer abort transmission*/
    HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_ABORT_TRAMSMIT);   
      
    __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_TRANSFER);       
    /* Enable the CANFD Transmit Complete Interrupt */
    __HAL_CANFD_ENABLE_IT(hcan, CANFD_IT_TRANSFER);
  }
  /* DMA Circular mode */
  else
  {
    /*Corresponding buffer abort transmission*/
    HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_ABORT_TRAMSMIT);   
      
#if (USE_HAL_CANFD_REGISTER_CALLBACKS == 1)
    /*Call registered Tx complete callback*/
    hcan->TxCpltCallback(hcan);
#else
    /*Call legacy weak Tx complete callback*/
    HAL_CANFD_TxCpltCallback(hcan);
#endif /* USE_HAL_CANFD_REGISTER_CALLBACKS */
  }
}





#endif

/**
  * @brief  This function handles CANFD interrupt request.
  * @param  hcan  Pointer to a CANFD_HandleTypeDef structure that contains
  *                the configuration information for the specified CANFD module.
  * @retval None
  */
void HAL_CANFD_IRQHandler(CANFD_HandleTypeDef *hcan)
{
  uint32_t config0flags = READ_REG(hcan->Instance->CONFIG0) & 0xFF000000;
  uint32_t config1imits = READ_REG(hcan->Instance->CONFIG1) & 0xFF;
  uint32_t errorflags = 0x00U;
    
  errorflags = config0flags & (uint32_t)(CANFD_IT_FLAG_ARBITRATION_LOST|CANFD_IT_FLAG_ERROR_WARNING| \
                        CANFD_IT_FLAG_ERROR_PASSIVE|CANFD_IT_FLAG_BUS_ERROR|CANFD_IT_FLAG_RECEIVE_OVF);
  
  if(errorflags == RESET)    
  {
    /* CANFD in mode Receiver -------------------------------------------------*/
    if (((config0flags & CANFD_IT_FLAG_RECEIVE) != RESET) && ((config1imits & CANFD_IT_RECEIVE) != RESET))
    {

      CANFD_Receive_IT(hcan);

      __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_RECEIVE);  
        
      return;
    }      
  }
    /* If some errors occur */
    if ((errorflags != RESET) && ((config1imits & (CANFD_IT_ARBITRATION_LOST | CANFD_IT_ERROR_WARNING | CANFD_IT_ERROR_PASSIVE|\
        CANFD_IT_BUS_ERROR | CANFD_IT_RECEIVE_OVF)) != RESET))
    {
         /* CANFD Arbitration lost error interrupt ----------------------------------*/
        if (((config0flags & CANFD_IT_FLAG_ARBITRATION_LOST) != RESET) && ((config1imits & CANFD_IT_ARBITRATION_LOST) != RESET))
        {
            hcan->ErrorCode |= HAL_CANFD_ERROR_ALI;
            __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_ARBITRATION_LOST);     
            
        }
        
        /* CANFD Error warning interrupt --------------------------------------------*/
        if (((config0flags & CANFD_IT_FLAG_ERROR_WARNING) != RESET) && (((config1imits & CANFD_IT_ERROR_WARNING) != RESET)))
        {
            hcan->ErrorCode |= HAL_CANFD_ERROR_EWI;
            __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_ERROR_WARNING);               
        }

        /* CANFD Error Passive Interrupt -----------------------------------------------*/
        if (((config0flags & CANFD_IT_FLAG_ERROR_PASSIVE) != RESET) && ((config1imits & CANFD_IT_ERROR_PASSIVE) != RESET))
        {
            hcan->ErrorCode |= HAL_CANFD_ERROR_EPI;
            __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_ERROR_PASSIVE);   
        }
        
        /* CANFD Bus error interrupt ---------------------------------------------------*/
        if (((config0flags & CANFD_IT_FLAG_BUS_ERROR) != RESET) && ((config1imits & CANFD_IT_BUS_ERROR) != RESET))
        {
            hcan->ErrorCode |= HAL_CANFD_ERROR_BEI;
            __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_BUS_ERROR);               
        }        
   
        /* CANFD Receive data overflow interrupt ----------------------------------------*/
        if (((config0flags & CANFD_IT_FLAG_RECEIVE_OVF) != RESET) && ((config1imits & CANFD_IT_RECEIVE_OVF) != RESET))
        {
            hcan->ErrorCode |= HAL_CANFD_ERROR_DOI;
            __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_RECEIVE_OVF);             
        }   

          /* Error is notified to user through user error callback */
#if (USE_HAL_CANFD_REGISTER_CALLBACKS == 1)
            /*Call registered error callback*/
            hcan->ErrorCallback(hcan);
#else
            /*Call legacy weak error callback*/
            HAL_CANFD_ErrorCallback(hcan);
#endif /* USE_HAL_CANFD_REGISTER_CALLBACKS */

            hcan->ErrorCode = HAL_CANFD_ERROR_NONE;
  
        return;
    } /* End if some error occurs */

    /* CANFD in mode Transmitter end --------------------------------------------*/
    if (((config0flags & CANFD_IT_FLAG_TRANSFER) != RESET) && ((config1imits & CANFD_IT_TRANSFER) != RESET))
    {
       /*Terminate transmission*/
       HAL_CANFD_TXCMD(hcan,CANFD_TXB_CMD_ABORT_TRAMSMIT);
        
      __HAL_CANFD_CLEAR_ITFLAG(hcan,CANFD_IT_FLAG_TRANSFER);  
          /*  User through Completion of sending callback */
#if (USE_HAL_CANFD_REGISTER_CALLBACKS == 1)
            /*Call Completion of sending callback*/
            hcan->TxCpltCallback(hcan);
#else
            /*Call Completion of sending callback*/
            HAL_CANFD_TxCpltCallback(hcan);
#endif /* USE_HAL_CANFD_REGISTER_CALLBACKS */
        
    }

}


/**
  * @brief  Initializes the CANFD MSP.
  * @param  hcan pointer to a CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @retval None
  */
__weak void HAL_CANFD_MspInit(CANFD_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CANFD_MspInit could be implemented in the user file
   */
}


/**
  * @brief  DeInitializes the CANFD MSP.
  * @param  hcan pointer to a CANFD_HandleTypeDef structure that contains
  *         the configuration information for the specified CANFD.
  * @retval None
  */
__weak void HAL_CANFD_MspDeInit(CANFD_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_CANFD_MspDeInit could be implemented in the user file
   */
}


/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hcan  Pointer to a CANFD_HandleTypeDef structure that contains
  *                the configuration information for the specified CANFD module.
  * @retval None
  */
__weak void HAL_CANFD_TxCpltCallback(CANFD_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
}


/**
  * @brief  Rx Transfer completed callbacks.
  * @param  hcan  Pointer to a CANFD_HandleTypeDef structure that contains
  *                the configuration information for the specified CANFD module.
  * @retval None
  */
__weak void HAL_CANFD_RxCpltCallback(CANFD_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_CANFD_RxCpltCallback could be implemented in the user file
  */
}


/**
  * @brief  CANFD error callbacks.
  * @param  hcan  Pointer to a CANFD_HandleTypeDef structure that contains
  *                the configuration information for the specified CANFD module.
  * @retval None
  */
__weak void HAL_CANFD_ErrorCallback(CANFD_HandleTypeDef *hcan)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcan);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_CANFD_ErrorCallback could be implemented in the user file
  */
}



#if (USE_HAL_CANFD_REGISTER_CALLBACKS == 1)
/**
  * @brief Register a User CANFD callback to be used instead of the weak predefined callback
  * @param hcan can handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_CANFD_TX_COMPLETE_CB_ID CANFD Tx Complete ID
  *          @arg @ref HAL_CANFD_RX_COMPLETE_CB_ID CANFD Rx Complete ID
  *          @arg @ref HAL_CANFD_ERROR_CB_ID CANFD Error ID

  *          @param pCallback pointer to the callback function
  *          @retval status
  */
HAL_StatusTypeDef HAL_CANFD_RegisterCallback(CANFD_HandleTypeDef *hcan, HAL_CANFD_CallbackIDTypeDef CallbackID,
        pCANFD_CallbackTypeDef pCallback)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (pCallback == NULL)
    {
        return HAL_ERROR;
    }

    if (hcan->State == HAL_CANFD_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_CANFD_TX_COMPLETE_CB_ID :
            hcan->TxCpltCallback                    = pCallback;
            break;

        case HAL_CANFD_RX_COMPLETE_CB_ID :
            hcan->RxCpltCallback                    = pCallback;
            break;

        case HAL_CANFD_ERROR_CB_ID :
            hcan->ErrorCallback                     = pCallback;
            break;
        default :
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (hcan->State == HAL_CANFD_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_CANFD_TX_COMPLETE_CB_ID :
            hcan->TxCpltCallback                = pCallback;
            break;

        case HAL_CANFD_RX_COMPLETE_CB_ID :
            hcan->RxCpltCallback                = pCallback;
            break;

        case HAL_CANFD_ERROR_CB_ID :
            hcan->ErrorCallback            = pCallback;
            break;
        default :
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Return error status */
        status =  HAL_ERROR;
    }

    return status;
}

/**
  * @brief  Unregister a CANFD callback
  *         CANFD callback is redirected to the weak predefined callback
  * @param hcan can handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref HAL_CANFD_TX_COMPLETE_CB_ID CANFD Tx Complete ID
  *          @arg @ref HAL_CANFD_RX_COMPLETE_CB_ID CANFD Rx Complete ID
  *          @arg @ref HAL_CANFD_ERROR_CB_ID CANFD Error ID
  *          @retval status
  */
HAL_StatusTypeDef HAL_CANFD_UnRegisterCallback(CANFD_HandleTypeDef *hcan, HAL_CANFD_CallbackIDTypeDef CallbackID)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (hcan->State == HAL_CANFD_STATE_READY)
    {
        switch (CallbackID)
        {
        case HAL_CANFD_MSPINIT_CB_ID :
            hcan->MspInitCallback   =   HAL_CANFD_MspInit;            /* Initializes the CANFD MSP Callback */
            break;

        case HAL_CANFD_MSPDEINIT_CB_ID :
            hcan->MspDeInitCallback =   HAL_CANFD_MspDeInit;          /* DeInitializes the CANFD MSP Callback */
            break;

        case HAL_CANFD_TX_COMPLETE_CB_ID :
            hcan->TxCpltCallback    = HAL_CANFD_TxCpltCallback;       /* Tx Transfer completed callbacks Callback */
            break;

        case HAL_CANFD_RX_COMPLETE_CB_ID :
            hcan->RxCpltCallback  = HAL_CANFD_RxCpltCallback;         /* Rx Transfer completed callbacks Callback */
            break;

        case HAL_CANFD_ERROR_CB_ID :
            hcan->ErrorCallback =   HAL_CANFD_ErrorCallback;          /* CANFD Error Callback */
            break;

        default :
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else if (hcan->State == HAL_TIM_STATE_RESET)
    {
        switch (CallbackID)
        {
        case HAL_CANFD_MSPINIT_CB_ID :
            hcan->MspInitCallback   =   HAL_CANFD_MspInit;            /* Legacy weak Base MspInit Callback */
            break;

        case HAL_CANFD_MSPDEINIT_CB_ID :
            hcan->MspDeInitCallback =   HAL_CANFD_MspDeInit;          /* Legacy weak Base Msp DeInit Callback */
            break;

        case HAL_CANFD_TX_COMPLETE_CB_ID :
            hcan->TxCpltCallback    = HAL_CANFD_TxCpltCallback;       /* Legacy weak IC Msp Init Callback */
            break;

        case HAL_CANFD_RX_COMPLETE_CB_ID :
            hcan->RxCpltCallback  = HAL_CANFD_RxCpltCallback;         /* Legacy weak IC Msp DeInit Callback */
            break;

        case HAL_CANFD_ERROR_CB_ID :
            hcan->ErrorCallback =   HAL_CANFD_ErrorCallback;          /* Legacy weak OC Msp Init Callback */
            break;

        default :
            /* Return error status */
            status =  HAL_ERROR;
            break;
        }
    }
    else
    {
        /* Return error status */
        status =  HAL_ERROR;
    }

    return status;
}

#endif /* USE_HAL_CANFD_REGISTER_CALLBACKS */


/**
  * @}
  */

#endif /* HAL_CANFD_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
