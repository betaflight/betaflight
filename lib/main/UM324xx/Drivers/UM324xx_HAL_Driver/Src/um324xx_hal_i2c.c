/**
  ******************************************************************************
  * @file     um324xx_hal_i2c.c 
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
  ***********
  *******************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

/** @addtogroup um324xx_HAL_Driver
  * @{
  */
#ifdef HAL_I2C_MODULE_ENABLED

/** @defgroup I2C I2C
  * @brief I2C HAL module driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup I2C_Private_Define I2C Private Define
  * @{
  */

/**
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

    
/** @defgroup DMA_Private_Functions DMA Private Functions
  * @{
  */
 /* Private functions to handle I2C transfer */
static HAL_StatusTypeDef I2C_BaudConfig(I2C_HandleTypeDef *hi2c);    
static HAL_StatusTypeDef HAL_I2C_WAIT_STATE(I2C_HandleTypeDef *hi2c, uint32_t I2c_State, uint32_t Timeout);
static HAL_StatusTypeDef HAL_I2C_Start(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef HAL_I2C_Write_Byte(I2C_HandleTypeDef *hi2c, uint8_t byte,uint8_t status);
static HAL_StatusTypeDef HAL_I2C_Read_Byte(I2C_HandleTypeDef *hi2c, uint8_t *byte,uint8_t status);
static HAL_StatusTypeDef I2C_Slave_ISR_IT(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);
static HAL_StatusTypeDef I2C_Master_ISR_IT(I2C_HandleTypeDef *hi2c, uint32_t ITFlags);

/**
  * @}
  */
  
/* Exported functions --------------------------------------------------------*/

/** @defgroup I2C_Exported_Functions I2C Exported Functions
  * @{
  */

/** @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
  * @{
  */
/**
  * @brief  Configure the I2C Baude Set
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef I2C_BaudConfig(I2C_HandleTypeDef *hi2c)
{
    switch(hi2c->Init.BaudRate)            
    { 
        /* 100Kbps*/
        case I2C_SPEED_STAD_100K:          
            MODIFY_REG(hi2c->Instance->CCR,I2C_CCR_FIELDS,I2C_BAUDRATE_100K);     
            break;
        
        /* 400Kbps*/
        case I2C_SPEED_FAST_400K:       
            MODIFY_REG(hi2c->Instance->CCR,I2C_CCR_FIELDS,I2C_BAUDRATE_400K);     
            break;
        
        /* 1Mbps*/
        case I2C_SPEED_HIGH_1M:  
            MODIFY_REG(hi2c->Instance->CCR,I2C_CCR_FIELDS,I2C_BAUDRATE_1M);             
            break;    

        default: 
            break;
    }
    return HAL_OK;
}


/**
  * @brief  Configure the I2C Master Setconfig
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */

HAL_StatusTypeDef HAL_I2C_Master_SetConfig(I2C_HandleTypeDef *hi2c)
{
  /* Check the I2C handle allocation */
  if ((hi2c == NULL) || (hi2c->Instance == NULL))
  {
    return HAL_ERROR;
  }
  
  /* Configure the IIC baud rate */
  I2C_BaudConfig(hi2c);
  return HAL_OK;
}

/**
  * @brief  Configure the I2C Slave Setconfig
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_I2C_Slave_SetConfig(I2C_HandleTypeDef *hi2c)
{

  /* Check the I2C handle allocation */
  if ((hi2c == NULL) || (hi2c->Instance == NULL))
  {
    return HAL_ERROR;
  }

    /* Configure I2Cx: Own Address and ack own address1 mode */
   if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
   {
        SET_BIT(hi2c->Instance->SAD0,(hi2c->Init.SlaveAddress0)<<I2C_SAD0_ADR0_Pos);
        
        SET_BIT(hi2c->Instance->SADM0,I2C_SADM0_AMR0); 
        
        CLEAR_BIT(hi2c->Instance->XSADM0,I2C_XSADM0_XAMR);
        
        
        SET_BIT(hi2c->Instance->SAD1,(hi2c->Init.SlaveAddress1)<<I2C_SAD1_ADR1_Pos);
        
        SET_BIT(hi2c->Instance->SADM1,I2C_SADM1_AMR1); 
        
        CLEAR_BIT(hi2c->Instance->XSADM0,I2C_XSADM0_XAMR);
        
        
        SET_BIT(hi2c->Instance->SAD2,(hi2c->Init.SlaveAddress2)<<I2C_SAD2_ADR2_Pos);
        
        SET_BIT(hi2c->Instance->SADM2,I2C_SADM2_AMR2); 
        
        CLEAR_BIT(hi2c->Instance->XSADM0,I2C_XSADM0_XAMR);
        
        
        SET_BIT(hi2c->Instance->SAD3,(hi2c->Init.SlaveAddress3)<<I2C_SAD3_ADR3_Pos);
        
        SET_BIT(hi2c->Instance->SADM3,I2C_SADM3_AMR3_Msk); 
        
        CLEAR_BIT(hi2c->Instance->XSADM0,I2C_XSADM0_XAMR);
   }
   else /* I2C_ADDRESSINGMODE_10BIT */
   {
       SET_BIT(hi2c->Instance->XSAD0,(hi2c->Init.SlaveAddress0)<<I2C_XSADM0_XAMR_Pos);
              
       CLEAR_BIT(hi2c->Instance->SADM0,I2C_SADM0_AMR0); 
       
       CLEAR_BIT(hi2c->Instance->SADM1,I2C_SADM1_AMR1); 
       
       CLEAR_BIT(hi2c->Instance->SADM2,I2C_SADM2_AMR2); 
       
       CLEAR_BIT(hi2c->Instance->SADM3,I2C_SADM3_AMR3);
   }
  

  /* I2C SLAVE enable */
  __HAL_I2C_SLAVE_ENABLE(hi2c);
  /* I2C SLAVE AAK enable */
  __HAL_I2C_AAK_ENABLE(hi2c);
  
  return HAL_OK;
}

/**
  * @brief  Configure the I2C Initialize
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
     /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }
 
  if (hi2c->State == HAL_I2C_STATE_RESET)
  {
    hi2c->Lock = HAL_UNLOCKED;
     
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
    if (hi2c->MspInitCallback == NULL)
    {
      hi2c->MspInitCallback = HAL_I2C_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    hi2c->MspInitCallback(hi2c);
#else
    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    HAL_I2C_MspInit(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */
    /* Reset I2C */
    __HAL_I2C_RSRT_ENABLE(hi2c);
	  
    /* I2C Master Init */
    uint8_t i2c_mode = hi2c->Mode;
      
    if(HAL_I2C_MODE_MASTER == i2c_mode)
    {
        HAL_I2C_Master_SetConfig(hi2c);
    }
    /* I2C Slave Init */
    else
    {
        HAL_I2C_Slave_SetConfig(hi2c);
    }
  
  }
  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State = HAL_I2C_STATE_READY;
  hi2c->Mode = HAL_I2C_MODE_NONE;
  return HAL_OK;
}

/**
  * @brief  DeInitialize the I2C peripheral.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c)
{
  /* Check the I2C handle allocation */
  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  hi2c->State = HAL_I2C_STATE_BUSY;


#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
  if (hi2c->MspDeInitCallback == NULL)
  {
    hi2c->MspDeInitCallback = HAL_I2C_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  hi2c->MspDeInitCallback(hi2c);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
  HAL_I2C_MspDeInit(hi2c);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

  hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  hi2c->State = HAL_I2C_STATE_RESET;
  hi2c->Mode = HAL_I2C_MODE_NONE;

  /* Release Lock */
  __HAL_UNLOCK(hi2c);

  return HAL_OK;
}

/**
  * @brief Initialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MspInit could be implemented in the user file
   */
}

/**
  * @brief DeInitialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
__weak void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  Wait the I2C status
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef HAL_I2C_WAIT_STATE(I2C_HandleTypeDef *hi2c, uint32_t I2c_State, uint32_t Timeout)
{
    while(__HAL_I2C_GET_FLAG(hi2c, I2c_State) == 0)
    {    
        if((hi2c->Instance->STAT == I2C_FLAG_SEND_ADDRW_NAK) || 
           (hi2c->Instance->STAT == I2C_FLAG_SEND_ADDRR_NAK) ||
           (hi2c->Instance->STAT == I2C_FLAG_MASTER_SEND_NAK)|| 
           (hi2c->Instance->STAT == I2C_FLAG_SLAVE_AAK1_NAK)) 
        {
            return HAL_ERROR;
        }
        if((Timeout--)==0)    
        {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_OK;
    
}


/**
  * @brief  Send I2C Start message
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef HAL_I2C_Start(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef RetValue = HAL_ERROR;
    
    /*Send Start Flag*/
    __HAL_I2C_START_SEND(hi2c);
    /*wait send start flag finish*/
    RetValue = HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_START,I2C_TIMEOUT);
    
    return RetValue;
}

/**
  * @brief  Send I2C Start message
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef HAL_I2C_Restart(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef RetValue = HAL_ERROR;
    
    /*Send Start Flag*/
    __HAL_I2C_START_SEND(hi2c);
    /*Clear interrupt flag bit*/
    __HAL_I2C_CLEAR_IFLG(hi2c);
    /*wait send start flag finish*/
    RetValue = HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RESTART,I2C_TIMEOUT);
    
    return RetValue;  
}

/**
  * @brief  I2C Send Byte Data 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef HAL_I2C_Write_Byte(I2C_HandleTypeDef *hi2c, uint8_t byte,uint8_t status)
{
    HAL_StatusTypeDef RetValue = HAL_ERROR;
    
    hi2c->Instance->DATA = byte;
    
    /*Clear interrupt flag bit*/
    __HAL_I2C_CLEAR_IFLG(hi2c);
    
    RetValue = HAL_I2C_WAIT_STATE(hi2c,status,I2C_TIMEOUT);
    
    return RetValue;
}

/**
  * @brief  I2C accept Byte Data 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef HAL_I2C_Read_Byte(I2C_HandleTypeDef *hi2c, uint8_t *byte,uint8_t status)
{
    HAL_StatusTypeDef RetValue = HAL_ERROR;
    
    RetValue = HAL_I2C_WAIT_STATE(hi2c,status,I2C_TIMEOUT);
    
    if(RetValue == HAL_OK)
    {
        *byte = hi2c->Instance->DATA;
        /*Clear interrupt flag bit*/
        __HAL_I2C_CLEAR_IFLG(hi2c);
    }
    
    return RetValue;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup I2C_Exported_Functions_Group2  operation functions
 *  @brief    Initialization and Configuration functions
 *

  * @{
  */

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    
  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);
    
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->State = HAL_I2C_STATE_BUSY;
    
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;
    
    
    /*Send start*/
    if(HAL_OK != HAL_I2C_Start(hi2c))
    {
        goto  __ERROR;
    }
    
    /*Send address*/
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,((uint8_t)DevAddress<<1)|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
    }
    else
    {
        uint8_t addr1, addr2;
        addr1  = 0xF0;
        addr1 |= (uint8_t)((DevAddress >> 7) & 0x06);
        addr2  = (uint8_t) DevAddress;
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
        
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr2,I2C_FLAG_SEND_10BITADDRW_ACK))
        {
            goto  __ERROR;
        }        

    }
    /*Send data*/
    while (hi2c->XferCount > 0U)
    {

        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,*hi2c->pBuffPtr,I2C_FLAG_MASTER_SEND_ACK))
        {
            if((hi2c->Instance->STAT) == I2C_FLAG_MASTER_SEND_NAK)            
            break;
            else
            goto  __ERROR;
        }                            
        hi2c->pBuffPtr++;

        hi2c->XferCount--;
        
    }
    
    __HAL_I2C_STOP_SEND(hi2c);
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
     /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
    return HAL_OK;
    
    __ERROR:
    __HAL_I2C_STOP_SEND(hi2c);
    return HAL_ERROR;     
  }
  else
  {
      return HAL_BUSY;
  }      
}


/**
  * @brief  Receive in slave mode an amount of data in blocking mode
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);
    
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->State = HAL_I2C_STATE_BUSY;
    
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
   
    
    /*Send start*/
    if(HAL_OK != HAL_I2C_Start(hi2c))
    {
        goto  __ERROR;
    }
            
    /*Send address + read*/
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,((uint8_t)DevAddress<<1)|I2C_READ,I2C_FLAG_SEND_ADDRR_ACK))
        {
            goto  __ERROR;
        }
    }
    else
    {
        uint8_t addr1, addr2;
        addr1  = 0xF0;
        addr1 |= (uint8_t)((DevAddress >> 7) & 0x06);
        addr2  = (uint8_t) DevAddress;
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
        
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr2,I2C_FLAG_SEND_10BITADDRW_ACK))
        {
            goto  __ERROR;
        }
        if(HAL_OK != HAL_I2C_Restart(hi2c))
        {
            goto  __ERROR;
        }
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_READ,I2C_FLAG_SEND_ADDRR_ACK))
        {
            goto  __ERROR;
        }
    }
    
    __HAL_I2C_AAK_ENABLE(hi2c);
    __HAL_I2C_CLEAR_IFLG(hi2c);
    
    /*accpect data*/
    while (hi2c->XferCount > 0U)
    {
        
        if(HAL_OK != HAL_I2C_Read_Byte(hi2c,hi2c->pBuffPtr, I2C_FLAG_MASTER_RECV_ACK))
        {
            goto  __ERROR;
        }
                
        hi2c->pBuffPtr++;

        hi2c->XferCount--;
    }
    
    __HAL_I2C_STOP_SEND(hi2c);
    hi2c->State = HAL_I2C_STATE_READY;
    
    /* Process Locked */
    __HAL_UNLOCK(hi2c);
    
    return HAL_OK;
    __ERROR:
    __HAL_I2C_STOP_SEND(hi2c);
    return HAL_ERROR;   
  }
  else
  {
      return HAL_BUSY;
  }      
}



/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef I2C_Master_ISR_IT(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
    uint32_t tmpITFlags = ITFlags;
    
    /*!< In host mode, data bytes have been received and ACK has been sent  */
    if (__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_MASTER_RECV_ACK) != 0)
    {
        if(hi2c->XferCount > 0)
        {
            *hi2c->pBuffPtr++ = hi2c->Instance->DATA;
            
            hi2c->XferCount--;
            
        }
				else if(hi2c->XferCount == 0)
				{
					__HAL_I2C_STOP_SEND(hi2c);
       
          hi2c->State  = HAL_I2C_STATE_READY;
          __HAL_I2C_DISABLE_IT(hi2c);
				}
        /*Clear interrupt flag bit*/
        __HAL_I2C_CLEAR_IFLG(hi2c);
        return HAL_OK;
    }
    else if (__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_MASTER_RECV_NAK) != 0)
    {
        
        hi2c->XferCount = 0;
        
        __HAL_I2C_STOP_SEND(hi2c);
			  hi2c->State = HAL_I2C_STATE_READY;
        /*Clear interrupt flag bit*/
        __HAL_I2C_CLEAR_IFLG(hi2c);
        
        __HAL_I2C_DISABLE_IT(hi2c);
        return HAL_OK;
    }
    
    else if (__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_SEND_ADDRW_ACK) != 0)
    {
      
         __HAL_I2C_CLEAR_IFLG(hi2c);
         return HAL_OK;
    }
    else if (__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_MASTER_SEND_ACK) != 0)
    {
        if(hi2c->XferCount > 0)
        {
            hi2c->Instance->DATA = *hi2c->pBuffPtr++;
            
            hi2c->XferCount--;
            
        }
					else if(hi2c->XferCount == 0)
				{
					__HAL_I2C_STOP_SEND(hi2c);
          hi2c->State = HAL_I2C_STATE_READY;
        
          __HAL_I2C_DISABLE_IT(hi2c);
				}
        /*Clear interrupt flag bit*/
        __HAL_I2C_CLEAR_IFLG(hi2c);
        return HAL_OK;
    }
    else if (__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_MASTER_SEND_NAK) != 0)
    {         
        hi2c->XferCount = 0;
        
        *hi2c->pBuffPtr = NULL;
         __HAL_I2C_STOP_SEND(hi2c);
			   hi2c->State = HAL_I2C_STATE_READY;
        /*Clear interrupt flag bit*/
        __HAL_I2C_CLEAR_IFLG(hi2c);
        
        __HAL_I2C_DISABLE_IT(hi2c);
        return HAL_OK;
    }
    else 
    {
        /*Clear interrupt flag bit*/
        __HAL_I2C_CLEAR_IFLG(hi2c);
        return HAL_OK;
    }
    
    
}


/**
  * @brief  Transmit in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                            HAL_StatusTypeDef (*recv_callback)())
{
	if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    hi2c->State = HAL_I2C_STATE_BUSY_TX;
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    
    if(recv_callback != NULL)
    {
        hi2c->XferISR = recv_callback;              /*!< Register the interrupt service function */
    }
    else
    {
        hi2c->XferISR = I2C_Master_ISR_IT;         /*!< Register the interrupt service function */
    }
    
    /*Send start*/
    if(HAL_OK != HAL_I2C_Start(hi2c))
    {
        goto  __ERROR;
    }
    
    /*Send address + read*/
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,((uint8_t)DevAddress<<1)|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
        
    }
    else
    {
        uint8_t addr1, addr2;
        addr1  = 0xF0;
        addr1 |= (uint8_t)((DevAddress >> 7) & 0x06);
        addr2  = (uint8_t) DevAddress;
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
        
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr2,I2C_FLAG_SEND_10BITADDRW_ACK))
        {
            goto  __ERROR;
        }        
        
    }
    hi2c->Instance->DATA = *hi2c->pBuffPtr++;
    __HAL_I2C_CLEAR_IFLG(hi2c);
   __HAL_I2C_ENABLE_IT(hi2c);
    return HAL_OK;
    
    __ERROR:
    __HAL_I2C_DISABLE_IT(hi2c);
    __HAL_I2C_STOP_SEND(hi2c);
        
    return HAL_ERROR;
	}
 	else
	{
		return HAL_BUSY;
	}
}


/**
  * @brief  Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */

HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                            HAL_StatusTypeDef (*recv_callback)())
{
	if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    hi2c->State = HAL_I2C_STATE_BUSY_RX;
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    
    if(recv_callback != NULL)
    {
        hi2c->XferISR = recv_callback;              /*!< Register the interrupt service function */
    }
    else
    {
        hi2c->XferISR = I2C_Master_ISR_IT;         /*!< Register the interrupt service function */
    }
    
    /*Send start*/
    if(HAL_OK != HAL_I2C_Start(hi2c))
    {
        goto  __ERROR;
    }
    
    /*Send address + read*/
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
       
       // hi2c->Instance->DATA = ((uint8_t)DevAddress<<1)|I2C_READ;
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,((uint8_t)DevAddress<<1)|I2C_READ,I2C_FLAG_SEND_ADDRR_ACK))
        {
            goto  __ERROR;
        } 
        __HAL_I2C_CLEAR_IFLG(hi2c);
    
        __HAL_I2C_ENABLE_IT(hi2c);  
    }
    else
    {
        uint8_t addr1, addr2;
        addr1  = 0xF0;
        addr1 |= (uint8_t)((DevAddress >> 7) & 0x06);
        addr2  = (uint8_t) DevAddress;
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
        
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr2,I2C_FLAG_SEND_10BITADDRW_ACK))
        {
            goto  __ERROR;
        }
        if(HAL_OK != HAL_I2C_Restart(hi2c))
        {
            goto  __ERROR;
        }
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_READ,I2C_FLAG_SEND_ADDRR_ACK))
        {
            goto  __ERROR;
        }
        
    }
    
    __HAL_I2C_AAK_ENABLE(hi2c);
    
    __HAL_I2C_CLEAR_IFLG(hi2c);
    
    __HAL_I2C_ENABLE_IT(hi2c);
    
    
    return HAL_OK;
    
    __ERROR:
    __HAL_I2C_DISABLE_IT(hi2c);
    __HAL_I2C_STOP_SEND(hi2c);
        
    return HAL_ERROR; 
	}
  else
	{
		return HAL_BUSY;
	}		
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint16_t MemAddress, uint16_t MemAddSize,
                                        uint8_t *pData, uint16_t Size,uint32_t Timeout)
{
  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);
    
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->State = HAL_I2C_STATE_BUSY;
    
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;
    
    /*Send start*/
    if(HAL_OK != HAL_I2C_Start(hi2c))
    {
        goto  __ERROR;
    }
    
    /*Send address*/
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,((uint8_t)DevAddress<<1)|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
    }
    else
    {
        uint8_t addr1, addr2;
        addr1  = 0xF0;
        addr1 |= (uint8_t)((DevAddress >> 7) & 0x06);
        addr2  = (uint8_t) DevAddress;
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
        
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr2,I2C_FLAG_SEND_10BITADDRW_ACK))
        {
            goto  __ERROR;
        }        

    }
    
      /* If Memory address size is 8Bit */
    if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
    {
      /* Send Memory Address */
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,I2C_MEM_ADD_LSB(MemAddress),I2C_FLAG_MASTER_SEND_ACK))
        {
            goto  __ERROR;
        }               
     
    }
    /* If Memory address size is 16Bit */
    else
    {
      /* Send MSB of Memory Address */
       if(HAL_OK != HAL_I2C_Write_Byte(hi2c,I2C_MEM_ADD_MSB(MemAddress),I2C_FLAG_MASTER_SEND_ACK))
       {
           goto  __ERROR;
       }        
    
      /* Send LSB of Memory Address */
      if(HAL_OK != HAL_I2C_Write_Byte(hi2c,I2C_MEM_ADD_LSB(MemAddress),I2C_FLAG_MASTER_SEND_ACK))
       {
           goto  __ERROR;
       }      
      
    }
    

    /*Send data*/
    while (hi2c->XferCount > 0U)
    {
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,*hi2c->pBuffPtr,I2C_FLAG_MASTER_SEND_ACK))
        {
            if((hi2c->Instance->STAT) == I2C_FLAG_MASTER_SEND_NAK)            
            break;
            else
            goto  __ERROR;
        }                            
        hi2c->pBuffPtr++;

        hi2c->XferCount--;
        
    }
    
    __HAL_I2C_STOP_SEND(hi2c);
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
     /* Process Unlocked */
    __HAL_UNLOCK(hi2c);
    return HAL_OK;
    
    __ERROR:
    __HAL_I2C_STOP_SEND(hi2c);
    return HAL_ERROR;     
  }
  else
  {
      return HAL_BUSY;
  }      
    
}


/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint16_t MemAddress, uint16_t MemAddSize,
                                      uint8_t *pData, uint16_t Size,uint32_t Timeout)
{
    if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    /* Process Locked */
    __HAL_LOCK(hi2c);
    
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->State = HAL_I2C_STATE_BUSY;
    
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;
    
    /*Send start*/
    if(HAL_OK != HAL_I2C_Start(hi2c))
    {
        goto  __ERROR;
    }
    
    /*Send address*/
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,((uint8_t)DevAddress<<1)|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
    }
    else
    {
        uint8_t addr1, addr2;
        addr1  = 0xF0;
        addr1 |= (uint8_t)((DevAddress >> 7) & 0x06);
        addr2  = (uint8_t) DevAddress;
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
        
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr2,I2C_FLAG_SEND_10BITADDRW_ACK))
        {
            goto  __ERROR;
        }        

    }
    
      /* If Memory address size is 8Bit */
    if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
    {
      /* Send Memory Address */
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,I2C_MEM_ADD_LSB(MemAddress),I2C_FLAG_MASTER_SEND_ACK))
        {
            goto  __ERROR;
        }               
     
    }
    /* If Memory address size is 16Bit */
    else
    {
      /* Send MSB of Memory Address */
       if(HAL_OK != HAL_I2C_Write_Byte(hi2c,I2C_MEM_ADD_MSB(MemAddress),I2C_FLAG_MASTER_SEND_ACK))
       {
           goto  __ERROR;
       }        
    
      /* Send LSB of Memory Address */
      if(HAL_OK != HAL_I2C_Write_Byte(hi2c,I2C_MEM_ADD_LSB(MemAddress),I2C_FLAG_MASTER_SEND_ACK))
       {
           goto  __ERROR;
       }      
      
    }
    
    /*Send start*/
    if(HAL_OK != HAL_I2C_Restart(hi2c))
    {
        goto  __ERROR;
    }
    
    /*Send address + read*/
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,((uint8_t)DevAddress<<1)|I2C_READ,I2C_FLAG_SEND_ADDRR_ACK))
        {
            goto  __ERROR;
        }
    }
    else
    {
        uint8_t addr1, addr2;
        addr1  = 0xF0;
        addr1 |= (uint8_t)((DevAddress >> 7) & 0x06);
        addr2  = (uint8_t) DevAddress;
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_WRITE,I2C_FLAG_SEND_ADDRW_ACK))
        {
            goto  __ERROR;
        }
        
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr2,I2C_FLAG_SEND_10BITADDRW_ACK))
        {
            goto  __ERROR;
        }
        if(HAL_OK != HAL_I2C_Restart(hi2c))
        {
            goto  __ERROR;
        }
        if(HAL_OK != HAL_I2C_Write_Byte(hi2c,addr1|I2C_READ,I2C_FLAG_SEND_ADDRR_ACK))
        {
            goto  __ERROR;
        }
    }
    
    __HAL_I2C_AAK_ENABLE(hi2c);
    __HAL_I2C_CLEAR_IFLG(hi2c);
    
    /*accpect data*/
    while (hi2c->XferCount > 0U)
    {
        if(hi2c->XferCount == 1)
        {
            __HAL_I2C_AAK_DISABLE(hi2c);
            
            if(HAL_OK != HAL_I2C_Read_Byte(hi2c,hi2c->pBuffPtr, I2C_FLAG_MASTER_RECV_NAK))
            {
                goto  __ERROR;
            }
            break;
        }
        
        if(HAL_OK != HAL_I2C_Read_Byte(hi2c,hi2c->pBuffPtr, I2C_FLAG_MASTER_RECV_ACK))
        {
            goto  __ERROR;
        }
                
        hi2c->pBuffPtr++;

        hi2c->XferCount--;
    }
    
    __HAL_I2C_STOP_SEND(hi2c);
    hi2c->State = HAL_I2C_STATE_READY;
    
    /* Process Locked */
    __HAL_UNLOCK(hi2c);
    
    return HAL_OK;
    __ERROR:
    __HAL_I2C_STOP_SEND(hi2c);
    return HAL_ERROR;   
  }
  else
  {
      return HAL_BUSY;
  }      
    
    
}
/**
 * @brief  Transmits in slave mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{    
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    
    __HAL_I2C_AAK_ENABLE(hi2c);
    __HAL_I2C_DISABLE_IT(hi2c);
    
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL; 
        
     /*accept adress 7BIT*/
    if (hi2c->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
    {
        if(HAL_OK != HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RECV_ADDRR_ACK,I2C_TIMEOUT))
        {
            goto  __ERROR;
        }
    }
     /*accept adress 10BIT*/
    else
    {
       if(HAL_OK != HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RECV_ADDRW_ACK,I2C_TIMEOUT))
        {
            goto  __ERROR;
        }
         /*Clear interrupt flag bit*/
        __HAL_I2C_CLEAR_IFLG(hi2c);
        
        if(HAL_OK != HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RECV_STOP_RS,I2C_TIMEOUT))
        {
            goto  __ERROR;
        }
         /*Clear interrupt flag bit*/
        __HAL_I2C_CLEAR_IFLG(hi2c);
        
         if(HAL_OK != HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RECV_ADDRR_ACK,I2C_TIMEOUT))
        {
            goto  __ERROR;
        }
         
    }
    
    
    while (hi2c->XferCount > 0U)
    {
        if(HAL_OK !=(HAL_I2C_Write_Byte(hi2c,*hi2c->pBuffPtr, I2C_FLAG_SLAVE_AAK1_ACK)))
        {
            if((hi2c->Instance->STAT) == I2C_FLAG_SLAVE_AAK1_NAK)
            break;
            else
            goto __ERROR;
        }
        hi2c->XferCount --;
        
        hi2c->pBuffPtr++;        
    }
    /*Clear interrupt flag bit*/
    __HAL_I2C_CLEAR_IFLG(hi2c);
    
    
    if(HAL_OK != HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RECV_STOP_RS,I2C_TIMEOUT))
    {
        goto __ERROR;
    }
    /*Clear interrupt flag bit*/
    __HAL_I2C_CLEAR_IFLG(hi2c);
    
    return HAL_OK;
    
    __ERROR:
    /*Clear interrupt flag bit*/
    __HAL_I2C_CLEAR_IFLG(hi2c);
    return HAL_ERROR;
}

/**
  * @brief  Receive in slave mode an amount of data in blocking mode
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
        
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;  
    
    
    /*accept adress */
    if(HAL_OK != HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RECV_ADDRW_ACK,I2C_TIMEOUT))
    {
        goto  __ERROR;
    }
    
    __HAL_I2C_CLEAR_IFLG(hi2c);
    
    /*accept data */
    while (hi2c->XferCount > 0U)
    {  
        if(HAL_OK != HAL_I2C_Read_Byte(hi2c,hi2c->pBuffPtr, I2C_FLAG_RECV_ADDRD_DATA_ACK)) 
        {                
            goto __ERROR;    
        }
        
        hi2c->pBuffPtr++;

        hi2c->XferCount--;
				
    }
    
    if(HAL_OK != HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RECV_STOP_RS,I2C_TIMEOUT))
    {
        goto __ERROR;
    }
    
    __HAL_I2C_CLEAR_IFLG(hi2c);
    
    return HAL_OK;
    
    __ERROR:
    __HAL_I2C_CLEAR_IFLG(hi2c);
    return HAL_ERROR;    
}


/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
 
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
static HAL_StatusTypeDef I2C_Slave_ISR_IT(I2C_HandleTypeDef *hi2c, uint32_t ITFlags)
{
    uint32_t tmpITFlags = ITFlags;

    /*Transmission interrupt processing*/
    if(__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_RECV_ADDRR_ACK))
    {
        if(hi2c->XferCount > 0)
        {
            hi2c->Instance->DATA = *hi2c->pBuffPtr++;
            hi2c->XferCount -- ;
        }
        
    }
    /*Transmission interrupt processing*/
    else if(__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_SLAVE_AAK1_ACK))
    {
        if(hi2c->XferCount > 0)
        {
            hi2c->Instance->DATA = *hi2c->pBuffPtr++;
            hi2c->XferCount -- ;
        }
        
        return HAL_OK;
    }
    /*Transmission interrupt processing*/
    else if(__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_RECV_STOP_RS))
    {
         __HAL_I2C_DISABLE_IT(hi2c);
         hi2c->State       = HAL_I2C_STATE_READY;
         hi2c->XferCount = 0;
         return HAL_OK;
    }
    
    
    
    /*Accept interrupt processing*/
    if (__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_RECV_ADDRD_DATA_ACK) != 0)
    {
        if(hi2c->XferCount > 0)
        {
            *hi2c->pBuffPtr++ = hi2c->Instance->DATA;
            
            hi2c->XferCount--;
            
        }
        return HAL_OK;
    }
      /*Accept interrupt processing*/
    else if (__HAL_I2C_CHECK_FLAG(tmpITFlags, I2C_FLAG_RECV_ADDRD_DATA_NAK) != 0)
    {
        hi2c->XferCount = 0;
        
        __HAL_I2C_AAK_ENABLE(hi2c);  
   
        __HAL_I2C_DISABLE_IT(hi2c);
        return HAL_OK;
    }
      /*Accept interrupt processing*/    
    else 
    {
        return HAL_OK;
    }
    
}

/**
  * @brief  Receive in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */

HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, HAL_StatusTypeDef (*recv_callback)())
{
  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    hi2c->State       = HAL_I2C_STATE_BUSY_RX;
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferISR     = I2C_Slave_ISR_IT;
        
    if(recv_callback != NULL)
    {
        /*User-defined callback function docking*/
        hi2c->XferISR = recv_callback;          
    }
    else
    {
        /*Register to accept interrupt function*/
        hi2c->XferISR = I2C_Slave_ISR_IT;           
    }
    
    __HAL_I2C_AAK_ENABLE(hi2c);
    
    __HAL_I2C_ENABLE_IT(hi2c);
    
//    if(HAL_OK != HAL_I2C_WAIT_STATE(hi2c,I2C_FLAG_RECV_STOP_RS,I2C_TIMEOUT))
//    {
//    }
    /*Clear interrupt flag bit*/
    __HAL_I2C_CLEAR_IFLG(hi2c); 
    
    return HAL_OK;
	}
	else
  {
    return HAL_BUSY;
  }
}

/**
  * @}
  */



/**
  * @brief Transmits in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @return HAL_StatusTypeDef
  *     @retval HAL_OK    nothing wrong
  *     @retval HAL_ERROR something wrong
  */
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, HAL_StatusTypeDef (*recv_callback)())
{
  if (hi2c->State == HAL_I2C_STATE_READY)
  {  
    if((pData == NULL) || (Size == 0U))
    {
        return HAL_ERROR;
    }
    hi2c->State       = HAL_I2C_STATE_BUSY_TX;
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferISR     = I2C_Slave_ISR_IT;
        
    if(recv_callback != NULL)
    {
        /*User-defined callback function docking*/
        hi2c->XferISR = recv_callback;          
    }
    else
    {
        /*Register to accept interrupt function*/
        hi2c->XferISR = I2C_Slave_ISR_IT;           
    }
    
    __HAL_I2C_AAK_ENABLE(hi2c);
    
    __HAL_I2C_ENABLE_IT(hi2c);
    
    /*Clear interrupt flag bit*/
    __HAL_I2C_CLEAR_IFLG(hi2c); 
    
    return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}



/**
  * @brief  This function handles I2C event interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */

void HAL_I2C_IRQHandler(I2C_HandleTypeDef *hi2c)
{
    uint32_t itflags = READ_REG(hi2c->Instance->STAT);
    
    /* I2C events treatment -------------------------------------*/
    if(__HAL_I2C_GET_FLAG(hi2c,I2C_FLAG_NONE) == 0)
    {
        __HAL_I2C_CLEAR_IFLG(hi2c);
        
        if (hi2c->XferISR != NULL)
        {
            hi2c->XferISR(hi2c, itflags);
            
        }

    }
}

  
/** @defgroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
  *  @brief   Peripheral State, Mode and Error functions
  *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the I2C handle state.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL state
  */
HAL_I2C_StateTypeDef HAL_I2C_GetState(const I2C_HandleTypeDef *hi2c)
{
  /* Return I2C handle state */
  return hi2c->State;
}

/**
  * @brief  Returns the I2C Master, Slave, Memory or no mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval HAL mode
  */
HAL_I2C_ModeTypeDef HAL_I2C_GetMode(const I2C_HandleTypeDef *hi2c)
{
  return hi2c->Mode;
}

/**
  * @brief  Return the I2C error code.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval I2C Error Code
  */
uint32_t HAL_I2C_GetError(const I2C_HandleTypeDef *hi2c)
{
  return hi2c->ErrorCode;
}

/**
  * @}
  */  
  
  
  
#endif /* HAL_I2C_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

