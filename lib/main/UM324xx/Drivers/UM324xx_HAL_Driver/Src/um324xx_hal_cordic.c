/**
  ******************************************************************************
  * @file    um324xx_hal_cordic.c
  * @author  MCU Team
  * @version V1.00
  * @date    2023-04-18
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

/** @defgroup CORDIC CORDIC
  * @brief CORDIC HAL driver modules.
  * @{
  */
#ifdef HAL_CORDIC_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup CORDIC_Private_Functions CORDIC Private Functions
  * @{
  */
static void CORDIC_WriteInDataIncrementPtr(CORDIC_HandleTypeDef *hcordic, int32_t **ppInBuff);
static void CORDIC_ReadOutDataIncrementPtr(CORDIC_HandleTypeDef *hcordic, int32_t **ppOutBuff);
static void CORDIC_DMAInCplt(DMA_HandleTypeDef *hdma);
static void CORDIC_DMAOutCplt(DMA_HandleTypeDef *hdma);
static void CORDIC_DMAError(DMA_HandleTypeDef *hdma);
/**
  * @}
  */
  
  
/* Exported functions --------------------------------------------------------*/

/** @defgroup CORDIC_Exported_Functions CORDIC Exported Functions
  * @{
  */

/** @defgroup CORDIC_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions.
  *


@endverbatim
  * @{
  */
/**
  * @brief  Initialize the CORDIC peripheral and the associated handle.
  * @param  hcordic pointer to a CORDIC_HandleTypeDef structure.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CORDIC_Init(CORDIC_HandleTypeDef *hcordic)
{
  uint32_t tmp;
  /* Check the CORDIC handle allocation */
  if (hcordic == NULL)
  {
    /* Return error status */
    return HAL_ERROR;
  }


#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
  if (hcordic->State == HAL_CORDIC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcordic->Lock = HAL_UNLOCKED;

    /* Reset callbacks to legacy functions */
    hcordic->ErrorCallback         = HAL_CORDIC_ErrorCallback;         /* Legacy weak ErrorCallback */
    hcordic->CalculateCpltCallback = HAL_CORDIC_CalculateCpltCallback; /* Legacy weak CalculateCpltCallback */

    if (hcordic->MspInitCallback == NULL)
    {
      hcordic->MspInitCallback = HAL_CORDIC_MspInit;                   /* Legacy weak MspInit */
    }

    /* Initialize the low level hardware */
    hcordic->MspInitCallback(hcordic);
  }
#else
  if (hcordic->State == HAL_CORDIC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcordic->Lock = HAL_UNLOCKED;

    /* Initialize the low level hardware */
    HAL_CORDIC_MspInit(hcordic);
  }
#endif /* (USE_HAL_CORDIC_REGISTER_CALLBACKS) */

  tmp = (uint32_t)(hcordic->Init.Function | hcordic->Init.WIDTH_IN | hcordic->Init.WIDTH_OUT | \
                   hcordic->Init.ADDR_IN | hcordic->Init.ADDR_OUT | hcordic->Init.MERGE_IN | \
                   hcordic->Init.MERGE_OUT | hcordic->Init.Iteration  | hcordic->Init.Scale);
  
  
  WRITE_REG (hcordic->Instance->CTRL,tmp);
  /* Set CORDIC error code to none */
  hcordic->ErrorCode = HAL_CORDIC_ERROR_NONE;

  /* Reset pInBuff and pOutBuff */
  hcordic->pInBuff = NULL;
  hcordic->pOutBuff = NULL;
  
  hcordic->DMADirection = CORDIC_DMA_DIR_NONE;


  /* Change CORDIC peripheral state */
  hcordic->State = HAL_CORDIC_STATE_READY;

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  DeInitialize the CORDIC peripheral.
  * @param  hcordic pointer to a CORDIC_HandleTypeDef structure.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CORDIC_DeInit(CORDIC_HandleTypeDef *hcordic)
{
  /* Check the CORDIC handle allocation */
  if (hcordic == NULL)
  {
    /* Return error status */
    return HAL_ERROR;
  }

  /* Change CORDIC peripheral state */
  hcordic->State = HAL_CORDIC_STATE_BUSY;

#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
  if (hcordic->MspDeInitCallback == NULL)
  {
    hcordic->MspDeInitCallback = HAL_CORDIC_MspDeInit;
  }

  /* De-Initialize the low level hardware */
  hcordic->MspDeInitCallback(hcordic);
#else
  /* De-Initialize the low level hardware: CLOCK, NVIC, DMA */
  HAL_CORDIC_MspDeInit(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */

  /* Set CORDIC error code to none */
  hcordic->ErrorCode = HAL_CORDIC_ERROR_NONE;

  /* Reset pInBuff and pOutBuff */
  hcordic->pInBuff = NULL;
  hcordic->pOutBuff = NULL;

 
  /* Change CORDIC peripheral state */
  hcordic->State = HAL_CORDIC_STATE_RESET;

  /* Reset Lock */
  hcordic->Lock = HAL_UNLOCKED;

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initialize the CORDIC MSP.
  * @param  hcordic CORDIC handle
  * @retval None
  */
__weak void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef *hcordic)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcordic);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_CORDIC_MspInit can be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the CORDIC MSP.
  * @param  hcordic CORDIC handle
  * @retval None
  */
__weak void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef *hcordic)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcordic);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_CORDIC_MspDeInit can be implemented in the user file
   */
}


/**
  * @}
  */

/**
  * @}
  */



/** @defgroup CORDIC_Exported_Functions_Group2 Peripheral Control functions
  *  @brief    Control functions.
  *
@verbatim
  ==============================================================================
                      ##### Peripheral Control functions #####
  ==============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the CORDIC peripheral: function, precision, scaling factor,
          number of input data and output data, size of input data and output data.
      (+) Calculate output data of CORDIC processing on input date, using the
          existing CORDIC configuration
    [..]  Four processing functions are available for calculation:
      (+) Polling mode
      (+) Polling mode, with Zero-Overhead register access
      (+) Interrupt mode
      (+) DMA mode

@endverbatim
  * @{
  */
  
/**
  * @brief  Carry out data of CORDIC processing in polling mode,
  *         according to the existing CORDIC configuration.
  * @param  hcordic pointer to a CORDIC_HandleTypeDef structure that contains
  *         the configuration information for CORDIC module.
  * @param  pInBuff Pointer to buffer containing input data for CORDIC processing.
  * @param  pOutBuff Pointer to buffer where output data of CORDIC processing will be stored.
  * @param  NbCalc Number of CORDIC calculation to process.
  * @param  Timeout Specify Timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CORDIC_Calculate(CORDIC_HandleTypeDef *hcordic, int32_t *pInBuff, int32_t *pOutBuff,uint32_t Timeout)
{
  uint32_t tickstart;
 
  int32_t *p_tmp_in_buff = pInBuff;
  int32_t *p_tmp_out_buff = pOutBuff;
#if defined(UM324xF) || defined(UM32x42x)
  if(hcordic->Init.Function == CORDIC_FUNCTION_MODE0)
  {
    if((*pInBuff>0x3cd00000)&&(*pInBuff<0x40000000))
    *pInBuff = 0x40000000+(0x40000000-*pInBuff);
    else if((*pInBuff>0xc0000000)&&(*pInBuff<0xc3300000))
    *pInBuff = 0xc0000000-(0xc3300000-*pInBuff);
  }
#endif
  /* Check parameters setting */
  if ((pInBuff == NULL) || (pOutBuff == NULL))
  {
    /* Update the error code */
    hcordic->ErrorCode |= HAL_CORDIC_ERROR_PARAM;

    /* Return error status */
    return HAL_ERROR;
  }

  /* Check handle state is ready */
  if (hcordic->State == HAL_CORDIC_STATE_READY)
  {
    /* Reset CORDIC error code */
    hcordic->ErrorCode = HAL_CORDIC_ERROR_NONE;

    /* Change the CORDIC state */
    hcordic->State = HAL_CORDIC_STATE_BUSY;

    /* Get tick */
    tickstart = HAL_GetTick();

    /* Write of input data in Write Data register, and increment input buffer pointer */
    CORDIC_WriteInDataIncrementPtr(hcordic, &p_tmp_in_buff);

    while (HAL_IS_BIT_CLR(hcordic->Instance->CTRL, CORDIC_CTRL_DATA_READY))
    {
        /* Check for the Timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
          if ((HAL_GetTick() - tickstart) > Timeout)
          {
            /* Set CORDIC error code */
            hcordic->ErrorCode = HAL_CORDIC_ERROR_TIMEOUT;

            /* Change the CORDIC state */
            hcordic->State = HAL_CORDIC_STATE_READY;

            /* Return function status */
            return HAL_ERROR;
          }
        }
    } 

      /* Read output data from Read Data register, and increment output buffer pointer */
      CORDIC_ReadOutDataIncrementPtr(hcordic, &p_tmp_out_buff);
    


    /* Change the CORDIC state */
    hcordic->State = HAL_CORDIC_STATE_READY;

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    /* Set CORDIC error code */
    hcordic->ErrorCode |= HAL_CORDIC_ERROR_NOT_READY;

    /* Return function status */
    return HAL_ERROR;
  }
}



/**
  * @brief  Carry out input and/or output data of CORDIC processing in DMA mode,
  *         according to the existing CORDIC configuration.
  * @param  hcordic pointer to a CORDIC_HandleTypeDef structure that contains
  *         the configuration information for CORDIC module.
  * @param  pInBuff Pointer to buffer containing input data for CORDIC processing.
  * @param  pOutBuff Pointer to buffer where output data of CORDIC processing will be stored.
  * @param  NbCalc Number of CORDIC calculation to process.
  * @param  DMADirection Direction of DMA transfers.
  *         This parameter can be one of the following values:
  *            @arg @ref CORDIC_DMA_Direction CORDIC DMA direction
  * @note   pInBuff or pOutBuff is unused in case of unique DMADirection transfer, and can
  *         be set to NULL value in this case.
  * @note   pInBuff and pOutBuff buffers must be 32-bit aligned to ensure a correct
  *         DMA transfer to and from the Peripheral.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CORDIC_Calculate_DMA(CORDIC_HandleTypeDef *hcordic, int32_t *pInBuff, int32_t *pOutBuff,
                                            uint32_t DMADirection)
{
  uint32_t sizeinbuff;
  uint32_t sizeoutbuff;
  uint32_t inputaddr;
  uint32_t outputaddr;
  
  /* Check if CORDIC DMA direction "Out" is requested */
  if ((DMADirection == CORDIC_DMA_DIR_OUT) || (DMADirection == CORDIC_DMA_DIR_IN_OUT))
  {
    /* Check parameters setting */
    if (pOutBuff == NULL)
    {
      /* Update the error code */
      hcordic->ErrorCode |= HAL_CORDIC_ERROR_PARAM;

      /* Return error status */
      return HAL_ERROR;
    }
  }

  /* Check if CORDIC DMA direction "In" is requested */
  if ((DMADirection == CORDIC_DMA_DIR_IN) || (DMADirection == CORDIC_DMA_DIR_IN_OUT))
  {
    /* Check parameters setting */
    if (pInBuff == NULL)
    {
      /* Update the error code */
      hcordic->ErrorCode |= HAL_CORDIC_ERROR_PARAM;

      /* Return error status */
      return HAL_ERROR;
    }
  }

  if (hcordic->State == HAL_CORDIC_STATE_READY)
  {
    /* Reset CORDIC error code */
    hcordic->ErrorCode = HAL_CORDIC_ERROR_NONE;
    
     /* Change the CORDIC state */
    hcordic->State = HAL_CORDIC_STATE_BUSY;
    
     /* Get DMA direction */
    hcordic->DMADirection = DMADirection;
    
     /* Check if CORDIC DMA direction "Out" is requested */
    if ((DMADirection == CORDIC_DMA_DIR_OUT) || (DMADirection == CORDIC_DMA_DIR_IN_OUT))
    {
      /* Set the CORDIC DMA transfer complete callback */
      hcordic->hdmaOut->XferTfrCallback = CORDIC_DMAOutCplt;
      /* Set the DMA error callback */
      hcordic->hdmaOut->XferErrorCallback = CORDIC_DMAError;
        
      outputaddr = (uint32_t)pOutBuff;
      
      if(HAL_IS_BIT_SET(hcordic->Instance->CTRL,CORDIC_CTRL_MERGE_OUT))
      {
          sizeoutbuff = 1U;
      }
      else
      {
          sizeoutbuff = 2U;
      }
      /* Enable the DMA stream managing CORDIC output data read */
      if (HAL_DMA_Start_IT(hcordic->hdmaOut, (uint32_t)&hcordic->Instance->DOUT1, outputaddr, sizeoutbuff) != HAL_OK)
      {
        /* Update the error code */
        hcordic->ErrorCode |= HAL_CORDIC_ERROR_DMA;
        
        /* Return error status */
        return HAL_ERROR;
      }
     SET_BIT(hcordic->Instance->CTRL, CORDIC_DMA_OUT);
    }

    /* Check if CORDIC DMA direction "In" is requested */
    if((DMADirection == CORDIC_DMA_DIR_IN) || (DMADirection == CORDIC_DMA_DIR_IN_OUT))
    {
       /* Set the CORDIC DMA transfer complete callback */
       hcordic->hdmaIn->XferTfrCallback = CORDIC_DMAInCplt;
       /* Set the DMA error callback */
       hcordic->hdmaIn->XferErrorCallback = CORDIC_DMAError;
        
       if(HAL_IS_BIT_SET(hcordic->Instance->CTRL,CORDIC_CTRL_MERGE_OUT))
       {
          sizeinbuff = 1U;
       }
       else
       {
          sizeinbuff = 2U;
       }
      
       inputaddr = (uint32_t)pInBuff;
       
      
       /* Enable the DMA stream managing CORDIC output data read */
       if (HAL_DMA_Start_IT(hcordic->hdmaIn, inputaddr ,(uint32_t)&hcordic->Instance->DIN1 , sizeinbuff) != HAL_OK)
       {
           /* Update the error code */
           hcordic->ErrorCode |= HAL_CORDIC_ERROR_DMA;
           /* Return error status */
           return HAL_ERROR;
       }
       SET_BIT(hcordic->Instance->CTRL, CORDIC_DMA_IN);
    }

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    /* Set CORDIC error code */
    hcordic->ErrorCode |= HAL_CORDIC_ERROR_NOT_READY;

    /* Return function status */
    return HAL_ERROR;
  }
}


/**
  * @}
  */

/** @defgroup CORDIC_Exported_Functions_Group3 Callback functions
  *  @brief    Callback functions.
  *
@verbatim
  ==============================================================================
                      ##### Callback functions  #####
  ==============================================================================
    [..]  This section provides Interruption and DMA callback functions:
      (+) DMA or Interrupt calculate complete
      (+) DMA or Interrupt error

@endverbatim
  * @{
  */

/**
  * @brief  CORDIC error callback.
  * @param  hcordic pointer to a CORDIC_HandleTypeDef structure that contains
  *         the configuration information for CORDIC module
  * @retval None
  */
__weak void HAL_CORDIC_ErrorCallback(CORDIC_HandleTypeDef *hcordic)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcordic);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_CORDIC_ErrorCallback can be implemented in the user file
   */
}

/**
  * @brief  CORDIC calculate complete callback.
  * @param  hcordic pointer to a CORDIC_HandleTypeDef structure that contains
  *         the configuration information for CORDIC module
  * @retval None
  */
__weak void HAL_CORDIC_CalculateCpltCallback(CORDIC_HandleTypeDef *hcordic)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcordic);

  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_CORDIC_CalculateCpltCallback can be implemented in the user file
   */
}

/**
  * @}
  */



/** @addtogroup CORDIC_Private_Functions
  * @{
  */

/**
  * @brief  Write input data for CORDIC processing, and increment input buffer pointer.
  * @param  hcordic pointer to a CORDIC_HandleTypeDef structure that contains
  *         the configuration information for CORDIC module.
  * @param  ppInBuff Pointer to pointer to input buffer.
  * @retval none
  */
static void CORDIC_WriteInDataIncrementPtr(CORDIC_HandleTypeDef *hcordic, int32_t **ppInBuff)
{
  /*16bit*/
  if(HAL_IS_BIT_SET(hcordic->Instance->CTRL, CORDIC_CTRL_WIDTH_IN)) 
  {
      if(HAL_IS_BIT_SET(hcordic->Instance->CTRL, CORDIC_CTRL_MERGE_IN))
      {
          int32_t intemp = 0;
          intemp = (uint16_t)**ppInBuff;
          (*ppInBuff)++;
          intemp |= (uint16_t)**ppInBuff << 16U;
          WRITE_REG(hcordic->Instance->DIN1, (uint32_t)intemp);
      }
      else
      {
        if(HAL_IS_BIT_SET(hcordic->Instance->CTRL, CORDIC_CTRL_ADDR_IN)) 
        {
            
            WRITE_REG(hcordic->Instance->DIN1, (uint16_t) **ppInBuff);
            (*ppInBuff)++;
            WRITE_REG(hcordic->Instance->DIN1, (uint16_t) **ppInBuff);
        }
        else
        {
            WRITE_REG(hcordic->Instance->DIN1, (uint16_t) **ppInBuff);
            (*ppInBuff)++;
            WRITE_REG(hcordic->Instance->DIN2, (uint16_t) **ppInBuff);
        }
      }
  }
  /*32bit*/
  else
  {
      if(HAL_IS_BIT_SET(hcordic->Instance->CTRL, CORDIC_CTRL_ADDR_IN)) 
      {
          WRITE_REG(hcordic->Instance->DIN1, (uint32_t) **ppInBuff);
          (*ppInBuff)++;
          WRITE_REG(hcordic->Instance->DIN1, (uint32_t) **ppInBuff);
      }
      else
      {
          WRITE_REG(hcordic->Instance->DIN1, (uint32_t) **ppInBuff);
          (*ppInBuff)++;
          WRITE_REG(hcordic->Instance->DIN2, (uint32_t) **ppInBuff);
      }
  }
 
}

/**
  * @brief  Read output data of CORDIC processing, and increment output buffer pointer.
  * @param  hcordic pointer to a CORDIC_HandleTypeDef structure that contains
  *         the configuration information for CORDIC module.
  * @param  ppOutBuff Pointer to pointer to output buffer.
  * @retval none
  */
static void CORDIC_ReadOutDataIncrementPtr(CORDIC_HandleTypeDef *hcordic, int32_t **ppOutBuff)
{
  /*16bit*/
  if(HAL_IS_BIT_SET(hcordic->Instance->CTRL, CORDIC_CTRL_WIDTH_OUT)) 
  {
      if(HAL_IS_BIT_SET(hcordic->Instance->CTRL, CORDIC_CTRL_MERGE_OUT))
      {
          int32_t intemp = READ_REG(hcordic->Instance->DOUT1);
          **ppOutBuff = (int16_t)intemp;
          (*ppOutBuff)++;
          **ppOutBuff = (int16_t)(intemp >> 16);
      }
      else
      {
         if(HAL_IS_BIT_SET(hcordic->Instance->CTRL, CORDIC_CTRL_ADDR_OUT)) 
         {
           **ppOutBuff = (int16_t)READ_REG(hcordic->Instance->DOUT1);
           (*ppOutBuff)++;
           **ppOutBuff = (int16_t)READ_REG(hcordic->Instance->DOUT1);
         }
         else
         {
           **ppOutBuff = (int16_t)READ_REG(hcordic->Instance->DOUT1);
           (*ppOutBuff)++;
           **ppOutBuff = (int16_t)READ_REG(hcordic->Instance->DOUT2);
         }
      }
  }
  /*32bit*/
  else
  {
      if(HAL_IS_BIT_SET(hcordic->Instance->CTRL, CORDIC_CTRL_ADDR_OUT)) 
      {
          **ppOutBuff = (int32_t)READ_REG(hcordic->Instance->DOUT1);
          (*ppOutBuff)++;
          **ppOutBuff = (int32_t)READ_REG(hcordic->Instance->DOUT1);
      }
      else
      {
           **ppOutBuff = (int32_t)READ_REG(hcordic->Instance->DOUT1);
           (*ppOutBuff)++;
           **ppOutBuff = (int32_t)READ_REG(hcordic->Instance->DOUT2);
      }
  }
}

/**
  * @brief  DMA CORDIC Input Data process complete callback.
  * @param  hdma DMA handle.
  * @retval None
  */
static void CORDIC_DMAInCplt(DMA_HandleTypeDef *hdma)
{
  CORDIC_HandleTypeDef *hcordic = (CORDIC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  CLEAR_BIT(hcordic->Instance->CTRL, CORDIC_DMA_IN);
  /* Check if DMA direction is CORDIC Input only (no DMA for CORDIC Output) */
  if (hcordic->DMADirection == CORDIC_DMA_DIR_IN)
  {
    /* Change the CORDIC DMA direction to none */
    hcordic->DMADirection = CORDIC_DMA_DIR_NONE;

    /* Change the CORDIC state to ready */
    hcordic->State = HAL_CORDIC_STATE_READY;

    /* Call calculation complete callback */
#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
    /*Call registered callback*/
    hcordic->CalculateCpltCallback(hcordic);
#else
    /*Call legacy weak (surcharged) callback*/
    HAL_CORDIC_CalculateCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
  }
}

/**
  * @brief  DMA CORDIC Output Data process complete callback.
  * @param  hdma DMA handle.
  * @retval None
  */
static void CORDIC_DMAOutCplt(DMA_HandleTypeDef *hdma)
{
  CORDIC_HandleTypeDef *hcordic = (CORDIC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Change the CORDIC DMA direction to none */
  hcordic->DMADirection = CORDIC_DMA_DIR_NONE;

  CLEAR_BIT(hcordic->Instance->CTRL, CORDIC_DMA_OUT);
  /* Change the CORDIC state to ready */
  hcordic->State = HAL_CORDIC_STATE_READY;

  /* Call calculation complete callback */
#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
  /*Call registered callback*/
  hcordic->CalculateCpltCallback(hcordic);
#else
  /*Call legacy weak (surcharged) callback*/
  HAL_CORDIC_CalculateCpltCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA CORDIC communication error callback.
  * @param  hdma DMA handle.
  * @retval None
  */
static void CORDIC_DMAError(DMA_HandleTypeDef *hdma)
{
  CORDIC_HandleTypeDef *hcordic = (CORDIC_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  /* Set CORDIC handle state to error */
  hcordic->State = HAL_CORDIC_STATE_READY;

  /* Set CORDIC handle error code to DMA error */
  hcordic->ErrorCode |= HAL_CORDIC_ERROR_DMA;

  /* Call user callback */
#if USE_HAL_CORDIC_REGISTER_CALLBACKS == 1
  /*Call registered callback*/
  hcordic->ErrorCallback(hcordic);
#else
  /*Call legacy weak (surcharged) callback*/
  HAL_CORDIC_ErrorCallback(hcordic);
#endif /* USE_HAL_CORDIC_REGISTER_CALLBACKS */
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
#endif /* HAL_CORDIC_MODULE_ENABLED */


/**************************(c) COPYRIGHT Unicmicro Co.,Ltd *****END OF FILE****/
