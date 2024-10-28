/**
  *
  * @file    apm32f4xx_dal_dma_ex.c
  * @brief   DMA Extension DAL module driver
  *         This file provides firmware functions to manage the following 
  *         functionalities of the DMA Extension peripheral:
  *           + Extended features functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
  The DMA Extension DAL driver can be used as follows:
   (#) Start a multi buffer transfer using the DAL_DMA_MultiBufferStart() function
       for polling mode or DAL_DMA_MultiBufferStart_IT() for interrupt mode.
                   
     -@-  In Memory-to-Memory transfer mode, Multi (Double) Buffer mode is not allowed.
     -@-  When Multi (Double) Buffer mode is enabled the, transfer is circular by default.
     -@-  In Multi (Double) buffer mode, it is possible to update the base address for 
          the AHB memory port on the fly (DMA_SxM0AR or DMA_SxM1AR) when the stream is enabled. 
  
  @endverbatim
  *
  * @attention
  *
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
  * OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * The original code has been modified by Geehy Semiconductor.
  *
  * Copyright (c) 2017 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup DMAEx DMAEx
  * @brief DMA Extended DAL module driver
  * @{
  */

#ifdef DAL_DMA_MODULE_ENABLED

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private Constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup DMAEx_Private_Functions
  * @{
  */
static void DMA_MultiBufferSetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @addtogroup DMAEx_Exported_Functions
  * @{
  */


/** @addtogroup DMAEx_Exported_Functions_Group1
  *
@verbatim   
 ===============================================================================
                #####  Extended features functions  #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and 
          Start MultiBuffer DMA transfer
      (+) Configure the source, destination address and data length and 
          Start MultiBuffer DMA transfer with interrupt
      (+) Change on the fly the memory0 or memory1 address.
      
@endverbatim
  * @{
  */


/**
  * @brief  Starts the multi_buffer DMA Transfer.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  SecondMemAddress The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength)
{
  DAL_StatusTypeDef status = DAL_OK;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_DMA_BUFFER_SIZE(DataLength));
  
  /* Memory-to-memory transfer not supported in double buffering mode */
  if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
  {
    hdma->ErrorCode = DAL_DMA_ERROR_NOT_SUPPORTED;
    status = DAL_ERROR;
  }
  else
  {
    /* Process Locked */
    __DAL_LOCK(hdma);
    
    if(DAL_DMA_STATE_READY == hdma->State)
    {
      /* Change DMA peripheral state */
      hdma->State = DAL_DMA_STATE_BUSY; 
      
      /* Enable the double buffer mode */
      hdma->Instance->SCFG |= (uint32_t)DMA_SCFGx_DBM;
      
      /* Configure DMA Stream destination address */
      hdma->Instance->M1ADDR = SecondMemAddress;
      
      /* Configure the source, destination address and the data length */
      DMA_MultiBufferSetConfig(hdma, SrcAddress, DstAddress, DataLength);
      
      /* Enable the peripheral */
      __DAL_DMA_ENABLE(hdma);
    }
    else
    {
      /* Return error status */
      status = DAL_BUSY;
    }
  }
  return status;
}

/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  SecondMemAddress The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength)
{
  DAL_StatusTypeDef status = DAL_OK;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_DMA_BUFFER_SIZE(DataLength));
  
  /* Memory-to-memory transfer not supported in double buffering mode */
  if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
  {
    hdma->ErrorCode = DAL_DMA_ERROR_NOT_SUPPORTED;
    return DAL_ERROR;
  }
  
  /* Check callback functions */
  if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback) || (NULL == hdma->XferErrorCallback))
  {
    hdma->ErrorCode = DAL_DMA_ERROR_PARAM;
    return DAL_ERROR;
  }
  
  /* Process locked */
  __DAL_LOCK(hdma);
  
  if(DAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = DAL_DMA_STATE_BUSY;
    
    /* Initialize the error code */
    hdma->ErrorCode = DAL_DMA_ERROR_NONE;
    
    /* Enable the Double buffer mode */
    hdma->Instance->SCFG |= (uint32_t)DMA_SCFGx_DBM;
    
    /* Configure DMA Stream destination address */
    hdma->Instance->M1ADDR = SecondMemAddress;
    
    /* Configure the source, destination address and the data length */
    DMA_MultiBufferSetConfig(hdma, SrcAddress, DstAddress, DataLength); 
    
    /* Clear all flags */
    __DAL_DMA_CLEAR_FLAG (hdma, __DAL_DMA_GET_TC_FLAG_INDEX(hdma));
    __DAL_DMA_CLEAR_FLAG (hdma, __DAL_DMA_GET_HT_FLAG_INDEX(hdma));
    __DAL_DMA_CLEAR_FLAG (hdma, __DAL_DMA_GET_TE_FLAG_INDEX(hdma));
    __DAL_DMA_CLEAR_FLAG (hdma, __DAL_DMA_GET_DME_FLAG_INDEX(hdma));
    __DAL_DMA_CLEAR_FLAG (hdma, __DAL_DMA_GET_FE_FLAG_INDEX(hdma));

    /* Enable Common interrupts*/
    hdma->Instance->SCFG  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    hdma->Instance->FCTRL |= DMA_IT_FE;
    
    if((hdma->XferHalfCpltCallback != NULL) || (hdma->XferM1HalfCpltCallback != NULL))
    {
      hdma->Instance->SCFG  |= DMA_IT_HT;
    }
    
    /* Enable the peripheral */
    __DAL_DMA_ENABLE(hdma); 
  }
  else
  {     
    /* Process unlocked */
    __DAL_UNLOCK(hdma);	  
    
    /* Return error status */
    status = DAL_BUSY;
  }  
  return status; 
}

/**
  * @brief  Change the memory0 or memory1 address on the fly.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  Address    The new address
  * @param  memory     the memory to be changed, This parameter can be one of 
  *                     the following values:
  *                      MEMORY0 /
  *                      MEMORY1
  * @note   The MEMORY0 address can be changed only when the current transfer use
  *         MEMORY1 and the MEMORY1 address can be changed only when the current 
  *         transfer use MEMORY0.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, DAL_DMA_MemoryTypeDef memory)
{
  if(memory == MEMORY0)
  {
    /* change the memory0 address */
    hdma->Instance->M0ADDR = Address;
  }
  else
  {
    /* change the memory1 address */
    hdma->Instance->M1ADDR = Address;
  }

  return DAL_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup DMAEx_Private_Functions
  * @{
  */

/**
  * @brief  Set the DMA Transfer parameter.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval DAL status
  */
static void DMA_MultiBufferSetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{  
  /* Configure DMA Stream data length */
  hdma->Instance->NDATA = DataLength;
  
  /* Peripheral to Memory */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {   
    /* Configure DMA Stream destination address */
    hdma->Instance->PADDR = DstAddress;
    
    /* Configure DMA Stream source address */
    hdma->Instance->M0ADDR = SrcAddress;
  }
  /* Memory to Peripheral */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PADDR = SrcAddress;
    
    /* Configure DMA Stream destination address */
    hdma->Instance->M0ADDR = DstAddress;
  }
}

/**
  * @}
  */

#endif /* DAL_DMA_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

