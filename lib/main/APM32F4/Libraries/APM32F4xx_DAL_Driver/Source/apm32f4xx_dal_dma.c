/**
  *
  * @file    apm32f4xx_dal_dma.c
  * @brief   DMA DAL module driver.
  *    
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Direct Memory Access (DMA) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and errors functions
  @verbatim     
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) Enable and configure the peripheral to be connected to the DMA Stream
       (except for internal SRAM/FLASH memories: no initialization is 
       necessary) please refer to Reference manual for connection between peripherals
       and DMA requests.

   (#) For a given Stream, program the required configuration through the following parameters:
       Transfer Direction, Source and Destination data formats, 
       Circular, Normal or peripheral flow control mode, Stream Priority level, 
       Source and Destination Increment mode, FIFO mode and its Threshold (if needed), 
       Burst mode for Source and/or Destination (if needed) using DAL_DMA_Init() function.

   -@-   Prior to DAL_DMA_Init() the clock must be enabled for DMA through the following macros:
         __DAL_RCM_DMA1_CLK_ENABLE() or __DAL_RCM_DMA2_CLK_ENABLE().

     *** Polling mode IO operation ***
     =================================
    [..]
          (+) Use DAL_DMA_Start() to start DMA transfer after the configuration of Source 
              address and destination address and the Length of data to be transferred.
          (+) Use DAL_DMA_PollForTransfer() to poll for the end of current transfer, in this  
              case a fixed Timeout can be configured by User depending from his application.
          (+) Use DAL_DMA_Abort() function to abort the current transfer.

     *** Interrupt mode IO operation ***
     ===================================
    [..]
          (+) Configure the DMA interrupt priority using DAL_NVIC_SetPriority()
          (+) Enable the DMA IRQ handler using DAL_NVIC_EnableIRQ() 
          (+) Use DAL_DMA_Start_IT() to start DMA transfer after the configuration of  
              Source address and destination address and the Length of data to be transferred. In this 
              case the DMA interrupt is configured 
          (+) Use DAL_DMA_IRQHandler() called under DMA_IRQHandler() Interrupt subroutine
          (+) At the end of data transfer DAL_DMA_IRQHandler() function is executed and user can 
              add his own function by customization of function pointer XferCpltCallback and 
              XferErrorCallback (i.e a member of DMA handle structure).
    [..]
     (#) Use DAL_DMA_GetState() function to return the DMA state and DAL_DMA_GetError() in case of error 
         detection.

     (#) Use DAL_DMA_Abort_IT() function to abort the current transfer

     -@-   In Memory-to-Memory transfer mode, Circular mode is not allowed.

     -@-   The FIFO is used mainly to reduce bus usage and to allow data packing/unpacking: it is
           possible to set different Data Sizes for the Peripheral and the Memory (ie. you can set
           Half-Word data size for the peripheral to access its data register and set Word data size
           for the Memory to gain in access time. Each two half words will be packed and written in
           a single access to a Word in the Memory).

     -@-   When FIFO is disabled, it is not allowed to configure different Data Sizes for Source
           and Destination. In this case the Peripheral Data Size will be applied to both Source
           and Destination.

     *** DMA DAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in DMA DAL driver.
       
      (+) __DAL_DMA_ENABLE: Enable the specified DMA Stream.
      (+) __DAL_DMA_DISABLE: Disable the specified DMA Stream.
      (+) __DAL_DMA_GET_IT_SOURCE: Check whether the specified DMA Stream interrupt has occurred or not. 

     [..]
      (@) You can refer to the DMA DAL driver header file for more useful macros

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

/** @defgroup DMA DMA
  * @brief DMA DAL module driver
  * @{
  */

#ifdef DAL_DMA_MODULE_ENABLED

/* Private types -------------------------------------------------------------*/
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup DMA_Private_Constants
 * @{
 */
 #define DAL_TIMEOUT_DMA_ABORT    5U  /* 5 ms */
/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup DMA_Private_Functions
  * @{
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static uint32_t DMA_CalcBaseAndBitshift(DMA_HandleTypeDef *hdma);
static DAL_StatusTypeDef DMA_CheckFifoParam(DMA_HandleTypeDef *hdma);

/**
  * @}
  */  

/* Exported functions ---------------------------------------------------------*/
/** @addtogroup DMA_Exported_Functions
  * @{
  */

/** @addtogroup DMA_Exported_Functions_Group1
  *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]
    This section provides functions allowing to initialize the DMA Stream source
    and destination addresses, incrementation and data sizes, transfer direction, 
    circular/normal mode selection, memory-to-memory mode selection and Stream priority value.
    [..]
    The DAL_DMA_Init() function follows the DMA configuration procedures as described in
    reference manual.

@endverbatim
  * @{
  */
  
/**
  * @brief  Initialize the DMA according to the specified
  *         parameters in the DMA_InitTypeDef and create the associated handle.
  * @param  hdma Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMA_Init(DMA_HandleTypeDef *hdma)
{
  uint32_t tmp = 0U;
  uint32_t tickstart = DAL_GetTick();
  DMA_Base_Registers *regs;

  /* Check the DMA peripheral state */
  if(hdma == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_DMA_STREAM_ALL_INSTANCE(hdma->Instance));
  ASSERT_PARAM(IS_DMA_CHANNEL(hdma->Init.Channel));
  ASSERT_PARAM(IS_DMA_DIRECTION(hdma->Init.Direction));
  ASSERT_PARAM(IS_DMA_PERIPHERAL_INC_STATE(hdma->Init.PeriphInc));
  ASSERT_PARAM(IS_DMA_MEMORY_INC_STATE(hdma->Init.MemInc));
  ASSERT_PARAM(IS_DMA_PERIPHERAL_DATA_SIZE(hdma->Init.PeriphDataAlignment));
  ASSERT_PARAM(IS_DMA_MEMORY_DATA_SIZE(hdma->Init.MemDataAlignment));
  ASSERT_PARAM(IS_DMA_MODE(hdma->Init.Mode));
  ASSERT_PARAM(IS_DMA_PRIORITY(hdma->Init.Priority));
  ASSERT_PARAM(IS_DMA_FIFO_MODE_STATE(hdma->Init.FIFOMode));
  /* Check the memory burst, peripheral burst and FIFO threshold parameters only
     when FIFO mode is enabled */
  if(hdma->Init.FIFOMode != DMA_FIFOMODE_DISABLE)
  {
    ASSERT_PARAM(IS_DMA_FIFO_THRESHOLD(hdma->Init.FIFOThreshold));
    ASSERT_PARAM(IS_DMA_MEMORY_BURST(hdma->Init.MemBurst));
    ASSERT_PARAM(IS_DMA_PERIPHERAL_BURST(hdma->Init.PeriphBurst));
  }

  /* Change DMA peripheral state */
  hdma->State = DAL_DMA_STATE_BUSY;

  /* Allocate lock resource */
  __DAL_UNLOCK(hdma);
  
  /* Disable the peripheral */
  __DAL_DMA_DISABLE(hdma);
  
  /* Check if the DMA Stream is effectively disabled */
  while((hdma->Instance->SCFG & DMA_SCFGx_EN) != RESET)
  {
    /* Check for the Timeout */
    if((DAL_GetTick() - tickstart ) > DAL_TIMEOUT_DMA_ABORT)
    {
      /* Update error code */
      hdma->ErrorCode = DAL_DMA_ERROR_TIMEOUT;
      
      /* Change the DMA state */
      hdma->State = DAL_DMA_STATE_TIMEOUT;
      
      return DAL_TIMEOUT;
    }
  }
  
  /* Get the SCFG register value */
  tmp = hdma->Instance->SCFG;

  /* Clear CHSEL, MBCFG, PBCFG, PRILCFG, MEMSIZECFG, PERSIZECFG, MEMIM, PERIM, CIRCMEN, DIRCFG, CTARG and DBM bits */
  tmp &= ((uint32_t)~(DMA_SCFGx_CHSEL | DMA_SCFGx_MBCFG | DMA_SCFGx_PBCFG | \
                      DMA_SCFGx_PRILCFG    | DMA_SCFGx_MEMSIZECFG  | DMA_SCFGx_PERSIZECFG  | \
                      DMA_SCFGx_MEMIM  | DMA_SCFGx_PERIM   | DMA_SCFGx_CIRCMEN   | \
                      DMA_SCFGx_DIRCFG   | DMA_SCFGx_CTARG     | DMA_SCFGx_DBM));

  /* Prepare the DMA Stream configuration */
  tmp |=  hdma->Init.Channel             | hdma->Init.Direction        |
          hdma->Init.PeriphInc           | hdma->Init.MemInc           |
          hdma->Init.PeriphDataAlignment | hdma->Init.MemDataAlignment |
          hdma->Init.Mode                | hdma->Init.Priority;

  /* the Memory burst and peripheral burst are not used when the FIFO is disabled */
  if(hdma->Init.FIFOMode == DMA_FIFOMODE_ENABLE)
  {
    /* Get memory burst and peripheral burst */
    tmp |=  hdma->Init.MemBurst | hdma->Init.PeriphBurst;
  }
  
  /* Write to DMA Stream SCFG register */
  hdma->Instance->SCFG = tmp;  

  /* Get the FCTRL register value */
  tmp = hdma->Instance->FCTRL;

  /* Clear Direct mode and FIFO threshold bits */
  tmp &= (uint32_t)~(DMA_FCTRLx_DMDEN | DMA_FCTRLx_FTHSEL);

  /* Prepare the DMA Stream FIFO configuration */
  tmp |= hdma->Init.FIFOMode;

  /* The FIFO threshold is not used when the FIFO mode is disabled */
  if(hdma->Init.FIFOMode == DMA_FIFOMODE_ENABLE)
  {
    /* Get the FIFO threshold */
    tmp |= hdma->Init.FIFOThreshold;
    
    /* Check compatibility between FIFO threshold level and size of the memory burst */
    /* for INCR4, INCR8, INCR16 bursts */
    if (hdma->Init.MemBurst != DMA_MBURST_SINGLE)
    {
      if (DMA_CheckFifoParam(hdma) != DAL_OK)
      {
        /* Update error code */
        hdma->ErrorCode = DAL_DMA_ERROR_PARAM;
        
        /* Change the DMA state */
        hdma->State = DAL_DMA_STATE_READY;
        
        return DAL_ERROR; 
      }
    }
  }
  
  /* Write to DMA Stream FCTRL */
  hdma->Instance->FCTRL = tmp;

  /* Initialize StreamBaseAddress and StreamIndex parameters to be used to calculate
     DMA steam Base Address needed by DAL_DMA_IRQHandler() and DAL_DMA_PollForTransfer() */
  regs = (DMA_Base_Registers *)DMA_CalcBaseAndBitshift(hdma);
  
  /* Clear all interrupt flags */
  regs->IFCR = 0x3FU << hdma->StreamIndex;

  /* Initialize the error code */
  hdma->ErrorCode = DAL_DMA_ERROR_NONE;
                                                                                     
  /* Initialize the DMA state */
  hdma->State = DAL_DMA_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  DeInitializes the DMA peripheral 
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMA_DeInit(DMA_HandleTypeDef *hdma)
{
  DMA_Base_Registers *regs;

  /* Check the DMA peripheral state */
  if(hdma == NULL)
  {
    return DAL_ERROR;
  }
  
  /* Check the DMA peripheral state */
  if(hdma->State == DAL_DMA_STATE_BUSY)
  {
    /* Return error status */
    return DAL_BUSY;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_DMA_STREAM_ALL_INSTANCE(hdma->Instance));

  /* Disable the selected DMA Streamx */
  __DAL_DMA_DISABLE(hdma);

  /* Reset DMA Streamx control register */
  hdma->Instance->SCFG   = 0U;

  /* Reset DMA Streamx number of data to transfer register */
  hdma->Instance->NDATA = 0U;

  /* Reset DMA Streamx peripheral address register */
  hdma->Instance->PADDR  = 0U;

  /* Reset DMA Streamx memory 0 address register */
  hdma->Instance->M0ADDR = 0U;
  
  /* Reset DMA Streamx memory 1 address register */
  hdma->Instance->M1ADDR = 0U;
  
  /* Reset DMA Streamx FIFO control register */
  hdma->Instance->FCTRL  = 0x00000021U;
  
  /* Get DMA steam Base Address */  
  regs = (DMA_Base_Registers *)DMA_CalcBaseAndBitshift(hdma);
  
  /* Clean all callbacks */
  hdma->XferCpltCallback = NULL;
  hdma->XferHalfCpltCallback = NULL;
  hdma->XferM1CpltCallback = NULL;
  hdma->XferM1HalfCpltCallback = NULL;
  hdma->XferErrorCallback = NULL;
  hdma->XferAbortCallback = NULL;

  /* Clear all interrupt flags at correct offset within the register */
  regs->IFCR = 0x3FU << hdma->StreamIndex;

  /* Reset the error code */
  hdma->ErrorCode = DAL_DMA_ERROR_NONE;

  /* Reset the DMA state */
  hdma->State = DAL_DMA_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hdma);

  return DAL_OK;
}

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group2
  *
@verbatim   
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and Start DMA transfer
      (+) Configure the source, destination address and data length and 
          Start DMA transfer with interrupt
      (+) Abort DMA transfer
      (+) Poll for transfer complete
      (+) Handle DMA interrupt request  

@endverbatim
  * @{
  */

/**
  * @brief  Starts the DMA Transfer.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  DAL_StatusTypeDef status = DAL_OK;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_DMA_BUFFER_SIZE(DataLength));

  /* Process locked */
  __DAL_LOCK(hdma);

  if(DAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = DAL_DMA_STATE_BUSY;
    
    /* Initialize the error code */
    hdma->ErrorCode = DAL_DMA_ERROR_NONE;
    
    /* Configure the source, destination address and the data length */
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

    /* Enable the Peripheral */
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
  * @brief  Start the DMA Transfer with interrupt enabled.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
  
  /* Check the parameters */
  ASSERT_PARAM(IS_DMA_BUFFER_SIZE(DataLength));
 
  /* Process locked */
  __DAL_LOCK(hdma);
  
  if(DAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = DAL_DMA_STATE_BUSY;
    
    /* Initialize the error code */
    hdma->ErrorCode = DAL_DMA_ERROR_NONE;
    
    /* Configure the source, destination address and the data length */
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);
    
    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << hdma->StreamIndex;
    
    /* Enable Common interrupts*/
    hdma->Instance->SCFG  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    
    if(hdma->XferHalfCpltCallback != NULL)
    {
      hdma->Instance->SCFG  |= DMA_IT_HT;
    }
    
    /* Enable the Peripheral */
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
  * @brief  Aborts the DMA Transfer.
  * @param  hdma   pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Stream.
  *                   
  * @note  After disabling a DMA Stream, a check for wait until the DMA Stream is 
  *        effectively disabled is added. If a Stream is disabled 
  *        while a data transfer is ongoing, the current data will be transferred
  *        and the Stream will be effectively disabled only after the transfer of
  *        this single data is finished.  
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
  
  uint32_t tickstart = DAL_GetTick();
  
  if(hdma->State != DAL_DMA_STATE_BUSY)
  {
    hdma->ErrorCode = DAL_DMA_ERROR_NO_XFER;
    
    /* Process Unlocked */
    __DAL_UNLOCK(hdma);
    
    return DAL_ERROR;
  }
  else
  {
    /* Disable all the transfer interrupts */
    hdma->Instance->SCFG  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
    hdma->Instance->FCTRL &= ~(DMA_IT_FE);
    
    if((hdma->XferHalfCpltCallback != NULL) || (hdma->XferM1HalfCpltCallback != NULL))
    {
      hdma->Instance->SCFG  &= ~(DMA_IT_HT);
    }
    
    /* Disable the stream */
    __DAL_DMA_DISABLE(hdma);
    
    /* Check if the DMA Stream is effectively disabled */
    while((hdma->Instance->SCFG & DMA_SCFGx_EN) != RESET)
    {
      /* Check for the Timeout */
      if((DAL_GetTick() - tickstart ) > DAL_TIMEOUT_DMA_ABORT)
      {
        /* Update error code */
        hdma->ErrorCode = DAL_DMA_ERROR_TIMEOUT;
        
        /* Change the DMA state */
        hdma->State = DAL_DMA_STATE_TIMEOUT;
        
        /* Process Unlocked */
        __DAL_UNLOCK(hdma);
        
        return DAL_TIMEOUT;
      }
    }
    
    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << hdma->StreamIndex;
    
    /* Change the DMA state*/
    hdma->State = DAL_DMA_STATE_READY;
    
    /* Process Unlocked */
    __DAL_UNLOCK(hdma);
  }
  return DAL_OK;
}

/**
  * @brief  Aborts the DMA Transfer in Interrupt mode.
  * @param  hdma   pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Stream.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
  if(hdma->State != DAL_DMA_STATE_BUSY)
  {
    hdma->ErrorCode = DAL_DMA_ERROR_NO_XFER;
    return DAL_ERROR;
  }
  else
  {
    /* Set Abort State  */
    hdma->State = DAL_DMA_STATE_ABORT;
    
    /* Disable the stream */
    __DAL_DMA_DISABLE(hdma);
  }

  return DAL_OK;
}

/**
  * @brief  Polling for transfer complete.
  * @param  hdma          pointer to a DMA_HandleTypeDef structure that contains
  *                        the configuration information for the specified DMA Stream.
  * @param  CompleteLevel Specifies the DMA level complete.
  * @note   The polling mode is kept in this version for legacy. it is recommended to use the IT model instead.
  *         This model could be used for debug purpose.
  * @note   The DAL_DMA_PollForTransfer API cannot be used in circular and double buffering mode (automatic circular mode). 
  * @param  Timeout       Timeout duration.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, DAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout)
{
  DAL_StatusTypeDef status = DAL_OK; 
  uint32_t mask_cpltlevel;
  uint32_t tickstart = DAL_GetTick(); 
  uint32_t tmpisr;
  
  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs;

  if(DAL_DMA_STATE_BUSY != hdma->State)
  {
    /* No transfer ongoing */
    hdma->ErrorCode = DAL_DMA_ERROR_NO_XFER;
    __DAL_UNLOCK(hdma);
    return DAL_ERROR;
  }

  /* Polling mode not supported in circular mode and double buffering mode */
  if ((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) != RESET)
  {
    hdma->ErrorCode = DAL_DMA_ERROR_NOT_SUPPORTED;
    return DAL_ERROR;
  }
  
  /* Get the level transfer complete flag */
  if(CompleteLevel == DAL_DMA_FULL_TRANSFER)
  {
    /* Transfer Complete flag */
    mask_cpltlevel = DMA_FLAG_TCIF0_4 << hdma->StreamIndex;
  }
  else
  {
    /* Half Transfer Complete flag */
    mask_cpltlevel = DMA_FLAG_HTIF0_4 << hdma->StreamIndex;
  }
  
  regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
  tmpisr = regs->ISR;
  
  while(((tmpisr & mask_cpltlevel) == RESET) && ((hdma->ErrorCode & DAL_DMA_ERROR_TE) == RESET))
  {
    /* Check for the Timeout (Not applicable in circular mode)*/
    if(Timeout != DAL_MAX_DELAY)
    {
      if((Timeout == 0U)||((DAL_GetTick() - tickstart ) > Timeout))
      {
        /* Update error code */
        hdma->ErrorCode = DAL_DMA_ERROR_TIMEOUT;
        
        /* Change the DMA state */
        hdma->State = DAL_DMA_STATE_READY;
        
        /* Process Unlocked */
        __DAL_UNLOCK(hdma);
        
        return DAL_TIMEOUT;
      }
    }

    /* Get the ISR register value */
    tmpisr = regs->ISR;

    if((tmpisr & (DMA_FLAG_TEIF0_4 << hdma->StreamIndex)) != RESET)
    {
      /* Update error code */
      hdma->ErrorCode |= DAL_DMA_ERROR_TE;
      
      /* Clear the transfer error flag */
      regs->IFCR = DMA_FLAG_TEIF0_4 << hdma->StreamIndex;
    }
    
    if((tmpisr & (DMA_FLAG_FEIF0_4 << hdma->StreamIndex)) != RESET)
    {
      /* Update error code */
      hdma->ErrorCode |= DAL_DMA_ERROR_FE;
      
      /* Clear the FIFO error flag */
      regs->IFCR = DMA_FLAG_FEIF0_4 << hdma->StreamIndex;
    }
    
    if((tmpisr & (DMA_FLAG_DMEIF0_4 << hdma->StreamIndex)) != RESET)
    {
      /* Update error code */
      hdma->ErrorCode |= DAL_DMA_ERROR_DME;
      
      /* Clear the Direct Mode error flag */
      regs->IFCR = DMA_FLAG_DMEIF0_4 << hdma->StreamIndex;
    }
  }
  
  if(hdma->ErrorCode != DAL_DMA_ERROR_NONE)
  {
    if((hdma->ErrorCode & DAL_DMA_ERROR_TE) != RESET)
    {
      DAL_DMA_Abort(hdma);
    
      /* Clear the half transfer and transfer complete flags */
      regs->IFCR = (DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4) << hdma->StreamIndex;
    
      /* Change the DMA state */
      hdma->State= DAL_DMA_STATE_READY;

      /* Process Unlocked */
      __DAL_UNLOCK(hdma);

      return DAL_ERROR;
   }
  }
  
  /* Get the level transfer complete flag */
  if(CompleteLevel == DAL_DMA_FULL_TRANSFER)
  {
    /* Clear the half transfer and transfer complete flags */
    regs->IFCR = (DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4) << hdma->StreamIndex;
    
    hdma->State = DAL_DMA_STATE_READY;
    
    /* Process Unlocked */
    __DAL_UNLOCK(hdma);
  }
  else
  {
    /* Clear the half transfer and transfer complete flags */
    regs->IFCR = (DMA_FLAG_HTIF0_4) << hdma->StreamIndex;
  }
  
  return status;
}

/**
  * @brief  Handles DMA interrupt request.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval None
  */
void DAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
  uint32_t tmpisr;
  __IO uint32_t count = 0U;
  uint32_t timeout = SystemCoreClock / 9600U;

  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;

  tmpisr = regs->ISR;

  /* Transfer Error Interrupt management ***************************************/
  if ((tmpisr & (DMA_FLAG_TEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__DAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TE) != RESET)
    {
      /* Disable the transfer error interrupt */
      hdma->Instance->SCFG  &= ~(DMA_IT_TE);
      
      /* Clear the transfer error flag */
      regs->IFCR = DMA_FLAG_TEIF0_4 << hdma->StreamIndex;
      
      /* Update error code */
      hdma->ErrorCode |= DAL_DMA_ERROR_TE;
    }
  }
  /* FIFO Error Interrupt management ******************************************/
  if ((tmpisr & (DMA_FLAG_FEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__DAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_FE) != RESET)
    {
      /* Clear the FIFO error flag */
      regs->IFCR = DMA_FLAG_FEIF0_4 << hdma->StreamIndex;

      /* Update error code */
      hdma->ErrorCode |= DAL_DMA_ERROR_FE;
    }
  }
  /* Direct Mode Error Interrupt management ***********************************/
  if ((tmpisr & (DMA_FLAG_DMEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__DAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_DME) != RESET)
    {
      /* Clear the direct mode error flag */
      regs->IFCR = DMA_FLAG_DMEIF0_4 << hdma->StreamIndex;

      /* Update error code */
      hdma->ErrorCode |= DAL_DMA_ERROR_DME;
    }
  }
  /* Half Transfer Complete Interrupt management ******************************/
  if ((tmpisr & (DMA_FLAG_HTIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__DAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_HT) != RESET)
    {
      /* Clear the half transfer complete flag */
      regs->IFCR = DMA_FLAG_HTIF0_4 << hdma->StreamIndex;
      
      /* Multi_Buffering mode enabled */
      if(((hdma->Instance->SCFG) & (uint32_t)(DMA_SCFGx_DBM)) != RESET)
      {
        /* Current memory buffer used is Memory 0 */
        if((hdma->Instance->SCFG & DMA_SCFGx_CTARG) == RESET)
        {
          if(hdma->XferHalfCpltCallback != NULL)
          {
            /* Half transfer callback */
            hdma->XferHalfCpltCallback(hdma);
          }
        }
        /* Current memory buffer used is Memory 1 */
        else
        {
          if(hdma->XferM1HalfCpltCallback != NULL)
          {
            /* Half transfer callback */
            hdma->XferM1HalfCpltCallback(hdma);
          }
        }
      }
      else
      {
        /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
        if((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == RESET)
        {
          /* Disable the half transfer interrupt */
          hdma->Instance->SCFG  &= ~(DMA_IT_HT);
        }
        
        if(hdma->XferHalfCpltCallback != NULL)
        {
          /* Half transfer callback */
          hdma->XferHalfCpltCallback(hdma);
        }
      }
    }
  }
  /* Transfer Complete Interrupt management ***********************************/
  if ((tmpisr & (DMA_FLAG_TCIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__DAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != RESET)
    {
      /* Clear the transfer complete flag */
      regs->IFCR = DMA_FLAG_TCIF0_4 << hdma->StreamIndex;
      
      if(DAL_DMA_STATE_ABORT == hdma->State)
      {
        /* Disable all the transfer interrupts */
        hdma->Instance->SCFG  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
        hdma->Instance->FCTRL &= ~(DMA_IT_FE);
        
        if((hdma->XferHalfCpltCallback != NULL) || (hdma->XferM1HalfCpltCallback != NULL))
        {
          hdma->Instance->SCFG  &= ~(DMA_IT_HT);
        }

        /* Clear all interrupt flags at correct offset within the register */
        regs->IFCR = 0x3FU << hdma->StreamIndex;

        /* Change the DMA state */
        hdma->State = DAL_DMA_STATE_READY;

        /* Process Unlocked */
        __DAL_UNLOCK(hdma);

        if(hdma->XferAbortCallback != NULL)
        {
          hdma->XferAbortCallback(hdma);
        }
        return;
      }

      if(((hdma->Instance->SCFG) & (uint32_t)(DMA_SCFGx_DBM)) != RESET)
      {
        /* Current memory buffer used is Memory 0 */
        if((hdma->Instance->SCFG & DMA_SCFGx_CTARG) == RESET)
        {
          if(hdma->XferM1CpltCallback != NULL)
          {
            /* Transfer complete Callback for memory1 */
            hdma->XferM1CpltCallback(hdma);
          }
        }
        /* Current memory buffer used is Memory 1 */
        else
        {
          if(hdma->XferCpltCallback != NULL)
          {
            /* Transfer complete Callback for memory0 */
            hdma->XferCpltCallback(hdma);
          }
        }
      }
      /* Disable the transfer complete interrupt if the DMA mode is not CIRCULAR */
      else
      {
        if((hdma->Instance->SCFG & DMA_SCFGx_CIRCMEN) == RESET)
        {
          /* Disable the transfer complete interrupt */
          hdma->Instance->SCFG  &= ~(DMA_IT_TC);

          /* Change the DMA state */
          hdma->State = DAL_DMA_STATE_READY;

          /* Process Unlocked */
          __DAL_UNLOCK(hdma);
        }

        if(hdma->XferCpltCallback != NULL)
        {
          /* Transfer complete callback */
          hdma->XferCpltCallback(hdma);
        }
      }
    }
  }
  
  /* manage error case */
  if(hdma->ErrorCode != DAL_DMA_ERROR_NONE)
  {
    if((hdma->ErrorCode & DAL_DMA_ERROR_TE) != RESET)
    {
      hdma->State = DAL_DMA_STATE_ABORT;

      /* Disable the stream */
      __DAL_DMA_DISABLE(hdma);

      do
      {
        if (++count > timeout)
        {
          break;
        }
      }
      while((hdma->Instance->SCFG & DMA_SCFGx_EN) != RESET);

      /* Change the DMA state */
      hdma->State = DAL_DMA_STATE_READY;

      /* Process Unlocked */
      __DAL_UNLOCK(hdma);
    }

    if(hdma->XferErrorCallback != NULL)
    {
      /* Transfer error callback */
      hdma->XferErrorCallback(hdma);
    }
  }
}

/**
  * @brief  Register callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Stream.
  * @param  CallbackID           User Callback identifier
  *                               a DMA_HandleTypeDef structure as parameter.
  * @param  pCallback            pointer to private callback function which has pointer to 
  *                               a DMA_HandleTypeDef structure as parameter.
  * @retval DAL status
  */                      
DAL_StatusTypeDef DAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, DAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma))
{

  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hdma);

  if(DAL_DMA_STATE_READY == hdma->State)
  {
    switch (CallbackID)
    {
    case  DAL_DMA_XFER_CPLT_CB_ID:
      hdma->XferCpltCallback = pCallback;
      break;

    case  DAL_DMA_XFER_HALFCPLT_CB_ID:
      hdma->XferHalfCpltCallback = pCallback;
      break;

    case  DAL_DMA_XFER_M1CPLT_CB_ID:
      hdma->XferM1CpltCallback = pCallback;
      break;

    case  DAL_DMA_XFER_M1HALFCPLT_CB_ID:
      hdma->XferM1HalfCpltCallback = pCallback;
      break;

    case  DAL_DMA_XFER_ERROR_CB_ID:
      hdma->XferErrorCallback = pCallback;
      break;

    case  DAL_DMA_XFER_ABORT_CB_ID:
      hdma->XferAbortCallback = pCallback;
      break;

    default:
      /* Return error status */
      status =  DAL_ERROR;
      break;
    }
  }
  else
  {
    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hdma);
  
  return status;
}

/**
  * @brief  UnRegister callbacks
  * @param  hdma                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Stream.
  * @param  CallbackID           User Callback identifier
  *                               a DAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @retval DAL status
  */              
DAL_StatusTypeDef DAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, DAL_DMA_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;
  
  /* Process locked */
  __DAL_LOCK(hdma);
  
  if(DAL_DMA_STATE_READY == hdma->State)
  {
    switch (CallbackID)
    {
    case  DAL_DMA_XFER_CPLT_CB_ID:
      hdma->XferCpltCallback = NULL;
      break;
      
    case  DAL_DMA_XFER_HALFCPLT_CB_ID:
      hdma->XferHalfCpltCallback = NULL;
      break;
      
    case  DAL_DMA_XFER_M1CPLT_CB_ID:
      hdma->XferM1CpltCallback = NULL;
      break;
      
    case  DAL_DMA_XFER_M1HALFCPLT_CB_ID:
      hdma->XferM1HalfCpltCallback = NULL;
      break;
      
    case  DAL_DMA_XFER_ERROR_CB_ID:
      hdma->XferErrorCallback = NULL;
      break;
      
    case  DAL_DMA_XFER_ABORT_CB_ID:
      hdma->XferAbortCallback = NULL;
      break; 
      
    case   DAL_DMA_XFER_ALL_CB_ID:
      hdma->XferCpltCallback = NULL;
      hdma->XferHalfCpltCallback = NULL;
      hdma->XferM1CpltCallback = NULL;
      hdma->XferM1HalfCpltCallback = NULL;
      hdma->XferErrorCallback = NULL;
      hdma->XferAbortCallback = NULL;
      break; 
      
    default:
      status = DAL_ERROR;
      break;
    }
  }
  else
  {
    status = DAL_ERROR;
  }
  
  /* Release Lock */
  __DAL_UNLOCK(hdma);
  
  return status;
}

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group3
  *
@verbatim
 ===============================================================================
                    ##### State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */

/**
  * @brief  Returns the DMA state.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.
  * @retval DAL state
  */
DAL_DMA_StateTypeDef DAL_DMA_GetState(DMA_HandleTypeDef *hdma)
{
  return hdma->State;
}

/**
  * @brief  Return the DMA error code
  * @param  hdma  pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Stream.
  * @retval DMA Error Code
  */
uint32_t DAL_DMA_GetError(DMA_HandleTypeDef *hdma)
{
  return hdma->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup DMA_Private_Functions
  * @{
  */

/**
  * @brief  Sets the DMA Transfer parameter.
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.
  * @param  SrcAddress The source memory Buffer address
  * @param  DstAddress The destination memory Buffer address
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval DAL status
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear DBM bit */
  hdma->Instance->SCFG &= (uint32_t)(~DMA_SCFGx_DBM);

  /* Configure DMA Stream data length */
  hdma->Instance->NDATA = DataLength;

  /* Memory to Peripheral */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PADDR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0ADDR = SrcAddress;
  }
  /* Peripheral to Memory */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PADDR = SrcAddress;

    /* Configure DMA Stream destination address */
    hdma->Instance->M0ADDR = DstAddress;
  }
}

/**
  * @brief  Returns the DMA Stream base address depending on stream number
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream. 
  * @retval Stream base address
  */
static uint32_t DMA_CalcBaseAndBitshift(DMA_HandleTypeDef *hdma)
{
  uint32_t stream_number = (((uint32_t)hdma->Instance & 0xFFU) - 16U) / 24U;
  
  /* lookup table for necessary bitshift of flags within status registers */
  static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
  hdma->StreamIndex = flagBitshiftOffset[stream_number];
  
  if (stream_number > 3U)
  {
    /* return pointer to HINTSTS and HIFCLR */
    hdma->StreamBaseAddress = (((uint32_t)hdma->Instance & (uint32_t)(~0x3FFU)) + 4U);
  }
  else
  {
    /* return pointer to LINTSTS and LIFCLR */
    hdma->StreamBaseAddress = ((uint32_t)hdma->Instance & (uint32_t)(~0x3FFU));
  }
  
  return hdma->StreamBaseAddress;
}

/**
  * @brief  Check compatibility between FIFO threshold level and size of the memory burst
  * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream. 
  * @retval DAL status
  */
static DAL_StatusTypeDef DMA_CheckFifoParam(DMA_HandleTypeDef *hdma)
{
  DAL_StatusTypeDef status = DAL_OK;
  uint32_t tmp = hdma->Init.FIFOThreshold;
  
  /* Memory Data size equal to Byte */
  if(hdma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE)
  {
    switch (tmp)
    {
    case DMA_FIFO_THRESHOLD_1QUARTERFULL:
    case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
      if ((hdma->Init.MemBurst & DMA_SCFGx_MBCFG_1) == DMA_SCFGx_MBCFG_1)
      {
        status = DAL_ERROR;
      }
      break;
    case DMA_FIFO_THRESHOLD_HALFFULL:
      if (hdma->Init.MemBurst == DMA_MBURST_INC16)
      {
        status = DAL_ERROR;
      }
      break;
    case DMA_FIFO_THRESHOLD_FULL:
      break;
    default:
      break;
    }
  }
  
  /* Memory Data size equal to Half-Word */
  else if (hdma->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD)
  {
    switch (tmp)
    {
    case DMA_FIFO_THRESHOLD_1QUARTERFULL:
    case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
      status = DAL_ERROR;
      break;
    case DMA_FIFO_THRESHOLD_HALFFULL:
      if ((hdma->Init.MemBurst & DMA_SCFGx_MBCFG_1) == DMA_SCFGx_MBCFG_1)
      {
        status = DAL_ERROR;
      }
      break;
    case DMA_FIFO_THRESHOLD_FULL:
      if (hdma->Init.MemBurst == DMA_MBURST_INC16)
      {
        status = DAL_ERROR;
      }
      break;   
    default:
      break;
    }
  }
  
  /* Memory Data size equal to Word */
  else
  {
    switch (tmp)
    {
    case DMA_FIFO_THRESHOLD_1QUARTERFULL:
    case DMA_FIFO_THRESHOLD_HALFFULL:
    case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
      status = DAL_ERROR;
      break;
    case DMA_FIFO_THRESHOLD_FULL:
      if ((hdma->Init.MemBurst & DMA_SCFGx_MBCFG_1) == DMA_SCFGx_MBCFG_1)
      {
        status = DAL_ERROR;
      }
      break;
    default:
      break;
    }
  } 
  
  return status; 
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

