/**
  *
  * @file    apm32f4xx_dal_dci.c
  * @brief   DCI DAL module driver
  *          This file provides firmware functions to manage the following
  *          functionalities of the Digital Camera Interface (DCI) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Error functions
  *
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
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
      The sequence below describes how to use this driver to capture image
      from a camera module connected to the DCI Interface.
      This sequence does not take into account the configuration of the
      camera module, which should be made before to configure and enable
      the DCI to capture images.

    (#) Program the required configuration through following parameters:
        horizontal and vertical polarity, pixel clock polarity, Capture Rate,
        Synchronization Mode, code of the frame delimiter and data width 
        using DAL_DCI_Init() function.

    (#) Configure the DMA2_Stream1 channel1 to transfer Data from DCI DR
        register to the destination memory buffer.

    (#) Program the required configuration through following parameters:
        DCI mode, destination memory Buffer address and the data length
        and enable capture using DAL_DCI_Start_DMA() function.

    (#) Optionally, configure and Enable the CROP feature to select a rectangular
        window from the received image using DAL_DCI_ConfigCrop()
        and DAL_DCI_EnableCROP() functions

    (#) The capture can be stopped using DAL_DCI_Stop() function.

    (#) To control DCI state you can use the function DAL_DCI_GetState().

     *** DCI DAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in DCI DAL driver.
       
      (+) __DAL_DCI_ENABLE: Enable the DCI peripheral.
      (+) __DAL_DCI_DISABLE: Disable the DCI peripheral.
      (+) __DAL_DCI_GET_FLAG: Get the DCI pending flags.
      (+) __DAL_DCI_CLEAR_FLAG: Clear the DCI pending flags.
      (+) __DAL_DCI_ENABLE_IT: Enable the specified DCI interrupts.
      (+) __DAL_DCI_DISABLE_IT: Disable the specified DCI interrupts.
      (+) __DAL_DCI_GET_IT_SOURCE: Check whether the specified DCI interrupt has occurred or not.

     [..]
       (@) You can refer to the DCI DAL driver header file for more useful macros
      
    *** Callback registration ***
    =============================

    The compilation define USE_DAL_DCI_REGISTER_CALLBACKS when set to 1
    allows the user to configure dynamically the driver callbacks.
    Use functions DAL_DCI_RegisterCallback() to register a user callback.

    Function DAL_DCI_RegisterCallback() allows to register following callbacks:
      (+) FrameEventCallback : DCI Frame Event.
      (+) VsyncEventCallback : DCI Vsync Event.
      (+) LineEventCallback  : DCI Line Event.
      (+) ErrorCallback      : DCI error.
      (+) MspInitCallback    : DCI MspInit.
      (+) MspDeInitCallback  : DCI MspDeInit.
    This function takes as parameters the DAL peripheral handle, the callback ID
    and a pointer to the user callback function.

    Use function DAL_DCI_UnRegisterCallback() to reset a callback to the default
    weak (surcharged) function.
    DAL_DCI_UnRegisterCallback() takes as parameters the DAL peripheral handle,
    and the callback ID.
    This function allows to reset following callbacks:
      (+) FrameEventCallback : DCI Frame Event.
      (+) VsyncEventCallback : DCI Vsync Event.
      (+) LineEventCallback  : DCI Line Event.
      (+) ErrorCallback      : DCI error.
      (+) MspInitCallback    : DCI MspInit.
      (+) MspDeInitCallback  : DCI MspDeInit.

    By default, after the DAL_DCI_Init and if the state is DAL_DCI_STATE_RESET
    all callbacks are reset to the corresponding legacy weak (surcharged) functions:
    examples FrameEventCallback(), DAL_DCI_ErrorCallback().
    Exception done for MspInit and MspDeInit callbacks that are respectively
    reset to the legacy weak (surcharged) functions in the DAL_DCI_Init
    and  DAL_DCI_DeInit only when these callbacks are null (not registered beforehand).
    If not, MspInit or MspDeInit are not null, the DAL_DCI_Init and DAL_DCI_DeInit
    keep and use the user MspInit/MspDeInit callbacks (registered beforehand).

    Callbacks can be registered/unregistered in READY state only.
    Exception done for MspInit/MspDeInit callbacks that can be registered/unregistered
    in READY or RESET state, thus registered (user) MspInit/DeInit callbacks can be used
    during the Init/DeInit.
    In that case first register the MspInit/MspDeInit user callbacks
    using DAL_DCI_RegisterCallback before calling DAL_DCI_DeInit
    or DAL_DCI_Init function.

    When the compilation define USE_DAL_DCI_REGISTER_CALLBACKS is set to 0 or
    not defined, the callback registering feature is not available
    and weak (surcharged) callbacks are used.
	
  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */
/** @defgroup DCI DCI
  * @brief DCI DAL module driver
  * @{
  */

#ifdef DAL_DCI_MODULE_ENABLED

#if defined(APM32F407xx) || defined(APM32F417xx)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DAL_TIMEOUT_DCI_STOP    14U  /* Set timeout to 1s  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void       DCI_DMAXferCplt(DMA_HandleTypeDef *hdma);
static void       DCI_DMAError(DMA_HandleTypeDef *hdma);

/* Exported functions --------------------------------------------------------*/

/** @defgroup DCI_Exported_Functions DCI Exported Functions
  * @{
  */

/** @defgroup DCI_Exported_Functions_Group1 Initialization and Configuration functions
  *  @brief   Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
                ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the DCI
      (+) De-initialize the DCI 

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the DCI according to the specified
  *         parameters in the DCI_InitTypeDef and create the associated handle.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval DAL status
  */
__weak DAL_StatusTypeDef DAL_DCI_Init(DCI_HandleTypeDef *hdci)
{
  /* Check the DCI peripheral state */
  if(hdci == NULL)
  {
     return DAL_ERROR;
  }

  /* Check function parameters */
  ASSERT_PARAM(IS_DCI_ALL_INSTANCE(hdci->Instance));
  ASSERT_PARAM(IS_DCI_PCKPOLARITY(hdci->Init.PCKPolarity));
  ASSERT_PARAM(IS_DCI_VSPOLARITY(hdci->Init.VSPolarity));
  ASSERT_PARAM(IS_DCI_HSPOLARITY(hdci->Init.HSPolarity));
  ASSERT_PARAM(IS_DCI_SYNCHRO(hdci->Init.SynchroMode));
  ASSERT_PARAM(IS_DCI_CAPTURE_RATE(hdci->Init.CaptureRate));
  ASSERT_PARAM(IS_DCI_EXTENDED_DATA(hdci->Init.ExtendedDataMode));
  ASSERT_PARAM(IS_DCI_MODE_JPEG(hdci->Init.JPEGMode));

  if(hdci->State == DAL_DCI_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hdci->Lock = DAL_UNLOCKED;
    /* Init the low level hardware */
  /* Init the DCI Callback settings */
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
    hdci->FrameEventCallback = DAL_DCI_FrameEventCallback; /* Legacy weak FrameEventCallback  */ 
    hdci->VsyncEventCallback = DAL_DCI_VsyncEventCallback; /* Legacy weak VsyncEventCallback  */ 
    hdci->LineEventCallback  = DAL_DCI_LineEventCallback;  /* Legacy weak LineEventCallback   */  
    hdci->ErrorCallback      = DAL_DCI_ErrorCallback;      /* Legacy weak ErrorCallback       */ 
    
    if(hdci->MspInitCallback == NULL)  
    {
      /* Legacy weak MspInit Callback        */
      hdci->MspInitCallback = DAL_DCI_MspInit;
    }
    /* Initialize the low level hardware (MSP) */
    hdci->MspInitCallback(hdci);
#else  
    /* Init the low level hardware : GPIO, CLOCK, NVIC and DMA */
    DAL_DCI_MspInit(hdci);
#endif /* (USE_DAL_DCI_REGISTER_CALLBACKS) */
    DAL_DCI_MspInit(hdci);
  }

  /* Change the DCI state */
  hdci->State = DAL_DCI_STATE_BUSY;

  /* Set DCI parameters */
  /* Configures the HS, VS, DE and PC polarity */
  hdci->Instance->CTRL &= ~(DCI_CTRL_PXCLKPOL | DCI_CTRL_HSYNCPOL  | DCI_CTRL_VSYNCPOL  | DCI_CTRL_EXDMOD_0 |
                           DCI_CTRL_EXDMOD_1  | DCI_CTRL_FCRCFG_0 | DCI_CTRL_FCRCFG_1 | DCI_CTRL_JPGFM  |
                           DCI_CTRL_ESYNCSEL);
  hdci->Instance->CTRL |=  (uint32_t)(hdci->Init.SynchroMode | hdci->Init.CaptureRate | \
                                     hdci->Init.VSPolarity  | hdci->Init.HSPolarity  | \
                                     hdci->Init.PCKPolarity | hdci->Init.ExtendedDataMode | \
                                     hdci->Init.JPEGMode);

  if(hdci->Init.SynchroMode == DCI_SYNCHRO_EMBEDDED)
  {
    hdci->Instance->ESYNCC = (((uint32_t)hdci->Init.SyncroCode.FrameStartCode)    |
                             ((uint32_t)hdci->Init.SyncroCode.LineStartCode << DCI_POSITION_ESCR_LSC)|
                             ((uint32_t)hdci->Init.SyncroCode.LineEndCode << DCI_POSITION_ESCR_LEC) |
                             ((uint32_t)hdci->Init.SyncroCode.FrameEndCode << DCI_POSITION_ESCR_FEC));
  }

  /* Enable the Line, Vsync, Error and Overrun interrupts */
  __DAL_DCI_ENABLE_IT(hdci, DCI_IT_LINE | DCI_IT_VSYNC | DCI_IT_ERR | DCI_IT_OVR);

  /* Update error code */
  hdci->ErrorCode = DAL_DCI_ERROR_NONE;

  /* Initialize the DCI state*/
  hdci->State  = DAL_DCI_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Deinitializes the DCI peripheral registers to their default reset
  *         values.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval DAL status
  */

DAL_StatusTypeDef DAL_DCI_DeInit(DCI_HandleTypeDef *hdci)
{
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)  
  if(hdci->MspDeInitCallback == NULL)  
  {
    hdci->MspDeInitCallback = DAL_DCI_MspDeInit;
  }
  /* De-Initialize the low level hardware (MSP) */
  hdci->MspDeInitCallback(hdci);
#else
  /* DeInit the low level hardware: GPIO, CLOCK, NVIC and DMA */
  DAL_DCI_MspDeInit(hdci);
#endif /* (USE_DAL_DCI_REGISTER_CALLBACKS) */

  /* Update error code */
  hdci->ErrorCode = DAL_DCI_ERROR_NONE;

  /* Initialize the DCI state*/
  hdci->State = DAL_DCI_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hdci);

  return DAL_OK;
}

/**
  * @brief  Initializes the DCI MSP.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval None
  */
__weak void DAL_DCI_MspInit(DCI_HandleTypeDef* hdci)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdci);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_DCI_MspInit could be implemented in the user file
   */ 
}

/**
  * @brief  DeInitializes the DCI MSP.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval None
  */
__weak void DAL_DCI_MspDeInit(DCI_HandleTypeDef* hdci)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdci);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_DCI_MspDeInit could be implemented in the user file
   */
}

/**
  * @}
  */
/** @defgroup DCI_Exported_Functions_Group2 IO operation functions
  *  @brief   IO operation functions
  *
@verbatim
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure destination address and data length and
          Enables DCI DMA request and enables DCI capture
      (+) Stop the DCI capture.
      (+) Handles DCI interrupt request.

@endverbatim
  * @{
  */

/**
  * @brief  Enables DCI DMA request and enables DCI capture
  * @param  hdci     pointer to a DCI_HandleTypeDef structure that contains
  *                    the configuration information for DCI.
  * @param  DCI_Mode DCI capture mode snapshot or continuous grab.
  * @param  pData     The destination memory Buffer address (LCD Frame buffer).
  * @param  Length    The length of capture to be transferred.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DCI_Start_DMA(DCI_HandleTypeDef* hdci, uint32_t DCI_Mode, uint32_t pData, uint32_t Length)
{
  /* Initialize the second memory address */
  uint32_t SecondMemAddress = 0U;

  /* Check function parameters */
  ASSERT_PARAM(IS_DCI_CAPTURE_MODE(DCI_Mode));

  /* Process Locked */
  __DAL_LOCK(hdci);

  /* Lock the DCI peripheral state */
  hdci->State = DAL_DCI_STATE_BUSY;
  
  /* Enable DCI by setting DCIEN bit */
  __DAL_DCI_ENABLE(hdci);

  /* Configure the DCI Mode */
  hdci->Instance->CTRL &= ~(DCI_CTRL_CMODE);
  hdci->Instance->CTRL |=  (uint32_t)(DCI_Mode);

  /* Set the DMA memory0 conversion complete callback */
  hdci->DMA_Handle->XferCpltCallback = DCI_DMAXferCplt;

  /* Set the DMA error callback */
  hdci->DMA_Handle->XferErrorCallback = DCI_DMAError;

  /* Set the dma abort callback */
  hdci->DMA_Handle->XferAbortCallback = NULL;
  
  /* Reset transfer counters value */ 
  hdci->XferCount = 0U;
  hdci->XferTransferNumber = 0U;

  if(Length <= 0xFFFFU)
  {
    /* Enable the DMA Stream */
    DAL_DMA_Start_IT(hdci->DMA_Handle, (uint32_t)&hdci->Instance->DATA, (uint32_t)pData, Length);
  }
  else /* DCI_DOUBLE_BUFFER Mode */
  {
    /* Set the DMA memory1 conversion complete callback */
    hdci->DMA_Handle->XferM1CpltCallback = DCI_DMAXferCplt;

    /* Initialize transfer parameters */
    hdci->XferCount = 1U;
    hdci->XferSize = Length;
    hdci->pBuffPtr = pData;

    /* Get the number of buffer */
    while(hdci->XferSize > 0xFFFFU)
    {
      hdci->XferSize = (hdci->XferSize/2U);
      hdci->XferCount = hdci->XferCount*2U;
    }

    /* Update DCI counter and transfer number*/
    hdci->XferCount = (hdci->XferCount - 2U);
    hdci->XferTransferNumber = hdci->XferCount;

    /* Update second memory address */
    SecondMemAddress = (uint32_t)(pData + (4U*hdci->XferSize));

    /* Start DMA multi buffer transfer */
    DAL_DMAEx_MultiBufferStart_IT(hdci->DMA_Handle, (uint32_t)&hdci->Instance->DATA, (uint32_t)pData, SecondMemAddress, hdci->XferSize);
  }

  /* Enable Capture */
  hdci->Instance->CTRL |= DCI_CTRL_CEN;

  /* Release Lock */
  __DAL_UNLOCK(hdci);

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Disable DCI DMA request and Disable DCI capture
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DCI_Stop(DCI_HandleTypeDef* hdci)
{
  __IO uint32_t count = SystemCoreClock / DAL_TIMEOUT_DCI_STOP;
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hdci);
  
  /* Lock the DCI peripheral state */
  hdci->State = DAL_DCI_STATE_BUSY;

  /* Disable Capture */
  hdci->Instance->CTRL &= ~(DCI_CTRL_CEN);

  /* Check if the DCI capture effectively disabled */
  do
  {
    if (count-- == 0U)
    {
      /* Update error code */
      hdci->ErrorCode |= DAL_DCI_ERROR_TIMEOUT;

      status = DAL_TIMEOUT;
      break;
    }
  }
  while((hdci->Instance->CTRL & DCI_CTRL_CEN) != 0U);

  /* Disable the DCI */
  __DAL_DCI_DISABLE(hdci);

  /* Disable the DMA */
  DAL_DMA_Abort(hdci->DMA_Handle);

  /* Update error code */
  hdci->ErrorCode |= DAL_DCI_ERROR_NONE;

  /* Change DCI state */
  hdci->State = DAL_DCI_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hdci);

  /* Return function status */
  return status;
}

/**
  * @brief  Suspend DCI capture  
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI. 
  * @retval DAL status     
  */
DAL_StatusTypeDef DAL_DCI_Suspend(DCI_HandleTypeDef* hdci)
{
  __IO uint32_t count = SystemCoreClock / DAL_TIMEOUT_DCI_STOP;
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hdci);

  if(hdci->State == DAL_DCI_STATE_BUSY)
  {
    /* Change DCI state */
    hdci->State = DAL_DCI_STATE_SUSPENDED;

    /* Disable Capture */
    hdci->Instance->CTRL &= ~(DCI_CTRL_CEN);

    /* Check if the DCI capture effectively disabled */
    do
    {
      if (count-- == 0U)
      {        
        /* Update error code */
        hdci->ErrorCode |= DAL_DCI_ERROR_TIMEOUT;
        
        /* Change DCI state */
        hdci->State = DAL_DCI_STATE_READY;
        
        status = DAL_TIMEOUT;
        break;
      }
    }
    while((hdci->Instance->CTRL & DCI_CTRL_CEN) != 0);
  }    
  /* Process Unlocked */
  __DAL_UNLOCK(hdci);
  
  /* Return function status */
  return status;
}

/**
  * @brief  Resume DCI capture  
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI. 
  * @retval DAL status     
  */
DAL_StatusTypeDef DAL_DCI_Resume(DCI_HandleTypeDef* hdci)
{
  /* Process locked */
  __DAL_LOCK(hdci);
  
  if(hdci->State == DAL_DCI_STATE_SUSPENDED)
  {
    /* Change DCI state */
    hdci->State = DAL_DCI_STATE_BUSY;
    
    /* Disable Capture */
    hdci->Instance->CTRL |= DCI_CTRL_CEN;
  } 
  /* Process Unlocked */
  __DAL_UNLOCK(hdci);
  
  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Handles DCI interrupt request.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for the DCI.
  * @retval None
  */
void DAL_DCI_IRQHandler(DCI_HandleTypeDef *hdci)
{
  uint32_t isr_value = READ_REG(hdci->Instance->MINTSTS);

  /* Synchronization error interrupt management *******************************/
  if((isr_value & DCI_FLAG_ERRRI) == DCI_FLAG_ERRRI)
  {
    /* Clear the Synchronization error flag */
    __DAL_DCI_CLEAR_FLAG(hdci, DCI_FLAG_ERRRI);

    /* Update error code */
    hdci->ErrorCode |= DAL_DCI_ERROR_SYNC;

    /* Change DCI state */
    hdci->State = DAL_DCI_STATE_ERROR;
    
    /* Set the synchronization error callback */
    hdci->DMA_Handle->XferAbortCallback = DCI_DMAError;

    /* Abort the DMA Transfer */
    DAL_DMA_Abort_IT(hdci->DMA_Handle);
  }
  /* Overflow interrupt management ********************************************/
  if((isr_value & DCI_FLAG_OVRRI) == DCI_FLAG_OVRRI)
  {
    /* Clear the Overflow flag */
    __DAL_DCI_CLEAR_FLAG(hdci, DCI_FLAG_OVRRI);

    /* Update error code */
    hdci->ErrorCode |= DAL_DCI_ERROR_OVR;

    /* Change DCI state */
    hdci->State = DAL_DCI_STATE_ERROR;
    
    /* Set the overflow callback */
    hdci->DMA_Handle->XferAbortCallback = DCI_DMAError;

    /* Abort the DMA Transfer */
    DAL_DMA_Abort_IT(hdci->DMA_Handle);
  }
  /* Line Interrupt management ************************************************/
  if((isr_value & DCI_FLAG_LINERI) == DCI_FLAG_LINERI)
  {
    /* Clear the Line interrupt flag */
    __DAL_DCI_CLEAR_FLAG(hdci, DCI_FLAG_LINERI);
    
    /* Line interrupt Callback */
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
    /*Call registered DCI line event callback*/
    hdci->LineEventCallback(hdci);
#else  
    DAL_DCI_LineEventCallback(hdci);
#endif /* USE_DAL_DCI_REGISTER_CALLBACKS */     
  }
  /* VSYNC interrupt management ***********************************************/
  if((isr_value & DCI_FLAG_VSYNCRI) == DCI_FLAG_VSYNCRI)
  {
    /* Clear the VSYNCRI flag */
    __DAL_DCI_CLEAR_FLAG(hdci, DCI_FLAG_VSYNCRI);
    
    /* VSYNC Callback */
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
    /*Call registered DCI vsync event callback*/
    hdci->VsyncEventCallback(hdci);
#else  
    DAL_DCI_VsyncEventCallback(hdci);
#endif /* USE_DAL_DCI_REGISTER_CALLBACKS */ 
  }
  /* FRAME interrupt management ***********************************************/
  if((isr_value & DCI_FLAG_FRAMERI) == DCI_FLAG_FRAMERI)
  {
    /* When snapshot mode, disable Vsync, Error and Overrun interrupts */
    if((hdci->Instance->CTRL & DCI_CTRL_CMODE) == DCI_MODE_SNAPSHOT)
    { 
      /* Disable the Line, Vsync, Error and Overrun interrupts */
      __DAL_DCI_DISABLE_IT(hdci, DCI_IT_LINE | DCI_IT_VSYNC | DCI_IT_ERR | DCI_IT_OVR);
    }

    /* Disable the Frame interrupt */
    __DAL_DCI_DISABLE_IT(hdci, DCI_IT_FRAME);

    /* Frame Callback */
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
    /*Call registered DCI frame event callback*/
    hdci->FrameEventCallback(hdci);
#else  
    DAL_DCI_FrameEventCallback(hdci);
#endif /* USE_DAL_DCI_REGISTER_CALLBACKS */      
  }
}

/**
  * @brief  Error DCI callback.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval None
  */
__weak void DAL_DCI_ErrorCallback(DCI_HandleTypeDef *hdci)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdci);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_DCI_ErrorCallback could be implemented in the user file
   */
}

/**
  * @brief  Line Event callback.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval None
  */
__weak void DAL_DCI_LineEventCallback(DCI_HandleTypeDef *hdci)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdci);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_DCI_LineEventCallback could be implemented in the user file
   */
}

/**
  * @brief  VSYNC Event callback.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval None
  */
__weak void DAL_DCI_VsyncEventCallback(DCI_HandleTypeDef *hdci)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdci);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_DCI_VsyncEventCallback could be implemented in the user file
   */
}

/**
  * @brief  Frame Event callback.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval None
  */
__weak void DAL_DCI_FrameEventCallback(DCI_HandleTypeDef *hdci)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdci);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_DCI_FrameEventCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup DCI_Exported_Functions_Group3 Peripheral Control functions
  *  @brief    Peripheral Control functions
  *
@verbatim
 ===============================================================================
                    ##### Peripheral Control functions #####
 ===============================================================================
[..]  This section provides functions allowing to:
      (+) Configure the CROP feature.
      (+) Enable/Disable the CROP feature.

@endverbatim
  * @{
  */

/**
  * @brief  Configure the DCI CROP coordinate.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @param  X0    DCI window X offset
  * @param  Y0    DCI window Y offset
  * @param  XSize DCI Pixel per line
  * @param  YSize DCI Line number
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DCI_ConfigCrop(DCI_HandleTypeDef *hdci, uint32_t X0, uint32_t Y0, uint32_t XSize, uint32_t YSize)
{
  /* Process Locked */
  __DAL_LOCK(hdci);

  /* Lock the DCI peripheral state */
  hdci->State = DAL_DCI_STATE_BUSY;

  /* Check the parameters */
  ASSERT_PARAM(IS_DCI_WINDOW_COORDINATE(X0));
  ASSERT_PARAM(IS_DCI_WINDOW_COORDINATE(YSize));
  ASSERT_PARAM(IS_DCI_WINDOW_COORDINATE(XSize));
  ASSERT_PARAM(IS_DCI_WINDOW_HEIGHT(Y0));

  /* Configure CROP */
  hdci->Instance->CROPWSIZE = (XSize | (YSize << DCI_POSITION_CWSIZE_VLINE));
  hdci->Instance->CROPWSTAT = (X0 | (Y0 << DCI_POSITION_CWSTRT_VST));

  /* Initialize the DCI state*/
  hdci->State  = DAL_DCI_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hdci);

  return DAL_OK;
}

/**
  * @brief  Disable the Crop feature.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DCI_DisableCrop(DCI_HandleTypeDef *hdci)
{
  /* Process Locked */
  __DAL_LOCK(hdci);

  /* Lock the DCI peripheral state */
  hdci->State = DAL_DCI_STATE_BUSY;

  /* Disable DCI Crop feature */
  hdci->Instance->CTRL &= ~(uint32_t)DCI_CTRL_CROPF;

  /* Change the DCI state*/
  hdci->State = DAL_DCI_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hdci);

  return DAL_OK;
}

/**
  * @brief  Enable the Crop feature.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_DCI_EnableCrop(DCI_HandleTypeDef *hdci)
{
  /* Process Locked */
  __DAL_LOCK(hdci);

  /* Lock the DCI peripheral state */
  hdci->State = DAL_DCI_STATE_BUSY;

  /* Enable DCI Crop feature */
  hdci->Instance->CTRL |= (uint32_t)DCI_CTRL_CROPF;

  /* Change the DCI state*/
  hdci->State = DAL_DCI_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hdci);

  return DAL_OK;
}

/**
  * @brief  Set embedded synchronization delimiters unmasks.
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *               the configuration information for DCI.
  * @param  SyncUnmask pointer to a DCI_SyncUnmaskTypeDef structure that contains
  *                    the embedded synchronization delimiters unmasks.
  * @retval DAL status
  */
DAL_StatusTypeDef  DAL_DCI_ConfigSyncUnmask(DCI_HandleTypeDef *hdci, DCI_SyncUnmaskTypeDef *SyncUnmask)
{
  /* Process Locked */
  __DAL_LOCK(hdci);

  /* Lock the DCI peripheral state */
  hdci->State = DAL_DCI_STATE_BUSY;

  /* Write DCI embedded synchronization unmask register */
  hdci->Instance->ESYNCUM = (((uint32_t)SyncUnmask->FrameStartUnmask) |\
                           ((uint32_t)SyncUnmask->LineStartUnmask << DCI_ESYNCUM_LSDUM_Pos)|\
                           ((uint32_t)SyncUnmask->LineEndUnmask << DCI_ESYNCUM_LEDUM_Pos)|\
                           ((uint32_t)SyncUnmask->FrameEndUnmask << DCI_ESYNCUM_FEDUM_Pos));

  /* Change the DCI state*/
  hdci->State = DAL_DCI_STATE_READY;

  /* Process Unlocked */
  __DAL_UNLOCK(hdci);

  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup DCI_Exported_Functions_Group4 Peripheral State functions
  *  @brief    Peripheral State functions
  *
@verbatim
 ===============================================================================
               ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DCI state.
      (+) Get the specific DCI error flag.

@endverbatim
  * @{
  */ 

/**
  * @brief  Return the DCI state
  * @param  hdci pointer to a DCI_HandleTypeDef structure that contains
  *                the configuration information for DCI.
  * @retval DAL state
  */
DAL_DCI_StateTypeDef DAL_DCI_GetState(DCI_HandleTypeDef *hdci)
{
  return hdci->State;
}

/**
  * @brief  Return the DCI error code
  * @param  hdci  pointer to a DCI_HandleTypeDef structure that contains
  *               the configuration information for DCI.
  * @retval DCI Error Code
  */
uint32_t DAL_DCI_GetError(DCI_HandleTypeDef *hdci)
{
  return hdci->ErrorCode;
}

#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
/**
  * @brief DCI Callback registering
  * @param hdci        pointer to a DCI_HandleTypeDef structure that contains
  *                     the configuration information for DCI.
  * @param CallbackID   dci Callback ID
  * @param pCallback    pointer to DCI_CallbackTypeDef structure
  * @retval status
  */
DAL_StatusTypeDef DAL_DCI_RegisterCallback(DCI_HandleTypeDef *hdci, DAL_DCI_CallbackIDTypeDef CallbackID, pDCI_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;
  
  if(pCallback == NULL)
  {
    /* update the error code */
    hdci->ErrorCode |= DAL_DCI_ERROR_INVALID_CALLBACK;  
    /* update return status */
    status = DAL_ERROR;
  }
  else
  {
    if(hdci->State == DAL_DCI_STATE_READY)
    {
      switch (CallbackID)
      {
      case DAL_DCI_FRAME_EVENT_CB_ID :
        hdci->FrameEventCallback = pCallback;
        break;
        
      case DAL_DCI_VSYNC_EVENT_CB_ID :
        hdci->VsyncEventCallback = pCallback;
        break; 
        
      case DAL_DCI_LINE_EVENT_CB_ID :
        hdci->LineEventCallback = pCallback;
        break;
        
      case DAL_DCI_ERROR_CB_ID :
        hdci->ErrorCallback = pCallback;
        break;
        
      case DAL_DCI_MSPINIT_CB_ID :
        hdci->MspInitCallback = pCallback;
        break;
        
      case DAL_DCI_MSPDEINIT_CB_ID :
        hdci->MspDeInitCallback = pCallback;
        break;	  
        
      default : 
        /* Return error status */
        status =  DAL_ERROR;
        break;
      }
    }
    else if(hdci->State == DAL_DCI_STATE_RESET)
    {
      switch (CallbackID)
      {
      case DAL_DCI_MSPINIT_CB_ID :
        hdci->MspInitCallback = pCallback;
        break;

      case DAL_DCI_MSPDEINIT_CB_ID :
        hdci->MspDeInitCallback = pCallback;
        break;	  
        
      default : 
        /* update the error code */
        hdci->ErrorCode |= DAL_DCI_ERROR_INVALID_CALLBACK;
        /* update return status */
        status = DAL_ERROR;
        break;
      }		
    }  
    else
    {
      /* update the error code */
      hdci->ErrorCode |= DAL_DCI_ERROR_INVALID_CALLBACK;
      /* update return status */
      status = DAL_ERROR;
    }
  }
  
  return status;
}

/**
  * @brief DCI Callback Unregistering
  * @param hdci       dci handle
  * @param CallbackID  dci Callback ID
  * @retval status
  */
DAL_StatusTypeDef DAL_DCI_UnRegisterCallback(DCI_HandleTypeDef *hdci, DAL_DCI_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;
  
  if(hdci->State == DAL_DCI_STATE_READY)
  {
    switch (CallbackID)
    {
    case DAL_DCI_FRAME_EVENT_CB_ID :
      hdci->FrameEventCallback = DAL_DCI_FrameEventCallback;  /* Legacy weak  FrameEventCallback  */ 
      break;
      
    case DAL_DCI_VSYNC_EVENT_CB_ID :
      hdci->VsyncEventCallback = DAL_DCI_VsyncEventCallback;  /* Legacy weak VsyncEventCallback       */ 
      break;
      
    case DAL_DCI_LINE_EVENT_CB_ID :
      hdci->LineEventCallback = DAL_DCI_LineEventCallback;    /* Legacy weak LineEventCallback   */
      break;
      
    case DAL_DCI_ERROR_CB_ID :
      hdci->ErrorCallback = DAL_DCI_ErrorCallback;           /* Legacy weak ErrorCallback        */ 
      break;  
      
    case DAL_DCI_MSPINIT_CB_ID :
      hdci->MspInitCallback = DAL_DCI_MspInit;
      break;
      
    case DAL_DCI_MSPDEINIT_CB_ID :
      hdci->MspDeInitCallback = DAL_DCI_MspDeInit;
      break;	  
      
    default : 
      /* update the error code */
      hdci->ErrorCode |= DAL_DCI_ERROR_INVALID_CALLBACK;
      /* update return status */
      status = DAL_ERROR;
      break;
    }
  }
  else if(hdci->State == DAL_DCI_STATE_RESET)
  {
    switch (CallbackID)
    {
    case DAL_DCI_MSPINIT_CB_ID :
      hdci->MspInitCallback = DAL_DCI_MspInit;
      break;
      
    case DAL_DCI_MSPDEINIT_CB_ID :
      hdci->MspDeInitCallback = DAL_DCI_MspDeInit;
      break;	  
      
    default : 
      /* update the error code */
      hdci->ErrorCode |= DAL_DCI_ERROR_INVALID_CALLBACK;
      /* update return status */
      status = DAL_ERROR;
      break;
    }		
  }   
  else
  {
    /* update the error code */
    hdci->ErrorCode |= DAL_DCI_ERROR_INVALID_CALLBACK;
    /* update return status */
    status = DAL_ERROR;
  }
  
  return status;
}
#endif /* USE_DAL_DCI_REGISTER_CALLBACKS */

/**
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/** @defgroup DCI_Private_Functions DCI Private Functions
  * @{
  */

/**
  * @brief  DMA conversion complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
static void DCI_DMAXferCplt(DMA_HandleTypeDef *hdma)
{
  uint32_t tmp = 0U;
 
  DCI_HandleTypeDef* hdci = ( DCI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  
  if(hdci->XferCount != 0U)
  {
    /* Update memory 0 address location */
    tmp = ((hdci->DMA_Handle->Instance->SCFG) & DMA_SCFGx_CTARG);
    if(((hdci->XferCount % 2U) == 0U) && (tmp != 0U))
    {
      tmp = hdci->DMA_Handle->Instance->M0ADDR;
      DAL_DMAEx_ChangeMemory(hdci->DMA_Handle, (tmp + (8U*hdci->XferSize)), MEMORY0);
      hdci->XferCount--;
    }
    /* Update memory 1 address location */
    else if((hdci->DMA_Handle->Instance->SCFG & DMA_SCFGx_CTARG) == 0U)
    {
      tmp = hdci->DMA_Handle->Instance->M1ADDR;
      DAL_DMAEx_ChangeMemory(hdci->DMA_Handle, (tmp + (8U*hdci->XferSize)), MEMORY1);
      hdci->XferCount--;
    }
  }
  /* Update memory 0 address location */
  else if((hdci->DMA_Handle->Instance->SCFG & DMA_SCFGx_CTARG) != 0U)
  {
    hdci->DMA_Handle->Instance->M0ADDR = hdci->pBuffPtr;
  }
  /* Update memory 1 address location */
  else if((hdci->DMA_Handle->Instance->SCFG & DMA_SCFGx_CTARG) == 0U)
  {
    tmp = hdci->pBuffPtr;
    hdci->DMA_Handle->Instance->M1ADDR = (tmp + (4U*hdci->XferSize));
    hdci->XferCount = hdci->XferTransferNumber;
  }
  
  /* Check if the frame is transferred */
  if(hdci->XferCount == hdci->XferTransferNumber)
  {
    /* Enable the Frame interrupt */
    __DAL_DCI_ENABLE_IT(hdci, DCI_IT_FRAME);
    
    /* When snapshot mode, set dci state to ready */
    if((hdci->Instance->CTRL & DCI_CTRL_CMODE) == DCI_MODE_SNAPSHOT)
    {  
      hdci->State= DAL_DCI_STATE_READY;
    }
  }
}

/**
  * @brief  DMA error callback 
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
static void DCI_DMAError(DMA_HandleTypeDef *hdma)
{
  DCI_HandleTypeDef* hdci = ( DCI_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;
  
  if(hdci->DMA_Handle->ErrorCode != DAL_DMA_ERROR_FE)
  {
    /* Initialize the DCI state*/
    hdci->State = DAL_DCI_STATE_READY;
  }

  /* DCI error Callback */
#if (USE_DAL_DCI_REGISTER_CALLBACKS == 1)
    /*Call registered DCI error callback*/
    hdci->ErrorCallback(hdci);
#else  
  DAL_DCI_ErrorCallback(hdci);
#endif /* USE_DAL_DCI_REGISTER_CALLBACKS */   

}

/**
  * @}
  */
  
/**
  * @}
  */
#endif /* APM32F407xx || APM32F417xx */
#endif /* DAL_DCI_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
