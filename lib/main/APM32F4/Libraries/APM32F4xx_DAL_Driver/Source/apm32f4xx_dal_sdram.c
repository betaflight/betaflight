/**
  *
  * @file    apm32f4xx_dal_sdram.c
  * @brief   SDRAM DAL module driver.
  *          This file provides a generic firmware to drive SDRAM memories
  *          mounted as external device.
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
  * Copyright (c) 2016 STMicroelectronics.
  * Copyright (C) 2023 Geehy Semiconductor.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  @verbatim
  ==============================================================================
                       ##### How to use this driver #####
  ==============================================================================
  [..]
    This driver is a generic layered driver which contains a set of APIs used to
    control SDRAM memories. It uses the DMC layer functions to interface
    with SDRAM devices.
    The following sequence should be followed to configure the DMC to interface
    with SDRAM memories:

   (#) Declare a SDRAM_HandleTypeDef handle structure, for example:
          SDRAM_HandleTypeDef  hsdram

       (++) Fill the SDRAM_HandleTypeDef handle "Init" field with the allowed
            values of the structure member.

       (++) Fill the SDRAM_HandleTypeDef handle "Instance" field with a predefined
            base register instance for SDRAM device

   (#) Declare a DMC_SDRAM_TimingTypeDef structure; for example:
          DMC_SDRAM_TimingTypeDef  Timing;
      and fill its fields with the allowed values of the structure member.

   (#) Initialize the SDRAM Controller by calling the function DAL_SDRAM_Init(). This function
       performs the following sequence:

       (##) MSP hardware layer configuration using the function DAL_SDRAM_MspInit()
       (##) Control register configuration using the DMC SDRAM interface function
            DMC_SDRAM_Init()
       (##) Timing register configuration using the DMC SDRAM interface function
            DMC_SDRAM_Timing_Init()
       (##) Program the SDRAM external device by applying its initialization sequence
            according to the device plugged in your hardware. This step is mandatory
            for accessing the SDRAM device.

   (#) At this stage you can perform read/write accesses from/to the memory connected
       to the SDRAM Bank. You can perform either polling or DMA transfer using the
       following APIs:
       (++) DAL_SDRAM_Read()/DAL_SDRAM_Write() for polling read/write access
       (++) DAL_SDRAM_Read_DMA()/DAL_SDRAM_Write_DMA() for DMA read/write transfer

   (#) You can continuously monitor the SDRAM device DAL state by calling the function
       DAL_SDRAM_GetState()

   *** Callback registration ***
    =============================================
    [..]
      The compilation define  USE_DAL_SDRAM_REGISTER_CALLBACKS when set to 1
      allows the user to configure dynamically the driver callbacks.

      Use Functions DAL_SDRAM_RegisterCallback() to register a user callback,
      it allows to register following callbacks:
        (+) MspInitCallback    : SDRAM MspInit.
        (+) MspDeInitCallback  : SDRAM MspDeInit.
      This function takes as parameters the DAL peripheral handle, the Callback ID
      and a pointer to the user callback function.

      Use function DAL_SDRAM_UnRegisterCallback() to reset a callback to the default
      weak (surcharged) function. It allows to reset following callbacks:
        (+) MspInitCallback    : SDRAM MspInit.
        (+) MspDeInitCallback  : SDRAM MspDeInit.
      This function) takes as parameters the DAL peripheral handle and the Callback ID.

      By default, after the DAL_SDRAM_Init and if the state is DAL_SDRAM_STATE_RESET
      all callbacks are reset to the corresponding legacy weak (surcharged) functions.
      Exception done for MspInit and MspDeInit callbacks that are respectively
      reset to the legacy weak (surcharged) functions in the DAL_SDRAM_Init
      and  DAL_SDRAM_DeInit only when these callbacks are null (not registered beforehand).
      If not, MspInit or MspDeInit are not null, the DAL_SDRAM_Init and DAL_SDRAM_DeInit
      keep and use the user MspInit/MspDeInit callbacks (registered beforehand)

      Callbacks can be registered/unregistered in READY state only.
      Exception done for MspInit/MspDeInit callbacks that can be registered/unregistered
      in READY or RESET state, thus registered (user) MspInit/DeInit callbacks can be used
      during the Init/DeInit.
      In that case first register the MspInit/MspDeInit user callbacks
      using DAL_SDRAM_RegisterCallback before calling DAL_SDRAM_DeInit
      or DAL_SDRAM_Init function.

      When The compilation define USE_DAL_SDRAM_REGISTER_CALLBACKS is set to 0 or
      not defined, the callback registering feature is not available
      and weak (surcharged) callbacks are used.

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

#if defined(DMC)

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#ifdef DAL_SDRAM_MODULE_ENABLED

/** @defgroup SDRAM SDRAM
  * @brief SDRAM driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SDRAM_DMACplt(DMA_HandleTypeDef *hdma);
static void SDRAM_DMACpltProt(DMA_HandleTypeDef *hdma);
static void SDRAM_DMAError(DMA_HandleTypeDef *hdma);

/* Exported functions --------------------------------------------------------*/
/** @defgroup SDRAM_Exported_Functions SDRAM Exported Functions
  * @{
  */

/** @defgroup SDRAM_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and Configuration functions
  *
  @verbatim
  ==============================================================================
           ##### SDRAM Initialization and de_initialization functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to initialize/de-initialize
    the SDRAM memory

@endverbatim
  * @{
  */

/**
  * @brief  Performs the SDRAM device initialization sequence.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  Timing Pointer to SDRAM control timing structure
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Init(SDRAM_HandleTypeDef *hsdram, DMC_SDRAM_TimingTypeDef *Timing)
{
  /* Check the SDRAM handle parameter */
  if (hsdram == NULL)
  {
    return DAL_ERROR;
  }

  if (hsdram->State == DAL_SDRAM_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hsdram->Lock = DAL_UNLOCKED;
#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
    if (hsdram->MspInitCallback == NULL)
    {
      hsdram->MspInitCallback = DAL_SDRAM_MspInit;
    }
    hsdram->DmaXferCpltCallback = DAL_SDRAM_DMA_XferCpltCallback;
    hsdram->DmaXferErrorCallback = DAL_SDRAM_DMA_XferErrorCallback;

    /* Init the low level hardware */
    hsdram->MspInitCallback(hsdram);
#else
    /* Initialize the low level hardware (MSP) */
    DAL_SDRAM_MspInit(hsdram);
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */
  }

  /* Initialize the SDRAM controller state */
  hsdram->State = DAL_SDRAM_STATE_BUSY;

  /* Initialize SDRAM control Interface */
  (void)DMC_SDRAM_Init(hsdram->Instance, &(hsdram->Init));

  /* Initialize SDRAM timing Interface */
  (void)DMC_SDRAM_Timing_Init(hsdram->Instance, Timing);
  /* Update the SDRAM controller state */
  hsdram->State = DAL_SDRAM_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Perform the SDRAM device initialization sequence.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_DeInit(SDRAM_HandleTypeDef *hsdram)
{
#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
  if (hsdram->MspDeInitCallback == NULL)
  {
    hsdram->MspDeInitCallback = DAL_SDRAM_MspDeInit;
  }

  /* DeInit the low level hardware */
  hsdram->MspDeInitCallback(hsdram);
#else
  /* Initialize the low level hardware (MSP) */
  DAL_SDRAM_MspDeInit(hsdram);
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */

  /* Configure the SDRAM registers with their reset values */
  (void)DMC_SDRAM_DeInit(hsdram->Instance);

  /* Reset the SDRAM controller state */
  hsdram->State = DAL_SDRAM_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hsdram);

  return DAL_OK;
}

/**
  * @brief  SDRAM MSP Init.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @retval None
  */
__weak void DAL_SDRAM_MspInit(SDRAM_HandleTypeDef *hsdram)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsdram);

  /* NOTE: This function Should not be modified, when the callback is needed,
            the DAL_SDRAM_MspInit could be implemented in the user file
   */
}

/**
  * @brief  SDRAM MSP DeInit.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @retval None
  */
__weak void DAL_SDRAM_MspDeInit(SDRAM_HandleTypeDef *hsdram)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsdram);

  /* NOTE: This function Should not be modified, when the callback is needed,
            the DAL_SDRAM_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  DMA transfer complete callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
__weak void DAL_SDRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma);

  /* NOTE: This function Should not be modified, when the callback is needed,
            the DAL_SDRAM_DMA_XferCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  DMA transfer complete error callback.
  * @param  hdma DMA handle
  * @retval None
  */
__weak void DAL_SDRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma);

  /* NOTE: This function Should not be modified, when the callback is needed,
            the DAL_SDRAM_DMA_XferErrorCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup SDRAM_Exported_Functions_Group2 Input and Output functions
  * @brief    Input Output and memory control functions
  *
  @verbatim
  ==============================================================================
                    ##### SDRAM Input and Output functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to use and control the SDRAM memory

@endverbatim
  * @{
  */

/**
  * @brief  Reads 8-bit data buffer from the SDRAM memory.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  pAddress Pointer to read start address
  * @param  pDstBuffer Pointer to destination buffer
  * @param  BufferSize Size of the buffer to read from memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Read_8b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint8_t *pDstBuffer,
                                    uint32_t BufferSize)
{
  uint32_t size;
  __IO uint8_t *pSdramAddress = (uint8_t *)pAddress;
  uint8_t *pdestbuff = pDstBuffer;
  DAL_SDRAM_StateTypeDef state = hsdram->State;
  
  /* Check the SDRAM controller state */
  if (state == DAL_SDRAM_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (state == DAL_SDRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsdram);

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Read data from source */
    for (size = BufferSize; size != 0U; size--)
    {
      *pdestbuff = *(__IO uint8_t *)pSdramAddress;
      pdestbuff++;
      pSdramAddress++;
    }

    /* Update the SDRAM controller state */
    hsdram->State = state;

    /* Process Unlocked */
    __DAL_UNLOCK(hsdram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Writes 8-bit data buffer to SDRAM memory.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  pAddress Pointer to write start address
  * @param  pSrcBuffer Pointer to source buffer to write
  * @param  BufferSize Size of the buffer to write to memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Write_8b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint8_t *pSrcBuffer,
                                     uint32_t BufferSize)
{
  uint32_t size;
  __IO uint8_t *pSdramAddress = (uint8_t *)pAddress;
  uint8_t *psrcbuff = pSrcBuffer;

  /* Check the SDRAM controller state */
  if (hsdram->State == DAL_SDRAM_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (hsdram->State == DAL_SDRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsdram);

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Write data to memory */
    for (size = BufferSize; size != 0U; size--)
    {
      *(__IO uint8_t *)pSdramAddress = *psrcbuff;
      psrcbuff++;
      pSdramAddress++;
    }

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hsdram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Reads 16-bit data buffer from the SDRAM memory.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  pAddress Pointer to read start address
  * @param  pDstBuffer Pointer to destination buffer
  * @param  BufferSize Size of the buffer to read from memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Read_16b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint16_t *pDstBuffer,
                                     uint32_t BufferSize)
{
  uint32_t size;
  __IO uint32_t *pSdramAddress = pAddress;
  uint16_t *pdestbuff = pDstBuffer;
  DAL_SDRAM_StateTypeDef state = hsdram->State;

  /* Check the SDRAM controller state */
  if (state == DAL_SDRAM_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (state == DAL_SDRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsdram);

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Read data from memory */
    for (size = BufferSize; size >= 2U ; size -= 2U)
    {
      *pdestbuff = (uint16_t)((*pSdramAddress) & 0x0000FFFFU);
      pdestbuff++;
      *pdestbuff = (uint16_t)(((*pSdramAddress) & 0xFFFF0000U) >> 16U);
      pdestbuff++;
      pSdramAddress++;
    }

    /* Read last 16-bits if size is not 32-bits multiple */
    if ((BufferSize % 2U) != 0U)
    {
      *pdestbuff = (uint16_t)((*pSdramAddress) & 0x0000FFFFU);
    }

    /* Update the SDRAM controller state */
    hsdram->State = state;

    /* Process Unlocked */
    __DAL_UNLOCK(hsdram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Writes 16-bit data buffer to SDRAM memory.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  pAddress Pointer to write start address
  * @param  pSrcBuffer Pointer to source buffer to write
  * @param  BufferSize Size of the buffer to write to memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Write_16b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint16_t *pSrcBuffer,
                                      uint32_t BufferSize)
{
  uint32_t size;
  __IO uint32_t *psdramaddress = pAddress;
  uint16_t *psrcbuff = pSrcBuffer;

  /* Check the SDRAM controller state */
  if (hsdram->State == DAL_SDRAM_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (hsdram->State == DAL_SDRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsdram);

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Write data to memory */
    for (size = BufferSize; size >= 2U ; size -= 2U)
    {
      *psdramaddress = (uint32_t)(*psrcbuff);
      psrcbuff++;
      *psdramaddress |= ((uint32_t)(*psrcbuff) << 16U);
      psrcbuff++;
      psdramaddress++;
    }

    /* Write last 16-bits if size is not 32-bits multiple */
    if ((BufferSize % 2U) != 0U)
    {
      *psdramaddress = ((uint32_t)(*psrcbuff) & 0x0000FFFFU) | ((*psdramaddress) & 0xFFFF0000U);
    }

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hsdram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Reads 32-bit data buffer from the SDRAM memory.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  pAddress Pointer to read start address
  * @param  pDstBuffer Pointer to destination buffer
  * @param  BufferSize Size of the buffer to read from memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Read_32b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                     uint32_t BufferSize)
{
  uint32_t size;
  __IO uint32_t *pSdramAddress = (uint32_t *)pAddress;
  uint32_t *pdestbuff = pDstBuffer;
  DAL_SDRAM_StateTypeDef state = hsdram->State;

  /* Check the SDRAM controller state */
  if (state == DAL_SDRAM_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (state == DAL_SDRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsdram);

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Read data from source */
    for (size = BufferSize; size != 0U; size--)
    {
      *pdestbuff = *(__IO uint32_t *)pSdramAddress;
      pdestbuff++;
      pSdramAddress++;
    }

    /* Update the SDRAM controller state */
    hsdram->State = state;

    /* Process Unlocked */
    __DAL_UNLOCK(hsdram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Writes 32-bit data buffer to SDRAM memory.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  pAddress Pointer to write start address
  * @param  pSrcBuffer Pointer to source buffer to write
  * @param  BufferSize Size of the buffer to write to memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Write_32b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                      uint32_t BufferSize)
{
  uint32_t size;
  __IO uint32_t *pSdramAddress = pAddress;
  uint32_t *psrcbuff = pSrcBuffer;

  /* Check the SDRAM controller state */
  if (hsdram->State == DAL_SDRAM_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (hsdram->State == DAL_SDRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsdram);

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Write data to memory */
    for (size = BufferSize; size != 0U; size--)
    {
      *pSdramAddress = *psrcbuff;
      psrcbuff++;
      pSdramAddress++;
    }

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_READY;

    /* Process Unlocked */
    __DAL_UNLOCK(hsdram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Reads a Words data from the SDRAM memory using DMA transfer.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  pAddress Pointer to read start address
  * @param  pDstBuffer Pointer to destination buffer
  * @param  BufferSize Size of the buffer to read from memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Read_DMA(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                     uint32_t BufferSize)
{
  DAL_StatusTypeDef status;
  DAL_SDRAM_StateTypeDef state = hsdram->State;

  /* Check the SDRAM controller state */
  if (state == DAL_SDRAM_STATE_BUSY)
  {
    status = DAL_BUSY;
  }
  else if (state == DAL_SDRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsdram);

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Configure DMA user callbacks */
    if (state == DAL_SDRAM_STATE_READY)
    {
      hsdram->hdma->XferCpltCallback = SDRAM_DMACplt;
    }
    else
    {
      hsdram->hdma->XferCpltCallback = SDRAM_DMACpltProt;
    }
    hsdram->hdma->XferErrorCallback = SDRAM_DMAError;

    /* Enable the DMA Stream */
    status = DAL_DMA_Start_IT(hsdram->hdma, (uint32_t)pAddress, (uint32_t)pDstBuffer, (uint32_t)BufferSize);

    /* Process Unlocked */
    __DAL_UNLOCK(hsdram);
  }
  else
  {
    status = DAL_ERROR;
  }

  return status;
}

/**
  * @brief  Writes a Words data buffer to SDRAM memory using DMA transfer.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  pAddress Pointer to write start address
  * @param  pSrcBuffer Pointer to source buffer to write
  * @param  BufferSize Size of the buffer to write to memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_Write_DMA(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                      uint32_t BufferSize)
{
  DAL_StatusTypeDef status;

  /* Check the SDRAM controller state */
  if (hsdram->State == DAL_SDRAM_STATE_BUSY)
  {
    status = DAL_BUSY;
  }
  else if (hsdram->State == DAL_SDRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsdram);

    /* Update the SDRAM controller state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Configure DMA user callbacks */
    hsdram->hdma->XferCpltCallback = SDRAM_DMACplt;
    hsdram->hdma->XferErrorCallback = SDRAM_DMAError;

    /* Enable the DMA Stream */
    status = DAL_DMA_Start_IT(hsdram->hdma, (uint32_t)pSrcBuffer, (uint32_t)pAddress, (uint32_t)BufferSize);

    /* Process Unlocked */
    __DAL_UNLOCK(hsdram);
  }
  else
  {
    status = DAL_ERROR;
  }

  return status;
}

#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User SDRAM Callback
  *         To be used instead of the weak (surcharged) predefined callback
  * @param hsdram : SDRAM handle
  * @param CallbackId : ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_SDRAM_MSP_INIT_CB_ID       SDRAM MspInit callback ID
  *          @arg @ref DAL_SDRAM_MSP_DEINIT_CB_ID     SDRAM MspDeInit callback ID
  * @param pCallback : pointer to the Callback function
  * @retval status
  */
DAL_StatusTypeDef DAL_SDRAM_RegisterCallback(SDRAM_HandleTypeDef *hsdram, DAL_SDRAM_CallbackIDTypeDef CallbackId,
                                             pSDRAM_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;
  DAL_SDRAM_StateTypeDef state;

  if (pCallback == NULL)
  {
    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hsdram);

  state = hsdram->State;
  if (state == DAL_SDRAM_STATE_READY)
  {
    switch (CallbackId)
    {
      case DAL_SDRAM_MSP_INIT_CB_ID :
        hsdram->MspInitCallback = pCallback;
        break;
      case DAL_SDRAM_MSP_DEINIT_CB_ID :
        hsdram->MspDeInitCallback = pCallback;
        break;
      default :
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hsdram->State == DAL_SDRAM_STATE_RESET)
  {
    switch (CallbackId)
    {
      case DAL_SDRAM_MSP_INIT_CB_ID :
        hsdram->MspInitCallback = pCallback;
        break;
      case DAL_SDRAM_MSP_DEINIT_CB_ID :
        hsdram->MspDeInitCallback = pCallback;
        break;
      default :
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* update return status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsdram);
  return status;
}

/**
  * @brief  Unregister a User SDRAM Callback
  *         SDRAM Callback is redirected to the weak (surcharged) predefined callback
  * @param hsdram : SDRAM handle
  * @param CallbackId : ID of the callback to be unregistered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_SDRAM_MSP_INIT_CB_ID       SDRAM MspInit callback ID
  *          @arg @ref DAL_SDRAM_MSP_DEINIT_CB_ID     SDRAM MspDeInit callback ID
  *          @arg @ref DAL_SDRAM_DMA_XFER_CPLT_CB_ID  SDRAM DMA Xfer Complete callback ID
  *          @arg @ref DAL_SDRAM_DMA_XFER_ERR_CB_ID   SDRAM DMA Xfer Error callback ID
  * @retval status
  */
DAL_StatusTypeDef DAL_SDRAM_UnRegisterCallback(SDRAM_HandleTypeDef *hsdram, DAL_SDRAM_CallbackIDTypeDef CallbackId)
{
  DAL_StatusTypeDef status = DAL_OK;
  DAL_SDRAM_StateTypeDef state;

  /* Process locked */
  __DAL_LOCK(hsdram);

  state = hsdram->State;
  if (state == DAL_SDRAM_STATE_READY)
  {
    switch (CallbackId)
    {
      case DAL_SDRAM_MSP_INIT_CB_ID :
        hsdram->MspInitCallback = DAL_SDRAM_MspInit;
        break;
      case DAL_SDRAM_MSP_DEINIT_CB_ID :
        hsdram->MspDeInitCallback = DAL_SDRAM_MspDeInit;
        break;
      case DAL_SDRAM_DMA_XFER_CPLT_CB_ID :
        hsdram->DmaXferCpltCallback = DAL_SDRAM_DMA_XferCpltCallback;
        break;
      case DAL_SDRAM_DMA_XFER_ERR_CB_ID :
        hsdram->DmaXferErrorCallback = DAL_SDRAM_DMA_XferErrorCallback;
        break;
      default :
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hsdram->State == DAL_SDRAM_STATE_RESET)
  {
    switch (CallbackId)
    {
      case DAL_SDRAM_MSP_INIT_CB_ID :
        hsdram->MspInitCallback = DAL_SDRAM_MspInit;
        break;
      case DAL_SDRAM_MSP_DEINIT_CB_ID :
        hsdram->MspDeInitCallback = DAL_SDRAM_MspDeInit;
        break;
      default :
        /* update return status */
        status = DAL_ERROR;
        break;
    }
  }
  else
  {
    /* update return status */
    status = DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsdram);
  return status;
}

/**
  * @brief  Register a User SDRAM Callback for DMA transfers
  *         To be used instead of the weak (surcharged) predefined callback
  * @param hsdram : SDRAM handle
  * @param CallbackId : ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_SDRAM_DMA_XFER_CPLT_CB_ID  SDRAM DMA Xfer Complete callback ID
  *          @arg @ref DAL_SDRAM_DMA_XFER_ERR_CB_ID   SDRAM DMA Xfer Error callback ID
  * @param pCallback : pointer to the Callback function
  * @retval status
  */
DAL_StatusTypeDef DAL_SDRAM_RegisterDmaCallback(SDRAM_HandleTypeDef *hsdram, DAL_SDRAM_CallbackIDTypeDef CallbackId,
                                                pSDRAM_DmaCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;
  DAL_SDRAM_StateTypeDef state;

  if (pCallback == NULL)
  {
    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hsdram);

  state = hsdram->State;
  if (state == DAL_SDRAM_STATE_READY)
  {
    switch (CallbackId)
    {
      case DAL_SDRAM_DMA_XFER_CPLT_CB_ID :
        hsdram->DmaXferCpltCallback = pCallback;
        break;
      case DAL_SDRAM_DMA_XFER_ERR_CB_ID :
        hsdram->DmaXferErrorCallback = pCallback;
        break;
      default :
        /* update return status */
        status = DAL_ERROR;
        break;
    }
  }
  else
  {
    /* update return status */
    status = DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hsdram);
  return status;
}
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup SDRAM_Exported_Functions_Group3 Control functions
  *  @brief   management functions
  *
@verbatim
  ==============================================================================
                         ##### SDRAM Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control dynamically
    the SDRAM interface.

@endverbatim
  * @{
  */

/**
  * @brief  Programs the SDRAM Memory Refresh period.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  RefreshPeriod The SDRAM refresh period value
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_ProgramRefreshPeriod(SDRAM_HandleTypeDef *hsdram, uint32_t RefreshPeriod)
{
  /* Check the SDRAM controller state */
  if (hsdram->State == DAL_SDRAM_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (hsdram->State == DAL_SDRAM_STATE_READY)
  {
    /* Update the SDRAM state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Program the refresh period */
    (void)DMC_SDRAM_ProgramRefreshPeriod(hsdram->Instance, RefreshPeriod);

    /* Update the SDRAM state */
    hsdram->State = DAL_SDRAM_STATE_READY;
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Sets the Number of consecutive SDRAM Memory open bank.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @param  OpenBankNumber The SDRAM open bank number
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SDRAM_SetOpenBankNumber(SDRAM_HandleTypeDef *hsdram, uint32_t OpenBankNumber)
{
  /* Check the SDRAM controller state */
  if (hsdram->State == DAL_SDRAM_STATE_BUSY)
  {
    return DAL_BUSY;
  }
  else if (hsdram->State == DAL_SDRAM_STATE_READY)
  {
    /* Update the SDRAM state */
    hsdram->State = DAL_SDRAM_STATE_BUSY;

    /* Set the open bank number */
    (void)DMC_SDRAM_SetOpenBankNumber(hsdram->Instance, OpenBankNumber);

    /* Update the SDRAM state */
    hsdram->State = DAL_SDRAM_STATE_READY;
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Returns the SDRAM memory current mode.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @retval The SDRAM memory mode.
  */
uint32_t DAL_SDRAM_GetModeStatus(SDRAM_HandleTypeDef *hsdram)
{
  /* Return the SDRAM memory current mode */
  return (DMC_SDRAM_GetModeStatus(hsdram->Instance));
}

/**
  * @}
  */

/** @defgroup SDRAM_Exported_Functions_Group4 State functions
  *  @brief   Peripheral State functions
  *
@verbatim
  ==============================================================================
                      ##### SDRAM State functions #####
  ==============================================================================
  [..]
    This subsection permits to get in run-time the status of the SDRAM controller
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the SDRAM state.
  * @param  hsdram pointer to a SDRAM_HandleTypeDef structure that contains
  *                the configuration information for SDRAM module.
  * @retval DAL state
  */
DAL_SDRAM_StateTypeDef DAL_SDRAM_GetState(SDRAM_HandleTypeDef *hsdram)
{
  return hsdram->State;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  DMA SDRAM process complete callback.
  * @param  hdma : DMA handle
  * @retval None
  */
static void SDRAM_DMACplt(DMA_HandleTypeDef *hdma)
{
  SDRAM_HandleTypeDef *hsdram = (SDRAM_HandleTypeDef *)(hdma->Parent);

  /* Disable the DMA channel */
  __DAL_DMA_DISABLE(hdma);

  /* Update the SDRAM controller state */
  hsdram->State = DAL_SDRAM_STATE_READY;

#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
  hsdram->DmaXferCpltCallback(hdma);
#else
  DAL_SDRAM_DMA_XferCpltCallback(hdma);
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SRAM process complete callback.
  * @param  hdma : DMA handle
  * @retval None
  */
static void SDRAM_DMACpltProt(DMA_HandleTypeDef *hdma)
{
  SDRAM_HandleTypeDef *hsdram = (SDRAM_HandleTypeDef *)(hdma->Parent);

  /* Disable the DMA channel */
  __DAL_DMA_DISABLE(hdma);

  /* Update the SDRAM controller state */
  hsdram->State = DAL_SDRAM_STATE_WRITE_PROTECTED;

#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
  hsdram->DmaXferCpltCallback(hdma);
#else
  DAL_SDRAM_DMA_XferCpltCallback(hdma);
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SDRAM error callback.
  * @param  hdma : DMA handle
  * @retval None
  */
static void SDRAM_DMAError(DMA_HandleTypeDef *hdma)
{
  SDRAM_HandleTypeDef *hsdram = (SDRAM_HandleTypeDef *)(hdma->Parent);

  /* Disable the DMA channel */
  __DAL_DMA_DISABLE(hdma);

  /* Update the SDRAM controller state */
  hsdram->State = DAL_SDRAM_STATE_ERROR;

#if (USE_DAL_SDRAM_REGISTER_CALLBACKS == 1)
  hsdram->DmaXferErrorCallback(hdma);
#else
  DAL_SDRAM_DMA_XferErrorCallback(hdma);
#endif /* USE_DAL_SDRAM_REGISTER_CALLBACKS */
}

/**
  * @}
  */

#endif /* DAL_SDRAM_MODULE_ENABLED */

/**
  * @}
  */

#endif /* DMC */
