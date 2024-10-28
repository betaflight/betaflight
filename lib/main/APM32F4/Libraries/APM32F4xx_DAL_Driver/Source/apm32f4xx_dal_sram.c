/**
  *
  * @file    apm32f4xx_dal_sram.c
  * @brief   SRAM DAL module driver.
  *          This file provides a generic firmware to drive SRAM memories
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
    control SRAM memories. It uses the FMC layer functions to interface
    with SRAM devices.
    The following sequence should be followed to configure the FMC/EMMC to interface
    with SRAM/PSRAM memories:

   (#) Declare a SRAM_HandleTypeDef handle structure, for example:
          SRAM_HandleTypeDef  hsram; and:

       (++) Fill the SRAM_HandleTypeDef handle "Init" field with the allowed
            values of the structure member.

       (++) Fill the SRAM_HandleTypeDef handle "Instance" field with a predefined
            base register instance for NOR or SRAM device

       (++) Fill the SRAM_HandleTypeDef handle "Extended" field with a predefined
            base register instance for NOR or SRAM extended mode

   (#) Declare two FMC_NORSRAM_TimingTypeDef structures, for both normal and extended
       mode timings; for example:
          FMC_NORSRAM_TimingTypeDef  Timing and FMC_NORSRAM_TimingTypeDef  ExTiming;
      and fill its fields with the allowed values of the structure member.

   (#) Initialize the SRAM Controller by calling the function DAL_SRAM_Init(). This function
       performs the following sequence:

       (##) MSP hardware layer configuration using the function DAL_SRAM_MspInit()
       (##) Control register configuration using the FMC NORSRAM interface function
            FMC_NORSRAM_Init()
       (##) Timing register configuration using the FMC NORSRAM interface function
            FMC_NORSRAM_Timing_Init()
       (##) Extended mode Timing register configuration using the FMC NORSRAM interface function
            FMC_NORSRAM_Extended_Timing_Init()
       (##) Enable the SRAM device using the macro __FMC_NORSRAM_ENABLE()

   (#) At this stage you can perform read/write accesses from/to the memory connected
       to the NOR/SRAM Bank. You can perform either polling or DMA transfer using the
       following APIs:
       (++) DAL_SRAM_Read()/DAL_SRAM_Write() for polling read/write access
       (++) DAL_SRAM_Read_DMA()/DAL_SRAM_Write_DMA() for DMA read/write transfer

   (#) You can also control the SRAM device by calling the control APIs DAL_SRAM_WriteOperation_Enable()/
       DAL_SRAM_WriteOperation_Disable() to respectively enable/disable the SRAM write operation

   (#) You can continuously monitor the SRAM device DAL state by calling the function
       DAL_SRAM_GetState()

       *** Callback registration ***
    =============================================
    [..]
      The compilation define  USE_DAL_SRAM_REGISTER_CALLBACKS when set to 1
      allows the user to configure dynamically the driver callbacks.

      Use Functions DAL_SRAM_RegisterCallback() to register a user callback,
      it allows to register following callbacks:
        (+) MspInitCallback    : SRAM MspInit.
        (+) MspDeInitCallback  : SRAM MspDeInit.
      This function takes as parameters the DAL peripheral handle, the Callback ID
      and a pointer to the user callback function.

      Use function DAL_SRAM_UnRegisterCallback() to reset a callback to the default
      weak (surcharged) function. It allows to reset following callbacks:
        (+) MspInitCallback    : SRAM MspInit.
        (+) MspDeInitCallback  : SRAM MspDeInit.
      This function) takes as parameters the DAL peripheral handle and the Callback ID.

      By default, after the DAL_SRAM_Init and if the state is DAL_SRAM_STATE_RESET
      all callbacks are reset to the corresponding legacy weak (surcharged) functions.
      Exception done for MspInit and MspDeInit callbacks that are respectively
      reset to the legacy weak (surcharged) functions in the DAL_SRAM_Init
      and  DAL_SRAM_DeInit only when these callbacks are null (not registered beforehand).
      If not, MspInit or MspDeInit are not null, the DAL_SRAM_Init and DAL_SRAM_DeInit
      keep and use the user MspInit/MspDeInit callbacks (registered beforehand)

      Callbacks can be registered/unregistered in READY state only.
      Exception done for MspInit/MspDeInit callbacks that can be registered/unregistered
      in READY or RESET state, thus registered (user) MspInit/DeInit callbacks can be used
      during the Init/DeInit.
      In that case first register the MspInit/MspDeInit user callbacks
      using DAL_SRAM_RegisterCallback before calling DAL_SRAM_DeInit
      or DAL_SRAM_Init function.

      When The compilation define USE_DAL_SRAM_REGISTER_CALLBACKS is set to 0 or
      not defined, the callback registering feature is not available
      and weak (surcharged) callbacks are used.

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

#if defined(SMC_Bank1)

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

#ifdef DAL_SRAM_MODULE_ENABLED

/** @defgroup SRAM SRAM
  * @brief SRAM driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SRAM_DMACplt(DMA_HandleTypeDef *hdma);
static void SRAM_DMACpltProt(DMA_HandleTypeDef *hdma);
static void SRAM_DMAError(DMA_HandleTypeDef *hdma);

/* Exported functions --------------------------------------------------------*/

/** @defgroup SRAM_Exported_Functions SRAM Exported Functions
  * @{
  */

/** @defgroup SRAM_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and Configuration functions.
  *
  @verbatim
  ==============================================================================
           ##### SRAM Initialization and de_initialization functions #####
  ==============================================================================
    [..]  This section provides functions allowing to initialize/de-initialize
          the SRAM memory

@endverbatim
  * @{
  */

/**
  * @brief  Performs the SRAM device initialization sequence
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  Timing Pointer to SRAM control timing structure
  * @param  ExtTiming Pointer to SRAM extended mode timing structure
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Init(SRAM_HandleTypeDef *hsram, FMC_NORSRAM_TimingTypeDef *Timing,
                                FMC_NORSRAM_TimingTypeDef *ExtTiming)
{
  /* Check the SRAM handle parameter */
  if (hsram == NULL)
  {
    return DAL_ERROR;
  }

  if (hsram->State == DAL_SRAM_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hsram->Lock = DAL_UNLOCKED;

#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
    if (hsram->MspInitCallback == NULL)
    {
      hsram->MspInitCallback = DAL_SRAM_MspInit;
    }
    hsram->DmaXferCpltCallback = DAL_SRAM_DMA_XferCpltCallback;
    hsram->DmaXferErrorCallback = DAL_SRAM_DMA_XferErrorCallback;

    /* Init the low level hardware */
    hsram->MspInitCallback(hsram);
#else
    /* Initialize the low level hardware (MSP) */
    DAL_SRAM_MspInit(hsram);
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS */
  }

  /* Initialize SRAM control Interface */
  (void)FMC_NORSRAM_Init(hsram->Instance, &(hsram->Init));

  /* Initialize SRAM timing Interface */
  (void)FMC_NORSRAM_Timing_Init(hsram->Instance, Timing, hsram->Init.NSBank);

  /* Initialize SRAM extended mode timing Interface */
  (void)FMC_NORSRAM_Extended_Timing_Init(hsram->Extended, ExtTiming, hsram->Init.NSBank,
                                         hsram->Init.ExtendedMode);

  /* Enable the NORSRAM device */
  __FMC_NORSRAM_ENABLE(hsram->Instance, hsram->Init.NSBank);

  /* Initialize the SRAM controller state */
  hsram->State = DAL_SRAM_STATE_READY;

  return DAL_OK;
}

/**
  * @brief  Performs the SRAM device De-initialization sequence.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_DeInit(SRAM_HandleTypeDef *hsram)
{
#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
  if (hsram->MspDeInitCallback == NULL)
  {
    hsram->MspDeInitCallback = DAL_SRAM_MspDeInit;
  }

  /* DeInit the low level hardware */
  hsram->MspDeInitCallback(hsram);
#else
  /* De-Initialize the low level hardware (MSP) */
  DAL_SRAM_MspDeInit(hsram);
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS */

  /* Configure the SRAM registers with their reset values */
  (void)FMC_NORSRAM_DeInit(hsram->Instance, hsram->Extended, hsram->Init.NSBank);

  /* Reset the SRAM controller state */
  hsram->State = DAL_SRAM_STATE_RESET;

  /* Release Lock */
  __DAL_UNLOCK(hsram);

  return DAL_OK;
}

/**
  * @brief  SRAM MSP Init.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @retval None
  */
__weak void DAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsram);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_SRAM_MspInit could be implemented in the user file
   */
}

/**
  * @brief  SRAM MSP DeInit.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @retval None
  */
__weak void DAL_SRAM_MspDeInit(SRAM_HandleTypeDef *hsram)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hsram);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_SRAM_MspDeInit could be implemented in the user file
   */
}

/**
  * @brief  DMA transfer complete callback.
  * @param  hdma pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @retval None
  */
__weak void DAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_SRAM_DMA_XferCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  DMA transfer complete error callback.
  * @param  hdma pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @retval None
  */
__weak void DAL_SRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hdma);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the DAL_SRAM_DMA_XferErrorCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup SRAM_Exported_Functions_Group2 Input Output and memory control functions
  * @brief    Input Output and memory control functions
  *
  @verbatim
  ==============================================================================
                  ##### SRAM Input and Output functions #####
  ==============================================================================
  [..]
    This section provides functions allowing to use and control the SRAM memory

@endverbatim
  * @{
  */

/**
  * @brief  Reads 8-bit buffer from SRAM memory.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  pAddress Pointer to read start address
  * @param  pDstBuffer Pointer to destination buffer
  * @param  BufferSize Size of the buffer to read from memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Read_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pDstBuffer,
                                   uint32_t BufferSize)
{
  uint32_t size;
  __IO uint8_t *psramaddress = (uint8_t *)pAddress;
  uint8_t *pdestbuff = pDstBuffer;
  DAL_SRAM_StateTypeDef state = hsram->State;

  /* Check the SRAM controller state */
  if ((state == DAL_SRAM_STATE_READY) || (state == DAL_SRAM_STATE_PROTECTED))
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Read data from memory */
    for (size = BufferSize; size != 0U; size--)
    {
      *pdestbuff = *psramaddress;
      pdestbuff++;
      psramaddress++;
    }

    /* Update the SRAM controller state */
    hsram->State = state;

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Writes 8-bit buffer to SRAM memory.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  pAddress Pointer to write start address
  * @param  pSrcBuffer Pointer to source buffer to write
  * @param  BufferSize Size of the buffer to write to memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Write_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pSrcBuffer,
                                    uint32_t BufferSize)
{
  uint32_t size;
  __IO uint8_t *psramaddress = (uint8_t *)pAddress;
  uint8_t *psrcbuff = pSrcBuffer;

  /* Check the SRAM controller state */
  if (hsram->State == DAL_SRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Write data to memory */
    for (size = BufferSize; size != 0U; size--)
    {
      *psramaddress = *psrcbuff;
      psrcbuff++;
      psramaddress++;
    }

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_READY;

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Reads 16-bit buffer from SRAM memory.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  pAddress Pointer to read start address
  * @param  pDstBuffer Pointer to destination buffer
  * @param  BufferSize Size of the buffer to read from memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Read_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pDstBuffer,
                                    uint32_t BufferSize)
{
  uint32_t size;
  __IO uint32_t *psramaddress = pAddress;
  uint16_t *pdestbuff = pDstBuffer;
  uint8_t limit;
  DAL_SRAM_StateTypeDef state = hsram->State;

  /* Check the SRAM controller state */
  if ((state == DAL_SRAM_STATE_READY) || (state == DAL_SRAM_STATE_PROTECTED))
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Check if the size is a 32-bits multiple */
    limit = (((BufferSize % 2U) != 0U) ? 1U : 0U);

    /* Read data from memory */
    for (size = BufferSize; size != limit; size -= 2U)
    {
      *pdestbuff = (uint16_t)((*psramaddress) & 0x0000FFFFU);
      pdestbuff++;
      *pdestbuff = (uint16_t)(((*psramaddress) & 0xFFFF0000U) >> 16U);
      pdestbuff++;
      psramaddress++;
    }

    /* Read last 16-bits if size is not 32-bits multiple */
    if (limit != 0U)
    {
      *pdestbuff = (uint16_t)((*psramaddress) & 0x0000FFFFU);
    }

    /* Update the SRAM controller state */
    hsram->State = state;

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Writes 16-bit buffer to SRAM memory.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  pAddress Pointer to write start address
  * @param  pSrcBuffer Pointer to source buffer to write
  * @param  BufferSize Size of the buffer to write to memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Write_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pSrcBuffer,
                                     uint32_t BufferSize)
{
  uint32_t size;
  __IO uint32_t *psramaddress = pAddress;
  uint16_t *psrcbuff = pSrcBuffer;
  uint8_t limit;

  /* Check the SRAM controller state */
  if (hsram->State == DAL_SRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Check if the size is a 32-bits multiple */
    limit = (((BufferSize % 2U) != 0U) ? 1U : 0U);

    /* Write data to memory */
    for (size = BufferSize; size != limit; size -= 2U)
    {
      *psramaddress = (uint32_t)(*psrcbuff);
      psrcbuff++;
      *psramaddress |= ((uint32_t)(*psrcbuff) << 16U);
      psrcbuff++;
      psramaddress++;
    }

    /* Write last 16-bits if size is not 32-bits multiple */
    if (limit != 0U)
    {
      *psramaddress = ((uint32_t)(*psrcbuff) & 0x0000FFFFU) | ((*psramaddress) & 0xFFFF0000U);
    }

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_READY;

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Reads 32-bit buffer from SRAM memory.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  pAddress Pointer to read start address
  * @param  pDstBuffer Pointer to destination buffer
  * @param  BufferSize Size of the buffer to read from memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Read_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                    uint32_t BufferSize)
{
  uint32_t size;
  __IO uint32_t *psramaddress = pAddress;
  uint32_t *pdestbuff = pDstBuffer;
  DAL_SRAM_StateTypeDef state = hsram->State;

  /* Check the SRAM controller state */
  if ((state == DAL_SRAM_STATE_READY) || (state == DAL_SRAM_STATE_PROTECTED))
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Read data from memory */
    for (size = BufferSize; size != 0U; size--)
    {
      *pdestbuff = *psramaddress;
      pdestbuff++;
      psramaddress++;
    }

    /* Update the SRAM controller state */
    hsram->State = state;

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Writes 32-bit buffer to SRAM memory.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  pAddress Pointer to write start address
  * @param  pSrcBuffer Pointer to source buffer to write
  * @param  BufferSize Size of the buffer to write to memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Write_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                     uint32_t BufferSize)
{
  uint32_t size;
  __IO uint32_t *psramaddress = pAddress;
  uint32_t *psrcbuff = pSrcBuffer;

  /* Check the SRAM controller state */
  if (hsram->State == DAL_SRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Write data to memory */
    for (size = BufferSize; size != 0U; size--)
    {
      *psramaddress = *psrcbuff;
      psrcbuff++;
      psramaddress++;
    }

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_READY;

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Reads a Words data from the SRAM memory using DMA transfer.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  pAddress Pointer to read start address
  * @param  pDstBuffer Pointer to destination buffer
  * @param  BufferSize Size of the buffer to read from memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Read_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                    uint32_t BufferSize)
{
  DAL_StatusTypeDef status;
  DAL_SRAM_StateTypeDef state = hsram->State;

  /* Check the SRAM controller state */
  if ((state == DAL_SRAM_STATE_READY) || (state == DAL_SRAM_STATE_PROTECTED))
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Configure DMA user callbacks */
    if (state == DAL_SRAM_STATE_READY)
    {
      hsram->hdma->XferCpltCallback = SRAM_DMACplt;
    }
    else
    {
      hsram->hdma->XferCpltCallback = SRAM_DMACpltProt;
    }
    hsram->hdma->XferErrorCallback = SRAM_DMAError;

    /* Enable the DMA Stream */
    status = DAL_DMA_Start_IT(hsram->hdma, (uint32_t)pAddress, (uint32_t)pDstBuffer, (uint32_t)BufferSize);

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    status = DAL_ERROR;
  }

  return status;
}

/**
  * @brief  Writes a Words data buffer to SRAM memory using DMA transfer.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @param  pAddress Pointer to write start address
  * @param  pSrcBuffer Pointer to source buffer to write
  * @param  BufferSize Size of the buffer to write to memory
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_Write_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                     uint32_t BufferSize)
{
  DAL_StatusTypeDef status;

  /* Check the SRAM controller state */
  if (hsram->State == DAL_SRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Configure DMA user callbacks */
    hsram->hdma->XferCpltCallback = SRAM_DMACplt;
    hsram->hdma->XferErrorCallback = SRAM_DMAError;

    /* Enable the DMA Stream */
    status = DAL_DMA_Start_IT(hsram->hdma, (uint32_t)pSrcBuffer, (uint32_t)pAddress, (uint32_t)BufferSize);

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    status = DAL_ERROR;
  }

  return status;
}

#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User SRAM Callback
  *         To be used instead of the weak (surcharged) predefined callback
  * @param hsram : SRAM handle
  * @param CallbackId : ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_SRAM_MSP_INIT_CB_ID       SRAM MspInit callback ID
  *          @arg @ref DAL_SRAM_MSP_DEINIT_CB_ID     SRAM MspDeInit callback ID
  * @param pCallback : pointer to the Callback function
  * @retval status
  */
DAL_StatusTypeDef DAL_SRAM_RegisterCallback(SRAM_HandleTypeDef *hsram, DAL_SRAM_CallbackIDTypeDef CallbackId,
                                            pSRAM_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;
  DAL_SRAM_StateTypeDef state;

  if (pCallback == NULL)
  {
    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hsram);

  state = hsram->State;
  if ((state == DAL_SRAM_STATE_READY) || (state == DAL_SRAM_STATE_RESET) || (state == DAL_SRAM_STATE_PROTECTED))
  {
    switch (CallbackId)
    {
      case DAL_SRAM_MSP_INIT_CB_ID :
        hsram->MspInitCallback = pCallback;
        break;
      case DAL_SRAM_MSP_DEINIT_CB_ID :
        hsram->MspDeInitCallback = pCallback;
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
  __DAL_UNLOCK(hsram);
  return status;
}

/**
  * @brief  Unregister a User SRAM Callback
  *         SRAM Callback is redirected to the weak (surcharged) predefined callback
  * @param hsram : SRAM handle
  * @param CallbackId : ID of the callback to be unregistered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_SRAM_MSP_INIT_CB_ID       SRAM MspInit callback ID
  *          @arg @ref DAL_SRAM_MSP_DEINIT_CB_ID     SRAM MspDeInit callback ID
  *          @arg @ref DAL_SRAM_DMA_XFER_CPLT_CB_ID  SRAM DMA Xfer Complete callback ID
  *          @arg @ref DAL_SRAM_DMA_XFER_ERR_CB_ID   SRAM DMA Xfer Error callback ID
  * @retval status
  */
DAL_StatusTypeDef DAL_SRAM_UnRegisterCallback(SRAM_HandleTypeDef *hsram, DAL_SRAM_CallbackIDTypeDef CallbackId)
{
  DAL_StatusTypeDef status = DAL_OK;
  DAL_SRAM_StateTypeDef state;

  /* Process locked */
  __DAL_LOCK(hsram);

  state = hsram->State;
  if ((state == DAL_SRAM_STATE_READY) || (state == DAL_SRAM_STATE_PROTECTED))
  {
    switch (CallbackId)
    {
      case DAL_SRAM_MSP_INIT_CB_ID :
        hsram->MspInitCallback = DAL_SRAM_MspInit;
        break;
      case DAL_SRAM_MSP_DEINIT_CB_ID :
        hsram->MspDeInitCallback = DAL_SRAM_MspDeInit;
        break;
      case DAL_SRAM_DMA_XFER_CPLT_CB_ID :
        hsram->DmaXferCpltCallback = DAL_SRAM_DMA_XferCpltCallback;
        break;
      case DAL_SRAM_DMA_XFER_ERR_CB_ID :
        hsram->DmaXferErrorCallback = DAL_SRAM_DMA_XferErrorCallback;
        break;
      default :
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (state == DAL_SRAM_STATE_RESET)
  {
    switch (CallbackId)
    {
      case DAL_SRAM_MSP_INIT_CB_ID :
        hsram->MspInitCallback = DAL_SRAM_MspInit;
        break;
      case DAL_SRAM_MSP_DEINIT_CB_ID :
        hsram->MspDeInitCallback = DAL_SRAM_MspDeInit;
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
  __DAL_UNLOCK(hsram);
  return status;
}

/**
  * @brief  Register a User SRAM Callback for DMA transfers
  *         To be used instead of the weak (surcharged) predefined callback
  * @param hsram : SRAM handle
  * @param CallbackId : ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_SRAM_DMA_XFER_CPLT_CB_ID  SRAM DMA Xfer Complete callback ID
  *          @arg @ref DAL_SRAM_DMA_XFER_ERR_CB_ID   SRAM DMA Xfer Error callback ID
  * @param pCallback : pointer to the Callback function
  * @retval status
  */
DAL_StatusTypeDef DAL_SRAM_RegisterDmaCallback(SRAM_HandleTypeDef *hsram, DAL_SRAM_CallbackIDTypeDef CallbackId,
                                               pSRAM_DmaCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;
  DAL_SRAM_StateTypeDef state;

  if (pCallback == NULL)
  {
    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hsram);

  state = hsram->State;
  if ((state == DAL_SRAM_STATE_READY) || (state == DAL_SRAM_STATE_PROTECTED))
  {
    switch (CallbackId)
    {
      case DAL_SRAM_DMA_XFER_CPLT_CB_ID :
        hsram->DmaXferCpltCallback = pCallback;
        break;
      case DAL_SRAM_DMA_XFER_ERR_CB_ID :
        hsram->DmaXferErrorCallback = pCallback;
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
  __DAL_UNLOCK(hsram);
  return status;
}
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup SRAM_Exported_Functions_Group3 Control functions
  *  @brief   Control functions
  *
@verbatim
  ==============================================================================
                        ##### SRAM Control functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to control dynamically
    the SRAM interface.

@endverbatim
  * @{
  */

/**
  * @brief  Enables dynamically SRAM write operation.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_WriteOperation_Enable(SRAM_HandleTypeDef *hsram)
{
  /* Check the SRAM controller state */
  if (hsram->State == DAL_SRAM_STATE_PROTECTED)
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Enable write operation */
    (void)FMC_NORSRAM_WriteOperation_Enable(hsram->Instance, hsram->Init.NSBank);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_READY;

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @brief  Disables dynamically SRAM write operation.
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_SRAM_WriteOperation_Disable(SRAM_HandleTypeDef *hsram)
{
  /* Check the SRAM controller state */
  if (hsram->State == DAL_SRAM_STATE_READY)
  {
    /* Process Locked */
    __DAL_LOCK(hsram);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_BUSY;

    /* Disable write operation */
    (void)FMC_NORSRAM_WriteOperation_Disable(hsram->Instance, hsram->Init.NSBank);

    /* Update the SRAM controller state */
    hsram->State = DAL_SRAM_STATE_PROTECTED;

    /* Process unlocked */
    __DAL_UNLOCK(hsram);
  }
  else
  {
    return DAL_ERROR;
  }

  return DAL_OK;
}

/**
  * @}
  */

/** @defgroup SRAM_Exported_Functions_Group4 Peripheral State functions
  *  @brief   Peripheral State functions
  *
@verbatim
  ==============================================================================
                      ##### SRAM State functions #####
  ==============================================================================
  [..]
    This subsection permits to get in run-time the status of the SRAM controller
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Returns the SRAM controller state
  * @param  hsram pointer to a SRAM_HandleTypeDef structure that contains
  *                the configuration information for SRAM module.
  * @retval DAL state
  */
DAL_SRAM_StateTypeDef DAL_SRAM_GetState(SRAM_HandleTypeDef *hsram)
{
  return hsram->State;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  DMA SRAM process complete callback.
  * @param  hdma : DMA handle
  * @retval None
  */
static void SRAM_DMACplt(DMA_HandleTypeDef *hdma)
{
  SRAM_HandleTypeDef *hsram = (SRAM_HandleTypeDef *)(hdma->Parent);

  /* Disable the DMA channel */
  __DAL_DMA_DISABLE(hdma);

  /* Update the SRAM controller state */
  hsram->State = DAL_SRAM_STATE_READY;

#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
  hsram->DmaXferCpltCallback(hdma);
#else
  DAL_SRAM_DMA_XferCpltCallback(hdma);
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SRAM process complete callback.
  * @param  hdma : DMA handle
  * @retval None
  */
static void SRAM_DMACpltProt(DMA_HandleTypeDef *hdma)
{
  SRAM_HandleTypeDef *hsram = (SRAM_HandleTypeDef *)(hdma->Parent);

  /* Disable the DMA channel */
  __DAL_DMA_DISABLE(hdma);

  /* Update the SRAM controller state */
  hsram->State = DAL_SRAM_STATE_PROTECTED;

#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
  hsram->DmaXferCpltCallback(hdma);
#else
  DAL_SRAM_DMA_XferCpltCallback(hdma);
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS */
}

/**
  * @brief  DMA SRAM error callback.
  * @param  hdma : DMA handle
  * @retval None
  */
static void SRAM_DMAError(DMA_HandleTypeDef *hdma)
{
  SRAM_HandleTypeDef *hsram = (SRAM_HandleTypeDef *)(hdma->Parent);

  /* Disable the DMA channel */
  __DAL_DMA_DISABLE(hdma);

  /* Update the SRAM controller state */
  hsram->State = DAL_SRAM_STATE_ERROR;

#if (USE_DAL_SRAM_REGISTER_CALLBACKS == 1)
  hsram->DmaXferErrorCallback(hdma);
#else
  DAL_SRAM_DMA_XferErrorCallback(hdma);
#endif /* USE_DAL_SRAM_REGISTER_CALLBACKS */
}

/**
  * @}
  */

#endif /* DAL_SRAM_MODULE_ENABLED */

/**
  * @}
  */

#endif /* SMC_Bank1 */
