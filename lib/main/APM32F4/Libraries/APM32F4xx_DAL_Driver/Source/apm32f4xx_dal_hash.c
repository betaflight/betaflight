/**
  *
  * @file    apm32f4xx_dal_hash.c
  * @brief   HASH DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the HASH peripheral:
  *           + Initialization and de-initialization methods
  *           + HASH or HMAC processing in polling mode
  *           + HASH or HMAC processing in interrupt mode
  *           + HASH or HMAC processing in DMA mode
  *           + Peripheral State methods
  *           + HASH or HMAC processing suspension/resumption
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
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    The HASH DAL driver can be used as follows:

    (#)Initialize the HASH low level resources by implementing the DAL_HASH_MspInit():
        (##) Enable the HASH interface clock using __HASH_CLK_ENABLE()
        (##) When resorting to interrupt-based APIs (e.g. DAL_HASH_xxx_Start_IT())
            (+++) Configure the HASH interrupt priority using DAL_NVIC_SetPriority()
            (+++) Enable the HASH IRQ handler using DAL_NVIC_EnableIRQ()
            (+++) In HASH IRQ handler, call DAL_HASH_IRQHandler() API
        (##) When resorting to DMA-based APIs  (e.g. DAL_HASH_xxx_Start_DMA())
            (+++) Enable the DMAx interface clock using
                   __DMAx_CLK_ENABLE()
            (+++) Configure and enable one DMA stream to manage data transfer from
                memory to peripheral (input stream). Managing data transfer from
                peripheral to memory can be performed only using CPU.
            (+++) Associate the initialized DMA handle to the HASH DMA handle
                using  __DAL_LINKDMA()
            (+++) Configure the priority and enable the NVIC for the transfer complete
                interrupt on the DMA stream: use
                 DAL_NVIC_SetPriority() and
                 DAL_NVIC_EnableIRQ()

    (#)Initialize the HASH DAL using DAL_HASH_Init(). This function:
        (##) resorts to DAL_HASH_MspInit() for low-level initialization,
        (##) configures the data type: 1-bit, 8-bit, 16-bit or 32-bit.

    (#)Three processing schemes are available:
        (##) Polling mode: processing APIs are blocking functions
             i.e. they process the data and wait till the digest computation is finished,
             e.g. DAL_HASH_xxx_Start() for HASH or DAL_HMAC_xxx_Start() for HMAC
        (##) Interrupt mode: processing APIs are not blocking functions
                i.e. they process the data under interrupt,
                e.g. DAL_HASH_xxx_Start_IT() for HASH or DAL_HMAC_xxx_Start_IT() for HMAC
        (##) DMA mode: processing APIs are not blocking functions and the CPU is
             not used for data transfer i.e. the data transfer is ensured by DMA,
                e.g. DAL_HASH_xxx_Start_DMA() for HASH or DAL_HMAC_xxx_Start_DMA()
                for HMAC. Note that in DMA mode, a call to DAL_HASH_xxx_Finish()
                is then required to retrieve the digest.

    (#)When the processing function is called after DAL_HASH_Init(), the HASH peripheral is
       initialized and processes the buffer fed in input. When the input data have all been
       fed to the Peripheral, the digest computation can start.

    (#)Multi-buffer processing is possible in polling, interrupt and DMA modes.
        (##) In polling mode, only multi-buffer HASH processing is possible.
             API DAL_HASH_xxx_Accumulate() must be called for each input buffer, except for the last one.
             User must resort to DAL_HASH_xxx_Accumulate_End() to enter the last one and retrieve as
             well the computed digest.

        (##) In interrupt mode, API DAL_HASH_xxx_Accumulate_IT() must be called for each input buffer,
             except for the last one.
             User must resort to DAL_HASH_xxx_Accumulate_End_IT() to enter the last one and retrieve as
             well the computed digest.

        (##) In DMA mode, multi-buffer HASH and HMAC processing are possible.
              (+++) HASH processing: once initialization is done, MDMAT bit must be set
               through __DAL_HASH_SET_MDMAT() macro.
             From that point, each buffer can be fed to the Peripheral through DAL_HASH_xxx_Start_DMA() API.
             Before entering the last buffer, reset the MDMAT bit with __DAL_HASH_RESET_MDMAT()
             macro then wrap-up the HASH processing in feeding the last input buffer through the
             same API DAL_HASH_xxx_Start_DMA(). The digest can then be retrieved with a call to
             API DAL_HASH_xxx_Finish().
             (+++) HMAC processing (requires to resort to extended functions):
             after initialization, the key and the first input buffer are entered
             in the Peripheral with the API DAL_HMACEx_xxx_Step1_2_DMA(). This carries out HMAC step 1 and
             starts step 2.
             The following buffers are next entered with the API  DAL_HMACEx_xxx_Step2_DMA(). At this
             point, the HMAC processing is still carrying out step 2.
             Then, step 2 for the last input buffer and step 3 are carried out by a single call
             to DAL_HMACEx_xxx_Step2_3_DMA().

             The digest can finally be retrieved with a call to API DAL_HASH_xxx_Finish().


    (#)Context swapping.
        (##) Two APIs are available to suspend HASH or HMAC processing:
             (+++) DAL_HASH_SwFeed_ProcessSuspend() when data are entered by software (polling or IT mode),
             (+++) DAL_HASH_DMAFeed_ProcessSuspend() when data are entered by DMA.

        (##) When HASH or HMAC processing is suspended, DAL_HASH_ContextSaving() allows
            to save in memory the Peripheral context. This context can be restored afterwards
            to resume the HASH processing thanks to DAL_HASH_ContextRestoring().

        (##) Once the HASH Peripheral has been restored to the same configuration as that at suspension
             time, processing can be restarted with the same API call (same API, same handle,
             same parameters) as done before the suspension. Relevant parameters to restart at
             the proper location are internally saved in the HASH handle.

    (#)Call DAL_HASH_DeInit() to deinitialize the HASH peripheral.

     *** Remarks on message length ***
     ===================================
     [..]
      (#) DAL in interruption mode (interruptions driven)

        (##)Due to HASH peripheral hardware design, the peripheral interruption is triggered every 64 bytes.
        This is why, for driver implementation simplicity’s sake, user is requested to enter a message the
        length of which is a multiple of 4 bytes.

        (##) When the message length (in bytes) is not a multiple of words, a specific field exists in HASH_START
        to specify which bits to discard at the end of the complete message to process only the message bits
        and not extra bits.

        (##) If user needs to perform a hash computation of a large input buffer that is spread around various places
        in memory and where each piece of this input buffer is not necessarily a multiple of 4 bytes in size, it becomes
        necessary to use a temporary buffer to format the data accordingly before feeding them to the Peripheral.
        It is advised to the user to
       (+++) achieve the first formatting operation by software then enter the data
       (+++) while the Peripheral is processing the first input set, carry out the second formatting
        operation by software, to be ready when DINIS occurs.
       (+++) repeat step 2 until the whole message is processed.

     [..]
      (#) DAL in DMA mode

        (##) Again, due to hardware design, the DMA transfer to feed the data can only be done on a word-basis.
        The same field described above in HASH_START is used to specify which bits to discard at the end of the
        DMA transfer to process only the message bits and not extra bits. Due to hardware implementation,
        this is possible only at the end of the complete message. When several DMA transfers are needed to
        enter the message, this is not applicable at the end of the intermediary transfers.

        (##) Similarly to the interruption-driven mode, it is suggested to the user to format the consecutive
        chunks of data by software while the DMA transfer and processing is on-going for the first parts of
        the message. Due to the 32-bit alignment required for the DMA transfer, it is underlined that the
        software formatting operation is more complex than in the IT mode.

     *** Callback registration ***
     ===================================
     [..]
      (#) The compilation define  USE_DAL_HASH_REGISTER_CALLBACKS when set to 1
          allows the user to configure dynamically the driver callbacks.
          Use function DAL_HASH_RegisterCallback() to register a user callback.

      (#) Function DAL_HASH_RegisterCallback() allows to register following callbacks:
            (+) InCpltCallback    : callback for input completion.
            (+) DgstCpltCallback  : callback for digest computation completion.
            (+) ErrorCallback     : callback for error.
            (+) MspInitCallback   : HASH MspInit.
            (+) MspDeInitCallback : HASH MspDeInit.
          This function takes as parameters the DAL peripheral handle, the Callback ID
          and a pointer to the user callback function.

      (#) Use function DAL_HASH_UnRegisterCallback() to reset a callback to the default
          weak (surcharged) function.
          DAL_HASH_UnRegisterCallback() takes as parameters the DAL peripheral handle,
          and the Callback ID.
          This function allows to reset following callbacks:
            (+) InCpltCallback    : callback for input completion.
            (+) DgstCpltCallback  : callback for digest computation completion.
            (+) ErrorCallback     : callback for error.
            (+) MspInitCallback   : HASH MspInit.
            (+) MspDeInitCallback : HASH MspDeInit.

      (#) By default, after the DAL_HASH_Init and if the state is DAL_HASH_STATE_RESET
          all callbacks are reset to the corresponding legacy weak (surcharged) functions:
          examples DAL_HASH_InCpltCallback(), DAL_HASH_DgstCpltCallback()
          Exception done for MspInit and MspDeInit callbacks that are respectively
          reset to the legacy weak (surcharged) functions in the DAL_HASH_Init
          and DAL_HASH_DeInit only when these callbacks are null (not registered beforehand)
          If not, MspInit or MspDeInit are not null, the DAL_HASH_Init and DAL_HASH_DeInit
          keep and use the user MspInit/MspDeInit callbacks (registered beforehand).

          Callbacks can be registered/unregistered in READY state only.
          Exception done for MspInit/MspDeInit callbacks that can be registered/unregistered
          in READY or RESET state, thus registered (user) MspInit/DeInit callbacks can be used
          during the Init/DeInit.
          In that case first register the MspInit/MspDeInit user callbacks
          using DAL_HASH_RegisterCallback before calling DAL_HASH_DeInit
          or DAL_HASH_Init function.

          When The compilation define USE_DAL_HASH_REGISTER_CALLBACKS is set to 0 or
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
#if defined (HASH)

/** @defgroup HASH  HASH
  * @brief HASH DAL module driver.
  * @{
  */

#ifdef DAL_HASH_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup HASH_Private_Constants HASH Private Constants
  * @{
  */

/** @defgroup HASH_Digest_Calculation_Status HASH Digest Calculation Status
  * @{
  */
#define HASH_DIGEST_CALCULATION_NOT_STARTED       ((uint32_t)0x00000000U) /*!< DCAL not set after input data written in DIN register */
#define HASH_DIGEST_CALCULATION_STARTED           ((uint32_t)0x00000001U) /*!< DCAL set after input data written in DIN register     */
/**
  * @}
  */

/** @defgroup HASH_Number_Of_CTSWAP_Registers HASH Number of Context Swap Registers
  * @{
  */
#define HASH_NUMBER_OF_CTSWAP_REGISTERS              54U     /*!< Number of Context Swap Registers */
/**
  * @}
  */

/** @defgroup HASH_TimeOut_Value HASH TimeOut Value
  * @{
  */
#define HASH_TIMEOUTVALUE                         1000U   /*!< Time-out value  */
/**
  * @}
  */

/** @defgroup HASH_DMA_Suspension_Words_Limit HASH DMA suspension words limit
  * @{
  */
#define HASH_DMA_SUSPENSION_WORDS_LIMIT             20U   /*!< Number of words below which DMA suspension is aborted */
/**
  * @}
  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup HASH_Private_Functions HASH Private Functions
  * @{
  */
static void HASH_DMAXferCplt(DMA_HandleTypeDef *hdma);
static void HASH_DMAError(DMA_HandleTypeDef *hdma);
static void HASH_GetDigest(uint8_t *pMsgDigest, uint8_t Size);
static DAL_StatusTypeDef HASH_WaitOnFlagUntilTimeout(HASH_HandleTypeDef *hhash, uint32_t Flag, FlagStatus Status,
                                                     uint32_t Timeout);
static DAL_StatusTypeDef HASH_WriteData(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size);
static DAL_StatusTypeDef HASH_IT(HASH_HandleTypeDef *hhash);
static uint32_t HASH_Write_Block_Data(HASH_HandleTypeDef *hhash);
static DAL_StatusTypeDef HMAC_Processing(HASH_HandleTypeDef *hhash, uint32_t Timeout);
/**
  * @}
  */

/** @defgroup HASH_Exported_Functions HASH Exported Functions
  * @{
  */

/** @defgroup HASH_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization, configuration and call-back functions.
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the HASH according to the specified parameters
          in the HASH_InitTypeDef and create the associated handle
      (+) DeInitialize the HASH peripheral
      (+) Initialize the HASH MCU Specific Package (MSP)
      (+) DeInitialize the HASH MSP

    [..]  This section provides as well call back functions definitions for user
          code to manage:
      (+) Input data transfer to Peripheral completion
      (+) Calculated digest retrieval completion
      (+) Error management



@endverbatim
  * @{
  */

/**
  * @brief  Initialize the HASH according to the specified parameters in the
            HASH_HandleTypeDef and create the associated handle.
  * @note   Only MDMAT and DATATYPE bits of HASH Peripheral are set by DAL_HASH_Init(),
  *         other configuration bits are set by HASH or HMAC processing APIs.
  * @note   MDMAT bit is systematically reset by DAL_HASH_Init(). To set it for
  *         multi-buffer HASH processing, user needs to resort to
  *         __DAL_HASH_SET_MDMAT() macro. For HMAC multi-buffer processing, the
  *         relevant APIs manage themselves the MDMAT bit.
  * @param  hhash HASH handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_Init(HASH_HandleTypeDef *hhash)
{
  /* Check the hash handle allocation */
  if (hhash == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_HASH_DATATYPE(hhash->Init.DataType));

#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
  if (hhash->State == DAL_HASH_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hhash->Lock = DAL_UNLOCKED;

    /* Reset Callback pointers in DAL_HASH_STATE_RESET only */
    hhash->InCpltCallback =  DAL_HASH_InCpltCallback;     /* Legacy weak (surcharged) input completion callback */
    hhash->DgstCpltCallback =  DAL_HASH_DgstCpltCallback; /* Legacy weak (surcharged) digest computation
                                                             completion callback */
    hhash->ErrorCallback =  DAL_HASH_ErrorCallback;       /* Legacy weak (surcharged) error callback */
    if (hhash->MspInitCallback == NULL)
    {
      hhash->MspInitCallback = DAL_HASH_MspInit;
    }

    /* Init the low level hardware */
    hhash->MspInitCallback(hhash);
  }
#else
  if (hhash->State == DAL_HASH_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hhash->Lock = DAL_UNLOCKED;

    /* Init the low level hardware */
    DAL_HASH_MspInit(hhash);
  }
#endif /* (USE_DAL_HASH_REGISTER_CALLBACKS) */

  /* Change the HASH state */
  hhash->State = DAL_HASH_STATE_BUSY;

  /* Reset HashInCount, HashITCounter, HashBuffSize and NbWordsAlreadyPushed */
  hhash->HashInCount = 0;
  hhash->HashBuffSize = 0;
  hhash->HashITCounter = 0;
  hhash->NbWordsAlreadyPushed = 0;
  /* Reset digest calculation bridle (MDMAT bit control) */
  hhash->DigestCalculationDisable = RESET;
  /* Set phase to READY */
  hhash->Phase = DAL_HASH_PHASE_READY;
  /* Reset suspension request flag */
  hhash->SuspendRequest = DAL_HASH_SUSPEND_NONE;

  /* Set the data type bit */
  MODIFY_REG(HASH->CTRL, HASH_CTRL_DTYPE, hhash->Init.DataType);
#if defined(HASH_CTRL_MDMAT)
  /* Reset MDMAT bit */
  __DAL_HASH_RESET_MDMAT();
#endif /* HASH_CTRL_MDMAT */
  /* Reset HASH handle status */
  hhash->Status = DAL_OK;

  /* Set the HASH state to Ready */
  hhash->State = DAL_HASH_STATE_READY;

  /* Initialise the error code */
  hhash->ErrorCode = DAL_HASH_ERROR_NONE;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  DeInitialize the HASH peripheral.
  * @param  hhash HASH handle.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_DeInit(HASH_HandleTypeDef *hhash)
{
  /* Check the HASH handle allocation */
  if (hhash == NULL)
  {
    return DAL_ERROR;
  }

  /* Change the HASH state */
  hhash->State = DAL_HASH_STATE_BUSY;

  /* Set the default HASH phase */
  hhash->Phase = DAL_HASH_PHASE_READY;

  /* Reset HashInCount, HashITCounter and HashBuffSize */
  hhash->HashInCount = 0;
  hhash->HashBuffSize = 0;
  hhash->HashITCounter = 0;
  /* Reset digest calculation bridle (MDMAT bit control) */
  hhash->DigestCalculationDisable = RESET;

#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
  if (hhash->MspDeInitCallback == NULL)
  {
    hhash->MspDeInitCallback = DAL_HASH_MspDeInit;
  }

  /* DeInit the low level hardware */
  hhash->MspDeInitCallback(hhash);
#else
  /* DeInit the low level hardware: CLOCK, NVIC */
  DAL_HASH_MspDeInit(hhash);
#endif /* (USE_DAL_HASH_REGISTER_CALLBACKS) */


  /* Reset HASH handle status */
  hhash->Status = DAL_OK;

  /* Set the HASH state to Ready */
  hhash->State = DAL_HASH_STATE_RESET;

  /* Initialise the error code */
  hhash->ErrorCode = DAL_HASH_ERROR_NONE;

  /* Reset multi buffers accumulation flag */
  hhash->Accumulation = 0U;

  /* Return function status */
  return DAL_OK;
}

/**
  * @brief  Initialize the HASH MSP.
  * @param  hhash HASH handle.
  * @retval None
  */
__weak void DAL_HASH_MspInit(HASH_HandleTypeDef *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhash);

  /* NOTE : This function should not be modified; when the callback is needed,
            DAL_HASH_MspInit() can be implemented in the user file.
   */
}

/**
  * @brief  DeInitialize the HASH MSP.
  * @param  hhash HASH handle.
  * @retval None
  */
__weak void DAL_HASH_MspDeInit(HASH_HandleTypeDef *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhash);

  /* NOTE : This function should not be modified; when the callback is needed,
            DAL_HASH_MspDeInit() can be implemented in the user file.
   */
}

/**
  * @brief  Input data transfer complete call back.
  * @note   DAL_HASH_InCpltCallback() is called when the complete input message
  *         has been fed to the Peripheral. This API is invoked only when input data are
  *         entered under interruption or through DMA.
  * @note   In case of HASH or HMAC multi-buffer DMA feeding case (MDMAT bit set),
  *         DAL_HASH_InCpltCallback() is called at the end of each buffer feeding
  *         to the Peripheral.
  * @param  hhash HASH handle.
  * @retval None
  */
__weak void DAL_HASH_InCpltCallback(HASH_HandleTypeDef *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhash);

  /* NOTE : This function should not be modified; when the callback is needed,
            DAL_HASH_InCpltCallback() can be implemented in the user file.
   */
}

/**
  * @brief  Digest computation complete call back.
  * @note   DAL_HASH_DgstCpltCallback() is used under interruption, is not
  *         relevant with DMA.
  * @param  hhash HASH handle.
  * @retval None
  */
__weak void DAL_HASH_DgstCpltCallback(HASH_HandleTypeDef *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhash);

  /* NOTE : This function should not be modified; when the callback is needed,
            DAL_HASH_DgstCpltCallback() can be implemented in the user file.
   */
}

/**
  * @brief  Error callback.
  * @note   Code user can resort to hhash->Status (DAL_ERROR, DAL_TIMEOUT,...)
  *         to retrieve the error type.
  * @param  hhash HASH handle.
  * @retval None
  */
__weak void DAL_HASH_ErrorCallback(HASH_HandleTypeDef *hhash)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhash);

  /* NOTE : This function should not be modified; when the callback is needed,
            DAL_HASH_ErrorCallback() can be implemented in the user file.
   */
}

#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User HASH Callback
  *         To be used instead of the weak (surcharged) predefined callback
  * @param hhash HASH handle
  * @param CallbackID ID of the callback to be registered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_HASH_INPUTCPLT_CB_ID HASH input completion Callback ID
  *          @arg @ref DAL_HASH_DGSTCPLT_CB_ID HASH digest computation completion Callback ID
  *          @arg @ref DAL_HASH_ERROR_CB_ID HASH error Callback ID
  *          @arg @ref DAL_HASH_MSPINIT_CB_ID HASH MspInit callback ID
  *          @arg @ref DAL_HASH_MSPDEINIT_CB_ID HASH MspDeInit callback ID
  * @param pCallback pointer to the Callback function
  * @retval status
  */
DAL_StatusTypeDef DAL_HASH_RegisterCallback(HASH_HandleTypeDef *hhash, DAL_HASH_CallbackIDTypeDef CallbackID,
                                            pHASH_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hhash->ErrorCode |= DAL_HASH_ERROR_INVALID_CALLBACK;
    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hhash);

  if (DAL_HASH_STATE_READY == hhash->State)
  {
    switch (CallbackID)
    {
      case DAL_HASH_INPUTCPLT_CB_ID :
        hhash->InCpltCallback = pCallback;
        break;

      case DAL_HASH_DGSTCPLT_CB_ID :
        hhash->DgstCpltCallback = pCallback;
        break;

      case DAL_HASH_ERROR_CB_ID :
        hhash->ErrorCallback = pCallback;
        break;

      case DAL_HASH_MSPINIT_CB_ID :
        hhash->MspInitCallback = pCallback;
        break;

      case DAL_HASH_MSPDEINIT_CB_ID :
        hhash->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hhash->ErrorCode |= DAL_HASH_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_HASH_STATE_RESET == hhash->State)
  {
    switch (CallbackID)
    {
      case DAL_HASH_MSPINIT_CB_ID :
        hhash->MspInitCallback = pCallback;
        break;

      case DAL_HASH_MSPDEINIT_CB_ID :
        hhash->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hhash->ErrorCode |= DAL_HASH_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hhash->ErrorCode |= DAL_HASH_ERROR_INVALID_CALLBACK;
    /* update return status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hhash);
  return status;
}

/**
  * @brief  Unregister a HASH Callback
  *         HASH Callback is redirected to the weak (surcharged) predefined callback
  * @param hhash HASH handle
  * @param CallbackID ID of the callback to be unregistered
  *        This parameter can be one of the following values:
  *          @arg @ref DAL_HASH_INPUTCPLT_CB_ID HASH input completion Callback ID
  *          @arg @ref DAL_HASH_DGSTCPLT_CB_ID HASH digest computation completion Callback ID
  *          @arg @ref DAL_HASH_ERROR_CB_ID HASH error Callback ID
  *          @arg @ref DAL_HASH_MSPINIT_CB_ID HASH MspInit callback ID
  *          @arg @ref DAL_HASH_MSPDEINIT_CB_ID HASH MspDeInit callback ID
  * @retval status
  */
DAL_StatusTypeDef DAL_HASH_UnRegisterCallback(HASH_HandleTypeDef *hhash, DAL_HASH_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hhash);

  if (DAL_HASH_STATE_READY == hhash->State)
  {
    switch (CallbackID)
    {
      case DAL_HASH_INPUTCPLT_CB_ID :
        hhash->InCpltCallback = DAL_HASH_InCpltCallback;     /* Legacy weak (surcharged) input completion callback */
        break;

      case DAL_HASH_DGSTCPLT_CB_ID :
        hhash->DgstCpltCallback = DAL_HASH_DgstCpltCallback; /* Legacy weak (surcharged) digest computation
                                                                completion callback */
        break;

      case DAL_HASH_ERROR_CB_ID :
        hhash->ErrorCallback = DAL_HASH_ErrorCallback;       /* Legacy weak (surcharged) error callback */
        break;

      case DAL_HASH_MSPINIT_CB_ID :
        hhash->MspInitCallback = DAL_HASH_MspInit;           /* Legacy weak (surcharged) Msp Init */
        break;

      case DAL_HASH_MSPDEINIT_CB_ID :
        hhash->MspDeInitCallback = DAL_HASH_MspDeInit;       /* Legacy weak (surcharged) Msp DeInit */
        break;

      default :
        /* Update the error code */
        hhash->ErrorCode |= DAL_HASH_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (DAL_HASH_STATE_RESET == hhash->State)
  {
    switch (CallbackID)
    {
      case DAL_HASH_MSPINIT_CB_ID :
        hhash->MspInitCallback = DAL_HASH_MspInit;           /* Legacy weak (surcharged) Msp Init */
        break;

      case DAL_HASH_MSPDEINIT_CB_ID :
        hhash->MspDeInitCallback = DAL_HASH_MspDeInit;       /* Legacy weak (surcharged) Msp DeInit */
        break;

      default :
        /* Update the error code */
        hhash->ErrorCode |= DAL_HASH_ERROR_INVALID_CALLBACK;
        /* update return status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hhash->ErrorCode |= DAL_HASH_ERROR_INVALID_CALLBACK;
    /* update return status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hhash);
  return status;
}
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group2 HASH processing functions in polling mode
  *  @brief   HASH processing functions using polling mode.
  *
@verbatim
 ===============================================================================
                 ##### Polling mode HASH processing functions #####
 ===============================================================================
    [..]  This section provides functions allowing to calculate in polling mode
          the hash value using one of the following algorithms:
      (+) MD5
         (++) DAL_HASH_MD5_Start()
         (++) DAL_HASH_MD5_Accmlt()
         (++) DAL_HASH_MD5_Accmlt_End()
      (+) SHA1
         (++) DAL_HASH_SHA1_Start()
         (++) DAL_HASH_SHA1_Accmlt()
         (++) DAL_HASH_SHA1_Accmlt_End()

    [..] For a single buffer to be hashed, user can resort to DAL_HASH_xxx_Start().

    [..]  In case of multi-buffer HASH processing (a single digest is computed while
          several buffers are fed to the Peripheral), the user can resort to successive calls
          to DAL_HASH_xxx_Accumulate() and wrap-up the digest computation by a call
          to DAL_HASH_xxx_Accumulate_End().

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the HASH peripheral in MD5 mode, next process pInBuffer then
  *         read the computed digest.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 16 bytes.
  * @param  Timeout Timeout value
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_MD5_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t *pOutBuffer,
                                     uint32_t Timeout)
{
  return HASH_Start(hhash, pInBuffer, Size, pOutBuffer, Timeout, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  If not already done, initialize the HASH peripheral in MD5 mode then
  *         processes pInBuffer.
  * @note   Consecutive calls to DAL_HASH_MD5_Accmlt() can be used to feed
  *         several input buffers back-to-back to the Peripheral that will yield a single
  *         HASH signature once all buffers have been entered. Wrap-up of input
  *         buffers feeding and retrieval of digest is done by a call to
  *         DAL_HASH_MD5_Accmlt_End().
  * @note   Field hhash->Phase of HASH handle is tested to check whether or not
  *         the Peripheral has already been initialized.
  * @note   Digest is not retrieved by this API, user must resort to DAL_HASH_MD5_Accmlt_End()
  *         to read it, feeding at the same time the last input buffer to the Peripheral.
  * @note   The input buffer size (in bytes) must be a multiple of 4 otherwise, the
  *         HASH digest computation is corrupted. Only DAL_HASH_MD5_Accmlt_End() is able
  *         to manage the ending buffer with a length in bytes not a multiple of 4.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes, must be a multiple of 4.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_MD5_Accmlt(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  return  HASH_Accumulate(hhash, pInBuffer, Size, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  End computation of a single HASH signature after several calls to DAL_HASH_MD5_Accmlt() API.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 16 bytes.
  * @param  Timeout Timeout value
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_MD5_Accmlt_End(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size,
                                          uint8_t *pOutBuffer, uint32_t Timeout)
{
  return HASH_Start(hhash, pInBuffer, Size, pOutBuffer, Timeout, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  Initialize the HASH peripheral in SHA1 mode, next process pInBuffer then
  *         read the computed digest.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 20 bytes.
  * @param  Timeout Timeout value
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_SHA1_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t *pOutBuffer,
                                      uint32_t Timeout)
{
  return HASH_Start(hhash, pInBuffer, Size, pOutBuffer, Timeout, HASH_ALGOSELECTION_SHA1);
}

/**
  * @brief  If not already done, initialize the HASH peripheral in SHA1 mode then
  *         processes pInBuffer.
  * @note   Consecutive calls to DAL_HASH_SHA1_Accmlt() can be used to feed
  *         several input buffers back-to-back to the Peripheral that will yield a single
  *         HASH signature once all buffers have been entered. Wrap-up of input
  *         buffers feeding and retrieval of digest is done by a call to
  *         DAL_HASH_SHA1_Accmlt_End().
  * @note   Field hhash->Phase of HASH handle is tested to check whether or not
  *         the Peripheral has already been initialized.
  * @note   Digest is not retrieved by this API, user must resort to DAL_HASH_SHA1_Accmlt_End()
  *         to read it, feeding at the same time the last input buffer to the Peripheral.
  * @note   The input buffer size (in bytes) must be a multiple of 4 otherwise, the
  *         HASH digest computation is corrupted. Only DAL_HASH_SHA1_Accmlt_End() is able
  *         to manage the ending buffer with a length in bytes not a multiple of 4.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes, must be a multiple of 4.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_SHA1_Accmlt(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  return  HASH_Accumulate(hhash, pInBuffer, Size, HASH_ALGOSELECTION_SHA1);
}

/**
  * @brief  End computation of a single HASH signature after several calls to DAL_HASH_SHA1_Accmlt() API.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 20 bytes.
  * @param  Timeout Timeout value
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_SHA1_Accmlt_End(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size,
                                           uint8_t *pOutBuffer, uint32_t Timeout)
{
  return HASH_Start(hhash, pInBuffer, Size, pOutBuffer, Timeout, HASH_ALGOSELECTION_SHA1);
}

/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group3 HASH processing functions in interrupt mode
  *  @brief   HASH processing functions using interrupt mode.
  *
@verbatim
 ===============================================================================
                 ##### Interruption mode HASH processing functions #####
 ===============================================================================
    [..]  This section provides functions allowing to calculate in interrupt mode
          the hash value using one of the following algorithms:
      (+) MD5
         (++) DAL_HASH_MD5_Start_IT()
         (++) DAL_HASH_MD5_Accmlt_IT()
         (++) DAL_HASH_MD5_Accmlt_End_IT()
      (+) SHA1
         (++) DAL_HASH_SHA1_Start_IT()
         (++) DAL_HASH_SHA1_Accmlt_IT()
         (++) DAL_HASH_SHA1_Accmlt_End_IT()

    [..]  API DAL_HASH_IRQHandler() manages each HASH interruption.

    [..] Note that DAL_HASH_IRQHandler() manages as well HASH Peripheral interruptions when in
         HMAC processing mode.


@endverbatim
  * @{
  */

/**
  * @brief  Initialize the HASH peripheral in MD5 mode, next process pInBuffer then
  *         read the computed digest in interruption mode.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 16 bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_MD5_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size,
                                        uint8_t *pOutBuffer)
{
  return HASH_Start_IT(hhash, pInBuffer, Size, pOutBuffer, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  If not already done, initialize the HASH peripheral in MD5 mode then
  *         processes pInBuffer in interruption mode.
  * @note   Consecutive calls to DAL_HASH_MD5_Accmlt_IT() can be used to feed
  *         several input buffers back-to-back to the Peripheral that will yield a single
  *         HASH signature once all buffers have been entered. Wrap-up of input
  *         buffers feeding and retrieval of digest is done by a call to
  *         DAL_HASH_MD5_Accmlt_End_IT().
  * @note   Field hhash->Phase of HASH handle is tested to check whether or not
  *         the Peripheral has already been initialized.
  * @note   The input buffer size (in bytes) must be a multiple of 4 otherwise, the
  *         HASH digest computation is corrupted. Only DAL_HASH_MD5_Accmlt_End_IT() is able
  *         to manage the ending buffer with a length in bytes not a multiple of 4.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes, must be a multiple of 4.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_MD5_Accmlt_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  return  HASH_Accumulate_IT(hhash, pInBuffer, Size, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  End computation of a single HASH signature after several calls to DAL_HASH_MD5_Accmlt_IT() API.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 16 bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_MD5_Accmlt_End_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size,
                                             uint8_t *pOutBuffer)
{
  return HASH_Start_IT(hhash, pInBuffer, Size, pOutBuffer, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  Initialize the HASH peripheral in SHA1 mode, next process pInBuffer then
  *         read the computed digest in interruption mode.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 20 bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_SHA1_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size,
                                         uint8_t *pOutBuffer)
{
  return HASH_Start_IT(hhash, pInBuffer, Size, pOutBuffer, HASH_ALGOSELECTION_SHA1);
}


/**
  * @brief  If not already done, initialize the HASH peripheral in SHA1 mode then
  *         processes pInBuffer in interruption mode.
  * @note   Consecutive calls to DAL_HASH_SHA1_Accmlt_IT() can be used to feed
  *         several input buffers back-to-back to the Peripheral that will yield a single
  *         HASH signature once all buffers have been entered. Wrap-up of input
  *         buffers feeding and retrieval of digest is done by a call to
  *         DAL_HASH_SHA1_Accmlt_End_IT().
  * @note   Field hhash->Phase of HASH handle is tested to check whether or not
  *         the Peripheral has already been initialized.
  * @note   The input buffer size (in bytes) must be a multiple of 4 otherwise, the
  *         HASH digest computation is corrupted. Only DAL_HASH_SHA1_Accmlt_End_IT() is able
  *         to manage the ending buffer with a length in bytes not a multiple of 4.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes, must be a multiple of 4.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_SHA1_Accmlt_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  return  HASH_Accumulate_IT(hhash, pInBuffer, Size, HASH_ALGOSELECTION_SHA1);
}

/**
  * @brief  End computation of a single HASH signature after several calls to DAL_HASH_SHA1_Accmlt_IT() API.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 20 bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_SHA1_Accmlt_End_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size,
                                              uint8_t *pOutBuffer)
{
  return HASH_Start_IT(hhash, pInBuffer, Size, pOutBuffer, HASH_ALGOSELECTION_SHA1);
}

/**
  * @brief Handle HASH interrupt request.
  * @param hhash HASH handle.
  * @note  DAL_HASH_IRQHandler() handles interrupts in HMAC processing as well.
  * @note  In case of error reported during the HASH interruption processing,
  *        DAL_HASH_ErrorCallback() API is called so that user code can
  *        manage the error. The error type is available in hhash->Status field.
  * @retval None
  */
void DAL_HASH_IRQHandler(HASH_HandleTypeDef *hhash)
{
  hhash->Status = HASH_IT(hhash);
  if (hhash->Status != DAL_OK)
  {
    hhash->ErrorCode |= DAL_HASH_ERROR_IT;
#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
    hhash->ErrorCallback(hhash);
#else
    DAL_HASH_ErrorCallback(hhash);
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */
    /* After error handling by code user, reset HASH handle DAL status */
    hhash->Status = DAL_OK;
  }
}

/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group4 HASH processing functions in DMA mode
  *  @brief   HASH processing functions using DMA mode.
  *
@verbatim
 ===============================================================================
                    ##### DMA mode HASH processing functions #####
 ===============================================================================
    [..]  This section provides functions allowing to calculate in DMA mode
          the hash value using one of the following algorithms:
      (+) MD5
         (++) DAL_HASH_MD5_Start_DMA()
         (++) DAL_HASH_MD5_Finish()
      (+) SHA1
         (++) DAL_HASH_SHA1_Start_DMA()
         (++) DAL_HASH_SHA1_Finish()

    [..]  When resorting to DMA mode to enter the data in the Peripheral, user must resort
          to  DAL_HASH_xxx_Start_DMA() then read the resulting digest with
          DAL_HASH_xxx_Finish().
    [..]  In case of multi-buffer HASH processing, MDMAT bit must first be set before
          the successive calls to DAL_HASH_xxx_Start_DMA(). Then, MDMAT bit needs to be
          reset before the last call to DAL_HASH_xxx_Start_DMA(). Digest is finally
          retrieved thanks to DAL_HASH_xxx_Finish().

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the HASH peripheral in MD5 mode then initiate a DMA transfer
  *         to feed the input buffer to the Peripheral.
  * @note   Once the DMA transfer is finished, DAL_HASH_MD5_Finish() API must
  *         be called to retrieve the computed digest.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_MD5_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  return HASH_Start_DMA(hhash, pInBuffer, Size, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  Return the computed digest in MD5 mode.
  * @note   The API waits for DCIS to be set then reads the computed digest.
  * @note   DAL_HASH_MD5_Finish() can be used as well to retrieve the digest in
  *         HMAC MD5 mode.
  * @param  hhash HASH handle.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 16 bytes.
  * @param  Timeout Timeout value.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_MD5_Finish(HASH_HandleTypeDef *hhash, uint8_t *pOutBuffer, uint32_t Timeout)
{
  return HASH_Finish(hhash, pOutBuffer, Timeout);
}

/**
  * @brief  Initialize the HASH peripheral in SHA1 mode then initiate a DMA transfer
  *         to feed the input buffer to the Peripheral.
  * @note   Once the DMA transfer is finished, DAL_HASH_SHA1_Finish() API must
  *         be called to retrieve the computed digest.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_SHA1_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  return HASH_Start_DMA(hhash, pInBuffer, Size, HASH_ALGOSELECTION_SHA1);
}


/**
  * @brief  Return the computed digest in SHA1 mode.
  * @note   The API waits for DCIS to be set then reads the computed digest.
  * @note   DAL_HASH_SHA1_Finish() can be used as well to retrieve the digest in
  *         HMAC SHA1 mode.
  * @param  hhash HASH handle.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 20 bytes.
  * @param  Timeout Timeout value.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_SHA1_Finish(HASH_HandleTypeDef *hhash, uint8_t *pOutBuffer, uint32_t Timeout)
{
  return HASH_Finish(hhash, pOutBuffer, Timeout);
}

/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group5 HMAC processing functions in polling mode
  *  @brief   HMAC processing functions using polling mode.
  *
@verbatim
 ===============================================================================
                 ##### Polling mode HMAC processing functions #####
 ===============================================================================
    [..]  This section provides functions allowing to calculate in polling mode
          the HMAC value using one of the following algorithms:
      (+) MD5
         (++) DAL_HMAC_MD5_Start()
      (+) SHA1
         (++) DAL_HMAC_SHA1_Start()


@endverbatim
  * @{
  */

/**
  * @brief  Initialize the HASH peripheral in HMAC MD5 mode, next process pInBuffer then
  *         read the computed digest.
  * @note   Digest is available in pOutBuffer.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 16 bytes.
  * @param  Timeout Timeout value.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HMAC_MD5_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t *pOutBuffer,
                                     uint32_t Timeout)
{
  return HMAC_Start(hhash, pInBuffer, Size, pOutBuffer, Timeout, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  Initialize the HASH peripheral in HMAC SHA1 mode, next process pInBuffer then
  *         read the computed digest.
  * @note   Digest is available in pOutBuffer.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 20 bytes.
  * @param  Timeout Timeout value.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HMAC_SHA1_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t *pOutBuffer,
                                      uint32_t Timeout)
{
  return HMAC_Start(hhash, pInBuffer, Size, pOutBuffer, Timeout, HASH_ALGOSELECTION_SHA1);
}

/**
  * @}
  */


/** @defgroup HASH_Exported_Functions_Group6 HMAC processing functions in interrupt mode
  *  @brief   HMAC processing functions using interrupt mode.
  *
@verbatim
 ===============================================================================
                 ##### Interrupt mode HMAC processing functions #####
 ===============================================================================
    [..]  This section provides functions allowing to calculate in interrupt mode
          the HMAC value using one of the following algorithms:
      (+) MD5
         (++) DAL_HMAC_MD5_Start_IT()
      (+) SHA1
         (++) DAL_HMAC_SHA1_Start_IT()

@endverbatim
  * @{
  */


/**
  * @brief  Initialize the HASH peripheral in HMAC MD5 mode, next process pInBuffer then
  *         read the computed digest in interrupt mode.
  * @note   Digest is available in pOutBuffer.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 16 bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HMAC_MD5_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size,
                                        uint8_t *pOutBuffer)
{
  return  HMAC_Start_IT(hhash, pInBuffer, Size, pOutBuffer, HASH_ALGOSELECTION_MD5);
}

/**
  * @brief  Initialize the HASH peripheral in HMAC SHA1 mode, next process pInBuffer then
  *         read the computed digest in interrupt mode.
  * @note   Digest is available in pOutBuffer.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest. Digest size is 20 bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HMAC_SHA1_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size,
                                         uint8_t *pOutBuffer)
{
  return  HMAC_Start_IT(hhash, pInBuffer, Size, pOutBuffer, HASH_ALGOSELECTION_SHA1);
}

/**
  * @}
  */



/** @defgroup HASH_Exported_Functions_Group7 HMAC processing functions in DMA mode
  *  @brief   HMAC processing functions using DMA modes.
  *
@verbatim
 ===============================================================================
                 ##### DMA mode HMAC processing functions #####
 ===============================================================================
    [..]  This section provides functions allowing to calculate in DMA mode
          the HMAC value using one of the following algorithms:
      (+) MD5
         (++) DAL_HMAC_MD5_Start_DMA()
      (+) SHA1
         (++) DAL_HMAC_SHA1_Start_DMA()

    [..]  When resorting to DMA mode to enter the data in the Peripheral for HMAC processing,
          user must resort to  DAL_HMAC_xxx_Start_DMA() then read the resulting digest
          with DAL_HASH_xxx_Finish().

@endverbatim
  * @{
  */


/**
  * @brief  Initialize the HASH peripheral in HMAC MD5 mode then initiate the required
  *         DMA transfers to feed the key and the input buffer to the Peripheral.
  * @note   Once the DMA transfers are finished (indicated by hhash->State set back
  *         to DAL_HASH_STATE_READY), DAL_HASH_MD5_Finish() API must be called to retrieve
  *         the computed digest.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @note   If MDMAT bit is set before calling this function (multi-buffer
  *          HASH processing case), the input buffer size (in bytes) must be
  *          a multiple of 4 otherwise, the HASH digest computation is corrupted.
  *          For the processing of the last buffer of the thread, MDMAT bit must
  *          be reset and the buffer length (in bytes) doesn't have to be a
  *          multiple of 4.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HMAC_MD5_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  return  HMAC_Start_DMA(hhash, pInBuffer, Size, HASH_ALGOSELECTION_MD5);
}


/**
  * @brief  Initialize the HASH peripheral in HMAC SHA1 mode then initiate the required
  *         DMA transfers to feed the key and the input buffer to the Peripheral.
  * @note   Once the DMA transfers are finished (indicated by hhash->State set back
  *         to DAL_HASH_STATE_READY), DAL_HASH_SHA1_Finish() API must be called to retrieve
  *         the computed digest.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @note   If MDMAT bit is set before calling this function (multi-buffer
  *          HASH processing case), the input buffer size (in bytes) must be
  *          a multiple of 4 otherwise, the HASH digest computation is corrupted.
  *          For the processing of the last buffer of the thread, MDMAT bit must
  *          be reset and the buffer length (in bytes) doesn't have to be a
  *          multiple of 4.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HMAC_SHA1_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  return  HMAC_Start_DMA(hhash, pInBuffer, Size, HASH_ALGOSELECTION_SHA1);
}

/**
  * @}
  */

/** @defgroup HASH_Exported_Functions_Group8 Peripheral states functions
  *  @brief   Peripheral State functions.
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State methods #####
 ===============================================================================
    [..]
    This section permits to get in run-time the state and the peripheral handle
    status of the peripheral:
      (+) DAL_HASH_GetState()
      (+) DAL_HASH_GetStatus()

    [..]
    Additionally, this subsection provides functions allowing to save and restore
    the HASH or HMAC processing context in case of calculation suspension:
      (+) DAL_HASH_ContextSaving()
      (+) DAL_HASH_ContextRestoring()

    [..]
    This subsection provides functions allowing to suspend the HASH processing
      (+) when input are fed to the Peripheral by software
          (++) DAL_HASH_SwFeed_ProcessSuspend()
      (+) when input are fed to the Peripheral by DMA
          (++) DAL_HASH_DMAFeed_ProcessSuspend()



@endverbatim
  * @{
  */

/**
  * @brief  Return the HASH handle state.
  * @note   The API yields the current state of the handle (BUSY, READY,...).
  * @param  hhash HASH handle.
  * @retval DAL HASH state
  */
DAL_HASH_StateTypeDef DAL_HASH_GetState(HASH_HandleTypeDef *hhash)
{
  return hhash->State;
}


/**
  * @brief Return the HASH DAL status.
  * @note  The API yields the DAL status of the handle: it is the result of the
  *        latest HASH processing and allows to report any issue (e.g. DAL_TIMEOUT).
  * @param  hhash HASH handle.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_GetStatus(HASH_HandleTypeDef *hhash)
{
  return hhash->Status;
}

/**
  * @brief  Save the HASH context in case of processing suspension.
  * @param  hhash HASH handle.
  * @param  pMemBuffer pointer to the memory buffer where the HASH context
  *         is saved.
  * @note   The INT, START, CTRL then all the CTSWAP registers are saved
  *         in that order. Only the r/w bits are read to be restored later on.
  * @note   By default, all the context swap registers (there are
  *         HASH_NUMBER_OF_CTSWAP_REGISTERS of those) are saved.
  * @note   pMemBuffer points to a buffer allocated by the user. The buffer size
  *         must be at least (HASH_NUMBER_OF_CTSWAP_REGISTERS + 3) * 4 uint8 long.
  * @retval None
  */
void DAL_HASH_ContextSaving(HASH_HandleTypeDef *hhash, uint8_t *pMemBuffer)
{
  uint32_t mem_ptr = (uint32_t)pMemBuffer;
  uint32_t csr_ptr = (uint32_t)HASH->CTSWAP;
  uint32_t i;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhash);

  /* Save INT register content */
  *(uint32_t *)(mem_ptr) = READ_BIT(HASH->INT, HASH_IT_DINI | HASH_IT_DCI);
  mem_ptr += 4U;
  /* Save START register content */
  *(uint32_t *)(mem_ptr) = READ_BIT(HASH->START, HASH_START_LWNUM);
  mem_ptr += 4U;
  /* Save CTRL register content */
#if defined(HASH_CTRL_MDMAT)
  *(uint32_t *)(mem_ptr) = READ_BIT(HASH->CTRL, HASH_CTRL_DMAEN | HASH_CTRL_DTYPE | HASH_CTRL_MODESEL | HASH_CTRL_ALGSEL |
                                    HASH_CTRL_LKEYSEL | HASH_CTRL_MDMAT);
#else
  *(uint32_t *)(mem_ptr) = READ_BIT(HASH->CTRL, HASH_CTRL_DMAEN | HASH_CTRL_DTYPE | HASH_CTRL_MODESEL | HASH_CTRL_ALGSEL |
                                    HASH_CTRL_LKEYSEL);
#endif /* HASH_CTRL_MDMAT*/
  mem_ptr += 4U;
  /* By default, save all CTSWAPs registers */
  for (i = HASH_NUMBER_OF_CTSWAP_REGISTERS; i > 0U; i--)
  {
    *(uint32_t *)(mem_ptr) = *(uint32_t *)(csr_ptr);
    mem_ptr += 4U;
    csr_ptr += 4U;
  }
}


/**
  * @brief  Restore the HASH context in case of processing resumption.
  * @param  hhash HASH handle.
  * @param  pMemBuffer pointer to the memory buffer where the HASH context
  *         is stored.
  * @note   The INT, START, CTRL then all the CTSWAP registers are restored
  *         in that order. Only the r/w bits are restored.
  * @note   By default, all the context swap registers (HASH_NUMBER_OF_CTSWAP_REGISTERS
  *         of those) are restored (all of them have been saved by default
  *         beforehand).
  * @retval None
  */
void DAL_HASH_ContextRestoring(HASH_HandleTypeDef *hhash, uint8_t *pMemBuffer)
{
  uint32_t mem_ptr = (uint32_t)pMemBuffer;
  uint32_t csr_ptr = (uint32_t)HASH->CTSWAP;
  uint32_t i;

  /* Prevent unused argument(s) compilation warning */
  UNUSED(hhash);

  /* Restore INT register content */
  WRITE_REG(HASH->INT, (*(uint32_t *)(mem_ptr)));
  mem_ptr += 4U;
  /* Restore START register content */
  WRITE_REG(HASH->START, (*(uint32_t *)(mem_ptr)));
  mem_ptr += 4U;
  /* Restore CTRL register content */
  WRITE_REG(HASH->CTRL, (*(uint32_t *)(mem_ptr)));
  mem_ptr += 4U;

  /* Reset the HASH processor before restoring the Context
  Swap Registers (CTSWAP) */
  __DAL_HASH_INIT();

  /* By default, restore all CTSWAP registers */
  for (i = HASH_NUMBER_OF_CTSWAP_REGISTERS; i > 0U; i--)
  {
    WRITE_REG((*(uint32_t *)(csr_ptr)), (*(uint32_t *)(mem_ptr)));
    mem_ptr += 4U;
    csr_ptr += 4U;
  }
}


/**
  * @brief  Initiate HASH processing suspension when in polling or interruption mode.
  * @param  hhash HASH handle.
  * @note   Set the handle field SuspendRequest to the appropriate value so that
  *         the on-going HASH processing is suspended as soon as the required
  *         conditions are met. Note that the actual suspension is carried out
  *         by the functions HASH_WriteData() in polling mode and HASH_IT() in
  *         interruption mode.
  * @retval None
  */
void DAL_HASH_SwFeed_ProcessSuspend(HASH_HandleTypeDef *hhash)
{
  /* Set Handle Suspend Request field */
  hhash->SuspendRequest = DAL_HASH_SUSPEND;
}

/**
  * @brief  Suspend the HASH processing when in DMA mode.
  * @param  hhash HASH handle.
  * @note   When suspension attempt occurs at the very end of a DMA transfer and
  *         all the data have already been entered in the Peripheral, hhash->State is
  *         set to DAL_HASH_STATE_READY and the API returns DAL_ERROR. It is
  *         recommended to wrap-up the processing in reading the digest as usual.
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_HASH_DMAFeed_ProcessSuspend(HASH_HandleTypeDef *hhash)
{
  uint32_t tmp_remaining_DMATransferSize_inWords;
  uint32_t tmp_initial_DMATransferSize_inWords;
  uint32_t tmp_words_already_pushed;

  if (hhash->State == DAL_HASH_STATE_READY)
  {
    return DAL_ERROR;
  }
  else
  {

    /* Make sure there is enough time to suspend the processing */
    tmp_remaining_DMATransferSize_inWords = ((DMA_Stream_TypeDef *)hhash->hdmain->Instance)->NDATA;

    if (tmp_remaining_DMATransferSize_inWords <= HASH_DMA_SUSPENSION_WORDS_LIMIT)
    {
      /* No suspension attempted since almost to the end of the transferred data. */
      /* Best option for user code is to wrap up low priority message hashing     */
      return DAL_ERROR;
    }

    /* Wait for BUSY flag to be reset */
    if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_BUSY, SET, HASH_TIMEOUTVALUE) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    if (__DAL_HASH_GET_FLAG(HASH_FLAG_DCIS) != RESET)
    {
      return DAL_ERROR;
    }

    /* Wait for BUSY flag to be set */
    if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_BUSY, RESET, HASH_TIMEOUTVALUE) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }
    /* Disable DMA channel */
    /* Note that the Abort function will
      - Clear the transfer error flags
      - Unlock
      - Set the State
    */
    if (DAL_DMA_Abort(hhash->hdmain) != DAL_OK)
    {
      return DAL_ERROR;
    }

    /* Clear DMAE bit */
    CLEAR_BIT(HASH->CTRL, HASH_CTRL_DMAEN);

    /* Wait for BUSY flag to be reset */
    if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_BUSY, SET, HASH_TIMEOUTVALUE) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    if (__DAL_HASH_GET_FLAG(HASH_FLAG_DCIS) != RESET)
    {
      return DAL_ERROR;
    }

    /* At this point, DMA interface is disabled and no transfer is on-going */
    /* Retrieve from the DMA handle how many words remain to be written */
    tmp_remaining_DMATransferSize_inWords = ((DMA_Stream_TypeDef *)hhash->hdmain->Instance)->NDATA;

    if (tmp_remaining_DMATransferSize_inWords == 0U)
    {
      /* All the DMA transfer is actually done. Suspension occurred at the very end
         of the transfer. Either the digest computation is about to start (HASH case)
         or processing is about to move from one step to another (HMAC case).
         In both cases, the processing can't be suspended at this point. It is
         safer to
         - retrieve the low priority block digest before starting the high
           priority block processing (HASH case)
         - re-attempt a new suspension (HMAC case)
         */
      return DAL_ERROR;
    }
    else
    {

      /* Compute how many words were supposed to be transferred by DMA */
      tmp_initial_DMATransferSize_inWords = (((hhash->HashInCount % 4U) != 0U) ? \
                                             ((hhash->HashInCount + 3U) / 4U) : (hhash->HashInCount / 4U));

      /* If discrepancy between the number of words reported by DMA Peripheral and
        the numbers of words entered as reported by HASH Peripheral, correct it */
      /* tmp_words_already_pushed reflects the number of words that were already pushed before
         the start of DMA transfer (multi-buffer processing case) */
      tmp_words_already_pushed = hhash->NbWordsAlreadyPushed;
      if (((tmp_words_already_pushed + tmp_initial_DMATransferSize_inWords - \
            tmp_remaining_DMATransferSize_inWords) % 16U) != HASH_NBW_PUSHED())
      {
        tmp_remaining_DMATransferSize_inWords--; /* one less word to be transferred again */
      }

      /* Accordingly, update the input pointer that points at the next word to be
         transferred to the Peripheral by DMA */
      hhash->pHashInBuffPtr +=  4U * (tmp_initial_DMATransferSize_inWords - tmp_remaining_DMATransferSize_inWords) ;

      /* And store in HashInCount the remaining size to transfer (in bytes) */
      hhash->HashInCount = 4U * tmp_remaining_DMATransferSize_inWords;

    }

    /* Set State as suspended */
    hhash->State = DAL_HASH_STATE_SUSPENDED;

    return DAL_OK;

  }
}

/**
  * @brief  Return the HASH handle error code.
  * @param  hhash pointer to a HASH_HandleTypeDef structure.
  * @retval HASH Error Code
  */
uint32_t DAL_HASH_GetError(HASH_HandleTypeDef *hhash)
{
  /* Return HASH Error Code */
  return hhash->ErrorCode;
}
/**
  * @}
  */


/**
  * @}
  */

/** @defgroup HASH_Private_Functions HASH Private Functions
  * @{
  */

/**
  * @brief DMA HASH Input Data transfer completion callback.
  * @param hdma DMA handle.
  * @note  In case of HMAC processing, HASH_DMAXferCplt() initiates
  *        the next DMA transfer for the following HMAC step.
  * @retval None
  */
static void HASH_DMAXferCplt(DMA_HandleTypeDef *hdma)
{
  HASH_HandleTypeDef *hhash = (HASH_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
  uint32_t inputaddr;
  uint32_t buffersize;
  DAL_StatusTypeDef status = DAL_OK;

  if (hhash->State != DAL_HASH_STATE_SUSPENDED)
  {

    /* Disable the DMA transfer */
    CLEAR_BIT(HASH->CTRL, HASH_CTRL_DMAEN);

    if (READ_BIT(HASH->CTRL, HASH_CTRL_MODESEL) == 0U)
    {
      /* If no HMAC processing, input data transfer is now over */

      /* Change the HASH state to ready */
      hhash->State = DAL_HASH_STATE_READY;

      /* Call Input data transfer complete call back */
#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
      hhash->InCpltCallback(hhash);
#else
      DAL_HASH_InCpltCallback(hhash);
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */

    }
    else
    {
      /* HMAC processing: depending on the current HMAC step and whether or
      not multi-buffer processing is on-going, the next step is initiated
      and MDMAT bit is set.  */


      if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_3)
      {
        /* This is the end of HMAC processing */

        /* Change the HASH state to ready */
        hhash->State = DAL_HASH_STATE_READY;

        /* Call Input data transfer complete call back
        (note that the last DMA transfer was that of the key
        for the outer HASH operation). */
#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
        hhash->InCpltCallback(hhash);
#else
        DAL_HASH_InCpltCallback(hhash);
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */

        return;
      }
      else if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_1)
      {
        inputaddr = (uint32_t)hhash->pHashMsgBuffPtr;     /* DMA transfer start address */
        buffersize = hhash->HashBuffSize;                 /* DMA transfer size (in bytes) */
        hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_2;        /* Move phase from Step 1 to Step 2 */

        /* In case of suspension request, save the new starting parameters */
        hhash->HashInCount = hhash->HashBuffSize;         /* Initial DMA transfer size (in bytes) */
        hhash->pHashInBuffPtr  = hhash->pHashMsgBuffPtr ; /* DMA transfer start address           */

        hhash->NbWordsAlreadyPushed = 0U;                  /* Reset number of words already pushed */
#if defined(HASH_CTRL_MDMAT)
        /* Check whether or not digest calculation must be disabled (in case of multi-buffer HMAC processing) */
        if (hhash->DigestCalculationDisable != RESET)
        {
          /* Digest calculation is disabled: Step 2 must start with MDMAT bit set,
          no digest calculation will be triggered at the end of the input buffer feeding to the Peripheral */
          __DAL_HASH_SET_MDMAT();
        }
#endif /* HASH_CTRL_MDMAT*/
      }
      else  /*case (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_2)*/
      {
        if (hhash->DigestCalculationDisable != RESET)
        {
          /* No automatic move to Step 3 as a new message buffer will be fed to the Peripheral
          (case of multi-buffer HMAC processing):
          DCAL must not be set.
          Phase remains in Step 2, MDMAT remains set at this point.
          Change the HASH state to ready and call Input data transfer complete call back. */
          hhash->State = DAL_HASH_STATE_READY;
#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
          hhash->InCpltCallback(hhash);
#else
          DAL_HASH_InCpltCallback(hhash);
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */
          return ;
        }
        else
        {
          /* Digest calculation is not disabled (case of single buffer input or last buffer
          of multi-buffer HMAC processing) */
          inputaddr = (uint32_t)hhash->Init.pKey;       /* DMA transfer start address */
          buffersize = hhash->Init.KeySize;             /* DMA transfer size (in bytes) */
          hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_3;    /* Move phase from Step 2 to Step 3 */
          /* In case of suspension request, save the new starting parameters */
          hhash->HashInCount = hhash->Init.KeySize;     /* Initial size for second DMA transfer (input data) */
          hhash->pHashInBuffPtr  = hhash->Init.pKey ;   /* address passed to DMA, now entering data message */

          hhash->NbWordsAlreadyPushed = 0U;              /* Reset number of words already pushed */
        }
      }

      /* Configure the Number of valid bits in last word of the message */
      __DAL_HASH_SET_NBVALIDBITS(buffersize);

      /* Set the HASH DMA transfer completion call back */
      hhash->hdmain->XferCpltCallback = HASH_DMAXferCplt;

      /* Enable the DMA In DMA stream */
      status = DAL_DMA_Start_IT(hhash->hdmain, inputaddr, (uint32_t)&HASH->INDATA, \
                                (((buffersize % 4U) != 0U) ? ((buffersize + (4U - (buffersize % 4U))) / 4U) : \
                                 (buffersize / 4U)));

      /* Enable DMA requests */
      SET_BIT(HASH->CTRL, HASH_CTRL_DMAEN);

      /* Return function status */
      if (status != DAL_OK)
      {
        /* Update HASH state machine to error */
        hhash->State = DAL_HASH_STATE_ERROR;
      }
      else
      {
        /* Change HASH state */
        hhash->State = DAL_HASH_STATE_BUSY;
      }
    }
  }

  return;
}

/**
  * @brief DMA HASH communication error callback.
  * @param hdma DMA handle.
  * @note  HASH_DMAError() callback invokes DAL_HASH_ErrorCallback() that
  *        can contain user code to manage the error.
  * @retval None
  */
static void HASH_DMAError(DMA_HandleTypeDef *hdma)
{
  HASH_HandleTypeDef *hhash = (HASH_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

  if (hhash->State != DAL_HASH_STATE_SUSPENDED)
  {
    hhash->ErrorCode |= DAL_HASH_ERROR_DMA;
    /* Set HASH state to ready to prevent any blocking issue in user code
       present in DAL_HASH_ErrorCallback() */
    hhash->State = DAL_HASH_STATE_READY;
    /* Set HASH handle status to error */
    hhash->Status = DAL_ERROR;
#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
    hhash->ErrorCallback(hhash);
#else
    DAL_HASH_ErrorCallback(hhash);
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */
    /* After error handling by code user, reset HASH handle DAL status */
    hhash->Status = DAL_OK;

  }
}

/**
  * @brief  Feed the input buffer to the HASH Peripheral.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to input buffer.
  * @param  Size the size of input buffer in bytes.
  * @note   HASH_WriteData() regularly reads hhash->SuspendRequest to check whether
  *         or not the HASH processing must be suspended. If this is the case, the
  *         processing is suspended when possible and the Peripheral feeding point reached at
  *         suspension time is stored in the handle for resumption later on.
  * @retval DAL status
  */
static DAL_StatusTypeDef HASH_WriteData(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size)
{
  uint32_t buffercounter;
  __IO uint32_t inputaddr = (uint32_t) pInBuffer;

  for (buffercounter = 0U; buffercounter < Size; buffercounter += 4U)
  {
    /* Write input data 4 bytes at a time */
    HASH->INDATA = *(uint32_t *)inputaddr;
    inputaddr += 4U;

    /* If the suspension flag has been raised and if the processing is not about
    to end, suspend processing */
    if ((hhash->SuspendRequest == DAL_HASH_SUSPEND) && ((buffercounter + 4U) < Size))
    {
      /* Wait for DINIS = 1, which occurs when 16 32-bit locations are free
      in the input buffer */
      if (__DAL_HASH_GET_FLAG(HASH_FLAG_DINIS))
      {
        /* Reset SuspendRequest */
        hhash->SuspendRequest = DAL_HASH_SUSPEND_NONE;

        /* Depending whether the key or the input data were fed to the Peripheral, the feeding point
        reached at suspension time is not saved in the same handle fields */
        if ((hhash->Phase == DAL_HASH_PHASE_PROCESS) || (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_2))
        {
          /* Save current reading and writing locations of Input and Output buffers */
          hhash->pHashInBuffPtr = (uint8_t *)inputaddr;
          /* Save the number of bytes that remain to be processed at this point */
          hhash->HashInCount    =  Size - (buffercounter + 4U);
        }
        else if ((hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_1) || (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_3))
        {
          /* Save current reading and writing locations of Input and Output buffers */
          hhash->pHashKeyBuffPtr  = (uint8_t *)inputaddr;
          /* Save the number of bytes that remain to be processed at this point */
          hhash->HashKeyCount  =  Size - (buffercounter + 4U);
        }
        else
        {
          /* Unexpected phase: unlock process and report error */
          hhash->State = DAL_HASH_STATE_READY;
          __DAL_UNLOCK(hhash);
          return DAL_ERROR;
        }

        /* Set the HASH state to Suspended and exit to stop entering data */
        hhash->State = DAL_HASH_STATE_SUSPENDED;

        return DAL_OK;
      } /* if (__DAL_HASH_GET_FLAG(HASH_FLAG_DINIS))  */
    } /* if ((hhash->SuspendRequest == DAL_HASH_SUSPEND) && ((buffercounter+4) < Size)) */
  }   /* for(buffercounter = 0; buffercounter < Size; buffercounter+=4)                 */

  /* At this point, all the data have been entered to the Peripheral: exit */
  return  DAL_OK;
}

/**
  * @brief  Retrieve the message digest.
  * @param  pMsgDigest pointer to the computed digest.
  * @param  Size message digest size in bytes.
  * @retval None
  */
static void HASH_GetDigest(uint8_t *pMsgDigest, uint8_t Size)
{
  uint32_t msgdigest = (uint32_t)pMsgDigest;

  switch (Size)
  {
    /* Read the message digest */
    case 16:  /* MD5 */
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[0]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[1]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[2]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[3]);
      break;
    case 20:  /* SHA1 */
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[0]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[1]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[2]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[3]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[4]);
      break;
    case 28:  /* SHA224 */
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[0]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[1]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[2]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[3]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[4]);
#if defined(HASH_CTRL_MDMAT)
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH_DIGEST->HR[5]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH_DIGEST->HR[6]);
#endif /* HASH_CTRL_MDMAT*/
      break;
    case 32:   /* SHA256 */
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[0]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[1]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[2]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[3]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH->DIG[4]);
#if defined(HASH_CTRL_MDMAT)
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH_DIGEST->HR[5]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH_DIGEST->HR[6]);
      msgdigest += 4U;
      *(uint32_t *)(msgdigest) = __REV(HASH_DIGEST->HR[7]);
#endif /* HASH_CTRL_MDMAT*/
      break;
    default:
      break;
  }
}



/**
  * @brief  Handle HASH processing Timeout.
  * @param  hhash HASH handle.
  * @param  Flag specifies the HASH flag to check.
  * @param  Status the Flag status (SET or RESET).
  * @param  Timeout Timeout duration.
  * @retval DAL status
  */
static DAL_StatusTypeDef HASH_WaitOnFlagUntilTimeout(HASH_HandleTypeDef *hhash, uint32_t Flag, FlagStatus Status,
                                                     uint32_t Timeout)
{
  uint32_t tickstart = DAL_GetTick();

  /* Wait until flag is set */
  if (Status == RESET)
  {
    while (__DAL_HASH_GET_FLAG(Flag) == RESET)
    {
      /* Check for the Timeout */
      if (Timeout != DAL_MAX_DELAY)
      {
        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          /* Set State to Ready to be able to restart later on */
          hhash->State  = DAL_HASH_STATE_READY;
          /* Store time out issue in handle status */
          hhash->Status = DAL_TIMEOUT;

          /* Process Unlocked */
          __DAL_UNLOCK(hhash);

          return DAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while (__DAL_HASH_GET_FLAG(Flag) != RESET)
    {
      /* Check for the Timeout */
      if (Timeout != DAL_MAX_DELAY)
      {
        if (((DAL_GetTick() - tickstart) > Timeout) || (Timeout == 0U))
        {
          /* Set State to Ready to be able to restart later on */
          hhash->State  = DAL_HASH_STATE_READY;
          /* Store time out issue in handle status */
          hhash->Status = DAL_TIMEOUT;

          /* Process Unlocked */
          __DAL_UNLOCK(hhash);

          return DAL_TIMEOUT;
        }
      }
    }
  }
  return DAL_OK;
}


/**
  * @brief  HASH processing in interruption mode.
  * @param  hhash HASH handle.
  * @note   HASH_IT() regularly reads hhash->SuspendRequest to check whether
  *         or not the HASH processing must be suspended. If this is the case, the
  *         processing is suspended when possible and the Peripheral feeding point reached at
  *         suspension time is stored in the handle for resumption later on.
  * @retval DAL status
  */
static DAL_StatusTypeDef HASH_IT(HASH_HandleTypeDef *hhash)
{
  if (hhash->State == DAL_HASH_STATE_BUSY)
  {
    /* ITCounter must not be equal to 0 at this point. Report an error if this is the case. */
    if (hhash->HashITCounter == 0U)
    {
      /* Disable Interrupts */
      __DAL_HASH_DISABLE_IT(HASH_IT_DINI | HASH_IT_DCI);
      /* HASH state set back to Ready to prevent any issue in user code
         present in DAL_HASH_ErrorCallback() */
      hhash->State = DAL_HASH_STATE_READY;
      return DAL_ERROR;
    }
    else if (hhash->HashITCounter == 1U)
    {
      /* This is the first call to HASH_IT, the first input data are about to be
         entered in the Peripheral. A specific processing is carried out at this point to
         start-up the processing. */
      hhash->HashITCounter = 2U;
    }
    else
    {
      /* Cruise speed reached, HashITCounter remains equal to 3 until the end of
        the HASH processing or the end of the current step for HMAC processing. */
      hhash->HashITCounter = 3U;
    }

    /* If digest is ready */
    if (__DAL_HASH_GET_FLAG(HASH_FLAG_DCIS))
    {
      /* Read the digest */
      HASH_GetDigest(hhash->pHashOutBuffPtr, HASH_DIGEST_LENGTH());

      /* Disable Interrupts */
      __DAL_HASH_DISABLE_IT(HASH_IT_DINI | HASH_IT_DCI);
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_READY;
      /* Reset HASH state machine */
      hhash->Phase = DAL_HASH_PHASE_READY;
      /* Call digest computation complete call back */
#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
      hhash->DgstCpltCallback(hhash);
#else
      DAL_HASH_DgstCpltCallback(hhash);
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */

      return DAL_OK;
    }

    /* If Peripheral ready to accept new data */
    if (__DAL_HASH_GET_FLAG(HASH_FLAG_DINIS))
    {

      /* If the suspension flag has been raised and if the processing is not about
         to end, suspend processing */
      if ((hhash->HashInCount != 0U) && (hhash->SuspendRequest == DAL_HASH_SUSPEND))
      {
        /* Disable Interrupts */
        __DAL_HASH_DISABLE_IT(HASH_IT_DINI | HASH_IT_DCI);

        /* Reset SuspendRequest */
        hhash->SuspendRequest = DAL_HASH_SUSPEND_NONE;

        /* Change the HASH state */
        hhash->State = DAL_HASH_STATE_SUSPENDED;

        return DAL_OK;
      }

      /* Enter input data in the Peripheral through HASH_Write_Block_Data() call and
        check whether the digest calculation has been triggered */
      if (HASH_Write_Block_Data(hhash) == HASH_DIGEST_CALCULATION_STARTED)
      {
        /* Call Input data transfer complete call back
           (called at the end of each step for HMAC) */
#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
        hhash->InCpltCallback(hhash);
#else
        DAL_HASH_InCpltCallback(hhash);
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */

        if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_1)
        {
          /* Wait until Peripheral is not busy anymore */
          if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_BUSY, SET, HASH_TIMEOUTVALUE) != DAL_OK)
          {
            /* Disable Interrupts */
            __DAL_HASH_DISABLE_IT(HASH_IT_DINI | HASH_IT_DCI);
            return DAL_TIMEOUT;
          }
          /* Initialization start for HMAC STEP 2 */
          hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_2;        /* Move phase from Step 1 to Step 2 */
          __DAL_HASH_SET_NBVALIDBITS(hhash->HashBuffSize);  /* Set NBLW for the input message */
          hhash->HashInCount = hhash->HashBuffSize;         /* Set the input data size (in bytes) */
          hhash->pHashInBuffPtr = hhash->pHashMsgBuffPtr;   /* Set the input data address */
          hhash->HashITCounter = 1;                         /* Set ITCounter to 1 to indicate the start
                                                               of a new phase */
          __DAL_HASH_ENABLE_IT(HASH_IT_DINI);               /* Enable IT (was disabled in HASH_Write_Block_Data) */
        }
        else if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_2)
        {
          /* Wait until Peripheral is not busy anymore */
          if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_BUSY, SET, HASH_TIMEOUTVALUE) != DAL_OK)
          {
            /* Disable Interrupts */
            __DAL_HASH_DISABLE_IT(HASH_IT_DINI | HASH_IT_DCI);
            return DAL_TIMEOUT;
          }
          /* Initialization start for HMAC STEP 3 */
          hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_3;         /* Move phase from Step 2 to Step 3 */
          __DAL_HASH_SET_NBVALIDBITS(hhash->Init.KeySize);   /* Set NBLW for the key */
          hhash->HashInCount = hhash->Init.KeySize;          /* Set the key size (in bytes) */
          hhash->pHashInBuffPtr = hhash->Init.pKey;          /* Set the key address */
          hhash->HashITCounter = 1;                          /* Set ITCounter to 1 to indicate the start
                                                                of a new phase */
          __DAL_HASH_ENABLE_IT(HASH_IT_DINI);                /* Enable IT (was disabled in HASH_Write_Block_Data) */
        }
        else
        {
          /* Nothing to do */
        }
      } /* if (HASH_Write_Block_Data(hhash) == HASH_DIGEST_CALCULATION_STARTED) */
    }  /* if (__DAL_HASH_GET_FLAG(HASH_FLAG_DINIS))*/

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }
}


/**
  * @brief  Write a block of data in HASH Peripheral in interruption mode.
  * @param  hhash HASH handle.
  * @note   HASH_Write_Block_Data() is called under interruption by HASH_IT().
  * @retval DAL status
  */
static uint32_t HASH_Write_Block_Data(HASH_HandleTypeDef *hhash)
{
  uint32_t inputaddr;
  uint32_t buffercounter;
  uint32_t inputcounter;
  uint32_t ret = HASH_DIGEST_CALCULATION_NOT_STARTED;

  /* If there are more than 64 bytes remaining to be entered */
  if (hhash->HashInCount > 64U)
  {
    inputaddr = (uint32_t)hhash->pHashInBuffPtr;
    /* Write the Input block in the Data IN register
      (16 32-bit words, or 64 bytes are entered) */
    for (buffercounter = 0U; buffercounter < 64U; buffercounter += 4U)
    {
      HASH->INDATA = *(uint32_t *)inputaddr;
      inputaddr += 4U;
    }
    /* If this is the start of input data entering, an additional word
      must be entered to start up the HASH processing */
    if (hhash->HashITCounter == 2U)
    {
      HASH->INDATA = *(uint32_t *)inputaddr;
      if (hhash->HashInCount >= 68U)
      {
        /* There are still data waiting to be entered in the Peripheral.
           Decrement buffer counter and set pointer to the proper
           memory location for the next data entering round. */
        hhash->HashInCount -= 68U;
        hhash->pHashInBuffPtr += 68U;
      }
      else
      {
        /* All the input buffer has been fed to the HW. */
        hhash->HashInCount = 0U;
      }
    }
    else
    {
      /* 64 bytes have been entered and there are still some remaining:
         Decrement buffer counter and set pointer to the proper
        memory location for the next data entering round.*/
      hhash->HashInCount -= 64U;
      hhash->pHashInBuffPtr += 64U;
    }
  }
  else
  {
    /* 64 or less bytes remain to be entered. This is the last
      data entering round. */

    /* Get the buffer address */
    inputaddr = (uint32_t)hhash->pHashInBuffPtr;
    /* Get the buffer counter */
    inputcounter = hhash->HashInCount;
    /* Disable Interrupts */
    __DAL_HASH_DISABLE_IT(HASH_IT_DINI);

    /* Write the Input block in the Data IN register */
    for (buffercounter = 0U; buffercounter < ((inputcounter + 3U) / 4U); buffercounter++)
    {
      HASH->INDATA = *(uint32_t *)inputaddr;
      inputaddr += 4U;
    }

    if (hhash->Accumulation == 1U)
    {
      /* Field accumulation is set, API only feeds data to the Peripheral and under interruption.
         The digest computation will be started when the last buffer data are entered. */

      /* Reset multi buffers accumulation flag */
      hhash->Accumulation = 0U;
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_READY;
      /* Call Input data transfer complete call back */
#if (USE_DAL_HASH_REGISTER_CALLBACKS == 1)
      hhash->InCpltCallback(hhash);
#else
      DAL_HASH_InCpltCallback(hhash);
#endif /* USE_DAL_HASH_REGISTER_CALLBACKS */
    }
    else
    {
      /* Start the Digest calculation */
      __DAL_HASH_START_DIGEST();
      /* Return indication that digest calculation has started:
         this return value triggers the call to Input data transfer
         complete call back as well as the proper transition from
         one step to another in HMAC mode. */
      ret = HASH_DIGEST_CALCULATION_STARTED;
    }
    /* Reset buffer counter */
    hhash->HashInCount = 0;
  }

  /* Return whether or digest calculation has started */
  return ret;
}

/**
  * @brief  HMAC processing in polling mode.
  * @param  hhash HASH handle.
  * @param  Timeout Timeout value.
  * @retval DAL status
  */
static DAL_StatusTypeDef HMAC_Processing(HASH_HandleTypeDef *hhash, uint32_t Timeout)
{
  /* Ensure first that Phase is correct */
  if ((hhash->Phase != DAL_HASH_PHASE_HMAC_STEP_1) && (hhash->Phase != DAL_HASH_PHASE_HMAC_STEP_2)
      && (hhash->Phase != DAL_HASH_PHASE_HMAC_STEP_3))
  {
    /* Change the HASH state */
    hhash->State = DAL_HASH_STATE_READY;

    /* Process Unlock */
    __DAL_UNLOCK(hhash);

    /* Return function status */
    return DAL_ERROR;
  }

  /* HMAC Step 1 processing */
  if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_1)
  {
    /************************** STEP 1 ******************************************/
    /* Configure the Number of valid bits in last word of the message */
    __DAL_HASH_SET_NBVALIDBITS(hhash->Init.KeySize);

    /* Write input buffer in Data register */
    hhash->Status = HASH_WriteData(hhash, hhash->pHashKeyBuffPtr, hhash->HashKeyCount);
    if (hhash->Status != DAL_OK)
    {
      return hhash->Status;
    }

    /* Check whether or not key entering process has been suspended */
    if (hhash->State == DAL_HASH_STATE_SUSPENDED)
    {
      /* Process Unlocked */
      __DAL_UNLOCK(hhash);

      /* Stop right there and return function status */
      return DAL_OK;
    }

    /* No processing suspension at this point: set DCAL bit. */
    __DAL_HASH_START_DIGEST();

    /* Wait for BUSY flag to be cleared */
    if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_BUSY, SET, Timeout) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    /* Move from Step 1 to Step 2 */
    hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_2;

  }

  /* HMAC Step 2 processing.
     After phase check, HMAC_Processing() may
     - directly start up from this point in resumption case
       if the same Step 2 processing was suspended previously
    - or fall through from the Step 1 processing carried out hereabove */
  if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_2)
  {
    /************************** STEP 2 ******************************************/
    /* Configure the Number of valid bits in last word of the message */
    __DAL_HASH_SET_NBVALIDBITS(hhash->HashBuffSize);

    /* Write input buffer in Data register */
    hhash->Status = HASH_WriteData(hhash, hhash->pHashInBuffPtr, hhash->HashInCount);
    if (hhash->Status != DAL_OK)
    {
      return hhash->Status;
    }

    /* Check whether or not data entering process has been suspended */
    if (hhash->State == DAL_HASH_STATE_SUSPENDED)
    {
      /* Process Unlocked */
      __DAL_UNLOCK(hhash);

      /* Stop right there and return function status */
      return DAL_OK;
    }

    /* No processing suspension at this point: set DCAL bit. */
    __DAL_HASH_START_DIGEST();

    /* Wait for BUSY flag to be cleared */
    if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_BUSY, SET, Timeout) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    /* Move from Step 2 to Step 3 */
    hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_3;
    /* In case Step 1 phase was suspended then resumed,
       set again Key input buffers and size before moving to
       next step */
    hhash->pHashKeyBuffPtr = hhash->Init.pKey;
    hhash->HashKeyCount    = hhash->Init.KeySize;
  }


  /* HMAC Step 3 processing.
      After phase check, HMAC_Processing() may
      - directly start up from this point in resumption case
        if the same Step 3 processing was suspended previously
     - or fall through from the Step 2 processing carried out hereabove */
  if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_3)
  {
    /************************** STEP 3 ******************************************/
    /* Configure the Number of valid bits in last word of the message */
    __DAL_HASH_SET_NBVALIDBITS(hhash->Init.KeySize);

    /* Write input buffer in Data register */
    hhash->Status = HASH_WriteData(hhash, hhash->pHashKeyBuffPtr, hhash->HashKeyCount);
    if (hhash->Status != DAL_OK)
    {
      return hhash->Status;
    }

    /* Check whether or not key entering process has been suspended */
    if (hhash->State == DAL_HASH_STATE_SUSPENDED)
    {
      /* Process Unlocked */
      __DAL_UNLOCK(hhash);

      /* Stop right there and return function status */
      return DAL_OK;
    }

    /* No processing suspension at this point: start the Digest calculation. */
    __DAL_HASH_START_DIGEST();

    /* Wait for DCIS flag to be set */
    if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_DCIS, RESET, Timeout) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    /* Read the message digest */
    HASH_GetDigest(hhash->pHashOutBuffPtr, HASH_DIGEST_LENGTH());

    /* Reset HASH state machine */
    hhash->Phase = DAL_HASH_PHASE_READY;
  }

  /* Change the HASH state */
  hhash->State = DAL_HASH_STATE_READY;

  /* Process Unlock */
  __DAL_UNLOCK(hhash);

  /* Return function status */
  return DAL_OK;
}


/**
  * @brief  Initialize the HASH peripheral, next process pInBuffer then
  *         read the computed digest.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest.
  * @param  Timeout Timeout value.
  * @param  Algorithm HASH algorithm.
  * @retval DAL status
  */
DAL_StatusTypeDef HASH_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t *pOutBuffer,
                             uint32_t Timeout, uint32_t Algorithm)
{
  uint8_t *pInBuffer_tmp;  /* input data address, input parameter of HASH_WriteData()         */
  uint32_t Size_tmp; /* input data size (in bytes), input parameter of HASH_WriteData() */
  DAL_HASH_StateTypeDef State_tmp = hhash->State;


  /* Initiate HASH processing in case of start or resumption */
  if ((State_tmp == DAL_HASH_STATE_READY) || (State_tmp == DAL_HASH_STATE_SUSPENDED))
  {
    /* Check input parameters */
    if ((pInBuffer == NULL) || (pOutBuffer == NULL))
    {
      hhash->State = DAL_HASH_STATE_READY;
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hhash);

    /* Check if initialization phase has not been already performed */
    if (hhash->Phase == DAL_HASH_PHASE_READY)
    {
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;

      /* Select the HASH algorithm, clear HMAC mode and long key selection bit, reset the HASH processor core */
      MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL, Algorithm | HASH_CTRL_INITCAL);

      /* Configure the number of valid bits in last word of the message */
      __DAL_HASH_SET_NBVALIDBITS(Size);

      /* pInBuffer_tmp and Size_tmp are initialized to be used afterwards as
      input parameters of HASH_WriteData() */
      pInBuffer_tmp = pInBuffer;   /* pInBuffer_tmp is set to the input data address */
      Size_tmp = Size;             /* Size_tmp contains the input data size in bytes */

      /* Set the phase */
      hhash->Phase = DAL_HASH_PHASE_PROCESS;
    }
    else if (hhash->Phase == DAL_HASH_PHASE_PROCESS)
    {
      /* if the Peripheral has already been initialized, two cases are possible */

      /* Process resumption time ... */
      if (hhash->State == DAL_HASH_STATE_SUSPENDED)
      {
        /* Since this is resumption, pInBuffer_tmp and Size_tmp are not set
        to the API input parameters but to those saved beforehand by HASH_WriteData()
        when the processing was suspended */
        pInBuffer_tmp = hhash->pHashInBuffPtr;
        Size_tmp = hhash->HashInCount;
      }
      /* ... or multi-buffer HASH processing end */
      else
      {
        /* pInBuffer_tmp and Size_tmp are initialized to be used afterwards as
        input parameters of HASH_WriteData() */
        pInBuffer_tmp = pInBuffer;
        Size_tmp = Size;
        /* Configure the number of valid bits in last word of the message */
        __DAL_HASH_SET_NBVALIDBITS(Size);
      }
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;
    }
    else
    {
      /* Phase error */
      hhash->State = DAL_HASH_STATE_READY;

      /* Process Unlocked */
      __DAL_UNLOCK(hhash);

      /* Return function status */
      return DAL_ERROR;
    }


    /* Write input buffer in Data register */
    hhash->Status = HASH_WriteData(hhash, pInBuffer_tmp, Size_tmp);
    if (hhash->Status != DAL_OK)
    {
      return hhash->Status;
    }

    /* If the process has not been suspended, carry on to digest calculation */
    if (hhash->State != DAL_HASH_STATE_SUSPENDED)
    {
      /* Start the Digest calculation */
      __DAL_HASH_START_DIGEST();

      /* Wait for DCIS flag to be set */
      if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_DCIS, RESET, Timeout) != DAL_OK)
      {
        return DAL_TIMEOUT;
      }

      /* Read the message digest */
      HASH_GetDigest(pOutBuffer, HASH_DIGEST_LENGTH());

      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_READY;

      /* Reset HASH state machine */
      hhash->Phase = DAL_HASH_PHASE_READY;

    }

    /* Process Unlocked */
    __DAL_UNLOCK(hhash);

    /* Return function status */
    return DAL_OK;

  }
  else
  {
    return DAL_BUSY;
  }
}


/**
  * @brief  If not already done, initialize the HASH peripheral then
  *         processes pInBuffer.
  * @note   Field hhash->Phase of HASH handle is tested to check whether or not
  *         the Peripheral has already been initialized.
  * @note   The input buffer size (in bytes) must be a multiple of 4 otherwise, the
  *         HASH digest computation is corrupted.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes, must be a multiple of 4.
  * @param  Algorithm HASH algorithm.
  * @retval DAL status
  */
DAL_StatusTypeDef HASH_Accumulate(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint32_t Algorithm)
{
  uint8_t *pInBuffer_tmp;   /* input data address, input parameter of HASH_WriteData()         */
  uint32_t Size_tmp;  /* input data size (in bytes), input parameter of HASH_WriteData() */
  DAL_HASH_StateTypeDef State_tmp = hhash->State;

  /* Make sure the input buffer size (in bytes) is a multiple of 4 */
  if ((Size % 4U) != 0U)
  {
    return  DAL_ERROR;
  }

  /* Initiate HASH processing in case of start or resumption */
  if ((State_tmp == DAL_HASH_STATE_READY) || (State_tmp == DAL_HASH_STATE_SUSPENDED))
  {
    /* Check input parameters */
    if ((pInBuffer == NULL) || (Size == 0U))
    {
      hhash->State = DAL_HASH_STATE_READY;
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hhash);

    /* If resuming the HASH processing */
    if (hhash->State == DAL_HASH_STATE_SUSPENDED)
    {
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;

      /* Since this is resumption, pInBuffer_tmp and Size_tmp are not set
         to the API input parameters but to those saved beforehand by HASH_WriteData()
         when the processing was suspended */
      pInBuffer_tmp = hhash->pHashInBuffPtr;  /* pInBuffer_tmp is set to the input data address */
      Size_tmp = hhash->HashInCount;          /* Size_tmp contains the input data size in bytes */

    }
    else
    {
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;

      /* pInBuffer_tmp and Size_tmp are initialized to be used afterwards as
         input parameters of HASH_WriteData() */
      pInBuffer_tmp = pInBuffer;    /* pInBuffer_tmp is set to the input data address */
      Size_tmp = Size;              /* Size_tmp contains the input data size in bytes */

      /* Check if initialization phase has already be performed */
      if (hhash->Phase == DAL_HASH_PHASE_READY)
      {
        /* Select the HASH algorithm, clear HMAC mode and long key selection bit, reset the HASH processor core */
        MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL, Algorithm | HASH_CTRL_INITCAL);
      }

      /* Set the phase */
      hhash->Phase = DAL_HASH_PHASE_PROCESS;

    }

    /* Write input buffer in Data register */
    hhash->Status = HASH_WriteData(hhash, pInBuffer_tmp, Size_tmp);
    if (hhash->Status != DAL_OK)
    {
      return hhash->Status;
    }

    /* If the process has not been suspended, move the state to Ready */
    if (hhash->State != DAL_HASH_STATE_SUSPENDED)
    {
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_READY;
    }

    /* Process Unlocked */
    __DAL_UNLOCK(hhash);

    /* Return function status */
    return DAL_OK;

  }
  else
  {
    return DAL_BUSY;
  }


}


/**
  * @brief  If not already done, initialize the HASH peripheral then
  *         processes pInBuffer in interruption mode.
  * @note   Field hhash->Phase of HASH handle is tested to check whether or not
  *         the Peripheral has already been initialized.
  * @note   The input buffer size (in bytes) must be a multiple of 4 otherwise, the
  *         HASH digest computation is corrupted.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes, must be a multiple of 4.
  * @param  Algorithm HASH algorithm.
  * @retval DAL status
  */
DAL_StatusTypeDef HASH_Accumulate_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint32_t Algorithm)
{
  DAL_HASH_StateTypeDef State_tmp = hhash->State;
  __IO uint32_t inputaddr = (uint32_t) pInBuffer;
  uint32_t SizeVar = Size;

  /* Make sure the input buffer size (in bytes) is a multiple of 4 */
  if ((Size % 4U) != 0U)
  {
    return  DAL_ERROR;
  }

  /* Initiate HASH processing in case of start or resumption */
  if ((State_tmp == DAL_HASH_STATE_READY) || (State_tmp == DAL_HASH_STATE_SUSPENDED))
  {
    /* Check input parameters */
    if ((pInBuffer == NULL) || (Size == 0U))
    {
      hhash->State = DAL_HASH_STATE_READY;
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hhash);

    /* If resuming the HASH processing */
    if (hhash->State == DAL_HASH_STATE_SUSPENDED)
    {
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;
    }
    else
    {
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;

      /* Check if initialization phase has already be performed */
      if (hhash->Phase == DAL_HASH_PHASE_READY)
      {
        /* Select the HASH algorithm, clear HMAC mode and long key selection bit, reset the HASH processor core */
        MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL, Algorithm | HASH_CTRL_INITCAL);
        hhash->HashITCounter = 1;
      }
      else
      {
        hhash->HashITCounter = 3; /* 'cruise-speed' reached during a previous buffer processing */
      }

      /* Set the phase */
      hhash->Phase = DAL_HASH_PHASE_PROCESS;

      /* If DINIS is equal to 0 (for example if an incomplete block has been previously
       fed to the Peripheral), the DINIE interruption won't be triggered when DINIE is set.
       Therefore, first words are manually entered until DINIS raises, or until there
       is not more data to enter. */
      while ((!(__DAL_HASH_GET_FLAG(HASH_FLAG_DINIS))) && (SizeVar > 0U))
      {

        /* Write input data 4 bytes at a time */
        HASH->INDATA = *(uint32_t *)inputaddr;
        inputaddr += 4U;
        SizeVar -= 4U;
      }

      /* If DINIS is still not set or if all the data have been fed, stop here */
      if ((!(__DAL_HASH_GET_FLAG(HASH_FLAG_DINIS))) || (SizeVar == 0U))
      {
        /* Change the HASH state */
        hhash->State = DAL_HASH_STATE_READY;

        /* Process Unlock */
        __DAL_UNLOCK(hhash);

        /* Return function status */
        return DAL_OK;
      }

      /* otherwise, carry on in interrupt-mode */
      hhash->HashInCount = SizeVar;               /* Counter used to keep track of number of data
                                                  to be fed to the Peripheral */
      hhash->pHashInBuffPtr = (uint8_t *)inputaddr;       /* Points at data which will be fed to the Peripheral at
                                                  the next interruption */
      /* In case of suspension, hhash->HashInCount and hhash->pHashInBuffPtr contain
         the information describing where the HASH process is stopped.
         These variables are used later on to resume the HASH processing at the
         correct location. */

    }

    /* Set multi buffers accumulation flag */
    hhash->Accumulation = 1U;

    /* Process Unlock */
    __DAL_UNLOCK(hhash);

    /* Enable Data Input interrupt */
    __DAL_HASH_ENABLE_IT(HASH_IT_DINI);

    /* Return function status */
    return DAL_OK;

  }
  else
  {
    return DAL_BUSY;
  }

}



/**
  * @brief  Initialize the HASH peripheral, next process pInBuffer then
  *         read the computed digest in interruption mode.
  * @note   Digest is available in pOutBuffer.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest.
  * @param  Algorithm HASH algorithm.
  * @retval DAL status
  */
DAL_StatusTypeDef HASH_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t *pOutBuffer,
                                uint32_t Algorithm)
{
  DAL_HASH_StateTypeDef State_tmp = hhash->State;
  __IO uint32_t inputaddr = (uint32_t) pInBuffer;
  uint32_t polling_step = 0U;
  uint32_t initialization_skipped = 0U;
  uint32_t SizeVar = Size;

  /* If State is ready or suspended, start or resume IT-based HASH processing */
  if ((State_tmp == DAL_HASH_STATE_READY) || (State_tmp == DAL_HASH_STATE_SUSPENDED))
  {
    /* Check input parameters */
    if ((pInBuffer == NULL) || (Size == 0U) || (pOutBuffer == NULL))
    {
      hhash->State = DAL_HASH_STATE_READY;
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hhash);

    /* Change the HASH state */
    hhash->State = DAL_HASH_STATE_BUSY;

    /* Initialize IT counter */
    hhash->HashITCounter = 1;

    /* Check if initialization phase has already be performed */
    if (hhash->Phase == DAL_HASH_PHASE_READY)
    {
      /* Select the HASH algorithm, clear HMAC mode and long key selection bit, reset the HASH processor core */
      MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL, Algorithm | HASH_CTRL_INITCAL);

      /* Configure the number of valid bits in last word of the message */
      __DAL_HASH_SET_NBVALIDBITS(SizeVar);


      hhash->HashInCount = SizeVar;            /* Counter used to keep track of number of data
                                                  to be fed to the Peripheral */
      hhash->pHashInBuffPtr = pInBuffer;       /* Points at data which will be fed to the Peripheral at
                                                  the next interruption */
      /* In case of suspension, hhash->HashInCount and hhash->pHashInBuffPtr contain
         the information describing where the HASH process is stopped.
         These variables are used later on to resume the HASH processing at the
         correct location. */

      hhash->pHashOutBuffPtr = pOutBuffer;     /* Points at the computed digest */
    }
    else
    {
      initialization_skipped = 1; /* info user later on in case of multi-buffer */
    }

    /* Set the phase */
    hhash->Phase = DAL_HASH_PHASE_PROCESS;

    /* If DINIS is equal to 0 (for example if an incomplete block has been previously
      fed to the Peripheral), the DINIE interruption won't be triggered when DINIE is set.
      Therefore, first words are manually entered until DINIS raises. */
    while ((!(__DAL_HASH_GET_FLAG(HASH_FLAG_DINIS))) && (SizeVar > 3U))
    {
      polling_step = 1U; /* note that some words are entered before enabling the interrupt */

      /* Write input data 4 bytes at a time */
      HASH->INDATA = *(uint32_t *)inputaddr;
      inputaddr += 4U;
      SizeVar -= 4U;
    }

    if (polling_step == 1U)
    {
      if (SizeVar == 0U)
      {
        /* If all the data have been entered at this point, it only remains to
         read the digest */
        hhash->pHashOutBuffPtr = pOutBuffer;     /* Points at the computed digest */

        /* Start the Digest calculation */
        __DAL_HASH_START_DIGEST();
        /* Process Unlock */
        __DAL_UNLOCK(hhash);

        /* Enable Interrupts */
        __DAL_HASH_ENABLE_IT(HASH_IT_DCI);

        /* Return function status */
        return DAL_OK;
      }
      else if (__DAL_HASH_GET_FLAG(HASH_FLAG_DINIS))
      {
        /* It remains data to enter and the Peripheral is ready to trigger DINIE,
           carry on as usual.
           Update HashInCount and pHashInBuffPtr accordingly. */
        hhash->HashInCount = SizeVar;
        hhash->pHashInBuffPtr = (uint8_t *)inputaddr;
        /* Update the configuration of the number of valid bits in last word of the message */
        __DAL_HASH_SET_NBVALIDBITS(SizeVar);
        hhash->pHashOutBuffPtr = pOutBuffer;  /* Points at the computed digest */
        if (initialization_skipped == 1U)
        {
          hhash->HashITCounter = 3; /* 'cruise-speed' reached during a previous buffer processing */
        }
      }
      else
      {
        /* DINIS is not set but it remains a few data to enter (not enough for a full word).
           Manually enter the last bytes before enabling DCIE. */
        __DAL_HASH_SET_NBVALIDBITS(SizeVar);
        HASH->INDATA = *(uint32_t *)inputaddr;

        /* Start the Digest calculation */
        hhash->pHashOutBuffPtr = pOutBuffer;     /* Points at the computed digest */
        __DAL_HASH_START_DIGEST();
        /* Process Unlock */
        __DAL_UNLOCK(hhash);

        /* Enable Interrupts */
        __DAL_HASH_ENABLE_IT(HASH_IT_DCI);

        /* Return function status */
        return DAL_OK;
      }
    } /*  if (polling_step == 1) */


    /* Process Unlock */
    __DAL_UNLOCK(hhash);

    /* Enable Interrupts */
    __DAL_HASH_ENABLE_IT(HASH_IT_DINI | HASH_IT_DCI);

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }

}


/**
  * @brief  Initialize the HASH peripheral then initiate a DMA transfer
  *         to feed the input buffer to the Peripheral.
  * @note   If MDMAT bit is set before calling this function (multi-buffer
  *          HASH processing case), the input buffer size (in bytes) must be
  *          a multiple of 4 otherwise, the HASH digest computation is corrupted.
  *          For the processing of the last buffer of the thread, MDMAT bit must
  *          be reset and the buffer length (in bytes) doesn't have to be a
  *          multiple of 4.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  Algorithm HASH algorithm.
  * @retval DAL status
  */
DAL_StatusTypeDef HASH_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint32_t Algorithm)
{
  uint32_t inputaddr;
  uint32_t inputSize;
  DAL_StatusTypeDef status ;
  DAL_HASH_StateTypeDef State_tmp = hhash->State;

#if defined (HASH_CTRL_MDMAT)
  /* Make sure the input buffer size (in bytes) is a multiple of 4 when MDMAT bit is set
     (case of multi-buffer HASH processing) */
  ASSERT_PARAM(IS_HASH_DMA_MULTIBUFFER_SIZE(Size));
#endif /* MDMA defined*/
  /* If State is ready or suspended, start or resume polling-based HASH processing */
  if ((State_tmp == DAL_HASH_STATE_READY) || (State_tmp == DAL_HASH_STATE_SUSPENDED))
  {
    /* Check input parameters */
    if ((pInBuffer == NULL) || (Size == 0U) ||
        /* Check phase coherency. Phase must be
           either READY (fresh start)
           or PROCESS (multi-buffer HASH management) */
        ((hhash->Phase != DAL_HASH_PHASE_READY) && (!(IS_HASH_PROCESSING(hhash)))))
    {
      hhash->State = DAL_HASH_STATE_READY;
      return  DAL_ERROR;
    }


    /* Process Locked */
    __DAL_LOCK(hhash);

    /* If not a resumption case */
    if (hhash->State == DAL_HASH_STATE_READY)
    {
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;

      /* Check if initialization phase has already been performed.
         If Phase is already set to DAL_HASH_PHASE_PROCESS, this means the
         API is processing a new input data message in case of multi-buffer HASH
         computation. */
      if (hhash->Phase == DAL_HASH_PHASE_READY)
      {
        /* Select the HASH algorithm, clear HMAC mode and long key selection bit, reset the HASH processor core */
        MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL, Algorithm | HASH_CTRL_INITCAL);

        /* Set the phase */
        hhash->Phase = DAL_HASH_PHASE_PROCESS;
      }

      /* Configure the Number of valid bits in last word of the message */
      __DAL_HASH_SET_NBVALIDBITS(Size);

      inputaddr = (uint32_t)pInBuffer;     /* DMA transfer start address   */
      inputSize = Size;                    /* DMA transfer size (in bytes) */

      /* In case of suspension request, save the starting parameters */
      hhash->pHashInBuffPtr =  pInBuffer;  /* DMA transfer start address   */
      hhash->HashInCount = Size;           /* DMA transfer size (in bytes) */

    }
    /* If resumption case */
    else
    {
      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;

      /* Resumption case, inputaddr and inputSize are not set to the API input parameters
         but to those saved beforehand by DAL_HASH_DMAFeed_ProcessSuspend() when the
         processing was suspended */
      inputaddr = (uint32_t)hhash->pHashInBuffPtr;  /* DMA transfer start address   */
      inputSize = hhash->HashInCount;               /* DMA transfer size (in bytes) */

    }

    /* Set the HASH DMA transfer complete callback */
    hhash->hdmain->XferCpltCallback = HASH_DMAXferCplt;
    /* Set the DMA error callback */
    hhash->hdmain->XferErrorCallback = HASH_DMAError;

    /* Store number of words already pushed to manage proper DMA processing suspension */
    hhash->NbWordsAlreadyPushed = HASH_NBW_PUSHED();

    /* Enable the DMA In DMA stream */
    status = DAL_DMA_Start_IT(hhash->hdmain, inputaddr, (uint32_t)&HASH->INDATA, \
                              (((inputSize % 4U) != 0U) ? ((inputSize + (4U - (inputSize % 4U))) / 4U) : \
                               (inputSize / 4U)));

    /* Enable DMA requests */
    SET_BIT(HASH->CTRL, HASH_CTRL_DMAEN);

    /* Process Unlock */
    __DAL_UNLOCK(hhash);

    /* Return function status */
    if (status != DAL_OK)
    {
      /* Update HASH state machine to error */
      hhash->State = DAL_HASH_STATE_ERROR;
    }

    return status;
  }
  else
  {
    return DAL_BUSY;
  }
}

/**
  * @brief  Return the computed digest.
  * @note   The API waits for DCIS to be set then reads the computed digest.
  * @param  hhash HASH handle.
  * @param  pOutBuffer pointer to the computed digest.
  * @param  Timeout Timeout value.
  * @retval DAL status
  */
DAL_StatusTypeDef HASH_Finish(HASH_HandleTypeDef *hhash, uint8_t *pOutBuffer, uint32_t Timeout)
{

  if (hhash->State == DAL_HASH_STATE_READY)
  {
    /* Check parameter */
    if (pOutBuffer == NULL)
    {
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hhash);

    /* Change the HASH state to busy */
    hhash->State = DAL_HASH_STATE_BUSY;

    /* Wait for DCIS flag to be set */
    if (HASH_WaitOnFlagUntilTimeout(hhash, HASH_FLAG_DCIS, RESET, Timeout) != DAL_OK)
    {
      return DAL_TIMEOUT;
    }

    /* Read the message digest */
    HASH_GetDigest(pOutBuffer, HASH_DIGEST_LENGTH());

    /* Change the HASH state to ready */
    hhash->State = DAL_HASH_STATE_READY;

    /* Reset HASH state machine */
    hhash->Phase = DAL_HASH_PHASE_READY;

    /* Process UnLock */
    __DAL_UNLOCK(hhash);

    /* Return function status */
    return DAL_OK;

  }
  else
  {
    return DAL_BUSY;
  }

}


/**
  * @brief  Initialize the HASH peripheral in HMAC mode, next process pInBuffer then
  *         read the computed digest.
  * @note   Digest is available in pOutBuffer.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest.
  * @param  Timeout Timeout value.
  * @param  Algorithm HASH algorithm.
  * @retval DAL status
  */
DAL_StatusTypeDef HMAC_Start(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t *pOutBuffer,
                             uint32_t Timeout, uint32_t Algorithm)
{
  DAL_HASH_StateTypeDef State_tmp = hhash->State;

  /* If State is ready or suspended, start or resume polling-based HASH processing */
  if ((State_tmp == DAL_HASH_STATE_READY) || (State_tmp == DAL_HASH_STATE_SUSPENDED))
  {
    /* Check input parameters */
    if ((pInBuffer == NULL) || (Size == 0U) || (hhash->Init.pKey == NULL) || (hhash->Init.KeySize == 0U)
        || (pOutBuffer == NULL))
    {
      hhash->State = DAL_HASH_STATE_READY;
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hhash);

    /* Change the HASH state */
    hhash->State = DAL_HASH_STATE_BUSY;

    /* Check if initialization phase has already be performed */
    if (hhash->Phase == DAL_HASH_PHASE_READY)
    {
      /* Check if key size is larger than 64 bytes, accordingly set LKEY and the other setting bits */
      if (hhash->Init.KeySize > 64U)
      {
        MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL,
                   Algorithm | HASH_ALGOMODE_HMAC | HASH_HMAC_KEYTYPE_LONGKEY | HASH_CTRL_INITCAL);
      }
      else
      {
        MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL,
                   Algorithm | HASH_ALGOMODE_HMAC | HASH_CTRL_INITCAL);
      }
      /* Set the phase to Step 1 */
      hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_1;
      /* Resort to hhash internal fields to feed the Peripheral.
         Parameters will be updated in case of suspension to contain the proper
         information at resumption time. */
      hhash->pHashOutBuffPtr  = pOutBuffer;            /* Output digest address    */
      hhash->pHashInBuffPtr   = pInBuffer;             /* Input data address, HMAC_Processing input
                                                          parameter for Step 2     */
      hhash->HashInCount      = Size;                  /* Input data size, HMAC_Processing input
                                                          parameter for Step 2        */
      hhash->HashBuffSize     = Size;                  /* Store the input buffer size for the whole HMAC process*/
      hhash->pHashKeyBuffPtr  = hhash->Init.pKey;      /* Key address, HMAC_Processing input parameter for Step
                                                          1 and Step 3 */
      hhash->HashKeyCount     = hhash->Init.KeySize;   /* Key size, HMAC_Processing input parameter for Step 1
                                                          and Step 3    */
    }

    /* Carry out HMAC processing */
    return HMAC_Processing(hhash, Timeout);

  }
  else
  {
    return DAL_BUSY;
  }
}



/**
  * @brief  Initialize the HASH peripheral in HMAC mode, next process pInBuffer then
  *         read the computed digest in interruption mode.
  * @note   Digest is available in pOutBuffer.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  pOutBuffer pointer to the computed digest.
  * @param  Algorithm HASH algorithm.
  * @retval DAL status
  */
DAL_StatusTypeDef HMAC_Start_IT(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint8_t *pOutBuffer,
                                uint32_t Algorithm)
{
  DAL_HASH_StateTypeDef State_tmp = hhash->State;

  /* If State is ready or suspended, start or resume IT-based HASH processing */
  if ((State_tmp == DAL_HASH_STATE_READY) || (State_tmp == DAL_HASH_STATE_SUSPENDED))
  {
    /* Check input parameters */
    if ((pInBuffer == NULL) || (Size == 0U) || (hhash->Init.pKey == NULL) || (hhash->Init.KeySize == 0U)
        || (pOutBuffer == NULL))
    {
      hhash->State = DAL_HASH_STATE_READY;
      return  DAL_ERROR;
    }

    /* Process Locked */
    __DAL_LOCK(hhash);

    /* Change the HASH state */
    hhash->State = DAL_HASH_STATE_BUSY;

    /* Initialize IT counter */
    hhash->HashITCounter = 1;

    /* Check if initialization phase has already be performed */
    if (hhash->Phase == DAL_HASH_PHASE_READY)
    {
      /* Check if key size is larger than 64 bytes, accordingly set LKEY and the other setting bits */
      if (hhash->Init.KeySize > 64U)
      {
        MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL,
                   Algorithm | HASH_ALGOMODE_HMAC | HASH_HMAC_KEYTYPE_LONGKEY | HASH_CTRL_INITCAL);
      }
      else
      {
        MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL,
                   Algorithm | HASH_ALGOMODE_HMAC | HASH_CTRL_INITCAL);
      }

      /* Resort to hhash internal fields hhash->pHashInBuffPtr and hhash->HashInCount
         to feed the Peripheral whatever the HMAC step.
         Lines below are set to start HMAC Step 1 processing where key is entered first. */
      hhash->HashInCount     = hhash->Init.KeySize; /* Key size                      */
      hhash->pHashInBuffPtr  = hhash->Init.pKey ;   /* Key address                   */

      /* Store input and output parameters in handle fields to manage steps transition
         or possible HMAC suspension/resumption */
      hhash->pHashKeyBuffPtr = hhash->Init.pKey;    /* Key address                   */
      hhash->pHashMsgBuffPtr = pInBuffer;           /* Input message address         */
      hhash->HashBuffSize    = Size;                /* Input message size (in bytes) */
      hhash->pHashOutBuffPtr = pOutBuffer;          /* Output digest address         */

      /* Configure the number of valid bits in last word of the key */
      __DAL_HASH_SET_NBVALIDBITS(hhash->Init.KeySize);

      /* Set the phase to Step 1 */
      hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_1;
    }
    else if ((hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_1) || (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_3))
    {
      /* Restart IT-based HASH processing after Step 1 or Step 3 suspension */

    }
    else if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_2)
    {
      /* Restart IT-based HASH processing after Step 2 suspension */

    }
    else
    {
      /* Error report as phase incorrect */
      /* Process Unlock */
      __DAL_UNLOCK(hhash);
      hhash->State = DAL_HASH_STATE_READY;
      return DAL_ERROR;
    }

    /* Process Unlock */
    __DAL_UNLOCK(hhash);

    /* Enable Interrupts */
    __DAL_HASH_ENABLE_IT(HASH_IT_DINI | HASH_IT_DCI);

    /* Return function status */
    return DAL_OK;
  }
  else
  {
    return DAL_BUSY;
  }

}



/**
  * @brief  Initialize the HASH peripheral in HMAC mode then initiate the required
  *         DMA transfers to feed the key and the input buffer to the Peripheral.
  * @note   Same key is used for the inner and the outer hash functions; pointer to key and
  *         key size are respectively stored in hhash->Init.pKey and hhash->Init.KeySize.
  * @note   In case of multi-buffer HMAC processing, the input buffer size (in bytes) must
  *         be a multiple of 4 otherwise, the HASH digest computation is corrupted.
  *         Only the length of the last buffer of the thread doesn't have to be a
  *         multiple of 4.
  * @param  hhash HASH handle.
  * @param  pInBuffer pointer to the input buffer (buffer to be hashed).
  * @param  Size length of the input buffer in bytes.
  * @param  Algorithm HASH algorithm.
  * @retval DAL status
  */
DAL_StatusTypeDef HMAC_Start_DMA(HASH_HandleTypeDef *hhash, uint8_t *pInBuffer, uint32_t Size, uint32_t Algorithm)
{
  uint32_t inputaddr;
  uint32_t inputSize;
  DAL_StatusTypeDef status ;
  DAL_HASH_StateTypeDef State_tmp = hhash->State;
  /* Make sure the input buffer size (in bytes) is a multiple of 4 when digest calculation
     is disabled (multi-buffer HMAC processing, MDMAT bit to be set) */
  ASSERT_PARAM(IS_HMAC_DMA_MULTIBUFFER_SIZE(hhash, Size));
  /* If State is ready or suspended, start or resume DMA-based HASH processing */
  if ((State_tmp == DAL_HASH_STATE_READY) || (State_tmp == DAL_HASH_STATE_SUSPENDED))
  {
    /* Check input parameters */
    if ((pInBuffer == NULL) || (Size == 0U) || (hhash->Init.pKey == NULL) || (hhash->Init.KeySize == 0U) ||
        /* Check phase coherency. Phase must be
            either READY (fresh start)
            or one of HMAC PROCESS steps (multi-buffer HASH management) */
        ((hhash->Phase != DAL_HASH_PHASE_READY) && (!(IS_HMAC_PROCESSING(hhash)))))
    {
      hhash->State = DAL_HASH_STATE_READY;
      return  DAL_ERROR;
    }


    /* Process Locked */
    __DAL_LOCK(hhash);

    /* If not a case of resumption after suspension */
    if (hhash->State == DAL_HASH_STATE_READY)
    {
      /* Check whether or not initialization phase has already be performed */
      if (hhash->Phase == DAL_HASH_PHASE_READY)
      {
        /* Change the HASH state */
        hhash->State = DAL_HASH_STATE_BUSY;
#if defined(HASH_CTRL_MDMAT)
        /* Check if key size is larger than 64 bytes, accordingly set LKEY and the other setting bits.
           At the same time, ensure MDMAT bit is cleared. */
        if (hhash->Init.KeySize > 64U)
        {
          MODIFY_REG(HASH->CTRL, HASH_CTRL_MDMAT | HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL,
                     Algorithm | HASH_ALGOMODE_HMAC | HASH_HMAC_KEYTYPE_LONGKEY | HASH_CTRL_INITCAL);
        }
        else
        {
          MODIFY_REG(HASH->CTRL, HASH_CTRL_MDMAT | HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL,
                     Algorithm | HASH_ALGOMODE_HMAC | HASH_CTRL_INITCAL);
        }
#else
        /* Check if key size is larger than 64 bytes, accordingly set LKEY and the other setting bits */
        if (hhash->Init.KeySize > 64U)
        {
          MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL,
                     Algorithm | HASH_ALGOMODE_HMAC | HASH_HMAC_KEYTYPE_LONGKEY | HASH_CTRL_INITCAL);
        }
        else
        {
          MODIFY_REG(HASH->CTRL, HASH_CTRL_LKEYSEL | HASH_CTRL_ALGSEL | HASH_CTRL_MODESEL | HASH_CTRL_INITCAL,
                     Algorithm | HASH_ALGOMODE_HMAC | HASH_CTRL_INITCAL);
        }
#endif /* HASH_CTRL_MDMAT*/
        /* Store input aparameters in handle fields to manage steps transition
           or possible HMAC suspension/resumption */
        hhash->HashInCount = hhash->Init.KeySize;   /* Initial size for first DMA transfer (key size)      */
        hhash->pHashKeyBuffPtr = hhash->Init.pKey;  /* Key address                                         */
        hhash->pHashInBuffPtr  = hhash->Init.pKey ; /* First address passed to DMA (key address at Step 1) */
        hhash->pHashMsgBuffPtr = pInBuffer;         /* Input data address                                  */
        hhash->HashBuffSize = Size;                 /* input data size (in bytes)                          */

        /* Set DMA input parameters */
        inputaddr = (uint32_t)(hhash->Init.pKey);   /* Address passed to DMA (start by entering Key message) */
        inputSize = hhash->Init.KeySize;            /* Size for first DMA transfer (in bytes) */

        /* Configure the number of valid bits in last word of the key */
        __DAL_HASH_SET_NBVALIDBITS(hhash->Init.KeySize);

        /* Set the phase to Step 1 */
        hhash->Phase = DAL_HASH_PHASE_HMAC_STEP_1;

      }
      else if (hhash->Phase == DAL_HASH_PHASE_HMAC_STEP_2)
      {
        /* Process a new input data message in case of multi-buffer HMAC processing
          (this is not a resumption case) */

        /* Change the HASH state */
        hhash->State = DAL_HASH_STATE_BUSY;

        /* Save input parameters to be able to manage possible suspension/resumption */
        hhash->HashInCount = Size;                /* Input message address       */
        hhash->pHashInBuffPtr = pInBuffer;        /* Input message size in bytes */

        /* Set DMA input parameters */
        inputaddr = (uint32_t)pInBuffer;           /* Input message address       */
        inputSize = Size;                          /* Input message size in bytes */

        if (hhash->DigestCalculationDisable == RESET)
        {
          /* This means this is the last buffer of the multi-buffer sequence: DCAL needs to be set. */
#if defined(HASH_CTRL_MDMAT)
          __DAL_HASH_RESET_MDMAT();
#endif  /* HASH_CTRL_MDMAT*/
          __DAL_HASH_SET_NBVALIDBITS(inputSize);
        }
      }
      else
      {
        /* Phase not aligned with handle READY state */
        __DAL_UNLOCK(hhash);
        /* Return function status */
        return DAL_ERROR;
      }
    }
    else
    {
      /* Resumption case (phase may be Step 1, 2 or 3) */

      /* Change the HASH state */
      hhash->State = DAL_HASH_STATE_BUSY;

      /* Set DMA input parameters at resumption location;
         inputaddr and inputSize are not set to the API input parameters
         but to those saved beforehand by DAL_HASH_DMAFeed_ProcessSuspend() when the
         processing was suspended. */
      inputaddr = (uint32_t)(hhash->pHashInBuffPtr);  /* Input message address       */
      inputSize = hhash->HashInCount;                 /* Input message size in bytes */
    }


    /* Set the HASH DMA transfer complete callback */
    hhash->hdmain->XferCpltCallback = HASH_DMAXferCplt;
    /* Set the DMA error callback */
    hhash->hdmain->XferErrorCallback = HASH_DMAError;

    /* Store number of words already pushed to manage proper DMA processing suspension */
    hhash->NbWordsAlreadyPushed = HASH_NBW_PUSHED();

    /* Enable the DMA In DMA stream */
    status = DAL_DMA_Start_IT(hhash->hdmain, inputaddr, (uint32_t)&HASH->INDATA,  \
                              (((inputSize % 4U) != 0U) ? ((inputSize + (4U - (inputSize % 4U))) / 4U) \
                              : (inputSize / 4U)));

    /* Enable DMA requests */
    SET_BIT(HASH->CTRL, HASH_CTRL_DMAEN);

    /* Process Unlocked */
    __DAL_UNLOCK(hhash);

    /* Return function status */
    if (status != DAL_OK)
    {
      /* Update HASH state machine to error */
      hhash->State = DAL_HASH_STATE_ERROR;
    }

    /* Return function status */
    return status;
  }
  else
  {
    return DAL_BUSY;
  }
}
/**
  * @}
  */

#endif /* DAL_HASH_MODULE_ENABLED */

/**
  * @}
  */
#endif /*  HASH*/
/**
  * @}
  */

