/**
  *
  * @file    apm32f4xx_dal_pcd.c
  * @brief   PCD DAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
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
      The PCD DAL driver can be used as follows:

     (#) Declare a PCD_HandleTypeDef handle structure, for example:
         PCD_HandleTypeDef  hpcd;

     (#) Fill parameters of Init structure in HCD handle

     (#) Call DAL_PCD_Init() API to initialize the PCD peripheral (Core, Device core, ...)

     (#) Initialize the PCD low level resources through the DAL_PCD_MspInit() API:
         (##) Enable the PCD/USB Low Level interface clock using
              (+++) __DAL_RCM_USB_OTG_FS_CLK_ENABLE();
              (+++) __DAL_RCM_USB_OTG_HS_CLK_ENABLE(); (For High Speed Mode)

         (##) Initialize the related GPIO clocks
         (##) Configure PCD pin-out
         (##) Configure PCD NVIC interrupt

     (#)Associate the Upper USB device stack to the DAL PCD Driver:
         (##) hpcd.pData = pdev;

     (#)Enable PCD transmission and reception:
         (##) DAL_PCD_Start();

  @endverbatim
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "apm32f4xx_dal.h"

/** @addtogroup APM32F4xx_DAL_Driver
  * @{
  */

/** @defgroup PCD PCD
  * @brief PCD DAL module driver
  * @{
  */

#ifdef DAL_PCD_MODULE_ENABLED

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup PCD_Private_Macros PCD Private Macros
  * @{
  */
#define PCD_MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define PCD_MAX(a, b)  (((a) > (b)) ? (a) : (b))
/**
  * @}
  */

/* Private functions prototypes ----------------------------------------------*/
/** @defgroup PCD_Private_Functions PCD Private Functions
  * @{
  */
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
static DAL_StatusTypeDef PCD_WriteEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum);
static DAL_StatusTypeDef PCD_EP_OutXfrComplete_int(PCD_HandleTypeDef *hpcd, uint32_t epnum);
static DAL_StatusTypeDef PCD_EP_OutSetupPacket_int(PCD_HandleTypeDef *hpcd, uint32_t epnum);
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */

/** @defgroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:

@endverbatim
  * @{
  */

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and initialize the associated handle.
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_Init(PCD_HandleTypeDef *hpcd)
{
  USB_OTG_GlobalTypeDef *USBx;
  uint8_t i;

  /* Check the PCD handle allocation */
  if (hpcd == NULL)
  {
    return DAL_ERROR;
  }

  /* Check the parameters */
  ASSERT_PARAM(IS_PCD_ALL_INSTANCE(hpcd->Instance));

  USBx = hpcd->Instance;

  if (hpcd->State == DAL_PCD_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hpcd->Lock = DAL_UNLOCKED;

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SOFCallback = DAL_PCD_SOFCallback;
    hpcd->SetupStageCallback = DAL_PCD_SetupStageCallback;
    hpcd->ResetCallback = DAL_PCD_ResetCallback;
    hpcd->SuspendCallback = DAL_PCD_SuspendCallback;
    hpcd->ResumeCallback = DAL_PCD_ResumeCallback;
    hpcd->ConnectCallback = DAL_PCD_ConnectCallback;
    hpcd->DisconnectCallback = DAL_PCD_DisconnectCallback;
    hpcd->DataOutStageCallback = DAL_PCD_DataOutStageCallback;
    hpcd->DataInStageCallback = DAL_PCD_DataInStageCallback;
    hpcd->ISOOUTIncompleteCallback = DAL_PCD_ISOOUTIncompleteCallback;
    hpcd->ISOINIncompleteCallback = DAL_PCD_ISOINIncompleteCallback;
    hpcd->LPMCallback = DAL_PCDEx_LPM_Callback;
    hpcd->BCDCallback = DAL_PCDEx_BCD_Callback;

    if (hpcd->MspInitCallback == NULL)
    {
      hpcd->MspInitCallback = DAL_PCD_MspInit;
    }

    /* Init the low level hardware */
    hpcd->MspInitCallback(hpcd);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    DAL_PCD_MspInit(hpcd);
#endif /* (USE_DAL_PCD_REGISTER_CALLBACKS) */
  }

  hpcd->State = DAL_PCD_STATE_BUSY;

  /* Disable DMA mode for FS instance */
  if ((USBx->GCID & (0x1U << 8)) == 0U)
  {
    hpcd->Init.dma_enable = 0U;
  }

  /* Disable the Interrupts */
  __DAL_PCD_DISABLE(hpcd);

  /*Init the Core (common init.) */
  if (USB_CoreInit(hpcd->Instance, hpcd->Init) != DAL_OK)
  {
    hpcd->State = DAL_PCD_STATE_ERROR;
    return DAL_ERROR;
  }

  /* Force Device Mode*/
  (void)USB_SetCurrentMode(hpcd->Instance, USB_DEVICE_MODE);

  /* Init endpoints structures */
  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    /* Init ep structure */
    hpcd->IN_ep[i].is_in = 1U;
    hpcd->IN_ep[i].num = i;
    hpcd->IN_ep[i].tx_fifo_num = i;
    /* Control until ep is activated */
    hpcd->IN_ep[i].type = EP_TYPE_CTRL;
    hpcd->IN_ep[i].maxpacket = 0U;
    hpcd->IN_ep[i].xfer_buff = 0U;
    hpcd->IN_ep[i].xfer_len = 0U;
  }

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    hpcd->OUT_ep[i].is_in = 0U;
    hpcd->OUT_ep[i].num = i;
    /* Control until ep is activated */
    hpcd->OUT_ep[i].type = EP_TYPE_CTRL;
    hpcd->OUT_ep[i].maxpacket = 0U;
    hpcd->OUT_ep[i].xfer_buff = 0U;
    hpcd->OUT_ep[i].xfer_len = 0U;
  }

  /* Init Device */
  if (USB_DevInit(hpcd->Instance, hpcd->Init) != DAL_OK)
  {
    hpcd->State = DAL_PCD_STATE_ERROR;
    return DAL_ERROR;
  }

  hpcd->USB_Address = 0U;
  hpcd->State = DAL_PCD_STATE_READY;

  (void)USB_DevDisconnect(hpcd->Instance);

  return DAL_OK;
}

/**
  * @brief  DeInitializes the PCD peripheral.
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_DeInit(PCD_HandleTypeDef *hpcd)
{
  /* Check the PCD handle allocation */
  if (hpcd == NULL)
  {
    return DAL_ERROR;
  }

  hpcd->State = DAL_PCD_STATE_BUSY;

  /* Stop Device */
  if (USB_StopDevice(hpcd->Instance) != DAL_OK)
  {
    return DAL_ERROR;
  }

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
  if (hpcd->MspDeInitCallback == NULL)
  {
    hpcd->MspDeInitCallback = DAL_PCD_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware */
  hpcd->MspDeInitCallback(hpcd);
#else
  /* DeInit the low level hardware: CLOCK, NVIC.*/
  DAL_PCD_MspDeInit(hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */

  hpcd->State = DAL_PCD_STATE_RESET;

  return DAL_OK;
}

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_MspDeInit could be implemented in the user file
   */
}

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User USB PCD Callback
  *         To be used instead of the weak predefined callback
  * @param  hpcd USB PCD handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_PCD_SOF_CB_ID USB PCD SOF callback ID
  *          @arg @ref DAL_PCD_SETUPSTAGE_CB_ID USB PCD Setup callback ID
  *          @arg @ref DAL_PCD_RESET_CB_ID USB PCD Reset callback ID
  *          @arg @ref DAL_PCD_SUSPEND_CB_ID USB PCD Suspend callback ID
  *          @arg @ref DAL_PCD_RESUME_CB_ID USB PCD Resume callback ID
  *          @arg @ref DAL_PCD_CONNECT_CB_ID USB PCD Connect callback ID
  *          @arg @ref DAL_PCD_DISCONNECT_CB_ID OTG PCD Disconnect callback ID
  *          @arg @ref DAL_PCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref DAL_PCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_RegisterCallback(PCD_HandleTypeDef *hpcd,
                                           DAL_PCD_CallbackIDTypeDef CallbackID,
                                           pPCD_CallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;
    return DAL_ERROR;
  }
  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_PCD_SOF_CB_ID :
        hpcd->SOFCallback = pCallback;
        break;

      case DAL_PCD_SETUPSTAGE_CB_ID :
        hpcd->SetupStageCallback = pCallback;
        break;

      case DAL_PCD_RESET_CB_ID :
        hpcd->ResetCallback = pCallback;
        break;

      case DAL_PCD_SUSPEND_CB_ID :
        hpcd->SuspendCallback = pCallback;
        break;

      case DAL_PCD_RESUME_CB_ID :
        hpcd->ResumeCallback = pCallback;
        break;

      case DAL_PCD_CONNECT_CB_ID :
        hpcd->ConnectCallback = pCallback;
        break;

      case DAL_PCD_DISCONNECT_CB_ID :
        hpcd->DisconnectCallback = pCallback;
        break;

      case DAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = pCallback;
        break;

      case DAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hpcd->State == DAL_PCD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = pCallback;
        break;

      case DAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;
    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);
  return status;
}

/**
  * @brief  Unregister an USB PCD Callback
  *         USB PCD callback is redirected to the weak predefined callback
  * @param  hpcd USB PCD handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref DAL_PCD_SOF_CB_ID USB PCD SOF callback ID
  *          @arg @ref DAL_PCD_SETUPSTAGE_CB_ID USB PCD Setup callback ID
  *          @arg @ref DAL_PCD_RESET_CB_ID USB PCD Reset callback ID
  *          @arg @ref DAL_PCD_SUSPEND_CB_ID USB PCD Suspend callback ID
  *          @arg @ref DAL_PCD_RESUME_CB_ID USB PCD Resume callback ID
  *          @arg @ref DAL_PCD_CONNECT_CB_ID USB PCD Connect callback ID
  *          @arg @ref DAL_PCD_DISCONNECT_CB_ID OTG PCD Disconnect callback ID
  *          @arg @ref DAL_PCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref DAL_PCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_UnRegisterCallback(PCD_HandleTypeDef *hpcd, DAL_PCD_CallbackIDTypeDef CallbackID)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hpcd);

  /* Setup Legacy weak Callbacks  */
  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    switch (CallbackID)
    {
      case DAL_PCD_SOF_CB_ID :
        hpcd->SOFCallback = DAL_PCD_SOFCallback;
        break;

      case DAL_PCD_SETUPSTAGE_CB_ID :
        hpcd->SetupStageCallback = DAL_PCD_SetupStageCallback;
        break;

      case DAL_PCD_RESET_CB_ID :
        hpcd->ResetCallback = DAL_PCD_ResetCallback;
        break;

      case DAL_PCD_SUSPEND_CB_ID :
        hpcd->SuspendCallback = DAL_PCD_SuspendCallback;
        break;

      case DAL_PCD_RESUME_CB_ID :
        hpcd->ResumeCallback = DAL_PCD_ResumeCallback;
        break;

      case DAL_PCD_CONNECT_CB_ID :
        hpcd->ConnectCallback = DAL_PCD_ConnectCallback;
        break;

      case DAL_PCD_DISCONNECT_CB_ID :
        hpcd->DisconnectCallback = DAL_PCD_DisconnectCallback;
        break;

      case DAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = DAL_PCD_MspInit;
        break;

      case DAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = DAL_PCD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else if (hpcd->State == DAL_PCD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case DAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = DAL_PCD_MspInit;
        break;

      case DAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = DAL_PCD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  DAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);
  return status;
}

/**
  * @brief  Register USB PCD Data OUT Stage Callback
  *         To be used instead of the weak DAL_PCD_DataOutStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Data OUT Stage Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_RegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd,
                                                       pPCD_DataOutStageCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->DataOutStageCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Data OUT Stage Callback
  *         USB PCD Data OUT Stage Callback is redirected to the weak DAL_PCD_DataOutStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_UnRegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->DataOutStageCallback = DAL_PCD_DataOutStageCallback; /* Legacy weak DataOutStageCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Data IN Stage Callback
  *         To be used instead of the weak DAL_PCD_DataInStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Data IN Stage Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_RegisterDataInStageCallback(PCD_HandleTypeDef *hpcd,
                                                      pPCD_DataInStageCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->DataInStageCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Data IN Stage Callback
  *         USB PCD Data OUT Stage Callback is redirected to the weak DAL_PCD_DataInStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_UnRegisterDataInStageCallback(PCD_HandleTypeDef *hpcd)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->DataInStageCallback = DAL_PCD_DataInStageCallback; /* Legacy weak DataInStageCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Iso OUT incomplete Callback
  *         To be used instead of the weak DAL_PCD_ISOOUTIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Iso OUT incomplete Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_RegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd,
                                                       pPCD_IsoOutIncpltCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->ISOOUTIncompleteCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Iso OUT incomplete Callback
  *         USB PCD Iso OUT incomplete Callback is redirected
  *         to the weak DAL_PCD_ISOOUTIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_UnRegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->ISOOUTIncompleteCallback = DAL_PCD_ISOOUTIncompleteCallback; /* Legacy weak ISOOUTIncompleteCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Iso IN incomplete Callback
  *         To be used instead of the weak DAL_PCD_ISOINIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Iso IN incomplete Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_RegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd,
                                                      pPCD_IsoInIncpltCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->ISOINIncompleteCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Iso IN incomplete Callback
  *         USB PCD Iso IN incomplete Callback is redirected
  *         to the weak DAL_PCD_ISOINIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_UnRegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->ISOINIncompleteCallback = DAL_PCD_ISOINIncompleteCallback; /* Legacy weak ISOINIncompleteCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD BCD Callback
  *         To be used instead of the weak DAL_PCDEx_BCD_Callback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD BCD Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_RegisterBcdCallback(PCD_HandleTypeDef *hpcd, pPCD_BcdCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->BCDCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD BCD Callback
  *         USB BCD Callback is redirected to the weak DAL_PCDEx_BCD_Callback() predefined callback
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_UnRegisterBcdCallback(PCD_HandleTypeDef *hpcd)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->BCDCallback = DAL_PCDEx_BCD_Callback; /* Legacy weak DAL_PCDEx_BCD_Callback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD LPM Callback
  *         To be used instead of the weak DAL_PCDEx_LPM_Callback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD LPM Callback function
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_RegisterLpmCallback(PCD_HandleTypeDef *hpcd, pPCD_LpmCallbackTypeDef pCallback)
{
  DAL_StatusTypeDef status = DAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    return DAL_ERROR;
  }

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->LPMCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD LPM Callback
  *         USB LPM Callback is redirected to the weak DAL_PCDEx_LPM_Callback() predefined callback
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_UnRegisterLpmCallback(PCD_HandleTypeDef *hpcd)
{
  DAL_StatusTypeDef status = DAL_OK;

  /* Process locked */
  __DAL_LOCK(hpcd);

  if (hpcd->State == DAL_PCD_STATE_READY)
  {
    hpcd->LPMCallback = DAL_PCDEx_LPM_Callback; /* Legacy weak DAL_PCDEx_LPM_Callback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= DAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  DAL_ERROR;
  }

  /* Release Lock */
  __DAL_UNLOCK(hpcd);

  return status;
}
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group2 Input and Output operation functions
  *  @brief   Data transfers functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the PCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Start the USB device
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_Start(PCD_HandleTypeDef *hpcd)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;

  __DAL_LOCK(hpcd);

  if ((hpcd->Init.battery_charging_enable == 1U) &&
      (hpcd->Init.phy_itface != USB_OTG_ULPI_PHY))
  {
    /* Enable USB Transceiver */
    USBx->GGCCFG |= USB_OTG_GGCCFG_PWEN;
  }

  __DAL_PCD_ENABLE(hpcd);
  (void)USB_DevConnect(hpcd->Instance);
  __DAL_UNLOCK(hpcd);

  return DAL_OK;
}

/**
  * @brief  Stop the USB device.
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_Stop(PCD_HandleTypeDef *hpcd)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;

  __DAL_LOCK(hpcd);
  __DAL_PCD_DISABLE(hpcd);
  (void)USB_DevDisconnect(hpcd->Instance);

  (void)USB_FlushTxFifo(hpcd->Instance, 0x10U);

  if ((hpcd->Init.battery_charging_enable == 1U) &&
      (hpcd->Init.phy_itface != USB_OTG_ULPI_PHY))
  {
    /* Disable USB Transceiver */
    USBx->GGCCFG &= ~(USB_OTG_GGCCFG_PWEN);
  }

  __DAL_UNLOCK(hpcd);

  return DAL_OK;
}

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
/**
  * @brief  Handles PCD interrupt request.
  * @param  hpcd PCD handle
  * @retval DAL status
  */
void DAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  USB_OTG_EPTypeDef *ep;
  uint32_t i;
  uint32_t ep_intr;
  uint32_t epint;
  uint32_t epnum;
  uint32_t fifoemptymsk;
  uint32_t RegVal;

  /* ensure that we are in device mode */
  if (USB_GetMode(hpcd->Instance) == USB_OTG_MODE_DEVICE)
  {
    /* avoid spurious interrupt */
    if (__DAL_PCD_IS_INVALID_INTERRUPT(hpcd))
    {
      return;
    }

    /* store current frame number */
    hpcd->FrameNumber = (USBx_DEVICE->DSTS & USB_OTG_DSTS_SOFNUM_Msk) >> USB_OTG_DSTS_SOFNUM_Pos;

    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_MMIS))
    {
      /* incorrect mode, acknowledge the interrupt */
      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_MMIS);
    }

    /* Handle RxQLevel Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_RXFNONE))
    {
      USB_MASK_INTERRUPT(hpcd->Instance, USB_OTG_GCINT_RXFNONE);

      RegVal = USBx->GRXSTSP;

      ep = &hpcd->OUT_ep[RegVal & USB_OTG_GRXSTSP_EPNUM];

      if (((RegVal & USB_OTG_GRXSTSP_PSTS) >> 17) ==  STS_DATA_UPDT)
      {
        if ((RegVal & USB_OTG_GRXSTSP_BCNT) != 0U)
        {
          (void)USB_ReadPacket(USBx, ep->xfer_buff,
                               (uint16_t)((RegVal & USB_OTG_GRXSTSP_BCNT) >> 4));

          ep->xfer_buff += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;
          ep->xfer_count += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;
        }
      }
      else if (((RegVal & USB_OTG_GRXSTSP_PSTS) >> 17) == STS_SETUP_UPDT)
      {
        (void)USB_ReadPacket(USBx, (uint8_t *)hpcd->Setup, 8U);
        ep->xfer_count += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;
      }
      else
      {
        /* ... */
      }

      USB_UNMASK_INTERRUPT(hpcd->Instance, USB_OTG_GCINT_RXFNONE);
    }

    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_ONEP))
    {
      epnum = 0U;

      /* Read in the device interrupt bits */
      ep_intr = USB_ReadDevAllOutEpInterrupt(hpcd->Instance);

      while (ep_intr != 0U)
      {
        if ((ep_intr & 0x1U) != 0U)
        {
          epint = USB_ReadDevOutEPInterrupt(hpcd->Instance, (uint8_t)epnum);

          if ((epint & USB_OTG_DOEPINT_TSFCMP) == USB_OTG_DOEPINT_TSFCMP)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_TSFCMP);
            (void)PCD_EP_OutXfrComplete_int(hpcd, epnum);
          }

          if ((epint & USB_OTG_DOEPINT_SETPCMP) == USB_OTG_DOEPINT_SETPCMP)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_SETPCMP);
            /* Class B setup phase done for previous decoded setup */
            (void)PCD_EP_OutSetupPacket_int(hpcd, epnum);
          }

          if ((epint & USB_OTG_DOEPINT_RXOTDIS) == USB_OTG_DOEPINT_RXOTDIS)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_RXOTDIS);
          }

          /* Clear OUT Endpoint disable interrupt */
          if ((epint & USB_OTG_DOEPINT_EPDIS) == USB_OTG_DOEPINT_EPDIS)
          {
            if ((USBx->GCINT & USB_OTG_GCINT_GONAKE) == USB_OTG_GCINT_GONAKE)
            {
              USBx_DEVICE->DCTRL |= USB_OTG_DCTRL_GONAKCLR;
            }

            ep = &hpcd->OUT_ep[epnum];

            if (ep->is_iso_incomplete == 1U)
            {
              ep->is_iso_incomplete = 0U;

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
              hpcd->ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
#else
              DAL_PCD_ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
            }

            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_EPDIS);
          }

          /* Clear Status Phase Received interrupt */
          if ((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
          }

          /* Clear OUT NAK interrupt */
          if ((epint & USB_OTG_DOEPINT_NAK) == USB_OTG_DOEPINT_NAK)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_NAK);
          }
        }
        epnum++;
        ep_intr >>= 1U;
      }
    }

    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_INEP))
    {
      /* Read in the device interrupt bits */
      ep_intr = USB_ReadDevAllInEpInterrupt(hpcd->Instance);

      epnum = 0U;

      while (ep_intr != 0U)
      {
        if ((ep_intr & 0x1U) != 0U) /* In ITR */
        {
          epint = USB_ReadDevInEPInterrupt(hpcd->Instance, (uint8_t)epnum);

          if ((epint & USB_OTG_DIEPINT_TSFCMP) == USB_OTG_DIEPINT_TSFCMP)
          {
            fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
            USBx_DEVICE->DIEIMASK &= ~fifoemptymsk;

            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TSFCMP);

            if (hpcd->Init.dma_enable == 1U)
            {
              hpcd->IN_ep[epnum].xfer_buff += hpcd->IN_ep[epnum].maxpacket;

              /* this is ZLP, so prepare EP0 for next setup */
              if ((epnum == 0U) && (hpcd->IN_ep[epnum].xfer_len == 0U))
              {
                /* prepare to rx more setup packets */
                (void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
              }
            }

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
            hpcd->DataInStageCallback(hpcd, (uint8_t)epnum);
#else
            DAL_PCD_DataInStageCallback(hpcd, (uint8_t)epnum);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
          }
          if ((epint & USB_OTG_DIEPINT_TO) == USB_OTG_DIEPINT_TO)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TO);
          }
          if ((epint & USB_OTG_DIEPINT_ITXEMP) == USB_OTG_DIEPINT_ITXEMP)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_ITXEMP);
          }
          if ((epint & USB_OTG_DIEPINT_IEPNAKE) == USB_OTG_DIEPINT_IEPNAKE)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_IEPNAKE);
          }
          if ((epint & USB_OTG_DIEPINT_EPDIS) == USB_OTG_DIEPINT_EPDIS)
          {
            (void)USB_FlushTxFifo(USBx, epnum);

            ep = &hpcd->IN_ep[epnum];

            if (ep->is_iso_incomplete == 1U)
            {
              ep->is_iso_incomplete = 0U;

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
              hpcd->ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
#else
              DAL_PCD_ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
            }

            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_EPDIS);
          }
          if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
          {
            (void)PCD_WriteEmptyTxFifo(hpcd, epnum);
          }
        }
        epnum++;
        ep_intr >>= 1U;
      }
    }

    /* Handle Resume Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_RWAKE))
    {
      /* Clear the Remote Wake-up Signaling */
      USBx_DEVICE->DCTRL &= ~USB_OTG_DCTRL_RWKUPS;

      if (hpcd->LPM_State == LPM_L1)
      {
        hpcd->LPM_State = LPM_L0;

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->LPMCallback(hpcd, PCD_LPM_L0_ACTIVE);
#else
        DAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
      }
      else
      {
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->ResumeCallback(hpcd);
#else
        DAL_PCD_ResumeCallback(hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
      }

      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_RWAKE);
    }

    /* Handle Suspend Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_USBSUS))
    {
      if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSSTS) == USB_OTG_DSTS_SUSSTS)
      {
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->SuspendCallback(hpcd);
#else
        DAL_PCD_SuspendCallback(hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
      }
      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_USBSUS);
    }

    /* Handle Reset Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_USBRST))
    {
      USBx_DEVICE->DCTRL &= ~USB_OTG_DCTRL_RWKUPS;
      (void)USB_FlushTxFifo(hpcd->Instance, 0x10U);

      for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
      {
        USBx_INEP(i)->DIEPINT = 0xFB7FU;
        USBx_INEP(i)->DIEPCTRL &= ~USB_OTG_DIEPCTRL_STALLH;
        USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
        USBx_OUTEP(i)->DOEPCTRL &= ~USB_OTG_DOEPCTRL_STALLH;
        USBx_OUTEP(i)->DOEPCTRL |= USB_OTG_DOEPCTRL_NAKSET;
      }
      USBx_DEVICE->DAEPIMASK |= 0x10001U;

      if (hpcd->Init.use_dedicated_ep1 != 0U)
      {
        USBx_DEVICE->DOUT1MASK |= USB_OTG_DOUTIMASK_SETPCMPM |
                                   USB_OTG_DOUTIMASK_TSFCMPM |
                                   USB_OTG_DOUTIMASK_EPDISM;

        USBx_DEVICE->DEPIMASK |= USB_OTG_DINIMASK_TOM |
                                  USB_OTG_DINIMASK_TSFCMPM |
                                  USB_OTG_DINIMASK_EPDISM;
      }
      else
      {
        USBx_DEVICE->DOUTIMASK |= USB_OTG_DOUTIMASK_SETPCMPM |
                                USB_OTG_DOUTIMASK_TSFCMPM |
                                USB_OTG_DOUTIMASK_EPDISM |
                                USB_OTG_DOUTIMASK_OTEPSPRM |
                                USB_OTG_DOUTIMASK_NAKM;

        USBx_DEVICE->DINIMASK |= USB_OTG_DINIMASK_TOM |
                                USB_OTG_DINIMASK_TSFCMPM |
                                USB_OTG_DINIMASK_EPDISM;
      }

      /* Set Default Address to 0 */
      USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DADDR;

      /* setup EP0 to receive SETUP packets */
      (void)USB_EP0_OutStart(hpcd->Instance, (uint8_t)hpcd->Init.dma_enable,
                             (uint8_t *)hpcd->Setup);

      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_USBRST);
    }

    /* Handle Enumeration done Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_ENUMD))
    {
      (void)USB_ActivateSetup(hpcd->Instance);
      hpcd->Init.speed = USB_GetDevSpeed(hpcd->Instance);

      /* Set USB Turnaround time */
      (void)USB_SetTurnaroundTime(hpcd->Instance,
                                  DAL_RCM_GetHCLKFreq(),
                                  (uint8_t)hpcd->Init.speed);

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->ResetCallback(hpcd);
#else
      DAL_PCD_ResetCallback(hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */

      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_ENUMD);
    }

    /* Handle SOF Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_SOF))
    {
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->SOFCallback(hpcd);
#else
      DAL_PCD_SOFCallback(hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */

      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_SOF);
    }

    /* Handle Global OUT NAK effective Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_GONAKE))
    {
      USBx->GINTMASK &= ~USB_OTG_GINTMASK_GONAKEM;

      for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
      {
        if (hpcd->OUT_ep[epnum].is_iso_incomplete == 1U)
        {
          /* Abort current transaction and disable the EP */
          (void)DAL_PCD_EP_Abort(hpcd, (uint8_t)epnum);
        }
      }
    }

    /* Handle Incomplete ISO IN Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_IIINTX))
    {
      for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
      {
        RegVal = USBx_INEP(epnum)->DIEPCTRL;

        if ((hpcd->IN_ep[epnum].type == EP_TYPE_ISOC) &&
            ((RegVal & USB_OTG_DIEPCTRL_EPEN) == USB_OTG_DIEPCTRL_EPEN))
        {
          hpcd->IN_ep[epnum].is_iso_incomplete = 1U;

          /* Abort current transaction and disable the EP */
          (void)DAL_PCD_EP_Abort(hpcd, (uint8_t)(epnum | 0x80U));
        }
      }

      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_IIINTX);
    }

    /* Handle Incomplete ISO OUT Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_IP_OUTTX))
    {
      for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
      {
        RegVal = USBx_OUTEP(epnum)->DOEPCTRL;

        if ((hpcd->OUT_ep[epnum].type == EP_TYPE_ISOC) &&
            ((RegVal & USB_OTG_DOEPCTRL_EPEN) == USB_OTG_DOEPCTRL_EPEN) &&
            ((RegVal & (0x1U << 16)) == (hpcd->FrameNumber & 0x1U)))
        {
          hpcd->OUT_ep[epnum].is_iso_incomplete = 1U;

          USBx->GINTMASK |= USB_OTG_GINTMASK_GONAKEM;

          if ((USBx->GCINT & USB_OTG_GCINT_GONAKE) == 0U)
          {
            USBx_DEVICE->DCTRL |= USB_OTG_DCTRL_GONAKSET;
            break;
          }
        }
      }

      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_IP_OUTTX);
    }

    /* Handle Connection event Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_SREQ))
    {
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->ConnectCallback(hpcd);
#else
      DAL_PCD_ConnectCallback(hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */

      __DAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GCINT_SREQ);
    }

    /* Handle Disconnection event Interrupt */
    if (__DAL_PCD_GET_FLAG(hpcd, USB_OTG_GCINT_OTG))
    {
      RegVal = hpcd->Instance->GINT;

      if ((RegVal & USB_OTG_GINT_SEFLG) == USB_OTG_GINT_SEFLG)
      {
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DisconnectCallback(hpcd);
#else
        DAL_PCD_DisconnectCallback(hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
      }
      hpcd->Instance->GINT |= RegVal;
    }
  }
}


/**
  * @brief  Handles PCD Wakeup interrupt request.
  * @param  hpcd PCD handle
  * @retval DAL status
  */
void DAL_PCD_WKUP_IRQHandler(PCD_HandleTypeDef *hpcd)
{
  USB_OTG_GlobalTypeDef *USBx;

  USBx = hpcd->Instance;

  if ((USBx->GCID & (0x1U << 8)) == 0U)
  {
    /* Clear EINT pending Bit */
    __DAL_USB_OTG_FS_WAKEUP_EINT_CLEAR_FLAG();
  }
  else
  {
    /* Clear EINT pending Bit */
    __DAL_USB_OTG_HS_WAKEUP_EINT_CLEAR_FLAG();
  }
}
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


/**
  * @brief  Data OUT stage callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void DAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_DataOutStageCallback could be implemented in the user file
   */
}

/**
  * @brief  Data IN stage callback
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void DAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_DataInStageCallback could be implemented in the user file
   */
}
/**
  * @brief  Setup stage callback
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_SetupStageCallback could be implemented in the user file
   */
}

/**
  * @brief  USB Start Of Frame callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_SOFCallback could be implemented in the user file
   */
}

/**
  * @brief  USB Reset callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_ResetCallback could be implemented in the user file
   */
}

/**
  * @brief  Suspend event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_SuspendCallback could be implemented in the user file
   */
}

/**
  * @brief  Resume event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_ResumeCallback could be implemented in the user file
   */
}

/**
  * @brief  Incomplete ISO OUT callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void DAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_ISOOUTIncompleteCallback could be implemented in the user file
   */
}

/**
  * @brief  Incomplete ISO IN callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void DAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_ISOINIncompleteCallback could be implemented in the user file
   */
}

/**
  * @brief  Connection event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_ConnectCallback could be implemented in the user file
   */
}

/**
  * @brief  Disconnection event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void DAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the DAL_PCD_DisconnectCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   management functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the PCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Connect the USB device
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;

  __DAL_LOCK(hpcd);

  if ((hpcd->Init.battery_charging_enable == 1U) &&
      (hpcd->Init.phy_itface != USB_OTG_ULPI_PHY))
  {
    /* Enable USB Transceiver */
    USBx->GGCCFG |= USB_OTG_GGCCFG_PWEN;
  }
  (void)USB_DevConnect(hpcd->Instance);
  __DAL_UNLOCK(hpcd);

  return DAL_OK;
}

/**
  * @brief  Disconnect the USB device.
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;

  __DAL_LOCK(hpcd);
  (void)USB_DevDisconnect(hpcd->Instance);

  if ((hpcd->Init.battery_charging_enable == 1U) &&
      (hpcd->Init.phy_itface != USB_OTG_ULPI_PHY))
  {
    /* Disable USB Transceiver */
    USBx->GGCCFG &= ~(USB_OTG_GGCCFG_PWEN);
  }

  __DAL_UNLOCK(hpcd);

  return DAL_OK;
}

/**
  * @brief  Set the USB Device address.
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
  __DAL_LOCK(hpcd);
  hpcd->USB_Address = address;
  (void)USB_SetDevAddress(hpcd->Instance, address);
  __DAL_UNLOCK(hpcd);

  return DAL_OK;
}
/**
  * @brief  Open and configure an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packet size
  * @param  ep_type endpoint type
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr,
                                  uint16_t ep_mps, uint8_t ep_type)
{
  DAL_StatusTypeDef  ret = DAL_OK;
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->num = ep_addr & EP_ADDR_MSK;
  ep->maxpacket = ep_mps;
  ep->type = ep_type;

  if (ep->is_in != 0U)
  {
    /* Assign a Tx FIFO */
    ep->tx_fifo_num = ep->num;
  }
  /* Set initial data PID. */
  if (ep_type == EP_TYPE_BULK)
  {
    ep->data_pid_start = 0U;
  }

  __DAL_LOCK(hpcd);
  (void)USB_ActivateEndpoint(hpcd->Instance, ep);
  __DAL_UNLOCK(hpcd);

  return ret;
}

/**
  * @brief  Deactivate an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }
  ep->num   = ep_addr & EP_ADDR_MSK;

  __DAL_LOCK(hpcd);
  (void)USB_DeactivateEndpoint(hpcd->Instance, ep);
  __DAL_UNLOCK(hpcd);
  return DAL_OK;
}


/**
  * @brief  Receive an amount of data.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @param  len amount of data to be received
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if (hpcd->Init.dma_enable == 1U)
  {
    ep->dma_addr = (uint32_t)pBuf;
  }

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0StartXfer(hpcd->Instance, ep, (uint8_t)hpcd->Init.dma_enable);
  }
  else
  {
    (void)USB_EPStartXfer(hpcd->Instance, ep, (uint8_t)hpcd->Init.dma_enable);
  }

  return DAL_OK;
}

/**
  * @brief  Get Received Data Size
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval Data Size
  */
uint32_t DAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  return hpcd->OUT_ep[ep_addr & EP_ADDR_MSK].xfer_count;
}
/**
  * @brief  Send an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the transmission buffer
  * @param  len amount of data to be sent
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if (hpcd->Init.dma_enable == 1U)
  {
    ep->dma_addr = (uint32_t)pBuf;
  }

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0StartXfer(hpcd->Instance, ep, (uint8_t)hpcd->Init.dma_enable);
  }
  else
  {
    (void)USB_EPStartXfer(hpcd->Instance, ep, (uint8_t)hpcd->Init.dma_enable);
  }

  return DAL_OK;
}

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & EP_ADDR_MSK) > hpcd->Init.dev_endpoints)
  {
    return DAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
    ep->is_in = 0U;
  }

  ep->is_stall = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __DAL_LOCK(hpcd);

  (void)USB_EPSetStall(hpcd->Instance, ep);

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0_OutStart(hpcd->Instance, (uint8_t)hpcd->Init.dma_enable, (uint8_t *)hpcd->Setup);
  }

  __DAL_UNLOCK(hpcd);

  return DAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints)
  {
    return DAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->is_stall = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __DAL_LOCK(hpcd);
  (void)USB_EPClearStall(hpcd->Instance, ep);
  __DAL_UNLOCK(hpcd);

  return DAL_OK;
}


/**
 * @brief     USB device get EP stall status
 *
 * @param     usbdh: USB device handler
 *
 * @param     epAddr: endpoint address
 *
 * @retval    Stall status
 */
uint8_t DAL_PCD_EP_ReadStallStatus(PCD_HandleTypeDef *hpcd, uint8_t epAddr)
{
    if ((epAddr & 0x80) == 0x80)
    {
        return (hpcd->IN_ep[epAddr & 0x7F].is_stall);
    }
    else
    {
        return (hpcd->OUT_ep[epAddr & 0x7F].is_stall);
    }
}

/**
   * @brief  Abort an USB EP transaction.
   * @param  hpcd PCD handle
   * @param  ep_addr endpoint address
   * @retval DAL status
   */
DAL_StatusTypeDef DAL_PCD_EP_Abort(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  DAL_StatusTypeDef ret;
  PCD_EPTypeDef *ep;

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
  }

  /* Stop Xfer */
  ret = USB_EPStopXfer(hpcd->Instance, ep);

  return ret;
}

/**
  * @brief  Flush an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  __DAL_LOCK(hpcd);

  if ((ep_addr & 0x80U) == 0x80U)
  {
    (void)USB_FlushTxFifo(hpcd->Instance, (uint32_t)ep_addr & EP_ADDR_MSK);
  }
  else
  {
    (void)USB_FlushRxFifo(hpcd->Instance);
  }

  __DAL_UNLOCK(hpcd);

  return DAL_OK;
}

/**
  * @brief  Activate remote wakeup signalling
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  return (USB_ActivateRemoteWakeup(hpcd->Instance));
}

/**
  * @brief  De-activate remote wakeup signalling.
  * @param  hpcd PCD handle
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  return (USB_DeActivateRemoteWakeup(hpcd->Instance));
}

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group4 Peripheral State functions
  *  @brief   Peripheral State functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */

/**
  * @brief  Return the PCD handle state.
  * @param  hpcd PCD handle
  * @retval DAL state
  */
PCD_StateTypeDef DAL_PCD_GetState(PCD_HandleTypeDef *hpcd)
{
  return hpcd->State;
}

/**
  * @brief  Set the USB Device high speed test mode.
  * @param  hpcd PCD handle
  * @param  testmode USB Device high speed test mode
  * @retval DAL status
  */
DAL_StatusTypeDef DAL_PCD_SetTestMode(PCD_HandleTypeDef *hpcd, uint8_t testmode)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;

  switch (testmode)
  {
    case TEST_J:
    case TEST_K:
    case TEST_SE0_NAK:
    case TEST_PACKET:
    case TEST_FORCE_EN:
      USBx_DEVICE->DCTRL |= (uint32_t)testmode << 4;
      break;

    default:
      break;
  }

  return DAL_OK;
}
/**
  * @}
  */

/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @addtogroup PCD_Private_Functions
  * @{
  */
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
/**
  * @brief  Check FIFO for the next packet to be loaded.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval DAL status
  */
static DAL_StatusTypeDef PCD_WriteEmptyTxFifo(PCD_HandleTypeDef *hpcd, uint32_t epnum)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  USB_OTG_EPTypeDef *ep;
  uint32_t len;
  uint32_t len32b;
  uint32_t fifoemptymsk;

  ep = &hpcd->IN_ep[epnum];

  if (ep->xfer_count > ep->xfer_len)
  {
    return DAL_ERROR;
  }

  len = ep->xfer_len - ep->xfer_count;

  if (len > ep->maxpacket)
  {
    len = ep->maxpacket;
  }

  len32b = (len + 3U) / 4U;

  while (((USBx_INEP(epnum)->DITXFSTS & USB_OTG_DITXFSTS_INEPTXFSA) >= len32b) &&
         (ep->xfer_count < ep->xfer_len) && (ep->xfer_len != 0U))
  {
    /* Write the FIFO */
    len = ep->xfer_len - ep->xfer_count;

    if (len > ep->maxpacket)
    {
      len = ep->maxpacket;
    }
    len32b = (len + 3U) / 4U;

    (void)USB_WritePacket(USBx, ep->xfer_buff, (uint8_t)epnum, (uint16_t)len,
                          (uint8_t)hpcd->Init.dma_enable);

    ep->xfer_buff  += len;
    ep->xfer_count += len;
  }

  if (ep->xfer_len <= ep->xfer_count)
  {
    fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
    USBx_DEVICE->DIEIMASK &= ~fifoemptymsk;
  }

  return DAL_OK;
}


/**
  * @brief  process EP OUT transfer complete interrupt.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval DAL status
  */
static DAL_StatusTypeDef PCD_EP_OutXfrComplete_int(PCD_HandleTypeDef *hpcd, uint32_t epnum)
{
  USB_OTG_EPTypeDef *ep;
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO uint32_t *)(&USBx->GCID + 0x1U);
  uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

  if (hpcd->Init.dma_enable == 1U)
  {
    if ((DoepintReg & USB_OTG_DOEPINT_SETPCMP) == USB_OTG_DOEPINT_SETPCMP) /* Class C */
    {
      /* StupPktRcvd = 1 this is a setup packet */
      if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
          ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX))
      {
        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
      }
    }
    else if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR) /* Class E */
    {
      CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
    }
    else if ((DoepintReg & (USB_OTG_DOEPINT_SETPCMP | USB_OTG_DOEPINT_OTEPSPR)) == 0U)
    {
      /* StupPktRcvd = 1 this is a setup packet */
      if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
          ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX))
      {
        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
      }
      else
      {
        ep = &hpcd->OUT_ep[epnum];

        /* out data packet received over EP */
        ep->xfer_count = ep->xfer_size - (USBx_OUTEP(epnum)->DOEPTRS & USB_OTG_DOEPTRS_EPTRS);

        if (epnum == 0U)
        {
          if (ep->xfer_len == 0U)
          {
            /* this is ZLP, so prepare EP0 for next setup */
            (void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
          }
          else
          {
            ep->xfer_buff += ep->xfer_count;
          }
        }

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DataOutStageCallback(hpcd, (uint8_t)epnum);
#else
        DAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
      }
    }
    else
    {
      /* ... */
    }
  }
  else
  {
    if (gSNPSiD == USB_OTG_CORE_ID_310A)
    {
      /* StupPktRcvd = 1 this is a setup packet */
      if ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)
      {
        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
      }
      else
      {
        if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
        {
          CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
        }

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DataOutStageCallback(hpcd, (uint8_t)epnum);
#else
        DAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
      }
    }
    else
    {
      if ((epnum == 0U) && (hpcd->OUT_ep[epnum].xfer_len == 0U))
      {
        /* this is ZLP, so prepare EP0 for next setup */
        (void)USB_EP0_OutStart(hpcd->Instance, 0U, (uint8_t *)hpcd->Setup);
      }

#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->DataOutStageCallback(hpcd, (uint8_t)epnum);
#else
      DAL_PCD_DataOutStageCallback(hpcd, (uint8_t)epnum);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */
    }
  }

  return DAL_OK;
}


/**
  * @brief  process EP OUT setup packet received interrupt.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval DAL status
  */
static DAL_StatusTypeDef PCD_EP_OutSetupPacket_int(PCD_HandleTypeDef *hpcd, uint32_t epnum)
{
  USB_OTG_GlobalTypeDef *USBx = hpcd->Instance;
  uint32_t USBx_BASE = (uint32_t)USBx;
  uint32_t gSNPSiD = *(__IO uint32_t *)(&USBx->GCID + 0x1U);
  uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

  if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
      ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX))
  {
    CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
  }

  /* Inform the upper layer that a setup packet is available */
#if (USE_DAL_PCD_REGISTER_CALLBACKS == 1U)
  hpcd->SetupStageCallback(hpcd);
#else
  DAL_PCD_SetupStageCallback(hpcd);
#endif /* USE_DAL_PCD_REGISTER_CALLBACKS */

  if ((gSNPSiD > USB_OTG_CORE_ID_300A) && (hpcd->Init.dma_enable == 1U))
  {
    (void)USB_EP0_OutStart(hpcd->Instance, 1U, (uint8_t *)hpcd->Setup);
  }

  return DAL_OK;
}
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


/**
  * @}
  */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */
#endif /* DAL_PCD_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */
