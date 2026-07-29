
/* Includes ------------------------------------------------------------------*/
#include "um324xx_hal.h"

#ifdef HAL_PCD_MODULE_ENABLED

static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd);

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and initialize the associated handle.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd)
{
  uint8_t i;

  /* Check the PCD handle allocation */
  if (hpcd == NULL)
  {
    return HAL_ERROR;
  }

  if (hpcd->State == HAL_PCD_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hpcd->Lock = HAL_UNLOCKED;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SOFCallback = HAL_PCD_SOFCallback;
    hpcd->SetupStageCallback = HAL_PCD_SetupStageCallback;
    hpcd->ResetCallback = HAL_PCD_ResetCallback;
    hpcd->SuspendCallback = HAL_PCD_SuspendCallback;
    hpcd->ResumeCallback = HAL_PCD_ResumeCallback;
    hpcd->ConnectCallback = HAL_PCD_ConnectCallback;
    hpcd->DisconnectCallback = HAL_PCD_DisconnectCallback;
    hpcd->DataOutStageCallback = HAL_PCD_DataOutStageCallback;
    hpcd->DataInStageCallback = HAL_PCD_DataInStageCallback;
    hpcd->ISOOUTIncompleteCallback = HAL_PCD_ISOOUTIncompleteCallback;
    hpcd->ISOINIncompleteCallback = HAL_PCD_ISOINIncompleteCallback;

    if (hpcd->MspInitCallback == NULL)
    {
      hpcd->MspInitCallback = HAL_PCD_MspInit;
    }

    /* Init the low level hardware */
    hpcd->MspInitCallback(hpcd);
#else
    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    HAL_PCD_MspInit(hpcd);
#endif /* (USE_HAL_PCD_REGISTER_CALLBACKS) */
  }

  hpcd->State = HAL_PCD_STATE_BUSY;

  /* Disable the Interrupts */
  __HAL_PCD_DISABLE(hpcd);

  /* Init endpoints structures */
  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    /* Init ep structure */
    hpcd->IN_ep[i].is_in = 1U;
    hpcd->IN_ep[i].num = i;
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
  if (USB_DevInit(hpcd->Instance, hpcd->Init) != HAL_OK)
  {
    hpcd->State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  hpcd->USB_Address = 0U;
  hpcd->State = HAL_PCD_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  DeInitializes the PCD peripheral.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd)
{
  /* Check the PCD handle allocation */
  if (hpcd == NULL)
  {
    return HAL_ERROR;
  }

  hpcd->State = HAL_PCD_STATE_BUSY;

  /* Stop Device */
  if (USB_StopDevice(hpcd->Instance) != HAL_OK)
  {
    return HAL_ERROR;
  }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  if (hpcd->MspDeInitCallback == NULL)
  {
    hpcd->MspDeInitCallback = HAL_PCD_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware */
  hpcd->MspDeInitCallback(hpcd);
#else
  /* DeInit the low level hardware: CLOCK, NVIC.*/
  HAL_PCD_MspDeInit(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

  hpcd->State = HAL_PCD_STATE_RESET;

  return HAL_OK;
}

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_MspInit could be implemented in the user file
   */
}

/**
  * @brief  DeInitializes PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_MspDeInit could be implemented in the user file
   */
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User USB PCD Callback
  *         To be used instead of the weak predefined callback
  * @param  hpcd USB PCD handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_PCD_SOF_CB_ID USB PCD SOF callback ID
  *          @arg @ref HAL_PCD_SETUPSTAGE_CB_ID USB PCD Setup callback ID
  *          @arg @ref HAL_PCD_RESET_CB_ID USB PCD Reset callback ID
  *          @arg @ref HAL_PCD_SUSPEND_CB_ID USB PCD Suspend callback ID
  *          @arg @ref HAL_PCD_RESUME_CB_ID USB PCD Resume callback ID
  *          @arg @ref HAL_PCD_CONNECT_CB_ID USB PCD Connect callback ID
  *          @arg @ref HAL_PCD_DISCONNECT_CB_ID USB PCD Disconnect callback ID
  *          @arg @ref HAL_PCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref HAL_PCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterCallback(PCD_HandleTypeDef *hpcd,
                                           HAL_PCD_CallbackIDTypeDef CallbackID,
                                           pPCD_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    switch (CallbackID)
    {
      case HAL_PCD_SOF_CB_ID :
        hpcd->SOFCallback = pCallback;
        break;

      case HAL_PCD_SETUPSTAGE_CB_ID :
        hpcd->SetupStageCallback = pCallback;
        break;

      case HAL_PCD_RESET_CB_ID :
        hpcd->ResetCallback = pCallback;
        break;

      case HAL_PCD_SUSPEND_CB_ID :
        hpcd->SuspendCallback = pCallback;
        break;

      case HAL_PCD_RESUME_CB_ID :
        hpcd->ResumeCallback = pCallback;
        break;

      case HAL_PCD_CONNECT_CB_ID :
        hpcd->ConnectCallback = pCallback;
        break;

      case HAL_PCD_DISCONNECT_CB_ID :
        hpcd->DisconnectCallback = pCallback;
        break;

      case HAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = pCallback;
        break;

      case HAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (hpcd->State == HAL_PCD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case HAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = pCallback;
        break;

      case HAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = pCallback;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);
  return status;
}

/**
  * @brief  Unregister an USB PCD Callback
  *         USB PCD callback is redirected to the weak predefined callback
  * @param  hpcd USB PCD handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_PCD_SOF_CB_ID USB PCD SOF callback ID
  *          @arg @ref HAL_PCD_SETUPSTAGE_CB_ID USB PCD Setup callback ID
  *          @arg @ref HAL_PCD_RESET_CB_ID USB PCD Reset callback ID
  *          @arg @ref HAL_PCD_SUSPEND_CB_ID USB PCD Suspend callback ID
  *          @arg @ref HAL_PCD_RESUME_CB_ID USB PCD Resume callback ID
  *          @arg @ref HAL_PCD_CONNECT_CB_ID USB PCD Connect callback ID
  *          @arg @ref HAL_PCD_DISCONNECT_CB_ID USB PCD Disconnect callback ID
  *          @arg @ref HAL_PCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref HAL_PCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterCallback(PCD_HandleTypeDef *hpcd, HAL_PCD_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  /* Setup Legacy weak Callbacks  */
  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    switch (CallbackID)
    {
      case HAL_PCD_SOF_CB_ID :
        hpcd->SOFCallback = HAL_PCD_SOFCallback;
        break;

      case HAL_PCD_SETUPSTAGE_CB_ID :
        hpcd->SetupStageCallback = HAL_PCD_SetupStageCallback;
        break;

      case HAL_PCD_RESET_CB_ID :
        hpcd->ResetCallback = HAL_PCD_ResetCallback;
        break;

      case HAL_PCD_SUSPEND_CB_ID :
        hpcd->SuspendCallback = HAL_PCD_SuspendCallback;
        break;

      case HAL_PCD_RESUME_CB_ID :
        hpcd->ResumeCallback = HAL_PCD_ResumeCallback;
        break;

      case HAL_PCD_CONNECT_CB_ID :
        hpcd->ConnectCallback = HAL_PCD_ConnectCallback;
        break;

      case HAL_PCD_DISCONNECT_CB_ID :
        hpcd->DisconnectCallback = HAL_PCD_DisconnectCallback;
        break;

      case HAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = HAL_PCD_MspInit;
        break;

      case HAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = HAL_PCD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else if (hpcd->State == HAL_PCD_STATE_RESET)
  {
    switch (CallbackID)
    {
      case HAL_PCD_MSPINIT_CB_ID :
        hpcd->MspInitCallback = HAL_PCD_MspInit;
        break;

      case HAL_PCD_MSPDEINIT_CB_ID :
        hpcd->MspDeInitCallback = HAL_PCD_MspDeInit;
        break;

      default :
        /* Update the error code */
        hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

        /* Return error status */
        status =  HAL_ERROR;
        break;
    }
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);
  return status;
}

/**
  * @brief  Register USB PCD Data OUT Stage Callback
  *         To be used instead of the weak HAL_PCD_DataOutStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Data OUT Stage Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd,
                                                       pPCD_DataOutStageCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->DataOutStageCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Data OUT Stage Callback
  *         USB PCD Data OUT Stage Callback is redirected to the weak HAL_PCD_DataOutStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->DataOutStageCallback = HAL_PCD_DataOutStageCallback; /* Legacy weak DataOutStageCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Data IN Stage Callback
  *         To be used instead of the weak HAL_PCD_DataInStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Data IN Stage Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterDataInStageCallback(PCD_HandleTypeDef *hpcd,
                                                      pPCD_DataInStageCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->DataInStageCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Data IN Stage Callback
  *         USB PCD Data OUT Stage Callback is redirected to the weak HAL_PCD_DataInStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterDataInStageCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->DataInStageCallback = HAL_PCD_DataInStageCallback; /* Legacy weak DataInStageCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Iso OUT incomplete Callback
  *         To be used instead of the weak HAL_PCD_ISOOUTIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Iso OUT incomplete Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd,
                                                       pPCD_IsoOutIncpltCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->ISOOUTIncompleteCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Iso OUT incomplete Callback
  *         USB PCD Iso OUT incomplete Callback is redirected
  *         to the weak HAL_PCD_ISOOUTIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->ISOOUTIncompleteCallback = HAL_PCD_ISOOUTIncompleteCallback; /* Legacy weak ISOOUTIncompleteCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Register USB PCD Iso IN incomplete Callback
  *         To be used instead of the weak HAL_PCD_ISOINIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Iso IN incomplete Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd,
                                                      pPCD_IsoInIncpltCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->ISOINIncompleteCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

/**
  * @brief  Unregister the USB PCD Iso IN incomplete Callback
  *         USB PCD Iso IN incomplete Callback is redirected
  *         to the weak HAL_PCD_ISOINIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hpcd);

  if (hpcd->State == HAL_PCD_STATE_READY)
  {
    hpcd->ISOINIncompleteCallback = HAL_PCD_ISOINIncompleteCallback; /* Legacy weak ISOINIncompleteCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd->ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hpcd);

  return status;
}

#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

/**
  * @brief  Start the USB device
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  __HAL_PCD_ENABLE(hpcd);

  HAL_PCDEx_SetConnectionState(hpcd, 1U);

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Stop the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  __HAL_PCD_DISABLE(hpcd);

  HAL_PCDEx_SetConnectionState(hpcd, 0U);

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Data OUT stage callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  UNUSED(hpcd);
  UNUSED(epnum);
}

/**
  * @brief  Data IN stage callback
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  UNUSED(hpcd);
  UNUSED(epnum);
}
/**
  * @brief  Setup stage callback
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  UNUSED(hpcd);
}

/**
  * @brief  USB Start Of Frame callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  UNUSED(hpcd);
}

/**
  * @brief  USB Reset callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  UNUSED(hpcd);
}

/**
  * @brief  Suspend event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  UNUSED(hpcd);
}

/**
  * @brief  Resume event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  UNUSED(hpcd);
}

/**
  * @brief  Incomplete ISO OUT callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  UNUSED(hpcd);
  UNUSED(epnum);
}

/**
  * @brief  Incomplete ISO IN callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  UNUSED(hpcd);
  UNUSED(epnum);
}

/**
  * @brief  Connection event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  UNUSED(hpcd);
}

/**
  * @brief  Disconnection event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  UNUSED(hpcd);
}

/**
  * @brief  Connect the USB device
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);

  HAL_PCDEx_SetConnectionState(hpcd, 1U);

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Disconnect the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);

  HAL_PCDEx_SetConnectionState(hpcd, 0U);

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Set the USB Device address.
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
  __HAL_LOCK(hpcd);
  hpcd->USB_Address = address;
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}
/**
  * @brief  Open and configure an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packet size
  * @param  ep_type endpoint type
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr,
                                  uint16_t ep_mps, uint8_t ep_type)
{
  HAL_StatusTypeDef  ret = HAL_OK;
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

  __HAL_LOCK(hpcd);
  (void)USB_ActivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return ret;
}

/**
  * @brief  Deactivate an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
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
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_DeactivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);
  return HAL_OK;
}


/**
  * @brief  Receive an amount of data.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @param  len amount of data to be received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  (void)USB_EPStartXfer(hpcd->Instance, ep);

  return HAL_OK;
}

/**
  * @brief  Send an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the transmission buffer
  * @param  len amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  (void)USB_EPStartXfer(hpcd->Instance, ep);

  return HAL_OK;
}

/**
  * @brief  Get Received Data Size
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval Data Size
  */
uint32_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef const *hpcd, uint8_t ep_addr)
{
  return hpcd->OUT_ep[ep_addr & EP_ADDR_MSK].xfer_count;
}

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & EP_ADDR_MSK) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
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

  __HAL_LOCK(hpcd);

  (void)USB_EPSetStall(hpcd->Instance, ep);

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
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

  __HAL_LOCK(hpcd);
  (void)USB_EPClearStall(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Flush an endpoint.
  * @param  hpcd: PCD handle
  * @param  ep_addr: endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hpcd);
  UNUSED(ep_addr);
    
  return HAL_OK;
}

/**
  * @brief  Activate remote wakeup signalling
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  return (USB_ActivateRemoteWakeup(hpcd->Instance));
}

/**
  * @brief  De-activate remote wakeup signalling.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  return (USB_DeActivateRemoteWakeup(hpcd->Instance));
}

/**
  * @brief  Return the PCD handle state.
  * @param  hpcd PCD handle
  * @retval HAL state
  */
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef const *hpcd)
{
  return hpcd->State;
}

/**
  * @brief  This function handles PCD interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
{
	uint32_t wIstr = USB_ReadInterrupts(hpcd->Instance);
	
  if (wIstr & (USB_INTSTATRAW_EP0_ACK_RAW_Msk | USB_INTSTATRAW_EP1_ACK_RAW_Msk | USB_INTSTATRAW_EP2_ACK_RAW_Msk | USB_INTSTATRAW_EP3_ACK_RAW_Msk | USB_INTSTATRAW_EP4_ACK_RAW_Msk))
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the INTSTATRAW flag into the sub */
    (void)PCD_EP_ISR_Handler(hpcd);

    return;
  }
	
  if (wIstr & USB_INTSTATRAW_BUS_RESET_RAW_Msk)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_INTCLR_BUS_RESET_CLR_Msk);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->ResetCallback(hpcd);
#else
    HAL_PCD_ResetCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    (void)HAL_PCD_SetAddress(hpcd, 0U);

    return;
  }
	
  if (wIstr & USB_INTSTATRAW_SOF_RAW_Msk)
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_INTCLR_SOF_CLR_Msk);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SOFCallback(hpcd);
#else
    HAL_PCD_SOFCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    return;
  }
}

/**
  * @brief  This function handles PCD Endpoint interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd)
{
  PCD_EPTypeDef *ep;
  uint16_t count;
  uint32_t wIstr;
  uint16_t TxPctSize;
  uint8_t epindex;
	
  while (USB_ReadInterrupts(hpcd->Instance) & (USB_INTSTATRAW_EP0_ACK_RAW_Msk | USB_INTSTATRAW_EP1_ACK_RAW_Msk | USB_INTSTATRAW_EP2_ACK_RAW_Msk | USB_INTSTATRAW_EP3_ACK_RAW_Msk | USB_INTSTATRAW_EP4_ACK_RAW_Msk))
  {
		wIstr = USB_ReadInterrupts(hpcd->Instance);
		
    /* extract highest priority endpoint number */
    epindex = (uint8_t)PCD_GET_EP_ISR_INDEX(wIstr);
		
		PCD_CLR_EP_ACK(hpcd->Instance, epindex);
		
		/* EP0 ************************************************************************/
    if (epindex == 0U)
		{
			/* SETUP */
			if (wIstr & USB_INTSTATRAW_SUDAV_RAW_Msk)
			{
				__HAL_PCD_CLEAR_FLAG(hpcd, USB_INTCLR_SUDAV_CLR_Msk);
				
				hpcd->Setup[0] = hpcd->Instance->SETUP03DATA;
				hpcd->Setup[1] = hpcd->Instance->SETUP47DATA;
				
				/* Process SETUP Packet*/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
				hpcd->SetupStageCallback(hpcd);
#else
				HAL_PCD_SetupStageCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
			}
			/* OUT */
			else if (wIstr & USB_INTSTATRAW_EP0_OUT_RAW_Msk)
			{
				__HAL_PCD_CLEAR_FLAG(hpcd, USB_INTCLR_EP0_OUT_CLR_Msk);
				
				ep = &hpcd->OUT_ep[0];
				
				/* Get Control Data OUT Packet */
				ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

				if ((ep->xfer_count != 0U) && (ep->xfer_buff != 0U))
				{
					USB_ReadPMA(hpcd->Instance, ep->xfer_buff,
											ep->pmaadress, (uint16_t)ep->xfer_count);

					ep->xfer_buff += ep->xfer_count;

					/* Process Control Data OUT Packet */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
					hpcd->DataOutStageCallback(hpcd, 0U);
#else
					HAL_PCD_DataOutStageCallback(hpcd, 0U);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
				}
				
				PCD_SET_EP_RX_VALID(hpcd->Instance, ep->num);
			}
			/* IN */
			else
			{
        ep = &hpcd->IN_ep[0];

        ep->xfer_count = PCD_GET_EP_TX_CNT(ep);
        ep->xfer_buff += ep->xfer_count;

        /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DataInStageCallback(hpcd, 0U);
#else
        HAL_PCD_DataInStageCallback(hpcd, 0U);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
			}
		}
		/* EP1/2/3/4 ******************************************************************/
		else
		{
			/* OUT */
			if (PCD_IS_EP_OUT(wIstr, epindex))
			{
				PCD_CLR_EP_OUT(hpcd->Instance, epindex);
				
				ep = &hpcd->OUT_ep[epindex];
				
				count = (uint16_t)PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

				if (count != 0U)
				{
					USB_ReadPMA(hpcd->Instance, ep->xfer_buff, ep->pmaadress, count);
				}
				
        /* multi-packet on the NON control OUT endpoint */
        ep->xfer_count += count;
        ep->xfer_buff += count;
				
        if ((ep->xfer_len == 0U) || (count < ep->maxpacket))
        {
          /* RX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->DataOutStageCallback(hpcd, ep->num);
#else
          HAL_PCD_DataOutStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
        {
          (void)USB_EPStartXfer(hpcd->Instance, ep);
        }
			}
			/* IN */
			else
			{
				ep = &hpcd->IN_ep[epindex];
				
        if (ep->type == EP_TYPE_ISOC)
        {
          ep->xfer_len = 0U;

          /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->DataInStageCallback(hpcd, ep->num);
#else
          HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
        {
					/* multi-packet on the NON control IN endpoint */
					TxPctSize = (uint16_t)PCD_GET_EP_TX_CNT(ep);

					if (ep->xfer_len > TxPctSize)
					{
						ep->xfer_len -= TxPctSize;
					}
					else
					{
						ep->xfer_len = 0U;
					}

					/* Zero Length Packet? */
					if (ep->xfer_len == 0U)
					{
						/* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
						hpcd->DataInStageCallback(hpcd, ep->num);
#else
						HAL_PCD_DataInStageCallback(hpcd, ep->num);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
					}
					else
					{
						/* Transfer is not yet Done */
						ep->xfer_buff += TxPctSize;
						ep->xfer_count += TxPctSize;
						(void)USB_EPStartXfer(hpcd->Instance, ep);
					}
        }
			}
		}
  }

  return HAL_OK;
}

#endif /* HAL_PCD_MODULE_ENABLED */
