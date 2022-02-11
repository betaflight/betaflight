/**
  ******************************************************************************
  * @file    usbd_core.c
  * @author  MCD Application Team
  * @brief   This file provides all the USBD core functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"

/** @addtogroup STM32_USBD_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CORE
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CORE_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CORE_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CORE_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CORE_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Private_Variables
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CORE_Private_Functions
  * @{
  */

/**
  * @brief  USBD_Init
  *         Initializes the device stack and load the class driver
  * @param  pdev: device instance
  * @param  pdesc: Descriptor structure address
  * @param  id: Low level core index
  * @retval None
  */
USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *pdev,
                             USBD_DescriptorsTypeDef *pdesc, uint8_t id)
{
  USBD_StatusTypeDef ret;

  /* Check whether the USB Host handle is valid */
  if (pdev == NULL)
  {
#if (USBD_DEBUG_LEVEL > 1U)
    USBD_ErrLog("Invalid Device handle");
#endif
    return USBD_FAIL;
  }

  /* Unlink previous class resources */
  pdev->pClass = NULL;
  pdev->pUserData = NULL;
  pdev->pConfDesc = NULL;

  /* Assign USBD Descriptors */
  if (pdesc != NULL)
  {
    pdev->pDesc = pdesc;
  }

  /* Set Device initial State */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->id = id;

  /* Initialize low level driver */
  ret = USBD_LL_Init(pdev);

  return ret;
}

/**
  * @brief  USBD_DeInit
  *         Re-Initialize the device library
  * @param  pdev: device instance
  * @retval status: status
  */
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev)
{
  USBD_StatusTypeDef ret;

  /* Disconnect the USB Device */
  (void)USBD_LL_Stop(pdev);

  /* Set Default State */
  pdev->dev_state = USBD_STATE_DEFAULT;

  /* Free Class Resources */
  if (pdev->pClass != NULL)
  {
    pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
    pdev->pClass = NULL;
    pdev->pUserData = NULL;
  }

  /* Free Device descriptors resources */
  pdev->pDesc = NULL;
  pdev->pConfDesc = NULL;

  /* DeInitialize low level driver */
  ret = USBD_LL_DeInit(pdev);

  return ret;
}

/**
  * @brief  USBD_RegisterClass
  *         Link class driver to Device Core.
  * @param  pDevice : Device Handle
  * @param  pclass: Class handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass)
{
  uint16_t len = 0U;

  if (pclass == NULL)
  {
#if (USBD_DEBUG_LEVEL > 1U)
    USBD_ErrLog("Invalid Class handle");
#endif
    return USBD_FAIL;
  }

  /* link the class to the USB Device handle */
  pdev->pClass = pclass;

  /* Get Device Configuration Descriptor */
#ifdef USE_USB_HS
  if (pdev->pClass->GetHSConfigDescriptor != NULL)
  {
    pdev->pConfDesc = (void *)pdev->pClass->GetHSConfigDescriptor(&len);
  }
#else /* Default USE_USB_FS */
  if (pdev->pClass->GetFSConfigDescriptor != NULL)
  {
    pdev->pConfDesc = (void *)pdev->pClass->GetFSConfigDescriptor(&len);
  }
#endif /* USE_USB_FS */

  return USBD_OK;
}

/**
  * @brief  USBD_Start
  *         Start the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *pdev)
{
  /* Start the low level driver  */
  return USBD_LL_Start(pdev);
}

/**
  * @brief  USBD_Stop
  *         Stop the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_Stop(USBD_HandleTypeDef *pdev)
{
  /* Disconnect USB Device */
  (void)USBD_LL_Stop(pdev);

  /* Free Class Resources */
  if (pdev->pClass != NULL)
  {
    (void)pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
  }

  return USBD_OK;
}

/**
  * @brief  USBD_RunTestMode
  *         Launch test mode process
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_RunTestMode(USBD_HandleTypeDef  *pdev)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);

  return USBD_OK;
}

/**
  * @brief  USBD_SetClassConfig
  *        Configure device and start the interface
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status
  */

USBD_StatusTypeDef USBD_SetClassConfig(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_StatusTypeDef ret = USBD_FAIL;

  if (pdev->pClass != NULL)
  {
    /* Set configuration and Start the Class */
    ret = (USBD_StatusTypeDef)pdev->pClass->Init(pdev, cfgidx);
  }

  return ret;
}

/**
  * @brief  USBD_ClrClassConfig
  *         Clear current configuration
  * @param  pdev: device instance
  * @param  cfgidx: configuration index
  * @retval status: USBD_StatusTypeDef
  */
USBD_StatusTypeDef USBD_ClrClassConfig(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* Clear configuration and De-initialize the Class process */
  if (pdev->pClass != NULL)
  {
    pdev->pClass->DeInit(pdev, cfgidx);
  }

  return USBD_OK;
}


/**
  * @brief  USBD_LL_SetupStage
  *         Handle the setup stage
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_SetupStage(USBD_HandleTypeDef *pdev, uint8_t *psetup)
{
  USBD_StatusTypeDef ret;

  USBD_ParseSetupRequest(&pdev->request, psetup);

  pdev->ep0_state = USBD_EP0_SETUP;

  pdev->ep0_data_len = pdev->request.wLength;

  switch (pdev->request.bmRequest & 0x1FU)
  {
    case USB_REQ_RECIPIENT_DEVICE:
      ret = USBD_StdDevReq(pdev, &pdev->request);
      break;

    case USB_REQ_RECIPIENT_INTERFACE:
      ret = USBD_StdItfReq(pdev, &pdev->request);
      break;

    case USB_REQ_RECIPIENT_ENDPOINT:
      ret = USBD_StdEPReq(pdev, &pdev->request);
      break;

    default:
      ret = USBD_LL_StallEP(pdev, (pdev->request.bmRequest & 0x80U));
      break;
  }

  return ret;
}

/**
  * @brief  USBD_LL_DataOutStage
  *         Handle data OUT stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @param  pdata: data pointer
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DataOutStage(USBD_HandleTypeDef *pdev,
                                        uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;
  USBD_StatusTypeDef ret;

  if (epnum == 0U)
  {
    pep = &pdev->ep_out[0];

    if (pdev->ep0_state == USBD_EP0_DATA_OUT)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        (void)USBD_CtlContinueRx(pdev, pdata, MIN(pep->rem_length, pep->maxpacket));
      }
      else
      {
        if (pdev->dev_state == USBD_STATE_CONFIGURED)
        {
          if (pdev->pClass->EP0_RxReady != NULL)
          {
            pdev->pClass->EP0_RxReady(pdev);
          }
        }

        (void)USBD_CtlSendStatus(pdev);
      }
    }
    else
    {
#if 0
      if (pdev->ep0_state == USBD_EP0_STATUS_OUT)
      {
        /*
          * STATUS PHASE completed, update ep0_state to idle
          */
        pdev->ep0_state = USBD_EP0_IDLE;
        (void)USBD_LL_StallEP(pdev, 0U);
      }
#endif
    }
  }
  else
  {
    if (pdev->dev_state == USBD_STATE_CONFIGURED)
    {
      if (pdev->pClass->DataOut != NULL)
      {
        ret = (USBD_StatusTypeDef)pdev->pClass->DataOut(pdev, epnum);

        if (ret != USBD_OK)
        {
          return ret;
        }
      }
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_DataInStage
  *         Handle data in stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DataInStage(USBD_HandleTypeDef *pdev,
                                       uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;
  USBD_StatusTypeDef ret;

  if (epnum == 0U)
  {
    pep = &pdev->ep_in[0];

    if (pdev->ep0_state == USBD_EP0_DATA_IN)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        (void)USBD_CtlContinueSendData(pdev, pdata, pep->rem_length);

        /* Prepare endpoint for premature end of transfer */
        (void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
      }
      else
      {
        /* last packet is MPS multiple, so send ZLP packet */
        if ((pep->maxpacket == pep->rem_length) &&
            (pep->total_length >= pep->maxpacket) &&
            (pep->total_length < pdev->ep0_data_len))
        {
          (void)USBD_CtlContinueSendData(pdev, NULL, 0U);
          pdev->ep0_data_len = 0U;

          /* Prepare endpoint for premature end of transfer */
          (void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
        }
        else
        {
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if (pdev->pClass->EP0_TxSent != NULL)
            {
              pdev->pClass->EP0_TxSent(pdev);
            }
          }
          (void)USBD_LL_StallEP(pdev, 0x80U);
          (void)USBD_CtlReceiveStatus(pdev);
        }
      }
    }
    else
    {
#if 0
      if ((pdev->ep0_state == USBD_EP0_STATUS_IN) ||
          (pdev->ep0_state == USBD_EP0_IDLE))
      {
        (void)USBD_LL_StallEP(pdev, 0x80U);
      }
#endif
    }

    if (pdev->dev_test_mode == 1U)
    {
      (void)USBD_RunTestMode(pdev);
      pdev->dev_test_mode = 0U;
    }
  }
  else
  {
    if (pdev->dev_state == USBD_STATE_CONFIGURED)
    {
      if (pdev->pClass->DataIn != NULL)
      {
        ret = (USBD_StatusTypeDef)pdev->pClass->DataIn(pdev, epnum);

        if (ret != USBD_OK)
        {
          return ret;
        }
      }
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_Reset
  *         Handle Reset event
  * @param  pdev: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_Reset(USBD_HandleTypeDef *pdev)
{
  /* Upon Reset call user call back */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->ep0_state = USBD_EP0_IDLE;
  pdev->dev_config = 0U;
  pdev->dev_remote_wakeup = 0U;

  if (pdev->pClass == NULL)
  {
    return USBD_FAIL;
  }

  if (pdev->pClassData != NULL)
  {
    if (pdev->pClass->DeInit != NULL)
    {
      (void)pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
    }
  }

  /* Open EP0 OUT */
  (void)USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_out[0x00U & 0xFU].is_used = 1U;

  pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

  /* Open EP0 IN */
  (void)USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_in[0x80U & 0xFU].is_used = 1U;

  pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

  return USBD_OK;
}

/**
  * @brief  USBD_LL_SetSpeed
  *         Handle Reset event
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_SetSpeed(USBD_HandleTypeDef *pdev,
                                    USBD_SpeedTypeDef speed)
{
  pdev->dev_speed = speed;

  return USBD_OK;
}

/**
  * @brief  USBD_LL_Suspend
  *         Handle Suspend event
  * @param  pdev: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_Suspend(USBD_HandleTypeDef *pdev)
{
  pdev->dev_old_state = pdev->dev_state;
  pdev->dev_state = USBD_STATE_SUSPENDED;

  return USBD_OK;
}

/**
  * @brief  USBD_LL_Resume
  *         Handle Resume event
  * @param  pdev: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_Resume(USBD_HandleTypeDef *pdev)
{
  if (pdev->dev_state == USBD_STATE_SUSPENDED)
  {
    pdev->dev_state = pdev->dev_old_state;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_SOF
  *         Handle SOF event
  * @param  pdev: device instance
  * @retval status
  */

USBD_StatusTypeDef USBD_LL_SOF(USBD_HandleTypeDef *pdev)
{
  if (pdev->pClass == NULL)
  {
    return USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass->SOF != NULL)
    {
      (void)pdev->pClass->SOF(pdev);
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_IsoINIncomplete
  *         Handle iso in incomplete event
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(USBD_HandleTypeDef *pdev,
                                           uint8_t epnum)
{
  if (pdev->pClass == NULL)
  {
    return USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass->IsoINIncomplete != NULL)
    {
      (void)pdev->pClass->IsoINIncomplete(pdev, epnum);
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_IsoOUTIncomplete
  *         Handle iso out incomplete event
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_IsoOUTIncomplete(USBD_HandleTypeDef *pdev,
                                            uint8_t epnum)
{
  if (pdev->pClass == NULL)
  {
    return USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass->IsoOUTIncomplete != NULL)
    {
      (void)pdev->pClass->IsoOUTIncomplete(pdev, epnum);
    }
  }

  return USBD_OK;
}

/**
  * @brief  USBD_LL_DevConnected
  *         Handle device connection event
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DevConnected(USBD_HandleTypeDef *pdev)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);

  return USBD_OK;
}

/**
  * @brief  USBD_LL_DevDisconnected
  *         Handle device disconnection event
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_LL_DevDisconnected(USBD_HandleTypeDef *pdev)
{
  /* Free Class Resources */
  pdev->dev_state = USBD_STATE_DEFAULT;

  if (pdev->pClass != NULL)
  {
    (void)pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
  }

  return USBD_OK;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

