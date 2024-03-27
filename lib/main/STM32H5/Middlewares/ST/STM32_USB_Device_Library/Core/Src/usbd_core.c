/**
  ******************************************************************************
  * @file    usbd_core.c
  * @author  MCD Application Team
  * @brief   This file provides all the USBD core functions.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"

#ifdef USE_USBD_COMPOSITE
#include "usbd_composite_builder.h"
#endif /* USE_USBD_COMPOSITE */

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
#endif /* (USBD_DEBUG_LEVEL > 1U) */
    return USBD_FAIL;
  }

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Unlink previous class*/
    pdev->pClass[i] = NULL;
    pdev->pUserData[i] = NULL;

    /* Set class as inactive */
    pdev->tclasslist[i].Active = 0;
    pdev->NumClasses = 0;
    pdev->classId = 0;
  }
#else
  /* Unlink previous class*/
  pdev->pClass[0] = NULL;
  pdev->pUserData[0] = NULL;
#endif /* USE_USBD_COMPOSITE */

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

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Free Class Resources */
        pdev->pClass[i]->DeInit(pdev, (uint8_t)pdev->dev_config);
      }
    }
  }
#else
  /* Free Class Resources */
  if (pdev->pClass[0] != NULL)
  {
    pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config);
  }

  pdev->pUserData[0] = NULL;

#endif /* USE_USBD_COMPOSITE */

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
#endif /* (USBD_DEBUG_LEVEL > 1U) */
    return USBD_FAIL;
  }

  /* link the class to the USB Device handle */
  pdev->pClass[0] = pclass;

  /* Get Device Configuration Descriptor */
#ifdef USE_USB_HS
  if (pdev->pClass[pdev->classId]->GetHSConfigDescriptor != NULL)
  {
    pdev->pConfDesc = (void *)pdev->pClass[pdev->classId]->GetHSConfigDescriptor(&len);
  }
#else /* Default USE_USB_FS */
  if (pdev->pClass[pdev->classId]->GetFSConfigDescriptor != NULL)
  {
    pdev->pConfDesc = (void *)pdev->pClass[pdev->classId]->GetFSConfigDescriptor(&len);
  }
#endif /* USE_USB_FS */

  /* Increment the NumClasses */
  pdev->NumClasses ++;

  return USBD_OK;
}

#ifdef USE_USBD_COMPOSITE
/**
  * @brief  USBD_RegisterClassComposite
  *         Link class driver to Device Core.
  * @param  pdev : Device Handle
  * @param  pclass: Class handle
  * @param  classtype: Class type
  * @param  EpAddr: Endpoint Address handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_RegisterClassComposite(USBD_HandleTypeDef *pdev, USBD_ClassTypeDef *pclass,
                                                USBD_CompositeClassTypeDef classtype, uint8_t *EpAddr)
{
  USBD_StatusTypeDef   ret = USBD_OK;
  uint16_t len = 0U;

  if ((pdev->classId < USBD_MAX_SUPPORTED_CLASS) && (pdev->NumClasses < USBD_MAX_SUPPORTED_CLASS))
  {
    if ((uint32_t)pclass != 0U)
    {
      /* Link the class to the USB Device handle */
      pdev->pClass[pdev->classId] = pclass;
      ret = USBD_OK;

      pdev->tclasslist[pdev->classId].EpAdd = EpAddr;

      /* Call the composite class builder */
      (void)USBD_CMPSIT_AddClass(pdev, pclass, classtype, 0);

      /* Increment the ClassId for the next occurrence */
      pdev->classId ++;
      pdev->NumClasses ++;
    }
    else
    {
#if (USBD_DEBUG_LEVEL > 1U)
      USBD_ErrLog("Invalid Class handle");
#endif /* (USBD_DEBUG_LEVEL > 1U) */
      ret = USBD_FAIL;
    }
  }

  if (ret == USBD_OK)
  {
    /* Get Device Configuration Descriptor */
#ifdef USE_USB_HS
    pdev->pConfDesc = USBD_CMPSIT.GetHSConfigDescriptor(&len);
#else /* Default USE_USB_FS */
    pdev->pConfDesc = USBD_CMPSIT.GetFSConfigDescriptor(&len);
#endif /* USE_USB_FS */
  }

  return ret;
}

/**
  * @brief  USBD_UnRegisterClassComposite
  *         UnLink all composite class drivers from Device Core.
  * @param  pDevice : Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_UnRegisterClassComposite(USBD_HandleTypeDef *pdev)
{
  USBD_StatusTypeDef   ret = USBD_FAIL;
  uint8_t idx1;
  uint8_t idx2;

  /* Unroll all activated classes */
  for (idx1 = 0; idx1 < pdev->NumClasses; idx1++)
  {
    /* Check if the class correspond to the requested type and if it is active */
    if (pdev->tclasslist[idx1].Active == 1U)
    {
      /* Set the new class ID */
      pdev->classId = idx1;

      /* Free resources used by the selected class */
      if (pdev->pClass[pdev->classId] != NULL)
      {
        /* Free Class Resources */
        if (pdev->pClass[pdev->classId]->DeInit(pdev, (uint8_t)pdev->dev_config) != 0U)
        {
#if (USBD_DEBUG_LEVEL > 1U)
          USBD_ErrLog("Class DeInit didn't succeed!, can't unregister selected class");
#endif /* (USBD_DEBUG_LEVEL > 1U) */

          ret = USBD_FAIL;
        }
      }

      /* Free the class pointer */
      pdev->pClass[pdev->classId] = NULL;

      /* Free the class location in classes table and reset its parameters to zero */
      pdev->tclasslist[pdev->classId].ClassType = CLASS_TYPE_NONE;
      pdev->tclasslist[pdev->classId].ClassId = 0U;
      pdev->tclasslist[pdev->classId].Active = 0U;
      pdev->tclasslist[pdev->classId].NumEps = 0U;
      pdev->tclasslist[pdev->classId].NumIf = 0U;
      pdev->tclasslist[pdev->classId].CurrPcktSze = 0U;

      for (idx2 = 0U; idx2 < USBD_MAX_CLASS_ENDPOINTS; idx2++)
      {
        pdev->tclasslist[pdev->classId].Eps[idx2].add = 0U;
        pdev->tclasslist[pdev->classId].Eps[idx2].type = 0U;
        pdev->tclasslist[pdev->classId].Eps[idx2].size = 0U;
        pdev->tclasslist[pdev->classId].Eps[idx2].is_used = 0U;
      }

      for (idx2 = 0U; idx2 < USBD_MAX_CLASS_INTERFACES; idx2++)
      {
        pdev->tclasslist[pdev->classId].Ifs[idx2] = 0U;
      }
    }
  }

  /* Reset the configuration descriptor */
  (void)USBD_CMPST_ClearConfDesc(pdev);

  /* Reset the class ID and number of classes */
  pdev->classId = 0U;
  pdev->NumClasses = 0U;

  return ret;
}


#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_Start
  *         Start the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *pdev)
{
#ifdef USE_USBD_COMPOSITE
  pdev->classId = 0U;
#endif /* USE_USBD_COMPOSITE */

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
#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Free Class Resources */
        (void)pdev->pClass[i]->DeInit(pdev, (uint8_t)pdev->dev_config);
      }
    }
  }

  /* Reset the class ID */
  pdev->classId = 0U;
#else
  if (pdev->pClass[0] != NULL)
  {
    (void)pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config);
  }
#endif /* USE_USBD_COMPOSITE */

  return USBD_OK;
}

/**
  * @brief  USBD_RunTestMode
  *         Launch test mode process
  * @param  pdev: device instance
  * @retval status
  */
USBD_StatusTypeDef USBD_RunTestMode(USBD_HandleTypeDef *pdev)
{
#ifdef USBD_HS_TESTMODE_ENABLE
  USBD_StatusTypeDef ret;

  /* Run USB HS test mode */
  ret = USBD_LL_SetTestMode(pdev, pdev->dev_test_mode);

  return ret;
#else
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);

  return USBD_OK;
#endif /* USBD_HS_TESTMODE_ENABLE */
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
  USBD_StatusTypeDef ret = USBD_OK;

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Set configuration  and Start the Class*/
        if (pdev->pClass[i]->Init(pdev, cfgidx) != 0U)
        {
          ret = USBD_FAIL;
        }
      }
    }
  }
#else
  if (pdev->pClass[0] != NULL)
  {
    /* Set configuration and Start the Class */
    ret = (USBD_StatusTypeDef)pdev->pClass[0]->Init(pdev, cfgidx);
  }
#endif /* USE_USBD_COMPOSITE */

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
  USBD_StatusTypeDef ret = USBD_OK;

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Clear configuration  and De-initialize the Class process */
        if (pdev->pClass[i]->DeInit(pdev, cfgidx) != 0U)
        {
          ret = USBD_FAIL;
        }
      }
    }
  }
#else
  /* Clear configuration  and De-initialize the Class process */
  if (pdev->pClass[0]->DeInit(pdev, cfgidx) != 0U)
  {
    ret = USBD_FAIL;
  }
#endif /* USE_USBD_COMPOSITE */

  return ret;
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
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t idx;

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
        /* Find the class ID relative to the current request */
        switch (pdev->request.bmRequest & 0x1FU)
        {
          case USB_REQ_RECIPIENT_DEVICE:
            /* Device requests must be managed by the first instantiated class
               (or duplicated by all classes for simplicity) */
            idx = 0U;
            break;

          case USB_REQ_RECIPIENT_INTERFACE:
            idx = USBD_CoreFindIF(pdev, LOBYTE(pdev->request.wIndex));
            break;

          case USB_REQ_RECIPIENT_ENDPOINT:
            idx = USBD_CoreFindEP(pdev, LOBYTE(pdev->request.wIndex));
            break;

          default:
            /* Back to the first class in case of doubt */
            idx = 0U;
            break;
        }

        if (idx < USBD_MAX_SUPPORTED_CLASS)
        {
          /* Setup the class ID and route the request to the relative class function */
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if (pdev->pClass[idx]->EP0_RxReady != NULL)
            {
              pdev->classId = idx;
              pdev->pClass[idx]->EP0_RxReady(pdev);
            }
          }
        }

        (void)USBD_CtlSendStatus(pdev);
      }
    }
  }
  else
  {
    /* Get the class index relative to this interface */
    idx = USBD_CoreFindEP(pdev, (epnum & 0x7FU));

    if (((uint16_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS))
    {
      /* Call the class data out function to manage the request */
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        if (pdev->pClass[idx]->DataOut != NULL)
        {
          pdev->classId = idx;
          ret = (USBD_StatusTypeDef)pdev->pClass[idx]->DataOut(pdev, epnum);
        }
      }
      if (ret != USBD_OK)
      {
        return ret;
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
  uint8_t idx;

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
            if (pdev->pClass[0]->EP0_TxSent != NULL)
            {
              pdev->classId = 0U;
              pdev->pClass[0]->EP0_TxSent(pdev);
            }
          }
          (void)USBD_LL_StallEP(pdev, 0x80U);
          (void)USBD_CtlReceiveStatus(pdev);
        }
      }
    }

    if (pdev->dev_test_mode != 0U)
    {
      (void)USBD_RunTestMode(pdev);
      pdev->dev_test_mode = 0U;
    }
  }
  else
  {
    /* Get the class index relative to this interface */
    idx = USBD_CoreFindEP(pdev, ((uint8_t)epnum | 0x80U));

    if (((uint16_t)idx != 0xFFU) && (idx < USBD_MAX_SUPPORTED_CLASS))
    {
      /* Call the class data out function to manage the request */
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        if (pdev->pClass[idx]->DataIn != NULL)
        {
          pdev->classId = idx;
          ret = (USBD_StatusTypeDef)pdev->pClass[idx]->DataIn(pdev, epnum);

          if (ret != USBD_OK)
          {
            return ret;
          }
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
  USBD_StatusTypeDef ret = USBD_OK;

  /* Upon Reset call user call back */
  pdev->dev_state = USBD_STATE_DEFAULT;
  pdev->ep0_state = USBD_EP0_IDLE;
  pdev->dev_config = 0U;
  pdev->dev_remote_wakeup = 0U;
  pdev->dev_test_mode = 0U;

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Clear configuration  and De-initialize the Class process*/

        if (pdev->pClass[i]->DeInit != NULL)
        {
          if (pdev->pClass[i]->DeInit(pdev, (uint8_t)pdev->dev_config) != USBD_OK)
          {
            ret = USBD_FAIL;
          }
        }
      }
    }
  }
#else

  if (pdev->pClass[0] != NULL)
  {
    if (pdev->pClass[0]->DeInit != NULL)
    {
      if (pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config) != USBD_OK)
      {
        ret = USBD_FAIL;
      }
    }
  }
#endif /* USE_USBD_COMPOSITE */

  /* Open EP0 OUT */
  (void)USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_out[0x00U & 0xFU].is_used = 1U;

  pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

  /* Open EP0 IN */
  (void)USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  pdev->ep_in[0x80U & 0xFU].is_used = 1U;

  pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

  return ret;
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
  if (pdev->dev_state != USBD_STATE_SUSPENDED)
  {
    pdev->dev_old_state = pdev->dev_state;
  }

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
  /* The SOF event can be distributed for all classes that support it */
  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
#ifdef USE_USBD_COMPOSITE
    /* Parse the table of classes in use */
    for (uint32_t i = 0; i < USBD_MAX_SUPPORTED_CLASS; i++)
    {
      /* Check if current class is in use */
      if ((pdev->tclasslist[i].Active) == 1U)
      {
        if (pdev->pClass[i] != NULL)
        {
          if (pdev->pClass[i]->SOF != NULL)
          {
            pdev->classId = i;
            (void)pdev->pClass[i]->SOF(pdev);
          }
        }
      }
    }
#else
    if (pdev->pClass[0] != NULL)
    {
      if (pdev->pClass[0]->SOF != NULL)
      {
        (void)pdev->pClass[0]->SOF(pdev);
      }
    }
#endif /* USE_USBD_COMPOSITE */
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
  if (pdev->pClass[pdev->classId] == NULL)
  {
    return USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass[pdev->classId]->IsoINIncomplete != NULL)
    {
      (void)pdev->pClass[pdev->classId]->IsoINIncomplete(pdev, epnum);
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
  if (pdev->pClass[pdev->classId] == NULL)
  {
    return USBD_FAIL;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (pdev->pClass[pdev->classId]->IsoOUTIncomplete != NULL)
    {
      (void)pdev->pClass[pdev->classId]->IsoOUTIncomplete(pdev, epnum);
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
  USBD_StatusTypeDef   ret = USBD_OK;

  /* Free Class Resources */
  pdev->dev_state = USBD_STATE_DEFAULT;

#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      if (pdev->pClass[i] != NULL)
      {
        pdev->classId = i;
        /* Clear configuration  and De-initialize the Class process*/
        if (pdev->pClass[i]->DeInit(pdev, (uint8_t)pdev->dev_config) != 0U)
        {
          ret = USBD_FAIL;
        }
      }
    }
  }
#else
  if (pdev->pClass[0] != NULL)
  {
    if (pdev->pClass[0]->DeInit(pdev, (uint8_t)pdev->dev_config) != 0U)
    {
      ret = USBD_FAIL;
    }
  }
#endif /* USE_USBD_COMPOSITE */

  return ret;
}

/**
  * @brief  USBD_CoreFindIF
  *         return the class index relative to the selected interface
  * @param  pdev: device instance
  * @param  index : selected interface number
  * @retval index of the class using the selected interface number. OxFF if no class found.
  */
uint8_t USBD_CoreFindIF(USBD_HandleTypeDef *pdev, uint8_t index)
{
#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      /* Parse all interfaces listed in the current class */
      for (uint32_t j = 0U; j < pdev->tclasslist[i].NumIf; j++)
      {
        /* Check if requested Interface matches the current class interface */
        if (pdev->tclasslist[i].Ifs[j] == index)
        {
          if (pdev->pClass[i]->Setup != NULL)
          {
            return (uint8_t)i;
          }
        }
      }
    }
  }

  return 0xFFU;
#else
  UNUSED(pdev);
  UNUSED(index);

  return 0x00U;
#endif /* USE_USBD_COMPOSITE */
}

/**
  * @brief  USBD_CoreFindEP
  *         return the class index relative to the selected endpoint
  * @param  pdev: device instance
  * @param  index : selected endpoint number
  * @retval index of the class using the selected endpoint number. 0xFF if no class found.
  */
uint8_t USBD_CoreFindEP(USBD_HandleTypeDef *pdev, uint8_t index)
{
#ifdef USE_USBD_COMPOSITE
  /* Parse the table of classes in use */
  for (uint32_t i = 0U; i < USBD_MAX_SUPPORTED_CLASS; i++)
  {
    /* Check if current class is in use */
    if ((pdev->tclasslist[i].Active) == 1U)
    {
      /* Parse all endpoints listed in the current class */
      for (uint32_t j = 0U; j < pdev->tclasslist[i].NumEps; j++)
      {
        /* Check if requested endpoint matches the current class endpoint */
        if (pdev->tclasslist[i].Eps[j].add == index)
        {
          if (pdev->pClass[i]->Setup != NULL)
          {
            return (uint8_t)i;
          }
        }
      }
    }
  }

  return 0xFFU;
#else
  UNUSED(pdev);
  UNUSED(index);

  return 0x00U;
#endif /* USE_USBD_COMPOSITE */
}

#ifdef USE_USBD_COMPOSITE
/**
  * @brief  USBD_CoreGetEPAdd
  *         Get the endpoint address relative to a selected class
  * @param  pdev: device instance
  * @param  ep_dir: USBD_EP_IN or USBD_EP_OUT
  * @param  ep_type: USBD_EP_TYPE_CTRL, USBD_EP_TYPE_ISOC, USBD_EP_TYPE_BULK or USBD_EP_TYPE_INTR
  * @param  ClassId: The Class ID
  * @retval Address of the selected endpoint or 0xFFU if no endpoint found.
  */
uint8_t USBD_CoreGetEPAdd(USBD_HandleTypeDef *pdev, uint8_t ep_dir, uint8_t ep_type, uint8_t ClassId)
{
  uint8_t idx;

  /* Find the EP address in the selected class table */
  for (idx = 0; idx < pdev->tclasslist[ClassId].NumEps; idx++)
  {
    if (((pdev->tclasslist[ClassId].Eps[idx].add & USBD_EP_IN) == ep_dir) && \
        (pdev->tclasslist[ClassId].Eps[idx].type == ep_type) && \
        (pdev->tclasslist[ClassId].Eps[idx].is_used != 0U))
    {
      return (pdev->tclasslist[ClassId].Eps[idx].add);
    }
  }

  /* If reaching this point, then no endpoint was found */
  return 0xFFU;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_GetEpDesc
  *         This function return the Endpoint descriptor
  * @param  pdev: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @param  EpAddr:  endpoint address
  * @retval pointer to video endpoint descriptor
  */
void *USBD_GetEpDesc(uint8_t *pConfDesc, uint8_t EpAddr)
{
  USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
  USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
  USBD_EpDescTypeDef *pEpDesc = NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDescriptorType == USB_DESC_TYPE_ENDPOINT)
      {
        pEpDesc = (USBD_EpDescTypeDef *)(void *)pdesc;

        if (pEpDesc->bEndpointAddress == EpAddr)
        {
          break;
        }
        else
        {
          pEpDesc = NULL;
        }
      }
    }
  }

  return (void *)pEpDesc;
}

/**
  * @brief  USBD_GetNextDesc
  *         This function return the next descriptor header
  * @param  buf: Buffer where the descriptor is available
  * @param  ptr: data pointer inside the descriptor
  * @retval next header
  */
USBD_DescHeaderTypeDef *USBD_GetNextDesc(uint8_t *pbuf, uint16_t *ptr)
{
  USBD_DescHeaderTypeDef *pnext = (USBD_DescHeaderTypeDef *)(void *)pbuf;

  *ptr += pnext->bLength;
  pnext = (USBD_DescHeaderTypeDef *)(void *)(pbuf + pnext->bLength);

  return (pnext);
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

