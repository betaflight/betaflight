/**
  ******************************************************************************
  * @file    usbd_dfu.c
  * @author  MCD Application Team
  * @brief   This file provides the DFU core functions.
  *
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
  * @verbatim
  *
  *          ===================================================================
  *                                DFU Class Driver Description
  *          ===================================================================
  *           This driver manages the DFU class V1.1 following the "Device Class Specification for
  *           Device Firmware Upgrade Version 1.1 Aug 5, 2004".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as DFU device (in DFU mode only)
  *             - Requests management (supporting ST DFU sub-protocol)
  *             - Memory operations management (Download/Upload/Erase/Detach/GetState/GetStatus)
  *             - DFU state machine implementation.
  *
  *           @note
  *            ST DFU sub-protocol is compliant with DFU protocol and use sub-requests to manage
  *            memory addressing, commands processing, specific memories operations (ie. Erase) ...
  *            As required by the DFU specification, only endpoint 0 is used in this application.
  *            Other endpoints and functions may be added to the application (ie. DFU ...)
  *
  *           These aspects may be enriched or modified for a specific user application.
  *
  *           This driver doesn't implement the following aspects of the specification
  *           (but it is possible to manage these features with some modifications on this driver):
  *             - Manifestation Tolerant mode
  *
  *  @endverbatim
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_DFU
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_DFU_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_DFU_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_DFU_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_DFU_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_DFU_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_DFU_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_DFU_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_DFU_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_DFU_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_DFU_SOF(USBD_HandleTypeDef *pdev);

#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_DFU_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_DFU_GetDeviceQualifierDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE */

#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *USBD_DFU_GetUsrStringDesc(USBD_HandleTypeDef *pdev,
                                          uint8_t index, uint16_t *length);
#endif /* USBD_SUPPORT_USER_STRING_DESC */

static void DFU_Detach(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void DFU_Download(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void DFU_Upload(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void DFU_GetStatus(USBD_HandleTypeDef *pdev);
static void DFU_ClearStatus(USBD_HandleTypeDef *pdev);
static void DFU_GetState(USBD_HandleTypeDef *pdev);
static void DFU_Abort(USBD_HandleTypeDef *pdev);
static void DFU_Leave(USBD_HandleTypeDef *pdev);
static void *USBD_DFU_GetDfuFuncDesc(uint8_t *pConfDesc);

/**
  * @}
  */

/** @defgroup USBD_DFU_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_DFU =
{
  USBD_DFU_Init,
  USBD_DFU_DeInit,
  USBD_DFU_Setup,
  USBD_DFU_EP0_TxReady,
  USBD_DFU_EP0_RxReady,
  NULL,
  NULL,
  USBD_DFU_SOF,
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_DFU_GetCfgDesc,
  USBD_DFU_GetCfgDesc,
  USBD_DFU_GetCfgDesc,
  USBD_DFU_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  USBD_DFU_GetUsrStringDesc
#endif /* USBD_SUPPORT_USER_STRING_DESC */
};

#ifndef USE_USBD_COMPOSITE
/* USB DFU device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_DFU_CfgDesc[USB_DFU_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09,                                                /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                         /* bDescriptorType: Configuration */
  USB_DFU_CONFIG_DESC_SIZ,                             /* wTotalLength: Bytes returned */
  0x00,
  0x01,                                                /* bNumInterfaces: 1 interface */
  0x01,                                                /* bConfigurationValue: Configuration value */
  0x02,                                                /* iConfiguration: Index of string descriptor
                                                          describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                                                /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80,                                                /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                                      /* MaxPower (mA) */
  /* 09 */

  /**********  Descriptor of DFU interface 0 Alternate setting 0 **************/
  USBD_DFU_IF_DESC(0U),                                /* This interface is mandatory for all devices */

#if (USBD_DFU_MAX_ITF_NUM > 1U)
  /**********  Descriptor of DFU interface 0 Alternate setting 1 **************/
  USBD_DFU_IF_DESC(1),
#endif /* (USBD_DFU_MAX_ITF_NUM > 1) */

#if (USBD_DFU_MAX_ITF_NUM > 2U)
  /**********  Descriptor of DFU interface 0 Alternate setting 2 **************/
  USBD_DFU_IF_DESC(2),
#endif /* (USBD_DFU_MAX_ITF_NUM > 2) */

#if (USBD_DFU_MAX_ITF_NUM > 3U)
  /**********  Descriptor of DFU interface 0 Alternate setting 3 **************/
  USBD_DFU_IF_DESC(3),
#endif /* (USBD_DFU_MAX_ITF_NUM > 3) */

#if (USBD_DFU_MAX_ITF_NUM > 4U)
  /**********  Descriptor of DFU interface 0 Alternate setting 4 **************/
  USBD_DFU_IF_DESC(4),
#endif /* (USBD_DFU_MAX_ITF_NUM > 4) */

#if (USBD_DFU_MAX_ITF_NUM > 5U)
  /**********  Descriptor of DFU interface 0 Alternate setting 5 **************/
  USBD_DFU_IF_DESC(5),
#endif /* (USBD_DFU_MAX_ITF_NUM > 5) */

#if (USBD_DFU_MAX_ITF_NUM > 6U)
#error "ERROR: usbd_dfu_core.c: Modify the file to support more descriptors!"
#endif /* (USBD_DFU_MAX_ITF_NUM > 6) */

  /******************** DFU Functional Descriptor********************/
  0x09,                                                /* blength = 9 Bytes */
  DFU_DESCRIPTOR_TYPE,                                 /* DFU Functional Descriptor */
  0x0B,                                                /* bmAttribute:
                                                          bitCanDnload             = 1      (bit 0)
                                                          bitCanUpload             = 1      (bit 1)
                                                          bitManifestationTolerant = 0      (bit 2)
                                                          bitWillDetach            = 1      (bit 3)
                                                          Reserved                          (bit4-6)
                                                          bitAcceleratedST         = 0      (bit 7) */
  0xFF,                                                /* DetachTimeOut= 255 ms*/
  0x00,
  /* WARNING: In DMA mode the multiple MPS packets feature is still not supported
   ==> In this case, when using DMA USBD_DFU_XFER_SIZE should be set to 64 in usbd_conf.h */
  TRANSFER_SIZE_BYTES(USBD_DFU_XFER_SIZE),       /* TransferSize = 1024 Byte */
  0x1A,                                /* bcdDFUVersion */
  0x01
  /***********************************************************/
  /* 9*/
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_DFU_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE */

/**
  * @}
  */

/** @defgroup USBD_DFU_Private_Functions
  * @{
  */

/**
  * @brief  USBD_DFU_Init
  *         Initialize the DFU interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_DFU_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_DFU_HandleTypeDef *hdfu;

  /* Allocate Audio structure */
  hdfu = (USBD_DFU_HandleTypeDef *)USBD_malloc(sizeof(USBD_DFU_HandleTypeDef));

  if (hdfu == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassDataCmsit[pdev->classId] = (void *)hdfu;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

  hdfu->alt_setting = 0U;
  hdfu->data_ptr = USBD_DFU_APP_DEFAULT_ADD;
  hdfu->wblock_num = 0U;
  hdfu->wlength = 0U;

  hdfu->manif_state = DFU_MANIFEST_COMPLETE;
  hdfu->dev_state = DFU_STATE_IDLE;

  hdfu->dev_status[0] = DFU_ERROR_NONE;
  hdfu->dev_status[1] = 0U;
  hdfu->dev_status[2] = 0U;
  hdfu->dev_status[3] = 0U;
  hdfu->dev_status[4] = DFU_STATE_IDLE;
  hdfu->dev_status[5] = 0U;

  /* Initialize Hardware layer */
  if (((USBD_DFU_MediaTypeDef *)pdev->pUserData[pdev->classId])->Init() != USBD_OK)
  {
    return (uint8_t)USBD_FAIL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_DFU_DeInit
  *         De-Initialize the DFU layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_DFU_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_DFU_HandleTypeDef *hdfu;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_EMEM;
  }

  hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  hdfu->wblock_num = 0U;
  hdfu->wlength = 0U;

  hdfu->dev_state = DFU_STATE_IDLE;
  hdfu->dev_status[0] = DFU_ERROR_NONE;
  hdfu->dev_status[4] = DFU_STATE_IDLE;

  /* DeInit  physical Interface components and Hardware Layer */
  ((USBD_DFU_MediaTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
  USBD_free(pdev->pClassDataCmsit[pdev->classId]);
  pdev->pClassDataCmsit[pdev->classId] = NULL;
  pdev->pClassData = NULL;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_DFU_Setup
  *         Handle the DFU specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_DFU_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t *pbuf;
  uint16_t len;
  uint16_t status_info = 0U;

  if (hdfu == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case DFU_DNLOAD:
          DFU_Download(pdev, req);
          break;

        case DFU_UPLOAD:
          DFU_Upload(pdev, req);
          break;

        case DFU_GETSTATUS:
          DFU_GetStatus(pdev);
          break;

        case DFU_CLRSTATUS:
          DFU_ClearStatus(pdev);
          break;

        case DFU_GETSTATE:
          DFU_GetState(pdev);
          break;

        case DFU_ABORT:
          DFU_Abort(pdev);
          break;

        case DFU_DETACH:
          DFU_Detach(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == DFU_DESCRIPTOR_TYPE)
          {
            pbuf = (uint8_t *)USBD_DFU_GetDfuFuncDesc(pdev->pConfDesc);

            if (pbuf != NULL)
            {
              len = MIN(USB_DFU_DESC_SIZ, req->wLength);
              (void)USBD_CtlSendData(pdev, pbuf, len);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&hdfu->alt_setting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if ((uint8_t)(req->wValue) < USBD_DFU_MAX_ITF_NUM)
          {
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
              hdfu->alt_setting = (uint8_t)(req->wValue);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else
          {
            /* Call the error management function (command will be NAKed */
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_DFU_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_DFU_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_DFU_CfgDesc);

  return USBD_DFU_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_DFU_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_DFU_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_DFU_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_DFU_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  USBD_SetupReqTypedef req;
  uint32_t addr;
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_DFU_MediaTypeDef *DfuInterface = (USBD_DFU_MediaTypeDef *)pdev->pUserData[pdev->classId];

  if (hdfu == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (hdfu->dev_state == DFU_STATE_DNLOAD_BUSY)
  {
    /* Decode the Special Command */
    if (hdfu->wblock_num == 0U)
    {
      if (hdfu->wlength == 1U)
      {
        if (hdfu->buffer.d8[0] == DFU_CMD_GETCOMMANDS)
        {
          /* Nothing to do */
        }
      }
      else if (hdfu->wlength == 5U)
      {
        if (hdfu->buffer.d8[0] == DFU_CMD_SETADDRESSPOINTER)
        {
          hdfu->data_ptr = hdfu->buffer.d8[1];
          hdfu->data_ptr += (uint32_t)hdfu->buffer.d8[2] << 8;
          hdfu->data_ptr += (uint32_t)hdfu->buffer.d8[3] << 16;
          hdfu->data_ptr += (uint32_t)hdfu->buffer.d8[4] << 24;
        }
        else if (hdfu->buffer.d8[0] == DFU_CMD_ERASE)
        {
          hdfu->data_ptr = hdfu->buffer.d8[1];
          hdfu->data_ptr += (uint32_t)hdfu->buffer.d8[2] << 8;
          hdfu->data_ptr += (uint32_t)hdfu->buffer.d8[3] << 16;
          hdfu->data_ptr += (uint32_t)hdfu->buffer.d8[4] << 24;

          if (DfuInterface->Erase(hdfu->data_ptr) != USBD_OK)
          {
            return (uint8_t)USBD_FAIL;
          }
        }
        else
        {
          return (uint8_t)USBD_FAIL;
        }
      }
      else
      {
        /* Reset the global length and block number */
        hdfu->wlength = 0U;
        hdfu->wblock_num = 0U;
        /* Call the error management function (command will be NAKed) */
        req.bmRequest = 0U;
        req.wLength = 1U;
        USBD_CtlError(pdev, &req);
      }
    }
    /* Regular Download Command */
    else
    {
      if (hdfu->wblock_num > 1U)
      {
        /* Decode the required address */
        addr = ((hdfu->wblock_num - 2U) * USBD_DFU_XFER_SIZE) + hdfu->data_ptr;

        /* Perform the write operation */
        if (DfuInterface->Write(hdfu->buffer.d8, (uint8_t *)addr, hdfu->wlength) != USBD_OK)
        {
          return (uint8_t)USBD_FAIL;
        }
      }
    }

    /* Reset the global length and block number */
    hdfu->wlength = 0U;
    hdfu->wblock_num = 0U;

    /* Update the state machine */
    hdfu->dev_state =  DFU_STATE_DNLOAD_SYNC;

    hdfu->dev_status[1] = 0U;
    hdfu->dev_status[2] = 0U;
    hdfu->dev_status[3] = 0U;
    hdfu->dev_status[4] = hdfu->dev_state;
  }
  else if (hdfu->dev_state == DFU_STATE_MANIFEST)/* Manifestation in progress */
  {
    /* Start leaving DFU mode */
    DFU_Leave(pdev);
  }
  else
  {
    return (uint8_t)USBD_FAIL;
  }

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_DFU_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_DFU_SOF(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);

  return (uint8_t)USBD_OK;
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_DFU_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_DFU_DeviceQualifierDesc);

  return USBD_DFU_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_DFU_GetUsrStringDesc
  *         Manages the transfer of memory interfaces string descriptors.
  * @param  pdev: device instance
  * @param  index: descriptor index
  * @param  length : pointer data length
  * @retval pointer to the descriptor table or NULL if the descriptor is not supported.
  */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *USBD_DFU_GetUsrStringDesc(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
  static uint8_t USBD_StrDesc[255];
  USBD_DFU_MediaTypeDef *DfuInterface = (USBD_DFU_MediaTypeDef *)pdev->pUserData[pdev->classId];

  /* Check if the requested string interface is supported */
  if (index <= (USBD_IDX_INTERFACE_STR + USBD_DFU_MAX_ITF_NUM))
  {
    USBD_GetString((uint8_t *)DfuInterface->pStrDesc, USBD_StrDesc, length);
    return USBD_StrDesc;
  }
  else
  {
    /* Not supported Interface Descriptor index */
    length = 0U;
    return NULL;
  }
}
#endif /* USBD_SUPPORT_USER_STRING_DESC */

/**
  * @brief  USBD_MSC_RegisterStorage
  * @param  pdev: device instance
  * @param  fops: storage callback
  * @retval status
  */
uint8_t USBD_DFU_RegisterMedia(USBD_HandleTypeDef *pdev,
                               USBD_DFU_MediaTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}

/******************************************************************************
     DFU Class requests management
  ******************************************************************************/
/**
  * @brief  DFU_Detach
  *         Handles the DFU DETACH request.
  * @param  pdev: device instance
  * @param  req: pointer to the request structure.
  * @retval None.
  */
static void DFU_Detach(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_DFUFuncDescTypeDef *pDfuFunc = (USBD_DFUFuncDescTypeDef *)USBD_DFU_GetDfuFuncDesc(pdev->pConfDesc);

  if ((hdfu == NULL) || (pDfuFunc == NULL))
  {
    return;
  }

  if ((hdfu->dev_state == DFU_STATE_IDLE) ||
      (hdfu->dev_state == DFU_STATE_DNLOAD_SYNC) ||
      (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE) ||
      (hdfu->dev_state == DFU_STATE_MANIFEST_SYNC) ||
      (hdfu->dev_state == DFU_STATE_UPLOAD_IDLE))
  {
    /* Update the state machine */
    hdfu->dev_state = DFU_STATE_IDLE;
    hdfu->dev_status[0] = DFU_ERROR_NONE;
    hdfu->dev_status[1] = 0U;
    hdfu->dev_status[2] = 0U;
    hdfu->dev_status[3] = 0U; /*bwPollTimeout=0ms*/
    hdfu->dev_status[4] = hdfu->dev_state;
    hdfu->dev_status[5] = 0U; /*iString*/
    hdfu->wblock_num = 0U;
    hdfu->wlength = 0U;
  }

  /* Check the detach capability in the DFU functional descriptor */
  if ((pDfuFunc->bmAttributes & DFU_DETACH_MASK) != 0U)
  {
    /* Perform an Attach-Detach operation on USB bus */
    (void)USBD_Stop(pdev);
    (void)USBD_Start(pdev);
  }
  else
  {
    /* Wait for the period of time specified in Detach request */
    USBD_Delay((uint32_t)req->wValue);
  }
}

/**
  * @brief  DFU_Download
  *         Handles the DFU DNLOAD request.
  * @param  pdev: device instance
  * @param  req: pointer to the request structure
  * @retval None
  */
static void DFU_Download(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hdfu == NULL)
  {
    return;
  }

  /* Data setup request */
  if (req->wLength > 0U)
  {
    if ((hdfu->dev_state == DFU_STATE_IDLE) || (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE))
    {
      /* Update the global length and block number */
      hdfu->wblock_num = req->wValue;
      hdfu->wlength = MIN(req->wLength, USBD_DFU_XFER_SIZE);

      /* Update the state machine */
      hdfu->dev_state = DFU_STATE_DNLOAD_SYNC;
      hdfu->dev_status[4] = hdfu->dev_state;

      /* Prepare the reception of the buffer over EP0 */
      (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hdfu->buffer.d8, hdfu->wlength);
    }
    /* Unsupported state */
    else
    {
      /* Call the error management function (command will be NAKed */
      USBD_CtlError(pdev, req);
    }
  }
  /* 0 Data DNLOAD request */
  else
  {
    /* End of DNLOAD operation*/
    if ((hdfu->dev_state == DFU_STATE_DNLOAD_IDLE) || (hdfu->dev_state == DFU_STATE_IDLE))
    {
      hdfu->manif_state = DFU_MANIFEST_IN_PROGRESS;
      hdfu->dev_state = DFU_STATE_MANIFEST_SYNC;
      hdfu->dev_status[1] = 0U;
      hdfu->dev_status[2] = 0U;
      hdfu->dev_status[3] = 0U;
      hdfu->dev_status[4] = hdfu->dev_state;
    }
    else
    {
      /* Call the error management function (command will be NAKed */
      USBD_CtlError(pdev, req);
    }
  }
}

/**
  * @brief  DFU_Upload
  *         Handles the DFU UPLOAD request.
  * @param  pdev: instance
  * @param  req: pointer to the request structure
  * @retval status
  */
static void DFU_Upload(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_DFU_MediaTypeDef *DfuInterface = (USBD_DFU_MediaTypeDef *)pdev->pUserData[pdev->classId];
  uint8_t *phaddr;
  uint32_t addr;

  if (hdfu == NULL)
  {
    return;
  }

  /* Data setup request */
  if (req->wLength > 0U)
  {
    if ((hdfu->dev_state == DFU_STATE_IDLE) || (hdfu->dev_state == DFU_STATE_UPLOAD_IDLE))
    {
      /* Update the global length and block number */
      hdfu->wblock_num = req->wValue;
      hdfu->wlength = MIN(req->wLength, USBD_DFU_XFER_SIZE);

      /* DFU Get Command */
      if (hdfu->wblock_num == 0U)
      {
        /* Update the state machine */
        hdfu->dev_state = (hdfu->wlength > 3U) ? DFU_STATE_IDLE : DFU_STATE_UPLOAD_IDLE;

        hdfu->dev_status[1] = 0U;
        hdfu->dev_status[2] = 0U;
        hdfu->dev_status[3] = 0U;
        hdfu->dev_status[4] = hdfu->dev_state;

        /* Store the values of all supported commands */
        hdfu->buffer.d8[0] = DFU_CMD_GETCOMMANDS;
        hdfu->buffer.d8[1] = DFU_CMD_SETADDRESSPOINTER;
        hdfu->buffer.d8[2] = DFU_CMD_ERASE;

        /* Send the status data over EP0 */
        (void)USBD_CtlSendData(pdev, (uint8_t *)(&(hdfu->buffer.d8[0])), 3U);
      }
      else if (hdfu->wblock_num > 1U)
      {
        hdfu->dev_state = DFU_STATE_UPLOAD_IDLE;

        hdfu->dev_status[1] = 0U;
        hdfu->dev_status[2] = 0U;
        hdfu->dev_status[3] = 0U;
        hdfu->dev_status[4] = hdfu->dev_state;

        addr = ((hdfu->wblock_num - 2U) * USBD_DFU_XFER_SIZE) + hdfu->data_ptr;

        /* Return the physical address where data are stored */
        phaddr = DfuInterface->Read((uint8_t *)addr, hdfu->buffer.d8, hdfu->wlength);

        /* Send the status data over EP0 */
        (void)USBD_CtlSendData(pdev, phaddr, hdfu->wlength);
      }
      else  /* unsupported hdfu->wblock_num */
      {
        hdfu->dev_state = DFU_ERROR_STALLEDPKT;

        hdfu->dev_status[1] = 0U;
        hdfu->dev_status[2] = 0U;
        hdfu->dev_status[3] = 0U;
        hdfu->dev_status[4] = hdfu->dev_state;

        /* Call the error management function (command will be NAKed */
        USBD_CtlError(pdev, req);
      }
    }
    /* Unsupported state */
    else
    {
      hdfu->wlength = 0U;
      hdfu->wblock_num = 0U;

      /* Call the error management function (command will be NAKed */
      USBD_CtlError(pdev, req);
    }
  }
  /* No Data setup request */
  else
  {
    hdfu->dev_state = DFU_STATE_IDLE;

    hdfu->dev_status[1] = 0U;
    hdfu->dev_status[2] = 0U;
    hdfu->dev_status[3] = 0U;
    hdfu->dev_status[4] = hdfu->dev_state;
  }
}

/**
  * @brief  DFU_GetStatus
  *         Handles the DFU GETSTATUS request.
  * @param  pdev: instance
  * @retval status
  */
static void DFU_GetStatus(USBD_HandleTypeDef *pdev)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_DFU_MediaTypeDef *DfuInterface = (USBD_DFU_MediaTypeDef *)pdev->pUserData[pdev->classId];
  USBD_DFUFuncDescTypeDef *pDfuFunc = (USBD_DFUFuncDescTypeDef *)USBD_DFU_GetDfuFuncDesc(pdev->pConfDesc);

  if ((hdfu == NULL) || (DfuInterface == NULL) || (pDfuFunc == NULL))
  {
    return;
  }

  switch (hdfu->dev_state)
  {
    case DFU_STATE_DNLOAD_SYNC:
      if (hdfu->wlength != 0U)
      {
        hdfu->dev_state = DFU_STATE_DNLOAD_BUSY;

        hdfu->dev_status[1] = 0U;
        hdfu->dev_status[2] = 0U;
        hdfu->dev_status[3] = 0U;
        hdfu->dev_status[4] = hdfu->dev_state;

        if ((hdfu->wblock_num == 0U) && (hdfu->buffer.d8[0] == DFU_CMD_ERASE))
        {
          DfuInterface->GetStatus(hdfu->data_ptr, DFU_MEDIA_ERASE, hdfu->dev_status);
        }
        else
        {
          DfuInterface->GetStatus(hdfu->data_ptr, DFU_MEDIA_PROGRAM, hdfu->dev_status);
        }
      }
      else  /* (hdfu->wlength==0)*/
      {
        hdfu->dev_state = DFU_STATE_DNLOAD_IDLE;

        hdfu->dev_status[1] = 0U;
        hdfu->dev_status[2] = 0U;
        hdfu->dev_status[3] = 0U;
        hdfu->dev_status[4] = hdfu->dev_state;
      }
      break;

    case DFU_STATE_MANIFEST_SYNC:
      if (hdfu->manif_state == DFU_MANIFEST_IN_PROGRESS)
      {
        hdfu->dev_state = DFU_STATE_MANIFEST;

        hdfu->dev_status[1] = 1U;             /*bwPollTimeout = 1ms*/
        hdfu->dev_status[2] = 0U;
        hdfu->dev_status[3] = 0U;
        hdfu->dev_status[4] = hdfu->dev_state;
      }
      else
      {
        if ((hdfu->manif_state == DFU_MANIFEST_COMPLETE) &&
            ((pDfuFunc->bmAttributes & DFU_MANIFEST_MASK) != 0U))
        {
          hdfu->dev_state = DFU_STATE_IDLE;

          hdfu->dev_status[1] = 0U;
          hdfu->dev_status[2] = 0U;
          hdfu->dev_status[3] = 0U;
          hdfu->dev_status[4] = hdfu->dev_state;
        }
      }
      break;

    default:
      break;
  }

  /* Send the status data over EP0 */
  (void)USBD_CtlSendData(pdev, (uint8_t *)(&(hdfu->dev_status[0])), 6U);
}

/**
  * @brief  DFU_ClearStatus
  *         Handles the DFU CLRSTATUS request.
  * @param  pdev: device instance
  * @retval status
  */
static void DFU_ClearStatus(USBD_HandleTypeDef *pdev)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hdfu == NULL)
  {
    return;
  }

  if (hdfu->dev_state == DFU_STATE_ERROR)
  {
    hdfu->dev_state = DFU_STATE_IDLE;
    hdfu->dev_status[0] = DFU_ERROR_NONE; /* bStatus */
    hdfu->dev_status[1] = 0U;
    hdfu->dev_status[2] = 0U;
    hdfu->dev_status[3] = 0U; /* bwPollTimeout=0ms */
    hdfu->dev_status[4] = hdfu->dev_state; /* bState */
    hdfu->dev_status[5] = 0U; /* iString */
  }
  else
  {
    /* State Error */
    hdfu->dev_state = DFU_STATE_ERROR;
    hdfu->dev_status[0] = DFU_ERROR_UNKNOWN; /* bStatus */
    hdfu->dev_status[1] = 0U;
    hdfu->dev_status[2] = 0U;
    hdfu->dev_status[3] = 0U; /* bwPollTimeout=0ms */
    hdfu->dev_status[4] = hdfu->dev_state; /* bState */
    hdfu->dev_status[5] = 0U; /* iString */
  }
}

/**
  * @brief  DFU_GetState
  *         Handles the DFU GETSTATE request.
  * @param  pdev: device instance
  * @retval None
  */
static void DFU_GetState(USBD_HandleTypeDef *pdev)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hdfu == NULL)
  {
    return;
  }

  /* Return the current state of the DFU interface */
  (void)USBD_CtlSendData(pdev, &hdfu->dev_state, 1U);
}

/**
  * @brief  DFU_Abort
  *         Handles the DFU ABORT request.
  * @param  pdev: device instance
  * @retval None
  */
static void DFU_Abort(USBD_HandleTypeDef *pdev)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hdfu == NULL)
  {
    return;
  }

  if ((hdfu->dev_state == DFU_STATE_IDLE) ||
      (hdfu->dev_state == DFU_STATE_DNLOAD_SYNC) ||
      (hdfu->dev_state == DFU_STATE_DNLOAD_IDLE) ||
      (hdfu->dev_state == DFU_STATE_MANIFEST_SYNC) ||
      (hdfu->dev_state == DFU_STATE_UPLOAD_IDLE))
  {
    hdfu->dev_state = DFU_STATE_IDLE;
    hdfu->dev_status[0] = DFU_ERROR_NONE;
    hdfu->dev_status[1] = 0U;
    hdfu->dev_status[2] = 0U;
    hdfu->dev_status[3] = 0U; /* bwPollTimeout=0ms */
    hdfu->dev_status[4] = hdfu->dev_state;
    hdfu->dev_status[5] = 0U; /* iString */
    hdfu->wblock_num = 0U;
    hdfu->wlength = 0U;
  }
}

/**
  * @brief  DFU_Leave
  *         Handles the sub-protocol DFU leave DFU mode request (leaves DFU mode
  *         and resets device to jump to user loaded code).
  * @param  pdev: device instance
  * @retval None
  */
static void DFU_Leave(USBD_HandleTypeDef *pdev)
{
  USBD_DFU_HandleTypeDef *hdfu = (USBD_DFU_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_DFUFuncDescTypeDef *pDfuFunc = (USBD_DFUFuncDescTypeDef *)USBD_DFU_GetDfuFuncDesc(pdev->pConfDesc);

  if ((hdfu == NULL) || (pDfuFunc == NULL))
  {
    return;
  }

  hdfu->manif_state = DFU_MANIFEST_COMPLETE;

  if ((pDfuFunc->bmAttributes & DFU_MANIFEST_MASK) != 0U)
  {
    hdfu->dev_state = DFU_STATE_MANIFEST_SYNC;

    hdfu->dev_status[1] = 0U;
    hdfu->dev_status[2] = 0U;
    hdfu->dev_status[3] = 0U;
    hdfu->dev_status[4] = hdfu->dev_state;
    return;
  }
  else
  {
    hdfu->dev_state = DFU_STATE_MANIFEST_WAIT_RESET;

    hdfu->dev_status[1] = 0U;
    hdfu->dev_status[2] = 0U;
    hdfu->dev_status[3] = 0U;
    hdfu->dev_status[4] = hdfu->dev_state;

    /* Disconnect the USB device */
    (void)USBD_Stop(pdev);

    /* Generate system reset to allow jumping to the user code */
    NVIC_SystemReset();

    /* The next instructions will not be reached (system reset) */
  }
}

/**
  * @brief  USBD_DFU_GetDfuFuncDesc
  *         This function return the DFU descriptor
  * @param  pdev: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @retval pointer to the DFU descriptor
  */
static void *USBD_DFU_GetDfuFuncDesc(uint8_t *pConfDesc)
{
  USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
  USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
  uint8_t *pDfuDesc = NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDescriptorType == DFU_DESCRIPTOR_TYPE)
      {
        pDfuDesc = (uint8_t *)pdesc;
        break;
      }
    }
  }
  return pDfuDesc;
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

