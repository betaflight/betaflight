/**
  ******************************************************************************
  * @file    usbd_billboard.c
  * @author  MCD Application Team
  * @brief   This file provides the high layer firmware functions to manage the
  *          following functionalities of the USB BillBoard Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as BillBoard Device
  *           - Error management
 * @verbatim
  *
  *          ===================================================================
  *                                BillBoard Class Description
  *          ===================================================================
  *           This module manages the BillBoard class V1.2.1 following the "Device Class Definition
  *           for BillBoard Devices (BB) Version R1.2.1 Sept 08, 2016".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as an USB BillBoard device
  *             - Enumeration & management of BillBoard device supported alternate modes
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_billboard.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_BB
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_BB_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_BB_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_BB_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_BB_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_BB_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_BB_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_BB_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_BB_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_BB_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_BB_EP0_RxReady(USBD_HandleTypeDef *pdev);

static uint8_t *USBD_BB_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_BB_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t *USBD_BB_GetOtherSpeedCfgDesc(uint16_t *length);

#if (USBD_CLASS_BOS_ENABLED == 1)
USBD_BB_DescHeader_t *USBD_BB_GetNextDesc(uint8_t *pbuf, uint16_t *ptr);
#endif



/**
  * @}
  */

/** @defgroup USBD_BB_Private_Variables
  * @{
  */
USBD_ClassTypeDef  USBD_BB =
{
  USBD_BB_Init,             /* Init */
  USBD_BB_DeInit,           /* DeInit */
  USBD_BB_Setup,            /* Setup */
  NULL,                     /* EP0_TxSent */
  USBD_BB_EP0_RxReady,      /* EP0_RxReady */
  USBD_BB_DataIn,           /* DataIn */
  USBD_BB_DataOut,          /* DataOut */
  NULL,                     /* SOF */
  NULL,
  NULL,
  USBD_BB_GetCfgDesc,
  USBD_BB_GetCfgDesc,
  USBD_BB_GetOtherSpeedCfgDesc,
  USBD_BB_GetDeviceQualifierDesc,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  NULL,
#endif
};

/* USB Standard Device Qualifier Descriptor */
__ALIGN_BEGIN static uint8_t USBD_BB_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC]  __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,     /* bLength */
  USB_DESC_TYPE_DEVICE_QUALIFIER, /* bDescriptorType */
  0x01,                           /* bcdUSB */
  0x20,
  0x11,                           /* bDeviceClass */
  0x00,                           /* bDeviceSubClass */
  0x00,                           /* bDeviceProtocol */
  0x40,                           /* bMaxPacketSize0 */
  0x01,                           /* bNumConfigurations */
  0x00,                           /* bReserved */
};

/* USB device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_BB_CfgDesc[USB_BB_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  0x09,                        /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_BB_CONFIG_DESC_SIZ,      /* wTotalLength: Bytes returned */
  0x00,
  0x01,                        /* bNumInterfaces: 1 interface */
  0x01,                        /* bConfigurationValue: Configuration value */
  USBD_IDX_CONFIG_STR,         /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,                        /* bmAttributes: bus powered and Support Remote Wake-up */
  0x00,                        /* MaxPower 100 mA: this current is used for detecting Vbus */
  /* 09 */

  /************** Descriptor of BillBoard interface ****************/
  /* 09 */
  0x09,                        /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,     /* bDescriptorType: Interface descriptor type */
  0x00,                        /* bInterfaceNumber: Number of Interface */
  0x00,                        /* bAlternateSetting: Alternate setting */
  0x00,                        /* bNumEndpoints */
  0x11,                        /* bInterfaceClass: billboard */
  0x00,                        /* bInterfaceSubClass */
  0x00,                        /* nInterfaceProtocol */
  USBD_BB_IF_STRING_INDEX,     /* iInterface: Index of string descriptor */
};

/* USB device Other Speed Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_BB_OtherSpeedCfgDesc[USB_BB_CONFIG_DESC_SIZ]   __ALIGN_END  =
{
  0x09,                        /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,
  USB_BB_CONFIG_DESC_SIZ,
  0x00,
  0x01,                        /* bNumInterfaces: 1 interface */
  0x01,                        /* bConfigurationValue: */
  USBD_IDX_CONFIG_STR,         /* iConfiguration: */
  0xC0,                        /* bmAttributes: */
  0x00,                        /* MaxPower 100 mA */

  /************** Descriptor of BillBoard interface ****************/
  /* 09 */
  0x09,                        /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,     /* bDescriptorType: Interface descriptor type */
  0x00,                        /* bInterfaceNumber: Number of Interface */
  0x00,                        /* bAlternateSetting: Alternate setting */
  0x00,                        /* bNumEndpoints*/
  0x11,                        /* bInterfaceClass: billboard */
  0x00,                        /* bInterfaceSubClass */
  0x00,                        /* nInterfaceProtocol */
  USBD_BB_IF_STRING_INDEX,     /* iInterface: Index of string descriptor */
} ;

/**
  * @}
  */

/** @defgroup USBD_BB_Private_Functions
  * @{
  */

/**
  * @brief  USBD_BB_Init
  *         Initialize the BB interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_BB_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(cfgidx);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_BB_Init
  *         DeInitialize the BB layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_BB_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(cfgidx);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_BB_Setup
  *         Handle the BB specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_BB_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t status_info = 0U;
  uint16_t AltSetting = 0U;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS:
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

    case USB_REQ_GET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        (void)USBD_CtlSendData(pdev, (uint8_t *)&AltSetting, 1U);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE:
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

/**
  * @brief  USBD_BB_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_BB_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(epnum);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_BB_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_BB_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);
  UNUSED(epnum);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_BB_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_BB_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  /* Prevent unused argument compilation warning */
  UNUSED(pdev);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_BB_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_BB_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_BB_CfgDesc);
  return USBD_BB_CfgDesc;
}

/**
  * @brief  USBD_BB_GetOtherSpeedCfgDesc
  *         return other speed configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_BB_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_BB_OtherSpeedCfgDesc);
  return USBD_BB_OtherSpeedCfgDesc;
}

/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_BB_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_BB_DeviceQualifierDesc);
  return USBD_BB_DeviceQualifierDesc;
}


#if (USBD_CLASS_BOS_ENABLED == 1U)
/**
  * @brief  USBD_BB_GetNextDesc
  *         This function return the next descriptor header
  * @param  buf: Buffer where the descriptor is available
  * @param  ptr: data pointer inside the descriptor
  * @retval next header
  */
USBD_BB_DescHeader_t *USBD_BB_GetNextDesc(uint8_t *pbuf, uint16_t *ptr)
{
  USBD_BB_DescHeader_t *pnext = (USBD_BB_DescHeader_t *)pbuf;

  *ptr += pnext->bLength;
  pnext = (USBD_BB_DescHeader_t *)(pbuf + pnext->bLength);

  return (pnext);
}


/**
  * @brief  USBD_BB_GetCapDesc
  *         This function return the Billboard Capability descriptor
  * @param  pdev: device instance
  * @param  pBosDesc:  pointer to Bos descriptor
  * @retval pointer to Billboard Capability descriptor
  */
void *USBD_BB_GetCapDesc(USBD_HandleTypeDef *pdev, uint8_t *pBosDesc)
{
  UNUSED(pdev);

  USBD_BB_DescHeader_t *pdesc = (USBD_BB_DescHeader_t *)pBosDesc;
  USBD_BosDescTypedef *desc = (USBD_BosDescTypedef *)pBosDesc;
  USBD_BosBBCapDescTypedef *pCapDesc = NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_BB_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDevCapabilityType == USBD_BILLBOARD_CAPABILITY)
      {
        pCapDesc = (USBD_BosBBCapDescTypedef *)pdesc;
        break;
      }
    }
  }
  return (void *)pCapDesc;
}


/**
  * @brief  USBD_BB_GetAltModeDesc
  *         This function return the Billboard Alternate Mode descriptor
  * @param  pdev: device instance
  * @param  pBosDesc:  pointer to Bos descriptor
  * @param  idx:  Index of requested Alternate Mode descriptor
  * @retval pointer to Alternate Mode descriptor
  */
void *USBD_BB_GetAltModeDesc(USBD_HandleTypeDef *pdev, uint8_t *pBosDesc, uint8_t idx)
{
  UNUSED(pdev);

  USBD_BB_DescHeader_t *pdesc = (USBD_BB_DescHeader_t *)pBosDesc;
  USBD_BosDescTypedef *desc = (USBD_BosDescTypedef *)pBosDesc;
  USBD_BB_AltModeCapDescTypeDef *pAltModDesc = NULL;
  uint8_t cnt = 0U;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_BB_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDevCapabilityType == USBD_BILLBOARD_ALTMODE_CAPABILITY)
      {
        if (cnt == idx)
        {
          pAltModDesc = (USBD_BB_AltModeCapDescTypeDef *)pdesc;
          break;
        }
        else
        {
          cnt++;
        }
      }
    }
  }
  return (void *)pAltModDesc;
}
#endif

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
