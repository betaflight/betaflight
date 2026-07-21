/**
  ******************************************************************************
  * @file    usbd_cdc_ecm.c
  * @author  MCD Application Team
  * @brief   This file provides the high layer firmware functions to manage the
  *          following functionalities of the USB CDC_ECM Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC_ECM Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
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

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_ecm.h"
#include "usbd_ctlreq.h"

#ifndef __USBD_CDC_ECM_IF_H
#include "usbd_cdc_ecm_if_template.h"
#endif


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC_ECM
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_ECM_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CDC_ECM_Private_Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_CDC_ECM_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CDC_ECM_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_CDC_ECM_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_ECM_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_ECM_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_ECM_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_ECM_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_CDC_ECM_Setup(USBD_HandleTypeDef *pdev,
                                  USBD_SetupReqTypedef *req);

static uint8_t *USBD_CDC_ECM_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_ECM_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_ECM_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_ECM_GetOtherSpeedCfgDesc(uint16_t *length);

#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *USBD_CDC_ECM_USRStringDescriptor(USBD_HandleTypeDef *pdev,
                                                 uint8_t index, uint16_t *length);
#endif

uint8_t *USBD_CDC_ECM_GetDeviceQualifierDescriptor(uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_ECM_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

static uint32_t ConnSpeedTab[2] = {CDC_ECM_CONNECT_SPEED_UPSTREAM,
                                   CDC_ECM_CONNECT_SPEED_DOWNSTREAM};

/**
  * @}
  */

/** @defgroup USBD_CDC_ECM_Private_Variables
  * @{
  */


/* CDC_ECM interface class callbacks structure */
USBD_ClassTypeDef USBD_CDC_ECM =
{
  USBD_CDC_ECM_Init,
  USBD_CDC_ECM_DeInit,
  USBD_CDC_ECM_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_CDC_ECM_EP0_RxReady,
  USBD_CDC_ECM_DataIn,
  USBD_CDC_ECM_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_CDC_ECM_GetHSCfgDesc,
  USBD_CDC_ECM_GetFSCfgDesc,
  USBD_CDC_ECM_GetOtherSpeedCfgDesc,
  USBD_CDC_ECM_GetDeviceQualifierDescriptor,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  USBD_CDC_ECM_USRStringDescriptor,
#endif
};

/* USB CDC_ECM device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_ECM_CfgHSDesc[] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                     /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,              /* bDescriptorType: Configuration */
  LOBYTE(CDC_ECM_CONFIG_DESC_SIZ),          /* wTotalLength:no of returned bytes */
  HIBYTE(CDC_ECM_CONFIG_DESC_SIZ),
  0x02,                                     /* bNumInterfaces: 2 interface */
  0x01,                                     /* bConfigurationValue: Configuration value */
  0x00,                                     /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,                                     /* bmAttributes: self powered */
  0x32,                                     /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/

  /* IAD descriptor */
  0x08,                                     /* bLength */
  0x0B,                                     /* bDescriptorType */
  0x00,                                     /* bFirstInterface */
  0x02,                                     /* bInterfaceCount */
  0x02,                                     /* bFunctionClass (Wireless Controller) */
  0x06,                                     /* bFunctionSubClass */
  0x00,                                     /* bFunctionProtocol */
  0x00,                                     /* iFunction */

  /* Interface Descriptor */
  0x09,                                     /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                  /* bDescriptorType: Interface descriptor type */
  CDC_ECM_CMD_ITF_NBR,                      /* bInterfaceNumber: Number of Interface */
  0x00,                                     /* bAlternateSetting: Alternate setting */
  0x01,                                     /* bNumEndpoints: One endpoint used */
  0x02,                                     /* bInterfaceClass: Communication Interface Class */
  0x06,                                     /* bInterfaceSubClass: Ethernet Control Model */
  0x00,                                     /* bInterfaceProtocol: No specific protocol required */
  0x00,                                     /* iInterface: */

  /* Header Functional Descriptor */
  0x05,                                     /* bLength: Endpoint Descriptor size */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x00,                                     /* bDescriptorSubtype: Header functional descriptor */
  0x10,                                     /* bcd CDC_ECM: spec release number: 1.10 */
  0x01,

  /* CDC_ECM Functional Descriptor */
  0x0D,                                     /* bFunctionLength */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x0F,                                     /* Ethernet Networking functional descriptor subtype  */
  CDC_ECM_MAC_STRING_INDEX,                 /* Device's MAC string index */
  CDC_ECM_ETH_STATS_BYTE3,                  /* Ethernet statistics byte 3 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE2,                  /* Ethernet statistics byte 2 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE1,                  /* Ethernet statistics byte 1 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE0,                  /* Ethernet statistics byte 0 (bitmap) */
  LOBYTE(CDC_ECM_ETH_MAX_SEGSZE),
  HIBYTE(CDC_ECM_ETH_MAX_SEGSZE),           /* wMaxSegmentSize: Ethernet Maximum Segment size, typically 1514 bytes */
  LOBYTE(CDC_ECM_ETH_NBR_MACFILTERS),
  HIBYTE(CDC_ECM_ETH_NBR_MACFILTERS),       /* wNumberMCFilters: the number of multicast filters */
  CDC_ECM_ETH_NBR_PWRFILTERS,               /* bNumberPowerFilters: the number of wakeup power filters */

  /* Union Functional Descriptor */
  0x05,                                     /* bFunctionLength */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x06,                                     /* bDescriptorSubtype: Union functional descriptor */
  0x00,                                     /* bMasterInterface: Communication class interface */
  0x01,                                     /* bSlaveInterface0: Data Class Interface */

  /* Communication Endpoint Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_CMD_EP,                           /* bEndpointAddress */
  0x03,                                     /* bmAttributes: Interrupt */
  LOBYTE(CDC_ECM_CMD_PACKET_SIZE),          /* wMaxPacketSize: */
  HIBYTE(CDC_ECM_CMD_PACKET_SIZE),
  CDC_ECM_HS_BINTERVAL,                     /* bInterval */

  /*----------------------*/

  /* Data class interface descriptor */
  0x09,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                  /* bDescriptorType: */
  CDC_ECM_COM_ITF_NBR,                      /* bInterfaceNumber: Number of Interface */
  0x00,                                     /* bAlternateSetting: Alternate setting */
  0x02,                                     /* bNumEndpoints: Two endpoints used */
  0x0A,                                     /* bInterfaceClass: CDC */
  0x00,                                     /* bInterfaceSubClass: */
  0x00,                                     /* bInterfaceProtocol: */
  0x00,                                     /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_OUT_EP,                           /* bEndpointAddress */
  0x02,                                     /* bmAttributes: Bulk */
  LOBYTE(CDC_ECM_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_ECM_DATA_HS_MAX_PACKET_SIZE),
  0xFF,                                     /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_IN_EP,                            /* bEndpointAddress */
  0x02,                                     /* bmAttributes: Bulk */
  LOBYTE(CDC_ECM_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_ECM_DATA_HS_MAX_PACKET_SIZE),
  0xFF                                      /* bInterval: ignore for Bulk transfer */
};


/* USB CDC_ECM device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_CDC_ECM_CfgFSDesc[] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                     /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,              /* bDescriptorType: Configuration */
  LOBYTE(CDC_ECM_CONFIG_DESC_SIZ),          /* wTotalLength: Total size of the Config descriptor */
  HIBYTE(CDC_ECM_CONFIG_DESC_SIZ),
  0x02,                                     /* bNumInterfaces: 2 interface */
  0x01,                                     /* bConfigurationValue: Configuration value */
  0x00,                                     /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,                                     /* bmAttributes: self powered */
  0x32,                                     /* MaxPower 0 mA */

  /*---------------------------------------------------------------------------*/
  /* IAD descriptor */
  0x08,                                     /* bLength */
  0x0B,                                     /* bDescriptorType */
  0x00,                                     /* bFirstInterface */
  0x02,                                     /* bInterfaceCount */
  0x02,                                     /* bFunctionClass (Wireless Controller) */
  0x06,                                     /* bFunctionSubClass */
  0x00,                                     /* bFunctionProtocol */
  0x00,                                     /* iFunction */

  /* Interface Descriptor */
  0x09,                                     /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                  /* bDescriptorType: Interface descriptor type */
  CDC_ECM_CMD_ITF_NBR,                      /* bInterfaceNumber: Number of Interface */
  0x00,                                     /* bAlternateSetting: Alternate setting */
  0x01,                                     /* bNumEndpoints: One endpoint used */
  0x02,                                     /* bInterfaceClass: Communication Interface Class */
  0x06,                                     /* bInterfaceSubClass: Ethernet Control Model */
  0x00,                                     /* bInterfaceProtocol: No specific protocol required */
  0x00,                                     /* iInterface: */

  /* Header Functional Descriptor */
  0x05,                                     /* bLength: Endpoint Descriptor size */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x00,                                     /* bDescriptorSubtype: Header functional descriptor */
  0x10,                                     /* bcd CDC_ECM : spec release number: 1.20 */
  0x01,

  /* Union Functional Descriptor */
  0x05,                                     /* bFunctionLength */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x06,                                     /* bDescriptorSubtype: Union functional descriptor */
  CDC_ECM_CMD_ITF_NBR,                      /* bMasterInterface: Communication class interface */
  CDC_ECM_COM_ITF_NBR,                      /* bSlaveInterface0: Data Class Interface */

  /* CDC_ECM Functional Descriptor */
  0x0D,                                     /* bFunctionLength */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x0F,                                     /* Ethernet Networking functional descriptor subtype  */
  CDC_ECM_MAC_STRING_INDEX,                 /* Device's MAC string index */
  CDC_ECM_ETH_STATS_BYTE3,                  /* Ethernet statistics byte 3 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE2,                  /* Ethernet statistics byte 2 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE1,                  /* Ethernet statistics byte 1 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE0,                  /* Ethernet statistics byte 0 (bitmap) */
  LOBYTE(CDC_ECM_ETH_MAX_SEGSZE),
  HIBYTE(CDC_ECM_ETH_MAX_SEGSZE),           /* wMaxSegmentSize: Ethernet Maximum Segment size, typically 1514 bytes */
  LOBYTE(CDC_ECM_ETH_NBR_MACFILTERS),
  HIBYTE(CDC_ECM_ETH_NBR_MACFILTERS),       /* wNumberMCFilters: the number of multicast filters */
  CDC_ECM_ETH_NBR_PWRFILTERS,               /* bNumberPowerFilters: the number of wakeup power filters */


  /* Communication Endpoint Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_CMD_EP,                           /* bEndpointAddress */
  0x03,                                     /* bmAttributes: Interrupt */
  LOBYTE(CDC_ECM_CMD_PACKET_SIZE),          /* wMaxPacketSize: */
  HIBYTE(CDC_ECM_CMD_PACKET_SIZE),
  CDC_ECM_FS_BINTERVAL,                     /* bInterval */

  /*----------------------*/

  /* Data class interface descriptor */
  0x09,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                  /* bDescriptorType: */
  CDC_ECM_COM_ITF_NBR,                      /* bInterfaceNumber: Number of Interface */
  0x00,                                     /* bAlternateSetting: Alternate setting */
  0x02,                                     /* bNumEndpoints: Two endpoints used */
  0x0A,                                     /* bInterfaceClass: CDC_ECM */
  0x00,                                     /* bInterfaceSubClass: */
  0x00,                                     /* bInterfaceProtocol: */
  0x00,                                     /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_OUT_EP,                           /* bEndpointAddress */
  0x02,                                     /* bmAttributes: Bulk */
  LOBYTE(CDC_ECM_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_ECM_DATA_FS_MAX_PACKET_SIZE),
  0xFF,                                     /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_IN_EP,                            /* bEndpointAddress */
  0x02,                                     /* bmAttributes: Bulk */
  LOBYTE(CDC_ECM_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_ECM_DATA_FS_MAX_PACKET_SIZE),
  0xFF                                      /* bInterval: ignore for Bulk transfer */
} ;

__ALIGN_BEGIN static uint8_t USBD_CDC_ECM_OtherSpeedCfgDesc[] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                     /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,              /* bDescriptorType: Configuration */
  LOBYTE(CDC_ECM_CONFIG_DESC_SIZ),          /* wTotalLength:no of returned bytes */
  HIBYTE(CDC_ECM_CONFIG_DESC_SIZ),
  0x02,                                     /* bNumInterfaces: 2 interface */
  0x01,                                     /* bConfigurationValue: Configuration value */
  0x04,                                     /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,                                     /* bmAttributes: self powered */
  0x32,                                     /* MaxPower 0 mA */

  /*--------------------------------------- ------------------------------------*/
  /* IAD descriptor */
  0x08,                                     /* bLength */
  0x0B,                                     /* bDescriptorType */
  0x00,                                     /* bFirstInterface */
  0x02,                                     /* bInterfaceCount */
  0x02,                                     /* bFunctionClass (Wireless Controller) */
  0x06,                                     /* bFunctionSubClass */
  0x00,                                     /* bFunctionProtocol */
  0x00,                                     /* iFunction */

  /* Interface Descriptor */
  0x09,                                     /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                  /* bDescriptorType: Interface descriptor type */
  0x00,                                     /* bInterfaceNumber: Number of Interface */
  0x00,                                     /* bAlternateSetting: Alternate setting */
  0x01,                                     /* bNumEndpoints: One endpoint used */
  0x02,                                     /* bInterfaceClass: Communication Interface Class */
  0x06,                                     /* bInterfaceSubClass: Ethernet Control Model */
  0x00,                                     /* bInterfaceProtocol: No specific protocol required */
  0x00,                                     /* iInterface: */

  /* Header Functional Descriptor */
  0x05,                                     /* bLength: Endpoint Descriptor size */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x00,                                     /* bDescriptorSubtype: Header functional descriptor */
  0x10,                                     /* bcd CDC_ECM : spec release number: 1.20 */
  0x01,

  /* CDC_ECM Functional Descriptor */
  0x0D,                                     /* bFunctionLength */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x0F,                                     /* Ethernet Networking functional descriptor subtype  */
  CDC_ECM_MAC_STRING_INDEX,                 /* Device's MAC string index */
  CDC_ECM_ETH_STATS_BYTE3,                  /* Ethernet statistics byte 3 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE2,                  /* Ethernet statistics byte 2 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE1,                  /* Ethernet statistics byte 1 (bitmap) */
  CDC_ECM_ETH_STATS_BYTE0,                  /* Ethernet statistics byte 0 (bitmap) */
  LOBYTE(CDC_ECM_ETH_MAX_SEGSZE),
  HIBYTE(CDC_ECM_ETH_MAX_SEGSZE),           /* wMaxSegmentSize: Ethernet Maximum Segment size, typically 1514 bytes */
  LOBYTE(CDC_ECM_ETH_NBR_MACFILTERS),
  HIBYTE(CDC_ECM_ETH_NBR_MACFILTERS),       /* wNumberMCFilters: the number of multicast filters */
  CDC_ECM_ETH_NBR_PWRFILTERS,               /* bNumberPowerFilters: the number of wakeup power filters */

  /* Union Functional Descriptor */
  0x05,                                     /* bFunctionLength */
  0x24,                                     /* bDescriptorType: CS_INTERFACE */
  0x06,                                     /* bDescriptorSubtype: Union functional descriptor */
  0x00,                                     /* bMasterInterface: Communication class interface */
  0x01,                                     /* bSlaveInterface0: Data Class Interface */

  /* Communication Endpoint Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_CMD_EP,                           /* bEndpointAddress */
  0x03,                                     /* bmAttributes: Interrupt */
  LOBYTE(CDC_ECM_CMD_PACKET_SIZE),          /* wMaxPacketSize: */
  HIBYTE(CDC_ECM_CMD_PACKET_SIZE),
  CDC_ECM_FS_BINTERVAL,                     /* bInterval */

  /*----------------------*/

  /* Data class interface descriptor */
  0x09,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                  /* bDescriptorType: */
  0x01,                                     /* bInterfaceNumber: Number of Interface */
  0x00,                                     /* bAlternateSetting: Alternate setting */
  0x02,                                     /* bNumEndpoints: Two endpoints used */
  0x0A,                                     /* bInterfaceClass: CDC */
  0x00,                                     /* bInterfaceSubClass: */
  0x00,                                     /* bInterfaceProtocol: */
  0x00,                                     /* iInterface: */

  /* Endpoint OUT Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_OUT_EP,                           /* bEndpointAddress */
  0x02,                                     /* bmAttributes: Bulk */
  0x40,                                     /* wMaxPacketSize: */
  0x00,
  0xFF,                                     /* bInterval: ignore for Bulk transfer */

  /* Endpoint IN Descriptor */
  0x07,                                     /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                   /* bDescriptorType: Endpoint */
  CDC_ECM_IN_EP,                            /* bEndpointAddress */
  0x02,                                     /* bmAttributes: Bulk */
  0x40,                                     /* wMaxPacketSize: */
  0x00,
  0xFF                                      /* bInterval: ignore for Bulk transfer */
};

/**
  * @}
  */

/** @defgroup USBD_CDC_ECM_Private_Functions
  * @{
  */

/**
  * @brief  USBD_CDC_ECM_Init
  *         Initialize the CDC_ECM interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_ECM_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_CDC_ECM_HandleTypeDef *hcdc;

  hcdc = USBD_malloc(sizeof(USBD_CDC_ECM_HandleTypeDef));

  if (hcdc == NULL)
  {
    pdev->pClassData = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassData = (void *)hcdc;

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, CDC_ECM_IN_EP, USBD_EP_TYPE_BULK,
                         CDC_ECM_DATA_HS_IN_PACKET_SIZE);

    pdev->ep_in[CDC_ECM_IN_EP & 0xFU].is_used = 1U;

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, CDC_ECM_OUT_EP, USBD_EP_TYPE_BULK,
                         CDC_ECM_DATA_HS_OUT_PACKET_SIZE);

    pdev->ep_out[CDC_ECM_OUT_EP & 0xFU].is_used = 1U;

    /* Set bInterval for CDC ECM CMD Endpoint */
    pdev->ep_in[CDC_ECM_CMD_EP & 0xFU].bInterval = CDC_ECM_HS_BINTERVAL;
  }
  else
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, CDC_ECM_IN_EP, USBD_EP_TYPE_BULK,
                         CDC_ECM_DATA_FS_IN_PACKET_SIZE);

    pdev->ep_in[CDC_ECM_IN_EP & 0xFU].is_used = 1U;

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, CDC_ECM_OUT_EP, USBD_EP_TYPE_BULK,
                         CDC_ECM_DATA_FS_OUT_PACKET_SIZE);

    pdev->ep_out[CDC_ECM_OUT_EP & 0xFU].is_used = 1U;

    /* Set bInterval for CDC ECM CMD Endpoint */
    pdev->ep_in[CDC_ECM_CMD_EP & 0xFU].bInterval = CDC_ECM_FS_BINTERVAL;
  }

  /* Open Command IN EP */
  (void)USBD_LL_OpenEP(pdev, CDC_ECM_CMD_EP, USBD_EP_TYPE_INTR, CDC_ECM_CMD_PACKET_SIZE);
  pdev->ep_in[CDC_ECM_CMD_EP & 0xFU].is_used = 1U;

  /* Init  physical Interface components */
  ((USBD_CDC_ECM_ItfTypeDef *)pdev->pUserData)->Init();

  /* Init Xfer states */
  hcdc->TxState = 0U;
  hcdc->RxState = 0U;
  hcdc->RxLength = 0U;
  hcdc->TxLength = 0U;
  hcdc->LinkStatus = 0U;
  hcdc->NotificationStatus = 0U;
  hcdc->MaxPcktLen = (pdev->dev_speed == USBD_SPEED_HIGH) ? CDC_ECM_DATA_HS_MAX_PACKET_SIZE : CDC_ECM_DATA_FS_MAX_PACKET_SIZE;

  /* Prepare Out endpoint to receive next packet */
  (void)USBD_LL_PrepareReceive(pdev, CDC_ECM_OUT_EP, hcdc->RxBuffer, hcdc->MaxPcktLen);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_ECM_DeInit
  *         DeInitialize the CDC_ECM layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_ECM_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, CDC_ECM_IN_EP);
  pdev->ep_in[CDC_ECM_IN_EP & 0xFU].is_used = 0U;

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, CDC_ECM_OUT_EP);
  pdev->ep_out[CDC_ECM_OUT_EP & 0xFU].is_used = 0U;

  /* Close Command IN EP */
  (void)USBD_LL_CloseEP(pdev, CDC_ECM_CMD_EP);
  pdev->ep_in[CDC_ECM_CMD_EP & 0xFU].is_used = 0U;
  pdev->ep_in[CDC_ECM_CMD_EP & 0xFU].bInterval = 0U;

  /* DeInit  physical Interface components */
  if (pdev->pClassData != NULL)
  {
    ((USBD_CDC_ECM_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_ECM_Setup
  *         Handle the CDC_ECM specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_CDC_ECM_Setup(USBD_HandleTypeDef *pdev,
                                  USBD_SetupReqTypedef *req)
{
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *) pdev->pClassData;
  USBD_CDC_ECM_ItfTypeDef *EcmInterface = (USBD_CDC_ECM_ItfTypeDef *)pdev->pUserData;
  USBD_StatusTypeDef ret = USBD_OK;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength != 0U)
    {
      if ((req->bmRequest & 0x80U) != 0U)
      {
        EcmInterface->Control(req->bRequest,
                              (uint8_t *)hcdc->data, req->wLength);

        (void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->data, req->wLength);
      }
      else
      {
        hcdc->CmdOpCode = req->bRequest;
        hcdc->CmdLength = (uint8_t)req->wLength;

        (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->data, req->wLength);
      }
    }
    else
    {
      EcmInterface->Control(req->bRequest, (uint8_t *)req, 0U);
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

    case USB_REQ_GET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        (void)USBD_CtlSendData(pdev, &ifalt, 1U);
      }
      else
      {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE:
      if (pdev->dev_state != USBD_STATE_CONFIGURED)
      {
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

/**
  * @brief  USBD_CDC_ECM_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_ECM_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *)pdev->pClassData;
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (epnum == (CDC_ECM_IN_EP & 0x7FU))
  {
    if ((pdev->ep_in[epnum].total_length > 0U) &&
        ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))
    {
      /* Update the packet total length */
      pdev->ep_in[epnum].total_length = 0U;

      /* Send ZLP */
      (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
    }
    else
    {
      hcdc->TxState = 0U;
      ((USBD_CDC_ECM_ItfTypeDef *)pdev->pUserData)->TransmitCplt(hcdc->TxBuffer, &hcdc->TxLength, epnum);
    }
  }
  else if (epnum == (CDC_ECM_CMD_EP & 0x7FU))
  {
    if (hcdc->NotificationStatus != 0U)
    {
      (void)USBD_CDC_ECM_SendNotification(pdev, CONNECTION_SPEED_CHANGE,
                                          0U, (uint8_t *)ConnSpeedTab);

      hcdc->NotificationStatus = 0U;
    }
  }
  else
  {
    return (uint8_t)USBD_FAIL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_ECM_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_ECM_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *)pdev->pClassData;
  uint32_t CurrPcktLen;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (epnum == CDC_ECM_OUT_EP)
  {
    /* Get the received data length */
    CurrPcktLen = USBD_LL_GetRxDataSize(pdev, epnum);

    /* Increment the frame length */
    hcdc->RxLength += CurrPcktLen;

    /* If the buffer size is less than max packet size: it is the last packet in current frame */
    if ((CurrPcktLen < hcdc->MaxPcktLen) || (hcdc->RxLength >= CDC_ECM_ETH_MAX_SEGSZE))
    {
      /* USB data will be immediately processed, this allow next USB traffic being
      NACKed till the end of the application Xfer */

      /* Process data by application (ie. copy to app buffer or notify user)
      hcdc->RxLength must be reset to zero at the end of the call of this function */
      ((USBD_CDC_ECM_ItfTypeDef *)pdev->pUserData)->Receive(hcdc->RxBuffer, &hcdc->RxLength);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet in current/new frame */
      (void)USBD_LL_PrepareReceive(pdev, CDC_ECM_OUT_EP,
                                   (uint8_t *)(hcdc->RxBuffer + hcdc->RxLength),
                                   hcdc->MaxPcktLen);
    }
  }
  else
  {
    return (uint8_t)USBD_FAIL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_ECM_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_CDC_ECM_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *)pdev->pClassData;

  if ((pdev->pUserData != NULL) && (hcdc->CmdOpCode != 0xFFU))
  {
    ((USBD_CDC_ECM_ItfTypeDef *)pdev->pUserData)->Control(hcdc->CmdOpCode,
                                                          (uint8_t *)hcdc->data,
                                                          (uint16_t)hcdc->CmdLength);
    hcdc->CmdOpCode = 0xFFU;

  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_ECM_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_ECM_GetFSCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_ECM_CfgFSDesc);

  return USBD_CDC_ECM_CfgFSDesc;
}

/**
  * @brief  USBD_CDC_ECM_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_ECM_GetHSCfgDesc(uint16_t *length)
{
  *length = (uint16_t) sizeof(USBD_CDC_ECM_CfgHSDesc);

  return USBD_CDC_ECM_CfgHSDesc;
}

/**
  * @brief  USBD_CDC_ECM_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_ECM_GetOtherSpeedCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_ECM_OtherSpeedCfgDesc);

  return USBD_CDC_ECM_OtherSpeedCfgDesc;
}

/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_ECM_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_CDC_ECM_DeviceQualifierDesc);

  return USBD_CDC_ECM_DeviceQualifierDesc;
}

/**
  * @brief  USBD_CDC_ECM_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t USBD_CDC_ECM_RegisterInterface(USBD_HandleTypeDef *pdev,
                                       USBD_CDC_ECM_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData = fops;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_ECM_USRStringDescriptor
  *         Manages the transfer of user string descriptors.
  * @param  speed : current device speed
  * @param  index: descriptor index
  * @param  length : pointer data length
  * @retval pointer to the descriptor table or NULL if the descriptor is not supported.
  */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *USBD_CDC_ECM_USRStringDescriptor(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
  static uint8_t USBD_StrDesc[255];

  /* Check if the requested string interface is supported */
  if (index == CDC_ECM_MAC_STRING_INDEX)
  {
    USBD_GetString((uint8_t *)((USBD_CDC_ECM_ItfTypeDef *)pdev->pUserData)->pStrDesc, USBD_StrDesc, length);
    return USBD_StrDesc;
  }
  /* Not supported Interface Descriptor index */
  else
  {
    return NULL;
  }
}
#endif

/**
  * @brief  USBD_CDC_ECM_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t USBD_CDC_ECM_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint32_t length)
{
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *)pdev->pClassData;

  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_ECM_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t USBD_CDC_ECM_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *)pdev->pClassData;

  hcdc->RxBuffer = pbuff;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_ECM_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_ECM_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *)pdev->pClassData;
  USBD_StatusTypeDef ret = USBD_BUSY;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (hcdc->TxState == 0U)
  {
    /* Tx Transfer in progress */
    hcdc->TxState = 1U;

    /* Update the packet total length */
    pdev->ep_in[CDC_ECM_IN_EP & 0xFU].total_length = hcdc->TxLength;

    /* Transmit next packet */
    (void)USBD_LL_Transmit(pdev, CDC_ECM_IN_EP, hcdc->TxBuffer, hcdc->TxLength);

     ret = USBD_OK;
  }

  return (uint8_t)ret;
}


/**
  * @brief  USBD_CDC_ECM_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_ECM_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *)pdev->pClassData;

  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Prepare Out endpoint to receive next packet */
  (void)USBD_LL_PrepareReceive(pdev, CDC_ECM_OUT_EP,hcdc->RxBuffer, hcdc->MaxPcktLen);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_ECM_SendNotification
  *         Transmit Notification packet on CMD IN interrupt endpoint
  * @param  pdev: device instance
  *         Notif: value of the notification type (from CDC_ECM_Notification_TypeDef enumeration list)
  *         bVal: value of the notification switch (ie. 0x00 or 0x01 for Network Connection notification)
  *         pData: pointer to data buffer (ie. upstream and downstream connection speed values)
  * @retval status
  */
uint8_t USBD_CDC_ECM_SendNotification(USBD_HandleTypeDef *pdev,
                                      USBD_CDC_ECM_NotifCodeTypeDef Notif,
                                      uint16_t bVal, uint8_t *pData)
{
  uint32_t Idx;
  uint32_t ReqSize = 0U;
  USBD_CDC_ECM_HandleTypeDef *hcdc = (USBD_CDC_ECM_HandleTypeDef *)pdev->pClassData;
  USBD_StatusTypeDef ret = USBD_OK;

  /* Initialize the request fields */
  (hcdc->Req).bmRequest = CDC_ECM_BMREQUEST_TYPE_ECM;
  (hcdc->Req).bRequest = (uint8_t)Notif;

  switch (Notif)
  {
    case NETWORK_CONNECTION:
      (hcdc->Req).wValue = bVal;
      (hcdc->Req).wIndex = CDC_ECM_CMD_ITF_NBR;
      (hcdc->Req).wLength = 0U;

      for (Idx = 0U; Idx < 8U; Idx++)
      {
        (hcdc->Req).data[Idx] = 0U;
      }
      ReqSize = 8U;
      break;

    case RESPONSE_AVAILABLE:
      (hcdc->Req).wValue = 0U;
      (hcdc->Req).wIndex = CDC_ECM_CMD_ITF_NBR;
      (hcdc->Req).wLength = 0U;
      for (Idx = 0U; Idx < 8U; Idx++)
      {
        (hcdc->Req).data[Idx] = 0U;
      }
      ReqSize = 8U;
      break;

    case CONNECTION_SPEED_CHANGE:
      (hcdc->Req).wValue = 0U;
      (hcdc->Req).wIndex = CDC_ECM_CMD_ITF_NBR;
      (hcdc->Req).wLength = 0x0008U;
      ReqSize = 16U;

      /* Check pointer to data buffer */
      if (pData != NULL)
      {
        for (Idx = 0U; Idx < 8U; Idx++)
        {
          (hcdc->Req).data[Idx] = pData[Idx];
        }
      }
      break;

    default:
      ret = USBD_FAIL;
      break;
  }

  /* Transmit notification packet */
  if (ReqSize != 0U)
  {
    (void)USBD_LL_Transmit(pdev, CDC_ECM_CMD_EP, (uint8_t *)&(hcdc->Req), ReqSize);
  }

  return (uint8_t)ret;
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
