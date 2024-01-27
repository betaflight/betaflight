/**
  ******************************************************************************
  * @file    usbd_cdc_rndis.c
  * @author  MCD Application Team
  * @brief   This file provides the high layer firmware functions to manage the
  *          following functionalities of the USB CDC_RNDIS Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC_RNDIS Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *  @verbatim
  *
  *          ===================================================================
  *                                CDC_RNDIS Class Driver Description
  *          ===================================================================
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  *
  *           These aspects may be enriched or modified for a specific user application.
  *
  *            This driver doesn't implement the following aspects of the specification
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *
  *  @endverbatim
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_rndis.h"
#include "usbd_ctlreq.h"

#ifndef __USBD_CDC_RNDIS_IF_H
#include "usbd_cdc_rndis_if_template.h"
#endif /* __USBD_CDC_RNDIS_IF_H */
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC_RNDIS
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_RNDIS_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_RNDIS_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_RNDIS_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CDC_RNDIS_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_CDC_RNDIS_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_CDC_RNDIS_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_CDC_RNDIS_Setup(USBD_HandleTypeDef *pdev,
                                    USBD_SetupReqTypedef *req);

static uint8_t USBD_CDC_RNDIS_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_RNDIS_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_CDC_RNDIS_EP0_RxReady(USBD_HandleTypeDef *pdev);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_CDC_RNDIS_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_RNDIS_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_RNDIS_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_CDC_RNDIS_GetOtherSpeedCfgDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *USBD_CDC_RNDIS_USRStringDescriptor(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length);
#endif /* USBD_SUPPORT_USER_STRING_DESC */
#ifndef USE_USBD_COMPOSITE
uint8_t *USBD_CDC_RNDIS_GetDeviceQualifierDescriptor(uint16_t *length);
#endif /* USE_USBD_COMPOSITE */

/* CDC_RNDIS Internal messages parsing and construction functions */
static uint8_t USBD_CDC_RNDIS_MsgParsing(USBD_HandleTypeDef *pdev, uint8_t *RxBuff);
static uint8_t USBD_CDC_RNDIS_ProcessInitMsg(USBD_HandleTypeDef *pdev, USBD_CDC_RNDIS_InitMsgTypeDef *Msg);
static uint8_t USBD_CDC_RNDIS_ProcessHaltMsg(USBD_HandleTypeDef *pdev, USBD_CDC_RNDIS_HaltMsgTypeDef *Msg);
static uint8_t USBD_CDC_RNDIS_ProcessKeepAliveMsg(USBD_HandleTypeDef *pdev, USBD_CDC_RNDIS_KpAliveMsgTypeDef *Msg);
static uint8_t USBD_CDC_RNDIS_ProcessQueryMsg(USBD_HandleTypeDef *pdev, USBD_CDC_RNDIS_QueryMsgTypeDef *Msg);
static uint8_t USBD_CDC_RNDIS_ProcessSetMsg(USBD_HandleTypeDef *pdev, USBD_CDC_RNDIS_SetMsgTypeDef *Msg);
static uint8_t USBD_CDC_RNDIS_ProcessResetMsg(USBD_HandleTypeDef *pdev, USBD_CDC_RNDIS_ResetMsgTypeDef *Msg);
static uint8_t USBD_CDC_RNDIS_ProcessPacketMsg(USBD_HandleTypeDef *pdev, USBD_CDC_RNDIS_PacketMsgTypeDef *Msg);
static uint8_t USBD_CDC_RNDIS_ProcessUnsupportedMsg(USBD_HandleTypeDef *pdev, USBD_CDC_RNDIS_CtrlMsgTypeDef *Msg);

/* USB Standard Device Descriptor */
#ifndef USE_USBD_COMPOSITE
__ALIGN_BEGIN static uint8_t USBD_CDC_RNDIS_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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
static uint8_t MAC_StrDesc[6] = {CDC_RNDIS_MAC_ADDR0, CDC_RNDIS_MAC_ADDR1, CDC_RNDIS_MAC_ADDR2,
                                 CDC_RNDIS_MAC_ADDR3, CDC_RNDIS_MAC_ADDR4, CDC_RNDIS_MAC_ADDR5
                                };

static uint32_t ConnSpeedTab[2] = {CDC_RNDIS_CONNECT_SPEED_UPSTREAM,
                                   CDC_RNDIS_CONNECT_SPEED_DOWNSTREAM
                                  };

static uint8_t EmptyResponse = 0x00U;

/**
  * @}
  */

/** @defgroup USBD_CDC_RNDIS_Private_Variables
  * @{
  */


/* CDC_RNDIS interface class callbacks structure */
USBD_ClassTypeDef USBD_CDC_RNDIS =
{
  USBD_CDC_RNDIS_Init,
  USBD_CDC_RNDIS_DeInit,
  USBD_CDC_RNDIS_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_CDC_RNDIS_EP0_RxReady,
  USBD_CDC_RNDIS_DataIn,
  USBD_CDC_RNDIS_DataOut,
  NULL,
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_CDC_RNDIS_GetHSCfgDesc,
  USBD_CDC_RNDIS_GetFSCfgDesc,
  USBD_CDC_RNDIS_GetOtherSpeedCfgDesc,
  USBD_CDC_RNDIS_GetDeviceQualifierDescriptor,
#endif /* USE_USBD_COMPOSITE */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  USBD_CDC_RNDIS_USRStringDescriptor,
#endif /* USBD_SUPPORT_USER_STRING_DESC */
};
#ifndef USE_USBD_COMPOSITE

/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t  USBD_CDC_RNDIS_CfgDesc[] __ALIGN_END =
{
  /* Configuration Descriptor */
  0x09,                                        /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                 /* bDescriptorType: Configuration */
  LOBYTE(CDC_RNDIS_CONFIG_DESC_SIZ),           /* wTotalLength: Total size of the Config descriptor */
  HIBYTE(CDC_RNDIS_CONFIG_DESC_SIZ),
  0x02,                                        /* bNumInterfaces: 2 interfaces */
  0x01,                                        /* bConfigurationValue: Configuration value */
  0x00,                                        /* iConfiguration: Index of string descriptor
                                                  describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                                        /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80,                                        /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                              /* MaxPower (mA) */

  /*---------------------------------------------------------------------------*/
  /* IAD descriptor */
  0x08,                                        /* bLength */
  0x0B,                                        /* bDescriptorType */
  0x00,                                        /* bFirstInterface */
  0x02,                                        /* bInterfaceCount */
  0xE0,                                        /* bFunctionClass (Wireless Controller) */
  0x01,                                        /* bFunctionSubClass */
  0x03,                                        /* bFunctionProtocol */
  0x00,                                        /* iFunction */

  /*---------------------------------------------------------------------------*/
  /* Interface Descriptor */
  0x09,                                        /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                     /* bDescriptorType: Interface descriptor type */
  CDC_RNDIS_CMD_ITF_NBR,                       /* bInterfaceNumber: Number of Interface */
  0x00,                                        /* bAlternateSetting: Alternate setting */
  0x01,                                        /* bNumEndpoints: One endpoint used */
  0x02,                                        /* bInterfaceClass: Communication Interface Class */
  0x02,                                        /* bInterfaceSubClass:Abstract Control Model */
  0xFF,                                        /* bInterfaceProtocol: Common AT commands */
  0x00,                                        /* iInterface */

  /* Header Functional Descriptor */
  0x05,                                        /* bLength: Endpoint Descriptor size */
  0x24,                                        /* bDescriptorType: CS_INTERFACE */
  0x00,                                        /* bDescriptorSubtype: Header functional descriptor */
  0x10,                                        /* bcdCDC: spec release number: 1.20 */
  0x01,

  /* Call Management Functional Descriptor */
  0x05,                                        /* bFunctionLength */
  0x24,                                        /* bDescriptorType: CS_INTERFACE */
  0x01,                                        /* bDescriptorSubtype: Call Management Func Desc */
  0x00,                                        /* bmCapabilities: D0+D1 */
  CDC_RNDIS_COM_ITF_NBR,                       /* bDataInterface: 1 */

  /* ACM Functional Descriptor */
  0x04,                                        /* bFunctionLength */
  0x24,                                        /* bDescriptorType: CS_INTERFACE */
  0x02,                                        /* bDescriptorSubtype: Abstract Control Management desc */
  0x00,                                        /* bmCapabilities */

  /* Union Functional Descriptor */
  0x05,                                        /* bFunctionLength */
  0x24,                                        /* bDescriptorType: CS_INTERFACE */
  0x06,                                        /* bDescriptorSubtype: Union functional descriptor */
  CDC_RNDIS_CMD_ITF_NBR,                       /* bMasterInterface: Communication class interface */
  CDC_RNDIS_COM_ITF_NBR,                       /* bSlaveInterface0: Data Class Interface */

  /* Notification Endpoint Descriptor */
  0x07,                                        /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                      /* bDescriptorType: Endpoint */
  CDC_RNDIS_CMD_EP,                            /* bEndpointAddress */
  0x03,                                        /* bmAttributes: Interrupt */
  LOBYTE(CDC_RNDIS_CMD_PACKET_SIZE),           /* wMaxPacketSize */
  HIBYTE(CDC_RNDIS_CMD_PACKET_SIZE),
  CDC_RNDIS_FS_BINTERVAL,                      /* bInterval */

  /*---------------------------------------------------------------------------*/
  /* Data class interface descriptor */
  0x09,                                        /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                     /* bDescriptorType: */
  CDC_RNDIS_COM_ITF_NBR,                       /* bInterfaceNumber: Number of Interface */
  0x00,                                        /* bAlternateSetting: Alternate setting */
  0x02,                                        /* bNumEndpoints: Two endpoints used */
  0x0A,                                        /* bInterfaceClass: CDC */
  0x00,                                        /* bInterfaceSubClass */
  0x00,                                        /* bInterfaceProtocol */
  0x00,                                        /* iInterface */

  /* Endpoint OUT Descriptor */
  0x07,                                        /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                      /* bDescriptorType: Endpoint */
  CDC_RNDIS_OUT_EP,                            /* bEndpointAddress */
  0x02,                                        /* bmAttributes: Bulk */
  LOBYTE(CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE),   /* wMaxPacketSize: */
  HIBYTE(CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE),
  0x00,                                        /* bInterval */

  /* Endpoint IN Descriptor */
  0x07,                                        /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                      /* bDescriptorType: Endpoint */
  CDC_RNDIS_IN_EP,                             /* bEndpointAddress */
  0x02,                                        /* bmAttributes: Bulk */
  LOBYTE(CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE),   /* wMaxPacketSize */
  HIBYTE(CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE),
  0x00                                         /* bInterval */
} ;
#endif /* USE_USBD_COMPOSITE */

static uint8_t RNDISInEpAdd = CDC_RNDIS_IN_EP;
static uint8_t RNDISOutEpAdd = CDC_RNDIS_OUT_EP;
static uint8_t RNDISCmdEpAdd = CDC_RNDIS_CMD_EP;

static const uint32_t CDC_RNDIS_SupportedOIDs[] =
{
  OID_GEN_SUPPORTED_LIST,
  OID_GEN_HARDWARE_STATUS,
  OID_GEN_MEDIA_SUPPORTED,
  OID_GEN_MEDIA_IN_USE,
  OID_GEN_MAXIMUM_FRAME_SIZE,
  OID_GEN_LINK_SPEED,
  OID_GEN_TRANSMIT_BLOCK_SIZE,
  OID_GEN_RECEIVE_BLOCK_SIZE,
  OID_GEN_VENDOR_ID,
  OID_GEN_VENDOR_DESCRIPTION,
  OID_GEN_CURRENT_PACKET_FILTER,
  OID_GEN_MAXIMUM_TOTAL_SIZE,
  OID_GEN_MEDIA_CONNECT_STATUS,
  OID_GEN_MAXIMUM_SEND_PACKETS,
  OID_802_3_PERMANENT_ADDRESS,
  OID_802_3_CURRENT_ADDRESS,
  OID_802_3_MULTICAST_LIST,
  OID_802_3_MAXIMUM_LIST_SIZE,
  OID_802_3_RCV_ERROR_ALIGNMENT,
  OID_802_3_XMIT_ONE_COLLISION,
  OID_802_3_XMIT_MORE_COLLISIONS,
};

/**
  * @}
  */

/** @defgroup USBD_CDC_RNDIS_Private_Functions
  * @{
  */

/**
  * @brief  USBD_CDC_RNDIS_Init
  *         Initialize the CDC CDC_RNDIS interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_CDC_RNDIS_HandleTypeDef *hcdc;

  hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)USBD_malloc(sizeof(USBD_CDC_RNDIS_HandleTypeDef));

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  RNDISInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  RNDISOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  RNDISCmdEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (hcdc == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  (void)USBD_memset(hcdc, 0, sizeof(USBD_CDC_RNDIS_HandleTypeDef));

  pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, RNDISInEpAdd, USBD_EP_TYPE_BULK,
                         CDC_RNDIS_DATA_HS_IN_PACKET_SIZE);

    pdev->ep_in[RNDISInEpAdd & 0xFU].is_used = 1U;

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, RNDISOutEpAdd, USBD_EP_TYPE_BULK,
                         CDC_RNDIS_DATA_HS_OUT_PACKET_SIZE);

    pdev->ep_out[RNDISOutEpAdd & 0xFU].is_used = 1U;

    /* Set bInterval for CDC RNDIS CMD Endpoint */
    pdev->ep_in[RNDISCmdEpAdd & 0xFU].bInterval = CDC_RNDIS_HS_BINTERVAL;
  }
  else
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, RNDISInEpAdd, USBD_EP_TYPE_BULK,
                         CDC_RNDIS_DATA_FS_IN_PACKET_SIZE);

    pdev->ep_in[RNDISInEpAdd & 0xFU].is_used = 1U;

    /* Open EP OUT */
    (void)USBD_LL_OpenEP(pdev, RNDISOutEpAdd, USBD_EP_TYPE_BULK,
                         CDC_RNDIS_DATA_FS_OUT_PACKET_SIZE);

    pdev->ep_out[RNDISOutEpAdd & 0xFU].is_used = 1U;

    /* Set bInterval for CDC RNDIS CMD Endpoint */
    pdev->ep_in[RNDISCmdEpAdd & 0xFU].bInterval = CDC_RNDIS_FS_BINTERVAL;
  }

  /* Open Command IN EP */
  (void)USBD_LL_OpenEP(pdev, RNDISCmdEpAdd, USBD_EP_TYPE_INTR, CDC_RNDIS_CMD_PACKET_SIZE);
  pdev->ep_in[RNDISCmdEpAdd & 0xFU].is_used = 1U;

  hcdc->RxBuffer = NULL;

  /* Init  physical Interface components */
  ((USBD_CDC_RNDIS_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init();

  /* Init the CDC_RNDIS state */
  hcdc->State = CDC_RNDIS_STATE_BUS_INITIALIZED;

  /* Init Xfer states */
  hcdc->TxState = 0U;
  hcdc->RxState = 0U;
  hcdc->RxLength = 0U;
  hcdc->TxLength = 0U;
  hcdc->LinkStatus = 0U;
  hcdc->NotificationStatus = 0U;
  hcdc->MaxPcktLen = (pdev->dev_speed == USBD_SPEED_HIGH) ? CDC_RNDIS_DATA_HS_MAX_PACKET_SIZE : \
                     CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE;

  if (hcdc->RxBuffer == NULL)
  {
    return (uint8_t)USBD_EMEM;
  }

  /* Prepare Out endpoint to receive next packet */
  (void)USBD_LL_PrepareReceive(pdev, RNDISOutEpAdd,
                               hcdc->RxBuffer, hcdc->MaxPcktLen);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_RNDIS_DeInit
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  RNDISInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  RNDISOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  RNDISCmdEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, RNDISInEpAdd);
  pdev->ep_in[RNDISInEpAdd & 0xFU].is_used = 0U;

  /* Close EP OUT */
  (void)USBD_LL_CloseEP(pdev, RNDISOutEpAdd);
  pdev->ep_out[RNDISOutEpAdd & 0xFU].is_used = 0U;

  /* Close Command IN EP */
  (void)USBD_LL_CloseEP(pdev, RNDISCmdEpAdd);
  pdev->ep_in[RNDISCmdEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[RNDISCmdEpAdd & 0xFU].bInterval = 0U;

  /* DeInit  physical Interface components */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    ((USBD_CDC_RNDIS_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
    USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_CDC_RNDIS_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_Setup(USBD_HandleTypeDef *pdev,
                                    USBD_SetupReqTypedef *req)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_CDC_RNDIS_CtrlMsgTypeDef *Msg;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  Msg = (USBD_CDC_RNDIS_CtrlMsgTypeDef *)(void *)hcdc->data;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      if (req->wLength != 0U)
      {
        /* Control Request Data from Device to Host, send data prepared by device */
        if ((req->bmRequest & 0x80U) != 0U)
        {
          /* Update opcode and length */
          hcdc->CmdOpCode = req->bRequest;
          hcdc->CmdLength = (uint8_t)req->wLength;

          if (hcdc->CmdOpCode == CDC_RNDIS_GET_ENCAPSULATED_RESPONSE)
          {
            /* Data of Response Message has already been prepared by USBD_CDC_RNDIS_MsgParsing.
            Just check that length is corresponding to right expected value */
            if (req->wLength != Msg->MsgLength)
            {
            }
          }

          /* Allow application layer to pre-process data or add own processing before sending response */
          ((USBD_CDC_RNDIS_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
                                                                                 (uint8_t *)hcdc->data,
                                                                                 req->wLength);
          /* Check if Response is ready */
          if (hcdc->ResponseRdy != 0U)
          {
            /* Clear Response Ready flag */
            hcdc->ResponseRdy = 0U;

            /* Send data on control endpoint */
            (void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->data,
                                   MIN(CDC_RNDIS_MAX_DATA_SZE, Msg->MsgLength));
          }
          else
          {
            /* CDC_RNDIS Specification says: If for some reason the device receives a GET ENCAPSULATED RESPONSE
            and is unable to respond with a valid data on the Control endpoint,
            then it should return a one-byte packet set to 0x00, rather than
            stalling the Control endpoint */
            (void)USBD_CtlSendData(pdev, &EmptyResponse, 1U);
          }
        }
        /* Control Request Data from Host to Device: Prepare reception of control data stage */
        else
        {
          hcdc->CmdOpCode = req->bRequest;
          hcdc->CmdLength = (uint8_t)MIN(CDC_RNDIS_MAX_INFO_BUFF_SZ, req->wLength);

          (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->data, hcdc->CmdLength);
        }
      }
      /* No Data control request: there is no such request for CDC_RNDIS protocol,
      so let application layer manage this case */
      else
      {
        ((USBD_CDC_RNDIS_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest,
                                                                               (uint8_t *)req, 0U);
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
  * @brief  USBD_CDC_RNDIS_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc;
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  RNDISInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (epnum == (RNDISInEpAdd & 0x7FU))
  {
    if ((pdev->ep_in[epnum & 0xFU].total_length > 0U) &&
        ((pdev->ep_in[epnum & 0xFU].total_length % hpcd->IN_ep[epnum & 0xFU].maxpacket) == 0U))
    {
      /* Update the packet total length */
      pdev->ep_in[epnum & 0xFU].total_length = 0U;

      /* Send ZLP */
      (void)USBD_LL_Transmit(pdev, epnum, NULL, 0U);
    }
    else
    {
      hcdc->TxState = 0U;

      if (((USBD_CDC_RNDIS_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt != NULL)
      {
        ((USBD_CDC_RNDIS_ItfTypeDef *)pdev->pUserData[pdev->classId])->TransmitCplt(hcdc->TxBuffer, \
                                                                                    &hcdc->TxLength, epnum);
      }
    }
  }
  else if (epnum == (RNDISCmdEpAdd & 0x7FU))
  {
    if (hcdc->NotificationStatus != 0U)
    {
      (void)USBD_CDC_RNDIS_SendNotification(pdev, CONNECTION_SPEED_CHANGE,
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
  * @brief  USBD_CDC_RNDIS_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc;
  uint32_t CurrPcktLen;

#ifdef USE_USBD_COMPOSITE  /* Get the Endpoints addresses allocated for this class instance */
  RNDISOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (epnum == RNDISOutEpAdd)
  {
    /* Get the received data length */
    CurrPcktLen = USBD_LL_GetRxDataSize(pdev, epnum);

    /* Increment the frame length */
    hcdc->RxLength += CurrPcktLen;

    /* If the buffer size is less than max packet size: it is the last packet in current frame */
    if ((CurrPcktLen < hcdc->MaxPcktLen) ||
        (hcdc->RxLength >= (CDC_RNDIS_ETH_MAX_SEGSZE + sizeof(USBD_CDC_RNDIS_PacketMsgTypeDef))))
    {
      /* USB data will be immediately processed, this allow next USB traffic being
      NAKed till the end of the application Xfer */

      /* Call data packet message parsing and processing function */
      (void)USBD_CDC_RNDIS_ProcessPacketMsg(pdev, (USBD_CDC_RNDIS_PacketMsgTypeDef *)(void *)hcdc->RxBuffer);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet in current/new frame */
      (void)USBD_LL_PrepareReceive(pdev, RNDISOutEpAdd,
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
  * @brief  USBD_CDC_RNDIS_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if ((pdev->pUserData[pdev->classId] != NULL) && (hcdc->CmdOpCode != 0xFFU))
  {
    /* Check if the received command is SendEncapsulated command */
    if (hcdc->CmdOpCode == CDC_RNDIS_SEND_ENCAPSULATED_COMMAND)
    {
      /* Process Received CDC_RNDIS Control Message */
      (void)USBD_CDC_RNDIS_MsgParsing(pdev, (uint8_t *)(hcdc->data));

      /* Reset the command opcode for next processing */
      hcdc->CmdOpCode = 0xFFU;
    }
    else
    {
      /* Reset the command opcode for next processing */
      hcdc->CmdOpCode = 0xFFU;

      /* Ignore the command and return fail */
      return (uint8_t)USBD_FAIL;
    }
  }

  return (uint8_t)USBD_OK;
}
#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_CDC_RNDIS_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_RNDIS_GetFSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = CDC_RNDIS_FS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)(sizeof(USBD_CDC_RNDIS_CfgDesc));
  return USBD_CDC_RNDIS_CfgDesc;
}

/**
  * @brief  USBD_CDC_RNDIS_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_RNDIS_GetHSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = CDC_RNDIS_HS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = CDC_RNDIS_DATA_HS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = CDC_RNDIS_DATA_HS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)(sizeof(USBD_CDC_RNDIS_CfgDesc));
  return USBD_CDC_RNDIS_CfgDesc;
}

/**
  * @brief  USBD_CDC_RNDIS_GetOtherSpeedCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_CDC_RNDIS_GetOtherSpeedCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_CDC_RNDIS_CfgDesc, CDC_RNDIS_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = CDC_RNDIS_FS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = CDC_RNDIS_DATA_FS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)(sizeof(USBD_CDC_RNDIS_CfgDesc));
  return USBD_CDC_RNDIS_CfgDesc;
}

/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_CDC_RNDIS_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)(sizeof(USBD_CDC_RNDIS_DeviceQualifierDesc));

  return USBD_CDC_RNDIS_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE */
/**
  * @brief  USBD_CDC_RNDIS_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t USBD_CDC_RNDIS_RegisterInterface(USBD_HandleTypeDef *pdev,
                                         USBD_CDC_RNDIS_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_USRStringDescriptor
  *         Manages the transfer of user string descriptors.
  * @param  pdev: device instance
  * @param  index: descriptor index
  * @param  length : pointer data length
  * @retval pointer to the descriptor table or NULL if the descriptor is not supported.
  */
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *USBD_CDC_RNDIS_USRStringDescriptor(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
  static uint8_t USBD_StrDesc[255];

  /* Check if the requested string interface is supported */
  if (index == CDC_RNDIS_MAC_STRING_INDEX)
  {
    USBD_GetString((uint8_t *)((USBD_CDC_RNDIS_ItfTypeDef *)pdev->pUserData[pdev->classId])->pStrDesc, USBD_StrDesc,
                   length);
    return USBD_StrDesc;
  }
  /* Not supported Interface Descriptor index */
  else
  {
    return NULL;
  }
}
#endif /* USBD_SUPPORT_USER_STRING_DESC */


/**
  * @brief  USBD_CDC_RNDIS_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @param  length: Tx Buffer length
  * @param  ClassId: The Class ID
  * @retval status
  */
#ifdef USE_USBD_COMPOSITE
uint8_t USBD_CDC_RNDIS_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint32_t length, uint8_t ClassId)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[ClassId];
#else
uint8_t USBD_CDC_RNDIS_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff, uint32_t length)
{

  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
#endif /* USE_USBD_COMPOSITE */

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t USBD_CDC_RNDIS_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc->RxBuffer = pbuff;

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @param  ClassId: The Class ID
  * @retval status
  */
#ifdef USE_USBD_COMPOSITE
uint8_t USBD_CDC_RNDIS_TransmitPacket(USBD_HandleTypeDef *pdev, uint8_t ClassId)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[ClassId];
#else
uint8_t USBD_CDC_RNDIS_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
#endif /* USE_USBD_COMPOSITE */

  USBD_CDC_RNDIS_PacketMsgTypeDef *PacketMsg;
  USBD_StatusTypeDef ret = USBD_BUSY;

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  RNDISInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, ClassId);
#endif /* USE_USBD_COMPOSITE */

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  PacketMsg = (USBD_CDC_RNDIS_PacketMsgTypeDef *)(void *)hcdc->TxBuffer;

  if (hcdc->TxState == 0U)
  {
    /* Tx Transfer in progress */
    hcdc->TxState = 1U;

    /* Format the packet information */
    PacketMsg->MsgType = CDC_RNDIS_PACKET_MSG_ID;
    PacketMsg->MsgLength = hcdc->TxLength;
    PacketMsg->DataOffset = sizeof(USBD_CDC_RNDIS_PacketMsgTypeDef) - CDC_RNDIS_PCKTMSG_DATAOFFSET_OFFSET;
    PacketMsg->DataLength = hcdc->TxLength - sizeof(USBD_CDC_RNDIS_PacketMsgTypeDef);
    PacketMsg->OOBDataOffset = 0U;
    PacketMsg->OOBDataLength = 0U;
    PacketMsg->NumOOBDataElements = 0U;
    PacketMsg->PerPacketInfoOffset = 0U;
    PacketMsg->PerPacketInfoLength = 0U;
    PacketMsg->VcHandle = 0U;
    PacketMsg->Reserved = 0U;

    /* Update the packet total length */
    pdev->ep_in[RNDISInEpAdd & 0xFU].total_length = hcdc->TxLength;

    /* Transmit next packet */
    (void)USBD_LL_Transmit(pdev, RNDISInEpAdd, hcdc->TxBuffer, hcdc->TxLength);

    ret = USBD_OK;
  }

  return (uint8_t)ret;
}


/**
  * @brief  USBD_CDC_RNDIS_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t USBD_CDC_RNDIS_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_RNDIS_HandleTypeDef *hcdc;

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  RNDISOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* Prepare Out endpoint to receive next packet */
  (void)USBD_LL_PrepareReceive(pdev, RNDISOutEpAdd,
                               hcdc->RxBuffer, hcdc->MaxPcktLen);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_SendNotification
  *         Transmit Notification packet on CMD IN interrupt endpoint
  * @param  pdev: device instance
  *         Notif: value of the notification type (from CDC_RNDIS_Notification_TypeDef enumeration list)
  *         bVal: value of the notification switch (ie. 0x00 or 0x01 for Network Connection notification)
  *         pData: pointer to data buffer (ie. upstream and downstream connection speed values)
  * @retval status
  */
uint8_t USBD_CDC_RNDIS_SendNotification(USBD_HandleTypeDef *pdev,
                                        USBD_CDC_NotifCodeTypeDef Notif,
                                        uint16_t bVal, uint8_t *pData)
{
  uint32_t Idx;
  uint16_t ReqSize = 0U;
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_StatusTypeDef ret = USBD_OK;

  UNUSED(bVal);
  UNUSED(pData);

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  RNDISCmdEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* Initialize the request fields */
  (hcdc->Req).bmRequest = CDC_RNDIS_BMREQUEST_TYPE_RNDIS;
  (hcdc->Req).bRequest = (uint8_t)Notif;

  switch (Notif)
  {
    case RESPONSE_AVAILABLE:
      (hcdc->Req).wValue = 0U;
      (hcdc->Req).wIndex = CDC_RNDIS_CMD_ITF_NBR;
      (hcdc->Req).wLength = 0U;

      for (Idx = 0U; Idx < 8U; Idx++)
      {
        (hcdc->Req).data[Idx] = 0U;
      }

      ReqSize = 8U;
      break;

    default:
      ret = USBD_FAIL;
      break;
  }

  /* Transmit notification packet */
  if (ReqSize != 0U)
  {
    (void)USBD_LL_Transmit(pdev, RNDISCmdEpAdd, (uint8_t *)&hcdc->Req, ReqSize);
  }

  return (uint8_t)ret;
}


/* ----------------------------- CDC_RNDIS Messages processing functions ----------------------- */
/**
  * @brief  USBD_CDC_RNDIS_MsgParsing
  *         Parse received message and process it depending on its nature.
  * @param  pdev: USB Device Handle pointer
  * @param  RxBuff: Pointer to the message data extracted from SendEncapsulated command
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_MsgParsing(USBD_HandleTypeDef *pdev, uint8_t *RxBuff)
{
  USBD_CDC_RNDIS_CtrlMsgTypeDef *Msg = (USBD_CDC_RNDIS_CtrlMsgTypeDef *)(void *)RxBuff;
  static uint8_t ret = (uint8_t)USBD_OK;

  /* Check message type */
  switch (Msg->MsgType)
  {
    /* CDC_RNDIS Initialize message */
    case CDC_RNDIS_INITIALIZE_MSG_ID:
      ret = USBD_CDC_RNDIS_ProcessInitMsg(pdev, (USBD_CDC_RNDIS_InitMsgTypeDef *)(void *)Msg);
      break;

    /* CDC_RNDIS Halt message */
    case CDC_RNDIS_HALT_MSG_ID:
      ret = USBD_CDC_RNDIS_ProcessHaltMsg(pdev, (USBD_CDC_RNDIS_HaltMsgTypeDef *)(void *)Msg);
      break;

    /* CDC_RNDIS Query message */
    case CDC_RNDIS_QUERY_MSG_ID:
      ret = USBD_CDC_RNDIS_ProcessQueryMsg(pdev, (USBD_CDC_RNDIS_QueryMsgTypeDef *)(void *)Msg);
      break;

    /* CDC_RNDIS Set message */
    case CDC_RNDIS_SET_MSG_ID:
      ret = USBD_CDC_RNDIS_ProcessSetMsg(pdev, (USBD_CDC_RNDIS_SetMsgTypeDef *)(void *)Msg);
      break;

    /* CDC_RNDIS Reset message */
    case CDC_RNDIS_RESET_MSG_ID:
      ret = USBD_CDC_RNDIS_ProcessResetMsg(pdev, (USBD_CDC_RNDIS_ResetMsgTypeDef *)(void *)Msg);
      break;

    /* CDC_RNDIS Keep-Alive message */
    case CDC_RNDIS_KEEPALIVE_MSG_ID:
      ret = USBD_CDC_RNDIS_ProcessKeepAliveMsg(pdev, (USBD_CDC_RNDIS_KpAliveMsgTypeDef *)(void *)Msg);
      break;

    /* CDC_RNDIS unsupported message */
    default:
      ret = USBD_CDC_RNDIS_ProcessUnsupportedMsg(pdev, (USBD_CDC_RNDIS_CtrlMsgTypeDef *)(void *)Msg);
      break;
  }

  return ret;
}


/**
  * @brief  USBD_CDC_RNDIS_ProcessInitMsg
  *         Parse, extract data and check correctness of CDC_RNDIS INIT_MSG command.
  * @param  pdev: USB Device Handle pointer
  * @param  Msg: Pointer to the message data extracted from SendEncapsulated command
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_ProcessInitMsg(USBD_HandleTypeDef *pdev,
                                             USBD_CDC_RNDIS_InitMsgTypeDef *Msg)
{
  /* Get the CDC_RNDIS handle pointer */
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* Get and format the Msg input */
  USBD_CDC_RNDIS_InitMsgTypeDef *InitMessage = (USBD_CDC_RNDIS_InitMsgTypeDef *)Msg;

  /* Use same Msg input buffer as response buffer */
  USBD_CDC_RNDIS_InitCpltMsgTypeDef *InitResponse = (USBD_CDC_RNDIS_InitCpltMsgTypeDef *)(void *)Msg;

  /* Store the Message Request ID */
  uint32_t ReqId = InitMessage->ReqId;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Check correctness of the message (MsgType already checked by entry to this function) */
  if ((InitMessage->MsgLength != sizeof(USBD_CDC_RNDIS_InitMsgTypeDef)) || \
      (InitMessage->MajorVersion < CDC_RNDIS_VERSION_MAJOR))
  {
    InitResponse->Status = CDC_RNDIS_STATUS_FAILURE;
  }
  else
  {
    InitResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
  }

  /* Setup the response buffer content */
  InitResponse->MsgType = CDC_RNDIS_INITIALIZE_CMPLT_ID;
  InitResponse->MsgLength = sizeof(USBD_CDC_RNDIS_InitCpltMsgTypeDef);
  InitResponse->ReqId = ReqId;
  InitResponse->MajorVersion = CDC_RNDIS_VERSION_MAJOR;
  InitResponse->MinorVersion = CDC_RNDIS_VERSION_MINOR;
  InitResponse->DeviceFlags = CDC_RNDIS_DF_CONNECTIONLESS;
  InitResponse->Medium = CDC_RNDIS_MEDIUM_802_3;
  InitResponse->MaxPacketsPerTransfer = 1U;
  InitResponse->MaxTransferSize = (sizeof(USBD_CDC_RNDIS_PacketMsgTypeDef) + CDC_RNDIS_ETH_FRAME_SIZE_MAX);
  InitResponse->PacketAlignmentFactor = 2U; /* Not needed as single packet by transfer set */
  InitResponse->AFListOffset = 0U; /* Reserved for connection-oriented devices. Set value to zero. */
  InitResponse->AFListSize = 0U; /* Reserved for connection-oriented devices. Set value to zero. */

  /* Set CDC_RNDIS state to INITIALIZED */
  hcdc->State = CDC_RNDIS_STATE_INITIALIZED;

  /* Set Response Ready field in order to send response during next control request */
  hcdc->ResponseRdy = 1U;

  /* Send Notification on Interrupt EP to inform Host that response is ready */
  (void)USBD_CDC_RNDIS_SendNotification(pdev, RESPONSE_AVAILABLE, 0U, NULL);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_ProcessHaltMsg
  *         Parse, extract data and check correctness of CDC_RNDIS Halt command.
  * @param  pdev: USB Device Handle pointer
  * @param  Msg: Pointer to the message data extracted from SendEncapsulated command
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_ProcessHaltMsg(USBD_HandleTypeDef *pdev,
                                             USBD_CDC_RNDIS_HaltMsgTypeDef *Msg)
{
  /* Get the CDC_RNDIS handle pointer */
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Set CDC_RNDIS state to INITIALIZED */
  hcdc->State = CDC_RNDIS_STATE_UNINITIALIZED;

  /* No response required for this message, so no notification (RESPONSE_AVAILABLE) is sent */

  UNUSED(Msg);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_ProcessKeepAliveMsg
  *         Parse, extract data and check correctness of CDC_RNDIS KeepAlive command.
  * @param  pdev: USB Device Handle pointer
  * @param  Msg: Pointer to the message data extracted from SendEncapsulated command
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_ProcessKeepAliveMsg(USBD_HandleTypeDef *pdev,
                                                  USBD_CDC_RNDIS_KpAliveMsgTypeDef *Msg)
{
  /* Get the CDC_RNDIS handle pointer */
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* Use same Msg input buffer as response buffer */
  USBD_CDC_RNDIS_KpAliveCpltMsgTypeDef *InitResponse = (USBD_CDC_RNDIS_KpAliveCpltMsgTypeDef *)(void *)Msg;

  /* Store the Message Request ID */
  uint32_t ReqId = Msg->ReqId;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Check correctness of the message (MsgType already checked by entry to this function) */
  if (Msg->MsgLength != sizeof(USBD_CDC_RNDIS_KpAliveMsgTypeDef))
  {
    InitResponse->Status = CDC_RNDIS_STATUS_FAILURE;
  }
  else
  {
    InitResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
  }

  /* Setup the response buffer content */
  InitResponse->MsgType = CDC_RNDIS_KEEPALIVE_CMPLT_ID;
  InitResponse->MsgLength = sizeof(USBD_CDC_RNDIS_KpAliveCpltMsgTypeDef);
  InitResponse->ReqId = ReqId;
  InitResponse->Status = CDC_RNDIS_STATUS_SUCCESS;

  /* Set Response Ready field in order to send response during next control request */
  hcdc->ResponseRdy = 1U;

  /* Send Notification on Interrupt EP to inform Host that response is ready */
  (void)USBD_CDC_RNDIS_SendNotification(pdev, RESPONSE_AVAILABLE, 0U, NULL);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_ProcessQueryMsg
  *         Parse, extract data and check correctness of CDC_RNDIS Query command.
  * @param  pdev: USB Device Handle pointer
  * @param  Msg: Pointer to the message data extracted from SendEncapsulated command
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_ProcessQueryMsg(USBD_HandleTypeDef *pdev,
                                              USBD_CDC_RNDIS_QueryMsgTypeDef *Msg)
{
  /* Get the CDC_RNDIS handle pointer */
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* Use same Msg input buffer as response buffer */
  USBD_CDC_RNDIS_QueryCpltMsgTypeDef *QueryResponse = (USBD_CDC_RNDIS_QueryCpltMsgTypeDef *)(void *)Msg;

  /* Store the Message Request ID */
  uint32_t ReqId = Msg->RequestId;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Process the OID depending on its code */
  switch (Msg->Oid)
  {
    case OID_GEN_SUPPORTED_LIST:
      QueryResponse->InfoBufLength = sizeof(CDC_RNDIS_SupportedOIDs);
      (void)USBD_memcpy(QueryResponse->InfoBuf, CDC_RNDIS_SupportedOIDs,
                        sizeof(CDC_RNDIS_SupportedOIDs));

      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_HARDWARE_STATUS:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = CDC_RNDIS_HW_STS_READY;
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_MEDIA_SUPPORTED:
    case OID_GEN_MEDIA_IN_USE:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = CDC_RNDIS_MEDIUM_802_3;
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_VENDOR_ID:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = USBD_CDC_RNDIS_VID;
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_MAXIMUM_FRAME_SIZE:
    case OID_GEN_TRANSMIT_BLOCK_SIZE:
    case OID_GEN_RECEIVE_BLOCK_SIZE:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = CDC_RNDIS_ETH_FRAME_SIZE_MAX;
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_VENDOR_DESCRIPTION:
      QueryResponse->InfoBufLength = (strlen(USBD_CDC_RNDIS_VENDOR_DESC) + 1U);
      (void)USBD_memcpy(QueryResponse->InfoBuf, USBD_CDC_RNDIS_VENDOR_DESC,
                        strlen(USBD_CDC_RNDIS_VENDOR_DESC));

      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_MEDIA_CONNECT_STATUS:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = CDC_RNDIS_MEDIA_STATE_CONNECTED;
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_MAXIMUM_SEND_PACKETS:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = 1U;
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_LINK_SPEED:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = USBD_CDC_RNDIS_LINK_SPEED;
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_802_3_PERMANENT_ADDRESS:
    case OID_802_3_CURRENT_ADDRESS:
      QueryResponse->InfoBufLength = 6U;
      (void)USBD_memcpy(QueryResponse->InfoBuf, MAC_StrDesc, 6);
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_802_3_MAXIMUM_LIST_SIZE:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = 1U; /* Only one multicast address supported */
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_CURRENT_PACKET_FILTER:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = 0xFFFFFFU; /* USBD_CDC_RNDIS_DEVICE.packetFilter; */
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_802_3_RCV_ERROR_ALIGNMENT:
    case OID_802_3_XMIT_ONE_COLLISION:
    case OID_802_3_XMIT_MORE_COLLISIONS:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      QueryResponse->InfoBuf[0] = 0U; /* Unused OIDs, return zero */
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_GEN_MAXIMUM_TOTAL_SIZE:
      QueryResponse->InfoBufLength = sizeof(uint32_t);
      /* Indicate maximum overall buffer (Ethernet frame and CDC_RNDIS header) the adapter can handle */
      QueryResponse->InfoBuf[0] = (CDC_RNDIS_MESSAGE_BUFFER_SIZE + CDC_RNDIS_ETH_FRAME_SIZE_MAX);
      QueryResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    default:
      /* Unknown or unsupported OID */
      QueryResponse->InfoBufLength = 0U;
      QueryResponse->Status = CDC_RNDIS_STATUS_FAILURE;
      break;
  }

  /* Setup the response buffer content */
  QueryResponse->MsgType = CDC_RNDIS_QUERY_CMPLT_ID;
  QueryResponse->MsgLength = QueryResponse->InfoBufLength + 24U;
  QueryResponse->ReqId = ReqId;
  QueryResponse->InfoBufOffset = 16U;

  /* Set Response Ready field in order to send response during next control request */
  hcdc->ResponseRdy = 1U;

  /* Send Notification on Interrupt EP to inform Host that response is ready */
  (void)USBD_CDC_RNDIS_SendNotification(pdev, RESPONSE_AVAILABLE, 0U, NULL);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_ProcessSetMsg
  *         Parse, extract data and check correctness of CDC_RNDIS Set Message command.
  * @param  pdev: USB Device Handle pointer
  * @param  Msg: Pointer to the message data extracted from SendEncapsulated command
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_ProcessSetMsg(USBD_HandleTypeDef *pdev,
                                            USBD_CDC_RNDIS_SetMsgTypeDef *Msg)
{
  /* Get the CDC_RNDIS handle pointer */
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* Get and format the Msg input */
  USBD_CDC_RNDIS_SetMsgTypeDef *SetMessage = (USBD_CDC_RNDIS_SetMsgTypeDef *)Msg;

  /* Use same Msg input buffer as response buffer */
  USBD_CDC_RNDIS_SetCpltMsgTypeDef *SetResponse = (USBD_CDC_RNDIS_SetCpltMsgTypeDef *)(void *)Msg;

  /* Store the Message Request ID */
  uint32_t ReqId = SetMessage->ReqId;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (SetMessage->Oid)
  {
    case OID_GEN_CURRENT_PACKET_FILTER:
      /* Setup the packet filter value */
      hcdc->PacketFilter = SetMessage->InfoBuf[0];
      SetResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    case OID_802_3_MULTICAST_LIST:
      /* List of multicast addresses on a miniport adapter */
      SetResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
      break;

    default:
      /* Report an error */
      SetResponse->Status = CDC_RNDIS_STATUS_FAILURE;
      break;
  }

  /* Prepare response buffer */
  SetResponse->MsgType = CDC_RNDIS_SET_CMPLT_ID;
  SetResponse->MsgLength = sizeof(USBD_CDC_RNDIS_SetCpltMsgTypeDef);
  SetResponse->ReqId = ReqId;

  /* Set Response Ready field in order to send response during next control request */
  hcdc->ResponseRdy = 1U;

  /* Send Notification on Interrupt EP to inform Host that response is ready */
  (void)USBD_CDC_RNDIS_SendNotification(pdev, RESPONSE_AVAILABLE, 0U, NULL);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_ProcessResetMsg
  *         Parse, extract data and check correctness of CDC_RNDIS Set Message command.
  * @param  pdev: USB Device Handle pointer
  * @param  Msg: Pointer to the message data extracted from SendEncapsulated command
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_ProcessResetMsg(USBD_HandleTypeDef *pdev,
                                              USBD_CDC_RNDIS_ResetMsgTypeDef *Msg)
{
  /* Get and format the Msg input */
  USBD_CDC_RNDIS_ResetMsgTypeDef *ResetMessage = (USBD_CDC_RNDIS_ResetMsgTypeDef *)Msg;
  /* Get the CDC_RNDIS handle pointer */
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  /* Use same Msg input buffer as response buffer */
  USBD_CDC_RNDIS_ResetCpltMsgTypeDef *ResetResponse = (USBD_CDC_RNDIS_ResetCpltMsgTypeDef *)(void *)Msg;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if ((ResetMessage->MsgLength != sizeof(USBD_CDC_RNDIS_ResetMsgTypeDef)) || \
      (ResetMessage->Reserved != 0U))
  {
    ResetResponse->Status = CDC_RNDIS_STATUS_FAILURE;
  }
  else
  {
    ResetResponse->Status = CDC_RNDIS_STATUS_SUCCESS;
  }

  /* Prepare response buffer */
  ResetResponse->MsgType = CDC_RNDIS_RESET_CMPLT_ID;
  ResetResponse->MsgLength = sizeof(USBD_CDC_RNDIS_ResetCpltMsgTypeDef);
  ResetResponse->AddrReset = 0U;

  /* Set CDC_RNDIS state to INITIALIZED */
  hcdc->State = CDC_RNDIS_STATE_BUS_INITIALIZED;
  hcdc->LinkStatus = 0U;

  /* Set Response Ready field in order to send response during next control request */
  hcdc->ResponseRdy = 1U;

  /* Send Notification on Interrupt EP to inform Host that response is ready */
  (void)USBD_CDC_RNDIS_SendNotification(pdev, RESPONSE_AVAILABLE, 0U, NULL);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_ProcessPacketMsg
  *         Parse, extract data and check correctness of CDC_RNDIS Data Packet.
  * @param  pdev: USB Device Handle pointer
  * @param  Msg: Pointer to the message data extracted from Packet
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_ProcessPacketMsg(USBD_HandleTypeDef *pdev,
                                               USBD_CDC_RNDIS_PacketMsgTypeDef *Msg)
{
  uint32_t tmp1;
  uint32_t tmp2;

  /* Get the CDC_RNDIS handle pointer */
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* Get and format the Msg input */
  USBD_CDC_RNDIS_PacketMsgTypeDef *PacketMsg = (USBD_CDC_RNDIS_PacketMsgTypeDef *)Msg;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Check correctness of the message */
  if ((PacketMsg->MsgType != CDC_RNDIS_PACKET_MSG_ID))
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Point to the payload and update the message length */

  /* Use temporary storage variables to comply with MISRA-C 2012 rule of (+) operand allowed types */
  tmp1 = (uint32_t)PacketMsg;
  tmp2 = (uint32_t)(PacketMsg->DataOffset);
  hcdc->RxBuffer = (uint8_t *)(tmp1 + tmp2 + CDC_RNDIS_PCKTMSG_DATAOFFSET_OFFSET);
  hcdc->RxLength = PacketMsg->DataLength;

  /* Process data by application */
  ((USBD_CDC_RNDIS_ItfTypeDef *)pdev->pUserData[pdev->classId])->Receive(hcdc->RxBuffer, &hcdc->RxLength);

  return (uint8_t)USBD_OK;
}


/**
  * @brief  USBD_CDC_RNDIS_ProcessUnsupportedMsg
  *         Parse, extract data and check correctness of CDC_RNDIS KeepAlive command.
  * @param  pdev: USB Device Handle pointer
  * @param  Msg: Pointer to the message data extracted from SendEncapsulated command
  * @retval status
  */
static uint8_t USBD_CDC_RNDIS_ProcessUnsupportedMsg(USBD_HandleTypeDef *pdev,
                                                    USBD_CDC_RNDIS_CtrlMsgTypeDef *Msg)
{
  /* Get the CDC_RNDIS handle pointer */
  USBD_CDC_RNDIS_HandleTypeDef *hcdc = (USBD_CDC_RNDIS_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* Use same Msg input buffer as response buffer */
  USBD_CDC_RNDIS_StsChangeMsgTypeDef *Response = (USBD_CDC_RNDIS_StsChangeMsgTypeDef *)(void *)Msg;

  if (hcdc == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Setup the response buffer content */
  Response->MsgType = CDC_RNDIS_INDICATE_STATUS_MSG_ID;
  Response->MsgLength = sizeof(USBD_CDC_RNDIS_StsChangeMsgTypeDef);
  Response->Status = CDC_RNDIS_STATUS_NOT_SUPPORTED;
  Response->StsBufLength = 0U;
  Response->StsBufOffset = 20U;

  /* Set Response Ready field in order to send response during next control request */
  hcdc->ResponseRdy = 1U;

  /* Send Notification on Interrupt EP to inform Host that response is ready */
  (void)USBD_CDC_RNDIS_SendNotification(pdev, RESPONSE_AVAILABLE, 0U, NULL);

  UNUSED(Msg);

  return (uint8_t)USBD_OK;
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
