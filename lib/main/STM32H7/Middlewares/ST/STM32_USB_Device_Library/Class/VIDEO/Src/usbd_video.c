/**
  ******************************************************************************
  * @file    usbd_video.c
  * @author  MCD Application Team
  * @brief   This file provides the Video core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                VIDEO Class  Description
  *          ===================================================================
  *           This driver manages the Video Class 1.1 following the "USB Device Class Definition for
  *           Video Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Interface Association Descriptor
  *             -Standard VC Interface Descriptor  = interface 0
  *             -Standard Vs Interface Descriptor  = interface 1
  *             - 1 Video Streaming Interface
  *             - 1 Video Streaming Endpoint
  *             - 1 Video Terminal Input (camera)
  *             - Video Class-Specific AC Interfaces
  *             - Video Class-Specific AS Interfaces
  *             - VideoControl Requests
  *             - Video Synchronization type: Asynchronous
  *          The current  Video class version supports the following Video features:
  *             - image JPEG format
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the USB DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usbd_video.h"
#include "usbd_ctlreq.h"
#include "usbd_core.h"
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_VIDEO
  * @brief USB Device Video Class core module
  * @{
  */

/** @defgroup USBD_VIDEO_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_VIDEO_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_VIDEO_Private_Macros
  * @{
  */

/* VIDEO Device library callbacks */
static uint8_t USBD_VIDEO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_VIDEO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_VIDEO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t *USBD_VIDEO_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_VIDEO_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_VIDEO_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_VIDEO_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t USBD_VIDEO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_VIDEO_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_VIDEO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);

/* VIDEO Requests management functions */
static void VIDEO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void VIDEO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static USBD_VIDEO_DescHeader_t *USBD_VIDEO_GetNextDesc(uint8_t *pbuf, uint16_t *ptr);
static void *USBD_VIDEO_GetEpDesc(uint8_t *pConfDesc, uint8_t EpAddr);
static void *USBD_VIDEO_GetVSFrameDesc(uint8_t *pConfDesc);


/**
  * @}
  */

/** @defgroup USBD_VIDEO_Private_Variables
  * @{
  */

USBD_ClassTypeDef  USBD_VIDEO =
{
  USBD_VIDEO_Init,
  USBD_VIDEO_DeInit,
  USBD_VIDEO_Setup,
  NULL,
  NULL,
  USBD_VIDEO_DataIn,
  NULL,
  USBD_VIDEO_SOF,
  USBD_VIDEO_IsoINIncomplete,
  NULL,
  USBD_VIDEO_GetHSCfgDesc,
  USBD_VIDEO_GetFSCfgDesc,
  USBD_VIDEO_GetOtherSpeedCfgDesc,
  USBD_VIDEO_GetDeviceQualifierDesc,
};

/* USB VIDEO device Configuration Descriptor (same for all speeds thanks to user defines) */
__ALIGN_BEGIN static uint8_t USBD_VIDEO_CfgDesc[] __ALIGN_END =
{
  /* Configuration 1 */
  USB_CONF_DESC_SIZE,                            /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                   /* bDescriptorType: Configuration */
  LOBYTE(UVC_CONFIG_DESC_SIZ),                   /* wTotalLength: no of returned bytes */
  HIBYTE(UVC_CONFIG_DESC_SIZ),
  0x02,                                          /* bNumInterfaces: 2 interface */
  0x01,                                          /* bConfigurationValue: Configuration value */
  0x00,                                          /* iConfiguration: Index of string descriptor describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xC0,                                          /* bmAttributes: Bus Powered according to user configuration */
#else
  0x80,                                          /* bmAttributes: Bus Powered according to user configuration */
#endif
  USBD_MAX_POWER,                                /* bMaxPower in mA according to user configuration */

  /* Interface Association Descriptor */
  USB_IAD_DESC_SIZE,                             /* bLength: Interface Association Descriptor size */
  USB_DESC_TYPE_IAD,                             /* bDescriptorType: interface association */
  0x00,                                          /* bFirstInterface */
  0x02,                                          /* bInterfaceCount */
  UVC_CC_VIDEO,                                  /* bFunctionClass: Video class */
  SC_VIDEO_INTERFACE_COLLECTION,                 /* bFunctionSubClass: Video Interface Collection */
  PC_PROTOCOL_UNDEFINED,                         /* bInterfaceProtocol: protocol undefined */
  0x00,                                          /* iFunction */

  /* Standard VC (Video Control) Interface Descriptor  = interface 0 */
  USB_IF_DESC_SIZE,                              /* bLength: interface descriptor size */
  USB_DESC_TYPE_INTERFACE,                       /* bDescriptorType: interface */
  UVC_VC_IF_NUM,                                 /* bInterfaceNumber: interface number */
  0x00,                                          /* bAlternateSetting: index of this alternate setting */
  0x00,                                          /* bNumEndpoints: No endpoints used for this interface */
  UVC_CC_VIDEO,                                  /* bInterfaceClass: Video Class */
  SC_VIDEOCONTROL,                               /* bInterfaceSubClass: Video Control */
  PC_PROTOCOL_UNDEFINED,                         /* bInterfaceProtocol: protocol is undefined */
  0x00,                                          /* iFunction: index of string descriptor relative to this item */

  /* Class-specific VC Interface Descriptor */
  VIDEO_VC_IF_HEADER_DESC_SIZE,                  /* bLength */
  CS_INTERFACE,                                  /* bDescriptorType */
  VC_HEADER,                                     /* bDescriptorSubtype */
  LOBYTE(UVC_VERSION),
  HIBYTE(UVC_VERSION),                           /* bcdUVC: UVC1.0 or UVC1.1 revision */
  VS_FRAME_DESC_SIZE,                            /* wTotalLength: total size of class-specific descriptors */
  0x00,
  0x00,                                          /* dwClockFrequency: not used. 48 Mhz value is set, but not used */
  0x6C,
  0xDC,
  0x02,
  0x01,                                          /* bInCollection: number of streaming interfaces */
  0x01,                                          /* baInterfaceNr(1): VideoStreaming interface 1 is part of VC interface */

  /* Input Terminal Descriptor */
  VIDEO_IN_TERMINAL_DESC_SIZE,                   /* bLength: Input terminal descriptor size */
  CS_INTERFACE,                                  /* bDescriptorType: INTERFACE */
  VC_INPUT_TERMINAL,                             /* bDescriptorSubtype: INPUT_TERMINAL */
  0x01,                                          /* bTerminalID: ID of this Terminal */
  LOBYTE(ITT_VENDOR_SPECIFIC),                   /* wTerminalType: 0x0200 ITT_VENDOR_SPECIFIC */
  HIBYTE(ITT_VENDOR_SPECIFIC),
  0x00,                                          /* bAssocTerminal: no Terminal is associated */
  0x00,                                          /* iTerminal: index of string descriptor relative to this item */

  /* Output Terminal Descriptor */
  VIDEO_OUT_TERMINAL_DESC_SIZE,                  /* bLength: output terminal descriptor size */
  CS_INTERFACE,                                  /* bDescriptorType */
  VC_OUTPUT_TERMINAL,                            /* bDescriptorSubtype */
  0x02,                                          /* bTerminalID */
  LOBYTE(TT_STREAMING),                          /* wTerminalType: USB streaming terminal */
  HIBYTE(TT_STREAMING),
  0x00,                                          /* bAssocTerminal: no Terminal is associated */
  0x01,                                          /* bSourceID: input is connected to output unit ID 1 */
  0x00,                                          /* iTerminal: index of string descriptor relative to this item */

  /* Standard VS (Video Streaming) Interface Descriptor = interface 1, alternate setting 0 = Zero Bandwidth
    (when no data are sent from the device) */
  USB_IF_DESC_SIZE,                              /* bLength: interface descriptor size */
  USB_DESC_TYPE_INTERFACE,                       /* bDescriptorType */
  UVC_VS_IF_NUM,                                 /* bInterfaceNumber */
  0x00,                                          /* bAlternateSetting */
  0x00,                                          /* bNumEndpoints: no endpoints used for alternate setting 0 */
  UVC_CC_VIDEO,                                  /* bInterfaceClass */
  SC_VIDEOSTREAMING,                             /* bInterfaceSubClass */
  PC_PROTOCOL_UNDEFINED,                         /* bInterfaceProtocol */
  0x00,                                          /* iInterface: index of string descriptor relative to this item */

  /* Class-specific VS Header Descriptor (Input) */
  VIDEO_VS_IF_IN_HEADER_DESC_SIZE,               /* bLength */
  CS_INTERFACE,                                  /* bDescriptorType */
  VS_INPUT_HEADER,                               /* bDescriptorSubtype */
  0x01,                                          /* bNumFormats: 1 format descriptor is used */
  VC_HEADER_SIZE,
  0x00,                                          /* Total size of Video Control Specific Descriptors */
  UVC_IN_EP,                                     /* bEndPointAddress: In endpoint is used for the alternate setting */
  0x00,                                          /* bmInfo: dynamic format change not supported */
  0x02,                                          /* bTerminalLink: output to terminal ID 2 */
  0x00,                                          /* bStillCaptureMethod: not supported */
  0x00,                                          /* bTriggerSupport: not supported */
  0x00,                                          /* bTriggerUsage: not supported */
  0x01,                                          /* bControlSize: 1 byte field size */
  0x00,                                          /* bmaControls: No specific controls used */

  /* Payload Format Descriptor */
  VS_FORMAT_DESC_SIZE,                           /* blength */
  CS_INTERFACE,                                  /* bDescriptorType */
  VS_FORMAT_SUBTYPE,                             /* bDescriptorSubType */
  0x01,                                          /* bFormatIndex */
  0x01,                                          /* bNumFrameDescriptor */
#ifdef USBD_UVC_FORMAT_UNCOMPRESSED
  DBVAL(UVC_UNCOMPRESSED_GUID),                  /* Giud Format: YUY2 {32595559-0000-0010-8000-00AA00389B71} */
  0x00, 0x00,
  0x10, 0x00,
  0x80, 0x00,
  0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71,
  UVC_BITS_PER_PIXEL,                            /* bBitsPerPixel : Number of bits per pixel */
#else
  0x01,                                          /* bmFlags: FixedSizeSamples */
#endif
  0x01,                                          /* bDefaultFrameIndex: default frame used is frame 1 (only one frame used) */
  0x00,                                          /* bAspectRatioX: not required by specification */
  0x00,                                          /* bAspectRatioY: not required by specification */
  0x00,                                          /* bInterlaceFlags: non interlaced stream */
  0x00,                                          /* bCopyProtect: no protection restrictions */

  /* Class-specific VS (Video Streaming) Frame Descriptor */
  VS_FRAME_DESC_SIZE,                            /* bLength */
  CS_INTERFACE,                                  /* bDescriptorType */
  VS_FRAME_SUBTYPE,                              /* bDescriptorSubType */
  0x01,                                          /* bFrameIndex */
  0x02,                                          /* bmCapabilities: fixed frame rate supported */
  WBVAL(UVC_WIDTH),                              /* wWidth: Image Frame Width */
  WBVAL(UVC_HEIGHT),                             /* wHeight: Image Frame Height */
  DBVAL(UVC_MIN_BIT_RATE(UVC_CAM_FPS_FS)),       /* dwMinBitRate: Minimum supported bit rate in bits/s  */
  DBVAL(UVC_MAX_BIT_RATE(UVC_CAM_FPS_FS)),       /* dwMaxBitRate: Maximum supported bit rate in bits/s  */
  DBVAL(UVC_MAX_FRAME_SIZE),                     /* dwMaxVideoFrameBufSize: Maximum video frame size, in bytes */
  DBVAL(UVC_INTERVAL(UVC_CAM_FPS_FS)),           /* dwDefaultFrameInterval: following number of FPS */
  0x01,                                          /* bFrameIntervalType: Discrete frame interval type */
  DBVAL(UVC_INTERVAL(UVC_CAM_FPS_FS)),           /* dwMinFrameInterval: One supported value of interval (FPS) */

#ifdef USBD_UVC_FORMAT_UNCOMPRESSED
  /* Color Matching Descriptor */
  VS_COLOR_MATCHING_DESC_SIZE,                   /* bLength */
  CS_INTERFACE,                                  /* bDescriptorType: CS_INTERFACE */
  VS_COLORFORMAT,                                /* bDescriptorSubType: VS_COLORFORMAT */
  UVC_COLOR_PRIMARIE,                            /* bColorPrimarie: 1: BT.709, sRGB (default) */
  UVC_TFR_CHARACTERISTICS,                       /* bTransferCharacteristics: 1: BT.709 (default) */
  UVC_MATRIX_COEFFICIENTS,                       /* bMatrixCoefficients: 4: BT.601, (default) */
#endif

  /* Standard VS Interface Descriptor  = interface 1, alternate setting 1 = data transfer mode  */
  USB_IF_DESC_SIZE,                              /* bLength */
  USB_DESC_TYPE_INTERFACE,                       /* bDescriptorType */
  UVC_VS_IF_NUM,                                 /* bInterfaceNumber */
  0x01,                                          /* bAlternateSetting */
  0x01,                                          /* bNumEndpoints: one endpoint is used */
  UVC_CC_VIDEO,                                  /* bInterfaceClass */
  SC_VIDEOSTREAMING,                             /* bInterfaceSubClass */
  PC_PROTOCOL_UNDEFINED,                         /* bInterfaceProtocol */
  0x00,                                          /* iInterface: index of string descriptor relative to this item */

  /* Standard VS (Video Streaming) data Endpoint */
  USB_EP_DESC_SIZE,                              /* bLength */
  USB_DESC_TYPE_ENDPOINT,                        /* bDescriptorType */
  UVC_IN_EP,                                     /* bEndpointAddress */
  0x05,                                          /* bmAttributes: ISO transfer */
  LOBYTE(UVC_ISO_FS_MPS),                        /* wMaxPacketSize */
  LOBYTE(UVC_ISO_FS_MPS),
  0x01,                                          /* bInterval: 1 frame interval */
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_VIDEO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0xEF,
  0x02,
  0x01,
  0x40,
  0x01,
  0x00,
};

/* Video Commit data structure */
static USBD_VideoControlTypeDef video_Commit_Control =
{
  .bmHint = 0x0000U,
  .bFormatIndex = 0x01U,
  .bFrameIndex = 0x01U,
  .dwFrameInterval = UVC_INTERVAL(UVC_CAM_FPS_FS),
  .wKeyFrameRate = 0x0000U,
  .wPFrameRate = 0x0000U,
  .wCompQuality = 0x0000U,
  .wCompWindowSize = 0x0000U,
  .wDelay = 0x0000U,
  .dwMaxVideoFrameSize = 0x0000U,
  .dwMaxPayloadTransferSize = 0x00000000U,
  .dwClockFrequency = 0x00000000U,
  .bmFramingInfo = 0x00U,
  .bPreferedVersion = 0x00U,
  .bMinVersion = 0x00U,
  .bMaxVersion = 0x00U,
};

/* Video Probe data structure */
static USBD_VideoControlTypeDef video_Probe_Control =
{
  .bmHint = 0x0000U,
  .bFormatIndex = 0x01U,
  .bFrameIndex = 0x01U,
  .dwFrameInterval = UVC_INTERVAL(UVC_CAM_FPS_FS),
  .wKeyFrameRate = 0x0000U,
  .wPFrameRate = 0x0000U,
  .wCompQuality = 0x0000U,
  .wCompWindowSize = 0x0000U,
  .wDelay = 0x0000U,
  .dwMaxVideoFrameSize = 0x0000U,
  .dwMaxPayloadTransferSize = 0x00000000U,
  .dwClockFrequency = 0x00000000U,
  .bmFramingInfo = 0x00U,
  .bPreferedVersion = 0x00U,
  .bMinVersion = 0x00U,
  .bMaxVersion = 0x00U,
};

/**
  * @}
  */

/** @defgroup USBD_VIDEO_Private_Functions
  * @{
  */

/**
  * @brief  USBD_VIDEO_Init
  *         Initialize the VIDEO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_VIDEO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_VIDEO_HandleTypeDef *hVIDEO;

  /* Allocate memory for the video control structure */
  hVIDEO = USBD_malloc(sizeof(USBD_VIDEO_HandleTypeDef));

  /* Check if allocated point is NULL, then exit with error code */
  if (hVIDEO == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Assign the pClassData pointer to the allocated structure */
  pdev->pClassData = (void *)hVIDEO;

  /* Open EP IN */
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    (void)USBD_LL_OpenEP(pdev, UVC_IN_EP, USBD_EP_TYPE_ISOC, UVC_ISO_HS_MPS);

    pdev->ep_in[UVC_IN_EP & 0xFU].is_used = 1U;
    pdev->ep_in[UVC_IN_EP & 0xFU].maxpacket = UVC_ISO_HS_MPS;
  }
  else
  {
    (void)USBD_LL_OpenEP(pdev, UVC_IN_EP, USBD_EP_TYPE_ISOC, UVC_ISO_FS_MPS);

    pdev->ep_in[UVC_IN_EP & 0xFU].is_used = 1U;
    pdev->ep_in[UVC_IN_EP & 0xFU].maxpacket = UVC_ISO_FS_MPS;
  }

  /* Init  physical Interface components */
  ((USBD_VIDEO_ItfTypeDef *)pdev->pUserData)->Init();

  /* Init Xfer states */
  hVIDEO->interface = 0U;

  /* Some calls to unused variables, to comply with MISRA-C 2012 rules */
  UNUSED(USBD_VIDEO_CfgDesc);
  UNUSED(cfgidx);

  /* Exit with no error code */
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_VIDEO_DeInit
  *         DeInitialize the VIDEO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_VIDEO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  /* Check if the video structure pointer is valid */
  if (pdev->pClassData == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, UVC_IN_EP);
  pdev->ep_in[UVC_IN_EP & 0xFU].is_used = 0U;

  /* DeInit  physical Interface components */
  ((USBD_VIDEO_ItfTypeDef *)pdev->pUserData)->DeInit();
  USBD_free(pdev->pClassData);
  pdev->pClassData = NULL;

  /* Exit with no error code */
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_VIDEO_Setup
  *         Handle the VIDEO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_VIDEO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_VIDEO_HandleTypeDef *hVIDEO = (USBD_VIDEO_HandleTypeDef *) pdev->pClassData;
  uint8_t ret = (uint8_t)USBD_OK;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* Class Requests -------------------------------*/
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case UVC_GET_CUR:
        case UVC_GET_DEF:
        case UVC_GET_MIN:
        case UVC_GET_MAX:
          VIDEO_REQ_GetCurrent(pdev, req);
          break;
        case UVC_GET_RES:
        case UVC_GET_LEN:
        case UVC_GET_INFO:
          break;
        case UVC_SET_CUR:
          VIDEO_REQ_SetCurrent(pdev, req);
          break;
        default:
          (void) USBD_CtlError(pdev, req);
          ret = (uint8_t)USBD_FAIL;
          break;
      }
      break;

    /* Standard Requests -------------------------------*/
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void) USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = (uint8_t)USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == CS_DEVICE)
          {
            pbuf = USBD_VIDEO_CfgDesc + 18;
            len = MIN((uint16_t)USB_CONF_DESC_SIZE, (uint16_t)req->wLength);
          }
          (void)USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)  USBD_CtlSendData(pdev, (uint8_t *)&hVIDEO->interface, 1);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = (uint8_t)USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE :
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if (req->wValue <= USBD_MAX_NUM_INTERFACES)
            {
              hVIDEO->interface = LOBYTE(req->wValue);
              if (hVIDEO->interface == 1U)
              {
                /* Start Streaming (First endpoint writing will be done on next SOF) */
                (void)USBD_LL_FlushEP(pdev, UVC_IN_EP);
                hVIDEO->uvc_state = UVC_PLAY_STATUS_READY;
              }
              else
              {
                /* Stop Streaming */
                hVIDEO->uvc_state = UVC_PLAY_STATUS_STOP;
                (void)USBD_LL_FlushEP(pdev, UVC_IN_EP);
              }
            }
            else
            {
              /* Call the error management function (command will be NAKed) */
              USBD_CtlError(pdev, req);
              ret = (uint8_t)USBD_FAIL;
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = (uint8_t)USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = (uint8_t)USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = (uint8_t)USBD_FAIL;
      break;
  }

  return ret;
}

/**
  * @brief  USBD_VIDEO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_VIDEO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_VIDEO_HandleTypeDef *hVIDEO = (USBD_VIDEO_HandleTypeDef *) pdev->pClassData;
  static uint8_t  packet[UVC_PACKET_SIZE + (UVC_HEADER_PACKET_CNT * 2U)] = {0x00U};
  static uint8_t *Pcktdata = packet;
  static uint16_t PcktIdx = 0U;
  static uint16_t PcktSze = UVC_PACKET_SIZE;
  static uint8_t  payload_header[2] = {0x02U, 0x00U};
  uint8_t i = 0U;
  uint32_t RemainData, DataOffset = 0U;

  /* Check if the Streaming has already been started */
  if (hVIDEO->uvc_state == UVC_PLAY_STATUS_STREAMING)
  {
    /* Get the current packet buffer, index and size from the application layer */
    ((USBD_VIDEO_ItfTypeDef *)pdev->pUserData)->Data(&Pcktdata, &PcktSze, &PcktIdx);

    /* Check if end of current image has been reached */
    if (PcktSze > 2U)
    {
      /* Check if this is the first packet in current image */
      if (PcktIdx == 0U)
      {
        /* Set the packet start index */
        payload_header[1] ^= 0x01U;
      }

      RemainData = PcktSze;

      /* fill the Transmit buffer */
      while (RemainData > 0U)
      {
        packet[((DataOffset + 0U) * i)] = payload_header[0];
        packet[((DataOffset + 0U) * i) + 1U] = payload_header[1];
        if (RemainData > pdev->ep_in[UVC_IN_EP & 0xFU].maxpacket)
        {
          DataOffset = pdev->ep_in[UVC_IN_EP & 0xFU].maxpacket;
          (void)USBD_memcpy((packet + ((DataOffset + 0U) * i) + 2U),
                            Pcktdata + ((DataOffset - 2U) * i), (DataOffset - 2U));

          RemainData -= DataOffset;
          i++;
        }
        else
        {
          (void)USBD_memcpy((packet + ((DataOffset + 0U) * i) + 2U),
                            Pcktdata + ((DataOffset - 2U) * i), (RemainData - 2U));

          RemainData = 0U;
        }
      }
    }
    else
    {
      /* Add the packet header */
      packet[0] = payload_header[0];
      packet[1] = payload_header[1];
    }

    /* Transmit the packet on Endpoint */
    (void)USBD_LL_Transmit(pdev, (uint8_t)(epnum | 0x80U),
                           (uint8_t *)&packet, (uint32_t)PcktSze);
  }

  /* Exit with no error code */
  return (uint8_t) USBD_OK;
}

/**
  * @brief  USBD_VIDEO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_VIDEO_SOF(USBD_HandleTypeDef *pdev)
{
  USBD_VIDEO_HandleTypeDef *hVIDEO = (USBD_VIDEO_HandleTypeDef *) pdev->pClassData;
  uint8_t payload[2] = {0x02U, 0x00U};

  /* Check if the Streaming has already been started by SetInterface AltSetting 1 */
  if (hVIDEO->uvc_state == UVC_PLAY_STATUS_READY)
  {
    /* Transmit the first packet indicating that Streaming is starting */
    (void)USBD_LL_Transmit(pdev, UVC_IN_EP, (uint8_t *)payload, 2U);

    /* Enable Streaming state */
    hVIDEO->uvc_state = UVC_PLAY_STATUS_STREAMING;
  }

  /* Exit with no error code */
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_VIDEO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_VIDEO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(pdev);
  UNUSED(epnum);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  VIDEO_Req_GetCurrent
  *         Handles the GET_CUR VIDEO control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void VIDEO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_VIDEO_HandleTypeDef *hVIDEO;
  hVIDEO = (USBD_VIDEO_HandleTypeDef *)(pdev->pClassData);
  static __IO uint8_t EntityStatus[8] = {0};

  /* Reset buffer to zeros */
  (void) USBD_memset(hVIDEO->control.data, 0, USB_MAX_EP0_SIZE);

  /* Manage Video Control interface requests */
  if (LOBYTE(req->wIndex) == 0x00U)
  {
    if (HIBYTE(req->wValue) == 0x02U)
    {
      /* Get the status of the current requested Entity */
      EntityStatus[0] = 0x06U;

      /* Send current status */
      (void) USBD_CtlSendData(pdev, (uint8_t *)&EntityStatus, 1U);
    }
    else
    {
      /* Unknown request */
      USBD_CtlError(pdev, req);
    }
  }
  /* Manage Video Streaming interface requests */
  else
  {
    if (LOBYTE(req->wValue) == (uint8_t)VS_PROBE_CONTROL)
    {
      /* Update bPreferedVersion, bMinVersion and bMaxVersion which must be set only by Device */
      video_Probe_Control.bPreferedVersion = 0x00U;
      video_Probe_Control.bMinVersion = 0x00U;
      video_Probe_Control.bMaxVersion = 0x00U;
      video_Probe_Control.dwMaxVideoFrameSize = UVC_MAX_FRAME_SIZE;

      video_Probe_Control.dwClockFrequency = 0x02DC6C00U;

      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
        video_Probe_Control.dwFrameInterval = (UVC_INTERVAL(UVC_CAM_FPS_HS));
        video_Probe_Control.dwMaxPayloadTransferSize = UVC_ISO_HS_MPS;
      }
      else
      {
        video_Probe_Control.dwFrameInterval = (UVC_INTERVAL(UVC_CAM_FPS_FS));
        video_Probe_Control.dwMaxPayloadTransferSize = UVC_ISO_FS_MPS;
      }

      /* Probe Request */
      (void) USBD_CtlSendData(pdev, (uint8_t *)&video_Probe_Control, req->wLength);
    }
    else if (LOBYTE(req->wValue) == (uint8_t)VS_COMMIT_CONTROL)
    {
      if (pdev->dev_speed == USBD_SPEED_HIGH)
      {
        video_Commit_Control.dwFrameInterval = (UVC_INTERVAL(UVC_CAM_FPS_HS));
        video_Commit_Control.dwMaxPayloadTransferSize = UVC_ISO_HS_MPS;
      }
      else
      {
        video_Commit_Control.dwFrameInterval = (UVC_INTERVAL(UVC_CAM_FPS_FS));
        video_Commit_Control.dwMaxPayloadTransferSize = UVC_ISO_FS_MPS;
      }

      /* Commit Request */
      (void) USBD_CtlSendData(pdev, (uint8_t *)&video_Commit_Control, req->wLength);
    }
    else
    {
      /* Send the current mute state */
      (void) USBD_CtlSendData(pdev, hVIDEO->control.data, req->wLength);
    }
  }
}

/**
  * @brief  VIDEO_Req_SetCurrent
  *         Handles the SET_CUR VIDEO control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void VIDEO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_VIDEO_HandleTypeDef *hVIDEO = (USBD_VIDEO_HandleTypeDef *)(pdev->pClassData);

  /* Check that the request has control data */
  if (req->wLength > 0U)
  {
    /* Prepare the reception of the buffer over EP0 */
    if (LOBYTE(req->wValue) == (uint8_t)VS_PROBE_CONTROL)
    {
      /* Probe Request */
      (void) USBD_CtlPrepareRx(pdev, (uint8_t *)&video_Probe_Control, req->wLength);
    }
    else if (LOBYTE(req->wValue) == (uint8_t)VS_COMMIT_CONTROL)
    {
      /* Commit Request */
      (void) USBD_CtlPrepareRx(pdev, (uint8_t *)&video_Commit_Control, req->wLength);
    }
    else
    {
      /* Prepare the reception of the buffer over EP0 */
      (void) USBD_CtlPrepareRx(pdev, hVIDEO->control.data, req->wLength);
    }
  }
}

/**
  * @brief  USBD_VIDEO_GetFSCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_VIDEO_GetFSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypedef *pEpDesc = USBD_VIDEO_GetEpDesc(USBD_VIDEO_CfgDesc, UVC_IN_EP);
  USBD_VIDEO_VSFrameDescTypeDef *pVSFrameDesc = USBD_VIDEO_GetVSFrameDesc(USBD_VIDEO_CfgDesc);

  if (pEpDesc != NULL)
  {
    pEpDesc->wMaxPacketSize = UVC_ISO_FS_MPS;
  }

  if (pVSFrameDesc != NULL)
  {
    pVSFrameDesc->dwMinBitRate = UVC_MIN_BIT_RATE(UVC_CAM_FPS_FS);
    pVSFrameDesc->dwMaxBitRate = UVC_MAX_BIT_RATE(UVC_CAM_FPS_FS);
    pVSFrameDesc->dwDefaultFrameInterval = UVC_INTERVAL(UVC_CAM_FPS_FS);
    pVSFrameDesc->dwMinFrameInterval = UVC_INTERVAL(UVC_CAM_FPS_FS);
  }

  *length = (uint16_t)(sizeof(USBD_VIDEO_CfgDesc));
  return USBD_VIDEO_CfgDesc;
}

/**
  * @brief  USBD_VIDEO_GetHSCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_VIDEO_GetHSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypedef *pEpDesc = USBD_VIDEO_GetEpDesc(USBD_VIDEO_CfgDesc, UVC_IN_EP);
  USBD_VIDEO_VSFrameDescTypeDef *pVSFrameDesc = USBD_VIDEO_GetVSFrameDesc(USBD_VIDEO_CfgDesc);

  if (pEpDesc != NULL)
  {
    pEpDesc->wMaxPacketSize = UVC_ISO_HS_MPS;
  }

  if (pVSFrameDesc != NULL)
  {
    pVSFrameDesc->dwMinBitRate = UVC_MIN_BIT_RATE(UVC_CAM_FPS_HS);
    pVSFrameDesc->dwMaxBitRate = UVC_MAX_BIT_RATE(UVC_CAM_FPS_HS);
    pVSFrameDesc->dwDefaultFrameInterval = UVC_INTERVAL(UVC_CAM_FPS_HS);
    pVSFrameDesc->dwMinFrameInterval = UVC_INTERVAL(UVC_CAM_FPS_HS);
  }

  *length = (uint16_t)(sizeof(USBD_VIDEO_CfgDesc));
  return USBD_VIDEO_CfgDesc;
}

/**
  * @brief  USBD_VIDEO_GetOtherSpeedCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_VIDEO_GetOtherSpeedCfgDesc(uint16_t *length)
{
  USBD_EpDescTypedef *pEpDesc = USBD_VIDEO_GetEpDesc(USBD_VIDEO_CfgDesc, UVC_IN_EP);
  USBD_VIDEO_VSFrameDescTypeDef *pVSFrameDesc = USBD_VIDEO_GetVSFrameDesc(USBD_VIDEO_CfgDesc);

  if (pEpDesc != NULL)
  {
    pEpDesc->wMaxPacketSize = UVC_ISO_FS_MPS;
  }

  if (pVSFrameDesc != NULL)
  {
    pVSFrameDesc->dwMinBitRate = UVC_MIN_BIT_RATE(UVC_CAM_FPS_FS);
    pVSFrameDesc->dwMaxBitRate = UVC_MAX_BIT_RATE(UVC_CAM_FPS_FS);
    pVSFrameDesc->dwDefaultFrameInterval = UVC_INTERVAL(UVC_CAM_FPS_FS);
    pVSFrameDesc->dwMinFrameInterval = UVC_INTERVAL(UVC_CAM_FPS_FS);
  }

  *length = (uint16_t)(sizeof(USBD_VIDEO_CfgDesc));
  return USBD_VIDEO_CfgDesc;
}

/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_VIDEO_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)(sizeof(USBD_VIDEO_DeviceQualifierDesc));
  return USBD_VIDEO_DeviceQualifierDesc;
}

/**
  * @brief  USBD_VIDEO_GetNextDesc
  *         This function return the next descriptor header
  * @param  buf: Buffer where the descriptor is available
  * @param  ptr: data pointer inside the descriptor
  * @retval next header
  */
static USBD_VIDEO_DescHeader_t *USBD_VIDEO_GetNextDesc(uint8_t *pbuf, uint16_t *ptr)
{
  USBD_VIDEO_DescHeader_t *pnext = (USBD_VIDEO_DescHeader_t *)(void *)pbuf;

  *ptr += pnext->bLength;
  pnext = (USBD_VIDEO_DescHeader_t *)(void *)(pbuf + pnext->bLength);

  return (pnext);
}

/**
  * @brief  USBD_VIDEO_GetVSFrameDesc
  *         This function return the Video Endpoint descriptor
  * @param  pdev: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @retval pointer to video endpoint descriptor
  */
static void *USBD_VIDEO_GetVSFrameDesc(uint8_t *pConfDesc)
{
  USBD_VIDEO_DescHeader_t *pdesc = (USBD_VIDEO_DescHeader_t *)(void *)pConfDesc;
  USBD_ConfigDescTypedef *desc = (USBD_ConfigDescTypedef *)(void *)pConfDesc;
  USBD_VIDEO_VSFrameDescTypeDef *pVSFrameDesc = NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_VIDEO_GetNextDesc((uint8_t *)pdesc, &ptr);

      if ((pdesc->bDescriptorSubType == VS_FRAME_MJPEG) ||
          (pdesc->bDescriptorSubType == VS_FRAME_UNCOMPRESSED))
      {
        pVSFrameDesc = (USBD_VIDEO_VSFrameDescTypeDef *)(void *)pdesc;
        break;
      }
    }
  }

  return (void *)pVSFrameDesc;
}

/**
  * @brief  USBD_VIDEO_GetEpDesc
  *         This function return the Video Endpoint descriptor
  * @param  pdev: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @param  EpAddr:  endpoint address
  * @retval pointer to video endpoint descriptor
  */
static void *USBD_VIDEO_GetEpDesc(uint8_t *pConfDesc, uint8_t EpAddr)
{
  USBD_VIDEO_DescHeader_t *pdesc = (USBD_VIDEO_DescHeader_t *)(void *)pConfDesc;
  USBD_ConfigDescTypedef *desc = (USBD_ConfigDescTypedef *)(void *)pConfDesc;
  USBD_EpDescTypedef *pEpDesc = NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_VIDEO_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDescriptorType == USB_DESC_TYPE_ENDPOINT)
      {
        pEpDesc = (USBD_EpDescTypedef *)(void *)pdesc;

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
  * @brief  USBD_VIDEO_RegisterInterface
  * @param  pdev: instance
  * @param  fops: VIDEO interface callback
  * @retval status
  */
uint8_t USBD_VIDEO_RegisterInterface(USBD_HandleTypeDef   *pdev, USBD_VIDEO_ItfTypeDef *fops)
{
  /* Check if the FOPS pointer is valid */
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Assign the FOPS pointer */
  pdev->pUserData = fops;

  /* Exit with no error code */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


