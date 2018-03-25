/**
  ******************************************************************************
  * @file    usbd_hid_cdc_wrapper.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    09-November-2015
  * @brief
  *
  * @verbatim
  *
  *          ===================================================================
  *                                composite HID_CDC
  *          ===================================================================
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid_core.h"
#include "usbd_cdc_core.h"
#include "usbd_hid_cdc_wrapper.h"
#include "usbd_desc.h"
#include "usbd_req.h"


/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_MSC_VCP
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_MSC_VCP_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_MSC_VCP_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_MSC_VCP_Private_Macros
  * @{
  */
/**
  * @}
  */




/** @defgroup USBD_MSC_VCP_Private_FunctionPrototypes
  * @{
  */

/*********************************************
   CDC Device library callbacks
 *********************************************/
extern uint8_t  usbd_cdc_Init        (void  *pdev, uint8_t cfgidx);
extern uint8_t  usbd_cdc_DeInit      (void  *pdev, uint8_t cfgidx);
extern uint8_t  usbd_cdc_Setup       (void  *pdev, USB_SETUP_REQ *req);
extern uint8_t  usbd_cdc_EP0_RxReady  (void *pdev);
extern uint8_t  usbd_cdc_DataIn      (void *pdev, uint8_t epnum);
extern uint8_t  usbd_cdc_DataOut     (void *pdev, uint8_t epnum);
extern uint8_t  usbd_cdc_SOF         (void *pdev);

extern uint8_t  *USBD_cdc_GetCfgDesc (uint8_t speed, uint16_t *length);
#ifdef USE_USB_OTG_HS
extern uint8_t  *USBD_cdc_GetOtherCfgDesc (uint8_t speed, uint16_t *length);
#endif

/*********************************************
   MSC Device library callbacks
 *********************************************/
extern uint8_t  USBD_HID_Init (void  *pdev, uint8_t cfgidx);
extern uint8_t  USBD_HID_DeInit (void  *pdev, uint8_t cfgidx);
extern uint8_t  USBD_HID_Setup (void  *pdev, USB_SETUP_REQ *req);
extern uint8_t  USBD_HID_DataIn (void  *pdev, uint8_t epnum);
extern uint8_t  *USBD_HID_GetCfgDesc (uint8_t speed, uint16_t *length);
extern uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ];

/**
  * @}
  */

/** @defgroup USBD_MSC_CDC_Private_Variables
  * @{
  */


static uint8_t  USBD_HID_CDC_Init         (void *pdev , uint8_t cfgidx);
static uint8_t  USBD_HID_CDC_DeInit       (void *pdev , uint8_t cfgidx);

/* Control Endpoints*/
static uint8_t  USBD_HID_CDC_Setup        (void *pdev , USB_SETUP_REQ  *req);
static uint8_t  USBD_HID_CDC_EP0_RxReady  (void *pdev );

/* Class Specific Endpoints*/
static uint8_t  USBD_HID_CDC_DataIn       (void *pdev , uint8_t epnum);
static uint8_t  USBD_HID_CDC_DataOut      (void *pdev , uint8_t epnum);
static uint8_t  USBD_HID_CDC_SOF          (void *pdev);
static uint8_t*  USBD_HID_CDC_GetConfigDescriptor( uint8_t speed , uint16_t *length);

#define USB_HID_CDC_CONFIG_DESC_SIZ  (USB_HID_CONFIG_DESC_SIZ - 9 + USB_CDC_CONFIG_DESC_SIZ + 8)

#define HID_INTERFACE 0x0
#define CDC_COM_INTERFACE 0x1


USBD_Class_cb_TypeDef  USBD_HID_CDC_cb =
{
  USBD_HID_CDC_Init,
  USBD_HID_CDC_DeInit,
  USBD_HID_CDC_Setup,
  NULL,
  USBD_HID_CDC_EP0_RxReady,
  USBD_HID_CDC_DataIn,
  USBD_HID_CDC_DataOut,
  USBD_HID_CDC_SOF,
  NULL,
  NULL,
  USBD_HID_CDC_GetConfigDescriptor,
};


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB MSC/CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CDC_CfgDesc[USB_HID_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
  USB_HID_CDC_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x03,         /*bNumInterfaces: 2 interfaces (1 for CDC, 1 for HID)*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xC0,         /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /************** Descriptor of Joystick Mouse interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
  HID_INTERFACE,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x01,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,         /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
  0x11,         /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

  HID_IN_EP,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  HID_IN_PACKET, /*wMaxPacketSize: 8 Byte max */
  0x00,
  0x0A,          /*bInterval: Polling Interval (10 ms)*/
  /* 34 */

  /******** /IAD should be positioned just before the CDC interfaces ******
               IAD to associate the two CDC interfaces */

  0x08, /* bLength */
  0x0B, /* bDescriptorType */
  0x01, /* bFirstInterface */
  0x02, /* bInterfaceCount */
  0x02, /* bFunctionClass */
  0x02, /* bFunctionSubClass */
  0x01, /* bFunctionProtocol */
  0x00, /* iFunction (Index of string descriptor describing this function) */

   /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_COM_INTERFACE,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */

  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,

  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x02,   /* bDataInterface: 2 */

  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */

  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x01,   /* bMasterInterface: Communication class interface */
  0x02,   /* bSlaveInterface0: Data Class Interface */

  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SZE),
  0xFF,                           /* bInterval: */

  /*---------------------------------------------------------------------------*/

  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
  0x02,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */

  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType: Endpoint */
  CDC_IN_EP,                        /* bEndpointAddress */
  0x02,                             /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
  0x00,                              /* bInterval */

} ;


/**
  * @}
  */

/** @defgroup USBD_CDC_MSC_Private_Functions
  * @{
  */

/**
  * @brief  USBD_MSC_CDC_Init
  *         Initialize the MSC & CDC interfaces
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_CDC_Init (void  *pdev,
                               uint8_t cfgidx)
{

  /* HID initialization */
  USBD_HID_Init (pdev,cfgidx);

  /* CDC initialization */
  usbd_cdc_Init (pdev,cfgidx);

  return USBD_OK;

}

/**
  * @brief  USBD_MSC_CDC_Init
  *         DeInitialize the MSC/CDC interfaces
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_HID_CDC_DeInit (void  *pdev,
                                 uint8_t cfgidx)
{
  /* MSC De-initialization */
  USBD_HID_DeInit (pdev,cfgidx);

  /* CDC De-initialization */
  usbd_cdc_DeInit (pdev,cfgidx);


  return USBD_OK;
}

/**
  * @brief  USBD_MSC_CDC_Setup
  *         Handle the MSC/CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_HID_CDC_Setup (void  *pdev,
                                    USB_SETUP_REQ *req)
{
  switch (req->bmRequest & USB_REQ_RECIPIENT_MASK)
  {
  case USB_REQ_RECIPIENT_INTERFACE:
    if (req->wIndex == HID_INTERFACE)
    {
      return (USBD_HID_Setup (pdev, req));
    }
    else
    {
      return (usbd_cdc_Setup(pdev, req));
    }

  case USB_REQ_RECIPIENT_ENDPOINT:
    if (req->wIndex == HID_IN_EP)
    {
      return (USBD_HID_Setup (pdev, req));
    }
    else
    {
      return (usbd_cdc_Setup(pdev, req));
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_MSC_CDC_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t  *USBD_HID_CDC_GetConfigDescriptor (uint8_t speed, uint16_t *length)
{
  (void)(speed);
  *length = sizeof (USBD_HID_CDC_CfgDesc);
  return USBD_HID_CDC_CfgDesc;
}

/**
  * @brief  USBD_MSC_CDC_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_HID_CDC_DataIn (void  *pdev,
                                     uint8_t epnum)
{
  /*DataIN can be for CDC or MSC */

  if (epnum == (CDC_IN_EP&~0x80) )
  {
    return (usbd_cdc_DataIn(pdev, epnum));
  }
  else
  {
    return (USBD_HID_DataIn(pdev, epnum));
  }
}


uint8_t  USBD_HID_CDC_DataOut(void *pdev , uint8_t epnum)
{
  /*DataOut can be for CDC */
  return (usbd_cdc_DataOut(pdev, epnum));
}


uint8_t  USBD_HID_CDC_SOF (void *pdev)
{
  /*SOF processing needed for CDC */
  return (usbd_cdc_SOF(pdev));
}


uint8_t  USBD_HID_CDC_EP0_RxReady  (void *pdev )
{
  /*RxReady processing needed for CDC only*/
  return (usbd_cdc_EP0_RxReady(pdev));
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

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
