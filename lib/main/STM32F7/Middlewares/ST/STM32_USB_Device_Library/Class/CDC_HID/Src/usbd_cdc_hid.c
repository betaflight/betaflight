/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Chris Hockuba (https://github.com/conkerkh)
 *
 */

#include "usbd_cdc_hid.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_def.h"
#include "usbd_conf.h"

#include "usbd_cdc.h"
#include "usbd_hid.h"

#define USB_HID_CDC_CONFIG_DESC_SIZ  (USB_HID_CONFIG_DESC_SIZ - 9 + USB_CDC_CONFIG_DESC_SIZ + 8)

#define HID_INTERFACE 0x0
#define HID_POOLING_INTERVAL 0x0A // 10ms - 100Hz update rate

#define CDC_COM_INTERFACE 0x1

#define USBD_VID             0x0483
#define USBD_PID             0x3256

__ALIGN_BEGIN uint8_t USBD_HID_CDC_DeviceDescriptor[USB_LEN_DEV_DESC] __ALIGN_END =
{
	0x12,                                  /*bLength */
	USB_DESC_TYPE_DEVICE,            /*bDescriptorType*/
	0x00, 0x02,                            /*bcdUSB */
	0xEF,                                  /*bDeviceClass*/
	0x02,                                  /*bDeviceSubClass*/
	0x01,                                  /*bDeviceProtocol*/
	USB_OTG_MAX_EP0_SIZE,                  /*bMaxPacketSize*/
	LOBYTE(USBD_VID), HIBYTE(USBD_VID),    /*idVendor*/
	LOBYTE(USBD_PID),
	HIBYTE(USBD_PID),                      /*idProduct*/
	0x00, 0x02,                            /*bcdDevice rel. 2.00*/
	USBD_IDX_MFC_STR,                      /*Index of manufacturer  string*/
	USBD_IDX_PRODUCT_STR,                  /*Index of product string*/
	USBD_IDX_SERIAL_STR,                   /*Index of serial number string*/
	USBD_MAX_NUM_CONFIGURATION                       /*bNumConfigurations*/
};

__ALIGN_BEGIN static uint8_t USBD_HID_CDC_CfgDesc[USB_HID_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09,                                    /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,       /* bDescriptorType: Configuration */
  USB_HID_CDC_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x03,                                    /*bNumInterfaces: 2 interfaces (1 for CDC, 1 for HID)*/
  0x01,                                    /*bConfigurationValue: Configuration value*/
  0x00,                                    /*iConfiguration: Index of string descriptor describing
                                              the configuration*/
  0xC0,                                    /*bmAttributes: bus powered and Support Remote Wake-up */
  0x32,                                    /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /************** Descriptor of Joystick Mouse interface ****************/
  /* 09 */
  0x09,                                    /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,           /*bDescriptorType: Interface descriptor type*/
  HID_INTERFACE,                           /*bInterfaceNumber: Number of Interface*/
  0x00,                                    /*bAlternateSetting: Alternate setting*/
  0x01,                                    /*bNumEndpoints*/
  0x03,                                    /*bInterfaceClass: HID*/
  0x00,                                    /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,                                    /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,                                       /*iInterface: Index of string descriptor*/
  /******************** Descriptor of Joystick Mouse HID ********************/
  /* 18 */
  0x09,                                    /*bLength: HID Descriptor size*/
  HID_DESCRIPTOR_TYPE,                     /*bDescriptorType: HID*/
  0x11,                                    /*bcdHID: HID Class Spec release number*/
  0x01,
  0x00,                                    /*bCountryCode: Hardware target country*/
  0x01,                                    /*bNumDescriptors: Number of HID class descriptors to follow*/
  0x22,                                    /*bDescriptorType*/
  HID_MOUSE_REPORT_DESC_SIZE,              /*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Mouse endpoint ********************/
  /* 27 */
  0x07,                                    /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT,            /*bDescriptorType:*/

  HID_EPIN_ADDR,                           /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,                                    /*bmAttributes: Interrupt endpoint*/
  HID_EPIN_SIZE,                           /*wMaxPacketSize: 8 Byte max */
  0x00,
  HID_POOLING_INTERVAL,                    /*bInterval: Polling Interval (10 ms)*/
  /* 34 */

  /******** /IAD should be positioned just before the CDC interfaces ******
               IAD to associate the two CDC interfaces */

  0x08,                                    /* bLength */
  0x0B,                                    /* bDescriptorType */
  0x01,                                    /* bFirstInterface */
  0x02,                                    /* bInterfaceCount */
  0x02,                                    /* bFunctionClass */
  0x02,                                    /* bFunctionSubClass */
  0x01,                                    /* bFunctionProtocol */
  0x00,                                    /* iFunction (Index of string descriptor describing this function) */

   /*Interface Descriptor */
  0x09,                                    /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_COM_INTERFACE,                       /* bInterfaceNumber: Number of Interface */
  0x00,                                    /* bAlternateSetting: Alternate setting */
  0x01,                                    /* bNumEndpoints: One endpoints used */
  0x02,                                    /* bInterfaceClass: Communication Interface Class */
  0x02,                                    /* bInterfaceSubClass: Abstract Control Model */
  0x01,                                    /* bInterfaceProtocol: Common AT commands */
  0x00,                                    /* iInterface: */

  /*Header Functional Descriptor*/
  0x05,                                    /* bLength: Endpoint Descriptor size */
  0x24,                                    /* bDescriptorType: CS_INTERFACE */
  0x00,                                    /* bDescriptorSubtype: Header Func Desc */
  0x10,                                    /* bcdCDC: spec release number */
  0x01,

  /*Call Management Functional Descriptor*/
  0x05,                                    /* bFunctionLength */
  0x24,                                    /* bDescriptorType: CS_INTERFACE */
  0x01,                                    /* bDescriptorSubtype: Call Management Func Desc */
  0x00,                                    /* bmCapabilities: D0+D1 */
  0x02,                                    /* bDataInterface: 2 */

  /*ACM Functional Descriptor*/
  0x04,                                    /* bFunctionLength */
  0x24,                                    /* bDescriptorType: CS_INTERFACE */
  0x02,                                    /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,                                    /* bmCapabilities */

  /*Union Functional Descriptor*/
  0x05,                                    /* bFunctionLength */
  0x24,                                    /* bDescriptorType: CS_INTERFACE */
  0x06,                                    /* bDescriptorSubtype: Union func desc */
  0x01,                                    /* bMasterInterface: Communication class interface */
  0x02,                                    /* bSlaveInterface0: Data Class Interface */

  /*Endpoint 2 Descriptor*/
  0x07,                                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                              /* bEndpointAddress */
  0x03,                                    /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),              /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  0xFF,                                    /* bInterval: */

  /*---------------------------------------------------------------------------*/

  /*Data class interface descriptor*/
  0x09,                                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,           /* bDescriptorType: */
  0x02,                                    /* bInterfaceNumber: Number of Interface */
  0x00,                                    /* bAlternateSetting: Alternate setting */
  0x02,                                    /* bNumEndpoints: Two endpoints used */
  0x0A,                                    /* bInterfaceClass: CDC */
  0x00,                                    /* bInterfaceSubClass: */
  0x00,                                    /* bInterfaceProtocol: */
  0x00,                                    /* iInterface: */

  /*Endpoint OUT Descriptor*/
  0x07,                                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                              /* bEndpointAddress */
  0x02,                                    /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                                    /* bInterval: ignore for Bulk transfer */

  /*Endpoint IN Descriptor*/
  0x07,                                    /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType: Endpoint */
  CDC_IN_EP,                               /* bEndpointAddress */
  0x02,                                    /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                                    /* bInterval */
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

/* Wrapper related callbacks */
static uint8_t  USBD_HID_CDC_Init         (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_HID_CDC_DeInit       (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

/* Control Endpoints*/
static uint8_t  USBD_HID_CDC_Setup        (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  USBD_HID_CDC_EP0_RxReady  (USBD_HandleTypeDef *pdev);

/* Class Specific Endpoints*/
static uint8_t  USBD_HID_CDC_DataIn       (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_HID_CDC_DataOut      (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  *USBD_HID_CDC_GetFSCfgDesc (uint16_t *length);
uint8_t  *USBD_HID_CDC_GetDeviceQualifierDescriptor (uint16_t *length);       //Will be NULL Callback because it's unused

/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_HID_CDC =
{
	USBD_HID_CDC_Init,
	USBD_HID_CDC_DeInit,
	USBD_HID_CDC_Setup,
	NULL,                 /* EP0_TxSent, */
	USBD_HID_CDC_EP0_RxReady,
	USBD_HID_CDC_DataIn,
	USBD_HID_CDC_DataOut,
	NULL,
	NULL,
	NULL,
	NULL,
	USBD_HID_CDC_GetFSCfgDesc,
	NULL,
	USBD_HID_CDC_GetDeviceQualifierDescriptor,
};

static uint8_t USBD_HID_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	//Init CDC
	USBD_CDC.Init(pdev, cfgidx);

	//Init HID
	USBD_HID.Init(pdev, cfgidx);

	return USBD_OK;
}

static uint8_t USBD_HID_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
	//DeInit CDC
	USBD_CDC.DeInit(pdev, cfgidx);

	//DeInit HID
	USBD_HID.DeInit(pdev, cfgidx);

	return USBD_OK;
}

static uint8_t USBD_HID_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	switch(req->bmRequest & USB_REQ_RECIPIENT_MASK) {
	case USB_REQ_RECIPIENT_INTERFACE:
		if (req->wIndex == HID_INTERFACE) {
			return USBD_HID.Setup(pdev, req);
		}
		else {
			return USBD_CDC.Setup(pdev, req);
		}
		break;
	case USB_REQ_RECIPIENT_ENDPOINT:
		if (req->wIndex == HID_EPIN_ADDR) {
			return USBD_HID.Setup(pdev, req);
		} else {
			return USBD_CDC.Setup(pdev, req);
		}
		break;
	}

	return USBD_OK;
}

static uint8_t USBD_HID_CDC_EP0_RxReady  (USBD_HandleTypeDef *pdev)
{
	return (USBD_CDC.EP0_RxReady(pdev));
}

static uint8_t USBD_HID_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	if (epnum == (CDC_IN_EP &~ 0x80)) {
		return USBD_CDC.DataIn(pdev, epnum);
	}
	else {
		return USBD_HID.DataIn(pdev, epnum);
	}

	return USBD_OK;
}

static uint8_t USBD_HID_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	if (epnum == (CDC_OUT_EP &~ 0x80)) {
		return (USBD_CDC.DataOut(pdev, epnum));
	}

	return USBD_OK;
}

static uint8_t  *USBD_HID_CDC_GetFSCfgDesc (uint16_t *length)
{
	*length = sizeof(USBD_HID_CDC_CfgDesc);
	return USBD_HID_CDC_CfgDesc;
}

uint8_t  *USBD_HID_CDC_GetDeviceQualifierDescriptor (uint16_t *length)
{
	*length = sizeof(USBD_HID_CDC_DeviceQualifierDesc);
	return USBD_HID_CDC_DeviceQualifierDesc;
}



