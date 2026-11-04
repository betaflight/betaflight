/*!
    \file  usb_cdc_hid.c
    \brief this file calls to the separate CDC and HID class layer handlers

    \version 2018-06-01, V1.0.0, application for GD32 USBD
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "usb_cdc_hid.h"
#include "standard_hid_core.h"
#include <string.h>
#include "usbd_cdc_vcp.h"
#define USBD_VID                          0x28E9U
#define USBD_PID                          0x028BU

/* local function prototypes ('static') */
static uint8_t cdc_hid_init (usb_dev *udev, uint8_t config_index);
static uint8_t cdc_hid_deinit (usb_dev *udev, uint8_t config_index);
static uint8_t cdc_hid_req_handler (usb_dev *udev, usb_req *req);
static uint8_t cdc_hid_ctlx_out (usb_dev *udev);
static uint8_t cdc_hid_data_in (usb_dev *udev, uint8_t ep_num);
static uint8_t cdc_hid_data_out (usb_dev *udev, uint8_t ep_num);
static uint8_t cdc_hid_sof (usb_dev *udev);

usb_class_core bf_usbd_cdc_hid_cb = {
    .init      = cdc_hid_init,
    .deinit    = cdc_hid_deinit,
    .req_proc  = cdc_hid_req_handler,
    .ctlx_out  = cdc_hid_ctlx_out,
    .data_in   = cdc_hid_data_in,
    .data_out  = cdc_hid_data_out,
    .SOF       = cdc_hid_sof,
};

/* note:it should use the C99 standard when compiling the below codes */
/* USB standard device descriptor */
const usb_desc_dev cdc_hid_dev_desc =
{
    .header =
     {
         .bLength          = USB_DEV_DESC_LEN,
         .bDescriptorType  = USB_DESCTYPE_DEV,
     },
    .bcdUSB                = 0x0200U,
    .bDeviceClass          = 0xEFU,
    .bDeviceSubClass       = 0x02U,
    .bDeviceProtocol       = 0x01U,
    .bMaxPacketSize0       = USB_FS_EP0_MAX_LEN,
    .idVendor              = USBD_VID,
    .idProduct             = USBD_PID,
    .bcdDevice             = 0x0100U,
    .iManufacturer         = STR_IDX_MFC,
    .iProduct              = STR_IDX_PRODUCT,
    .iSerialNumber         = STR_IDX_SERIAL,
    .bNumberConfigurations = USBD_CFG_MAX_NUM,
};

/* USB device configuration descriptor */
const usb_cdc_hid_desc_config_set cdc_hid_config_desc =
{
    .config =
    {
        .header =
         {
             .bLength         = sizeof(usb_desc_config),
             .bDescriptorType = USB_DESCTYPE_CONFIG,
         },
        .wTotalLength         = HID_CDC_CONFIG_DESC_SIZE,
        .bNumInterfaces       = 0x03U,
        .bConfigurationValue  = 0x01U,
        .iConfiguration       = 0x00U,
        .bmAttributes         = 0xC0U,
        .bMaxPower            = 0x32U
    },

    .hid_interface =
    {
        .header =
         {
             .bLength         = sizeof(usb_desc_itf),
             .bDescriptorType = USB_DESCTYPE_ITF
         },
        .bInterfaceNumber     = USBD_HID_INTERFACE,
        .bAlternateSetting    = 0x00U,
        .bNumEndpoints        = 0x01U,
        .bInterfaceClass      = 0x03U,
        .bInterfaceSubClass   = 0x00U,
        .bInterfaceProtocol   = 0x00U,
        .iInterface           = 0x00U
    },

    .hid_vendor_hid =
    {
        .header =
         {
             .bLength = sizeof(usb_desc_hid),
             .bDescriptorType = USB_DESCTYPE_HID
         },
        .bcdHID               = 0x0111U,
        .bCountryCode         = 0x00U,
        .bNumDescriptors      = 0x01U,
        .bDescriptorType      = USB_DESCTYPE_REPORT,
        .wDescriptorLength    = HID_MOUSE_REPORT_DESC_SIZE,
    },

    .hid_report_in_endpoint =
    {
        .header =
         {
             .bLength = sizeof(usb_desc_ep),
             .bDescriptorType = USB_DESCTYPE_EP
         },
        .bEndpointAddress     = HID_IN_EP,
        .bmAttributes         = 0x03U,
        .wMaxPacketSize       = HID_IN_PACKET,
        .bInterval            = 0x0AU
    },

    .iad =
    {
        .header =
         {
             .bLength = sizeof(usb_desc_IAD),
             .bDescriptorType = 0x0BU
         },
         .bFirstInterface     = 0x01U,
         .bInterfaceCount     = 0x02U,
         .bFunctionClass      = 0x02U,
         .bFunctionSubClass   = 0x02U,
         .bFunctionProtocol   = 0x01U,
         .iFunction           = 0x00U
    },

    .cmd_itf =
    {
        .header =
         {
             .bLength         = sizeof(usb_desc_itf),
             .bDescriptorType = USB_DESCTYPE_ITF
         },
        .bInterfaceNumber     = 0x01U,
        .bAlternateSetting    = 0x00U,
        .bNumEndpoints        = 0x01U,
        .bInterfaceClass      = USB_CLASS_CDC,
        .bInterfaceSubClass   = 0x02U,
        .bInterfaceProtocol   = 0x01U,
        .iInterface           = 0x00U
    },

    .cdc_header =
    {
        .header =
         {
            .bLength         = sizeof(usb_desc_header_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype  = 0x00U,
        .bcdCDC              = 0x0110U
    },

    .cdc_call_managment =
    {
        .header =
         {
            .bLength         = sizeof(usb_desc_call_managment_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype  = 0x01U,
        .bmCapabilities      = 0x00U,
        .bDataInterface      = 0x02U
    },

    .cdc_acm =
    {
        .header =
         {
            .bLength         = sizeof(usb_desc_acm_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype  = 0x02U,
        .bmCapabilities      = 0x02U,
    },

    .cdc_union =
    {
        .header =
         {
            .bLength         = sizeof(usb_desc_union_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype  = 0x06U,
        .bMasterInterface    = 0x01U,
        .bSlaveInterface0    = 0x02U,
    },

    .cdc_cmd_endpoint =
    {
        .header =
         {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP,
         },
        .bEndpointAddress    = CDC_CMD_EP,
        .bmAttributes        = 0x03U,
        .wMaxPacketSize      = USB_CDC_CMD_PACKET_SIZE,
        .bInterval           = 0x0AU
    },

    .cdc_data_interface =
    {
        .header =
         {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF,
         },
        .bInterfaceNumber    = 0x02U,
        .bAlternateSetting   = 0x00U,
        .bNumEndpoints       = 0x02U,
        .bInterfaceClass     = 0x0AU,
        .bInterfaceSubClass  = 0x00U,
        .bInterfaceProtocol  = 0x00U,
        .iInterface          = 0x00U
    },

    .cdc_out_endpoint =
    {
        .header =
         {
             .bLength         = sizeof(usb_desc_ep),
             .bDescriptorType = USB_DESCTYPE_EP,
         },
        .bEndpointAddress     = CDC_DATA_OUT_EP,
        .bmAttributes         = USB_EP_ATTR_BULK,
        .wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
        .bInterval            = 0x00U
    },

    .cdc_in_endpoint =
    {
        .header =
         {
             .bLength         = sizeof(usb_desc_ep),
             .bDescriptorType = USB_DESCTYPE_EP
         },
        .bEndpointAddress     = CDC_DATA_IN_EP,
        .bmAttributes         = USB_EP_ATTR_BULK,
        .wMaxPacketSize       = USB_CDC_DATA_PACKET_SIZE,
        .bInterval            = 0x00U
    }
};

extern void *const usbd_bf_cdc_strings[];

usb_desc bf_cdc_hid_desc = {
    .dev_desc    = (uint8_t *)&cdc_hid_dev_desc,
    .config_desc = (uint8_t *)&cdc_hid_config_desc,
    .strings     = usbd_bf_cdc_strings
};

__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] __ALIGN_END =
{
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,        // USAGE (Game Pad)
    0xa1, 0x01,        // COLLECTION (Application)
    0xa1, 0x00,        //   COLLECTION (Physical)
    0x05, 0x01,        //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,        //     USAGE (X)
    0x09, 0x31,        //     USAGE (Y)
    0x09, 0x32,        //     USAGE (Z)
    0x09, 0x33,        //     USAGE (Rx)
    0x09, 0x35,        //     USAGE (Rz)
    0x09, 0x34,        //     USAGE (Ry)
    0x09, 0x36,        //     USAGE (Slider)
    0x09, 0x37,        //     USAGE (Dial)
    0x15, 0x81,        //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,        //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x08,        //     REPORT_COUNT (8)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)
    0x05, 0x09,        //     USAGE_PAGE (Button)
    0x19, 0x01,        //     USAGE_MINIMUM (Button 1)
    0x29, 0x08,        //     USAGE_MAXIMUM (Button 8)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x25, 0x01,        //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,        //     REPORT_COUNT (8)
    0x75, 0x01,        //     REPORT_SIZE (1)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)
    0xc0,              //   END_COLLECTION
    0xc0               // END_COLLECTION
};

/*!
    \brief      initialize the HID/CDC device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_hid_init (usb_dev *udev, uint8_t config_index)
{
    (void)(config_index);

    /* HID initialization */
    usbd_hid_cb.init(udev, config_index);

    /* CDC initialization */
    bf_cdc_class.init(udev, config_index);

    return USBD_OK;
}

/*!
    \brief      de-initialize the HID/CDC device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_hid_deinit (usb_dev *udev, uint8_t config_index)
{
    (void)(config_index);

    /* HID De-initialization */
    usbd_hid_cb.deinit(udev, config_index);

    /* CDC De-initialization */
    bf_cdc_class.deinit(udev, config_index);

    return USBD_OK;
}

/*!
    \brief      handle the custom HID/CDC class-specific request
    \param[in]  pudev: pointer to USB device instance
    \param[in]  req: device class request
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_hid_req_handler (usb_dev *udev, usb_req *req)
{
    if (req->wIndex == USBD_HID_INTERFACE) {
            usb_transc *transc = NULL;

            standard_hid_handler *hid = (standard_hid_handler *)udev->dev.class_data[USBD_HID_INTERFACE];

            switch (req->bRequest) {
            case GET_REPORT:
                break;

            case GET_IDLE:
                transc = &udev->dev.transc_in[0];
                transc->xfer_buf = (uint8_t *)&hid->idle_state;
                transc->remain_len = 1U;
                break;

            case GET_PROTOCOL:
                transc = &udev->dev.transc_in[0];
                transc->xfer_buf = (uint8_t *)&hid->protocol;
                transc->remain_len = 1U;
                break;

            case SET_REPORT:
                break;

            case SET_IDLE:
                hid->idle_state = (uint8_t)(req->wValue >> 8U);
                break;

            case SET_PROTOCOL:
                hid->protocol = (uint8_t)(req->wValue);
                break;

            case USB_GET_DESCRIPTOR:
                if (USB_DESCTYPE_REPORT == (req->wValue >> 8U)) {
                    transc = &udev->dev.transc_in[0];
                    transc->remain_len = USB_MIN(HID_MOUSE_REPORT_DESC_SIZE, req->wLength);
                    transc->xfer_buf = (uint8_t *)HID_MOUSE_ReportDesc;
                } else {
                    return USBD_FAIL;
                }
                break;

            default:
                return USBD_FAIL;
            }


    return USBD_OK;
    } else {
        return bf_cdc_class.req_proc(udev, req);
    }
}

static uint8_t cdc_hid_ctlx_out (usb_dev *udev)
{
    usb_req req = udev->dev.control.req;

    if (CDC_COM_INTERFACE == req.wIndex) {
        return bf_cdc_class.ctlx_out(udev);
    }

    return 0;
}

/*!
    \brief      handle data stage
    \param[in]  pudev: pointer to USB device instance
    \param[in]  rx_tx: data transfer direction:
      \arg        USBD_TX
      \arg        USBD_RX
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_hid_data_in (usb_dev *udev, uint8_t ep_num)
{
    if ((HID_IN_EP & 0x7FU) == ep_num) {
        standard_hid_handler *hid = (standard_hid_handler *)udev->dev.class_data[USBD_HID_INTERFACE];
        hid->prev_transfer_complete = 1U;
        return USBD_OK;
    }

    return bf_cdc_class.data_in(udev, ep_num);
}

/*!
    \brief      handle data stage
    \param[in]  pudev: pointer to USB device instance
    \param[in]  rx_tx: data transfer direction:
      \arg        USBD_TX
      \arg        USBD_RX
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_hid_data_out (usb_dev *udev, uint8_t ep_num)
{

    return bf_cdc_class.data_out(udev, ep_num);

}

static uint8_t cdc_hid_sof (usb_dev *udev)
{
    return bf_cdc_class.SOF ? bf_cdc_class.SOF(udev) : USBD_OK;
}

void sendReport(uint8_t *report, uint8_t len)
{
    standard_hid_handler *hid = (standard_hid_handler *)USB_OTG_dev.dev.class_data[USBD_HID_INTERFACE];

    if (USB_OTG_dev.dev.class_core != &bf_usbd_cdc_hid_cb
        || USB_OTG_dev.dev.cur_status != (uint8_t)USBD_CONFIGURED
        || hid == NULL
        || !hid->prev_transfer_complete) {
        return;
    }

    len = USB_MIN(len, HID_IN_PACKET);
    memcpy(hid->data, report, len);
    hid_report_send(&USB_OTG_dev, hid->data, len);
}
