/*!
    \file    usbd_desc.c
    \brief   CDC ACM driver

    \version 2024-12-20, V3.3.1, firmware for GD32F4xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

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

#include "usbd_desc.h"

#include "platform.h"
#include "build/version.h"

#include "pg/pg.h"
#include "pg/usb.h"

#define USBD_VID                          0x28E9U
#define USBD_PID                          0x018AU

volatile uint32_t APP_Rx_ptr_in  = 0;
volatile uint32_t APP_Rx_ptr_out = 0;
uint32_t APP_Rx_length  = 0;

uint8_t  USB_Tx_State = USB_CDC_IDLE;
__ALIGN_BEGIN uint8_t USB_Rx_Buffer   [USB_CDC_DATA_PACKET_SIZE] __ALIGN_END ;
__ALIGN_BEGIN uint8_t APP_Rx_Buffer   [APP_RX_DATA_SIZE] __ALIGN_END ;
__ALIGN_BEGIN uint8_t CmdBuff[USB_CDC_DATA_PACKET_SIZE] __ALIGN_END ;
extern CDC_IF_Prop_TypeDef  APP_FOPS;
extern LINE_CODING g_lc;
static void Handle_USBAsynchXfer  (usb_dev *udev);
extern void (*ctrlLineStateCb)(void* context, uint16_t ctrlLineState);
extern void *ctrlLineStateCbContext;
extern void (*baudRateCb)(void *context, uint32_t baud);
extern void *baudRateCbContext;

/* note:it should use the C99 standard when compiling the below codes */
/* USB standard device descriptor */
__ALIGN_BEGIN const usb_desc_dev bf_cdc_dev_desc __ALIGN_END = {
    .header =
    {
        .bLength          = USB_DEV_DESC_LEN,
        .bDescriptorType  = USB_DESCTYPE_DEV
    },
    .bcdUSB                = 0x0200U,
    .bDeviceClass          = USB_CLASS_CDC,
    .bDeviceSubClass       = 0x02U,
    .bDeviceProtocol       = 0x00U,
    .bMaxPacketSize0       = USB_FS_EP0_MAX_LEN,
    .idVendor              = USBD_VID,
    .idProduct             = USBD_PID,
    .bcdDevice             = 0x0200U,
    .iManufacturer         = STR_IDX_MFC,
    .iProduct              = STR_IDX_PRODUCT,
    .iSerialNumber         = STR_IDX_SERIAL,
    .bNumberConfigurations = USBD_CFG_MAX_NUM
};

/* USB device configuration descriptor */
__ALIGN_BEGIN const usb_cdc_desc_config_set bf_cdc_config_desc __ALIGN_END = {
    .config =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_config),
            .bDescriptorType = USB_DESCTYPE_CONFIG
        },
        .wTotalLength         = USB_CDC_ACM_CONFIG_DESC_SIZE,
        .bNumInterfaces       = 0x02U,
        .bConfigurationValue  = 0x01U,
        .iConfiguration       = 0x00U,
        .bmAttributes         = 0xC0U,
        .bMaxPower            = 0x32U
    },

    .cmd_itf =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF
        },
        .bInterfaceNumber     = 0x00U,
        .bAlternateSetting    = 0x00U,
        .bNumEndpoints        = 0x01U,
        .bInterfaceClass      = USB_CLASS_CDC,
        .bInterfaceSubClass   = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol   = USB_CDC_PROTOCOL_AT,
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
        .bDataInterface      = 0x01U
    },

    .cdc_acm =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_acm_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
        },
        .bDescriptorSubtype  = 0x02U,
        .bmCapabilities      = 0x02U
    },

    .cdc_union =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_union_func),
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
        },
        .bDescriptorSubtype  = 0x06U,
        .bMasterInterface    = 0x00U,
        .bSlaveInterface0    = 0x01U
    },

    .cdc_cmd_endpoint =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
        },
        .bEndpointAddress    = CDC_CMD_EP,
        .bmAttributes        = USB_EP_ATTR_INT,
        .wMaxPacketSize      = USB_CDC_CMD_PACKET_SIZE,
        .bInterval           = 0xFFU
    },

    .cdc_data_interface =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_itf),
            .bDescriptorType = USB_DESCTYPE_ITF
        },
        .bInterfaceNumber    = 0x01U,
        .bAlternateSetting   = 0x00U,
        .bNumEndpoints       = 0x02U,
        .bInterfaceClass     = USB_CLASS_DATA,
        .bInterfaceSubClass  = 0x00U,
        .bInterfaceProtocol  = USB_CDC_PROTOCOL_NONE,
        .iInterface          = 0x00U
    },

    .cdc_out_endpoint =
    {
        .header =
        {
            .bLength         = sizeof(usb_desc_ep),
            .bDescriptorType = USB_DESCTYPE_EP
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

/* USB language ID Descriptor */
static __ALIGN_BEGIN const usb_desc_LANGID usbd_language_id_desc __ALIGN_END = {
    .header =
    {
        .bLength         = sizeof(usb_desc_LANGID),
        .bDescriptorType = USB_DESCTYPE_STR
    },
    .wLANGID              = ENG_LANGID
};

/* USB manufacture string */
static __ALIGN_BEGIN usb_desc_str manufacturer_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(10U),
        .bDescriptorType = USB_DESCTYPE_STR
    },
    .unicode_string = {'G', 'i', 'g', 'a', 'D', 'e', 'v', 'i', 'c', 'e'}
};

/* USB product string */
static __ALIGN_BEGIN usb_desc_str product_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(12U),
        .bDescriptorType = USB_DESCTYPE_STR
    },
    .unicode_string = {'G', 'D', '3', '2', '-', 'C', 'D', 'C', '_', 'A', 'C', 'M'}
};

/* USBD serial string */
static __ALIGN_BEGIN usb_desc_str serial_string __ALIGN_END = {
    .header =
    {
        .bLength         = USB_STRING_LEN(12U),
        .bDescriptorType = USB_DESCTYPE_STR
    }
};

/* USB string descriptor set */
void *const usbd_bf_cdc_strings[] = {
    [STR_IDX_LANGID]  = (uint8_t *)&usbd_language_id_desc,
    [STR_IDX_MFC]     = (uint8_t *)&manufacturer_string,
    [STR_IDX_PRODUCT] = (uint8_t *)&product_string,
    [STR_IDX_SERIAL]  = (uint8_t *)&serial_string
};

usb_desc bf_cdc_desc = {
    .dev_desc    = (uint8_t *)&bf_cdc_dev_desc,
    .config_desc = (uint8_t *)&bf_cdc_config_desc,
    .strings     = usbd_bf_cdc_strings
};

/* local function prototypes ('static') */
static uint8_t bf_cdc_acm_init(usb_dev *udev, uint8_t config_index);
static uint8_t bf_cdc_acm_deinit(usb_dev *udev, uint8_t config_index);
static uint8_t bf_cdc_acm_req(usb_dev *udev, usb_req *req);
static uint8_t bf_cdc_acm_ctlx_out(usb_dev *udev);
static uint8_t bf_cdc_acm_in(usb_dev *udev, uint8_t ep_num);
static uint8_t bf_cdc_acm_out(usb_dev *udev, uint8_t ep_num);
static uint8_t bf_cdc_acm_sof(usb_dev *udev);

/* USB CDC device class callbacks structure */
usb_class_core bf_cdc_class = {
    .command   = NO_CMD,
    .alter_set = 0U,

    .init      = bf_cdc_acm_init,
    .deinit    = bf_cdc_acm_deinit,
    .req_proc  = bf_cdc_acm_req,
    .ctlx_out  = bf_cdc_acm_ctlx_out,
    .data_in   = bf_cdc_acm_in,
    .data_out  = bf_cdc_acm_out,
    .SOF       = bf_cdc_acm_sof
};

/*!
    \brief      usbd string get
    \param[in]  src: pointer to source string
    \param[in]  unicode : formatted string buffer (unicode)
    \param[in]  len : descriptor length
    \param[out] none
    \retval     USB device operation status
*/
static void usbd_string_buf_get(const char *src, uint8_t *unicode)
{
  uint8_t idx = 0;
  uint8_t len = 0;
  
  if (src != NULL) 
  {
    len =  USB_STRING_LEN(strlen(src));    
    unicode[idx++] = len;
    unicode[idx++] =  USB_DESCTYPE_STR;
    
    while (*src != '\0') 
    {
      unicode[idx++] = *src++;
      unicode[idx++] =  0x00;
    }
  } 
}


void usbd_desc_string_update(void)
{
#if defined(USBD_PRODUCT_STRING)
    usbd_string_buf_get(USBD_PRODUCT_STRING, (uint8_t *)&product_string);
#endif
#if defined(FC_FIRMWARE_NAME)
    usbd_string_buf_get(FC_FIRMWARE_NAME, (uint8_t *)&manufacturer_string);
#endif
}


/*!
    \brief      initialize the CDC ACM device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t bf_cdc_acm_init(usb_dev *udev, uint8_t config_index)
{
    (void)(config_index);

    static __ALIGN_BEGIN usb_cdc_handler cdc_handler __ALIGN_END;
    
    /* initialize the data TX endpoint */
    usbd_ep_setup(udev, &(bf_cdc_config_desc.cdc_in_endpoint));

    /* initialize the data RX endpoint */
    usbd_ep_setup(udev, &(bf_cdc_config_desc.cdc_out_endpoint));

    /* initialize the command TX endpoint */
    usbd_ep_setup(udev, &(bf_cdc_config_desc.cdc_cmd_endpoint));
    
    udev->dev.class_data[CDC_COM_INTERFACE] = (void *)&cdc_handler;
    
    APP_FOPS.pIf_Init();

    usbd_ep_recev(udev, CDC_DATA_OUT_EP, (uint8_t*)(USB_Rx_Buffer), USB_CDC_DATA_PACKET_SIZE);
    return USBD_OK;
}

/*!
    \brief      deinitialize the CDC ACM device
    \param[in]  udev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t bf_cdc_acm_deinit(usb_dev *udev, uint8_t config_index)
{
    (void)(config_index);
    
    /* deinitialize the data TX/RX endpoint */
    usbd_ep_clear(udev, CDC_DATA_IN_EP);
    usbd_ep_clear(udev, CDC_DATA_OUT_EP);

    /* deinitialize the command TX endpoint */
    usbd_ep_clear(udev, CDC_CMD_EP);

    APP_FOPS.pIf_DeInit();
    
    return USBD_OK;
}

/*!
    \brief      handle the CDC ACM class-specific requests
    \param[in]  udev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t bf_cdc_acm_req(usb_dev *udev, usb_req *req)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[0];
    usb_transc *transc = NULL;

    switch(req->bRequest) {
    case SEND_ENCAPSULATED_COMMAND:
        /* no operation for this driver */
        break;

    case GET_ENCAPSULATED_RESPONSE:
        /* no operation for this driver */
        break;

    case SET_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case GET_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case CLEAR_COMM_FEATURE:
        /* no operation for this driver */
        break;

    case SET_LINE_CODING:
        transc = &udev->dev.transc_out[0];

        /* set the value of the current command to be processed */
        udev->dev.class_core->alter_set = req->bRequest;

        /* enable EP0 prepare to receive command data packet */
        transc->remain_len = req->wLength;
        transc->xfer_buf = cdc->cmd;
        break;

    case GET_LINE_CODING:
        transc = &udev->dev.transc_in[0];

        cdc->cmd[0] = (uint8_t)(cdc->line_coding.dwDTERate);
        cdc->cmd[1] = (uint8_t)(cdc->line_coding.dwDTERate >> 8);
        cdc->cmd[2] = (uint8_t)(cdc->line_coding.dwDTERate >> 16);
        cdc->cmd[3] = (uint8_t)(cdc->line_coding.dwDTERate >> 24);
        cdc->cmd[4] = cdc->line_coding.bCharFormat;
        cdc->cmd[5] = cdc->line_coding.bParityType;
        cdc->cmd[6] = cdc->line_coding.bDataBits;

        transc->xfer_buf = cdc->cmd;
        transc->remain_len = 7U;
        break;

    case SET_CONTROL_LINE_STATE:
        transc = &udev->dev.transc_out[0];
        if (ctrlLineStateCb) {
            ctrlLineStateCb(ctrlLineStateCbContext, *((uint16_t *)CmdBuff));
        }
        transc->remain_len = req->wLength;
        transc->xfer_buf = CmdBuff;
        break;

    case SEND_BREAK:
        /* no operation for this driver */
        break;

    default:
        break;
    }

    return USBD_OK;
}

/*!
    \brief      command data received on control endpoint
    \param[in]  udev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t bf_cdc_acm_ctlx_out(usb_dev *udev)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[0];

    if(NO_CMD != udev->dev.class_core->alter_set) {
        /* process the command data */
        cdc->line_coding.dwDTERate = (uint32_t)((uint32_t)cdc->cmd[0] | \
                                                ((uint32_t)cdc->cmd[1] << 8) | \
                                                ((uint32_t)cdc->cmd[2] << 16) | \
                                                ((uint32_t)cdc->cmd[3] << 24));

        cdc->line_coding.bCharFormat = cdc->cmd[4];
        cdc->line_coding.bParityType = cdc->cmd[5];
        cdc->line_coding.bDataBits = cdc->cmd[6];

        udev->dev.class_core->alter_set = NO_CMD;
    }
    if (baudRateCb) {
        baudRateCb(baudRateCbContext, cdc->line_coding.dwDTERate);
    }

    return USBD_OK;
}

/*!
    \brief      handle CDC ACM data IN stage
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t bf_cdc_acm_in(usb_dev *udev, uint8_t ep_num)
{  
    (void)(ep_num);
    uint16_t USB_Tx_length;

    if (USB_Tx_State == USB_CDC_BUSY)
    {
        if (APP_Rx_length == 0)
        {
            USB_Tx_State = USB_CDC_IDLE;
        } else {
            if (APP_Rx_length > USB_CDC_DATA_PACKET_SIZE) {
                USB_Tx_length = USB_CDC_DATA_PACKET_SIZE;
            } else {
                USB_Tx_length = APP_Rx_length;

                if (USB_Tx_length == USB_CDC_DATA_PACKET_SIZE) {
                    USB_Tx_State = USB_CDC_ZLP;
                }
            }

            /* Prepare the available data buffer to be sent on IN endpoint */
            usbd_ep_send(udev, CDC_DATA_IN_EP, (uint8_t*)&APP_Rx_Buffer[APP_Rx_ptr_out], USB_Tx_length);

            // Advance the out pointer
            APP_Rx_ptr_out = (APP_Rx_ptr_out + USB_Tx_length) % APP_RX_DATA_SIZE;
            APP_Rx_length -= USB_Tx_length;

            return USBD_OK;
        }
    }

    /* Avoid any asynchronous transfer during ZLP */
    if (USB_Tx_State == USB_CDC_ZLP) {
        /*Send ZLP to indicate the end of the current transfer */
        usbd_ep_send(udev, CDC_DATA_IN_EP, NULL, 0);

        USB_Tx_State = USB_CDC_IDLE;
    }
    return USBD_OK;
}

/*!
    \brief      handle CDC ACM data OUT stage
    \param[in]  udev: pointer to USB device instance
    \param[in]  ep_num: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t bf_cdc_acm_out(usb_dev *udev, uint8_t ep_num)
{
  uint16_t USB_Rx_Cnt;
  
  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = ((usb_core_driver *)udev)->dev.transc_out[ep_num].xfer_count;
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */
  APP_FOPS.pIf_DataRx(USB_Rx_Buffer, USB_Rx_Cnt);
  
  /* Prepare Out endpoint to receive next packet */
  usbd_ep_recev(udev, CDC_DATA_OUT_EP, (uint8_t *)(uint8_t*)(USB_Rx_Buffer), USB_CDC_DATA_PACKET_SIZE);
  
  return USBD_OK;
}

/**
  * @brief  usbd_audio_SOF
  *         Start Of Frame event management
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t bf_cdc_acm_sof(usb_dev *udev)
{      
  static uint32_t FrameCount = 0;
  
  if (FrameCount++ == CDC_IN_FRAME_INTERVAL)
  {
    /* Reset the frame counter */
    FrameCount = 0;
    
    /* Check the data to be sent through IN pipe */
    Handle_USBAsynchXfer(udev);
  }
  
  return USBD_OK;
}

/**
  * @brief  Handle_USBAsynchXfer
  *         Send data to USB
  * @param  pdev: instance
  * @retval None
  */
static void Handle_USBAsynchXfer (usb_dev *udev)
{
    uint16_t USB_Tx_length;

    if (USB_Tx_State == USB_CDC_IDLE) {
        if (APP_Rx_ptr_out == APP_Rx_ptr_in) {
            // Ring buffer is empty
            return;
        }

        if (APP_Rx_ptr_out > APP_Rx_ptr_in) {
            // Transfer bytes up to the end of the ring buffer
            APP_Rx_length = APP_RX_DATA_SIZE - APP_Rx_ptr_out;
        } else {
            // Transfer all bytes in ring buffer
            APP_Rx_length = APP_Rx_ptr_in - APP_Rx_ptr_out;
        }

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
        // Only transfer whole 32 bit words of data
        APP_Rx_length &= ~0x03;
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

        if (APP_Rx_length > USB_CDC_DATA_PACKET_SIZE) {
            USB_Tx_length = USB_CDC_DATA_PACKET_SIZE;

            USB_Tx_State = USB_CDC_BUSY;
        } else {
            USB_Tx_length = APP_Rx_length;

            if (USB_Tx_length == USB_CDC_DATA_PACKET_SIZE) {
                USB_Tx_State = USB_CDC_ZLP;
            } else {
                USB_Tx_State = USB_CDC_BUSY;
            }
        }
    
        usbd_ep_send(udev, CDC_DATA_IN_EP, (uint8_t*)&APP_Rx_Buffer[APP_Rx_ptr_out], USB_Tx_length);

        APP_Rx_ptr_out = (APP_Rx_ptr_out + USB_Tx_length) % APP_RX_DATA_SIZE;
        APP_Rx_length -= USB_Tx_length;
    }
}
