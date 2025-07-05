/*!
    \file  cdc_hid_wrapper.h
    \brief the header file of cdc hid wrapper driver

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

#ifndef __USB_HID_CORE_H_
#define __USB_HID_CORE_H_

#include "usbd_desc.h"
#include "standard_hid_core.h"
#include "usbd_enum.h"

#define HID_CDC_CONFIG_DESC_SIZE                0x64U
#define HID_MOUSE_REPORT_DESC_SIZE              74

#pragma pack(1)

typedef struct
{
    usb_desc_header  header;              /*!< regular descriptor header containing the descriptor's type and length */
    uint8_t bFirstInterface;              /*!< bFirstInterface */
    uint8_t bInterfaceCount;              /*!< bInterfaceCount */
    uint8_t bFunctionClass;               /*!< bFunctionClass */
    uint8_t bFunctionSubClass;            /*!< bFunctionSubClass */
    uint8_t bFunctionProtocol;            /*!< bFunctionProtocol */
    uint8_t iFunction;                    /*!< iFunction  */
} usb_desc_IAD;

#pragma pack()

typedef struct
{
    usb_desc_config                           config;
    usb_desc_itf                              hid_interface;
    usb_desc_hid                              hid_vendor_hid;
    usb_desc_ep                               hid_report_in_endpoint;
    usb_desc_IAD                              iad;
    usb_desc_itf                              cmd_itf;
    usb_desc_header_func                      cdc_header;
    usb_desc_call_managment_func              cdc_call_managment;
    usb_desc_acm_func                         cdc_acm;
    usb_desc_union_func                       cdc_union;
    usb_desc_ep                               cdc_cmd_endpoint;
    usb_desc_itf                              cdc_data_interface;
    usb_desc_ep                               cdc_out_endpoint;
    usb_desc_ep                               cdc_in_endpoint;
} usb_cdc_hid_desc_config_set;

extern usb_desc bf_cdc_hid_desc;
extern usb_class_core bf_usbd_cdc_hid_cb;

#endif  /* __CDC_HID_WRAPPER_H */
