/*!
    \file    usbd_conf.h
    \brief   the header file of USB device configuration

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

#ifndef USBD_CONF_H
#define USBD_CONF_H

#include "usb_conf.h"

/* USB configure exported defines */
#define USBD_CFG_MAX_NUM                1U
#define USBD_ITF_MAX_NUM                1U
#define USB_STR_DESC_MAX_SIZE           255U
#define USB_STRING_COUNT                4U

/* CDC */
#define CDC_DATA_IN_EP                  EP1_IN  /* EP1 for data IN */
#define CDC_DATA_OUT_EP                 EP1_OUT /* EP3 for data OUT */
#define CDC_CMD_EP                      EP2_IN  /* EP2 for CDC commands */

#define CDC_COM_INTERFACE               0U

#ifdef USE_USB_HS
    #ifdef USE_ULPI_PHY
        #define USB_CDC_DATA_PACKET_SIZE     512U
        #define USB_CDC_CMD_PACKET_SIZE      8U
        #define CDC_IN_FRAME_INTERVAL        40U
        #define APP_TX_DATA_SIZE             2048U
        #define APP_RX_DATA_SIZE             2048U
    #else
        #define USB_CDC_DATA_PACKET_SIZE     512U
        #define USB_CDC_CMD_PACKET_SIZE      8U
        #define CDC_IN_FRAME_INTERVAL        5U
        #define APP_TX_DATA_SIZE             2048U
        #define APP_RX_DATA_SIZE             2048U
    #endif
#else /*USE_USB_FS*/
        #define USB_CDC_DATA_PACKET_SIZE     64U
        #define USB_CDC_CMD_PACKET_SIZE      8U
        #define CDC_IN_FRAME_INTERVAL        15U
        #define APP_TX_DATA_SIZE             2048U
        #define APP_RX_DATA_SIZE             2048U
#endif
/* END CDC */

/* HID */
#define HID_IN_EP                       EP3_IN
#define HID_IN_PACKET                   9U
#define USBD_HID_INTERFACE              0U
/* END HID */

/* MSC */
#define MSC_IN_EP                       EP1_IN
#define MSC_OUT_EP                      EP1_OUT
#define USBD_MSC_INTERFACE              0U

#define MSC_MEDIA_PACKET_SIZE           4096U
#define MEM_LUN_NUM                     1U

#ifdef USE_USB_HS
    #ifdef USE_ULPI_PHY
        #define MSC_DATA_PACKET_SIZE    512U
    #else
        #define MSC_DATA_PACKET_SIZE    64U
    #endif
#else /*USE_USB_FS*/
    #define MSC_DATA_PACKET_SIZE        64U
#endif
/* END MSC */

#define APP_FOPS                        VCP_fops

#endif /* USBD_CONF_H */
