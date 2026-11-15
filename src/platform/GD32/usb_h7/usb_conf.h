/*!
    \file    usb_conf.h
    \brief   USB core driver basic configuration

    \version 2025-01-24, V1.4.0, firmware for GD32H7xx
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

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

#ifndef USB_CONF_H
#define USB_CONF_H

#include "gd32h7xx.h"

/* USB Core and PHY interface configuration */

/* USB HS PHY CONFIGURATION */

/* on-chip full-speed USB PHY */
#ifdef USE_USB_FS
    #define OC_FS_PHY
#endif

/* on-chip high-speed USB PHY */
#ifdef USE_USB_HS
    #define OC_HS_PHY
#endif /* USE_USB_HS */

/* USB FIFO size config */
#define RX_FIFO_SIZE                          512U
#define TX0_FIFO_SIZE                         128U
#define TX1_FIFO_SIZE                         384U
#define TX2_FIFO_SIZE                         0U
#define TX3_FIFO_SIZE                         0U
#define TX4_FIFO_SIZE                         0U
#define TX5_FIFO_SIZE                         0U
#define TX6_FIFO_SIZE                         0U
#define TX7_FIFO_SIZE                         0U


#ifdef USE_ULPI_PHY
    #define USB_EXTERNAL_ULPI_PHY_ENABLED
#else
    #ifdef OC_FS_PHY
         #define USB_EMBEDDED_FS_PHY_ENABLED
    #elif defined(OC_HS_PHY)
         #define USB_EMBEDDED_HS_PHY_ENABLED
    #else
         #error "PHY is not selected"
    #endif /* OC_FS_PHY */
#endif /* USE_ULPI_PHY */

//#define USB_INTERNAL_DMA_ENABLED
//#define USB_DEDICATED_EP1_ENABLED

#define USB_SOF_OUTPUT                        1U
#define USB_LOW_POWER                         0U

/* if uncomment it, need jump to USB JP */
//#define VBUS_SENSING_ENABLED

//#define USE_HOST_MODE
#define USE_DEVICE_MODE
//#define USE_OTG_MODE

#ifndef OC_FS_PHY
    #ifndef OC_HS_PHY
        #error  "OC_FS_PHY or OC_HS_PHY should be defined!"
    #endif
#endif /* OC_FS_PHY */

#ifndef USE_DEVICE_MODE
    #ifndef USE_HOST_MODE
        #error  "USE_DEVICE_MODE or USE_HOST_MODE should be defined!"
    #endif
#endif /* USE_DEVICE_MODE */

#ifndef USE_USB_HS
    #ifndef USE_USB_FS
        #error  "USE_USB_HS or USE_USB_FS should be defined!"
    #endif
#endif /* USE_USB_HS */

/* all variables and data structures during the transaction process should be 4-bytes aligned */


/* all variables and data structures during the transaction process should be 4-bytes aligned */
#if defined (__GNUC__)         /* GNU Compiler */
    #define __ALIGN_END __attribute__ ((aligned (4)))
    #define __ALIGN_BEGIN
#else
    #define __ALIGN_END

    #if defined (__CC_ARM)     /* ARM Compiler */
        #define __ALIGN_BEGIN __align(4)
    #elif defined (__ICCARM__) /* IAR Compiler */
        #define __ALIGN_BEGIN
    #elif defined (__TASKING__)/* TASKING Compiler */
        #define __ALIGN_BEGIN __align(4)
    #endif /* __CC_ARM */
#endif /* __GNUC__ */


/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__GNUC__)       /* GNU Compiler */
    #ifndef __packed
        #define __packed __attribute__((packed))
    #endif
#elif defined (__TASKING__)    /* TASKING Compiler */
    #define __packed __unaligned
#endif /* __GNUC__ */

#endif /* USB_CONF_H */
