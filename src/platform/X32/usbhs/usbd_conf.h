/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

/* Includes ------------------------------------------------------------------*/
#include "usbhs_conf.h"


#define USBD_CFG_MAX_NUM           1
#define USBD_ITF_MAX_NUM           1

#define USB_MAX_STR_DESC_SIZ       255

#define USBD_SELF_POWERED


/************** USB DEVICE ENDPOINT CONFIGURATION *****************************/
#define MSC_IN_EP                      0x83
#define MSC_OUT_EP                     0x03

#define MSC_MEDIA_PACKET               4096
#ifdef USB_USB_HS_IN_HS
#define MSC_MAX_PACKET                 512
#else
#define MSC_MAX_PACKET                 64
#endif

#define CDC_IN_EP                      0x81u
#define CDC_OUT_EP                     0x01u
#define CDC_CMD_EP                     0x82u

#ifdef  USE_USB_HS_IN_FS
#define CDC_DATA_MAX_PACKET_SIZE       64   /* Endpoint IN & OUT Packet size */
#define CDC_IN_FRAME_INTERVAL          10    /* Number of micro-frames between IN transfers */
#endif

#ifdef  USE_USB_HS_IN_HS
#define CDC_DATA_MAX_PACKET_SIZE       512   /* Endpoint IN & OUT Packet size */
#define CDC_IN_FRAME_INTERVAL          10    /* Number of micro-frames between IN transfers */
#endif

#define CDC_CMD_PACKET_SIZE            8    /* Control Endpoint Packet size */
#define APP_RX_DATA_SIZE               2048 /* Total size of IN buffer: APP_RX_DATA_SIZE*8/MAX_BAUDARATE*1000 should be > CDC_IN_FRAME_INTERVAL*8 */

#define APP_FOPS                       VCP_fops

#endif /* __USBD_CONF__H__ */
