/*!
 * @file        usbd_winusb.h
 *
 * @brief       usb device winUSB class handler header file
 *
 * @version     V1.0.0
 *
 * @date        2023-01-16
 *
 * @attention
 *
 *  Copyright (C) 2023 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be useful and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

/* Define to prevent recursive inclusion */
#ifndef _USBD_WINUSB_H_
#define _USBD_WINUSB_H_

/* Includes */
#include "usbd_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_WINUSB_Class
  @{
  */

/** @defgroup USBD_WINUSB_Macros Macros
  @{
*/

#define USBD_WINUSB_OS_FEATURE_DESC_SIZE            0x28
#define USBD_WINUSB_OS_PROPERTY_DESC_SIZE           0x8E

#define USBD_WINUSB_FS_MP_SIZE                      0x40
#define USBD_WINUSB_HS_MP_SIZE                      0x200
#define USBD_WINUSB_CMD_MP_SIZE                     0x08
#define USBD_WINUSB_DATA_MP_SIZE                    0x07

#define USBD_WINUSB_CMD_EP_ADDR                     0x82
#define USBD_WINUSB_DATA_IN_EP_ADDR                 0x81
#define USBD_WINUSB_DATA_OUT_EP_ADDR                0x01

#define USBD_WINUSB_FS_INTERVAL                     16
#define USBD_WINUSB_HS_INTERVAL                     16

/**@} end of group USBD_WINUSB_Macros*/

/** @defgroup USBD_WINUSB_Enumerates Enumerates
  @{
  */

/**
 * @brief   USB device WINUSB xfer status
 */
typedef enum
{
    USBD_WINUSB_XFER_IDLE,
    USBD_WINUSB_XFER_BUSY,
} USBD_WINUSB_XFER_STA_T;

/**@} end of group USBD_WINUSB_Enumerates*/

/** @defgroup USBD_WINUSB_Structures Structures
  @{
  */

/**
 * @brief   USB device WINUSB interface handler
 */
typedef struct
{
    const char*  itfName;
    USBD_STA_T (*ItfInit)(void);
    USBD_STA_T (*ItfDeInit)(void);
    USBD_STA_T (*ItfCtrl)(uint8_t command, uint8_t *buffer, uint16_t length);
    USBD_STA_T (*ItfSend)(uint8_t *buffer, uint16_t length);
    USBD_STA_T (*ItfSendEnd)(uint8_t epNum, uint8_t *buffer, uint32_t *length);
    USBD_STA_T (*ItfReceive)(uint8_t *buffer, uint32_t *length);
} USBD_WINUSB_INTERFACE_T;

/**
 * @brief   USB device WINUSB data handler
 */
typedef struct
{
    __IO uint8_t state;
    uint8_t *buffer;
    uint32_t length;
} USBD_WINUSB_DATA_XFER_T;

/**
 * @brief   USB device WINUSB command handler
 */
typedef struct
{
    uint8_t opcode;
    uint8_t length;
} USBD_WINUSB_CMD_XFER_T;

/**
 * @brief    WINUSB information management
 */
typedef struct
{
    uint8_t                     itf;
    uint8_t                     epInAddr;
    uint8_t                     epOutAddr;
    USBD_WINUSB_DATA_XFER_T     winusbTx;
    USBD_WINUSB_DATA_XFER_T     winusbRx;
    uint32_t                    data[USBD_WINUSB_HS_MP_SIZE / 4];
} USBD_WINUSB_INFO_T;

extern USBD_CLASS_T USBD_WINUSB_CLASS;

/**@} end of group USBD_WINUSB_Structures*/

/** @defgroup USBD_WINUSB_Functions Functions
  @{
  */

USBD_STA_T USBD_WINUSB_TxPacket(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_WINUSB_RxPacket(USBD_INFO_T* usbInfo);
uint8_t USBD_WINUSB_ReadInterval(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_WINUSB_ConfigTxBuffer(USBD_INFO_T* usbInfo, uint8_t *buffer, uint32_t length);
USBD_STA_T USBD_WINUSB_ConfigRxBuffer(USBD_INFO_T* usbInfo, uint8_t *buffer);
USBD_STA_T USBD_WINUSB_RegisterItf(USBD_INFO_T* usbInfo, USBD_WINUSB_INTERFACE_T* itf);

/**@} end of group USBD_WINUSB_Functions */
/**@} end of group USBD_WINUSB_Class */
/**@} end of group APM32_USB_Library */

#endif
