/*!
 * @file        usbd_cdc.h
 *
 * @brief       usb device cdc class handler header file
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
#ifndef _USBD_CDC_H_
#define _USBD_CDC_H_

/* Includes */
#include "usbd_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_CDC_Class
  @{
  */

/** @defgroup USBD_CDC_Macros Macros
  @{
*/

#define USBD_CDC_FS_MP_SIZE                     0x40
#define USBD_CDC_HS_MP_SIZE                     0x200
#define USBD_CDC_CMD_MP_SIZE                    0x08
#define USBD_CDC_DATA_MP_SIZE                   0x07

#define USBD_CDC_CMD_EP_ADDR                    0x82
#define USBD_CDC_DATA_IN_EP_ADDR                0x81
#define USBD_CDC_DATA_OUT_EP_ADDR               0x01

#define USBD_CDC_FS_INTERVAL                    16
#define USBD_CDC_HS_INTERVAL                    16

/**@} end of group USBD_CDC_Macros*/

/** @defgroup USBD_CDC_Enumerates Enumerates
  @{
  */

/**
 * @brief   USB device CDC xfer status
 */
typedef enum
{
    USBD_CDC_XFER_IDLE,
    USBD_CDC_XFER_BUSY,
} USBD_CDC_XFER_STA_T;

/**
 * @brief   USB device CDC control status
 */
typedef enum
{
    USBD_CDC_SEND_ENCAPSULATED_COMMAND          = 0x00,
    USBD_CDC_GET_ENCAPSULATED_RESPONSE          = 0x01,
    USBD_CDC_SET_COMM_FEATURE                   = 0x02,
    USBD_CDC_GET_COMM_FEATURE                   = 0x03,
    USBD_CDC_CLEAR_COMM_FEATURE                 = 0x04,
    USBD_CDC_SET_LINE_CODING                    = 0x20,
    USBD_CDC_GET_LINE_CODING                    = 0x21,
    USBD_CDC_SET_CONTROL_LINE_STATE             = 0x22,
    USBD_CDC_SEND_BREAK                         = 0x23,
} USBD_CDC_CTRL_STA_T;

/**@} end of group USBD_CDC_Enumerates*/

/** @defgroup USBD_CDC_Structures Structures
  @{
  */

/**
 * @brief   USB device CDC Line Coding Structure
 */
typedef struct
{
    uint32_t    baudRate;
    uint8_t     format;
    uint8_t     parityType;
    uint8_t     WordLen;
} USBD_CDC_LINE_CODING_T;

/**
 * @brief   USB device CDC interface handler
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
    USBD_STA_T (*ItfSOF)(void);
} USBD_CDC_INTERFACE_T;

/**
 * @brief   USB device CDC data handler
 */
typedef struct
{
    __IO uint8_t state;
    uint8_t *buffer;
    uint32_t length;
} USBD_CDC_DATA_XFER_T;

/**
 * @brief   USB device CDC command handler
 */
typedef struct
{
    uint8_t opcode;
    uint8_t length;
} USBD_CDC_CMD_XFER_T;

/**
 * @brief    CDC information management
 */
typedef struct
{
    uint8_t                 itf;
    uint8_t                 epInAddr;
    uint8_t                 epOutAddr;
    uint8_t                 epCmdAddr;
    USBD_CDC_DATA_XFER_T    cdcTx;
    USBD_CDC_DATA_XFER_T    cdcRx;
    uint32_t                data[USBD_CDC_HS_MP_SIZE / 4];
    USBD_CDC_CMD_XFER_T     cdcCmd;
} USBD_CDC_INFO_T;

extern USBD_CLASS_T USBD_CDC_CLASS;

/**@} end of group USBD_CDC_Structures*/

/** @defgroup USBD_CDC_Functions Functions
  @{
  */

USBD_STA_T USBD_CDC_TxPacket(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_CDC_RxPacket(USBD_INFO_T* usbInfo);
uint8_t USBD_CDC_ReadInterval(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_CDC_ConfigTxBuffer(USBD_INFO_T* usbInfo, uint8_t *buffer, uint32_t length);
USBD_STA_T USBD_CDC_ConfigRxBuffer(USBD_INFO_T* usbInfo, uint8_t *buffer);
USBD_STA_T USBD_CDC_RegisterItf(USBD_INFO_T* usbInfo, USBD_CDC_INTERFACE_T* itf);

/**@} end of group USBD_CDC_Functions */
/**@} end of group USBD_CDC_Class */
/**@} end of group APM32_USB_Library */

#endif
