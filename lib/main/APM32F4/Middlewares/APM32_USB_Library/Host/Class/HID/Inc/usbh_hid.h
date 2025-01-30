/*!
 * @file        usbh_hid.h
 *
 * @brief       USB HID core function head file
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
#ifndef _USBH_HID_H_
#define _USBH_HID_H_

/* Includes */
#include "usbh_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_HID_Class
  @{
  */

/** @defgroup USBH_HID_Macros Macros
  @{
*/

#define USBH_HID_CLASS_CODE             0x03
#define USBH_HID_BOOT_CODE              0x01
#define USBH_HID_MOUSE_BOOT_CODE        0x02
#define USBH_HID_KEYBOARD_BOOT_CODE     0x01

#define USBH_HID_QUEUE_MAX_SIZE         10
#define USBH_HID_POLL_MIN_NUM           10

/**@} end of group USBH_HID_Macros*/

/** @defgroup USBH_HID_Enumerates Enumerates
  @{
  */

/**
 * @brief    USB HID state table
 */
typedef enum
{
    USBH_HID_INIT = 0,
    USBH_HID_IDLE = 1,
    USBH_HID_SYNC,
    USBH_HID_IN_DATA,
    USBH_HID_POLL,
    USBH_HID_OUT_DATA,
    USBH_HID_BUSY,
    USBH_HID_ERR,
} USBH_HID_STATE_T;

/**
 * @brief    USB HID request state table
 */
typedef enum
{
    USBH_HID_REQ_INIT,
    USBH_HID_REQ_IDLE,
    USBH_HID_REQ_GET_REP_DESC,
    USBH_HID_REQ_GET_HID_DESC,
    USBH_HID_REQ_SET_IDLE,
    USBH_HID_REQ_SET_PROTOCOL,
    USBH_HID_REQ_SET_REPORT,
} USBH_HID_REQ_STA_T;

/**
 * @brief    USB HID report type
 */
typedef enum
{
    HID_INPUT_REPORT = 1,
    HID_OUTPUT_REPORT,
    HID_FEATURE_REPORT,
} USBH_HID_REPORT_T;

/**
 * @brief   USB HID device class requests type
 */
typedef enum
{
    USBH_HID_GET_REPORT         = 1,
    USBH_HID_GET_IDLE,
    USBH_HID_GET_PROTOCOL,
    USBH_HID_SET_REPORT         = 9,
    USBH_HID_SET_IDLE,
    USBH_HID_SET_PROTOCOL,
} USBH_HID_REQ_TYPE_T;

/**@} end of group USBH_HID_Enumerates*/

/** @defgroup USBH_HID_Structures Structures
  @{
  */

/**
 * @brief    USB HID descriptor
 */
typedef struct
{
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bcdHID[2];
    uint8_t     bCountryCode;
    uint8_t     bNumDescriptors;
    uint8_t     bReportDescriptorType;
    uint8_t     wDescriptorLength[2];

} USBH_HID_DESC_T;

/* Host HID class state handler function */
typedef USBH_STA_T(*USBH_HIDStateHandler_T)(USBH_INFO_T* usbInfo);

/**
 * @brief   USB host HID class handler
 */
typedef struct
{
    USBH_STA_T(*InitHandler)(USBH_INFO_T* usbInfo);
    USBH_STA_T(*DecodeHandler)(USBH_INFO_T* usbInfo);
} USBH_HID_CLASS_T;

/**
 * @brief    HID information management
 */
typedef struct
{
    uint8_t*                 buffer;
    USBH_HID_STATE_T        state;
    uint8_t                 classReqState;
    uint8_t                 inChNum;
    uint8_t                 outChNum;
    uint8_t                 intOutEpAddr;
    uint8_t                 intInEpAddr;

    uint8_t                 epAddr;
    uint8_t                 epSize;
    uint16_t                pollInterval;
    uint8_t                 dataFlag;
    USBH_HID_DESC_T         desc;
    USBH_HID_CLASS_T*        callback;
    uint32_t                timer;
} USBH_HID_INFO_T;

extern USBH_CLASS_T USBH_HID_CLASS;

/**@} end of group USBH_HID_Structures*/

/** @defgroup USBH_HID_Functions Functions
  @{
  */

void USBH_HID_PollCallback(USBH_INFO_T* usbInfo);

/**@} end of group USBH_HID_Functions */
/**@} end of group USBH_HID_Class */
/**@} end of group APM32_USB_Library */

#endif
