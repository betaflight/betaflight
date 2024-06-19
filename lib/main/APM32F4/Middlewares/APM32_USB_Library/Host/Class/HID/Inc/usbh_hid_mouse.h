/*!
 * @file        usbh_hid_mouse.h
 *
 * @brief       USB host HID mouse function head file
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
#ifndef _USBH_HID_MOUSE_H_
#define _USBH_HID_MOUSE_H_

/* Includes */
#include "usbh_core.h"
#include "usbh_hid.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_HID_Class
  @{
  */

/** @defgroup USBH_HID_Macros Macros
  @{
*/

#define USBH_HID_MOUSE_BUTTON_MAX_NUM       3

/**@} end of group USBH_HID_Macros*/

/** @defgroup USBH_HID_Structures Structures
  @{
  */

/**
 * @brief    HID mouse report information
 */
typedef struct
{
    uint32_t data[2];
    uint32_t rxBuffer[2];
} USBH_HID_MOUSE_REPORT_T;

/**
 * @brief    HID mouse information management
 */
typedef struct
{
    uint8_t                     x;
    uint8_t                     y;
    uint8_t                     z;
    uint8_t                     button[USBH_HID_MOUSE_BUTTON_MAX_NUM];
    USBH_HID_MOUSE_REPORT_T     report;
} USBH_HID_MOUSE_INFO_T;

extern USBH_HID_MOUSE_INFO_T usbHostHidMouse;
extern USBH_HID_CLASS_T USBH_HID_MOUSE_Handler;

/**@} end of group USBH_HID_Structures*/

/** @defgroup USBH_HID_Functions Functions
  @{
  */

USBH_STA_T USBH_HID_MouseInit(USBH_INFO_T* usbInfo);
USBH_STA_T USBH_HID_MouseDecode(USBH_INFO_T* usbInfo);
USBH_STA_T USBH_HID_MouseCallback(USBH_INFO_T* usbInfo);

/**@} end of group USBH_HID_Functions */
/**@} end of group USBH_HID_Class */
/**@} end of group APM32_USB_Library */

#endif
