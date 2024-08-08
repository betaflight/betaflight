/*!
 * @file        usbd_dataXfer.h
 *
 * @brief       USB device input and output hander function head file
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
#ifndef _USBD_DATAXFER_H_
#define _USBD_DATAXFER_H_

/* Includes */
#include "usbd_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_Core
  @{
  */

/** @defgroup USBD_Core_Functions Functions
  @{
  */

USBD_STA_T USBD_CtrlSendData(USBD_INFO_T* usbInfo, uint8_t* buffer, uint32_t length);
USBD_STA_T USBD_CtrlSendNextData(USBD_INFO_T* usbInfo, uint8_t* buffer, uint32_t length);
USBD_STA_T USBD_CtrlReceiveData(USBD_INFO_T* usbInfo, uint8_t* buffer, uint32_t length);
USBD_STA_T USBD_CtrlSendStatus(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_CtrlReceiveStatus(USBD_INFO_T* usbInfo);

USBD_STA_T USBH_SetupReqParse(uint8_t* buffer, USBD_REQ_SETUP_T* req);

/**@} end of group USBD_Core_Functions */
/**@} end of group USBD_Core */
/**@} end of group APM32_USB_Library */

#endif
