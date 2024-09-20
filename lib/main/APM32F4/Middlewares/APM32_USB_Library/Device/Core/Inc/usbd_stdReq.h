/*!
 * @file        usbd_stdReq.h
 *
 * @brief       USB standard request process
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
#ifndef _USBD_STDREQ_H_
#define _USBD_STDREQ_H_

/* Includes */
#include "usbd_core.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_Core
  @{
  */

/** @defgroup USBD_Core_Structures Structures
  @{
  */

extern USBD_StdDevReqCallback_T USBD_StdDevReqHandler[];

/**@} end of group USBD_Core_Structures*/

/** @defgroup USBD_Core_Functions Functions
  @{
  */

USBD_STA_T USBD_REQ_CtrlError(USBD_INFO_T* usbInfo, USBD_REQ_SETUP_T* req);

/**@} end of group USBD_Core_Functions */
/**@} end of group USBD_Core */
/**@} end of group APM32_USB_Library */

#endif
