/*!
 * @file        usbh_dataXfer.h
 *
 * @brief       USB host input and output hander function head file
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
#ifndef _USBH_DATAXFER_H_
#define _USBH_DATAXFER_H_

/* Includes */
#include "usbh_config.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Structures Structures
  @{
  */

extern USBH_CtrlStateHandler_T USBH_CtrlStateHandler[];

/**@} end of group USBH_Core_Structures*/

/** @defgroup USBH_Core_Functions Functions
  @{
  */

USBH_STA_T USBH_BulkSendDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                uint8_t* buffer, uint16_t length, \
                                uint8_t pingStatus);

USBH_STA_T USBH_BulkReceiveDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                   uint8_t* buffer, uint16_t length);

USBH_STA_T USBH_IntSendDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                               uint8_t* buffer, uint8_t length);

USBH_STA_T USBH_IntReceiveDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                  uint8_t* buffer, uint8_t length);

USBH_STA_T USBH_IntSendDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                               uint8_t* buffer, uint8_t length);

USBH_STA_T USBH_IsoReceiveDataReq(USBH_INFO_T* usbInfo, uint8_t channelNum, \
                                  uint8_t* buffer, uint8_t length);

/**@} end of group USBH_Core_Functions */
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */

#endif
