/*!
 * @file        usbh_channel.h
 *
 * @brief       USB host channel handler function
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
#ifndef _USBH_CHANNEL_H_
#define _USBH_CHANNEL_H_

/* Includes */
#include "usbh_config.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Functions Functions
  @{
  */

void USBH_CH_Clear(USBH_INFO_T* usbInfo);
void USBH_CH_FreeChannel(USBH_INFO_T* usbInfo, uint8_t chNum);
uint8_t USBH_CH_AllocChannel(USBH_INFO_T* usbInfo, uint8_t epAddr);

/**@} end of group USBH_Core_Functions */
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */

#endif
