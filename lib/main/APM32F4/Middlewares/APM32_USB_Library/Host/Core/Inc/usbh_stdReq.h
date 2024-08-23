/*!
 * @file        usbh_stdReq.h
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
#ifndef _USBH_STDREQ_H_
#define _USBH_STDREQ_H_

/* Includes */
#include "usbh_config.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_Core
  @{
  */

/** @defgroup USBH_Core_Macros Macros
  @{
*/

#define USBH_SETUP_PACKET_SIZE          8
#define USBH_ReadConfigurationItfNum(usbInfo)  \
        usbInfo->devInfo.desc.configuration.bNumInterfaces

#define USBH_ReadInterfaceClass(usbInfo, itfIndex) \
        usbInfo->devInfo.desc.interface[itfIndex].interfaceDesc.bInterfaceClass

#define USBH_ReadInterfaceSubClass(usbInfo, itfIndex) \
        usbInfo->devInfo.desc.interface[itfIndex].interfaceDesc.bInterfaceSubClass

#define USBH_ReadInterfaceProtocol(usbInfo, itfIndex) \
        usbInfo->devInfo.desc.interface[itfIndex].interfaceDesc.bInterfaceProtocol

#define USBH_ReadInterfaceEpNum(usbInfo, itfIndex) \
        usbInfo->devInfo.desc.interface[itfIndex].interfaceDesc.bNumEndpoints

#define USBH_ReadEndpointAddress(usbInfo, itfIndex, epIndex) \
        usbInfo->devInfo.desc.interface[itfIndex].endpointDesc[epIndex].bEndpointAddress

#define USBH_ReadEndpointMPS(usbInfo, itfIndex, epIndex) \
        ((uint16_t)usbInfo->devInfo.desc.interface[itfIndex].endpointDesc[epIndex].wMaxPacketSize[0] | \
        (uint16_t)usbInfo->devInfo.desc.interface[itfIndex].endpointDesc[epIndex].wMaxPacketSize[1] << 8)

#define USBH_ReadEndpointInterval(usbInfo, itfIndex, epIndex) \
        usbInfo->devInfo.desc.interface[itfIndex].endpointDesc[epIndex].bInterval

/**@} end of group USBH_Core_Macros*/

/** @defgroup USBH_Core_Functions Functions
  @{
  */

USBH_STA_T USBH_GetDevDesc(USBH_INFO_T* usbInfo, uint8_t desLength);
USBH_STA_T USBH_SetAddr(USBH_INFO_T* usbInfo, uint8_t address);
USBH_STA_T USBH_GetCfgDesc(USBH_INFO_T* usbInfo, uint16_t desLength);
USBH_STA_T USBH_GetStringDesc(USBH_INFO_T* usbInfo, uint8_t stringIndex, \
                              uint8_t* buffer, uint16_t desLength);

USBH_STA_T USBH_REQ_GetDescriptor(USBH_INFO_T* usbInfo, uint8_t reqType, \
                                  uint16_t desType, uint8_t* buffer, uint16_t length);

USBH_STA_T USBH_SetConfiguration(USBH_INFO_T* usbInfo, uint16_t configuration);
USBH_STA_T USBH_SetFeature(USBH_INFO_T* usbInfo, uint8_t feature);
USBH_STA_T USBH_ClearFeature(USBH_INFO_T* usbInfo, uint8_t epNum);

USBH_STA_T USBH_REQ_CtrlXferHandler(USBH_INFO_T* usbInfo, uint8_t* buffer, uint16_t length);

/**@} end of group USBH_Core_Functions */
/**@} end of group USBH_Core */
/**@} end of group APM32_USB_Library */

#endif
