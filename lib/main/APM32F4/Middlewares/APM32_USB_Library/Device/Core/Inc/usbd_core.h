/*!
 * @file        usbd_core.h
 *
 * @brief       USB device core function
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
#ifndef _USBD_CORE_H_
#define _USBD_CORE_H_

/* Includes */
#include "usbd_config.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_Core
  @{
  */

/** @defgroup USBD_Core_Enumerates Enumerates
  @{
  */

/**
 * @brief   USB device user status
 */
typedef enum
{
    USBD_USER_RESET = 1,
    USBD_USER_RESUME,
    USBD_USER_SUSPEND,
    USBD_USER_CONNECT,
    USBD_USER_DISCONNECT,
    USBD_USER_ENUM_DONE,
    USBD_USER_ERROR,
} USBD_USER_STATUS;

/**@} end of group USBD_Core_Enumerates*/

/** @defgroup USBD_Core_Functions Functions
  @{
  */

USBD_STA_T USBD_Init(USBD_INFO_T* usbInfo, USBD_SPEED_T usbDevSpeed, \
                     USBD_DESC_T* usbDevDesc, \
                     USBD_CLASS_T* usbDevClass, \
                     void (*userCallbackFunc)(struct _USBD_INFO_T*, uint8_t));
USBD_STA_T USBD_DeInit(USBD_INFO_T* usbInfo);
void USBD_HardwareInit(USBD_INFO_T* usbInfo);
void USBD_HardwareReset(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_SetSpeed(USBD_INFO_T* usbInfo, USBD_DEVICE_SPEED_T speed);
USBD_STA_T USBD_SetupStage(USBD_INFO_T* usbInfo, uint8_t* setup);
USBD_STA_T USBD_DataOutStage(USBD_INFO_T* usbInfo, uint8_t epNum, uint8_t* buffer);
USBD_STA_T USBD_DataInStage(USBD_INFO_T* usbInfo, uint8_t epNum, uint8_t* buffer);
USBD_STA_T USBD_Resume(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_Suspend(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_Reset(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_HandleSOF(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_IsoInInComplete(USBD_INFO_T* usbInfo, uint8_t epNum);
USBD_STA_T USBD_IsoOutInComplete(USBD_INFO_T* usbInfo, uint8_t epNum);
USBD_STA_T USBD_Connect(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_Disconnect(USBD_INFO_T* usbInfo);

void USBD_StartCallback(USBD_INFO_T* usbInfo);
void USBD_StopCallback(USBD_INFO_T* usbInfo);
void USBD_StopDeviceCallback(USBD_INFO_T* usbInfo);
USBD_STA_T USBD_EP_StallCallback(USBD_INFO_T* usbInfo, uint8_t epAddr);
USBD_STA_T USBD_EP_ClearStallCallback(USBD_INFO_T* usbInfo, uint8_t epAddr);
uint8_t USBD_EP_ReadStallStatusCallback(USBD_INFO_T* usbInfo, uint8_t epAddr);
void USBD_EP_OpenCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
                          uint8_t epType, uint16_t epMps);
void USBD_EP_CloseCallback(USBD_INFO_T* usbInfo, uint8_t epAddr);

uint32_t USBD_EP_ReadRxDataLenCallback(USBD_INFO_T* usbInfo, uint8_t epAddr);
USBD_STA_T USBD_EP_ReceiveCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
                                   uint8_t* buffer, uint32_t length);

USBD_STA_T USBD_EP_TransferCallback(USBD_INFO_T* usbInfo, uint8_t epAddr, \
                                    uint8_t* buffer, uint32_t length);
USBD_STA_T USBD_EP_FlushCallback(USBD_INFO_T* usbInfo, uint8_t epAddr);

USBD_STA_T USBD_SetDevAddressCallback(USBD_INFO_T* usbInfo, uint8_t address);

/**@} end of group USBD_Core_Functions */
/**@} end of group USBD_Core */
/**@} end of group APM32_USB_Library */

#endif
