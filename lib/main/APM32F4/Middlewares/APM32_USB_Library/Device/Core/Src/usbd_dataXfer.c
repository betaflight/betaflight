/*!
 * @file        usbd_dataXfer.c
 *
 * @brief       USB device input and output hander function
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

/* Includes */
#include "usbd_dataXfer.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBD_Core
  @{
  */

/** @defgroup USBD_Core_Functions Functions
  @{
  */

/*!
 * @brief     Parse setup request
 *
 * @param     buffer : parse buffer
 *
 * @param     req : setup request data
 *
 * @retval    usb device status
 */
USBD_STA_T USBH_SetupReqParse(uint8_t* buffer, USBD_REQ_SETUP_T* req)
{
    USBD_STA_T usbStatus = USBD_OK;

    /* bmRequestType */
    req->REQ_DATA[0]            = *(uint8_t*)(buffer + 0);

    req->DATA_FIELD.bRequest    = *(uint8_t*)(buffer + 1);

    req->DATA_FIELD.wValue[0]   = *(uint8_t*)(buffer + 2);
    req->DATA_FIELD.wValue[1]   = *(uint8_t*)(buffer + 3);

    req->DATA_FIELD.wIndex[0]   = *(uint8_t*)(buffer + 4);
    req->DATA_FIELD.wIndex[1]   = *(uint8_t*)(buffer + 5);

    req->DATA_FIELD.wLength[0]  = *(uint8_t*)(buffer + 6);
    req->DATA_FIELD.wLength[1]  = *(uint8_t*)(buffer + 7);

    return usbStatus;
}

/*!
 * @brief     USB device continue receive CTRL data
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_CtrlReceiveData(USBD_INFO_T* usbInfo, uint8_t* buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    USBD_EP_ReceiveCallback(usbInfo, USBD_EP_0, buffer, length);

    return usbStatus;
}

/*!
 * @brief     USB device send CTRL status
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_CtrlSendStatus(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;
    
    usbInfo->devEp0State = USBD_DEV_EP0_STATUS_IN;

    USBD_EP_TransferCallback(usbInfo, USBD_EP_0, NULL, 0);

    return usbStatus;
}

/*!
 * @brief     USB device receive CTRL status
 *
 * @param     usbInfo : usb handler information
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_CtrlReceiveStatus(USBD_INFO_T* usbInfo)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->devEp0State = USBD_DEV_EP0_STATUS_OUT;

    USBD_EP_ReceiveCallback(usbInfo, USBD_EP_0, NULL, 0);

    return usbStatus;
}

/*!
 * @brief     USB device send CTRL data
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_CtrlSendData(USBD_INFO_T* usbInfo, uint8_t* buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    usbInfo->devEp0State = USBD_DEV_EP0_DATA_IN;
    usbInfo->devEpIn[USBD_EP_0].length = length;
    usbInfo->devEpIn[USBD_EP_0].remainLen = length;

    USBD_EP_TransferCallback(usbInfo, USBD_EP_0, buffer, length);

    return usbStatus;
}

/*!
 * @brief     USB device send next CTRL data
 *
 * @param     usbInfo : usb handler information
 *
 * @param     buffer : data buffer
 *
 * @param     length : length of data
 *
 * @retval    usb device status
 */
USBD_STA_T USBD_CtrlSendNextData(USBD_INFO_T* usbInfo, uint8_t* buffer, uint32_t length)
{
    USBD_STA_T usbStatus = USBD_OK;

    USBD_EP_TransferCallback(usbInfo, USBD_EP_0, buffer, length);

    return usbStatus;
}

/**@} end of group USBD_Core_Functions */
/**@} end of group USBD_Core */
/**@} end of group APM32_USB_Library */
