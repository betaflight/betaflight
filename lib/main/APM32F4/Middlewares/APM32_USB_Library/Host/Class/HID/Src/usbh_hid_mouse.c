/*!
 * @file        usbh_hid_mouse.c
 *
 * @brief       USB host HID mouse function
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
#include "usbh_hid_mouse.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_HID_Class
  @{
  */

/** @defgroup USBH_HID_Structures Structures
  @{
  */

USBH_HID_MOUSE_INFO_T usbHostHidMouse;

/* HID mouse class handler */
USBH_HID_CLASS_T USBH_HID_MOUSE_Handler =
{
    USBH_HID_MouseInit,
    USBH_HID_MouseDecode,
};

/**@} end of group USBH_HID_Structures*/

/** @defgroup USBH_HID_Functions Functions
  @{
  */

/*!
 * @brief       USB host HID init mouse
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_HID_MouseInit(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;
    uint16_t i;

    usbHostHidMouse.x = 0;
    usbHostHidMouse.y = 0;

    for (i = 0; i < USBH_HID_MOUSE_BUTTON_MAX_NUM; i++)
    {
        usbHostHidMouse.button[i] = 0;
    }

    for (i = 0; i < (sizeof(usbHostHidMouse.report.data) \
                     / sizeof(usbHostHidMouse.report.data[0])); i++)
    {
        usbHostHidMouse.report.data[i] = 0;
        usbHostHidMouse.report.rxBuffer[i] = 0;
    }

    if (usbHostHID->epSize > sizeof(usbHostHidMouse.report.data))
    {
        usbHostHID->epSize = (uint16_t)sizeof(usbHostHidMouse.report.data);
    }

    /* Init buffer point */
    usbHostHID->buffer = (uint8_t*)(void*)usbHostHidMouse.report.data;

    return usbStatus;
}

/*!
 * @brief       USB host HID decode mouse
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_HID_MouseDecode(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    uint8_t buttonTemp;
    uint8_t coordinateX;
    uint8_t coordinateY;
    uint8_t coordinateZ;

    buttonTemp = *((uint8_t*)(void*)usbHostHidMouse.report.data + 0);
    coordinateX = *((uint8_t*)(void*)usbHostHidMouse.report.data + 1);
    coordinateY = *((uint8_t*)(void*)usbHostHidMouse.report.data + 2);
    coordinateZ = *((uint8_t*)(void*)usbHostHidMouse.report.data + 3);

    usbHostHidMouse.button[0] = (buttonTemp & 0x01) > 0 ? 1 : 0;
    usbHostHidMouse.button[1] = (buttonTemp & 0x02) > 0 ? 1 : 0;
    usbHostHidMouse.button[2] = (buttonTemp & 0x04) > 0 ? 1 : 0;

    usbHostHidMouse.x         = coordinateX;
    usbHostHidMouse.y         = coordinateY;
    usbHostHidMouse.z         = coordinateZ;

    /* Process mouse data */
    USBH_HID_MouseCallback(usbInfo);

    return usbStatus;
}

/*!
 * @brief       USB host HID mouse data process
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
__weak USBH_STA_T USBH_HID_MouseCallback(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    USBH_USR_LOG("x:%02X", usbHostHidMouse.x);
    USBH_USR_LOG("y:%02X", usbHostHidMouse.y);
    USBH_USR_LOG("z:%02X", usbHostHidMouse.z);
    USBH_USR_LOG("b1:%02X", usbHostHidMouse.button[0]);
    USBH_USR_LOG("b2:%02X", usbHostHidMouse.button[1]);
    USBH_USR_LOG("b3:%02X", usbHostHidMouse.button[2]);
    USBH_USR_LOG(" ");

    return usbStatus;
}

/**@} end of group USBH_HID_Functions */
/**@} end of group USBH_HID_Class */
/**@} end of group APM32_USB_Library */
