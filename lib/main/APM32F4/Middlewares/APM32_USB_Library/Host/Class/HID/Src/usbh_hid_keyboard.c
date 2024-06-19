/*!
 * @file        usbh_hid_keyboard.c
 *
 * @brief       USB host HID keyboard function
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
#include "usbh_hid_keyboard.h"

/** @addtogroup APM32_USB_Library
  @{
  */

/** @addtogroup USBH_HID_Class
  @{
  */

/** @defgroup USBH_HID_Structures Structures
  @{
  */

USBH_HID_KEYBOARD_INFO_T usbHostHidKeyboard;

/* HID keyboard class handler */
USBH_HID_CLASS_T USBH_HID_KEYBOARD_Handler =
{
    USBH_HID_KeyBoardInit,
    USBH_HID_KeyboardDecode,
};

/**@} end of group USBH_HID_Structures*/

/** @defgroup USBH_HID_Functions Functions
  @{
  */

/*!
 * @brief       USB host HID init keyboard
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_HID_KeyBoardInit(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    USBH_HID_INFO_T* usbHostHID = (USBH_HID_INFO_T*)usbInfo->activeClass->classData;
    uint16_t i;

    usbHostHidKeyboard.ctrlLeft = 0;
    usbHostHidKeyboard.altLeft = 0;
    usbHostHidKeyboard.shiftLeft = 0;
    usbHostHidKeyboard.winLeft = 0;

    usbHostHidKeyboard.ctrlRight = 0;
    usbHostHidKeyboard.altRight = 0;
    usbHostHidKeyboard.shiftRight = 0;
    usbHostHidKeyboard.winRight = 0;

    for (i = 0; i < USBH_HID_KEY_PRESSED_NUM_MAX; i++)
    {
        usbHostHidKeyboard.key[i] = 0;
    }

    for (i = 0; i < (sizeof(usbHostHidKeyboard.report.data) \
                     / sizeof(usbHostHidKeyboard.report.data[0])); i++)
    {
        usbHostHidKeyboard.report.data[i] = 0;
        usbHostHidKeyboard.report.rxBuffer[i] = 0;
    }

    if (usbHostHID->epSize > sizeof(usbHostHidKeyboard.report.data))
    {
        usbHostHID->epSize = (uint16_t)sizeof(usbHostHidKeyboard.report.data);
    }

    /* Init buffer point */
    usbHostHID->buffer = (uint8_t*)(void*)usbHostHidKeyboard.report.data;

    return usbStatus;
}

/*!
 * @brief       USB host HID decode keyboard
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
USBH_STA_T USBH_HID_KeyboardDecode(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;

    uint8_t funKeyTemp;
    uint8_t i;

    if ((usbHostHidKeyboard.report.data[1] == 0x01010101) && \
            (usbHostHidKeyboard.report.data[0] == 0x01010000))
    {
        USBH_USR_LOG("Keyboard do not support this operation");
    }
    else
    {
        for (i = 2; i < 2 + USBH_HID_KEY_PRESSED_NUM_MAX; i++)
        {
            funKeyTemp = *((uint8_t*)(void*)usbHostHidKeyboard.report.data + i);
            if ((funKeyTemp == KEYBOARD_ERROR_ROLL_OVER) || \
                    (funKeyTemp == KEYBOARD_POST_FAIL) || \
                    (funKeyTemp == KEYBOARD_ERROR_UNDEFINED))
            {
                return USBH_FAIL;
            }
        }

        funKeyTemp = *((uint8_t*)(void*)usbHostHidKeyboard.report.data + 0);

        usbHostHidKeyboard.ctrlLeft     = (funKeyTemp & 0x01) > 0 ? 1 : 0;
        usbHostHidKeyboard.shiftLeft    = (funKeyTemp & 0x02) > 0 ? 1 : 0;
        usbHostHidKeyboard.altLeft      = (funKeyTemp & 0x04) > 0 ? 1 : 0;
        usbHostHidKeyboard.winLeft      = (funKeyTemp & 0x08) > 0 ? 1 : 0;

        usbHostHidKeyboard.ctrlRight    = (funKeyTemp & 0x10) > 0 ? 1 : 0;
        usbHostHidKeyboard.shiftRight   = (funKeyTemp & 0x20) > 0 ? 1 : 0;
        usbHostHidKeyboard.altRight     = (funKeyTemp & 0x40) > 0 ? 1 : 0;
        usbHostHidKeyboard.winRight     = (funKeyTemp & 0x80) > 0 ? 1 : 0;

        for (i = 0; i < USBH_HID_KEY_PRESSED_NUM_MAX; i++)
        {
            usbHostHidKeyboard.key[i] = *((uint8_t*)(void*)usbHostHidKeyboard.report.data + i + 2);
        }

        /* Process keyboard data */
        USBH_HID_KeyboardCallback(usbInfo);
    }

    return usbStatus;
}

/*!
 * @brief       USB host HID keyboard data process
 *
 * @param       usbInfo: usb host information
 *
 * @retval      USB host operation status
 */
__weak USBH_STA_T USBH_HID_KeyboardCallback(USBH_INFO_T* usbInfo)
{
    USBH_STA_T  usbStatus = USBH_OK;
    uint8_t i;

    USBH_USR_LOG("CTRL  L:%02X", usbHostHidKeyboard.ctrlLeft);
    USBH_USR_LOG("SHIFT L:%02X", usbHostHidKeyboard.shiftLeft);
    USBH_USR_LOG("ALT   L:%02X", usbHostHidKeyboard.altLeft);
    USBH_USR_LOG("WIN   L:%02X", usbHostHidKeyboard.winLeft);

    USBH_USR_LOG("CTRL  R:%02X", usbHostHidKeyboard.ctrlRight);
    USBH_USR_LOG("SHIFT R:%02X", usbHostHidKeyboard.shiftRight);
    USBH_USR_LOG("ALT   R:%02X", usbHostHidKeyboard.altRight);
    USBH_USR_LOG("WIN   R:%02X", usbHostHidKeyboard.winRight);

    for (i = 0; i < USBH_HID_KEY_PRESSED_NUM_MAX; i++)
    {
        USBH_USR_LOG("KEY[%d] :%02X", i, usbHostHidKeyboard.key[i]);
    }

    USBH_USR_LOG(" ");

    return usbStatus;
}

/**@} end of group USBH_HID_Functions */
/**@} end of group USBH_HID_Class */
/**@} end of group APM32_USB_Library */
