/**
  **************************************************************************
  * @file     usbh_hid_keyboard.h
  * @brief    usb host hid keyboard header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBH_HID_KEYBOARD_H
#define __USBH_HID_KEYBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_conf.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @addtogroup USBH_hid_class_keyboard
  * @{
  */

/** @defgroup USBH_hid_class_keyboard_definition
  * @{
  */

  /**
  * @brief  usb keyboard option code
  */
#define KEYBOARD_LEFT_CTRL               0x01
#define KEYBOARD_LEFT_SHIFT              0x02
#define KEYBOARD_LEFT_ALT                0x04
#define KEYBOARD_LEFT_GUI                0x08
#define KEYBOARD_RIGHT_CTRL              0x10
#define KEYBOARD_RIGHT_SHIFT             0x20
#define KEYBOARD_RIGHT_ALT               0x40
#define KEYBOARD_RIGHT_GUI               0x80

#define KEYBOARD_MAX_NB_PRESSED          6

#ifndef AZERTY_KEYBOARD
  #define QWERTY_KEYBOARD
#endif

void usbh_hid_keyboard_decode(uint8_t *data);


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif
