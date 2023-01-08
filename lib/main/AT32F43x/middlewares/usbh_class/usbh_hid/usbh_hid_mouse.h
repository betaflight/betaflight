/**
  **************************************************************************
  * @file     usbh_hid_mouse.h
  * @brief    usb host hid mouse header file
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
#ifndef __USBH_HID_MOUSE_H
#define __USBH_HID_MOUSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_conf.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @addtogroup USBH_hid_class_mouse
  * @{
  */

/** @defgroup USBH_hid_class_mouse_definition
  * @{
  */

/**
  * @brief  usb hid mouse x y
  */
#define MOUSE_WINDOW_X                   100
#define MOUSE_WINDOW_Y                   220
#define MOUSE_WINDOW_HEIGHT              90
#define MOUSE_WINDOW_WIDTH               128

  /**
  * @brief  usb hid mouse button
  */
#define MOUSE_BUTTON_LEFT                0x00
#define MOUSE_BUTTON_RIGHT               0x01
#define MOUSE_BUTTON_MIDDLE              0x02

/**
  * @brief  usb hid mouse type
  */
typedef struct
{
  uint8_t                                button;
  uint8_t                                x;
  uint8_t                                y;
  uint8_t                                z;
}usb_hid_mouse_type;

void usbh_hid_mouse_decode(uint8_t *mouse_data);


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

