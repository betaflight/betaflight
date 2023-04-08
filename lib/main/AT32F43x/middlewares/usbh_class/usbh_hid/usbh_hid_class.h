/**
  **************************************************************************
  * @file     usbh_hid_class.h
  * @brief    usb host hid class header file
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
#ifndef __USBH_HID_CLASS_H
#define __USBH_HID_CLASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbh_core.h"
#include "usb_conf.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @addtogroup USBH_hid_class
  * @{
  */

/** @defgroup USBH_hid_class_definition
  * @{
  */

/**
  * @brief  usb hid protocol code
  */
#define USB_HID_NONE_PROTOCOL_CODE       0x00
#define USB_HID_KEYBOARD_PROTOCOL_CODE   0x01
#define USB_HID_MOUSE_PROTOCOL_CODE      0x02

/**
  * @brief  usb hid request code
  */
#define USB_HID_GET_REPORT               0x01
#define USB_HID_GET_IDLE                 0x02
#define USB_HID_GET_PROTOCOL             0x03
#define USB_HID_SET_REPORT               0x09
#define USB_HID_SET_IDLE                 0x0A
#define USB_HID_SET_PROTOCOL             0x0B

/**
  * @brief  usb hid request state
  */
typedef enum
{
  USB_HID_STATE_IDLE,
  USB_HID_STATE_GET_DESC,
  USB_HID_STATE_GET_REPORT,
  USB_HID_STATE_SET_IDLE,
  USB_HID_STATE_SET_PROTOCOL,
  USB_HID_STATE_COMPLETE,
}usb_hid_ctrl_state_type;

/**
  * @brief  usb hid process state
  */
typedef enum
{
  USB_HID_INIT,
  USB_HID_GET,
  USB_HID_SEND,
  USB_HID_POLL,
  USB_HID_BUSY,
  USB_HID_ERROR,
}usb_hid_state_type;

/**
  * @brief  usb hid descriptor type
  */
typedef struct
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdHID;
  uint8_t bCountryCode;
  uint8_t bNumDescriptors;
  uint8_t bReportDescriptorType;
  uint16_t wItemLength;
}usb_hid_desc_type;

/**
  * @brief  usb hid struct
  */
typedef struct
{
  uint8_t                                chin;
  uint8_t                                eptin;
  uint16_t                               in_maxpacket;
  uint8_t                                in_poll;

  uint8_t                                chout;
  uint8_t                                eptout;
  uint16_t                               out_maxpacket;
  uint8_t                                out_poll;
  uint8_t                                protocol;


  usb_hid_desc_type                      hid_desc;
  usb_hid_ctrl_state_type                ctrl_state;
  usb_hid_state_type                     state;
  uint16_t                               poll_timer;
  uint32_t buffer[16];
}usbh_hid_type;

extern usbh_class_handler_type uhost_hid_class_handler;


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
