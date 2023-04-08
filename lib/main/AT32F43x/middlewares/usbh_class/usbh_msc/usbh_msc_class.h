/**
  **************************************************************************
  * @file     usbh_msc_class.h
  * @brief    usb host msc class header file
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
#ifndef __USBH_MSC_CLASS_H
#define __USBH_MSC_CLASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbh_core.h"
#include "usb_conf.h"
#include "usbh_msc_bot_scsi.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @addtogroup USBH_msc_class
  * @{
  */

/** @defgroup USBH_msc_class_definition
  * @{
  */

/**
  * @brief  usb msc subclass code
  */
#define MSC_SUBCLASS_SCSI_TRANS          0x06

/**
  * @brief  usb msc protocol code
  */
#define MSC_PROTOCOL_BBB                 0x50

/**
  * @brief  usb msc request code
  */
#define MSC_REQ_GET_MAX_LUN              0xFE
#define MSC_REQ_BOMSR                    0xFF

/**
  * @brief  usb hid request code
  */
#define USB_HID_GET_REPORT               0x01
#define USB_HID_GET_IDLE                 0x02
#define USB_HID_GET_PROTOCOL             0x03
#define USB_HID_SET_REPORT               0x09
#define USB_HID_SET_IDLE                 0x0A
#define USB_HID_SET_PROTOCOL             0x0B


#define USBH_SUPPORT_MAX_LUN             0x2

/**
  * @brief  usb msc request state
  */
typedef enum
{
  USBH_MSC_STATE_IDLE,
  USBH_MSC_STATE_GET_LUN,
  USBH_MSC_STATE_ERROR,
  USBH_MSC_STATE_COMPLETE,
}usbh_msc_ctrl_state_type;

/**
  * @brief  usb msc struct
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

  uint32_t                               max_lun;
  uint32_t                               cur_lun;
  uint32_t                               use_lun;


  usbh_msc_ctrl_state_type               ctrl_state;
  msc_state_type                         state;
  uint8_t                                error;
  msc_bot_trans_type                     bot_trans;
  usbh_msc_unit_type                     l_unit_n[USBH_SUPPORT_MAX_LUN];
  uint16_t                               poll_timer;
  uint8_t buffer[64];
}usbh_msc_type;

extern usbh_class_handler_type uhost_msc_class_handler;
extern usbh_msc_type usbh_msc;
msc_error_type usbh_msc_is_ready(void *uhost, uint8_t lun);
usb_sts_type usbh_msc_write(void *uhost, uint32_t address, uint32_t len, uint8_t *buffer, uint8_t lun);
usb_sts_type usbh_msc_read(void *uhost, uint32_t address, uint32_t len, uint8_t *buffer, uint8_t lun);
usb_sts_type usbh_msc_rw_handle(void *uhost, uint32_t address, uint32_t len, uint8_t *buffer, uint8_t lun);
usb_sts_type msc_bot_scsi_init(usbh_msc_type *msc_struct);

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
