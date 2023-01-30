/**
  **************************************************************************
  * @file     usbh_hid_class.c
  * @brief    usb host hid class type
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
 #include "usbh_hid_class.h"
 #include "usb_conf.h"
 #include "usbh_core.h"
 #include "usbh_ctrl.h"
 #include "usbh_hid_mouse.h"
 #include "usbh_hid_keyboard.h"

 /** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @defgroup USBH_hid_class
  * @brief usb host class hid demo
  * @{
  */

/** @defgroup USBH_hid_class_private_functions
  * @{
  */

 static usb_sts_type uhost_init_handler(void *uhost);
 static usb_sts_type uhost_reset_handler(void *uhost);
 static usb_sts_type uhost_request_handler(void *uhost);
 static usb_sts_type uhost_process_handler(void *uhost);

 usbh_hid_type usbh_hid;
 usbh_class_handler_type uhost_hid_class_handler =
 {
   uhost_init_handler,
   uhost_reset_handler,
   uhost_request_handler,
   uhost_process_handler,
   &usbh_hid
 };

/**
  * @brief  usb host class init handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_init_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_OK;
  uint8_t hidx, eptidx = 0;
  usbh_hid_type *phid =  &usbh_hid;

  puhost->class_handler->pdata = &usbh_hid;

  /* get hid interface */
  hidx = usbh_find_interface(puhost, USB_CLASS_CODE_HID, 0x01, 0xFF);
  if(hidx == 0xFF)
  {
    USBH_DEBUG("Unsupport Device!");
    return USB_NOT_SUPPORT;
  }

  /* get hid protocol */
  phid->protocol = puhost->dev.cfg_desc.interface[hidx].interface.bInterfaceProtocol;

  if(phid->protocol == USB_HID_MOUSE_PROTOCOL_CODE)
  {
    USBH_DEBUG("Mouse Device!");
  }
  else if(phid->protocol == USB_HID_KEYBOARD_PROTOCOL_CODE)
  {
    USBH_DEBUG("Keyboard Device!");
  }

  for(eptidx = 0; eptidx < puhost->dev.cfg_desc.interface[hidx].interface.bNumEndpoints; eptidx ++)
  {
    if(puhost->dev.cfg_desc.interface[hidx].endpoint[eptidx].bEndpointAddress & 0x80)
    {
      /* find interface out endpoint information */
      phid->eptin = puhost->dev.cfg_desc.interface[hidx].endpoint[eptidx].bEndpointAddress;
      phid->in_maxpacket = puhost->dev.cfg_desc.interface[hidx].endpoint[eptidx].wMaxPacketSize;
      phid->in_poll = puhost->dev.cfg_desc.interface[hidx].endpoint[eptidx].bInterval;

      phid->chin = usbh_alloc_channel(puhost, phid->eptin);
      /* enable channel */
      usbh_hc_open(puhost, phid->chin,phid->eptin,
                    puhost->dev.address, EPT_INT_TYPE,
                    phid->in_maxpacket,
                    puhost->dev.speed);
      usbh_set_toggle(puhost, phid->chin, 0);
    }
    else
    {
      /* get interface out endpoint information */
      phid->eptout = puhost->dev.cfg_desc.interface[hidx].endpoint[eptidx].bEndpointAddress;
      phid->out_maxpacket = puhost->dev.cfg_desc.interface[hidx].endpoint[eptidx].wMaxPacketSize;
      phid->out_poll = puhost->dev.cfg_desc.interface[hidx].endpoint[eptidx].bInterval;

      phid->chout = usbh_alloc_channel(puhost, usbh_hid.eptout);
      /* enable channel */
      usbh_hc_open(puhost, phid->chout, phid->eptout,
                    puhost->dev.address, EPT_INT_TYPE,
                    phid->out_maxpacket,
                    puhost->dev.speed);
      usbh_set_toggle(puhost, phid->chout, 0);
    }
  }
  phid->ctrl_state = USB_HID_STATE_IDLE;
  return status;
}

/**
  * @brief  usb host class reset handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_reset_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_hid_type *phid =  (usbh_hid_type *)puhost->class_handler->pdata;
  usb_sts_type status = USB_OK;
  if(puhost->class_handler->pdata == NULL)
  {
    return status;
  }

  if(phid->chin != 0)
  {
    /* free in channel */
    usbh_free_channel(puhost, phid->chin);
    usbh_ch_disable(puhost, phid->chin);
    phid->chin = 0;
  }

  if(phid->chout != 0)
  {
    /* free out channel */
    usbh_free_channel(puhost, phid->chout);
    usbh_ch_disable(puhost, phid->chout);
    phid->chout = 0;
  }

  return status;
}

/**
  * @brief  usb host hid class get descriptor
  * @param  uhost: to the structure of usbh_core_type
  * @param  length: descriptor length
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_hid_get_desc(void *uhost, uint16_t length)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_hid_type *phid =  (usbh_hid_type *)puhost->class_handler->pdata;
  usb_sts_type status = USB_WAIT;
  uint8_t bm_req;
  uint16_t wvalue;
  if(puhost->ctrl.state == CONTROL_IDLE)
  {
    bm_req = USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_STANDARD;
    wvalue = (0x21 << 8) & 0xFF00;

    usbh_get_descriptor(puhost, length, bm_req,
                                 wvalue, puhost->rx_buffer);
  }
  else
  {
    if(usbh_ctrl_result_check(puhost, CONTROL_IDLE, ENUM_IDLE) == USB_OK)
    {
      phid->hid_desc.bLength = puhost->rx_buffer[0];
      phid->hid_desc.bDescriptorType = puhost->rx_buffer[1];
      phid->hid_desc.bcdHID = SWAPBYTE(puhost->rx_buffer+2);
      phid->hid_desc.bCountryCode = puhost->rx_buffer[4];
      phid->hid_desc.bNumDescriptors = puhost->rx_buffer[5];
      phid->hid_desc.bReportDescriptorType = puhost->rx_buffer[6];
      phid->hid_desc.wItemLength = SWAPBYTE(puhost->rx_buffer+7);
      status = USB_OK;
    }
  }
  return status;
}


/**
  * @brief  usb host hid class get report
  * @param  uhost: to the structure of usbh_core_type
  * @param  length: reprot length
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_hid_get_report(void *uhost, uint16_t length)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_WAIT;
  uint8_t bm_req;
  uint16_t wvalue;
  if(puhost->ctrl.state == CONTROL_IDLE)
  {
    bm_req = USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_STANDARD;
    wvalue = (0x22 << 8) & 0xFF00;

    usbh_get_descriptor(puhost, length, bm_req,
                                 wvalue, puhost->rx_buffer);
  }
  else
  {
    if(usbh_ctrl_result_check(puhost, CONTROL_IDLE, ENUM_IDLE) == USB_OK)
    {
      status = USB_OK;
    }
  }
  return status;
}

/**
  * @brief  usb host hid class set idle
  * @param  uhost: to the structure of usbh_core_type
  * @param  id: id
  * @param  dr: dr
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_hid_set_idle(void *uhost, uint8_t id, uint8_t dr)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_WAIT;
  if(puhost->ctrl.state == CONTROL_IDLE)
  {
    puhost->ctrl.setup.bmRequestType = USB_DIR_H2D | USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_CLASS;
    puhost->ctrl.setup.bRequest = USB_HID_SET_IDLE;
    puhost->ctrl.setup.wValue = (dr << 8) | id;
    puhost->ctrl.setup.wIndex = 0;
    puhost->ctrl.setup.wLength = 0;
    usbh_ctrl_request(puhost, 0, 0);
  }
  else
  {
    status = usbh_ctrl_result_check(puhost, CONTROL_IDLE, ENUM_IDLE);
    if(status == USB_OK || status == USB_NOT_SUPPORT)
    {
      status = USB_OK;
    }
  }
  return status;
}

/**
  * @brief  usb host hid class set protocol
  * @param  uhost: to the structure of usbh_core_type
  * @param  protocol: portocol number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_hid_set_protocol(void *uhost, uint8_t protocol)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usb_sts_type status = USB_WAIT;
  if(puhost->ctrl.state == CONTROL_IDLE)
  {
    puhost->ctrl.setup.bmRequestType = USB_DIR_H2D | USB_REQ_RECIPIENT_INTERFACE | USB_REQ_TYPE_CLASS;
    puhost->ctrl.setup.bRequest = USB_HID_SET_PROTOCOL;
    puhost->ctrl.setup.wValue = protocol;
    puhost->ctrl.setup.wIndex = 0;
    puhost->ctrl.setup.wLength = 0;
    usbh_ctrl_request(puhost, 0, 0);
  }
  else
  {
    status = usbh_ctrl_result_check(puhost, CONTROL_IDLE, ENUM_IDLE);
    if(status == USB_OK || status == USB_NOT_SUPPORT)
    {
      status = USB_OK;
    }
  }
  return status;
}

/**
  * @brief  usb host clear feature
  * @param  uhost: to the structure of usbh_core_type
  * @param  ept_num: endpoint number
  * @param  hc_num: channel number
  * @retval status: usb_sts_type status
  */
usb_sts_type usbh_clear_endpoint_feature(usbh_core_type *uhost, uint8_t ept_num, uint8_t hc_num)
{
  usb_sts_type status = USB_WAIT;
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  uint8_t bm_req;
  bm_req = USB_REQ_RECIPIENT_ENDPOINT | USB_REQ_TYPE_STANDARD;
  if(puhost->ctrl.state == CONTROL_IDLE)
  {
    puhost->ctrl.setup.bmRequestType = USB_DIR_H2D | bm_req;
    puhost->ctrl.setup.bRequest = USB_STD_REQ_CLEAR_FEATURE;
    puhost->ctrl.setup.wValue = 0x00;
    puhost->ctrl.setup.wLength = 0;
    puhost->ctrl.setup.wIndex = ept_num;
    if((ept_num & 0x80) == USB_DIR_D2H)
    {
      puhost->hch[hc_num].toggle_in = 0;
    }
    else
    {
      puhost->hch[hc_num].toggle_out = 0;
    }
    status = usbh_ctrl_request(puhost, 0, 0);
  }
  if(usbh_ctrl_result_check(puhost, CONTROL_IDLE, ENUM_IDLE) == USB_OK)
  {
    status = USB_OK;
  }
  return status;
}

/**
  * @brief  usb host hid class request handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_request_handler(void *uhost)
{
  usb_sts_type status = USB_WAIT;
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_hid_type *phid =  (usbh_hid_type *)puhost->class_handler->pdata;

  switch(phid->ctrl_state)
  {
    case USB_HID_STATE_IDLE:
      phid->ctrl_state = USB_HID_STATE_GET_DESC;
      break;
    case USB_HID_STATE_GET_DESC:
      if(usbh_hid_get_desc(puhost, 9) == USB_OK)
      {
        phid->ctrl_state = USB_HID_STATE_GET_REPORT;
      }
      break;
    case USB_HID_STATE_GET_REPORT:
      if(usbh_hid_get_report(puhost, phid->hid_desc.wItemLength) == USB_OK)
      {
        phid->ctrl_state = USB_HID_STATE_SET_IDLE;
      }
      break;
    case USB_HID_STATE_SET_IDLE:
      if(usbh_hid_set_idle(puhost, 0, 0) == USB_OK)
      {
        phid->ctrl_state = USB_HID_STATE_SET_PROTOCOL;
      }
      break;
    case USB_HID_STATE_SET_PROTOCOL:
      if(usbh_hid_set_protocol(puhost, 0) == USB_OK)
      {
        phid->ctrl_state = USB_HID_STATE_COMPLETE;
      }
      break;
    case USB_HID_STATE_COMPLETE:
      phid->state = USB_HID_INIT;
      status = USB_OK;
      break;
    default:
      break;
  }

  return status;
}

/**
  * @brief  usb host hid class process handler
  * @param  uhost: to the structure of usbh_core_type
  * @retval status: usb_sts_type status
  */
static usb_sts_type uhost_process_handler(void *uhost)
{
  usbh_core_type *puhost = (usbh_core_type *)uhost;
  usbh_hid_type *phid =  (usbh_hid_type *)puhost->class_handler->pdata;
  urb_sts_type urb_status;
  switch(phid->state)
  {
    case USB_HID_INIT:
      phid->state = USB_HID_GET;
      break;

    case USB_HID_GET:
      usbh_interrupt_recv(puhost, phid->chin, (uint8_t *)phid->buffer, phid->in_maxpacket);
      phid->state = USB_HID_POLL;
      phid->poll_timer = usbh_get_frame(puhost->usb_reg);
      break;

    case USB_HID_POLL:
      if((usbh_get_frame(puhost->usb_reg) - phid->poll_timer) >= phid->in_poll )
      {
        phid->state = USB_HID_GET;
      }
      else
      {
        urb_status = usbh_get_urb_status(puhost, phid->chin);
        if(urb_status == URB_DONE)
        {
          puhost->urb_state[phid->chin] = URB_IDLE;
          if(phid->protocol == USB_HID_MOUSE_PROTOCOL_CODE)
          {
            usbh_hid_mouse_decode((uint8_t *)phid->buffer);
          }
          else if(phid->protocol == USB_HID_KEYBOARD_PROTOCOL_CODE)
          {
            usbh_hid_keyboard_decode((uint8_t *)phid->buffer);
          }

        }
        else if(urb_status == URB_STALL)
        {
          if(usbh_clear_endpoint_feature(puhost, phid->eptin, phid->chin) ==  USB_OK)
          {
            phid->state = USB_HID_GET;
          }
        }
      }
      break;

    default:
      break;
  }
  return USB_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
