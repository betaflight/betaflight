/**
  **************************************************************************
  * @file     usbh_hid_mouse.c
  * @brief    usb host hid mouse type
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
#include "usbh_hid_mouse.h"

/** @addtogroup AT32F435_437_middlewares_usbh_class
  * @{
  */

/** @defgroup USBH_hid_class_mouse
  * @brief usb host class hid mouse
  * @{
  */

/** @defgroup USBH_hid_class_mouse_private_functions
  * @{
  */

usb_hid_mouse_type hid_mouse;
uint16_t x_pos = 0, y_pos = 0;

/**
  * @brief  usb host hid position
  * @param  x: x position
  * @param  y: y position
  * @retval none
  */
void usbh_hid_mouse_position(uint8_t x, uint8_t y)
{
  if((x != 0) || (y != 0))
  {
    x_pos += x / 2;
    y_pos += y / 2;

    if(x_pos > MOUSE_WINDOW_WIDTH - 12)
    {
      x_pos = MOUSE_WINDOW_WIDTH - 12;
    }
    if(y_pos > MOUSE_WINDOW_HEIGHT - 12)
    {
      y_pos = MOUSE_WINDOW_HEIGHT - 12;
    }

    if(x_pos < 2)
    {
      x_pos = 2;
    }
    if(y_pos < 2)
    {
      y_pos = 2;
    }
    USBH_DEBUG("Moving Mouse");
  }

}

/**
  * @brief  usb host hid button press
  * @param  button: button id
  * @retval none
  */
void usbh_hid_mouse_button_press(uint8_t button)
{
  switch(button)
  {
    case MOUSE_BUTTON_LEFT:
      /* left button */
      USBH_DEBUG("Left Button Pressed ");
      break;
    case MOUSE_BUTTON_RIGHT:
      USBH_DEBUG("Right Button Pressed ");
      /* left button */
      break;
    case MOUSE_BUTTON_MIDDLE:
      USBH_DEBUG("Middle Button Pressed ");
      /* middle button */
      break;
  }
}

/**
  * @brief  usb host hid button release
  * @param  button: button id
  * @retval none
  */
void usbh_hid_mouse_button_release(uint8_t button)
{
  switch(button)
  {
    case MOUSE_BUTTON_LEFT:
      /* left button */
      USBH_DEBUG("Left Button Released ");
      break;
    case MOUSE_BUTTON_RIGHT:
      /* left button */
      USBH_DEBUG("Right Button Released ");
      break;
    case MOUSE_BUTTON_MIDDLE:
      /* middle button */
      USBH_DEBUG("Middle Button Released ");
      break;
  }
}

/**
  * @brief  usb host hid mouse process
  * @param  mouse: mouse data type
  * @retval none
  */
void usbh_hid_mouse_process(usb_hid_mouse_type *mouse)
{
  uint8_t idx = 1;
  static uint8_t b_state[3] = {0};
  if((mouse->x != 0) && (mouse->y != 0))
  {
    usbh_hid_mouse_position(mouse->x, mouse->y);
  }

  for(idx = 0; idx < 3; idx ++)
  {
    if(mouse->button & 1 << idx)
    {
      if(b_state[idx] == 0)
      {
        usbh_hid_mouse_button_press(idx);
        b_state[idx] = 1;
      }
    }
    else
    {
      if(b_state[idx] == 1)
      {
        usbh_hid_mouse_button_release(idx);
        b_state[idx] = 0;
      }
    }
  }
}

/**
  * @brief  usb host hid mouse decode
  * @param  mouse_data: mouse data
  * @retval none
  */
void usbh_hid_mouse_decode(uint8_t *mouse_data)
{
  hid_mouse.button = mouse_data[0];
  hid_mouse.x = mouse_data[1];
  hid_mouse.y = mouse_data[2];
  hid_mouse.z = mouse_data[3];

  usbh_hid_mouse_process(&hid_mouse);
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

