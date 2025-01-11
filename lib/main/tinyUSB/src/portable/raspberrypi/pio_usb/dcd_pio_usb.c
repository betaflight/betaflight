/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && (CFG_TUSB_MCU == OPT_MCU_RP2040) && CFG_TUD_RPI_PIO_USB

#include "pico.h"
#include "pio_usb.h"
#include "pio_usb_ll.h"

#include "device/dcd.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define RHPORT_OFFSET     1
#define RHPORT_PIO(_x)    ((_x)-RHPORT_OFFSET)

//-------------  -------------//
static usb_device_t *usb_device = NULL;
static usb_descriptor_buffers_t desc;

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Initialize controller to device mode
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;
  static pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
  usb_device = pio_usb_device_init(&config, &desc);

  return true;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{
  (void) rhport;
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport)
{
  (void) rhport;
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  // must be called before queuing status
  pio_usb_device_set_address(dev_addr);
  dcd_edpt_xfer(rhport, 0x80, NULL, 0);
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
  (void) rhport;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void) rhport;
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_ep)
{
  (void) rhport;
  return pio_usb_device_endpoint_open((uint8_t const*) desc_ep);
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;
  endpoint_t *ep = pio_usb_device_get_endpoint_by_address(ep_addr);
  return pio_usb_ll_transfer_start(ep, buffer, total_bytes);
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
//bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
//{
//  (void) rhport;
//  (void) ep_addr;
//  (void) ff;
//  (void) total_bytes;
//  return false;
//}

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  endpoint_t *ep = pio_usb_device_get_endpoint_by_address(ep_addr);
  ep->has_transfer = false;
  ep->stalled = true;
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  endpoint_t *ep = pio_usb_device_get_endpoint_by_address(ep_addr);
  ep->data_id = 0;
  ep->stalled = false;
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

static void __no_inline_not_in_flash_func(handle_endpoint_irq)(uint8_t tu_rhport, xfer_result_t result, volatile uint32_t* ep_reg)
{
  const uint32_t ep_all = *ep_reg;

  for(uint8_t ep_idx = 0; ep_idx < PIO_USB_EP_POOL_CNT; ep_idx++)
  {
    uint32_t const mask = (1u << ep_idx);

    if (ep_all & mask)
    {
      endpoint_t* ep = PIO_USB_ENDPOINT(ep_idx);
      dcd_event_xfer_complete(tu_rhport, ep->ep_num, ep->actual_len, result, true);
    }
  }

  // clear all
  (*ep_reg) &= ~ep_all;
}

// IRQ Handler
void __no_inline_not_in_flash_func(pio_usb_device_irq_handler)(uint8_t root_id)
{
  uint8_t const tu_rhport = root_id + 1;
  root_port_t* rport = PIO_USB_ROOT_PORT(root_id);
  uint32_t const ints = rport->ints;

  if (ints & PIO_USB_INTS_RESET_END_BITS)
  {
    dcd_event_bus_reset(tu_rhport, TUSB_SPEED_FULL, true);
  }

  if (ints & PIO_USB_INTS_SETUP_REQ_BITS)
  {
    dcd_event_setup_received(tu_rhport, rport->setup_packet, true);
  }

  if ( ints & PIO_USB_INTS_ENDPOINT_COMPLETE_BITS )
  {
    handle_endpoint_irq(tu_rhport, XFER_RESULT_SUCCESS, &rport->ep_complete);
  }

  if ( ints & PIO_USB_INTS_ENDPOINT_STALLED_BITS )
  {
    handle_endpoint_irq(tu_rhport, XFER_RESULT_STALLED, &rport->ep_stalled);
  }

  if ( ints & PIO_USB_INTS_ENDPOINT_ERROR_BITS )
  {
    handle_endpoint_irq(tu_rhport, XFER_RESULT_FAILED, &rport->ep_error);
  }

  // clear all
  rport->ints &= ~ints;
}

#endif
