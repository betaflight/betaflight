/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Copyright (c) 2021 Ha Thach (tinyusb.org) for Double Buffered
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

#if CFG_TUH_ENABLED && (CFG_TUSB_MCU == OPT_MCU_RP2040) && !CFG_TUH_RPI_PIO_USB && !CFG_TUH_MAX3421

#include "pico.h"
#include "rp2040_usb.h"

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "osal/osal.h"

#include "host/hcd.h"
#include "host/usbh.h"

// port 0 is native USB port, other is counted as software PIO
#define RHPORT_NATIVE 0

//--------------------------------------------------------------------+
// Low level rp2040 controller functions
//--------------------------------------------------------------------+

#ifndef PICO_USB_HOST_INTERRUPT_ENDPOINTS
#define PICO_USB_HOST_INTERRUPT_ENDPOINTS (USB_MAX_ENDPOINTS - 1)
#endif
static_assert(PICO_USB_HOST_INTERRUPT_ENDPOINTS <= USB_MAX_ENDPOINTS, "");

// Host mode uses one shared endpoint register for non-interrupt endpoint
static struct hw_endpoint ep_pool[1 + PICO_USB_HOST_INTERRUPT_ENDPOINTS];
#define epx (ep_pool[0])

// Flags we set by default in sie_ctrl (we add other bits on top)
enum {
  SIE_CTRL_BASE = USB_SIE_CTRL_SOF_EN_BITS      | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS |
                  USB_SIE_CTRL_PULLDOWN_EN_BITS | USB_SIE_CTRL_EP0_INT_1BUF_BITS
};

static struct hw_endpoint *get_dev_ep(uint8_t dev_addr, uint8_t ep_addr)
{
  uint8_t num = tu_edpt_number(ep_addr);
  if ( num == 0 ) return &epx;

  for ( uint32_t i = 1; i < TU_ARRAY_SIZE(ep_pool); i++ )
  {
    struct hw_endpoint *ep = &ep_pool[i];
    if ( ep->configured && (ep->dev_addr == dev_addr) && (ep->ep_addr == ep_addr) ) return ep;
  }

  return NULL;
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t dev_speed(void)
{
  return (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) >> USB_SIE_STATUS_SPEED_LSB;
}

TU_ATTR_ALWAYS_INLINE static inline bool need_pre(uint8_t dev_addr)
{
  // If this device is different to the speed of the root device
  // (i.e. is a low speed device on a full speed hub) then need pre
  return hcd_port_speed_get(0) != tuh_speed_get(dev_addr);
}

static void __tusb_irq_path_func(hw_xfer_complete)(struct hw_endpoint *ep, xfer_result_t xfer_result)
{
  // Mark transfer as done before we tell the tinyusb stack
  uint8_t dev_addr = ep->dev_addr;
  uint8_t ep_addr = ep->ep_addr;
  uint xferred_len = ep->xferred_len;
  hw_endpoint_reset_transfer(ep);
  hcd_event_xfer_complete(dev_addr, ep_addr, xferred_len, xfer_result, true);
}

static void __tusb_irq_path_func(_handle_buff_status_bit)(uint bit, struct hw_endpoint *ep)
{
  usb_hw_clear->buf_status = bit;
  // EP may have been stalled?
  assert(ep->active);
  bool done = hw_endpoint_xfer_continue(ep);
  if ( done )
  {
    hw_xfer_complete(ep, XFER_RESULT_SUCCESS);
  }
}

static void __tusb_irq_path_func(hw_handle_buff_status)(void)
{
  uint32_t remaining_buffers = usb_hw->buf_status;
  pico_trace("buf_status 0x%08lx\n", remaining_buffers);

  // Check EPX first
  uint bit = 0b1;
  if ( remaining_buffers & bit )
  {
    remaining_buffers &= ~bit;
    struct hw_endpoint * ep = &epx;

    uint32_t ep_ctrl = *ep->endpoint_control;
    if ( ep_ctrl & EP_CTRL_DOUBLE_BUFFERED_BITS )
    {
      TU_LOG(3, "Double Buffered: ");
    }
    else
    {
      TU_LOG(3, "Single Buffered: ");
    }
    TU_LOG_HEX(3, ep_ctrl);

    _handle_buff_status_bit(bit, ep);
  }

  // Check "interrupt" (asynchronous) endpoints for both IN and OUT
  for ( uint i = 1; i <= USB_HOST_INTERRUPT_ENDPOINTS && remaining_buffers; i++ )
  {
    // EPX is bit 0 & 1
    // IEP1 IN  is bit 2
    // IEP1 OUT is bit 3
    // IEP2 IN  is bit 4
    // IEP2 OUT is bit 5
    // IEP3 IN  is bit 6
    // IEP3 OUT is bit 7
    // etc
    for ( uint j = 0; j < 2; j++ )
    {
      bit = 1 << (i * 2 + j);
      if ( remaining_buffers & bit )
      {
        remaining_buffers &= ~bit;
        _handle_buff_status_bit(bit, &ep_pool[i]);
      }
    }
  }

  if ( remaining_buffers )
  {
    panic("Unhandled buffer %d\n", remaining_buffers);
  }
}

static void __tusb_irq_path_func(hw_trans_complete)(void)
{
  if (usb_hw->sie_ctrl & USB_SIE_CTRL_SEND_SETUP_BITS)
  {
    pico_trace("Sent setup packet\n");
    struct hw_endpoint *ep = &epx;
    assert(ep->active);
    // Set transferred length to 8 for a setup packet
    ep->xferred_len = 8;
    hw_xfer_complete(ep, XFER_RESULT_SUCCESS);
  }
  else
  {
    // Don't care. Will handle this in buff status
    return;
  }
}

static void __tusb_irq_path_func(hcd_rp2040_irq)(void)
{
  uint32_t status = usb_hw->ints;
  uint32_t handled = 0;

  if ( status & USB_INTS_HOST_CONN_DIS_BITS )
  {
    handled |= USB_INTS_HOST_CONN_DIS_BITS;

    if ( dev_speed() )
    {
      hcd_event_device_attach(RHPORT_NATIVE, true);
    }
    else
    {
      hcd_event_device_remove(RHPORT_NATIVE, true);
    }

    // Clear speed change interrupt
    usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;
  }

  if ( status & USB_INTS_STALL_BITS )
  {
    // We have rx'd a stall from the device
    // NOTE THIS SHOULD HAVE PRIORITY OVER BUFF_STATUS
    // AND TRANS_COMPLETE as the stall is an alternative response
    // to one of those events
    pico_trace("Stall REC\n");
    handled |= USB_INTS_STALL_BITS;
    usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;
    hw_xfer_complete(&epx, XFER_RESULT_STALLED);
  }

  if ( status & USB_INTS_BUFF_STATUS_BITS )
  {
    handled |= USB_INTS_BUFF_STATUS_BITS;
    TU_LOG(2, "Buffer complete\r\n");
    hw_handle_buff_status();
  }

  if ( status & USB_INTS_TRANS_COMPLETE_BITS )
  {
    handled |= USB_INTS_TRANS_COMPLETE_BITS;
    usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;
    TU_LOG(2, "Transfer complete\r\n");
    hw_trans_complete();
  }

  if ( status & USB_INTS_ERROR_RX_TIMEOUT_BITS )
  {
    handled |= USB_INTS_ERROR_RX_TIMEOUT_BITS;
    usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;
  }

  if ( status & USB_INTS_ERROR_DATA_SEQ_BITS )
  {
    usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;
    TU_LOG(3, "  Seq Error: [0] = 0x%04u  [1] = 0x%04x\r\n",
           tu_u32_low16(*epx.buffer_control),
           tu_u32_high16(*epx.buffer_control));
    panic("Data Seq Error \n");
  }

  if ( status ^ handled )
  {
    panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
  }
}

void __tusb_irq_path_func(hcd_int_handler)(uint8_t rhport, bool in_isr) {
  (void) rhport;
  (void) in_isr;
  hcd_rp2040_irq();
}

static struct hw_endpoint *_next_free_interrupt_ep(void)
{
  struct hw_endpoint * ep = NULL;
  for ( uint i = 1; i < TU_ARRAY_SIZE(ep_pool); i++ )
  {
    ep = &ep_pool[i];
    if ( !ep->configured )
    {
      // Will be configured by _hw_endpoint_init / _hw_endpoint_allocate
      ep->interrupt_num = (uint8_t) (i - 1);
      return ep;
    }
  }
  return ep;
}

static struct hw_endpoint *_hw_endpoint_allocate(uint8_t transfer_type)
{
  struct hw_endpoint * ep = NULL;

  if ( transfer_type != TUSB_XFER_CONTROL )
  {
    // Note: even though datasheet name these "Interrupt" endpoints. These are actually
    // "Asynchronous" endpoints and can be used for other type such as: Bulk  (ISO need confirmation)
    ep = _next_free_interrupt_ep();
    pico_info("Allocate %s ep %d\n", tu_edpt_type_str(transfer_type), ep->interrupt_num);
    assert(ep);
    ep->buffer_control = &usbh_dpram->int_ep_buffer_ctrl[ep->interrupt_num].ctrl;
    ep->endpoint_control = &usbh_dpram->int_ep_ctrl[ep->interrupt_num].ctrl;
    // 0 for epx (double buffered): TODO increase to 1024 for ISO
    // 2x64 for intep0
    // 3x64 for intep1
    // etc
    ep->hw_data_buf = &usbh_dpram->epx_data[64 * (ep->interrupt_num + 2)];
  }
  else
  {
    ep = &epx;
    ep->buffer_control = &usbh_dpram->epx_buf_ctrl;
    ep->endpoint_control = &usbh_dpram->epx_ctrl;
    ep->hw_data_buf = &usbh_dpram->epx_data[0];
  }

  return ep;
}

static void _hw_endpoint_init(struct hw_endpoint *ep, uint8_t dev_addr, uint8_t ep_addr, uint16_t wMaxPacketSize, uint8_t transfer_type, uint8_t bmInterval)
{
  // Already has data buffer, endpoint control, and buffer control allocated at this point
  assert(ep->endpoint_control);
  assert(ep->buffer_control);
  assert(ep->hw_data_buf);

  uint8_t const num = tu_edpt_number(ep_addr);
  tusb_dir_t const dir = tu_edpt_dir(ep_addr);

  ep->ep_addr = ep_addr;
  ep->dev_addr = dev_addr;

  // For host, IN to host == RX, anything else rx == false
  ep->rx = (dir == TUSB_DIR_IN);

  // Response to a setup packet on EP0 starts with pid of 1
  ep->next_pid = (num == 0 ? 1u : 0u);
  ep->wMaxPacketSize = wMaxPacketSize;
  ep->transfer_type = transfer_type;

  pico_trace("hw_endpoint_init dev %d ep %02X xfer %d\n", ep->dev_addr, ep->ep_addr, ep->transfer_type);
  pico_trace("dev %d ep %02X setup buffer @ 0x%p\n", ep->dev_addr, ep->ep_addr, ep->hw_data_buf);
  uint dpram_offset = hw_data_offset(ep->hw_data_buf);
  // Bits 0-5 should be 0
  assert(!(dpram_offset & 0b111111));

  // Fill in endpoint control register with buffer offset
  uint32_t ep_reg = EP_CTRL_ENABLE_BITS
                    | EP_CTRL_INTERRUPT_PER_BUFFER
                    | (ep->transfer_type << EP_CTRL_BUFFER_TYPE_LSB)
                    | dpram_offset;
  if ( bmInterval )
  {
    ep_reg |= (uint32_t) ((bmInterval - 1) << EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB);
  }
  *ep->endpoint_control = ep_reg;
  pico_trace("endpoint control (0x%p) <- 0x%lx\n", ep->endpoint_control, ep_reg);
  ep->configured = true;

  if ( ep != &epx )
  {
    // Endpoint has its own addr_endp and interrupt bits to be setup!
    // This is an interrupt/async endpoint. so need to set up ADDR_ENDP register with:
    // - device address
    // - endpoint number / direction
    // - preamble
    uint32_t reg = (uint32_t) (dev_addr | (num << USB_ADDR_ENDP1_ENDPOINT_LSB));

    if ( dir == TUSB_DIR_OUT )
    {
      reg |= USB_ADDR_ENDP1_INTEP_DIR_BITS;
    }

    if ( need_pre(dev_addr) )
    {
      reg |= USB_ADDR_ENDP1_INTEP_PREAMBLE_BITS;
    }
    usb_hw->int_ep_addr_ctrl[ep->interrupt_num] = reg;

    // Finally, enable interrupt that endpoint
    usb_hw_set->int_ep_ctrl = 1 << (ep->interrupt_num + 1);

    // If it's an interrupt endpoint we need to set up the buffer control
    // register
  }
}

//--------------------------------------------------------------------+
// HCD API
//--------------------------------------------------------------------+
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;
  pico_trace("hcd_init %d\n", rhport);
  assert(rhport == 0);

  // Reset any previous state
  rp2040_usb_init();

  // Force VBUS detect to always present, for now we assume vbus is always provided (without using VBUS En)
  usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

  // Remove shared irq if it was previously added so as not to fill up shared irq slots
  irq_remove_handler(USBCTRL_IRQ, hcd_rp2040_irq);

  irq_add_shared_handler(USBCTRL_IRQ, hcd_rp2040_irq, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);

  // clear epx and interrupt eps
  memset(&ep_pool, 0, sizeof(ep_pool));

  // Enable in host mode with SOF / Keep alive on
  usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS | USB_MAIN_CTRL_HOST_NDEVICE_BITS;
  usb_hw->sie_ctrl = SIE_CTRL_BASE;
  usb_hw->inte = USB_INTE_BUFF_STATUS_BITS      |
                 USB_INTE_HOST_CONN_DIS_BITS    |
                 USB_INTE_HOST_RESUME_BITS      |
                 USB_INTE_STALL_BITS            |
                 USB_INTE_TRANS_COMPLETE_BITS   |
                 USB_INTE_ERROR_RX_TIMEOUT_BITS |
                 USB_INTE_ERROR_DATA_SEQ_BITS   ;

  return true;
}

bool hcd_deinit(uint8_t rhport) {
  (void) rhport;

  irq_remove_handler(USBCTRL_IRQ, hcd_rp2040_irq);
  reset_block(RESETS_RESET_USBCTRL_BITS);
  unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

  return true;
}

void hcd_port_reset(uint8_t rhport)
{
  (void) rhport;
  pico_trace("hcd_port_reset\n");
  assert(rhport == 0);
  // TODO: Nothing to do here yet. Perhaps need to reset some state?
}

void hcd_port_reset_end(uint8_t rhport)
{
  (void) rhport;
}

bool hcd_port_connect_status(uint8_t rhport)
{
  (void) rhport;
  pico_trace("hcd_port_connect_status\n");
  assert(rhport == 0);
  return usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
  (void) rhport;
  assert(rhport == 0);

  // TODO: Should enumval this register
  switch ( dev_speed() )
  {
    case 1:
      return TUSB_SPEED_LOW;
    case 2:
      return TUSB_SPEED_FULL;
    default:
      panic("Invalid speed\n");
      // return TUSB_SPEED_INVALID;
  }
}

// Close all opened endpoint belong to this device
void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
  pico_trace("hcd_device_close %d\n", dev_addr);
  (void) rhport;

  if (dev_addr == 0) return;

  for (size_t i = 1; i < TU_ARRAY_SIZE(ep_pool); i++)
  {
    hw_endpoint_t* ep = &ep_pool[i];

    if (ep->dev_addr == dev_addr && ep->configured)
    {
      // in case it is an interrupt endpoint, disable it
      usb_hw_clear->int_ep_ctrl = (1 << (ep->interrupt_num + 1));
      usb_hw->int_ep_addr_ctrl[ep->interrupt_num] = 0;

      // unconfigure the endpoint
      ep->configured = false;
      *ep->endpoint_control = 0;
      *ep->buffer_control = 0;
      hw_endpoint_reset_transfer(ep);
    }
  }
}

uint32_t hcd_frame_number(uint8_t rhport)
{
  (void) rhport;
  return usb_hw->sof_rd;
}

void hcd_int_enable(uint8_t rhport)
{
  (void) rhport;
  assert(rhport == 0);
  irq_set_enabled(USBCTRL_IRQ, true);
}

void hcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  // todo we should check this is disabling from the correct core; note currently this is never called
  assert(rhport == 0);
  irq_set_enabled(USBCTRL_IRQ, false);
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;

  pico_trace("hcd_edpt_open dev_addr %d, ep_addr %d\n", dev_addr, ep_desc->bEndpointAddress);

  // Allocated differently based on if it's an interrupt endpoint or not
  struct hw_endpoint *ep = _hw_endpoint_allocate(ep_desc->bmAttributes.xfer);
  TU_ASSERT(ep);

  _hw_endpoint_init(ep,
                    dev_addr,
                    ep_desc->bEndpointAddress,
                    tu_edpt_packet_size(ep_desc),
                    ep_desc->bmAttributes.xfer,
                    ep_desc->bInterval);

  return true;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen)
{
  (void) rhport;

  pico_trace("hcd_edpt_xfer dev_addr %d, ep_addr 0x%x, len %d\n", dev_addr, ep_addr, buflen);

  uint8_t const ep_num = tu_edpt_number(ep_addr);
  tusb_dir_t const ep_dir = tu_edpt_dir(ep_addr);

  // Get appropriate ep. Either EPX or interrupt endpoint
  struct hw_endpoint *ep = get_dev_ep(dev_addr, ep_addr);

  TU_ASSERT(ep);

  // EP should be inactive
  assert(!ep->active);

  // Control endpoint can change direction 0x00 <-> 0x80
  if ( ep_addr != ep->ep_addr )
  {
    assert(ep_num == 0);

    // Direction has flipped on endpoint control so re init it but with same properties
    _hw_endpoint_init(ep, dev_addr, ep_addr, ep->wMaxPacketSize, ep->transfer_type, 0);
  }

  // If a normal transfer (non-interrupt) then initiate using
  // sie ctrl registers. Otherwise interrupt ep registers should
  // already be configured
  if ( ep == &epx )
  {
    hw_endpoint_xfer_start(ep, buffer, buflen);

    // That has set up buffer control, endpoint control etc
    // for host we have to initiate the transfer
    usb_hw->dev_addr_ctrl = (uint32_t) (dev_addr | (ep_num << USB_ADDR_ENDP_ENDPOINT_LSB));

    uint32_t flags = USB_SIE_CTRL_START_TRANS_BITS | SIE_CTRL_BASE |
                     (ep_dir ? USB_SIE_CTRL_RECEIVE_DATA_BITS : USB_SIE_CTRL_SEND_DATA_BITS) |
                     (need_pre(dev_addr) ? USB_SIE_CTRL_PREAMBLE_EN_BITS : 0);
    // START_TRANS bit on SIE_CTRL seems to exhibit the same behavior as the AVAILABLE bit
    // described in RP2040 Datasheet, release 2.1, section "4.1.2.5.1. Concurrent access".
    // We write everything except the START_TRANS bit first, then wait some cycles.
    usb_hw->sie_ctrl = flags & ~USB_SIE_CTRL_START_TRANS_BITS;
    busy_wait_at_least_cycles(12);
    usb_hw->sie_ctrl = flags;
  }else
  {
    hw_endpoint_xfer_start(ep, buffer, buflen);
  }

  return true;
}

bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  // TODO not implemented yet
  return false;
}

bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
  (void) rhport;

  // Copy data into setup packet buffer
  for ( uint8_t i = 0; i < 8; i++ )
  {
    usbh_dpram->setup_packet[i] = setup_packet[i];
  }

  // Configure EP0 struct with setup info for the trans complete
  struct hw_endpoint * ep = _hw_endpoint_allocate(0);
  TU_ASSERT(ep);

  // EPX should be inactive
  assert(!ep->active);

  // EP0 out
  _hw_endpoint_init(ep, dev_addr, 0x00, ep->wMaxPacketSize, 0, 0);
  assert(ep->configured);

  ep->remaining_len = 8;
  ep->active = true;

  // Set device address
  usb_hw->dev_addr_ctrl = dev_addr;

  // Set pre if we are a low speed device on full speed hub
  uint32_t const flags = SIE_CTRL_BASE | USB_SIE_CTRL_SEND_SETUP_BITS | USB_SIE_CTRL_START_TRANS_BITS |
                         (need_pre(dev_addr) ? USB_SIE_CTRL_PREAMBLE_EN_BITS : 0);

  // START_TRANS bit on SIE_CTRL seems to exhibit the same behavior as the AVAILABLE bit
  // described in RP2040 Datasheet, release 2.1, section "4.1.2.5.1. Concurrent access".
  // We write everything except the START_TRANS bit first, then wait some cycles.
  usb_hw->sie_ctrl = flags & ~USB_SIE_CTRL_START_TRANS_BITS;
  busy_wait_at_least_cycles(12);
  usb_hw->sie_ctrl = flags;

  return true;
}

bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;

  panic("hcd_clear_stall");
  // return true;
}

#endif
