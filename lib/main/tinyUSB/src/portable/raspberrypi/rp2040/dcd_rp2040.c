/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
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

#if CFG_TUD_ENABLED && (CFG_TUSB_MCU == OPT_MCU_RP2040) && !CFG_TUD_RPI_PIO_USB

#include "pico.h"
#include "hardware/sync.h"
#include "rp2040_usb.h"

#if TUD_OPT_RP2040_USB_DEVICE_ENUMERATION_FIX
#include "pico/fix/rp2040_usb_device_enumeration.h"
#endif

#include "device/dcd.h"

// Current implementation force vbus detection as always present, causing device think it is always plugged into host.
// Therefore it cannot detect disconnect event, mistaken it as suspend.
// Note: won't work if change to 0 (for now)
#define FORCE_VBUS_DETECT   1

/*------------------------------------------------------------------*/
/* Low level controller
 *------------------------------------------------------------------*/
// Init these in dcd_init
static uint8_t* next_buffer_ptr;

// USB_MAX_ENDPOINTS Endpoints, direction TUSB_DIR_OUT for out and TUSB_DIR_IN for in.
static struct hw_endpoint hw_endpoints[USB_MAX_ENDPOINTS][2];

// SOF may be used by remote wakeup as RESUME, this indicate whether SOF is actually used by usbd
static bool _sof_enable = false;

TU_ATTR_ALWAYS_INLINE static inline struct hw_endpoint* hw_endpoint_get_by_num(uint8_t num, tusb_dir_t dir) {
  return &hw_endpoints[num][dir];
}

TU_ATTR_ALWAYS_INLINE static inline struct hw_endpoint* hw_endpoint_get_by_addr(uint8_t ep_addr) {
  uint8_t num = tu_edpt_number(ep_addr);
  tusb_dir_t dir = tu_edpt_dir(ep_addr);
  return hw_endpoint_get_by_num(num, dir);
}

// Allocate from the USB buffer space (max 3840 bytes)
static void hw_endpoint_alloc(struct hw_endpoint* ep, size_t size) {
  // round up size to multiple of 64
  size = tu_round_up(ep->wMaxPacketSize, 64);

  // double buffered Bulk endpoint
  if (ep->transfer_type == TUSB_XFER_BULK) {
    size *= 2u;
  }

  // assign buffer
  ep->hw_data_buf = next_buffer_ptr;
  next_buffer_ptr += size;

  hard_assert(next_buffer_ptr < usb_dpram->epx_data + sizeof(usb_dpram->epx_data));
  pico_info("  Allocated %d bytes (0x%p)\r\n", size,  ep->hw_data_buf);
}

// Enable endpoint
TU_ATTR_ALWAYS_INLINE static inline void hw_endpoint_enable(struct hw_endpoint* ep) {
  uint32_t const reg = EP_CTRL_ENABLE_BITS | ((uint) ep->transfer_type << EP_CTRL_BUFFER_TYPE_LSB) | hw_data_offset(ep->hw_data_buf);
  *ep->endpoint_control = reg;
}

// main processing for dcd_edpt_iso_activate
static void hw_endpoint_init(uint8_t ep_addr, uint16_t wMaxPacketSize, uint8_t transfer_type) {
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);

  const uint8_t num = tu_edpt_number(ep_addr);
  const tusb_dir_t dir = tu_edpt_dir(ep_addr);

  ep->ep_addr = ep_addr;

  // For device, IN is a tx transfer and OUT is an rx transfer
  ep->rx = (dir == TUSB_DIR_OUT);

  ep->next_pid = 0u;
  ep->wMaxPacketSize = wMaxPacketSize;
  ep->transfer_type = transfer_type;

  // Every endpoint has a buffer control register in dpram
  if (dir == TUSB_DIR_IN) {
    ep->buffer_control = &usb_dpram->ep_buf_ctrl[num].in;
  } else {
    ep->buffer_control = &usb_dpram->ep_buf_ctrl[num].out;
  }

  // Clear existing buffer control state
  *ep->buffer_control = 0;

  if (num == 0) {
    // EP0 has no endpoint control register because the buffer offsets are fixed
    ep->endpoint_control = NULL;

    // Buffer offset is fixed (also double buffered)
    ep->hw_data_buf = (uint8_t*) &usb_dpram->ep0_buf_a[0];
  } else {
    // Set the endpoint control register (starts at EP1, hence num-1)
    if (dir == TUSB_DIR_IN) {
      ep->endpoint_control = &usb_dpram->ep_ctrl[num - 1].in;
    } else {
      ep->endpoint_control = &usb_dpram->ep_ctrl[num - 1].out;
    }
  }
}

// Init, allocate buffer and enable endpoint
static void hw_endpoint_open(uint8_t ep_addr, uint16_t wMaxPacketSize, uint8_t transfer_type) {
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);
  hw_endpoint_init(ep_addr, wMaxPacketSize, transfer_type);
  const uint8_t num = tu_edpt_number(ep_addr);
  if (num != 0) {
    // EP0 is already enabled
    hw_endpoint_alloc(ep, ep->wMaxPacketSize);
    hw_endpoint_enable(ep);
  }
}

static void hw_endpoint_xfer(uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);
  hw_endpoint_xfer_start(ep, buffer, total_bytes);
}

static void __tusb_irq_path_func(hw_handle_buff_status)(void) {
  uint32_t remaining_buffers = usb_hw->buf_status;
  pico_trace("buf_status = 0x%08lx\r\n", remaining_buffers);
  uint bit = 1u;
  for (uint8_t i = 0; remaining_buffers && i < USB_MAX_ENDPOINTS * 2; i++) {
    if (remaining_buffers & bit) {
      // clear this in advance
      usb_hw_clear->buf_status = bit;

      // IN transfer for even i, OUT transfer for odd i
      struct hw_endpoint* ep = hw_endpoint_get_by_num(i >> 1u, (i & 1u) ? TUSB_DIR_OUT : TUSB_DIR_IN);

      // Continue xfer
      bool done = hw_endpoint_xfer_continue(ep);
      if (done) {
        // Notify
        dcd_event_xfer_complete(0, ep->ep_addr, ep->xferred_len, XFER_RESULT_SUCCESS, true);
        hw_endpoint_reset_transfer(ep);
      }
      remaining_buffers &= ~bit;
    }
    bit <<= 1u;
  }
}

TU_ATTR_ALWAYS_INLINE static inline void reset_ep0(void) {
  // If we have finished this transfer on EP0 set pid back to 1 for next
  // setup transfer. Also clear a stall in case
  for (uint8_t dir = 0; dir < 2; dir++) {
    struct hw_endpoint* ep = hw_endpoint_get_by_num(0, dir);
    if (ep->active) {
      // Abort any pending transfer from a prior control transfer per USB specs
      // Due to Errata RP2040-E2: ABORT flag is only applicable for B2 and later (unusable for B0, B1).
      // Which means we are not guaranteed to safely abort pending transfer on B0 and B1.
      uint32_t const abort_mask = (dir ? USB_EP_ABORT_EP0_IN_BITS : USB_EP_ABORT_EP0_OUT_BITS);
      if (rp2040_chip_version() >= 2) {
        usb_hw_set->abort = abort_mask;
        while ((usb_hw->abort_done & abort_mask) != abort_mask) {}
      }

      _hw_endpoint_buffer_control_set_value32(ep, USB_BUF_CTRL_DATA1_PID | USB_BUF_CTRL_SEL);
      hw_endpoint_reset_transfer(ep);

      if (rp2040_chip_version() >= 2) {
        usb_hw_clear->abort_done = abort_mask;
        usb_hw_clear->abort = abort_mask;
      }
    }
    ep->next_pid = 1u;
  }
}

static void __tusb_irq_path_func(reset_non_control_endpoints)(void) {
  // Disable all non-control
  for (uint8_t i = 0; i < USB_MAX_ENDPOINTS - 1; i++) {
    usb_dpram->ep_ctrl[i].in = 0;
    usb_dpram->ep_ctrl[i].out = 0;
  }

  // clear non-control hw endpoints
  tu_memclr(hw_endpoints[1], sizeof(hw_endpoints) - 2 * sizeof(hw_endpoint_t));

  // reclaim buffer space
  next_buffer_ptr = &usb_dpram->epx_data[0];
}

static void __tusb_irq_path_func(dcd_rp2040_irq)(void) {
  uint32_t const status = usb_hw->ints;
  uint32_t handled = 0;

  if (status & USB_INTF_DEV_SOF_BITS) {
    bool keep_sof_alive = false;

    handled |= USB_INTF_DEV_SOF_BITS;

#if TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX
    // Errata 15 workaround for Device Bulk-In endpoint
    e15_last_sof = time_us_32();

    for (uint8_t i = 0; i < USB_MAX_ENDPOINTS; i++) {
      struct hw_endpoint* ep = hw_endpoint_get_by_num(i, TUSB_DIR_IN);

      // Active Bulk IN endpoint requires SOF
      if ((ep->transfer_type == TUSB_XFER_BULK) && ep->active) {
        keep_sof_alive = true;

        hw_endpoint_lock_update(ep, 1);

        // Deferred enable?
        if (ep->pending) {
          ep->pending = 0;
          hw_endpoint_start_next_buffer(ep);
        }

        hw_endpoint_lock_update(ep, -1);
      }
    }
#endif

    // disable SOF interrupt if it is used for RESUME in remote wakeup
    if (!keep_sof_alive && !_sof_enable) usb_hw_clear->inte = USB_INTS_DEV_SOF_BITS;

    dcd_event_sof(0, usb_hw->sof_rd & USB_SOF_RD_BITS, true);
  }

  // xfer events are handled before setup req. So if a transfer completes immediately
  // before closing the EP, the events will be delivered in same order.
  if (status & USB_INTS_BUFF_STATUS_BITS) {
    handled |= USB_INTS_BUFF_STATUS_BITS;
    hw_handle_buff_status();
  }

  if (status & USB_INTS_SETUP_REQ_BITS) {
    handled |= USB_INTS_SETUP_REQ_BITS;
    uint8_t const* setup = remove_volatile_cast(uint8_t const*, &usb_dpram->setup_packet);

    // reset pid to both 1 (data and ack)
    reset_ep0();

    // Pass setup packet to tiny usb
    dcd_event_setup_received(0, setup, true);
    usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
  }

#if FORCE_VBUS_DETECT == 0
  // Since we force VBUS detect On, device will always think it is connected and
  // couldn't distinguish between disconnect and suspend
  if (status & USB_INTS_DEV_CONN_DIS_BITS)
  {
    handled |= USB_INTS_DEV_CONN_DIS_BITS;

    if ( usb_hw->sie_status & USB_SIE_STATUS_CONNECTED_BITS )
    {
      // Connected: nothing to do
    }else
    {
      // Disconnected
      dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, true);
    }

    usb_hw_clear->sie_status = USB_SIE_STATUS_CONNECTED_BITS;
  }
#endif

  // SE0 for 2.5 us or more (will last at least 10ms)
  if (status & USB_INTS_BUS_RESET_BITS) {
    pico_trace("BUS RESET\r\n");

    handled |= USB_INTS_BUS_RESET_BITS;

    usb_hw->dev_addr_ctrl = 0;
    reset_non_control_endpoints();
    dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
    usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;

#if TUD_OPT_RP2040_USB_DEVICE_ENUMERATION_FIX
    // Only run enumeration workaround if pull up is enabled
    if (usb_hw->sie_ctrl & USB_SIE_CTRL_PULLUP_EN_BITS) rp2040_usb_device_enumeration_fix();
#endif
  }

  /* Note from pico datasheet 4.1.2.6.4 (v1.2)
   * If you enable the suspend interrupt, it is likely you will see a suspend interrupt when
   * the device is first connected but the bus is idle. The bus can be idle for a few ms before
   * the host begins sending start of frame packets. You will also see a suspend interrupt
   * when the device is disconnected if you do not have a VBUS detect circuit connected. This is
   * because without VBUS detection, it is impossible to tell the difference between
   * being disconnected and suspended.
   */
  if (status & USB_INTS_DEV_SUSPEND_BITS) {
    handled |= USB_INTS_DEV_SUSPEND_BITS;
    dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
    usb_hw_clear->sie_status = USB_SIE_STATUS_SUSPENDED_BITS;
  }

  if (status & USB_INTS_DEV_RESUME_FROM_HOST_BITS) {
    handled |= USB_INTS_DEV_RESUME_FROM_HOST_BITS;
    dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
    usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;
  }

  if (status ^ handled) {
    panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
  }
}

#define USB_INTS_ERROR_BITS ( \
    USB_INTS_ERROR_DATA_SEQ_BITS      |  \
    USB_INTS_ERROR_BIT_STUFF_BITS     |  \
    USB_INTS_ERROR_CRC_BITS           |  \
    USB_INTS_ERROR_RX_OVERFLOW_BITS   |  \
    USB_INTS_ERROR_RX_TIMEOUT_BITS)

/*------------------------------------------------------------------*/
/* Controller API
 *------------------------------------------------------------------*/

// older SDK
#ifndef PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY
#define PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY 0xff
#endif

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  assert(rhport == 0);

  TU_LOG(2, "Chip Version B%u\r\n", rp2040_chip_version());

  // Reset hardware to default state
  rp2040_usb_init();

#if FORCE_VBUS_DETECT
  // Force VBUS detect so the device thinks it is plugged into a host
  usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
#endif

  irq_add_shared_handler(USBCTRL_IRQ, dcd_rp2040_irq, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);

  // Init control endpoints
  tu_memclr(hw_endpoints[0], 2 * sizeof(hw_endpoint_t));
  hw_endpoint_open(0x0, 64, TUSB_XFER_CONTROL);
  hw_endpoint_open(0x80, 64, TUSB_XFER_CONTROL);

  // Init non-control endpoints
  reset_non_control_endpoints();

  // Initializes the USB peripheral for device mode and enables it.
  // Don't need to enable the pull up here. Force VBUS
  usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

  // Enable individual controller IRQS here. Processor interrupt enable will be used
  // for the global interrupt enable...
  // Note: Force VBUS detect cause disconnection not detectable
  usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
  usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS |
                 USB_INTS_DEV_SUSPEND_BITS | USB_INTS_DEV_RESUME_FROM_HOST_BITS |
                 (FORCE_VBUS_DETECT ? 0 : USB_INTS_DEV_CONN_DIS_BITS);

  dcd_connect(rhport);
  return true;
}

bool dcd_deinit(uint8_t rhport) {
  (void) rhport;

  reset_non_control_endpoints();
  irq_remove_handler(USBCTRL_IRQ, dcd_rp2040_irq);

  // reset usb hardware into initial state
  reset_block(RESETS_RESET_USBCTRL_BITS);
  unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

  return true;
}

void dcd_int_enable(__unused uint8_t rhport) {
  assert(rhport == 0);
  irq_set_enabled(USBCTRL_IRQ, true);
}

void dcd_int_disable(__unused uint8_t rhport) {
  assert(rhport == 0);
  irq_set_enabled(USBCTRL_IRQ, false);
}

void dcd_set_address(__unused uint8_t rhport, __unused uint8_t dev_addr) {
  assert(rhport == 0);

  // Can't set device address in hardware until status xfer has complete
  // Send 0len complete response on EP0 IN
  hw_endpoint_xfer(0x80, NULL, 0);
}

void dcd_remote_wakeup(__unused uint8_t rhport) {
  pico_info("dcd_remote_wakeup %d\n", rhport);
  assert(rhport == 0);

  // since RESUME interrupt is not triggered if we are the one initiate
  // briefly enable SOF to notify usbd when bus is ready
  usb_hw_set->inte = USB_INTS_DEV_SOF_BITS;
  usb_hw_set->sie_ctrl = USB_SIE_CTRL_RESUME_BITS;
}

// disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(__unused uint8_t rhport) {
  (void) rhport;
  usb_hw_clear->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

// connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(__unused uint8_t rhport) {
  (void) rhport;
  usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

void dcd_sof_enable(uint8_t rhport, bool en) {
  (void) rhport;

  _sof_enable = en;

  if (en) {
    usb_hw_set->inte = USB_INTS_DEV_SOF_BITS;
  }
#if !TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX
  else {
    // Don't clear immediately if the SOF workaround is in use.
    // The SOF handler will conditionally disable the interrupt.
    usb_hw_clear->inte = USB_INTS_DEV_SOF_BITS;
  }
#endif
}

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const* request) {
  (void) rhport;

  if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
      request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
      request->bRequest == TUSB_REQ_SET_ADDRESS) {
    usb_hw->dev_addr_ctrl = (uint8_t) request->wValue;
  }
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const* desc_edpt) {
  (void) rhport;
  const uint8_t xfer_type = desc_edpt->bmAttributes.xfer;
  TU_VERIFY(xfer_type != TUSB_XFER_ISOCHRONOUS);
  hw_endpoint_open(desc_edpt->bEndpointAddress, tu_edpt_packet_size(desc_edpt), xfer_type);
  return true;
}

// New API: Allocate packet buffer used by ISO endpoints
// Some MCU need manual packet buffer allocation, we allocate the largest size to avoid clustering
bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
  (void) rhport;
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);
  hw_endpoint_init(ep_addr, largest_packet_size, TUSB_XFER_ISOCHRONOUS);
  hw_endpoint_alloc(ep, largest_packet_size);
  return true;
}

// New API: Configure and enable an ISO endpoint according to descriptor
bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc) {
  (void) rhport;
  const uint8_t ep_addr = ep_desc->bEndpointAddress;
  // Fill in endpoint control register with buffer offset
  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);
  TU_ASSERT(ep->hw_data_buf != NULL); // must be inited and buffer allocated
  ep->wMaxPacketSize = ep_desc->wMaxPacketSize;

  hw_endpoint_enable(ep);
  return true;
}

void dcd_edpt_close_all(uint8_t rhport) {
  (void) rhport;

  // may need to use EP Abort
  reset_non_control_endpoints();
}

bool dcd_edpt_xfer(__unused uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  assert(rhport == 0);
  hw_endpoint_xfer(ep_addr, buffer, total_bytes);
  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  if (tu_edpt_number(ep_addr) == 0) {
    // A stall on EP0 has to be armed so it can be cleared on the next setup packet
    usb_hw_set->ep_stall_arm = (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) ? USB_EP_STALL_ARM_EP0_IN_BITS
                                                                     : USB_EP_STALL_ARM_EP0_OUT_BITS;
  }

  struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);

  // stall and clear current pending buffer
  // may need to use EP_ABORT
  _hw_endpoint_buffer_control_set_value32(ep, USB_BUF_CTRL_STALL);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  if (tu_edpt_number(ep_addr)) {
    struct hw_endpoint* ep = hw_endpoint_get_by_addr(ep_addr);

    // clear stall also reset toggle to DATA0, ready for next transfer
    ep->next_pid = 0;
    _hw_endpoint_buffer_control_clear_mask32(ep, USB_BUF_CTRL_STALL);
  }
}

void __tusb_irq_path_func(dcd_int_handler)(uint8_t rhport) {
  (void) rhport;
  dcd_rp2040_irq();
}

#endif
