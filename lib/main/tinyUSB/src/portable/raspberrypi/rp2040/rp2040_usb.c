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

#if CFG_TUSB_MCU == OPT_MCU_RP2040

#include <stdlib.h>
#include "rp2040_usb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPE
//--------------------------------------------------------------------+
static void _hw_endpoint_xfer_sync(struct hw_endpoint* ep);

#if TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX
  static bool e15_is_bulkin_ep(struct hw_endpoint* ep);
  static bool e15_is_critical_frame_period(struct hw_endpoint* ep);
#else
  #define e15_is_bulkin_ep(x)             (false)
  #define e15_is_critical_frame_period(x) (false)
#endif

// if usb hardware is in host mode
TU_ATTR_ALWAYS_INLINE static inline bool is_host_mode(void) {
  return (usb_hw->main_ctrl & USB_MAIN_CTRL_HOST_NDEVICE_BITS) ? true : false;
}

//--------------------------------------------------------------------+
// Implementation
//--------------------------------------------------------------------+
// Provide own byte by byte memcpy as not all copies are aligned
static void unaligned_memcpy(void *dst, const void *src, size_t n) {
  uint8_t *dst_byte = (uint8_t*)dst;
  const uint8_t *src_byte = (const uint8_t*)src;
  while (n--) {
    *dst_byte++ = *src_byte++;
  }
}

void rp2040_usb_init(void) {
  // Reset usb controller
  reset_block(RESETS_RESET_USBCTRL_BITS);
  unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

#ifdef __GNUC__
  // Clear any previous state just in case
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#if __GNUC__ > 6
#pragma GCC diagnostic ignored "-Wstringop-overflow"
#endif
#endif
  memset(usb_dpram, 0, sizeof(*usb_dpram));
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

  // Mux the controller to the onboard usb phy
  usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

  TU_LOG2_INT(sizeof(hw_endpoint_t));
}

void __tusb_irq_path_func(hw_endpoint_reset_transfer)(struct hw_endpoint* ep) {
  ep->active = false;
  ep->remaining_len = 0;
  ep->xferred_len = 0;
  ep->user_buf = 0;
}

void __tusb_irq_path_func(_hw_endpoint_buffer_control_update32)(struct hw_endpoint* ep, uint32_t and_mask,
                                                               uint32_t or_mask) {
  uint32_t value = 0;

  if (and_mask) {
    value = *ep->buffer_control & and_mask;
  }

  if (or_mask) {
    value |= or_mask;
    if (or_mask & USB_BUF_CTRL_AVAIL) {
      if (*ep->buffer_control & USB_BUF_CTRL_AVAIL) {
        panic("ep %02X was already available", ep->ep_addr);
      }
      *ep->buffer_control = value & ~USB_BUF_CTRL_AVAIL;
      // 4.1.2.5.1 Con-current access: 12 cycles (should be good for 48*12Mhz = 576Mhz) after write to buffer control
      // Don't need delay in host mode as host is in charge
      if ( !is_host_mode()) {
        busy_wait_at_least_cycles(12);
      }
    }
  }

  *ep->buffer_control = value;
}

// prepare buffer, return buffer control
static uint32_t __tusb_irq_path_func(prepare_ep_buffer)(struct hw_endpoint* ep, uint8_t buf_id) {
  uint16_t const buflen = tu_min16(ep->remaining_len, ep->wMaxPacketSize);
  ep->remaining_len = (uint16_t) (ep->remaining_len - buflen);

  uint32_t buf_ctrl = buflen | USB_BUF_CTRL_AVAIL;

  // PID
  buf_ctrl |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
  ep->next_pid ^= 1u;

  if (!ep->rx) {
    // Copy data from user buffer to hw buffer
    unaligned_memcpy(ep->hw_data_buf + buf_id * 64, ep->user_buf, buflen);
    ep->user_buf += buflen;

    // Mark as full
    buf_ctrl |= USB_BUF_CTRL_FULL;
  }

  // Is this the last buffer? Only really matters for host mode. Will trigger
  // the trans complete irq but also stop it polling. We only really care about
  // trans complete for setup packets being sent
  if (ep->remaining_len == 0) {
    buf_ctrl |= USB_BUF_CTRL_LAST;
  }

  if (buf_id) buf_ctrl = buf_ctrl << 16;

  return buf_ctrl;
}

// Prepare buffer control register value
void __tusb_irq_path_func(hw_endpoint_start_next_buffer)(struct hw_endpoint* ep) {
  uint32_t ep_ctrl = *ep->endpoint_control;

  // always compute and start with buffer 0
  uint32_t buf_ctrl = prepare_ep_buffer(ep, 0) | USB_BUF_CTRL_SEL;

  // For now: skip double buffered for OUT endpoint in Device mode, since
  // host could send < 64 bytes and cause short packet on buffer0
  // NOTE: this could happen to Host mode IN endpoint
  // Also, Host mode "interrupt" endpoint hardware is only single buffered,
  // NOTE2: Currently Host bulk is implemented using "interrupt" endpoint
  bool const is_host = is_host_mode();
  bool const force_single = (!is_host && !tu_edpt_dir(ep->ep_addr)) ||
                            (is_host && tu_edpt_number(ep->ep_addr) != 0);

  if (ep->remaining_len && !force_single) {
    // Use buffer 1 (double buffered) if there is still data
    // TODO: Isochronous for buffer1 bit-field is different than CBI (control bulk, interrupt)

    buf_ctrl |= prepare_ep_buffer(ep, 1);

    // Set endpoint control double buffered bit if needed
    ep_ctrl &= ~EP_CTRL_INTERRUPT_PER_BUFFER;
    ep_ctrl |= EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER;
  } else {
    // Single buffered since 1 is enough
    ep_ctrl &= ~(EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER);
    ep_ctrl |= EP_CTRL_INTERRUPT_PER_BUFFER;
  }

  *ep->endpoint_control = ep_ctrl;

  TU_LOG(3, "  Prepare BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n", tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));

  // Finally, write to buffer_control which will trigger the transfer
  // the next time the controller polls this dpram address
  _hw_endpoint_buffer_control_set_value32(ep, buf_ctrl);
}

void hw_endpoint_xfer_start(struct hw_endpoint* ep, uint8_t* buffer, uint16_t total_len) {
  hw_endpoint_lock_update(ep, 1);

  if (ep->active) {
    // TODO: Is this acceptable for interrupt packets?
    TU_LOG(1, "WARN: starting new transfer on already active ep %02X\r\n", ep->ep_addr);
    hw_endpoint_reset_transfer(ep);
  }

  // Fill in info now that we're kicking off the hw
  ep->remaining_len = total_len;
  ep->xferred_len = 0;
  ep->active = true;
  ep->user_buf = buffer;

  if (e15_is_bulkin_ep(ep)) {
    usb_hw_set->inte = USB_INTS_DEV_SOF_BITS;
  }

  if (e15_is_critical_frame_period(ep)) {
    ep->pending = 1;
  } else {
    hw_endpoint_start_next_buffer(ep);
  }

  hw_endpoint_lock_update(ep, -1);
}

// sync endpoint buffer and return transferred bytes
static uint16_t __tusb_irq_path_func(sync_ep_buffer)(struct hw_endpoint* ep, uint8_t buf_id) {
  uint32_t buf_ctrl = _hw_endpoint_buffer_control_get_value32(ep);
  if (buf_id) buf_ctrl = buf_ctrl >> 16;

  uint16_t xferred_bytes = buf_ctrl & USB_BUF_CTRL_LEN_MASK;

  if (!ep->rx) {
    // We are continuing a transfer here. If we are TX, we have successfully
    // sent some data can increase the length we have sent
    assert(!(buf_ctrl & USB_BUF_CTRL_FULL));

    ep->xferred_len = (uint16_t) (ep->xferred_len + xferred_bytes);
  } else {
    // If we have received some data, so can increase the length
    // we have received AFTER we have copied it to the user buffer at the appropriate offset
    assert(buf_ctrl & USB_BUF_CTRL_FULL);

    unaligned_memcpy(ep->user_buf, ep->hw_data_buf + buf_id * 64, xferred_bytes);
    ep->xferred_len = (uint16_t) (ep->xferred_len + xferred_bytes);
    ep->user_buf += xferred_bytes;
  }

  // Short packet
  if (xferred_bytes < ep->wMaxPacketSize) {
    pico_trace("  Short packet on buffer %d with %u bytes\r\n", buf_id, xferred_bytes);
    // Reduce total length as this is last packet
    ep->remaining_len = 0;
  }

  return xferred_bytes;
}

static void __tusb_irq_path_func(_hw_endpoint_xfer_sync)(struct hw_endpoint* ep) {
  // Update hw endpoint struct with info from hardware
  // after a buff status interrupt

  uint32_t __unused buf_ctrl = _hw_endpoint_buffer_control_get_value32(ep);
  TU_LOG(3, "  Sync BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n", tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));

  // always sync buffer 0
  uint16_t buf0_bytes = sync_ep_buffer(ep, 0);

  // sync buffer 1 if double buffered
  if ((*ep->endpoint_control) & EP_CTRL_DOUBLE_BUFFERED_BITS) {
    if (buf0_bytes == ep->wMaxPacketSize) {
      // sync buffer 1 if not short packet
      sync_ep_buffer(ep, 1);
    } else {
      // short packet on buffer 0
      // TODO couldn't figure out how to handle this case which happen with net_lwip_webserver example
      // At this time (currently trigger per 2 buffer), the buffer1 is probably filled with data from
      // the next transfer (not current one). For now we disable double buffered for device OUT
      // NOTE this could happen to Host IN
#if 0
      uint8_t const ep_num = tu_edpt_number(ep->ep_addr);
      uint8_t const dir =  (uint8_t) tu_edpt_dir(ep->ep_addr);
      uint8_t const ep_id = 2*ep_num + (dir ? 0 : 1);

      // abort queued transfer on buffer 1
      usb_hw->abort |= TU_BIT(ep_id);

      while ( !(usb_hw->abort_done & TU_BIT(ep_id)) ) {}

      uint32_t ep_ctrl = *ep->endpoint_control;
      ep_ctrl &= ~(EP_CTRL_DOUBLE_BUFFERED_BITS | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER);
      ep_ctrl |= EP_CTRL_INTERRUPT_PER_BUFFER;

      _hw_endpoint_buffer_control_set_value32(ep, 0);

      usb_hw->abort &= ~TU_BIT(ep_id);

      TU_LOG(3, "----SHORT PACKET buffer0 on EP %02X:\r\n", ep->ep_addr);
      TU_LOG(3, "  BufCtrl: [0] = 0x%04x  [1] = 0x%04x\r\n", tu_u32_low16(buf_ctrl), tu_u32_high16(buf_ctrl));
#endif
    }
  }
}

// Returns true if transfer is complete
bool __tusb_irq_path_func(hw_endpoint_xfer_continue)(struct hw_endpoint* ep) {
  hw_endpoint_lock_update(ep, 1);

  // Part way through a transfer
  if (!ep->active) {
    panic("Can't continue xfer on inactive ep %02X", ep->ep_addr);
  }

  // Update EP struct from hardware state
  _hw_endpoint_xfer_sync(ep);

  // Now we have synced our state with the hardware. Is there more data to transfer?
  // If we are done then notify tinyusb
  if (ep->remaining_len == 0) {
    pico_trace("Completed transfer of %d bytes on ep %02X\r\n", ep->xferred_len, ep->ep_addr);
    // Notify caller we are done so it can notify the tinyusb stack
    hw_endpoint_lock_update(ep, -1);
    return true;
  } else {
    if (e15_is_critical_frame_period(ep)) {
      ep->pending = 1;
    } else {
      hw_endpoint_start_next_buffer(ep);
    }
  }

  hw_endpoint_lock_update(ep, -1);
  // More work to do
  return false;
}

//--------------------------------------------------------------------+
// Errata 15
//--------------------------------------------------------------------+

#if TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX

/* Don't mark IN buffers as available during the last 200us of a full-speed
   frame. This avoids a situation seen with the USB2.0 hub on a Raspberry
   Pi 4 where a late IN token before the next full-speed SOF can cause port
   babble and a corrupt ACK packet. The nature of the data corruption has a
   chance to cause device lockup.

   Use the next SOF to mark delayed buffers as available. This reduces
   available Bulk IN bandwidth by approximately 20%, and requires that the
   SOF interrupt is enabled while these transfers are ongoing.

   Inherit the top-level enable from the corresponding Pico-SDK flag.
   Applications that will not use the device in a situation where it could
   be plugged into a Pi 4 or Pi 400 (for example, when directly connected
   to a commodity hub or other host) can turn off the flag in the SDK.
*/

volatile uint32_t e15_last_sof = 0;

// check if Errata 15 is needed for this endpoint i.e device bulk-in
static bool __tusb_irq_path_func(e15_is_bulkin_ep)(struct hw_endpoint* ep) {
  return (!is_host_mode() && tu_edpt_dir(ep->ep_addr) == TUSB_DIR_IN &&
          ep->transfer_type == TUSB_XFER_BULK);
}

// check if we need to apply Errata 15 workaround : i.e
// Endpoint is BULK IN and is currently in critical frame period i.e 20% of last usb frame
static bool __tusb_irq_path_func(e15_is_critical_frame_period)(struct hw_endpoint* ep) {
  TU_VERIFY(e15_is_bulkin_ep(ep));

  /* Avoid the last 200us (uframe 6.5-7) of a frame, up to the EOF2 point.
   * The device state machine cannot recover from receiving an incorrect PID
   * when it is expecting an ACK.
   */
  uint32_t delta = time_us_32() - e15_last_sof;
  if (delta < 800 || delta > 998) {
    return false;
  }
  TU_LOG(3, "Avoiding sof %lu now %lu last %lu\r\n", (usb_hw->sof_rd + 1) & USB_SOF_RD_BITS, time_us_32(),
         e15_last_sof);
  return true;
}

#endif // TUD_OPT_RP2040_USB_DEVICE_UFRAME_FIX
#endif
