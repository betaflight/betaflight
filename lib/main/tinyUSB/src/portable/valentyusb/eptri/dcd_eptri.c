/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#if CFG_TUD_ENABLED && (CFG_TUSB_MCU == OPT_MCU_VALENTYUSB_EPTRI)

#ifndef DEBUG
#define DEBUG 0
#endif

#ifndef LOG_USB
#define LOG_USB 0
#endif

#include "device/dcd.h"
#include "dcd_eptri.h"
#include "csr.h"
#include "irq.h"
void fomu_error(uint32_t line);

#if LOG_USB
struct usb_log {
  uint8_t ep_num;
  uint8_t size;
  uint8_t data[66];
};
__attribute__((used))
struct usb_log usb_log[128];
__attribute__((used))
uint8_t usb_log_offset;

struct xfer_log {
  uint8_t ep_num;
  uint16_t size;
};
__attribute__((used))
struct xfer_log xfer_log[64];
__attribute__((used))
uint8_t xfer_log_offset;

__attribute__((used))
struct xfer_log queue_log[64];
__attribute__((used))
uint8_t queue_log_offset;
#endif

//--------------------------------------------------------------------+
// SIE Command
//--------------------------------------------------------------------+

#define EP_SIZE 64

uint16_t volatile rx_buffer_offset[16];
uint8_t* volatile rx_buffer[16];
uint16_t volatile rx_buffer_max[16];

volatile uint8_t tx_ep;
volatile bool tx_active;
volatile uint16_t tx_buffer_offset[16];
uint8_t* volatile tx_buffer[16];
volatile uint16_t tx_buffer_max[16];
volatile uint8_t reset_count;

#if DEBUG
__attribute__((used)) uint8_t volatile * last_tx_buffer;
__attribute__((used)) volatile uint8_t last_tx_ep;
uint8_t setup_packet_bfr[10];
#endif

//--------------------------------------------------------------------+
// PIPE HELPER
//--------------------------------------------------------------------+

static bool advance_tx_ep(void) {
  // Move on to the next transmit buffer in a round-robin manner
  uint8_t prev_tx_ep = tx_ep;
  for (tx_ep = (tx_ep + 1) & 0xf; tx_ep != prev_tx_ep; tx_ep = ((tx_ep + 1) & 0xf)) {
    if (tx_buffer[tx_ep])
      return true;
  }
  if (!tx_buffer[tx_ep])
    return false;
  return true;
}

#if LOG_USB
void xfer_log_append(uint8_t ep_num, uint16_t sz) {
  xfer_log[xfer_log_offset].ep_num = ep_num;
  xfer_log[xfer_log_offset].size = sz;
  xfer_log_offset++;
  if (xfer_log_offset >= sizeof(xfer_log)/sizeof(*xfer_log))
    xfer_log_offset = 0;
}

void queue_log_append(uint8_t ep_num, uint16_t sz) {
  queue_log[queue_log_offset].ep_num = ep_num;
  queue_log[queue_log_offset].size = sz;
  queue_log_offset++;
  if (queue_log_offset >= sizeof(queue_log)/sizeof(*queue_log))
    queue_log_offset = 0;
}
#endif

static void tx_more_data(void) {
  // Send more data
  uint8_t added_bytes;
  for (added_bytes = 0; (added_bytes < EP_SIZE) && (tx_buffer_offset[tx_ep] < tx_buffer_max[tx_ep]); added_bytes++) {
#if LOG_USB
    usb_log[usb_log_offset].data[added_bytes] = tx_buffer[tx_ep][tx_buffer_offset[tx_ep]];
#endif
    usb_in_data_write(tx_buffer[tx_ep][tx_buffer_offset[tx_ep]++]);
  }

#if LOG_USB
  usb_log[usb_log_offset].ep_num = tu_edpt_addr(tx_ep, TUSB_DIR_IN);
  usb_log[usb_log_offset].size = added_bytes;
  usb_log_offset++;
  if (usb_log_offset >= sizeof(usb_log)/sizeof(*usb_log))
    usb_log_offset = 0;
#endif

  // Updating the epno queues the data
  usb_in_ctrl_write(tx_ep & 0xf);
}

static void process_tx(void) {
#if DEBUG
  // If the system isn't idle, then something is very wrong.
  uint8_t in_status = usb_in_status_read();
  if (!(in_status & (1 << CSR_USB_IN_STATUS_IDLE_OFFSET)))
    fomu_error(__LINE__);
#endif

  // If the buffer is now empty, search for the next buffer to fill.
  if (!tx_buffer[tx_ep]) {
    if (advance_tx_ep())
      tx_more_data();
    else
      tx_active = false;
    return;
  }

  if (tx_buffer_offset[tx_ep] >= tx_buffer_max[tx_ep]) {
#if DEBUG
    last_tx_buffer = tx_buffer[tx_ep];
    last_tx_ep = tx_ep;
#endif
    tx_buffer[tx_ep] = NULL;
    uint16_t xferred_bytes = tx_buffer_max[tx_ep];
    uint8_t xferred_ep = tx_ep;

    if (!advance_tx_ep())
      tx_active = false;
#if LOG_USB
    xfer_log_append(tu_edpt_addr(xferred_ep, TUSB_DIR_IN), xferred_bytes);
#endif
    dcd_event_xfer_complete(0, tu_edpt_addr(xferred_ep, TUSB_DIR_IN), xferred_bytes, XFER_RESULT_SUCCESS, true);
    if (!tx_active)
      return;
  }

  tx_more_data();
  return;
}

static void process_rx(void) {
  uint8_t out_status = usb_out_status_read();
#if DEBUG
  // If the OUT handler is still waiting to send, don't do anything.
  if (!(out_status & (1 << CSR_USB_OUT_STATUS_HAVE_OFFSET)))
    fomu_error(__LINE__);
    // return;
#endif
  uint8_t rx_ep = (out_status >> CSR_USB_OUT_STATUS_EPNO_OFFSET) & 0xf;

  // If the destination buffer doesn't exist, don't drain the hardware
  // fifo.  Note that this can cause deadlocks if the host is waiting
  // on some other endpoint's data!
#if DEBUG
  if (rx_buffer[rx_ep] == NULL) {
    fomu_error(__LINE__);
    return;
  }
#endif

  // Drain the FIFO into the destination buffer
  uint32_t total_read = 0;
  uint32_t current_offset = rx_buffer_offset[rx_ep];
#if DEBUG
  uint8_t test_buffer[256];
  memset(test_buffer, 0, sizeof(test_buffer));
  if (current_offset > rx_buffer_max[rx_ep])
    fomu_error(__LINE__);
#endif
#if LOG_USB
  usb_log[usb_log_offset].ep_num = tu_edpt_addr(rx_ep, TUSB_DIR_OUT);
  usb_log[usb_log_offset].size = 0;
#endif
  while (usb_out_status_read() & (1 << CSR_USB_OUT_STATUS_HAVE_OFFSET)) {
    uint8_t c = usb_out_data_read();
#if DEBUG
    test_buffer[total_read] = c;
#endif
    total_read++;
    if (current_offset < rx_buffer_max[rx_ep]) {
#if LOG_USB
      usb_log[usb_log_offset].data[usb_log[usb_log_offset].size++] = c;
#endif
      if (rx_buffer[rx_ep] != (volatile uint8_t *)0xffffffff)
        rx_buffer[rx_ep][current_offset++] = c;
    }
  }
#if LOG_USB
  usb_log_offset++;
  if (usb_log_offset >= sizeof(usb_log)/sizeof(*usb_log))
    usb_log_offset = 0;
#endif
#if DEBUG
  if (total_read > 66)
    fomu_error(__LINE__);
  if (total_read < 2)
    total_read = 2;
    // fomu_error(__LINE__);
#endif

  // Strip off the CRC16
  rx_buffer_offset[rx_ep] += (total_read - 2);
  if (rx_buffer_offset[rx_ep] > rx_buffer_max[rx_ep])
    rx_buffer_offset[rx_ep] = rx_buffer_max[rx_ep];

  // If there's no more data, complete the transfer to tinyusb
  if ((rx_buffer_max[rx_ep] == rx_buffer_offset[rx_ep])
  // ZLP with less than the total amount of data
  || ((total_read == 2) && ((rx_buffer_offset[rx_ep] & 63) == 0))
  // Short read, but not a full packet
  || (((rx_buffer_offset[rx_ep] & 63) != 0) && (total_read < 66))) {
#if DEBUG
    if (rx_buffer[rx_ep] == NULL)
      fomu_error(__LINE__);
#endif

    // Free up this buffer.
    rx_buffer[rx_ep] = NULL;
    uint16_t len = rx_buffer_offset[rx_ep];

#if DEBUG
    // Validate that all enabled endpoints have buffers,
    // and no disabled endpoints have buffers.
    uint16_t ep_en_mask = usb_out_enable_status_read();
    int i;
    for (i = 0; i < 16; i++) {
      if ((!!(ep_en_mask & (1 << i))) ^ (!!(rx_buffer[i]))) {
        uint8_t new_status = usb_out_status_read();
        // Another IRQ came in while we were processing, so ignore this endpoint.
        if ((new_status & 0x20) && ((new_status & 0xf) == i))
          continue;
        fomu_error(__LINE__);
      }
    }
#endif
#if LOG_USB
    xfer_log_append(tu_edpt_addr(rx_ep, TUSB_DIR_OUT), len);
#endif
    dcd_event_xfer_complete(0, tu_edpt_addr(rx_ep, TUSB_DIR_OUT), len, XFER_RESULT_SUCCESS, true);
  }
  else {
    // If there's more data, re-enable data reception on this endpoint
    usb_out_ctrl_write((1 << CSR_USB_OUT_CTRL_ENABLE_OFFSET) | rx_ep);
  }

  // Now that the buffer is drained, clear the pending IRQ.
  usb_out_ev_pending_write(usb_out_ev_pending_read());
}

//--------------------------------------------------------------------+
// CONTROLLER API
//--------------------------------------------------------------------+

static void dcd_reset(void)
{
  reset_count++;
  usb_setup_ev_enable_write(0);
  usb_in_ev_enable_write(0);
  usb_out_ev_enable_write(0);

  usb_address_write(0);

  // Reset all three FIFO handlers
  usb_setup_ctrl_write(1 << CSR_USB_SETUP_CTRL_RESET_OFFSET);
  usb_in_ctrl_write(1 << CSR_USB_IN_CTRL_RESET_OFFSET);
  usb_out_ctrl_write(1 << CSR_USB_OUT_CTRL_RESET_OFFSET);

  memset((void *)(uintptr_t) rx_buffer, 0, sizeof(rx_buffer));
  memset((void *)(uintptr_t) rx_buffer_max, 0, sizeof(rx_buffer_max));
  memset((void *)(uintptr_t) rx_buffer_offset, 0, sizeof(rx_buffer_offset));

  memset((void *)(uintptr_t) tx_buffer, 0, sizeof(tx_buffer));
  memset((void *)(uintptr_t) tx_buffer_max, 0, sizeof(tx_buffer_max));
  memset((void *)(uintptr_t) tx_buffer_offset, 0, sizeof(tx_buffer_offset));
  tx_ep = 0;
  tx_active = false;

  // Enable all event handlers and clear their contents
  usb_setup_ev_pending_write(0xff);
  usb_in_ev_pending_write(0xff);
  usb_out_ev_pending_write(0xff);
  usb_in_ev_enable_write(1);
  usb_out_ev_enable_write(1);
  usb_setup_ev_enable_write(3);

  dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
}

// Initializes the USB peripheral for device mode and enables it.
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  usb_pullup_out_write(0);

  // Enable all event handlers and clear their contents
  usb_setup_ev_pending_write(usb_setup_ev_pending_read());
  usb_in_ev_pending_write(usb_in_ev_pending_read());
  usb_out_ev_pending_write(usb_out_ev_pending_read());
  usb_in_ev_enable_write(1);
  usb_out_ev_enable_write(1);
  usb_setup_ev_enable_write(3);

  // Turn on the external pullup
  usb_pullup_out_write(1);

  return true;
}

// Enables or disables the USB device interrupt(s). May be used to
// prevent concurrency issues when mutating data structures shared
// between main code and the interrupt handler.
void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;
	irq_setmask(irq_getmask() | (1 << USB_INTERRUPT));
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  irq_setmask(irq_getmask() & ~(1 << USB_INTERRUPT));
}

// Called when the device is given a new bus address.
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  // Respond with ACK status first before changing device address
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);

  // Wait for the response packet to get sent
  while (tx_active)
    ;

  // Activate the new address
  usb_address_write(dev_addr);
}

// Called to remote wake up host when suspended (e.g hid keyboard)
void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  usb_pullup_out_write(1);
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  usb_pullup_out_write(0);
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// DCD Endpoint Port
//--------------------------------------------------------------------+
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * p_endpoint_desc)
{
  (void) rhport;
  uint8_t ep_num = tu_edpt_number(p_endpoint_desc->bEndpointAddress);
  uint8_t ep_dir = tu_edpt_dir(p_endpoint_desc->bEndpointAddress);

  if (p_endpoint_desc->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS)
    return false; // Not supported

  if (ep_dir == TUSB_DIR_OUT) {
    rx_buffer_offset[ep_num] = 0;
    rx_buffer_max[ep_num] = 0;
    rx_buffer[ep_num] = NULL;
  }

  else if (ep_dir == TUSB_DIR_IN) {
    tx_buffer_offset[ep_num] = 0;
    tx_buffer_max[ep_num] = 0;
    tx_buffer[ep_num] = NULL;
  }

  return true;
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport; (void) ep_addr;
  // TODO implement dcd_edpt_close()
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  if (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT) {
    uint8_t enable = 0;
    if (rx_buffer[ep_addr])
      enable = 1;
    usb_out_ctrl_write((1 << CSR_USB_OUT_CTRL_STALL_OFFSET) | (enable << CSR_USB_OUT_CTRL_ENABLE_OFFSET) | tu_edpt_number(ep_addr));
  }
  else
    usb_in_ctrl_write((1 << CSR_USB_IN_CTRL_STALL_OFFSET) | tu_edpt_number(ep_addr));
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  if (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT) {
    uint8_t enable = 0;
    if (rx_buffer[ep_addr])
      enable = 1;
    usb_out_ctrl_write((0 << CSR_USB_OUT_CTRL_STALL_OFFSET) | (enable << CSR_USB_OUT_CTRL_ENABLE_OFFSET) | tu_edpt_number(ep_addr));
  }
  // IN endpoints will get un-stalled when more data is written.
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes)
{
  (void)rhport;
  uint8_t ep_num = tu_edpt_number(ep_addr);
  uint8_t ep_dir = tu_edpt_dir(ep_addr);
  TU_ASSERT(ep_num < 16);

  // Give a nonzero buffer when we transmit 0 bytes, so that the
  // system doesn't think the endpoint is idle.
  if ((buffer == NULL) && (total_bytes == 0)) {
    buffer = (uint8_t *)0xffffffff;
  }

  TU_ASSERT(buffer != NULL);

  if (ep_dir == TUSB_DIR_IN) {
    // Wait for the tx pipe to free up
    uint8_t previous_reset_count = reset_count;
    // Continue until the buffer is empty, the system is idle, and the fifo is empty.
    while (tx_buffer[ep_num] != NULL)
      ;

    dcd_int_disable(0);
#if LOG_USB
    queue_log_append(ep_addr, total_bytes);
#endif
    // If a reset happens while we're waiting, abort the transfer
    if (previous_reset_count != reset_count)
      return true;

    TU_ASSERT(tx_buffer[ep_num] == NULL);
    tx_buffer_offset[ep_num] = 0;
    tx_buffer_max[ep_num] = total_bytes;
    tx_buffer[ep_num] = buffer;

    // If the current buffer is NULL, then that means the tx logic is idle.
    // Update the tx_ep to point to our endpoint number and queue the data.
    // Otherwise, let it be and it'll get picked up after the next transfer
    // finishes.
    if (!tx_active) {
      tx_ep = ep_num;
      tx_active = true;
      tx_more_data();
    }
    dcd_int_enable(0);
  }

  else if (ep_dir == TUSB_DIR_OUT) {
    while (rx_buffer[ep_num] != NULL)
      ;

    TU_ASSERT(rx_buffer[ep_num] == NULL);
    dcd_int_disable(0);
#if LOG_USB
    queue_log_append(ep_addr, total_bytes);
#endif
    rx_buffer[ep_num] = buffer;
    rx_buffer_offset[ep_num] = 0;
    rx_buffer_max[ep_num] = total_bytes;

    // Enable receiving on this particular endpoint
    usb_out_ctrl_write((1 << CSR_USB_OUT_CTRL_ENABLE_OFFSET) | ep_num);
#if DEBUG
    uint16_t ep_en_mask = usb_out_enable_status_read();
    int i;
    for (i = 0; i < 16; i++) {
      if ((!!(ep_en_mask & (1 << i))) ^ (!!(rx_buffer[i]))) {
        if (rx_buffer[i] && usb_out_ev_pending_read() && (usb_out_status_read() & 0xf) == i)
          continue;
        fomu_error(__LINE__);
      }
    }
#endif
    dcd_int_enable(0);
  }
  return true;
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+

static void handle_out(void)
{
  // An "OUT" transaction just completed so we have new data.
  // (But only if we can accept the data)
#if DEBUG
  if (!usb_out_ev_pending_read())
    fomu_error(__LINE__);
  if (!usb_out_ev_enable_read())
    fomu_error(__LINE__);
#endif
  process_rx();
}

static void handle_in(void)
{
#if DEBUG
  if (!usb_in_ev_pending_read())
    fomu_error(__LINE__);
  if (!usb_in_ev_enable_read())
    fomu_error(__LINE__);
#endif
  usb_in_ev_pending_write(usb_in_ev_pending_read());
  process_tx();
}

static void handle_reset(void)
{
#if DEBUG
  uint8_t setup_pending   = usb_setup_ev_pending_read() & usb_setup_ev_enable_read();
  if (!(setup_pending & 2))
    fomu_error(__LINE__);
#endif
  usb_setup_ev_pending_write(2);

  // This event means a bus reset occurred.  Reset everything, and
  // abandon any further processing.
  dcd_reset();
}

static void handle_setup(void)
{
#if !DEBUG
  uint8_t setup_packet_bfr[10];
#endif

#if DEBUG
  uint8_t setup_pending   = usb_setup_ev_pending_read() & usb_setup_ev_enable_read();
  if (!(setup_pending & 1))
    fomu_error(__LINE__);
#endif

  // We got a SETUP packet.  Copy it to the setup buffer and clear
  // the "pending" bit.
  // Setup packets are always 8 bytes, plus two bytes of crc16.
  uint32_t setup_length = 0;

#if DEBUG
  if (!(usb_setup_status_read() & (1 << CSR_USB_SETUP_STATUS_HAVE_OFFSET)))
    fomu_error(__LINE__);
#endif

  while (usb_setup_status_read() & (1 << CSR_USB_SETUP_STATUS_HAVE_OFFSET)) {
    uint8_t c = usb_setup_data_read();
    if (setup_length < sizeof(setup_packet_bfr))
      setup_packet_bfr[setup_length] = c;
    setup_length++;
  }

  // If we have 10 bytes, that's a full SETUP packet plus CRC16.
  // Otherwise, it was an RX error.
  if (setup_length == 10) {
    dcd_event_setup_received(0, setup_packet_bfr, true);
  }
#if DEBUG
  else {
    fomu_error(__LINE__);
  }
#endif

  usb_setup_ev_pending_write(1);
}
void dcd_int_handler(uint8_t rhport)
{
  (void)rhport;
  uint8_t next_ev;
  while ((next_ev = usb_next_ev_read())) {
    switch (next_ev) {
    case 1 << CSR_USB_NEXT_EV_IN_OFFSET:
      handle_in();
      break;
    case 1 << CSR_USB_NEXT_EV_OUT_OFFSET:
      handle_out();
      break;
    case 1 << CSR_USB_NEXT_EV_SETUP_OFFSET:
      handle_setup();
      break;
    case 1 << CSR_USB_NEXT_EV_RESET_OFFSET:
      handle_reset();
      break;
    }
  }
}

#endif
