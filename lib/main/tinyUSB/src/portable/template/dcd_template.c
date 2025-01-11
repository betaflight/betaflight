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

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE

#include "device/dcd.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+


/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/

// Initialize controller to device mode
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport; (void) rh_init;
  return true;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport) {
  (void) rhport;
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport) {
  (void) rhport;
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  (void) dev_addr;
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport) {
  (void) rhport;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport) {
  (void) rhport;
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport) {
  (void) rhport;
}

void dcd_sof_enable(uint8_t rhport, bool en) {
  (void) rhport;
  (void) en;
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * ep_desc) {
  (void) rhport;
  (void) ep_desc;
  return false;
}

// Allocate packet buffer used by ISO endpoints
// Some MCU need manual packet buffer allocation, we allocate the largest size to avoid clustering
bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
  (void) rhport;
  (void) ep_addr;
  (void) largest_packet_size;
  return false;
}

// Configure and enable an ISO endpoint according to descriptor
bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep) {
  (void) rhport;
  (void) desc_ep;
  return false;
}

void dcd_edpt_close_all (uint8_t rhport) {
  (void) rhport;
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes) {
  (void) rhport;
  (void) ep_addr;
  (void) buffer;
  (void) total_bytes;
  return false;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes) {
  (void) rhport;
  (void) ep_addr;
  (void) ff;
  (void) total_bytes;
  return false;
}

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  (void) ep_addr;
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  (void) ep_addr;
}

#endif
