/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2024 Hardy Griech
 * Copyright (c) 2020 Jacob Berg Potter
 * Copyright (c) 2020 Peter Lawrence
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

/**
 * Small Glossary (from the spec)
 * --------------
 * Datagram - A collection of bytes forming a single item of information, passed as a unit from source to destination.
 * NCM      - Network Control Model
 * NDP      - NCM Datagram Pointer: NTB structure that delineates Datagrams (typically Ethernet frames) within an NTB
 * NTB      - NCM Transfer Block: a data structure for efficient USB encapsulation of one or more datagrams
 *            Each NTB is designed to be a single USB transfer
 * NTH      - NTB Header: a data structure at the front of each NTB, which provides the information needed to validate
 *            the NTB and begin decoding
 *
 * Some explanations
 * -----------------
 * - rhport        is the USB port of the device, in most cases "0"
 * - itf_data_alt  if != 0 -> data xmit/recv are allowed (see spec)
 * - ep_in         IN endpoints take data from the device intended to go in to the host (the device transmits)
 * - ep_out        OUT endpoints send data out of the host to the device (the device receives)
 */

#include "tusb_option.h"

#if (CFG_TUD_ENABLED && CFG_TUD_NCM)

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "ncm.h"
#include "net_device.h"

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUD_NCM_LOG_LEVEL
  #define CFG_TUD_NCM_LOG_LEVEL   CFG_TUD_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUD_NCM_LOG_LEVEL, __VA_ARGS__)

// Alignment must be 4
#define TUD_NCM_ALIGNMENT   4
// calculate alignment of xmit datagrams within an NTB
#define XMIT_ALIGN_OFFSET(x) ((TUD_NCM_ALIGNMENT - ((x) & (TUD_NCM_ALIGNMENT - 1))) & (TUD_NCM_ALIGNMENT - 1))

//-----------------------------------------------------------------------------
//
// Module global things
//
#define XMIT_NTB_N CFG_TUD_NCM_IN_NTB_N
#define RECV_NTB_N CFG_TUD_NCM_OUT_NTB_N

typedef struct {
  // general
  uint8_t ep_in;        // endpoint for outgoing datagrams (naming is a little bit confusing)
  uint8_t ep_out;       // endpoint for incoming datagrams (naming is a little bit confusing)
  uint8_t ep_notif;     // endpoint for notifications
  uint8_t itf_num;      // interface number
  uint8_t itf_data_alt; // ==0 -> no endpoints, i.e. no network traffic, ==1 -> normal operation with two endpoints (spec, chapter 5.3)
  uint8_t rhport;       // storage of \a rhport because some callbacks are done without it

  // recv handling
  recv_ntb_t *recv_free_ntb[RECV_NTB_N];                // free list of recv NTBs
  recv_ntb_t *recv_ready_ntb[RECV_NTB_N];               // NTBs waiting for transmission to glue logic
  recv_ntb_t *recv_tinyusb_ntb;                         // buffer for the running transfer TinyUSB -> driver
  recv_ntb_t *recv_glue_ntb;                            // buffer for the running transfer driver -> glue logic
  uint16_t recv_glue_ntb_datagram_ndx;                  // index into \a recv_glue_ntb_datagram

  // xmit handling
  xmit_ntb_t *xmit_free_ntb[XMIT_NTB_N];                // free list of xmit NTBs
  xmit_ntb_t *xmit_ready_ntb[XMIT_NTB_N];               // NTBs waiting for transmission to TinyUSB
  xmit_ntb_t *xmit_tinyusb_ntb;                         // buffer for the running transfer driver -> TinyUSB
  xmit_ntb_t *xmit_glue_ntb;                            // buffer for the running transfer glue logic -> driver
  uint16_t xmit_sequence;                               // NTB sequence counter
  uint16_t xmit_glue_ntb_datagram_ndx;                  // index into \a xmit_glue_ntb_datagram

  // notification handling
  enum {
    NOTIFICATION_SPEED,
    NOTIFICATION_CONNECTED,
    NOTIFICATION_DONE
  } notification_xmit_state;                            // state of notification transmission
  bool notification_xmit_is_running;                    // notification is currently transmitted

  // misc
  bool tud_network_recv_renew_active;                   // tud_network_recv_renew() is active (avoid recursive invocations)
  bool tud_network_recv_renew_process_again;            // tud_network_recv_renew() should process again
} ncm_interface_t;

typedef struct {
  struct {
    TUD_EPBUF_TYPE_DEF(recv_ntb_t, ntb);
  } recv[RECV_NTB_N];

  struct {
    TUD_EPBUF_TYPE_DEF(xmit_ntb_t, ntb);
  } xmit[XMIT_NTB_N];

  TUD_EPBUF_TYPE_DEF(ncm_notify_t, epnotif);
} ncm_epbuf_t;

static ncm_interface_t ncm_interface;
CFG_TUD_MEM_SECTION static ncm_epbuf_t ncm_epbuf;

/**
 * This is the NTB parameter structure
 *
 * \attention
 *     We are lucky, that byte order is correct
 */
TU_ATTR_ALIGNED(4) static const ntb_parameters_t ntb_parameters = {
  .wLength                  = sizeof(ntb_parameters_t),
  .bmNtbFormatsSupported    = 0x01,// 16-bit NTB supported
  .dwNtbInMaxSize           = CFG_TUD_NCM_IN_NTB_MAX_SIZE,
  .wNdbInDivisor            = 1,
  .wNdbInPayloadRemainder   = 0,
  .wNdbInAlignment          = TUD_NCM_ALIGNMENT,
  .wReserved                = 0,
  .dwNtbOutMaxSize          = CFG_TUD_NCM_OUT_NTB_MAX_SIZE,
  .wNdbOutDivisor           = 1,
  .wNdbOutPayloadRemainder  = 0,
  .wNdbOutAlignment         = TUD_NCM_ALIGNMENT,
  .wNtbOutMaxDatagrams      = CFG_TUD_NCM_OUT_MAX_DATAGRAMS_PER_NTB,
};

// Some confusing remarks about wNtbOutMaxDatagrams...
//      ==1 -> SystemView packets/s goes up to 2000 and events are lost during startup
//      ==0 -> SystemView runs fine, iperf shows in wireshark a lot of error
//      ==6 -> SystemView runs fine, iperf also
//      >6  -> iperf starts to show errors
//      -> 6 seems to be the best value.  Why?  Don't know, perhaps only on my system?
//
//      iperf:    for MSS in 100 200 400 800 1200 1450 1500; do iperf -c 192.168.14.1 -e -i 1 -M $MSS -l 8192 -P 1; sleep 2; done
//      sysview:  SYSTICKS_PER_SEC=35000, IDLE_US=1000, PRINT_MOD=1000
//

//-----------------------------------------------------------------------------
//
// everything about notifications
//

/**
 * Transmit next notification to the host (if appropriate).
 * Notifications are transferred to the host once during connection setup.
 */
static void notification_xmit(uint8_t rhport, bool force_next) {
  TU_LOG_DRV("notification_xmit(%d, %d) - %d %d\n", force_next, rhport, ncm_interface.notification_xmit_state, ncm_interface.notification_xmit_is_running);

  if (!force_next && ncm_interface.notification_xmit_is_running) {
    return;
  }

  if (ncm_interface.notification_xmit_state == NOTIFICATION_SPEED) {
    TU_LOG_DRV("  NOTIFICATION_SPEED\n");
    ncm_notify_t notify_speed_change = {
      .header = {
        .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_IN
        },
        .bRequest = CDC_NOTIF_CONNECTION_SPEED_CHANGE,
        .wValue = 0,
        .wIndex = ncm_interface.itf_num,
        .wLength = 8
      }
    };
    if (tud_speed_get() == TUSB_SPEED_HIGH) {
      notify_speed_change.downlink = 480000000;
      notify_speed_change.uplink = 480000000;
    } else {
      notify_speed_change.downlink = 12000000;
      notify_speed_change.uplink = 12000000;
    }

    uint16_t notif_len = sizeof(notify_speed_change.header) + notify_speed_change.header.wLength;
    ncm_epbuf.epnotif = notify_speed_change;
    usbd_edpt_xfer(rhport, ncm_interface.ep_notif, (uint8_t*) &ncm_epbuf.epnotif, notif_len);

    ncm_interface.notification_xmit_state = NOTIFICATION_CONNECTED;
    ncm_interface.notification_xmit_is_running = true;
  } else if (ncm_interface.notification_xmit_state == NOTIFICATION_CONNECTED) {
    TU_LOG_DRV("  NOTIFICATION_CONNECTED\n");
    ncm_notify_t notify_connected = {
      .header = {
        .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_IN
        },
        .bRequest = CDC_NOTIF_NETWORK_CONNECTION,
        .wValue = 1 /* Connected */,
        .wIndex = ncm_interface.itf_num,
        .wLength = 0,
      },
    };

    uint16_t notif_len = sizeof(notify_connected.header) + notify_connected.header.wLength;
    ncm_epbuf.epnotif = notify_connected;
    usbd_edpt_xfer(rhport, ncm_interface.ep_notif, (uint8_t *) &ncm_epbuf.epnotif, notif_len);

    ncm_interface.notification_xmit_state = NOTIFICATION_DONE;
    ncm_interface.notification_xmit_is_running = true;
  } else {
    TU_LOG_DRV("  NOTIFICATION_FINISHED\n");
  }
} // notification_xmit

//-----------------------------------------------------------------------------
//
// everything about packet transmission (driver -> TinyUSB)
//

/**
 * Put NTB into the transmitter free list.
 */
static void xmit_put_ntb_into_free_list(xmit_ntb_t *free_ntb) {
  TU_LOG_DRV("xmit_put_ntb_into_free_list() - %p\n", ncm_interface.xmit_tinyusb_ntb);

  if (free_ntb == NULL) { // can happen due to ZLPs
    return;
  }

  for (int i = 0; i < XMIT_NTB_N; ++i) {
    if (ncm_interface.xmit_free_ntb[i] == NULL) {
      ncm_interface.xmit_free_ntb[i] = free_ntb;
      return;
    }
  }
  TU_LOG_DRV("(EE) xmit_put_ntb_into_free_list - no entry in free list\n");// this should not happen
} // xmit_put_ntb_into_free_list

/**
 * Get an NTB from the free list
 */
static xmit_ntb_t *xmit_get_free_ntb(void) {
  TU_LOG_DRV("xmit_get_free_ntb()\n");

  for (int i = 0; i < XMIT_NTB_N; ++i) {
    if (ncm_interface.xmit_free_ntb[i] != NULL) {
      xmit_ntb_t *free = ncm_interface.xmit_free_ntb[i];
      ncm_interface.xmit_free_ntb[i] = NULL;
      return free;
    }
  }
  return NULL;
} // xmit_get_free_ntb

/**
 * Put a filled NTB into the ready list
 */
static void xmit_put_ntb_into_ready_list(xmit_ntb_t *ready_ntb) {
  TU_LOG_DRV("xmit_put_ntb_into_ready_list(%p) %d\n", ready_ntb, ready_ntb->nth.wBlockLength);

  for (int i = 0; i < XMIT_NTB_N; ++i) {
    if (ncm_interface.xmit_ready_ntb[i] == NULL) {
      ncm_interface.xmit_ready_ntb[i] = ready_ntb;
      return;
    }
  }
  TU_LOG_DRV("(EE) xmit_put_ntb_into_ready_list: ready list full\n");// this should not happen
} // xmit_put_ntb_into_ready_list

/**
 * Get the next NTB from the ready list (and remove it from the list).
 * If the ready list is empty, return NULL.
 */
static xmit_ntb_t *xmit_get_next_ready_ntb(void) {
  xmit_ntb_t *r = NULL;

  r = ncm_interface.xmit_ready_ntb[0];
  memmove(ncm_interface.xmit_ready_ntb + 0, ncm_interface.xmit_ready_ntb + 1, sizeof(ncm_interface.xmit_ready_ntb) - sizeof(ncm_interface.xmit_ready_ntb[0]));
  ncm_interface.xmit_ready_ntb[XMIT_NTB_N - 1] = NULL;

  TU_LOG_DRV("recv_get_next_ready_ntb: %p\n", r);
  return r;
} // xmit_get_next_ready_ntb

/**
 * Transmit a ZLP if required
 *
 * \note
 *    Insertion of the ZLPs is a little bit different then described in the spec.
 *    But the below implementation actually works.  Don't know if this is a spec
 *    or TinyUSB issue.
 *
 * \pre
 *    This must be called from netd_xfer_cb() so that ep_in is ready
 */
static bool xmit_insert_required_zlp(uint8_t rhport, uint32_t xferred_bytes) {
  TU_LOG_DRV("xmit_insert_required_zlp(%d,%ld)\n", rhport, xferred_bytes);

  if (xferred_bytes == 0 || xferred_bytes % CFG_TUD_NET_ENDPOINT_SIZE != 0) {
    return false;
  }

  TU_ASSERT(ncm_interface.itf_data_alt == 1, false);
  TU_ASSERT(!usbd_edpt_busy(rhport, ncm_interface.ep_in), false);

  TU_LOG_DRV("xmit_insert_required_zlp! (%u)\n", (unsigned) xferred_bytes);

  // start transmission of the ZLP
  usbd_edpt_xfer(rhport, ncm_interface.ep_in, NULL, 0);

  return true;
} // xmit_insert_required_zlp

/**
 * Start transmission if it there is a waiting packet and if can be done from interface side.
 */
static void xmit_start_if_possible(uint8_t rhport) {
  TU_LOG_DRV("xmit_start_if_possible()\n");

  if (ncm_interface.xmit_tinyusb_ntb != NULL) {
    TU_LOG_DRV("  !xmit_start_if_possible 1\n");
    return;
  }
  if (ncm_interface.itf_data_alt != 1) {
    TU_LOG_DRV("(EE) !xmit_start_if_possible 2\n");
    return;
  }
  if (usbd_edpt_busy(rhport, ncm_interface.ep_in)) {
    TU_LOG_DRV("  !xmit_start_if_possible 3\n");
    return;
  }

  ncm_interface.xmit_tinyusb_ntb = xmit_get_next_ready_ntb();
  if (ncm_interface.xmit_tinyusb_ntb == NULL) {
    if (ncm_interface.xmit_glue_ntb == NULL || ncm_interface.xmit_glue_ntb_datagram_ndx == 0) {
      // -> really nothing is waiting
      return;
    }
    ncm_interface.xmit_tinyusb_ntb = ncm_interface.xmit_glue_ntb;
    ncm_interface.xmit_glue_ntb = NULL;
  }

  #if CFG_TUD_NCM_LOG_LEVEL >= 3
  {
    uint16_t len = ncm_interface.xmit_tinyusb_ntb->nth.wBlockLength;
    TU_LOG_BUF(3, ncm_interface.xmit_tinyusb_ntb->data[i], len);
  }
  #endif

  if (ncm_interface.xmit_glue_ntb_datagram_ndx != 1) {
    TU_LOG_DRV(">> %d %d\n", ncm_interface.xmit_tinyusb_ntb->nth.wBlockLength, ncm_interface.xmit_glue_ntb_datagram_ndx);
  }

  // Kick off an endpoint transfer
  usbd_edpt_xfer(0, ncm_interface.ep_in, ncm_interface.xmit_tinyusb_ntb->data, ncm_interface.xmit_tinyusb_ntb->nth.wBlockLength);
} // xmit_start_if_possible

/**
 * check if a new datagram fits into the current NTB
 */
static bool xmit_requested_datagram_fits_into_current_ntb(uint16_t datagram_size) {
  TU_LOG_DRV("xmit_requested_datagram_fits_into_current_ntb(%d) - %p %p\n", datagram_size, ncm_interface.xmit_tinyusb_ntb, ncm_interface.xmit_glue_ntb);

  if (ncm_interface.xmit_glue_ntb == NULL) {
    return false;
  }
  if (ncm_interface.xmit_glue_ntb_datagram_ndx >= CFG_TUD_NCM_IN_MAX_DATAGRAMS_PER_NTB) {
    return false;
  }
  if (ncm_interface.xmit_glue_ntb->nth.wBlockLength + datagram_size + XMIT_ALIGN_OFFSET(datagram_size) > CFG_TUD_NCM_OUT_NTB_MAX_SIZE) {
    return false;
  }
  return true;
} // xmit_requested_datagram_fits_into_current_ntb

/**
 * Setup an NTB for the glue logic
 */
static bool xmit_setup_next_glue_ntb(void) {
  TU_LOG_DRV("xmit_setup_next_glue_ntb - %p\n", ncm_interface.xmit_glue_ntb);

  if (ncm_interface.xmit_glue_ntb != NULL) {
    // put NTB into waiting list (the new datagram did not fit in)
    xmit_put_ntb_into_ready_list(ncm_interface.xmit_glue_ntb);
  }

  ncm_interface.xmit_glue_ntb = xmit_get_free_ntb();// get next buffer (if any)
  if (ncm_interface.xmit_glue_ntb == NULL) {
    TU_LOG_DRV("  xmit_setup_next_glue_ntb - nothing free\n");// should happen rarely
    return false;
  }

  ncm_interface.xmit_glue_ntb_datagram_ndx = 0;

  xmit_ntb_t *ntb = ncm_interface.xmit_glue_ntb;

  // Fill in NTB header
  ntb->nth.dwSignature = NTH16_SIGNATURE;
  ntb->nth.wHeaderLength = sizeof(ntb->nth);
  ntb->nth.wSequence = ncm_interface.xmit_sequence++;
  ntb->nth.wBlockLength = sizeof(ntb->nth) + sizeof(ntb->ndp) + sizeof(ntb->ndp_datagram);
  ntb->nth.wNdpIndex = sizeof(ntb->nth);

  // Fill in NDP16 header and terminator
  ntb->ndp.dwSignature = NDP16_SIGNATURE_NCM0;
  ntb->ndp.wLength = sizeof(ntb->ndp) + sizeof(ntb->ndp_datagram);
  ntb->ndp.wNextNdpIndex = 0;

  memset(ntb->ndp_datagram, 0, sizeof(ntb->ndp_datagram));
  return true;
} // xmit_setup_next_glue_ntb

//-----------------------------------------------------------------------------
//
// all the recv_*() stuff (TinyUSB -> driver -> glue logic)
//

/**
 * Return pointer to an available receive buffer or NULL.
 * Returned buffer (if any) has the size \a CFG_TUD_NCM_OUT_NTB_MAX_SIZE.
 */
static recv_ntb_t *recv_get_free_ntb(void) {
  TU_LOG_DRV("recv_get_free_ntb()\n");

  for (int i = 0; i < RECV_NTB_N; ++i) {
    if (ncm_interface.recv_free_ntb[i] != NULL) {
      recv_ntb_t *free = ncm_interface.recv_free_ntb[i];
      ncm_interface.recv_free_ntb[i] = NULL;
      return free;
    }
  }
  return NULL;
} // recv_get_free_ntb

/**
 * Get the next NTB from the ready list (and remove it from the list).
 * If the ready list is empty, return NULL.
 */
static recv_ntb_t *recv_get_next_ready_ntb(void) {
  recv_ntb_t *r = NULL;

  r = ncm_interface.recv_ready_ntb[0];
  memmove(ncm_interface.recv_ready_ntb + 0, ncm_interface.recv_ready_ntb + 1, sizeof(ncm_interface.recv_ready_ntb) - sizeof(ncm_interface.recv_ready_ntb[0]));
  ncm_interface.recv_ready_ntb[RECV_NTB_N - 1] = NULL;

  TU_LOG_DRV("recv_get_next_ready_ntb: %p\n", r);
  return r;
} // recv_get_next_ready_ntb

/**
 * Put NTB into the receiver free list.
 */
static void recv_put_ntb_into_free_list(recv_ntb_t *free_ntb) {
  TU_LOG_DRV("recv_put_ntb_into_free_list(%p)\n", free_ntb);

  for (int i = 0; i < RECV_NTB_N; ++i) {
    if (ncm_interface.recv_free_ntb[i] == NULL) {
      ncm_interface.recv_free_ntb[i] = free_ntb;
      return;
    }
  }
  TU_LOG_DRV("(EE) recv_put_ntb_into_free_list - no entry in free list\n");// this should not happen
} // recv_put_ntb_into_free_list

/**
 * \a ready_ntb holds a validated NTB,
 * put this buffer into the waiting list.
 */
static void recv_put_ntb_into_ready_list(recv_ntb_t *ready_ntb) {
  TU_LOG_DRV("recv_put_ntb_into_ready_list(%p) %d\n", ready_ntb, ready_ntb->nth.wBlockLength);

  for (int i = 0; i < RECV_NTB_N; ++i) {
    if (ncm_interface.recv_ready_ntb[i] == NULL) {
      ncm_interface.recv_ready_ntb[i] = ready_ntb;
      return;
    }
  }
  TU_LOG_DRV("(EE) recv_put_ntb_into_ready_list: ready list full\n");// this should not happen
} // recv_put_ntb_into_ready_list

/**
 * If possible, start a new reception TinyUSB -> driver.
 */
static void recv_try_to_start_new_reception(uint8_t rhport) {
  TU_LOG_DRV("recv_try_to_start_new_reception(%d)\n", rhport);

  if (ncm_interface.itf_data_alt != 1) {
    return;
  }
  if (ncm_interface.recv_tinyusb_ntb != NULL) {
    return;
  }
  if (usbd_edpt_busy(rhport, ncm_interface.ep_out)) {
    return;
  }

  ncm_interface.recv_tinyusb_ntb = recv_get_free_ntb();
  if (ncm_interface.recv_tinyusb_ntb == NULL) {
    return;
  }

  // initiate transfer
  TU_LOG_DRV("  start reception\n");
  bool r = usbd_edpt_xfer(rhport, ncm_interface.ep_out, ncm_interface.recv_tinyusb_ntb->data, CFG_TUD_NCM_OUT_NTB_MAX_SIZE);
  if (!r) {
    recv_put_ntb_into_free_list(ncm_interface.recv_tinyusb_ntb);
    ncm_interface.recv_tinyusb_ntb = NULL;
  }
} // recv_try_to_start_new_reception

/**
 * Validate incoming datagram.
 * \return true if valid
 *
 * \note
 *    \a ndp16->wNextNdpIndex != 0 is not supported
 */
static bool recv_validate_datagram(const recv_ntb_t *ntb, uint32_t len) {
  const nth16_t *nth16 = &(ntb->nth);

  TU_LOG_DRV("recv_validate_datagram(%p, %d)\n", ntb, (int) len);

  // check header
  if (nth16->wHeaderLength != sizeof(nth16_t)) {
    TU_LOG_DRV("(EE) ill nth16 length: %d\n", nth16->wHeaderLength);
    return false;
  }
  if (nth16->dwSignature != NTH16_SIGNATURE) {
    TU_LOG_DRV("(EE) ill signature: 0x%08x\n", (unsigned) nth16->dwSignature);
    return false;
  }
  if (len < sizeof(nth16_t) + sizeof(ndp16_t) + 2 * sizeof(ndp16_datagram_t)) {
    TU_LOG_DRV("(EE) ill min len: %lu\n", len);
    return false;
  }
  if (nth16->wBlockLength > len) {
    TU_LOG_DRV("(EE) ill block length: %d > %lu\n", nth16->wBlockLength, len);
    return false;
  }
  if (nth16->wBlockLength > CFG_TUD_NCM_OUT_NTB_MAX_SIZE) {
    TU_LOG_DRV("(EE) ill block length2: %d > %d\n", nth16->wBlockLength, CFG_TUD_NCM_OUT_NTB_MAX_SIZE);
    return false;
  }
  if (nth16->wNdpIndex < sizeof(nth16) || nth16->wNdpIndex > len - (sizeof(ndp16_t) + 2 * sizeof(ndp16_datagram_t))) {
    TU_LOG_DRV("(EE) ill position of first ndp: %d (%lu)\n", nth16->wNdpIndex, len);
    return false;
  }

  // check (first) NDP(16)
  const ndp16_t *ndp16 = (const ndp16_t *) (ntb->data + nth16->wNdpIndex);

  if (ndp16->wLength < sizeof(ndp16_t) + 2 * sizeof(ndp16_datagram_t)) {
    TU_LOG_DRV("(EE) ill ndp16 length: %d\n", ndp16->wLength);
    return false;
  }
  if (ndp16->dwSignature != NDP16_SIGNATURE_NCM0 && ndp16->dwSignature != NDP16_SIGNATURE_NCM1) {
    TU_LOG_DRV("(EE) ill signature: 0x%08x\n", (unsigned) ndp16->dwSignature);
    return false;
  }
  if (ndp16->wNextNdpIndex != 0) {
    TU_LOG_DRV("(EE) cannot handle wNextNdpIndex!=0 (%d)\n", ndp16->wNextNdpIndex);
    return false;
  }

  const ndp16_datagram_t *ndp16_datagram = (const ndp16_datagram_t *) (ntb->data + nth16->wNdpIndex + sizeof(ndp16_t));
  int ndx = 0;
  uint16_t max_ndx = (uint16_t) ((ndp16->wLength - sizeof(ndp16_t)) / sizeof(ndp16_datagram_t));

  if (max_ndx > 2) { // number of datagrams in NTB > 1
    TU_LOG_DRV("<< %d (%d)\n", max_ndx - 1, ntb->nth.wBlockLength);
  }
  if (ndp16_datagram[max_ndx - 1].wDatagramIndex != 0 || ndp16_datagram[max_ndx - 1].wDatagramLength != 0) {
    TU_LOG_DRV("  max_ndx != 0\n");
    return false;
  }
  while (ndp16_datagram[ndx].wDatagramIndex != 0 && ndp16_datagram[ndx].wDatagramLength != 0) {
    TU_LOG_DRV("  << %d %d\n", ndp16_datagram[ndx].wDatagramIndex, ndp16_datagram[ndx].wDatagramLength);
    if (ndp16_datagram[ndx].wDatagramIndex > len) {
      TU_LOG_DRV("(EE) ill start of datagram[%d]: %d (%lu)\n", ndx, ndp16_datagram[ndx].wDatagramIndex, len);
      return false;
    }
    if (ndp16_datagram[ndx].wDatagramIndex + ndp16_datagram[ndx].wDatagramLength > len) {
      TU_LOG_DRV("(EE) ill end of datagram[%d]: %d (%lu)\n", ndx, ndp16_datagram[ndx].wDatagramIndex + ndp16_datagram[ndx].wDatagramLength, len);
      return false;
    }
    ++ndx;
  }

  #if CFG_TUD_NCM_LOG_LEVEL >= 3
  TU_LOG_BUF(3, ntb->data[i], len);
  #endif

  // -> ntb contains a valid packet structure
  //    ok... I did not check for garbage within the datagram indices...
  return true;
} // recv_validate_datagram

/**
 * Transfer the next (pending) datagram to the glue logic and return receive buffer if empty.
 */
static void recv_transfer_datagram_to_glue_logic(void) {
  TU_LOG_DRV("recv_transfer_datagram_to_glue_logic()\n");

  if (ncm_interface.recv_glue_ntb == NULL) {
    ncm_interface.recv_glue_ntb = recv_get_next_ready_ntb();
    TU_LOG_DRV("  new buffer for glue logic: %p\n", ncm_interface.recv_glue_ntb);
    ncm_interface.recv_glue_ntb_datagram_ndx = 0;
  }

  if (ncm_interface.recv_glue_ntb != NULL) {
    const ndp16_datagram_t *ndp16_datagram = (ndp16_datagram_t *) (ncm_interface.recv_glue_ntb->data + ncm_interface.recv_glue_ntb->nth.wNdpIndex + sizeof(ndp16_t));

    if (ndp16_datagram[ncm_interface.recv_glue_ntb_datagram_ndx].wDatagramIndex == 0) {
      TU_LOG_DRV("(EE) SOMETHING WENT WRONG 1\n");
    } else if (ndp16_datagram[ncm_interface.recv_glue_ntb_datagram_ndx].wDatagramLength == 0) {
      TU_LOG_DRV("(EE) SOMETHING WENT WRONG 2\n");
    } else {
      uint16_t datagramIndex = ndp16_datagram[ncm_interface.recv_glue_ntb_datagram_ndx].wDatagramIndex;
      uint16_t datagramLength = ndp16_datagram[ncm_interface.recv_glue_ntb_datagram_ndx].wDatagramLength;

      TU_LOG_DRV("  recv[%d] - %d %d\n", ncm_interface.recv_glue_ntb_datagram_ndx, datagramIndex, datagramLength);
      if (tud_network_recv_cb(ncm_interface.recv_glue_ntb->data + datagramIndex, datagramLength)) {
        // send datagram successfully to glue logic
        TU_LOG_DRV("    OK\n");
        datagramIndex = ndp16_datagram[ncm_interface.recv_glue_ntb_datagram_ndx + 1].wDatagramIndex;
        datagramLength = ndp16_datagram[ncm_interface.recv_glue_ntb_datagram_ndx + 1].wDatagramLength;

        if (datagramIndex != 0 && datagramLength != 0) {
          // -> next datagram
          ++ncm_interface.recv_glue_ntb_datagram_ndx;
        } else {
          // end of datagrams reached
          recv_put_ntb_into_free_list(ncm_interface.recv_glue_ntb);
          ncm_interface.recv_glue_ntb = NULL;
        }
      }
    }
  }
} // recv_transfer_datagram_to_glue_logic

//-----------------------------------------------------------------------------
//
// all the tud_network_*() stuff (glue logic -> driver)
//

/**
 * Check if the glue logic is allowed to call tud_network_xmit().
 * This function also fetches a next buffer if required, so that tud_network_xmit() is ready for copy
 * and transmission operation.
 */
bool tud_network_can_xmit(uint16_t size) {
  TU_LOG_DRV("tud_network_can_xmit(%d)\n", size);

  TU_ASSERT(size <= CFG_TUD_NCM_OUT_NTB_MAX_SIZE - (sizeof(nth16_t) + sizeof(ndp16_t) + 2 * sizeof(ndp16_datagram_t)), false);

  if (xmit_requested_datagram_fits_into_current_ntb(size) || xmit_setup_next_glue_ntb()) {
    // -> everything is fine
    return true;
  }
  xmit_start_if_possible(ncm_interface.rhport);
  TU_LOG_DRV("(II) tud_network_can_xmit: request blocked\n");// could happen if all xmit buffers are full (but should happen rarely)
  return false;
} // tud_network_can_xmit

/**
 * Put a datagram into a waiting NTB.
 * If currently no transmission is started, then initiate transmission.
 */
void tud_network_xmit(void *ref, uint16_t arg) {
  TU_LOG_DRV("tud_network_xmit(%p, %d)\n", ref, arg);

  if (ncm_interface.xmit_glue_ntb == NULL) {
    TU_LOG_DRV("(EE) tud_network_xmit: no buffer\n");// must not happen (really)
    return;
  }

  xmit_ntb_t *ntb = ncm_interface.xmit_glue_ntb;

  // copy new datagram to the end of the current NTB
  uint16_t size = tud_network_xmit_cb(ntb->data + ntb->nth.wBlockLength, ref, arg);

  // correct NTB internals
  ntb->ndp_datagram[ncm_interface.xmit_glue_ntb_datagram_ndx].wDatagramIndex = ntb->nth.wBlockLength;
  ntb->ndp_datagram[ncm_interface.xmit_glue_ntb_datagram_ndx].wDatagramLength = size;
  ncm_interface.xmit_glue_ntb_datagram_ndx += 1;

  ntb->nth.wBlockLength += (uint16_t) (size + XMIT_ALIGN_OFFSET(size));

  if (ntb->nth.wBlockLength > CFG_TUD_NCM_OUT_NTB_MAX_SIZE) {
    TU_LOG_DRV("(EE) tud_network_xmit: buffer overflow\n"); // must not happen (really)
    return;
  }

  xmit_start_if_possible(ncm_interface.rhport);
} // tud_network_xmit

/**
 * Keep the receive logic busy and transfer pending packets to the glue logic.
 * Avoid recursive calls due to wrong expectations of the net glue logic,
 * see https://github.com/hathach/tinyusb/issues/2711
 */
void tud_network_recv_renew(void) {
  TU_LOG_DRV("tud_network_recv_renew()\n");

  ncm_interface.tud_network_recv_renew_process_again = true;

  if (ncm_interface.tud_network_recv_renew_active) {
    TU_LOG_DRV("Re-entrant into tud_network_recv_renew, will process later\n");
    return;
  }

  while (ncm_interface.tud_network_recv_renew_process_again) {
    ncm_interface.tud_network_recv_renew_process_again = false;

    // If the current function is called within recv_transfer_datagram_to_glue_logic,
    // tud_network_recv_renew_process_again will become true, and the loop will run again
    // Otherwise the loop will not run again
    ncm_interface.tud_network_recv_renew_active = true;
    recv_transfer_datagram_to_glue_logic();
    ncm_interface.tud_network_recv_renew_active = false;
  }
  recv_try_to_start_new_reception(ncm_interface.rhport);
} // tud_network_recv_renew

/**
 * Same as tud_network_recv_renew() but knows \a rhport
 */
static void tud_network_recv_renew_r(uint8_t rhport) {
  TU_LOG_DRV("tud_network_recv_renew_r(%d)\n", rhport);

  ncm_interface.rhport = rhport;
  tud_network_recv_renew();
} // tud_network_recv_renew

//-----------------------------------------------------------------------------
//
// all the netd_*() stuff (interface TinyUSB -> driver)
//
/**
 * Initialize the driver data structures.
 * Might be called several times.
 */
void netd_init(void) {
  TU_LOG_DRV("netd_init()\n");

  memset(&ncm_interface, 0, sizeof(ncm_interface));

  for (int i = 0; i < XMIT_NTB_N; ++i) {
    ncm_interface.xmit_free_ntb[i] = &ncm_epbuf.xmit[i].ntb;
  }
  for (int i = 0; i < RECV_NTB_N; ++i) {
    ncm_interface.recv_free_ntb[i] = &ncm_epbuf.recv[i].ntb;
  }
} // netd_init

/**
 * Deinit driver
 */
bool netd_deinit(void) {
  return true;
}

/**
 * Resets the port.
 * In this driver this is the same as netd_init()
 */
void netd_reset(uint8_t rhport) {
  (void) rhport;

  netd_init();
} // netd_reset

/**
 * Open the USB interface.
 * - parse the USB descriptor \a TUD_CDC_NCM_DESCRIPTOR for itfnum and endpoints
 * - a specific order of elements in the descriptor is tested.
 *
 * \note
 *   Actually all of the information could be read directly from \a itf_desc, because the
 *   structure and the values are well known.  But we do it this way.
 *
 * \post
 * - \a itf_num set
 * - \a ep_notif, \a ep_in and \a ep_out are set
 * - USB interface is open
 */
uint16_t netd_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
  TU_ASSERT(ncm_interface.ep_notif == 0, 0);// assure that the interface is only opened once

  ncm_interface.itf_num = itf_desc->bInterfaceNumber;// management interface

  // skip the two first entries and the following TUSB_DESC_CS_INTERFACE entries
  uint16_t drv_len = sizeof(tusb_desc_interface_t);
  uint8_t const *p_desc = tu_desc_next(itf_desc);
  while (tu_desc_type(p_desc) == TUSB_DESC_CS_INTERFACE && drv_len <= max_len) {
    drv_len += tu_desc_len(p_desc);
    p_desc = tu_desc_next(p_desc);
  }

  // get notification endpoint
  TU_ASSERT(tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT, 0);
  TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *) p_desc), 0);
  ncm_interface.ep_notif = ((tusb_desc_endpoint_t const *) p_desc)->bEndpointAddress;
  drv_len += tu_desc_len(p_desc);
  p_desc = tu_desc_next(p_desc);

  // skip the following TUSB_DESC_INTERFACE entries (which must be TUSB_CLASS_CDC_DATA)
  while (tu_desc_type(p_desc) == TUSB_DESC_INTERFACE && drv_len <= max_len) {
    tusb_desc_interface_t const *data_itf_desc = (tusb_desc_interface_t const *) p_desc;
    TU_ASSERT(data_itf_desc->bInterfaceClass == TUSB_CLASS_CDC_DATA, 0);

    drv_len += tu_desc_len(p_desc);
    p_desc = tu_desc_next(p_desc);
  }

  // a TUSB_DESC_ENDPOINT (actually two) must follow, open these endpoints
  TU_ASSERT(tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT, 0);
  TU_ASSERT(usbd_open_edpt_pair(rhport, p_desc, 2, TUSB_XFER_BULK, &ncm_interface.ep_out, &ncm_interface.ep_in));
  drv_len += 2 * sizeof(tusb_desc_endpoint_t);

  return drv_len;
} // netd_open

/**
 * Handle TinyUSB requests to process transfer events.
 */
bool netd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void) result;

  if (ep_addr == ncm_interface.ep_out) {
    // new NTB received
    // - make the NTB valid
    // - if ready transfer datagrams to the glue logic for further processing
    // - if there is a free receive buffer, initiate reception
    if (!recv_validate_datagram(ncm_interface.recv_tinyusb_ntb, xferred_bytes)) {
      // verification failed: ignore NTB and return it to free
      TU_LOG_DRV("(EE) VALIDATION FAILED. WHAT CAN WE DO IN THIS CASE?\n");
    } else {
      // packet ok -> put it into ready list
      recv_put_ntb_into_ready_list(ncm_interface.recv_tinyusb_ntb);
    }
    ncm_interface.recv_tinyusb_ntb = NULL;
    tud_network_recv_renew_r(rhport);
  } else if (ep_addr == ncm_interface.ep_in) {
    // transmission of an NTB finished
    // - free the transmitted NTB buffer
    // - insert ZLPs when necessary
    // - if there is another transmit NTB waiting, try to start transmission
    xmit_put_ntb_into_free_list(ncm_interface.xmit_tinyusb_ntb);
    ncm_interface.xmit_tinyusb_ntb = NULL;
    if (!xmit_insert_required_zlp(rhport, xferred_bytes)) {
      xmit_start_if_possible(rhport);
    }
  } else if (ep_addr == ncm_interface.ep_notif) {
    // next transfer on notification channel
    notification_xmit(rhport, true);
  }

  return true;
} // netd_xfer_cb

/**
 * Respond to TinyUSB control requests.
 * At startup transmission of notification packets are done here.
 */
bool netd_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  if (stage != CONTROL_STAGE_SETUP) {
    return true;
  }

  switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_STANDARD:

      switch (request->bRequest) {
        case TUSB_REQ_GET_INTERFACE: {
          TU_VERIFY(ncm_interface.itf_num + 1 == request->wIndex, false);

          tud_control_xfer(rhport, request, &ncm_interface.itf_data_alt, 1);
        } break;

        case TUSB_REQ_SET_INTERFACE: {
          TU_VERIFY(ncm_interface.itf_num + 1 == request->wIndex && request->wValue < 2, false);

          ncm_interface.itf_data_alt = (uint8_t) request->wValue;

          if (ncm_interface.itf_data_alt == 1) {
            tud_network_recv_renew_r(rhport);
            notification_xmit(rhport, false);
          }
          tud_control_status(rhport, request);
        } break;

        // unsupported request
        default:
          return false;
      }
      break;

    case TUSB_REQ_TYPE_CLASS:
      TU_VERIFY(ncm_interface.itf_num == request->wIndex, false);
      switch (request->bRequest) {
        case NCM_GET_NTB_PARAMETERS: {
          // transfer NTB parameters to host.
          tud_control_xfer(rhport, request, (void *) (uintptr_t) &ntb_parameters, sizeof(ntb_parameters));
        } break;

          // unsupported request
        default:
          return false;
      }
      break;
      // unsupported request
    default:
      return false;
  }

  return true;
} // netd_control_xfer_cb

#endif // ( CFG_TUD_ENABLED && CFG_TUD_NCM )
