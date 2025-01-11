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

#if (CFG_TUH_ENABLED && CFG_TUH_HID)

#include "host/usbh.h"
#include "host/usbh_pvt.h"

#include "hid_host.h"

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUH_HID_LOG_LEVEL
  #define CFG_TUH_HID_LOG_LEVEL   CFG_TUH_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUH_HID_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct {
  uint8_t daddr;
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;

  bool mounted;           // Enumeration is complete
  uint8_t itf_protocol;   // None, Keyboard, Mouse
  uint8_t protocol_mode;  // Boot (0) or Report protocol (1)

  uint8_t report_desc_type;
  uint16_t report_desc_len;

  uint16_t epin_size;
  uint16_t epout_size;
} hidh_interface_t;

typedef struct {
  TUH_EPBUF_DEF(epin, CFG_TUH_HID_EPIN_BUFSIZE);
  TUH_EPBUF_DEF(epout, CFG_TUH_HID_EPOUT_BUFSIZE);
} hidh_epbuf_t;

static hidh_interface_t _hidh_itf[CFG_TUH_HID];
CFG_TUH_MEM_SECTION static hidh_epbuf_t _hidh_epbuf[CFG_TUH_HID];

static uint8_t _hidh_default_protocol = HID_PROTOCOL_BOOT;

//--------------------------------------------------------------------+
// Helper
//--------------------------------------------------------------------+
TU_ATTR_ALWAYS_INLINE static inline hidh_interface_t* get_hid_itf(uint8_t daddr, uint8_t idx) {
  TU_ASSERT(daddr > 0 && idx < CFG_TUH_HID, NULL);
  hidh_interface_t* p_hid = &_hidh_itf[idx];
  return (p_hid->daddr == daddr) ? p_hid : NULL;
}

TU_ATTR_ALWAYS_INLINE static inline hidh_epbuf_t* get_hid_epbuf(uint8_t idx) {
  return &_hidh_epbuf[idx];
}

// Get instance ID by endpoint address
static uint8_t get_idx_by_epaddr(uint8_t daddr, uint8_t ep_addr) {
  for (uint8_t idx = 0; idx < CFG_TUH_HID; idx++) {
    hidh_interface_t const* p_hid = &_hidh_itf[idx];
    if (p_hid->daddr == daddr &&
        (p_hid->ep_in == ep_addr || p_hid->ep_out == ep_addr)) {
      return idx;
    }
  }
  return TUSB_INDEX_INVALID_8;
}

static hidh_interface_t* find_new_itf(void) {
  for (uint8_t i = 0; i < CFG_TUH_HID; i++) {
    if (_hidh_itf[i].daddr == 0) return &_hidh_itf[i];
  }
  return NULL;
}

//--------------------------------------------------------------------+
// Interface API
//--------------------------------------------------------------------+
uint8_t tuh_hid_itf_get_count(uint8_t daddr) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < CFG_TUH_HID; i++) {
    if (_hidh_itf[i].daddr == daddr) count++;
  }
  return count;
}

uint8_t tuh_hid_itf_get_total_count(void) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < CFG_TUH_HID; i++) {
    if (_hidh_itf[i].daddr != 0) count++;
  }
  return count;
}

bool tuh_hid_mounted(uint8_t daddr, uint8_t idx) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid);
  return p_hid->mounted;
}

bool tuh_hid_itf_get_info(uint8_t daddr, uint8_t idx, tuh_itf_info_t* info) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid && info);

  info->daddr = daddr;

  // re-construct descriptor
  tusb_desc_interface_t* desc = &info->desc;
  desc->bLength = sizeof(tusb_desc_interface_t);
  desc->bDescriptorType = TUSB_DESC_INTERFACE;

  desc->bInterfaceNumber = p_hid->itf_num;
  desc->bAlternateSetting = 0;
  desc->bNumEndpoints = (uint8_t) ((p_hid->ep_in ? 1u : 0u) + (p_hid->ep_out ? 1u : 0u));
  desc->bInterfaceClass = TUSB_CLASS_HID;
  desc->bInterfaceSubClass = (p_hid->itf_protocol ? HID_SUBCLASS_BOOT : HID_SUBCLASS_NONE);
  desc->bInterfaceProtocol = p_hid->itf_protocol;
  desc->iInterface = 0; // not used yet

  return true;
}

uint8_t tuh_hid_itf_get_index(uint8_t daddr, uint8_t itf_num) {
  for (uint8_t idx = 0; idx < CFG_TUH_HID; idx++) {
    hidh_interface_t const* p_hid = &_hidh_itf[idx];
    if (p_hid->daddr == daddr && p_hid->itf_num == itf_num) return idx;
  }

  return TUSB_INDEX_INVALID_8;
}

uint8_t tuh_hid_interface_protocol(uint8_t daddr, uint8_t idx) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  return p_hid ? p_hid->itf_protocol : 0;
}

//--------------------------------------------------------------------+
// Control Endpoint API
//--------------------------------------------------------------------+
uint8_t tuh_hid_get_protocol(uint8_t daddr, uint8_t idx) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  return p_hid ? p_hid->protocol_mode : 0;
}

static void set_protocol_complete(tuh_xfer_t* xfer) {
  uint8_t const itf_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);
  uint8_t const daddr = xfer->daddr;
  uint8_t const idx = tuh_hid_itf_get_index(daddr, itf_num);

  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid,);

  if (XFER_RESULT_SUCCESS == xfer->result) {
    p_hid->protocol_mode = (uint8_t) tu_le16toh(xfer->setup->wValue);
  }

  if (tuh_hid_set_protocol_complete_cb) {
    tuh_hid_set_protocol_complete_cb(daddr, idx, p_hid->protocol_mode);
  }
}

void tuh_hid_set_default_protocol(uint8_t protocol) {
  _hidh_default_protocol = protocol;
}

static bool _hidh_set_protocol(uint8_t daddr, uint8_t itf_num, uint8_t protocol,
                               tuh_xfer_cb_t complete_cb, uintptr_t user_data) {
  TU_LOG_DRV("HID Set Protocol = %d\r\n", protocol);

  tusb_control_request_t const request = {
      .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type      = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_OUT
      },
      .bRequest = HID_REQ_CONTROL_SET_PROTOCOL,
      .wValue   = protocol,
      .wIndex   = itf_num,
      .wLength  = 0
  };

  tuh_xfer_t xfer = {
      .daddr       = daddr,
      .ep_addr     = 0,
      .setup       = &request,
      .buffer      = NULL,
      .complete_cb = complete_cb,
      .user_data   = user_data
  };

  return tuh_control_xfer(&xfer);
}

bool tuh_hid_set_protocol(uint8_t daddr, uint8_t idx, uint8_t protocol) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid && p_hid->itf_protocol != HID_ITF_PROTOCOL_NONE);

  return _hidh_set_protocol(daddr, p_hid->itf_num, protocol, set_protocol_complete, 0);
}

static void get_report_complete(tuh_xfer_t* xfer) {
  TU_LOG_DRV("HID Get Report complete\r\n");

  if (tuh_hid_get_report_complete_cb) {
    uint8_t const itf_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);
    uint8_t const idx = tuh_hid_itf_get_index(xfer->daddr, itf_num);

    uint8_t const report_type = tu_u16_high(xfer->setup->wValue);
    uint8_t const report_id = tu_u16_low(xfer->setup->wValue);

    tuh_hid_get_report_complete_cb(xfer->daddr, idx, report_id, report_type,
                                   (xfer->result == XFER_RESULT_SUCCESS) ? xfer->setup->wLength : 0);
  }
}

bool tuh_hid_get_report(uint8_t daddr, uint8_t idx, uint8_t report_id, uint8_t report_type, void* report, uint16_t len) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid);
  TU_LOG_DRV("HID Get Report: id = %u, type = %u, len = %u\r\n", report_id, report_type, len);

  tusb_control_request_t const request = {
      .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type      = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_IN
      },
      .bRequest = HID_REQ_CONTROL_GET_REPORT,
      .wValue   = tu_htole16(tu_u16(report_type, report_id)),
      .wIndex   = tu_htole16((uint16_t) p_hid->itf_num),
      .wLength  = len
  };

  tuh_xfer_t xfer = {
      .daddr       = daddr,
      .ep_addr     = 0,
      .setup       = &request,
      .buffer      = report,
      .complete_cb = get_report_complete,
      .user_data   = 0
  };

  return tuh_control_xfer(&xfer);
}

static void set_report_complete(tuh_xfer_t* xfer) {
  TU_LOG_DRV("HID Set Report complete\r\n");

  if (tuh_hid_set_report_complete_cb) {
    uint8_t const itf_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);
    uint8_t const idx = tuh_hid_itf_get_index(xfer->daddr, itf_num);

    uint8_t const report_type = tu_u16_high(xfer->setup->wValue);
    uint8_t const report_id = tu_u16_low(xfer->setup->wValue);

    tuh_hid_set_report_complete_cb(xfer->daddr, idx, report_id, report_type,
                                   (xfer->result == XFER_RESULT_SUCCESS) ? xfer->setup->wLength : 0);
  }
}

bool tuh_hid_set_report(uint8_t daddr, uint8_t idx, uint8_t report_id, uint8_t report_type, void* report, uint16_t len) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid);
  TU_LOG_DRV("HID Set Report: id = %u, type = %u, len = %u\r\n", report_id, report_type, len);

  tusb_control_request_t const request = {
      .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type      = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_OUT
      },
      .bRequest = HID_REQ_CONTROL_SET_REPORT,
      .wValue   = tu_htole16(tu_u16(report_type, report_id)),
      .wIndex   = tu_htole16((uint16_t) p_hid->itf_num),
      .wLength  = len
  };

  tuh_xfer_t xfer = {
      .daddr       = daddr,
      .ep_addr     = 0,
      .setup       = &request,
      .buffer      = report,
      .complete_cb = set_report_complete,
      .user_data   = 0
  };

  return tuh_control_xfer(&xfer);
}

static bool _hidh_set_idle(uint8_t daddr, uint8_t itf_num, uint16_t idle_rate,
                           tuh_xfer_cb_t complete_cb, uintptr_t user_data) {
  // SET IDLE request, device can stall if not support this request
  TU_LOG_DRV("HID Set Idle \r\n");

  tusb_control_request_t const request = {
      .bmRequestType_bit = {
          .recipient = TUSB_REQ_RCPT_INTERFACE,
          .type      = TUSB_REQ_TYPE_CLASS,
          .direction = TUSB_DIR_OUT
      },
      .bRequest = HID_REQ_CONTROL_SET_IDLE,
      .wValue   = tu_htole16(idle_rate),
      .wIndex   = tu_htole16((uint16_t) itf_num),
      .wLength  = 0
  };

  tuh_xfer_t xfer = {
      .daddr       = daddr,
      .ep_addr     = 0,
      .setup       = &request,
      .buffer      = NULL,
      .complete_cb = complete_cb,
      .user_data   = user_data
  };

  return tuh_control_xfer(&xfer);
}

//--------------------------------------------------------------------+
// Interrupt Endpoint API
//--------------------------------------------------------------------+

// Check if HID interface is ready to receive report
bool tuh_hid_receive_ready(uint8_t dev_addr, uint8_t idx) {
  hidh_interface_t* p_hid = get_hid_itf(dev_addr, idx);
  TU_VERIFY(p_hid);
  return !usbh_edpt_busy(dev_addr, p_hid->ep_in);
}

bool tuh_hid_receive_report(uint8_t daddr, uint8_t idx) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid);
  hidh_epbuf_t* epbuf = get_hid_epbuf(idx);

  // claim endpoint
  TU_VERIFY(usbh_edpt_claim(daddr, p_hid->ep_in));

  if (!usbh_edpt_xfer(daddr, p_hid->ep_in, epbuf->epin, p_hid->epin_size)) {
    usbh_edpt_release(daddr, p_hid->ep_in);
    return false;
  }

  return true;
}
bool tuh_hid_receive_abort(uint8_t dev_addr, uint8_t idx) {
  hidh_interface_t* p_hid = get_hid_itf(dev_addr, idx);
  TU_VERIFY(p_hid);
  return tuh_edpt_abort_xfer(dev_addr, p_hid->ep_in);
}

bool tuh_hid_send_ready(uint8_t dev_addr, uint8_t idx) {
  hidh_interface_t* p_hid = get_hid_itf(dev_addr, idx);
  TU_VERIFY(p_hid);
  return !usbh_edpt_busy(dev_addr, p_hid->ep_out);
}

bool tuh_hid_send_report(uint8_t daddr, uint8_t idx, uint8_t report_id, const void* report, uint16_t len) {
  TU_LOG_DRV("HID Send Report %d\r\n", report_id);

  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid);
  hidh_epbuf_t* epbuf = get_hid_epbuf(idx);

  if (p_hid->ep_out == 0) {
    // This HID does not have an out endpoint (other than control)
    return false;
  } else if (len > CFG_TUH_HID_EPOUT_BUFSIZE ||
             (report_id != 0 && len > (CFG_TUH_HID_EPOUT_BUFSIZE - 1))) {
    // ep_out buffer is not large enough to hold contents
    return false;
  }

  // claim endpoint
  TU_VERIFY(usbh_edpt_claim(daddr, p_hid->ep_out));

  if (report_id == 0) {
    // No report ID in transmission
    memcpy(&epbuf->epout[0], report, len);
  } else {
    epbuf->epout[0] = report_id;
    memcpy(&epbuf->epout[1], report, len);
    ++len; // 1 more byte for report_id
  }

  TU_LOG3_MEM(p_hid->epout_buf, len, 2);

  if (!usbh_edpt_xfer(daddr, p_hid->ep_out, epbuf->epout, len)) {
    usbh_edpt_release(daddr, p_hid->ep_out);
    return false;
  }

  return true;
}

//--------------------------------------------------------------------+
// USBH API
//--------------------------------------------------------------------+
bool hidh_init(void) {
  TU_LOG_DRV("sizeof(hidh_interface_t) = %u\r\n", sizeof(hidh_interface_t));
  tu_memclr(_hidh_itf, sizeof(_hidh_itf));
  return true;
}

bool hidh_deinit(void) {
  return true;
}

bool hidh_xfer_cb(uint8_t daddr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void) result;

  uint8_t const dir = tu_edpt_dir(ep_addr);
  uint8_t const idx = get_idx_by_epaddr(daddr, ep_addr);

  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid);
  hidh_epbuf_t* epbuf = get_hid_epbuf(idx);

  if (dir == TUSB_DIR_IN) {
    TU_LOG_DRV("  Get Report callback (%u, %u)\r\n", daddr, idx);
    TU_LOG3_MEM(p_hid->epin_buf, xferred_bytes, 2);
    tuh_hid_report_received_cb(daddr, idx, epbuf->epin, (uint16_t) xferred_bytes);
  } else {
    if (tuh_hid_report_sent_cb) {
      tuh_hid_report_sent_cb(daddr, idx, epbuf->epout, (uint16_t) xferred_bytes);
    }
  }

  return true;
}

void hidh_close(uint8_t daddr) {
  for (uint8_t i = 0; i < CFG_TUH_HID; i++) {
    hidh_interface_t* p_hid = &_hidh_itf[i];
    if (p_hid->daddr == daddr) {
      TU_LOG_DRV("  HIDh close addr = %u index = %u\r\n", daddr, i);
      if (tuh_hid_umount_cb) tuh_hid_umount_cb(daddr, i);
      tu_memclr(p_hid, sizeof(hidh_interface_t));
    }
  }
}

//--------------------------------------------------------------------+
// Enumeration
//--------------------------------------------------------------------+

bool hidh_open(uint8_t rhport, uint8_t daddr, tusb_desc_interface_t const* desc_itf, uint16_t max_len) {
  (void) rhport;
  (void) max_len;

  TU_VERIFY(TUSB_CLASS_HID == desc_itf->bInterfaceClass);
  TU_LOG_DRV("[%u] HID opening Interface %u\r\n", daddr, desc_itf->bInterfaceNumber);

  // len = interface + hid + n*endpoints
  uint16_t const drv_len = (uint16_t) (sizeof(tusb_desc_interface_t) + sizeof(tusb_hid_descriptor_hid_t) +
                                       desc_itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
  TU_ASSERT(max_len >= drv_len);
  uint8_t const* p_desc = (uint8_t const*) desc_itf;

  //------------- HID descriptor -------------//
  p_desc = tu_desc_next(p_desc);
  tusb_hid_descriptor_hid_t const* desc_hid = (tusb_hid_descriptor_hid_t const*) p_desc;
  TU_ASSERT(HID_DESC_TYPE_HID == desc_hid->bDescriptorType);

  hidh_interface_t* p_hid = find_new_itf();
  TU_ASSERT(p_hid); // not enough interface, try to increase CFG_TUH_HID
  p_hid->daddr = daddr;

  //------------- Endpoint Descriptors -------------//
  p_desc = tu_desc_next(p_desc);
  tusb_desc_endpoint_t const* desc_ep = (tusb_desc_endpoint_t const*) p_desc;

  for (int i = 0; i < desc_itf->bNumEndpoints; i++) {
    TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType);
    TU_ASSERT(tuh_edpt_open(daddr, desc_ep));

    if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
      p_hid->ep_in = desc_ep->bEndpointAddress;
      p_hid->epin_size = tu_edpt_packet_size(desc_ep);
    } else {
      p_hid->ep_out = desc_ep->bEndpointAddress;
      p_hid->epout_size = tu_edpt_packet_size(desc_ep);
    }

    p_desc = tu_desc_next(p_desc);
    desc_ep = (tusb_desc_endpoint_t const*) p_desc;
  }

  p_hid->itf_num = desc_itf->bInterfaceNumber;

  // Assume bNumDescriptors = 1
  p_hid->report_desc_type = desc_hid->bReportType;
  p_hid->report_desc_len = tu_unaligned_read16(&desc_hid->wReportLength);

  // Per HID Specs: default is Report protocol, though we will force Boot protocol when set_config
  p_hid->protocol_mode = _hidh_default_protocol;
  if (HID_SUBCLASS_BOOT == desc_itf->bInterfaceSubClass) {
    p_hid->itf_protocol = desc_itf->bInterfaceProtocol;
  }

  return true;
}

//--------------------------------------------------------------------+
// Set Configure
//--------------------------------------------------------------------+

enum {
  CONFG_SET_IDLE,
  CONFIG_SET_PROTOCOL,
  CONFIG_GET_REPORT_DESC,
  CONFIG_COMPLETE
};

static void config_driver_mount_complete(uint8_t daddr, uint8_t idx, uint8_t const* desc_report, uint16_t desc_len);
static void process_set_config(tuh_xfer_t* xfer);

bool hidh_set_config(uint8_t daddr, uint8_t itf_num) {
  tusb_control_request_t request;
  request.wIndex = tu_htole16((uint16_t) itf_num);

  tuh_xfer_t xfer;
  xfer.daddr = daddr;
  xfer.result = XFER_RESULT_SUCCESS;
  xfer.setup = &request;
  xfer.user_data = CONFG_SET_IDLE;

  // fake request to kick-off the set config process
  process_set_config(&xfer);

  return true;
}

static void process_set_config(tuh_xfer_t* xfer) {
  // Stall is a valid response for SET_IDLE, sometime SET_PROTOCOL as well
  // therefore we could ignore its result
  if (!(xfer->setup->bRequest == HID_REQ_CONTROL_SET_IDLE ||
        xfer->setup->bRequest == HID_REQ_CONTROL_SET_PROTOCOL)) {
    TU_ASSERT(xfer->result == XFER_RESULT_SUCCESS,);
  }

  uintptr_t const state = xfer->user_data;
  uint8_t const itf_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);
  uint8_t const daddr = xfer->daddr;

  uint8_t const idx = tuh_hid_itf_get_index(daddr, itf_num);
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid,);

  switch (state) {
    case CONFG_SET_IDLE: {
      // Idle rate = 0 mean only report when there is changes
      const uint16_t idle_rate = 0;
      const uintptr_t next_state = (p_hid->itf_protocol != HID_ITF_PROTOCOL_NONE)
                                   ? CONFIG_SET_PROTOCOL : CONFIG_GET_REPORT_DESC;
      _hidh_set_idle(daddr, itf_num, idle_rate, process_set_config, next_state);
      break;
    }

    case CONFIG_SET_PROTOCOL:
      _hidh_set_protocol(daddr, p_hid->itf_num, _hidh_default_protocol, process_set_config, CONFIG_GET_REPORT_DESC);
      break;

    case CONFIG_GET_REPORT_DESC:
      // Get Report Descriptor if possible
      // using usbh enumeration buffer since report descriptor can be very long
      if (p_hid->report_desc_len > CFG_TUH_ENUMERATION_BUFSIZE) {
        TU_LOG_DRV("HID Skip Report Descriptor since it is too large %u bytes\r\n", p_hid->report_desc_len);

        // Driver is mounted without report descriptor
        config_driver_mount_complete(daddr, idx, NULL, 0);
      } else {
        tuh_descriptor_get_hid_report(daddr, itf_num, p_hid->report_desc_type, 0,
                                      usbh_get_enum_buf(), p_hid->report_desc_len,
                                      process_set_config, CONFIG_COMPLETE);
      }
      break;

    case CONFIG_COMPLETE: {
      uint8_t const* desc_report = usbh_get_enum_buf();
      uint16_t const desc_len = tu_le16toh(xfer->setup->wLength);

      config_driver_mount_complete(daddr, idx, desc_report, desc_len);
      break;
    }

    default:
      break;
  }
}

static void config_driver_mount_complete(uint8_t daddr, uint8_t idx, uint8_t const* desc_report, uint16_t desc_len) {
  hidh_interface_t* p_hid = get_hid_itf(daddr, idx);
  TU_VERIFY(p_hid,);
  p_hid->mounted = true;

  // enumeration is complete
  if (tuh_hid_mount_cb) tuh_hid_mount_cb(daddr, idx, desc_report, desc_len);

  // notify usbh that driver enumeration is complete
  usbh_driver_set_config_complete(daddr, p_hid->itf_num);
}

//--------------------------------------------------------------------+
// Report Descriptor Parser
//--------------------------------------------------------------------+

uint8_t tuh_hid_parse_report_descriptor(tuh_hid_report_info_t* report_info_arr, uint8_t arr_count,
                                        uint8_t const* desc_report, uint16_t desc_len) {
  // Report Item 6.2.2.2 USB HID 1.11
  union TU_ATTR_PACKED {
    uint8_t byte;
    struct TU_ATTR_PACKED {
      uint8_t size : 2;
      uint8_t type : 2;
      uint8_t tag : 4;
    };
  } header;

  tu_memclr(report_info_arr, arr_count * sizeof(tuh_hid_report_info_t));

  uint8_t report_num = 0;
  tuh_hid_report_info_t* info = report_info_arr;

  // current parsed report count & size from descriptor
//  uint8_t ri_report_count = 0;
//  uint8_t ri_report_size = 0;

  uint8_t ri_collection_depth = 0;
  while (desc_len && report_num < arr_count) {
    header.byte = *desc_report++;
    desc_len--;

    uint8_t const tag = header.tag;
    uint8_t const type = header.type;
    uint8_t const size = header.size;

    uint8_t const data8 = desc_report[0];

    TU_LOG(3, "tag = %d, type = %d, size = %d, data = ", tag, type, size);
    for (uint32_t i = 0; i < size; i++) {
      TU_LOG(3, "%02X ", desc_report[i]);
    }
    TU_LOG(3, "\r\n");

    switch (type) {
      case RI_TYPE_MAIN:
        switch (tag) {
          case RI_MAIN_INPUT: break;
          case RI_MAIN_OUTPUT: break;
          case RI_MAIN_FEATURE: break;
          case RI_MAIN_COLLECTION:
            ri_collection_depth++;
            break;

          case RI_MAIN_COLLECTION_END:
            ri_collection_depth--;
            if (ri_collection_depth == 0) {
              info++;
              report_num++;
            }
            break;

          default:break;
        }
        break;

      case RI_TYPE_GLOBAL:
        switch (tag) {
          case RI_GLOBAL_USAGE_PAGE:
            // only take in account the "usage page" before REPORT ID
            if (ri_collection_depth == 0) memcpy(&info->usage_page, desc_report, size);
            break;

          case RI_GLOBAL_LOGICAL_MIN: break;
          case RI_GLOBAL_LOGICAL_MAX: break;
          case RI_GLOBAL_PHYSICAL_MIN: break;
          case RI_GLOBAL_PHYSICAL_MAX: break;

          case RI_GLOBAL_REPORT_ID:
            info->report_id = data8;
            break;

          case RI_GLOBAL_REPORT_SIZE:
//            ri_report_size = data8;
            break;

          case RI_GLOBAL_REPORT_COUNT:
//            ri_report_count = data8;
            break;

          case RI_GLOBAL_UNIT_EXPONENT: break;
          case RI_GLOBAL_UNIT: break;
          case RI_GLOBAL_PUSH: break;
          case RI_GLOBAL_POP: break;

          default: break;
        }
        break;

      case RI_TYPE_LOCAL:
        switch (tag) {
          case RI_LOCAL_USAGE:
            // only take in account the "usage" before starting REPORT ID
            if (ri_collection_depth == 0) info->usage = data8;
            break;

          case RI_LOCAL_USAGE_MIN: break;
          case RI_LOCAL_USAGE_MAX: break;
          case RI_LOCAL_DESIGNATOR_INDEX: break;
          case RI_LOCAL_DESIGNATOR_MIN: break;
          case RI_LOCAL_DESIGNATOR_MAX: break;
          case RI_LOCAL_STRING_INDEX: break;
          case RI_LOCAL_STRING_MIN: break;
          case RI_LOCAL_STRING_MAX: break;
          case RI_LOCAL_DELIMITER: break;
          default: break;
        }
        break;

        // error
      default: break;
    }

    desc_report += size;
    desc_len -= size;
  }

  for (uint8_t i = 0; i < report_num; i++) {
    info = report_info_arr + i;
    TU_LOG_DRV("%u: id = %u, usage_page = %u, usage = %u\r\n", i, info->report_id, info->usage_page, info->usage);
  }

  return report_num;
}

#endif
