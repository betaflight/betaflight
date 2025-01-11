/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
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

#if (CFG_TUD_ENABLED && CFG_TUD_BTH)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "bth_device.h"
#include <device/usbd_pvt.h>

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct {
  uint8_t itf_num;
  uint8_t ep_ev;
  uint8_t ep_acl_in;
  uint16_t ep_acl_in_pkt_sz;
  uint8_t ep_acl_out;
  uint8_t ep_voice[2];  // Not used yet
  uint8_t ep_voice_size[2][CFG_TUD_BTH_ISO_ALT_COUNT];

  // Previous amount of bytes sent when issuing ZLP
  uint32_t prev_xferred_bytes;
} btd_interface_t;

typedef struct {
  TUD_EPBUF_DEF(epout_buf, CFG_TUD_BTH_DATA_EPSIZE);
  TUD_EPBUF_TYPE_DEF(bt_hci_cmd_t, hci_cmd);
} btd_epbuf_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static btd_interface_t _btd_itf;
CFG_TUD_MEM_SECTION static btd_epbuf_t _btd_epbuf;

static bool bt_tx_data(uint8_t ep, void *data, uint16_t len)
{
  uint8_t const rhport = 0;

  // skip if previous transfer not complete
  TU_VERIFY(!usbd_edpt_busy(rhport, ep));

  TU_ASSERT(usbd_edpt_xfer(rhport, ep, data, len));

  return true;
}

//--------------------------------------------------------------------+
// READ API
//--------------------------------------------------------------------+


//--------------------------------------------------------------------+
// WRITE API
//--------------------------------------------------------------------+

bool tud_bt_event_send(void *event, uint16_t event_len)
{
  return bt_tx_data(_btd_itf.ep_ev, event, event_len);
}

bool tud_bt_acl_data_send(void *event, uint16_t event_len)
{
  return bt_tx_data(_btd_itf.ep_acl_in, event, event_len);
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void btd_init(void) {
  tu_memclr(&_btd_itf, sizeof(_btd_itf));
}

bool btd_deinit(void) {
  return true;
}

void btd_reset(uint8_t rhport)
{
  (void)rhport;
}

uint16_t btd_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
  tusb_desc_endpoint_t const *desc_ep;
  uint16_t drv_len = 0;
  // Size of single alternative of ISO interface
  const uint16_t iso_alt_itf_size = sizeof(tusb_desc_interface_t) + 2 * sizeof(tusb_desc_endpoint_t);
  // Size of hci interface
  const uint16_t hci_itf_size = sizeof(tusb_desc_interface_t) + 3 * sizeof(tusb_desc_endpoint_t);
  // Ensure this is BT Primary Controller
  TU_VERIFY(TUSB_CLASS_WIRELESS_CONTROLLER == itf_desc->bInterfaceClass &&
            TUD_BT_APP_SUBCLASS == itf_desc->bInterfaceSubClass &&
            TUD_BT_PROTOCOL_PRIMARY_CONTROLLER == itf_desc->bInterfaceProtocol, 0);

  TU_ASSERT(itf_desc->bNumEndpoints == 3 && max_len >= hci_itf_size);

  _btd_itf.itf_num = itf_desc->bInterfaceNumber;

  desc_ep = (tusb_desc_endpoint_t const *) tu_desc_next(itf_desc);

  TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType && TUSB_XFER_INTERRUPT == desc_ep->bmAttributes.xfer, 0);
  TU_ASSERT(usbd_edpt_open(rhport, desc_ep), 0);
  _btd_itf.ep_ev = desc_ep->bEndpointAddress;

  desc_ep = (tusb_desc_endpoint_t const *)tu_desc_next(desc_ep);

  // Open endpoint pair
  TU_ASSERT(usbd_open_edpt_pair(rhport, (uint8_t const *)desc_ep, 2,
                                TUSB_XFER_BULK, &_btd_itf.ep_acl_out,
                                &_btd_itf.ep_acl_in),
            0);

  // Save acl in endpoint max packet size
  tusb_desc_endpoint_t const *desc_ep_acl_in = desc_ep;
  for (size_t p = 0; p < 2; p++) {
    if (tu_edpt_dir(desc_ep_acl_in->bEndpointAddress) == TUSB_DIR_IN) {
      _btd_itf.ep_acl_in_pkt_sz = tu_edpt_packet_size(desc_ep_acl_in);
      break;
    }
    desc_ep_acl_in = (tusb_desc_endpoint_t const *)tu_desc_next(desc_ep_acl_in);
  }

  itf_desc = (tusb_desc_interface_t const *)tu_desc_next(tu_desc_next(desc_ep));

  // Prepare for incoming data from host
  TU_ASSERT(usbd_edpt_xfer(rhport, _btd_itf.ep_acl_out, _btd_epbuf.epout_buf, CFG_TUD_BTH_DATA_EPSIZE), 0);

  drv_len = hci_itf_size;

  // Ensure this is still BT Primary Controller
  TU_ASSERT(TUSB_CLASS_WIRELESS_CONTROLLER == itf_desc->bInterfaceClass &&
            TUD_BT_APP_SUBCLASS == itf_desc->bInterfaceSubClass &&
            TUD_BT_PROTOCOL_PRIMARY_CONTROLLER == itf_desc->bInterfaceProtocol, 0);
  TU_ASSERT(itf_desc->bNumEndpoints == 2 && max_len >= iso_alt_itf_size + drv_len);

  uint8_t dir;

  desc_ep = (tusb_desc_endpoint_t const *)tu_desc_next(itf_desc);
  TU_ASSERT(itf_desc->bAlternateSetting < CFG_TUD_BTH_ISO_ALT_COUNT, 0);
  TU_ASSERT(desc_ep->bDescriptorType == TUSB_DESC_ENDPOINT, 0);
  dir = tu_edpt_dir(desc_ep->bEndpointAddress);
  _btd_itf.ep_voice[dir] = desc_ep->bEndpointAddress;
  // Store endpoint size for alternative
  _btd_itf.ep_voice_size[dir][itf_desc->bAlternateSetting] = (uint8_t) tu_edpt_packet_size(desc_ep);

  desc_ep = (tusb_desc_endpoint_t const *)tu_desc_next(desc_ep);
  TU_ASSERT(desc_ep->bDescriptorType == TUSB_DESC_ENDPOINT, 0);
  dir = tu_edpt_dir(desc_ep->bEndpointAddress);
  _btd_itf.ep_voice[dir] = desc_ep->bEndpointAddress;
  // Store endpoint size for alternative
  _btd_itf.ep_voice_size[dir][itf_desc->bAlternateSetting] = (uint8_t) tu_edpt_packet_size(desc_ep);
  drv_len += iso_alt_itf_size;

  for (int i = 1; i < CFG_TUD_BTH_ISO_ALT_COUNT && drv_len + iso_alt_itf_size <= max_len; ++i) {
    // Make sure rest of alternatives matches
    itf_desc = (tusb_desc_interface_t const *)tu_desc_next(desc_ep);
    if (itf_desc->bDescriptorType != TUSB_DESC_INTERFACE ||
        TUSB_CLASS_WIRELESS_CONTROLLER != itf_desc->bInterfaceClass ||
        TUD_BT_APP_SUBCLASS != itf_desc->bInterfaceSubClass ||
        TUD_BT_PROTOCOL_PRIMARY_CONTROLLER != itf_desc->bInterfaceProtocol)
    {
      // Not an Iso interface instance
      break;
    }
    TU_ASSERT(itf_desc->bAlternateSetting < CFG_TUD_BTH_ISO_ALT_COUNT, 0);

    desc_ep = (tusb_desc_endpoint_t const *)tu_desc_next(itf_desc);
    dir = tu_edpt_dir(desc_ep->bEndpointAddress);
    // Verify that alternative endpoint are same as first ones
    TU_ASSERT(desc_ep->bDescriptorType == TUSB_DESC_ENDPOINT &&
              _btd_itf.ep_voice[dir] == desc_ep->bEndpointAddress, 0);
    _btd_itf.ep_voice_size[dir][itf_desc->bAlternateSetting] = (uint8_t) tu_edpt_packet_size(desc_ep);

    desc_ep = (tusb_desc_endpoint_t const *)tu_desc_next(desc_ep);
    dir = tu_edpt_dir(desc_ep->bEndpointAddress);
    // Verify that alternative endpoint are same as first ones
    TU_ASSERT(desc_ep->bDescriptorType == TUSB_DESC_ENDPOINT &&
              _btd_itf.ep_voice[dir] == desc_ep->bEndpointAddress, 0);
    _btd_itf.ep_voice_size[dir][itf_desc->bAlternateSetting] = (uint8_t) tu_edpt_packet_size(desc_ep);
    drv_len += iso_alt_itf_size;
  }

  return drv_len;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool btd_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  (void)rhport;

  if ( stage == CONTROL_STAGE_SETUP )
  {
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
        request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE)
    {
      // HCI command packet addressing for single function Primary Controllers
      // also compatible with historical mode if enabled
      TU_VERIFY((request->bRequest == 0 && request->wValue == 0 && request->wIndex == 0) ||
                (CFG_TUD_BTH_HISTORICAL_COMPATIBLE && request->bRequest == 0xe0));
    }
    else if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE)
    {
      if (request->bRequest == TUSB_REQ_SET_INTERFACE && _btd_itf.itf_num + 1 == request->wIndex)
      {
        // TODO: Set interface it would involve changing size of endpoint size
      }
      else
      {
        // HCI command packet for Primary Controller function in a composite device
        TU_VERIFY(request->bRequest == 0 && request->wValue == 0 && request->wIndex == _btd_itf.itf_num);
      }
    }
    else return false;

    return tud_control_xfer(rhport, request, &_btd_epbuf.hci_cmd, sizeof(bt_hci_cmd_t));
  }
  else if ( stage == CONTROL_STAGE_DATA )
  {
    // Handle class request only
    TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);

    if (tud_bt_hci_cmd_cb) {
      tud_bt_hci_cmd_cb(&_btd_epbuf.hci_cmd, tu_min16(request->wLength, sizeof(bt_hci_cmd_t)));
      }
  }

  return true;
}

bool btd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result,
                 uint32_t xferred_bytes) {
  // received new data from host
  if (ep_addr == _btd_itf.ep_acl_out)
  {
    if (tud_bt_acl_data_received_cb) tud_bt_acl_data_received_cb(_btd_epbuf.epout_buf, xferred_bytes);

    // prepare for next data
    TU_ASSERT(usbd_edpt_xfer(rhport, _btd_itf.ep_acl_out, _btd_epbuf.epout_buf, CFG_TUD_BTH_DATA_EPSIZE));
  }
  else if (ep_addr == _btd_itf.ep_ev)
  {
    if (tud_bt_event_sent_cb) tud_bt_event_sent_cb((uint16_t)xferred_bytes);
  }
  else if (ep_addr == _btd_itf.ep_acl_in)
  {
    if ((result == XFER_RESULT_SUCCESS) && (xferred_bytes > 0) &&
        ((xferred_bytes & (_btd_itf.ep_acl_in_pkt_sz - 1)) == 0)) {
      // Save number of transferred bytes
      _btd_itf.prev_xferred_bytes = xferred_bytes;

      // Send zero-length packet
      tud_bt_acl_data_send(NULL, 0);
    } else if (tud_bt_acl_data_sent_cb) {
      if (xferred_bytes == 0) {
        xferred_bytes = _btd_itf.prev_xferred_bytes;
        _btd_itf.prev_xferred_bytes = 0;
      }
      tud_bt_acl_data_sent_cb((uint16_t)xferred_bytes);
    }
  }

  return true;
}

#endif
