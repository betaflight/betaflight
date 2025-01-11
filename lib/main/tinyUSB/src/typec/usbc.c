/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ha Thach (thach@tinyusb.org)
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

#if CFG_TUC_ENABLED

#include "tcd.h"
#include "usbc.h"

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

// Debug level of USBD
#define USBC_DEBUG   2
#define TU_LOG_USBC(...)   TU_LOG(USBC_DEBUG, __VA_ARGS__)

// Event queue
// usbc_int_set() is used as mutex in OS NONE config
void usbc_int_set(bool enabled);
OSAL_QUEUE_DEF(usbc_int_set, _usbc_qdef, CFG_TUC_TASK_QUEUE_SZ, tcd_event_t);
tu_static osal_queue_t _usbc_q;

// if stack is initialized
static bool _usbc_inited = false;

// if port is initialized
static bool _port_inited[TUP_TYPEC_RHPORTS_NUM];

// Max possible PD size is 262 bytes
static uint8_t _rx_buf[64] TU_ATTR_ALIGNED(4);
static uint8_t _tx_buf[64] TU_ATTR_ALIGNED(4);

bool usbc_msg_send(uint8_t rhport, pd_header_t const* header, void const* data);
bool parse_msg_data(uint8_t rhport, pd_header_t const* header, uint8_t const* dobj, uint8_t const* p_end);
bool parse_msg_control(uint8_t rhport, pd_header_t const* header);

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
bool tuc_inited(uint8_t rhport) {
  return _usbc_inited && _port_inited[rhport];
}

bool tuc_init(uint8_t rhport, uint32_t port_type) {
  // Initialize stack
  if (!_usbc_inited) {
    tu_memclr(_port_inited, sizeof(_port_inited));

    _usbc_q = osal_queue_create(&_usbc_qdef);
    TU_ASSERT(_usbc_q != NULL);

    _usbc_inited = true;
  }

  // skip if port already initialized
  if ( _port_inited[rhport] ) {
    return true;
  }

  TU_LOG_USBC("USBC init on port %u\r\n", rhport);
  TU_LOG_INT(USBC_DEBUG, sizeof(tcd_event_t));

  TU_ASSERT(tcd_init(rhport, port_type));
  tcd_int_enable(rhport);

  _port_inited[rhport] = true;
  return true;
}

void tuc_task_ext(uint32_t timeout_ms, bool in_isr) {
  (void) in_isr; // not implemented yet

  // Skip if stack is not initialized
  if (!_usbc_inited) return;

  // Loop until there is no more events in the queue
  while (1) {
    tcd_event_t event;
    if (!osal_queue_receive(_usbc_q, &event, timeout_ms)) return;

    switch (event.event_id) {
      case TCD_EVENT_CC_CHANGED:
        break;

      case TCD_EVENT_RX_COMPLETE:
        // TODO process message here in ISR, move to thread later
        if (event.xfer_complete.result == XFER_RESULT_SUCCESS) {
          pd_header_t const* header = (pd_header_t const*) _rx_buf;

          if (header->n_data_obj == 0) {
            parse_msg_control(event.rhport, header);

          }else {
            uint8_t const* p_end = _rx_buf + event.xfer_complete.xferred_bytes;
            uint8_t const * dobj = _rx_buf + sizeof(pd_header_t);

            parse_msg_data(event.rhport, header, dobj, p_end);
          }
        }

        // prepare for next message
        tcd_msg_receive(event.rhport, _rx_buf, sizeof(_rx_buf));
        break;

      case TCD_EVENT_TX_COMPLETE:
        break;

      default: break;
    }
  }
}

bool parse_msg_data(uint8_t rhport, pd_header_t const* header, uint8_t const* dobj, uint8_t const* p_end) {
  if (tuc_pd_data_received_cb) {
    tuc_pd_data_received_cb(rhport, header, dobj, p_end);
  }

  return true;
}

bool parse_msg_control(uint8_t rhport, pd_header_t const* header) {
  if (tuc_pd_control_received_cb) {
    tuc_pd_control_received_cb(rhport, header);
  }

  return true;
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

bool usbc_msg_send(uint8_t rhport, pd_header_t const* header, void const* data) {
  // copy header
  memcpy(_tx_buf, header, sizeof(pd_header_t));

  // copy data objcet if available
  uint16_t const n_data_obj = header->n_data_obj;
  if (n_data_obj > 0) {
    memcpy(_tx_buf + sizeof(pd_header_t), data, n_data_obj * 4);
  }

  return tcd_msg_send(rhport, _tx_buf, sizeof(pd_header_t) + n_data_obj * 4);
}

bool tuc_msg_request(uint8_t rhport, void const* rdo) {
  pd_header_t const header = {
      .msg_type = PD_DATA_REQUEST,
      .data_role = PD_DATA_ROLE_UFP,
      .specs_rev = PD_REV_30,
      .power_role = PD_POWER_ROLE_SINK,
      .msg_id = 0,
      .n_data_obj = 1,
      .extended = 0,
  };

  return usbc_msg_send(rhport, &header, rdo);
}

void tcd_event_handler(tcd_event_t const * event, bool in_isr) {
  (void) in_isr;
  switch(event->event_id) {
    case TCD_EVENT_CC_CHANGED:
      if (event->cc_changed.cc_state[0] || event->cc_changed.cc_state[1]) {
        // Attach, start receiving
        tcd_msg_receive(event->rhport, _rx_buf, sizeof(_rx_buf));
      }else {
        // Detach
      }
      break;

    default: break;
  }

  osal_queue_send(_usbc_q, event, in_isr);
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
void usbc_int_set(bool enabled) {
  // Disable all controllers since they shared the same event queue
  for (uint8_t p = 0; p < TUP_TYPEC_RHPORTS_NUM; p++) {
    if ( _port_inited[p] ) {
      if (enabled) {
        tcd_int_enable(p);
      }else {
        tcd_int_disable(p);
      }
    }
  }
}

#endif
