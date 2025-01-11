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

#if (CFG_TUD_ENABLED && CFG_TUD_CDC)

#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "cdc_device.h"

// Level where CFG_TUSB_DEBUG must be at least for this driver is logged
#ifndef CFG_TUD_CDC_LOG_LEVEL
  #define CFG_TUD_CDC_LOG_LEVEL   CFG_TUD_LOG_LEVEL
#endif

#define TU_LOG_DRV(...)   TU_LOG(CFG_TUD_CDC_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
#define BULK_PACKET_SIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)

typedef struct {
  uint8_t itf_num;
  uint8_t ep_notif;
  uint8_t ep_in;
  uint8_t ep_out;

  // Bit 0:  DTR (Data Terminal Ready), Bit 1: RTS (Request to Send)
  uint8_t line_state;

  /*------------- From this point, data is not cleared by bus reset -------------*/
  char wanted_char;
  TU_ATTR_ALIGNED(4) cdc_line_coding_t line_coding;

  // FIFO
  tu_fifo_t rx_ff;
  tu_fifo_t tx_ff;

  uint8_t rx_ff_buf[CFG_TUD_CDC_RX_BUFSIZE];
  uint8_t tx_ff_buf[CFG_TUD_CDC_TX_BUFSIZE];

  OSAL_MUTEX_DEF(rx_ff_mutex);
  OSAL_MUTEX_DEF(tx_ff_mutex);
} cdcd_interface_t;

#define ITF_MEM_RESET_SIZE   offsetof(cdcd_interface_t, wanted_char)

typedef struct {
  TUD_EPBUF_DEF(epout, CFG_TUD_CDC_EP_BUFSIZE);
  TUD_EPBUF_DEF(epin, CFG_TUD_CDC_EP_BUFSIZE);
} cdcd_epbuf_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static cdcd_interface_t _cdcd_itf[CFG_TUD_CDC];
CFG_TUD_MEM_SECTION static cdcd_epbuf_t _cdcd_epbuf[CFG_TUD_CDC];

static tud_cdc_configure_fifo_t _cdcd_fifo_cfg;

static bool _prep_out_transaction(uint8_t itf) {
  const uint8_t rhport = 0;
  cdcd_interface_t* p_cdc = &_cdcd_itf[itf];
  cdcd_epbuf_t* p_epbuf = &_cdcd_epbuf[itf];

  // Skip if usb is not ready yet
  TU_VERIFY(tud_ready() && p_cdc->ep_out);

  uint16_t available = tu_fifo_remaining(&p_cdc->rx_ff);

  // Prepare for incoming data but only allow what we can store in the ring buffer.
  // TODO Actually we can still carry out the transfer, keeping count of received bytes
  // and slowly move it to the FIFO when read().
  // This pre-check reduces endpoint claiming
  TU_VERIFY(available >= CFG_TUD_CDC_EP_BUFSIZE);

  // claim endpoint
  TU_VERIFY(usbd_edpt_claim(rhport, p_cdc->ep_out));

  // fifo can be changed before endpoint is claimed
  available = tu_fifo_remaining(&p_cdc->rx_ff);

  if (available >= CFG_TUD_CDC_EP_BUFSIZE) {
    return usbd_edpt_xfer(rhport, p_cdc->ep_out, p_epbuf->epout, CFG_TUD_CDC_EP_BUFSIZE);
  } else {
    // Release endpoint since we don't make any transfer
    usbd_edpt_release(rhport, p_cdc->ep_out);
    return false;
  }
}

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+

bool tud_cdc_configure_fifo(const tud_cdc_configure_fifo_t* cfg) {
  TU_VERIFY(cfg);
  _cdcd_fifo_cfg = (*cfg);
  return true;
}

bool tud_cdc_n_ready(uint8_t itf) {
  return tud_ready() && _cdcd_itf[itf].ep_in != 0 && _cdcd_itf[itf].ep_out != 0;
}

bool tud_cdc_n_connected(uint8_t itf) {
  // DTR (bit 0) active  is considered as connected
  return tud_ready() && tu_bit_test(_cdcd_itf[itf].line_state, 0);
}

uint8_t tud_cdc_n_get_line_state(uint8_t itf) {
  return _cdcd_itf[itf].line_state;
}

void tud_cdc_n_get_line_coding(uint8_t itf, cdc_line_coding_t* coding) {
  (*coding) = _cdcd_itf[itf].line_coding;
}

void tud_cdc_n_set_wanted_char(uint8_t itf, char wanted) {
  _cdcd_itf[itf].wanted_char = wanted;
}

//--------------------------------------------------------------------+
// READ API
//--------------------------------------------------------------------+
uint32_t tud_cdc_n_available(uint8_t itf) {
  return tu_fifo_count(&_cdcd_itf[itf].rx_ff);
}

uint32_t tud_cdc_n_read(uint8_t itf, void* buffer, uint32_t bufsize) {
  cdcd_interface_t* p_cdc = &_cdcd_itf[itf];
  uint32_t num_read = tu_fifo_read_n(&p_cdc->rx_ff, buffer, (uint16_t) TU_MIN(bufsize, UINT16_MAX));
  _prep_out_transaction(itf);
  return num_read;
}

bool tud_cdc_n_peek(uint8_t itf, uint8_t* chr) {
  return tu_fifo_peek(&_cdcd_itf[itf].rx_ff, chr);
}

void tud_cdc_n_read_flush(uint8_t itf) {
  cdcd_interface_t* p_cdc = &_cdcd_itf[itf];
  tu_fifo_clear(&p_cdc->rx_ff);
  _prep_out_transaction(itf);
}

//--------------------------------------------------------------------+
// WRITE API
//--------------------------------------------------------------------+
uint32_t tud_cdc_n_write(uint8_t itf, const void* buffer, uint32_t bufsize) {
  cdcd_interface_t* p_cdc = &_cdcd_itf[itf];
  uint16_t ret = tu_fifo_write_n(&p_cdc->tx_ff, buffer, (uint16_t) TU_MIN(bufsize, UINT16_MAX));

  // flush if queue more than packet size
  if (tu_fifo_count(&p_cdc->tx_ff) >= BULK_PACKET_SIZE
      #if CFG_TUD_CDC_TX_BUFSIZE < BULK_PACKET_SIZE
      || tu_fifo_full(&p_cdc->tx_ff) // check full if fifo size is less than packet size
      #endif
      ) {
    tud_cdc_n_write_flush(itf);
  }

  return ret;
}

uint32_t tud_cdc_n_write_flush(uint8_t itf) {
  cdcd_interface_t* p_cdc = &_cdcd_itf[itf];
  cdcd_epbuf_t* p_epbuf = &_cdcd_epbuf[itf];

  // Skip if usb is not ready yet
  TU_VERIFY(tud_ready(), 0);

  // No data to send
  if (!tu_fifo_count(&p_cdc->tx_ff)) {
    return 0;
  }

  const uint8_t rhport = 0;

  // Claim the endpoint
  TU_VERIFY(usbd_edpt_claim(rhport, p_cdc->ep_in), 0);

  // Pull data from FIFO
  const uint16_t count = tu_fifo_read_n(&p_cdc->tx_ff, p_epbuf->epin, CFG_TUD_CDC_EP_BUFSIZE);

  if (count) {
    TU_ASSERT(usbd_edpt_xfer(rhport, p_cdc->ep_in, p_epbuf->epin, count), 0);
    return count;
  } else {
    // Release endpoint since we don't make any transfer
    // Note: data is dropped if terminal is not connected
    usbd_edpt_release(rhport, p_cdc->ep_in);
    return 0;
  }
}

uint32_t tud_cdc_n_write_available(uint8_t itf) {
  return tu_fifo_remaining(&_cdcd_itf[itf].tx_ff);
}

bool tud_cdc_n_write_clear(uint8_t itf) {
  return tu_fifo_clear(&_cdcd_itf[itf].tx_ff);
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void cdcd_init(void) {
  tu_memclr(_cdcd_itf, sizeof(_cdcd_itf));
  tu_memclr(&_cdcd_fifo_cfg, sizeof(_cdcd_fifo_cfg));

  for (uint8_t i = 0; i < CFG_TUD_CDC; i++) {
    cdcd_interface_t* p_cdc = &_cdcd_itf[i];

    p_cdc->wanted_char = (char) -1;

    // default line coding is : stop bit = 1, parity = none, data bits = 8
    p_cdc->line_coding.bit_rate = 115200;
    p_cdc->line_coding.stop_bits = 0;
    p_cdc->line_coding.parity = 0;
    p_cdc->line_coding.data_bits = 8;

    // Config RX fifo
    tu_fifo_config(&p_cdc->rx_ff, p_cdc->rx_ff_buf, TU_ARRAY_SIZE(p_cdc->rx_ff_buf), 1, false);

    // Config TX fifo as overwritable at initialization and will be changed to non-overwritable
    // if terminal supports DTR bit. Without DTR we do not know if data is actually polled by terminal.
    // In this way, the most current data is prioritized.
    tu_fifo_config(&p_cdc->tx_ff, p_cdc->tx_ff_buf, TU_ARRAY_SIZE(p_cdc->tx_ff_buf), 1, true);

    #if OSAL_MUTEX_REQUIRED
    osal_mutex_t mutex_rd = osal_mutex_create(&p_cdc->rx_ff_mutex);
    osal_mutex_t mutex_wr = osal_mutex_create(&p_cdc->tx_ff_mutex);
    TU_ASSERT(mutex_rd != NULL && mutex_wr != NULL, );

    tu_fifo_config_mutex(&p_cdc->rx_ff, NULL, mutex_rd);
    tu_fifo_config_mutex(&p_cdc->tx_ff, mutex_wr, NULL);
    #endif
  }
}

bool cdcd_deinit(void) {
  #if OSAL_MUTEX_REQUIRED
  for(uint8_t i=0; i<CFG_TUD_CDC; i++) {
    cdcd_interface_t* p_cdc = &_cdcd_itf[i];
    osal_mutex_t mutex_rd = p_cdc->rx_ff.mutex_rd;
    osal_mutex_t mutex_wr = p_cdc->tx_ff.mutex_wr;

    if (mutex_rd) {
      osal_mutex_delete(mutex_rd);
      tu_fifo_config_mutex(&p_cdc->rx_ff, NULL, NULL);
    }

    if (mutex_wr) {
      osal_mutex_delete(mutex_wr);
      tu_fifo_config_mutex(&p_cdc->tx_ff, NULL, NULL);
    }
  }
  #endif

  return true;
}

void cdcd_reset(uint8_t rhport) {
  (void) rhport;

  for (uint8_t i = 0; i < CFG_TUD_CDC; i++) {
    cdcd_interface_t* p_cdc = &_cdcd_itf[i];

    tu_memclr(p_cdc, ITF_MEM_RESET_SIZE);
    if (!_cdcd_fifo_cfg.rx_persistent) {
      tu_fifo_clear(&p_cdc->rx_ff);
    }
    if (!_cdcd_fifo_cfg.tx_persistent) {
      tu_fifo_clear(&p_cdc->tx_ff);
    }
    tu_fifo_set_overwritable(&p_cdc->tx_ff, true);
  }
}

uint16_t cdcd_open(uint8_t rhport, const tusb_desc_interface_t* itf_desc, uint16_t max_len) {
  // Only support ACM subclass
  TU_VERIFY( TUSB_CLASS_CDC                           == itf_desc->bInterfaceClass &&
             CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL == itf_desc->bInterfaceSubClass, 0);

  // Find available interface
  cdcd_interface_t* p_cdc;
  uint8_t cdc_id;
  for (cdc_id = 0; cdc_id < CFG_TUD_CDC; cdc_id++) {
    p_cdc = &_cdcd_itf[cdc_id];
    if (p_cdc->ep_in == 0) {
      break;
    }
  }
  TU_ASSERT(cdc_id < CFG_TUD_CDC, 0);

  //------------- Control Interface -------------//
  p_cdc->itf_num = itf_desc->bInterfaceNumber;

  uint16_t drv_len = sizeof(tusb_desc_interface_t);
  const uint8_t* p_desc = tu_desc_next(itf_desc);

  // Communication Functional Descriptors
  while (TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len) {
    drv_len += tu_desc_len(p_desc);
    p_desc = tu_desc_next(p_desc);
  }

  if (TUSB_DESC_ENDPOINT == tu_desc_type(p_desc)) {
    // notification endpoint
    const tusb_desc_endpoint_t* desc_ep = (const tusb_desc_endpoint_t*) p_desc;

    TU_ASSERT(usbd_edpt_open(rhport, desc_ep), 0);
    p_cdc->ep_notif = desc_ep->bEndpointAddress;

    drv_len += tu_desc_len(p_desc);
    p_desc = tu_desc_next(p_desc);
  }

  //------------- Data Interface (if any) -------------//
  if ((TUSB_DESC_INTERFACE == tu_desc_type(p_desc)) &&
      (TUSB_CLASS_CDC_DATA == ((const tusb_desc_interface_t*) p_desc)->bInterfaceClass)) {
    // next to endpoint descriptor
    drv_len += tu_desc_len(p_desc);
    p_desc = tu_desc_next(p_desc);

    // Open endpoint pair
    TU_ASSERT(usbd_open_edpt_pair(rhport, p_desc, 2, TUSB_XFER_BULK, &p_cdc->ep_out, &p_cdc->ep_in), 0);

    drv_len += 2 * sizeof(tusb_desc_endpoint_t);
  }

  // Prepare for incoming data
  _prep_out_transaction(cdc_id);

  return drv_len;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool cdcd_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
  // Handle class request only
  TU_VERIFY(request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS);

  uint8_t itf;
  cdcd_interface_t* p_cdc;

  // Identify which interface to use
  for (itf = 0; itf < CFG_TUD_CDC; itf++) {
    p_cdc = &_cdcd_itf[itf];
    if (p_cdc->itf_num == request->wIndex) {
      break;
    }
  }
  TU_VERIFY(itf < CFG_TUD_CDC);

  switch (request->bRequest) {
    case CDC_REQUEST_SET_LINE_CODING:
      if (stage == CONTROL_STAGE_SETUP) {
        TU_LOG_DRV("  Set Line Coding\r\n");
        tud_control_xfer(rhport, request, &p_cdc->line_coding, sizeof(cdc_line_coding_t));
      } else if (stage == CONTROL_STAGE_ACK) {
        if (tud_cdc_line_coding_cb) {
          tud_cdc_line_coding_cb(itf, &p_cdc->line_coding);
        }
      }
      break;

    case CDC_REQUEST_GET_LINE_CODING:
      if (stage == CONTROL_STAGE_SETUP) {
        TU_LOG_DRV("  Get Line Coding\r\n");
        tud_control_xfer(rhport, request, &p_cdc->line_coding, sizeof(cdc_line_coding_t));
      }
      break;

    case CDC_REQUEST_SET_CONTROL_LINE_STATE:
      if (stage == CONTROL_STAGE_SETUP) {
        tud_control_status(rhport, request);
      } else if (stage == CONTROL_STAGE_ACK) {
        // CDC PSTN v1.2 section 6.3.12
        // Bit 0: Indicates if DTE is present or not.
        //        This signal corresponds to V.24 signal 108/2 and RS-232 signal DTR (Data Terminal Ready)
        // Bit 1: Carrier control for half-duplex modems.
        //        This signal corresponds to V.24 signal 105 and RS-232 signal RTS (Request to Send)
        bool const dtr = tu_bit_test(request->wValue, 0);
        bool const rts = tu_bit_test(request->wValue, 1);

        p_cdc->line_state = (uint8_t) request->wValue;

        // Disable fifo overwriting if DTR bit is set
        tu_fifo_set_overwritable(&p_cdc->tx_ff, !dtr);

        TU_LOG_DRV("  Set Control Line State: DTR = %d, RTS = %d\r\n", dtr, rts);

        // Invoke callback
        if (tud_cdc_line_state_cb) {
          tud_cdc_line_state_cb(itf, dtr, rts);
        }
      }
      break;

    case CDC_REQUEST_SEND_BREAK:
      if (stage == CONTROL_STAGE_SETUP) {
        tud_control_status(rhport, request);
      } else if (stage == CONTROL_STAGE_ACK) {
        TU_LOG_DRV("  Send Break\r\n");
        if (tud_cdc_send_break_cb) {
          tud_cdc_send_break_cb(itf, request->wValue);
        }
      }
      break;

    default:
      return false; // stall unsupported request
  }

  return true;
}

bool cdcd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void) result;

  uint8_t itf;
  cdcd_interface_t* p_cdc;

  // Identify which interface to use
  for (itf = 0; itf < CFG_TUD_CDC; itf++) {
    p_cdc = &_cdcd_itf[itf];
    if ((ep_addr == p_cdc->ep_out) || (ep_addr == p_cdc->ep_in)) {
      break;
    }
  }
  TU_ASSERT(itf < CFG_TUD_CDC);
  cdcd_epbuf_t* p_epbuf = &_cdcd_epbuf[itf];

  // Received new data
  if (ep_addr == p_cdc->ep_out) {
    tu_fifo_write_n(&p_cdc->rx_ff, p_epbuf->epout, (uint16_t) xferred_bytes);

    // Check for wanted char and invoke callback if needed
    if (tud_cdc_rx_wanted_cb && (((signed char) p_cdc->wanted_char) != -1)) {
      for (uint32_t i = 0; i < xferred_bytes; i++) {
        if ((p_cdc->wanted_char == p_epbuf->epout[i]) && !tu_fifo_empty(&p_cdc->rx_ff)) {
          tud_cdc_rx_wanted_cb(itf, p_cdc->wanted_char);
        }
      }
    }

    // invoke receive callback (if there is still data)
    if (tud_cdc_rx_cb && !tu_fifo_empty(&p_cdc->rx_ff)) {
      tud_cdc_rx_cb(itf);
    }

    // prepare for OUT transaction
    _prep_out_transaction(itf);
  }

  // Data sent to host, we continue to fetch from tx fifo to send.
  // Note: This will cause incorrect baudrate set in line coding.
  //       Though maybe the baudrate is not really important !!!
  if (ep_addr == p_cdc->ep_in) {
    // invoke transmit callback to possibly refill tx fifo
    if (tud_cdc_tx_complete_cb) {
      tud_cdc_tx_complete_cb(itf);
    }

    if (0 == tud_cdc_n_write_flush(itf)) {
      // If there is no data left, a ZLP should be sent if
      // xferred_bytes is multiple of EP Packet size and not zero
      if (!tu_fifo_count(&p_cdc->tx_ff) && xferred_bytes && (0 == (xferred_bytes & (BULK_PACKET_SIZE - 1)))) {
        if (usbd_edpt_claim(rhport, p_cdc->ep_in)) {
          usbd_edpt_xfer(rhport, p_cdc->ep_in, NULL, 0);
        }
      }
    }
  }

  // nothing to do with notif endpoint for now

  return true;
}

#endif
