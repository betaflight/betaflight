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

#if (CFG_TUD_ENABLED && CFG_TUD_VENDOR)

#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "vendor_device.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct {
  uint8_t itf_num;

  /*------------- From this point, data is not cleared by bus reset -------------*/
  struct {
    tu_edpt_stream_t stream;
    #if CFG_TUD_VENDOR_TX_BUFSIZE > 0
    uint8_t ff_buf[CFG_TUD_VENDOR_TX_BUFSIZE];
    #endif
  } tx;

  struct {
    tu_edpt_stream_t stream;
    #if CFG_TUD_VENDOR_RX_BUFSIZE > 0
    uint8_t ff_buf[CFG_TUD_VENDOR_RX_BUFSIZE];
    #endif
  } rx;

} vendord_interface_t;

#define ITF_MEM_RESET_SIZE   (offsetof(vendord_interface_t, itf_num) + sizeof(((vendord_interface_t *)0)->itf_num))

static vendord_interface_t _vendord_itf[CFG_TUD_VENDOR];

typedef struct {
  TUD_EPBUF_DEF(epout, CFG_TUD_VENDOR_EPSIZE);
  TUD_EPBUF_DEF(epin, CFG_TUD_VENDOR_EPSIZE);
} vendord_epbuf_t;

CFG_TUD_MEM_SECTION static vendord_epbuf_t _vendord_epbuf[CFG_TUD_VENDOR];

//--------------------------------------------------------------------
// Application API
//--------------------------------------------------------------------

bool tud_vendor_n_mounted(uint8_t itf) {
  TU_VERIFY(itf < CFG_TUD_VENDOR);
  vendord_interface_t* p_itf = &_vendord_itf[itf];
  return p_itf->rx.stream.ep_addr || p_itf->tx.stream.ep_addr;
}

//--------------------------------------------------------------------+
// Read API
//--------------------------------------------------------------------+
uint32_t tud_vendor_n_available(uint8_t itf) {
  TU_VERIFY(itf < CFG_TUD_VENDOR, 0);
  vendord_interface_t* p_itf = &_vendord_itf[itf];

  return tu_edpt_stream_read_available(&p_itf->rx.stream);
}

bool tud_vendor_n_peek(uint8_t itf, uint8_t* u8) {
  TU_VERIFY(itf < CFG_TUD_VENDOR);
  vendord_interface_t* p_itf = &_vendord_itf[itf];

  return tu_edpt_stream_peek(&p_itf->rx.stream, u8);
}

uint32_t tud_vendor_n_read (uint8_t itf, void* buffer, uint32_t bufsize) {
  TU_VERIFY(itf < CFG_TUD_VENDOR, 0);
  vendord_interface_t* p_itf = &_vendord_itf[itf];
  const uint8_t rhport = 0;

  return tu_edpt_stream_read(rhport, &p_itf->rx.stream, buffer, bufsize);
}

void tud_vendor_n_read_flush (uint8_t itf) {
  TU_VERIFY(itf < CFG_TUD_VENDOR, );
  vendord_interface_t* p_itf = &_vendord_itf[itf];
  const uint8_t rhport = 0;

  tu_edpt_stream_clear(&p_itf->rx.stream);
  tu_edpt_stream_read_xfer(rhport, &p_itf->rx.stream);
}

//--------------------------------------------------------------------+
// Write API
//--------------------------------------------------------------------+
uint32_t tud_vendor_n_write (uint8_t itf, const void* buffer, uint32_t bufsize) {
  TU_VERIFY(itf < CFG_TUD_VENDOR, 0);
  vendord_interface_t* p_itf = &_vendord_itf[itf];
  const uint8_t rhport = 0;

  return tu_edpt_stream_write(rhport, &p_itf->tx.stream, buffer, (uint16_t) bufsize);
}

uint32_t tud_vendor_n_write_flush (uint8_t itf) {
  TU_VERIFY(itf < CFG_TUD_VENDOR, 0);
  vendord_interface_t* p_itf = &_vendord_itf[itf];
  const uint8_t rhport = 0;

  return tu_edpt_stream_write_xfer(rhport, &p_itf->tx.stream);
}

uint32_t tud_vendor_n_write_available (uint8_t itf) {
  TU_VERIFY(itf < CFG_TUD_VENDOR, 0);
  vendord_interface_t* p_itf = &_vendord_itf[itf];
  const uint8_t rhport = 0;

  return tu_edpt_stream_write_available(rhport, &p_itf->tx.stream);
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void vendord_init(void) {
  tu_memclr(_vendord_itf, sizeof(_vendord_itf));

  for(uint8_t i=0; i<CFG_TUD_VENDOR; i++) {
    vendord_interface_t* p_itf = &_vendord_itf[i];
    vendord_epbuf_t* p_epbuf = &_vendord_epbuf[i];

    uint8_t* rx_ff_buf =
                        #if CFG_TUD_VENDOR_RX_BUFSIZE > 0
                          p_itf->rx.ff_buf;
                        #else
                          NULL;
                        #endif

    tu_edpt_stream_init(&p_itf->rx.stream, false, false, false,
                        rx_ff_buf, CFG_TUD_VENDOR_RX_BUFSIZE,
                        p_epbuf->epout, CFG_TUD_VENDOR_EPSIZE);

    uint8_t* tx_ff_buf =
                        #if CFG_TUD_VENDOR_TX_BUFSIZE > 0
                          p_itf->tx.ff_buf;
                        #else
                          NULL;
                        #endif

    tu_edpt_stream_init(&p_itf->tx.stream, false, true, false,
                        tx_ff_buf, CFG_TUD_VENDOR_TX_BUFSIZE,
                        p_epbuf->epin, CFG_TUD_VENDOR_EPSIZE);
  }
}

bool vendord_deinit(void) {
  for(uint8_t i=0; i<CFG_TUD_VENDOR; i++) {
    vendord_interface_t* p_itf = &_vendord_itf[i];
    tu_edpt_stream_deinit(&p_itf->rx.stream);
    tu_edpt_stream_deinit(&p_itf->tx.stream);
  }
  return true;
}

void vendord_reset(uint8_t rhport) {
  (void) rhport;

  for(uint8_t i=0; i<CFG_TUD_VENDOR; i++) {
    vendord_interface_t* p_itf = &_vendord_itf[i];
    tu_memclr(p_itf, ITF_MEM_RESET_SIZE);
    tu_edpt_stream_clear(&p_itf->rx.stream);
    tu_edpt_stream_clear(&p_itf->tx.stream);
    tu_edpt_stream_close(&p_itf->rx.stream);
    tu_edpt_stream_close(&p_itf->tx.stream);
  }
}

uint16_t vendord_open(uint8_t rhport, const tusb_desc_interface_t* desc_itf, uint16_t max_len) {
  TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == desc_itf->bInterfaceClass, 0);
  const uint8_t* p_desc = tu_desc_next(desc_itf);
  const uint8_t* desc_end = p_desc + max_len;

  // Find available interface
  vendord_interface_t* p_vendor = NULL;
  for(uint8_t i=0; i<CFG_TUD_VENDOR; i++) {
    if (!tud_vendor_n_mounted(i)) {
      p_vendor = &_vendord_itf[i];
      break;
    }
  }
  TU_VERIFY(p_vendor, 0);

  p_vendor->itf_num = desc_itf->bInterfaceNumber;
  uint8_t found_ep = 0;
  while (found_ep < desc_itf->bNumEndpoints) {
    // skip non-endpoint descriptors
    while ( (TUSB_DESC_ENDPOINT != tu_desc_type(p_desc)) && (p_desc < desc_end) ) {
      p_desc = tu_desc_next(p_desc);
    }
    if (p_desc >= desc_end) {
      break;
    }

    const tusb_desc_endpoint_t* desc_ep = (const tusb_desc_endpoint_t*) p_desc;
    TU_ASSERT(usbd_edpt_open(rhport, desc_ep));
    found_ep++;

    if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
      tu_edpt_stream_open(&p_vendor->tx.stream, desc_ep);
      tud_vendor_n_write_flush((uint8_t)(p_vendor - _vendord_itf));
    } else {
      tu_edpt_stream_open(&p_vendor->rx.stream, desc_ep);
      TU_ASSERT(tu_edpt_stream_read_xfer(rhport, &p_vendor->rx.stream) > 0, 0); // prepare for incoming data
    }

    p_desc = tu_desc_next(p_desc);
  }

  return (uint16_t) ((uintptr_t) p_desc - (uintptr_t) desc_itf);
}

bool vendord_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void) result;

  uint8_t itf;
  vendord_interface_t* p_vendor;

  for (itf = 0; itf < CFG_TUD_VENDOR; itf++) {
    p_vendor = &_vendord_itf[itf];
    if ((ep_addr == p_vendor->rx.stream.ep_addr) || (ep_addr == p_vendor->tx.stream.ep_addr)) {
      break;
    }
  }
  TU_VERIFY(itf < CFG_TUD_VENDOR);
  vendord_epbuf_t* p_epbuf = &_vendord_epbuf[itf];

  if ( ep_addr == p_vendor->rx.stream.ep_addr ) {
    // Received new data: put into stream's fifo
    tu_edpt_stream_read_xfer_complete(&p_vendor->rx.stream, xferred_bytes);

    // Invoked callback if any
    if (tud_vendor_rx_cb) {
      tud_vendor_rx_cb(itf, p_epbuf->epout, (uint16_t) xferred_bytes);
    }

    tu_edpt_stream_read_xfer(rhport, &p_vendor->rx.stream);
  } else if ( ep_addr == p_vendor->tx.stream.ep_addr ) {
    // Send complete
    if (tud_vendor_tx_cb) {
      tud_vendor_tx_cb(itf, (uint16_t) xferred_bytes);
    }

    #if CFG_TUD_VENDOR_TX_BUFSIZE > 0
    // try to send more if possible
    if ( 0 == tu_edpt_stream_write_xfer(rhport, &p_vendor->tx.stream) ) {
      // If there is no data left, a ZLP should be sent if xferred_bytes is multiple of EP Packet size and not zero
      tu_edpt_stream_write_zlp_if_needed(rhport, &p_vendor->tx.stream, xferred_bytes);
    }
    #endif
  }

  return true;
}

#endif
