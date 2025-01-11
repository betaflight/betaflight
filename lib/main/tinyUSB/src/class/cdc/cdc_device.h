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

#ifndef TUSB_CDC_DEVICE_H_
#define TUSB_CDC_DEVICE_H_

#include "cdc.h"

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+
#if !defined(CFG_TUD_CDC_EP_BUFSIZE) && defined(CFG_TUD_CDC_EPSIZE)
  #warning CFG_TUD_CDC_EPSIZE is renamed to CFG_TUD_CDC_EP_BUFSIZE, please update to use the new name
  #define CFG_TUD_CDC_EP_BUFSIZE    CFG_TUD_CDC_EPSIZE
#endif

#ifndef CFG_TUD_CDC_EP_BUFSIZE
  #define CFG_TUD_CDC_EP_BUFSIZE    (TUD_OPT_HIGH_SPEED ? 512 : 64)
#endif

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Driver Configuration
//--------------------------------------------------------------------+

typedef struct TU_ATTR_PACKED {
  uint8_t rx_persistent : 1; // keep rx fifo on bus reset or disconnect
  uint8_t tx_persistent : 1; // keep tx fifo on bus reset or disconnect
} tud_cdc_configure_fifo_t;

// Configure CDC FIFOs behavior
bool tud_cdc_configure_fifo(tud_cdc_configure_fifo_t const* cfg);

//--------------------------------------------------------------------+
// Application API (Multiple Ports) i.e. CFG_TUD_CDC > 1
//--------------------------------------------------------------------+

// Check if interface is ready
bool tud_cdc_n_ready(uint8_t itf);

// Check if terminal is connected to this port
bool tud_cdc_n_connected(uint8_t itf);

// Get current line state. Bit 0:  DTR (Data Terminal Ready), Bit 1: RTS (Request to Send)
uint8_t tud_cdc_n_get_line_state(uint8_t itf);

// Get current line encoding: bit rate, stop bits parity etc ..
void tud_cdc_n_get_line_coding(uint8_t itf, cdc_line_coding_t* coding);

// Set special character that will trigger tud_cdc_rx_wanted_cb() callback on receiving
void tud_cdc_n_set_wanted_char(uint8_t itf, char wanted);

// Get the number of bytes available for reading
uint32_t tud_cdc_n_available(uint8_t itf);

// Read received bytes
uint32_t tud_cdc_n_read(uint8_t itf, void* buffer, uint32_t bufsize);

// Read a byte, return -1 if there is none
TU_ATTR_ALWAYS_INLINE static inline int32_t tud_cdc_n_read_char(uint8_t itf) {
  uint8_t ch;
  return tud_cdc_n_read(itf, &ch, 1) ? (int32_t) ch : -1;
}

// Clear the received FIFO
void tud_cdc_n_read_flush(uint8_t itf);

// Get a byte from FIFO without removing it
bool tud_cdc_n_peek(uint8_t itf, uint8_t* ui8);

// Write bytes to TX FIFO, data may remain in the FIFO for a while
uint32_t tud_cdc_n_write(uint8_t itf, void const* buffer, uint32_t bufsize);

// Write a byte
TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_n_write_char(uint8_t itf, char ch) {
  return tud_cdc_n_write(itf, &ch, 1);
}

// Write a null-terminated string
TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_n_write_str(uint8_t itf, char const* str) {
  return tud_cdc_n_write(itf, str, strlen(str));
}

// Force sending data if possible, return number of forced bytes
uint32_t tud_cdc_n_write_flush(uint8_t itf);

// Return the number of bytes (characters) available for writing to TX FIFO buffer in a single n_write operation.
uint32_t tud_cdc_n_write_available(uint8_t itf);

// Clear the transmit FIFO
bool tud_cdc_n_write_clear(uint8_t itf);

//--------------------------------------------------------------------+
// Application API (Single Port)
//--------------------------------------------------------------------+

TU_ATTR_ALWAYS_INLINE static inline bool tud_cdc_ready(void) {
  return tud_cdc_n_ready(0);
}

TU_ATTR_ALWAYS_INLINE static inline bool tud_cdc_connected(void) {
  return tud_cdc_n_connected(0);
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t tud_cdc_get_line_state(void) {
  return tud_cdc_n_get_line_state(0);
}

TU_ATTR_ALWAYS_INLINE static inline void tud_cdc_get_line_coding(cdc_line_coding_t* coding) {
  tud_cdc_n_get_line_coding(0, coding);
}

TU_ATTR_ALWAYS_INLINE static inline void tud_cdc_set_wanted_char(char wanted) {
  tud_cdc_n_set_wanted_char(0, wanted);
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_available(void) {
  return tud_cdc_n_available(0);
}

TU_ATTR_ALWAYS_INLINE static inline int32_t tud_cdc_read_char(void) {
  return tud_cdc_n_read_char(0);
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_read(void* buffer, uint32_t bufsize) {
  return tud_cdc_n_read(0, buffer, bufsize);
}

TU_ATTR_ALWAYS_INLINE static inline void tud_cdc_read_flush(void) {
  tud_cdc_n_read_flush(0);
}

TU_ATTR_ALWAYS_INLINE static inline bool tud_cdc_peek(uint8_t* ui8) {
  return tud_cdc_n_peek(0, ui8);
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_write_char(char ch) {
  return tud_cdc_n_write_char(0, ch);
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_write(void const* buffer, uint32_t bufsize) {
  return tud_cdc_n_write(0, buffer, bufsize);
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_write_str(char const* str) {
  return tud_cdc_n_write_str(0, str);
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_write_flush(void) {
  return tud_cdc_n_write_flush(0);
}

TU_ATTR_ALWAYS_INLINE static inline uint32_t tud_cdc_write_available(void) {
  return tud_cdc_n_write_available(0);
}

TU_ATTR_ALWAYS_INLINE static inline bool tud_cdc_write_clear(void) {
  return tud_cdc_n_write_clear(0);
}

//--------------------------------------------------------------------+
// Application Callback API (weak is optional)
//--------------------------------------------------------------------+

// Invoked when received new data
TU_ATTR_WEAK void tud_cdc_rx_cb(uint8_t itf);

// Invoked when received `wanted_char`
TU_ATTR_WEAK void tud_cdc_rx_wanted_cb(uint8_t itf, char wanted_char);

// Invoked when a TX is complete and therefore space becomes available in TX buffer
TU_ATTR_WEAK void tud_cdc_tx_complete_cb(uint8_t itf);

// Invoked when line state DTR & RTS are changed via SET_CONTROL_LINE_STATE
TU_ATTR_WEAK void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts);

// Invoked when line coding is change via SET_LINE_CODING
TU_ATTR_WEAK void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding);

// Invoked when received send break
// \param[in]  itf  interface for which send break was received.
// \param[in]  duration_ms  the length of time, in milliseconds, of the break signal. If a value of FFFFh, then the
//                          device will send a break until another SendBreak request is received with value 0000h.
TU_ATTR_WEAK void tud_cdc_send_break_cb(uint8_t itf, uint16_t duration_ms);

//--------------------------------------------------------------------+
// INTERNAL USBD-CLASS DRIVER API
//--------------------------------------------------------------------+
void     cdcd_init            (void);
bool     cdcd_deinit          (void);
void     cdcd_reset           (uint8_t rhport);
uint16_t cdcd_open            (uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
bool     cdcd_control_xfer_cb (uint8_t rhport, uint8_t stage, tusb_control_request_t const * request);
bool     cdcd_xfer_cb         (uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CDC_DEVICE_H_ */
