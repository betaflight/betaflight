/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ha Thach (tinyusb.org)
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

#if CFG_TUH_ENABLED && defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421

#include <stdatomic.h>
#include "host/hcd.h"
#include "host/usbh.h"

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

// Command format is
// Reg [7:3] | 0 [2] | Dir [1] | Ack [0]

enum {
  CMDBYTE_WRITE = 0x02,
};

enum {
  RCVVFIFO_ADDR = 1u  << 3, // 0x08
  SNDFIFO_ADDR  = 2u  << 3, // 0x10
  SUDFIFO_ADDR  = 4u  << 3, // 0x20
  RCVBC_ADDR    = 6u  << 3, // 0x30
  SNDBC_ADDR    = 7u  << 3, // 0x38
  USBIRQ_ADDR   = 13u << 3, // 0x68
  USBIEN_ADDR   = 14u << 3, // 0x70
  USBCTL_ADDR   = 15u << 3, // 0x78
  CPUCTL_ADDR   = 16u << 3, // 0x80
  PINCTL_ADDR   = 17u << 3, // 0x88
  REVISION_ADDR = 18u << 3, // 0x90
  // 19 is not used
  IOPINS1_ADDR  = 20u << 3, // 0xA0
  IOPINS2_ADDR  = 21u << 3, // 0xA8
  GPINIRQ_ADDR  = 22u << 3, // 0xB0
  GPINIEN_ADDR  = 23u << 3, // 0xB8
  GPINPOL_ADDR  = 24u << 3, // 0xC0
  HIRQ_ADDR     = 25u << 3, // 0xC8
  HIEN_ADDR     = 26u << 3, // 0xD0
  MODE_ADDR     = 27u << 3, // 0xD8
  PERADDR_ADDR  = 28u << 3, // 0xE0
  HCTL_ADDR     = 29u << 3, // 0xE8
  HXFR_ADDR     = 30u << 3, // 0xF0
  HRSL_ADDR     = 31u << 3, // 0xF8
};

enum {
  USBIRQ_OSCOK_IRQ  = 1u << 0,
  USBIRQ_NOVBUS_IRQ = 1u << 5,
  USBIRQ_VBUS_IRQ   = 1u << 6,
};

enum {
  USBCTL_PWRDOWN = 1u << 4,
  USBCTL_CHIPRES = 1u << 5,
};

enum {
  CPUCTL_IE        = 1u << 0,
  CPUCTL_PULSEWID0 = 1u << 6,
  CPUCTL_PULSEWID1 = 1u << 7,
};

enum {
  PINCTL_GPXA     = 1u << 0,
  PINCTL_GPXB     = 1u << 1,
  PINCTL_POSINT   = 1u << 2,
  PINCTL_INTLEVEL = 1u << 3,
  PINCTL_FDUPSPI  = 1u << 4,
};

enum {
  HIRQ_BUSEVENT_IRQ = 1u << 0,
  HIRQ_RWU_IRQ      = 1u << 1,
  HIRQ_RCVDAV_IRQ   = 1u << 2,
  HIRQ_SNDBAV_IRQ   = 1u << 3,
  HIRQ_SUSDN_IRQ    = 1u << 4,
  HIRQ_CONDET_IRQ   = 1u << 5,
  HIRQ_FRAME_IRQ    = 1u << 6,
  HIRQ_HXFRDN_IRQ   = 1u << 7,
};

enum {
  MODE_HOST      = 1u << 0,
  MODE_LOWSPEED  = 1u << 1,
  MODE_HUBPRE    = 1u << 2,
  MODE_SOFKAENAB = 1u << 3,
  MODE_SEPIRQ    = 1u << 4,
  MODE_DELAYISO  = 1u << 5,
  MODE_DMPULLDN  = 1u << 6,
  MODE_DPPULLDN  = 1u << 7,
};

enum {
  HCTL_BUSRST    = 1u << 0,
  HCTL_FRMRST    = 1u << 1,
  HCTL_SAMPLEBUS = 1u << 2,
  HCTL_SIGRSM    = 1u << 3,
  HCTL_RCVTOG0   = 1u << 4,
  HCTL_RCVTOG1   = 1u << 5,
  HCTL_SNDTOG0   = 1u << 6,
  HCTL_SNDTOG1   = 1u << 7,
};

enum {
  HXFR_EPNUM_MASK = 0x0f,
  HXFR_SETUP      = 1u << 4,
  HXFR_OUT_NIN    = 1u << 5,
  HXFR_ISO        = 1u << 6,
  HXFR_HS         = 1u << 7,
};

enum {
  HRSL_RESULT_MASK = 0x0f,
  HRSL_RCVTOGRD    = 1u << 4,
  HRSL_SNDTOGRD    = 1u << 5,
  HRSL_KSTATUS     = 1u << 6,
  HRSL_JSTATUS     = 1u << 7,
};

enum {
  HRSL_SUCCESS = 0,
  HRSL_BUSY,
  HRSL_BAD_REQ,
  HRSL_UNDEF,
  HRSL_NAK,
  HRSL_STALL,
  HRSL_TOG_ERR,
  HRSL_WRONG_PID,
  HRSL_BAD_BYTECOUNT,
  HRSL_PID_ERR,
  HRSL_PKT_ERR,
  HRSL_CRC_ERR,
  HRSL_K_ERR,
  HRSL_J_ERR,
  HRSL_TIMEOUT,
  HRSL_BABBLE,
};

enum {
  DEFAULT_HIEN = HIRQ_CONDET_IRQ | HIRQ_FRAME_IRQ | HIRQ_HXFRDN_IRQ | HIRQ_RCVDAV_IRQ
};

enum {
  MAX_NAK_DEFAULT = 1 // Number of NAK per endpoint per usb frame to save CPU/SPI bus usage
};

enum {
  EP_STATE_IDLE        = 0,
  EP_STATE_COMPLETE    = 1,
  EP_STATE_ABORTING    = 2,
  EP_STATE_ATTEMPT_1   = 3, // Number of attempts to transfer in a frame. Incremented after each NAK
  EP_STATE_ATTEMPT_MAX = 15
};

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

typedef struct TU_ATTR_PACKED {
  uint8_t ep_num   : 4;
  uint8_t is_setup : 1;
  uint8_t is_out   : 1;
  uint8_t is_iso   : 1;
} hxfr_bm_t;

TU_VERIFY_STATIC(sizeof(hxfr_bm_t) == 1, "size is not correct");

typedef struct {
  uint8_t daddr;

  union {
    hxfr_bm_t hxfr_bm;
    uint8_t hxfr;
  };

  struct TU_ATTR_PACKED {
    uint8_t state        : 4;
    uint8_t data_toggle  : 1;
    uint16_t packet_size : 11;
  };

  uint16_t total_len;
  uint16_t xferred_len;
  uint8_t* buf;
} max3421_ep_t;

TU_VERIFY_STATIC(sizeof(max3421_ep_t) == 12, "size is not correct");

typedef struct {
  volatile uint16_t frame_count;

  // cached register
  uint8_t sndbc;
  uint8_t hirq;
  uint8_t hien;
  uint8_t mode;
  uint8_t peraddr;
  union {
    hxfr_bm_t hxfr_bm;
    uint8_t hxfr;
  };

  // owner of data in SNDFIFO, for retrying NAKed without re-writing to FIFO
  struct {
    uint8_t daddr;
    uint8_t hxfr;
  }sndfifo_owner;

  atomic_flag busy; // busy transferring

#if OSAL_MUTEX_REQUIRED
  OSAL_MUTEX_DEF(spi_mutexdef);
  osal_mutex_t spi_mutex;
#endif

  max3421_ep_t ep[CFG_TUH_MAX3421_ENDPOINT_TOTAL]; // [0] is reserved for addr0
} max3421_data_t;

static max3421_data_t _hcd_data;

// max NAK before giving up in a frame. 0 means infinite NAKs
static tuh_configure_max3421_t _tuh_cfg = {
    .max_nak = MAX_NAK_DEFAULT,
    .cpuctl = 0, // default: INT pulse width = 10.6 us
    .pinctl = 0, // default: negative edge interrupt
};

//--------------------------------------------------------------------+
// API: SPI transfer with MAX3421E
// - spi_cs_api(), spi_xfer_api(), int_api(): must be implemented by application
// - reg_read(), reg_write(): is implemented by this driver, can be used by application
//--------------------------------------------------------------------+

// API to control MAX3421 SPI CS
extern void tuh_max3421_spi_cs_api(uint8_t rhport, bool active);

// API to transfer data with MAX3421 SPI
// Either tx_buf or rx_buf can be NULL, which means transfer is write or read only
extern bool tuh_max3421_spi_xfer_api(uint8_t rhport, uint8_t const* tx_buf, uint8_t* rx_buf, size_t xfer_bytes);

// API to enable/disable MAX3421 INTR pin interrupt
extern void tuh_max3421_int_api(uint8_t rhport, bool enabled);

// API to read MAX3421's register. Implemented by TinyUSB
uint8_t tuh_max3421_reg_read(uint8_t rhport, uint8_t reg, bool in_isr);

// API to write MAX3421's register. Implemented by TinyUSB
bool tuh_max3421_reg_write(uint8_t rhport, uint8_t reg, uint8_t data, bool in_isr);

//--------------------------------------------------------------------+
// SPI Commands and Helper
//--------------------------------------------------------------------+

#define reg_read  tuh_max3421_reg_read
#define reg_write tuh_max3421_reg_write

static void max3421_spi_lock(uint8_t rhport, bool in_isr) {
  // disable interrupt and mutex lock (for pre-emptive RTOS) if not in_isr
  if (!in_isr) {
    (void) osal_mutex_lock(_hcd_data.spi_mutex, OSAL_TIMEOUT_WAIT_FOREVER);
    tuh_max3421_int_api(rhport, false);
  }

  // assert CS
  tuh_max3421_spi_cs_api(rhport, true);
}

static void max3421_spi_unlock(uint8_t rhport, bool in_isr) {
  // de-assert CS
  tuh_max3421_spi_cs_api(rhport, false);

  // mutex unlock and re-enable interrupt
  if (!in_isr) {
    tuh_max3421_int_api(rhport, true);
    (void) osal_mutex_unlock(_hcd_data.spi_mutex);
  }
}

uint8_t tuh_max3421_reg_read(uint8_t rhport, uint8_t reg, bool in_isr) {
  uint8_t tx_buf[2] = {reg, 0};
  uint8_t rx_buf[2] = {0, 0};

  max3421_spi_lock(rhport, in_isr);
  bool ret = tuh_max3421_spi_xfer_api(rhport, tx_buf, rx_buf, 2);
  max3421_spi_unlock(rhport, in_isr);

  _hcd_data.hirq = rx_buf[0];
  return ret ? rx_buf[1] : 0;
}

bool tuh_max3421_reg_write(uint8_t rhport, uint8_t reg, uint8_t data, bool in_isr) {
  uint8_t tx_buf[2] = {reg | CMDBYTE_WRITE, data};
  uint8_t rx_buf[2] = {0, 0};

  max3421_spi_lock(rhport, in_isr);
  bool ret = tuh_max3421_spi_xfer_api(rhport, tx_buf, rx_buf, 2);
  max3421_spi_unlock(rhport, in_isr);

  // HIRQ register since we are in full-duplex mode
  _hcd_data.hirq = rx_buf[0];

  return ret;
}

//--------------------------------------------------------------------
// Register helper
//--------------------------------------------------------------------
TU_ATTR_ALWAYS_INLINE static inline void hirq_write(uint8_t rhport, uint8_t data, bool in_isr) {
  reg_write(rhport, HIRQ_ADDR, data, in_isr);
  // HIRQ write 1 is clear
  _hcd_data.hirq &= (uint8_t) ~data;
}

TU_ATTR_ALWAYS_INLINE static inline void hien_write(uint8_t rhport, uint8_t data, bool in_isr) {
  _hcd_data.hien = data;
  reg_write(rhport, HIEN_ADDR, data, in_isr);
}

TU_ATTR_ALWAYS_INLINE static inline void mode_write(uint8_t rhport, uint8_t data, bool in_isr) {
  _hcd_data.mode = data;
  reg_write(rhport, MODE_ADDR, data, in_isr);
}

TU_ATTR_ALWAYS_INLINE static inline void peraddr_write(uint8_t rhport, uint8_t data, bool in_isr) {
  if ( _hcd_data.peraddr == data ) return; // no need to change address

  _hcd_data.peraddr = data;
  reg_write(rhport, PERADDR_ADDR, data, in_isr);
}

TU_ATTR_ALWAYS_INLINE static inline void hxfr_write(uint8_t rhport, uint8_t data, bool in_isr) {
  _hcd_data.hxfr = data;
  reg_write(rhport, HXFR_ADDR, data, in_isr);
}

TU_ATTR_ALWAYS_INLINE static inline void sndbc_write(uint8_t rhport, uint8_t data, bool in_isr) {
  _hcd_data.sndbc = data;
  reg_write(rhport, SNDBC_ADDR, data, in_isr);
}

//--------------------------------------------------------------------
// FIFO access (receive, send, setup)
//--------------------------------------------------------------------
static void hwfifo_write(uint8_t rhport, uint8_t reg, const uint8_t* buffer, uint8_t len, bool in_isr) {
  uint8_t hirq;
  reg |= CMDBYTE_WRITE;

  max3421_spi_lock(rhport, in_isr);

  tuh_max3421_spi_xfer_api(rhport, &reg, &hirq, 1);
  _hcd_data.hirq = hirq;
  tuh_max3421_spi_xfer_api(rhport, buffer, NULL, len);

  max3421_spi_unlock(rhport, in_isr);
}

// Write to SNDFIFO if len > 0 and update SNDBC
TU_ATTR_ALWAYS_INLINE static inline void hwfifo_send(uint8_t rhport, const uint8_t* buffer, uint8_t len, bool in_isr) {
  if (len) {
    hwfifo_write(rhport, SNDFIFO_ADDR, buffer, len, in_isr);
  }
  sndbc_write(rhport, len, in_isr);
}

TU_ATTR_ALWAYS_INLINE static inline void hwfifo_setup(uint8_t rhport, const uint8_t* buffer, bool in_isr) {
  hwfifo_write(rhport, SUDFIFO_ADDR, buffer, 8, in_isr);
}

static void hwfifo_receive(uint8_t rhport, uint8_t * buffer, uint16_t len, bool in_isr) {
  uint8_t hirq;
  uint8_t const reg = RCVVFIFO_ADDR;

  max3421_spi_lock(rhport, in_isr);

  tuh_max3421_spi_xfer_api(rhport, &reg, &hirq, 1);
  _hcd_data.hirq = hirq;
  tuh_max3421_spi_xfer_api(rhport, NULL, buffer, len);

  max3421_spi_unlock(rhport, in_isr);
}

//--------------------------------------------------------------------+
// Endpoint helper
//--------------------------------------------------------------------+

static max3421_ep_t* find_ep_not_addr0(uint8_t daddr, uint8_t ep_num, uint8_t ep_dir) {
  uint8_t const is_out = 1-ep_dir;
  for(size_t i=1; i<CFG_TUH_MAX3421_ENDPOINT_TOTAL; i++) {
    max3421_ep_t* ep = &_hcd_data.ep[i];
    // control endpoint is bi-direction (skip check)
    if (daddr == ep->daddr && ep_num == ep->hxfr_bm.ep_num && (ep_num == 0 || is_out == ep->hxfr_bm.is_out)) {
      return ep;
    }
  }

  return NULL;
}

// daddr = 0 and ep_num = 0 means find a free (allocate) endpoint
TU_ATTR_ALWAYS_INLINE static inline max3421_ep_t * allocate_ep(void) {
  return find_ep_not_addr0(0, 0, 0);
}

TU_ATTR_ALWAYS_INLINE static inline max3421_ep_t * find_opened_ep(uint8_t daddr, uint8_t ep_num, uint8_t ep_dir) {
  if (daddr == 0 && ep_num == 0) {
    return &_hcd_data.ep[0];
  }else{
    return find_ep_not_addr0(daddr, ep_num, ep_dir);
  }
}

// free all endpoints belong to device address
static void free_ep(uint8_t daddr) {
  for (size_t i=1; i<CFG_TUH_MAX3421_ENDPOINT_TOTAL; i++) {
    max3421_ep_t* ep = &_hcd_data.ep[i];
    if (ep->daddr == daddr) {
      tu_memclr(ep, sizeof(max3421_ep_t));
    }
  }
}

// Check if endpoint has a queued transfer and not reach max NAK in this frame
TU_ATTR_ALWAYS_INLINE static inline bool is_ep_pending(max3421_ep_t const * ep) {
  uint8_t const state = ep->state;
  return ep->packet_size && (state >= EP_STATE_ATTEMPT_1) &&
         (_tuh_cfg.max_nak == 0 || state < EP_STATE_ATTEMPT_1 + _tuh_cfg.max_nak);
}

// Find the next pending endpoint using round-robin scheduling, starting from next endpoint.
// return NULL if not found
// TODO respect interrupt endpoint's interval
static max3421_ep_t * find_next_pending_ep(max3421_ep_t * cur_ep) {
  size_t const idx = (size_t) (cur_ep - _hcd_data.ep);

  // starting from next endpoint
  for (size_t i = idx + 1; i < CFG_TUH_MAX3421_ENDPOINT_TOTAL; i++) {
    max3421_ep_t* ep = &_hcd_data.ep[i];
    if (is_ep_pending(ep)) {
      return ep;
    }
  }

  // wrap around including current endpoint
  for (size_t i = 0; i <= idx; i++) {
    max3421_ep_t* ep = &_hcd_data.ep[i];
    if (is_ep_pending(ep)) {
      return ep;
    }
  }

  return NULL;
}

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

// optional hcd configuration, called by tuh_configure()
bool hcd_configure(uint8_t rhport, uint32_t cfg_id, const void* cfg_param) {
  (void) rhport;
  TU_VERIFY(cfg_id == TUH_CFGID_MAX3421 && cfg_param != NULL);

  tuh_configure_param_t const* cfg = (tuh_configure_param_t const*) cfg_param;
  _tuh_cfg = cfg->max3421;
  _tuh_cfg.max_nak = tu_min8(_tuh_cfg.max_nak, EP_STATE_ATTEMPT_MAX-EP_STATE_ATTEMPT_1);
  return true;
}

// Initialize controller to host mode
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;

  tuh_max3421_int_api(rhport, false);

  TU_LOG2_INT(sizeof(max3421_ep_t));
  TU_LOG2_INT(sizeof(max3421_data_t));
  TU_LOG2_INT(offsetof(max3421_data_t, ep));

  tu_memclr(&_hcd_data, sizeof(_hcd_data));
  _hcd_data.peraddr = 0xff; // invalid

#if OSAL_MUTEX_REQUIRED
  _hcd_data.spi_mutex = osal_mutex_create(&_hcd_data.spi_mutexdef);
#endif

  // NOTE: driver does not seem to work without nRST pin signal

  // full duplex, interrupt negative edge
  reg_write(rhport, PINCTL_ADDR, _tuh_cfg.pinctl | PINCTL_FDUPSPI, false);

  // v1 is 0x01, v2 is 0x12, v3 is 0x13
  // Note: v1 and v2 has host OUT errata whose workaround is not implemented in this driver
  uint8_t const revision = reg_read(rhport, REVISION_ADDR, false);
  TU_LOG2_HEX(revision);
  TU_ASSERT(revision == 0x01 || revision == 0x12 || revision == 0x13, false);

  // reset
  reg_write(rhport, USBCTL_ADDR, USBCTL_CHIPRES, false);
  reg_write(rhport, USBCTL_ADDR, 0, false);
  while( !(reg_read(rhport, USBIRQ_ADDR, false) & USBIRQ_OSCOK_IRQ) ) {
    // wait for oscillator to stabilize
  }

  // Mode: Host and DP/DM pull down
  mode_write(rhport, MODE_DPPULLDN | MODE_DMPULLDN | MODE_HOST, false);

  // frame reset & bus reset, this will trigger CONDET IRQ if device is already connected
  reg_write(rhport, HCTL_ADDR, HCTL_BUSRST | HCTL_FRMRST, false);

  // clear all previously pending IRQ
  hirq_write(rhport, 0xff, false);

  // Enable IRQ
  hien_write(rhport, DEFAULT_HIEN, false);

  tuh_max3421_int_api(rhport, true);

  // Enable Interrupt pin
  reg_write(rhport, CPUCTL_ADDR, _tuh_cfg.cpuctl | CPUCTL_IE, false);

  return true;
}

bool hcd_deinit(uint8_t rhport) {
  (void) rhport;

  // disable interrupt
  tuh_max3421_int_api(rhport, false);

  // reset max3421 and power down
  reg_write(rhport, USBCTL_ADDR, USBCTL_CHIPRES, false);
  reg_write(rhport, USBCTL_ADDR, USBCTL_PWRDOWN, false);

  #if OSAL_MUTEX_REQUIRED
  osal_mutex_delete(_hcd_data.spi_mutex);
  _hcd_data.spi_mutex = NULL;
  #endif

  return true;
}

// Enable USB interrupt
// Not actually enable GPIO interrupt, just set variable to prevent handler to process
void hcd_int_enable (uint8_t rhport) {
  tuh_max3421_int_api(rhport, true);
}

// Disable USB interrupt
// Not actually disable GPIO interrupt, just set variable to prevent handler to process
void hcd_int_disable(uint8_t rhport) {
  tuh_max3421_int_api(rhport, false);
}

// Get frame number (1ms)
uint32_t hcd_frame_number(uint8_t rhport) {
  (void) rhport;
  return (uint32_t ) _hcd_data.frame_count;
}

//--------------------------------------------------------------------+
// Port API
//--------------------------------------------------------------------+

// Get the current connect status of roothub port
bool hcd_port_connect_status(uint8_t rhport) {
  (void) rhport;
  return (_hcd_data.mode & MODE_SOFKAENAB) ? true : false;
}

// Reset USB bus on the port. Return immediately, bus reset sequence may not be complete.
// Some port would require hcd_port_reset_end() to be invoked after 10ms to complete the reset sequence.
void hcd_port_reset(uint8_t rhport) {
  reg_write(rhport, HCTL_ADDR, HCTL_BUSRST, false);
}

// Complete bus reset sequence, may be required by some controllers
void hcd_port_reset_end(uint8_t rhport) {
  reg_write(rhport, HCTL_ADDR, 0, false);
}

// Get port link speed
tusb_speed_t hcd_port_speed_get(uint8_t rhport) {
  (void) rhport;
  return (_hcd_data.mode & MODE_LOWSPEED) ? TUSB_SPEED_LOW : TUSB_SPEED_FULL;
}

// HCD closes all opened endpoints belong to this device
void hcd_device_close(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  (void) dev_addr;
}

//--------------------------------------------------------------------+
// Endpoints API
//--------------------------------------------------------------------+

// Open an endpoint
bool hcd_edpt_open(uint8_t rhport, uint8_t daddr, tusb_desc_endpoint_t const * ep_desc) {
  (void) rhport;

  uint8_t const ep_num = tu_edpt_number(ep_desc->bEndpointAddress);
  tusb_dir_t const ep_dir = tu_edpt_dir(ep_desc->bEndpointAddress);

  max3421_ep_t * ep;
  if (daddr == 0 && ep_num == 0) {
    ep = &_hcd_data.ep[0];
  }else {
    ep = allocate_ep();
    TU_ASSERT(ep);
    ep->daddr = daddr;
    ep->hxfr_bm.ep_num = (uint8_t) (ep_num & 0x0f);
    ep->hxfr_bm.is_out = (ep_dir == TUSB_DIR_OUT) ? 1 : 0;
    ep->hxfr_bm.is_iso = (TUSB_XFER_ISOCHRONOUS == ep_desc->bmAttributes.xfer) ? 1 : 0;
  }

  ep->packet_size = (uint16_t) (tu_edpt_packet_size(ep_desc) & 0x7ff);

  return true;
}

/* The microcontroller repeatedly writes the SNDFIFO register R2 to load the FIFO with up to 64 data bytes.
 * Then the microcontroller writes the SNDBC register, which this does three things:
 * 1. Tells the MAX3421E SIE (Serial Interface Engine) how many bytes in the FIFO to send.
 * 2. Connects the SNDFIFO and SNDBC register to the USB logic for USB transmission.
 * 3. Clears the SNDBAVIRQ interrupt flag. If the second FIFO is available for µC loading, the SNDBAVIRQ immediately re-asserts.

                                               +-----------+
                                           --->| SNDBC-A   |
                                          /    | SNDFIFO-A |
                                         /     +-----------+
      +------+       +-------------+    /                              +----------+
      | MCU  |------>| R2: SNDFIFO |----     << Write R7 Flip >>    ---| MAX3241E |
      |(hcd) |       | R7: SNDBC   |                               /   |   SIE    |
      +------+       +-------------+                              /    +----------+
                                              +-----------+      /
                                               | SNDBC-B   |    /
                                               | SNDFIFO-B |<---
                                               +-----------+
  Note: xact_out() is called when starting a new transfer, continue a transfer (isr) or retry a transfer (NAK)
        For NAK retry, we do not need to write to FIFO or SNDBC register again.
*/
static void xact_out(uint8_t rhport, max3421_ep_t *ep, bool switch_ep, bool in_isr) {
  // Page 12: Programming BULK-OUT Transfers
  // TODO: double buffering for ISO transfer
  if (switch_ep) {
    peraddr_write(rhport, ep->daddr, in_isr);
    const uint8_t hctl = (ep->data_toggle ? HCTL_SNDTOG1 : HCTL_SNDTOG0);
    reg_write(rhport, HCTL_ADDR, hctl, in_isr);
  }

  // Only write to sndfifo and sdnbc register if it is not a NAKed retry
  if (!(ep->daddr == _hcd_data.sndfifo_owner.daddr && ep->hxfr == _hcd_data.sndfifo_owner.hxfr)) {
    // skip SNDBAV IRQ check, overwrite sndfifo if needed
    const uint8_t xact_len = (uint8_t) tu_min16(ep->total_len - ep->xferred_len, ep->packet_size);
    hwfifo_send(rhport, ep->buf, xact_len, in_isr);
  }
  _hcd_data.sndfifo_owner.daddr = ep->daddr;
  _hcd_data.sndfifo_owner.hxfr = ep->hxfr;

  hxfr_write(rhport, ep->hxfr, in_isr);
}

static void xact_in(uint8_t rhport, max3421_ep_t *ep, bool switch_ep, bool in_isr) {
  // Page 13: Programming BULK-IN Transfers
  if (switch_ep) {
    peraddr_write(rhport, ep->daddr, in_isr);

    uint8_t const hctl = (ep->data_toggle ? HCTL_RCVTOG1 : HCTL_RCVTOG0);
    reg_write(rhport, HCTL_ADDR, hctl, in_isr);
  }

  hxfr_write(rhport, ep->hxfr, in_isr);
}

static void xact_setup(uint8_t rhport, max3421_ep_t *ep, bool in_isr) {
  peraddr_write(rhport, ep->daddr, in_isr);
  hwfifo_setup(rhport, ep->buf, in_isr);
  hxfr_write(rhport, HXFR_SETUP, in_isr);
}

static void xact_generic(uint8_t rhport, max3421_ep_t *ep, bool switch_ep, bool in_isr) {
  if (ep->hxfr_bm.ep_num == 0 ) {
    // setup
    if (ep->hxfr_bm.is_setup) {
      xact_setup(rhport, ep, in_isr);
      return;
    }

    // status
    if (ep->buf == NULL || ep->total_len == 0) {
      const uint8_t hxfr = (uint8_t) (HXFR_HS | (ep->hxfr & HXFR_OUT_NIN));
      peraddr_write(rhport, ep->daddr, in_isr);
      hxfr_write(rhport, hxfr, in_isr);
      return;
    }
  }

  if (ep->hxfr_bm.is_out) {
    xact_out(rhport, ep, switch_ep, in_isr);
  }else {
    xact_in(rhport, ep, switch_ep, in_isr);
  }
}

// Submit a transfer, when complete hcd_event_xfer_complete() must be invoked
bool hcd_edpt_xfer(uint8_t rhport, uint8_t daddr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen) {
  uint8_t const ep_num = tu_edpt_number(ep_addr);
  uint8_t const ep_dir = (uint8_t) tu_edpt_dir(ep_addr);
  max3421_ep_t* ep = find_opened_ep(daddr, ep_num, ep_dir);
  TU_VERIFY(ep);

  if (ep_num == 0) {
    // control transfer can switch direction
    ep->hxfr_bm.is_out = ep_dir ? 0 : 1;
    ep->hxfr_bm.is_setup = 0;
    ep->data_toggle = 1;
  }

  ep->buf = buffer;
  ep->total_len = buflen;
  ep->xferred_len = 0;
  ep->state = EP_STATE_ATTEMPT_1;

  // carry out transfer if not busy
  if (!atomic_flag_test_and_set(&_hcd_data.busy)) {
    xact_generic(rhport, ep, true, false);
  }

  return true;
}

bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t daddr, uint8_t ep_addr) {
  uint8_t const ep_num = tu_edpt_number(ep_addr);
  uint8_t const ep_dir = (uint8_t) tu_edpt_dir(ep_addr);
  max3421_ep_t* ep = find_opened_ep(daddr, ep_num, ep_dir);
  TU_VERIFY(ep);

  if (EP_STATE_ATTEMPT_1 <= ep->state && ep->state < EP_STATE_ATTEMPT_MAX) {
    hcd_int_disable(rhport);
    ep->state = EP_STATE_ABORTING;
    hcd_int_enable(rhport);
  }

  return true;
}

// Submit a special transfer to send 8-byte Setup Packet, when complete hcd_event_xfer_complete() must be invoked
bool hcd_setup_send(uint8_t rhport, uint8_t daddr, uint8_t const setup_packet[8]) {
  (void) rhport;

  max3421_ep_t* ep = find_opened_ep(daddr, 0, 0);
  TU_ASSERT(ep);

  ep->hxfr_bm.is_out = 1;
  ep->hxfr_bm.is_setup = 1;
  ep->buf = (uint8_t*)(uintptr_t) setup_packet;
  ep->total_len = 8;
  ep->xferred_len = 0;
  ep->state = EP_STATE_ATTEMPT_1;

  // carry out transfer if not busy
  if (!atomic_flag_test_and_set(&_hcd_data.busy)) {
    xact_setup(rhport, ep, false);
  }

  return true;
}

// clear stall, data toggle is also reset to DATA0
bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;

  return false;
}

//--------------------------------------------------------------------+
// Interrupt Handler
//--------------------------------------------------------------------+

static void handle_connect_irq(uint8_t rhport, bool in_isr) {
  uint8_t const hrsl = reg_read(rhport, HRSL_ADDR, in_isr);
  uint8_t const jk = hrsl & (HRSL_JSTATUS | HRSL_KSTATUS);

  uint8_t new_mode = MODE_DPPULLDN | MODE_DMPULLDN | MODE_HOST;
  TU_LOG2_HEX(jk);

  switch(jk) {
    case 0x00:                          // SEO is disconnected
    case (HRSL_JSTATUS | HRSL_KSTATUS): // SE1 is illegal
      mode_write(rhport, new_mode, in_isr);

      // port reset anyway, this will help to stable bus signal for next connection
      reg_write(rhport, HCTL_ADDR, HCTL_BUSRST, in_isr);
      hcd_event_device_remove(rhport, in_isr);
      reg_write(rhport, HCTL_ADDR, 0, in_isr);
      break;

    default: {
      // Bus Reset also cause CONDET IRQ, skip if we are already connected and doing bus reset
      if ((_hcd_data.hirq & HIRQ_BUSEVENT_IRQ) && (_hcd_data.mode & MODE_SOFKAENAB)) {
        break;
      }

      // Low speed if (LS = 1 and J-state) or (LS = 0 and K-State)
      // However, since we are always in full speed mode, we can just check J-state
      if (jk == HRSL_KSTATUS) {
        new_mode |= MODE_LOWSPEED;
        TU_LOG3("Low speed\r\n");
      }else {
        TU_LOG3("Full speed\r\n");
      }
      new_mode |= MODE_SOFKAENAB;
      mode_write(rhport, new_mode, in_isr);

      // FIXME multiple MAX3421 rootdevice address is not 1
      uint8_t const daddr = 1;
      free_ep(daddr);

      hcd_event_device_attach(rhport, in_isr);
      break;
    }
  }
}

static void xfer_complete_isr(uint8_t rhport, max3421_ep_t *ep, xfer_result_t result, uint8_t hrsl, bool in_isr) {
  uint8_t const ep_dir = 1-ep->hxfr_bm.is_out;
  uint8_t const ep_addr = tu_edpt_addr(ep->hxfr_bm.ep_num, ep_dir);

  // save data toggle
  if (ep_dir) {
    ep->data_toggle = (hrsl & HRSL_RCVTOGRD) ? 1u : 0u;
  }else {
    ep->data_toggle = (hrsl & HRSL_SNDTOGRD) ? 1u : 0u;
  }

  ep->state = EP_STATE_IDLE;
  hcd_event_xfer_complete(ep->daddr, ep_addr, ep->xferred_len, result, in_isr);

  // Find next pending endpoint
  max3421_ep_t * next_ep = find_next_pending_ep(ep);
  if (next_ep) {
    xact_generic(rhport, next_ep, true, in_isr);
  }else {
    // no more pending
    atomic_flag_clear(&_hcd_data.busy);
  }
}

static void handle_xfer_done(uint8_t rhport, bool in_isr) {
  const uint8_t hrsl = reg_read(rhport, HRSL_ADDR, in_isr);
  const uint8_t hresult = hrsl & HRSL_RESULT_MASK;
  const uint8_t ep_num = _hcd_data.hxfr_bm.ep_num;
  const uint8_t hxfr_type = _hcd_data.hxfr & 0xf0;
  const uint8_t ep_dir = ((hxfr_type & HXFR_SETUP) || (hxfr_type & HXFR_OUT_NIN)) ? 0 : 1;

  max3421_ep_t *ep = find_opened_ep(_hcd_data.peraddr, ep_num, ep_dir);
  TU_VERIFY(ep, );

  xfer_result_t xfer_result;
  switch(hresult) {
    case HRSL_NAK:
      if (ep->state == EP_STATE_ABORTING) {
        ep->state = EP_STATE_IDLE;
      } else {
        if (ep_num == 0) {
          // control endpoint -> retry immediately and return
          hxfr_write(rhport, _hcd_data.hxfr, in_isr);
          return;
        }
        if (EP_STATE_ATTEMPT_1 <= ep->state && ep->state < EP_STATE_ATTEMPT_MAX) {
          ep->state++;
        }
      }

      max3421_ep_t * next_ep = find_next_pending_ep(ep);
      if (ep == next_ep) {
        // this endpoint is only one pending -> retry immediately
        hxfr_write(rhport, _hcd_data.hxfr, in_isr);
      } else if (next_ep) {
        // switch to next pending endpoint
        xact_generic(rhport, next_ep, true, in_isr);
      } else {
        // no more pending in this frame -> clear busy
        atomic_flag_clear(&_hcd_data.busy);
      }
      return;

    case HRSL_BAD_REQ:
      // occurred when initialized without any pending transfer. Skip for now
      return;

    case HRSL_SUCCESS:
      xfer_result = XFER_RESULT_SUCCESS;
      break;

    case HRSL_STALL:
      xfer_result = XFER_RESULT_STALLED;
      break;

    default:
      TU_LOG3("HRSL: %02X\r\n", hrsl);
      xfer_result = XFER_RESULT_FAILED;
      break;
  }

  if (xfer_result != XFER_RESULT_SUCCESS) {
    xfer_complete_isr(rhport, ep, xfer_result, hrsl, in_isr);
    return;
  }

  if (ep_dir) {
    // IN transfer: fifo data is already received in RCVDAV IRQ

    // mark control handshake as complete
    if (hxfr_type & HXFR_HS) {
      ep->state = EP_STATE_COMPLETE;
    }

    // short packet or all bytes transferred
    if (ep->state == EP_STATE_COMPLETE) {
      xfer_complete_isr(rhport, ep, xfer_result, hrsl, in_isr);
    }else {
      hxfr_write(rhport, _hcd_data.hxfr, in_isr); // more to transfer
    }
  } else {
    // SETUP or OUT transfer

    // clear sndfifo owner since data is sent
    _hcd_data.sndfifo_owner.daddr = 0xff;
    _hcd_data.sndfifo_owner.hxfr = 0xff;

    uint8_t xact_len;

    if (hxfr_type & HXFR_SETUP) {
      xact_len = 8;
    } else if (hxfr_type & HXFR_HS) {
      xact_len = 0;
    } else {
      xact_len = _hcd_data.sndbc;
    }

    ep->xferred_len += xact_len;
    ep->buf += xact_len;

    if (xact_len < ep->packet_size || ep->xferred_len >= ep->total_len) {
      xfer_complete_isr(rhport, ep, xfer_result, hrsl, in_isr);
    } else {
      xact_out(rhport, ep, false, in_isr); // more to transfer
    }
  }
}

#if CFG_TUSB_DEBUG >= 3
void print_hirq(uint8_t hirq) {
  TU_LOG3_HEX(hirq);

  if (hirq & HIRQ_HXFRDN_IRQ)   TU_LOG3(" HXFRDN");
  if (hirq & HIRQ_FRAME_IRQ)    TU_LOG3(" FRAME");
  if (hirq & HIRQ_CONDET_IRQ)   TU_LOG3(" CONDET");
  if (hirq & HIRQ_SUSDN_IRQ)    TU_LOG3(" SUSDN");
  if (hirq & HIRQ_SNDBAV_IRQ)   TU_LOG3(" SNDBAV");
  if (hirq & HIRQ_RCVDAV_IRQ)   TU_LOG3(" RCVDAV");
  if (hirq & HIRQ_RWU_IRQ)      TU_LOG3(" RWU");
  if (hirq & HIRQ_BUSEVENT_IRQ) TU_LOG3(" BUSEVENT");

  TU_LOG3("\r\n");
}
#else
  #define print_hirq(hirq)
#endif

// Interrupt handler
void hcd_int_handler(uint8_t rhport, bool in_isr) {
  uint8_t hirq = reg_read(rhport, HIRQ_ADDR, in_isr) & _hcd_data.hien;
  if (!hirq) return;
//  print_hirq(hirq);

  if (hirq & HIRQ_FRAME_IRQ) {
    _hcd_data.frame_count++;

    // reset all endpoints nak counter, retry with 1st pending ep.
    max3421_ep_t* ep_retry = NULL;
    for (size_t i = 0; i < CFG_TUH_MAX3421_ENDPOINT_TOTAL; i++) {
      max3421_ep_t* ep = &_hcd_data.ep[i];
      if (ep->packet_size && ep->state > EP_STATE_ATTEMPT_1) {
        ep->state = EP_STATE_ATTEMPT_1;

        if (ep_retry == NULL) {
          ep_retry = ep;
        }
      }
    }

    // start usb transfer if not busy
    if (ep_retry != NULL && !atomic_flag_test_and_set(&_hcd_data.busy)) {
      xact_generic(rhport, ep_retry, true, in_isr);
    }
  }

  if (hirq & HIRQ_CONDET_IRQ) {
    handle_connect_irq(rhport, in_isr);
  }

  // queue more transfer in handle_xfer_done() can cause hirq to be set again while external IRQ may not catch and/or
  // not call this handler again. So we need to loop until all IRQ are cleared
  while (hirq & (HIRQ_RCVDAV_IRQ | HIRQ_HXFRDN_IRQ)) {
    if (hirq & HIRQ_RCVDAV_IRQ) {
      const uint8_t ep_num = _hcd_data.hxfr_bm.ep_num;
      max3421_ep_t* ep = find_opened_ep(_hcd_data.peraddr, ep_num, 1);
      uint8_t xact_len = 0;

      // RCVDAV_IRQ can trigger 2 times (dual buffered)
      while (hirq & HIRQ_RCVDAV_IRQ) {
        const uint8_t rcvbc = reg_read(rhport, RCVBC_ADDR, in_isr);
        xact_len = (uint8_t) tu_min16(rcvbc, ep->total_len - ep->xferred_len);
        if (xact_len) {
          hwfifo_receive(rhport, ep->buf, xact_len, in_isr);
          ep->buf += xact_len;
          ep->xferred_len += xact_len;
        }

        // ack RCVDVAV IRQ
        hirq_write(rhport, HIRQ_RCVDAV_IRQ, in_isr);
        hirq = reg_read(rhport, HIRQ_ADDR, in_isr);
      }

      if (xact_len < ep->packet_size || ep->xferred_len >= ep->total_len) {
        ep->state = EP_STATE_COMPLETE;
      }
    }

    if (hirq & HIRQ_HXFRDN_IRQ) {
      hirq_write(rhport, HIRQ_HXFRDN_IRQ, in_isr);
      handle_xfer_done(rhport, in_isr);
    }

    hirq = reg_read(rhport, HIRQ_ADDR, in_isr);
  }

  // clear all interrupt except SNDBAV_IRQ (never clear by us). Note RCVDAV_IRQ, HXFRDN_IRQ already clear while processing
  hirq &= (uint8_t) ~HIRQ_SNDBAV_IRQ;
  if (hirq) {
    hirq_write(rhport, hirq, in_isr);
  }
}

#endif
