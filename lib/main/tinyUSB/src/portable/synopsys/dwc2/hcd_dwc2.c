/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Ha Thach (tinyusb.org)
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

#if CFG_TUH_ENABLED && defined(TUP_USBIP_DWC2)

#if !(CFG_TUH_DWC2_SLAVE_ENABLE || CFG_TUH_DWC2_DMA_ENABLE)
#error DWC2 require either CFG_TUH_DWC2_SLAVE_ENABLE or CFG_TUH_DWC2_DMA_ENABLE to be enabled
#endif

// Debug level for DWC2
#define DWC2_DEBUG    2

#include "host/hcd.h"
#include "dwc2_common.h"

// Max number of endpoints application can open, can be larger than DWC2_CHANNEL_COUNT_MAX
#ifndef CFG_TUH_DWC2_ENDPOINT_MAX
#define CFG_TUH_DWC2_ENDPOINT_MAX 16
#endif

#define DWC2_CHANNEL_COUNT_MAX    16 // absolute max channel count
#define DWC2_CHANNEL_COUNT(_dwc2) tu_min8((_dwc2)->ghwcfg2_bm.num_host_ch + 1, DWC2_CHANNEL_COUNT_MAX)

TU_VERIFY_STATIC(CFG_TUH_DWC2_ENDPOINT_MAX <= 255, "currently only use 8-bit for index");

enum {
  HPRT_W1_MASK = HPRT_CONN_DETECT | HPRT_ENABLE | HPRT_ENABLE_CHANGE | HPRT_OVER_CURRENT_CHANGE | HPRT_SUSPEND
};

enum {
  HCD_XFER_ERROR_MAX = 3
};

enum {
  HCD_XFER_PERIOD_SPLIT_NYET_MAX = 3
};

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------

// Host driver struct for each opened endpoint
typedef struct {
  union {
    uint32_t hcchar;
    dwc2_channel_char_t hcchar_bm;
  };
  union {
    uint32_t hcsplt;
    dwc2_channel_split_t hcsplt_bm;
  };

  struct TU_ATTR_PACKED {
    uint32_t uframe_interval : 18; // micro-frame interval
    uint32_t speed    : 2;
    uint32_t next_pid : 2;
    uint32_t do_ping  : 1;
    // uint32_t : 9;
  };

  uint32_t uframe_countdown; // micro-frame count down to transfer for periodic, only need 18-bit

  uint8_t* buffer;
  uint16_t buflen;
} hcd_endpoint_t;

// Additional info for each channel when it is active
typedef struct {
  volatile bool allocated;
  uint8_t ep_id;
  struct TU_ATTR_PACKED {
    uint8_t err_count : 3;
    uint8_t period_split_nyet_count : 3;
    uint8_t halted_nyet : 1;
    uint8_t halted_sof_schedule : 1;
  };
  uint8_t result;

  uint16_t xferred_bytes;  // bytes that accumulate transferred though USB bus for the whole hcd_edpt_xfer(), which can
                           // be composed of multiple channel_xfer_start() (retry with NAK/NYET)
  uint16_t fifo_bytes;     // bytes written/read from/to FIFO (may not be transferred on USB bus).
} hcd_xfer_t;

typedef struct {
  hcd_xfer_t xfer[DWC2_CHANNEL_COUNT_MAX];
  hcd_endpoint_t edpt[CFG_TUH_DWC2_ENDPOINT_MAX];
} hcd_data_t;

hcd_data_t _hcd_data;

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------
TU_ATTR_ALWAYS_INLINE static inline tusb_speed_t hprt_speed_get(dwc2_regs_t* dwc2) {
  tusb_speed_t speed;
  switch(dwc2->hprt_bm.speed) {
    case HPRT_SPEED_HIGH: speed = TUSB_SPEED_HIGH; break;
    case HPRT_SPEED_FULL: speed = TUSB_SPEED_FULL; break;
    case HPRT_SPEED_LOW : speed = TUSB_SPEED_LOW ; break;
    default:
      speed = TUSB_SPEED_INVALID;
      TU_BREAKPOINT();
    break;
  }
  return speed;
}

TU_ATTR_ALWAYS_INLINE static inline bool dma_host_enabled(const dwc2_regs_t* dwc2) {
  (void) dwc2;
  // Internal DMA only
  return CFG_TUH_DWC2_DMA_ENABLE && dwc2->ghwcfg2_bm.arch == GHWCFG2_ARCH_INTERNAL_DMA;
}

#if CFG_TUH_MEM_DCACHE_ENABLE
bool hcd_dcache_clean(const void* addr, uint32_t data_size) {
  TU_VERIFY(addr && data_size);
  return dwc2_dcache_clean(addr, data_size);
}

bool hcd_dcache_invalidate(const void* addr, uint32_t data_size) {
  TU_VERIFY(addr && data_size);
  return dwc2_dcache_invalidate(addr, data_size);
}

bool hcd_dcache_clean_invalidate(const void* addr, uint32_t data_size) {
  TU_VERIFY(addr && data_size);
  return dwc2_dcache_clean_invalidate(addr, data_size);
}
#endif

// Allocate a channel for new transfer
TU_ATTR_ALWAYS_INLINE static inline uint8_t channel_alloc(dwc2_regs_t* dwc2) {
  const uint8_t max_channel = DWC2_CHANNEL_COUNT(dwc2);
  for (uint8_t ch_id = 0; ch_id < max_channel; ch_id++) {
    hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
    if (!xfer->allocated) {
      tu_memclr(xfer, sizeof(hcd_xfer_t));
      xfer->allocated = true;
      return ch_id;
    }
  }
  return TUSB_INDEX_INVALID_8;
}

// Check if is periodic (interrupt/isochronous)
TU_ATTR_ALWAYS_INLINE static inline bool edpt_is_periodic(uint8_t ep_type) {
  return ep_type == HCCHAR_EPTYPE_INTERRUPT || ep_type == HCCHAR_EPTYPE_ISOCHRONOUS;
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t req_queue_avail(const dwc2_regs_t* dwc2, bool is_period) {
  if (is_period) {
    return dwc2->hptxsts_bm.req_queue_available;
  } else {
    return dwc2->hnptxsts_bm.req_queue_available;
  }
}

TU_ATTR_ALWAYS_INLINE static inline void channel_dealloc(dwc2_regs_t* dwc2, uint8_t ch_id) {
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  xfer->allocated = false;
  dwc2->haintmsk &= ~TU_BIT(ch_id);
}

TU_ATTR_ALWAYS_INLINE static inline bool channel_disable(const dwc2_regs_t* dwc2, dwc2_channel_t* channel) {
  // disable also require request queue
  TU_ASSERT(req_queue_avail(dwc2, edpt_is_periodic(channel->hcchar_bm.ep_type)));
  channel->hcintmsk |= HCINT_HALTED;
  channel->hcchar |= HCCHAR_CHDIS | HCCHAR_CHENA; // must set both CHDIS and CHENA
  return true;
}

// attempt to send IN token to receive data
TU_ATTR_ALWAYS_INLINE static inline bool channel_send_in_token(const dwc2_regs_t* dwc2, dwc2_channel_t* channel) {
  TU_ASSERT(req_queue_avail(dwc2, edpt_is_periodic(channel->hcchar_bm.ep_type)));
  channel->hcchar |= HCCHAR_CHENA;
  return true;
}

// Find currently enabled channel. Note: EP0 is bidirectional
TU_ATTR_ALWAYS_INLINE static inline uint8_t channel_find_enabled(dwc2_regs_t* dwc2, uint8_t dev_addr, uint8_t ep_num, uint8_t ep_dir) {
  const uint8_t max_channel = DWC2_CHANNEL_COUNT(dwc2);
  for (uint8_t ch_id = 0; ch_id < max_channel; ch_id++) {
    if (_hcd_data.xfer[ch_id].allocated) {
      const dwc2_channel_char_t hcchar_bm = dwc2->channel[ch_id].hcchar_bm;
      if (hcchar_bm.dev_addr == dev_addr && hcchar_bm.ep_num == ep_num && (ep_num == 0 || hcchar_bm.ep_dir == ep_dir)) {
        return ch_id;
      }
    }
  }
  return TUSB_INDEX_INVALID_8;
}


// Allocate a new endpoint
TU_ATTR_ALWAYS_INLINE static inline uint8_t edpt_alloc(void) {
  for (uint32_t i = 0; i < CFG_TUH_DWC2_ENDPOINT_MAX; i++) {
    hcd_endpoint_t* edpt = &_hcd_data.edpt[i];
    if (edpt->hcchar_bm.enable == 0) {
      tu_memclr(edpt, sizeof(hcd_endpoint_t));
      edpt->hcchar_bm.enable = 1;
      return i;
    }
  }
  return TUSB_INDEX_INVALID_8;
}

// Find a endpoint that is opened previously with hcd_edpt_open()
// Note: EP0 is bidirectional
TU_ATTR_ALWAYS_INLINE static inline uint8_t edpt_find_opened(uint8_t dev_addr, uint8_t ep_num, uint8_t ep_dir) {
  for (uint8_t i = 0; i < (uint8_t)CFG_TUH_DWC2_ENDPOINT_MAX; i++) {
    const dwc2_channel_char_t* hcchar_bm = &_hcd_data.edpt[i].hcchar_bm;
    if (hcchar_bm->enable && hcchar_bm->dev_addr == dev_addr &&
        hcchar_bm->ep_num == ep_num && (ep_num == 0 || hcchar_bm->ep_dir == ep_dir)) {
      return i;
    }
  }
  return TUSB_INDEX_INVALID_8;
}

TU_ATTR_ALWAYS_INLINE static inline uint16_t cal_packet_count(uint16_t len, uint16_t ep_size) {
  if (len == 0) {
    return 1;
  } else {
    return tu_div_ceil(len, ep_size);
  }
}

TU_ATTR_ALWAYS_INLINE static inline uint8_t cal_next_pid(uint8_t pid, uint8_t packet_count) {
  if (packet_count & 0x01) {
    return pid ^ 0x02; // toggle DATA0 and DATA1
  } else {
    return pid;
  }
}

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------

/* USB Data FIFO Layout

  The FIFO is split up into
  - EPInfo: for storing DMA metadata (check dcd_dwc2.c for more details)
  - 1 RX FIFO: for receiving data
  - 1 TX FIFO for non-periodic (NPTX)
  - 1 TX FIFO for periodic (PTX)

  We allocated TX FIFO from top to bottom (using top pointer), this to allow the RX FIFO to grow dynamically which is
  possible since the free space is located between the RX and TX FIFOs.

   ----------------- ep_fifo_size
  |    HCDMAn    |
  |--------------|-- gdfifocfg.EPINFOBASE (max is ghwcfg3.dfifo_depth)
  | Non-Periodic |
  |   TX FIFO    |
  |--------------|--- GNPTXFSIZ.addr (fixed size)
  |   Periodic   |
  |   TX FIFO    |
  |--------------|--- HPTXFSIZ.addr (expandable downward)
  |    FREE      |
  |              |
  |--------------|-- GRXFSIZ (expandable upward)
  |  RX FIFO     |
  ---------------- 0
*/

/* Programming Guide 2.1.2 FIFO RAM allocation
 * RX
 * - Largest-EPsize/4 + 2 (status info). recommended x2 if high bandwidth or multiple ISO are used.
 * - 2 for transfer complete and channel halted status
 * - 1 for each Control/Bulk out endpoint to Handle NAK/NYET (i.e max is number of host channel)
 *
 * TX non-periodic (NPTX)
 * - At least largest-EPsize/4, recommended x2
 *
 * TX periodic (PTX)
 * - At least largest-EPsize*MulCount/4 (MulCount up to 3 for high-bandwidth ISO/interrupt)
*/
static void dfifo_host_init(uint8_t rhport) {
  const dwc2_controller_t* dwc2_controller = &_dwc2_controller[rhport];
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  // Scatter/Gather DMA mode is not yet supported. Buffer DMA only need 1 words per channel
  const bool is_dma = dma_host_enabled(dwc2);
  uint16_t dfifo_top = dwc2_controller->ep_fifo_size/4;
  if (is_dma) {
    dfifo_top -= dwc2->ghwcfg2_bm.num_host_ch;
  }

  // fixed allocation for now, improve later:
    // - ptx_largest is limited to 256 for FS since most FS core only has 1024 bytes total
  bool is_highspeed = dwc2_core_is_highspeed(dwc2, TUSB_ROLE_HOST);
  uint32_t nptx_largest = is_highspeed ? TUSB_EPSIZE_BULK_HS/4 : TUSB_EPSIZE_BULK_FS/4;
  uint32_t ptx_largest = is_highspeed ? TUSB_EPSIZE_ISO_HS_MAX/4 : 256/4;

  uint16_t nptxfsiz = 2 * nptx_largest;
  uint16_t rxfsiz = 2 * (ptx_largest + 2) + dwc2->ghwcfg2_bm.num_host_ch;
  TU_ASSERT(dfifo_top >= (nptxfsiz + rxfsiz),);
  uint16_t ptxfsiz = dfifo_top - (nptxfsiz + rxfsiz);

  dwc2->gdfifocfg = (dfifo_top << GDFIFOCFG_EPINFOBASE_SHIFT) | dfifo_top;

  dfifo_top -= rxfsiz;
  dwc2->grxfsiz = rxfsiz;

  dfifo_top -= nptxfsiz;
  dwc2->gnptxfsiz = tu_u32_from_u16(nptxfsiz, dfifo_top);

  dfifo_top -= ptxfsiz;
  dwc2->hptxfsiz = tu_u32_from_u16(ptxfsiz, dfifo_top);
}

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+

// optional hcd configuration, called by tuh_configure()
bool hcd_configure(uint8_t rhport, uint32_t cfg_id, const void* cfg_param) {
  (void) rhport;
  (void) cfg_id;
  (void) cfg_param;

  return true;
}

// Initialize controller to host mode
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  tu_memclr(&_hcd_data, sizeof(_hcd_data));

  // Core Initialization
  const bool is_highspeed = dwc2_core_is_highspeed(dwc2, TUSB_ROLE_HOST);
  const bool is_dma = dma_host_enabled(dwc2);
  TU_ASSERT(dwc2_core_init(rhport, is_highspeed, is_dma));

  //------------- 3.1 Host Initialization -------------//

  // work at max supported speed
  dwc2->hcfg &= ~HCFG_FSLS_ONLY;

  // Enable HFIR reload
  if (dwc2->gsnpsid >= DWC2_CORE_REV_2_92a) {
    dwc2->hfir |= HFIR_RELOAD_CTRL;
  }

  // force host mode and wait for mode switch
  dwc2->gusbcfg = (dwc2->gusbcfg & ~GUSBCFG_FDMOD) | GUSBCFG_FHMOD;
  while ((dwc2->gintsts & GINTSTS_CMOD) != GINTSTS_CMODE_HOST) {}

  // configure fixed-allocated fifo scheme
  dfifo_host_init(rhport);

  dwc2->hprt = HPRT_W1_MASK; // clear all write-1-clear bits
  dwc2->hprt = HPRT_POWER; // turn on VBUS

  // Enable required interrupts
  dwc2->gintmsk |= GINTSTS_OTGINT | GINTSTS_CONIDSTSCHNG | GINTSTS_HPRTINT | GINTSTS_HCINT;

  // NPTX can hold at least 2 packet, change interrupt level to half-empty
  uint32_t gahbcfg = dwc2->gahbcfg & ~GAHBCFG_TX_FIFO_EPMTY_LVL;
  gahbcfg |= GAHBCFG_GINT;   // Enable global interrupt
  dwc2->gahbcfg = gahbcfg;

  return true;
}

// Enable USB interrupt
void hcd_int_enable (uint8_t rhport) {
  dwc2_int_set(rhport, TUSB_ROLE_HOST, true);
}

// Disable USB interrupt
void hcd_int_disable(uint8_t rhport) {
  dwc2_int_set(rhport, TUSB_ROLE_HOST, false);
}

// Get frame number (1ms)
uint32_t hcd_frame_number(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  return dwc2->hfnum & HFNUM_FRNUM_Msk;
}

//--------------------------------------------------------------------+
// Port API
//--------------------------------------------------------------------+

// Get the current connect status of roothub port
bool hcd_port_connect_status(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  return dwc2->hprt & HPRT_CONN_STATUS;
}

// Reset USB bus on the port. Return immediately, bus reset sequence may not be complete.
// Some port would require hcd_port_reset_end() to be invoked after 10ms to complete the reset sequence.
void hcd_port_reset(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  uint32_t hprt = dwc2->hprt & ~HPRT_W1_MASK;
  hprt |= HPRT_RESET;
  dwc2->hprt = hprt;
}

// Complete bus reset sequence, may be required by some controllers
void hcd_port_reset_end(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  uint32_t hprt = dwc2->hprt & ~HPRT_W1_MASK; // skip w1c bits
  hprt &= ~HPRT_RESET;
  dwc2->hprt = hprt;
}

// Get port link speed
tusb_speed_t hcd_port_speed_get(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const tusb_speed_t speed = hprt_speed_get(dwc2);
  return speed;
}

// HCD closes all opened endpoints belong to this device
void hcd_device_close(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  for (uint8_t i = 0; i < (uint8_t) CFG_TUH_DWC2_ENDPOINT_MAX; i++) {
    hcd_endpoint_t* edpt = &_hcd_data.edpt[i];
    if (edpt->hcchar_bm.enable && edpt->hcchar_bm.dev_addr == dev_addr) {
      tu_memclr(edpt, sizeof(hcd_endpoint_t));
    }
  }
}

//--------------------------------------------------------------------+
// Endpoints API
//--------------------------------------------------------------------+

// Open an endpoint
bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, const tusb_desc_endpoint_t* desc_ep) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const tusb_speed_t rh_speed = hprt_speed_get(dwc2);

  hcd_devtree_info_t devtree_info;
  hcd_devtree_get_info(dev_addr, &devtree_info);

  // find a free endpoint
  const uint8_t ep_id = edpt_alloc();
  TU_ASSERT(ep_id < CFG_TUH_DWC2_ENDPOINT_MAX);
  hcd_endpoint_t* edpt = &_hcd_data.edpt[ep_id];

  dwc2_channel_char_t* hcchar_bm = &edpt->hcchar_bm;
  hcchar_bm->ep_size         = tu_edpt_packet_size(desc_ep);
  hcchar_bm->ep_num          = tu_edpt_number(desc_ep->bEndpointAddress);
  hcchar_bm->ep_dir          = tu_edpt_dir(desc_ep->bEndpointAddress);
  hcchar_bm->low_speed_dev   = (devtree_info.speed == TUSB_SPEED_LOW) ? 1 : 0;
  hcchar_bm->ep_type         = desc_ep->bmAttributes.xfer; // ep_type matches TUSB_XFER_*
  hcchar_bm->err_multi_count = 0;
  hcchar_bm->dev_addr        = dev_addr;
  hcchar_bm->odd_frame       = 0;
  hcchar_bm->disable         = 0;
  hcchar_bm->enable          = 1;

  dwc2_channel_split_t* hcsplt_bm = &edpt->hcsplt_bm;
  hcsplt_bm->hub_port        = devtree_info.hub_port;
  hcsplt_bm->hub_addr        = devtree_info.hub_addr;
  hcsplt_bm->xact_pos        = 0;
  hcsplt_bm->split_compl     = 0;
  hcsplt_bm->split_en        = (rh_speed == TUSB_SPEED_HIGH && devtree_info.speed != TUSB_SPEED_HIGH) ? 1 : 0;

  edpt->speed = devtree_info.speed;
  edpt->next_pid = HCTSIZ_PID_DATA0;
  if (desc_ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) {
    edpt->uframe_interval = 1 << (desc_ep->bInterval - 1);
    if (devtree_info.speed == TUSB_SPEED_FULL) {
      edpt->uframe_interval <<= 3;
    }
  } else if (desc_ep->bmAttributes.xfer == TUSB_XFER_INTERRUPT) {
    if (devtree_info.speed == TUSB_SPEED_HIGH) {
      edpt->uframe_interval = 1 << (desc_ep->bInterval - 1);
    } else {
      edpt->uframe_interval = desc_ep->bInterval << 3;
    }
  }

  return true;
}

// clean up channel after part of transfer is done but the whole urb is not complete
static void channel_xfer_out_wrapup(dwc2_regs_t* dwc2, uint8_t ch_id) {
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  dwc2_channel_t* channel = &dwc2->channel[ch_id];
  hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];

  edpt->next_pid = channel->hctsiz_bm.pid; // save PID

  /* Since hctsiz.xfersize field reflects the number of bytes transferred via the AHB, not the USB)
   * For IN: we can use hctsiz.xfersize as remaining bytes.
   * For OUT: Must use the hctsiz.pktcnt field to determine how much data has been transferred. This field reflects the
   * number of packets that have been transferred via the USB. This is always an integral number of packets if the
   * transfer was halted before its normal completion.
   */
  const uint16_t remain_packets = channel->hctsiz_bm.packet_count;
  const uint16_t total_packets = cal_packet_count(edpt->buflen, channel->hcchar_bm.ep_size);
  const uint16_t actual_bytes = (total_packets - remain_packets) * channel->hcchar_bm.ep_size;

  xfer->fifo_bytes = 0;
  xfer->xferred_bytes += actual_bytes;
  edpt->buffer += actual_bytes;
  edpt->buflen -= actual_bytes;
}

static bool channel_xfer_start(dwc2_regs_t* dwc2, uint8_t ch_id) {
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];
  dwc2_channel_char_t* hcchar_bm = &edpt->hcchar_bm;
  dwc2_channel_t* channel = &dwc2->channel[ch_id];
  bool const is_period = edpt_is_periodic(hcchar_bm->ep_type);

  // clear previous state
  xfer->fifo_bytes = 0;

  // hchar: restore but don't enable yet
  if (is_period) {
    hcchar_bm->odd_frame = 1 - (dwc2->hfnum & 1);   // transfer on next frame
  }
  channel->hcchar = (edpt->hcchar & ~HCCHAR_CHENA);

  // hctsiz: zero length packet still count as 1
  const uint16_t packet_count = cal_packet_count(edpt->buflen, hcchar_bm->ep_size);
  uint32_t hctsiz = (edpt->next_pid << HCTSIZ_PID_Pos) | (packet_count << HCTSIZ_PKTCNT_Pos) | edpt->buflen;
  if (edpt->do_ping && edpt->speed == TUSB_SPEED_HIGH &&
     edpt->next_pid != HCTSIZ_PID_SETUP && hcchar_bm->ep_dir == TUSB_DIR_OUT) {
    hctsiz |= HCTSIZ_DOPING;
  }
  channel->hctsiz = hctsiz;
  edpt->do_ping = 0;

  // pre-calculate next PID based on packet count, adjusted in transfer complete interrupt if short packet
  if (hcchar_bm->ep_num == 0) {
    edpt->next_pid = HCTSIZ_PID_DATA1; // control data and status stage always start with DATA1
  } else {
    edpt->next_pid = cal_next_pid(edpt->next_pid, packet_count);
  }

  channel->hcsplt = edpt->hcsplt;
  channel->hcint = 0xFFFFFFFFU; // clear all channel interrupts

  if (dma_host_enabled(dwc2)) {
    uint32_t hcintmsk = HCINT_HALTED;
    channel->hcintmsk = hcintmsk;
    dwc2->haintmsk |= TU_BIT(ch_id);

    channel->hcdma = (uint32_t) edpt->buffer;

    if (hcchar_bm->ep_dir == TUSB_DIR_IN) {
      channel_send_in_token(dwc2, channel);
    } else {
      hcd_dcache_clean(edpt->buffer, edpt->buflen);
      channel->hcchar |= HCCHAR_CHENA;
    }
  } else {
    uint32_t hcintmsk = HCINT_NAK | HCINT_XACT_ERR | HCINT_STALL | HCINT_XFER_COMPLETE | HCINT_DATATOGGLE_ERR;
    if (hcchar_bm->ep_dir == TUSB_DIR_IN) {
      hcintmsk |= HCINT_BABBLE_ERR | HCINT_DATATOGGLE_ERR | HCINT_ACK;
    } else {
      hcintmsk |= HCINT_NYET;
      if (edpt->hcsplt_bm.split_en) {
        hcintmsk |= HCINT_ACK;
      }
    }
    channel->hcintmsk = hcintmsk;
    dwc2->haintmsk |= TU_BIT(ch_id);

    // enable channel for slave mode:
    // - OUT: it will enable corresponding FIFO channel
    // - IN : it will write an IN request to the Non-periodic Request Queue, this will have dwc2 trying to send
    // IN Token. If we got NAK, we have to re-enable the channel again in the interrupt. Due to the way usbh stack only
    // call hcd_edpt_xfer() once, we will need to manage de-allocate/re-allocate IN channel dynamically.
    if (hcchar_bm->ep_dir == TUSB_DIR_IN) {
      channel_send_in_token(dwc2, channel);
    } else {
      channel->hcchar |= HCCHAR_CHENA;
      if (edpt->buflen > 0) {
        // To prevent conflict with other channel, we will enable periodic/non-periodic FIFO empty interrupt accordingly
        // And write packet in the interrupt handler
        dwc2->gintmsk |= (is_period ? GINTSTS_PTX_FIFO_EMPTY : GINTSTS_NPTX_FIFO_EMPTY);
      }
    }
  }

  return true;
}

// kick-off transfer with an endpoint
static bool edpt_xfer_kickoff(dwc2_regs_t* dwc2, uint8_t ep_id) {
  uint8_t ch_id = channel_alloc(dwc2);
  TU_ASSERT(ch_id < 16); // all channel are in used
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  xfer->ep_id = ep_id;
  xfer->result = XFER_RESULT_INVALID;

  return channel_xfer_start(dwc2, ch_id);
}

// Submit a transfer, when complete hcd_event_xfer_complete() must be invoked
bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const uint8_t ep_num = tu_edpt_number(ep_addr);
  const uint8_t ep_dir = tu_edpt_dir(ep_addr);

  uint8_t ep_id = edpt_find_opened(dev_addr, ep_num, ep_dir);
  TU_ASSERT(ep_id < CFG_TUH_DWC2_ENDPOINT_MAX);
  hcd_endpoint_t* edpt = &_hcd_data.edpt[ep_id];

  edpt->buffer = buffer;
  edpt->buflen = buflen;

  if (ep_num == 0) {
    // update ep_dir since control endpoint can switch direction
    edpt->hcchar_bm.ep_dir = ep_dir;
  }

  return edpt_xfer_kickoff(dwc2, ep_id);
}

// Abort a queued transfer. Note: it can only abort transfer that has not been started
// Return true if a queued transfer is aborted, false if there is no transfer to abort
bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const uint8_t ep_num = tu_edpt_number(ep_addr);
  const uint8_t ep_dir = tu_edpt_dir(ep_addr);
  const uint8_t ep_id = edpt_find_opened(dev_addr, ep_num, ep_dir);
  TU_VERIFY(ep_id < CFG_TUH_DWC2_ENDPOINT_MAX);

  // hcd_int_disable(rhport);

  // Find enabled channeled and disable it, channel will be de-allocated in the interrupt handler
  const uint8_t ch_id = channel_find_enabled(dwc2, dev_addr, ep_num, ep_dir);
  if (ch_id < 16) {
    dwc2_channel_t* channel = &dwc2->channel[ch_id];
    channel_disable(dwc2, channel);
  }

  // hcd_int_enable(rhport);

  return true;
}

// Submit a special transfer to send 8-byte Setup Packet, when complete hcd_event_xfer_complete() must be invoked
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, const uint8_t setup_packet[8]) {
  uint8_t ep_id = edpt_find_opened(dev_addr, 0, TUSB_DIR_OUT);
  TU_ASSERT(ep_id < CFG_TUH_DWC2_ENDPOINT_MAX); // no opened endpoint
  hcd_endpoint_t* edpt = &_hcd_data.edpt[ep_id];
  edpt->next_pid = HCTSIZ_PID_SETUP;

  return hcd_edpt_xfer(rhport, dev_addr, 0, (uint8_t*)(uintptr_t) setup_packet, 8);
}

// clear stall, data toggle is also reset to DATA0
bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  const uint8_t ep_num = tu_edpt_number(ep_addr);
  const uint8_t ep_dir = tu_edpt_dir(ep_addr);
  const uint8_t ep_id = edpt_find_opened(dev_addr, ep_num, ep_dir);
  TU_VERIFY(ep_id < CFG_TUH_DWC2_ENDPOINT_MAX);
  hcd_endpoint_t* edpt = &_hcd_data.edpt[ep_id];

  edpt->next_pid = HCTSIZ_PID_DATA0;

  return true;
}

//--------------------------------------------------------------------
// HCD Event Handler
//--------------------------------------------------------------------
static void channel_xfer_in_retry(dwc2_regs_t* dwc2, uint8_t ch_id, uint32_t hcint) {
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  dwc2_channel_t* channel = &dwc2->channel[ch_id];
  hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];

  if (edpt_is_periodic(channel->hcchar_bm.ep_type)){
    // retry immediately for periodic split NYET if we haven't reach max retry
    if (channel->hcsplt_bm.split_en && channel->hcsplt_bm.split_compl && (hcint & HCINT_NYET || xfer->halted_nyet)) {
      xfer->period_split_nyet_count++;
      xfer->halted_nyet = 0;
      if (xfer->period_split_nyet_count < HCD_XFER_PERIOD_SPLIT_NYET_MAX) {
        channel->hcchar_bm.odd_frame = 1 - (dwc2->hfnum & 1); // transfer on next frame
        channel_send_in_token(dwc2, channel);
        return;
      } else {
        // too many NYET, de-allocate channel with below code
        xfer->period_split_nyet_count = 0;
      }
    }

    // for periodic, de-allocate channel, enable SOF set frame counter for later transfer
    edpt->next_pid = channel->hctsiz_bm.pid; // save PID
    edpt->uframe_countdown = edpt->uframe_interval;
    dwc2->gintmsk |= GINTSTS_SOF;

    if (hcint & HCINT_HALTED) {
      // already halted, de-allocate channel (called from DMA isr)
      channel_dealloc(dwc2, ch_id);
    } else {
      // disable channel first if not halted (called slave isr)
      xfer->halted_sof_schedule = 1;
      channel_disable(dwc2, channel);
    }
  } else {
    // for control/bulk: retry immediately
    channel_send_in_token(dwc2, channel);
  }
}

#if CFG_TUSB_DEBUG
TU_ATTR_ALWAYS_INLINE static inline void print_hcint(uint32_t hcint) {
  const char* str[] = {
    "XFRC", "HALTED", "AHBERR", "STALL",
    "NAK", "ACK", "NYET", "XERR",
    "BBLERR", "FRMOR", "DTERR", "BNA",
    "XCSERR", "DESC_LST"
  };

  for(uint32_t i=0; i<14; i++) {
    if (hcint & TU_BIT(i)) {
      TU_LOG1("%s ", str[i]);
    }
  }
  TU_LOG1("\r\n");
}
#endif

#if CFG_TUH_DWC2_SLAVE_ENABLE
static void handle_rxflvl_irq(uint8_t rhport) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  // Pop control word off FIFO
  const dwc2_grxstsp_t grxstsp_bm = dwc2->grxstsp_bm;
  const uint8_t ch_id = grxstsp_bm.ep_ch_num;

  switch (grxstsp_bm.packet_status) {
    case GRXSTS_PKTSTS_RX_DATA: {
      // In packet received, pop this entry --> ACK interrupt
      const uint16_t byte_count = grxstsp_bm.byte_count;
      hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
      TU_ASSERT(xfer->ep_id < CFG_TUH_DWC2_ENDPOINT_MAX,);
      hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];

      if (byte_count) {
        dfifo_read_packet(dwc2, edpt->buffer + xfer->xferred_bytes, byte_count);
        xfer->xferred_bytes += byte_count;
        xfer->fifo_bytes = byte_count;
      }
      break;
    }

    case GRXSTS_PKTSTS_RX_COMPLETE:
      // In transfer complete: After this entry is popped from the rx FIFO, dwc2 asserts a Transfer Completed
      // interrupt --> handle_channel_irq()
      break;

    case GRXSTS_PKTSTS_HOST_DATATOGGLE_ERR:
      TU_ASSERT(0, ); // maybe try to change DToggle
      break;

    case GRXSTS_PKTSTS_HOST_CHANNEL_HALTED:
      // triggered when channel.hcchar_bm.disable is set
      // TODO handle later
      break;

    default: break; // ignore other status
  }
}

// return true if there is still pending data and need more ISR
static bool handle_txfifo_empty(dwc2_regs_t* dwc2, bool is_periodic) {
  // Use period txsts for both p/np to get request queue space available (1-bit difference, it is small enough)
  volatile dwc2_hptxsts_t* txsts_bm = (volatile dwc2_hptxsts_t*) (is_periodic ? &dwc2->hptxsts : &dwc2->hnptxsts);

  const uint8_t max_channel = DWC2_CHANNEL_COUNT(dwc2);
  for (uint8_t ch_id = 0; ch_id < max_channel; ch_id++) {
    dwc2_channel_t* channel = &dwc2->channel[ch_id];
    // skip writing to FIFO if channel is expecting halted.
    if (!(channel->hcintmsk & HCINT_HALTED) && (channel->hcchar_bm.ep_dir == TUSB_DIR_OUT)) {
      hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
      TU_ASSERT(xfer->ep_id < CFG_TUH_DWC2_ENDPOINT_MAX);
      hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];

      const uint16_t remain_packets = channel->hctsiz_bm.packet_count;
      for (uint16_t i = 0; i < remain_packets; i++) {
        const uint16_t remain_bytes = edpt->buflen - xfer->fifo_bytes;
        const uint16_t xact_bytes = tu_min16(remain_bytes, channel->hcchar_bm.ep_size);

        // skip if there is not enough space in FIFO and RequestQueue.
        // Packet's last word written to FIFO will trigger a request queue
        if ((xact_bytes > (txsts_bm->fifo_available << 2)) || (txsts_bm->req_queue_available == 0)) {
          return true;
        }

        dfifo_write_packet(dwc2, ch_id, edpt->buffer + xfer->fifo_bytes, xact_bytes);
        xfer->fifo_bytes += xact_bytes;
      }
    }
  }

  return false; // no channel has pending data
}

static bool handle_channel_in_slave(dwc2_regs_t* dwc2, uint8_t ch_id, uint32_t hcint) {
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  dwc2_channel_t* channel = &dwc2->channel[ch_id];
  hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];
  bool is_done = false;

  // if (channel->hcsplt_bm.split_en) {
  // if (edpt->hcchar_bm.ep_num == 1) {
  //   TU_LOG1("Frame %u, ch %u: ep %u, hcint 0x%04lX ", dwc2->hfnum_bm.num, ch_id, channel->hcchar_bm.ep_num, hcint);
  //   print_hcint(hcint);
  // }

  if (hcint & HCINT_XFER_COMPLETE) {
    if (edpt->hcchar_bm.ep_num != 0) {
      edpt->next_pid = channel->hctsiz_bm.pid; // save pid (already toggled)
    }

    const uint16_t remain_packets = channel->hctsiz_bm.packet_count;
    if (channel->hcsplt_bm.split_en && remain_packets && xfer->fifo_bytes == edpt->hcchar_bm.ep_size) {
      // Split can only complete 1 transaction (up to 1 packet) at a time, schedule more
      channel->hcsplt_bm.split_compl = 0;
    } else {
      xfer->result = XFER_RESULT_SUCCESS;
    }

    channel_disable(dwc2, channel);
  } else if (hcint & (HCINT_XACT_ERR | HCINT_BABBLE_ERR | HCINT_STALL)) {
    if (hcint & HCINT_STALL) {
      xfer->result = XFER_RESULT_STALLED;
    } else if (hcint & HCINT_BABBLE_ERR) {
      xfer->result = XFER_RESULT_FAILED;
    } else if (hcint & HCINT_XACT_ERR) {
      xfer->err_count++;
      channel->hcintmsk |= HCINT_ACK;
    }

    channel_disable(dwc2, channel);
  } else if (hcint & HCINT_NYET) {
    // restart complete split
    channel->hcsplt_bm.split_compl = 1;
    xfer->halted_nyet = 1;
    channel_disable(dwc2, channel);
  } else if (hcint & HCINT_NAK) {
    // NAK received, re-enable channel if request queue is available
    if (channel->hcsplt_bm.split_en) {
      channel->hcsplt_bm.split_compl = 0; // restart with start-split
    }

    channel_disable(dwc2, channel);
  } else if (hcint & HCINT_ACK) {
    xfer->err_count = 0;

    if (channel->hcsplt_bm.split_en) {
      if (!channel->hcsplt_bm.split_compl) {
        // start split is ACK --> do complete split
        channel->hcintmsk |= HCINT_NYET;
        channel->hcsplt_bm.split_compl = 1;
        channel_send_in_token(dwc2, channel);
      } else {
        // do nothing for complete split with DATA, this will trigger XferComplete and handled there
      }
    } else {
      // ACK with data
      const uint16_t remain_packets = channel->hctsiz_bm.packet_count;
      if (remain_packets) {
        // still more packet to receive, also reset to start split
        channel->hcsplt_bm.split_compl = 0;
        channel_send_in_token(dwc2, channel);
      }
    }
  } else if (hcint & HCINT_HALTED) {
    channel->hcintmsk &= ~HCINT_HALTED;
    if (xfer->halted_sof_schedule) {
      // de-allocate channel but does not complete xfer, we schedule it in the SOF interrupt
      channel_dealloc(dwc2, ch_id);
    } else if (xfer->result != XFER_RESULT_INVALID) {
      is_done = true;
    } else if (xfer->err_count == HCD_XFER_ERROR_MAX) {
      xfer->result = XFER_RESULT_FAILED;
      is_done = true;
    } else {
      // got here due to NAK or NYET
      channel_xfer_in_retry(dwc2, ch_id, hcint);
    }
  } else if (hcint & HCINT_DATATOGGLE_ERR) {
    xfer->err_count = 0;
    TU_ASSERT(false);
  }
  return is_done;
}

static bool handle_channel_out_slave(dwc2_regs_t* dwc2, uint8_t ch_id, uint32_t hcint) {
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  dwc2_channel_t* channel = &dwc2->channel[ch_id];
  hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];
  bool is_done = false;

  if (hcint & HCINT_XFER_COMPLETE) {
    is_done = true;
    xfer->result = XFER_RESULT_SUCCESS;
    channel->hcintmsk &= ~HCINT_ACK;
  } else if (hcint & HCINT_STALL) {
    xfer->result = XFER_RESULT_STALLED;
    channel_disable(dwc2, channel);
  } else if (hcint & HCINT_NYET) {
    xfer->err_count = 0;
    if (channel->hcsplt_bm.split_en) {
      // retry complete split
      channel->hcsplt_bm.split_compl = 1;
      channel->hcchar |= HCCHAR_CHENA;
    } else {
      edpt->do_ping = 1;
      channel_xfer_out_wrapup(dwc2, ch_id);
      channel_disable(dwc2, channel);
    }
  } else if (hcint & (HCINT_NAK | HCINT_XACT_ERR)) {
    // clean up transfer so far, disable and start again later
    channel_xfer_out_wrapup(dwc2, ch_id);
    channel_disable(dwc2, channel);
    if (hcint & HCINT_XACT_ERR) {
      xfer->err_count++;
      channel->hcintmsk |= HCINT_ACK;
    } else {
      // NAK disable channel to flush all posted request and try again
      edpt->do_ping = 1;
      xfer->err_count = 0;
    }
  } else if (hcint & HCINT_HALTED) {
    channel->hcintmsk &= ~HCINT_HALTED;
    if (xfer->result != XFER_RESULT_INVALID) {
      is_done = true;
    } else if (xfer->err_count == HCD_XFER_ERROR_MAX) {
      xfer->result = XFER_RESULT_FAILED;
      is_done = true;
    } else {
      // Got here due to NAK or NYET
      TU_ASSERT(channel_xfer_start(dwc2, ch_id));
    }
  } else if (hcint & HCINT_ACK) {
    xfer->err_count = 0;
    channel->hcintmsk &= ~HCINT_ACK;
    if (channel->hcsplt_bm.split_en && !channel->hcsplt_bm.split_compl) {
      // start split is ACK --> do complete split
      channel->hcsplt_bm.split_compl = 1;
      channel->hcchar |= HCCHAR_CHENA;
    }
  }

  if (is_done) {
    xfer->xferred_bytes += xfer->fifo_bytes;
    xfer->fifo_bytes = 0;
  }

  return is_done;
}
#endif

#if CFG_TUH_DWC2_DMA_ENABLE
static bool handle_channel_in_dma(dwc2_regs_t* dwc2, uint8_t ch_id, uint32_t hcint) {
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  dwc2_channel_t* channel = &dwc2->channel[ch_id];
  hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];

  bool is_done = false;

  // TU_LOG1("in  hcint = %02lX\r\n", hcint);

  if (hcint & HCINT_HALTED) {
    if (hcint & (HCINT_XFER_COMPLETE | HCINT_STALL | HCINT_BABBLE_ERR)) {
      const uint16_t remain_bytes = (uint16_t) channel->hctsiz_bm.xfer_size;
      const uint16_t remain_packets = channel->hctsiz_bm.packet_count;
      const uint16_t actual_len = edpt->buflen - remain_bytes;
      xfer->xferred_bytes += actual_len;

      is_done = true;

      if (hcint & HCINT_STALL) {
        xfer->result = XFER_RESULT_STALLED;
      } else if (hcint & HCINT_BABBLE_ERR) {
        xfer->result = XFER_RESULT_FAILED;
      } else if (channel->hcsplt_bm.split_en && remain_packets && actual_len == edpt->hcchar_bm.ep_size) {
        // Split can only complete 1 transaction (up to 1 packet) at a time, schedule more
        is_done = false;
        edpt->buffer += actual_len;
        edpt->buflen -= actual_len;

        channel->hcsplt_bm.split_compl = 0;
        channel_xfer_in_retry(dwc2, ch_id, hcint);
      } else {
        xfer->result = XFER_RESULT_SUCCESS;
      }

      xfer->err_count = 0;
      channel->hcintmsk &= ~HCINT_ACK;
    } else if (hcint & HCINT_XACT_ERR) {
      xfer->err_count++;
      if (xfer->err_count >=  HCD_XFER_ERROR_MAX) {
        is_done = true;
        xfer->result = XFER_RESULT_FAILED;
      } else {
        channel->hcintmsk |= HCINT_ACK | HCINT_NAK | HCINT_DATATOGGLE_ERR;
        channel->hcsplt_bm.split_compl = 0;
        channel_xfer_in_retry(dwc2, ch_id, hcint);
      }
    } else if (hcint & HCINT_NYET) {
      // Must handle nyet before nak or ack. Could get a nyet at the same time as either of those on a BULK/CONTROL
      // OUT that started with a PING. The nyet takes precedence.
      if (channel->hcsplt_bm.split_en) {
        // split not yet mean hub has no data, retry complete split
        channel->hcsplt_bm.split_compl = 1;
        channel_xfer_in_retry(dwc2, ch_id, hcint);
      }
    } else if (hcint & HCINT_ACK) {
      xfer->err_count = 0;
      channel->hcintmsk &= ~HCINT_ACK;
      if (channel->hcsplt_bm.split_en) {
        // start split is ACK --> do complete split
        // TODO: for ISO must use xact_pos to plan complete split based on microframe (up to 187.5 bytes/uframe)
        channel->hcsplt_bm.split_compl = 1;
        if (edpt_is_periodic(channel->hcchar_bm.ep_type)) {
          channel->hcchar_bm.odd_frame = 1 - (dwc2->hfnum & 1); // transfer on next frame
        }
        channel_send_in_token(dwc2, channel);
      }
    } else if (hcint & (HCINT_NAK | HCINT_DATATOGGLE_ERR)) {
      xfer->err_count = 0;
      channel->hcintmsk &= ~(HCINT_NAK | HCINT_DATATOGGLE_ERR);
      channel->hcsplt_bm.split_compl = 0; // restart with start-split
      channel_xfer_in_retry(dwc2, ch_id, hcint);
    } else if (hcint & HCINT_FARME_OVERRUN) {
      // retry start-split in next binterval
      channel_xfer_in_retry(dwc2, ch_id, hcint);
    }
  }

  return is_done;
}

static bool handle_channel_out_dma(dwc2_regs_t* dwc2, uint8_t ch_id, uint32_t hcint) {
  hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
  dwc2_channel_t* channel = &dwc2->channel[ch_id];
  hcd_endpoint_t* edpt = &_hcd_data.edpt[xfer->ep_id];

  bool is_done = false;

  // TU_LOG1("out hcint = %02lX\r\n", hcint);

  if (hcint & HCINT_HALTED) {
    if (hcint & (HCINT_XFER_COMPLETE | HCINT_STALL)) {
      is_done = true;
      xfer->err_count = 0;
      if (hcint & HCINT_XFER_COMPLETE) {
        xfer->result = XFER_RESULT_SUCCESS;
        xfer->xferred_bytes += edpt->buflen;
      } else {
        xfer->result = XFER_RESULT_STALLED;
        channel_xfer_out_wrapup(dwc2, ch_id);
      }
      channel->hcintmsk &= ~HCINT_ACK;
    } else if (hcint & HCINT_XACT_ERR) {
     if (hcint & (HCINT_NAK | HCINT_NYET | HCINT_ACK)) {
       xfer->err_count = 0;
       // clean up transfer so far and start again
       channel_xfer_out_wrapup(dwc2, ch_id);
       channel_xfer_start(dwc2, ch_id);
     } else {
       xfer->err_count++;
       if (xfer->err_count >= HCD_XFER_ERROR_MAX) {
         xfer->result = XFER_RESULT_FAILED;
         is_done = true;
       } else {
         // clean up transfer so far and start again
         channel_xfer_out_wrapup(dwc2, ch_id);
         channel_xfer_start(dwc2, ch_id);
       }
     }
    } else if (hcint & HCINT_NYET) {
      if (channel->hcsplt_bm.split_en && channel->hcsplt_bm.split_compl) {
        // split not yet mean hub has no data, retry complete split
        channel->hcsplt_bm.split_compl = 1;
        channel->hcchar |= HCCHAR_CHENA;
      }
    } else if (hcint & HCINT_ACK) {
      xfer->err_count = 0;
      if (channel->hcsplt_bm.split_en && !channel->hcsplt_bm.split_compl) {
        // start split is ACK --> do complete split
        channel->hcsplt_bm.split_compl = 1;
        channel->hcchar |= HCCHAR_CHENA;
      }
    }
  } else if (hcint & HCINT_ACK) {
    xfer->err_count = 0;
    channel->hcintmsk &= ~HCINT_ACK;
  }

  return is_done;
}
#endif

static void handle_channel_irq(uint8_t rhport, bool in_isr) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const bool is_dma = dma_host_enabled(dwc2);
  const uint8_t max_channel = DWC2_CHANNEL_COUNT(dwc2);

  for (uint8_t ch_id = 0; ch_id < max_channel; ch_id++) {
    if (tu_bit_test(dwc2->haint, ch_id)) {
      dwc2_channel_t* channel = &dwc2->channel[ch_id];
      hcd_xfer_t* xfer = &_hcd_data.xfer[ch_id];
      TU_ASSERT(xfer->ep_id < CFG_TUH_DWC2_ENDPOINT_MAX,);
      dwc2_channel_char_t hcchar_bm = channel->hcchar_bm;

      const uint32_t hcint = channel->hcint;
      channel->hcint = hcint; // clear interrupt

      bool is_done = false;
      if (is_dma) {
        #if CFG_TUH_DWC2_DMA_ENABLE
        if (hcchar_bm.ep_dir == TUSB_DIR_OUT) {
          is_done = handle_channel_out_dma(dwc2, ch_id, hcint);
        } else {
          is_done = handle_channel_in_dma(dwc2, ch_id, hcint);
          if (is_done && (channel->hcdma > xfer->xferred_bytes)) {
            // hcdma is increased by word --> need to align4
            hcd_dcache_invalidate((void*) tu_align4(channel->hcdma - xfer->xferred_bytes), xfer->xferred_bytes);
          }
        }
        #endif
      } else {
        #if CFG_TUH_DWC2_SLAVE_ENABLE
        if (hcchar_bm.ep_dir == TUSB_DIR_OUT) {
          is_done = handle_channel_out_slave(dwc2, ch_id, hcint);
        } else {
          is_done = handle_channel_in_slave(dwc2, ch_id, hcint);
        }
        #endif
      }

      if (is_done) {
        const uint8_t ep_addr = tu_edpt_addr(hcchar_bm.ep_num, hcchar_bm.ep_dir);
        hcd_event_xfer_complete(hcchar_bm.dev_addr, ep_addr, xfer->xferred_bytes, xfer->result, in_isr);
        channel_dealloc(dwc2, ch_id);
      }
    }
  }
}

// SOF is enabled for scheduled periodic transfer
static bool handle_sof_irq(uint8_t rhport, bool in_isr) {
  (void) in_isr;
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  bool more_isr = false;

  // If highspeed then SOF is 125us, else 1ms
  const uint32_t ucount = (hprt_speed_get(dwc2) == TUSB_SPEED_HIGH ? 1 : 8);

  for(uint8_t ep_id = 0; ep_id < CFG_TUH_DWC2_ENDPOINT_MAX; ep_id++) {
    hcd_endpoint_t* edpt = &_hcd_data.edpt[ep_id];
    if (edpt->hcchar_bm.enable && edpt_is_periodic(edpt->hcchar_bm.ep_type) && edpt->uframe_countdown > 0) {
      edpt->uframe_countdown -= tu_min32(ucount, edpt->uframe_countdown);
      if (edpt->uframe_countdown == 0) {
        if (!edpt_xfer_kickoff(dwc2, ep_id)) {
          edpt->uframe_countdown = ucount; // failed to start, try again next frame
        }
      }

      more_isr = true;
    }
  }

  return more_isr;
}

// Config HCFG FS/LS clock and HFIR for SOF interval according to link speed (value is in PHY clock unit)
static void port0_enable(dwc2_regs_t* dwc2, tusb_speed_t speed) {
  uint32_t hcfg = dwc2->hcfg & ~HCFG_FSLS_PHYCLK_SEL;

  const dwc2_gusbcfg_t gusbcfg_bm = dwc2->gusbcfg_bm;
  uint32_t phy_clock;

  if (gusbcfg_bm.phy_sel) {
    phy_clock = 48; // dedicated FS is 48Mhz
    if (speed == TUSB_SPEED_LOW) {
      hcfg |= HCFG_FSLS_PHYCLK_SEL_6MHZ;
    } else {
      hcfg |= HCFG_FSLS_PHYCLK_SEL_48MHZ;
    }
  } else {
    if (gusbcfg_bm.ulpi_utmi_sel) {
      phy_clock = 60; // ULPI 8-bit is 60Mhz
    } else {
      // UTMI+ 16-bit is 30Mhz, 8-bit is 60Mhz
      phy_clock = gusbcfg_bm.phy_if16 ? 30 : 60;

      // Enable UTMI+ low power mode 48Mhz external clock if not highspeed
      if (speed == TUSB_SPEED_HIGH) {
        dwc2->gusbcfg &= ~GUSBCFG_PHYLPCS;
      } else {
        dwc2->gusbcfg |= GUSBCFG_PHYLPCS;
        // may need to reset port
      }
    }
    hcfg |= HCFG_FSLS_PHYCLK_SEL_30_60MHZ;
  }

  dwc2->hcfg = hcfg;

  uint32_t hfir = dwc2->hfir & ~HFIR_FRIVL_Msk;
  if (speed == TUSB_SPEED_HIGH) {
    hfir |= 125*phy_clock;
  } else {
    hfir |= 1000*phy_clock;
  }

  dwc2->hfir = hfir;
}

/* Handle Host Port interrupt, possible source are:
   - Connection Detection
   - Enable Change
   - Over Current Change
*/
static void handle_hprt_irq(uint8_t rhport, bool in_isr) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  uint32_t hprt = dwc2->hprt & ~HPRT_W1_MASK;
  const dwc2_hprt_t hprt_bm = dwc2->hprt_bm;

  if (dwc2->hprt & HPRT_CONN_DETECT) {
    // Port Connect Detect
    hprt |= HPRT_CONN_DETECT;

    if (hprt_bm.conn_status) {
      hcd_event_device_attach(rhport, in_isr);
    } else {
      hcd_event_device_remove(rhport, in_isr);
    }
  }

  if (dwc2->hprt & HPRT_ENABLE_CHANGE) {
    // Port enable change
    hprt |= HPRT_ENABLE_CHANGE;

    if (hprt_bm.enable) {
      // Port enable
      const tusb_speed_t speed = hprt_speed_get(dwc2);
      port0_enable(dwc2, speed);
    } else {
      // TU_ASSERT(false, );
    }
  }

  dwc2->hprt = hprt; // clear interrupt
}

/* Interrupt Hierarchy
               HCINTn       HPRT
                 |           |
               HAINT.CHn     |
                 |           |
    GINTSTS :  HCInt     | PrtInt | NPTxFEmp | PTxFEmpp | RXFLVL | SOF
*/
void hcd_int_handler(uint8_t rhport, bool in_isr) {
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);
  const uint32_t gintmsk = dwc2->gintmsk;
  const uint32_t gintsts = dwc2->gintsts & gintmsk;

  // TU_LOG1_HEX(gintsts);

  if (gintsts & GINTSTS_CONIDSTSCHNG) {
    // Connector ID status change
    dwc2->gintsts = GINTSTS_CONIDSTSCHNG;

    //if (dwc2->gotgctl)
    // dwc2->hprt = HPRT_POWER; // power on port to turn on VBUS
    //dwc2->gintmsk |= GINTMSK_PRTIM;
    // TODO wait for SRP if OTG
  }

  if (gintsts & GINTSTS_SOF) {
    const bool more_sof = handle_sof_irq(rhport, in_isr);
    if (!more_sof) {
      dwc2->gintmsk &= ~GINTSTS_SOF;
    }
  }

  if (gintsts & GINTSTS_HPRTINT) {
    // Host port interrupt: source is cleared in HPRT register
    // TU_LOG1_HEX(dwc2->hprt);
    handle_hprt_irq(rhport, in_isr);
  }

  if (gintsts & GINTSTS_HCINT) {
    // Host Channel interrupt: source is cleared in HCINT register
    // must be handled after TX FIFO empty
    handle_channel_irq(rhport, in_isr);
  }

#if CFG_TUH_DWC2_SLAVE_ENABLE
  // RxFIFO non-empty interrupt handling
  if (gintsts & GINTSTS_RXFLVL) {
    // RXFLVL bit is read-only
    dwc2->gintmsk &= ~GINTSTS_RXFLVL; // disable RXFLVL interrupt while reading

    do {
      handle_rxflvl_irq(rhport); // read all packets
    } while(dwc2->gintsts & GINTSTS_RXFLVL);

    dwc2->gintmsk |= GINTSTS_RXFLVL;
  }

  if (gintsts & GINTSTS_NPTX_FIFO_EMPTY) {
    // NPTX FIFO empty interrupt, this is read-only and cleared by hardware when FIFO is written
    const bool more_nptxfe = handle_txfifo_empty(dwc2, false);
    if (!more_nptxfe) {
      // no more pending packet, disable interrupt
      dwc2->gintmsk &= ~GINTSTS_NPTX_FIFO_EMPTY;
    }
  }

  if (gintsts & GINTSTS_PTX_FIFO_EMPTY) {
    // PTX FIFO empty interrupt, this is read-only and cleared by hardware when FIFO is written
    const bool more_ptxfe = handle_txfifo_empty(dwc2, true);
    if (!more_ptxfe) {
      // no more pending packet, disable interrupt
      dwc2->gintmsk &= ~GINTSTS_PTX_FIFO_EMPTY;
    }
  }
#endif
}

#endif
