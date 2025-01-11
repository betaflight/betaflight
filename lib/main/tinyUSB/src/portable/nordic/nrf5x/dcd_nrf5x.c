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

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_NRF5X

#include <stdatomic.h>

// Suppress warning caused by nrfx driver
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include "nrf.h"
#include "nrf_clock.h"
#include "nrfx_usbd_errata.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "device/dcd.h"

// TODO remove later
#include "device/usbd.h"
#include "device/usbd_pvt.h" // to use defer function helper

#if CFG_TUSB_OS == OPT_OS_MYNEWT
#include "mcu/mcu.h"
#endif

/* Try to detect nrfx version if not configured with CFG_TUD_NRF_NRFX_VERSION
 * nrfx v1 and v2 are concurrently developed. There is no NRFX_VERSION only MDK VERSION which is as follows:
 * - v3.0.0: 8.53.1 (conflict with v2.11.0), v3.1.0: 8.55.0 ...
 * - v2.11.0: 8.53.1, v2.6.0: 8.44.1, v2.5.0: 8.40.2, v2.4.0: 8.37.0, v2.3.0: 8.35.0, v2.2.0: 8.32.1, v2.1.0: 8.30.2, v2.0.0: 8.29.0
 * - v1.9.0: 8.40.3, v1.8.6: 8.35.0 (conflict with v2.3.0), v1.8.5: 8.32.3, v1.8.4: 8.32.1 (conflict with v2.2.0),
 *   v1.8.2: 8.32.1 (conflict with v2.2.0), v1.8.1: 8.27.1
 * Therefore the check for v1 would be:
 * - MDK < 8.29.0 (v2.0), MDK == 8.32.3, 8.40.3
 * - in case of conflict User of those version must upgrade to other 1.x version or set CFG_TUD_NRF_NRFX_VERSION
*/
#ifndef CFG_TUD_NRF_NRFX_VERSION
  #define _MDK_VERSION  (10000*MDK_MAJOR_VERSION + 100*MDK_MINOR_VERSION + MDK_MICRO_VERSION)

  #if _MDK_VERSION < 82900 || _MDK_VERSION == 83203 || _MDK_VERSION == 84003
    // nrfx <= 1.8.1, or 1.8.5 or 1.9.0
    #define CFG_TUD_NRF_NRFX_VERSION 1
  #else
    #define CFG_TUD_NRF_NRFX_VERSION 2
  #endif
#endif

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
enum {
  // Max allowed by USB specs
  MAX_PACKET_SIZE = 64,

  // Mask of all END event (IN & OUT) for all endpoints. ENDEPIN0-7, ENDEPOUT0-7, ENDISOIN, ENDISOOUT
  EDPT_END_ALL_MASK = (0xff << USBD_INTEN_ENDEPIN0_Pos) | (0xff << USBD_INTEN_ENDEPOUT0_Pos) |
                      USBD_INTENCLR_ENDISOIN_Msk | USBD_INTEN_ENDISOOUT_Msk
};

enum {
  EP_ISO_NUM = 8, // Endpoint number is fixed (8) for ISOOUT and ISOIN
  EP_CBI_COUNT = 8  // Control Bulk Interrupt endpoints count
};

// Transfer Descriptor
typedef struct {
  uint8_t* buffer;
  uint16_t total_len;
  volatile uint16_t actual_len;
  uint16_t mps; // max packet size

  // nRF will auto accept OUT packet after DMA is done
  // indicate packet is already ACK
  volatile bool data_received;
  volatile bool started;

  // Set to true when data was transferred from RAM to ISO IN output buffer.
  // New data can be put in ISO IN output buffer after SOF.
  bool iso_in_transfer_ready;

} xfer_td_t;

// Data for managing dcd
static struct {
  // All 8 endpoints including control IN & OUT (offset 1)
  // +1 for ISO endpoints
  xfer_td_t xfer[EP_CBI_COUNT + 1][2];

  // nRF can only carry one DMA at a time, this is used to guard the access to EasyDMA
  atomic_flag dma_running;

  // Track whether sof has been manually enabled
  bool sof_enabled;
} _dcd;

/*------------------------------------------------------------------*/
/* Control / Bulk / Interrupt (CBI) Transfer
 *------------------------------------------------------------------*/

// check if we are in ISR
TU_ATTR_ALWAYS_INLINE static inline bool is_in_isr(void) {
  return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) ? true : false;
}

// helper to start DMA
static void start_dma(volatile uint32_t* reg_startep) {
  (*reg_startep) = 1;
  __ISB();
  __DSB();

  // TASKS_EP0STATUS, TASKS_EP0RCVOUT seem to need EasyDMA to be available
  // However these don't trigger any DMA transfer and got ENDED event subsequently
  // Therefore dma_pending is corrected right away
  if ((reg_startep == &NRF_USBD->TASKS_EP0STATUS) || (reg_startep == &NRF_USBD->TASKS_EP0RCVOUT)) {
    atomic_flag_clear(&_dcd.dma_running);
  }
}

static void edpt_dma_start(volatile uint32_t* reg_startep) {
  if (atomic_flag_test_and_set(&_dcd.dma_running)) {
    usbd_defer_func((osal_task_func_t)(uintptr_t ) edpt_dma_start, (void*) (uintptr_t) reg_startep, is_in_isr());
  } else {
    start_dma(reg_startep);
  }
}

// DMA is complete
static void edpt_dma_end(void) {
  atomic_flag_clear(&_dcd.dma_running);
}

// helper getting td
static inline xfer_td_t* get_td(uint8_t epnum, uint8_t dir) {
  return &_dcd.xfer[epnum][dir];
}

static void xact_out_dma(uint8_t epnum);

// Function wraps xact_out_dma which wants uint8_t while usbd_defer_func wants void (*)(void *)
static void xact_out_dma_wrapper(void* epnum) {
  xact_out_dma((uint8_t) ((uintptr_t) epnum));
}

// Start DMA to move data from Endpoint -> RAM
static void xact_out_dma(uint8_t epnum) {
  xfer_td_t* xfer = get_td(epnum, TUSB_DIR_OUT);
  uint32_t xact_len;

  // DMA can't be active during read of SIZE.EPOUT or SIZE.ISOOUT, so try to lock,
  // If already running defer call regardless if it was called from ISR or task,
  if (atomic_flag_test_and_set(&_dcd.dma_running)) {
    usbd_defer_func((osal_task_func_t) xact_out_dma_wrapper, (void*) (uint32_t) epnum, is_in_isr());
    return;
  }
  if (epnum == EP_ISO_NUM) {
    xact_len = NRF_USBD->SIZE.ISOOUT;
    // If ZERO bit is set, ignore ISOOUT length
    if (xact_len & USBD_SIZE_ISOOUT_ZERO_Msk) {
      xact_len = 0;
      atomic_flag_clear(&_dcd.dma_running);
    } else {
      if (xfer->started) {
        // Trigger DMA move data from Endpoint -> SRAM
        NRF_USBD->ISOOUT.PTR = (uint32_t) xfer->buffer;
        NRF_USBD->ISOOUT.MAXCNT = xact_len;

        start_dma(&NRF_USBD->TASKS_STARTISOOUT);
      } else {
        atomic_flag_clear(&_dcd.dma_running);
      }
    }
  } else {
    // limit xact len to remaining length
    xact_len = tu_min16((uint16_t) NRF_USBD->SIZE.EPOUT[epnum], xfer->total_len - xfer->actual_len);

    // Trigger DMA move data from Endpoint -> SRAM
    NRF_USBD->EPOUT[epnum].PTR = (uint32_t) xfer->buffer;
    NRF_USBD->EPOUT[epnum].MAXCNT = xact_len;

    start_dma(&NRF_USBD->TASKS_STARTEPOUT[epnum]);
  }
}

// Prepare for a CBI transaction IN, call at the start
// it start DMA to transfer data from RAM -> Endpoint
static void xact_in_dma(uint8_t epnum) {
  xfer_td_t* xfer = get_td(epnum, TUSB_DIR_IN);

  // Each transaction is up to Max Packet Size
  uint16_t const xact_len = tu_min16(xfer->total_len - xfer->actual_len, xfer->mps);

  NRF_USBD->EPIN[epnum].PTR = (uint32_t) xfer->buffer;
  NRF_USBD->EPIN[epnum].MAXCNT = xact_len;

  edpt_dma_start(&NRF_USBD->TASKS_STARTEPIN[epnum]);
}

//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;
  TU_LOG2("dcd init\r\n");
  return true;
}

void dcd_int_enable(uint8_t rhport) {
  (void) rhport;
  NVIC_EnableIRQ(USBD_IRQn);
}

void dcd_int_disable(uint8_t rhport) {
  (void) rhport;
  NVIC_DisableIRQ(USBD_IRQn);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  (void) dev_addr;
  // Set Address is automatically update by hw controller, nothing to do

  // Enable usbevent for suspend and resume detection
  // Since the bus signal D+/D- are stable now.

  // Clear current pending first
  NRF_USBD->EVENTCAUSE |= NRF_USBD->EVENTCAUSE;
  NRF_USBD->EVENTS_USBEVENT = 0;

  NRF_USBD->INTENSET = USBD_INTEN_USBEVENT_Msk;
}

void dcd_remote_wakeup(uint8_t rhport) {
  (void) rhport;

  // Bring controller out of low power mode
  // will start wakeup when USBWUALLOWED is set
  NRF_USBD->LOWPOWER = 0;
}

// disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport) {
  (void) rhport;
  NRF_USBD->USBPULLUP = 0;

  // Disable Pull-up does not trigger Power USB Removed, in fact it have no
  // impact on the USB Power status at all -> need to submit unplugged event to the stack.
  dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, false);
}

// connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport) {
  (void) rhport;
  NRF_USBD->USBPULLUP = 1;
}

void dcd_sof_enable(uint8_t rhport, bool en) {
  (void) rhport;
  if (en) {
    _dcd.sof_enabled = true;
    NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
  } else {
    _dcd.sof_enabled = false;
    NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
  }
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const* desc_edpt) {
  (void) rhport;

  uint8_t const ep_addr = desc_edpt->bEndpointAddress;
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  _dcd.xfer[epnum][dir].mps = tu_edpt_packet_size(desc_edpt);

  if (desc_edpt->bmAttributes.xfer != TUSB_XFER_ISOCHRONOUS) {
    if (dir == TUSB_DIR_OUT) {
      NRF_USBD->INTENSET = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + epnum);
      NRF_USBD->EPOUTEN |= TU_BIT(epnum);

      // Write any value to SIZE register will allow nRF to ACK/accept data
      NRF_USBD->SIZE.EPOUT[epnum] = 0;
    } else {
      NRF_USBD->INTENSET = TU_BIT(USBD_INTEN_ENDEPIN0_Pos + epnum);
      NRF_USBD->EPINEN |= TU_BIT(epnum);
    }
    // clear stall and reset DataToggle
    NRF_USBD->EPSTALL = (USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | ep_addr;
    NRF_USBD->DTOGGLE = (USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | ep_addr;
  } else {
    TU_ASSERT(epnum == EP_ISO_NUM);
    if (dir == TUSB_DIR_OUT) {
      // SPLIT ISO buffer when ISO IN endpoint is already opened.
      if (_dcd.xfer[EP_ISO_NUM][TUSB_DIR_IN].mps) NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;

      // Clear old events
      NRF_USBD->EVENTS_ENDISOOUT = 0;

      // Clear SOF event in case interrupt was not enabled yet.
      if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0) NRF_USBD->EVENTS_SOF = 0;

      // Enable SOF and ISOOUT interrupts, and ISOOUT endpoint.
      NRF_USBD->INTENSET = USBD_INTENSET_ENDISOOUT_Msk | USBD_INTENSET_SOF_Msk;
      NRF_USBD->EPOUTEN |= USBD_EPOUTEN_ISOOUT_Msk;
    } else {
      NRF_USBD->EVENTS_ENDISOIN = 0;

      // SPLIT ISO buffer when ISO OUT endpoint is already opened.
      if (_dcd.xfer[EP_ISO_NUM][TUSB_DIR_OUT].mps) NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;

      // Clear SOF event in case interrupt was not enabled yet.
      if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0) NRF_USBD->EVENTS_SOF = 0;

      // Enable SOF and ISOIN interrupts, and ISOIN endpoint.
      NRF_USBD->INTENSET = USBD_INTENSET_ENDISOIN_Msk | USBD_INTENSET_SOF_Msk;
      NRF_USBD->EPINEN |= USBD_EPINEN_ISOIN_Msk;
    }
  }

  __ISB();
  __DSB();

  return true;
}

void dcd_edpt_close_all(uint8_t rhport) {
  // disable interrupt to prevent race condition
  dcd_int_disable(rhport);

  // disable all non-control (bulk + interrupt) endpoints
  for (uint8_t ep = 1; ep < EP_CBI_COUNT; ep++) {
    NRF_USBD->INTENCLR = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + ep) | TU_BIT(USBD_INTEN_ENDEPIN0_Pos + ep);

    NRF_USBD->TASKS_STARTEPIN[ep] = 0;
    NRF_USBD->TASKS_STARTEPOUT[ep] = 0;

    tu_memclr(_dcd.xfer[ep], 2 * sizeof(xfer_td_t));
  }

  // disable both ISO
  NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk | USBD_INTENCLR_ENDISOOUT_Msk | USBD_INTENCLR_ENDISOIN_Msk;
  NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_OneDir;

  NRF_USBD->TASKS_STARTISOIN = 0;
  NRF_USBD->TASKS_STARTISOOUT = 0;

  tu_memclr(_dcd.xfer[EP_ISO_NUM], 2 * sizeof(xfer_td_t));

  // de-activate all non-control
  NRF_USBD->EPOUTEN = 1UL;
  NRF_USBD->EPINEN = 1UL;

  dcd_int_enable(rhport);
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  if (epnum != EP_ISO_NUM) {
    // CBI
    if (dir == TUSB_DIR_OUT) {
      NRF_USBD->INTENCLR = TU_BIT(USBD_INTEN_ENDEPOUT0_Pos + epnum);
      NRF_USBD->EPOUTEN &= ~TU_BIT(epnum);
    } else {
      NRF_USBD->INTENCLR = TU_BIT(USBD_INTEN_ENDEPIN0_Pos + epnum);
      NRF_USBD->EPINEN &= ~TU_BIT(epnum);
    }
  } else {
    _dcd.xfer[EP_ISO_NUM][dir].mps = 0;
    // ISO
    if (dir == TUSB_DIR_OUT) {
      NRF_USBD->INTENCLR = USBD_INTENCLR_ENDISOOUT_Msk;
      NRF_USBD->EPOUTEN &= ~USBD_EPOUTEN_ISOOUT_Msk;
      NRF_USBD->EVENTS_ENDISOOUT = 0;
    } else {
      NRF_USBD->INTENCLR = USBD_INTENCLR_ENDISOIN_Msk;
      NRF_USBD->EPINEN &= ~USBD_EPINEN_ISOIN_Msk;
    }
    // One of the ISO endpoints closed, no need to split buffers any more.
    NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_OneDir;
    // When both ISO endpoint are close there is no need for SOF any more.
    if (_dcd.xfer[EP_ISO_NUM][TUSB_DIR_IN].mps + _dcd.xfer[EP_ISO_NUM][TUSB_DIR_OUT].mps == 0)
      NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
  }
  _dcd.xfer[epnum][dir].started = false;
  __ISB();
  __DSB();
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes) {
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  xfer_td_t* xfer = get_td(epnum, dir);

  TU_ASSERT(!xfer->started);
  xfer->buffer = buffer;
  xfer->total_len = total_bytes;
  xfer->actual_len = 0;

  // Control endpoint with zero-length packet and opposite direction to 1st request byte --> status stage
  bool const control_status = (epnum == 0 && total_bytes == 0 && dir != tu_edpt_dir(NRF_USBD->BMREQUESTTYPE));

  if (control_status) {
    // The nRF doesn't interrupt on status transmit so we queue up a success response.
    dcd_event_xfer_complete(0, ep_addr, 0, XFER_RESULT_SUCCESS, is_in_isr());

    // Status Phase also requires EasyDMA has to be available as well !!!!
    edpt_dma_start(&NRF_USBD->TASKS_EP0STATUS);
  } else if (dir == TUSB_DIR_OUT) {
    xfer->started = true;
    if (epnum == 0) {
      // Accept next Control Out packet. TASKS_EP0RCVOUT also require EasyDMA
      edpt_dma_start(&NRF_USBD->TASKS_EP0RCVOUT);
    } else {
      // started just set, it could start DMA transfer if interrupt was trigger after this line
      // code only needs to start transfer (from Endpoint to RAM) when data_received was set
      // before started was set. If started is NOT set but data_received is, it means that
      // current transfer was already finished and next data is already present in endpoint and
      // can be consumed by future transfer
      __ISB();
      __DSB();
      if (xfer->data_received && xfer->started) {
        // Data is already received previously
        // start DMA to copy to SRAM
        xfer->data_received = false;
        xact_out_dma(epnum);
      } else {
        // nRF auto accept next Bulk/Interrupt OUT packet
        // nothing to do
      }
    }
  } else {
    // Start DMA to copy data from RAM -> Endpoint
    xact_in_dma(epnum);
  }

  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  xfer_td_t* xfer = get_td(epnum, dir);

  if (epnum == 0) {
    NRF_USBD->TASKS_EP0STALL = 1;
  } else if (epnum != EP_ISO_NUM) {
    NRF_USBD->EPSTALL = (USBD_EPSTALL_STALL_Stall << USBD_EPSTALL_STALL_Pos) | ep_addr;

    // Note: nRF can auto ACK packet OUT before get stalled.
    // There maybe data in endpoint fifo already, we need to pull it out
    if ((dir == TUSB_DIR_OUT) && xfer->data_received) {
      xfer->data_received = false;
      xact_out_dma(epnum);
    }
  }

  __ISB();
  __DSB();
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  (void) rhport;
  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  if (epnum != 0 && epnum != EP_ISO_NUM) {
    // reset data toggle to DATA0
    // First write this register with VALUE=Nop to select the endpoint, then either read it to get the status from
    // VALUE, or write it again with VALUE=Data0 or Data1
    NRF_USBD->DTOGGLE = ep_addr;
    NRF_USBD->DTOGGLE = (USBD_DTOGGLE_VALUE_Data0 << USBD_DTOGGLE_VALUE_Pos) | ep_addr;

    // clear stall
    NRF_USBD->EPSTALL = (USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos) | ep_addr;

    // Write any value to SIZE register will allow nRF to ACK/accept data
    if (dir == TUSB_DIR_OUT) NRF_USBD->SIZE.EPOUT[epnum] = 0;

    __ISB();
    __DSB();
  }
}

/*------------------------------------------------------------------*/
/* Interrupt Handler
 *------------------------------------------------------------------*/
static void bus_reset(void) {
  // 6.35.6 USB controller automatically disabled all endpoints (except control)
  NRF_USBD->EPOUTEN = 1UL;
  NRF_USBD->EPINEN = 1UL;

  for (int i = 0; i < 8; i++) {
    NRF_USBD->TASKS_STARTEPIN[i] = 0;
    NRF_USBD->TASKS_STARTEPOUT[i] = 0;
  }

  NRF_USBD->TASKS_STARTISOIN = 0;
  NRF_USBD->TASKS_STARTISOOUT = 0;

  // Clear USB Event Interrupt
  NRF_USBD->EVENTS_USBEVENT = 0;
  NRF_USBD->EVENTCAUSE |= NRF_USBD->EVENTCAUSE;

  // Reset interrupt
  NRF_USBD->INTENCLR = NRF_USBD->INTEN;
  NRF_USBD->INTENSET = USBD_INTEN_USBRESET_Msk | USBD_INTEN_USBEVENT_Msk | USBD_INTEN_EPDATA_Msk |
                       USBD_INTEN_EP0SETUP_Msk | USBD_INTEN_EP0DATADONE_Msk | USBD_INTEN_ENDEPIN0_Msk |
                       USBD_INTEN_ENDEPOUT0_Msk;

  tu_varclr(&_dcd);
  _dcd.xfer[0][TUSB_DIR_IN].mps = MAX_PACKET_SIZE;
  _dcd.xfer[0][TUSB_DIR_OUT].mps = MAX_PACKET_SIZE;
}

void dcd_int_handler(uint8_t rhport) {
  (void) rhport;

  uint32_t const inten = NRF_USBD->INTEN;
  uint32_t int_status = 0;

  volatile uint32_t* regevt = &NRF_USBD->EVENTS_USBRESET;

  for (uint8_t i = 0; i < USBD_INTEN_EPDATA_Pos + 1; i++) {
    if (tu_bit_test(inten, i) && regevt[i]) {
      int_status |= TU_BIT(i);

      // event clear
      regevt[i] = 0;
      __ISB();
      __DSB();
    }
  }

  if (int_status & USBD_INTEN_USBRESET_Msk) {
    bus_reset();
    dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
  }

  // ISOIN: Data was moved to endpoint buffer, client will be notified in SOF
  if (int_status & USBD_INTEN_ENDISOIN_Msk) {
    xfer_td_t* xfer = get_td(EP_ISO_NUM, TUSB_DIR_IN);

    xfer->actual_len = NRF_USBD->ISOIN.AMOUNT;
    // Data transferred from RAM to endpoint output buffer.
    // Next transfer can be scheduled after SOF.
    xfer->iso_in_transfer_ready = true;
  }

  if (int_status & USBD_INTEN_SOF_Msk) {
    bool iso_enabled = false;

    // ISOOUT: Transfer data gathered in previous frame from buffer to RAM
    if (NRF_USBD->EPOUTEN & USBD_EPOUTEN_ISOOUT_Msk) {
      iso_enabled = true;
      // Transfer from endpoint to RAM only if data is not corrupted
      if ((int_status & USBD_INTEN_USBEVENT_Msk) == 0 ||
          (NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_ISOOUTCRC_Msk) == 0) {
        xact_out_dma(EP_ISO_NUM);
      }
    }

    // ISOIN: Notify client that data was transferred
    if (NRF_USBD->EPINEN & USBD_EPINEN_ISOIN_Msk) {
      iso_enabled = true;

      xfer_td_t* xfer = get_td(EP_ISO_NUM, TUSB_DIR_IN);
      if (xfer->iso_in_transfer_ready) {
        xfer->iso_in_transfer_ready = false;
        dcd_event_xfer_complete(0, EP_ISO_NUM | TUSB_DIR_IN_MASK, xfer->actual_len, XFER_RESULT_SUCCESS, true);
      }
    }

    if (!iso_enabled && !_dcd.sof_enabled) {
      // SOF interrupt not manually enabled and ISO endpoint is not used,
      // SOF is only enabled one-time for remote wakeup so we disable it now

      NRF_USBD->INTENCLR = USBD_INTENCLR_SOF_Msk;
    }

    const uint32_t frame = NRF_USBD->FRAMECNTR;
    dcd_event_sof(0, frame, true);
    //dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
  }

  if (int_status & USBD_INTEN_USBEVENT_Msk) {
    TU_LOG(3, "EVENTCAUSE = 0x%04" PRIX32 "\r\n", NRF_USBD->EVENTCAUSE);

    enum {
      EVT_CAUSE_MASK = USBD_EVENTCAUSE_SUSPEND_Msk | USBD_EVENTCAUSE_RESUME_Msk | USBD_EVENTCAUSE_USBWUALLOWED_Msk |
                       USBD_EVENTCAUSE_ISOOUTCRC_Msk
    };
    uint32_t const evt_cause = NRF_USBD->EVENTCAUSE & EVT_CAUSE_MASK;
    NRF_USBD->EVENTCAUSE = evt_cause; // clear interrupt

    if (evt_cause & USBD_EVENTCAUSE_SUSPEND_Msk) {
      // Put controller into low power mode
      // Leave HFXO disable to application, since it may be used by other peripherals
      NRF_USBD->LOWPOWER = 1;

      dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
    }

    if (evt_cause & USBD_EVENTCAUSE_USBWUALLOWED_Msk) {
      // USB is out of low power mode, and wakeup is allowed
      // Initiate RESUME signal
      NRF_USBD->DPDMVALUE = USBD_DPDMVALUE_STATE_Resume;
      NRF_USBD->TASKS_DPDMDRIVE = 1;

      // There is no Resume interrupt for remote wakeup, enable SOF for to report bus ready state
      // Clear SOF event in case interrupt was not enabled yet.
      if ((NRF_USBD->INTEN & USBD_INTEN_SOF_Msk) == 0) NRF_USBD->EVENTS_SOF = 0;
      NRF_USBD->INTENSET = USBD_INTENSET_SOF_Msk;
    }

    if (evt_cause & USBD_EVENTCAUSE_RESUME_Msk) {
      dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
    }
  }

  // Setup tokens are specific to the Control endpoint.
  if (int_status & USBD_INTEN_EP0SETUP_Msk) {
    uint8_t const setup[8] = {
        NRF_USBD->BMREQUESTTYPE, NRF_USBD->BREQUEST, NRF_USBD->WVALUEL, NRF_USBD->WVALUEH,
        NRF_USBD->WINDEXL, NRF_USBD->WINDEXH, NRF_USBD->WLENGTHL, NRF_USBD->WLENGTHH
    };

    // nrf5x hw auto handle set address, there is no need to inform usb stack
    tusb_control_request_t const* request = (tusb_control_request_t const*) setup;

    if (!(TUSB_REQ_RCPT_DEVICE == request->bmRequestType_bit.recipient &&
          TUSB_REQ_TYPE_STANDARD == request->bmRequestType_bit.type &&
          TUSB_REQ_SET_ADDRESS == request->bRequest)) {
      dcd_event_setup_received(0, setup, true);
    }
  }

  if (int_status & EDPT_END_ALL_MASK) {
    // DMA complete move data from SRAM <-> Endpoint
    // Must before endpoint transfer handling
    edpt_dma_end();
  }

  //--------------------------------------------------------------------+
  /* Control/Bulk/Interrupt (CBI) Transfer
   *
   * Data flow is:
   *           (bus)              (dma)
   *    Host <-------> Endpoint <-------> RAM
   *
   * For CBI OUT:
   *  - Host -> Endpoint
   *      EPDATA (or EP0DATADONE) interrupted, check EPDATASTATUS.EPOUT[i]
   *      to start DMA. For Bulk/Interrupt, this step can occur automatically (without sw),
   *      which means data may or may not be ready (out_received flag).
   *  - Endpoint -> RAM
   *      ENDEPOUT[i] interrupted, transaction complete, sw prepare next transaction
   *
   * For CBI IN:
   *  - RAM -> Endpoint
   *      ENDEPIN[i] interrupted indicate DMA is complete. HW will start
   *      to move data to host
   *  - Endpoint -> Host
   *      EPDATA (or EP0DATADONE) interrupted, check EPDATASTATUS.EPIN[i].
   *      Transaction is complete, sw prepare next transaction
   *
   * Note: in both Control In and Out of Data stage from Host <-> Endpoint
   * EP0DATADONE will be set as interrupt source
   */
  //--------------------------------------------------------------------+

  /* CBI OUT: Endpoint -> SRAM (aka transaction complete)
   * Note: Since nRF controller auto ACK next packet without SW awareness
   * We must handle this stage before Host -> Endpoint just in case 2 event happens at once
   *
   * ISO OUT: Transaction must fit in single packet, it can be shorter then total
   * len if Host decides to sent fewer bytes, it this case transaction is also
   * complete and next transfer is not initiated here like for CBI.
   */
  for (uint8_t epnum = 0; epnum < EP_CBI_COUNT + 1; epnum++) {
    if (tu_bit_test(int_status, USBD_INTEN_ENDEPOUT0_Pos + epnum)) {
      xfer_td_t* xfer = get_td(epnum, TUSB_DIR_OUT);
      uint16_t const xact_len = NRF_USBD->EPOUT[epnum].AMOUNT;

      xfer->buffer += xact_len;
      xfer->actual_len += xact_len;

      // Transfer complete if transaction len < Max Packet Size or total len is transferred
      if ((epnum != EP_ISO_NUM) && (xact_len == xfer->mps) && (xfer->actual_len < xfer->total_len)) {
        if (epnum == 0) {
          // Accept next Control Out packet. TASKS_EP0RCVOUT also require EasyDMA
          edpt_dma_start(&NRF_USBD->TASKS_EP0RCVOUT);
        } else {
          // nRF auto accept next Bulk/Interrupt OUT packet
          // nothing to do
        }
      } else {
        TU_ASSERT(xfer->started,);
        xfer->total_len = xfer->actual_len;
        xfer->started = false;

        // CBI OUT complete
        dcd_event_xfer_complete(0, epnum, xfer->actual_len, XFER_RESULT_SUCCESS, true);
      }
    }

    // Ended event for CBI IN : nothing to do
  }

  // Endpoint <-> Host ( In & OUT )
  if (int_status & (USBD_INTEN_EPDATA_Msk | USBD_INTEN_EP0DATADONE_Msk)) {
    uint32_t data_status = NRF_USBD->EPDATASTATUS;
    NRF_USBD->EPDATASTATUS = data_status;
    __ISB();
    __DSB();

    // EP0DATADONE is set with either Control Out on IN Data
    // Since EPDATASTATUS cannot be used to determine whether it is control OUT or IN.
    // We will use BMREQUESTTYPE in setup packet to determine the direction
    bool const is_control_in = (int_status & USBD_INTEN_EP0DATADONE_Msk) && (NRF_USBD->BMREQUESTTYPE & TUSB_DIR_IN_MASK);
    bool const is_control_out = (int_status & USBD_INTEN_EP0DATADONE_Msk) && !(NRF_USBD->BMREQUESTTYPE & TUSB_DIR_IN_MASK);

    // CBI In: Endpoint -> Host (transaction complete)
    for (uint8_t epnum = 0; epnum < EP_CBI_COUNT; epnum++) {
      if (tu_bit_test(data_status, epnum) || (epnum == 0 && is_control_in)) {
        xfer_td_t* xfer = get_td(epnum, TUSB_DIR_IN);
        uint8_t const xact_len = NRF_USBD->EPIN[epnum].AMOUNT;

        xfer->buffer += xact_len;
        xfer->actual_len += xact_len;

        if (xfer->actual_len < xfer->total_len) {
          // Start DMA to copy next data packet
          xact_in_dma(epnum);
        } else {
          // CBI IN complete
          dcd_event_xfer_complete(0, epnum | TUSB_DIR_IN_MASK, xfer->actual_len, XFER_RESULT_SUCCESS, true);
        }
      }
    }

    // CBI OUT: Host -> Endpoint
    for (uint8_t epnum = 0; epnum < EP_CBI_COUNT; epnum++) {
      if (tu_bit_test(data_status, 16 + epnum) || (epnum == 0 && is_control_out)) {
        xfer_td_t* xfer = get_td(epnum, TUSB_DIR_OUT);

        if (xfer->started && xfer->actual_len < xfer->total_len) {
          xact_out_dma(epnum);
        } else {
          // Data overflow !!! Nah, nRF will auto accept next Bulk/Interrupt OUT packet
          // Mark this endpoint with data received
          xfer->data_received = true;
        }
      }
    }
  }
}

//--------------------------------------------------------------------+
// HFCLK helper
//--------------------------------------------------------------------+
#ifdef SOFTDEVICE_PRESENT

// For enable/disable hfclk with SoftDevice
#include "nrf_mbr.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#ifndef SD_MAGIC_NUMBER
  #define SD_MAGIC_NUMBER   0x51B1E5DB
#endif

TU_ATTR_ALWAYS_INLINE static inline bool is_sd_existed(void) {
  return *((uint32_t*)(SOFTDEVICE_INFO_STRUCT_ADDRESS+4)) == SD_MAGIC_NUMBER;
}

// check if SD is existed and enabled
TU_ATTR_ALWAYS_INLINE static inline bool is_sd_enabled(void) {
  if ( !is_sd_existed() ) return false;
  uint8_t sd_en = false;
  (void) sd_softdevice_is_enabled(&sd_en);
  return sd_en;
}
#endif

static bool hfclk_running(void) {
#ifdef SOFTDEVICE_PRESENT
  if ( is_sd_enabled() ) {
    uint32_t is_running = 0;
    (void) sd_clock_hfclk_is_running(&is_running);
    return (is_running ? true : false);
  }
#endif

#if CFG_TUD_NRF_NRFX_VERSION == 1
  return nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY);
#else
  return nrf_clock_hf_is_running(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY);
#endif
}

static void hfclk_enable(void) {
#if CFG_TUSB_OS == OPT_OS_MYNEWT
  usb_clock_request();
  return;
#else

  // already running, nothing to do
  if (hfclk_running()) return;

#ifdef SOFTDEVICE_PRESENT
  if ( is_sd_enabled() ) {
    (void)sd_clock_hfclk_request();
    return;
  }
#endif

#if CFG_TUD_NRF_NRFX_VERSION == 1
  nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);
#else
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
#endif
#endif
}

static void hfclk_disable(void) {
#if CFG_TUSB_OS == OPT_OS_MYNEWT
  usb_clock_release();
  return;
#else

#ifdef SOFTDEVICE_PRESENT
  if ( is_sd_enabled() ) {
    (void)sd_clock_hfclk_release();
    return;
  }
#endif

#if CFG_TUD_NRF_NRFX_VERSION == 1
  nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTOP);
#else
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTOP);
#endif
#endif
}

// Power & Clock Peripheral on nRF5x to manage USB
//
// USB Bus power is managed by Power module, there are 3 VBUS power events:
// Detected, Ready, Removed. Upon these power events, This function will
// enable ( or disable ) usb & hfclk peripheral, set the usb pin pull up
// accordingly to the controller Startup/Standby Sequence in USBD 51.4 specs.
//
// Therefore this function must be called to handle USB power event by
// - nrfx_power_usbevt_init() : if Softdevice is not used or enabled
// - SoftDevice SOC event : if SD is used and enabled
void tusb_hal_nrf_power_event(uint32_t event);
void tusb_hal_nrf_power_event(uint32_t event) {
  // Value is chosen to be as same as NRFX_POWER_USB_EVT_* in nrfx_power.h
  enum {
    USB_EVT_DETECTED = 0,
    USB_EVT_REMOVED = 1,
    USB_EVT_READY = 2
  };

#if CFG_TUSB_DEBUG >= 3
  const char* const power_evt_str[] = {"Detected", "Removed", "Ready"};
  TU_LOG(3, "Power USB event: %s\r\n", power_evt_str[event]);
#endif

  switch (event) {
    case USB_EVT_DETECTED:
      if (!NRF_USBD->ENABLE) {
        // Prepare for receiving READY event: disable interrupt since we will blocking wait
        NRF_USBD->INTENCLR = USBD_INTEN_USBEVENT_Msk;
        NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
        __ISB();
        __DSB(); // for sync

#ifdef NRF52_SERIES // NRF53 does not need this errata
        // ERRATA 171, 187, 166
        if (nrfx_usbd_errata_187()) {
          // CRITICAL_REGION_ENTER();
          if (*((volatile uint32_t*) (0x4006EC00)) == 0x00000000) {
            *((volatile uint32_t*) (0x4006EC00)) = 0x00009375;
            *((volatile uint32_t*) (0x4006ED14)) = 0x00000003;
            *((volatile uint32_t*) (0x4006EC00)) = 0x00009375;
          } else {
            *((volatile uint32_t*) (0x4006ED14)) = 0x00000003;
          }
          // CRITICAL_REGION_EXIT();
        }

        if (nrfx_usbd_errata_171()) {
          // CRITICAL_REGION_ENTER();
          if (*((volatile uint32_t*) (0x4006EC00)) == 0x00000000) {
            *((volatile uint32_t*) (0x4006EC00)) = 0x00009375;
            *((volatile uint32_t*) (0x4006EC14)) = 0x000000C0;
            *((volatile uint32_t*) (0x4006EC00)) = 0x00009375;
          } else {
            *((volatile uint32_t*) (0x4006EC14)) = 0x000000C0;
          }
          // CRITICAL_REGION_EXIT();
        }
#endif

        // Enable the peripheral (will cause Ready event)
        NRF_USBD->ENABLE = 1;
        __ISB();
        __DSB(); // for sync

        // Enable HFCLK
        hfclk_enable();
      }
      break;

    case USB_EVT_READY:
      // Skip if pull-up is enabled and HCLK is already running.
      // Application probably call this more than necessary.
      if (NRF_USBD->USBPULLUP && hfclk_running()) break;

      // Waiting for USBD peripheral enabled
      while (!(USBD_EVENTCAUSE_READY_Msk & NRF_USBD->EVENTCAUSE)) {}

      NRF_USBD->EVENTCAUSE = USBD_EVENTCAUSE_READY_Msk;
      __ISB();
      __DSB(); // for sync

#ifdef NRF52_SERIES
      if (nrfx_usbd_errata_171()) {
        // CRITICAL_REGION_ENTER();
        if (*((volatile uint32_t*) (0x4006EC00)) == 0x00000000) {
          *((volatile uint32_t*) (0x4006EC00)) = 0x00009375;
          *((volatile uint32_t*) (0x4006EC14)) = 0x00000000;
          *((volatile uint32_t*) (0x4006EC00)) = 0x00009375;
        } else {
          *((volatile uint32_t*) (0x4006EC14)) = 0x00000000;
        }

        // CRITICAL_REGION_EXIT();
      }

      if (nrfx_usbd_errata_187()) {
        // CRITICAL_REGION_ENTER();
        if (*((volatile uint32_t*) (0x4006EC00)) == 0x00000000) {
          *((volatile uint32_t*) (0x4006EC00)) = 0x00009375;
          *((volatile uint32_t*) (0x4006ED14)) = 0x00000000;
          *((volatile uint32_t*) (0x4006EC00)) = 0x00009375;
        } else {
          *((volatile uint32_t*) (0x4006ED14)) = 0x00000000;
        }
        // CRITICAL_REGION_EXIT();
      }

      if (nrfx_usbd_errata_166()) {
        *((volatile uint32_t*) (NRF_USBD_BASE + 0x800)) = 0x7E3;
        *((volatile uint32_t*) (NRF_USBD_BASE + 0x804)) = 0x40;

        __ISB();
        __DSB();
      }
#endif

      // ISO buffer Lower half for IN, upper half for OUT
      NRF_USBD->ISOSPLIT = USBD_ISOSPLIT_SPLIT_HalfIN;

      // Enable bus-reset interrupt
      NRF_USBD->INTENSET = USBD_INTEN_USBRESET_Msk;

      // Enable interrupt, priorities should be set by application
      NVIC_ClearPendingIRQ(USBD_IRQn);

      // Don't enable USBD interrupt yet, if dcd_init() did not finish yet
      // Interrupt will be enabled by tud_init(), when USB stack is ready
      // to handle interrupts.
      if (tud_inited()) {
        NVIC_EnableIRQ(USBD_IRQn);
      }

      // Wait for HFCLK
      while (!hfclk_running()) {}

      // Enable pull up
      NRF_USBD->USBPULLUP = 1;
      __ISB();
      __DSB(); // for sync
      break;

    case USB_EVT_REMOVED:
      if (NRF_USBD->ENABLE) {
        // Abort all transfers

        // Disable pull up
        NRF_USBD->USBPULLUP = 0;
        __ISB();
        __DSB(); // for sync

        // Disable Interrupt
        NVIC_DisableIRQ(USBD_IRQn);

        // disable all interrupt
        NRF_USBD->INTENCLR = NRF_USBD->INTEN;

        NRF_USBD->ENABLE = 0;
        __ISB();
        __DSB(); // for sync

        hfclk_disable();

        dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, is_in_isr());
      }
      break;

    default:
      break;
  }
}

#endif
