/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 SE TEAM
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

#if CFG_TUD_ENABLED && ( CFG_TUSB_MCU == OPT_MCU_MM32F327X )

#include "reg_usb_otg_fs.h"
#include "mm32_device.h"
#include "hal_conf.h"
#include "device/dcd.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

enum {
  TOK_PID_OUT   = 0x1u,
  TOK_PID_IN    = 0x9u,
  TOK_PID_SETUP = 0xDu,
};

typedef struct TU_ATTR_PACKED
{
  union {
    uint32_t head;
    struct {
      union {
        struct {
          uint16_t          :  2;
          uint16_t tok_pid  :  4;
          uint16_t data     :  1;
          uint16_t own      :  1;
          uint16_t          :  8;
        };
        struct {
          uint16_t          :  2;
          uint16_t bdt_stall:  1;
          uint16_t dts      :  1;
          uint16_t ninc     :  1;
          uint16_t keep     :  1;
          uint16_t          : 10;
        };
      };
      uint16_t bc          : 10;
      uint16_t             :  6;
    };
  };
  uint8_t *addr;
}buffer_descriptor_t;

TU_VERIFY_STATIC( sizeof(buffer_descriptor_t) == 8, "size is not correct" );

typedef struct TU_ATTR_PACKED
{
  union {
    uint32_t state;
    struct {
      uint32_t max_packet_size :11;
      uint32_t                 : 5;
      uint32_t odd             : 1;
      uint32_t                 :15;
    };
  };
  uint16_t length;
  uint16_t remaining;
}endpoint_state_t;

TU_VERIFY_STATIC( sizeof(endpoint_state_t) == 8, "size is not correct" );

typedef struct
{
  union {
    /* [#EP][OUT,IN][EVEN,ODD] */
    buffer_descriptor_t bdt[16][2][2];
    uint16_t            bda[512];
  };
  TU_ATTR_ALIGNED(4) union {
    endpoint_state_t endpoint[16][2];
    endpoint_state_t endpoint_unified[16 * 2];
  };
  uint8_t setup_packet[8];
  uint8_t addr;
}dcd_data_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
// BDT(Buffer Descriptor Table) must be 256-byte aligned
CFG_TUD_MEM_SECTION TU_ATTR_ALIGNED(512) static dcd_data_t _dcd;

TU_VERIFY_STATIC( sizeof(_dcd.bdt) == 512, "size is not correct" );

static void prepare_next_setup_packet(uint8_t rhport)
{
  const unsigned out_odd = _dcd.endpoint[0][0].odd;
  const unsigned in_odd  = _dcd.endpoint[0][1].odd;
  if (_dcd.bdt[0][0][out_odd].own) {
    TU_LOG1("DCD fail to prepare the next SETUP %d %d\r\n", out_odd, in_odd);
    return;
  }
  _dcd.bdt[0][0][out_odd].data     = 0;
  _dcd.bdt[0][0][out_odd ^ 1].data = 1;
  _dcd.bdt[0][1][in_odd].data      = 1;
  _dcd.bdt[0][1][in_odd ^ 1].data  = 0;
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_OUT),
                _dcd.setup_packet, sizeof(_dcd.setup_packet));
}

static void process_stall(uint8_t rhport)
{
  if (USB_OTG_FS->EP_CTL[0] & USB_ENDPT_EPSTALL_MASK) {
    /* clear stall condition of the control pipe */
    prepare_next_setup_packet(rhport);
    USB_OTG_FS->EP_CTL[0] &= ~USB_ENDPT_EPSTALL_MASK;
  }
}

static void process_tokdne(uint8_t rhport)
{
  const unsigned s = USB_OTG_FS->STAT;
  USB_OTG_FS->INT_STAT = USB_ISTAT_TOKDNE_MASK; /* fetch the next token if received */
  buffer_descriptor_t *bd = (buffer_descriptor_t *)&_dcd.bda[s];
  endpoint_state_t    *ep = &_dcd.endpoint_unified[s >> 3];
  unsigned odd = (s & USB_STAT_ODD_MASK) ? 1 : 0;

  /* fetch pid before discarded by the next steps */
  const unsigned pid = bd->tok_pid;
  /* reset values for a next transfer */
  bd->bdt_stall = 0;
  bd->dts       = 1;
  bd->ninc      = 0;
  bd->keep      = 0;
  /* update the odd variable to prepare for the next transfer */
  ep->odd       = odd ^ 1;
  if (pid == TOK_PID_SETUP) {
    dcd_event_setup_received(rhport, bd->addr, true);
    USB_OTG_FS->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
    return;
  }
  if (s >> 4) {
    TU_LOG1("TKDNE %x\r\n", s);
  }

  const unsigned bc = bd->bc;
  const unsigned remaining = ep->remaining - bc;
  if (remaining && bc == ep->max_packet_size) {
    /* continue the transferring consecutive data */
    ep->remaining = remaining;
    const int next_remaining = remaining - ep->max_packet_size;
    if (next_remaining > 0) {
      /* prepare to the after next transfer */
      bd->addr += ep->max_packet_size * 2;
      bd->bc    = next_remaining > ep->max_packet_size ? ep->max_packet_size: next_remaining;
      __DSB();
      bd->own   = 1; /* the own bit must set after addr */
    }
    return;
  }
  const unsigned length = ep->length;
  dcd_event_xfer_complete(rhport,
                          ((s & USB_STAT_TX_MASK) << 4) | (s >> USB_STAT_ENDP_SHIFT),
                          length - remaining, XFER_RESULT_SUCCESS, true);
  if (0 == (s & USB_STAT_ENDP_MASK) && 0 == length) {
    /* After completion a ZLP of control transfer,
     * it prepares for the next steup transfer. */
    if (_dcd.addr) {
      /* When the transfer was the SetAddress,
       * the device address should be updated here. */
      USB_OTG_FS->ADDR = _dcd.addr;
      _dcd.addr  = 0;
    }
    prepare_next_setup_packet(rhport);
  }
}

static void process_bus_reset(uint8_t rhport)
{
  USB_OTG_FS->CTL     |= USB_CTL_ODDRST_MASK;
  USB_OTG_FS->ADDR     = 0;
  USB_OTG_FS->INT_ENB    = (USB_OTG_FS->INT_ENB & ~USB_INTEN_RESUMEEN_MASK) | USB_INTEN_SLEEPEN_MASK;

  USB_OTG_FS->EP_CTL[0] = USB_ENDPT_EPHSHK_MASK | USB_ENDPT_EPRXEN_MASK | USB_ENDPT_EPTXEN_MASK;
  for (unsigned i = 1; i < 16; ++i) {
    USB_OTG_FS->EP_CTL[i] = 0;
  }
  buffer_descriptor_t *bd = _dcd.bdt[0][0];
  for (unsigned i = 0; i < sizeof(_dcd.bdt)/sizeof(*bd); ++i, ++bd) {
    bd->head = 0;
  }
  const endpoint_state_t ep0 = {
    .max_packet_size = CFG_TUD_ENDPOINT0_SIZE,
    .odd             = 0,
    .length          = 0,
    .remaining       = 0,
  };
  _dcd.endpoint[0][0] = ep0;
  _dcd.endpoint[0][1] = ep0;
  tu_memclr(_dcd.endpoint[1], sizeof(_dcd.endpoint) - sizeof(_dcd.endpoint[0]));
  _dcd.addr = 0;
  prepare_next_setup_packet(rhport);
  USB_OTG_FS->CTL &= ~USB_CTL_ODDRST_MASK;
  dcd_event_bus_reset(rhport, TUSB_SPEED_FULL, true);
}

static void process_bus_inactive(uint8_t rhport)
{
  (void) rhport;
  const unsigned inten = USB_OTG_FS->INT_ENB;
  USB_OTG_FS->INT_ENB    = (inten & ~USB_INTEN_SLEEPEN_MASK) | USB_INTEN_RESUMEEN_MASK;
  dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
}

static void process_bus_active(uint8_t rhport)
{
  (void) rhport;
  const unsigned inten = USB_OTG_FS->INT_ENB;
  USB_OTG_FS->INT_ENB    = (inten & ~USB_INTEN_RESUMEEN_MASK) | USB_INTEN_SLEEPEN_MASK;
  dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  tu_memclr(&_dcd, sizeof(_dcd));
  USB_OTG_FS->BDT_PAGE_01 = (uint8_t)((uintptr_t)_dcd.bdt >>  8);
  USB_OTG_FS->BDT_PAGE_02 = (uint8_t)((uintptr_t)_dcd.bdt >> 16);
  USB_OTG_FS->BDT_PAGE_03 = (uint8_t)((uintptr_t)_dcd.bdt >> 24);

  dcd_connect(rhport);
  NVIC_ClearPendingIRQ(USB_FS_IRQn);
  return true;
}
#define USB_DEVICE_INTERRUPT_PRIORITY (3U)
void dcd_int_enable(uint8_t rhport)
{
  uint8_t irqNumber;
  irqNumber = USB_FS_IRQn;
  (void) rhport;
  USB_OTG_FS->INT_ENB = USB_INTEN_USBRSTEN_MASK | USB_INTEN_TOKDNEEN_MASK |
    USB_INTEN_SLEEPEN_MASK | USB_INTEN_ERROREN_MASK | USB_INTEN_STALLEN_MASK;
  NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(USB_FS_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
  NVIC_DisableIRQ(USB_FS_IRQn);
  USB_OTG_FS->INT_ENB = 0;
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;
  _dcd.addr = dev_addr & 0x7F;
  /* Response with status first before changing device address */
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

#ifdef __GNUC__ // caused by extra declaration of SystemCoreClock in freeRTOSConfig.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
#endif

extern u32 SystemCoreClock;

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;
  unsigned cnt = SystemCoreClock / 100;
  USB_OTG_FS->CTL |= USB_CTL_RESUME_MASK;
  while (cnt--) __NOP();
  USB_OTG_FS->CTL &= ~USB_CTL_RESUME_MASK;
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  USB_OTG_FS->CTL     |= USB_CTL_USBENSOFEN_MASK;
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  USB_OTG_FS->CTL      = 0;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;

  const unsigned ep_addr  = ep_desc->bEndpointAddress;
  const unsigned epn      = ep_addr & 0xFu;
  const unsigned dir      = (ep_addr & TUSB_DIR_IN_MASK) ? TUSB_DIR_IN : TUSB_DIR_OUT;
  const unsigned xfer     = ep_desc->bmAttributes.xfer;
  endpoint_state_t *ep    = &_dcd.endpoint[epn][dir];
  const unsigned odd      = ep->odd;
  buffer_descriptor_t *bd = &_dcd.bdt[epn][dir][0];

  /* No support for control transfer */
  TU_ASSERT(epn && (xfer != TUSB_XFER_CONTROL));

  ep->max_packet_size = tu_edpt_packet_size(ep_desc);
  unsigned val = USB_ENDPT_EPCTLDIS_MASK;
  val |= (xfer != TUSB_XFER_ISOCHRONOUS) ? USB_ENDPT_EPHSHK_MASK: 0;
  val |= dir ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;
  USB_OTG_FS->EP_CTL[epn] |= val;

  if (xfer != TUSB_XFER_ISOCHRONOUS) {
    bd[odd].dts      = 1;
    bd[odd].data     = 0;
    bd[odd ^ 1].dts  = 1;
    bd[odd ^ 1].data = 1;
  }

  return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
  (void) rhport;
  // TODO implement dcd_edpt_close_all()
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;

  const unsigned epn      = ep_addr & 0xFu;
  const unsigned dir      = (ep_addr & TUSB_DIR_IN_MASK) ? TUSB_DIR_IN : TUSB_DIR_OUT;
  endpoint_state_t *ep    = &_dcd.endpoint[epn][dir];
  buffer_descriptor_t *bd = &_dcd.bdt[epn][dir][0];
  const unsigned msk      = dir ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;
  USB_OTG_FS->EP_CTL[epn] &= ~msk;
  ep->max_packet_size = 0;
  ep->length          = 0;
  ep->remaining       = 0;
  bd->head            = 0;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes)
{
  (void) rhport;
  NVIC_DisableIRQ(USB_FS_IRQn);
  const unsigned epn = ep_addr & 0xFu;
  const unsigned dir = (ep_addr & TUSB_DIR_IN_MASK) ? TUSB_DIR_IN : TUSB_DIR_OUT;
  endpoint_state_t    *ep = &_dcd.endpoint[epn][dir];
  buffer_descriptor_t *bd = &_dcd.bdt[epn][dir][ep->odd];

  if (bd->own) {
    TU_LOG1("DCD XFER fail %x %d %lx %lx\r\n", ep_addr, total_bytes, ep->state, bd->head);
    return false; /* The last transfer has not completed */
  }
  ep->length    = total_bytes;
  ep->remaining = total_bytes;

  const unsigned mps = ep->max_packet_size;
  if (total_bytes > mps) {
    buffer_descriptor_t *next = ep->odd ? bd - 1: bd + 1;
    /* When total_bytes is greater than the max packet size,
     * it prepares to the next transfer to avoid NAK in advance. */
    next->bc   = total_bytes >= 2 * mps ? mps: total_bytes - mps;
    next->addr = buffer + mps;
    next->own  = 1;
  }
  bd->bc        = total_bytes >= mps ? mps: total_bytes;
  bd->addr      = buffer;
  __DSB();
  bd->own  = 1; /* the own bit must set after addr */
  NVIC_EnableIRQ(USB_FS_IRQn);
  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  const unsigned epn = ep_addr & 0xFu;
  if (0 == epn) {
    USB_OTG_FS->EP_CTL[epn] |=  USB_ENDPT_EPSTALL_MASK;
  } else {
    const unsigned dir      = (ep_addr & TUSB_DIR_IN_MASK) ? TUSB_DIR_IN : TUSB_DIR_OUT;
    buffer_descriptor_t *bd = _dcd.bdt[epn][dir];
    bd[0].bdt_stall = 1;
    bd[1].bdt_stall = 1;
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  const unsigned epn      = ep_addr & 0xFu;
  const unsigned dir      = (ep_addr & TUSB_DIR_IN_MASK) ? TUSB_DIR_IN : TUSB_DIR_OUT;
  const unsigned odd      = _dcd.endpoint[epn][dir].odd;
  buffer_descriptor_t *bd = _dcd.bdt[epn][dir];

  bd[odd ^ 1].own       = 0;
  bd[odd ^ 1].data      = 1;
  bd[odd ^ 1].bdt_stall = 0;
  bd[odd].own           = 0;
  bd[odd].data          = 0;
  bd[odd].bdt_stall     = 0;
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
void dcd_int_handler(uint8_t rhport)
{
  (void) rhport;

  uint32_t is  = USB_OTG_FS->INT_STAT;
  uint32_t msk = USB_OTG_FS->INT_ENB;
  USB_OTG_FS->INT_STAT = is & ~msk;
  is &= msk;
  if (is & USB_ISTAT_ERROR_MASK) {
    /* TODO: */
    uint32_t es = USB_OTG_FS->ERR_STAT;
    USB_OTG_FS->ERR_STAT = es;
    USB_OTG_FS->INT_STAT   = is; /* discard any pending events */
    return;
  }

  if (is & USB_ISTAT_USBRST_MASK) {
    USB_OTG_FS->INT_STAT = is; /* discard any pending events */
    process_bus_reset(rhport);
    return;
  }
  if (is & USB_ISTAT_SLEEP_MASK) {
    USB_OTG_FS->INT_STAT = USB_ISTAT_SLEEP_MASK;
    process_bus_inactive(rhport);
    return;
  }
  if (is & USB_ISTAT_RESUME_MASK) {
    USB_OTG_FS->INT_STAT = USB_ISTAT_RESUME_MASK;
    process_bus_active(rhport);
    return;
  }
  if (is & USB_ISTAT_SOFTOK_MASK) {
    USB_OTG_FS->INT_STAT = USB_ISTAT_SOFTOK_MASK;
    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
    return;
  }
  if (is & USB_ISTAT_STALL_MASK) {
    USB_OTG_FS->INT_STAT = USB_ISTAT_STALL_MASK;
    process_stall(rhport);
    return;
  }
  if (is & USB_ISTAT_TOKDNE_MASK) {
    process_tokdne(rhport);
    return;
  }
}

#endif
