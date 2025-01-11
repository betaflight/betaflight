/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Koji Kitayama
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

#if CFG_TUD_ENABLED && defined(TUP_USBIP_CHIPIDEA_FS)

#include "device/dcd.h"
#include "ci_fs_type.h"

#if defined(TUP_USBIP_CHIPIDEA_FS_KINETIS)
  #include "ci_fs_kinetis.h"
#elif defined(TUP_USBIP_CHIPIDEA_FS_MCX)
  #include "ci_fs_mcx.h"
#else
  #error "MCU is not supported"
#endif

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
               uint16_t           :  2;
          __IO uint16_t tok_pid   :  4;
               uint16_t data      :  1;
          __IO uint16_t own       :  1;
               uint16_t           :  8;
        };
        struct {
               uint16_t           :  2;
               uint16_t bdt_stall :  1;
               uint16_t dts       :  1;
               uint16_t ninc      :  1;
               uint16_t keep      :  1;
               uint16_t           : 10;
        };
      };
      __IO uint16_t bc : 10;
           uint16_t    :  6;
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
  TU_ASSERT(0 == _dcd.bdt[0][0][out_odd].own, );

  _dcd.bdt[0][0][out_odd].data     = 0;
  _dcd.bdt[0][0][out_odd ^ 1].data = 1;
  _dcd.bdt[0][1][in_odd].data      = 1;
  _dcd.bdt[0][1][in_odd ^ 1].data  = 0;
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_OUT),
                _dcd.setup_packet, sizeof(_dcd.setup_packet));
}

static void process_stall(uint8_t rhport)
{
  for (int i = 0; i < 16; ++i) {
    uint32_t const ep_ctl = CI_REG->EP[i].CTL;

    if (ep_ctl & USB_ENDPT_EPSTALL_MASK) {
      // prepare next setup if endpoint0
      if ( i == 0 ) prepare_next_setup_packet(rhport);

      // clear stall bit
      CI_REG->EP[i].CTL = ep_ctl & ~USB_ENDPT_EPSTALL_MASK;
    }
  }
}

static void process_tokdne(uint8_t rhport)
{
  const unsigned s = CI_REG->STAT;
  CI_REG->INT_STAT = USB_ISTAT_TOKDNE_MASK; /* fetch the next token if received */

  uint8_t const epnum = (s >> USB_STAT_ENDP_SHIFT);
  uint8_t const dir   = (s & USB_STAT_TX_MASK) >> USB_STAT_TX_SHIFT;
  unsigned const odd  = (s & USB_STAT_ODD_MASK) ? 1 : 0;

  buffer_descriptor_t *bd = (buffer_descriptor_t *)&_dcd.bda[s];
  endpoint_state_t    *ep = &_dcd.endpoint_unified[s >> 3];

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
    CI_REG->CTL &= ~USB_CTL_TXSUSPENDTOKENBUSY_MASK;
    return;
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
                          tu_edpt_addr(epnum, dir),
                          length - remaining, XFER_RESULT_SUCCESS, true);
  if (0 == epnum && 0 == length) {
    /* After completion a ZLP of control transfer,
     * it prepares for the next steup transfer. */
    if (_dcd.addr) {
      /* When the transfer was the SetAddress,
       * the device address should be updated here. */
      CI_REG->ADDR = _dcd.addr;
      _dcd.addr  = 0;
    }
    prepare_next_setup_packet(rhport);
  }
}

static void process_bus_reset(uint8_t rhport)
{
  CI_REG->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
  CI_REG->CTL     |= USB_CTL_ODDRST_MASK;
  CI_REG->ADDR     = 0;
  CI_REG->INT_EN   = USB_INTEN_USBRSTEN_MASK | USB_INTEN_TOKDNEEN_MASK | USB_INTEN_SLEEPEN_MASK |
                     USB_INTEN_ERROREN_MASK | USB_INTEN_STALLEN_MASK;

  CI_REG->EP[0].CTL = USB_ENDPT_EPHSHK_MASK | USB_ENDPT_EPRXEN_MASK | USB_ENDPT_EPTXEN_MASK;
  for (unsigned i = 1; i < 16; ++i) {
    CI_REG->EP[i].CTL = 0;
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
  CI_REG->CTL &= ~USB_CTL_ODDRST_MASK;
  dcd_event_bus_reset(rhport, TUSB_SPEED_FULL, true);
}

static void process_bus_sleep(uint8_t rhport)
{
  // Enable resume & disable suspend interrupt
  const unsigned inten = CI_REG->INT_EN;

  CI_REG->INT_EN   = (inten & ~USB_INTEN_SLEEPEN_MASK) | USB_INTEN_RESUMEEN_MASK;
  CI_REG->USBTRC0 |= USB_USBTRC0_USBRESMEN_MASK;
  CI_REG->USBCTRL |= USB_USBCTRL_SUSP_MASK;

  dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
}

static void process_bus_resume(uint8_t rhport)
{
  // Enable suspend & disable resume interrupt
  const unsigned inten = CI_REG->INT_EN;

  CI_REG->USBCTRL &= ~USB_USBCTRL_SUSP_MASK; // will also clear USB_USBTRC0_USB_RESUME_INT_MASK
  CI_REG->USBTRC0 &= ~USB_USBTRC0_USBRESMEN_MASK;
  CI_REG->INT_EN   = (inten & ~USB_INTEN_RESUMEEN_MASK) | USB_INTEN_SLEEPEN_MASK;

  dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  // save crystal-less setting (if available)
  #if defined(FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED) && FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED == 1
  uint32_t clk_recover_irc_en = CI_REG->CLK_RECOVER_IRC_EN;
  uint32_t clk_recover_ctrl = CI_REG->CLK_RECOVER_CTRL;
  #endif

  CI_REG->USBTRC0 |= USB_USBTRC0_USBRESET_MASK;
  while (CI_REG->USBTRC0 & USB_USBTRC0_USBRESET_MASK);

  // restore crystal-less setting (if available)
  #if defined(FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED) && FSL_FEATURE_USB_KHCI_IRC48M_MODULE_CLOCK_ENABLED == 1
  CI_REG->CLK_RECOVER_IRC_EN = clk_recover_irc_en;
  CI_REG->CLK_RECOVER_CTRL  |= clk_recover_ctrl;
  #endif

  tu_memclr(&_dcd, sizeof(_dcd));
  CI_REG->USBTRC0 |= TU_BIT(6); /* software must set this bit to 1 */
  CI_REG->BDT_PAGE1 = (uint8_t)((uintptr_t)_dcd.bdt >> 8);
  CI_REG->BDT_PAGE2 = (uint8_t)((uintptr_t)_dcd.bdt >> 16);
  CI_REG->BDT_PAGE3 = (uint8_t)((uintptr_t)_dcd.bdt >> 24);

  CI_REG->INT_EN = USB_INTEN_USBRSTEN_MASK;

  dcd_connect(rhport);
  // NVIC_ClearPendingIRQ(CIFS_IRQN);
  return true;
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  _dcd.addr = dev_addr & 0x7F;
  /* Response with status first before changing device address */
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;

  CI_REG->CTL |= USB_CTL_RESUME_MASK;

  unsigned cnt = SystemCoreClock / 1000;
  while (cnt--) __NOP();

  CI_REG->CTL &= ~USB_CTL_RESUME_MASK;
}

void dcd_connect(uint8_t rhport)
{
  (void) rhport;
  CI_REG->USBCTRL  = 0;
  CI_REG->CONTROL |= USB_CONTROL_DPPULLUPNONOTG_MASK;
  CI_REG->CTL     |= USB_CTL_USBENSOFEN_MASK;
}

void dcd_disconnect(uint8_t rhport)
{
  (void) rhport;
  CI_REG->CTL      = 0;
  CI_REG->CONTROL &= ~USB_CONTROL_DPPULLUPNONOTG_MASK;
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
  const unsigned epn      = tu_edpt_number(ep_addr);
  const unsigned dir      = tu_edpt_dir(ep_addr);
  const unsigned xfer     = ep_desc->bmAttributes.xfer;
  endpoint_state_t *ep    = &_dcd.endpoint[epn][dir];
  const unsigned odd      = ep->odd;
  buffer_descriptor_t *bd = _dcd.bdt[epn][dir];

  /* No support for control transfer */
  TU_ASSERT(epn && (xfer != TUSB_XFER_CONTROL));

  ep->max_packet_size = tu_edpt_packet_size(ep_desc);
  unsigned val = USB_ENDPT_EPCTLDIS_MASK;
  val |= (xfer != TUSB_XFER_ISOCHRONOUS) ? USB_ENDPT_EPHSHK_MASK: 0;
  val |= dir ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;
  CI_REG->EP[epn].CTL |= val;

  if (xfer != TUSB_XFER_ISOCHRONOUS) {
    bd[odd].dts      = 1;
    bd[odd].data     = 0;
    bd[odd ^ 1].dts  = 1;
    bd[odd ^ 1].data = 1;
  }

  return true;
}

void dcd_edpt_close_all(uint8_t rhport)
{
  dcd_int_disable(rhport);

  for (unsigned i = 1; i < 16; ++i) {
    CI_REG->EP[i].CTL = 0;
  }

  dcd_int_enable(rhport);

  buffer_descriptor_t *bd = _dcd.bdt[1][0];
  for (unsigned i = 2; i < sizeof(_dcd.bdt)/sizeof(*bd); ++i, ++bd) {
    bd->head = 0;
  }

  endpoint_state_t *ep = &_dcd.endpoint[1][0];
  for (unsigned i = 2; i < sizeof(_dcd.endpoint)/sizeof(*ep); ++i, ++ep) {
    /* Clear except the odd */
    ep->max_packet_size = 0;
    ep->length          = 0;
    ep->remaining       = 0;
  }
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  const unsigned epn      = tu_edpt_number(ep_addr);
  const unsigned dir      = tu_edpt_dir(ep_addr);
  endpoint_state_t *ep    = &_dcd.endpoint[epn][dir];
  buffer_descriptor_t *bd = _dcd.bdt[epn][dir];
  const unsigned msk      = dir ? USB_ENDPT_EPTXEN_MASK : USB_ENDPT_EPRXEN_MASK;

  dcd_int_disable(rhport);

  CI_REG->EP[epn].CTL &= ~msk;
  ep->max_packet_size = 0;
  ep->length          = 0;
  ep->remaining       = 0;
  bd[0].head          = 0;
  bd[1].head          = 0;

  dcd_int_enable(rhport);
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes)
{
  const unsigned epn      = tu_edpt_number(ep_addr);
  const unsigned dir      = tu_edpt_dir(ep_addr);
  endpoint_state_t    *ep = &_dcd.endpoint[epn][dir];
  buffer_descriptor_t *bd = &_dcd.bdt[epn][dir][ep->odd];
  TU_ASSERT(0 == bd->own);

  dcd_int_disable(rhport);

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
  bd->bc   = total_bytes >= mps ? mps: total_bytes;
  bd->addr = buffer;
  __DSB();
  bd->own  = 1; /* This bit must be set last */

  dcd_int_enable(rhport);

  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  const unsigned epn = tu_edpt_number(ep_addr);

  if (0 == epn) {
    CI_REG->EP[epn].CTL |=  USB_ENDPT_EPSTALL_MASK;
  } else {
    const unsigned dir      = tu_edpt_dir(ep_addr);
    const unsigned odd      = _dcd.endpoint[epn][dir].odd;
    buffer_descriptor_t *bd = &_dcd.bdt[epn][dir][odd];
    TU_ASSERT(0 == bd->own,);

    dcd_int_disable(rhport);

    bd->bdt_stall = 1;
    __DSB();
    bd->own       = 1; /* This bit must be set last */

    dcd_int_enable(rhport);
  }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  const unsigned epn      = tu_edpt_number(ep_addr);
  TU_VERIFY(epn,);
  const unsigned dir      = tu_edpt_dir(ep_addr);
  const unsigned odd      = _dcd.endpoint[epn][dir].odd;
  buffer_descriptor_t *bd = _dcd.bdt[epn][dir];
  TU_VERIFY(bd[odd].own,);

  dcd_int_disable(rhport);

  bd[odd].own = 0;
  __DSB();

  // clear stall
  bd[odd].bdt_stall  = 0;

  // Reset data toggle
  bd[odd    ].data = 0;
  bd[odd ^ 1].data = 1;

  // We already cleared this in ISR, but just clear it here to be safe
  const uint32_t ep_ctl = CI_REG->EP[epn].CTL;
  if (ep_ctl & USB_ENDPT_EPSTALL_MASK) {
    CI_REG->EP[epn].CTL = ep_ctl & ~USB_ENDPT_EPSTALL_MASK;
  }

  dcd_int_enable(rhport);
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
void dcd_int_handler(uint8_t rhport)
{
  uint32_t is  = CI_REG->INT_STAT;
  uint32_t msk = CI_REG->INT_EN;

  // clear non-enabled interrupts
  CI_REG->INT_STAT = is & ~msk;
  is &= msk;

  if (is & USB_ISTAT_ERROR_MASK) {
    /* TODO: */
    uint32_t es = CI_REG->ERR_STAT;
    CI_REG->ERR_STAT = es;
    CI_REG->INT_STAT = is; /* discard any pending events */
  }

  if (is & USB_ISTAT_USBRST_MASK) {
    CI_REG->INT_STAT = is; /* discard any pending events */
    process_bus_reset(rhport);
  }

  if (is & USB_ISTAT_SLEEP_MASK) {
    // TU_LOG2("Suspend: "); TU_LOG2_HEX(is);

    // Note Host usually has extra delay after bus reset (without SOF), which could falsely
    // detected as Sleep event. Though usbd has debouncing logic so we are good
    CI_REG->INT_STAT = USB_ISTAT_SLEEP_MASK;
    process_bus_sleep(rhport);
  }

#if 0 // ISTAT_RESUME never trigger, probably for host mode ?
  if (is & USB_ISTAT_RESUME_MASK) {
    // TU_LOG2("ISTAT Resume: "); TU_LOG2_HEX(is);
    KHCI->ISTAT = USB_ISTAT_RESUME_MASK;
    process_bus_resume(rhport);
  }
#endif

  if (CI_REG->USBTRC0 & USB_USBTRC0_USB_RESUME_INT_MASK) {
     // TU_LOG2("USBTRC0 Resume: "); TU_LOG2_HEX(is); TU_LOG2_HEX(KHCI->USBTRC0);
    process_bus_resume(rhport);
  }

  if (is & USB_ISTAT_SOFTOK_MASK) {
    CI_REG->INT_STAT = USB_ISTAT_SOFTOK_MASK;
    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
  }

  if (is & USB_ISTAT_STALL_MASK) {
    CI_REG->INT_STAT = USB_ISTAT_STALL_MASK;
    process_stall(rhport);
  }

  if (is & USB_ISTAT_TOKDNE_MASK) {
    process_tokdne(rhport);
  }
}

#endif
