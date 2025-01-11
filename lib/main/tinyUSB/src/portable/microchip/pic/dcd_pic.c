/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Koji Kitayama
 * Copyright (c) 2022 Reimu NotMoe <reimu@sudomaker.com>
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

#if CFG_TUD_ENABLED && \
    (CFG_TUSB_MCU == OPT_MCU_PIC32MX || CFG_TUSB_MCU == OPT_MCU_PIC32MM || \
     CFG_TUSB_MCU == OPT_MCU_PIC32MK || CFG_TUSB_MCU == OPT_MCU_PIC24 || \
     CFG_TUSB_MCU == OPT_MCU_DSPIC33)

#include <xc.h>

#include "device/dcd.h"


#if (CFG_TUSB_MCU == OPT_MCU_PIC32MX || CFG_TUSB_MCU == OPT_MCU_PIC32MM || CFG_TUSB_MCU == OPT_MCU_PIC32MK)

#define TU_PIC_INT_SIZE         4

#elif (CFG_TUSB_MCU == OPT_MCU_PIC24 || CFG_TUSB_MCU == OPT_MCU_DSPIC33)

#define TU_PIC_INT_SIZE         2

#else

#error Unsupportd PIC MCU

#endif


#if TU_PIC_INT_SIZE == 4

#ifndef KVA_TO_PA
#define KVA_TO_PA(kva)    ((uint32_t)(kva) & 0x1fffffff)
#endif

#ifndef PA_TO_KVA1
#define PA_TO_KVA1(pa)    ((uint32_t)(pa) | 0xA0000000)
#endif

#else

#ifndef KVA_TO_PA
#define KVA_TO_PA(kva)    (kva)
#endif

#ifndef PA_TO_KVA1
#define PA_TO_KVA1(pa)    (pa)
#endif

#endif


//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

enum {
  TOK_PID_OUT   = 0x1u,
  TOK_PID_IN    = 0x9u,
  TOK_PID_SETUP = 0xDu,
};

// The BDT is 8 bytes on 32bit PICs and 4 bytes on 8/16bit PICs
#if TU_PIC_INT_SIZE == 4
typedef struct TU_ATTR_PACKED
{
  union {
    uint32_t head;
    struct {
      union {
        struct {
          uint16_t           :  2;
          uint16_t tok_pid   :  4;
          uint16_t data      :  1;
          uint16_t own       :  1;
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
      uint16_t bc : 10;
      uint16_t    :  6;
    };
  };
  uint8_t *addr;
} buffer_descriptor_t;

TU_VERIFY_STATIC( sizeof(buffer_descriptor_t) == 8, "size is not correct" );
#else
typedef struct TU_ATTR_PACKED
{
  union {
    uint16_t head;

    struct {
      uint16_t           :  10;
      uint16_t tok_pid   :  4;
      uint16_t data      :  1;
      uint16_t own       :  1;
    };
    struct {
      uint16_t           :  10;
      uint16_t bdt_stall :  1;
      uint16_t dts       :  1;
      uint16_t ninc      :  1;
      uint16_t keep      :  1;
    };

    struct {
      uint16_t bc : 10;
      uint16_t    :  6;
    };
  };
  uint8_t *addr;
} buffer_descriptor_t;

TU_VERIFY_STATIC( sizeof(buffer_descriptor_t) == 4, "size is not correct" );
#endif


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
} endpoint_state_t;

TU_VERIFY_STATIC( sizeof(endpoint_state_t) == 8, "size is not correct" );

typedef struct
{
  union {
    /* [#EP][OUT,IN][EVEN,ODD] */
    buffer_descriptor_t bdt[16][2][2];
#if TU_PIC_INT_SIZE == 4
    uint16_t            bda[256];
#else
    uint8_t            bda[256];
#endif
  };
  TU_ATTR_ALIGNED(4) union {
    endpoint_state_t endpoint[16][2];
    endpoint_state_t endpoint_unified[16 * 2];
  };
  uint8_t setup_packet[8];
  uint8_t addr;
} dcd_data_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
// BDT(Buffer Descriptor Table) must be 256-byte aligned
CFG_TUD_MEM_SECTION TU_ATTR_ALIGNED(512) volatile static dcd_data_t _dcd;

#if TU_PIC_INT_SIZE == 4
TU_VERIFY_STATIC( sizeof(_dcd.bdt) == 512, "size is not correct" );
#else
TU_VERIFY_STATIC( sizeof(_dcd.bdt) == 256, "size is not correct" );
#endif

#if TU_PIC_INT_SIZE == 4
typedef uint32_t ep_reg_t;
#elif TU_PIC_INT_SIZE == 2
typedef uint16_t ep_reg_t;
#endif

static inline volatile void *ep_addr(uint8_t rhport, uint8_t ep_num) {
#if CFG_TUSB_MCU == OPT_MCU_PIC32MK
  volatile void *ep_reg_base = rhport ? (&U2EP0) : (&U1EP0);
#else
  volatile void *ep_reg_base = &U1EP0;
#endif
#if TU_PIC_INT_SIZE == 4
  const size_t offset = 0x10;
#else
  const size_t offset = 0x2;
#endif
  return ep_reg_base + offset * ep_num;
}

static inline ep_reg_t ep_read(uint8_t rhport, uint8_t ep_num) {
  volatile ep_reg_t *ep = ep_addr(rhport, ep_num);
  return *ep;
}

static inline void ep_write(uint8_t rhport, uint8_t ep_num, ep_reg_t val) {
  volatile ep_reg_t *ep = ep_addr(rhport, ep_num);
  *ep = val;
}

static inline void ep_clear(uint8_t rhport, uint8_t ep_num, ep_reg_t val) {
#if TU_PIC_INT_SIZE == 4
  volatile ep_reg_t *ep_clr = (ep_addr(rhport, ep_num) + 0x4);
  *ep_clr = val;
#else
  ep_reg_t v = ep_read(rhport, ep_num);
  v &= ~val;
  ep_write(rhport, ep_num, v);
#endif
}

static inline void ep_set(uint8_t rhport, uint8_t ep_num, ep_reg_t val) {
#if TU_PIC_INT_SIZE == 4
  volatile ep_reg_t *ep_s = (ep_addr(rhport, ep_num) + 0x8);
  *ep_s = val;
#else
  ep_reg_t v = ep_read(rhport, ep_num);
  v |= val;
  ep_write(rhport, ep_num, v);
#endif
}

static inline void intr_enable(uint8_t rhport) {
#if CFG_TUSB_MCU == OPT_MCU_PIC32MM
  IEC0SET = _IEC0_USBIE_MASK;
#elif CFG_TUSB_MCU == OPT_MCU_PIC32MX
  IEC1SET = _IEC1_USBIE_MASK;
#elif CFG_TUSB_MCU == OPT_MCU_PIC32MK
  if (rhport == 0)
    IEC1SET = _IEC1_USB1IE_MASK;
  else
    IEC7SET = _IEC7_USB2IE_MASK;
#elif (CFG_TUSB_MCU == OPT_MCU_PIC24) || (CFG_TUSB_MCU == OPT_MCU_DSPIC33)
  IEC5bits.USB1IE = 1;
#endif
}

static inline void intr_disable(uint8_t rhport) {
#if CFG_TUSB_MCU == OPT_MCU_PIC32MM
  IEC0CLR = _IEC0_USBIE_MASK;
#elif CFG_TUSB_MCU == OPT_MCU_PIC32MX
  IEC1CLR = _IEC1_USBIE_MASK;
#elif CFG_TUSB_MCU == OPT_MCU_PIC32MK
  if (rhport == 0)
    IEC1CLR = _IEC1_USB1IE_MASK;
  else
    IEC7CLR = _IEC7_USB2IE_MASK;
#elif (CFG_TUSB_MCU == OPT_MCU_PIC24) || (CFG_TUSB_MCU == OPT_MCU_DSPIC33)
  IEC5bits.USB1IE = 0;
#endif
}

static inline int intr_is_enabled(uint8_t rhport) {
#if CFG_TUSB_MCU == OPT_MCU_PIC32MM
  return IEC0bits.USBIE;
#elif CFG_TUSB_MCU == OPT_MCU_PIC32MX
  return IEC1bits.USBIE;
#elif CFG_TUSB_MCU == OPT_MCU_PIC32MK
  if (rhport == 0)
    return IEC1bits.USB1IE;
  else
    return IEC7bits.USB2IE;
#elif (CFG_TUSB_MCU == OPT_MCU_PIC24) || (CFG_TUSB_MCU == OPT_MCU_DSPIC33)
  return IEC5bits.USB1IE;
#endif
}

static inline void intr_clear(uint8_t rhport) {
#if CFG_TUSB_MCU == OPT_MCU_PIC32MM
  IFS0CLR = _IFS0_USBIF_MASK;
#elif CFG_TUSB_MCU == OPT_MCU_PIC32MX
  IFS1CLR = _IFS1_USBIF_MASK;
#elif CFG_TUSB_MCU == OPT_MCU_PIC32MK
  if (rhport == 0)
    IFS1CLR = _IFS1_USB1IF_MASK;
  else
    IFS7CLR = _IFS7_USB2IF_MASK;
#elif (CFG_TUSB_MCU == OPT_MCU_PIC24) || (CFG_TUSB_MCU == OPT_MCU_DSPIC33)
  IFS5bits.USB1IF = 0;
#endif
}

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
    unsigned const endpt = ep_read(rhport, i);

    if (endpt & _U1EP0_EPSTALL_MASK) {
      // prepare next setup if endpoint0
      if ( i == 0 ) prepare_next_setup_packet(rhport);

      // clear stall bit
      ep_clear(rhport, i, _U1EP0_EPSTALL_MASK);
    }
  }
}

static void process_tokdne(uint8_t rhport)
{
  ep_reg_t s = U1STAT;

  U1IR = _U1IR_TRNIF_MASK;

  uint8_t epnum = (s >> _U1STAT_ENDPT0_POSITION);
  uint8_t dir   = (s & _U1STAT_DIR_MASK) >> _U1STAT_DIR_POSITION;
  unsigned odd  = (s & _U1STAT_PPBI_MASK) ? 1 : 0;

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
    dcd_event_setup_received(rhport, (uint8_t *)PA_TO_KVA1(bd->addr), true);
#if TU_PIC_INT_SIZE == 4
    U1CONCLR = _U1CON_PKTDIS_TOKBUSY_MASK;
#else
    U1CONbits.PKTDIS = 0;
#endif
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
      U1ADDR = _dcd.addr;
      _dcd.addr  = 0;
    }
    prepare_next_setup_packet(rhport);
  }
}

static void process_bus_reset(uint8_t rhport)
{
#if TU_PIC_INT_SIZE == 4
  U1PWRCCLR = _U1PWRC_USUSPEND_MASK;
  U1CONSET = _U1CON_PPBRST_MASK;
#else
  U1PWRCbits.USUSPND = 0;
  U1CONbits.PPBRST = 1;
#endif
  U1ADDR = 0;

  U1IE = _U1IE_URSTIE_MASK | _U1IE_TRNIE_MASK | _U1IE_IDLEIE_MASK |
         _U1IE_UERRIE_MASK | _U1IE_STALLIE_MASK;

  U1EP0 = _U1EP0_EPHSHK_MASK | _U1EP0_EPRXEN_MASK | _U1EP0_EPTXEN_MASK;

  for (unsigned i = 1; i < 16; ++i) {
    ep_write(rhport, i, 0);
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
#if TU_PIC_INT_SIZE == 4
  U1CONCLR = _U1CON_PPBRST_MASK;
#else
  U1CONbits.PPBRST = 0;
#endif
  dcd_event_bus_reset(rhport, TUSB_SPEED_FULL, true);
}

static void process_bus_sleep(uint8_t rhport)
{
  // Enable resume & disable suspend interrupt
  dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
}

static void process_bus_resume(uint8_t rhport)
{
  // Enable suspend & disable resume interrupt
#if TU_PIC_INT_SIZE == 4
  U1PWRCCLR = _U1PWRC_USUSPEND_MASK;
  U1IECLR = _U1IE_RESUMEIE_MASK;
  U1IESET = _U1IE_IDLEIE_MASK;
#else
  U1PWRCbits.USUSPND = 0;
  U1IEbits.RESUMEIE = 0;
  U1IEbits.IDLEIE = 1;
#endif

  dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
}

/*------------------------------------------------------------------*/
/* Device API
 *------------------------------------------------------------------*/
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  intr_disable(rhport);
  intr_clear(rhport);

#if CFG_TUSB_MCU == OPT_MCU_PIC32MM
  TRISBbits.TRISB6 = 1;
#endif

  tu_memclr(&_dcd, sizeof(_dcd));

#if TU_PIC_INT_SIZE == 4
  U1PWRCSET = _U1PWRC_USBPWR_MASK;
#else
  U1PWRCbits.USBPWR = 1;
#endif

#if TU_PIC_INT_SIZE == 4
  uint32_t bdt_phys = KVA_TO_PA((uintptr_t)_dcd.bdt);

  U1BDTP1 = (uint8_t)(bdt_phys >>  8);
  U1BDTP2 = (uint8_t)(bdt_phys >> 16);
  U1BDTP3 = (uint8_t)(bdt_phys >> 24);
#else
  U1BDTP1 = (uint8_t)((uint16_t)(void *)_dcd.bdt >> 8);

  U1CNFG1bits.PPB = 2;
#endif

  U1IE = _U1IE_URSTIE_MASK;

  dcd_connect(rhport);
  return true;
}

void dcd_int_enable(uint8_t rhport)
{
  intr_enable(rhport);
}

void dcd_int_disable(uint8_t rhport)
{
  intr_disable(rhport);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  _dcd.addr = dev_addr & 0x7F;
  /* Response with status first before changing device address */
  dcd_edpt_xfer(rhport, tu_edpt_addr(0, TUSB_DIR_IN), NULL, 0);
}

void dcd_remote_wakeup(uint8_t rhport)
{
#if TU_PIC_INT_SIZE == 4
  U1CONSET = _U1CON_RESUME_MASK;
#else
  U1CONbits.RESUME = 1;
#endif
  unsigned cnt = 25000000 / 1000;
  while (cnt--) asm volatile("nop");

#if TU_PIC_INT_SIZE == 4
  U1CONCLR = _U1CON_RESUME_MASK;
#else
  U1CONbits.RESUME = 0;
#endif
}

void dcd_connect(uint8_t rhport)
{
  while (!U1CONbits.USBEN) {
#if TU_PIC_INT_SIZE == 4
    U1CONSET = _U1CON_USBEN_SOFEN_MASK;
#else
    U1CONbits.USBEN = 1;
#endif
  }
}

void dcd_disconnect(uint8_t rhport)
{
  U1CON = 0;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
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


  unsigned val = _U1EP0_EPCONDIS_MASK;
  val |= (xfer != TUSB_XFER_ISOCHRONOUS) ? _U1EP0_EPHSHK_MASK : 0;
  val |= dir ? _U1EP0_EPTXEN_MASK : _U1EP0_EPRXEN_MASK;

  ep_reg_t tmp = ep_read(rhport, epn);
  tmp |= val;
  ep_write(rhport, epn, tmp);

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
  const unsigned ie = intr_is_enabled(rhport);
  intr_disable(rhport);

  for (unsigned i = 1; i < 16; ++i) {
    ep_write(rhport, i, 0);
  }

  if (ie) intr_enable(rhport);

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
  const unsigned msk      = dir ? _U1EP0_EPTXEN_MASK : _U1EP0_EPRXEN_MASK;
  const unsigned ie       = intr_is_enabled(rhport);

  intr_disable(rhport);

  ep_clear(rhport, epn, msk);

  ep->max_packet_size = 0;
  ep->length          = 0;
  ep->remaining       = 0;
  bd[0].head          = 0;
  bd[1].head          = 0;

  if (ie) intr_enable(rhport);
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes)
{

  const unsigned epn      = tu_edpt_number(ep_addr);
  const unsigned dir      = tu_edpt_dir(ep_addr);
  endpoint_state_t    *ep = &_dcd.endpoint[epn][dir];
  buffer_descriptor_t *bd = &_dcd.bdt[epn][dir][ep->odd];
  TU_ASSERT(0 == bd->own);

  const unsigned ie       = intr_is_enabled(rhport);

  intr_disable(rhport);

  ep->length    = total_bytes;
  ep->remaining = total_bytes;

  const unsigned mps = ep->max_packet_size;
  if (total_bytes > mps) {
    buffer_descriptor_t *next = ep->odd ? bd - 1: bd + 1;
    /* When total_bytes is greater than the max packet size,
     * it prepares to the next transfer to avoid NAK in advance. */
    next->bc   = total_bytes >= 2 * mps ? mps: total_bytes - mps;
    next->addr = (uint8_t *)KVA_TO_PA(buffer + mps);
    next->own  = 1;
  }
  bd->bc   = total_bytes >= mps ? mps: total_bytes;
  bd->addr = (uint8_t *)KVA_TO_PA(buffer);
  bd->own  = 1; /* This bit must be set last */

  if (ie) intr_enable(rhport);

  return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void) rhport;
  const unsigned epn = tu_edpt_number(ep_addr);

  if (0 == epn) {
    ep_set(rhport, epn, _U1EP0_EPSTALL_MASK);
  } else {
    const unsigned dir      = tu_edpt_dir(ep_addr);
    const unsigned odd      = _dcd.endpoint[epn][dir].odd;
    buffer_descriptor_t *bd = &_dcd.bdt[epn][dir][odd];
    TU_ASSERT(0 == bd->own,);

    const unsigned ie       = intr_is_enabled(rhport);

    intr_disable(rhport);

    bd->bdt_stall = 1;
    bd->own       = 1; /* This bit must be set last */

    if (ie) intr_enable(rhport);
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

  const unsigned ie       = intr_is_enabled(rhport);

  intr_disable(rhport);

  bd[odd].own = 0;

  // clear stall
  bd[odd].bdt_stall  = 0;

  // Reset data toggle
  bd[odd    ].data = 0;
  bd[odd ^ 1].data = 1;

  // We already cleared this in ISR, but just clear it here to be safe
  const unsigned endpt = ep_read(rhport, epn);
  if (endpt & _U1EP0_EPSTALL_MASK) {
    ep_clear(rhport, endpt, _U1EP0_EPSTALL_MASK);
  }

  if (ie) intr_enable(rhport);
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
void dcd_int_handler(uint8_t rhport)
{
  uint32_t is  = U1IR;
  uint32_t msk = U1IE;

  U1IR = is & ~msk;
  is &= msk;

  if (is & _U1IR_UERRIF_MASK) {
    uint32_t es = U1EIR;
    U1EIR = es;
    U1IR   = is; /* discard any pending events */
  }

  if (is & _U1IR_URSTIF_MASK) {
    U1IR = is; /* discard any pending events */
    process_bus_reset(rhport);
  }

  if (is & _U1IR_IDLEIF_MASK) {
    // Note Host usually has extra delay after bus reset (without SOF), which could falsely
    // detected as Sleep event. Though usbd has debouncing logic so we are good
    U1IR = _U1IR_IDLEIF_MASK;
    process_bus_sleep(rhport);
  }

  if (is & _U1IR_RESUMEIF_MASK) {
    U1IR = _U1IR_RESUMEIF_MASK;
    process_bus_resume(rhport);
  }

  if (is & _U1IR_SOFIF_MASK) {
    U1IR = _U1IR_SOFIF_MASK;
    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
  }

  if (is & _U1IR_STALLIF_MASK) {
    U1IR = _U1IR_STALLIF_MASK;
    process_stall(rhport);
  }

  if (is & _U1IR_TRNIF_MASK) {
    process_tokdne(rhport);
  }

  intr_clear(rhport);
}

#endif
