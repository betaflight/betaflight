/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Koji Kitayama
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

#if CFG_TUH_ENABLED && defined(TUP_USBIP_CHIPIDEA_FS)

#ifdef TUP_USBIP_CHIPIDEA_FS_KINETIS
  #include "fsl_device_registers.h"
  #define KHCI        USB0
#else
  #error "MCU is not supported"
#endif

#include "host/hcd.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

enum {
  TOK_PID_OUT   = 0x1u,
  TOK_PID_IN    = 0x9u,
  TOK_PID_SETUP = 0xDu,
  TOK_PID_DATA0 = 0x3u,
  TOK_PID_DATA1 = 0xbu,
  TOK_PID_ACK   = 0x2u,
  TOK_PID_STALL = 0xeu,
  TOK_PID_NAK   = 0xau,
  TOK_PID_BUSTO = 0x0u,
  TOK_PID_ERR   = 0xfu,
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
      uint32_t pipenum:16;
      uint32_t odd    : 1;
      uint32_t        : 0;
    };
  };
  uint8_t *buffer;
  uint16_t length;
  uint16_t remaining;
} endpoint_state_t;

typedef struct TU_ATTR_PACKED
{
  uint8_t  dev_addr;
  uint8_t  ep_addr;
  uint16_t max_packet_size;
  union {
    uint8_t flags;
    struct {
      uint8_t data : 1;
      uint8_t xfer : 2;
      uint8_t      : 0;
    };
  };
  uint8_t *buffer;
  uint16_t length;
  uint16_t remaining;
} pipe_state_t;


typedef struct
{
  union {
    /* [OUT,IN][EVEN,ODD] */
    buffer_descriptor_t bdt[2][2];
    uint16_t            bda[2*2];
  };
  endpoint_state_t endpoint[2];
  pipe_state_t pipe[CFG_TUH_ENDPOINT_MAX * 2];
  uint32_t     in_progress; /* Bitmap. Each bit indicates that a transfer of the corresponding pipe is in progress */
  uint32_t     pending;     /* Bitmap. Each bit indicates that a transfer of the corresponding pipe will be resume the next frame */
  bool         need_reset;  /* The device has not been reset after connection. */
} hcd_data_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
// BDT(Buffer Descriptor Table) must be 256-byte aligned
CFG_TUH_MEM_SECTION TU_ATTR_ALIGNED(512) static hcd_data_t _hcd;
//CFG_TUH_MEM_SECTION TU_ATTR_ALIGNED(4) static uint8_t _rx_buf[1024];

static int find_pipe(uint8_t dev_addr, uint8_t ep_addr)
{
  /* Find the target pipe */
  int num;
  for (num = 0; num < CFG_TUH_ENDPOINT_MAX * 2; ++num) {
    pipe_state_t *p = &_hcd.pipe[num];
    if ((p->dev_addr == dev_addr) && (p->ep_addr == ep_addr))
      return num;
  }
  return -1;
}

static int prepare_packets(int pipenum)
{
  pipe_state_t *pipe      = &_hcd.pipe[pipenum];
  unsigned const dir_tx   = tu_edpt_dir(pipe->ep_addr) ? 0 : 1;
  endpoint_state_t *ep    = &_hcd.endpoint[dir_tx];
  unsigned const odd      = ep->odd;
  buffer_descriptor_t *bd = _hcd.bdt[dir_tx];
  TU_ASSERT(0 == bd[odd].own, -1);

  // TU_LOG1("  %p dir %d odd %d data %d\r\n", &bd[odd], dir_tx, odd, pipe->data);

  ep->pipenum = pipenum;

  bd[odd    ].data      = pipe->data;
  bd[odd ^ 1].data      = pipe->data ^ 1;
  bd[odd ^ 1].own       = 0;
  /* reset values for a next transfer */

  int num_tokens = 0; /* The number of prepared packets */
  unsigned const mps = pipe->max_packet_size;
  unsigned const rem = pipe->remaining;
  if (rem > mps) {
    /* When total_bytes is greater than the max packet size,
     * it prepares to the next transfer to avoid NAK in advance. */
    bd[odd ^ 1].bc   = rem >= 2 * mps ? mps: rem - mps;
    bd[odd ^ 1].addr = pipe->buffer + mps;
    bd[odd ^ 1].own  = 1;
    if (dir_tx) ++num_tokens;
  }
  bd[odd].bc   = rem >= mps ? mps: rem;
  bd[odd].addr = pipe->buffer;
  __DSB();
  bd[odd].own  = 1; /* This bit must be set last */
  ++num_tokens;
  return num_tokens;
}

static int select_next_pipenum(int pipenum)
{
  unsigned wip  = _hcd.in_progress & ~_hcd.pending;
  if (!wip) return -1;
  unsigned msk  = TU_GENMASK(31, pipenum);
  int      next = __builtin_ctz(wip & msk);
  if (next) return next;
  msk  = TU_GENMASK(pipenum, 0);
  next = __builtin_ctz(wip & msk);
  return next;
}

/* When transfer is completed, return true. */
static bool continue_transfer(int pipenum, buffer_descriptor_t *bd)
{
  pipe_state_t *pipe = &_hcd.pipe[pipenum];
  unsigned const bc  = bd->bc;
  unsigned const rem = pipe->remaining - bc;

  pipe->remaining = rem;
  if (rem && bc == pipe->max_packet_size) {
    int const next_rem = rem - pipe->max_packet_size;
    if (next_rem > 0) {
      /* Prepare to the after next transfer */
      bd->addr += pipe->max_packet_size * 2;
      bd->bc    = next_rem > pipe->max_packet_size ? pipe->max_packet_size: next_rem;
      __DSB();
      bd->own   = 1; /* This bit must be set last */
      while (KHCI->CTL & USB_CTL_TXSUSPENDTOKENBUSY_MASK) ;
      KHCI->TOKEN = KHCI->TOKEN; /* Queue the same token as the last */
    } else if (TUSB_DIR_IN == tu_edpt_dir(pipe->ep_addr)) { /* IN */
      while (KHCI->CTL & USB_CTL_TXSUSPENDTOKENBUSY_MASK) ;
      KHCI->TOKEN = KHCI->TOKEN;
    }
    return true;
  }
  pipe->data = bd->data ^ 1;
  return false;
}

static bool resume_transfer(int pipenum)
{
  int num_tokens = prepare_packets(pipenum);
  TU_ASSERT(0 <= num_tokens);

  const unsigned ie = NVIC_GetEnableIRQ(USB0_IRQn);
  NVIC_DisableIRQ(USB0_IRQn);
  pipe_state_t *pipe = &_hcd.pipe[pipenum];

  unsigned flags = KHCI->ENDPOINT[0].ENDPT & USB_ENDPT_HOSTWOHUB_MASK;
  flags |= USB_ENDPT_EPRXEN_MASK | USB_ENDPT_EPTXEN_MASK;
  switch (pipe->xfer) {
  case TUSB_XFER_CONTROL:
    flags |= USB_ENDPT_EPHSHK_MASK;
    break;
  case TUSB_XFER_ISOCHRONOUS:
    flags |= USB_ENDPT_EPCTLDIS_MASK | USB_ENDPT_RETRYDIS_MASK;
    break;
  default:
    flags |= USB_ENDPT_EPHSHK_MASK | USB_ENDPT_EPCTLDIS_MASK | USB_ENDPT_RETRYDIS_MASK;
    break;
  }
  // TU_LOG1("  resume pipenum %d flags %x\r\n", pipenum, flags);

  KHCI->ENDPOINT[0].ENDPT = flags;
  KHCI->ADDR  = (KHCI->ADDR & USB_ADDR_LSEN_MASK) | pipe->dev_addr;

  unsigned const token = tu_edpt_number(pipe->ep_addr) |
    ((tu_edpt_dir(pipe->ep_addr) ? TOK_PID_IN: TOK_PID_OUT) << USB_TOKEN_TOKENPID_SHIFT);
  do {
    while (KHCI->CTL & USB_CTL_TXSUSPENDTOKENBUSY_MASK) ;
    KHCI->TOKEN = token;
  } while (--num_tokens);
  if (ie) NVIC_EnableIRQ(USB0_IRQn);
  return true;
}

static void suspend_transfer(int pipenum, buffer_descriptor_t *bd)
{
  pipe_state_t *pipe = &_hcd.pipe[pipenum];
  pipe->buffer  = bd->addr;
  pipe->data    = bd->data ^ 1;
  if ((TUSB_XFER_INTERRUPT == pipe->xfer) ||
      (TUSB_XFER_BULK == pipe->xfer)) {
    _hcd.pending |= TU_BIT(pipenum);
    KHCI->INTEN |= USB_ISTAT_SOFTOK_MASK;
  }
}

static void process_tokdne(uint8_t rhport)
{
  (void)rhport;
  const unsigned s = KHCI->STAT;
  KHCI->ISTAT = USB_ISTAT_TOKDNE_MASK; /* fetch the next token if received */
  uint8_t const dir_in = (s & USB_STAT_TX_MASK) ? TUSB_DIR_OUT: TUSB_DIR_IN;
  unsigned const odd   = (s & USB_STAT_ODD_MASK) ? 1 : 0;

  buffer_descriptor_t *bd = (buffer_descriptor_t *)&_hcd.bda[s];
  endpoint_state_t    *ep = &_hcd.endpoint[s >> 3];

  /* fetch status before discarded by the next steps */
  const unsigned pid = bd->tok_pid;

  /* reset values for a next transfer */
  bd->bdt_stall = 0;
  bd->dts       = 1;
  bd->ninc      = 0;
  bd->keep      = 0;
  /* Update the odd variable to prepare for the next transfer */
  ep->odd       = odd ^ 1;

  int pipenum = ep->pipenum;
  int next_pipenum;
  // TU_LOG1("TOKDNE %x PID %x pipe %d\r\n", s, pid, pipenum);

  xfer_result_t result;
  switch (pid) {
    default:
      if (continue_transfer(pipenum, bd))
        return;
      result = XFER_RESULT_SUCCESS;
      break;
    case TOK_PID_NAK:
      suspend_transfer(pipenum, bd);
      next_pipenum = select_next_pipenum(pipenum);
      if (0 <= next_pipenum)
        resume_transfer(next_pipenum);
      return;
    case TOK_PID_STALL:
      result = XFER_RESULT_STALLED;
      break;
    case TOK_PID_ERR: /* mismatch toggle bit */
    case TOK_PID_BUSTO:
      result = XFER_RESULT_FAILED;
      break;
  }
  _hcd.in_progress  &= ~TU_BIT(pipenum);
  pipe_state_t *pipe = &_hcd.pipe[ep->pipenum];
  hcd_event_xfer_complete(pipe->dev_addr,
                          tu_edpt_addr(KHCI->TOKEN & USB_TOKEN_TOKENENDPT_MASK, dir_in),
                          pipe->length - pipe->remaining,
                          result, true);
  next_pipenum = select_next_pipenum(pipenum);
  if (0 <= next_pipenum)
    resume_transfer(next_pipenum);
}

static void process_attach(uint8_t rhport)
{
  unsigned ctl = KHCI->CTL;
  if (!(ctl & USB_CTL_JSTATE_MASK)) {
    /* The attached device is a low speed device. */
    KHCI->ADDR = USB_ADDR_LSEN_MASK;
    KHCI->ENDPOINT[0].ENDPT = USB_ENDPT_HOSTWOHUB_MASK;
  }
  hcd_event_device_attach(rhport, true);
}

static void process_bus_reset(uint8_t rhport)
{
  KHCI->ISTAT    = USB_ISTAT_TOKDNE_MASK;
  KHCI->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
  KHCI->CTL     &= ~USB_CTL_USBENSOFEN_MASK;
  KHCI->ADDR     = 0;
  KHCI->ENDPOINT[0].ENDPT = 0;

  hcd_event_device_remove(rhport, true);

  _hcd.in_progress = 0;
  _hcd.pending     = 0;
  buffer_descriptor_t *bd = &_hcd.bdt[0][0];
  for (unsigned i = 0; i < 2; ++i, ++bd) {
    bd->head = 0;
  }
}

/*------------------------------------------------------------------*/
/* Host API
 *------------------------------------------------------------------*/
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;
  KHCI->USBTRC0 |= USB_USBTRC0_USBRESET_MASK;
  while (KHCI->USBTRC0 & USB_USBTRC0_USBRESET_MASK);

  tu_memclr(&_hcd, sizeof(_hcd));
  KHCI->USBTRC0 |= TU_BIT(6); /* software must set this bit to 1 */
  KHCI->BDTPAGE1 = (uint8_t)((uintptr_t)_hcd.bdt >>  8);
  KHCI->BDTPAGE2 = (uint8_t)((uintptr_t)_hcd.bdt >> 16);
  KHCI->BDTPAGE3 = (uint8_t)((uintptr_t)_hcd.bdt >> 24);

  KHCI->USBCTRL &= ~USB_USBCTRL_SUSP_MASK;
  KHCI->CTL     |= USB_CTL_ODDRST_MASK;
  for (unsigned i = 0; i < 16; ++i) {
    KHCI->ENDPOINT[i].ENDPT = 0;
  }
  KHCI->CTL &= ~USB_CTL_ODDRST_MASK;

  KHCI->SOFTHLD = 74; /* for 64-byte packets */
  // KHCI->SOFTHLD = 144; /* for low speed 8-byte packets */
  KHCI->CTL     = USB_CTL_HOSTMODEEN_MASK | USB_CTL_SE0_MASK;
  KHCI->USBCTRL = USB_USBCTRL_PDE_MASK;

  NVIC_ClearPendingIRQ(USB0_IRQn);
  KHCI->INTEN = USB_INTEN_ATTACHEN_MASK | USB_INTEN_TOKDNEEN_MASK |
    USB_INTEN_USBRSTEN_MASK | USB_INTEN_ERROREN_MASK | USB_INTEN_STALLEN_MASK;
  KHCI->ERREN = 0xff;

  return true;
}

void hcd_int_enable(uint8_t rhport)
{
  (void)rhport;
  NVIC_EnableIRQ(USB0_IRQn);
}

void hcd_int_disable(uint8_t rhport)
{
  (void)rhport;
  NVIC_DisableIRQ(USB0_IRQn);
}

uint32_t hcd_frame_number(uint8_t rhport)
{
  (void)rhport;
  /* The device must be reset at least once after connection
   * in order to start the frame counter. */
  if (_hcd.need_reset) hcd_port_reset(rhport);
  uint32_t frmnum = KHCI->FRMNUML;
  frmnum |= KHCI->FRMNUMH << 8u;
   return frmnum;
}

/*--------------------------------------------------------------------+
 * Port API
 *--------------------------------------------------------------------+ */
bool hcd_port_connect_status(uint8_t rhport)
{
  (void)rhport;
  if (KHCI->ISTAT & USB_ISTAT_ATTACH_MASK)
    return true;
  return false;
}

void hcd_port_reset(uint8_t rhport)
{
  (void)rhport;
  KHCI->CTL &= ~USB_CTL_USBENSOFEN_MASK;
  KHCI->CTL |= USB_CTL_RESET_MASK;
  unsigned cnt = SystemCoreClock / 100;
  while (cnt--) __NOP();
  KHCI->CTL &= ~USB_CTL_RESET_MASK;
  KHCI->CTL |= USB_CTL_USBENSOFEN_MASK;
  _hcd.need_reset = false;
}

void hcd_port_reset_end(uint8_t rhport) {
  (void) rhport;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
  (void)rhport;
  tusb_speed_t speed = TUSB_SPEED_FULL;
  const unsigned ie = NVIC_GetEnableIRQ(USB0_IRQn);
  NVIC_DisableIRQ(USB0_IRQn);
  if (KHCI->ADDR & USB_ADDR_LSEN_MASK)
    speed = TUSB_SPEED_LOW;
  if (ie) NVIC_EnableIRQ(USB0_IRQn);
  return speed;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
  (void)rhport;
  const unsigned ie = NVIC_GetEnableIRQ(USB0_IRQn);
  NVIC_DisableIRQ(USB0_IRQn);
  pipe_state_t *p   = &_hcd.pipe[0];
  pipe_state_t *end = &_hcd.pipe[CFG_TUH_ENDPOINT_MAX * 2];
  for (;p != end; ++p) {
    if (p->dev_addr == dev_addr)
      tu_memclr(p, sizeof(*p));
  }
  if (ie) NVIC_EnableIRQ(USB0_IRQn);
}

//--------------------------------------------------------------------+
// Endpoints API
//--------------------------------------------------------------------+
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
  (void)rhport;
  // TU_LOG1("SETUP %u\r\n", dev_addr);
  TU_ASSERT(0 == (_hcd.in_progress & TU_BIT(0)));

  int pipenum = find_pipe(dev_addr, 0);
  if (pipenum < 0) return false;

  pipe_state_t *pipe = &_hcd.pipe[pipenum];
  pipe[0].data       = 0;
  pipe[0].buffer     = (uint8_t*)(uintptr_t)setup_packet;
  pipe[0].length     = 8;
  pipe[0].remaining  = 8;
  pipe[1].data       = 1;

  if (1 != prepare_packets(pipenum))
    return false;

  _hcd.in_progress |= TU_BIT(pipenum);

  unsigned hostwohub = KHCI->ENDPOINT[0].ENDPT & USB_ENDPT_HOSTWOHUB_MASK;
  KHCI->ENDPOINT[0].ENDPT = hostwohub |
    USB_ENDPT_EPHSHK_MASK | USB_ENDPT_EPRXEN_MASK | USB_ENDPT_EPTXEN_MASK;
  KHCI->ADDR  = (KHCI->ADDR & USB_ADDR_LSEN_MASK) | dev_addr;
  while (KHCI->CTL & USB_CTL_TXSUSPENDTOKENBUSY_MASK) ;
  KHCI->TOKEN = (TOK_PID_SETUP << USB_TOKEN_TOKENPID_SHIFT);
  return true;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
  (void)rhport;
  uint8_t const ep_addr = ep_desc->bEndpointAddress;
  // TU_LOG1("O %u %x\r\n", dev_addr, ep_addr);
  /* Find a free pipe */
  pipe_state_t *p = &_hcd.pipe[0];
  pipe_state_t *end = &_hcd.pipe[CFG_TUH_ENDPOINT_MAX * 2];
  if (dev_addr || ep_addr) {
    p += 2;
    for (; p < end && (p->dev_addr || p->ep_addr); ++p) ;
    if (p == end) return false;
  }
  p->dev_addr        = dev_addr;
  p->ep_addr         = ep_addr;
  p->max_packet_size = ep_desc->wMaxPacketSize;
  p->xfer            = ep_desc->bmAttributes.xfer;
  p->data            = 0;
  if (!ep_addr) {
    /* Open one more pipe for Control IN transfer */
    TU_ASSERT(TUSB_XFER_CONTROL == p->xfer);
    pipe_state_t *q = p + 1;
    TU_ASSERT(!q->dev_addr && !q->ep_addr);
    q->dev_addr        = dev_addr;
    q->ep_addr         = tu_edpt_addr(0, TUSB_DIR_IN);
    q->max_packet_size = ep_desc->wMaxPacketSize;
    q->xfer            = ep_desc->bmAttributes.xfer;
    q->data            = 1;
  }
  return true;
}

/* The address of buffer must be aligned to 4 byte boundary. And it must be at least 4 bytes long.
 * DMA writes data in 4 byte unit */
bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t buflen)
{
  (void)rhport;
  // TU_LOG1("X %u %x %x %d\r\n", dev_addr, ep_addr, (uintptr_t)buffer, buflen);

  int pipenum = find_pipe(dev_addr, ep_addr);
  TU_ASSERT(0 <= pipenum);

  TU_ASSERT(0 == (_hcd.in_progress & TU_BIT(pipenum)));
  unsigned const ie  = NVIC_GetEnableIRQ(USB0_IRQn);
  NVIC_DisableIRQ(USB0_IRQn);
  pipe_state_t *pipe = &_hcd.pipe[pipenum];
  pipe->buffer       = buffer;
  pipe->length       = buflen;
  pipe->remaining    = buflen;
  _hcd.in_progress  |= TU_BIT(pipenum);
  _hcd.pending      |= TU_BIT(pipenum); /* Send at the next Frame */
  KHCI->INTEN |= USB_ISTAT_SOFTOK_MASK;
  if (ie) NVIC_EnableIRQ(USB0_IRQn);
  return true;
}

bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  // TODO not implemented yet
  return false;
}

bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  if (!tu_edpt_number(ep_addr)) return true;
  int num = find_pipe(dev_addr, ep_addr);
  if (num < 0) return false;
  pipe_state_t *p = &_hcd.pipe[num];
  p->data = 0; /* Reset data toggle */
  return true;
}

/*--------------------------------------------------------------------+
 * ISR
 *--------------------------------------------------------------------+*/
void hcd_int_handler(uint8_t rhport, bool in_isr)
{
  (void) in_isr;
  uint32_t is  = KHCI->ISTAT;
  uint32_t msk = KHCI->INTEN;

  // TU_LOG1("S %lx\r\n", is);

  /* clear disabled interrupts */
  KHCI->ISTAT = (is & ~msk & ~USB_ISTAT_TOKDNE_MASK) | USB_ISTAT_SOFTOK_MASK;
  is &= msk;

  if (is & USB_ISTAT_ERROR_MASK) {
    unsigned err = KHCI->ERRSTAT;
    if (err) {
      TU_LOG1(" ERR %x\r\n", err);
      KHCI->ERRSTAT = err;
    } else {
      KHCI->INTEN &= ~USB_ISTAT_ERROR_MASK;
    }
  }

  if (is & USB_ISTAT_USBRST_MASK) {
    KHCI->INTEN = (msk & ~USB_INTEN_USBRSTEN_MASK) | USB_INTEN_ATTACHEN_MASK;
    process_bus_reset(rhport);
    return;
  }
  if (is & USB_ISTAT_ATTACH_MASK) {
    KHCI->INTEN = (msk & ~USB_INTEN_ATTACHEN_MASK) | USB_INTEN_USBRSTEN_MASK;
    _hcd.need_reset = true;
    process_attach(rhport);
    return;
  }
  if (is & USB_ISTAT_STALL_MASK) {
    KHCI->ISTAT = USB_ISTAT_STALL_MASK;
  }
  if (is & USB_ISTAT_SOFTOK_MASK) {
    msk &= ~USB_ISTAT_SOFTOK_MASK;
    KHCI->INTEN = msk;
    if (_hcd.pending) {
      int pipenum = __builtin_ctz(_hcd.pending);
      _hcd.pending = 0;
      if (!(is & USB_ISTAT_TOKDNE_MASK))
        resume_transfer(pipenum);
    }
  }
  if (is & USB_ISTAT_TOKDNE_MASK) {
    process_tokdne(rhport);
  }
}

#endif
