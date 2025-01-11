/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Koji Kitayama
 * Portions copyrighted (c) 2021 Roland Winistoerfer
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

#if CFG_TUH_ENABLED && defined(TUP_USBIP_RUSB2)

#include "host/hcd.h"
#include "rusb2_type.h"

#if TU_CHECK_MCU(OPT_MCU_RX63X, OPT_MCU_RX65X, OPT_MCU_RX72N)
  #include "rusb2_rx.h"
#elif TU_CHECK_MCU(OPT_MCU_RAXXX)
  #include "rusb2_ra.h"
#else
  #error "Unsupported MCU"
#endif

#define TU_RUSB2_HCD_DBG   2

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
enum {
  PIPE_COUNT = 10,
};

TU_ATTR_PACKED_BEGIN
TU_ATTR_BIT_FIELD_ORDER_BEGIN

typedef union TU_ATTR_PACKED {
  struct {
    volatile uint16_t u8: 8;
    volatile uint16_t   : 0;
  };
  volatile uint16_t u16;
} hw_fifo_t;

typedef struct TU_ATTR_PACKED {
  void      *buf;      /* the start address of a transfer data buffer */
  uint16_t  length;    /* the number of bytes in the buffer */
  uint16_t  remaining; /* the number of bytes remaining in the buffer */
  struct {
    uint32_t ep  : 8;  /* an assigned endpoint address */
    uint32_t dev : 8;  /* an assigned device address */
    uint32_t ff  : 1;  /* `buf` is TU_FUFO or POD */
    uint32_t     : 0;
  };
} pipe_state_t;

TU_ATTR_PACKED_END  // End of definition of packed structs (used by the CCRX toolchain)
TU_ATTR_BIT_FIELD_ORDER_END

typedef struct
{
  bool         need_reset; /* The device has not been reset after connection. */
  pipe_state_t pipe[PIPE_COUNT];
  uint8_t ep[4][2][15];   /* a lookup table for a pipe index from an endpoint address */
  uint8_t      ctl_mps[5]; /* EP0 max packet size for each device */
} hcd_data_t;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
static hcd_data_t _hcd;

// TODO merged with DCD
// Transfer conditions specifiable for each pipe for most MCUs
// - Pipe 0: Control transfer with 64-byte single buffer
// - Pipes 1 and 2: Bulk or ISO
// - Pipes 3 to 5: Bulk
// - Pipes 6 to 9: Interrupt
//
// Note: for small mcu such as
// - RA2A1: only pipe 4-7 are available, and no support for ISO
static unsigned find_pipe(unsigned xfer_type) {
  const uint8_t pipe_idx_arr[4][2] = {
      { 0, 0 }, // Control
      { 1, 2 }, // Isochronous
      { 1, 5 }, // Bulk
      { 6, 9 }, // Interrupt
  };

  // find backward since only pipe 1, 2 support ISO
  const uint8_t idx_first = pipe_idx_arr[xfer_type][0];
  const uint8_t idx_last  = pipe_idx_arr[xfer_type][1];

  for (int i = idx_last; i >= idx_first; i--) {
    if (0 == _hcd.pipe[i].ep) return i;
  }

  return 0;
}

static volatile uint16_t* get_pipectr(rusb2_reg_t *rusb, unsigned num)
{
  if (num) {
    return (volatile uint16_t*)&(rusb->PIPE_CTR[num - 1]);
  } else {
    return (volatile uint16_t*)&(rusb->DCPCTR);
  }
}

static volatile reg_pipetre_t* get_pipetre(rusb2_reg_t *rusb, unsigned num)
{
  volatile reg_pipetre_t* tre = NULL;
  if ((1 <= num) && (num <= 5)) {
    tre = (volatile reg_pipetre_t*)&(rusb->PIPE_TR[num - 1].E);
  }
  return tre;
}

static volatile uint16_t* addr_to_pipectr(uint8_t rhport, uint8_t dev_addr, unsigned ep_addr)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  const unsigned epn = tu_edpt_number(ep_addr);

  if (epn) {
    const unsigned dir_in = tu_edpt_dir(ep_addr);
    const unsigned num = _hcd.ep[dev_addr][dir_in][epn - 1];
    return get_pipectr(rusb, num);
  } else {
    return get_pipectr(rusb, 0);
  }
}

static uint16_t edpt0_max_packet_size(rusb2_reg_t* rusb)
{
  return rusb->DCPMAXP_b.MXPS;
}

static uint16_t edpt_max_packet_size(rusb2_reg_t *rusb, unsigned num)
{
  rusb->PIPESEL = num;
  return rusb->PIPEMAXP_b.MXPS;
}

static inline void pipe_wait_for_ready(rusb2_reg_t* rusb, unsigned num)
{
  while (rusb->D0FIFOSEL_b.CURPIPE != num) ;
  while (!rusb->D0FIFOCTR_b.FRDY) {}
}

static void pipe_write_packet(void *buf, volatile void *fifo, unsigned len)
{
  // NOTE: unlike DCD, Highspeed 32-bit FIFO does not need to adjust the fifo address
  volatile hw_fifo_t *reg = (volatile hw_fifo_t*)fifo;
  uintptr_t addr = (uintptr_t)buf;
  while (len >= 2) {
    reg->u16 = *(const uint16_t *)addr;
    addr += 2;
    len  -= 2;
  }
  if (len) {
    reg->u8 = *(const uint8_t *)addr;
    ++addr;
  }
}

static void pipe_read_packet(void *buf, volatile void *fifo, unsigned len)
{
  uint8_t *p   = (uint8_t*)buf;
  volatile uint8_t *reg = (volatile uint8_t*)fifo;  /* byte access is always at base register address */
  while (len--) *p++ = *reg;
}

static bool pipe0_xfer_in(rusb2_reg_t* rusb)
{
  pipe_state_t *pipe = &_hcd.pipe[0];
  const unsigned rem = pipe->remaining;

  const unsigned mps = edpt0_max_packet_size(rusb);
  const unsigned vld = rusb->CFIFOCTR_b.DTLN;
  const unsigned len = TU_MIN(TU_MIN(rem, mps), vld);
  void          *buf = pipe->buf;
  if (len) {
    rusb->DCPCTR = RUSB2_PIPE_CTR_PID_NAK;
    pipe_read_packet(buf, (volatile void*)&rusb->CFIFO, len);
    pipe->buf = (uint8_t*)buf + len;
  }
  if (len < mps) {
    rusb->CFIFOCTR = RUSB2_CFIFOCTR_BCLR_Msk;
  }
  pipe->remaining = rem - len;
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return true;
  }
  rusb->DCPCTR = RUSB2_PIPE_CTR_PID_BUF;
  return false;
}

static bool pipe0_xfer_out(rusb2_reg_t* rusb)
{
  pipe_state_t *pipe = &_hcd.pipe[0];
  const unsigned rem = pipe->remaining;
  if (!rem) {
    pipe->buf = NULL;
    return true;
  }
  const unsigned mps = edpt0_max_packet_size(rusb);
  const unsigned len = TU_MIN(mps, rem);
  void          *buf = pipe->buf;
  if (len) {
    pipe_write_packet(buf, (volatile void*)&rusb->CFIFO, len);
    pipe->buf = (uint8_t*)buf + len;
  }
  if (len < mps) {
    rusb->CFIFOCTR = RUSB2_CFIFOCTR_BVAL_Msk;
  }
  pipe->remaining = rem - len;
  return false;
}

static bool pipe_xfer_in(rusb2_reg_t* rusb, unsigned num)
{
  pipe_state_t  *pipe = &_hcd.pipe[num];
  const unsigned rem  = pipe->remaining;

  rusb->D0FIFOSEL = num | RUSB2_FIFOSEL_MBW_8BIT;
  const unsigned mps  = edpt_max_packet_size(rusb, num);
  pipe_wait_for_ready(rusb, num);
  const unsigned vld  = rusb->D0FIFOCTR_b.DTLN;
  const unsigned len  = TU_MIN(TU_MIN(rem, mps), vld);
  void          *buf  = pipe->buf;
  if (len) {
    pipe_read_packet(buf, (volatile void*)&rusb->D0FIFO, len);
    pipe->buf = (uint8_t*)buf + len;
  }
  if (len < mps) {
    rusb->D0FIFOCTR = RUSB2_D0FIFOCTR_BCLR_Msk;
  }
  rusb->D0FIFOSEL = 0;
  while (rusb->D0FIFOSEL_b.CURPIPE) ; /* if CURPIPE bits changes, check written value */
  pipe->remaining = rem - len;
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return NULL != buf;
  }
  return false;
}

static bool pipe_xfer_out(rusb2_reg_t* rusb, unsigned num)
{
  pipe_state_t  *pipe = &_hcd.pipe[num];
  const unsigned rem  = pipe->remaining;

  if (!rem) {
    pipe->buf = NULL;
    return true;
  }

  rusb->D0FIFOSEL = num | RUSB2_FIFOSEL_MBW_16BIT | (TU_BYTE_ORDER == TU_BIG_ENDIAN ? RUSB2_FIFOSEL_BIGEND : 0);
  const unsigned mps  = edpt_max_packet_size(rusb, num);
  pipe_wait_for_ready(rusb, num);
  const unsigned len  = TU_MIN(rem, mps);
  void          *buf  = pipe->buf;
  if (len) {
    pipe_write_packet(buf, (volatile void*)&rusb->D0FIFO, len);
    pipe->buf = (uint8_t*)buf + len;
  }
  if (len < mps) {
    rusb->D0FIFOCTR = RUSB2_D0FIFOCTR_BVAL_Msk;
  }
  rusb->D0FIFOSEL = 0;
  while (rusb->D0FIFOSEL_b.CURPIPE) ; /* if CURPIPE bits changes, check written value */
  pipe->remaining = rem - len;
  return false;
}

static bool process_pipe0_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, void* buffer, uint16_t buflen)
{
  (void)dev_addr;

  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  const unsigned dir_in = tu_edpt_dir(ep_addr);

  /* configure fifo direction and access unit settings */
  if (dir_in) { /* IN, a byte */
    rusb->CFIFOSEL = RUSB2_FIFOSEL_MBW_8BIT;
    while (rusb->CFIFOSEL & RUSB2_CFIFOSEL_ISEL_WRITE) ;
  } else { /* OUT, 2 bytes */
    rusb->CFIFOSEL = RUSB2_CFIFOSEL_ISEL_WRITE | RUSB2_FIFOSEL_MBW_16BIT |
                         (TU_BYTE_ORDER == TU_BIG_ENDIAN ? RUSB2_FIFOSEL_BIGEND : 0);
    while (!(rusb->CFIFOSEL & RUSB2_CFIFOSEL_ISEL_WRITE)) ;
  }

  pipe_state_t *pipe = &_hcd.pipe[0];
  pipe->ep        = ep_addr;
  pipe->length    = buflen;
  pipe->remaining = buflen;
  if (buflen) {
    pipe->buf     = buffer;
    if (!dir_in) { /* OUT */
      TU_ASSERT(rusb->DCPCTR_b.BSTS && (rusb->USBREQ & 0x80));
      pipe0_xfer_out(rusb);
    }
  } else { /* ZLP */
    pipe->buf        = NULL;
    if (!dir_in) { /* OUT */
      rusb->CFIFOCTR = RUSB2_CFIFOCTR_BVAL_Msk;
    }
    if (dir_in == rusb->DCPCFG_b.DIR) {
      TU_ASSERT(RUSB2_PIPE_CTR_PID_NAK == rusb->DCPCTR_b.PID);
      rusb->DCPCTR_b.SQSET = 1;
      rusb->DCPCFG_b.DIR = dir_in ^ 1;
    }
  }
  rusb->DCPCTR = RUSB2_PIPE_CTR_PID_BUF;
  return true;
}

static bool process_pipe_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, void *buffer, uint16_t buflen)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);

  const unsigned epn    = tu_edpt_number(ep_addr);
  const unsigned dir_in = tu_edpt_dir(ep_addr);
  const unsigned num    = _hcd.ep[dev_addr - 1][dir_in][epn - 1];

  TU_ASSERT(num);

  pipe_state_t *pipe  = &_hcd.pipe[num];
  pipe->buf       = buffer;
  pipe->length    = buflen;
  pipe->remaining = buflen;
  if (!dir_in) { /* OUT */
    if (buflen) {
      pipe_xfer_out(rusb, num);
    } else { /* ZLP */
      rusb->D0FIFOSEL = num;
      pipe_wait_for_ready(rusb, num);
      rusb->D0FIFOCTR = RUSB2_D0FIFOCTR_BVAL_Msk;
      rusb->D0FIFOSEL = 0;
      while (rusb->D0FIFOSEL_b.CURPIPE) {} /* if CURPIPE bits changes, check written value */
    }
  } else {
    volatile uint16_t     *ctr = get_pipectr(rusb, num);
    volatile reg_pipetre_t *pt = get_pipetre(rusb, num);
    if (pt) {
      const unsigned     mps = edpt_max_packet_size(rusb, num);
      if (*ctr & 0x3) *ctr = RUSB2_PIPE_CTR_PID_NAK;
      pt->TRE   = TU_BIT(8);
      pt->TRN   = (buflen + mps - 1) / mps;
      pt->TRENB = 1;
    }
    *ctr = RUSB2_PIPE_CTR_PID_BUF;
  }
  return true;
}

static bool process_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, void* buffer, uint16_t buflen)
{
  const unsigned epn = tu_edpt_number(ep_addr);
  if (0 == epn) {
    return process_pipe0_xfer(rhport, dev_addr, ep_addr, buffer, buflen);
  } else {
    return process_pipe_xfer(rhport, dev_addr, ep_addr, buffer, buflen);
  }
}

static void process_pipe0_bemp(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  bool completed = pipe0_xfer_out(rusb);
  if (completed) {
    pipe_state_t *pipe = &_hcd.pipe[0];
    hcd_event_xfer_complete(pipe->dev,
                            tu_edpt_addr(0, TUSB_DIR_OUT),
                            pipe->length - pipe->remaining,
                            XFER_RESULT_SUCCESS, true);
  }
}

static void process_pipe_nrdy(uint8_t rhport, unsigned num)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  xfer_result_t result;
  uint16_t volatile *ctr = get_pipectr(rusb, num);
  TU_LOG(TU_RUSB2_HCD_DBG, "NRDY %d %x\r\n", num, *ctr);
  switch (*ctr & RUSB2_PIPE_CTR_PID_Msk) {
    default: return;
    case RUSB2_PIPE_CTR_PID_STALL: result = XFER_RESULT_STALLED; break;
    case RUSB2_PIPE_CTR_PID_STALL2: result = XFER_RESULT_STALLED; break;
    case RUSB2_PIPE_CTR_PID_NAK:   result = XFER_RESULT_FAILED;  break;
  }
  pipe_state_t *pipe = &_hcd.pipe[num];
  hcd_event_xfer_complete(pipe->dev, pipe->ep,
                          pipe->length - pipe->remaining,
                          result, true);
}

static void process_pipe_brdy(uint8_t rhport, unsigned num)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  pipe_state_t  *pipe   = &_hcd.pipe[num];
  const unsigned dir_in = tu_edpt_dir(pipe->ep);
  bool completed;

  if (dir_in) { /* IN */
    if (num) {
      completed = pipe_xfer_in(rusb, num);
    } else {
      completed = pipe0_xfer_in(rusb);
    }
  } else {
    completed = pipe_xfer_out(rusb, num);
  }
  if (completed) {
    hcd_event_xfer_complete(pipe->dev, pipe->ep,
                            pipe->length - pipe->remaining,
                            XFER_RESULT_SUCCESS, true);
    TU_LOG(TU_RUSB2_HCD_DBG, "C %d %d\r\n", num, pipe->length - pipe->remaining);
  }
}

/*------------------------------------------------------------------*/
/* Host API
 *------------------------------------------------------------------*/

#if 0 // previously present in the rx driver before generalization
static uint32_t disable_interrupt(void)
{
  uint32_t pswi;
#if defined(__CCRX__)
  pswi = get_psw() & 0x010000;
  clrpsw_i();
#else
  pswi = __builtin_rx_mvfc(0) & 0x010000;
  __builtin_rx_clrpsw('I');
#endif
  return pswi;
}

static void enable_interrupt(uint32_t pswi)
{
#if defined(__CCRX__)
  set_psw(get_psw() | pswi);
#else
  __builtin_rx_mvtc(0, __builtin_rx_mvfc(0) | pswi);
#endif
}
#endif

bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  rusb2_module_start(rhport, true);

#ifdef RUSB2_SUPPORT_HIGHSPEED
  if (rusb2_is_highspeed_rhport(rhport) ) {
    rusb->SYSCFG_b.HSE = 1;
    rusb->PHYSET_b.HSEB = 0;
    rusb->PHYSET_b.DIRPD = 0;
    R_BSP_SoftwareDelay((uint32_t) 1, BSP_DELAY_UNITS_MILLISECONDS);
    rusb->PHYSET_b.PLLRESET = 0;
    rusb->LPSTS_b.SUSPENDM = 1;
    while ( !rusb->PLLSTA_b.PLLLOCK );
    rusb->SYSCFG_b.DRPD = 1;
    rusb->SYSCFG_b.DCFM = 1;
    rusb->SYSCFG_b.DPRPU = 0;
    rusb->SYSCFG_b.CNEN = 1;
    rusb->BUSWAIT |= 0x0F00U;
    rusb->SOFCFG_b.INTL = 1;
    rusb->DVSTCTR0_b.VBUSEN = 1;
    rusb->CFIFOSEL_b.MBW = 1;
    rusb->D0FIFOSEL_b.MBW = 1;
    rusb->D1FIFOSEL_b.MBW = 1;
    rusb->INTSTS0 = 0;
    for ( volatile int i = 0; i < 30000; ++i );
    rusb->SYSCFG_b.USBE = 1;
  } else
#endif
  {
    rusb->SYSCFG_b.SCKE = 1;
    while ( !rusb->SYSCFG_b.SCKE ) {}
    rusb->SYSCFG_b.DCFM = 1;         // Host function
    rusb->SYSCFG_b.DPRPU = 0;        // Disable D+ pull up
    rusb->SYSCFG_b.DRPD = 1;         // Enable D+/D- pull down

    rusb->DVSTCTR0_b.VBUSEN = 1;
    for ( volatile int i = 0; i < 30000; ++i ) {} // FIXME do we need to wait here? how long ?
    //R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    rusb->SYSCFG_b.USBE = 1;

    // MCU specific PHY init
    rusb2_phy_init();

    rusb->PHYSLEW = 0x5;
    rusb->DPUSR0R_FS_b.FIXPHY0 = 0u; /* Transceiver Output fixed */
  }

  /* Setup default control pipe */
  rusb->DCPCFG  = RUSB2_PIPECFG_SHTNAK_Msk;
  rusb->DCPMAXP = 64;
  rusb->INTENB0 = RUSB2_INTSTS0_BRDY_Msk | RUSB2_INTSTS0_NRDY_Msk | RUSB2_INTSTS0_BEMP_Msk;
  rusb->INTENB1 = RUSB2_INTSTS1_SACK_Msk | RUSB2_INTSTS1_SIGN_Msk | RUSB2_INTSTS1_ATTCH_Msk | RUSB2_INTSTS1_DTCH_Msk;
  rusb->BEMPENB = 1;
  rusb->NRDYENB = 1;
  rusb->BRDYENB = 1;

  return true;
}

void hcd_int_enable(uint8_t rhport) {
  rusb2_int_enable(rhport);
}

void hcd_int_disable(uint8_t rhport) {
  rusb2_int_disable(rhport);
}

uint32_t hcd_frame_number(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);

  /* The device must be reset at least once after connection
   * in order to start the frame counter. */
  if (_hcd.need_reset) hcd_port_reset(rhport);
  return rusb->FRMNUM_b.FRNM;
}

/*--------------------------------------------------------------------+
 * Port API
 *--------------------------------------------------------------------+*/
bool hcd_port_connect_status(uint8_t rhport) {
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  return rusb->INTSTS1_b.ATTCH ? true : false;
}

void hcd_port_reset(uint8_t rhport) {
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  rusb->DCPCTR = RUSB2_PIPE_CTR_PID_NAK;
  while (rusb->DCPCTR_b.PBUSY) {}

  hcd_int_disable(rhport);
  rusb->DVSTCTR0_b.UACT = 0;
  if (rusb->DCPCTR_b.SUREQ) {
    rusb->DCPCTR_b.SUREQCLR = 1;
  }
  hcd_int_enable(rhport);

  /* Reset should be asserted 10-20ms. */
  rusb->DVSTCTR0_b.USBRST = 1;
  for (volatile int i = 0; i < 2400000; ++i) {}
  rusb->DVSTCTR0_b.USBRST = 0;

  rusb->DVSTCTR0_b.UACT = 1;
  _hcd.need_reset = false;
}

void hcd_port_reset_end(uint8_t rhport) {
  (void) rhport;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport) {
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  switch (rusb->DVSTCTR0_b.RHST) {
    case RUSB2_DVSTCTR0_RHST_HS: return TUSB_SPEED_HIGH;
    case RUSB2_DVSTCTR0_RHST_FS: return TUSB_SPEED_FULL;
    case RUSB2_DVSTCTR0_RHST_LS: return TUSB_SPEED_LOW;
    default: return TUSB_SPEED_INVALID;
  }
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr) {
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  uint16_t volatile *ctr;

  TU_ASSERT(dev_addr < 6,); /* USBa can only handle addresses from 0 to 5. */
  if (!dev_addr) return;

  _hcd.ctl_mps[dev_addr] = 0;
  uint8_t *ep = &_hcd.ep[dev_addr - 1][0][0];

  for (int i = 0; i < 2 * 15; ++i, ++ep) {
    unsigned num = *ep;
    if (!num || (dev_addr != _hcd.pipe[num].dev)) continue;

    ctr = (uint16_t volatile*)&rusb->PIPE_CTR[num - 1];
    *ctr = 0;
    rusb->NRDYENB &= ~TU_BIT(num);
    rusb->BRDYENB &= ~TU_BIT(num);
    rusb->PIPESEL = num;
    rusb->PIPECFG = 0;
    rusb->PIPEMAXP = 0;

    _hcd.pipe[num].ep  = 0;
    _hcd.pipe[num].dev = 0;
    *ep                = 0;
  }
}

/*--------------------------------------------------------------------+
 * Endpoints API
 *--------------------------------------------------------------------+*/
bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
  TU_ASSERT(dev_addr < 6); /* USBa can only handle addresses from 0 to 5. */

  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  TU_LOG(TU_RUSB2_HCD_DBG, "S %d %x\r\n", dev_addr, rusb->DCPCTR);

  TU_ASSERT(0 == rusb->DCPCTR_b.SUREQ);

  rusb->DCPCTR = RUSB2_PIPE_CTR_PID_NAK;

  _hcd.pipe[0].buf       = NULL;
  _hcd.pipe[0].length    = 8;
  _hcd.pipe[0].remaining = 0;
  _hcd.pipe[0].dev       = dev_addr;

  while (rusb->DCPCTR_b.PBUSY) ;
  rusb->DCPMAXP = (dev_addr << 12) | _hcd.ctl_mps[dev_addr];

  /* Set direction in advance for DATA stage */
  uint8_t const bmRequesttype = setup_packet[0];
  rusb->DCPCFG_b.DIR = tu_edpt_dir(bmRequesttype) ? 0: 1;

  uint16_t const* p = (uint16_t const*)(uintptr_t)&setup_packet[0];
  rusb->USBREQ  = tu_htole16(p[0]);
  rusb->USBVAL  = p[1];
  rusb->USBINDX = p[2];
  rusb->USBLENG = p[3];

  rusb->DCPCTR_b.SUREQ = 1;
  return true;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const *ep_desc)
{
  TU_ASSERT(dev_addr < 6); /* USBa can only handle addresses from 0 to 5. */
  rusb2_reg_t* rusb = RUSB2_REG(rhport);

  const unsigned ep_addr = ep_desc->bEndpointAddress;
  const unsigned epn     = tu_edpt_number(ep_addr);
  const unsigned mps     = tu_edpt_packet_size(ep_desc);

  if (0 == epn) {
    rusb->DCPCTR = RUSB2_PIPE_CTR_PID_NAK;
    hcd_devtree_info_t devtree;
    hcd_devtree_get_info(dev_addr, &devtree);
    uint16_t volatile *devadd = (uint16_t volatile *)(uintptr_t) &rusb->DEVADD[0];
    devadd += dev_addr;
    while (rusb->DCPCTR_b.PBUSY) {}
    rusb->DCPMAXP = (dev_addr << 12) | mps;
    *devadd = (TUSB_SPEED_FULL == devtree.speed) ? RUSB2_DEVADD_USBSPD_FS : RUSB2_DEVADD_USBSPD_LS;
    _hcd.ctl_mps[dev_addr] = mps;
    return true;
  }

  const unsigned dir_in = tu_edpt_dir(ep_addr);
  const unsigned xfer   = ep_desc->bmAttributes.xfer;
  if (xfer == TUSB_XFER_ISOCHRONOUS && mps > 256) {
    /* USBa supports up to 256 bytes */
    return false;
  }
  const unsigned num = find_pipe(xfer);
  if (!num) return false;

  _hcd.pipe[num].dev = dev_addr;
  _hcd.pipe[num].ep  = ep_addr;
  _hcd.ep[dev_addr - 1][dir_in][epn - 1] = num;

  /* setup pipe */
  hcd_int_disable(rhport);

  rusb->PIPESEL = num;
  rusb->PIPEMAXP = (dev_addr << 12) | mps;
  volatile uint16_t *ctr = get_pipectr(rusb, num);
  *ctr = RUSB2_PIPE_CTR_ACLRM_Msk | RUSB2_PIPE_CTR_SQCLR_Msk;
  *ctr = 0;

  unsigned cfg = ((1 ^ dir_in) << 4) | epn;
  if (xfer == TUSB_XFER_BULK) {
    cfg |= RUSB2_PIPECFG_TYPE_BULK | RUSB2_PIPECFG_SHTNAK_Msk | RUSB2_PIPECFG_DBLB_Msk;
  } else if (xfer == TUSB_XFER_INTERRUPT) {
    cfg |= RUSB2_PIPECFG_TYPE_INT;
  } else {
    cfg |= RUSB2_PIPECFG_TYPE_ISO | RUSB2_PIPECFG_DBLB_Msk;
  }

  rusb->PIPECFG = cfg;
  rusb->BRDYSTS = 0x3FFu ^ TU_BIT(num);
  rusb->NRDYENB |= TU_BIT(num);
  rusb->BRDYENB |= TU_BIT(num);

  if (!dir_in) {
    *ctr = RUSB2_PIPE_CTR_PID_BUF;
  }

  hcd_int_enable(rhport);

  return true;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
  bool r;
  hcd_int_disable(rhport);
  TU_LOG(TU_RUSB2_HCD_DBG, "X %d %x %u\r\n", dev_addr, ep_addr, buflen);
  r = process_edpt_xfer(rhport, dev_addr, ep_addr, buffer, buflen);
  hcd_int_enable(rhport);
  return r;
}

bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  // TODO not implemented yet
  return false;
}

bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  uint16_t volatile *ctr = addr_to_pipectr(rhport, dev_addr, ep_addr);
  TU_ASSERT(ctr);

  const uint32_t pid = *ctr & 0x3;
  if (pid & 2) {
    *ctr = pid & 2;
    *ctr = 0;
  }
  *ctr = RUSB2_PIPE_CTR_SQCLR_Msk;
  unsigned const epn = tu_edpt_number(ep_addr);
  if (!epn) return true;

  if (!tu_edpt_dir(ep_addr)) { /* OUT */
    *ctr = RUSB2_PIPE_CTR_PID_BUF;
  }
  return true;
}

//--------------------------------------------------------------------+
// ISR
//--------------------------------------------------------------------+
#if defined(__CCRX__)
TU_ATTR_ALWAYS_INLINE static inline unsigned __builtin_ctz(unsigned int value) {
  unsigned int count = 0;
  while ((value & 1) == 0) {
    value >>= 1;
    count++;
  }
  return count;
}
#endif

void hcd_int_handler(uint8_t rhport, bool in_isr) {
  (void) in_isr;

  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  unsigned is0 = rusb->INTSTS0;
  unsigned is1 = rusb->INTSTS1;

  /* clear active bits except VALID (don't write 0 to already cleared bits according to the HW manual) */
  rusb->INTSTS1 = ~((RUSB2_INTSTS1_SACK_Msk | RUSB2_INTSTS1_SIGN_Msk | RUSB2_INTSTS1_ATTCH_Msk | RUSB2_INTSTS1_DTCH_Msk) & is1);
  rusb->INTSTS0 = ~((RUSB2_INTSTS0_BRDY_Msk | RUSB2_INTSTS0_NRDY_Msk | RUSB2_INTSTS0_BEMP_Msk) & is0);

  TU_LOG3("IS %04x %04x\r\n", is0, is1);
  is1 &= rusb->INTENB1;
  is0 &= rusb->INTENB0;

  if (is1 & RUSB2_INTSTS1_SACK_Msk) {
    /* Set DATA1 in advance for the next transfer. */
    rusb->DCPCTR_b.SQSET = 1;
    hcd_event_xfer_complete(rusb->DCPMAXP_b.DEVSEL, tu_edpt_addr(0, TUSB_DIR_OUT), 8, XFER_RESULT_SUCCESS, true);
  }

  if (is1 & RUSB2_INTSTS1_SIGN_Msk) {
    hcd_event_xfer_complete(rusb->DCPMAXP_b.DEVSEL, tu_edpt_addr(0, TUSB_DIR_OUT), 8, XFER_RESULT_FAILED, true);
  }

  if (is1 & RUSB2_INTSTS1_ATTCH_Msk) {
    rusb->DVSTCTR0_b.UACT = 1;
    _hcd.need_reset = true;
    rusb->INTENB1 = (rusb->INTENB1 & ~RUSB2_INTSTS1_ATTCH_Msk) | RUSB2_INTSTS1_DTCH_Msk;
    hcd_event_device_attach(rhport, true);
  }

  if (is1 & RUSB2_INTSTS1_DTCH_Msk) {
    rusb->DVSTCTR0_b.UACT = 0;
    if (rusb->DCPCTR_b.SUREQ) {
      rusb->DCPCTR_b.SUREQCLR = 1;
    }
    rusb->INTENB1 = (rusb->INTENB1 & ~RUSB2_INTSTS1_DTCH_Msk) | RUSB2_INTSTS1_ATTCH_Msk;
    hcd_event_device_remove(rhport, true);
  }

  if (is0 & RUSB2_INTSTS0_BEMP_Msk) {
    const unsigned s = rusb->BEMPSTS;
    rusb->BEMPSTS = 0;
    if (s & 1) {
      process_pipe0_bemp(rhport);
    }
  }

  if (is0 & RUSB2_INTSTS0_NRDY_Msk) {
    const unsigned m = rusb->NRDYENB;
    unsigned s = rusb->NRDYSTS & m;
    rusb->NRDYSTS = ~s;
    while (s) {
      const unsigned num = __builtin_ctz(s);
      process_pipe_nrdy(rhport, num);
      s &= ~TU_BIT(num);
    }
  }
  if (is0 & RUSB2_INTSTS0_BRDY_Msk) {
    const unsigned m = rusb->BRDYENB;
    unsigned s = rusb->BRDYSTS & m;
    /* clear active bits (don't write 0 to already cleared bits according to the HW manual) */
    rusb->BRDYSTS = ~s;
    while (s) {
      const unsigned num = __builtin_ctz(s);
      process_pipe_brdy(rhport, num);
      s &= ~TU_BIT(num);
    }
  }
}

#endif
