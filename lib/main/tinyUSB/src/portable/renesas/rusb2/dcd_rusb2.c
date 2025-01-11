/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Koji Kitayama
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

#if CFG_TUD_ENABLED && defined(TUP_USBIP_RUSB2)

#include "device/dcd.h"
#include "rusb2_type.h"

#if TU_CHECK_MCU(OPT_MCU_RX63X, OPT_MCU_RX65X, OPT_MCU_RX72N)
  #include "rusb2_rx.h"
#elif TU_CHECK_MCU(OPT_MCU_RAXXX)
  #include "rusb2_ra.h"
  #if defined(RENESAS_CORTEX_M23)
    #define D0FIFO CFIFO
    #define D0FIFOSEL CFIFOSEL
    #define D0FIFOSEL_b CFIFOSEL_b
    #define D1FIFOSEL CFIFOSEL
    #define D1FIFOSEL_b CFIFOSEL_b
    #define D0FIFOCTR CFIFOCTR
    #define D0FIFOCTR_b CFIFOCTR_b
  #endif

#else
  #error "Unsupported MCU"
#endif

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
enum {
  PIPE_COUNT = 10,
};

typedef struct {
  void      *buf;      /* the start address of a transfer data buffer */
  uint16_t  length;    /* the number of bytes in the buffer */
  uint16_t  remaining; /* the number of bytes remaining in the buffer */

  uint8_t ep; /* an assigned endpoint address */
  uint8_t ff; /* `buf` is TU_FUFO or POD */
} pipe_state_t;

typedef struct
{
  pipe_state_t pipe[PIPE_COUNT];
  uint8_t ep[2][16];   /* a lookup table for a pipe index from an endpoint address */
  // Track whether sof has been manually enabled
  bool sof_enabled;
} dcd_data_t;

static dcd_data_t _dcd;

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+


// Transfer conditions specifiable for each pipe for most MCUs
// - Pipe 0: Control transfer with 64-byte single buffer
// - Pipes 1 and 2: Bulk or ISO
// - Pipes 3 to 5: Bulk
// - Pipes 6 to 9: Interrupt
//
// Note: for small mcu such as
// - RA2A1: only pipe 4-7 are available, and no support for ISO
static unsigned find_pipe(unsigned xfer_type) {
  #if defined(BSP_MCU_GROUP_RA2A1)
  const uint8_t pipe_idx_arr[4][2] = {
      { 0, 0 }, // Control
      { 0, 0 }, // Isochronous not supported
      { 4, 5 }, // Bulk
      { 6, 7 }, // Interrupt
  };
  #else
  const uint8_t pipe_idx_arr[4][2] = {
      { 0, 0 }, // Control
      { 1, 2 }, // Isochronous
      { 1, 5 }, // Bulk
      { 6, 9 }, // Interrupt
  };
  #endif

  // find backward since only pipe 1, 2 support ISO
  const uint8_t idx_first = pipe_idx_arr[xfer_type][0];
  const uint8_t idx_last  = pipe_idx_arr[xfer_type][1];

  for (int i = idx_last; i >= idx_first; i--) {
    if (0 == _dcd.pipe[i].ep) return i;
  }

  return 0;
}

static volatile uint16_t* get_pipectr(rusb2_reg_t *rusb, unsigned num) {
  if (num) {
    return (volatile uint16_t*)&(rusb->PIPE_CTR[num - 1]);
  } else {
    return (volatile uint16_t*)&(rusb->DCPCTR);
  }
}

static volatile reg_pipetre_t* get_pipetre(rusb2_reg_t *rusb, unsigned num) {
  volatile reg_pipetre_t* tre = NULL;
  if ((1 <= num) && (num <= 5)) {
    tre = (volatile reg_pipetre_t*)&(rusb->PIPE_TR[num - 1].E);
  }
  return tre;
}

static volatile uint16_t* ep_addr_to_pipectr(uint8_t rhport, unsigned ep_addr) {
  rusb2_reg_t *rusb = RUSB2_REG(rhport);
  const unsigned epn = tu_edpt_number(ep_addr);

  if (epn) {
    const unsigned dir = tu_edpt_dir(ep_addr);
    const unsigned num = _dcd.ep[dir][epn];
    return get_pipectr(rusb, num);
  } else {
    return get_pipectr(rusb, 0);
  }
}

static uint16_t edpt0_max_packet_size(rusb2_reg_t* rusb) {
  return rusb->DCPMAXP_b.MXPS;
}

static uint16_t edpt_max_packet_size(rusb2_reg_t *rusb, unsigned num) {
  rusb->PIPESEL = num;
  return rusb->PIPEMAXP;
}

static inline void pipe_wait_for_ready(rusb2_reg_t * rusb, unsigned num) {
  while ( rusb->D0FIFOSEL_b.CURPIPE != num ) {}
  while ( !rusb->D0FIFOCTR_b.FRDY ) {}
}

//--------------------------------------------------------------------+
// Pipe FIFO
//--------------------------------------------------------------------+

// Write data buffer --> hw fifo
static void pipe_write_packet(rusb2_reg_t * rusb, void *buf, volatile void *fifo, unsigned len)
{
  (void) rusb;

  volatile uint16_t *ff16;
  volatile uint8_t *ff8;

  // Highspeed FIFO is 32-bit
  if ( rusb2_is_highspeed_reg(rusb) ) {
    // TODO 32-bit access for better performance
    ff16 = (volatile uint16_t*) ((uintptr_t) fifo+2);
    ff8  = (volatile uint8_t *) ((uintptr_t) fifo+3);
  }else {
    ff16 = (volatile uint16_t*) fifo;
    ff8  = ((volatile uint8_t*) fifo);
  }

  uint8_t const* buf8 = (uint8_t const*) buf;

  while (len >= 2) {
    *ff16 = tu_unaligned_read16(buf8);
    buf8 += 2;
    len  -= 2;
  }

  if (len > 0) {
    *ff8 = *buf8;
    ++buf8;
  }
}

// Read data buffer <-- hw fifo
static void pipe_read_packet(rusb2_reg_t * rusb, void *buf, volatile void *fifo, unsigned len)
{
  (void) rusb;

  // TODO 16/32-bit access for better performance

  uint8_t *p = (uint8_t*)buf;
  volatile uint8_t *reg = (volatile uint8_t*)fifo;  /* byte access is always at base register address */
  while (len--) *p++ = *reg;
}

// Write data sw fifo --> hw fifo
static void pipe_write_packet_ff(rusb2_reg_t * rusb, tu_fifo_t *f, volatile void *fifo, uint16_t total_len) {
  tu_fifo_buffer_info_t info;
  tu_fifo_get_read_info(f, &info);

  uint16_t count = tu_min16(total_len, info.len_lin);
  pipe_write_packet(rusb, info.ptr_lin, fifo, count);

  uint16_t rem = total_len - count;
  if (rem) {
    rem = tu_min16(rem, info.len_wrap);
    pipe_write_packet(rusb, info.ptr_wrap, fifo, rem);
    count += rem;
  }

  tu_fifo_advance_read_pointer(f, count);
}

// Read data sw fifo <-- hw fifo
static void pipe_read_packet_ff(rusb2_reg_t * rusb, tu_fifo_t *f, volatile void *fifo, uint16_t total_len) {
  tu_fifo_buffer_info_t info;
  tu_fifo_get_write_info(f, &info);

  uint16_t count = tu_min16(total_len, info.len_lin);
  pipe_read_packet(rusb, info.ptr_lin, fifo, count);

  uint16_t rem = total_len - count;
  if (rem) {
    rem = tu_min16(rem, info.len_wrap);
    pipe_read_packet(rusb, info.ptr_wrap, fifo, rem);
    count += rem;
  }

  tu_fifo_advance_write_pointer(f, count);
}

//--------------------------------------------------------------------+
// Pipe Transfer
//--------------------------------------------------------------------+

static bool pipe0_xfer_in(rusb2_reg_t* rusb)
{
  pipe_state_t *pipe = &_dcd.pipe[0];
  const unsigned rem = pipe->remaining;

  if (!rem) {
    pipe->buf = NULL;
    return true;
  }

  const uint16_t mps = edpt0_max_packet_size(rusb);
  const uint16_t len = tu_min16(mps, rem);
  void          *buf = pipe->buf;

  if (len) {
    if (pipe->ff) {
      pipe_write_packet_ff(rusb, (tu_fifo_t*)buf, (volatile void*)&rusb->CFIFO, len);
    } else {
      pipe_write_packet(rusb, buf, (volatile void*)&rusb->CFIFO, len);
      pipe->buf = (uint8_t*)buf + len;
    }
  }

  if (len < mps) {
    rusb->CFIFOCTR = RUSB2_CFIFOCTR_BVAL_Msk;
  }

  pipe->remaining = rem - len;
  return false;
}

static bool pipe0_xfer_out(rusb2_reg_t* rusb)
{
  pipe_state_t *pipe = &_dcd.pipe[0];
  const unsigned rem = pipe->remaining;

  const uint16_t mps = edpt0_max_packet_size(rusb);
  const uint16_t vld = rusb->CFIFOCTR_b.DTLN;
  const uint16_t len = tu_min16(tu_min16(rem, mps), vld);
  void          *buf = pipe->buf;

  if (len) {
    if (pipe->ff) {
      pipe_read_packet_ff(rusb, (tu_fifo_t*)buf, (volatile void*)&rusb->CFIFO, len);
    } else {
      pipe_read_packet(rusb, buf, (volatile void*)&rusb->CFIFO, len);
      pipe->buf = (uint8_t*)buf + len;
    }
  }

  if (len < mps) {
    rusb->CFIFOCTR = RUSB2_CFIFOCTR_BCLR_Msk;
  }

  pipe->remaining = rem - len;
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return true;
  }

  return false;
}

static bool pipe_xfer_in(rusb2_reg_t* rusb, unsigned num)
{
  pipe_state_t  *pipe = &_dcd.pipe[num];
  const unsigned rem  = pipe->remaining;

  if (!rem) {
    pipe->buf = NULL;
    return true;
  }

  rusb->D0FIFOSEL = num | RUSB2_FIFOSEL_MBW_16BIT | (TU_BYTE_ORDER == TU_BIG_ENDIAN ? RUSB2_FIFOSEL_BIGEND : 0);
  const uint16_t mps  = edpt_max_packet_size(rusb, num);
  pipe_wait_for_ready(rusb, num);
  const uint16_t len  = tu_min16(rem, mps);
  void          *buf  = pipe->buf;

  if (len) {
    if (pipe->ff) {
      pipe_write_packet_ff(rusb, (tu_fifo_t*)buf, (volatile void*)&rusb->D0FIFO, len);
    } else {
      pipe_write_packet(rusb, buf, (volatile void*)&rusb->D0FIFO, len);
      pipe->buf = (uint8_t*)buf + len;
    }
  }

  if (len < mps) {
    rusb->D0FIFOCTR = RUSB2_CFIFOCTR_BVAL_Msk;
  }

  rusb->D0FIFOSEL = 0;
  while (rusb->D0FIFOSEL_b.CURPIPE) {} /* if CURPIPE bits changes, check written value */

  pipe->remaining = rem - len;

  return false;
}

static bool pipe_xfer_out(rusb2_reg_t* rusb, unsigned num)
{
  pipe_state_t  *pipe = &_dcd.pipe[num];
  const uint16_t rem  = pipe->remaining;

  rusb->D0FIFOSEL = num | RUSB2_FIFOSEL_MBW_8BIT;
  const uint16_t mps = edpt_max_packet_size(rusb, num);
  pipe_wait_for_ready(rusb, num);

  const uint16_t vld  = rusb->D0FIFOCTR_b.DTLN;
  const uint16_t len  = tu_min16(tu_min16(rem, mps), vld);
  void          *buf  = pipe->buf;

  if (len) {
    if (pipe->ff) {
      pipe_read_packet_ff(rusb, (tu_fifo_t*)buf, (volatile void*)&rusb->D0FIFO, len);
    } else {
      pipe_read_packet(rusb, buf, (volatile void*)&rusb->D0FIFO, len);
      pipe->buf = (uint8_t*)buf + len;
    }
  }

  if (len < mps) {
    rusb->D0FIFOCTR = RUSB2_CFIFOCTR_BCLR_Msk;
  }

  rusb->D0FIFOSEL = 0;
  while (rusb->D0FIFOSEL_b.CURPIPE) {} /* if CURPIPE bits changes, check written value */

  pipe->remaining = rem - len;
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return NULL != buf;
  }

  return false;
}

static void process_setup_packet(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  if (0 == (rusb->INTSTS0 & RUSB2_INTSTS0_VALID_Msk)) return;

  rusb->CFIFOCTR = RUSB2_CFIFOCTR_BCLR_Msk;
  uint16_t setup_packet[4] = {
      tu_htole16(rusb->USBREQ),
      tu_htole16(rusb->USBVAL),
      tu_htole16(rusb->USBINDX),
      tu_htole16(rusb->USBLENG)
  };

  rusb->INTSTS0 = ~((uint16_t) RUSB2_INTSTS0_VALID_Msk);
  dcd_event_setup_received(rhport, (const uint8_t*)&setup_packet[0], true);
}

static void process_status_completion(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  uint8_t ep_addr;
  /* Check the data stage direction */
  if (rusb->CFIFOSEL & RUSB2_CFIFOSEL_ISEL_WRITE) {
    /* IN transfer. */
    ep_addr = tu_edpt_addr(0, TUSB_DIR_IN);
  } else {
    /* OUT transfer. */
    ep_addr = tu_edpt_addr(0, TUSB_DIR_OUT);
  }

  dcd_event_xfer_complete(rhport, ep_addr, 0, XFER_RESULT_SUCCESS, true);
}

static bool process_pipe0_xfer(rusb2_reg_t* rusb, int buffer_type, uint8_t ep_addr, void* buffer, uint16_t total_bytes)
{
  /* configure fifo direction and access unit settings */
  if ( ep_addr ) {
    /* IN, 2 bytes */
    rusb->CFIFOSEL = RUSB2_CFIFOSEL_ISEL_WRITE | RUSB2_FIFOSEL_MBW_16BIT |
                     (TU_BYTE_ORDER == TU_BIG_ENDIAN ? RUSB2_FIFOSEL_BIGEND : 0);
    while ( !(rusb->CFIFOSEL & RUSB2_CFIFOSEL_ISEL_WRITE) ) {}
  } else {
    /* OUT, a byte */
    rusb->CFIFOSEL = RUSB2_FIFOSEL_MBW_8BIT;
    while ( rusb->CFIFOSEL & RUSB2_CFIFOSEL_ISEL_WRITE ) {}
  }

  pipe_state_t *pipe = &_dcd.pipe[0];
  pipe->ff        = buffer_type;
  pipe->length    = total_bytes;
  pipe->remaining = total_bytes;

  if ( total_bytes ) {
    pipe->buf = buffer;
    if ( ep_addr ) {
      /* IN */
      TU_ASSERT(rusb->DCPCTR_b.BSTS && (rusb->USBREQ & 0x80));
      pipe0_xfer_in(rusb);
    }
    rusb->DCPCTR = RUSB2_PIPE_CTR_PID_BUF;
  } else {
    /* ZLP */
    pipe->buf = NULL;
    rusb->DCPCTR = RUSB2_DCPCTR_CCPL_Msk | RUSB2_PIPE_CTR_PID_BUF;
  }

  return true;
}

static bool process_pipe_xfer(rusb2_reg_t* rusb, int buffer_type, uint8_t ep_addr, void* buffer, uint16_t total_bytes)
{
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);
  const unsigned num = _dcd.ep[dir][epn];

  TU_ASSERT(num);

  pipe_state_t *pipe  = &_dcd.pipe[num];
  pipe->ff        = buffer_type;
  pipe->buf       = buffer;
  pipe->length    = total_bytes;
  pipe->remaining = total_bytes;

  if (dir) {
    /* IN */
    if (total_bytes) {
      pipe_xfer_in(rusb, num);
    } else {
      /* ZLP */
      rusb->D0FIFOSEL = num;
      pipe_wait_for_ready(rusb, num);
      rusb->D0FIFOCTR = RUSB2_CFIFOCTR_BVAL_Msk;
      rusb->D0FIFOSEL = 0;
      /* if CURPIPE bits changes, check written value */
      while (rusb->D0FIFOSEL_b.CURPIPE) {}
    }
  } else {
    // OUT
    volatile reg_pipetre_t *pt = get_pipetre(rusb, num);

    if (pt) {
      const uint16_t     mps = edpt_max_packet_size(rusb, num);
      volatile uint16_t *ctr = get_pipectr(rusb, num);

      if (*ctr & 0x3) *ctr = RUSB2_PIPE_CTR_PID_NAK;

      pt->TRE   = TU_BIT(8);
      pt->TRN   = (total_bytes + mps - 1) / mps;
      pt->TRENB = 1;
      *ctr = RUSB2_PIPE_CTR_PID_BUF;
    }
  }

  //  TU_LOG2("X %x %d %d\r\n", ep_addr, total_bytes, buffer_type);
  return true;
}

static bool process_edpt_xfer(rusb2_reg_t* rusb, int buffer_type, uint8_t ep_addr, void* buffer, uint16_t total_bytes)
{
  const unsigned epn = tu_edpt_number(ep_addr);
  if (0 == epn) {
    return process_pipe0_xfer(rusb, buffer_type, ep_addr, buffer, total_bytes);
  } else {
    return process_pipe_xfer(rusb, buffer_type, ep_addr, buffer, total_bytes);
  }
}

static void process_pipe0_bemp(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  bool completed = pipe0_xfer_in(rusb);
  if (completed) {
    pipe_state_t *pipe = &_dcd.pipe[0];
    dcd_event_xfer_complete(rhport, tu_edpt_addr(0, TUSB_DIR_IN),
                            pipe->length, XFER_RESULT_SUCCESS, true);
  }
}

static void process_pipe_brdy(uint8_t rhport, unsigned num)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  pipe_state_t  *pipe = &_dcd.pipe[num];
  const unsigned dir  = tu_edpt_dir(pipe->ep);
  bool completed;

  if (dir) {
    /* IN */
    completed = pipe_xfer_in(rusb, num);
  } else {
    // OUT
    if (num) {
      completed = pipe_xfer_out(rusb, num);
    } else {
      completed = pipe0_xfer_out(rusb);
    }
  }
  if (completed) {
    dcd_event_xfer_complete(rhport, pipe->ep,
                            pipe->length - pipe->remaining,
                            XFER_RESULT_SUCCESS, true);
    //  TU_LOG1("C %d %d\r\n", num, pipe->length - pipe->remaining);
  }
}

static void process_bus_reset(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);

  rusb->BEMPENB = 1;
  rusb->BRDYENB = 1;
  rusb->CFIFOCTR = RUSB2_CFIFOCTR_BCLR_Msk;

  rusb->D0FIFOSEL = 0;
  while (rusb->D0FIFOSEL_b.CURPIPE) {} /* if CURPIPE bits changes, check written value */

  rusb->D1FIFOSEL = 0;
  while (rusb->D1FIFOSEL_b.CURPIPE) {} /* if CURPIPE bits changes, check written value */

  volatile uint16_t *ctr = (volatile uint16_t*)((uintptr_t) (&rusb->PIPE_CTR[0]));
  volatile uint16_t *tre = (volatile uint16_t*)((uintptr_t) (&rusb->PIPE_TR[0].E));

  for (int i = 1; i <= 5; ++i) {
    rusb->PIPESEL = i;
    rusb->PIPECFG = 0;
    *ctr = RUSB2_PIPE_CTR_ACLRM_Msk;
    *ctr = 0;
    ++ctr;
    *tre = TU_BIT(8);
    tre += 2;
  }

  for (int i = 6; i <= 9; ++i) {
    rusb->PIPESEL = i;
    rusb->PIPECFG = 0;
    *ctr = RUSB2_PIPE_CTR_ACLRM_Msk;
    *ctr = 0;
    ++ctr;
  }
  tu_varclr(&_dcd);

  TU_LOG3("Bus reset, RHST = %u\r\n", rusb->DVSTCTR0_b.RHST);
  tusb_speed_t speed;
  switch(rusb->DVSTCTR0 & RUSB2_DVSTCTR0_RHST_Msk) {
    case RUSB2_DVSTCTR0_RHST_LS:
      speed = TUSB_SPEED_LOW;
      break;

    case RUSB2_DVSTCTR0_RHST_FS:
      speed = TUSB_SPEED_FULL;
      break;

    case RUSB2_DVSTCTR0_RHST_HS:
      speed = TUSB_SPEED_HIGH;
      break;

    default:
      TU_ASSERT(false, );
  }

  dcd_event_bus_reset(rhport, speed, true);
}

static void process_set_address(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  const uint16_t addr = rusb->USBADDR_b.USBADDR;
  if (!addr) return;

  const tusb_control_request_t setup_packet = {
#if defined(__CCRX__)
      .bmRequestType = { 0 },  /* Note: CCRX needs the braces over this struct member */
#else
      .bmRequestType = 0,
#endif
      .bRequest      = TUSB_REQ_SET_ADDRESS,
      .wValue        = addr,
      .wIndex        = 0,
      .wLength       = 0,
  };

  dcd_event_setup_received(rhport, (const uint8_t *) &setup_packet, true);
}

/*------------------------------------------------------------------*/
/* Device API
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

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  rusb2_module_start(rhport, true);

  // We disable SOF for now until needed later on.
  // Since TinyUSB doesn't use SOF for now, and this interrupt often (1ms interval)
  _dcd.sof_enabled = false;

#ifdef RUSB2_SUPPORT_HIGHSPEED
  if ( rusb2_is_highspeed_rhport(rhport) ) {
    rusb->SYSCFG_b.HSE = 1;

    // leave CLKSEL as default (0x11) 24Mhz

    // Power and reset UTMI Phy
    uint16_t physet = (rusb->PHYSET | RUSB2_PHYSET_PLLRESET_Msk) & ~RUSB2_PHYSET_DIRPD_Msk;
    rusb->PHYSET = physet;
    R_BSP_SoftwareDelay((uint32_t) 1, BSP_DELAY_UNITS_MILLISECONDS);
    rusb->PHYSET_b.PLLRESET = 0;

    // set UTMI to operating mode and wait for PLL lock confirmation
    rusb->LPSTS_b.SUSPENDM = 1;
    while (!rusb->PLLSTA_b.PLLLOCK) {}

    rusb->SYSCFG_b.DRPD = 0;
    rusb->SYSCFG_b.USBE = 1;

    // Set CPU bus wait time (fine tunne later)
    // rusb2->BUSWAIT |= 0x0F00U;

    rusb->PHYSET_b.REPSEL = 1;
  } else
#endif
  {
    rusb->SYSCFG_b.SCKE = 1;
    while (!rusb->SYSCFG_b.SCKE) {}
    rusb->SYSCFG_b.DRPD = 0;
    rusb->SYSCFG_b.DCFM = 0;
    rusb->SYSCFG_b.USBE = 1;

    // MCU specific PHY init
    rusb2_phy_init();

    rusb->PHYSLEW = 0x5;
    rusb->DPUSR0R_FS_b.FIXPHY0 = 0u; /* USB_BASE Transceiver Output fixed */
  }

  /* Setup default control pipe */
  rusb->DCPMAXP_b.MXPS = 64;

  rusb->INTSTS0 = 0;
  rusb->INTENB0 = RUSB2_INTSTS0_VBINT_Msk | RUSB2_INTSTS0_BRDY_Msk | RUSB2_INTSTS0_BEMP_Msk |
                  RUSB2_INTSTS0_DVST_Msk | RUSB2_INTSTS0_CTRT_Msk | (_dcd.sof_enabled ? RUSB2_INTSTS0_SOFR_Msk : 0) |
                  RUSB2_INTSTS0_RESM_Msk;
  rusb->BEMPENB = 1;
  rusb->BRDYENB = 1;

  // If VBUS (detect) pin is not used, application need to call tud_connect() manually after tud_init()
  if (rusb->INTSTS0_b.VBSTS) {
    dcd_connect(rhport);
  }

  return true;
}

void dcd_int_enable(uint8_t rhport) {
  rusb2_int_enable(rhport);
}

void dcd_int_disable(uint8_t rhport) {
  rusb2_int_disable(rhport);
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr) {
  (void) rhport;
  (void) dev_addr;
}

void dcd_remote_wakeup(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  rusb->DVSTCTR0_b.WKUP = 1;
}

void dcd_connect(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);

  if ( rusb2_is_highspeed_rhport(rhport)) {
    rusb->SYSCFG_b.CNEN = 1;
  }
  rusb->SYSCFG_b.DPRPU = 1;
}

void dcd_disconnect(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  rusb->SYSCFG_b.DPRPU = 0;
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);
  _dcd.sof_enabled = en;
  rusb->INTENB0_b.SOFE = en ? 1: 0;
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void)rhport;

  rusb2_reg_t * rusb = RUSB2_REG(rhport);
  const unsigned ep_addr = ep_desc->bEndpointAddress;
  const unsigned epn     = tu_edpt_number(ep_addr);
  const unsigned dir     = tu_edpt_dir(ep_addr);
  const unsigned xfer    = ep_desc->bmAttributes.xfer;

  const unsigned mps = tu_edpt_packet_size(ep_desc);

  if (xfer == TUSB_XFER_ISOCHRONOUS) {
    // Fullspeed ISO is limit to 256 bytes
    if ( !rusb2_is_highspeed_rhport(rhport) && mps > 256) {
      return false;
    }
  }

  const unsigned num = find_pipe(xfer);
  TU_ASSERT(num);

  _dcd.pipe[num].ep = ep_addr;
  _dcd.ep[dir][epn] = num;

  /* setup pipe */
  dcd_int_disable(rhport);

  if ( rusb2_is_highspeed_rhport(rhport) ) {
    // FIXME shouldn't be after pipe selection and config, also the BUFNMB should be changed
    //       depending on the allocation scheme
    rusb->PIPEBUF = 0x7C08;
  }

  rusb->PIPESEL = num;
  rusb->PIPEMAXP = mps;
  volatile uint16_t *ctr = get_pipectr(rusb, num);
  *ctr = RUSB2_PIPE_CTR_ACLRM_Msk | RUSB2_PIPE_CTR_SQCLR_Msk;
  *ctr = 0;
  unsigned cfg = (dir << 4) | epn;

  if (xfer == TUSB_XFER_BULK) {
    cfg |= (RUSB2_PIPECFG_TYPE_BULK | RUSB2_PIPECFG_SHTNAK_Msk | RUSB2_PIPECFG_DBLB_Msk);
  } else if (xfer == TUSB_XFER_INTERRUPT) {
    cfg |= RUSB2_PIPECFG_TYPE_INT;
  } else {
    cfg |= (RUSB2_PIPECFG_TYPE_ISO | RUSB2_PIPECFG_DBLB_Msk);
  }

  rusb->PIPECFG = cfg;
  rusb->BRDYSTS = 0x3FFu ^ TU_BIT(num);
  rusb->BRDYENB |= TU_BIT(num);

  if (dir || (xfer != TUSB_XFER_BULK)) {
    *ctr = RUSB2_PIPE_CTR_PID_BUF;
  }

  // TU_LOG1("O %d %x %x\r\n", rusb->PIPESEL, rusb->PIPECFG, rusb->PIPEMAXP);
  dcd_int_enable(rhport);

  return true;
}

void dcd_edpt_close_all(uint8_t rhport)
{
  unsigned i = TU_ARRAY_SIZE(_dcd.pipe);
  dcd_int_disable(rhport);
  while (--i) { /* Close all pipes except 0 */
    const unsigned ep_addr = _dcd.pipe[i].ep;
    if (!ep_addr) continue;
    dcd_edpt_close(rhport, ep_addr);
  }
  dcd_int_enable(rhport);
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  rusb2_reg_t * rusb = RUSB2_REG(rhport);
  const unsigned epn = tu_edpt_number(ep_addr);
  const unsigned dir = tu_edpt_dir(ep_addr);
  const unsigned num = _dcd.ep[dir][epn];

  rusb->BRDYENB &= ~TU_BIT(num);
  volatile uint16_t *ctr = get_pipectr(rusb, num);
  *ctr = 0;
  rusb->PIPESEL = num;
  rusb->PIPECFG = 0;
  _dcd.pipe[num].ep = 0;
  _dcd.ep[dir][epn] = 0;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);

  dcd_int_disable(rhport);
  bool r = process_edpt_xfer(rusb, 0, ep_addr, buffer, total_bytes);
  dcd_int_enable(rhport);

  return r;
}

bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  // USB buffers always work in bytes so to avoid unnecessary divisions we demand item_size = 1
  TU_ASSERT(ff->item_size == 1);
  rusb2_reg_t* rusb = RUSB2_REG(rhport);

  dcd_int_disable(rhport);
  bool r = process_edpt_xfer(rusb, 1, ep_addr, ff, total_bytes);
  dcd_int_enable(rhport);

  return r;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  volatile uint16_t *ctr = ep_addr_to_pipectr(rhport, ep_addr);
  if (!ctr) return;
  dcd_int_disable(rhport);
  const uint32_t pid = *ctr & 0x3;
  *ctr = pid | RUSB2_PIPE_CTR_PID_STALL;
  *ctr = RUSB2_PIPE_CTR_PID_STALL;
  dcd_int_enable(rhport);
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  rusb2_reg_t * rusb = RUSB2_REG(rhport);
  volatile uint16_t *ctr = ep_addr_to_pipectr(rhport, ep_addr);
  if (!ctr) return;

  dcd_int_disable(rhport);
  *ctr = RUSB2_PIPE_CTR_SQCLR_Msk;

  if (tu_edpt_dir(ep_addr)) { /* IN */
    *ctr = RUSB2_PIPE_CTR_PID_BUF;
  } else {
    const unsigned num = _dcd.ep[0][tu_edpt_number(ep_addr)];
    rusb->PIPESEL = num;
    if (rusb->PIPECFG_b.TYPE != 1) {
      *ctr = RUSB2_PIPE_CTR_PID_BUF;
    }
  }
  dcd_int_enable(rhport);
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

void dcd_int_handler(uint8_t rhport)
{
  rusb2_reg_t* rusb = RUSB2_REG(rhport);

  uint16_t is0 = rusb->INTSTS0;

  /* clear active bits except VALID (don't write 0 to already cleared bits according to the HW manual) */
  rusb->INTSTS0 = ~((RUSB2_INTSTS0_CTRT_Msk | RUSB2_INTSTS0_DVST_Msk | RUSB2_INTSTS0_SOFR_Msk |
                     RUSB2_INTSTS0_RESM_Msk | RUSB2_INTSTS0_VBINT_Msk) & is0) | RUSB2_INTSTS0_VALID_Msk;

  // VBUS changes
  if ( is0 & RUSB2_INTSTS0_VBINT_Msk ) {
    if ( rusb->INTSTS0_b.VBSTS ) {
      dcd_connect(rhport);
    } else {
      dcd_disconnect(rhport);
    }
  }

  // Resumed
  if ( is0 & RUSB2_INTSTS0_RESM_Msk ) {
    dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
    if (!_dcd.sof_enabled) {
      rusb->INTENB0_b.SOFE = 0;
    }
  }

  // SOF received
  if ( (is0 & RUSB2_INTSTS0_SOFR_Msk) && rusb->INTENB0_b.SOFE ) {
    // USBD will exit suspended mode when SOF event is received
    const uint32_t frame = rusb->FRMNUM_b.FRNM;
    dcd_event_sof(rhport, frame, true);
    if (!_dcd.sof_enabled) {
      rusb->INTENB0_b.SOFE = 0;
    }
  }

  // Device state changes
  if ( is0 & RUSB2_INTSTS0_DVST_Msk ) {
    switch (is0 & RUSB2_INTSTS0_DVSQ_Msk) {
      case RUSB2_INTSTS0_DVSQ_STATE_DEF:
        process_bus_reset(rhport);
        break;

      case RUSB2_INTSTS0_DVSQ_STATE_ADDR:
        process_set_address(rhport);
        break;

      case RUSB2_INTSTS0_DVSQ_STATE_SUSP0:
      case RUSB2_INTSTS0_DVSQ_STATE_SUSP1:
      case RUSB2_INTSTS0_DVSQ_STATE_SUSP2:
      case RUSB2_INTSTS0_DVSQ_STATE_SUSP3:
        dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
        if (!_dcd.sof_enabled) {
          rusb->INTENB0_b.SOFE = 1;
        }

      default: break;
    }
  }

//  if ( is0 & RUSB2_INTSTS0_NRDY_Msk ) {
//    rusb->NRDYSTS = 0;
//  }

  // Control transfer stage changes
  if ( is0 & RUSB2_INTSTS0_CTRT_Msk ) {
    if ( is0 & RUSB2_INTSTS0_CTSQ_CTRL_RDATA ) {
      /* A setup packet has been received. */
      process_setup_packet(rhport);
    } else if ( 0 == (is0 & RUSB2_INTSTS0_CTSQ_Msk) ) {
      /* A ZLP has been sent/received. */
      process_status_completion(rhport);
    }
  }

  // Buffer empty
  if ( is0 & RUSB2_INTSTS0_BEMP_Msk ) {
    const uint16_t s = rusb->BEMPSTS;
    rusb->BEMPSTS = 0;
    if ( s & 1 ) {
      process_pipe0_bemp(rhport);
    }
  }

  // Buffer ready
  if ( is0 & RUSB2_INTSTS0_BRDY_Msk ) {
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
