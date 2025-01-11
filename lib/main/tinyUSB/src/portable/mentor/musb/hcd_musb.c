/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Koji KITAYAMA
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

#if CFG_TUH_ENABLED && \
  TU_CHECK_MCU(OPT_MCU_MSP432E4, OPT_MCU_TM4C123, OPT_MCU_TM4C129)

#if __GNUC__ > 8 && defined(__ARM_FEATURE_UNALIGNED)
/* GCC warns that an address may be unaligned, even though
 * the target CPU has the capability for unaligned memory access. */
_Pragma("GCC diagnostic ignored \"-Waddress-of-packed-member\"");
#endif

#include "host/hcd.h"

#include "musb_type.h"

#if TU_CHECK_MCU(OPT_MCU_MSP432E4, OPT_MCU_TM4C123, OPT_MCU_TM4C129)
  #include "musb_ti.h"
#else
  #error "Unsupported MCUs"
#endif

#ifndef HCD_ATTR_ENDPOINT_MAX
# define HCD_ATTR_ENDPOINT_MAX 8
#endif

/*------------------------------------------------------------------
 * MACRO TYPEDEF CONSTANT ENUM DECLARATION
 *------------------------------------------------------------------*/
#define REQUEST_TYPE_INVALID  (0xFFu)

typedef struct {
  uint_fast16_t beg; /* offset of including first element */
  uint_fast16_t end; /* offset of excluding the last element */
} free_block_t;

typedef struct TU_ATTR_PACKED {
  uint8_t TXFUNCADDR;
  uint8_t RESERVED0;
  uint8_t TXHUBADDR;
  uint8_t TXHUBPORT;
  uint8_t RXFUNCADDR;
  uint8_t RESERVED1;
  uint8_t RXHUBADDR;
  uint8_t RXHUBPORT;
} hw_addr_t;

typedef struct TU_ATTR_PACKED {
  uint16_t TXMAXP;
  uint8_t  TXCSRL;
  uint8_t  TXCSRH;
  uint16_t RXMAXP;
  uint8_t  RXCSRL;
  uint8_t  RXCSRH;
  uint16_t RXCOUNT;
  uint8_t  TXTYPE;
  uint8_t  TXINTERVAL;
  uint8_t  RXTYPE;
  uint8_t  RXINTERVAL;
  uint16_t RESERVED;
} hw_endpoint_t;

typedef union {
  uint8_t   u8;
  uint16_t  u16;
  uint32_t  u32;
} hw_fifo_t;

typedef struct TU_ATTR_PACKED
{
  void      *buf;      /* the start address of a transfer data buffer */
  uint16_t  length;    /* the number of bytes in the buffer */
  uint16_t  remaining; /* the number of bytes remaining in the buffer */
} pipe_state_t;

typedef struct TU_ATTR_PACKED
{
  uint8_t dev;
  uint8_t ep;
} pipe_addr_t;

typedef struct
{
  bool         need_reset;     /* The device has not been reset after connection. */
  uint8_t      bmRequestType;
  uint8_t      ctl_mps[7]; /* EP0 max packet size for each device */
  pipe_state_t pipe0;
  pipe_state_t pipe[7][2];   /* pipe[pipe number - 1][direction 0:RX 1:TX] */
  pipe_addr_t  addr[7][2];   /* addr[pipe number - 1][direction 0:RX 1:TX] */
} hcd_data_t;

/*------------------------------------------------------------------
 * INTERNAL OBJECT & FUNCTION DECLARATION
 *------------------------------------------------------------------*/
static hcd_data_t _hcd;

static inline free_block_t *find_containing_block(free_block_t *beg, free_block_t *end, uint_fast16_t addr)
{
  free_block_t *cur = beg;
  for (; cur < end && ((addr < cur->beg) || (cur->end <= addr)); ++cur) ;
  return cur;
}

static inline int update_free_block_list(free_block_t *blks, unsigned num, uint_fast16_t addr, uint_fast16_t size)
{
  free_block_t *p = find_containing_block(blks, blks + num, addr);
  TU_ASSERT(p != blks + num, -2);
  if (p->beg == addr) {
    /* Shrink block */
    p->beg = addr + size;
    if (p->beg != p->end) return 0;
    /* remove block */
    free_block_t *end = blks + num;
    while (p + 1 < end) {
      *p = *(p + 1);
      ++p;
    }
    return -1;
  } else {
    /* Split into 2 blocks */
    free_block_t tmp = {
      .beg = addr + size,
      .end = p->end
    };
    p->end = addr;
    if (p->beg == p->end) {
      if (tmp.beg != tmp.end) {
        *p = tmp;
        return 0;
      }
      /* remove block */
      free_block_t *end = blks + num;
      while (p + 1 < end) {
        *p = *(p + 1);
        ++p;
      }
      return -1;
    }
    if (tmp.beg == tmp.end) return 0;
    blks[num] = tmp;
    return 1;
  }
}

static inline unsigned free_block_size(free_block_t const *blk)
{
  return blk->end - blk->beg;
}

static unsigned find_free_memory(uint_fast16_t size_in_log2_minus3)
{
  free_block_t free_blocks[2 * (HCD_ATTR_ENDPOINT_MAX - 1)];
  unsigned num_blocks = 1;

  /* Initialize free memory block list */
  free_blocks[0].beg = 64 / 8;
  free_blocks[0].end = (4 << 10) / 8; /* 4KiB / 8 bytes */
  for (int i = 1; i < HCD_ATTR_ENDPOINT_MAX; ++i) {
    uint_fast16_t addr;
    int num;
    USB0->EPIDX = i;
    addr = USB0->TXFIFOADD;
    if (addr) {
      unsigned sz  = USB0->TXFIFOSZ;
      unsigned sft = (sz & USB_TXFIFOSZ_SIZE_M) + ((sz & USB_TXFIFOSZ_DPB) ? 1: 0);
      num = update_free_block_list(free_blocks, num_blocks, addr, 1 << sft);
      TU_ASSERT(-2 < num, 0);
      num_blocks += num;
    }
    addr = USB0->RXFIFOADD;
    if (addr) {
      unsigned sz  = USB0->RXFIFOSZ;
      unsigned sft = (sz & USB_RXFIFOSZ_SIZE_M) + ((sz & USB_RXFIFOSZ_DPB) ? 1: 0);
      num = update_free_block_list(free_blocks, num_blocks, addr, 1 << sft);
      TU_ASSERT(-2 < num, 0);
      num_blocks += num;
    }
  }

  /* Find the best fit memory block */
  uint_fast16_t size_in_8byte_unit = 1 << size_in_log2_minus3;
  free_block_t const *min = NULL;
  uint_fast16_t    min_sz = 0xFFFFu;
  free_block_t const *end = &free_blocks[num_blocks];
  for (free_block_t const *cur = &free_blocks[0]; cur < end; ++cur) {
    uint_fast16_t sz = free_block_size(cur);
    if (sz < size_in_8byte_unit) continue;
    if (size_in_8byte_unit == sz) return cur->beg;
    if (sz < min_sz) min = cur;
  }
  TU_ASSERT(min, 0);
  return min->beg;
}

static inline volatile hw_endpoint_t* edpt_regs(unsigned epnum_minus1)
{
  volatile hw_endpoint_t *regs = (volatile hw_endpoint_t*)((uintptr_t)&USB0->TXMAXP1);
  return regs + epnum_minus1;
}

static unsigned find_pipe(uint_fast8_t dev_addr, uint_fast8_t ep_addr)
{
  unsigned const dir_tx = tu_edpt_dir(ep_addr) ? 0: 1;
  pipe_addr_t const *p = &_hcd.addr[0][dir_tx];
  for (unsigned i = 0; i < sizeof(_hcd.addr)/sizeof(_hcd.addr[0]); ++i, p += 2) {
    if ((dev_addr == p->dev) && (ep_addr == p->ep))
      return i + 1;
  }
  return 0;
}

static void pipe_write_packet(void *buf, volatile void *fifo, unsigned len)
{
  volatile hw_fifo_t *reg = (volatile hw_fifo_t*)fifo;
  uintptr_t addr = (uintptr_t)buf;
  while (len >= 4) {
    reg->u32 = *(uint32_t const *)addr;
    addr += 4;
    len  -= 4;
  }
  if (len >= 2) {
    reg->u16 = *(uint16_t const *)addr;
    addr += 2;
    len  -= 2;
  }
  if (len) {
    reg->u8 = *(uint8_t const *)addr;
  }
}

static void pipe_read_packet(void *buf, volatile void *fifo, unsigned len)
{
  volatile hw_fifo_t *reg = (volatile hw_fifo_t*)fifo;
  uintptr_t addr = (uintptr_t)buf;
  while (len >= 4) {
    *(uint32_t *)addr = reg->u32;
    addr += 4;
    len  -= 4;
  }
  if (len >= 2) {
    *(uint16_t *)addr = reg->u16;
    addr += 2;
    len  -= 2;
  }
  if (len) {
    *(uint8_t *)addr = reg->u8;
  }
}

static bool edpt0_xfer_out(void)
{
  pipe_state_t *pipe = &_hcd.pipe0;
  unsigned const rem = pipe->remaining;
  if (!rem) {
    pipe->buf = NULL;
    return true;
  }
  unsigned const dev_addr = USB0->TXFUNCADDR0;
  unsigned const mps = _hcd.ctl_mps[dev_addr];
  unsigned const len = TU_MIN(rem, mps);
  void          *buf = pipe->buf;
  if (len) {
    pipe_write_packet(buf, &USB0->FIFO0_WORD, len);
    pipe->buf = (uint8_t*)buf + len;
  }
  pipe->remaining = rem - len;
  USB0->CSRL0 = USB_CSRL0_TXRDY;
  return false;
}

static bool edpt0_xfer_in(void)
{
  pipe_state_t *pipe = &_hcd.pipe0;
  unsigned const rem = pipe->remaining;
  unsigned const dev_addr = USB0->TXFUNCADDR0;
  unsigned const mps = _hcd.ctl_mps[dev_addr];
  unsigned const vld = USB0->COUNT0;
  unsigned const len = TU_MIN(TU_MIN(rem, mps), vld);
  void          *buf = pipe->buf;
  if (len) {
    pipe_read_packet(buf, &USB0->FIFO0_WORD, len);
    pipe->buf = (uint8_t*)buf + len;
  }
  pipe->remaining = rem - len;
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return true;
  }
  USB0->CSRL0 = USB_CSRL0_REQPKT;
  return false;
}

static bool edpt0_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
  (void)rhport;

  unsigned const req = _hcd.bmRequestType;
  TU_ASSERT(req != REQUEST_TYPE_INVALID);
  TU_ASSERT(dev_addr < sizeof(_hcd.ctl_mps));

  USB0->TXFUNCADDR0 = dev_addr;
  const unsigned dir_in = tu_edpt_dir(ep_addr);
  if (tu_edpt_dir(req) == dir_in) { /* DATA stage */
    TU_ASSERT(buffer);
    _hcd.pipe0.buf       = buffer;
    _hcd.pipe0.length    = buflen;
    _hcd.pipe0.remaining = buflen;
    if (dir_in)
      USB0->CSRL0 = USB_CSRL0_REQPKT;
    else
      edpt0_xfer_out();
  } else { /* STATUS stage */
    _hcd.pipe0.buf       = NULL;
    _hcd.pipe0.length    = 0;
    _hcd.pipe0.remaining = 0;
    USB0->CSRL0 = USB_CSRL0_STATUS | (dir_in ? USB_CSRL0_REQPKT: USB_CSRL0_TXRDY);
  }
  return true;
}

static bool pipe_xfer_out(uint_fast8_t pipenum)
{
  pipe_state_t *pipe = &_hcd.pipe[pipenum - 1][1];
  unsigned const rem = pipe->remaining;
  if (!rem) {
    pipe->buf = NULL;
    return true;
  }
  hw_endpoint_t volatile *regs = edpt_regs(pipenum - 1);
  unsigned const mps = regs->TXMAXP;
  unsigned const len = TU_MIN(rem, mps);
  void          *buf = pipe->buf;
  if (len) {
    pipe_write_packet(buf, &USB0->FIFO0_WORD + pipenum, len);
    pipe->buf = (uint8_t*)buf + len;
  }
  pipe->remaining = rem - len;
  regs->TXCSRL = USB_TXCSRL1_TXRDY;
  return false;
}

static bool pipe_xfer_in(uint_fast8_t pipenum)
{
  pipe_state_t *pipe = &_hcd.pipe[pipenum - 1][0];
  volatile hw_endpoint_t *regs = edpt_regs(pipenum - 1);

  TU_ASSERT(regs->RXCSRL & USB_RXCSRL1_RXRDY);

  const unsigned mps = regs->RXMAXP;
  const unsigned rem = pipe->remaining;
  const unsigned vld = regs->RXCOUNT;
  const unsigned len = TU_MIN(TU_MIN(rem, mps), vld);
  void          *buf = pipe->buf;
  if (len) {
    pipe_read_packet(buf, &USB0->FIFO0_WORD + pipenum, len);
    pipe->buf       = buf + len;
    pipe->remaining = rem - len;
  }
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return NULL != buf;
  }
  regs->RXCSRL = USB_RXCSRL1_REQPKT;
  return false;
}

static bool edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
  (void)rhport;
  unsigned const pipenum = find_pipe(dev_addr, ep_addr);
  unsigned const dir_tx  = tu_edpt_dir(ep_addr) ? 0: 1;
  pipe_state_t *pipe = &_hcd.pipe[pipenum - 1][dir_tx];
  pipe->buf          = buffer;
  pipe->length       = buflen;
  pipe->remaining    = buflen;
  if (dir_tx) {
    pipe_xfer_out(pipenum);
  } else {
    volatile hw_endpoint_t *regs = edpt_regs(pipenum - 1);
    regs->RXCSRL = USB_RXCSRL1_REQPKT;
  }
  return true;
}

static void process_ep0(uint8_t rhport)
{
  (void)rhport;

  uint_fast8_t csrl = USB0->CSRL0;
  // TU_LOG1(" EP0 CSRL = %x\r\n", csrl);

  unsigned const dev_addr = USB0->TXFUNCADDR0;
  unsigned const req = _hcd.bmRequestType;
  if (csrl & (USB_CSRL0_ERROR | USB_CSRL0_NAKTO | USB_CSRL0_STALLED)) {
    /* No response / NAK timed out / Stall received */
    if (csrl & (USB_CSRL0_RXRDY | USB_CSRL0_TXRDY))
      USB0->CSRH0 = USB_CSRH0_FLUSH;
    USB0->CSRL0 = 0;
    _hcd.bmRequestType = REQUEST_TYPE_INVALID;
    uint8_t result = (csrl & USB_CSRL0_STALLED) ? XFER_RESULT_STALLED: XFER_RESULT_FAILED;
    if (REQUEST_TYPE_INVALID == req) { /* SETUP */
      uint8_t const ep_addr = tu_edpt_addr(0, TUSB_DIR_OUT);
      hcd_event_xfer_complete(dev_addr, ep_addr,
                              _hcd.pipe0.length - _hcd.pipe0.remaining,
                              result, true);
    } else if (csrl & USB_CSRL0_STATUS) { /* STATUS */
      uint8_t const ep_addr = tu_edpt_dir(req) ?
        tu_edpt_addr(0, TUSB_DIR_OUT): tu_edpt_addr(0, TUSB_DIR_IN);
      hcd_event_xfer_complete(dev_addr, ep_addr,
                              _hcd.pipe0.length - _hcd.pipe0.remaining,
                              result, true);
    } else { /* DATA */
      uint8_t const ep_addr = tu_edpt_dir(req) ?
        tu_edpt_addr(0, TUSB_DIR_IN): tu_edpt_addr(0, TUSB_DIR_OUT);
      hcd_event_xfer_complete(dev_addr, ep_addr,
                              _hcd.pipe0.length - _hcd.pipe0.remaining,
                              result, true);
    }
    return;
  }
  if (csrl & USB_CSRL0_STATUS) {
    /* STATUS IN */
    TU_ASSERT(USB_CSRL0_RXRDY == (csrl & USB_CSRL0_RXRDY),);
    TU_ASSERT(0 == USB0->COUNT0,);
    USB0->CSRH0 = USB_CSRH0_FLUSH;
    USB0->CSRL0 = 0;
    _hcd.bmRequestType = REQUEST_TYPE_INVALID;
    hcd_event_xfer_complete(dev_addr, tu_edpt_addr(0, TUSB_DIR_IN),
                            0, XFER_RESULT_SUCCESS, true);
    return;
  }
  if (csrl & USB_CSRL0_RXRDY) {
    /* DATA IN */
    TU_ASSERT(REQUEST_TYPE_INVALID != req,);
    TU_ASSERT(_hcd.pipe0.buf,);
    if (edpt0_xfer_in()) {
      hcd_event_xfer_complete(dev_addr, tu_edpt_addr(0, TUSB_DIR_IN),
                              _hcd.pipe0.length - _hcd.pipe0.remaining,
                              XFER_RESULT_SUCCESS, true);
    }
    return;
  }

  /* When CSRL0 is zero, it means that completion of sending a any length packet. */
  if (!_hcd.pipe0.buf) {
    /* STATUS OUT */
    TU_ASSERT(REQUEST_TYPE_INVALID != req,);
    _hcd.bmRequestType = REQUEST_TYPE_INVALID;
    /* EP address is the reverse direction of DATA stage */
    uint8_t const ep_addr = tu_edpt_dir(req) ?
      tu_edpt_addr(0, TUSB_DIR_OUT): tu_edpt_addr(0, TUSB_DIR_IN);
    hcd_event_xfer_complete(dev_addr, ep_addr, 0, XFER_RESULT_SUCCESS, true);
    return;
  }
  if (REQUEST_TYPE_INVALID == req) {
    /* SETUP */
    _hcd.bmRequestType = *(uint8_t*)_hcd.pipe0.buf;
    _hcd.pipe0.buf = NULL;
    hcd_event_xfer_complete(dev_addr, tu_edpt_addr(0, TUSB_DIR_OUT),
                            8, XFER_RESULT_SUCCESS, true);
    return;
  }

  /* DATA OUT */
  if (edpt0_xfer_out()) {
    hcd_event_xfer_complete(dev_addr, tu_edpt_addr(0, TUSB_DIR_OUT),
                            _hcd.pipe0.length - _hcd.pipe0.remaining,
                            XFER_RESULT_SUCCESS, true);
  }
}

static void process_pipe_tx(uint8_t rhport, uint_fast8_t pipenum)
{
  (void)rhport;
  bool completed;
  uint8_t result;

  volatile hw_endpoint_t *regs = edpt_regs(pipenum - 1);
  unsigned const csrl = regs->TXCSRL;
  // TU_LOG1(" TXCSRL%d = %x\r\n", pipenum, csrl);
  if (csrl & (USB_TXCSRL1_STALLED | USB_TXCSRL1_ERROR)) {
    if (csrl & USB_TXCSRL1_TXRDY)
      regs->TXCSRL = (csrl & ~(USB_TXCSRL1_STALLED | USB_TXCSRL1_ERROR)) | USB_TXCSRL1_FLUSH;
    else
      regs->TXCSRL = csrl & ~(USB_TXCSRL1_STALLED | USB_TXCSRL1_ERROR);
    completed = true;
    result    = (csrl & USB_TXCSRL1_STALLED) ? XFER_RESULT_STALLED: XFER_RESULT_FAILED;
  } else {
    completed = pipe_xfer_out(pipenum);
    result    = XFER_RESULT_SUCCESS;
  }
  if (completed) {
    pipe_addr_t  *addr = &_hcd.addr[pipenum - 1][1];
    pipe_state_t *pipe = &_hcd.pipe[pipenum - 1][1];
    hcd_event_xfer_complete(addr->dev, addr->ep,
                            pipe->length - pipe->remaining,
                            result, true);
  }
}

static void process_pipe_rx(uint8_t rhport, uint_fast8_t pipenum)
{
  (void)rhport;
  bool completed;
  uint8_t result;

  volatile hw_endpoint_t *regs = edpt_regs(pipenum - 1);
  unsigned const csrl = regs->RXCSRL;
  // TU_LOG1(" RXCSRL%d = %x\r\n", pipenum, csrl);
  if (csrl & (USB_RXCSRL1_STALLED | USB_RXCSRL1_ERROR)) {
    if (csrl & USB_RXCSRL1_RXRDY)
      regs->RXCSRL = (csrl & ~(USB_RXCSRL1_STALLED | USB_RXCSRL1_ERROR)) | USB_RXCSRL1_FLUSH;
    else
      regs->RXCSRL = csrl & ~(USB_RXCSRL1_STALLED | USB_RXCSRL1_ERROR);
    completed = true;
    result    = (csrl & USB_RXCSRL1_STALLED) ? XFER_RESULT_STALLED: XFER_RESULT_FAILED;
  } else {
    completed = pipe_xfer_in(pipenum);
    result    = XFER_RESULT_SUCCESS;
  }
  if (completed) {
    pipe_addr_t  *addr = &_hcd.addr[pipenum - 1][0];
    pipe_state_t *pipe = &_hcd.pipe[pipenum - 1][0];
    hcd_event_xfer_complete(addr->dev, addr->ep,
                            pipe->length - pipe->remaining,
                            result, true);
  }
}

/*------------------------------------------------------------------
 * Host API
 *------------------------------------------------------------------*/

bool hcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rhport;
  (void) rh_init;

  NVIC_ClearPendingIRQ(USB0_IRQn);
  _hcd.bmRequestType = REQUEST_TYPE_INVALID;
  USB0->DEVCTL |= USB_DEVCTL_SESSION;
  USB0->IE = USB_IE_DISCON | USB_IE_CONN | USB_IE_BABBLE | USB_IE_RESUME;
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
  return USB0->FRAME;
}

//--------------------------------------------------------------------+
// Port API
//--------------------------------------------------------------------+

bool hcd_port_connect_status(uint8_t rhport)
{
  (void)rhport;
  unsigned devctl = USB0->DEVCTL;
  if (!(devctl & USB_DEVCTL_HOST)) return false;
  if (devctl & (USB_DEVCTL_LSDEV | USB_DEVCTL_FSDEV)) return true;
  return false;
}

void hcd_port_reset(uint8_t rhport)
{
  (void)rhport;
  USB0->POWER |= USB_POWER_HSENAB | USB_POWER_RESET;
  unsigned cnt = SystemCoreClock / 1000 * 20;
  while (cnt--) __NOP();
  USB0->POWER &= ~USB_POWER_RESET;
  _hcd.need_reset = false;
}

void hcd_port_reset_end(uint8_t rhport)
{
  (void) rhport;
}

tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
  (void)rhport;
  unsigned devctl = USB0->DEVCTL;
  if (devctl & USB_DEVCTL_LSDEV)      return TUSB_SPEED_LOW;
  if (!(devctl & USB_DEVCTL_FSDEV))   return TUSB_SPEED_INVALID;
  if (USB0->POWER & USB_POWER_HSMODE) return TUSB_SPEED_HIGH;
  return TUSB_SPEED_FULL;
}

void hcd_device_close(uint8_t rhport, uint8_t dev_addr)
{
  (void)rhport;
  if (sizeof(_hcd.ctl_mps) <= dev_addr) return;

  unsigned const ie = NVIC_GetEnableIRQ(USB0_IRQn);
  NVIC_DisableIRQ(USB0_IRQn);
  _hcd.ctl_mps[dev_addr] = 0;
  if (!dev_addr) return;

  pipe_addr_t *p = &_hcd.addr[0][0];
  for (unsigned i = 0; i < sizeof(_hcd.addr)/sizeof(_hcd.addr[0]); ++i) {
    for (unsigned j = 0; j < 2; ++j, ++p) {
      if (dev_addr != p->dev) continue;
      hw_addr_t volatile     *fadr = (hw_addr_t volatile*)&USB0->TXFUNCADDR0 + i + 1;
      hw_endpoint_t volatile *regs = edpt_regs(i);
      USB0->EPIDX = i + 1;
      if (j) {
        USB0->TXIE      &= ~TU_BIT(i + 1);
        if (regs->TXCSRL & USB_TXCSRL1_TXRDY)
          regs->TXCSRL   = USB_TXCSRL1_CLRDT | USB_TXCSRL1_FLUSH;
        else
          regs->TXCSRL   = USB_TXCSRL1_CLRDT;
        regs->TXMAXP     = 0;
        regs->TXTYPE     = 0;
        regs->TXINTERVAL = 0;
        fadr->TXFUNCADDR = 0;
        fadr->TXHUBADDR  = 0;
        fadr->TXHUBPORT  = 0;
        USB0->TXFIFOADD  = 0;
        USB0->TXFIFOSZ   = 0;
      } else {
        USB0->RXIE      &= ~TU_BIT(i + 1);
        if (regs->RXCSRL & USB_RXCSRL1_RXRDY)
          regs->RXCSRL   = USB_RXCSRL1_CLRDT | USB_RXCSRL1_FLUSH;
        else
          regs->RXCSRL   = USB_RXCSRL1_CLRDT;
        regs->RXMAXP     = 0;
        regs->RXTYPE     = 0;
        regs->RXINTERVAL = 0;
        fadr->RXFUNCADDR = 0;
        fadr->RXHUBADDR  = 0;
        fadr->RXHUBPORT  = 0;
        USB0->RXFIFOADD  = 0;
        USB0->RXFIFOSZ   = 0;
      }
      p->dev = 0;
      p->ep  = 0;
      pipe_state_t *pipe = &_hcd.pipe[i][j];
      pipe->buf       = NULL;
      pipe->length    = 0;
      pipe->remaining = 0;
    }
  }
  if (ie) NVIC_EnableIRQ(USB0_IRQn);
}

//--------------------------------------------------------------------+
// Endpoints API
//--------------------------------------------------------------------+

bool hcd_setup_send(uint8_t rhport, uint8_t dev_addr, uint8_t const setup_packet[8])
{
  (void)rhport;
  pipe_write_packet((void*)(uintptr_t)setup_packet, &USB0->FIFO0_WORD, 8);
  _hcd.pipe0.buf       = (void*)(uintptr_t)setup_packet;
  _hcd.pipe0.length    = 8;
  _hcd.pipe0.remaining = 0;

  hcd_devtree_info_t devtree;
  hcd_devtree_get_info(dev_addr, &devtree);
  switch (devtree.speed) {
    default: return false;
    case TUSB_SPEED_LOW:  USB0->TYPE0 = USB_TYPE0_SPEED_LOW;  break;
    case TUSB_SPEED_FULL: USB0->TYPE0 = USB_TYPE0_SPEED_FULL; break;
    case TUSB_SPEED_HIGH: USB0->TYPE0 = USB_TYPE0_SPEED_HIGH; break;
  }
  USB0->TXHUBADDR0     = devtree.hub_addr;
  USB0->TXHUBPORT0     = devtree.hub_port;
  USB0->TXFUNCADDR0    = dev_addr;
  USB0->CSRL0 = USB_CSRL0_TXRDY | USB_CSRL0_SETUP;
  return true;
}

bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const * ep_desc)
{
  (void)rhport;
  if (sizeof(_hcd.ctl_mps) <= dev_addr) return false;
  unsigned const ep_addr = ep_desc->bEndpointAddress;
  unsigned const epn     = tu_edpt_number(ep_addr);
  if (0 == epn) {
    _hcd.ctl_mps[dev_addr] = ep_desc->wMaxPacketSize;
    return true;
  }

  unsigned const dir_tx = tu_edpt_dir(ep_addr) ? 0: 1;
  /* Find a free pipe */
  unsigned pipenum = 0;
  pipe_addr_t *p = &_hcd.addr[0][dir_tx];
  for (unsigned i = 0; i < sizeof(_hcd.addr)/sizeof(_hcd.addr[0]); ++i, p += 2) {
    if (0 == p->ep) {
      p->dev  = dev_addr;
      p->ep   = ep_addr;
      pipenum = i + 1;
      break;
    }
  }
  if (!pipenum) return false;

  unsigned const xfer = ep_desc->bmAttributes.xfer;
  unsigned const mps  = tu_edpt_packet_size(ep_desc);

  pipe_state_t *pipe = &_hcd.pipe[pipenum - 1][dir_tx];
  pipe->buf       = NULL;
  pipe->length    = 0;
  pipe->remaining = 0;

  uint8_t pipe_type = 0;
  hcd_devtree_info_t devtree;
  hcd_devtree_get_info(dev_addr, &devtree);
  switch (devtree.speed) {
    default: return false;
    case TUSB_SPEED_LOW:  pipe_type |= USB_TXTYPE1_SPEED_LOW;  break;
    case TUSB_SPEED_FULL: pipe_type |= USB_TXTYPE1_SPEED_FULL; break;
    case TUSB_SPEED_HIGH: pipe_type |= USB_TXTYPE1_SPEED_HIGH; break;
  }
  switch (xfer) {
    default: return false;
    case TUSB_XFER_BULK:        pipe_type |= USB_TXTYPE1_PROTO_BULK; break;
    case TUSB_XFER_INTERRUPT:   pipe_type |= USB_TXTYPE1_PROTO_INT;  break;
    case TUSB_XFER_ISOCHRONOUS: pipe_type |= USB_TXTYPE1_PROTO_ISOC; break;
  }

  hw_addr_t volatile     *fadr = (hw_addr_t volatile*)&USB0->TXFUNCADDR0 + pipenum;
  hw_endpoint_t volatile *regs = edpt_regs(pipenum - 1);
  if (dir_tx) {
    fadr->TXFUNCADDR = dev_addr;
    fadr->TXHUBADDR  = devtree.hub_addr;
    fadr->TXHUBPORT  = devtree.hub_port;
    regs->TXMAXP     = mps;
    regs->TXTYPE     = pipe_type | epn;
    regs->TXINTERVAL = ep_desc->bInterval;
    if (regs->TXCSRL & USB_TXCSRL1_TXRDY)
      regs->TXCSRL = USB_TXCSRL1_CLRDT | USB_TXCSRL1_FLUSH;
    else
      regs->TXCSRL = USB_TXCSRL1_CLRDT;
    USB0->TXIE |= TU_BIT(pipenum);
  } else {
    fadr->RXFUNCADDR = dev_addr;
    fadr->RXHUBADDR  = devtree.hub_addr;
    fadr->RXHUBPORT  = devtree.hub_port;
    regs->RXMAXP     = mps;
    regs->RXTYPE     = pipe_type | epn;
    regs->RXINTERVAL = ep_desc->bInterval;
    if (regs->RXCSRL & USB_RXCSRL1_RXRDY)
      regs->RXCSRL = USB_RXCSRL1_CLRDT | USB_RXCSRL1_FLUSH;
    else
      regs->RXCSRL = USB_RXCSRL1_CLRDT;
    USB0->RXIE |= TU_BIT(pipenum);
  }

  /* Setup FIFO */
  int size_in_log2_minus3 = 28 - TU_MIN(28, __CLZ((uint32_t)mps));
  if ((8u << size_in_log2_minus3) < mps) ++size_in_log2_minus3;
  unsigned addr = find_free_memory(size_in_log2_minus3);
  TU_ASSERT(addr);

  USB0->EPIDX = pipenum;
  if (dir_tx) {
    USB0->TXFIFOADD = addr;
    USB0->TXFIFOSZ  = size_in_log2_minus3;
  } else {
    USB0->RXFIFOADD = addr;
    USB0->RXFIFOSZ  = size_in_log2_minus3;
  }
  return true;
}

bool hcd_edpt_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr, uint8_t *buffer, uint16_t buflen)
{
  (void)rhport;
  bool ret = false;
  if (0 == tu_edpt_number(ep_addr)) {
    ret = edpt0_xfer(rhport, dev_addr, ep_addr, buffer, buflen);
  } else {
    ret = edpt_xfer(rhport, dev_addr, ep_addr, buffer, buflen);
  }
  return ret;
}

bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  (void) dev_addr;
  (void) ep_addr;
  // TODO not implemented yet
  return false;
}

// clear stall, data toggle is also reset to DATA0
bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t dev_addr, uint8_t ep_addr) {
  (void) rhport;
  unsigned const pipenum = find_pipe(dev_addr, ep_addr);
  if (!pipenum) return false;
  hw_endpoint_t volatile *regs = edpt_regs(pipenum - 1);
  unsigned const dir_tx = tu_edpt_dir(ep_addr) ? 0: 1;
  if (dir_tx)
    regs->TXCSRL = USB_TXCSRL1_CLRDT;
  else
    regs->RXCSRL = USB_RXCSRL1_CLRDT;
  return true;
}

/*-------------------------------------------------------------------
 * ISR
 *-------------------------------------------------------------------*/
void hcd_int_handler(uint8_t rhport, bool in_isr)
{
  (void) in_isr;

  uint_fast8_t is, txis, rxis;

  is   = USB0->IS;   /* read and clear interrupt status */
  txis = USB0->TXIS; /* read and clear interrupt status */
  rxis = USB0->RXIS; /* read and clear interrupt status */
  // TU_LOG1("D%2x T%2x R%2x\r\n", is, txis, rxis);

  is &= USB0->IE; /* Clear disabled interrupts */
  if (is & USB_IS_RESUME) {
  }
  if (is & USB_IS_CONN) {
    _hcd.need_reset = true;
    hcd_event_device_attach(rhport, true);
  }
  if (is & USB_IS_DISCON) {
    hcd_event_device_remove(rhport, true);
  }
  if (is & USB_IS_BABBLE) {
  }
  txis &= USB0->TXIE; /* Clear disabled interrupts */
  if (txis & USB_TXIE_EP0) {
    process_ep0(rhport);
    txis &= ~TU_BIT(0);
  }
  while (txis) {
    unsigned const num = __builtin_ctz(txis);
    process_pipe_tx(rhport, num);
    txis &= ~TU_BIT(num);
  }
  rxis &= USB0->RXIE; /* Clear disabled interrupts */
  while (rxis) {
    unsigned const num = __builtin_ctz(rxis);
    process_pipe_rx(rhport, num);
    rxis &= ~TU_BIT(num);
  }
}

#endif
