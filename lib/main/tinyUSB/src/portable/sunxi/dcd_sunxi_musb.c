/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Koji KITAYAMA
 * Copyright (c) 2021 Tian Yunhao (t123yh)
 * Copyright (c) 2021 Ha Thach (tinyusb.org)
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

#include <stdint.h>
#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_F1C100S

#include "osal/osal.h"
#include <f1c100s-irq.h>
#include <device/dcd.h>
#include "musb_def.h"

//#include "bsp/board_api.h"
extern uint32_t board_millis(void); // TODO remove

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;


#define REQUEST_TYPE_INVALID  (0xFFu)

typedef struct {
  uint_fast16_t beg; /* offset of including first element */
  uint_fast16_t end; /* offset of excluding the last element */
} free_block_t;

typedef struct TU_ATTR_PACKED
{
  void      *buf;      /* the start address of a transfer data buffer */
  uint16_t  length;    /* the number of bytes in the buffer */
  uint16_t  remaining; /* the number of bytes remaining in the buffer */
} pipe_state_t;

typedef struct
{
  CFG_TUD_MEM_ALIGN tusb_control_request_t setup_packet;
  uint16_t     remaining_ctrl; /* The number of bytes remaining in data stage of control transfer. */
  int8_t       status_out;
  pipe_state_t pipe0;
  pipe_state_t pipe[2][7];   /* pipe[direction][endpoint number - 1] */
  uint16_t     pipe_buf_is_fifo[2]; /* Bitmap. Each bit means whether 1:TU_FIFO or 0:POD. */
} dcd_data_t;

/*------------------------------------------------------------------
 * SUNXI FUNCTION
 *------------------------------------------------------------------*/

static void usb_phy_write(int addr, int data, int len)
{
	int j = 0, usbc_bit = 0;
	void *dest = (void *)USBC_REG_CSR(USBC0_BASE);

	usbc_bit = 1 << (0 * 2);
	for (j = 0; j < len; j++)
	{
		/* set the bit address to be written */
		USBC_ClrBit_Mask_l(dest, 0xff << 8);
		USBC_SetBit_Mask_l(dest, (addr + j) << 8);

		USBC_ClrBit_Mask_l(dest, usbc_bit);
		/* set data bit */
		if (data & 0x1)
			USBC_SetBit_Mask_l(dest, 1 << 7);
		else
			USBC_ClrBit_Mask_l(dest, 1 << 7);

		USBC_SetBit_Mask_l(dest, usbc_bit);

		USBC_ClrBit_Mask_l(dest, usbc_bit);

		data >>= 1;
	}
}

static void delay_ms(uint32_t ms)
{
#if CFG_TUSB_OS == OPT_OS_NONE
  int now = board_millis();
  while (board_millis() - now <= ms) asm("nop");
#else
  osal_task_delay(ms);
#endif
}

static void USBC_HardwareReset(void)
{
  // Reset phy and controller
  USBC_REG_set_bit_l(USBPHY_CLK_RST_BIT, USBPHY_CLK_REG);
	USBC_REG_set_bit_l(BUS_RST_USB_BIT, BUS_CLK_RST_REG);
  delay_ms(2);

	USBC_REG_set_bit_l(USBPHY_CLK_GAT_BIT, USBPHY_CLK_REG);
  USBC_REG_set_bit_l(USBPHY_CLK_RST_BIT, USBPHY_CLK_REG);

	USBC_REG_set_bit_l(BUS_CLK_USB_BIT, BUS_CLK_GATE0_REG);
	USBC_REG_set_bit_l(BUS_RST_USB_BIT, BUS_CLK_RST_REG);
}

static void USBC_PhyConfig(void)
{
	/* Regulation 45 ohms */
	usb_phy_write(0x0c, 0x01, 1);

	/* adjust PHY's magnitude and rate */
	usb_phy_write(0x20, 0x14, 5);

	/* threshold adjustment disconnect */
	usb_phy_write(0x2a, 3, 2);

	return;
}

static void USBC_ConfigFIFO_Base(void)
{
	u32 reg_value;

	/* config usb fifo, 8kb mode */
	reg_value = USBC_Readl(SUNXI_SRAMC_BASE + 0x04);
	reg_value &= ~(0x03 << 0);
	reg_value |= (1 << 0);
	USBC_Writel(reg_value, SUNXI_SRAMC_BASE + 0x04);
}

static unsigned int USBC_WakeUp_ClearChangeDetect(unsigned int reg_val)
{
	unsigned int temp = reg_val;
    /* vbus, id, dpdm, these bit is set 1 to clear, so we clear these bit when operate other bits */
	temp &= ~(1 << USBC_BP_ISCR_VBUS_CHANGE_DETECT);
	temp &= ~(1 << USBC_BP_ISCR_ID_CHANGE_DETECT);
	temp &= ~(1 << USBC_BP_ISCR_DPDM_CHANGE_DETECT);

	return temp;
}

static void USBC_EnableDpDmPullUp(void)
{
	u32 reg_val = USBC_Readl(USBC_REG_ISCR(USBC0_BASE));
	reg_val |= (1 << USBC_BP_ISCR_DPDM_PULLUP_EN);
	reg_val |= 3<<USBC_BP_ISCR_VBUS_VALID_SRC;
	reg_val = USBC_WakeUp_ClearChangeDetect(reg_val);
	USBC_Writel(reg_val, USBC_REG_ISCR(USBC0_BASE));
}

static void USBC_ForceIdToHigh(void)
{
	/* first write 00, then write 10 */
	u32 reg_val = USBC_Readl(USBC_REG_ISCR(USBC0_BASE));
	reg_val |= (0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val = USBC_WakeUp_ClearChangeDetect(reg_val);
	USBC_Writel(reg_val, USBC_REG_ISCR(USBC0_BASE));
}

static void USBC_ForceVbusValidToHigh(void)
{
	/* first write 00, then write 11 */
	u32 reg_val = USBC_Readl(USBC_REG_ISCR(USBC0_BASE));
	reg_val |= (0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val = USBC_WakeUp_ClearChangeDetect(reg_val);
	USBC_Writel(reg_val, USBC_REG_ISCR(USBC0_BASE));
}

void USBC_SelectBus(u32 io_type, u32 ep_type, u32 ep_index)
{
	u32 reg_val = 0;

	reg_val = USBC_Readb(USBC_REG_VEND0(USBC0_BASE));
	if (io_type == USBC_IO_TYPE_DMA) {
		if (ep_type == USBC_EP_TYPE_TX) {
			reg_val |= ((ep_index - 0x01) << 1) << USBC_BP_VEND0_DRQ_SEL;  //drq_sel
			reg_val |= 0x1<<USBC_BP_VEND0_BUS_SEL;   //io_dma
		} else {
			reg_val |= ((ep_index << 1) - 0x01) << USBC_BP_VEND0_DRQ_SEL;
			reg_val |= 0x1<<USBC_BP_VEND0_BUS_SEL;
		}
	} else {
		//reg_val &= ~(0x1 << USBC_BP_VEND0_DRQ_SEL);  //clear drq_sel, select pio
		reg_val &= 0x00;  // clear drq_sel, select pio
	}

	/* in 1667 1673 and later ic, FIFO_BUS_SEL bit(bit24 of reg0x40 for host/device)
	 * is fixed to 1, the hw guarantee that it's ok for cpu/inner_dma/outer_dma transfer */

//	reg_val |= 0x1<<USBC_BP_VEND0_BUS_SEL;  //for 1663 set 1: enable dma, set 0: enable fifo

	USBC_Writeb(reg_val, USBC_REG_VEND0(USBC0_BASE));
}

static void USBC_SelectActiveEp(u8 ep_index)
{
	USBC_Writeb(ep_index, USBC_REG_EPIND(USBC0_BASE));
}

static u8 USBC_GetActiveEp(void)
{
	return USBC_Readb(USBC_REG_EPIND(USBC0_BASE));
}

static void __USBC_Dev_ep0_SendStall(void)
{
	USBC_REG_set_bit_w(USBC_BP_CSR0_D_SEND_STALL, USBC_REG_CSR0(USBC0_BASE));
}

static void __USBC_Dev_ep0_ClearStall(void)
{
	USBC_REG_clear_bit_w(USBC_BP_CSR0_D_SEND_STALL, USBC_REG_CSR0(USBC0_BASE));
	USBC_REG_clear_bit_w(USBC_BP_CSR0_D_SENT_STALL, USBC_REG_CSR0(USBC0_BASE));
}

static void USBC_Dev_Ctrl_ClearSetupEnd(void)
{
	USBC_REG_set_bit_w(USBC_BP_CSR0_D_SERVICED_SETUP_END, USBC_REG_CSR0(USBC0_BASE));
}

static void USBC_Dev_SetAddress(u8 address)
{
	USBC_Writeb(address, USBC_REG_FADDR(USBC0_BASE));
}

static void __USBC_Dev_Tx_SendStall(void)
{
	//send stall, and fifo is flushed automatically
	USBC_REG_set_bit_w(USBC_BP_TXCSR_D_SEND_STALL, USBC_REG_TXCSR(USBC0_BASE));
}
static u32 __USBC_Dev_Tx_IsEpStall(void)
{
	return USBC_REG_test_bit_w(USBC_BP_TXCSR_D_SENT_STALL, USBC_REG_TXCSR(USBC0_BASE));
}
static void __USBC_Dev_Tx_ClearStall(void)
{
	u32 reg_val = USBC_Readw(USBC_REG_TXCSR(USBC0_BASE));
	reg_val &= ~((1 << USBC_BP_TXCSR_D_SENT_STALL)|(1 << USBC_BP_TXCSR_D_SEND_STALL)|(1<<USBC_BP_TXCSR_D_UNDER_RUN));
  reg_val |= (1 << USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE);
	USBC_Writew(reg_val, USBC_REG_TXCSR(USBC0_BASE));
}

static void __USBC_Dev_Rx_SendStall(void)
{
	USBC_REG_set_bit_w(USBC_BP_RXCSR_D_SEND_STALL, USBC_REG_RXCSR(USBC0_BASE));
}

static u32 __USBC_Dev_Rx_IsEpStall(void)
{
	return USBC_REG_test_bit_w(USBC_BP_RXCSR_D_SENT_STALL, USBC_REG_RXCSR(USBC0_BASE));
}

static void __USBC_Dev_Rx_ClearStall(void)
{
	u32 reg_val = USBC_Readw(USBC_REG_RXCSR(USBC0_BASE));
	reg_val &= ~((1 << USBC_BP_RXCSR_D_SENT_STALL)|(1 << USBC_BP_RXCSR_D_SEND_STALL)|(1<<USBC_BP_RXCSR_D_OVERRUN));
  reg_val |= (1 << USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE);
	USBC_Writew(reg_val, USBC_REG_RXCSR(USBC0_BASE));
}

static tusb_speed_t USBC_Dev_QueryTransferMode(void)
{
	if (USBC_REG_test_bit_b(USBC_BP_POWER_D_HIGH_SPEED_FLAG, USBC_REG_PCTL(USBC0_BASE)))
		return TUSB_SPEED_HIGH;
  else
		return TUSB_SPEED_FULL;
}

static void __USBC_Dev_ep0_ReadDataHalf(void)
{
	USBC_Writew(1<<USBC_BP_CSR0_D_SERVICED_RX_PKT_READY, USBC_REG_CSR0(USBC0_BASE));
}

static void __USBC_Dev_ep0_ReadDataComplete(void)
{
	USBC_Writew((1<<USBC_BP_CSR0_D_SERVICED_RX_PKT_READY) | (1<<USBC_BP_CSR0_D_DATA_END),
	USBC_REG_CSR0(USBC0_BASE));
}


static void __USBC_Dev_ep0_WriteDataHalf(void)
{
	USBC_Writew(1<<USBC_BP_CSR0_D_TX_PKT_READY, USBC_REG_CSR0(USBC0_BASE));
}

static void __USBC_Dev_ep0_WriteDataComplete(void)
{
	USBC_Writew((1<<USBC_BP_CSR0_D_TX_PKT_READY) | (1<<USBC_BP_CSR0_D_DATA_END),
	USBC_REG_CSR0(USBC0_BASE));
}

static void __USBC_Dev_Tx_WriteDataComplete(void)
{
	USBC_Writeb((1 << USBC_BP_TXCSR_D_TX_READY), USBC_REG_TXCSR(USBC0_BASE));
}

static void __USBC_Dev_Rx_ReadDataComplete(void)
{
	USBC_Writeb(0, USBC_REG_RXCSR(USBC0_BASE));
}

static u32 __USBC_Dev_Rx_IsReadDataReady(void)
{
	return USBC_REG_test_bit_w(USBC_BP_RXCSR_D_RX_PKT_READY, USBC_REG_RXCSR(USBC0_BASE));
}

/* open a tx ep's interrupt */
static void USBC_INT_EnableTxEp(u8 ep_index)
{
	USBC_REG_set_bit_w(ep_index, USBC_REG_INTTxE(USBC0_BASE));
}

/* open a rx ep's interrupt */
static void USBC_INT_EnableRxEp(u8 ep_index)
{
	USBC_REG_set_bit_w(ep_index, USBC_REG_INTRxE(USBC0_BASE));
}

/* close a tx ep's interrupt */
static void USBC_INT_DisableTxEp(u8 ep_index)
{
	USBC_REG_clear_bit_w(ep_index, USBC_REG_INTTxE(USBC0_BASE));
}

/* close a rx ep's interrupt */
static void USBC_INT_DisableRxEp(u8 ep_index)
{
	USBC_REG_clear_bit_w(ep_index, USBC_REG_INTRxE(USBC0_BASE));
}

/*------------------------------------------------------------------
 * INTERNAL FUNCTION DECLARATION
 *------------------------------------------------------------------*/

CFG_TUD_MEM_ALIGN static dcd_data_t _dcd;

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

#if 0
static inline void print_block_list(free_block_t const *blk, unsigned num)
{
  TU_LOG1("*************\r\n");
  for (unsigned i = 0; i < num; ++i) {
    TU_LOG1(" Blk%u %u %u\r\n", i, blk->beg, blk->end);
    ++blk;
  }
}
#else
#define print_block_list(a,b)
#endif

#if CFG_TUSB_MCU == OPT_MCU_F1C100S
#define USB_FIFO_SIZE_KB 4
#else
#error "Unsupported MCU"
#endif

static unsigned find_free_memory(uint_fast16_t size_in_log2_minus3)
{
  free_block_t free_blocks[2 * (TUP_DCD_ENDPOINT_MAX - 1)];
  unsigned num_blocks = 1;
  /* Backup current EP to restore later */
  u8 backup_ep = USBC_GetActiveEp();

  /* Initialize free memory block list */
  free_blocks[0].beg = 64 / 8;
  free_blocks[0].end = (USB_FIFO_SIZE_KB << 10) / 8; /* 2KiB / 8 bytes */
  for (int i = 1; i < TUP_DCD_ENDPOINT_MAX; ++i) {
    uint_fast16_t addr;
    int num;
    USBC_SelectActiveEp(i);
    addr = USBC_Readw(USBC_REG_TXFIFOAD(USBC0_BASE));
    if (addr) {
      unsigned sz  = USBC_Readb(USBC_REG_TXFIFOSZ(USBC0_BASE));
      unsigned sft = (sz & USB_TXFIFOSZ_SIZE_M) + ((sz & USB_TXFIFOSZ_DPB) ? 1: 0);
      num = update_free_block_list(free_blocks, num_blocks, addr, 1 << sft);
      TU_ASSERT(-2 < num, 0);
      num_blocks += num;
      print_block_list(free_blocks, num_blocks);
    }
    addr = USBC_Readw(USBC_REG_RXFIFOAD(USBC0_BASE));
    if (addr) {
      unsigned sz  = USBC_Readb(USBC_REG_RXFIFOSZ(USBC0_BASE));
      unsigned sft = (sz & USB_RXFIFOSZ_SIZE_M) + ((sz & USB_RXFIFOSZ_DPB) ? 1: 0);
      num = update_free_block_list(free_blocks, num_blocks, addr, 1 << sft);
      TU_ASSERT(-2 < num, 0);
      num_blocks += num;
      print_block_list(free_blocks, num_blocks);
    }
  }
  print_block_list(free_blocks, num_blocks);

  USBC_SelectActiveEp(backup_ep);

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

static void pipe_write_packet(void *buff, volatile void *fifo, unsigned cnt)
{
 	u32 len = 0;
	u32 i32 = 0;
	u32 i8  = 0;
	u8  *buf8  = 0;
	u32 *buf32 = 0;

	//--<1>-- adjust data
	buf32 = buff;
	len   = cnt;

	i32 = len >> 2;
	i8  = len & 0x03;

	//--<2>-- deal with 4byte part
	while (i32--) {
		USBC_Writel(*buf32++, fifo);
	}

	//--<3>-- deal with no 4byte part
	buf8 = (u8 *)buf32;
	while (i8--) {
		USBC_Writeb(*buf8++, fifo);
	}
}

static void pipe_read_packet(void *buff, volatile void *fifo, unsigned cnt)
{
	u32 len = 0;
	u32 i32 = 0;
	u32 i8  = 0;
	u8  *buf8  = 0;
	u32 *buf32 = 0;

	//--<1>-- adjust data
	buf32 = buff;
	len   = cnt;

	i32 = len >> 2;
	i8  = len & 0x03;

	//--<2>-- deal with 4byte part
	while (i32--) {
		*buf32++ = USBC_Readl(fifo);
	}

	//--<3>-- deal with no 4byte part
	buf8 = (u8 *)buf32;
	while (i8--) {
		*buf8++ = USBC_Readb(fifo);
	}
}

static void pipe_read_write_packet_ff(tu_fifo_t *f, volatile void *fifo, unsigned len, unsigned dir)
{
  static const struct {
    void (*tu_fifo_get_info)(tu_fifo_t *f, tu_fifo_buffer_info_t *info);
    void (*tu_fifo_advance)(tu_fifo_t *f, uint16_t n);
    void (*pipe_read_write)(void *buf, volatile void *fifo, unsigned len);
  } ops[] = {
    /* OUT */ {tu_fifo_get_write_info,tu_fifo_advance_write_pointer,pipe_read_packet},
    /* IN  */ {tu_fifo_get_read_info, tu_fifo_advance_read_pointer, pipe_write_packet},
  };
  tu_fifo_buffer_info_t info;
  ops[dir].tu_fifo_get_info(f, &info);
  unsigned total_len = len;
  len = TU_MIN(total_len, info.len_lin);
  ops[dir].pipe_read_write(info.ptr_lin, fifo, len);
  unsigned rem = total_len - len;
  if (rem) {
    len = TU_MIN(rem, info.len_wrap);
    ops[dir].pipe_read_write(info.ptr_wrap, fifo, len);
    rem -= len;
  }
  ops[dir].tu_fifo_advance(f, total_len - rem);
}

/*------------------------------------------------------------------
 * TRANSFER FUNCTION DECLARATION
 *------------------------------------------------------------------*/

static void process_setup_packet(uint8_t rhport)
{
  uint32_t *p = (uint32_t*)(uintptr_t) &_dcd.setup_packet;
  p[0]        = USBC_Readl(USBC_REG_EPFIFO0(USBC0_BASE));
  p[1]        = USBC_Readl(USBC_REG_EPFIFO0(USBC0_BASE));

  _dcd.pipe0.buf       = NULL;
  _dcd.pipe0.length    = 0;
  _dcd.pipe0.remaining = 0;
  dcd_event_setup_received(rhport, (const uint8_t*)(uintptr_t)&_dcd.setup_packet, true);

  const unsigned len    = _dcd.setup_packet.wLength;
  _dcd.remaining_ctrl   = len;
  const unsigned dir_in = tu_edpt_dir(_dcd.setup_packet.bmRequestType);
  /* Clear RX FIFO and reverse the transaction direction */
  if (len && dir_in) __USBC_Dev_ep0_ReadDataHalf();
}

static bool handle_xfer_in(uint_fast8_t ep_addr)
{
  unsigned epnum_minus1 = tu_edpt_number(ep_addr) - 1;
  pipe_state_t  *pipe = &_dcd.pipe[tu_edpt_dir(ep_addr)][epnum_minus1];
  const unsigned rem  = pipe->remaining;

  if (!rem) {
    pipe->buf = NULL;
    return true;
  }

  const unsigned mps = USBC_Readw(USBC_REG_TXMAXP(USBC0_BASE));
  const unsigned len = TU_MIN(mps, rem);
  uint8_t          *buf = pipe->buf;
  // TU_LOG1("   %p mps %d len %d rem %d\r\n", buf, mps, len, rem);
  if (len) {
    volatile void* addr = (volatile void*)(USBC_REG_EPFIFO1(USBC0_BASE) + (epnum_minus1 << 2));
    if (_dcd.pipe_buf_is_fifo[TUSB_DIR_IN] & TU_BIT(epnum_minus1)) {
      pipe_read_write_packet_ff((tu_fifo_t *)(uintptr_t) buf, addr, len, TUSB_DIR_IN);
    } else {
      pipe_write_packet(buf, addr, len);
      pipe->buf       = buf + len;
    }
    pipe->remaining = rem - len;
  }
  __USBC_Dev_Tx_WriteDataComplete();
  // TU_LOG1(" TXCSRL%d = %x %d\r\n", epnum_minus1 + 1, regs->TXCSRL, rem - len);
  return false;
}

static bool handle_xfer_out(uint_fast8_t ep_addr)
{
  unsigned epnum_minus1 = tu_edpt_number(ep_addr) - 1;
  pipe_state_t  *pipe = &_dcd.pipe[tu_edpt_dir(ep_addr)][epnum_minus1];
  // TU_LOG1(" RXCSRL%d = %x\r\n", epnum_minus1 + 1, regs->RXCSRL);

  TU_ASSERT(__USBC_Dev_Rx_IsReadDataReady());

  const unsigned mps = USBC_Readw(USBC_REG_RXMAXP(USBC0_BASE));
  const unsigned rem = pipe->remaining;
  const unsigned vld = USBC_Readw(USBC_REG_RXCOUNT(USBC0_BASE));
  const unsigned len = TU_MIN(TU_MIN(rem, mps), vld);
  uint8_t          *buf = pipe->buf;
  if (len) {
    volatile void* addr = (volatile void*)(USBC_REG_EPFIFO1(USBC0_BASE) + (epnum_minus1 << 2));
    if (_dcd.pipe_buf_is_fifo[TUSB_DIR_OUT] & TU_BIT(epnum_minus1)) {
      pipe_read_write_packet_ff((tu_fifo_t *)(uintptr_t )buf, addr, len, TUSB_DIR_OUT);
    } else {
      pipe_read_packet(buf, addr, len);
      pipe->buf       = buf + len;
    }
    pipe->remaining = rem - len;
  }
  if ((len < mps) || (rem == len)) {
    pipe->buf = NULL;
    return NULL != buf;
  }
  __USBC_Dev_Rx_ReadDataComplete();
  return false;
}

static bool edpt_n_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void)rhport;

  unsigned epnum_minus1 = tu_edpt_number(ep_addr) - 1;
  unsigned dir_in       = tu_edpt_dir(ep_addr);

  pipe_state_t *pipe = &_dcd.pipe[dir_in][epnum_minus1];
  pipe->buf          = buffer;
  pipe->length       = total_bytes;
  pipe->remaining    = total_bytes;

  USBC_SelectActiveEp(tu_edpt_number(ep_addr));

  if (dir_in) {
    handle_xfer_in(ep_addr);
  } else {
    if (__USBC_Dev_Rx_IsReadDataReady())
      __USBC_Dev_Rx_ReadDataComplete();
  }
  return true;
}

static bool edpt0_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void)rhport;
  TU_ASSERT(total_bytes <= 64); /* Current implementation supports for only up to 64 bytes. */

  const unsigned req = _dcd.setup_packet.bmRequestType;
  TU_ASSERT(req != REQUEST_TYPE_INVALID || total_bytes == 0);

  USBC_SelectActiveEp(0);

  if (req == REQUEST_TYPE_INVALID || _dcd.status_out) {
    /* STATUS OUT stage.
     * MUSB controller automatically handles STATUS OUT packets without
     * software helps. We do not have to do anything. And STATUS stage
     * may have already finished and received the next setup packet
     * without calling this function, so we have no choice but to
     * invoke the callback function of status packet here. */
    // TU_LOG1(" STATUS OUT CSRL0 = %x\r\n", CSRL0);
    _dcd.status_out = 0;
    if (req == REQUEST_TYPE_INVALID) {
      dcd_event_xfer_complete(rhport, ep_addr, total_bytes, XFER_RESULT_SUCCESS, false);
    } else {
      /* The next setup packet has already been received, it aborts
       * invoking callback function to avoid confusing TUSB stack. */
      TU_LOG1("Drop CONTROL_STAGE_ACK\r\n");
    }
    return true;
  }
  const unsigned dir_in = tu_edpt_dir(ep_addr);
  if (tu_edpt_dir(req) == dir_in) { /* DATA stage */
    TU_ASSERT(total_bytes <= _dcd.remaining_ctrl);
    const unsigned rem = _dcd.remaining_ctrl;
    const unsigned len = TU_MIN(TU_MIN(rem, 64), total_bytes);
    if (dir_in) {
      pipe_write_packet(buffer, (volatile void*) USBC_REG_EPFIFO0(USBC0_BASE), len);

      _dcd.pipe0.buf       = buffer + len;
      _dcd.pipe0.length    = len;
      _dcd.pipe0.remaining = 0;

      _dcd.remaining_ctrl  = rem - len;
      if ((len < 64) || (rem == len)) {
        _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID; /* Change to STATUS/SETUP stage */
        _dcd.status_out = 1;
        /* Flush TX FIFO and reverse the transaction direction. */
        __USBC_Dev_ep0_WriteDataComplete();
      } else {
        __USBC_Dev_ep0_WriteDataHalf();
      }
      // TU_LOG1(" IN CSRL0 = %x\r\n", CSRL0);
    } else {
      // TU_LOG1(" OUT CSRL0 = %x\r\n", CSRL0);
      _dcd.pipe0.buf       = buffer;
      _dcd.pipe0.length    = len;
      _dcd.pipe0.remaining = len;
      __USBC_Dev_ep0_ReadDataHalf();
    }
  } else if (dir_in) {
    // TU_LOG1(" STATUS IN CSRL0 = %x\r\n", CSRL0);
    _dcd.pipe0.buf = NULL;
    _dcd.pipe0.length    = 0;
    _dcd.pipe0.remaining = 0;
    /* Clear RX FIFO and reverse the transaction direction */
    __USBC_Dev_ep0_ReadDataComplete();
  }
  return true;
}

static void process_ep0(uint8_t rhport)
{
  USBC_SelectActiveEp(0);
  uint_fast8_t csrl = USBC_Readw(USBC_REG_CSR0(USBC0_BASE));

  // TU_LOG1(" EP0 CSRL0 = %x\r\n", csrl);

  if (csrl & USB_CSRL0_STALLED) {
    /* Returned STALL packet to HOST. */
    __USBC_Dev_ep0_ClearStall();
    return;
  }

  unsigned req = _dcd.setup_packet.bmRequestType;
  if (csrl & USB_CSRL0_SETEND) {
    // TU_LOG1("   ABORT by the next packets\r\n");
    USBC_Dev_Ctrl_ClearSetupEnd();
    if (req != REQUEST_TYPE_INVALID && _dcd.pipe0.buf) {
      /* DATA stage was aborted by receiving STATUS or SETUP packet. */
      _dcd.pipe0.buf = NULL;
      _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
      dcd_event_xfer_complete(rhport,
                              req & TUSB_DIR_IN_MASK,
                              _dcd.pipe0.length - _dcd.pipe0.remaining,
                              XFER_RESULT_SUCCESS, true);
    }
    req = REQUEST_TYPE_INVALID;
    if (!(csrl & USB_CSRL0_RXRDY)) return; /* Received SETUP packet */
  }

  if (csrl & USB_CSRL0_RXRDY) {
    /* Received SETUP or DATA OUT packet */
    if (req == REQUEST_TYPE_INVALID) {
      /* SETUP */
      TU_ASSERT(sizeof(tusb_control_request_t) == USBC_Readw(USBC_REG_COUNT0(USBC0_BASE)),);
      process_setup_packet(rhport);
      return;
    }
    if (_dcd.pipe0.buf) {
      /* DATA OUT */
      const unsigned vld = USBC_Readw(USBC_REG_COUNT0(USBC0_BASE));
      const unsigned rem = _dcd.pipe0.remaining;
      const unsigned len = TU_MIN(TU_MIN(rem, 64), vld);
      pipe_read_packet(_dcd.pipe0.buf, (volatile void*)USBC_REG_EPFIFO0(USBC0_BASE), len);

      _dcd.pipe0.remaining = rem - len;
      _dcd.remaining_ctrl -= len;

      _dcd.pipe0.buf = NULL;
      dcd_event_xfer_complete(rhport,
                              tu_edpt_addr(0, TUSB_DIR_OUT),
                              _dcd.pipe0.length - _dcd.pipe0.remaining,
                              XFER_RESULT_SUCCESS, true);
    }
    return;
  }

  /* When CSRL0 is zero, it means that completion of sending a any length packet
   * or receiving a zero length packet. */
  if (req != REQUEST_TYPE_INVALID && !tu_edpt_dir(req)) {
    /* STATUS IN */
    if (*(const uint16_t*)(uintptr_t)&_dcd.setup_packet == 0x0500) {
      /* The address must be changed on completion of the control transfer. */
	  USBC_Dev_SetAddress((uint8_t)_dcd.setup_packet.wValue);
    }
    _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
    dcd_event_xfer_complete(rhport,
                            tu_edpt_addr(0, TUSB_DIR_IN),
                            _dcd.pipe0.length - _dcd.pipe0.remaining,
                            XFER_RESULT_SUCCESS, true);
    return;
  }
  if (_dcd.pipe0.buf) {
    /* DATA IN */
    _dcd.pipe0.buf = NULL;
    dcd_event_xfer_complete(rhport,
                            tu_edpt_addr(0, TUSB_DIR_IN),
                            _dcd.pipe0.length - _dcd.pipe0.remaining,
                            XFER_RESULT_SUCCESS, true);
  }
}

static void process_edpt_n(uint8_t rhport, uint_fast8_t ep_addr)
{
  bool completed;
  const unsigned dir_in     = tu_edpt_dir(ep_addr);
  const unsigned epn        = tu_edpt_number(ep_addr);

  USBC_SelectActiveEp(epn);

  if (dir_in) {
    // TU_LOG1(" TXCSRL%d = %x\r\n", epn_minus1 + 1, regs->TXCSRL);
    if (__USBC_Dev_Tx_IsEpStall()) {
	  __USBC_Dev_Tx_ClearStall();
      return;
    }
    completed = handle_xfer_in(ep_addr);
  } else {
    // TU_LOG1(" RXCSRL%d = %x\r\n", epn_minus1 + 1, regs->RXCSRL);
    if (__USBC_Dev_Rx_IsEpStall()) {
	    __USBC_Dev_Rx_ClearStall();
      return;
    }
    completed = handle_xfer_out(ep_addr);
  }

  if (completed) {
    pipe_state_t *pipe = &_dcd.pipe[dir_in][tu_edpt_number(ep_addr) - 1];
    dcd_event_xfer_complete(rhport, ep_addr,
                            pipe->length - pipe->remaining,
                            XFER_RESULT_SUCCESS, true);
  }
}

static void process_bus_reset(uint8_t rhport)
{
  /* When bmRequestType is REQUEST_TYPE_INVALID(0xFF),
   * a control transfer state is SETUP or STATUS stage. */
  _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
  _dcd.status_out = 0;
  /* When pipe0.buf has not NULL, DATA stage works in progress. */
  _dcd.pipe0.buf = NULL;

  USBC_Writew(1, USBC_REG_INTTxE(USBC0_BASE)); /* Enable only EP0 */
  USBC_Writew(0, USBC_REG_INTRxE(USBC0_BASE));

  dcd_event_bus_reset(rhport, USBC_Dev_QueryTransferMode(), true);
}

/*------------------------------------------------------------------
 * Device API
 *------------------------------------------------------------------*/

static void usb_isr_handler(void) {
	dcd_int_handler(0);
}

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;

  dcd_disconnect(rhport);
  USBC_HardwareReset();
  USBC_PhyConfig();
  USBC_ConfigFIFO_Base();
  USBC_EnableDpDmPullUp();
  USBC_ForceIdToHigh(); // Force device mode
  USBC_ForceVbusValidToHigh();
  USBC_SelectBus(USBC_IO_TYPE_PIO, 0, 0);
  dcd_edpt_close_all(rhport);

  #if TUD_OPT_HIGH_SPEED
    USBC_REG_set_bit_b(USBC_BP_POWER_D_HIGH_SPEED_EN, USBC_REG_PCTL(USBC0_BASE));
  #else
    USBC_REG_clear_bit_b(USBC_BP_POWER_D_HIGH_SPEED_EN, USBC_REG_PCTL(USBC0_BASE));
  #endif

  USBC_Writeb((1 << USBC_BP_INTUSBE_EN_SUSPEND)
    | (1 << USBC_BP_INTUSBE_EN_RESUME)
    | (1 << USBC_BP_INTUSBE_EN_RESET)
    | (1 << USBC_BP_INTUSBE_EN_SOF)
    | (1 << USBC_BP_INTUSBE_EN_DISCONNECT)
    , USBC_REG_INTUSBE(USBC0_BASE));
  f1c100s_intc_clear_pend(F1C100S_IRQ_USBOTG);
  f1c100s_intc_set_isr(F1C100S_IRQ_USBOTG, usb_isr_handler);

  dcd_connect(rhport);

  return true;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void)rhport;
  USBC_REG_set_bit_b(USBC_BP_POWER_D_SOFT_CONNECT, USBC_REG_PCTL(USBC0_BASE));
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void)rhport;
  USBC_REG_clear_bit_b(USBC_BP_POWER_D_SOFT_CONNECT, USBC_REG_PCTL(USBC0_BASE));
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

void dcd_int_enable(uint8_t rhport)
{
  (void)rhport;
  f1c100s_intc_enable_irq(F1C100S_IRQ_USBOTG);
}

static void musb_int_mask(void)
{
  f1c100s_intc_mask_irq(F1C100S_IRQ_USBOTG);
}

void dcd_int_disable(uint8_t rhport)
{
  (void)rhport;
  f1c100s_intc_disable_irq(F1C100S_IRQ_USBOTG);
}

static void musb_int_unmask(void)
{
  f1c100s_intc_unmask_irq(F1C100S_IRQ_USBOTG);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void)rhport;
  (void)dev_addr;
  _dcd.pipe0.buf       = NULL;
  _dcd.pipe0.length    = 0;
  _dcd.pipe0.remaining = 0;
  /* Clear RX FIFO to return ACK. */
  USBC_SelectActiveEp(0);
  __USBC_Dev_ep0_ReadDataComplete();
}

// Wake up host
void dcd_remote_wakeup(uint8_t rhport)
{
  (void)rhport;
  USBC_REG_set_bit_b(USBC_BP_POWER_D_RESUME, USBC_REG_PCTL(USBC0_BASE));
  delay_ms(10);
  USBC_REG_clear_bit_b(USBC_BP_POWER_D_RESUME, USBC_REG_PCTL(USBC0_BASE));
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

#ifndef __ARMCC_VERSION
#define __clz __builtin_clz
#endif

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;

  uint16_t reg_val;

  const unsigned ep_addr = ep_desc->bEndpointAddress;
  const unsigned epn     = tu_edpt_number(ep_addr);
  const unsigned dir_in  = tu_edpt_dir(ep_addr);
  const unsigned xfer    = ep_desc->bmAttributes.xfer;
  const unsigned mps     = tu_edpt_packet_size(ep_desc);

  TU_ASSERT(epn < TUP_DCD_ENDPOINT_MAX);

  pipe_state_t *pipe = &_dcd.pipe[dir_in][epn - 1];
  pipe->buf       = NULL;
  pipe->length    = 0;
  pipe->remaining = 0;

  musb_int_mask();

  // volatile hw_endpoint_t *regs = edpt_regs(epn - 1);
  USBC_SelectActiveEp(epn);
  if (dir_in) {
    USBC_Writew(mps, USBC_REG_TXMAXP(USBC0_BASE));

    reg_val = (1 << USBC_BP_TXCSR_D_MODE)
      | (1 << USBC_BP_TXCSR_D_FLUSH_FIFO)
      | (1 << USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE);
    if  (xfer == TUSB_XFER_ISOCHRONOUS)
      reg_val |= (1 << USBC_BP_TXCSR_D_ISO);
	  USBC_Writew(reg_val, USBC_REG_TXCSR(USBC0_BASE));

    USBC_INT_EnableTxEp(epn);
  } else {
    USBC_Writew(mps, USBC_REG_RXMAXP(USBC0_BASE));

    reg_val = (1 << USBC_BP_RXCSR_D_FLUSH_FIFO)
      | (1 << USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE);
    if  (xfer == TUSB_XFER_ISOCHRONOUS)
      reg_val |= (1 << USBC_BP_RXCSR_D_ISO);
    USBC_Writew(reg_val, USBC_REG_RXCSR(USBC0_BASE));

    USBC_INT_EnableRxEp(epn);
  }

  /* Setup FIFO */
  int size_in_log2_minus3 = 28 - TU_MIN(28, __clz((uint32_t)mps));
  if ((8u << size_in_log2_minus3) < mps) ++size_in_log2_minus3;
  unsigned addr = find_free_memory(size_in_log2_minus3);
  TU_ASSERT(addr);

  if (dir_in) {
    USBC_Writew(addr, USBC_REG_TXFIFOAD(USBC0_BASE));
    USBC_Writeb(size_in_log2_minus3, USBC_REG_TXFIFOSZ(USBC0_BASE));
  } else {
    USBC_Writew(addr, USBC_REG_RXFIFOAD(USBC0_BASE));
    USBC_Writeb(size_in_log2_minus3, USBC_REG_RXFIFOSZ(USBC0_BASE));
  }

  musb_int_unmask();

  return true;
}

void dcd_edpt_close_all(uint8_t rhport)
{
  (void) rhport;
  musb_int_mask();
  USBC_Writew(1, USBC_REG_INTTxE(USBC0_BASE)); /* Enable only EP0 */
  USBC_Writew(0, USBC_REG_INTRxE(USBC0_BASE));
  for (unsigned i = 1; i < TUP_DCD_ENDPOINT_MAX; ++i) {
    USBC_SelectActiveEp(i);
    USBC_Writew(0, USBC_REG_TXMAXP(USBC0_BASE));
		USBC_Writew((1 << USBC_BP_TXCSR_D_MODE) | (1 << USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE) | (1 << USBC_BP_TXCSR_D_FLUSH_FIFO),
      USBC_REG_TXCSR(USBC0_BASE));

    USBC_Writew(0, USBC_REG_RXMAXP(USBC0_BASE));
	  USBC_Writew((1 << USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE) | (1 << USBC_BP_RXCSR_D_FLUSH_FIFO),
      USBC_REG_RXCSR(USBC0_BASE));

    USBC_Writew(0, USBC_REG_TXFIFOAD(USBC0_BASE));
    USBC_Writeb(0, USBC_REG_TXFIFOSZ(USBC0_BASE));
    USBC_Writew(0, USBC_REG_RXFIFOAD(USBC0_BASE));
    USBC_Writeb(0, USBC_REG_RXFIFOSZ(USBC0_BASE));
  }
  musb_int_unmask();
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;
  unsigned const epn    = tu_edpt_number(ep_addr);
  unsigned const dir_in = tu_edpt_dir(ep_addr);

  musb_int_mask();
  USBC_SelectActiveEp(epn);
  if (dir_in) {
    USBC_INT_DisableTxEp(epn);
    USBC_Writew(0, USBC_REG_TXMAXP(USBC0_BASE));
		USBC_Writew((1 << USBC_BP_TXCSR_D_MODE) | (1 << USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE) | (1 << USBC_BP_TXCSR_D_FLUSH_FIFO),
      USBC_REG_TXCSR(USBC0_BASE));

    USBC_Writew(0, USBC_REG_TXFIFOAD(USBC0_BASE));
    USBC_Writeb(0, USBC_REG_TXFIFOSZ(USBC0_BASE));
  } else {
    USBC_INT_DisableRxEp(epn);
    USBC_Writew(0, USBC_REG_RXMAXP(USBC0_BASE));
	  USBC_Writew((1 << USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE) | (1 << USBC_BP_RXCSR_D_FLUSH_FIFO),
      USBC_REG_RXCSR(USBC0_BASE));

    USBC_Writew(0, USBC_REG_RXFIFOAD(USBC0_BASE));
    USBC_Writeb(0, USBC_REG_RXFIFOSZ(USBC0_BASE));
  }
  musb_int_unmask();
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void)rhport;
  bool ret;
  // TU_LOG1("X %x %d\r\n", ep_addr, total_bytes);
  unsigned const epnum = tu_edpt_number(ep_addr);
  musb_int_mask();

  if (epnum) {
    _dcd.pipe_buf_is_fifo[tu_edpt_dir(ep_addr)] &= ~TU_BIT(epnum - 1);
    ret = edpt_n_xfer(rhport, ep_addr, buffer, total_bytes);
  } else {
    ret = edpt0_xfer(rhport, ep_addr, buffer, total_bytes);
  }
  musb_int_unmask();
  return ret;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes)
{
  (void)rhport;
  bool ret;
  // TU_LOG1("X %x %d\r\n", ep_addr, total_bytes);
  unsigned const epnum = tu_edpt_number(ep_addr);
  TU_ASSERT(epnum);

  musb_int_mask();
  _dcd.pipe_buf_is_fifo[tu_edpt_dir(ep_addr)] |= TU_BIT(epnum - 1);
  ret = edpt_n_xfer(rhport, ep_addr, (uint8_t*)ff, total_bytes);
  musb_int_unmask();

  return ret;
}

// Stall endpoint
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;
  unsigned const epn = tu_edpt_number(ep_addr);
  musb_int_mask();
  USBC_SelectActiveEp(epn);
  if (0 == epn) {
    if (!ep_addr) { /* Ignore EP80 */
      _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
      _dcd.pipe0.buf = NULL;
      __USBC_Dev_ep0_SendStall();
    }
  } else {
    if (tu_edpt_dir(ep_addr)) { /* IN */
      __USBC_Dev_Tx_SendStall();
    } else { /* OUT */
      TU_ASSERT(!__USBC_Dev_Rx_IsReadDataReady(),);
      __USBC_Dev_Rx_SendStall();
    }
  }
  musb_int_unmask();
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;
  unsigned const epn = tu_edpt_number(ep_addr);
  musb_int_mask();
  USBC_SelectActiveEp(epn);
  if (0 != epn) {
    if (tu_edpt_dir(ep_addr)) { /* IN */
      __USBC_Dev_Tx_ClearStall();
    } else { /* OUT */
      __USBC_Dev_Rx_ClearStall();
    }
  }
  musb_int_unmask();
}


void dcd_int_handler(uint8_t rhport)
{
  uint8_t is;
  uint16_t txis, rxis;

  is   = USBC_Readb(USBC_REG_INTUSB(USBC0_BASE));   /* read interrupt status */
  txis = USBC_Readw(USBC_REG_INTTx(USBC0_BASE)); /* read interrupt status */
  rxis = USBC_Readw(USBC_REG_INTRx(USBC0_BASE)); /* read interrupt status */

  is &= USBC_Readb(USBC_REG_INTUSBE(USBC0_BASE)); /* ignore disabled interrupts */
  USBC_Writeb(is, USBC_REG_INTUSB(USBC0_BASE)); /* sunxi musb requires a write to interrupt register to clear */
  if (is & USBC_INTUSB_DISCONNECT) {
	dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
  }
  if (is & USBC_INTUSB_SOF) {
    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
  }
  if (is & USBC_INTUSB_RESET) {
    /* ep0 FADDR must be 0 when (re)entering peripheral mode */
    USBC_SelectActiveEp(0);
    USBC_Dev_SetAddress(0);
    process_bus_reset(rhport);
  }
  if (is & USBC_INTUSB_RESUME) {
    dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
  }
  if (is & USBC_INTUSB_SUSPEND) {
    dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
  }

  txis &= USBC_Readw(USBC_REG_INTTxE(USBC0_BASE));
  USBC_Writew(txis, USBC_REG_INTTx(USBC0_BASE));
  if (txis & USBC_INTTx_FLAG_EP0) {
    process_ep0(rhport);
    txis &= ~TU_BIT(0);
  }
  while (txis) {
    unsigned const num = __builtin_ctz(txis);
    process_edpt_n(rhport, tu_edpt_addr(num, TUSB_DIR_IN));
    txis &= ~TU_BIT(num);
  }

  rxis &= USBC_Readw(USBC_REG_INTRxE(USBC0_BASE));
  USBC_Writew(rxis, USBC_REG_INTRx(USBC0_BASE));
  while (rxis) {
    unsigned const num = __builtin_ctz(rxis);
    process_edpt_n(rhport, tu_edpt_addr(num, TUSB_DIR_OUT));
    rxis &= ~TU_BIT(num);
  }
}

#endif
