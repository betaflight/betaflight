/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, Koji Kitayama
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

/* How to connect JLink and GR-CITRUS
 *
 * GR-CITRUS needs to solder some pads to enable JTAG interface.
 * - Short the following pads individually with solder.
 *   - J4
 *   - J5
 * - Short EMLE pad and 3.3V(GR-CITRUS pin name) with a wire.
 *
 * The pads are [the back side of GR-CITRUS](https://www.slideshare.net/MinaoYamamoto/grcitrusrx631/2).
 *
 * Connect the pins between GR-CITRUS and JLink as follows.
 *
 * | Function  | GR-CITRUS pin | JLink pin No.| note     |
 * |:---------:|:-------------:|:------------:|:--------:|
 * | VTref     |   3.3V        |   1          |          |
 * | TRST      |   5           |   3          |          |
 * | GND       |   GND         |   4          |          |
 * | TDI       |   3           |   5          |          |
 * | TMS       |   2           |   7          |          |
 * | TCK/FINEC |   14          |   9          | short J4 |
 * | TDO       |   9           |  13          | short J5 |
 * | nRES      |   RST         |  15          |          |
 *
 * JLink firmware needs to update to V6.96 or newer version to avoid
 * [a bug](https://forum.segger.com/index.php/Thread/7758-SOLVED-Bug-in-JLink-from-V6-88b-regarding-RX65N)
 * regarding downloading.
 */

#include "bsp/board_api.h"
#include "iodefine.h"
#include "interrupt_handlers.h"

#define IRQ_PRIORITY_CMT0     5
#define IRQ_PRIORITY_USBI0    6
#define IRQ_PRIORITY_SCI0     5

#define SYSTEM_PRCR_PRC1      (1<<1)
#define SYSTEM_PRCR_PRKEY     (0xA5u<<8)

#define CMT_PCLK              48000000
#define CMT_CMCR_CKS_DIV_128  2
#define CMT_CMCR_CMIE         (1<<6)
#define MPC_PFS_ISEL          (1<<6)

#define SCI_PCLK              48000000
#define SCI_SSR_FER           (1<<4)
#define SCI_SSR_ORER          (1<<5)

#define SCI_SCR_TEIE          (1u<<2)
#define SCI_SCR_RE            (1u<<4)
#define SCI_SCR_TE            (1u<<5)
#define SCI_SCR_RIE           (1u<<6)
#define SCI_SCR_TIE           (1u<<7)

//--------------------------------------------------------------------+
// SCI0 handling
//--------------------------------------------------------------------+
typedef struct {
  uint8_t *buf;
  uint32_t cnt;
} sci_buf_t;
static volatile sci_buf_t sci0_buf[2];

void INT_Excep_SCI0_TXI0(void)
{
  uint8_t *buf = sci0_buf[0].buf;
  uint32_t cnt = sci0_buf[0].cnt;

  if (!buf || !cnt) {
    SCI0.SCR.BYTE &= ~(SCI_SCR_TEIE | SCI_SCR_TE | SCI_SCR_TIE);
    return;
  }
  SCI0.TDR = *buf;
  if (--cnt) {
    ++buf;
  } else {
    buf = NULL;
    SCI0.SCR.BIT.TIE  = 0;
    SCI0.SCR.BIT.TEIE = 1;
  }
  sci0_buf[0].buf = buf;
  sci0_buf[0].cnt = cnt;
}

void INT_Excep_SCI0_TEI0(void)
{
  SCI0.SCR.BYTE &= ~(SCI_SCR_TEIE | SCI_SCR_TE | SCI_SCR_TIE);
}

void INT_Excep_SCI0_RXI0(void)
{
  uint8_t *buf = sci0_buf[1].buf;
  uint32_t cnt = sci0_buf[1].cnt;

  if (!buf || !cnt ||
      (SCI0.SSR.BYTE & (SCI_SSR_FER | SCI_SSR_ORER))) {
    sci0_buf[1].buf = NULL;
    SCI0.SSR.BYTE   = 0;
    SCI0.SCR.BYTE  &= ~(SCI_SCR_RE | SCI_SCR_RIE);
    return;
  }
  *buf = SCI0.RDR;
  if (--cnt) {
    ++buf;
  } else {
    buf = NULL;
    SCI0.SCR.BYTE &= ~(SCI_SCR_RE | SCI_SCR_RIE);
  }
  sci0_buf[1].buf = buf;
  sci0_buf[1].cnt = cnt;
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void INT_Excep_USB0_USBI0(void)
{
  tud_int_handler(0);
}

void board_init(void)
{
#if CFG_TUSB_OS == OPT_OS_NONE
  /* Enable CMT0 */
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY | SYSTEM_PRCR_PRC1;
  MSTP(CMT0)       = 0;
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY;
  /* Setup 1ms tick timer */
  CMT0.CMCNT      = 0;
  CMT0.CMCOR      = CMT_PCLK / 1000 / 128;
  CMT0.CMCR.WORD  = CMT_CMCR_CMIE | CMT_CMCR_CKS_DIV_128;
  IR(CMT0, CMI0)  = 0;
  IPR(CMT0, CMI0) = IRQ_PRIORITY_CMT0;
  IEN(CMT0, CMI0) = 1;
  CMT.CMSTR0.BIT.STR0 = 1;
#endif

  /* Unlock MPC registers */
  MPC.PWPR.BIT.B0WI  = 0;
  MPC.PWPR.BIT.PFSWE = 1;
  /* LED PA0 */
  PORTA.PMR.BIT.B0  = 0U;
  PORTA.PODR.BIT.B0 = 0U;
  PORTA.PDR.BIT.B0  = 1U;
  /* UART TXD0 => P20, RXD0 => P21 */
  PORT2.PMR.BIT.B0 = 1U;
  PORT2.PCR.BIT.B0 = 1U;
  MPC.P20PFS.BYTE  = 0b01010;
  PORT2.PMR.BIT.B1 = 1U;
  MPC.P21PFS.BYTE  = 0b01010;
  /* USB VBUS -> P16 DPUPE -> P14 */
  PORT1.PMR.BIT.B4 = 1U;
  PORT1.PMR.BIT.B6 = 1U;
  MPC.P14PFS.BYTE  = 0b10001;
  MPC.P16PFS.BYTE  = MPC_PFS_ISEL | 0b10001;
  MPC.PFUSB0.BIT.PUPHZS = 1;
  /* Lock MPC registers */
  MPC.PWPR.BIT.PFSWE = 0;
  MPC.PWPR.BIT.B0WI  = 1;

  IR(USB0, USBI0)  = 0;
  IPR(USB0, USBI0) = IRQ_PRIORITY_USBI0;

  /* Enable SCI0 */
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY | SYSTEM_PRCR_PRC1;
  MSTP(SCI0) = 0;
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY;
  SCI0.BRR = (SCI_PCLK / (32 * 115200)) - 1;
  IR(SCI0,  RXI0)  = 0;
  IR(SCI0,  TXI0)  = 0;
  IR(SCI0,  TEI0)  = 0;
  IPR(SCI0, RXI0) = IRQ_PRIORITY_SCI0;
  IPR(SCI0, TXI0) = IRQ_PRIORITY_SCI0;
  IPR(SCI0, TEI0) = IRQ_PRIORITY_SCI0;
  IEN(SCI0, RXI0) = 1;
  IEN(SCI0, TXI0) = 1;
  IEN(SCI0, TEI0) = 1;

  /* Enable USB0 */
  unsigned short oldPRCR = SYSTEM.PRCR.WORD;
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY | SYSTEM_PRCR_PRC1;
  MSTP(USB0) = 0;
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY | oldPRCR;
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  PORTA.PODR.BIT.B0 = state ? 1 : 0;
}

uint32_t board_button_read(void)
{
  return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
  sci0_buf[1].buf = buf;
  sci0_buf[1].cnt = len;
  SCI0.SCR.BYTE |= SCI_SCR_RE | SCI_SCR_RIE;
  while (SCI0.SCR.BIT.RE) ;
  return len - sci0_buf[1].cnt;
}

int board_uart_write(void const *buf, int len)
{
  sci0_buf[0].buf = (uint8_t*)(uintptr_t) buf;
  sci0_buf[0].cnt = len;
  SCI0.SCR.BYTE |= SCI_SCR_TE | SCI_SCR_TIE;
  while (SCI0.SCR.BIT.TE) ;
  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void INT_Excep_CMT0_CMI0(void)
{
  ++system_ticks;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#else
uint32_t SystemCoreClock = 96000000;
#endif

int close(int fd)
{
    (void)fd;
    return -1;
}
int fstat(int fd, void *pstat)
{
    (void)fd;
    (void)pstat;
    return 0;
}
off_t lseek(int fd, off_t pos, int whence)
{
    (void)fd;
    (void)pos;
    (void)whence;
    return 0;
}
int isatty(int fd)
{
    (void)fd;
    return 1;
}
