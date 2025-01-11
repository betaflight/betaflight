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

/* How to connect JLink and RX65n Target and option board
 * (For original comment https://github.com/hathach/tinyusb/pull/922#issuecomment-869786131)
 *
 * To enable JTAG, RX65N requires following connections on main board.
 * - short EJ2 jumper header, to disable onboard E2L.
 * - short EMLE(J1-2) and 3V3(J1-14 or J2-10), to enable In-Circuit Emulator.
 *
 * Note: For RX65N-Cloud-Kit, the option board's JTAG pins to some switches or floating.
 * To use JLink with the option board, I think some further modifications will be necessary.
 *
 * | Function  | RX65N pin  | main board | option board | JLink connector |
 * |:---------:|:----------:|:----------:|:------------:|:---------------:|
 * | 3V3       | VCC        |   J1-14    | CN5-6        |    1            |
 * | TRST      | P34        |   J1-16    | CN5-7        |    3            |
 * | GND       | VSS        |   J1-12    | CN5-5        |    4            |
 * | TDI       | P30        |   J1-20    | CN5-10       |    5            |
 * | TMS       | P31        |   J1-19    | USER_SW      |    7            |
 * | TCK/FINEC | P27        |   J1-21    | N/A          |    9            |
 * | TDO       | P26        |   J1-22    | CN5-9        |   13            |
 * | nRES      | RES#       |   J1-10    | RESET_SW     |   15            |
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
#define IRQ_PRIORITY_SCI5     5

#define SYSTEM_PRCR_PRC1      (1<<1)
#define SYSTEM_PRCR_PRKEY     (0xA5u<<8)

#define CMT_PCLK              60000000
#define CMT_CMCR_CKS_DIV_128  2
#define CMT_CMCR_CMIE         (1<<6)
#define MPC_PFS_ISEL          (1<<6)

#define SCI_PCLK              60000000
#define SCI_SSR_FER           (1<<4)
#define SCI_SSR_ORER          (1<<5)

#define SCI_SCR_TEIE          (1u<<2)
#define SCI_SCR_RE            (1u<<4)
#define SCI_SCR_TE            (1u<<5)
#define SCI_SCR_RIE           (1u<<6)
#define SCI_SCR_TIE           (1u<<7)
#define INT_Excep_SCI5_TEI5   INT_Excep_ICU_GROUPBL0

#define IRQ_USB0_USBI0        62
#define SLIBR_USBI0           SLIBR185
#define IPR_USB0_USBI0        IPR_PERIB_INTB185
#define INT_Excep_USB0_USBI0  INT_Excep_PERIB_INTB185

void HardwareSetup(void)
{
  FLASH.ROMCIV.WORD = 1;
  while (FLASH.ROMCIV.WORD) ;
  FLASH.ROMCE.WORD = 1;
  while (!FLASH.ROMCE.WORD) ;

  SYSTEM.PRCR.WORD = 0xA503u;
  if (!SYSTEM.RSTSR1.BYTE) {
    RTC.RCR4.BYTE = 0;
    RTC.RCR3.BYTE = 12;
    while (12 != RTC.RCR3.BYTE) ;
  }
  SYSTEM.SOSCCR.BYTE = 1;

  if (SYSTEM.HOCOCR.BYTE) {
    SYSTEM.HOCOCR.BYTE = 0;
    while (!SYSTEM.OSCOVFSR.BIT.HCOVF) ;
  }
  SYSTEM.PLLCR.WORD  = 0x1D10u; /* HOCO x 15 */
  SYSTEM.PLLCR2.BYTE = 0;
  while (!SYSTEM.OSCOVFSR.BIT.PLOVF) ;

  SYSTEM.SCKCR.LONG  = 0x21C11222u;
  SYSTEM.SCKCR2.WORD = 0x0041u;
  SYSTEM.ROMWT.BYTE  = 0x02u;
  while (0x02u != SYSTEM.ROMWT.BYTE) ;
  SYSTEM.SCKCR3.WORD = 0x400u;
  SYSTEM.PRCR.WORD   = 0xA500u;
}

//--------------------------------------------------------------------+
// SCI handling
//--------------------------------------------------------------------+
typedef struct {
  uint8_t *buf;
  uint32_t cnt;
} sci_buf_t;
static volatile sci_buf_t sci_buf[2];

void INT_Excep_SCI5_TXI5(void)
{
  uint8_t *buf = sci_buf[0].buf;
  uint32_t cnt = sci_buf[0].cnt;

  if (!buf || !cnt) {
    SCI5.SCR.BYTE &= ~(SCI_SCR_TEIE | SCI_SCR_TE | SCI_SCR_TIE);
    return;
  }
  SCI5.TDR = *buf;
  if (--cnt) {
    ++buf;
  } else {
    buf = NULL;
    SCI5.SCR.BIT.TIE  = 0;
    SCI5.SCR.BIT.TEIE = 1;
  }
  sci_buf[0].buf = buf;
  sci_buf[0].cnt = cnt;
}

void INT_Excep_SCI5_TEI5(void)
{
  SCI5.SCR.BYTE &= ~(SCI_SCR_TEIE | SCI_SCR_TE | SCI_SCR_TIE);
}

void INT_Excep_SCI5_RXI5(void)
{
  uint8_t *buf = sci_buf[1].buf;
  uint32_t cnt = sci_buf[1].cnt;

  if (!buf || !cnt ||
      (SCI5.SSR.BYTE & (SCI_SSR_FER | SCI_SSR_ORER))) {
    sci_buf[1].buf = NULL;
    SCI5.SSR.BYTE   = 0;
    SCI5.SCR.BYTE  &= ~(SCI_SCR_RE | SCI_SCR_RIE);
    return;
  }
  *buf = SCI5.RDR;
  if (--cnt) {
    ++buf;
  } else {
    buf = NULL;
    SCI5.SCR.BYTE &= ~(SCI_SCR_RE | SCI_SCR_RIE);
  }
  sci_buf[1].buf = buf;
  sci_buf[1].cnt = cnt;
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void INT_Excep_USB0_USBI0(void)
{
#if CFG_TUH_ENABLED
  tuh_int_handler(0, true);
#endif
#if CFG_TUD_ENABLED
  tud_int_handler(0);
#endif
}

void board_init(void)
{
  /* setup software configurable interrupts */
  ICU.SLIBR_USBI0.BYTE = IRQ_USB0_USBI0;
  ICU.SLIPRCR.BYTE     = 1;

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
  // SW PB1
  PORTB.PMR.BIT.B1 = 0U;
  PORTB.PDR.BIT.B1 = 0U;
  // LED PD6
  PORTD.PODR.BIT.B6 = 1U;
  PORTD.ODR1.BIT.B4 = 1U;
  PORTD.PMR.BIT.B6  = 0U;
  PORTD.PDR.BIT.B6  = 1U;
  /* UART TXD5 => PA4, RXD5 => PA3 */
  PORTA.PMR.BIT.B4 = 1U;
  PORTA.PCR.BIT.B4 = 1U;
  MPC.PA4PFS.BYTE  = 0b01010;
  PORTA.PMR.BIT.B3 = 1U;
  MPC.PA5PFS.BYTE  = 0b01010;
  /* USB VBUS -> P16 */
  PORT1.PMR.BIT.B6 = 1U;
  MPC.P16PFS.BYTE  = MPC_PFS_ISEL | 0b10001;
  /* Lock MPC registers */
  MPC.PWPR.BIT.PFSWE = 0;
  MPC.PWPR.BIT.B0WI  = 1;

  /* Enable SCI5 */
  SYSTEM.PRCR.WORD   = SYSTEM_PRCR_PRKEY | SYSTEM_PRCR_PRC1;
  MSTP(SCI5)         = 0;
  SYSTEM.PRCR.WORD   = SYSTEM_PRCR_PRKEY;
  SCI5.SEMR.BIT.ABCS = 1;
  SCI5.SEMR.BIT.BGDM = 1;
  SCI5.BRR           = (SCI_PCLK / (8 * 115200)) - 1;
  IR(SCI5,  RXI5)    = 0;
  IR(SCI5,  TXI5)    = 0;
  IS(SCI5,  TEI5)    = 0;
  IR(ICU, GROUPBL0)  = 0;
  IPR(SCI5, RXI5)    = IRQ_PRIORITY_SCI5;
  IPR(SCI5, TXI5)    = IRQ_PRIORITY_SCI5;
  IPR(ICU,GROUPBL0)  = IRQ_PRIORITY_SCI5;
  IEN(SCI5, RXI5)    = 1;
  IEN(SCI5, TXI5)    = 1;
  IEN(ICU,GROUPBL0)  = 1;
  EN(SCI5, TEI5)     = 1;

  /* Enable USB0 */
  unsigned short oldPRCR = SYSTEM.PRCR.WORD;
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY | SYSTEM_PRCR_PRC1;
  MSTP(USB0) = 0;
  SYSTEM.PRCR.WORD = SYSTEM_PRCR_PRKEY | oldPRCR;

  /* setup USBI0 interrupt. */
  IR(USB0, USBI0)  = 0;
  IPR(USB0, USBI0) = IRQ_PRIORITY_USBI0;
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  PORTD.PODR.BIT.B6 = state ? 0 : 1;
}

uint32_t board_button_read(void)
{
  return PORTB.PIDR.BIT.B1 ? 0 : 1;
}

int board_uart_read(uint8_t* buf, int len)
{
  sci_buf[1].buf = buf;
  sci_buf[1].cnt = len;
  SCI5.SCR.BYTE |= SCI_SCR_RE | SCI_SCR_RIE;
  // TODO change to non blocking, return -1 immediately if no data
  while (SCI5.SCR.BIT.RE) ;
  return len - sci_buf[1].cnt;
}

int board_uart_write(void const *buf, int len)
{
  sci_buf[0].buf = (uint8_t*)(uintptr_t) buf;
  sci_buf[0].cnt = len;
  SCI5.SCR.BYTE |= SCI_SCR_TE | SCI_SCR_TIE;
  while (SCI5.SCR.BIT.TE) ;
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
uint32_t SystemCoreClock = 120000000;
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
