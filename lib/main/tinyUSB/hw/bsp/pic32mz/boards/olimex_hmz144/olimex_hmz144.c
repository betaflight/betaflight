/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Jerzy Kasenberg
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
#include <stdbool.h>
#include <xc.h>
#include "tusb.h"

/* JTAG on, WDT off */
#pragma config FDMTEN=0, FSOSCEN=0, DMTCNT=1
#pragma config DEBUG=ON
#pragma config JTAGEN=ON
#pragma config FSLEEP=OFF
#pragma config TRCEN=OFF
#pragma config ICESEL=ICS_PGx2

#pragma config POSCMOD = HS
#pragma config FNOSC = SPLL
/* 24MHz posc input to pll, div by 3, multiply by 50, div by 2 -> 200mhz*/
#pragma config FPLLICLK=0, FPLLIDIV=DIV_3, FPLLRNG=RANGE_5_10_MHZ, FPLLMULT=MUL_50, FPLLODIV=DIV_2
#pragma config FUSBIDIO=1
#pragma config WINDIS=NORMAL
#pragma config WDTSPGM=1
#pragma config WDTPS=15
#pragma config FWDTEN=OFF

void button_init(void)
{
  // RB12 - button
  // ANSELB B12 not analog
  ANSELBCLR = TU_BIT(12);
  // TRISB B12 input
  TRISBSET = TU_BIT(12);
}

void led_init(void)
{
  // RH2 - LED
  // ANASELH no analog function on RH2
  // TRISH RH2 output
  TRISHCLR = TU_BIT(2);
  // Initial value 0, LED off
  LATHCLR = TU_BIT(2);
}

void uart_init(void)
{
  // RE8/RE9 Uart2 TX/RX
  // ANSELE - TX/RX not analog
  ANSELECLR = TU_BIT(8) | TU_BIT(9);

  /* Unlock system for PPS configuration */
  SYSKEY = 0x00000000;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
  CFGCONbits.IOLOCK = 0;

  // PPS Input Remapping
  // U2RX -> RE9
  U2RXR = 13;

  // PPS Output Remapping
  // RE8 -> U2TX
  RPE8R = 2;

  // Lock back the system after PPS configuration
  CFGCONbits.IOLOCK = 1;
  SYSKEY = 0x00000000;

  // UART2
  // High speed mode
  // 8 bits, no parity, no RTS/CTS, no flow control
  U2MODE = 0x0;

  // Enable UART2 Receiver and Transmitter
  U2STASET = (_U2STA_UTXEN_MASK | _U2STA_URXEN_MASK | _U2STA_UTXISEL1_MASK);

  // BAUD Rate register Setup
  U2BRG = 100000000 / (16 * 115200) + 1;

  // Disable Interrupts
  IEC4CLR = _IEC4_U2EIE_MASK | _IEC4_U2RXIE_MASK | _IEC4_U2TXIE_MASK;

  // Turn ON UART2
  U2MODESET = _U2MODE_ON_MASK;
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  if (state)
  {
    LATHSET = TU_BIT(2);
  }
  else
  {
    LATHCLR = TU_BIT(2);
  }
}

uint32_t board_button_read(void)
{
  return ((PORTB >> 12) & 1) == 0;
}

int board_uart_write(void const * buf, int len)
{
  int i = len;
  uint8_t const * data = buf;
  while (i--)
  {
    while (U2STAbits.UTXBF) ;
    U2TXREG = *data++;
  }
  return len;
}
