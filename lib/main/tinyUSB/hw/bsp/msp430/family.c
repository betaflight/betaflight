/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#include "bsp/board_api.h"
#include "board.h"
#include "msp430.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void __attribute__ ((interrupt(USB_UBM_VECTOR))) USB_UBM_ISR(void)
{
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

uint32_t cnt = 0;

static void SystemClock_Config(void)
{
  WDTCTL = WDTPW + WDTHOLD; // Disable watchdog.

  // Increase VCore to level 2- required for 16 MHz operation on this MCU.
  PMMCTL0 = PMMPW + PMMCOREV_2;

  UCSCTL3 = SELREF__XT2CLK; // FLL is fed by XT2.

  // XT1 used for ACLK (default- not used in this demo)
  P5SEL |= BIT4; // Required to enable XT1
  // Loop until XT1 fault flag is cleared.
  do
  {
    UCSCTL7 &= ~XT1LFOFFG;
  }while(UCSCTL7 & XT1LFOFFG);

  // XT2 is 4 MHz an external oscillator, use PLL to boost to 16 MHz.
  P5SEL |= BIT2; // Required to enable XT2.
  // Loop until XT2 fault flag is cleared
  do
  {
    UCSCTL7 &= ~XT2OFFG;
  }while(UCSCTL7 & XT2OFFG);

  // Kickstart the DCO into the correct frequency range, otherwise a
  // fault will occur.
  // FIXME: DCORSEL_6 should work according to datasheet params, but generates
  // a fault. I am not sure why it faults.
  UCSCTL1 = DCORSEL_7;
  UCSCTL2 = FLLD_2 + 3; // DCO freq = D * (N + 1) * (FLLREFCLK / n)
                        // DCOCLKDIV freq = (N + 1) * (FLLREFCLK / n)
                        // N = 3, D = 2, thus DCO freq = 32 MHz.

  // MCLK configured for 16 MHz using XT2.
  // SMCLK configured for 8 MHz using XT2.
  UCSCTL4 |= SELM__DCOCLKDIV + SELS__DCOCLKDIV;
  UCSCTL5 |= DIVM__16 + DIVS__2;

  // Now wait till everything's stabilized.
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
    SFRIFG1 &= ~OFIFG;
  }while(SFRIFG1 & OFIFG);

  // Configure Timer A to use SMCLK as a source. Count 1000 ticks at 1 MHz.
  TA0CCTL0 |= CCIE;
  TA0CCR0 = 999; // 1000 ticks.
  TA0CTL |= TASSEL_2 + ID_3 + MC__UP; // Use SMCLK, divide by 8, start timer.

  // Initialize USB power and PLL.
  USBKEYPID = USBKEY;

  // VUSB enabled automatically.
  // Wait two milliseconds to stabilize, per manual recommendation.
  uint32_t ms_elapsed = board_millis();
  do
  {
    while((board_millis() - ms_elapsed) < 2);
  }while(!(USBPWRCTL & USBBGVBV));

  // USB uses XT2 (4 MHz) directly. Enable the PLL.
  USBPLLDIVB |= USBPLL_SETCLK_4_0;
  USBPLLCTL |= (UPFDEN | UPLLEN);

  // Wait until PLL locks. Check every 2ms, per manual.
  ms_elapsed = board_millis();
  do
  {
    USBPLLIR &= ~USBOOLIFG;
    while((board_millis() - ms_elapsed) < 2);
  }while(USBPLLIR & USBOOLIFG);

  USBKEYPID = 0;
}

uint32_t wait = 0;

void board_init(void)
{
  __bis_SR_register(GIE); // Enable interrupts.
  SystemClock_Config();

  // Enable basic I/O.
  P1DIR |= LED_PIN; // LED output.
  P1REN |= BUTTON_PIN; // Internal resistor enable.
  P1OUT |= BUTTON_PIN; // Pullup.

  // Enable the backchannel UART (115200)
  P4DIR |= BIT5;
  P4SEL |= (BIT5 | BIT4);

  UCA1CTL1 |= (UCSSEL__SMCLK | UCSWRST); // Hold in reset, use SMCLK.
  UCA1BRW = 4;
  UCA1MCTL |= (UCBRF_3 | UCBRS_5 | UCOS16); // Overampling mode, 115200 baud.
                                            // Copied from manual.
  UCA1CTL1 &= ~UCSWRST;

  // Set up USB pins.
  USBKEYPID = USBKEY;
  USBPHYCTL |= PUSEL; // Convert USB D+/D- pins to USB functionality.
  USBKEYPID = 0;
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  if(state)
  {
    LED_PORT |= LED_PIN;
  }
  else
  {
    LED_PORT &= ~LED_PIN;
  }
}

uint32_t board_button_read(void)
{
  return ((P1IN & BIT1) >> 1) == BUTTON_STATE_ACTIVE;
}

int board_uart_read(uint8_t * buf, int len)
{
  for(int i = 0; i < len; i++)
  {
    // Wait until something to receive (cleared by reading buffer).
    while(!(UCA1IFG & UCRXIFG));
    buf[i] = UCA1RXBUF;
  }

  return len;
}

int board_uart_write(void const * buf, int len)
{
  const char * char_buf = (const char *) buf;

  for(int i = 0; i < len; i++)
  {
    // Wait until TX buffer is empty (cleared by writing buffer).
    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = char_buf[i];
  }

  return len;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
{
  system_ticks++;
  // TAxCCR0 CCIFG resets itself as soon as interrupt is invoked.
}

uint32_t board_millis(void)
{
  uint32_t systick_mirror;

  // 32-bit update is not atomic on MSP430. We can read the bottom 16-bits,
  // an interrupt occurs, updates _all_ 32 bits, and then we return a
  // garbage value. And I've seen it happen!
  TA0CCTL0 &= ~CCIE;
  systick_mirror = system_ticks;
  TA0CCTL0 |= CCIE;

  return systick_mirror;
}
#endif
