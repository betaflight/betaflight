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
#include "msp.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB0_IRQHandler(void)
{
#if CFG_TUH_ENABLED
  tuh_int_handler(0, true);
#endif
#if CFG_TUD_ENABLED
  tud_int_handler(0);
#endif
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

void board_init(void)
{
  unsigned bits;
  /* Turn off power domains that unused peripherals belong to */
  SYSCTL->PCCAN  = 0u;
#ifdef __MCU_HAS_LCD0__
  SYSCTL->PCLCD  = 0u;
#endif
  SYSCTL->PCEMAC = 0u;
  SYSCTL->PCEPHY = 0u;
  SYSCTL->PCCCM  = 0u;

  /* --- Setup system clock --- */
  /* Start power-up process of the main oscillator */
  SYSCTL->MOSCCTL = SYSCTL_MOSCCTL_OSCRNG;
  while (!(SYSCTL->RIS & SYSCTL_RIS_MOSCPUPRIS)) ; /* Wait for completion */
  SYSCTL->MISC = SYSCTL_MISC_MOSCPUPMIS; /* Clear the completion interrupt status */
  /* Set the main oscillator to PLL reference clock */
  SYSCTL->RSCLKCFG = SYSCTL_RSCLKCFG_PLLSRC_MOSC;
  /* PLL freq. = (MOSC freq. / 10) * 96 = 240MHz */
  SYSCTL->PLLFREQ1 = (4 << SYSCTL_PLLFREQ1_N_S) | (1 << SYSCTL_PLLFREQ1_Q_S);
  SYSCTL->PLLFREQ0 = (96 << SYSCTL_PLLFREQ0_MINT_S) | SYSCTL_PLLFREQ0_PLLPWR;
  /* Set BCHT=6, BCE=0, WS=5 for 120MHz system clock */
  SYSCTL->MEMTIM0 = SYSCTL_MEMTIM0_EBCHT_3_5 | (5 << SYSCTL_MEMTIM0_EWS_S) |
    SYSCTL_MEMTIM0_FBCHT_3_5 | (5 << SYSCTL_MEMTIM0_FWS_S) | SYSCTL_MEMTIM0_MB1;
  /* Wait for completion of PLL power-up process */
  while (!(SYSCTL->RIS & SYSCTL_RIS_PLLLRIS)) ;
  SYSCTL->MISC = SYSCTL_MISC_PLLLMIS; /* Clear the completion interrupt status */
  /* Switch the system clock to PLL/4 */
  SYSCTL->RSCLKCFG = SYSCTL_RSCLKCFG_MEMTIMU | SYSCTL_RSCLKCFG_ACG |
         SYSCTL_RSCLKCFG_USEPLL | SYSCTL_RSCLKCFG_PLLSRC_MOSC | (1 << SYSCTL_RSCLKCFG_PSYSDIV_S);

  SystemCoreClockUpdate();
#if CFG_TUSB_OS == OPT_OS_NONE
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  /* USR_LED1 ON1 */
  bits              = TU_BIT(CLK_LED);
  SYSCTL->RCGCGPIO |= bits;
  while (bits != (SYSCTL->RCGCGPIO & bits)) ;
  GPIO_LED->DIR     = TU_BIT(GPIO_LED_PIN);
  GPIO_LED->DEN     = TU_BIT(GPIO_LED_PIN);

  /* USR_SW1 PJ0 */
  bits              = TU_BIT(CLK_BUTTON);
  SYSCTL->RCGCGPIO |= bits;
  while (bits != (SYSCTL->RCGCGPIO & bits)) ;
  GPIO_BUTTON->PUR  = TU_BIT(GPIO_BUTTON_PIN);
  GPIO_BUTTON->DEN  = TU_BIT(GPIO_BUTTON_PIN);

  /* UART PA0,1 */
  bits              = TU_BIT(0);
  SYSCTL->RCGCGPIO |= bits;
  while (bits != (SYSCTL->RCGCGPIO & bits)) ;
  GPIOA->AFSEL      = 3u;
  GPIOA->PCTL       = 0x11u;
  GPIOA->DEN        = 3u;

  SYSCTL->RCGCUART |= 1u << 0;
  while (!(SYSCTL->PRUART & (1u << 0))) ;
  UART0->CTL        = 0;
  UART0->IBRD       = 8;  /* 8.68056 = 16MHz / (16 * 115200) */
  UART0->FBRD       = 44; /* 0.6875 = 44/64 -> 115108bps (0.08%) */
  UART0->LCRH       = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
  UART0->CC         = UART_CC_CS_PIOSC; /* Set the baud clock to PIOSC */
  UART0->CTL        = UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN;

  /* USB PB0(ID) PB1(VBUS) PL6,7(DP,DM) */
  bits              = TU_BIT(1) | TU_BIT(10);
  SYSCTL->RCGCGPIO |= bits;
  while (bits != (SYSCTL->RCGCGPIO & bits)) ;
  GPIOB->AMSEL      = TU_BIT(0) | TU_BIT(1);
  GPIOL->AMSEL      = TU_BIT(6) | TU_BIT(7);

#if CFG_TUH_ENABLED
  /* USB PD6(EPEN) */
  bits              = TU_BIT(3);
  SYSCTL->RCGCGPIO |= bits;
  while (bits != (SYSCTL->RCGCGPIO & bits)) ;
  GPIOD->AFSEL      = TU_BIT(6);
  GPIOD->PCTL       = 0x05000000u;
  GPIOD->DEN        = TU_BIT(6);
#endif

  SYSCTL->RCGCUSB   = 1u; /* Open the clock gate for SYSCLK */
  while (!(SYSCTL->PRUSB & (1u << 0))) ;
  USB0->CC          = USB_CC_CLKEN | (3u << USB_CC_CLKDIV_S); /* 60MHz = 240MHz / 4 */
  __DMB(); /* Wait for completion of opening of the clock gate */

  SYSCTL->SRUSB     = 1u;
  for (int i = 0; i < 16; ++i) __NOP();
  SYSCTL->SRUSB     = 0u;

  USB0->CC          = USB_CC_CLKEN | (3u << USB_CC_CLKDIV_S); /* 60MHz = 240MHz / 4 */
  __DMB(); /* Wait for completion of opening of the clock gate */
#if CFG_TUH_ENABLED
  USB0->GPCS = USB_GPCS_DEVMOD_OTG;
  USB0->EPC  = USB_EPC_EPENDE | USB_EPC_EPEN_HIGH;
#endif
#if CFG_TUD_ENABLED
  USB0->GPCS = USB_GPCS_DEVMOD_DEVVBUS;
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  if (state)
    GPIO_LED->DATA |= TU_BIT(GPIO_LED_PIN);
  else
    GPIO_LED->DATA &= ~TU_BIT(GPIO_LED_PIN);
}

uint32_t board_button_read(void)
{
  return (GPIO_BUTTON->DATA & TU_BIT(GPIO_BUTTON_PIN)) ? 0u : 1u;
}

int board_uart_read(uint8_t * buf, int len)
{
  for (int i = 0; i < len; ++i) {
    while (UART0->FR & UART_FR_RXFE) ;
    *buf++ = UART0->DR;
  }
  return len;
}

int board_uart_write(void const * buf, int len)
{
  uint8_t const *p = (uint8_t const *)buf;
  for (int i = 0; i < len; ++i) {
    while (UART0->FR & UART_FR_TXFF) ;
    UART0->DR = *p++;
  }
  return len;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0u;
void SysTick_Handler(void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif
