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

#include "sam.h"
#include "bsp/board_api.h"
#include "board.h"

// Suppress warning caused by mcu driver
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

#include "hal/include/hal_gpio.h"
#include "hal/include/hal_init.h"
#include "hri/hri_nvmctrl_d21.h"

#include "hpl/gclk/hpl_gclk_base.h"
#include "hpl_pm_config.h"
#include "hpl/pm/hpl_pm_base.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

/* Referenced GCLKs, should be initialized firstly */
#define _GCLK_INIT_1ST (1 << 0 | 1 << 1)

/* Not referenced GCLKs, initialized last */
#define _GCLK_INIT_LAST (~_GCLK_INIT_1ST)

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_Handler(void) {
  tud_int_handler(0);
}

//--------------------------------------------------------------------+
// Implementation
//--------------------------------------------------------------------+
static void uart_init(void);

#if CFG_TUH_ENABLED && defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
#define MAX3421_SERCOM TU_XSTRCAT(SERCOM, MAX3421_SERCOM_ID)

static void max3421_init(void);

#endif

void board_init(void) {
  // Clock init ( follow hpl_init.c )
  hri_nvmctrl_set_CTRLB_RWS_bf(NVMCTRL, 2);

  _pm_init();
  _sysctrl_init_sources();
#if _GCLK_INIT_1ST
  _gclk_init_generators_by_fref(_GCLK_INIT_1ST);
#endif
  _sysctrl_init_referenced_generators();
  _gclk_init_generators_by_fref(_GCLK_INIT_LAST);

  // Update SystemCoreClock since it is hard coded with asf4 and not correct
  // Init 1ms tick timer (samd SystemCoreClock may not correct)
  SystemCoreClock = CONF_CPU_FREQUENCY;
#if CFG_TUSB_OS == OPT_OS_NONE
  SysTick_Config(CONF_CPU_FREQUENCY / 1000);
#endif

  // Led init
#ifdef LED_PIN
  gpio_set_pin_direction(LED_PIN, GPIO_DIRECTION_OUT);
  board_led_write(false);
#endif

  // Button init
#ifdef BUTTON_PIN
  gpio_set_pin_direction(BUTTON_PIN, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(BUTTON_PIN, BUTTON_STATE_ACTIVE ? GPIO_PULL_DOWN : GPIO_PULL_UP);
#endif

  uart_init();

#if CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

  /* USB Clock init
   * The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
   * for low speed and full speed operation. */
  _pm_enable_bus_clock(PM_BUS_APBB, USB);
  _pm_enable_bus_clock(PM_BUS_AHB, USB);
  _gclk_enable_channel(USB_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

  // USB Pin Init
  gpio_set_pin_direction(PIN_PA24, GPIO_DIRECTION_OUT);
  gpio_set_pin_level(PIN_PA24, false);
  gpio_set_pin_pull_mode(PIN_PA24, GPIO_PULL_OFF);
  gpio_set_pin_direction(PIN_PA25, GPIO_DIRECTION_OUT);
  gpio_set_pin_level(PIN_PA25, false);
  gpio_set_pin_pull_mode(PIN_PA25, GPIO_PULL_OFF);

  gpio_set_pin_function(PIN_PA24, PINMUX_PA24G_USB_DM);
  gpio_set_pin_function(PIN_PA25, PINMUX_PA25G_USB_DP);

  // Output 500hz PWM on D12 (PA19 - TCC0 WO[3]) so we can validate the GCLK0 clock speed with a Saleae.
  _pm_enable_bus_clock(PM_BUS_APBC, TCC0);
  TCC0->PER.bit.PER = 48000000 / 1000;
  TCC0->CC[3].bit.CC = 48000000 / 2000;
  TCC0->CTRLA.bit.ENABLE = true;

  gpio_set_pin_function(PIN_PA19, PINMUX_PA19F_TCC0_WO3);
  _gclk_enable_channel(TCC0_GCLK_ID, GCLK_CLKCTRL_GEN_GCLK0_Val);

#if CFG_TUH_ENABLED && defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
  max3421_init();
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  (void) state;
#ifdef LED_PIN
  gpio_set_pin_level(LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
#endif
}

uint32_t board_button_read(void) {
#ifdef BUTTON_PIN
  return BUTTON_STATE_ACTIVE == gpio_get_pin_level(BUTTON_PIN);
#else
  return 0;
#endif
}

#if defined(UART_SERCOM)

#define BOARD_SERCOM2(n)  SERCOM ## n
#define BOARD_SERCOM(n) BOARD_SERCOM2(n)

static void uart_init(void)
{
#if UART_SERCOM == 0
  #if UART_TX_PIN == 6
    gpio_set_pin_function(PIN_PA06, PINMUX_PA06D_SERCOM0_PAD2);
  #elif UART_TX_PIN == 10
    gpio_set_pin_function(PIN_PA10, PINMUX_PA10C_SERCOM0_PAD2);
  #else
    #error "UART_TX_PIN not supported"
  #endif

  #if UART_RX_PIN == 7
    gpio_set_pin_function(PIN_PA07, PINMUX_PA07D_SERCOM0_PAD3);
  #elif UART_RX_PIN == 11
    gpio_set_pin_function(PIN_PA11, PINMUX_PA11C_SERCOM0_PAD3);
  #else
    #error "UART_RX_PIN not supported"
#endif

  // setup clock (48MHz)
  _pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
  _gclk_enable_channel(SERCOM0_GCLK_ID_CORE, GCLK_CLKCTRL_GEN_GCLK0_Val);

  SERCOM0->USART.CTRLA.bit.SWRST = 1; /* reset SERCOM & enable config */
  while(SERCOM0->USART.SYNCBUSY.bit.SWRST);

  SERCOM0->USART.CTRLA.reg  =  /* CMODE = 0 -> async, SAMPA = 0, FORM = 0 -> USART frame, SMPR = 0 -> arithmetic baud rate */
    SERCOM_USART_CTRLA_SAMPR(1) | /* 0 = 16x / arithmetic baud rate, 1 = 16x / fractional baud rate */
//    SERCOM_USART_CTRLA_FORM(0) | /* 0 = USART Frame, 2 = LIN Master */
    SERCOM_USART_CTRLA_DORD | /* LSB first */
    SERCOM_USART_CTRLA_MODE(1) | /* 0 = Asynchronous, 1 = USART with internal clock */
    SERCOM_USART_CTRLA_RXPO(3) | /* pad 3 */
    SERCOM_USART_CTRLA_TXPO(1);  /* pad 2 */

  SERCOM0->USART.CTRLB.reg =
    SERCOM_USART_CTRLB_TXEN | /* tx enabled */
    SERCOM_USART_CTRLB_RXEN;  /* rx enabled */

  /* 115200 */
  SERCOM0->USART.BAUD.reg = SERCOM_USART_BAUD_FRAC_FP(0) | SERCOM_USART_BAUD_FRAC_BAUD(26);

  SERCOM0->USART.CTRLA.bit.ENABLE = 1; /* activate SERCOM */
  while(SERCOM0->USART.SYNCBUSY.bit.ENABLE); /* wait for SERCOM to be ready */
#endif
}

static inline void uart_send_buffer(uint8_t const *text, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    BOARD_SERCOM(UART_SERCOM)->USART.DATA.reg = text[i];
    while((BOARD_SERCOM(UART_SERCOM)->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
  }
}

static inline void uart_send_str(const char* text)
{
  while (*text) {
    BOARD_SERCOM(UART_SERCOM)->USART.DATA.reg = *text++;
    while((BOARD_SERCOM(UART_SERCOM)->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC) == 0);
  }
}

int board_uart_read(uint8_t* buf, int len)
{
  (void) buf; (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
  if (len < 0) {
    uart_send_str(buf);
  } else {
    uart_send_buffer(buf, len);
  }
  return len;
}

#else // ! defined(UART_SERCOM)

static void uart_init(void) {
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

#endif

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}

#endif

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
#if CFG_TUH_ENABLED && defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421

static void max3421_init(void) {
  //------------- SPI Init -------------//
  // MAX3421E max SPI clock is 26MHz however SAMD can only work reliably at 12 Mhz
  uint32_t const baudrate = 12000000u;

  // Enable the APB clock for SERCOM
  PM->APBCMASK.reg |= 1u << (PM_APBCMASK_SERCOM0_Pos + MAX3421_SERCOM_ID);

  // Configure GCLK for SERCOM
//  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM4_CORE | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM0_CORE_Val + MAX3421_SERCOM_ID) |
                      GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  Sercom* sercom = MAX3421_SERCOM;

  // Disable the SPI module
  sercom->SPI.CTRLA.bit.ENABLE = 0;

  // Reset the SPI module
  sercom->SPI.CTRLA.bit.SWRST = 1;
  while (sercom->SPI.SYNCBUSY.bit.SWRST);

  // Set up SPI in master mode, MSB first, SPI mode 0
  sercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_DOPO(MAX3421_TX_PAD) | SERCOM_SPI_CTRLA_DIPO(MAX3421_RX_PAD) |
                          SERCOM_SPI_CTRLA_MODE(3);

  sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(0) | SERCOM_SPI_CTRLB_RXEN;
  while (sercom->SPI.SYNCBUSY.bit.CTRLB == 1);

  // Set the baud rate
  sercom->SPI.BAUD.reg = (uint8_t) (SystemCoreClock / (2 * baudrate) - 1);

  // Configure PA12 as MOSI (PAD0), PA13 as SCK (PAD1), PA14 as MISO (PAD2), function C (sercom)
  gpio_set_pin_direction(MAX3421_SCK_PIN, GPIO_DIRECTION_OUT);
  gpio_set_pin_pull_mode(MAX3421_SCK_PIN, GPIO_PULL_OFF);
  gpio_set_pin_function(MAX3421_SCK_PIN, MAX3421_SERCOM_FUNCTION);

  gpio_set_pin_direction(MAX3421_MOSI_PIN, GPIO_DIRECTION_OUT);
  gpio_set_pin_pull_mode(MAX3421_MOSI_PIN, GPIO_PULL_OFF);
  gpio_set_pin_function(MAX3421_MOSI_PIN, MAX3421_SERCOM_FUNCTION);

  gpio_set_pin_direction(MAX3421_MISO_PIN, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(MAX3421_MISO_PIN, GPIO_PULL_OFF);
  gpio_set_pin_function(MAX3421_MISO_PIN, MAX3421_SERCOM_FUNCTION);

  // CS pin
  gpio_set_pin_direction(MAX3421_CS_PIN, GPIO_DIRECTION_OUT);
  gpio_set_pin_level(MAX3421_CS_PIN, 1);

  // Enable the SPI module
  sercom->SPI.CTRLA.bit.ENABLE = 1;
  while (sercom->SPI.SYNCBUSY.bit.ENABLE);

  //------------- External Interrupt -------------//

  // Enable the APB clock for EIC (External Interrupt Controller)
  PM->APBAMASK.reg |= PM_APBAMASK_EIC;

  // Configure GCLK for EIC
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EIC | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Configure PA20 as an input with function A (external interrupt)
  gpio_set_pin_direction(MAX3421_INTR_PIN, GPIO_DIRECTION_IN);
  gpio_set_pin_pull_mode(MAX3421_INTR_PIN, GPIO_PULL_UP);
  gpio_set_pin_function(MAX3421_INTR_PIN, 0);

  // Disable EIC
  EIC->CTRL.bit.ENABLE = 0;
  while (EIC->STATUS.bit.SYNCBUSY);

  // Configure EIC to trigger on falling edge
  uint8_t const sense_shift = MAX3421_INTR_EIC_ID * 4;
  EIC->CONFIG[0].reg &= ~(7 << sense_shift);
  EIC->CONFIG[0].reg |= 2 << sense_shift;

#if CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(EIC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

  // Enable External Interrupt
  EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << MAX3421_INTR_EIC_ID);

  // Enable EIC
  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY);
}

void EIC_Handler(void) {
  // Clear the interrupt flag
  EIC->INTFLAG.reg = EIC_INTFLAG_EXTINT(1 << MAX3421_INTR_EIC_ID);

  // Call the TinyUSB interrupt handler
  tuh_int_handler(1, true);
}

// API to enable/disable MAX3421 INTR pin interrupt
void tuh_max3421_int_api(uint8_t rhport, bool enabled) {
  (void) rhport;

  if (enabled) {
    NVIC_EnableIRQ(EIC_IRQn);
  } else {
    NVIC_DisableIRQ(EIC_IRQn);
  }
}

// API to control MAX3421 SPI CS
void tuh_max3421_spi_cs_api(uint8_t rhport, bool active) {
  (void) rhport;
  gpio_set_pin_level(MAX3421_CS_PIN, active ? 0 : 1);
}

// API to transfer data with MAX3421 SPI
// Either tx_buf or rx_buf can be NULL, which means transfer is write or read only
bool tuh_max3421_spi_xfer_api(uint8_t rhport, uint8_t const* tx_buf, uint8_t* rx_buf, size_t xfer_bytes) {
  (void) rhport;

  Sercom* sercom = MAX3421_SERCOM;

  for (size_t count = 0; count < xfer_bytes; count++) {
    // Wait for the transmit buffer to be empty
    while (!sercom->SPI.INTFLAG.bit.DRE);

    // Write data to be transmitted
    uint8_t data = 0x00;
    if (tx_buf) {
      data = tx_buf[count];
    }

    sercom->SPI.DATA.reg = (uint32_t) data;

    // Wait for the receive buffer to be filled
    while (!sercom->SPI.INTFLAG.bit.RXC);

    // Read received data
    data = (uint8_t) sercom->SPI.DATA.reg;
    if (rx_buf) {
      rx_buf[count] = data;
    }
  }

  // wait for bus idle and clear flags
  while (!(sercom->SPI.INTFLAG.reg & (SERCOM_SPI_INTFLAG_TXC | SERCOM_SPI_INTFLAG_DRE)));
  sercom->SPI.INTFLAG.reg = SERCOM_SPI_INTFLAG_TXC | SERCOM_SPI_INTFLAG_DRE;

  return true;
}

#endif
