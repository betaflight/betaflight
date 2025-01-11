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

#include "drv_usb_hw.h"
#include "drv_usb_dev.h"

#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

void USBFS_IRQHandler(void) { tud_int_handler(0); }

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+

#define USB_NO_VBUS_PIN

// According to GD32VF103 user manual clock tree:
// Systick clock = AHB clock / 4.
#define TIMER_TICKS         ((SystemCoreClock / 4) / 1000)

#define BUTTON_PORT         GPIOA
#define BUTTON_PIN          GPIO_PIN_0
#define BUTTON_STATE_ACTIVE 1

#define UART_DEV            SOC_DEBUG_UART

#define LED_PIN             LED_R

void board_init(void) {
  /* Disable interrupts during init */
  __disable_irq();

#if CFG_TUSB_OS == OPT_OS_NONE
  SysTick_Config(TIMER_TICKS);
#endif

  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_GPIOD);
  rcu_periph_clock_enable(RCU_AF);

#ifdef BUTTON_PIN
  gpio_init(BUTTON_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, BUTTON_PIN);
#endif

#ifdef LED_PIN
  gd_led_init(LED_PIN);
#endif

#if defined(UART_DEV)
  gd_com_init(UART_DEV);
#endif

  /* USB D+ and D- pins don't need to be configured. */
  /* Configure VBUS Pin */
#ifndef USB_NO_VBUS_PIN
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
#endif

  /* This for ID line debug */
  // gpio_init(GPIOA, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

  /* Enable USB OTG clock */
  usb_rcu_config();

  /* Reset USB OTG peripheral */
  rcu_periph_reset_enable(RCU_USBFSRST);
  rcu_periph_reset_disable(RCU_USBFSRST);

  /* Configure USBFS IRQ */
  ECLIC_Register_IRQ(USBFS_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                     ECLIC_POSTIVE_EDGE_TRIGGER, 3, 0, NULL);

  /* Retrieve otg core registers */
  usb_gr* otg_core_regs = (usb_gr*)(USBFS_REG_BASE + USB_REG_OFFSET_CORE);

#ifdef USB_NO_VBUS_PIN
  /* Disable VBUS sense*/
  otg_core_regs->GCCFG |= GCCFG_VBUSIG | GCCFG_PWRON | GCCFG_VBUSBCEN;
#else
  /* Enable VBUS sense via pin PA9 */
  otg_core_regs->GCCFG |= GCCFG_VBUSIG | GCCFG_PWRON | GCCFG_VBUSBCEN;
  otg_core_regs->GCCFG &= ~GCCFG_VBUSIG;
#endif

  /* Enable interrupts globally */
  __enable_irq();
}

void gd32vf103_reset(void) {
  /* The MTIMER unit of the GD32VF103 doesn't have the MSFRST
   * register to generate a software reset request.
   * BUT instead two undocumented registers in the debug peripheral
   * that allow issuing a software reset.
   * https://github.com/esmil/gd32vf103inator/blob/master/include/gd32vf103/dbg.h
   */
  DBG_KEY = DBG_KEY_UNLOCK;
  DBG_CMD = DBG_CMD_RESET;
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  state ? gd_led_on(LED_PIN) : gd_led_off(LED_PIN);
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN);
}

int board_uart_read(uint8_t* buf, int len) {
#if defined(UART_DEV)
  int rxsize = len;
  while (rxsize--) {
    *(uint8_t*)buf = usart_read(UART_DEV);
    buf++;
  }
  return len;
#else
  (void)buf;
  (void)len;
  return 0;
#endif
}

int board_uart_write(void const* buf, int len) {
#if defined(UART_DEV)
  int txsize = len;
  while (txsize--) {
    usart_write(UART_DEV, *(uint8_t const*)buf);
    buf++;
  }
  return len;
#else
  (void)buf;
  (void)len;
  return 0;
#endif
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void eclic_mtip_handler(void) {
  system_ticks++;
  SysTick_Reload(TIMER_TICKS);
}
uint32_t board_millis(void) { return system_ticks; }
#endif

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(char* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
