/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 MM32 SE TEAM
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

#include "hal_conf.h"
#include "mm32_device.h"

#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#ifdef __GNUC__ // caused by extra declaration of SystemCoreClock in freeRTOSConfig.h
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
#endif

extern u32 SystemCoreClock;

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void OTG_FS_IRQHandler(void) {
  tud_int_handler(0);
}

void USB_DeviceClockInit(void) {
  /* Select USBCLK source */
  //  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
  RCC->CFGR &= ~(0x3 << 22);
  RCC->CFGR |= (0x1 << 22);

  /* Enable USB clock */
  RCC->AHB2ENR |= 0x1 << 7;
}

void board_init(void) {
//   usb clock
  USB_DeviceClockInit();

  SysTick_Config(SystemCoreClock / 1000);
  NVIC_SetPriority(SysTick_IRQn, 0x0);

  RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);

  // LED
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  board_led_write(true);

  #ifdef BUTTON_PORT
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = BUTTON_PIN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStruct.GPIO_Mode = BUTTON_STATE_ACTIVE ? GPIO_Mode_IPD : GPIO_Mode_IPU;
  GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
  #endif

  #ifdef UART_DEV
  // UART
  UART_InitTypeDef UART_InitStruct;

  RCC_APB2PeriphClockCmd(RCC_APB2ENR_UART1, ENABLE);  //enableUART1,GPIOAclock
  GPIO_PinAFConfig(GPIOA, UART_TX_PIN, UART_GPIO_AF);
  GPIO_PinAFConfig(GPIOA, UART_RX_PIN, UART_GPIO_AF);

  UART_StructInit(&UART_InitStruct);
  UART_InitStruct.UART_BaudRate = CFG_BOARD_UART_BAUDRATE;
  UART_InitStruct.UART_WordLength = UART_WordLength_8b;
  UART_InitStruct.UART_StopBits = UART_StopBits_1;
  UART_InitStruct.UART_Parity = UART_Parity_No;
  UART_InitStruct.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_InitStruct.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;

  UART_Init(UART_DEV, &UART_InitStruct);
  UART_Cmd(UART_DEV, ENABLE);

  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = 1 << UART_TX_PIN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = 1 << UART_RX_PIN;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);
  #endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  GPIO_WriteBit(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
}

uint32_t board_button_read(void) {
#ifdef BUTTON_PORT
  return GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN) == BUTTON_STATE_ACTIVE;
#else
  return 0;
#endif
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  #ifdef UART_DEV
  const char* buff = buf;
  while (len) {
    while ((UART1->CSR & UART_IT_TXIEN) == 0);    //The loop is sent until it is finished
    UART1->TDR = (*buff & 0xFF);
    buff++;
    len--;
  }
  return len;
  #else
  (void) buf;
  (void) len;
  return 0;
  #endif
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}
#endif

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {
}
