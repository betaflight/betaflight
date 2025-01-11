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

#include "stm32f4xx_hal.h"
#include "bsp/board_api.h"

typedef struct {
  GPIO_TypeDef* port;
  GPIO_InitTypeDef pin_init;
  uint8_t active_state;
} board_pindef_t;

#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void OTG_FS_IRQHandler(void) {
  tusb_int_handler(0, true);
}

void OTG_HS_IRQHandler(void) {
  tusb_int_handler(1, true);
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
UART_HandleTypeDef UartHandle = {
    .Instance = UART_DEV,
    .Init = {
      .BaudRate   = CFG_BOARD_UART_BAUDRATE,
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits   = UART_STOPBITS_1,
      .Parity     = UART_PARITY_NONE,
      .HwFlowCtl  = UART_HWCONTROL_NONE,
      .Mode       = UART_MODE_TX_RX,
      .OverSampling = UART_OVERSAMPLING_16
    }
};

void board_init(void) {
  board_clock_init();
  //SystemCoreClockUpdate();

  // Enable All GPIOs clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
#ifdef __HAL_RCC_GPIOE_CLK_ENABLE
  __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_GPIOF_CLK_ENABLE
  __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_GPIOG_CLK_ENABLE
  __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
  __HAL_RCC_GPIOH_CLK_ENABLE();
#ifdef __HAL_RCC_GPIOI_CLK_ENABLE
  __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
#ifdef __HAL_RCC_GPIOJ_CLK_ENABLE
  __HAL_RCC_GPIOJ_CLK_ENABLE();
#endif

  for (uint8_t i = 0; i < TU_ARRAY_SIZE(board_pindef); i++) {
    HAL_GPIO_Init(board_pindef[i].port, &board_pindef[i].pin_init);
  }

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // Explicitly disable systick to prevent its ISR runs before scheduler start
  SysTick->CTRL &= ~1U;

  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(OTG_FS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  board_led_write(false);

#ifdef UART_DEV
  HAL_UART_Init(&UartHandle);
#endif

  //------------- USB FS -------------//
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure USB D+ D- Pins */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure VBUS Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ID Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable USB OTG clock
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  //------------- USB HS -------------//
#ifdef __HAL_RCC_USB_OTG_HS_CLK_ENABLE
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure VBUS Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ID Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Enable USB OTG clock
  __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
#endif

#ifdef STM32F412Zx
  /* Configure POWER_SWITCH IO pin */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
#endif

#if CFG_TUD_ENABLED
  board_vbus_sense_init(BOARD_TUD_RHPORT);
#endif

#if CFG_TUH_ENABLED
  board_vbus_set(BOARD_TUD_RHPORT, true);
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
#ifdef PINID_LED
  board_pindef_t* pindef = &board_pindef[PINID_LED];
  GPIO_PinState pin_state = state == pindef->active_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(pindef->port, pindef->pin_init.Pin, pin_state);
#else
  (void) state;
#endif
}

uint32_t board_button_read(void) {
#ifdef PINID_BUTTON
  board_pindef_t* pindef = &board_pindef[PINID_BUTTON];
  return pindef->active_state == HAL_GPIO_ReadPin(pindef->port, pindef->pin_init.Pin);
#else
  return 0;
#endif
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;
  volatile uint32_t *stm32_uuid = (volatile uint32_t *) UID_BASE;
  uint32_t *id32 = (uint32_t *) (uintptr_t) id;
  uint8_t const len = 12;

  id32[0] = stm32_uuid[0];
  id32[1] = stm32_uuid[1];
  id32[2] = stm32_uuid[2];

  return len;
}

int board_uart_read(uint8_t *buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const *buf, int len) {
#ifdef UART_DEV
  HAL_UART_Transmit(&UartHandle, (uint8_t *) (uintptr_t) buf, len, 0xffff);
  return len;
#else
  (void) buf; (void) len; (void) UartHandle;
  return 0;
#endif
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  HAL_IncTick();
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}

#endif

void HardFault_Handler(void) {
  __asm("BKPT #0\n");
}

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {
}
