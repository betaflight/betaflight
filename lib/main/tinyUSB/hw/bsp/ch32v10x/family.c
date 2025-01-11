#include <stdio.h>

// https://github.com/openwch/ch32v307/pull/90
// https://github.com/openwch/ch32v20x/pull/12
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif

#include "ch32v10x.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "bsp/board_api.h"
#include "board.h"

__attribute__((interrupt)) __attribute__((used))
void USBHD_IRQHandler(void) {
  #if CFG_TUD_WCH_USBIP_USBFS
  tud_int_handler(0);
  #endif
}

__attribute__((interrupt)) __attribute__((used))
void USBWakeUp_IRQHandler(void) {
  #if CFG_TUD_WCH_USBIP_USBFS
  tud_int_handler(0);
  #endif
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

__attribute__((interrupt)) __attribute__((used))
void SysTick_Handler(void) {
  SysTick->CNTL0 = SysTick->CNTL1 = SysTick->CNTL2 = SysTick->CNTL3 = 0;
  SysTick->CNTH0 = SysTick->CNTH1 = SysTick->CNTH2 = SysTick->CNTH3 = 0;
  system_ticks++;
}

uint32_t SysTick_Config(uint32_t ticks) {
  NVIC_EnableIRQ(SysTicK_IRQn);
  SysTick->CTLR = 0;
  SysTick->CNTL0 = SysTick->CNTL1 = SysTick->CNTL2 = SysTick->CNTL3 = 0;
  SysTick->CNTH0 = SysTick->CNTH1 = SysTick->CNTH2 = SysTick->CNTH3 = 0;

  SysTick->CMPLR0 = (u8)(ticks & 0xFF);
  SysTick->CMPLR1 = (u8)(ticks >> 8);
  SysTick->CMPLR2 = (u8)(ticks >> 16);
  SysTick->CMPLR3 = (u8)(ticks >> 24);

  SysTick->CMPHR0 = SysTick->CMPHR1 = SysTick->CMPHR2 =   SysTick->CMPHR3 = 0;
  SysTick->CTLR = 1;
  return 0;
}

uint32_t board_millis(void) {
  return system_ticks;
}
#endif

void board_init(void) {
  __disable_irq();

#if CFG_TUSB_OS == OPT_OS_NONE
  SysTick_Config(SystemCoreClock / 1000);
#endif

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  EXTEN->EXTEN_CTR |= EXTEN_USBFS_IO_EN;
  uint8_t usb_div;
  switch (SystemCoreClock) {
    case 48000000: usb_div = RCC_USBCLKSource_PLLCLK_Div1; break;
    case 72000000: usb_div = RCC_USBCLKSource_PLLCLK_1Div5; break;
    default: TU_ASSERT(0,); break;
  }
  RCC_USBCLKConfig(usb_div);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);

  #ifdef LED_PIN
  GPIO_InitTypeDef led_init = {
      .GPIO_Pin = LED_PIN,
      .GPIO_Mode = GPIO_Mode_Out_OD,
      .GPIO_Speed = GPIO_Speed_50MHz,
  };
  GPIO_Init(LED_PORT, &led_init);
  #endif

  #ifdef BUTTON_PIN
  GPIO_InitTypeDef button_init = {
      .GPIO_Pin = BUTTON_PIN,
      .GPIO_Mode = GPIO_Mode_IPU,
      .GPIO_Speed = GPIO_Speed_50MHz,
  };
  GPIO_Init(BUTTON_PORT, &button_init);
  #endif

  // UART TX is PA9
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  GPIO_InitTypeDef usart_init = {
    .GPIO_Pin = GPIO_Pin_9,
    .GPIO_Speed = GPIO_Speed_50MHz,
    .GPIO_Mode = GPIO_Mode_AF_PP,
  };
  GPIO_Init(GPIOA, &usart_init);

  USART_InitTypeDef usart = {
    .USART_BaudRate = 115200,
    .USART_WordLength = USART_WordLength_8b,
    .USART_StopBits = USART_StopBits_1,
    .USART_Parity = USART_Parity_No,
    .USART_Mode = USART_Mode_Tx,
    .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
  };
  USART_Init(USART1, &usart);
  USART_Cmd(USART1, ENABLE);

  __enable_irq();

  board_led_write(true);
}

void board_led_write(bool state) {
  GPIO_WriteBit(LED_PORT, LED_PIN, state ? LED_STATE_ON : (1-LED_STATE_ON));
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN);
}

int board_uart_read(uint8_t *buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const *buf, int len) {
  const char *bufc = (const char *) buf;
  for (int i = 0; i < len; i++) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    USART_SendData(USART1, *bufc++);
  }

  return len;
}
