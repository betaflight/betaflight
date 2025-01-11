#include "TM4C123.h"
#include "bsp/board_api.h"
#include "board.h"

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB0_Handler(void) {
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

static void board_uart_init(void) {
  SYSCTL->RCGCUART |= (1 << 0);                // Enable the clock to UART0
  SYSCTL->RCGCGPIO |= (1 << 0);                // Enable the clock to GPIOA

  GPIOA->AFSEL |= (1 << 1) | (1 << 0);         // Enable the alternate function on pin PA0 & PA1
  GPIOA->PCTL |= (1 << 0) | (1 << 4);          // Configure the GPIOPCTL register to select UART0 in PA0 and PA1
  GPIOA->DEN |= (1 << 0) | (1 << 1);           // Enable the digital functionality in PA0 and PA1

  // BAUDRATE = 115200, with SystemCoreClock = 50 Mhz refer manual for calculation
  //  - BRDI = SystemCoreClock / (16* baud)
  //  - BRDF = int(fraction*64 + 0.5)
  UART0->CTL &= ~(1 << 0);                     // Disable UART0 by clearing UARTEN bit in the UARTCTL register
  UART0->IBRD = 27;                            // Write the integer portion of the BRD to the UARTIRD register
  UART0->FBRD = 8;                             // Write the fractional portion of the BRD to the UARTFBRD registerer

  UART0->LCRH = (0x3 << 5);                    // 8-bit, no parity, 1 stop bit
  UART0->CC = 0x0;                             // Configure the UART clock source as system clock

  UART0->CTL = (1 << 0) | (1 << 8) | (1 << 9); // UART0 Enable, Transmit Enable, Receive Enable
}

static void initialize_board_led(GPIOA_Type* port, uint8_t PinMsk, uint8_t dirmsk) {
  /* Enable PortF Clock */
  SYSCTL->RCGCGPIO |= (1 << 5);

  /* Let the clock stabilize */
  while (!((SYSCTL->PRGPIO) & (1 << 5))) {}

  /* Port Digital Enable */
  port->DEN |= PinMsk;

  /* Set direction */
  port->DIR = dirmsk;
}

static void WriteGPIOPin(GPIOA_Type* port, uint8_t PinMsk, bool state) {
  if (state) {
    port->DATA |= PinMsk;
  } else {
    port->DATA &= ~(PinMsk);
  }
}

static uint32_t ReadGPIOPin(GPIOA_Type* port, uint8_t pinMsk) {
  return (port->DATA & pinMsk);
}

void board_init(void) {
  SystemCoreClockUpdate();

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  /* Reset USB */
  SYSCTL->SRCR2 |= (1u << 16);

  for (volatile uint8_t i = 0; i < 20; i++) {}

  SYSCTL->SRCR2 &= ~(1u << 16);

  /* Open the USB clock gate */
  SYSCTL->RCGCUSB |= (1 << 0);

  /* Power-up USB PLL */
  SYSCTL->RCC2 &= ~(1u << 14);

  /* USB IO Initialization */
  SYSCTL->RCGCGPIO |= (1u << 3);

  /* Let the clock stabilize */
  while (!(SYSCTL->PRGPIO & (1u << 3))) {}

  /* USB IOs to Analog Mode */
  GPIOD->AFSEL &= ~((1u << 4) | (1u << 5));
  GPIOD->DEN &= ~((1u << 4) | (1u << 5));
  GPIOD->AMSEL |= ((1u << 4) | (1u << 5));

  uint8_t leds = (1 << LED_PIN_RED) | (1 << LED_PIN_BLUE) | (1 << LED_PIN_GREEN);
  uint8_t dirmsk = (1 << LED_PIN_RED) | (1 << LED_PIN_BLUE) | (1 << LED_PIN_GREEN);

  /* Configure GPIO for board LED */
  initialize_board_led(LED_PORT, leds, dirmsk);

  /* Configure GPIO for board switch */
  GPIOF->DIR &= ~(1 << BOARD_BTN);
  GPIOF->PUR |= (1 << BOARD_BTN);
  GPIOF->DEN |= (1 << BOARD_BTN);

  /* Initialize board UART */
  board_uart_init();

  TU_LOG1_INT(SystemCoreClock);
}

void board_led_write(bool state) {
  WriteGPIOPin(LED_PORT, (1 << LED_PIN_BLUE), state);
}

uint32_t board_button_read(void) {
  uint32_t gpio_value = ReadGPIOPin(BOARD_BTN_PORT, BOARD_BTN_Msk);
  return BUTTON_STATE_ACTIVE ? gpio_value : !gpio_value;
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;
  uint8_t const len = 8;
  // Note: DID0, DID1 are variant ID, they aer used since TM4C123 does not have unique ID
  memcpy(id, (void*)(uintptr_t) &SYSCTL->DID0, len);
  return len;
}

int board_uart_write(void const* buf, int len) {
  uint8_t const* data = buf;

  for (int i = 0; i < len; i++) {
    while ((UART0->FR & (1 << 5)) != 0) {} // Poll until previous data was shofted out
    UART0->DR = data[i];                   // Write UART0 DATA REGISTER
  }

  return len;
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
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
