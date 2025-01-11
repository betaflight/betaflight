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

// Suppress warning caused by mcu driver
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wundef"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#endif

#include "nrfx.h"
#include "hal/nrf_gpio.h"
#include "drivers/include/nrfx_gpiote.h"
#include "drivers/include/nrfx_power.h"
#include "drivers/include/nrfx_uarte.h"
#include "drivers/include/nrfx_spim.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif


// There is API changes between nrfx v2 and v3
#if 85301 >= (10000*MDK_MAJOR_VERSION + 100*MDK_MINOR_VERSION + MDK_MICRO_VERSION)
  // note MDK 8.53.1 is also used by nrfx v3.0.0, just skip this version and use later 3.x
  #define NRFX_VER 2
#else
  #define NRFX_VER 3
#endif

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USBD_IRQHandler(void) {
  tud_int_handler(0);
}

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/

// Value is chosen to be as same as NRFX_POWER_USB_EVT_* in nrfx_power.h
enum {
  USB_EVT_DETECTED = 0,
  USB_EVT_REMOVED = 1,
  USB_EVT_READY = 2
};

#ifdef NRF5340_XXAA
  #define LFCLK_SRC_RC CLOCK_LFCLKSRC_SRC_LFRC
  #define VBUSDETECT_Msk USBREG_USBREGSTATUS_VBUSDETECT_Msk
  #define OUTPUTRDY_Msk USBREG_USBREGSTATUS_OUTPUTRDY_Msk
  #define GPIOTE_IRQn GPIOTE1_IRQn
#else
  #define LFCLK_SRC_RC CLOCK_LFCLKSRC_SRC_RC
  #define VBUSDETECT_Msk POWER_USBREGSTATUS_VBUSDETECT_Msk
  #define OUTPUTRDY_Msk POWER_USBREGSTATUS_OUTPUTRDY_Msk
#endif

static nrfx_uarte_t _uart_id = NRFX_UARTE_INSTANCE(0);

// tinyusb function that handles power event (detected, ready, removed)
// We must call it within SD's SOC event handler, or set it as power event handler if SD is not enabled.
extern void tusb_hal_nrf_power_event(uint32_t event);

// nrf power callback, could be unused if SD is enabled or usb is disabled (board_test example)
TU_ATTR_UNUSED static void power_event_handler(nrfx_power_usb_evt_t event) {
  tusb_hal_nrf_power_event((uint32_t) event);
}

//------------- Host using MAX2341E -------------//
#if CFG_TUH_ENABLED && defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
static void max3421_init(void);
static nrfx_spim_t _spi = NRFX_SPIM_INSTANCE(1);

#if NRFX_VER > 2
static nrfx_gpiote_t _gpiote = NRFX_GPIOTE_INSTANCE(0);
#endif

#endif

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

void board_init(void) {
  // stop LF clock just in case we jump from application without reset
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // Use Internal OSC to compatible with all boards
  NRF_CLOCK->LFCLKSRC = LFCLK_SRC_RC;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

  // LED
  nrf_gpio_cfg_output(LED_PIN);
  board_led_write(false);

  // Button
  nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);

  // UART
  #if NRFX_VER <= 2
  nrfx_uarte_config_t uart_cfg = {
      .pseltxd   = UART_TX_PIN,
      .pselrxd   = UART_RX_PIN,
      .pselcts   = NRF_UARTE_PSEL_DISCONNECTED,
      .pselrts   = NRF_UARTE_PSEL_DISCONNECTED,
      .p_context = NULL,
      .baudrate  = NRF_UARTE_BAUDRATE_115200, // CFG_BOARD_UART_BAUDRATE
      .interrupt_priority = 7,
      .hal_cfg = {
          .hwfc      = NRF_UARTE_HWFC_DISABLED,
          .parity    = NRF_UARTE_PARITY_EXCLUDED,
      }
  };
  #else
  nrfx_uarte_config_t uart_cfg = {
      .txd_pin   = UART_TX_PIN,
      .rxd_pin   = UART_RX_PIN,
      .rts_pin   = NRF_UARTE_PSEL_DISCONNECTED,
      .cts_pin   = NRF_UARTE_PSEL_DISCONNECTED,
      .p_context = NULL,
      .baudrate  = NRF_UARTE_BAUDRATE_115200, // CFG_BOARD_UART_BAUDRATE
      .interrupt_priority = 7,
      .config = {
          .hwfc      = NRF_UARTE_HWFC_DISABLED,
          .parity    = NRF_UARTE_PARITY_EXCLUDED,
      }
  };
  #endif

  nrfx_uarte_init(&_uart_id, &uart_cfg, NULL); //uart_handler);

  //------------- USB -------------//
#if CFG_TUD_ENABLED
  // Priorities 0, 1, 4 (nRF52) are reserved for SoftDevice
  // 2 is highest for application
  NVIC_SetPriority(USBD_IRQn, 2);

  // USB power may already be ready at this time -> no event generated
  // We need to invoke the handler based on the status initially
  uint32_t usb_reg;

#ifdef SOFTDEVICE_PRESENT
  uint8_t sd_en = false;
  sd_softdevice_is_enabled(&sd_en);

  if ( sd_en ) {
    sd_power_usbdetected_enable(true);
    sd_power_usbpwrrdy_enable(true);
    sd_power_usbremoved_enable(true);

    sd_power_usbregstatus_get(&usb_reg);
  }else
#endif
  {
    // Power module init
    const nrfx_power_config_t pwr_cfg = {0};
    nrfx_power_init(&pwr_cfg);

    // Register tusb function as USB power handler
    // cause cast-function-type warning
    const nrfx_power_usbevt_config_t config = {.handler = power_event_handler};
    nrfx_power_usbevt_init(&config);
    nrfx_power_usbevt_enable();

    // USB power may already be ready at this time -> no event generated
    // We need to invoke the handler based on the status initially
#ifdef NRF5340_XXAA
    usb_reg = NRF_USBREGULATOR->USBREGSTATUS;
#else
    usb_reg = NRF_POWER->USBREGSTATUS;
#endif
  }

  if ( usb_reg & VBUSDETECT_Msk ) tusb_hal_nrf_power_event(USB_EVT_DETECTED);
  if ( usb_reg & OUTPUTRDY_Msk  ) tusb_hal_nrf_power_event(USB_EVT_READY);
#endif

#if CFG_TUH_ENABLED && defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
  max3421_init();
#endif

}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state) {
  nrf_gpio_pin_write(LED_PIN, state ? LED_STATE_ON : (1 - LED_STATE_ON));
}

uint32_t board_button_read(void) {
  return BUTTON_STATE_ACTIVE == nrf_gpio_pin_read(BUTTON_PIN);
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;

#ifdef NRF5340_XXAA
  uintptr_t did_addr = (uintptr_t) NRF_FICR->INFO.DEVICEID;
#else
  uintptr_t did_addr = (uintptr_t) NRF_FICR->DEVICEID;
#endif

  const uint8_t* device_id = (const uint8_t*) did_addr;
  for(uint8_t i=0; i<8; i++) {
    id[i] = device_id[i];
  }
  return 8;
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
//  nrfx_err_t err = nrfx_uarte_rx(&_uart_id, buf, (size_t) len);
//  return NRFX_SUCCESS == err ? len : 0;
}

int board_uart_write(void const* buf, int len) {
  nrfx_err_t err = nrfx_uarte_tx(&_uart_id, (uint8_t const*) buf, (size_t) len
                                 #if NRFX_VER > 2
                                 ,0
                                 #endif
                                );
  return (NRFX_SUCCESS == err) ? len : 0;
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

#ifndef __ICCARM__
// Implement _start() since we use linker flag '-nostartfiles'.
// Requires defined __STARTUP_CLEAR_BSS,
extern int main(void);
TU_ATTR_UNUSED void _start(void) {
  // called by startup code
  main();
  while (1) {}
}
#endif

//--------------------------------------------------------------------+
// Softdevice running
//--------------------------------------------------------------------+
#ifdef SOFTDEVICE_PRESENT
// process SOC event from SD
uint32_t proc_soc(void) {
  uint32_t soc_evt;
  uint32_t err = sd_evt_get(&soc_evt);

  if (NRF_SUCCESS == err)
  {
    /*------------- usb power event handler -------------*/
    int32_t usbevt = (soc_evt == NRF_EVT_POWER_USB_DETECTED   ) ? NRFX_POWER_USB_EVT_DETECTED:
                     (soc_evt == NRF_EVT_POWER_USB_POWER_READY) ? NRFX_POWER_USB_EVT_READY   :
                     (soc_evt == NRF_EVT_POWER_USB_REMOVED    ) ? NRFX_POWER_USB_EVT_REMOVED : -1;

    if ( usbevt >= 0) tusb_hal_nrf_power_event(usbevt);
  }

  return err;
}

uint32_t proc_ble(void) {
  // do nothing with ble
  return NRF_ERROR_NOT_FOUND;
}

void SD_EVT_IRQHandler(void) {
  // process BLE and SOC until there is no more events
  while( (NRF_ERROR_NOT_FOUND != proc_ble()) || (NRF_ERROR_NOT_FOUND != proc_soc()) ) {
  }
}

void nrf_error_cb(uint32_t id, uint32_t pc, uint32_t info) {
  (void) id;
  (void) pc;
  (void) info;
}
#endif

//--------------------------------------------------------------------+
// API: SPI transfer with MAX3421E, must be implemented by application
//--------------------------------------------------------------------+
#if CFG_TUH_ENABLED && defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421

#if NRFX_VER <= 2
void max3421_int_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action ) {
  if (action != NRF_GPIOTE_POLARITY_HITOLO) return;
#else
void max3421_int_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t action, void* p_context) {
  (void) p_context;
  if (action != NRFX_GPIOTE_TRIGGER_HITOLO) return;
#endif

  if (pin != MAX3421_INTR_PIN) return;
  tuh_int_handler(1, true);
}

static void max3421_init(void) {
  // Somehow pca10056/95 is not working probably due to signal incompatible (1.8V 3v3) with MAC3421E !?

  // manually manage CS
  nrf_gpio_cfg_output(MAX3421_CS_PIN);
  nrf_gpio_pin_write(MAX3421_CS_PIN, 1);

  // USB host using max3421e usb controller via SPI
  nrfx_spim_config_t cfg = {
      .sck_pin        = MAX3421_SCK_PIN,
      .mosi_pin       = MAX3421_MOSI_PIN,
      .miso_pin       = MAX3421_MISO_PIN,
  #if NRFX_VER <= 2
      .ss_pin         = NRFX_SPIM_PIN_NOT_USED,
      .frequency      = NRF_SPIM_FREQ_4M,
  #else
      .ss_pin         = NRF_SPIM_PIN_NOT_CONNECTED,
      .frequency      = 4000000u,
  #endif
      .ss_active_high = false,
      .irq_priority   = 3,
      .orc            = 0xFF,
      // default setting 4 Mhz, Mode 0, MSB first
      .mode           = NRF_SPIM_MODE_0,
      .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,
      .miso_pull      = NRF_GPIO_PIN_NOPULL,
  };

  // no handler --> blocking
  TU_ASSERT(NRFX_SUCCESS == nrfx_spim_init(&_spi, &cfg, NULL, NULL), );

  // max3421e interrupt pin
  #if NRFX_VER <= 2
  nrfx_gpiote_init(1);
  nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
  NVIC_SetPriority(GPIOTE_IRQn, 2);
  nrfx_gpiote_in_init(MAX3421_INTR_PIN, &in_config, max3421_int_handler);
  nrfx_gpiote_trigger_enable(MAX3421_INTR_PIN, true);
  #else
  nrf_gpio_pin_pull_t intr_pull = NRF_GPIO_PIN_PULLUP;
  nrfx_gpiote_trigger_config_t intr_trigger = {
      .trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
      .p_in_channel = NULL, // sensing mechanism
  };
  nrfx_gpiote_handler_config_t intr_handler = {
      .handler = max3421_int_handler,
      .p_context = NULL,
  };
  nrfx_gpiote_input_pin_config_t intr_config = {
      .p_pull_config = &intr_pull,
      .p_trigger_config = &intr_trigger,
      .p_handler_config = &intr_handler,
  };

  nrfx_gpiote_init(&_gpiote, 1);
  NVIC_SetPriority(GPIOTE_IRQn, 2);

  nrfx_gpiote_input_configure(&_gpiote, MAX3421_INTR_PIN, &intr_config);
  nrfx_gpiote_trigger_enable(&_gpiote, MAX3421_INTR_PIN, true);
  #endif
}

// API to enable/disable MAX3421 INTR pin interrupt
void tuh_max3421_int_api(uint8_t rhport, bool enabled) {
  (void) rhport;

  // use NVIC_Enable/Disable instead since nrfx_gpiote_trigger_enable/disable clear pending and can miss interrupt
  // when disabled and re-enabled.
  if (enabled) {
    NVIC_EnableIRQ(GPIOTE_IRQn);
  } else {
    NVIC_DisableIRQ(GPIOTE_IRQn);
  }
}

// API to control MAX3421 SPI CS
void tuh_max3421_spi_cs_api(uint8_t rhport, bool active) {
  (void) rhport;
  nrf_gpio_pin_write(MAX3421_CS_PIN, active ? 0 : 1);
}

// API to transfer data with MAX3421 SPI
// Either tx_buf or rx_buf can be NULL, which means transfer is write or read only
bool tuh_max3421_spi_xfer_api(uint8_t rhport, uint8_t const* tx_buf, uint8_t* rx_buf, size_t xfer_bytes) {
  (void) rhport;

  nrfx_spim_xfer_desc_t xfer = {
      .p_tx_buffer = tx_buf,
      .tx_length   = tx_buf ? xfer_bytes : 0,
      .p_rx_buffer = rx_buf,
      .rx_length   = rx_buf ? xfer_bytes : 0,
  };

  return nrfx_spim_xfer(&_spi, &xfer, 0) == NRFX_SUCCESS;
}

#endif
