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

#ifndef BOARD_API_H_
#define BOARD_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "tusb.h"

#if CFG_TUSB_OS == OPT_OS_FREERTOS
#if TUSB_MCU_VENDOR_ESPRESSIF
  // ESP-IDF need "freertos/" prefix in include path.
  // CFG_TUSB_OS_INC_PATH should be defined accordingly.
  #include "freertos/FreeRTOS.h"
  #include "freertos/semphr.h"
  #include "freertos/queue.h"
  #include "freertos/task.h"
  #include "freertos/timers.h"
#else
  #include "FreeRTOS.h"
  #include "semphr.h"
  #include "queue.h"
  #include "task.h"
  #include "timers.h"
#endif
#endif

// Define the default baudrate
#ifndef CFG_BOARD_UART_BAUDRATE
#define CFG_BOARD_UART_BAUDRATE 115200   ///< Default baud rate
#endif

//--------------------------------------------------------------------+
// Board Porting API
// For simplicity, only one LED and one Button are used
//--------------------------------------------------------------------+

// Initialize on-board peripherals : led, button, uart and USB
void board_init(void);

// Init board after tinyusb is initialized
void board_init_after_tusb(void) TU_ATTR_WEAK;

// Jump to bootloader
void board_reset_to_bootloader(void) TU_ATTR_WEAK;

// Turn LED on or off
void board_led_write(bool state);

// Control led pattern using phase duration in ms.
// For each phase, LED is toggle then repeated, board_led_task() is required to be called
//void board_led_pattern(uint32_t const phase_ms[], uint8_t count);

// Get the current state of button
// a '1' means active (pressed), a '0' means inactive.
uint32_t board_button_read(void);

// Get board unique ID for USB serial number. Return number of bytes. Note max_len is typically 16
TU_ATTR_WEAK size_t board_get_unique_id(uint8_t id[], size_t max_len);

// Get characters from UART. Return number of read bytes
int board_uart_read(uint8_t *buf, int len);

// Send characters to UART. Return number of sent bytes
int board_uart_write(void const *buf, int len);

#if CFG_TUSB_OS == OPT_OS_NONE
// Get current milliseconds, must be implemented when no RTOS is used
uint32_t board_millis(void);

#elif CFG_TUSB_OS == OPT_OS_FREERTOS
static inline uint32_t board_millis(void) {
  return ( ( ((uint64_t) xTaskGetTickCount()) * 1000) / configTICK_RATE_HZ );
}

#elif CFG_TUSB_OS == OPT_OS_MYNEWT
static inline uint32_t board_millis(void) {
  return os_time_ticks_to_ms32( os_time_get() );
}

#elif CFG_TUSB_OS == OPT_OS_PICO
#include "pico/time.h"
static inline uint32_t board_millis(void) {
  return to_ms_since_boot(get_absolute_time());
}

#elif CFG_TUSB_OS == OPT_OS_RTTHREAD
static inline uint32_t board_millis(void) {
  return (((uint64_t)rt_tick_get()) * 1000 / RT_TICK_PER_SECOND);
}

#elif CFG_TUSB_OS == OPT_OS_CUSTOM
// Implement your own board_millis() in any of .c file
uint32_t board_millis(void);

#else
  #error "board_millis() is not implemented for this OS"
#endif

//--------------------------------------------------------------------+
// Helper functions
//--------------------------------------------------------------------+
static inline void board_led_on(void) {
  board_led_write(true);
}

static inline void board_led_off(void) {
  board_led_write(false);
}

// Get USB Serial number string from unique ID if available. Return number of character.
// Input is string descriptor from index 1 (index 0 is type + len)
static inline size_t board_usb_get_serial(uint16_t desc_str1[], size_t max_chars) {
  uint8_t uid[16] TU_ATTR_ALIGNED(4);
  size_t uid_len;

  // TODO work with make, but not working with esp32s3 cmake
  if ( board_get_unique_id ) {
    uid_len = board_get_unique_id(uid, sizeof(uid));
  }else {
    // fixed serial string is 01234567889ABCDEF
    uint32_t* uid32 = (uint32_t*) (uintptr_t) uid;
    uid32[0] = 0x67452301;
    uid32[1] = 0xEFCDAB89;
    uid_len = 8;
  }

  if ( uid_len > max_chars / 2 ) uid_len = max_chars / 2;

  for ( size_t i = 0; i < uid_len; i++ ) {
    for ( size_t j = 0; j < 2; j++ ) {
      const char nibble_to_hex[16] = {
          '0', '1', '2', '3', '4', '5', '6', '7',
          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
      };
      uint8_t const nibble = (uid[i] >> (j * 4)) & 0xf;
      desc_str1[i * 2 + (1 - j)] = nibble_to_hex[nibble]; // UTF-16-LE
    }
  }

  return 2 * uid_len;
}

// TODO remove
static inline void board_delay(uint32_t ms) {
  uint32_t start_ms = board_millis();
  while ( board_millis() - start_ms < ms ) {
    // take chance to run usb background
    #if CFG_TUD_ENABLED
    tud_task();
    #endif

    #if CFG_TUH_ENABLED
    tuh_task();
    #endif
  }
}

// stdio getchar() is blocking, this is non-blocking version
int board_getchar(void);

#ifdef __cplusplus
}
#endif

#endif
