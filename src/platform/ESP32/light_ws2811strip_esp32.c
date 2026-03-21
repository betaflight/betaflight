/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_LED_STRIP

#include "common/color.h"
#include "common/maths.h"
#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/time.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "hal/rmt_ll.h"
#include "hal/gpio_ll.h"
#pragma GCC diagnostic pop

#include "hal/rmt_types.h"
#include "soc/gpio_struct.h"
#include "soc/soc_caps.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#define WS2812_RMT_CHANNEL     3       // Use last RMT TX channel (0-3 are TX on ESP32-S3)
#define WS2812_RMT_CLK_DIV    4       // 80MHz / 4 = 20MHz (50ns per tick)
#define WS2812_T0H_TICKS      8       // 400ns
#define WS2812_T0L_TICKS      17      // 850ns
#define WS2812_T1H_TICKS      16      // 800ns
#define WS2812_T1L_TICKS      9       // 450ns

// RMTMEM is provided by periph_regs_esp32.c at 0x60016800
// Each channel has 48 words (items) of memory
extern uint32_t RMTMEM[];

#define WS2812_BITS_PER_LED    24
#define WS2812_MAX_LEDS        WS2811_LED_STRIP_LENGTH

static IO_t ledStripIO = IO_NONE;
static bool ws2812Initialized = false;
static ledStripFormatRGB_e ledFormat = LED_GRB;

// Software buffer for LED strip RMT items
// 24 bits per LED (GRB), plus end marker
static rmt_symbol_word_t ws2812RmtBuffer[WS2812_MAX_LEDS * WS2812_BITS_PER_LED + 1];

static void ws2812EncodeByte(rmt_symbol_word_t *items, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if (byte & (1 << i)) {
            items[7 - i].duration0 = WS2812_T1H_TICKS;
            items[7 - i].level0 = 1;
            items[7 - i].duration1 = WS2812_T1L_TICKS;
            items[7 - i].level1 = 0;
        } else {
            items[7 - i].duration0 = WS2812_T0H_TICKS;
            items[7 - i].level0 = 1;
            items[7 - i].duration1 = WS2812_T0L_TICKS;
            items[7 - i].level1 = 0;
        }
    }
}

void ws2811LedStripInit(ioTag_t ioTag, ledStripFormatRGB_e format)
{
    ledFormat = format;

    if (!ioTag) return;

    ledStripIO = IOGetByTag(ioTag);
    if (!ledStripIO) return;

    IOInit(ledStripIO, OWNER_LED_STRIP, 0);
    IOConfigGPIO(ledStripIO, IOCFG_OUT_PP);
}

bool ws2811LedStripHardwareInit(void)
{
    if (!ledStripIO) return false;

    // Enable RMT peripheral (may already be enabled by DShot)
    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));
    rmt_ll_enable_bus_clock(0, true);
    rmt_ll_enable_periph_clock(&RMT, true);

    // Enable direct memory access (non-FIFO mode)
    rmt_ll_enable_mem_access_nonfifo(&RMT, true);

    uint8_t ch = WS2812_RMT_CHANNEL;
    uint32_t pin = IO_Pin(ledStripIO);

    rmt_ll_tx_set_channel_clock_div(&RMT, ch, WS2812_RMT_CLK_DIV);
    rmt_ll_tx_set_mem_blocks(&RMT, ch, 1);

    // Connect RMT output to GPIO pin
    esp_rom_gpio_pad_select_gpio(pin);
    gpio_ll_output_enable(&GPIO, pin);
    esp_rom_gpio_connect_out_signal(pin, RMT_SIG_OUT0_IDX + ch, false, false);

    ws2812Initialized = true;
    return true;
}

void ws2811LedStripDMAEnable(void)
{
    // NOOP - RMT handles transfer
}

void ws2811LedStripUpdateTransferBuffer(const rgbColor24bpp_t *color, unsigned ledIndex)
{
    if (!ws2812Initialized || ledIndex >= WS2812_MAX_LEDS) return;

    rmt_symbol_word_t *items = &ws2812RmtBuffer[ledIndex * WS2812_BITS_PER_LED];

    switch (ledFormat) {
    case LED_RGB:
        ws2812EncodeByte(&items[0], color->rgb.r);
        ws2812EncodeByte(&items[8], color->rgb.g);
        ws2812EncodeByte(&items[16], color->rgb.b);
        break;
    case LED_GRB:
    default:
        ws2812EncodeByte(&items[0], color->rgb.g);
        ws2812EncodeByte(&items[8], color->rgb.r);
        ws2812EncodeByte(&items[16], color->rgb.b);
        break;
    }
}

void ws2811LedStripStartTransfer(void)
{
    if (!ws2812Initialized) return;

    ws2811LedDataTransferInProgress = true;

    const uint8_t ch = WS2812_RMT_CHANNEL;
    const uint32_t totalItems = WS2811_LED_STRIP_LENGTH * WS2812_BITS_PER_LED;

    // Write RMT items to hardware memory via RMTMEM
    // Each channel has SOC_RMT_MEM_WORDS_PER_CHANNEL (48) items
    // For longer strips we are limited to what fits in one memory block
    volatile uint32_t *rmtMem = &RMTMEM[ch * SOC_RMT_MEM_WORDS_PER_CHANNEL];
    const uint32_t copyLen = MIN(totalItems, (uint32_t)(SOC_RMT_MEM_WORDS_PER_CHANNEL - 1));  // Leave room for end marker

    for (uint32_t i = 0; i < copyLen; i++) {
        rmtMem[i] = ws2812RmtBuffer[i].val;
    }
    // End marker (zero duration signals end of transmission)
    rmtMem[copyLen] = 0;

    rmt_ll_tx_reset_pointer(&RMT, ch);
    rmt_ll_tx_start(&RMT, ch);

    // Wait for completion (simple polling for now)
    // ~30us per LED plus 50us reset pulse
    delayMicroseconds(50 + (WS2811_LED_STRIP_LENGTH * 30));

    ws2811LedDataTransferInProgress = false;
}

#endif // USE_LED_STRIP
