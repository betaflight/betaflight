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
#include <string.h>

#include "platform.h"

#ifdef USE_LED_STRIP

#include "common/color.h"

#include "drivers/dma.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/light_ws2811strip.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#define WS2812_WRAP_TARGET 0
#define WS2812_WRAP 3
#define WS2812_PIO_VERSION 0

#define WS2812_T1 3
#define WS2812_T2 3
#define WS2812_T3 4

#define WS2812_BYTES_PER_LED 4 // Number of bytes per WS2812 LED (RGB)


static IO_t ledStripIO = IO_NONE;
static ioTag_t ledStripIoTag = IO_TAG_NONE;
static ledStripFormatRGB_e ledStripFormat = LED_GRB; // Default format is RGB

// DMA channel
static uint8_t dma_chan;

// Buffer to hold the LED color data
static uint8_t led_data[WS2811_LED_STRIP_LENGTH][WS2812_BYTES_PER_LED];

static const uint16_t ws2812_program_instructions[] = {
            //     .wrap_target
    0x6321, //  0: out    x, 1            side 0 [3]
    0x1223, //  1: jmp    !x, 3           side 1 [2]
    0x1200, //  2: jmp    0               side 1 [2]
    0xa242, //  3: nop                    side 0 [2]
            //     .wrap
};

static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = ARRAYLEN(ws2812_program_instructions),
    .origin = -1,
    .pio_version = WS2812_PIO_VERSION,
    .used_gpio_ranges = 0x0
};

static inline pio_sm_config ws2812_program_get_default_config(uint offset)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + WS2812_WRAP_TARGET, offset + WS2812_WRAP);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

static inline void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq, bool rgbw)
{
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, rgbw ? 32 : 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    int cycles_per_bit = WS2812_T1 + WS2812_T2 + WS2812_T3;
    int div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static FAST_IRQ_HANDLER void ws2811LedStripDmaHandler(dmaChannelDescriptor_t* descriptor)
{
    UNUSED(descriptor);
    ws2811LedDataTransferInProgress = false;
}

void ws2811LedStripInit(ioTag_t ioTag, ledStripFormatRGB_e ledFormat)
{
    ledStripFormat = ledFormat; // Store the format globally
    memset(led_data, 0, sizeof(led_data));
    ledStripIoTag = ioTag;
}

bool ws2811LedStripHardwareInit(void)
{
    if (!ledStripIoTag) {
        return false;
    }

    IO_t io = IOGetByTag(ledStripIoTag);
    if (!IOIsFreeOrPreinit(io)) {
        return false;
    }

    // This will find a free pio and state machine for our program and load it for us
    PIO pio;
    uint sm;
    uint offset;

    int pinIndex = DEFIO_TAG_PIN(ledStripIoTag);
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm, &offset, pinIndex, 1, true);
    if (!success) {
        return false; // No free PIO or state machine available
    }
    ws2812_program_init(pio, sm, offset, pinIndex, WS2811_CARRIER_HZ, ledStripFormat == LED_GRBW);

    // --- DMA Configuration ---
    const dmaIdentifier_e dma_id = dmaGetFreeIdentifier();
    if (dma_id == DMA_NONE) {
        return false; // No free DMA channel available
    }

    dma_chan = DMA_IDENTIFIER_TO_CHANNEL(dma_id);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(
        dma_chan,
        &c,
        &pio->txf[sm],            // Write address (PIO TX FIFO)
        NULL,                     // Read address (set later)
        WS2811_LED_STRIP_LENGTH,  // Number of transfers
        false                     // Don't start immediately
    );

    // --- Interrupt Configuration ---
    dmaSetHandler(dma_id, ws2811LedStripDmaHandler, 0, 0);

    IOInit(io, OWNER_LED_STRIP, 0);
    ledStripIO = io;
    return true;
}

void ws2811LedStripStartTransfer(void)
{
    if (!ledStripIO) {
        ws2811LedDataTransferInProgress = false;
        return; // Not initialized
    }

    // Set the read address to the led_data buffer
    dma_channel_set_read_addr(dma_chan, led_data, false);
    // Start the DMA transfer
    dma_channel_set_trans_count(dma_chan, WS2811_LED_STRIP_LENGTH, true);
}

void ws2811LedStripUpdateTransferBuffer(rgbColor24bpp_t *color, unsigned ledIndex)
{
    if (ledIndex >= WS2811_LED_STRIP_LENGTH) {
        return; // Index out of bounds
    }

    switch(ledStripFormat) {
    case LED_RGB: // WS2811 drivers use RGB format
        led_data[ledIndex][0] = color->rgb.r;
        led_data[ledIndex][1] = color->rgb.g;
        led_data[ledIndex][2] = color->rgb.b;
        break;

    case LED_GRBW: // SK6812 drivers use this
        // Reconstruct white channel from RGB, making the intensity a bit nonlinear, but that's fine for this use case
        {
            uint8_t white = MIN(MIN(color->rgb.r, color->rgb.g), color->rgb.b);
            led_data[ledIndex][0] = color->rgb.g; // Green
            led_data[ledIndex][1] = color->rgb.r; // Red
            led_data[ledIndex][2] = color->rgb.b; // Blue
            led_data[ledIndex][3] = white; // White
        }
        break;
    case LED_GRB: // WS2812 drivers use GRB format
    default:
        led_data[ledIndex][0] = color->rgb.g;
        led_data[ledIndex][1] = color->rgb.r;
        led_data[ledIndex][2] = color->rgb.b;
        break;
    }
}
#endif
