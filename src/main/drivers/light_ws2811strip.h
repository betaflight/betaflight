/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define WS2811_LED_STRIP_LENGTH 28
#define WS2811_BITS_PER_LED 24
#define WS2811_DELAY_BUFFER_LENGTH 42 // for 50us delay

#define WS2811_DMA_BUFFER_SIZE (WS2811_BITS_PER_LED * WS2811_LED_STRIP_LENGTH + WS2811_DELAY_BUFFER_LENGTH)   // number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes)

typedef enum {
    CC_RED = 0,
    CC_GREEN,
    CC_BLUE
} colorComponent_e;

#define COLOR_COMPONENT_COUNT (CC_BLUE + 1)

struct rgbColor24bpp_s {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

typedef union {
    struct rgbColor24bpp_s rgb;
    uint8_t raw[COLOR_COMPONENT_COUNT];
} rgbColor24bpp_t;

void ws2811LedStripInit(void);

void ws2811LedStripHardwareInit(void);
void ws2811LedStripDMAEnable(void);

void ws2811UpdateStrip(void);
void setLedColor(uint16_t index, const rgbColor24bpp_t *color);
void setLedBrightness(uint16_t index, const uint8_t scalePercent);
void setStripColor(const rgbColor24bpp_t *color);
void setStripColors(const rgbColor24bpp_t *colors);

bool isWS2811LedStripReady(void);

extern uint8_t ledStripDMABuffer[WS2811_DMA_BUFFER_SIZE];
extern volatile uint8_t ws2811LedDataTransferInProgress;

extern const rgbColor24bpp_t black;
extern const rgbColor24bpp_t white;
extern const rgbColor24bpp_t orange;
