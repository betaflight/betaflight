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

void ws2811UpdateStrip(void);
void setLedColor(uint16_t index, const rgbColor24bpp_t *color);
void setStripColor(const rgbColor24bpp_t *color);
void setStripColors(const rgbColor24bpp_t *colors);

bool isWS2811LedStripReady(void);

extern const rgbColor24bpp_t black;
extern const rgbColor24bpp_t white;
extern const rgbColor24bpp_t orange;
