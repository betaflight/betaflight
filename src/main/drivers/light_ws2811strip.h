/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "platform.h"

#include "drivers/io_types.h"

#define WS2811_LED_STRIP_LENGTH    LED_STRIP_MAX_LENGTH
#ifndef WS2811_CARRIER_HZ
#define WS2811_CARRIER_HZ          800000
#endif

#if defined(USE_WS2811_SINGLE_COLOUR)
#define WS2811_DATA_BUFFER_SIZE    1
#else
#define WS2811_DATA_BUFFER_SIZE    WS2811_LED_STRIP_LENGTH
#endif

// Enumeration to match the string options defined in lookupLedStripFormatRGB in settings.c
typedef enum {
    LED_GRB,
    LED_RGB,
    LED_GRBW
} ledStripFormatRGB_e;

extern volatile bool ws2811LedDataTransferInProgress;

void ws2811LedStripInit(ioTag_t ioTag, ledStripFormatRGB_e ledFormat);
void ws2811LedStripEnable(void);

bool ws2811LedStripHardwareInit(void);
void ws2811LedStripStartTransfer(void);
void ws2811LedStripUpdateTransferBuffer(const rgbColor24bpp_t *color, unsigned ledIndex);

bool ws2811UpdateStrip(uint8_t brightness);

void setLedHsv(uint16_t index, const hsvColor_t *color);
void getLedHsv(uint16_t index, hsvColor_t *color);

void scaleLedValue(uint16_t index, const uint8_t scalePercent);
void setLedValue(uint16_t index, const uint8_t value);

void setStripColor(const hsvColor_t *color);
void setStripColors(const hsvColor_t *colors);

void setUsedLedCount(unsigned ledCount);

bool isWS2811LedStripReady(void);
