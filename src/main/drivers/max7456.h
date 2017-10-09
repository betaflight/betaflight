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

/** PAL or NTSC, value is number of chars total */
#define VIDEO_BUFFER_CHARS_NTSC   390
#define VIDEO_BUFFER_CHARS_PAL    480
#define VIDEO_LINES_NTSC          13
#define VIDEO_LINES_PAL           16

extern uint16_t maxScreenSize;

struct vcdProfile_s;
void    max7456HardwareReset(void);
void    max7456Init(const struct vcdProfile_s *vcdProfile);
void    max7456Invert(bool invert);
void    max7456Brightness(uint8_t black, uint8_t white);
void    max7456DrawScreen(void);
void    max7456WriteNvm(uint8_t char_address, const uint8_t *font_data);
uint8_t max7456GetRowsCount(void);
void    max7456Write(uint8_t x, uint8_t y, const char *buff);
void    max7456WriteChar(uint8_t x, uint8_t y, uint8_t c);
void    max7456ClearScreen(void);
void    max7456RefreshAll(void);
uint8_t* max7456GetScreenBuffer(void);
bool    max7456DmaInProgress(void);

typedef struct max7456Config_s {
    uint8_t clockConfig; // 0 = force half clock, 1 = half if OC, 2 = force full
} max7456Config_t;

#define MAX7456_CLOCK_CONFIG_HALF 0
#define MAX7456_CLOCK_CONFIG_OC   1
#define MAX7456_CLOCK_CONFIG_FULL 2

PG_DECLARE(max7456Config_t, max7456Config);
