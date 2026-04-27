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

#pragma once

#include <stdint.h>

// 5x7 bitmap font for LCD console panel backends. 5 columns x 7 visible rows
// per glyph, packed as 5 bytes per glyph in column-major order — each byte
// holds 8 vertical pixels (LSB at the top), matching SSD1306-style page
// memory directly. Cells are 8x8 with 3 columns and 1 row of background
// padding handled by the backend.
//
// Coverage: printable ASCII 0x20 (space) through 0x7E (tilde) = 95 glyphs.
// Unsupported codepoints render as the fallback glyph (index for '?').

#define LCD_PANEL_FONT_FIRST_GLYPH 0x20
#define LCD_PANEL_FONT_LAST_GLYPH  0x7E
#define LCD_PANEL_FONT_GLYPH_COLS  5
#define LCD_PANEL_FONT_GLYPH_ROWS  7
#define LCD_PANEL_FONT_CELL_WIDTH  8
#define LCD_PANEL_FONT_CELL_HEIGHT 8

extern const uint8_t lcdPanelFont5x7[][LCD_PANEL_FONT_GLYPH_COLS];

// Returns a pointer to the 5-byte glyph for a codepoint, or the '?' glyph
// when out of range.
const uint8_t *lcdPanelFont5x7Glyph(uint8_t codepoint);
