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

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// L2 — terminal logic for the LCD console. Hardware-agnostic; calls into
// the L3 panel vtable (drivers/lcd_panel.h). Code in src/main/ must not
// include any panel/MCU-specific headers — the only seam is lcdPanelGet().

#ifndef LCD_CONSOLE_COLS
#define LCD_CONSOLE_COLS 80
#endif
#ifndef LCD_CONSOLE_ROWS
#define LCD_CONSOLE_ROWS 25
#endif

// Initialise the terminal layer and the underlying panel. Idempotent and
// lazy-safe: the first call into any putc/write API will call this, but
// callers can also invoke it explicitly during boot to get earlier failure
// signal if the panel can't come up.
bool lcdConsoleInit(void);

// Write a single byte to the terminal. Honours \r \n \b \t; everything else
// is treated as a printable codepoint and written into the cell at the
// current cursor. Wraps to the next row at the right edge; scrolls up at
// the bottom.
void lcdConsolePutc(uint8_t c);

// Write a buffer. Equivalent to looping lcdConsolePutc() but lets the
// backend buffer draws and flush once at the end.
void lcdConsoleWrite(const uint8_t *buf, size_t len);

// Reset the cursor to (0, 0) and blank the grid.
void lcdConsoleClear(void);

// Commit any buffered draws to the panel.
void lcdConsoleFlush(void);

// True while the panel still has pending bus/DMA traffic. Used by the L1
// virtual serialPort_t to honour isSerialTransmitBufferEmpty.
bool lcdConsoleIsBusy(void);

// Read-only pointer to a row of LCD_CONSOLE_COLS cells in the L2 grid,
// or NULL if `row` is out of range or the layer hasn't been initialised.
// Used by diagnostic surfaces (e.g. the `lcd` CLI command) to inspect what
// the console currently holds without coupling to the panel backend.
const uint8_t *lcdConsoleRow(uint16_t row);
