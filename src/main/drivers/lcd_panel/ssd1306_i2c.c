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

#include "platform.h"

#if defined(LCD_CONSOLE_PANEL_SSD1306_I2C)

#include <string.h>

#include "drivers/bus_i2c.h"
#include "drivers/lcd_console.h"
#include "drivers/lcd_panel.h"
#include "drivers/lcd_panel/lcd_panel_font_5x7.h"

// SSD1306 monochrome OLED, driven over I2C in page-addressing mode. Each of
// the eight 128-column pages stores 8 vertical pixels per byte, which lines
// up exactly with our 8x8 cell grid — one cell row maps to one panel page,
// and a glyph blit is a 3-byte command preamble plus the 8-byte cell data.
//
// Configuration knobs (config.h):
//   SSD1306_I2C_DEVICE   required, e.g. I2CDEV_1
//   SSD1306_I2C_ADDR     7-bit address, default 0x3C
//   SSD1306_WIDTH        panel width in pixels, default 128
//   SSD1306_HEIGHT       panel height in pixels, default 64 (use 32 for
//                        128x32 modules)
//
// The L2 layer expects an LCD_CONSOLE_COLS x LCD_CONSOLE_ROWS character grid;
// pick those to match cell coverage (e.g. 16 cols x 8 rows for 128x64,
// 16 x 4 for 128x32).

#if !defined(SSD1306_I2C_DEVICE)
#error "LCD_CONSOLE_PANEL_SSD1306_I2C selected but SSD1306_I2C_DEVICE is not defined (e.g. I2CDEV_1)."
#endif

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR    0x3C
#endif
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH       128
#endif
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT      64
#endif

#define SSD1306_PAGE_COUNT  (SSD1306_HEIGHT / 8)
#define SSD1306_CTRL_CMD    0x00
#define SSD1306_CTRL_DATA   0x40

// SSD1306 init runs late: i2cInit() doesn't run until initPhase1/2/3, but
// lcdConsoleInit() runs right after systemInit() in main.c. Defer the
// hardware bring-up until the first drawGlyphCell — by then the bus is up.
static bool hwInitialised;

static bool ssd1306WriteCommands(const uint8_t *cmds, uint8_t len)
{
    return i2cWriteBuffer(SSD1306_I2C_DEVICE, SSD1306_I2C_ADDR,
                          SSD1306_CTRL_CMD, len, (uint8_t *)cmds);
}

static bool ssd1306WriteData(const uint8_t *data, uint8_t len)
{
    return i2cWriteBuffer(SSD1306_I2C_DEVICE, SSD1306_I2C_ADDR,
                          SSD1306_CTRL_DATA, len, (uint8_t *)data);
}

static bool ssd1306HwInit(void)
{
    static const uint8_t initSeq[] = {
        0xAE,                       // display off
        0xD5, 0x80,                 // clock divide ratio / oscillator freq
        0xA8, SSD1306_HEIGHT - 1,   // multiplex ratio
        0xD3, 0x00,                 // display offset
        0x40,                       // start line = 0
        0x8D, 0x14,                 // charge pump enable
        0x20, 0x02,                 // page addressing mode
        0xA1,                       // segment remap (column 127 → SEG0)
        0xC8,                       // COM output scan dir reversed
#if SSD1306_HEIGHT == 32
        0xDA, 0x02,                 // COM pins config for 128x32
#else
        0xDA, 0x12,                 // COM pins config for 128x64
#endif
        0x81, 0xCF,                 // contrast
        0xD9, 0xF1,                 // pre-charge period
        0xDB, 0x40,                 // VCOMH deselect level
        0xA4,                       // display follows RAM
        0xA6,                       // normal (non-inverted) display
        0x2E,                       // deactivate scroll
        0xAF,                       // display on
    };

    if (!ssd1306WriteCommands(initSeq, sizeof(initSeq))) {
        return false;
    }

    // Blank the panel RAM so any leftover content from a prior boot is gone
    // before the first glyph lands.
    static const uint8_t zeros[16] = { 0 };
    for (uint8_t page = 0; page < SSD1306_PAGE_COUNT; page++) {
        const uint8_t setAddr[] = { (uint8_t)(0xB0 | page), 0x00, 0x10 };
        if (!ssd1306WriteCommands(setAddr, sizeof(setAddr))) {
            return false;
        }
        for (uint16_t col = 0; col < SSD1306_WIDTH; col += sizeof(zeros)) {
            if (!ssd1306WriteData(zeros, sizeof(zeros))) {
                return false;
            }
        }
    }
    return true;
}

static bool ssd1306EnsureHw(void)
{
    if (hwInitialised) {
        return true;
    }
    if (ssd1306HwInit()) {
        hwInitialised = true;
        return true;
    }
    return false;
}

static bool ssd1306Init(lcdPanel_t *panel)
{
    UNUSED(panel);
    // Lazy: the I2C subsystem isn't up yet at lcdConsoleInit time. Returning
    // true here just claims the panel slot; the first real draw triggers
    // ssd1306HwInit() once the bus is available.
    hwInitialised = false;
    return true;
}

static void ssd1306DrawGlyphCell(lcdPanel_t *panel,
                                 uint16_t row, uint16_t col,
                                 uint8_t glyph, uint8_t attr)
{
    UNUSED(panel);
    UNUSED(attr);
    if (!ssd1306EnsureHw()) {
        return;
    }
    if (row >= SSD1306_PAGE_COUNT) {
        return;
    }

    const uint16_t pixelCol = col * LCD_PANEL_FONT_CELL_WIDTH;
    if (pixelCol + LCD_PANEL_FONT_CELL_WIDTH > SSD1306_WIDTH) {
        return;
    }

    const uint8_t setAddr[] = {
        (uint8_t)(0xB0 | row),
        (uint8_t)(0x00 | (pixelCol & 0x0F)),
        (uint8_t)(0x10 | ((pixelCol >> 4) & 0x0F)),
    };
    if (!ssd1306WriteCommands(setAddr, sizeof(setAddr))) {
        return;
    }

    uint8_t cell[LCD_PANEL_FONT_CELL_WIDTH] = { 0 };
    const uint8_t *bitmap = lcdPanelFont5x7Glyph(glyph);
    // Place the 5-pixel glyph in the leftmost columns; remaining 3 columns
    // stay zero for inter-cell spacing.
    memcpy(cell, bitmap, LCD_PANEL_FONT_GLYPH_COLS);
    if (!ssd1306WriteData(cell, sizeof(cell))) {
        return;
    }
}

static void ssd1306ClearRect(lcdPanel_t *panel,
                             uint16_t row, uint16_t col,
                             uint16_t rows, uint16_t cols)
{
    UNUSED(panel);
    if (!ssd1306EnsureHw()) {
        return;
    }
    static const uint8_t zeroCell[LCD_PANEL_FONT_CELL_WIDTH] = { 0 };
    for (uint16_t r = row; r < row + rows && r < SSD1306_PAGE_COUNT; r++) {
        for (uint16_t c = col; c < col + cols; c++) {
            const uint16_t pixelCol = c * LCD_PANEL_FONT_CELL_WIDTH;
            if (pixelCol + LCD_PANEL_FONT_CELL_WIDTH > SSD1306_WIDTH) {
                break;
            }
            const uint8_t setAddr[] = {
                (uint8_t)(0xB0 | r),
                (uint8_t)(0x00 | (pixelCol & 0x0F)),
                (uint8_t)(0x10 | ((pixelCol >> 4) & 0x0F)),
            };
            if (!ssd1306WriteCommands(setAddr, sizeof(setAddr))) {
                return;
            }
            if (!ssd1306WriteData(zeroCell, sizeof(zeroCell))) {
                return;
            }
        }
    }
}

// scrollUp deliberately left NULL — let the L2 layer mark every row dirty
// and redraw via drawGlyphCell. SSD1306 has hardware vertical scrolling but
// it scrolls the whole panel as one unit, which doesn't compose with our
// blit-and-flush model and would also require the host to handle the
// blank-and-rewrite of the new bottom row anyway.

static const lcdPanelVTable_t ssd1306VTable = {
    .init = ssd1306Init,
    .drawGlyphCell = ssd1306DrawGlyphCell,
    .clearRect = ssd1306ClearRect,
    .scrollUp = NULL,
    .flush = NULL,
    .isBusy = NULL,
};

static lcdPanel_t ssd1306Panel = {
    .vtable = &ssd1306VTable,
    .cols = LCD_CONSOLE_COLS,
    .rows = LCD_CONSOLE_ROWS,
    .priv = NULL,
};

lcdPanel_t *lcdPanelGet(void)
{
    return &ssd1306Panel;
}

#endif // LCD_CONSOLE_PANEL_SSD1306_I2C
