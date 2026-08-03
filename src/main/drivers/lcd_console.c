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

#if ENABLE_LCD_CONSOLE

// Compile-time check: ENABLE_LCD_CONSOLE requires exactly one panel selector
// to be defined. Without one, lcdPanelGet() is undefined and the link fails
// with a less obvious message; with more than one, the duplicate definitions
// of lcdPanelGet() collide at link time. Add new backends to this list as
// they land.
#if (defined(LCD_CONSOLE_PANEL_STUB) \
   + defined(LCD_CONSOLE_PANEL_LTDC) \
   + defined(LCD_CONSOLE_PANEL_SSD1306_I2C)) != 1
#error "ENABLE_LCD_CONSOLE requires exactly one LCD_CONSOLE_PANEL_<NAME> selector to be defined. See drivers/lcd_panel/ or src/platform/."
#endif

#include <string.h>

#include "drivers/lcd_console.h"
#include "drivers/lcd_panel.h"

#define COLS LCD_CONSOLE_COLS
#define ROWS LCD_CONSOLE_ROWS

#define TAB_STOP 8
#define BLANK_GLYPH ' '

static uint8_t grid[ROWS][COLS];
static uint8_t dirty[ROWS];   // 1 if row needs redraw
static uint16_t cursorRow;
static uint16_t cursorCol;
static lcdPanel_t *panel;
static bool initialised;

static void blankRow(uint16_t row)
{
    memset(&grid[row][0], BLANK_GLYPH, COLS);
    dirty[row] = 1;
}

static void scrollUpOne(void)
{
    // Shift the in-memory grid up by one. Pending dirty rows must shift in
    // lockstep so a later flush still re-draws the right cells (otherwise a
    // cell that was dirty at the old position would be lost while a freshly
    // flushed-from cell would silently move out of sync with the panel).
    memmove(&grid[0][0], &grid[1][0], (ROWS - 1) * COLS);
    memmove(&dirty[0], &dirty[1], ROWS - 1);
    blankRow(ROWS - 1);
    if (panel && panel->vtable->scrollUp) {
        panel->vtable->scrollUp(panel, 1);
    } else {
        // Backend can't hardware-scroll; mark all rows for redraw.
        for (uint16_t r = 0; r < ROWS; r++) {
            dirty[r] = 1;
        }
    }
}

static void newline(void)
{
    cursorCol = 0;
    if (cursorRow + 1 < ROWS) {
        cursorRow++;
    } else {
        scrollUpOne();
    }
}

static void putGlyph(uint8_t glyph)
{
    if (cursorCol >= COLS) {
        newline();
    }
    grid[cursorRow][cursorCol] = glyph;
    dirty[cursorRow] = 1;
    cursorCol++;
}

bool lcdConsoleInit(void)
{
    if (initialised) {
        return true;
    }
    panel = lcdPanelGet();
    // init and drawGlyphCell are mandatory (see lcd_panel.h); the rest are
    // optional and NULL-checked at the call sites.
    if (!panel || !panel->vtable
            || !panel->vtable->init
            || !panel->vtable->drawGlyphCell) {
        return false;
    }
    if (!panel->vtable->init(panel)) {
        return false;
    }
    for (uint16_t r = 0; r < ROWS; r++) {
        memset(&grid[r][0], BLANK_GLYPH, COLS);
        dirty[r] = 0;
    }
    cursorRow = 0;
    cursorCol = 0;
    initialised = true;
    return true;
}

void lcdConsoleClear(void)
{
    if (!initialised && !lcdConsoleInit()) {
        return;
    }
    for (uint16_t r = 0; r < ROWS; r++) {
        blankRow(r);
    }
    cursorRow = 0;
    cursorCol = 0;
    if (panel->vtable->clearRect) {
        panel->vtable->clearRect(panel, 0, 0, ROWS, COLS);
    }
    lcdConsoleFlush();
}

void lcdConsolePutc(uint8_t c)
{
    if (!initialised && !lcdConsoleInit()) {
        return;
    }
    switch (c) {
    case '\r':
        cursorCol = 0;
        return;
    case '\n':
        newline();
        return;
    case '\b':
        if (cursorCol > 0) {
            cursorCol--;
            grid[cursorRow][cursorCol] = BLANK_GLYPH;
            dirty[cursorRow] = 1;
        }
        return;
    case '\t': {
        uint16_t next = (cursorCol + TAB_STOP) & ~(TAB_STOP - 1);
        if (next > COLS) {
            next = COLS;
        }
        while (cursorCol < next) {
            putGlyph(BLANK_GLYPH);
        }
        return;
    }
    default:
        if (c < 0x20 || c == 0x7f) {
            return;     // ignore other control chars for now
        }
        putGlyph(c);
        return;
    }
}

void lcdConsoleWrite(const uint8_t *buf, size_t len)
{
    if (!initialised && !lcdConsoleInit()) {
        return;
    }
    for (size_t i = 0; i < len; i++) {
        lcdConsolePutc(buf[i]);
    }
    lcdConsoleFlush();
}

void lcdConsoleFlush(void)
{
    if (!initialised) {
        return;
    }
    for (uint16_t r = 0; r < ROWS; r++) {
        if (!dirty[r]) {
            continue;
        }
        for (uint16_t col = 0; col < COLS; col++) {
            panel->vtable->drawGlyphCell(panel, r, col, grid[r][col], 0);
        }
        dirty[r] = 0;
    }
    if (panel->vtable->flush) {
        panel->vtable->flush(panel);
    }
}

bool lcdConsoleIsBusy(void)
{
    if (!initialised || !panel || !panel->vtable->isBusy) {
        return false;
    }
    return panel->vtable->isBusy(panel);
}

const uint8_t *lcdConsoleRow(uint16_t row)
{
    if (!initialised || row >= ROWS) {
        return NULL;
    }
    return &grid[row][0];
}

#endif // ENABLE_LCD_CONSOLE
