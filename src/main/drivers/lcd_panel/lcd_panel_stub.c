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

#if defined(LCD_CONSOLE_PANEL_STUB)

#include <string.h>

#include "drivers/lcd_console.h"
#include "drivers/lcd_panel.h"

// RAM-only L4 backend. Mirrors the L2 grid into a static buffer so the
// console output is inspectable from a debugger or a CLI dump command
// without any real LCD attached. Useful for validating the abstraction
// during PR-A bring-up and as a regression aid.

#define STUB_COLS LCD_CONSOLE_COLS
#define STUB_ROWS LCD_CONSOLE_ROWS

static char stubGrid[STUB_ROWS][STUB_COLS];

static bool stubInit(lcdPanel_t *panel)
{
    UNUSED(panel);
    memset(stubGrid, ' ', sizeof(stubGrid));
    return true;
}

static void stubDrawGlyphCell(lcdPanel_t *panel, uint16_t row, uint16_t col,
                              uint8_t glyph, uint8_t attr)
{
    UNUSED(panel);
    UNUSED(attr);
    if (row < STUB_ROWS && col < STUB_COLS) {
        stubGrid[row][col] = (char)glyph;
    }
}

static void stubClearRect(lcdPanel_t *panel, uint16_t row, uint16_t col,
                          uint16_t rows, uint16_t cols)
{
    UNUSED(panel);
    for (uint16_t r = row; r < row + rows && r < STUB_ROWS; r++) {
        for (uint16_t c = col; c < col + cols && c < STUB_COLS; c++) {
            stubGrid[r][c] = ' ';
        }
    }
}

static void stubScrollUp(lcdPanel_t *panel, uint16_t rows)
{
    UNUSED(panel);
    if (rows == 0 || rows >= STUB_ROWS) {
        memset(stubGrid, ' ', sizeof(stubGrid));
        return;
    }
    memmove(&stubGrid[0][0], &stubGrid[rows][0], (STUB_ROWS - rows) * STUB_COLS);
    memset(&stubGrid[STUB_ROWS - rows][0], ' ', rows * STUB_COLS);
}

static void stubFlush(lcdPanel_t *panel)
{
    UNUSED(panel);
}

static bool stubIsBusy(lcdPanel_t *panel)
{
    UNUSED(panel);
    return false;
}

static const lcdPanelVTable_t stubVTable = {
    .init = stubInit,
    .drawGlyphCell = stubDrawGlyphCell,
    .clearRect = stubClearRect,
    .scrollUp = stubScrollUp,
    .flush = stubFlush,
    .isBusy = stubIsBusy,
};

static lcdPanel_t stubPanel = {
    .vtable = &stubVTable,
    .cols = STUB_COLS,
    .rows = STUB_ROWS,
    .priv = NULL,
};

lcdPanel_t *lcdPanelGet(void)
{
    return &stubPanel;
}

#endif // LCD_CONSOLE_PANEL_STUB
