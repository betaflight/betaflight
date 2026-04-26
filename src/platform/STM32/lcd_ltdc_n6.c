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

#if defined(LCD_CONSOLE_PANEL_LTDC)

#include <string.h>

#include "drivers/lcd_console.h"
#include "drivers/lcd_panel.h"

// L4 backend for STM32N6 LTDC. Currently a placeholder: it claims the panel
// slot, mirrors writes into a debug RAM grid that GDB can inspect, and is
// otherwise a no-op. Real LTDC bring-up (PSRAM via XSPI1+APS256XX, PLL3
// LCD clock, DSI host, RK050HR18 panel init, DMA2D glyph blits, framebuffer
// in PSRAM) is intentionally deferred — that work needs iteration on real
// silicon. This file keeps the panel registration path live so the build
// path, config wiring, and L1/L2/L3 layers are exercised end-to-end against
// the STM32N657DK config.
//
// To replace with real LTDC: keep `lcdPanelGet()` and the vtable shape, swap
// the body of init/drawGlyphCell/scrollUp for HAL_LTDC + DMA2D calls, point
// the framebuffer at a `.psram_bss` linker section (LCD_FRAMEBUFFER_SECTION),
// and ensure XSPI1/APS256XX bring-up runs in systemInit() before init() here.

#define COLS LCD_CONSOLE_COLS
#define ROWS LCD_CONSOLE_ROWS

// Mirror grid retained in normal RAM for inspection. Lives in `.bss` so the
// linker doesn't grow yet; size is COLS*ROWS bytes.
static char ltdcMirrorGrid[ROWS][COLS];

static bool ltdcInit(lcdPanel_t *panel)
{
    UNUSED(panel);
    memset(ltdcMirrorGrid, ' ', sizeof(ltdcMirrorGrid));
    // TODO: bring up XSPI1+APS256XX PSRAM, configure LTDC PLL3, init DSI host,
    //       send RK050HR18 init sequence, allocate framebuffer in PSRAM, enable
    //       LTDC layer 0. See lib/main/STM32N6/Projects/STM32N6570-DK/Examples
    //       and Drivers/BSP/Components/rk050hr18 for reference.
    return true;
}

static void ltdcDrawGlyphCell(lcdPanel_t *panel,
                              uint16_t row, uint16_t col,
                              uint8_t glyph, uint8_t attr)
{
    UNUSED(panel);
    UNUSED(attr);
    if (row < ROWS && col < COLS) {
        ltdcMirrorGrid[row][col] = (char)glyph;
    }
    // TODO: blit the 8x8 glyph bitmap into the LTDC framebuffer at
    //       (col * cellWidth, row * cellHeight) using DMA2D.
}

static void ltdcClearRect(lcdPanel_t *panel,
                          uint16_t row, uint16_t col,
                          uint16_t rows, uint16_t cols)
{
    UNUSED(panel);
    for (uint16_t r = row; r < row + rows && r < ROWS; r++) {
        for (uint16_t c = col; c < col + cols && c < COLS; c++) {
            ltdcMirrorGrid[r][c] = ' ';
        }
    }
    // TODO: DMA2D fill of the framebuffer rectangle to background colour.
}

static void ltdcScrollUp(lcdPanel_t *panel, uint16_t rows)
{
    UNUSED(panel);
    if (rows == 0) {
        return;
    }
    if (rows >= ROWS) {
        memset(ltdcMirrorGrid, ' ', sizeof(ltdcMirrorGrid));
        return;
    }
    memmove(&ltdcMirrorGrid[0][0], &ltdcMirrorGrid[rows][0],
            (ROWS - rows) * COLS);
    memset(&ltdcMirrorGrid[ROWS - rows][0], ' ', rows * COLS);
    // TODO: DMA2D blit framebuffer up by `rows * cellHeight` lines, then
    //       fill the bottom strip to background colour.
}

static const lcdPanelVTable_t ltdcVTable = {
    .init = ltdcInit,
    .drawGlyphCell = ltdcDrawGlyphCell,
    .clearRect = ltdcClearRect,
    .scrollUp = ltdcScrollUp,
    .flush = NULL,      // synchronous mirror; no buffering yet
    .isBusy = NULL,     // never busy in placeholder
};

static lcdPanel_t ltdcPanel = {
    .vtable = &ltdcVTable,
    .cols = COLS,
    .rows = ROWS,
    .priv = NULL,
};

lcdPanel_t *lcdPanelGet(void)
{
    return &ltdcPanel;
}

#endif // LCD_CONSOLE_PANEL_LTDC
