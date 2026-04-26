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

// L3 vtable for an LCD console backend.
//
// A backend renders a fixed grid of monochrome glyphs. The L2 terminal layer
// owns the glyph grid (cursor, scroll buffer, font lookup) and calls these
// hooks to push state to the panel. Backends are selected at compile time
// via LCD_CONSOLE_PANEL_<NAME> defines in the per-config config.h; only one
// backend exposes lcdPanelGet() per build.

struct lcdPanel_s;

typedef struct lcdPanelVTable_s {
    // Bring up the underlying hardware/bus and clear the panel. Called once
    // by the L2 layer at first use.
    bool (*init)(struct lcdPanel_s *panel);

    // Render a single glyph cell at (row, col). The glyph is an 8-bit
    // codepoint mapped through the font selected by L2; attr is reserved
    // (0 = normal). The backend may buffer; flush() commits.
    void (*drawGlyphCell)(struct lcdPanel_s *panel,
                          uint16_t row, uint16_t col,
                          uint8_t glyph, uint8_t attr);

    // Clear an inclusive rectangle of cells.
    void (*clearRect)(struct lcdPanel_s *panel,
                      uint16_t row, uint16_t col,
                      uint16_t rows, uint16_t cols);

    // Scroll the visible region up by `rows` cell rows. Backends that can
    // do this with a hardware/framebuffer copy should; otherwise the L2
    // layer falls back to row-by-row redraw via drawGlyphCell.
    void (*scrollUp)(struct lcdPanel_s *panel, uint16_t rows);

    // Commit any buffered draws. Idempotent.
    void (*flush)(struct lcdPanel_s *panel);

    // Return true if the backend still has pending bus/DMA traffic. The
    // virtual serialPort_t reports !isBusy() to its isSerialTransmitBufferEmpty
    // caller so writers can back-pressure correctly.
    bool (*isBusy)(struct lcdPanel_s *panel);
} lcdPanelVTable_t;

typedef struct lcdPanel_s {
    const lcdPanelVTable_t *vtable;
    uint16_t cols;
    uint16_t rows;
    void *priv;     // backend-private state
} lcdPanel_t;

// Each L4 backend provides exactly one lcdPanelGet() definition selected at
// compile time. The L2 layer calls it during lcdConsoleInit().
lcdPanel_t *lcdPanelGet(void);
