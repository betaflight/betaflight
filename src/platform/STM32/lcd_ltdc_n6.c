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

#include "stm32n6xx_hal.h"
#include "stm32n6xx_hal_ltdc.h"
#include "stm32n6xx_hal_rcc_ex.h"

#include "drivers/lcd_console.h"
#include "drivers/lcd_panel.h"
#include "drivers/lcd_panel/lcd_panel_font_5x7.h"

// L4 backend for the STM32N6570-DK on-board RK050HR18 panel driven via the
// LTDC peripheral over a 24-bit parallel-RGB connection (no DSI on the DK
// despite the panel's MIPI capability — confirmed against ST's
// LTDC_Horizontal_Mirroring example for the same board).
//
// First-pass implementation. Expect hardware iteration before the panel
// shows clean output:
//   - Clocks (PLL4 / IC16) match the ST example. If they don't drive
//     pixel clock cleanly the panel will roll or be blank — capture the
//     LTDC pin clock and verify ~25 MHz.
//   - Framebuffer lives in AXISRAM3 (.lcdbuffer linker section). 800x480
//     L8 (8-bit indexed) = 376 KiB — fits the 448 KiB region with room.
//   - 2-entry CLUT: index 0 = black, index 1 = white. Future extension can
//     add more colours / inverse video / blink without touching L1/L2.
//   - RIF master/slave attributes are NOT configured here. Betaflight runs
//     as a non-secure application on N6, where HAL_RIF_RIMC_* are gated to
//     CPU_AS_TRUSTED_DOMAIN+CPU_IN_SECURE_STATE only. The boot ROM / FSBL
//     is expected to leave LTDC permitted to read framebuffer memory; if
//     the panel goes black on bring-up, suspect a RIF protection hit and
//     either chase the FSBL config or move LTDC RIF setup into the secure
//     init layer.
//
// Pin map (verbatim from ST LTDC_Horizontal_Mirroring example MSP):
//   PH3  LTDC_B4   PH6  LTDC_B5   PH4  LTDC_R4
//   PB14 LTDC_HSYNC PB13 LTDC_CLK PB15 LTDC_G4
//   PB11 LTDC_G6   PB12 LTDC_G5  PB4  LTDC_R3
//   PE11 LTDC_VSYNC
//   PD8  LTDC_R7
//   PG6  LTDC_B3   PG15 LTDC_B0  PG1  LTDC_G1
//   PG12 LTDC_G0   PG8  LTDC_G7  PG13 LTDC_DE
//   PG11 LTDC_R6
//   PA1  LTDC_G2   PA15 LTDC_R5  PA7  LTDC_B1
//   PA2  LTDC_B7   PA8  LTDC_B6  PA0  LTDC_G3
//   PQ6  LCD_BL_CTRL  PQ3  LCD_ONOFF
// Pixels not listed (R0..R2, B2) are discarded by the panel and left
// floating — LTDC is configured 24-bit so it drives them but the panel
// only looks at the upper bits.

#define PANEL_WIDTH       800
#define PANEL_HEIGHT      480
#define PANEL_HSYNC       4
#define PANEL_VSYNC       4
#define PANEL_HBP         8     // back porch
#define PANEL_VBP         8
#define PANEL_HFP         8     // front porch
#define PANEL_VFP         12

#define COLS              LCD_CONSOLE_COLS
#define ROWS              LCD_CONSOLE_ROWS
#define CELL_W            LCD_PANEL_FONT_CELL_WIDTH    // 8
#define CELL_H            LCD_PANEL_FONT_CELL_HEIGHT   // 8

#define FB_PIXELS         (PANEL_WIDTH * PANEL_HEIGHT)
#define COLOUR_BG         0
#define COLOUR_FG         1

// 32-byte aligned framebuffer in AXISRAM3 via the linker section. Linker
// reserves 448K; this buffer is ~376K with the rest available for future
// use (second framebuffer, scratch).
__attribute__((section(".lcdbuffer"), aligned(32)))
static uint8_t framebuffer[FB_PIXELS];

static LTDC_HandleTypeDef hltdc;
static bool hwInitialised;

static void ltdcConfigClock(void)
{
    // LTDC PCLK ~25 MHz from PLL4 via IC16. Mirrors the ST example;
    // PLL4 is already enabled from the firmware's main clock setup if
    // configured, otherwise this enable is a no-op (TODO: verify the
    // existing N6 clock tree leaves PLL4 reachable for IC16 here).
    RCC_PeriphCLKInitTypeDef periph = { 0 };
    periph.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    periph.LtdcClockSelection = RCC_LTDCCLKSOURCE_IC16;
    periph.ICSelection[RCC_IC16].ClockSelection = RCC_ICCLKSOURCE_PLL4;
    periph.ICSelection[RCC_IC16].ClockDivider = 64;
    HAL_RCCEx_PeriphCLKConfig(&periph);

    __HAL_RCC_LTDC_CLK_ENABLE();
}

static void ltdcConfigGpio(void)
{
    GPIO_InitTypeDef pin = { 0 };
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOQ_CLK_ENABLE();

    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    pin.Alternate = GPIO_AF14_LCD;

    pin.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOH, &pin);

    pin.Pin = GPIO_PIN_4 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13
            | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &pin);

    pin.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOE, &pin);

    pin.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOD, &pin);

    pin.Pin = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_11
            | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &pin);

    pin.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7
            | GPIO_PIN_8 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &pin);

    // Backlight enable + LCD on/off (push-pull GPIO outputs, not LCD AF).
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Alternate = 0;
    pin.Pin = GPIO_PIN_3 | GPIO_PIN_6;        // PQ3 = LCD_ONOFF, PQ6 = LCD_BL_CTRL
    HAL_GPIO_Init(GPIOQ, &pin);
    HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_3 | GPIO_PIN_6, GPIO_PIN_SET);
}

static bool ltdcConfigController(void)
{
    LTDC_LayerCfgTypeDef layer = { 0 };

    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync = PANEL_HSYNC - 1;
    hltdc.Init.VerticalSync = PANEL_VSYNC - 1;
    hltdc.Init.AccumulatedHBP = PANEL_HSYNC + PANEL_HBP - 1;
    hltdc.Init.AccumulatedVBP = PANEL_VSYNC + PANEL_VBP - 1;
    hltdc.Init.AccumulatedActiveW = PANEL_HSYNC + PANEL_HBP + PANEL_WIDTH - 1;
    hltdc.Init.AccumulatedActiveH = PANEL_VSYNC + PANEL_VBP + PANEL_HEIGHT - 1;
    hltdc.Init.TotalWidth = PANEL_HSYNC + PANEL_HBP + PANEL_WIDTH + PANEL_HFP - 1;
    hltdc.Init.TotalHeigh = PANEL_VSYNC + PANEL_VBP + PANEL_HEIGHT + PANEL_VFP - 1;
    hltdc.Init.Backcolor.Blue = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red = 0;

    if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
        return false;
    }

    layer.WindowX0 = 0;
    layer.WindowX1 = PANEL_WIDTH;
    layer.WindowY0 = 0;
    layer.WindowY1 = PANEL_HEIGHT;
    layer.PixelFormat = LTDC_PIXEL_FORMAT_L8;
    layer.Alpha = 255;
    layer.Alpha0 = 0;
    layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    layer.FBStartAdress = (uint32_t)framebuffer;
    layer.ImageWidth = PANEL_WIDTH;
    layer.ImageHeight = PANEL_HEIGHT;
    layer.Backcolor.Blue = 0;
    layer.Backcolor.Green = 0;
    layer.Backcolor.Red = 0;

    if (HAL_LTDC_ConfigLayer(&hltdc, &layer, 0) != HAL_OK) {
        return false;
    }

    // Two-entry CLUT: index 0 = black, index 1 = white. The remaining
    // entries are zero (black) which is the safe default for any pixel
    // value that escapes the glyph blitter.
    static uint32_t clut[256];
    clut[COLOUR_BG] = 0x00000000;       // ARGB: opaque black
    clut[COLOUR_FG] = 0x00FFFFFF;       // ARGB: opaque white
    if (HAL_LTDC_ConfigCLUT(&hltdc, clut, 256, 0) != HAL_OK) {
        return false;
    }
    if (HAL_LTDC_EnableCLUT(&hltdc, 0) != HAL_OK) {
        return false;
    }

    return true;
}

static bool ltdcEnsureHw(void)
{
    if (hwInitialised) {
        return true;
    }
    memset(framebuffer, COLOUR_BG, sizeof(framebuffer));
    ltdcConfigClock();
    ltdcConfigGpio();
    if (!ltdcConfigController()) {
        return false;
    }
    hwInitialised = true;
    return true;
}

static bool ltdcInit(lcdPanel_t *panel)
{
    UNUSED(panel);
    // Defer hardware bring-up: lcdConsoleInit runs early in main.c
    // (right after systemInit) and HAL_RCCEx_PeriphCLKConfig in particular
    // expects PLL4 to be reachable. Returning true claims the panel slot;
    // the first drawGlyphCell triggers ltdcEnsureHw().
    hwInitialised = false;
    return true;
}

static void ltdcBlitGlyph(uint16_t row, uint16_t col, uint8_t glyph)
{
    const uint16_t pixelX = col * CELL_W;
    const uint16_t pixelY = row * CELL_H;
    if (pixelX + CELL_W > PANEL_WIDTH || pixelY + CELL_H > PANEL_HEIGHT) {
        return;
    }
    const uint8_t *bitmap = lcdPanelFont5x7Glyph(glyph);

    // Render 8 columns x 8 rows. The font is 5x7 in the upper-left of an
    // 8x8 cell with implicit black padding to the right and bottom.
    for (uint16_t cy = 0; cy < CELL_H; cy++) {
        uint8_t *line = &framebuffer[(pixelY + cy) * PANEL_WIDTH + pixelX];
        for (uint16_t cx = 0; cx < CELL_W; cx++) {
            uint8_t pixel = COLOUR_BG;
            if (cx < LCD_PANEL_FONT_GLYPH_COLS && cy < LCD_PANEL_FONT_GLYPH_ROWS) {
                if (bitmap[cx] & (1u << cy)) {
                    pixel = COLOUR_FG;
                }
            }
            line[cx] = pixel;
        }
    }
}

static void ltdcDrawGlyphCell(lcdPanel_t *panel,
                              uint16_t row, uint16_t col,
                              uint8_t glyph, uint8_t attr)
{
    UNUSED(panel);
    UNUSED(attr);
    if (!ltdcEnsureHw()) {
        return;
    }
    ltdcBlitGlyph(row, col, glyph);
}

static void ltdcClearRect(lcdPanel_t *panel,
                          uint16_t row, uint16_t col,
                          uint16_t rows, uint16_t cols)
{
    UNUSED(panel);
    if (!ltdcEnsureHw()) {
        return;
    }
    const uint16_t startX = col * CELL_W;
    if (startX >= PANEL_WIDTH) {
        return;
    }
    const uint16_t span = cols * CELL_W;
    const uint16_t clipped = (startX + span > PANEL_WIDTH)
                             ? (PANEL_WIDTH - startX) : span;
    for (uint16_t r = row; r < row + rows && r * CELL_H < PANEL_HEIGHT; r++) {
        for (uint16_t cy = 0; cy < CELL_H && r * CELL_H + cy < PANEL_HEIGHT; cy++) {
            uint8_t *line = &framebuffer[(r * CELL_H + cy) * PANEL_WIDTH + startX];
            memset(line, COLOUR_BG, clipped);
        }
    }
}

static void ltdcScrollUp(lcdPanel_t *panel, uint16_t rows)
{
    UNUSED(panel);
    if (!ltdcEnsureHw() || rows == 0) {
        return;
    }
    const uint32_t pixelRowsToScroll = (uint32_t)rows * CELL_H;
    if (pixelRowsToScroll >= PANEL_HEIGHT) {
        memset(framebuffer, COLOUR_BG, sizeof(framebuffer));
        return;
    }
    const uint32_t bytesToShift = (PANEL_HEIGHT - pixelRowsToScroll) * PANEL_WIDTH;
    memmove(framebuffer,
            &framebuffer[pixelRowsToScroll * PANEL_WIDTH],
            bytesToShift);
    memset(&framebuffer[(PANEL_HEIGHT - pixelRowsToScroll) * PANEL_WIDTH],
           COLOUR_BG,
           pixelRowsToScroll * PANEL_WIDTH);
}

static const lcdPanelVTable_t ltdcVTable = {
    .init = ltdcInit,
    .drawGlyphCell = ltdcDrawGlyphCell,
    .clearRect = ltdcClearRect,
    .scrollUp = ltdcScrollUp,
    .flush = NULL,      // synchronous framebuffer writes; LTDC scans on its own
    .isBusy = NULL,
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
