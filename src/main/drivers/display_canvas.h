/*
 * This file is part of Cleanflight, Betaflight and INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * @author Alberto Garcia Hierro <alberto@garciahierro.com>
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DISPLAY_CANVAS_BITMAP_OPT_INVERT_COLORS = 1 << 0,
    DISPLAY_CANVAS_BITMAP_OPT_SOLID_BACKGROUND = 1 << 1,
    DISPLAY_CANVAS_BITMAP_OPT_ERASE_TRANSPARENT = 1 << 2,
} displayCanvasBitmapOption_t;

typedef enum {
    DISPLAY_CANVAS_COLOR_BLACK = 0,
    DISPLAY_CANVAS_COLOR_TRANSPARENT = 1,
    DISPLAY_CANVAS_COLOR_WHITE = 2,
    DISPLAY_CANVAS_COLOR_GRAY = 3,
} displayCanvasColor_e;

typedef enum {
    DISPLAY_CANVAS_OUTLINE_TYPE_NONE = 0,
    DISPLAY_CANVAS_OUTLINE_TYPE_TOP = 1 << 0,
    DISPLAY_CANVAS_OUTLINE_TYPE_RIGHT = 1 << 1,
    DISPLAY_CANVAS_OUTLINE_TYPE_BOTTOM = 1 << 2,
    DISPLAY_CANVAS_OUTLINE_TYPE_LEFT = 1 << 3,
} displayCanvasOutlineType_e;

struct displayCanvasVTable_s;

typedef struct displayCanvas_s {
    const struct displayCanvasVTable_s *vTable;
    void *device;
    uint16_t width;
    uint16_t height;
    uint8_t gridElementWidth;
    uint8_t gridElementHeight;
} displayCanvas_t;

typedef struct displayCanvasVTable_s {
    void (*setStrokeColor)(displayCanvas_t *displayCanvas, displayCanvasColor_e color);
    void (*setFillColor)(displayCanvas_t *displayCanvas, displayCanvasColor_e color);
    void (*setStrokeAndFillColor)(displayCanvas_t *displayCanvas, displayCanvasColor_e color);
    void (*setColorInversion)(displayCanvas_t *displayCanvas, bool inverted);
    void (*setPixel)(displayCanvas_t *displayCanvas, int x, int y, displayCanvasColor_e color);
    void (*setPixelToStrokeColor)(displayCanvas_t *displayCanvas, int x, int y);
    void (*setPixelToFillColor)(displayCanvas_t *displayCanvas, int x, int y);
    void (*setStrokeWidth)(displayCanvas_t *displayCanvas, unsigned w);
    void (*setLineOutlineType)(displayCanvas_t *displayCanvas, displayCanvasOutlineType_e outlineType);
    void (*setLineOutlineColor)(displayCanvas_t *displayCanvas, displayCanvasColor_e outlineColor);

    void (*clipToRect)(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
    void (*clearRect)(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
    void (*resetDrawingState)(displayCanvas_t *displayCanvas);
    void (*drawCharacter)(displayCanvas_t *displayCanvas, int x, int y, uint16_t chr, displayCanvasBitmapOption_t opts);
    void (*drawCharacterMask)(displayCanvas_t *displayCanvas, int x, int y, uint16_t chr, displayCanvasColor_e color, displayCanvasBitmapOption_t opts);
    void (*drawString)(displayCanvas_t *displayCanvas, int x, int y, const char *s, displayCanvasBitmapOption_t opts);
    void (*drawStringMask)(displayCanvas_t *displayCanvas, int x, int y, const char *s, displayCanvasColor_e color, displayCanvasBitmapOption_t opts);
    void (*moveToPoint)(displayCanvas_t *displayCanvas, int x, int y);
    void (*strokeLineToPoint)(displayCanvas_t *displayCanvas, int x, int y);
    void (*strokeTriangle)(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3);
    void (*fillTriangle)(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3);
    void (*fillStrokeTriangle)(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3);
    void (*strokeRect)(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
    void (*fillRect)(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
    void (*fillStrokeRect)(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
    void (*strokeEllipseInRect)(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
    void (*fillEllipseInRect)(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
    void (*fillStrokeEllipseInRect)(displayCanvas_t *displayCanvas, int x, int y, int w, int h);

    void (*ctmReset)(displayCanvas_t *displayCanvas);
    void (*ctmSet)(displayCanvas_t *displayCanvas, float m11, float m12, float m21, float m22, float m31, float m32);
    void (*ctmTranslate)(displayCanvas_t *displayCanvas, float tx, float ty);
    void (*ctmScale)(displayCanvas_t *displayCanvas, float sx, float sy);
    void (*ctmRotate)(displayCanvas_t *displayCanvas, float r);

    void (*contextPush)(displayCanvas_t *displayCanvas);
    void (*contextPop)(displayCanvas_t *displayCanvas);
} displayCanvasVTable_t;

void displayCanvasSetStrokeColor(displayCanvas_t *displayCanvas, displayCanvasColor_e color);
void displayCanvasSetFillColor(displayCanvas_t *displayCanvas, displayCanvasColor_e color);
void displayCanvasSetStrokeAndFillColor(displayCanvas_t *displayCanvas, displayCanvasColor_e color);
void displayCanvasSetColorInversion(displayCanvas_t *displayCanvas, bool inverted);
void displayCanvasSetPixel(displayCanvas_t *displayCanvas, int x, int y, displayCanvasColor_e);
void displayCanvasSetPixelToStrokeColor(displayCanvas_t *displayCanvas, int x, int y);
void displayCanvasSetPixelToFillColor(displayCanvas_t *displayCanvas, int x, int y);
void displayCanvasSetStrokeWidth(displayCanvas_t *displayCanvas, unsigned w);
void displayCanvasSetLineOutlineType(displayCanvas_t *displayCanvas, displayCanvasOutlineType_e outlineType);
void displayCanvasSetLineOutlineColor(displayCanvas_t *displayCanvas, displayCanvasColor_e outlineColor);

void displayCanvasClipToRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
void displayCanvasClearRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
void displayCanvasResetDrawingState(displayCanvas_t *displayCanvas);
void displayCanvasDrawCharacter(displayCanvas_t *displayCanvas, int x, int y, uint16_t chr, displayCanvasBitmapOption_t opts);
void displayCanvasDrawCharacterMask(displayCanvas_t *displayCanvas, int x, int y, uint16_t chr, displayCanvasColor_e color, displayCanvasBitmapOption_t opts);
void displayCanvasDrawString(displayCanvas_t *displayCanvas, int x, int y, const char *s, displayCanvasBitmapOption_t opts);
void displayCanvasDrawStringMask(displayCanvas_t *displayCanvas, int x, int y, const char *s, displayCanvasColor_e color, displayCanvasBitmapOption_t opts);
void displayCanvasMoveToPoint(displayCanvas_t *displayCanvas, int x, int y);
void displayCanvasStrokeLineToPoint(displayCanvas_t *displayCanvas, int x, int y);
void displayCanvasStrokeTriangle(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3);
void displayCanvasFillTriangle(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3);
void displayCanvasFillStrokeTriangle(displayCanvas_t *displayCanvas, int x1, int y1, int x2, int y2, int x3, int y3);
void displayCanvasStrokeRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
void displayCanvasFillRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
void displayCanvasFillStrokeRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
void displayCanvasStrokeEllipseInRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
void displayCanvasFillEllipseInRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h);
void displayCanvasFillStrokeEllipseInRect(displayCanvas_t *displayCanvas, int x, int y, int w, int h);

void displayCanvasCtmReset(displayCanvas_t *displayCanvas);
void displayCanvasCtmSet(displayCanvas_t *displayCanvas, float m11, float m12, float m21, float m22, float m31, float m32);
void displayCanvasCtmTranslate(displayCanvas_t *displayCanvas, float tx, float ty);
void displayCanvasCtmScale(displayCanvas_t *displayCanvas, float sx, float sy);
void displayCanvasCtmRotate(displayCanvas_t *displayCanvas, float r);

void displayCanvasContextPush(displayCanvas_t *displayCanvas);
void displayCanvasContextPop(displayCanvas_t *displayCanvas);
