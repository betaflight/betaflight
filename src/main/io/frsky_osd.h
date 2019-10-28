#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/display.h"
#include "drivers/osd.h"

typedef enum {
    FRSKY_OSD_TRANSACTION_OPT_PROFILED = 1 << 0,
    FRSKY_OSD_TRANSACTION_OPT_RESET_DRAWING = 1 << 1,
} frskyOsdTransactionOptions_e;

typedef enum {
    FRSKY_OSD_COLOR_BLACK = 0,
    FRSKY_OSD_COLOR_TRANSPARENT = 1,
    FRSKY_OSD_COLOR_WHITE = 2,
    FRSKY_OSD_COLOR_GRAY = 3,
} frskyOsdColor_e;

typedef enum {
    FRSKY_OSD_OUTLINE_TYPE_NONE = 0,
    FRSKY_OSD_OUTLINE_TYPE_TOP = 1 << 0,
    FRSKY_OSD_OUTLINE_TYPE_RIGHT = 1 << 1,
    FRSKY_OSD_OUTLINE_TYPE_BOTTOM = 1 << 2,
    FRSKY_OSD_OUTLINE_TYPE_LEFT = 1 << 3,
} frskyOsdLineOutlineType_e;

bool frskyOsdInit(videoSystem_e videoSystem);
bool frskyOsdIsReady(void);
void frskyOsdUpdate(void);
void frskyOsdBeginTransaction(frskyOsdTransactionOptions_e opts);
void frskyOsdCommitTransaction(void);
void frskyOsdFlushSendBuffer(void);
bool frskyOsdReadFontCharacter(unsigned char_address, osdCharacter_t *chr);
bool frskyOsdWriteFontCharacter(unsigned char_address, const osdCharacter_t *chr);

unsigned frskyOsdGetGridRows(void);
unsigned frskyOsdGetGridCols(void);

unsigned frskyOsdGetPixelWidth(void);
unsigned frskyOsdGetPixelHeight(void);

void frskyOsdDrawStringInGrid(unsigned x, unsigned y, const char *buff);
void frskyOsdDrawCharInGrid(unsigned x, unsigned y, uint16_t chr);
bool frskyOsdReadCharInGrid(unsigned x, unsigned y, uint16_t *c);
void frskyOsdClearScreen(void);

void frskyOsdSetStrokeColor(frskyOsdColor_e color);
void frskyOsdSetFillColor(frskyOsdColor_e color);
void frskyOsdSetStrokeAndFillColor(frskyOsdColor_e color);
void frskyOsdSetColorInversion(bool inverted);
void frskyOsdSetPixel(int x, int y, frskyOsdColor_e color);
void frskyOsdSetPixelToStrokeColor(int x, int y);
void frskyOsdSetPixelToFillColor(int x, int y);
void frskyOsdSetStrokeWidth(unsigned width);
void frskyOsdSetLineOutlineType(frskyOsdLineOutlineType_e outlineType);
void frskyOsdSetLineOutlineColor(frskyOsdColor_e outlineColor);

void frskyOsdClipToRect(int x, int y, int w, int h);
void frskyOsdClearRect(int x, int y, int w, int h);
void frskyOsdResetDrawingState(void);
void frskyOsdDrawCharacter(int x, int y, uint16_t chr, uint8_t opts);
void frskyOsdDrawCharacterMask(int x, int y, uint16_t chr, frskyOsdColor_e color, uint8_t opts);
void frskyOsdDrawString(int x, int y, const char *s, uint8_t opts);
void frskyOsdDrawStringMask(int x, int y, const char *s, frskyOsdColor_e color, uint8_t opts);
void frskyOsdMoveToPoint(int x, int y);
void frskyOsdStrokeLineToPoint(int x, int y);
void frskyOsdStrokeTriangle(int x1, int y1, int x2, int y2, int x3, int y3);
void frskyOsdFillTriangle(int x1, int y1, int x2, int y2, int x3, int y3);
void frskyOsdFillStrokeTriangle(int x1, int y1, int x2, int y2, int x3, int y3);
void frskyOsdStrokeRect(int x, int y, int w, int h);
void frskyOsdFillRect(int x, int y, int w, int h);
void frskyOsdFillStrokeRect(int x, int y, int w, int h);
void frskyOsdStrokeEllipseInRect(int x, int y, int w, int h);
void frskyOsdFillEllipseInRect(int x, int y, int w, int h);
void frskyOsdFillStrokeEllipseInRect(int x, int y, int w, int h);

void frskyOsdCtmReset(void);
void frskyOsdCtmSet(float m11, float m12, float m21, float m22, float m31, float m32);
void frskyOsdCtmTranslate(float tx, float ty);
void frskyOsdCtmScale(float sx, float sy);
void frskyOsdCtmRotate(float r);

void frskyOsdContextPush(void);
void frskyOsdContextPop(void);
