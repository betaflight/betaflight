/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#define TEXT_SCREEN_CHAR uint8_t

extern textScreen_t osdTextScreen;
extern TEXT_SCREEN_CHAR textScreenBuffer[];

extern const uint8_t *asciiToFontMapping;

typedef int8_t osdCoordVal_t;

void osdSetTextScreen(textScreen_t *textScreen);
void osdClearScreen(void);
void osdResetCursor(void);
void osdSetCursor(osdCoordVal_t x, osdCoordVal_t y);
void osdPrint(char *message);
void osdPrintAt(osdCoordVal_t x, osdCoordVal_t y, char *message);
void osdSetCharacterAtPosition(osdCoordVal_t x, osdCoordVal_t y, char c);
void osdSetRawCharacterAtPosition(osdCoordVal_t x, osdCoordVal_t y, char c);

