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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <platform.h>
#include "build/debug.h"

#include "drivers/video_textscreen.h"
#include "common/utils.h"

#include "osd/osd_element.h"
#include "osd/osd_screen.h"


textScreen_t osdTextScreen;

typedef struct osdCursor_s {
    uint8_t x;
    uint8_t y;
} osdCursor_t;

static osdCursor_t cursor = {0, 0};

static uint16_t osdCalculateBufferOffset(osdCoordVal_t x, osdCoordVal_t y) {
    osdCoordVal_t yy;
    if (y >= 0) {
        // positive y = top aligned
        yy = y;
    } else {
        // negative y = bottom aligned
        yy = osdTextScreen.height + y;
    }
    uint16_t offset = (yy * osdTextScreen.width) + x;
    return offset;
}
// Does not move the cursor.
void osdSetCharacterAtPosition(osdCoordVal_t x, osdCoordVal_t y, char c)
{
    uint8_t mappedCharacter = asciiToFontMapping[(uint8_t)c];

    uint16_t offset = osdCalculateBufferOffset(x,y);
    textScreenBuffer[offset] = mappedCharacter;
}

// Does not move the cursor.
void osdSetRawCharacterAtPosition(osdCoordVal_t x, osdCoordVal_t y, char c)
{
    uint16_t offset = osdCalculateBufferOffset(x,y);
    textScreenBuffer[offset] = c;
}

void osdResetCursor(void)
{
    cursor.x = 0;
    cursor.y = 0;
}

void osdSetCursor(osdCoordVal_t x, osdCoordVal_t y)
{
    cursor.x = x;
    cursor.y = y;
}

// software cursor, handles line wrapping and row wrapping, resets to 0,0 when the end of the screen is reached
static void osdAdvanceCursor(void)
{
    cursor.x++;
    if (cursor.x >= osdTextScreen.width) {
        cursor.y++;
        cursor.x = 0;
        if (cursor.y > osdTextScreen.height) {
            cursor.y = 0;
        }
    }
}

void osdPrint(char *message)
{
    char *charPtr = message;

    while(*charPtr) {
        osdSetCharacterAtPosition(cursor.x, cursor.y, *charPtr);
        osdAdvanceCursor();

        charPtr++;
    }
}

void osdPrintAt(osdCoordVal_t x, osdCoordVal_t y, char *message)
{
    osdSetCursor(x, y);
    osdPrint(message);
}

void osdSetTextScreen(textScreen_t *textScreen)
{
    osdTextScreen = *textScreen;
}

void osdClearScreen(void)
{
    uint8_t mappedSpaceCharacter = asciiToFontMapping[(uint8_t)' '];

    int offset = 0;
    for (int y = 0; y < osdTextScreen.height; y++) {
        for (int x = 0; x < osdTextScreen.width; x++) {
            textScreenBuffer[offset++] = mappedSpaceCharacter;
        }
    }
}
