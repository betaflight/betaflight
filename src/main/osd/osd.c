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

/*
 * Note: this code is very early code that justs gets basic information on the screen.
 *
 * This code is currently dependent on the max7465 chip but that is NOT the final goal.  Display driver abstraction is required.
 * The idea is that basic textual information should be able to be displayed by all OSD video hardware and all hardware layers should
 * translate an in-memory character buffer to the display as required.  In the case of the max7456 chip we will just copy the in-memory display
 * buffer to the max7456 character memory.
 *
 * Later on when the code is more mature support can be added for non-character based display drivers.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <platform.h>
#include "debug.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "common/printf.h"

#include "fc/rc_controls.h" // FIXME dependency on FC code for throttle status

#include "sensors/battery.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/video_textscreen.h"

#include "osd/config.h"

#include "osd/fc_state.h"

#include "osd/osd.h"

PG_REGISTER(osdFontConfig_t, osdFontConfig, PG_OSD_FONT_CONFIG, 0);

textScreen_t osdTextScreen;

typedef struct osdCursor_s {
    uint8_t x;
    uint8_t y;
} osdCursor_t;

static osdCursor_t cursor = {0, 0};

// Does not move the cursor.
void osdSetCharacterAtPosition(uint8_t x, uint8_t y, char c)
{
    uint8_t mappedCharacter = asciiToFontMapping[(uint8_t)c];

    unsigned int offset = (y * osdTextScreen.width) + x;
    textScreenBuffer[offset] = mappedCharacter;
}

// Does not move the cursor.
void osdSetRawCharacterAtPosition(uint8_t x, uint8_t y, char c)
{
    unsigned int offset = (y * osdTextScreen.width) + x;
    textScreenBuffer[offset] = c;
}

void osdResetCursor(void)
{
    cursor.x = 0;
    cursor.y = 0;
}

void osdSetCursor(uint8_t x, uint8_t y)
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

void osdPrintAt(uint8_t x, uint8_t y, char *message)
{
    osdSetCursor(x, y);
    osdPrint(message);
}


void osdDisplaySplash(void)
{
    osdHardwareDrawLogo();

    osdPrintAt(10, 7, "CLEANFLIGHT");
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

void osdInit(void)
{
    osdHardwareInit();

    osdClearScreen();

    osdDisplaySplash();
    osdHardwareUpdate();
}

void osdUpdate(void)
{
    char lineBuffer[31];
    tfp_sprintf(lineBuffer, "RSSI:%3d%%", fcStatus.rssi / 10);
    osdPrintAt(2, 2, lineBuffer);

    char *flightMode;
    if (fcStatus.fcState & (1 << FC_STATE_ANGLE)) {
        flightMode = "ANGL";
    } else if (fcStatus.fcState & (1 << FC_STATE_HORIZON)) {
        flightMode = "HORI";
    } else if (fcStatus.fcState & (1 << FC_STATE_ARM)) {
        flightMode  = "ACRO";
    } else  {
        flightMode  = "OFF";
    }

    tfp_sprintf(lineBuffer, "%1s %1s %4s",
        fcStatus.fcState & (1 << FC_STATE_MAG) ? "M" : "",
        fcStatus.fcState & (1 << FC_STATE_BARO) ? "B" : "",
        flightMode
    );
    osdPrintAt(18, 2, lineBuffer);


/*
    // TODO rework ADC and battery code to provide volt meters
    tfp_sprintf(lineBuffer, "12V:%3d.%dV", voltMeters[0].voltage / 10, voltMeters[0].voltage % 10);
    max7465_print(2, 12, lineBuffer);
    tfp_sprintf(lineBuffer, " 5V:%3d.%dV", voltMeters[1].voltage / 10, voltMeters[1].voltage % 10);
    max7465_print(2, 13, lineBuffer);
*/
    tfp_sprintf(lineBuffer, "BAT:%3d.%dV", vbat / 10, vbat % 10);
    osdPrintAt(18, 12, lineBuffer);
    tfp_sprintf(lineBuffer, " FC:%3d.%dV", fcStatus.vbat / 10, fcStatus.vbat % 10);
    osdPrintAt(18, 13, lineBuffer);
    tfp_sprintf(lineBuffer, "AMP:%2d.%02dA", amperage / 100, amperage % 100);
    osdPrintAt(2, 14, lineBuffer);
    tfp_sprintf(lineBuffer, "mAh:%5d", mAhDrawn);
    osdPrintAt(18, 14, lineBuffer);

    osdHardwareUpdate();
}

