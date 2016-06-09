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

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/video_textscreen.h"
#include "drivers/video_max7456.h"

#include "osd/config.h"

#include "osd/fonts/font_max7456_12x18.h"

#include "osd/osd.h"

char textScreenBuffer[MAX7456_PAL_CHARACTER_COUNT]; // PAL has more characters than NTSC.
const uint8_t *asciiToFontMapping = &font_max7456_12x18_asciiToFontMapping[0];

void osdHardwareInit(void)
{
    LED0_ON;
    delay(500);
    max7456_hardwareReset();
    LED0_OFF;

    max7456_init();

    textScreen_t *max7456TextScreen = max7456_getTextScreen();
    osdSetTextScreen(max7456TextScreen);

    if (osdFontConfig()->fontVersion != FONT_VERSION) {
        // before
        max7456_showFont();
        delay(5000);

        max7456_resetFont();

        // after
        max7456_showFont();
        delay(5000);

        osdFontConfig()->fontVersion = FONT_VERSION;
        writeEEPROM();

        max7456_clearScreen();
    }
}

void osdHardwareUpdate(void)
{
    debug[2] = max7456_readStatus();

    max7456_writeScreen(&osdTextScreen, textScreenBuffer);
}

void osdHardwareCheck(void)
{
    static int checkCount = 0;

    checkCount++;

    if (!max7456_isOSDEnabled()) {
        max7456_init();
    }

    if (checkCount == 5) {
        osdClearScreen();
    }

#ifdef FACTORY_TEST
    if (checkCount == 10) {
        max7456_init();
    }
#endif
}

void osdHardwareDrawLogo(void)
{
    osdSetRawCharacterAtPosition(14, 5, FONT_CHARACTER_CF_LOGO_W3xH2__1x1);
    osdSetRawCharacterAtPosition(15, 5, FONT_CHARACTER_CF_LOGO_W3xH2__1x2);
    osdSetRawCharacterAtPosition(16, 5, FONT_CHARACTER_CF_LOGO_W3xH2__1x3);
    osdSetRawCharacterAtPosition(14, 6, FONT_CHARACTER_CF_LOGO_W3xH2__2x1);
    osdSetRawCharacterAtPosition(15, 6, FONT_CHARACTER_CF_LOGO_W3xH2__2x2);
    osdSetRawCharacterAtPosition(16, 6, FONT_CHARACTER_CF_LOGO_W3xH2__2x3);
}
