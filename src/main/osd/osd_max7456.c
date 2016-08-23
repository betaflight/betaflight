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
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/video.h"
#include "drivers/video_textscreen.h"
#include "drivers/video_max7456.h"

#include "osd/config.h"
#include "osd/osd_element.h"
#include "osd/osd_screen.h"

#include "osd/fonts/font_max7456_12x18.h"

#include "osd/osd.h"


TEXT_SCREEN_CHAR textScreenBuffer[MAX7456_PAL_CHARACTER_COUNT]; // PAL has more characters than NTSC.
const uint8_t *asciiToFontMapping = &font_max7456_12x18_asciiToFontMapping[0];

#ifdef STM32F303
static const extiConfig_t max7456LOSExtiConfig = {
        .gpioAHBPeripherals = MAX7456_LOS_GPIO_PERIPHERAL,
        .gpioPort = MAX7456_LOS_GPIO,
        .gpioPin = MAX7456_LOS_PIN,
        .io = IO_TAG(MAX7456_LOS_IO),
};

static const extiConfig_t max7456VSYNCExtiConfig = {
        .gpioAHBPeripherals = MAX7456_VSYNC_GPIO_PERIPHERAL,
        .gpioPort = MAX7456_VSYNC_GPIO,
        .gpioPin = MAX7456_VSYNC_PIN,
        .io = IO_TAG(MAX7456_VSYNC_IO),
};

static const extiConfig_t max7456HSYNCExtiConfig = {
        .gpioAHBPeripherals = MAX7456_HSYNC_GPIO_PERIPHERAL,
        .gpioPort = MAX7456_HSYNC_GPIO,
        .gpioPin = MAX7456_HSYNC_PIN,
        .io = IO_TAG(MAX7456_HSYNC_IO),
};
#endif

#ifdef STM32F10X
static const extiConfig_t max7456LOSExtiConfig = {
        .gpioAPB2Peripherals = MAX7456_LOS_GPIO_PERIPHERAL,
        .gpioPort = MAX7456_LOS_GPIO,
        .gpioPin = MAX7456_LOS_PIN,
        .io = IO_TAG(MAX7456_LOS_IO),
};

static const extiConfig_t max7456VSYNCExtiConfig = {
        .gpioAPB2Peripherals = MAX7456_VSYNC_GPIO_PERIPHERAL,
        .gpioPort = MAX7456_VSYNC_GPIO,
        .gpioPin = MAX7456_VSYNC_PIN,
        .io = IO_TAG(MAX7456_VSYNC_IO),
};

static const extiConfig_t max7456HSYNCExtiConfig = {
        .gpioAPB2Peripherals = MAX7456_HSYNC_GPIO_PERIPHERAL,
        .gpioPort = MAX7456_HSYNC_GPIO,
        .gpioPin = MAX7456_HSYNC_PIN,
        .io = IO_TAG(MAX7456_HSYNC_IO),
};
#endif

void osdHardwareApplyConfiguration(videoMode_e videoMode)
{
    max7456_init(videoMode);

    textScreen_t *max7456TextScreen = max7456_getTextScreen();
    osdSetTextScreen(max7456TextScreen);
}

void osdHardwareInit(void)
{
    LED0_ON;
    delay(500);
    max7456_hardwareReset();
    LED0_OFF;

    osdHardwareApplyConfiguration(osdVideoConfig()->videoMode);

    max7456_extiConfigure(&max7456LOSExtiConfig, &max7456VSYNCExtiConfig, &max7456HSYNCExtiConfig);

    if (osdFontConfig()->fontVersion != FONT_VERSION) {
        // before
        max7456_showFont();
        delay(5000); // give the user a chance to power off before changing

        max7456_resetFont();

        // after
        max7456_showFont();
        delay(5000); // give the user a chance to power off after changing

        osdFontConfig()->fontVersion = FONT_VERSION;
        writeEEPROM();

    	max7456_clearScreen();
    	max7456_ensureDisplayClearIsComplete();
    }
}

void osdHardwareUpdate(void)
{
    max7456_updateStatus();

    osdState.videoMode = max7456State.detectedVideoMode;
    osdState.cameraConnected = !max7456State.los;

    max7456_writeScreen(&osdTextScreen, textScreenBuffer);
}

void osdHardwareCheck(void)
{
    videoMode_e desiredVideoMode = osdVideoConfig()->videoMode;
    if (!max7456_isOSDEnabled()) {
        max7456_init(desiredVideoMode);
    }

    max7456_updateStatus();

    bool correctVideoMode = max7456State.configuredVideoMode == max7456State.detectedVideoMode;

    if (correctVideoMode && !max7456State.los) {
        max7456State.useSync = true;
    } else {
        max7456State.useSync = false;
    }

    if (!max7456State.los && max7456State.detectedVideoMode != VIDEO_AUTO) {
        // there is a valid video mode
        if (desiredVideoMode == VIDEO_AUTO && !correctVideoMode) {
            osdHardwareApplyConfiguration(max7456State.detectedVideoMode);
        };
    }

#ifdef FACTORY_TEST
    static int checkCount = 0;

    checkCount++;

    if (checkCount == 10) {
        max7456_init(desiredVideoMode);
    }
#endif
}

static const uint8_t logoElement[] = {
	FONT_CHARACTER_CF_LOGO_W3xH2__1x1,
	FONT_CHARACTER_CF_LOGO_W3xH2__1x2,
	FONT_CHARACTER_CF_LOGO_W3xH2__1x3,
	FONT_CHARACTER_CF_LOGO_W3xH2__2x1,
	FONT_CHARACTER_CF_LOGO_W3xH2__2x2,
	FONT_CHARACTER_CF_LOGO_W3xH2__2x3
};

static void osdDrawStaticElement(uint8_t x, uint8_t y, const uint8_t element[], uint8_t elementLength, uint8_t width) {
	uint8_t w = 0;

	for (uint8_t i = 0; i < elementLength; i++) {
	    osdSetRawCharacterAtPosition(x + w, y, element[i]);
	    w++;
	    if (w == width) {
	    	w = 0;
	    	y++;
	    }
	}
}

void osdHardwareDrawLogo(void)
{
	osdDrawStaticElement(14,5, logoElement, ARRAYLEN(logoElement), 3);
}

void osdHardwareDisplayMotor(uint8_t x, uint8_t y, uint8_t percent)
{
    uint8_t c = FONT_CHARACTER_MOTOR_OFF - (MIN(percent, 99) / 10);

    osdSetRawCharacterAtPosition(13 + x, osdTextScreen.height - 4 + y, c);
}


