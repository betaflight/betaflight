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

#include "stdbool.h"
#include "stdint.h"

#include "platform.h"

#include "build_config.h"

#ifdef DISPLAY

#include "drivers/system.h"
#include "drivers/display_ug2864hsweg01.h"

#include "display.h"

#define TEST_I2C_DISPLAY

#ifdef TEST_I2C_DISPLAY
static const char *messages[] = {
        "CLEANFLIGHT",
        " DISPLAY   ",
        "  TESTING  "
};
static uint8_t messageIndex = 0;
static uint8_t messageCount = 3;
static uint8_t rowIndex = 0;
static uint8_t rowCount = 8;
#endif

#define DISPLAY_STRIP_10HZ ((1000 * 1000) / 10)

uint32_t nextDisplayUpdateAt = 0;

void updateDisplay(void)
{
    uint32_t now = micros();

    bool updateNow = (int32_t)(now - nextDisplayUpdateAt) >= 0L;
    if (!updateNow) {
        return;
    }

    nextDisplayUpdateAt = now + DISPLAY_STRIP_10HZ;

#ifdef TEST_I2C_DISPLAY
    i2c_OLED_set_line(rowIndex++);
    i2c_OLED_send_string(messages[messageIndex++]);
    messageIndex = messageIndex % messageCount;
    rowIndex = rowIndex % rowCount;
#endif
}


void displayInit(void)
{
    delay(20);
    ug2864hsweg01InitI2C();
}

#endif
