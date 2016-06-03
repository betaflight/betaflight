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
#include "drivers/video_max7456.h"

#include "osd/fc_state.h"

void osdDisplaySplash(void)
{
    max7465_print(9, 10, "CLEANFLIGHT");
}

void osdClearScreen(void)
{
    max7456_clearScreen();
}

void osdInit(void)
{
    osdDisplaySplash();
}

void osdUpdate(void)
{
    debug[2] = max7456_readStatus();

//    if (debug[2] == 0) {
//        max7456_init();
//    }

//    max7456_fillScreen();
//
//    max7465_print(2, 1, "1234567890 ()");
//    max7465_print(2, 2, ".?;:,'/\"-<>@");
//    max7465_print(2, 3, "ABCDEFGHIJKLM");
//    max7465_print(2, 4, "NOPQRSTUVWXYZ");

    char lineBuffer[31];
    tfp_sprintf(lineBuffer, "RSSI:%3d%%", fcStatus.rssi / 10);
    max7465_print(2, 2, lineBuffer);

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
    max7465_print(18, 2, lineBuffer);


/*
    tfp_sprintf(lineBuffer, "12V:%3d.%dV", batteryStates[1].vbat / 10, batteryStates[1].vbat % 10);
    max7465_print(2, 12, lineBuffer);
    tfp_sprintf(lineBuffer, " 5V:%3d.%dV", batteryStates[2].vbat / 10, batteryStates[2].vbat % 10);
    max7465_print(2, 13, lineBuffer);
*/
    tfp_sprintf(lineBuffer, "BAT:%3d.%dV", vbat / 10, vbat % 10);
    max7465_print(18, 12, lineBuffer);
    tfp_sprintf(lineBuffer, " FC:%3d.%dV", fcStatus.vbat / 10, fcStatus.vbat % 10);
    max7465_print(18, 13, lineBuffer);
    tfp_sprintf(lineBuffer, "AMP:%2d.%02dA", amperage / 100, amperage % 100);
    max7465_print(2, 14, lineBuffer);
    tfp_sprintf(lineBuffer, "mAh:%5d", mAhDrawn);
    max7465_print(18, 14, lineBuffer);
}

void osdHardwareCheck(void)
{
    static int checkCount = 0;

    checkCount++;

    if (!max7456_isOSDEnabled()) {
        max7456_init();
    }

    if (checkCount == 5) {
        max7456_clearScreen();
    }

#ifdef FACTORY_TEST
    if (checkCount == 10) {
        max7456_init();
    }
#endif
}
