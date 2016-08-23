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
#include <string.h>

#include <platform.h>
#include "build/debug.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/system.h"
#include "drivers/video.h"
#include "drivers/video_textscreen.h"

#include "osd/config.h"

#include "osd/fc_state.h"
#include "osd/msp_client_osd.h"

#include "osd/osd_screen.h"
#include "osd/osd_element.h"
#include "osd/osd.h"

PG_REGISTER(osdFontConfig_t, osdFontConfig, PG_OSD_FONT_CONFIG, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(osdVideoConfig_t, osdVideoConfig, PG_OSD_VIDEO_CONFIG, 0);

PG_REGISTER_WITH_RESET_FN(osdElementConfig_t, osdElementConfig, PG_OSD_ELEMENT_CONFIG, 0);

#ifndef DEFAULT_VIDEO_MODE
#define DEFAULT_VIDEO_MODE VIDEO_AUTO
#endif

PG_RESET_TEMPLATE(osdVideoConfig_t, osdVideoConfig,
    .videoMode = DEFAULT_VIDEO_MODE,
);


static const element_t osdDefaultElements[] = {
    {  2,  1, EF_ENABLED | EF_FLASH_ON_DISCONNECT, OSD_ELEMENT_RSSI_FC },
    { 20,  1, EF_ENABLED | EF_FLASH_ON_DISCONNECT, OSD_ELEMENT_INDICATOR_MAG },
    { 22,  1, EF_ENABLED | EF_FLASH_ON_DISCONNECT, OSD_ELEMENT_INDICATOR_BARO },
    { 24,  1, EF_ENABLED | EF_FLASH_ON_DISCONNECT, OSD_ELEMENT_FLIGHT_MODE },
    {  7, -4, EF_ENABLED, OSD_ELEMENT_ON_DURATION },
    { 18, -4, EF_ENABLED, OSD_ELEMENT_ARMED_DURATION },
    {  2, -3, EF_ENABLED, OSD_ELEMENT_VOLTAGE_12V },
    { 18, -3, EF_ENABLED, OSD_ELEMENT_VOLTAGE_BATTERY },
    {  2, -2, EF_ENABLED, OSD_ELEMENT_VOLTAGE_5V },
    { 18, -2, EF_ENABLED | EF_FLASH_ON_DISCONNECT, OSD_ELEMENT_VOLTAGE_BATTERY_FC },
    {  2, -1, EF_ENABLED, OSD_ELEMENT_AMPERAGE },
    { 18, -1, EF_ENABLED, OSD_ELEMENT_MAH_DRAWN },
};

void pgResetFn_osdElementConfig(osdElementConfig_t *osdElementConfig) {
    memset(osdElementConfig, 0, sizeof(osdElementConfig_t));
    memcpy_fn(&osdElementConfig->elements, &osdDefaultElements, sizeof(osdDefaultElements));
}

osdState_t osdState;

void osdDisplaySplash(void)
{
    osdHardwareDrawLogo();

    osdPrintAt(10, 7, "CLEANFLIGHT");
}

void osdInit(void)
{
    memset(&osdState, 0, sizeof(osdState));

    osdHardwareInit();

    osdClearScreen();

    osdDisplaySplash();
    osdHardwareUpdate();
}

void osdApplyConfiguration(void)
{
    osdHardwareApplyConfiguration(osdVideoConfig()->videoMode);
}

#define OSD_HZ(hz) ((int32_t)((1000 * 1000) / (hz)))
#define OSD_MS(ms) ((int32_t)(1000 * (ms)))

typedef enum {
    tim1Hz,
    tim2Hz,
    tim5Hz,
    tim10Hz,
    timTimerCount
} timId_e;

typedef struct timerState_s {
    uint32_t val;
    bool toggle;  // toggled when the timer is updated/due.
    uint8_t counter;  // incremented when the timer is updated/due.
} timerState_t;

timerState_t timerState[timTimerCount];

static const struct {
    int8_t timId;
    uint32_t delay;
} timerTable[] = {
    // LAYER 1
    { tim1Hz,  OSD_HZ(1)  }, // for slow flashing messages                         (e.g. camera not connected)
    { tim2Hz,  OSD_HZ(2)  }, // for low urgency alarms                             (e.g. altitude to high)
    { tim5Hz,  OSD_HZ(5)  }, // for medium urgency alarms                          (e.g. battery low)
    { tim10Hz, OSD_HZ(10) }, // for high urgency alarms and eye-catching effects.  (e.g. fc disconnection, battery critical)
};


// 4x4 grid
struct quadMotorCoordinateOffset_s {
    uint8_t x;
    uint8_t y;
} quadMotorCoordinateOffsets[4] = {
    {3, 3},
    {3, 1},
    {0, 3},
    {0, 1}
};

bool osdIsCameraConnected(void)
{
    return osdState.cameraConnected;
}

void osdDisplayMotors(void)
{
    const int maxMotors = 4; // just quad for now
    for (int i = 0; i < maxMotors; i++) {
        if (!fcMotors[i]) {
            continue; // skip unused/uninitialsed motors.
        }
        int percent = scaleRange(fcMotors[i], 1000, 2000, 0, 100); // FIXME should use min/max command as used by the FC.

        osdHardwareDisplayMotor(quadMotorCoordinateOffsets[i].x, quadMotorCoordinateOffsets[i].y, percent);
    }
}

void osdUpdate(void)
{
    TIME_SECTION_BEGIN(0);

    uint32_t now = micros();

    static bool armed = false;
    static uint32_t armedAt = 0;

    bool armedNow = fcStatus.fcState & (1 << FC_STATE_ARM);
    if (armed != armedNow) {
        if (armedNow) {
            armedAt = now;
        }

        armed = armedNow;
    }

    if (armed) {
        if (now > armedAt) {

            fcStatus.armedDuration = (now - armedAt) / 1000;
        }
    }

    osdClearScreen();


    // test all timers, setting corresponding bits
    uint32_t timActive = 0;
    for(timId_e timId = 0; timId < timTimerCount; timId++) {
        if(cmp32(now, timerState[timId].val) < 0)
            continue;  // not ready yet
        timActive |= 1 << timId;
        // sanitize timer value, so that it can be safely incremented. Handles inital timerVal value.
        // max delay is limited to 5s
        if(cmp32(now, timerState[timId].val) >= OSD_MS(100) || cmp32(now, timerState[timId].val) < OSD_HZ(5000) ) {
            timerState[timId].val = now;
        }
    }

    // update all timers
    for(unsigned i = 0; i < ARRAYLEN(timerTable); i++) {
        int timId = timerTable[i].timId;

        bool updateNow = timActive & (1 << timId);
        if (!updateNow) {
            continue;
        }
        timerState[timId].toggle = !timerState[timId].toggle;
        timerState[timId].counter++;
        timerState[timId].val += timerTable[i].delay;
    }

    bool showNowOrFlashWhenFCTimeoutOccured = !mspClientStatus.timeoutOccured || (mspClientStatus.timeoutOccured && timerState[tim10Hz].toggle);

    osdSetElementFlashOnDisconnectState(showNowOrFlashWhenFCTimeoutOccured);

    for (uint8_t elementIndex = 0; elementIndex < MAX_OSD_ELEMENT_COUNT; elementIndex++) {
        element_t *element = &osdElementConfig()->elements[elementIndex];

        osdDrawTextElement(element);
    }

    //
    // flash the logo for a few seconds
    // then leave it on for a few more before turning it off
    //

    static bool splashDone = false;
    if (!splashDone) {
        if (
            (timerState[tim1Hz].counter > 3 && timerState[tim1Hz].counter < 6) ||
            (timerState[tim1Hz].counter <= 3 && timerState[tim10Hz].toggle)
        ) {
            osdDisplaySplash();
        }

        if (timerState[tim1Hz].counter >= 10) {
            splashDone = true;
        }
    }

    if (timerState[tim2Hz].toggle && splashDone) {
        if (!osdIsCameraConnected()) {
            osdPrintAt(11, 4, "NO CAMERA");
        }
    }

    osdDisplayMotors();

    TIME_SECTION_END(0);

    osdHardwareUpdate();

}

