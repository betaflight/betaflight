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
#include <math.h>

#include <platform.h>
#include "scheduler/scheduler.h"
#include "build/debug.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"
#include "common/filter.h"
#include "common/printf.h"
#include "common/streambuf.h"

#include "drivers/light_led.h"

#include "drivers/buf_writer.h"
#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/video_textscreen.h"

#include "fc/rc_controls.h" // FIXME required due to virtual current meter.

#include "sensors/battery.h"

#include "io/statusindicator.h"
#include "io/serial.h"

#include "msp/msp.h"
#include "msp/msp_serial.h"

#include "osd/msp_server_osd.h"
#include "osd/msp_client_osd.h"

#include "osd/osd.h"

#include "osd/config.h"

/* VBAT monitoring interval (in microseconds) - 1s*/
#define VBATINTERVAL (6 * 3500)
/* IBat monitoring interval (in microseconds) - 6 default looptimes */
#define IBATINTERVAL (6 * 3500)

uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little

extern uint32_t currentTime;    // from scheduler

void taskUpdateCycleTime(void)
{
    cycleTime = getTaskDeltaTime(TASK_SELF);
}

void taskTest(void)
{
}

void taskUpdateFCState(void)
{
    mspClientProcess();
}

void taskDrawScreen(void)
{
    osdUpdate();
}

void taskHardwareWatchdog(void)
{
    osdHardwareCheck();
}

void taskStatusLed(void)
{
    if (mspClientStatus.timeoutOccured) {
        warningLedCode(3);
    } else if (!osdIsCameraConnected()) {
        warningLedCode(2);
    } else {
        warningLedPulse();
    }
    warningLedUpdate();
}

void taskMSP(void)
{
    mspSerialProcess();
}

void taskUpdateBattery(void)
{
    static uint32_t vbatLastServiced = 0;
    static uint32_t ibatLastServiced = 0;

    if (cmp32(currentTime, vbatLastServiced) >= VBATINTERVAL) {
        vbatLastServiced = currentTime;
        updateBattery();
    }

    int32_t ibatTimeSinceLastServiced = cmp32(currentTime, ibatLastServiced);

    if (ibatTimeSinceLastServiced >= IBATINTERVAL) {
        ibatLastServiced = currentTime;

        updateCurrentMeter(ibatTimeSinceLastServiced);
    }
}
