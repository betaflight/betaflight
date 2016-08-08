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
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef VTX

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "drivers/vtx_rtc6705.h"


#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/ledstrip.h"
#include "io/gps.h"
#include "io/vtx.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/runtime_config.h"

static uint8_t locked = 0;

void vtxInit(void)
{
    rtc6705Init();
    if (masterConfig.vtx_mode == 0) {
        rtc6705SetChannel(masterConfig.vtx_band, masterConfig.vtx_channel);
    } else if (masterConfig.vtx_mode == 1) {
        rtc6705SetFreq(masterConfig.vtx_mhz);
    }
}

static void setChannelSaveAndNotify(uint8_t *bandOrChannel, uint8_t step, int32_t min, int32_t max)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (masterConfig.vtx_mode == 0 && !locked) {
        uint8_t temp = (*bandOrChannel) + step;
        temp = constrain(temp, min, max);
        *bandOrChannel = temp;

        rtc6705SetChannel(masterConfig.vtx_band, masterConfig.vtx_channel);
        writeEEPROM();
        readEEPROM();
        beeperConfirmationBeeps(temp);
    }
}

void vtxIncrementBand(void)
{
    setChannelSaveAndNotify(&(masterConfig.vtx_band), 1, RTC6705_BAND_MIN, RTC6705_BAND_MAX);
}

void vtxDecrementBand(void)
{
    setChannelSaveAndNotify(&(masterConfig.vtx_band), -1, RTC6705_BAND_MIN, RTC6705_BAND_MAX);
}

void vtxIncrementChannel(void)
{
    setChannelSaveAndNotify(&(masterConfig.vtx_channel), 1, RTC6705_CHANNEL_MIN, RTC6705_CHANNEL_MAX);
}

void vtxDecrementChannel(void)
{
    setChannelSaveAndNotify(&(masterConfig.vtx_channel), -1, RTC6705_CHANNEL_MIN, RTC6705_CHANNEL_MAX);
}

void vtxUpdateActivatedChannel(void)
{
    if (ARMING_FLAG(ARMED)) {
        locked = 1;
    }

    if (masterConfig.vtx_mode == 2 && !locked) {
        static uint8_t lastIndex = -1;
        uint8_t index;

        for (index = 0; index < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; index++) {
            vtxChannelActivationCondition_t *vtxChannelActivationCondition = &masterConfig.vtxChannelActivationConditions[index];

            if (isRangeActive(vtxChannelActivationCondition->auxChannelIndex, &vtxChannelActivationCondition->range)
                && index != lastIndex) {
                lastIndex = index;
                rtc6705SetChannel(vtxChannelActivationCondition->band, vtxChannelActivationCondition->channel);
                break;
            }
        }
    }
}

#endif

