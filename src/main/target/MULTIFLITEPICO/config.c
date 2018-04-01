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

#include <platform.h>

#ifdef USE_TARGET_CONFIG

#include "common/axis.h"
#include "common/maths.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#define VBAT_SCALE 100

// alternative defaults settings for MULTIFLITEPICO targets
void targetConfiguration(void)
{
    compassConfigMutable()->mag_hardware = MAG_NONE;            // disabled by default

    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = VBAT_SCALE;
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatresdivval = 15;
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatresdivmultiplier = 4;
    batteryConfigMutable()->vbatmaxcellvoltage = 44;
    batteryConfigMutable()->vbatmincellvoltage = 32;
    batteryConfigMutable()->vbatwarningcellvoltage = 33;

    rxConfigMutable()->spektrum_sat_bind = 5;
    rxConfigMutable()->spektrum_sat_bind_autoreset = 1;

    rcControlsConfigMutable()->yaw_deadband = 2;
    rcControlsConfigMutable()->deadband = 2;

    modeActivationConditionsMutable(0)->modeId          = BOXANGLE;
    modeActivationConditionsMutable(0)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(0)->range.endStep   = CHANNEL_VALUE_TO_STEP(1400);
    modeActivationConditionsMutable(1)->modeId          = BOXHORIZON;
    modeActivationConditionsMutable(1)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1425);
    modeActivationConditionsMutable(1)->range.endStep   = CHANNEL_VALUE_TO_STEP(1575);

    failsafeConfigMutable()->failsafe_delay = 2;
    failsafeConfigMutable()->failsafe_off_delay = 0;

    motorConfigMutable()->dev.motorPwmRate = 17000;

    gyroConfigMutable()->gyro_sync_denom = 4;
    pidConfigMutable()->pid_process_denom = 1;

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        pidProfile->pid[PID_ROLL].P = 70;
        pidProfile->pid[PID_ROLL].I = 62;
        pidProfile->pid[PID_ROLL].D = 19;
        pidProfile->pid[PID_PITCH].P = 70;
        pidProfile->pid[PID_PITCH].I = 62;
        pidProfile->pid[PID_PITCH].D = 19;
        pidProfile->pid[PID_LEVEL].I = 40;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        controlRateConfig->rcRates[FD_ROLL] = 70;
        controlRateConfig->rcRates[FD_PITCH] = 70;
    }
}
#endif
