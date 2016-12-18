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

#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/compass.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"

#include "config/config_profile.h"
#include "config/config_master.h"

// alternative defaults settings for MULTIFLITEPICO targets
void targetConfiguration(master_t *config)
{
    config->compassConfig.mag_hardware = MAG_NONE;            // disabled by default

    config->batteryConfig.vbatscale = 100;
    config->batteryConfig.vbatresdivval = 15;
    config->batteryConfig.vbatresdivmultiplier = 4;
    config->batteryConfig.vbatmaxcellvoltage = 44;
    config->batteryConfig.vbatmincellvoltage = 32;
    config->batteryConfig.vbatwarningcellvoltage = 33;

    config->rxConfig.spektrum_sat_bind = 5;
    config->rxConfig.spektrum_sat_bind_autoreset = 1;

    config->rcControlsConfig.yaw_deadband = 2;
    config->rcControlsConfig.deadband = 2;

    config->modeActivationProfile.modeActivationConditions[0].modeId          = BOXANGLE;
    config->modeActivationProfile.modeActivationConditions[0].auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    config->modeActivationProfile.modeActivationConditions[0].range.startStep = CHANNEL_VALUE_TO_STEP(900);
    config->modeActivationProfile.modeActivationConditions[0].range.endStep   = CHANNEL_VALUE_TO_STEP(1400);
    config->modeActivationProfile.modeActivationConditions[1].modeId          = BOXHORIZON;
    config->modeActivationProfile.modeActivationConditions[1].auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    config->modeActivationProfile.modeActivationConditions[1].range.startStep = CHANNEL_VALUE_TO_STEP(1425);
    config->modeActivationProfile.modeActivationConditions[1].range.endStep   = CHANNEL_VALUE_TO_STEP(1575);

    config->failsafeConfig.failsafe_delay = 2;
    config->failsafeConfig.failsafe_off_delay = 0;

    config->motorConfig.motorPwmRate = 17000;

    config->gyroConfig.gyro_sync_denom = 4;
    config->pidConfig.pid_process_denom = 1;

    config->profile[0].pidProfile.P8[ROLL] = 70;
    config->profile[0].pidProfile.I8[ROLL] = 62;
    config->profile[0].pidProfile.D8[ROLL] = 19;
    config->profile[0].pidProfile.P8[PITCH] = 70;
    config->profile[0].pidProfile.I8[PITCH] = 62;
    config->profile[0].pidProfile.D8[PITCH] = 19;

    config->profile[0].controlRateProfile[0].rcRate8 = 70;
    config->profile[0].pidProfile.I8[PIDLEVEL] = 40;

}
