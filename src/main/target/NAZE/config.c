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

#include <stdint.h>

#include <platform.h>

#include "build_config.h"

#include "blackbox/blackbox_io.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "drivers/pwm_output.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
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

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/altitudehold.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "config/config_profile.h"
#include "config/config_master.h"

#ifdef BEEBRAIN
// alternative defaults settings for Beebrain target
void targetConfiguration(master_t *config)
{
    config->motor_pwm_rate = 4000;
    config->failsafeConfig.failsafe_delay = 2;
    config->failsafeConfig.failsafe_off_delay = 0;

    config->escAndServoConfig.minthrottle = 1049;

    config->gyro_lpf = 1;
    config->gyro_soft_lpf_hz = 100;
    config->gyro_soft_notch_hz_1 = 0;

    for (int channel = 0; channel < NON_AUX_CHANNEL_COUNT; channel++) {
        config->rxConfig.channelRanges[channel].min = 1180;
        config->rxConfig.channelRanges[channel].max = 1860;
    }

    for (int profileId = 0; profileId < 2; profileId++) {
        config->profile[profileId].pidProfile.P8[ROLL] = 55;
        config->profile[profileId].pidProfile.I8[ROLL] = 40;
        config->profile[profileId].pidProfile.D8[ROLL] = 20;
        config->profile[profileId].pidProfile.P8[PITCH] = 55;
        config->profile[profileId].pidProfile.I8[PITCH] = 40;
        config->profile[profileId].pidProfile.D8[PITCH] = 20;
        config->profile[profileId].pidProfile.P8[YAW] = 180;
        config->profile[profileId].pidProfile.I8[YAW] = 45;
        config->profile[profileId].pidProfile.P8[PIDLEVEL] = 50;
        config->profile[profileId].pidProfile.D8[PIDLEVEL] = 50;
        config->profile[profileId].pidProfile.levelSensitivity = 1.0f;

        for (int rateProfileId = 0; rateProfileId < MAX_RATEPROFILES; rateProfileId++) {
            config->profile[profileId].controlRateProfile[rateProfileId].rcRate8 = 120;
            config->profile[profileId].controlRateProfile[rateProfileId].rcYawRate8 = 120;
            config->profile[profileId].controlRateProfile[rateProfileId].rates[ROLL] = 99;
            config->profile[profileId].controlRateProfile[rateProfileId].rates[PITCH] = 99;
            config->profile[profileId].controlRateProfile[rateProfileId].rates[YAW] = 99;

            config->profile[profileId].pidProfile.setpointRelaxRatio = 100;
        }
    }
}
#endif
