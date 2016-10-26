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
#include "drivers/compass.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"
#include "drivers/pwm_output.h"
#include "drivers/max7456.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"

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
#include "io/osd.h"
#include "io/vtx.h"

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

// alternative defaults settings for PICOBFC targets
void targetConfiguration(master_t *config) {
    config->mag_hardware = MAG_NONE;            // disabled by default
    
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
     
    config->modeActivationConditions[0].modeId          = BOXANGLE;
    config->modeActivationConditions[0].auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    config->modeActivationConditions[0].range.startStep = CHANNEL_VALUE_TO_STEP(900);
    config->modeActivationConditions[0].range.endStep   = CHANNEL_VALUE_TO_STEP(1400);
    config->modeActivationConditions[1].modeId          = BOXHORIZON;
    config->modeActivationConditions[1].auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    config->modeActivationConditions[1].range.startStep = CHANNEL_VALUE_TO_STEP(1425);
    config->modeActivationConditions[1].range.endStep   = CHANNEL_VALUE_TO_STEP(1575);
    
    config->failsafeConfig.failsafe_delay = 2;
    config->failsafeConfig.failsafe_off_delay = 0;
    
    config->motor_pwm_rate = 17000;
    
    config->gyro_sync_denom = 4;
    config->pid_process_denom = 1;
    
    config->profile[0].pidProfile.P8[ROLL] = 58;
    config->profile[0].pidProfile.I8[ROLL] = 62;
    config->profile[0].pidProfile.D8[ROLL] = 19;
    config->profile[0].pidProfile.P8[PITCH] = 58;
    config->profile[0].pidProfile.I8[PITCH] = 62;
    config->profile[0].pidProfile.D8[PITCH] = 19;
    
    config->profile[0].controlRateProfile[0].rcRate8 = 70;
    
}
