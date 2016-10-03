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

// alternative defaults settings for COLIBRI RACE targets
void targetConfiguration(master_t *config) {
    config->escAndServoConfig.minthrottle = 1025;
    config->escAndServoConfig.maxthrottle = 1980;
    config->escAndServoConfig.mincommand = 1000;
    config->escAndServoConfig.servoCenterPulse = 1500;

    config->batteryConfig.vbatmaxcellvoltage = 45;
    config->batteryConfig.vbatmincellvoltage = 30;
    config->batteryConfig.vbatwarningcellvoltage = 35;

    config->flight3DConfig.deadband3d_low = 1406;
    config->flight3DConfig.deadband3d_high = 1514;
    config->flight3DConfig.neutral3d = 1460;
    config->flight3DConfig.deadband3d_throttle = 0;
    
    config->failsafeConfig.failsafe_procedure = 1;
    config->failsafeConfig.failsafe_throttle_low_delay = 10;

    config->gyro_sync_denom = 1;
    config->pid_process_denom = 3;
    config->blackbox_rate_num = 1;
    config->blackbox_rate_denom = 1;

    config->rcControlsConfig.deadband = 5;
    config->rcControlsConfig.yaw_deadband = 5;

    config->failsafeConfig.failsafe_delay = 10;

    config->telemetryConfig.telemetry_inversion = 1;

    config->pid_process_denom = 4;

    config->profile[0].pidProfile.vbatPidCompensation = 1;

    config->profile[0].pidProfile.P8[ROLL] = 46;     // new PID with preliminary defaults test carefully
    config->profile[0].pidProfile.I8[ROLL] = 48;
    config->profile[0].pidProfile.D8[ROLL] = 23;
    config->profile[0].pidProfile.P8[PITCH] = 89;
    config->profile[0].pidProfile.I8[PITCH] = 59;
    config->profile[0].pidProfile.D8[PITCH] = 25;
    config->profile[0].pidProfile.P8[YAW] = 129;
    config->profile[0].pidProfile.I8[YAW] = 50;
    config->profile[0].pidProfile.D8[YAW] = 20;

    config->profile[0].controlRateProfile[0].rates[FD_ROLL] = 86;
    config->profile[0].controlRateProfile[0].rates[FD_PITCH] = 86;
    config->profile[0].controlRateProfile[0].rates[FD_YAW] = 80;


    featureSet(FEATURE_VBAT);
    featureSet(FEATURE_RX_SERIAL);
    featureSet(FEATURE_FAILSAFE);
    featureSet(FEATURE_AIRMODE);
}
