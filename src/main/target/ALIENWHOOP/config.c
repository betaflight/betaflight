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




         \   |   _ _| __|  \ |\ \      /|  |  _ \  _ \ _ \
        _ \  |     |  _|  .  | \ \ \  / __ | (   |(   |__/
      _/  _\____|___|___|_|\_|  \_/\_/ _| _|\___/\___/_|


              Take me to your leader-board...



*/

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#ifdef TARGET_CONFIG

#include "fc/rc_modes.h"
#include "common/axis.h"
#include "config/feature.h"
#include "drivers/pwm_esc_detect.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "io/beeper.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"

#ifdef BRUSHED_MOTORS_PWM_RATE
#undef BRUSHED_MOTORS_PWM_RATE
#endif

#define BRUSHED_MOTORS_PWM_RATE 666           // 666Hz };-)>~ low PWM rate seems to give better power and cooler motors...


void targetConfiguration(void)
{
    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfigMutable()->minthrottle = 1080;
        motorConfigMutable()->maxthrottle = 2000;
    }

    /* Default to Spektrum */
    rxConfigMutable()->serialrx_provider = SERIALRX_SPEKTRUM2048;
    rxConfigMutable()->spektrum_sat_bind = 5;
    rxConfigMutable()->spektrum_sat_bind_autoreset = 1;
    parseRcChannels("TAER1234", rxConfigMutable());
#if defined(ALIENWHOOPF4)
    rxConfigMutable()->sbus_inversion = 0; // TODO: what to do about F4 inversion?
#else
    rxConfigMutable()->sbus_inversion = 1; // invert on F7
#endif

    beeperOffSet((BEEPER_BAT_CRIT_LOW | BEEPER_BAT_LOW | BEEPER_RX_SET) ^ BEEPER_GYRO_CALIBRATED);

    /* Breadboard-specific settings for development purposes only
     */
#if defined(BREADBOARD)
    boardAlignmentMutable()->pitchDegrees = 90; // vertical breakout board
    barometerConfigMutable()->baro_hardware = BARO_DEFAULT; // still testing not on V1 or V2 pcb
#else
    barometerConfigMutable()->baro_hardware = BARO_NONE;
#endif

    compassConfigMutable()->mag_hardware =  MAG_DEFAULT;

    /* F4 (especially overclocked) and F7 ALIENWHOOP perform splendidly with 32kHz gyro enabled */
    gyroConfigMutable()->gyro_use_32khz = 1;
    gyroConfigMutable()->gyro_sync_denom = 2;  // 16kHz gyro
    pidConfigMutable()->pid_process_denom = 1; // 16kHz PID

    featureSet((FEATURE_DYNAMIC_FILTER | FEATURE_AIRMODE | FEATURE_ANTI_GRAVITY) ^ FEATURE_RX_PARALLEL_PWM);

    /* AlienWhoop PIDs based on Ole Gravy Leg (aka Matt Williamson's) PIDs 
     */
    for (int profileId = 0; profileId < MAX_PROFILE_COUNT; profileId++) {
        /* AlienWhoop PIDs tested with 6mm and 7mm motors on most frames */
        pidProfilesMutable(profileId)->pid[PID_PITCH].P = 75;
        pidProfilesMutable(profileId)->pid[PID_PITCH].I = 36;
        pidProfilesMutable(profileId)->pid[PID_PITCH].D = 25;
        pidProfilesMutable(profileId)->pid[PID_ROLL].P = 75;
        pidProfilesMutable(profileId)->pid[PID_ROLL].I = 36;
        pidProfilesMutable(profileId)->pid[PID_ROLL].D = 25;
        pidProfilesMutable(profileId)->pid[PID_YAW].P = 70;
        pidProfilesMutable(profileId)->pid[PID_YAW].I = 36;

        pidProfilesMutable(profileId)->pid[PID_LEVEL].P = 30;
        pidProfilesMutable(profileId)->pid[PID_LEVEL].D = 30;

        /* Setpoints */
        pidProfilesMutable(profileId)->dtermSetpointWeight = 100;
        pidProfilesMutable(profileId)->setpointRelaxRatio = 100; // default to snappy for racers

        /* Throttle PID Attenuation (TPA) */
        pidProfilesMutable(profileId)->itermThrottleThreshold = 400;
    }

    for (int rateProfileId = 0; rateProfileId < CONTROL_RATE_PROFILE_COUNT; rateProfileId++) {
        /* RC Rates */
        controlRateProfilesMutable(rateProfileId)->rcRate8 = 100;
        controlRateProfilesMutable(rateProfileId)->rcYawRate8 = 100;
        controlRateProfilesMutable(rateProfileId)->rcExpo8 = 0;

        /* Super Expo Rates */
        controlRateProfilesMutable(rateProfileId)->rates[FD_ROLL] = 80;
        controlRateProfilesMutable(rateProfileId)->rates[FD_PITCH] = 80;
        controlRateProfilesMutable(rateProfileId)->rates[FD_YAW] = 85;

        /* Throttle PID Attenuation (TPA) */
        controlRateProfilesMutable(rateProfileId)->dynThrPID = 0; // tpa_rate off
        controlRateProfilesMutable(rateProfileId)->tpa_breakpoint = 1600;
    }
}
#endif
