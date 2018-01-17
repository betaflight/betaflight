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

#ifdef USE_TARGET_CONFIG

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
    rxConfigMutable()->serialrx_inverted = true; // TODO: what to do about F4 inversion?
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
    for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

        /* AlienWhoop PIDs tested with 6mm and 7mm motors on most frames */
        pidProfile->pid[PID_PITCH].P = 75;
        pidProfile->pid[PID_PITCH].I = 36;
        pidProfile->pid[PID_PITCH].D = 25;
        pidProfile->pid[PID_ROLL].P = 75;
        pidProfile->pid[PID_ROLL].I = 36;
        pidProfile->pid[PID_ROLL].D = 25;
        pidProfile->pid[PID_YAW].P = 70;
        pidProfile->pid[PID_YAW].I = 36;

        pidProfile->pid[PID_LEVEL].P = 30;
        pidProfile->pid[PID_LEVEL].D = 30;

        /* Setpoints */
        pidProfile->dtermSetpointWeight = 100;
        pidProfile->setpointRelaxRatio = 100; // default to snappy for racers

        /* Throttle PID Attenuation (TPA) */
        pidProfile->itermThrottleThreshold = 400;
    }

    for (uint8_t rateProfileIndex = 0; rateProfileIndex < CONTROL_RATE_PROFILE_COUNT; rateProfileIndex++) {
        controlRateConfig_t *controlRateConfig = controlRateProfilesMutable(rateProfileIndex);

        /* RC Rates */
        controlRateConfig->rcRates[FD_ROLL] = 100;
        controlRateConfig->rcRates[FD_PITCH] = 100;
        controlRateConfig->rcRates[FD_YAW] = 100;
        controlRateConfig->rcExpo[FD_ROLL] = 0;
        controlRateConfig->rcExpo[FD_PITCH] = 0;

        /* Super Expo Rates */
        controlRateConfig->rates[FD_ROLL] = 80;
        controlRateConfig->rates[FD_PITCH] = 80;
        controlRateConfig->rates[FD_YAW] = 85;

        /* Throttle PID Attenuation (TPA) */
        controlRateConfig->dynThrPID = 0; // tpa_rate off
        controlRateConfig->tpa_breakpoint = 1600;
    }
}
#endif
