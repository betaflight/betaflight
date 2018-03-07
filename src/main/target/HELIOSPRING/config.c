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

#include "config/config_eeprom.h"
#include "drivers/pwm_output.h"
#include "common/filter.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "fc/config.h"
#include "rx/rx.h"

void targetConfiguration(void) {
    voltageSensorADCConfigMutable(VOLTAGE_SENSOR_ADC_VBAT)->vbatscale = VBAT_SCALE;
    rxConfigMutable()->rcInterpolation = 3;
    rxConfigMutable()->rcInterpolationInterval = 14;
    rxConfigMutable()->rcInterpolationChannels = 2;
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_MULTISHOT;
    pidConfigMutable()->pid_process_denom = 1; // 16kHz PID
    gyroConfigMutable()->gyro_use_32khz = 1;
    gyroConfigMutable()->gyro_sync_denom = 2;  // 16kHz gyro
    gyroConfigMutable()->gyro_soft_lpf_hz = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    gyroConfigMutable()->gyroMovementCalibrationThreshold = 5;

    for (uint8_t pidProfileIndex = 0; pidProfileIndex < MAX_PROFILE_COUNT; pidProfileIndex++) {
        pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

    #if  defined(HELIO_RACE)
        //optimizng for strech-x
        pidProfile->pid[PID_PITCH].P = 30;	
        pidProfile->pid[PID_PITCH].I = 60;	
        pidProfile->pid[PID_PITCH].D = 17;	
        pidProfile->pid[PID_ROLL].P = 28;	
        pidProfile->pid[PID_ROLL].I = 70;	
        pidProfile->pid[PID_ROLL].D = 20;
    #elif defined(HELIO_FREESTYLE)
        //optimizng for squished-x
        pidProfile->pid[PID_PITCH].P = 40;	
        pidProfile->pid[PID_PITCH].I = 55;	
        pidProfile->pid[PID_PITCH].D = 27;	
        pidProfile->pid[PID_ROLL].P = 43;	
        pidProfile->pid[PID_ROLL].I = 45;	
        pidProfile->pid[PID_ROLL].D = 25;
    #elif defined(HELIO_BANGOOD_SPECIAL)
        //optimizng for IDKWTF set the normal defaults
    #else
        //optimizng for true-x and most standard tunes.
        pidProfile->pid[PID_PITCH].P = 45;	
        pidProfile->pid[PID_PITCH].I = 50;	
        pidProfile->pid[PID_PITCH].D = 30;	
        pidProfile->pid[PID_ROLL].P = 45;	
        pidProfile->pid[PID_ROLL].I = 50;	
        pidProfile->pid[PID_ROLL].D = 30;
    #endif

        /* Setpoints */
        pidProfile->dtermSetpointWeight = 100;
        pidProfile->setpointRelaxRatio = 100; // default to snappy for racers
        pidProfile->itermAcceleratorGain = 5000;
        // should't need to set these since they don't get init in gyro.c with USE_GYRO_IMUF
        // pidProfile->yaw_lpf_hz = 0;
        // pidProfile->dterm_lpf_hz = 0;    
        // pidProfile->dterm_notch_hz = 0;
        // pidProfile->dterm_notch_cutoff = 0;
        pidProfile->dterm_filter_type = FILTER_BIQUAD;
        pidProfile->dterm_filter_style = KD_FILTER_NOSP;
        pidProfile->dterm_lpf_hz = 60;
    }
}

