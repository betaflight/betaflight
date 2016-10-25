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

#pragma once

#include <stdlib.h>

#include "config/config_profile.h"

#include "drivers/pwm_rx.h"
#include "drivers/sound_beeper.h"
#include "drivers/sonar_hcsr04.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/navigation.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/motors.h"
#include "io/servos.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/ledstrip.h"
#include "io/vtx.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"


// System-wide
typedef struct master_s {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    uint8_t mixerMode;
    uint32_t enabledFeatures;

    // motor/esc/servo related stuff
    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
    motorConfig_t motorConfig;
    flight3DConfig_t flight3DConfig;

#ifdef USE_SERVOS
    servoConfig_t servoConfig;
    servoMixerConfig_t servoMixerConfig;
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
    // Servo-related stuff
    servoParam_t servoConf[MAX_SUPPORTED_SERVOS]; // servo configuration
    // gimbal-related configuration
    gimbalConfig_t gimbalConfig;
#endif

    // global sensor-related stuff
    sensorAlignmentConfig_t sensorAlignmentConfig;
    boardAlignment_t boardAlignment;

    int8_t yaw_control_direction;           // change control direction of yaw (inverted, normal)
    uint8_t acc_hardware;                   // Which acc hardware to use on boards with more than one device
    uint8_t acc_for_fast_looptime;          // shorten acc processing time by using 1 out of 9 samples. For combination with fast looptimes.
    uint16_t gyro_lpf;                      // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.
    uint8_t gyro_sync_denom;                // Gyro sample divider
    uint8_t gyro_soft_type;                 // Gyro Filter Type
    uint8_t gyro_soft_lpf_hz;               // Biquad gyro lpf hz
    uint16_t gyro_soft_notch_hz_1;          // Biquad gyro notch hz
    uint16_t gyro_soft_notch_cutoff_1;      // Biquad gyro notch low cutoff
    uint16_t gyro_soft_notch_hz_2;          // Biquad gyro notch hz
    uint16_t gyro_soft_notch_cutoff_2;      // Biquad gyro notch low cutoff
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)

    uint8_t pid_process_denom;              // Processing denominator for PID controller vs gyro sampling rate

    uint8_t debug_mode;                     // Processing denominator for PID controller vs gyro sampling rate

    gyroConfig_t gyroConfig;

    uint8_t mag_hardware;                   // Which mag hardware to use on boards with more than one device
    uint8_t baro_hardware;                  // Barometer hardware to use
    int16_t mag_declination;                // Get your magnetic decliniation from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.

    rollAndPitchTrims_t accelerometerTrims; // accelerometer trim

    uint16_t acc_lpf_hz;                       // cutoff frequency for the low pass filter used on the acc z-axis for althold in Hz
    accDeadband_t accDeadband;
    barometerConfig_t barometerConfig;
    uint8_t acc_unarmedcal;                 // turn automatic acc compensation on/off

    uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
    uint8_t throttle_correction_value;      // the correction that will be applied at throttle_correction_angle.
    batteryConfig_t batteryConfig;

    // Radio/ESC-related configuration
    rcControlsConfig_t rcControlsConfig;

#ifdef GPS
    gpsProfile_t gpsProfile;
    gpsConfig_t gpsConfig;
#endif

    uint16_t max_angle_inclination;         // max inclination allowed in angle (level) mode. default 500 (50 degrees).
    flightDynamicsTrims_t accZero;
    flightDynamicsTrims_t magZero;

    rxConfig_t rxConfig;
    inputFilteringMode_e inputFilteringMode;  // Use hardware input filtering, e.g. for OrangeRX PPM/PWM receivers.


    uint8_t gyro_cal_on_first_arm;          // allow disarm/arm on throttle down + roll left/right
    uint8_t disarm_kill_switch;             // allow disarm via AUX switch regardless of throttle value
    uint8_t auto_disarm_delay;              // allow automatically disarming multicopters after auto_disarm_delay seconds of zero throttle. Disabled when 0
    uint8_t small_angle;

    // mixer-related configuration
    mixerConfig_t mixerConfig;
    airplaneConfig_t airplaneConfig;

    failsafeConfig_t failsafeConfig;
    serialConfig_t serialConfig;
    telemetryConfig_t telemetryConfig;

#ifndef SKIP_RX_PWM_PPM
    ppmConfig_t ppmConfig;
    pwmConfig_t pwmConfig;
#endif
    
#ifdef BEEPER
    beeperConfig_t beeperConfig;
#endif

#ifdef SONAR
    sonarConfig_t sonarConfig;
#endif

#ifdef LED_STRIP
    ledConfig_t ledConfigs[LED_MAX_STRIP_LENGTH];
    hsvColor_t colors[LED_CONFIGURABLE_COLOR_COUNT];
    modeColorIndexes_t modeColors[LED_MODE_COUNT];
    specialColorIndexes_t specialColors;
    uint8_t ledstrip_visual_beeper; // suppress LEDLOW mode if beeper is on
#endif

#ifdef TRANSPONDER
    uint8_t transponderData[6];
#endif

#ifdef USE_RTC6705
    uint8_t vtx_channel;
    uint8_t vtx_power;
#endif

#ifdef OSD
    osd_profile_t osdProfile;
#endif

    profile_t profile[MAX_PROFILE_COUNT];
    uint8_t current_profile_index;

    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
    adjustmentRange_t adjustmentRanges[MAX_ADJUSTMENT_RANGE_COUNT];

#ifdef VTX
    uint8_t vtx_band; //1=A, 2=B, 3=E, 4=F(Airwaves/Fatshark), 5=Raceband
    uint8_t vtx_channel; //1-8
    uint8_t vtx_mode; //0=ch+band 1=mhz
    uint16_t vtx_mhz; //5740

    vtxChannelActivationCondition_t vtxChannelActivationConditions[MAX_CHANNEL_ACTIVATION_CONDITION_COUNT];
#endif

#ifdef BLACKBOX
    uint8_t blackbox_rate_num;
    uint8_t blackbox_rate_denom;
    uint8_t blackbox_device;
    uint8_t blackbox_on_motor_test;
#endif

    uint32_t beeper_off_flags;
    uint32_t preferred_beeper_off_flags;

    char name[MAX_NAME_LENGTH + 1];

    uint8_t magic_ef;                       // magic number, should be 0xEF
    uint8_t chk;                            // XOR checksum 
    /* 
        do not add properties after the CHK
        as it is assumed to exist at length-1 
    */
} master_t;

extern master_t masterConfig;
extern profile_t *currentProfile;
extern controlRateConfig_t *currentControlRateProfile;

void createDefaultConfig(master_t *config);
