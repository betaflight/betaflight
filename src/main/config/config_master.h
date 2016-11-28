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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "blackbox/blackbox.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"

#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/pid.h"

#include "io/gimbal.h"
#include "io/gps.h"
#include "io/osd.h"
#include "io/ledstrip.h"
#include "io/motors.h"
#include "io/serial.h"
#include "io/servos.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"

#include "telemetry/telemetry.h"

#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

// System-wide
typedef struct master_s {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    uint32_t enabledFeatures;
    uint8_t persistentFlags;
    uint16_t looptime;                      // imu loop time in us
    uint8_t i2c_overclock;                  // Overclock i2c Bus for faster IMU readings
    uint8_t gyroSync;                       // Enable interrupt based loop
    uint8_t gyroSyncDenominator;            // Gyro sync Denominator

#ifdef ASYNC_GYRO_PROCESSING
    uint16_t accTaskFrequency;
    uint16_t attitudeTaskFrequency;
    uint8_t asyncMode;
#endif

    // motor/esc/servo related stuff
    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
    motorConfig_t motorConfig;
    flight3DConfig_t flight3DConfig;

#ifdef USE_SERVOS
    servoConfig_t servoConfig;
    servoMixerConfig_t servoMixerConfig;
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
#endif

    // global sensor-related stuff
    sensorSelectionConfig_t sensorSelectionConfig;
    sensorAlignmentConfig_t sensorAlignmentConfig;
    sensorTrims_t sensorTrims;
    boardAlignment_t boardAlignment;

    uint16_t dcm_kp_acc;                    // DCM filter proportional gain ( x 10000) for accelerometer
    uint16_t dcm_ki_acc;                    // DCM filter integral gain ( x 10000) for accelerometer
    uint16_t dcm_kp_mag;                    // DCM filter proportional gain ( x 10000) for magnetometer and GPS heading
    uint16_t dcm_ki_mag;                    // DCM filter integral gain ( x 10000) for magnetometer and GPS heading

    uint8_t gyro_lpf;                       // gyro LPF setting - values are driver specific, in case of invalid number, a reasonable default ~30-40HZ is chosen.

    gyroConfig_t gyroConfig;

    barometerConfig_t barometerConfig;

    pitotmeterConfig_t pitotmeterConfig;

    batteryConfig_t batteryConfig;

    rxConfig_t rxConfig;
    inputFilteringMode_e inputFilteringMode;  // Use hardware input filtering, e.g. for OrangeRX PPM/PWM receivers.

#ifdef GPS
    gpsConfig_t gpsConfig;
#endif

#ifdef NAV
    navConfig_t navConfig;
#endif

    armingConfig_t armingConfig;

    // mixer-related configuration
    mixerConfig_t mixerConfig;

    failsafeConfig_t failsafeConfig;
    serialConfig_t serialConfig;
#ifdef TELEMETRY
    telemetryConfig_t telemetryConfig;
#endif


#ifdef LED_STRIP
    ledConfig_t ledConfigs[LED_MAX_STRIP_LENGTH];
    hsvColor_t colors[LED_CONFIGURABLE_COLOR_COUNT];
    modeColorIndexes_t modeColors[LED_MODE_COUNT];
    specialColorIndexes_t specialColors;
    uint8_t ledstrip_visual_beeper; // suppress LEDLOW mode if beeper is on
#endif

#ifdef OSD
    osd_profile_t osdProfile;
#endif

    profile_t profile[MAX_PROFILE_COUNT];
    uint8_t current_profile_index;
    controlRateConfig_t controlRateProfiles[MAX_CONTROL_RATE_PROFILE_COUNT];

#ifdef BLACKBOX
    blackboxConfig_t blackboxConfig;
#endif

    uint32_t beeper_off_flags;
    uint32_t preferred_beeper_off_flags;

    char name[MAX_NAME_LENGTH + 1];

    uint8_t magic_ef;                       // magic number, should be 0xEF
    uint8_t chk;                            // XOR checksum
    /*
        do not add properties after the MAGIC_EF and CHK
        as it is assumed to exist at length-2 and length-1
    */
} master_t;

extern master_t masterConfig;
extern profile_t *currentProfile;
extern controlRateConfig_t *currentControlRateProfile;
