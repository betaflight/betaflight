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

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "config/config_profile.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/pwm_rx.h"
#include "drivers/serial.h"

#include "fc/config.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"
#include "flight/pid.h"

#include "io/gps.h"
#include "io/osd.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "telemetry/telemetry.h"

#define servoMixerConfig(x) (&masterConfig.servoMixerConfig)
#define servoMixerConfigMutable(x) (&masterConfig.servoMixerConfig)
#define imuConfig(x) (&masterConfig.imuConfig)
#define imuConfigMutable(x) (&masterConfig.imuConfig)
#define rcControlsConfig(x) (&masterConfig.rcControlsConfig)
#define rcControlsConfigMutable(x) (&masterConfig.rcControlsConfig)
#define gpsConfig(x) (&masterConfig.gpsConfig)
#define gpsConfigMutable(x) (&masterConfig.gpsConfig)
#define navConfig(x) (&masterConfig.navConfig)
#define navConfigMutable(x) (&masterConfig.navConfig)
#define armingConfig(x) (&masterConfig.armingConfig)
#define armingConfigMutable(x) (&masterConfig.armingConfig)
#define serialConfig(x) (&masterConfig.serialConfig)
#define serialConfigMutable(x) (&masterConfig.serialConfig)
#define telemetryConfig(x) (&masterConfig.telemetryConfig)
#define telemetryConfigMutable(x) (&masterConfig.telemetryConfig)
#define osdProfile(x) (&masterConfig.osdProfile)
#define osdProfileMutable(x) (&masterConfig.osdProfile)
#define ledStripConfig(x) (&masterConfig.ledStripConfig)
#define ledStripConfigMutable(x) (&masterConfig.ledStripConfig)
#define pwmRxConfig(x) (&masterConfig.pwmRxConfig)
#define pwmRxConfigMutable(x) (&masterConfig.pwmRxConfig)
#define customServoMixer(i) (&masterConfig.customServoMixer[i])
#define customServoMixerMutable(i) (&masterConfig.customServoMixer[i])


// System-wide
typedef struct master_s {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    uint32_t enabledFeatures;
    uint8_t persistentFlags;
    uint8_t i2c_overclock;                  // Overclock i2c Bus for faster IMU readings

#ifdef ASYNC_GYRO_PROCESSING
    uint16_t accTaskFrequency;
    uint16_t attitudeTaskFrequency;
    uint8_t asyncMode;
#endif

#ifdef USE_SERVOS
    servoMixerConfig_t servoMixerConfig;
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
#endif

    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
    modeActivationOperator_e modeActivationOperator;

    adjustmentRange_t adjustmentRanges[MAX_ADJUSTMENT_RANGE_COUNT];

    // Radio/ESC-related configuration

    rcControlsConfig_t rcControlsConfig;

    uint8_t throttle_tilt_compensation_strength;      // the correction that will be applied at throttle_correction_angle.

#ifdef USE_SERVOS
    // Servo-related stuff
    servoParam_t servoConf[MAX_SUPPORTED_SERVOS]; // servo configuration

    uint16_t flaperon_throw_offset;
    uint8_t flaperon_throw_inverted;

#endif

    imuConfig_t imuConfig;

#ifdef GPS
    gpsConfig_t gpsConfig;
#endif

#ifdef NAV
    navConfig_t navConfig;
#endif

    pwmRxConfig_t pwmRxConfig;

    armingConfig_t armingConfig;

    serialConfig_t serialConfig;
#ifdef TELEMETRY
    telemetryConfig_t telemetryConfig;
#endif


#ifdef LED_STRIP
    ledStripConfig_t ledStripConfig;
#endif

#ifdef OSD
    osd_profile_t osdProfile;
#endif

    profile_t profile[MAX_PROFILE_COUNT];
    uint8_t current_profile_index;
    controlRateConfig_t controlRateProfiles[MAX_CONTROL_RATE_PROFILE_COUNT];

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

void createDefaultConfig(master_t *config);
