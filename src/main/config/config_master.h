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
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"

#include "telemetry/telemetry.h"

#include "config/config.h"
#include "config/config_profile.h"

#define motorConfig(x) (&masterConfig.motorConfig)
#define flight3DConfig(x) (&masterConfig.flight3DConfig)
#define servoConfig(x) (&masterConfig.servoConfig)
#define servoMixerConfig(x) (&masterConfig.servoMixerConfig)
#define boardAlignment(x) (&masterConfig.boardAlignment)
#define imuConfig(x) (&masterConfig.imuConfig)
#define gyroConfig(x) (&masterConfig.gyroConfig)
#define accelerometerConfig(x) (&masterConfig.accelerometerConfig)
#define barometerConfig(x) (&masterConfig.barometerConfig)
#define compassConfig(x) (&masterConfig.compassConfig)
#define pitotmeterConfig(x) (&masterConfig.pitotmeterConfig)
#define batteryConfig(x) (&masterConfig.batteryConfig)
#define gpsConfig(x) (&masterConfig.gpsConfig)
#define navConfig(x) (&masterConfig.navConfig)
#define rxConfig(x) (&masterConfig.rxConfig)
#define armingConfig(x) (&masterConfig.armingConfig)
#define mixerConfig(x) (&masterConfig.mixerConfig)
#define failsafeConfig(x) (&masterConfig.failsafeConfig)
#define serialConfig(x) (&masterConfig.serialConfig)
#define telemetryConfig(x) (&masterConfig.telemetryConfig)
#define osdProfile(x) (&masterConfig.osdProfile)
#define blackboxConfig(x) (&masterConfig.blackboxConfig)
#define ledStripConfig(x) (&masterConfig.ledStripConfig)
#define pwmRxConfig(x) (&masterConfig.pwmRxConfig)


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

    // motor/esc/servo related stuff
    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
    motorConfig_t motorConfig;
    flight3DConfig_t flight3DConfig;

#ifdef USE_SERVOS
    servoConfig_t servoConfig;
    servoMixerConfig_t servoMixerConfig;
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
#endif

    boardAlignment_t boardAlignment;

    imuConfig_t imuConfig;

    gyroConfig_t gyroConfig;

    accelerometerConfig_t accelerometerConfig;

    barometerConfig_t barometerConfig;

    compassConfig_t compassConfig;

    pitotmeterConfig_t pitotmeterConfig;

    batteryConfig_t batteryConfig;

#ifdef GPS
    gpsConfig_t gpsConfig;
#endif

#ifdef NAV
    navConfig_t navConfig;
#endif

    rxConfig_t rxConfig;
    pwmRxConfig_t pwmRxConfig;

    armingConfig_t armingConfig;

    // mixer-related configuration
    mixerConfig_t mixerConfig;

    failsafeConfig_t failsafeConfig;
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
