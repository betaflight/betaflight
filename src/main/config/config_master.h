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

#include "blackbox/blackbox.h"

#include "cms/cms.h"

#include "drivers/adc.h"
#include "drivers/pwm_rx.h"
#include "drivers/sound_beeper.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/vcd.h"
#include "drivers/light_led.h"
#include "drivers/flash.h"

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
#include "sensors/compass.h"

#define motorConfig(x) (&masterConfig.motorConfig)
#define flight3DConfig(x) (&masterConfig.flight3DConfig)
#define servoConfig(x) (&masterConfig.servoConfig)
#define servoMixerConfig(x) (&masterConfig.servoMixerConfig)
#define gimbalConfig(x) (&masterConfig.gimbalConfig)
#define boardAlignment(x) (&masterConfig.boardAlignment)
#define imuConfig(x) (&masterConfig.imuConfig)
#define gyroConfig(x) (&masterConfig.gyroConfig)
#define compassConfig(x) (&masterConfig.compassConfig)
#define accelerometerConfig(x) (&masterConfig.accelerometerConfig)
#define barometerConfig(x) (&masterConfig.barometerConfig)
#define throttleCorrectionConfig(x) (&masterConfig.throttleCorrectionConfig)
#define batteryConfig(x) (&masterConfig.batteryConfig)
#define rcControlsConfig(x) (&masterConfig.rcControlsConfig)
#define gpsProfile(x) (&masterConfig.gpsProfile)
#define gpsConfig(x) (&masterConfig.gpsConfig)
#define rxConfig(x) (&masterConfig.rxConfig)
#define armingConfig(x) (&masterConfig.armingConfig)
#define mixerConfig(x) (&masterConfig.mixerConfig)
#define airplaneConfig(x) (&masterConfig.airplaneConfig)
#define failsafeConfig(x) (&masterConfig.failsafeConfig)
#define serialConfig(x) (&masterConfig.serialConfig)
#define telemetryConfig(x) (&masterConfig.telemetryConfig)
#define ppmConfig(x) (&masterConfig.ppmConfig)
#define pwmConfig(x) (&masterConfig.pwmConfig)
#define adcConfig(x) (&masterConfig.adcConfig)
#define beeperConfig(x) (&masterConfig.beeperConfig)
#define sonarConfig(x) (&masterConfig.sonarConfig)
#define ledStripConfig(x) (&masterConfig.ledStripConfig)
#define statusLedConfig(x) (&masterConfig.statusLedConfig)
#define osdProfile(x) (&masterConfig.osdProfile)
#define vcdProfile(x) (&masterConfig.vcdProfile)
#define sdcardConfig(x) (&masterConfig.sdcardConfig)
#define blackboxConfig(x) (&masterConfig.blackboxConfig)
#define flashConfig(x) (&masterConfig.flashConfig)
#define pidConfig(x) (&masterConfig.pidConfig)


// System-wide
typedef struct master_s {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

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

    boardAlignment_t boardAlignment;

    imuConfig_t imuConfig;

    rollAndPitchTrims_t accelerometerTrims; // accelerometer trim

    pidConfig_t pidConfig;

    uint8_t debug_mode;                     // Processing denominator for PID controller vs gyro sampling rate

    gyroConfig_t gyroConfig;
    compassConfig_t compassConfig;

    accelerometerConfig_t accelerometerConfig;

    barometerConfig_t barometerConfig;

    throttleCorrectionConfig_t throttleCorrectionConfig;

    batteryConfig_t batteryConfig;

    // Radio/ESC-related configuration
    rcControlsConfig_t rcControlsConfig;

#ifdef GPS
    gpsProfile_t gpsProfile;
    gpsConfig_t gpsConfig;
#endif

    rxConfig_t rxConfig;
    inputFilteringMode_e inputFilteringMode;  // Use hardware input filtering, e.g. for OrangeRX PPM/PWM receivers.

    armingConfig_t armingConfig;

    // mixer-related configuration
    mixerConfig_t mixerConfig;
    airplaneConfig_t airplaneConfig;

    failsafeConfig_t failsafeConfig;
    serialConfig_t serialConfig;
    telemetryConfig_t telemetryConfig;

    statusLedConfig_t statusLedConfig;

#ifdef USE_PPM
    ppmConfig_t ppmConfig;
#endif

#ifdef USE_PWM
    pwmConfig_t pwmConfig;
#endif

#ifdef USE_ADC
    adcConfig_t adcConfig;
#endif

#ifdef BEEPER
    beeperConfig_t beeperConfig;
#endif

#ifdef SONAR
    sonarConfig_t sonarConfig;
#endif

#ifdef LED_STRIP
    ledStripConfig_t ledStripConfig;
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

#ifdef USE_MAX7456
    vcdProfile_t vcdProfile;
#endif

#ifdef USE_SDCARD
    sdcardConfig_t sdcardConfig;
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
    blackboxConfig_t blackboxConfig;
#endif

#ifdef USE_FLASHFS
    flashConfig_t flashConfig;
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

void createDefaultConfig(master_t *config);
