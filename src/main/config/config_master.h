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
#include "config/feature.h"

#include "blackbox/blackbox.h"

#include "cms/cms.h"

#include "drivers/adc.h"
#include "drivers/rx_pwm.h"
#include "drivers/sound_beeper.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/vcd.h"
#include "drivers/light_led.h"
#include "drivers/flash.h"
#include "drivers/display.h"
#include "drivers/serial.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/fc_core.h"

#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/servos.h"
#include "io/transponder_ir.h"
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

#ifndef USE_PARAMETER_GROUPS
#define featureConfig(x) (&masterConfig.featureConfig)
#define systemConfig(x) (&masterConfig.systemConfig)
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
#define navigationConfig(x) (&masterConfig.navigationConfig)
#define gpsConfig(x) (&masterConfig.gpsConfig)
#define rxConfig(x) (&masterConfig.rxConfig)
#define armingConfig(x) (&masterConfig.armingConfig)
#define mixerConfig(x) (&masterConfig.mixerConfig)
#define airplaneConfig(x) (&masterConfig.airplaneConfig)
#define failsafeConfig(x) (&masterConfig.failsafeConfig)
#define serialPinConfig(x) (&masterConfig.serialPinConfig)
#define serialConfig(x) (&masterConfig.serialConfig)
#define telemetryConfig(x) (&masterConfig.telemetryConfig)
#define ibusTelemetryConfig(x) (&masterConfig.telemetryConfig)
#define ppmConfig(x) (&masterConfig.ppmConfig)
#define pwmConfig(x) (&masterConfig.pwmConfig)
#define adcConfig(x) (&masterConfig.adcConfig)
#define beeperDevConfig(x) (&masterConfig.beeperDevConfig)
#define sonarConfig(x) (&masterConfig.sonarConfig)
#define ledStripConfig(x) (&masterConfig.ledStripConfig)
#define statusLedConfig(x) (&masterConfig.statusLedConfig)
#define osdConfig(x) (&masterConfig.osdConfig)
#define vcdProfile(x) (&masterConfig.vcdProfile)
#define sdcardConfig(x) (&masterConfig.sdcardConfig)
#define blackboxConfig(x) (&masterConfig.blackboxConfig)
#define flashConfig(x) (&masterConfig.flashConfig)
#define pidConfig(x) (&masterConfig.pidConfig)
#define adjustmentProfile(x) (&masterConfig.adjustmentProfile)
#define modeActivationProfile(x) (&masterConfig.modeActivationProfile)
#define servoProfile(x) (&masterConfig.servoProfile)
#define customMotorMixer(i) (&masterConfig.customMotorMixer[i])
#define customServoMixers(i) (&masterConfig.customServoMixer[i])
#define displayPortProfileMsp(x) (&masterConfig.displayPortProfileMsp)
#define displayPortProfileMax7456(x) (&masterConfig.displayPortProfileMax7456)
#define displayPortProfileOled(x) (&masterConfig.displayPortProfileOled)
#define vtxConfig(x) (&masterConfig.vtxConfig)
#define beeperConfig(x) (&masterConfig.beeperConfig)
#define transponderConfig(x) (&masterConfig.transponderConfig)

#define featureConfigMutable(x) (&masterConfig.featureConfig)
#define systemConfigMutable(x) (&masterConfig.systemConfig)
#define motorConfigMutable(x) (&masterConfig.motorConfig)
#define flight3DConfigMutable(x) (&masterConfig.flight3DConfig)
#define servoConfigMutable(x) (&masterConfig.servoConfig)
#define servoMixerConfigMutable(x) (&masterConfig.servoMixerConfig)
#define gimbalConfigMutable(x) (&masterConfig.gimbalConfig)
#define boardAlignmentMutable(x) (&masterConfig.boardAlignment)
#define imuConfigMutable(x) (&masterConfig.imuConfig)
#define gyroConfigMutable(x) (&masterConfig.gyroConfig)
#define compassConfigMutable(x) (&masterConfig.compassConfig)
#define accelerometerConfigMutable(x) (&masterConfig.accelerometerConfig)
#define barometerConfigMutable(x) (&masterConfig.barometerConfig)
#define throttleCorrectionConfigMutable(x) (&masterConfig.throttleCorrectionConfig)
#define batteryConfigMutable(x) (&masterConfig.batteryConfig)
#define rcControlsConfigMutable(x) (&masterConfig.rcControlsConfig)
#define navigationConfigMutable(x) (&masterConfig.navigationConfig)
#define gpsConfigMutable(x) (&masterConfig.gpsConfig)
#define rxConfigMutable(x) (&masterConfig.rxConfig)
#define armingConfigMutable(x) (&masterConfig.armingConfig)
#define mixerConfigMutable(x) (&masterConfig.mixerConfig)
#define airplaneConfigMutable(x) (&masterConfig.airplaneConfig)
#define failsafeConfigMutable(x) (&masterConfig.failsafeConfig)
#define serialConfigMutable(x) (&masterConfig.serialConfig)
#define telemetryConfigMutable(x) (&masterConfig.telemetryConfig)
#define ibusTelemetryConfigMutable(x) (&masterConfig.telemetryConfig)
#define ppmConfigMutable(x) (&masterConfig.ppmConfig)
#define pwmConfigMutable(x) (&masterConfig.pwmConfig)
#define adcConfigMutable(x) (&masterConfig.adcConfig)
#define beeperDevConfigMutable(x) (&masterConfig.beeperDevConfig)
#define sonarConfigMutable(x) (&masterConfig.sonarConfig)
#define ledStripConfigMutable(x) (&masterConfig.ledStripConfig)
#define statusLedConfigMutable(x) (&masterConfig.statusLedConfig)
#define osdConfigMutable(x) (&masterConfig.osdConfig)
#define vcdProfileMutable(x) (&masterConfig.vcdProfile)
#define sdcardConfigMutable(x) (&masterConfig.sdcardConfig)
#define blackboxConfigMutable(x) (&masterConfig.blackboxConfig)
#define flashConfigMutable(x) (&masterConfig.flashConfig)
#define pidConfigMutable(x) (&masterConfig.pidConfig)
#define adjustmentProfileMutable(x) (&masterConfig.adjustmentProfile)
#define modeActivationProfileMutable(x) (&masterConfig.modeActivationProfile)
#define servoProfileMutable(x) (&masterConfig.servoProfile)
#define customMotorMixerMutable(i) (&masterConfig.customMotorMixer[i])
#define customServoMixersMutable(i) (&masterConfig.customServoMixer[i])
#define displayPortProfileMspMutable(x) (&masterConfig.displayPortProfileMsp)
#define displayPortProfileMax7456Mutable(x) (&masterConfig.displayPortProfileMax7456)
#define displayPortProfileOledMutable(x) (&masterConfig.displayPortProfileOled)
#define vtxConfigMutable(x) (&masterConfig.vtxConfig)
#define beeperConfigMutable(x) (&masterConfig.beeperConfig)
#define transponderConfigMutable(x) (&masterConfig.transponderConfig)

#define servoParams(i) (&servoProfile()->servoConf[i])
#define adjustmentRanges(i) (&adjustmentProfile()->adjustmentRanges[i])
#define rxFailsafeChannelConfigs(i) (&masterConfig.rxConfig.failsafe_channel_configurations[i])
#define modeActivationConditions(i) (&masterConfig.modeActivationProfile.modeActivationConditions[i])
#define controlRateProfiles(i) (&masterConfig.controlRateProfile[i])
#define pidProfiles(i) (&masterConfig.profile[i].pidProfile)

#define servoParamsMutable(i) (&servoProfile()->servoConf[i])
#define adjustmentRangesMutable(i) (&masterConfig.adjustmentProfile.adjustmentRanges[i])
#define rxFailsafeChannelConfigsMutable(i) (&masterConfig.rxConfig.>failsafe_channel_configurations[i])
#define modeActivationConditionsMutable(i) (&masterConfig.modeActivationProfile.modeActivationConditions[i])
#define controlRateProfilesMutable(i) (&masterConfig.controlRateProfile[i])
#define pidProfilesMutable(i) (&masterConfig.profile[i].pidProfile)

// System-wide
typedef struct master_s {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE

    featureConfig_t featureConfig;

    systemConfig_t systemConfig;

    // motor/esc/servo related stuff
    motorMixer_t customMotorMixer[MAX_SUPPORTED_MOTORS];
    motorConfig_t motorConfig;
    flight3DConfig_t flight3DConfig;

#ifdef USE_SERVOS
    servoConfig_t servoConfig;
    servoMixer_t customServoMixer[MAX_SERVO_RULES];
    // Servo-related stuff
    servoProfile_t servoProfile;
    // gimbal-related configuration
    gimbalConfig_t gimbalConfig;
#endif

    boardAlignment_t boardAlignment;

    imuConfig_t imuConfig;

    pidConfig_t pidConfig;

    gyroConfig_t gyroConfig;
    compassConfig_t compassConfig;

    accelerometerConfig_t accelerometerConfig;

    barometerConfig_t barometerConfig;

    throttleCorrectionConfig_t throttleCorrectionConfig;

    batteryConfig_t batteryConfig;

    // Radio/ESC-related configuration
    rcControlsConfig_t rcControlsConfig;

#ifdef GPS
    navigationConfig_t navigationConfig;
    gpsConfig_t gpsConfig;
#endif

    rxConfig_t rxConfig;

    armingConfig_t armingConfig;

    // mixer-related configuration
    mixerConfig_t mixerConfig;
    airplaneConfig_t airplaneConfig;

    failsafeConfig_t failsafeConfig;
    serialPinConfig_t serialPinConfig;
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
    beeperDevConfig_t beeperDevConfig;
#endif

#ifdef SONAR
    sonarConfig_t sonarConfig;
#endif

#ifdef LED_STRIP
    ledStripConfig_t ledStripConfig;
#endif

#ifdef TRANSPONDER
    transponderConfig_t transponderConfig;
#endif

#ifdef OSD
    osdConfig_t osdConfig;
#endif

#ifdef USE_MAX7456
    vcdProfile_t vcdProfile;
#endif

#ifdef USE_MSP_DISPLAYPORT
    displayPortProfile_t displayPortProfileMsp;
#endif
#ifdef USE_MAX7456
    displayPortProfile_t displayPortProfileMax7456;
# endif

#ifdef USE_SDCARD
    sdcardConfig_t sdcardConfig;
#endif

    profile_t profile[MAX_PROFILE_COUNT];
    controlRateConfig_t controlRateProfile[CONTROL_RATE_PROFILE_COUNT];

    modeActivationProfile_t modeActivationProfile;
    adjustmentProfile_t adjustmentProfile;
#if defined(USE_RTC6705) || defined(VTX)
    vtxConfig_t vtxConfig;
#endif
#ifdef BLACKBOX
    blackboxConfig_t blackboxConfig;
#endif

#ifdef USE_FLASHFS
    flashConfig_t flashConfig;
#endif

    beeperConfig_t beeperConfig;

    uint8_t chk;                            // XOR checksum
    /*
        do not add properties after the MAGIC_EF and CHK
        as it is assumed to exist at length-2 and length-1
    */
} master_t;

extern master_t masterConfig;

void createDefaultConfig(master_t *config);
#endif // USE_PARAMETER_GROUPS
