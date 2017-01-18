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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "blackbox/blackbox_io.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/system.h"
#include "drivers/rx_spi.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"

#include "io/beeper.h"
#include "io/serial.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/gps.h"
#include "io/osd.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/navigation_rewrite.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"

#include "build/debug.h"

#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif
#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 400

master_t masterConfig;                 // master config struct with data independent from profiles
const profile_t *currentProfile;

PG_REGISTER(profileSelection_t, profileSelection, PG_PROFILE_SELECTION, 0);

#ifdef NAV
void validateNavConfig(void)
{
    // Make sure minAlt is not more than maxAlt, maxAlt cannot be set lower than 500.
    navConfigMutable()->general.land_slowdown_minalt = MIN(navConfig()->general.land_slowdown_minalt, navConfig()->general.land_slowdown_maxalt - 100);
}
#endif


#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

#ifdef ASYNC_GYRO_PROCESSING
uint32_t getPidUpdateRate(void) {
    if (masterConfig.asyncMode == ASYNC_MODE_NONE) {
        return getGyroUpdateRate();
    } else {
        return gyroConfig()->looptime;
    }
}

timeDelta_t getGyroUpdateRate(void) {
    return gyro.targetLooptime;
}

uint16_t getAccUpdateRate(void) {
    if (masterConfig.asyncMode == ASYNC_MODE_ALL) {
        return 1000000 / masterConfig.accTaskFrequency;
    } else {
        return getPidUpdateRate();
    }
}

uint16_t getAttitudeUpdateRate(void) {
    if (masterConfig.asyncMode == ASYNC_MODE_ALL) {
        return 1000000 / masterConfig.attitudeTaskFrequency;
    } else {
        return getPidUpdateRate();
    }
}

uint8_t getAsyncMode(void) {
    return masterConfig.asyncMode;
}
#endif

uint16_t getCurrentMinthrottle(void)
{
    return motorConfig()->minthrottle;
}

// Default settings
void createDefaultConfig(master_t *config)
{
    // Clear all configuration
    memset(config, 0, sizeof(master_t));

    config->version = EEPROM_CONF_VERSION;

#ifdef OSD
    osdResetConfig(&config->osdProfile);
#endif

    config->debug_mode = DEBUG_NONE;

    config->pwmRxConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;

    config->i2c_overclock = 0;

#ifdef ASYNC_GYRO_PROCESSING
    config->accTaskFrequency = ACC_TASK_FREQUENCY_DEFAULT;
    config->attitudeTaskFrequency = ATTITUDE_TASK_FREQUENCY_DEFAULT;
    config->asyncMode = ASYNC_MODE_NONE;
#endif

    // for (int i = 0; i < CHECKBOXITEMS; i++)
    //     cfg.activate[i] = 0;

    // Radio
#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234");
#else
    parseRcChannels("AETR1234");
#endif

    config->throttle_tilt_compensation_strength = 0;      // 0-100, 0 - disabled

#ifdef BLACKBOX
#ifdef ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
    featureSet(FEATURE_BLACKBOX);
#endif
#endif

    // alternative defaults settings for ALIENFLIGHTF1 and ALIENFLIGHTF3 targets
#ifdef ALIENFLIGHTF1
#ifdef ALIENFLIGHTF3
    config->serialConfig.portConfigs[2].functionMask = FUNCTION_RX_SERIAL;
    config->batteryConfig.vbatscale = 20;
#else
    config->serialConfig.portConfigs[1].functionMask = FUNCTION_RX_SERIAL;
#endif
    config->rxConfig.spektrum_sat_bind = 5;
    config->motorConfig.minthrottle = 1000;
    config->motorConfig.maxthrottle = 2000;
    config->motorConfig.motorPwmRate = 32000;
    config->looptime = 2000;
    pidProfileMutable()->P8[ROLL] = 36;
    pidProfileMutable()->P8[PITCH] = 36;
    config->failsafeConfig.failsafe_delay = 2;
    config->failsafeConfig.failsafe_off_delay = 0;
    config->controlRateProfiles[0].rates[FD_PITCH] = CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_DEFAULT;
    config->controlRateProfiles[0].rates[FD_ROLL] = CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_DEFAULT;
    config->controlRateProfiles[0].rates[FD_YAW] = CONTROL_RATE_CONFIG_YAW_RATE_DEFAULT;
    parseRcChannels("TAER1234");

    //  { 1.0f, -0.414178f,  1.0f, -1.0f },          // REAR_R
    config->customMotorMixer[0].throttle = 1.0f;
    config->customMotorMixer[0].roll = -0.414178f;
    config->customMotorMixer[0].pitch = 1.0f;
    config->customMotorMixer[0].yaw = -1.0f;

    //  { 1.0f, -0.414178f, -1.0f,  1.0f },          // FRONT_R
    config->customMotorMixer[1].throttle = 1.0f;
    config->customMotorMixer[1].roll = -0.414178f;
    config->customMotorMixer[1].pitch = -1.0f;
    config->customMotorMixer[1].yaw = 1.0f;

    //  { 1.0f,  0.414178f,  1.0f,  1.0f },          // REAR_L
    config->customMotorMixer[2].throttle = 1.0f;
    config->customMotorMixer[2].roll = 0.414178f;
    config->customMotorMixer[2].pitch = 1.0f;
    config->customMotorMixer[2].yaw = 1.0f;

    //  { 1.0f,  0.414178f, -1.0f, -1.0f },          // FRONT_L
    config->customMotorMixer[3].throttle = 1.0f;
    config->customMotorMixer[3].roll = 0.414178f;
    config->customMotorMixer[3].pitch = -1.0f;
    config->customMotorMixer[3].yaw = -1.0f;

    //  { 1.0f, -1.0f, -0.414178f, -1.0f },          // MIDFRONT_R
    config->customMotorMixer[4].throttle = 1.0f;
    config->customMotorMixer[4].roll = -1.0f;
    config->customMotorMixer[4].pitch = -0.414178f;
    config->customMotorMixer[4].yaw = -1.0f;

    //  { 1.0f,  1.0f, -0.414178f,  1.0f },          // MIDFRONT_L
    config->customMotorMixer[5].throttle = 1.0f;
    config->customMotorMixer[5].roll = 1.0f;
    config->customMotorMixer[5].pitch = -0.414178f;
    config->customMotorMixer[5].yaw = 1.0f;

    //  { 1.0f, -1.0f,  0.414178f,  1.0f },          // MIDREAR_R
    config->customMotorMixer[6].throttle = 1.0f;
    config->customMotorMixer[6].roll = -1.0f;
    config->customMotorMixer[6].pitch = 0.414178f;
    config->customMotorMixer[6].yaw = 1.0f;

    //  { 1.0f,  1.0f,  0.414178f, -1.0f },          // MIDREAR_L
    config->customMotorMixer[7].throttle = 1.0f;
    config->customMotorMixer[7].roll = 1.0f;
    config->customMotorMixer[7].pitch = 0.414178f;
    config->customMotorMixer[7].yaw = -1.0f;
#endif

#if defined(TARGET_CONFIG)
    targetConfiguration(&masterConfig);
#endif

    // copy first profile into remaining profile
    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        memcpy(&config->profile[i], &config->profile[0], sizeof(profile_t));
    }

    for (int i = 1; i < MAX_PROFILE_COUNT; i++) {
        config->profile[i].defaultRateProfileIndex = i % MAX_CONTROL_RATE_PROFILE_COUNT;
    }
}

void resetConfigs(void)
{
    pgResetAll(MAX_PROFILE_COUNT);
    pgActivateProfile(0);

    createDefaultConfig(&masterConfig);

    setProfile(getCurrentProfileIndex());
    setControlRateProfile(getCurrentProfileIndex());
#ifdef LED_STRIP
    reevaluateLedConfig();
#endif
}

static void activateConfig(void)
{
    activateControlRateConfig();

    resetAdjustmentStates();

    failsafeReset();

    setAccelerationCalibrationValues();
    setAccelerationFilter();

    imuConfigure();

    pidInit();

#ifdef NAV
    navigationUsePIDs();
#endif
}

void validateAndFixConfig(void)
{
    // FIXME: Completely disable sonar
#if defined(SONAR)
    featureClear(FEATURE_SONAR);
#endif

#ifdef USE_GYRO_NOTCH_1
    if (gyroConfig()->gyro_soft_notch_cutoff_1 >= gyroConfig()->gyro_soft_notch_hz_1) {
        gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    }
#endif
#ifdef USE_GYRO_NOTCH_2
    if (gyroConfig()->gyro_soft_notch_cutoff_2 >= gyroConfig()->gyro_soft_notch_hz_2) {
        gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    }
#endif
#ifdef USE_DTERM_NOTCH
    if (pidProfile()->dterm_soft_notch_cutoff >= currentProfile->pidProfile.dterm_soft_notch_hz) {
        pidProfileMutable()->dterm_soft_notch_hz = 0;
    }
#endif
    // Disable unused features
    featureClear(FEATURE_UNUSED_1 | FEATURE_UNUSED_2);

#ifdef DISABLE_RX_PWM_FEATURE
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureClear(FEATURE_RX_PARALLEL_PWM);
    }
#endif

    if (!(featureConfigured(FEATURE_RX_PARALLEL_PWM) || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_SERIAL) || featureConfigured(FEATURE_RX_MSP) || featureConfigured(FEATURE_RX_SPI))) {
        featureSet(DEFAULT_RX_FEATURE);
    }

    if (featureConfigured(FEATURE_RX_PPM)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_SPI);
    }

    if (featureConfigured(FEATURE_RX_MSP)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

    if (featureConfigured(FEATURE_RX_SERIAL)) {
        featureClear(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

    if (featureConfigured(FEATURE_RX_SPI)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_MSP);
    }

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
#if defined(STM32F10X)
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
            featureClear(FEATURE_CURRENT_METER);
        }
#if defined(CC3D)
        // There is a timer clash between PWM RX pins and motor output pins - this forces us to have same timer tick rate for these timers
        // which is only possible when using brushless motors w/o oneshot (timer tick rate is PWM_TIMER_MHZ)
        // On CC3D OneShot is incompatible with PWM RX
        motorConfigMutable()->motorPwmProtocol = PWM_TYPE_STANDARD;
        motorConfigMutable()->motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
#endif
#endif

#if defined(STM32F10X) || defined(CHEBUZZ) || defined(STM32F3DISCOVERY)
        // led strip needs the same ports
        featureClear(FEATURE_LED_STRIP);
#endif

        // software serial needs free PWM ports
        featureClear(FEATURE_SOFTSERIAL);
    }

#ifdef USE_SOFTSPI
    if (featureConfigured(FEATURE_SOFTSPI)) {
        featureClear(FEATURE_RX_PPM | FEATURE_RX_PARALLEL_PWM | FEATURE_SOFTSERIAL | FEATURE_VBAT);
#if defined(STM32F10X)
        featureClear(FEATURE_LED_STRIP);
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
            featureClear(FEATURE_CURRENT_METER);
        }
#endif
    }
#endif

#ifdef ASYNC_GYRO_PROCESSING
    /*
     * When async processing mode is enabled, gyroSync has to be forced to "ON"
     */
    if (getAsyncMode() != ASYNC_MODE_NONE) {
        gyroConfigMutable()->gyroSync = 1;
    }
#endif

#ifdef STM32F10X
    // avoid overloading the CPU on F1 targets when using gyro sync and GPS.

    if (featureConfigured(FEATURE_GPS)) {
        // avoid overloading the CPU when looptime < 2000 and GPS

        uint8_t denominatorLimit = 2;

        if (gyroConfig()->gyro_lpf == 0) {
            denominatorLimit = 16;
        }

        if (gyroConfig()->gyroSyncDenominator < denominatorLimit) {
            gyroConfigMutable()->gyroSyncDenominator = denominatorLimit;
        }

        if (gyroConfig()->looptime < 2000) {
            gyroConfigMutable()->looptime = 2000;
        }

    }
#else

#endif

#if defined(LED_STRIP) && (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && (
            0
#ifdef USE_SOFTSERIAL1
            || (WS2811_TIMER == SOFTSERIAL_1_TIMER)
#endif
#ifdef USE_SOFTSERIAL2
            || (WS2811_TIMER == SOFTSERIAL_2_TIMER)
#endif
    )) {
        // led strip needs the same timer as softserial
        featureClear(FEATURE_LED_STRIP);
    }
#endif

#if defined(NAZE) && defined(SONAR)
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM) && featureConfigured(FEATURE_SONAR) && featureConfigured(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(OLIMEXINO) && defined(SONAR)
    if (feature(FEATURE_SONAR) && feature(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(CC3D) && defined(USE_DASHBOARD) && defined(USE_UART3)
    if (doesConfigurationUsePort(SERIAL_PORT_USART3) && feature(FEATURE_DASHBOARD)) {
        featureClear(FEATURE_DASHBOARD);
    }
#endif

#if defined(CC3D)
#if defined(CC3D_PPM1)
#if defined(SONAR) && defined(USE_SOFTSERIAL1)
    if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
        featureClear(FEATURE_SONAR);
    }
#endif
#else
#if defined(SONAR) && defined(USE_SOFTSERIAL1) && defined(RSSI_ADC_GPIO)
    // shared pin
    if ((featureConfigured(FEATURE_SONAR) + featureConfigured(FEATURE_SOFTSERIAL) + featureConfigured(FEATURE_RSSI_ADC)) > 1) {
       featureClear(FEATURE_SONAR);
       featureClear(FEATURE_SOFTSERIAL);
       featureClear(FEATURE_RSSI_ADC);
    }
#endif
#endif // CC3D_PPM1
#endif // CC3D

#ifndef USE_PMW_SERVO_DRIVER
    featureClear(FEATURE_PWM_SERVO_DRIVER);
#endif

    if (!isSerialConfigValid(serialConfigMutable())) {
        pgResetCopy(serialConfigMutable(), PG_SERIAL_CONFIG);
    }

    /*
     * If provided predefined mixer setup is disabled, fallback to default one
     */
    if (!isMixerEnabled(mixerConfig()->mixerMode)) {
        mixerConfigMutable()->mixerMode = DEFAULT_MIXER;
    }

#if defined(NAV)
    // Ensure sane values of navConfig settings
    validateNavConfig();
#endif

    /* Limitations of different protocols */
#ifdef BRUSHED_MOTORS
    motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 500, 32000);
#else
    switch (motorConfig()->motorPwmProtocol) {
    case PWM_TYPE_STANDARD: // Limited to 490 Hz
        motorConfigMutable()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 490);
        break;

    case PWM_TYPE_ONESHOT125:   // Limited to 3900 Hz
        motorConfigMutable()->motorPwmRate = MIN(motorConfig()->motorPwmRate, 3900);
        break;

    case PWM_TYPE_ONESHOT42:    // 2-8 kHz
        motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 2000, 8000);
        break;

    case PWM_TYPE_MULTISHOT:    // 2-16 kHz
        motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 2000, 16000);
        break;
    case PWM_TYPE_BRUSHED:      // 500Hz - 32kHz
        motorConfigMutable()->motorPwmRate = constrain(motorConfig()->motorPwmRate, 500, 32000);
        break;
    }
#endif
}

void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch)
{
    updateBoardAlignment(roll, pitch);

    saveConfigAndNotify();
}

void readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check, read flash
    if (!loadEEPROM()) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    setProfile(getCurrentProfileIndex());
    setControlRateProfile(getCurrentProfileIndex());
    pgActivateProfile(getCurrentProfileIndex());

    validateAndFixConfig();
    activateConfig();

    resumeRxSignal();
}

void writeEEPROM(void)
{
    suspendRxSignal();

    writeConfigToEEPROM();

    resumeRxSignal();
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }
    resetEEPROM();
}

void resetEEPROM(void)
{
    resetConfigs();
    writeEEPROM();
}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(1);
}

uint8_t getCurrentProfileIndex(void)
{
    return profileSelection()->current_profile_index;
}

void setProfile(uint8_t profileIndex)
{
    if (profileIndex >= MAX_PROFILE_COUNT) {// sanity check
        profileIndex = 0;
    }
    profileSelectionMutable()->current_profile_index = profileIndex;
    currentProfile = &masterConfig.profile[profileIndex];
}

void changeProfile(uint8_t profileIndex)
{
    if (profileIndex >= MAX_PROFILE_COUNT) {
        profileIndex = MAX_PROFILE_COUNT - 1;
    }
    profileSelectionMutable()->current_profile_index = profileIndex;
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(profileIndex + 1);
}

void persistentFlagClearAll()
{
    masterConfig.persistentFlags = 0;
}

bool persistentFlag(uint8_t mask)
{
    return masterConfig.persistentFlags & mask;
}

void persistentFlagSet(uint8_t mask)
{
    masterConfig.persistentFlags |= mask;
}

void persistentFlagClear(uint8_t mask)
{
    masterConfig.persistentFlags &= ~(mask);
}

void beeperOffSet(uint32_t mask)
{
    masterConfig.beeper_off_flags |= mask;
}

void beeperOffSetAll(uint8_t beeperCount)
{
    masterConfig.beeper_off_flags = (1 << beeperCount) -1;
}

void beeperOffClear(uint32_t mask)
{
    masterConfig.beeper_off_flags &= ~(mask);
}

void beeperOffClearAll(void)
{
    masterConfig.beeper_off_flags = 0;
}

uint32_t getBeeperOffMask(void)
{
    return masterConfig.beeper_off_flags;
}

void setBeeperOffMask(uint32_t mask)
{
    masterConfig.beeper_off_flags = mask;
}

uint32_t getPreferredBeeperOffMask(void)
{
    return masterConfig.preferred_beeper_off_flags;
}

void setPreferredBeeperOffMask(uint32_t mask)
{
    masterConfig.preferred_beeper_off_flags = mask;
}
