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
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "drivers/system.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/fc_rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "pg/beeper.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#ifndef USE_OSD_SLAVE
pidProfile_t *currentPidProfile;
#endif

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);

PG_RESET_TEMPLATE(pilotConfig_t, pilotConfig,
    .name = { 0 }
);

PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);

PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .pidProfileIndex = 0,
    .activeRateProfile = 0,
    .debug_mode = DEBUG_MODE,
    .task_statistics = true,
    .cpu_overclock = 0,
    .powerOnArmingGraceTime = 5,
    .boardIdentifier = TARGET_BOARD_IDENTIFIER
);

#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

#ifndef USE_OSD_SLAVE
uint8_t getCurrentPidProfileIndex(void)
{
    return systemConfig()->pidProfileIndex;
}

static void setPidProfile(uint8_t pidProfileIndex)
{
    if (pidProfileIndex < MAX_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = pidProfileIndex;
        currentPidProfile = pidProfilesMutable(pidProfileIndex);
    }
}

uint8_t getCurrentControlRateProfileIndex(void)
{
    return systemConfigMutable()->activeRateProfile;
}

uint16_t getCurrentMinthrottle(void)
{
    return motorConfig()->minthrottle;
}
#endif // USE_OSD_SLAVE

void resetConfigs(void)
{
    pgResetAll();

#if defined(USE_TARGET_CONFIG)
    targetConfiguration();
#endif

#ifndef USE_OSD_SLAVE
    setPidProfile(0);
    setControlRateProfile(0);
#endif

#ifdef USE_LED_STRIP
    reevaluateLedConfig();
#endif
}

void activateConfig(void)
{
#ifndef USE_OSD_SLAVE
    initRcProcessing();

    resetAdjustmentStates();

    pidInit(currentPidProfile);
    useRcControlsConfig(currentPidProfile);
    useAdjustmentConfig(currentPidProfile);

#ifdef USE_NAV
    gpsUsePIDs(currentPidProfile);
#endif

    failsafeReset();
    setAccelerationTrims(&accelerometerConfigMutable()->accZero);
    accInitFilters();

    imuConfigure(throttleCorrectionConfig()->throttle_correction_angle);
#endif // USE_OSD_SLAVE
}

static void validateAndFixConfig(void)
{
#if !defined(USE_QUAD_MIXER_ONLY) && !defined(USE_OSD_SLAVE)
    // Reset unsupported mixer mode to default.
    // This check will be gone when motor/servo mixers are loaded dynamically
    // by configurator as a part of configuration procedure.

    mixerMode_e mixerMode = mixerConfigMutable()->mixerMode;

    if (!(mixerMode == MIXER_CUSTOM || mixerMode == MIXER_CUSTOM_AIRPLANE || mixerMode == MIXER_CUSTOM_TRI)) {
        if (mixers[mixerMode].motorCount && mixers[mixerMode].motor == NULL)
            mixerConfigMutable()->mixerMode = MIXER_CUSTOM;
#ifdef USE_SERVOS
        if (mixers[mixerMode].useServo && servoMixers[mixerMode].servoRuleCount == 0)
            mixerConfigMutable()->mixerMode = MIXER_CUSTOM_AIRPLANE;
#endif
    }
#endif

#ifndef USE_OSD_SLAVE
    if (systemConfig()->activeRateProfile >= CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = 0;
    }
    setControlRateProfile(systemConfig()->activeRateProfile);

    if (systemConfig()->pidProfileIndex >= MAX_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = 0;
    }
    setPidProfile(systemConfig()->pidProfileIndex);

    // Prevent invalid notch cutoff
    if (currentPidProfile->dterm_notch_cutoff >= currentPidProfile->dterm_notch_hz) {
        currentPidProfile->dterm_notch_hz = 0;
    }

    if ((motorConfig()->dev.motorPwmProtocol == PWM_TYPE_BRUSHED) && (motorConfig()->mincommand < 1000)) {
        motorConfigMutable()->mincommand = 1000;
    }

    if ((motorConfig()->dev.motorPwmProtocol == PWM_TYPE_STANDARD) && (motorConfig()->dev.motorPwmRate > BRUSHLESS_MOTORS_PWM_RATE)) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
    }

    validateAndFixGyroConfig();

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

#ifdef USE_RX_SPI
    if (featureConfigured(FEATURE_RX_SPI)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_MSP);
    }
#endif // USE_RX_SPI

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
#if defined(STM32F10X)
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (batteryConfig()->currentMeterSource == CURRENT_METER_ADC) {
            batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;
        }
#endif // STM32F10X
        // software serial needs free PWM ports
        featureClear(FEATURE_SOFTSERIAL);
    }

#ifdef USE_SOFTSPI
    if (featureConfigured(FEATURE_SOFTSPI)) {
        featureClear(FEATURE_RX_PPM | FEATURE_RX_PARALLEL_PWM | FEATURE_SOFTSERIAL);
        batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_NONE;
#if defined(STM32F10X)
        featureClear(FEATURE_LED_STRIP);
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (batteryConfig()->currentMeterSource == CURRENT_METER_ADC) {
            batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;
        }
#endif // STM32F10X
    }
#endif // USE_SOFTSPI

#endif // USE_OSD_SLAVE

    if (!isSerialConfigValid(serialConfig())) {
        pgResetFn_serialConfig(serialConfigMutable());
    }

// clear features that are not supported.
// I have kept them all here in one place, some could be moved to sections of code above.

#ifndef USE_PPM
    featureClear(FEATURE_RX_PPM);
#endif

#ifndef USE_SERIAL_RX
    featureClear(FEATURE_RX_SERIAL);
#endif

#if !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2)
    featureClear(FEATURE_SOFTSERIAL);
#endif

#ifndef USE_GPS
    featureClear(FEATURE_GPS);
#endif

#ifndef USE_RANGEFINDER
    featureClear(FEATURE_RANGEFINDER);
#endif

#ifndef USE_TELEMETRY
    featureClear(FEATURE_TELEMETRY);
#endif

#ifndef USE_PWM
    featureClear(FEATURE_RX_PARALLEL_PWM);
#endif

#ifndef USE_RX_MSP
    featureClear(FEATURE_RX_MSP);
#endif

#ifndef USE_LED_STRIP
    featureClear(FEATURE_LED_STRIP);
#endif

#ifndef USE_DASHBOARD
    featureClear(FEATURE_DASHBOARD);
#endif

#ifndef USE_OSD
    featureClear(FEATURE_OSD);
#endif

#ifndef USE_SERVOS
    featureClear(FEATURE_SERVO_TILT | FEATURE_CHANNEL_FORWARDING);
#endif

#ifndef USE_TRANSPONDER
    featureClear(FEATURE_TRANSPONDER);
#endif

#ifndef USE_RX_SPI
    featureClear(FEATURE_RX_SPI);
#endif

#ifndef USE_SOFTSPI
    featureClear(FEATURE_SOFTSPI);
#endif

#ifndef USE_ESC_SENSOR
    featureClear(FEATURE_ESC_SENSOR);
#endif

#ifndef USE_GYRO_DATA_ANALYSE
    featureClear(FEATURE_DYNAMIC_FILTER);
#endif

#if defined(TARGET_VALIDATECONFIG)
    targetValidateConfiguration();
#endif
}

#ifndef USE_OSD_SLAVE
void validateAndFixGyroConfig(void)
{
    #ifdef USE_GYRO_IMUF9001
    if ((motorConfigMutable()->dev.motorPwmProtocol == PWM_TYPE_DSHOT1200 
      || motorConfigMutable()->dev.motorPwmProtocol == PWM_TYPE_DSHOT600)
      && (pidConfigMutable()->pid_process_denom == 1 && gyroConfigMutable()->gyro_sync_denom == 1)
      && rxConfigMutable()->rcInterpolation != RC_SMOOTHING_OFF
      && systemConfigMutable()->cpu_overclock != 2) 
    {
        systemConfigMutable()->cpu_overclock = 2; //216MHZ is required for dshot + rc_interpolation + 32K pid loop.
    }
    gyroConfigMutable()->imuf_rate = constrain(gyroConfigMutable()->gyro_sync_denom - 1, 0, 5);
    #endif
    // Prevent invalid notch cutoff
    if (gyroConfig()->gyro_soft_notch_cutoff_1 >= gyroConfig()->gyro_soft_notch_hz_1) {
        gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    }
    if (gyroConfig()->gyro_soft_notch_cutoff_2 >= gyroConfig()->gyro_soft_notch_hz_2) {
        gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    }

    if (gyroConfig()->gyro_lpf != GYRO_LPF_256HZ && gyroConfig()->gyro_lpf != GYRO_LPF_NONE) {
        pidConfigMutable()->pid_process_denom = 1; // When gyro set to 1khz always set pid speed 1:1 to sampling speed
        gyroConfigMutable()->gyro_sync_denom = 1;
        gyroConfigMutable()->gyro_use_32khz = false;
    }

    if (gyroConfig()->gyro_use_32khz) {
        // F1 and F3 can't handle high sample speed.
#if defined(STM32F1)
        gyroConfigMutable()->gyro_sync_denom = MAX(gyroConfig()->gyro_sync_denom, 16);
#elif defined(STM32F3)
        gyroConfigMutable()->gyro_sync_denom = MAX(gyroConfig()->gyro_sync_denom, 4);
#endif
    } else {
#if defined(STM32F1)
        gyroConfigMutable()->gyro_sync_denom = MAX(gyroConfig()->gyro_sync_denom, 3);
#endif
    }

    float samplingTime;
    switch (gyroMpuDetectionResult()->sensor) {
    case ICM_20649_SPI:
        samplingTime = 1.0f / 9000.0f;
        break;
    case BMI_160_SPI:
        samplingTime = 0.0003125f;
        break;
    default:
        samplingTime = 0.000125f;
        break;
    }
    if (gyroConfig()->gyro_lpf != GYRO_LPF_256HZ && gyroConfig()->gyro_lpf != GYRO_LPF_NONE) {
        switch (gyroMpuDetectionResult()->sensor) {
        case ICM_20649_SPI:
            samplingTime = 1.0f / 1100.0f;
            break;
        default:
            samplingTime = 0.001f;
            break;
        }
    }
    if (gyroConfig()->gyro_use_32khz) {
        samplingTime = 0.00003125;
    }

    // check for looptime restrictions based on motor protocol. Motor times have safety margin
    float motorUpdateRestriction;
    switch (motorConfig()->dev.motorPwmProtocol) {
    case PWM_TYPE_STANDARD:
            motorUpdateRestriction = 1.0f / BRUSHLESS_MOTORS_PWM_RATE;
            break;
    case PWM_TYPE_ONESHOT125:
            motorUpdateRestriction = 0.0005f;
            break;
    case PWM_TYPE_ONESHOT42:
            motorUpdateRestriction = 0.0001f;
            break;
#ifdef USE_DSHOT
    case PWM_TYPE_DSHOT150:
            motorUpdateRestriction = 0.000250f;
            break;
    case PWM_TYPE_DSHOT300:
            motorUpdateRestriction = 0.0001f;
            break;
#endif
    default:
        motorUpdateRestriction = 0.00003125f;
        break;
    }

    if (motorConfig()->dev.useUnsyncedPwm) {
        // Prevent overriding the max rate of motors
        if ((motorConfig()->dev.motorPwmProtocol <= PWM_TYPE_BRUSHED) && (motorConfig()->dev.motorPwmProtocol != PWM_TYPE_STANDARD)) {
            const uint32_t maxEscRate = lrintf(1.0f / motorUpdateRestriction);
            motorConfigMutable()->dev.motorPwmRate = MIN(motorConfig()->dev.motorPwmRate, maxEscRate);
        }
    } else {
        const float pidLooptime = samplingTime * gyroConfig()->gyro_sync_denom * pidConfig()->pid_process_denom;
        if (pidLooptime < motorUpdateRestriction) {
            const uint8_t minPidProcessDenom = constrain(motorUpdateRestriction / (samplingTime * gyroConfig()->gyro_sync_denom), 1, MAX_PID_PROCESS_DENOM);
            pidConfigMutable()->pid_process_denom = MAX(pidConfigMutable()->pid_process_denom, minPidProcessDenom);
        }
    }
}
#endif // USE_OSD_SLAVE

void readEEPROM(void)
{
#ifndef USE_OSD_SLAVE
    suspendRxSignal();
#endif

    // Sanity check, read flash
    if (!loadEEPROM()) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    validateAndFixConfig();
    activateConfig();

#ifndef USE_OSD_SLAVE
    resumeRxSignal();
#endif
}

void writeEEPROM(void)
{
#ifndef USE_OSD_SLAVE
    suspendRxSignal();
#endif

    writeConfigToEEPROM();

#ifndef USE_OSD_SLAVE
    resumeRxSignal();
#endif
}

void resetEEPROM(void)
{
    resetConfigs();
    writeEEPROM();
}

void ensureEEPROMContainsValidData(void)
{
    if (isEEPROMContentValid()) {
        return;
    }
    resetEEPROM();
}

void saveConfigAndNotify(void)
{
    writeEEPROM();
    readEEPROM();
    beeperConfirmationBeeps(1);
}

#ifndef USE_OSD_SLAVE
void changePidProfile(uint8_t pidProfileIndex)
{
    if (pidProfileIndex >= MAX_PROFILE_COUNT) {
        pidProfileIndex = MAX_PROFILE_COUNT - 1;
    }
    systemConfigMutable()->pidProfileIndex = pidProfileIndex;
    currentPidProfile = pidProfilesMutable(pidProfileIndex);
    beeperConfirmationBeeps(pidProfileIndex + 1);
}
#endif

void beeperOffSet(uint32_t mask)
{
#ifdef BEEPER
    beeperConfigMutable()->beeper_off_flags |= mask;
#else
    UNUSED(mask);
#endif
}

void beeperOffSetAll(uint8_t beeperCount)
{
#ifdef BEEPER
    beeperConfigMutable()->beeper_off_flags = (1 << beeperCount) -1;
#else
    UNUSED(beeperCount);
#endif
}

void beeperOffClear(uint32_t mask)
{
#ifdef BEEPER
    beeperConfigMutable()->beeper_off_flags &= ~(mask);
#else
    UNUSED(mask);
#endif
}

void beeperOffClearAll(void)
{
#ifdef BEEPER
    beeperConfigMutable()->beeper_off_flags = 0;
#endif
}

uint32_t getBeeperOffMask(void)
{
#ifdef BEEPER
    return beeperConfig()->beeper_off_flags;
#else
    return 0;
#endif
}

void setBeeperOffMask(uint32_t mask)
{
#ifdef BEEPER
    beeperConfigMutable()->beeper_off_flags = mask;
#else
    UNUSED(mask);
#endif
}

uint32_t getPreferredBeeperOffMask(void)
{
#ifdef BEEPER
    return beeperConfig()->preferred_beeper_off_flags;
#else
    return 0;
#endif
}

void setPreferredBeeperOffMask(uint32_t mask)
{
#ifdef BEEPER
    beeperConfigMutable()->preferred_beeper_off_flags = mask;
#else
    UNUSED(mask);
#endif
}
