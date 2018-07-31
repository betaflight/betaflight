/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
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
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/rx.h"
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

#define DYNAMIC_FILTER_MAX_SUPPORTED_LOOP_TIME HZ_TO_INTERVAL_US(2000)

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

#ifndef USE_OSD_SLAVE
uint8_t getCurrentPidProfileIndex(void)
{
    return systemConfig()->pidProfileIndex;
}

static void loadPidProfile(void)
{
    currentPidProfile = pidProfilesMutable(systemConfig()->pidProfileIndex);
}

uint8_t getCurrentControlRateProfileIndex(void)
{
    return systemConfig()->activeRateProfile;
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
}

static void activateConfig(void)
{
#ifndef USE_OSD_SLAVE
    loadPidProfile();
    loadControlRateProfile();

    initRcProcessing();

    resetAdjustmentStates();

    pidInit(currentPidProfile);
    useRcControlsConfig(currentPidProfile);
    useAdjustmentConfig(currentPidProfile);

    failsafeReset();
    setAccelerationTrims(&accelerometerConfigMutable()->accZero);
    accInitFilters();

    imuConfigure(throttleCorrectionConfig()->throttle_correction_angle, throttleCorrectionConfig()->throttle_correction_value);
#endif // USE_OSD_SLAVE

#ifdef USE_LED_STRIP
    reevaluateLedConfig();
#endif
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

    if (!isSerialConfigValid(serialConfig())) {
        pgResetFn_serialConfig(serialConfigMutable());
    }

    if (
#if defined(USE_GPS)
        !findSerialPortConfig(FUNCTION_GPS) &&
#endif
        true) {
        featureClear(FEATURE_GPS);
    }

#ifndef USE_OSD_SLAVE
    if (systemConfig()->activeRateProfile >= CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = 0;
    }
    loadControlRateProfile();

    if (systemConfig()->pidProfileIndex >= MAX_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = 0;
    }
    loadPidProfile();

    // Prevent invalid notch cutoff
    if (currentPidProfile->dterm_notch_cutoff >= currentPidProfile->dterm_notch_hz) {
        currentPidProfile->dterm_notch_hz = 0;
    }

    if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_BRUSHED) {
        featureClear(FEATURE_3D);

        if (motorConfig()->mincommand < 1000) {
            motorConfigMutable()->mincommand = 1000;
        }
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

#if defined(USE_ADC)
    if (featureConfigured(FEATURE_RSSI_ADC)) {
        rxConfigMutable()->rssi_channel = 0;
        rxConfigMutable()->rssi_src_frame_errors = false;
    } else
#endif
    if (rxConfigMutable()->rssi_channel
#if defined(USE_PWM) || defined(USE_PPM)
        || featureConfigured(FEATURE_RX_PPM) || featureConfigured(FEATURE_RX_PARALLEL_PWM)
#endif
        ) {
        rxConfigMutable()->rssi_src_frame_errors = false;
    }

    if (!rcSmoothingIsEnabled() || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_T) {
        for (unsigned i = 0; i < MAX_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->pid[PID_ROLL].F = 0;
            pidProfilesMutable(i)->pid[PID_PITCH].F = 0;
        }
    }

    if (!rcSmoothingIsEnabled() ||
        (rxConfig()->rcInterpolationChannels != INTERPOLATION_CHANNELS_RPY &&
         rxConfig()->rcInterpolationChannels != INTERPOLATION_CHANNELS_RPYT)) {

        for (unsigned i = 0; i < MAX_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->pid[PID_YAW].F = 0;
        }
    }
    
#if defined(USE_THROTTLE_BOOST)
    if (!rcSmoothingIsEnabled() ||
        !(rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_RPYT
        || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_T
        || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_RPT)) {
        for (unsigned i = 0; i < MAX_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->throttle_boost = 0;
        }
    }
#endif

    if (
        featureConfigured(FEATURE_3D) || !featureConfigured(FEATURE_GPS)
#if !defined(USE_GPS) || !defined(USE_GPS_RESCUE)
        || true
#endif
        ) {
        if (failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_GPS_RESCUE) {
            failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT;
        }

        if (isModeActivationConditionPresent(BOXGPSRESCUE)) {
            removeModeActivationCondition(BOXGPSRESCUE);
        }
    }
#endif // USE_OSD_SLAVE

#if defined(USE_ESC_SENSOR)
    if (!findSerialPortConfig(FUNCTION_ESC_SENSOR)) {
        featureClear(FEATURE_ESC_SENSOR);
    }
#endif

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

#if !defined(USE_ADC)
    featureClear(FEATURE_RSSI_ADC);
#endif

#if defined(USE_BEEPER)
    if (beeperDevConfig()->frequency && !timerGetByTag(beeperDevConfig()->ioTag)) {
        beeperDevConfigMutable()->frequency = 0;
    }

    if (beeperConfig()->beeper_off_flags & ~BEEPER_ALLOWED_MODES) {
        beeperConfigMutable()->beeper_off_flags = 0;
    }

#ifdef USE_DSHOT
    if (beeperConfig()->dshotBeaconOffFlags & ~DSHOT_BEACON_ALLOWED_MODES) {
        beeperConfigMutable()->dshotBeaconOffFlags = 0;
    }

    if (beeperConfig()->dshotBeaconTone < DSHOT_CMD_BEACON1
        || beeperConfig()->dshotBeaconTone > DSHOT_CMD_BEACON5) {
        beeperConfigMutable()->dshotBeaconTone = DSHOT_CMD_BEACON1;
    }
#endif
#endif

#if defined(TARGET_VALIDATECONFIG)
    targetValidateConfiguration();
#endif
}

#ifndef USE_OSD_SLAVE
void validateAndFixGyroConfig(void)
{
#ifdef USE_GYRO_DATA_ANALYSE
    // Disable dynamic filter if gyro loop is less than 2KHz
    if (gyro.targetLooptime > DYNAMIC_FILTER_MAX_SUPPORTED_LOOP_TIME) {
        featureClear(FEATURE_DYNAMIC_FILTER);
    }
#endif
    
    // Prevent invalid notch cutoff
    if (gyroConfig()->gyro_soft_notch_cutoff_1 >= gyroConfig()->gyro_soft_notch_hz_1) {
        gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    }
    if (gyroConfig()->gyro_soft_notch_cutoff_2 >= gyroConfig()->gyro_soft_notch_hz_2) {
        gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    }

    if (gyroConfig()->gyro_hardware_lpf != GYRO_HARDWARE_LPF_NORMAL && gyroConfig()->gyro_hardware_lpf != GYRO_HARDWARE_LPF_EXPERIMENTAL) {
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
    if (gyroConfig()->gyro_hardware_lpf != GYRO_HARDWARE_LPF_NORMAL && gyroConfig()->gyro_hardware_lpf != GYRO_HARDWARE_LPF_EXPERIMENTAL) {
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

bool readEEPROM(void)
{
#ifndef USE_OSD_SLAVE
    suspendRxSignal();
#endif

    // Sanity check, read flash
    bool success = loadEEPROM();

    validateAndFixConfig();

    activateConfig();

#ifndef USE_OSD_SLAVE
    resumeRxSignal();
#endif

    return success;
}

void writeEEPROM(void)
{
    validateAndFixConfig();

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

    activateConfig();
}

void ensureEEPROMStructureIsValid(void)
{
    if (isEEPROMStructureValid()) {
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
    if (pidProfileIndex < MAX_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = pidProfileIndex;
        loadPidProfile();
    }

    beeperConfirmationBeeps(pidProfileIndex + 1);
}
#endif
