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

#include "blackbox/blackbox.h"

#include "build/debug.h"

#include "config/config_eeprom.h"
#include "config/feature.h"

#include "drivers/system.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
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
#include "io/gps.h"

#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/rx.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/rpm_filter.h"

#include "scheduler/scheduler.h"

static bool configIsDirty; /* someone indicated that the config is modified and it is not yet saved */

pidProfile_t *currentPidProfile;

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

#define DYNAMIC_FILTER_MAX_SUPPORTED_LOOP_TIME HZ_TO_INTERVAL_US(2000)

PG_REGISTER_WITH_RESET_TEMPLATE(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 1);

PG_RESET_TEMPLATE(pilotConfig_t, pilotConfig,
    .name = { 0 },
    .displayName = { 0 },
);

PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);

PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .pidProfileIndex = 0,
    .activeRateProfile = 0,
    .debug_mode = DEBUG_MODE,
    .task_statistics = true,
    .cpu_overclock = 0,
    .powerOnArmingGraceTime = 5,
    .boardIdentifier = TARGET_BOARD_IDENTIFIER,
    .hseMhz = SYSTEM_HSE_VALUE,  // Not used for non-F4 targets
    .configured = false,
    .schedulerOptimizeRate = false,
);

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

void resetConfigs(void)
{
    pgResetAll();

#if defined(USE_TARGET_CONFIG)
    targetConfiguration();
#endif
}

static void activateConfig(void)
{
    schedulerOptimizeRate(systemConfig()->schedulerOptimizeRate);
    loadPidProfile();
    loadControlRateProfile();

    initRcProcessing();

    resetAdjustmentStates();

    pidInit(currentPidProfile);

    rcControlsInit();

    failsafeReset();
#ifdef USE_ACC
    setAccelerationTrims(&accelerometerConfigMutable()->accZero);
    accInitFilters();
#endif

    imuConfigure(throttleCorrectionConfig()->throttle_correction_angle, throttleCorrectionConfig()->throttle_correction_value);

#if defined(USE_LED_STRIP_STATUS_MODE)
    reevaluateLedConfig();
#endif
}

static void validateAndFixConfig(void)
{
#if !defined(USE_QUAD_MIXER_ONLY)
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

#if defined(USE_GPS)
    serialPortConfig_t *gpsSerial = findSerialPortConfig(FUNCTION_GPS);
    if (gpsConfig()->provider == GPS_MSP && gpsSerial) {
        serialRemovePort(gpsSerial->identifier);
    }
#endif
    if (
#if defined(USE_GPS)
        gpsConfig()->provider != GPS_MSP && !gpsSerial &&
#endif
        true) {
        featureDisable(FEATURE_GPS);
    }

    for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
        // Prevent invalid notch cutoff
        if (pidProfilesMutable(i)->dterm_notch_cutoff >= pidProfilesMutable(i)->dterm_notch_hz) {
            pidProfilesMutable(i)->dterm_notch_hz = 0;
        }

#ifdef USE_DYN_LPF
        //Prevent invalid dynamic lowpass
        if (pidProfilesMutable(i)->dyn_lpf_dterm_min_hz > pidProfilesMutable(i)->dyn_lpf_dterm_max_hz) {
            pidProfilesMutable(i)->dyn_lpf_dterm_min_hz = 0;
        }
#endif

        if (pidProfilesMutable(i)->motor_output_limit > 100 || pidProfilesMutable(i)->motor_output_limit == 0) {
            pidProfilesMutable(i)->motor_output_limit = 100;
        }

        if (pidProfilesMutable(i)->auto_profile_cell_count > MAX_AUTO_DETECT_CELL_COUNT || pidProfilesMutable(i)->auto_profile_cell_count < AUTO_PROFILE_CELL_COUNT_CHANGE) {
            pidProfilesMutable(i)->auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY;
        }
    }

    if (motorConfig()->dev.motorPwmProtocol == PWM_TYPE_BRUSHED) {
        featureDisable(FEATURE_3D);

        if (motorConfig()->mincommand < 1000) {
            motorConfigMutable()->mincommand = 1000;
        }
    }

    if ((motorConfig()->dev.motorPwmProtocol == PWM_TYPE_STANDARD) && (motorConfig()->dev.motorPwmRate > BRUSHLESS_MOTORS_PWM_RATE)) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
    }

    validateAndFixGyroConfig();

    if (!(featureIsEnabled(FEATURE_RX_PARALLEL_PWM) || featureIsEnabled(FEATURE_RX_PPM) || featureIsEnabled(FEATURE_RX_SERIAL) || featureIsEnabled(FEATURE_RX_MSP) || featureIsEnabled(FEATURE_RX_SPI))) {
        featureEnable(DEFAULT_RX_FEATURE);
    }

    if (featureIsEnabled(FEATURE_RX_PPM)) {
        featureDisable(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_SPI);
    }

    if (featureIsEnabled(FEATURE_RX_MSP)) {
        featureDisable(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

    if (featureIsEnabled(FEATURE_RX_SERIAL)) {
        featureDisable(FEATURE_RX_PARALLEL_PWM | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

#ifdef USE_RX_SPI
    if (featureIsEnabled(FEATURE_RX_SPI)) {
        featureDisable(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_MSP);
    }
#endif // USE_RX_SPI

    if (featureIsEnabled(FEATURE_RX_PARALLEL_PWM)) {
        featureDisable(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
    }

#ifdef USE_SOFTSPI
    if (featureIsEnabled(FEATURE_SOFTSPI)) {
        featureDisable(FEATURE_RX_PPM | FEATURE_RX_PARALLEL_PWM | FEATURE_SOFTSERIAL);
        batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_NONE;
#if defined(STM32F10X)
        featureDisable(FEATURE_LED_STRIP);
        // rssi adc needs the same ports
        featureDisable(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (batteryConfig()->currentMeterSource == CURRENT_METER_ADC) {
            batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;
        }
#endif // STM32F10X
    }
#endif // USE_SOFTSPI

#if defined(USE_ADC)
    if (featureIsEnabled(FEATURE_RSSI_ADC)) {
        rxConfigMutable()->rssi_channel = 0;
        rxConfigMutable()->rssi_src_frame_errors = false;
    } else
#endif
    if (rxConfigMutable()->rssi_channel
#if defined(USE_PWM) || defined(USE_PPM)
        || featureIsEnabled(FEATURE_RX_PPM) || featureIsEnabled(FEATURE_RX_PARALLEL_PWM)
#endif
        ) {
        rxConfigMutable()->rssi_src_frame_errors = false;
    }

    if (!rcSmoothingIsEnabled() || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_T) {
        for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->pid[PID_ROLL].F = 0;
            pidProfilesMutable(i)->pid[PID_PITCH].F = 0;
        }
    }

    if (!rcSmoothingIsEnabled() ||
        (rxConfig()->rcInterpolationChannels != INTERPOLATION_CHANNELS_RPY &&
         rxConfig()->rcInterpolationChannels != INTERPOLATION_CHANNELS_RPYT)) {

        for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->pid[PID_YAW].F = 0;
        }
    }

#if defined(USE_THROTTLE_BOOST)
    if (!rcSmoothingIsEnabled() ||
        !(rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_RPYT
        || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_T
        || rxConfig()->rcInterpolationChannels == INTERPOLATION_CHANNELS_RPT)) {
        for (unsigned i = 0; i < PID_PROFILE_COUNT; i++) {
            pidProfilesMutable(i)->throttle_boost = 0;
        }
    }
#endif

    if (
        featureIsEnabled(FEATURE_3D) || !featureIsEnabled(FEATURE_GPS)
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

#if defined(USE_ESC_SENSOR)
    if (!findSerialPortConfig(FUNCTION_ESC_SENSOR)) {
        featureDisable(FEATURE_ESC_SENSOR);
    }
#endif

    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = modeActivationConditions(i);

        if (mac->linkedTo) {
            if (mac->modeId == BOXARM || isModeActivationConditionLinked(mac->linkedTo)) {
                removeModeActivationCondition(mac->modeId);
            }
        }
    }

// clear features that are not supported.
// I have kept them all here in one place, some could be moved to sections of code above.

#ifndef USE_PPM
    featureDisable(FEATURE_RX_PPM);
#endif

#ifndef USE_SERIAL_RX
    featureDisable(FEATURE_RX_SERIAL);
#endif

#if !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2)
    featureDisable(FEATURE_SOFTSERIAL);
#endif

#ifndef USE_RANGEFINDER
    featureDisable(FEATURE_RANGEFINDER);
#endif

#ifndef USE_TELEMETRY
    featureDisable(FEATURE_TELEMETRY);
#endif

#ifndef USE_PWM
    featureDisable(FEATURE_RX_PARALLEL_PWM);
#endif

#ifndef USE_RX_MSP
    featureDisable(FEATURE_RX_MSP);
#endif

#ifndef USE_LED_STRIP
    featureDisable(FEATURE_LED_STRIP);
#endif

#ifndef USE_DASHBOARD
    featureDisable(FEATURE_DASHBOARD);
#endif

#ifndef USE_OSD
    featureDisable(FEATURE_OSD);
#endif

#ifndef USE_SERVOS
    featureDisable(FEATURE_SERVO_TILT | FEATURE_CHANNEL_FORWARDING);
#endif

#ifndef USE_TRANSPONDER
    featureDisable(FEATURE_TRANSPONDER);
#endif

#ifndef USE_RX_SPI
    featureDisable(FEATURE_RX_SPI);
#endif

#ifndef USE_SOFTSPI
    featureDisable(FEATURE_SOFTSPI);
#endif

#ifndef USE_ESC_SENSOR
    featureDisable(FEATURE_ESC_SENSOR);
#endif

#ifndef USE_GYRO_DATA_ANALYSE
    featureDisable(FEATURE_DYNAMIC_FILTER);
#endif

#if !defined(USE_ADC)
    featureDisable(FEATURE_RSSI_ADC);
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

#if defined(USE_DSHOT_TELEMETRY)
    bool usingDshotProtocol;
    switch (motorConfig()->dev.motorPwmProtocol) {
    case PWM_TYPE_PROSHOT1000:
    case PWM_TYPE_DSHOT1200:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT150:
        usingDshotProtocol = true;
        break;
    default:
        usingDshotProtocol = false;
        break;
    }

    if ((!usingDshotProtocol || motorConfig()->dev.useBurstDshot || !systemConfig()->schedulerOptimizeRate)
        && motorConfig()->dev.useDshotTelemetry) {
        motorConfigMutable()->dev.useDshotTelemetry = false;
    }
#endif

    // Temporary workaround until RPM Filter supports dual-gyro using both sensors
    // Once support is added remove this block
#if defined(USE_MULTI_GYRO) && defined(USE_RPM_FILTER)
    if (gyroConfig()->gyro_to_use == GYRO_CONFIG_USE_GYRO_BOTH && isRpmFilterEnabled()) {
        gyroConfigMutable()->gyro_to_use = GYRO_CONFIG_USE_GYRO_1;
    }
#endif

#if defined(TARGET_VALIDATECONFIG)
    targetValidateConfiguration();
#endif
}

void validateAndFixGyroConfig(void)
{
#ifdef USE_GYRO_DATA_ANALYSE
    // Disable dynamic filter if gyro loop is less than 2KHz
    if (gyro.targetLooptime > DYNAMIC_FILTER_MAX_SUPPORTED_LOOP_TIME) {
        featureDisable(FEATURE_DYNAMIC_FILTER);
    }
#endif

    // Prevent invalid notch cutoff
    if (gyroConfig()->gyro_soft_notch_cutoff_1 >= gyroConfig()->gyro_soft_notch_hz_1) {
        gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    }
    if (gyroConfig()->gyro_soft_notch_cutoff_2 >= gyroConfig()->gyro_soft_notch_hz_2) {
        gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    }
#ifdef USE_DYN_LPF
    //Prevent invalid dynamic lowpass filter
    if (gyroConfig()->dyn_lpf_gyro_min_hz > gyroConfig()->dyn_lpf_gyro_max_hz) {
        gyroConfigMutable()->dyn_lpf_gyro_min_hz = 0;
    }
#endif

    if (gyroConfig()->gyro_hardware_lpf == GYRO_HARDWARE_LPF_1KHZ_SAMPLE) {
        pidConfigMutable()->pid_process_denom = 1; // When gyro set to 1khz always set pid speed 1:1 to sampling speed
        gyroConfigMutable()->gyro_sync_denom = 1;
    }

#if defined(STM32F1)
    gyroConfigMutable()->gyro_sync_denom = MAX(gyroConfig()->gyro_sync_denom, 3);
#endif

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
    if (gyroConfig()->gyro_hardware_lpf == GYRO_HARDWARE_LPF_1KHZ_SAMPLE) {
        switch (gyroMpuDetectionResult()->sensor) {
        case ICM_20649_SPI:
            samplingTime = 1.0f / 1100.0f;
            break;
        default:
            samplingTime = 0.001f;
            break;
        }
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

#ifdef USE_BLACKBOX
#ifndef USE_FLASHFS
    if (blackboxConfig()->device == 1) {  // BLACKBOX_DEVICE_FLASH (but not defined)
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
    }
#endif // USE_FLASHFS

#ifndef USE_SDCARD
    if (blackboxConfig()->device == 2) {  // BLACKBOX_DEVICE_SDCARD (but not defined)
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
    }
#endif // USE_SDCARD
#endif // USE_BLACKBOX

    if (systemConfig()->activeRateProfile >= CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = 0;
    }
    loadControlRateProfile();

    if (systemConfig()->pidProfileIndex >= PID_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = 0;
    }
    loadPidProfile();
}

bool readEEPROM(void)
{
    suspendRxPwmPpmSignal();

    // Sanity check, read flash
    bool success = loadEEPROM();

    validateAndFixConfig();

    activateConfig();

    resumeRxPwmPpmSignal();

    return success;
}

static void ValidateAndWriteConfigToEEPROM(bool setConfigured)
{
    validateAndFixConfig();

    suspendRxPwmPpmSignal();

#ifdef USE_CONFIGURATION_STATE
    if (setConfigured) {
        systemConfigMutable()->configured = true;
    }
#else
    UNUSED(setConfigured);
#endif

    writeConfigToEEPROM();

    resumeRxPwmPpmSignal();
    configIsDirty = false;
}

void writeEEPROM(void)
{
    ValidateAndWriteConfigToEEPROM(true);
}

void writeEEPROMWithFeatures(uint32_t features)
{
    featureDisableAll();
    featureEnable(features);

    ValidateAndWriteConfigToEEPROM(true);
}

void resetEEPROM(void)
{
    resetConfigs();

    ValidateAndWriteConfigToEEPROM(false);

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
    ValidateAndWriteConfigToEEPROM(true);
    readEEPROM();
    beeperConfirmationBeeps(1);
}

void setConfigDirty(void)
{
    configIsDirty = true;
}

bool isConfigDirty(void)
{
    return configIsDirty;
}

void changePidProfileFromCellCount(uint8_t cellCount)
{
    if (currentPidProfile->auto_profile_cell_count == cellCount || currentPidProfile->auto_profile_cell_count == AUTO_PROFILE_CELL_COUNT_STAY) {
        return;
    }

    unsigned profileIndex = (systemConfig()->pidProfileIndex + 1) % PID_PROFILE_COUNT;
    int matchingProfileIndex = -1;
    while (profileIndex != systemConfig()->pidProfileIndex) {
        if (pidProfiles(profileIndex)->auto_profile_cell_count == cellCount) {
            matchingProfileIndex = profileIndex;

            break;
        } else if (matchingProfileIndex < 0 && pidProfiles(profileIndex)->auto_profile_cell_count == AUTO_PROFILE_CELL_COUNT_STAY) {
            matchingProfileIndex = profileIndex;
        }

        profileIndex = (profileIndex + 1) % PID_PROFILE_COUNT;
    }

    if (matchingProfileIndex >= 0) {
        changePidProfile(matchingProfileIndex);
    }
}

void changePidProfile(uint8_t pidProfileIndex)
{
    if (pidProfileIndex < PID_PROFILE_COUNT) {
        systemConfigMutable()->pidProfileIndex = pidProfileIndex;
        loadPidProfile();

        pidInit(currentPidProfile);
    }

    beeperConfirmationBeeps(pidProfileIndex + 1);
}

bool isSystemConfigured(void)
{
#ifdef USE_CONFIGURATION_STATE
    return systemConfig()->configured;
#else
    return true;
#endif
}
