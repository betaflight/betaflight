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
#include "build/debug.h"

#include "blackbox/blackbox_io.h"

#include "common/color.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/system.h"
#include "drivers/rx_spi.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/bus_i2c.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/rangefinder.h"

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

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "navigation/navigation.h"

#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif
#ifndef DEFAULT_FEATURES
#define DEFAULT_FEATURES 0
#endif
#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 400

#if !defined(VBAT_ADC_CHANNEL)
#define VBAT_ADC_CHANNEL ADC_CHN_NONE
#endif
#if !defined(RSSI_ADC_CHANNEL)
#define RSSI_ADC_CHANNEL ADC_CHN_NONE
#endif
#if !defined(CURRENT_METER_ADC_CHANNEL)
#define CURRENT_METER_ADC_CHANNEL ADC_CHN_NONE
#endif
#if !defined(AIRSPEED_ADC_CHANNEL)
#define AIRSPEED_ADC_CHANNEL ADC_CHN_NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);

PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
    .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE
);

PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 1);

PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .current_profile_index = 0,
    .debug_mode = DEBUG_NONE,
    .i2c_speed = I2C_SPEED_400KHZ,
    .cpuUnderclock = 0,
    .accTaskFrequency = ACC_TASK_FREQUENCY_DEFAULT,
    .attitudeTaskFrequency = ATTITUDE_TASK_FREQUENCY_DEFAULT,
    .asyncMode = ASYNC_MODE_NONE,
    .throttle_tilt_compensation_strength = 0,      // 0-100, 0 - disabled
    .pwmRxInputFilteringMode = INPUT_FILTERING_DISABLED,
    .name = { 0 }
);

PG_REGISTER(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 0);

PG_REGISTER_WITH_RESET_TEMPLATE(adcChannelConfig_t, adcChannelConfig, PG_ADC_CHANNEL_CONFIG, 0);

PG_RESET_TEMPLATE(adcChannelConfig_t, adcChannelConfig,
    .adcFunctionChannel = {
        [ADC_BATTERY]   = VBAT_ADC_CHANNEL,
        [ADC_RSSI]      = RSSI_ADC_CHANNEL,
        [ADC_CURRENT]   = CURRENT_METER_ADC_CHANNEL,
        [ADC_AIRSPEED]  = AIRSPEED_ADC_CHANNEL,
    }
);

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

uint32_t getPidUpdateRate(void)
{
#ifdef ASYNC_GYRO_PROCESSING
    if (systemConfig()->asyncMode == ASYNC_MODE_NONE) {
        return getGyroUpdateRate();
    } else {
        return gyroConfig()->looptime;
    }
#else
    return gyro.targetLooptime;
#endif
}

timeDelta_t getGyroUpdateRate(void)
{
    return gyro.targetLooptime;
}
uint16_t getAccUpdateRate(void)
{
#ifdef ASYNC_GYRO_PROCESSING
    // ACC will be updated at its own rate
    if (systemConfig()->asyncMode == ASYNC_MODE_ALL) {
        return 1000000 / systemConfig()->accTaskFrequency;
    } else {
        return getPidUpdateRate();
    }
#else
    // ACC updated at same frequency in taskMainPidLoop in mw.c
    return gyro.targetLooptime;
#endif
}

#ifdef ASYNC_GYRO_PROCESSING
uint16_t getAttitudeUpdateRate(void) {
    if (systemConfig()->asyncMode == ASYNC_MODE_ALL) {
        return 1000000 / systemConfig()->attitudeTaskFrequency;
    } else {
        return getPidUpdateRate();
    }
}

uint8_t getAsyncMode(void) {
    return systemConfig()->asyncMode;
}
#endif

void validateAndFixConfig(void)
{
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
    if (pidProfile()->dterm_soft_notch_cutoff >= pidProfile()->dterm_soft_notch_hz) {
        pidProfileMutable()->dterm_soft_notch_hz = 0;
    }
#endif

#ifdef USE_ACC_NOTCH
    if (accelerometerConfig()->acc_notch_cutoff >= accelerometerConfig()->acc_notch_hz) {
        accelerometerConfigMutable()->acc_notch_hz = 0;
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
        motorConfigMutable()->motorPwmProtocol = PWM_TYPE_STANDARD; // Motor PWM rate will be handled later
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
#endif

#if defined(LED_STRIP) && (defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2))
    if (featureConfigured(FEATURE_SOFTSERIAL) && featureConfigured(FEATURE_LED_STRIP)) {
        const timerHardware_t *ledTimerHardware = timerGetByTag(IO_TAG(WS2811_PIN), TIM_USE_ANY);
        if (ledTimerHardware != NULL) {
            bool sameTimerUsed = false;

#if defined(USE_SOFTSERIAL1)
            const timerHardware_t *ss1TimerHardware = timerGetByTag(IO_TAG(SOFTSERIAL_1_RX_PIN), TIM_USE_ANY);
            if (ss1TimerHardware != NULL && ss1TimerHardware->tim == ledTimerHardware->tim) {
                sameTimerUsed = true;
            }
#endif
#if defined(USE_SOFTSERIAL2)
            const timerHardware_t *ss2TimerHardware = timerGetByTag(IO_TAG(SOFTSERIAL_2_RX_PIN), TIM_USE_ANY);
            if (ss2TimerHardware != NULL && ss2TimerHardware->tim == ledTimerHardware->tim) {
                sameTimerUsed = true;
            }
#endif
            if (sameTimerUsed) {
                // led strip needs the same timer as softserial
                featureClear(FEATURE_LED_STRIP);
            }
        }
    }
#endif

#if defined(NAZE) && defined(USE_RANGEFINDER_HCSR04)
    if (featureConfigured(FEATURE_RX_PARALLEL_PWM) && (rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) && featureConfigured(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
        featureClear(FEATURE_CURRENT_METER);
    }
#endif

#if defined(OLIMEXINO) && defined(USE_RANGEFINDER_HCSR04)
    if ((rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) && feature(FEATURE_CURRENT_METER) && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC) {
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
    #if defined(USE_RANGEFINDER_HCSR04) && defined(USE_SOFTSERIAL1)
        if ((rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) && feature(FEATURE_SOFTSERIAL)) {
            rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NONE;
        }
    #endif
#else
    #if defined(USE_RANGEFINDER_HCSR04) && defined(USE_SOFTSERIAL1) && defined(RSSI_ADC_GPIO)
        // shared pin
        if (((rangefinderConfig()->rangefinder_hardware == RANGEFINDER_HCSR04) + featureConfigured(FEATURE_SOFTSERIAL) + featureConfigured(FEATURE_RSSI_ADC)) > 1) {
           rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NONE;
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

    // If provided predefined mixer setup is disabled, fallback to default one
    if (!isMixerEnabled(mixerConfig()->mixerMode)) {
        mixerConfigMutable()->mixerMode = DEFAULT_MIXER;
    }

#if defined(NAV)
    // Ensure sane values of navConfig settings
    validateNavConfig();
#endif

    // Limitations of different protocols
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

// Default settings
void createDefaultConfig(void)
{
    // Radio
#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234");
#else
    parseRcChannels("AETR1234");
#endif

#ifdef BLACKBOX
#ifdef ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
    featureSet(FEATURE_BLACKBOX);
#endif
#endif

#if defined(TARGET_CONFIG)
    targetConfiguration();
#endif
}

void resetConfigs(void)
{
    pgResetAll(MAX_PROFILE_COUNT);
    pgActivateProfile(0);

    createDefaultConfig();

    setConfigProfile(getConfigProfile());
#ifdef LED_STRIP
    reevaluateLedConfig();
#endif
}

static void activateConfig(void)
{
    activateControlRateConfig();

    resetAdjustmentStates();

    updateUsedModeActivationConditionFlags();

    failsafeReset();

    accSetCalibrationValues();
    accInitFilters();

    imuConfigure();

    pidInit();

#ifdef NAV
    navigationUsePIDs();
#endif
}

void readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check, read flash
    if (!loadEEPROM()) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    setConfigProfile(getConfigProfile());

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

uint8_t getConfigProfile(void)
{
    return systemConfig()->current_profile_index;
}

bool setConfigProfile(uint8_t profileIndex)
{
    bool ret = true; // return true if current_profile_index has changed
    if (systemConfig()->current_profile_index == profileIndex) {
        ret =  false;
    }
    if (profileIndex >= MAX_PROFILE_COUNT) {// sanity check
        profileIndex = 0;
    }
    pgActivateProfile(profileIndex);
    systemConfigMutable()->current_profile_index = profileIndex;
    // set the control rate profile to match
    setControlRateProfile(profileIndex);
    return ret;
}

void setConfigProfileAndWriteEEPROM(uint8_t profileIndex)
{
    if (setConfigProfile(profileIndex)) {
        // profile has changed, so ensure current values saved before new profile is loaded
        writeEEPROM();
        readEEPROM();
    }
    beeperConfirmationBeeps(profileIndex + 1);
}

void beeperOffSet(uint32_t mask)
{
    beeperConfigMutable()->beeper_off_flags |= mask;
}

void beeperOffSetAll(uint8_t beeperCount)
{
    beeperConfigMutable()->beeper_off_flags = (1 << beeperCount) -1;
}

void beeperOffClear(uint32_t mask)
{
    beeperConfigMutable()->beeper_off_flags &= ~(mask);
}

void beeperOffClearAll(void)
{
    beeperConfigMutable()->beeper_off_flags = 0;
}

uint32_t getBeeperOffMask(void)
{
    return beeperConfig()->beeper_off_flags;
}

void setBeeperOffMask(uint32_t mask)
{
    beeperConfigMutable()->beeper_off_flags = mask;
}

uint32_t getPreferredBeeperOffMask(void)
{
    return beeperConfig()->preferred_beeper_off_flags;
}

void setPreferredBeeperOffMask(uint32_t mask)
{
    beeperConfigMutable()->preferred_beeper_off_flags = mask;
}
