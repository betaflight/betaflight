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

#include "build/build_config.h"
#include "build/debug.h"

#include "blackbox/blackbox_io.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_spi.h"
#include "drivers/compass/compass.h"
#include "drivers/inverter.h"
#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/light_ws2811strip.h"
#include "drivers/max7456.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/pwm_output.h"
#include "drivers/rx_pwm.h"
#include "drivers/rx_spi.h"
#include "drivers/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sound_beeper.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/vcd.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/fc_rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/altitude.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/motors.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/servos.h"
#include "io/vtx_control.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/telemetry.h"

#ifndef USE_OSD_SLAVE
pidProfile_t *currentPidProfile;
#endif

#ifndef DEFAULT_FEATURES
#define DEFAULT_FEATURES 0
#endif
#ifndef DEFAULT_RX_FEATURE
#define DEFAULT_RX_FEATURE FEATURE_RX_PARALLEL_PWM
#endif
#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(featureConfig_t, featureConfig, PG_FEATURE_CONFIG, 0);

PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
    .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE
);

PG_REGISTER_WITH_RESET_TEMPLATE(pilotConfig_t, pilotConfig, PG_PILOT_CONFIG, 0);

PG_RESET_TEMPLATE(pilotConfig_t, pilotConfig,
    .name = { 0 }
);

PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);

#ifndef USE_OSD_SLAVE
#if defined(STM32F4) && !defined(DISABLE_OVERCLOCK)
PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .pidProfileIndex = 0,
    .activeRateProfile = 0,
    .debug_mode = DEBUG_MODE,
    .task_statistics = true,
    .cpu_overclock = false,
    .powerOnArmingGraceTime = 5,
    .boardIdentifier = TARGET_BOARD_IDENTIFIER
);
#else
PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .pidProfileIndex = 0,
    .activeRateProfile = 0,
    .debug_mode = DEBUG_MODE,
    .task_statistics = true,
    .boardIdentifier = TARGET_BOARD_IDENTIFIER
);
#endif
#endif

#ifdef USE_OSD_SLAVE
PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .debug_mode = DEBUG_MODE,
    .task_statistics = true,
    .boardIdentifier = TARGET_BOARD_IDENTIFIER
);
#endif

#ifdef BEEPER
PG_REGISTER_WITH_RESET_TEMPLATE(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 0);
PG_RESET_TEMPLATE(beeperConfig_t, beeperConfig,
    .dshotForward = true
);
#endif
#ifdef USE_ADC
PG_REGISTER_WITH_RESET_FN(adcConfig_t, adcConfig, PG_ADC_CONFIG, 0);
#endif
#ifdef USE_PWM
PG_REGISTER_WITH_RESET_FN(pwmConfig_t, pwmConfig, PG_PWM_CONFIG, 0);
#endif
#ifdef USE_PPM
PG_REGISTER_WITH_RESET_FN(ppmConfig_t, ppmConfig, PG_PPM_CONFIG, 0);
#endif

#ifdef USE_FLASHFS
PG_REGISTER_WITH_RESET_FN(flashConfig_t, flashConfig, PG_FLASH_CONFIG, 0);

void pgResetFn_flashConfig(flashConfig_t *flashConfig)
{
#ifdef M25P16_CS_PIN
    flashConfig->csTag = IO_TAG(M25P16_CS_PIN);
#else
    flashConfig->csTag = IO_TAG_NONE;
#endif
    flashConfig->spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance(M25P16_SPI_INSTANCE));
}
#endif // USE_FLASH_FS

#ifdef USE_SDCARD
PG_REGISTER_WITH_RESET_TEMPLATE(sdcardConfig_t, sdcardConfig, PG_SDCARD_CONFIG, 0);

PG_RESET_TEMPLATE(sdcardConfig_t, sdcardConfig,
    .useDma = false
);
#endif // USE_SDCARD

// no template required since defaults are zero
PG_REGISTER(vcdProfile_t, vcdProfile, PG_VCD_CONFIG, 0);

#ifdef USE_ADC
void pgResetFn_adcConfig(adcConfig_t *adcConfig)
{
#ifdef VBAT_ADC_PIN
    adcConfig->vbat.enabled = true;
    adcConfig->vbat.ioTag = IO_TAG(VBAT_ADC_PIN);
#endif

#ifdef EXTERNAL1_ADC_PIN
    adcConfig->external1.enabled = true;
    adcConfig->external1.ioTag = IO_TAG(EXTERNAL1_ADC_PIN);
#endif

#ifdef CURRENT_METER_ADC_PIN
    adcConfig->current.enabled = true;
    adcConfig->current.ioTag = IO_TAG(CURRENT_METER_ADC_PIN);
#endif

#ifdef RSSI_ADC_PIN
    adcConfig->rssi.enabled = true;
    adcConfig->rssi.ioTag = IO_TAG(RSSI_ADC_PIN);
#endif

}
#endif // USE_ADC


#if defined(USE_PWM) || defined(USE_PPM)
void pgResetFn_ppmConfig(ppmConfig_t *ppmConfig)
{
#ifdef PPM_PIN
    ppmConfig->ioTag = IO_TAG(PPM_PIN);
#else
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_PPM) {
            ppmConfig->ioTag = timerHardware[i].tag;
            return;
        }
    }

    ppmConfig->ioTag = IO_TAG_NONE;
#endif
}

void pgResetFn_pwmConfig(pwmConfig_t *pwmConfig)
{
    pwmConfig->inputFilteringMode = INPUT_FILTERING_DISABLED;
    int inputIndex = 0;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && inputIndex < PWM_INPUT_PORT_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_PWM) {
            pwmConfig->ioTags[inputIndex] = timerHardware[i].tag;
            inputIndex++;
        }
    }
}
#endif

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
        pidInit(currentPidProfile); // re-initialise pid controller to re-initialise filters and config
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
#endif

void resetConfigs(void)
{
    pgResetAll();

#if defined(TARGET_CONFIG)
    targetConfiguration();
#endif

#ifndef USE_OSD_SLAVE
    setPidProfile(0);
    setControlRateProfile(0);
#endif

#ifdef LED_STRIP
    reevaluateLedConfig();
#endif
}

void activateConfig(void)
{
#ifndef USE_OSD_SLAVE
    generateThrottleCurve();

    resetAdjustmentStates();

    useRcControlsConfig(currentPidProfile);
    useAdjustmentConfig(currentPidProfile);

#ifdef GPS
    gpsUsePIDs(currentPidProfile);
#endif

    failsafeReset();
    setAccelerationTrims(&accelerometerConfigMutable()->accZero);
    setAccelerationFilter(accelerometerConfig()->acc_lpf_hz);

    imuConfigure(throttleCorrectionConfig()->throttle_correction_angle);
#endif
}

void validateAndFixConfig(void)
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
    if ((motorConfig()->dev.motorPwmProtocol == PWM_TYPE_BRUSHED) && (motorConfig()->mincommand < 1000)) {
        motorConfigMutable()->mincommand = 1000;
    }

    if ((motorConfig()->dev.motorPwmProtocol == PWM_TYPE_STANDARD) && (motorConfig()->dev.motorPwmRate > BRUSHLESS_MOTORS_PWM_RATE)) {
        motorConfigMutable()->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
    }

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
#endif

    if (featureConfigured(FEATURE_RX_PARALLEL_PWM)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_MSP | FEATURE_RX_PPM | FEATURE_RX_SPI);
#if defined(STM32F10X)
        // rssi adc needs the same ports
        featureClear(FEATURE_RSSI_ADC);
        // current meter needs the same ports
        if (batteryConfig()->currentMeterSource == CURRENT_METER_ADC) {
            batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;
        }
#endif
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
#endif
    }
#endif

    // Prevent invalid notch cutoff
    if (currentPidProfile->dterm_notch_cutoff >= currentPidProfile->dterm_notch_hz) {
        currentPidProfile->dterm_notch_hz = 0;
    }

    validateAndFixGyroConfig();
#endif

    if (!isSerialConfigValid(serialConfig())) {
        pgResetFn_serialConfig(serialConfigMutable());
    }

// clear features that are not supported.
// I have kept them all here in one place, some could be moved to sections of code above.

#ifndef USE_PPM
    featureClear(FEATURE_RX_PPM);
#endif

#ifndef SERIAL_RX
    featureClear(FEATURE_RX_SERIAL);
#endif

#if !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2)
    featureClear(FEATURE_SOFTSERIAL);
#endif

#ifndef GPS
    featureClear(FEATURE_GPS);
#endif

#ifndef SONAR
    featureClear(FEATURE_SONAR);
#endif

#ifndef TELEMETRY
    featureClear(FEATURE_TELEMETRY);
#endif

#ifndef USE_PWM
    featureClear(FEATURE_RX_PARALLEL_PWM);
#endif

#ifndef USE_RX_MSP
    featureClear(FEATURE_RX_MSP);
#endif

#ifndef LED_STRIP
    featureClear(FEATURE_LED_STRIP);
#endif

#ifndef USE_DASHBOARD
    featureClear(FEATURE_DASHBOARD);
#endif

#ifndef OSD
    featureClear(FEATURE_OSD);
#endif

#ifndef USE_SERVOS
    featureClear(FEATURE_SERVO_TILT | FEATURE_CHANNEL_FORWARDING);
#endif

#ifndef TRANSPONDER
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
    // Prevent invalid notch cutoff
    if (gyroConfig()->gyro_soft_notch_cutoff_1 >= gyroConfig()->gyro_soft_notch_hz_1) {
        gyroConfigMutable()->gyro_soft_notch_hz_1 = 0;
    }
    if (gyroConfig()->gyro_soft_notch_cutoff_2 >= gyroConfig()->gyro_soft_notch_hz_2) {
        gyroConfigMutable()->gyro_soft_notch_hz_2 = 0;
    }

    float samplingTime = 0.000125f;

    if (gyroConfig()->gyro_lpf != GYRO_LPF_256HZ && gyroConfig()->gyro_lpf != GYRO_LPF_NONE) {
        pidConfigMutable()->pid_process_denom = 1; // When gyro set to 1khz always set pid speed 1:1 to sampling speed
        gyroConfigMutable()->gyro_sync_denom = 1;
        gyroConfigMutable()->gyro_use_32khz = false;
        samplingTime = 0.001f;
    }

    if (gyroConfig()->gyro_use_32khz) {
        samplingTime = 0.00003125;
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

    // check for looptime restrictions based on motor protocol. Motor times have safety margin
    const float pidLooptime = samplingTime * gyroConfig()->gyro_sync_denom * pidConfig()->pid_process_denom;
    float motorUpdateRestriction;
    switch (motorConfig()->dev.motorPwmProtocol) {
        case (PWM_TYPE_STANDARD):
            motorUpdateRestriction = 1.0f/BRUSHLESS_MOTORS_PWM_RATE;
            break;
        case (PWM_TYPE_ONESHOT125):
            motorUpdateRestriction = 0.0005f;
            break;
        case (PWM_TYPE_ONESHOT42):
            motorUpdateRestriction = 0.0001f;
            break;
#ifdef USE_DSHOT
        case (PWM_TYPE_DSHOT150):
            motorUpdateRestriction = 0.000250f;
            break;
        case (PWM_TYPE_DSHOT300):
            motorUpdateRestriction = 0.0001f;
            break;
#endif
        default:
            motorUpdateRestriction = 0.00003125f;
    }

    if (pidLooptime < motorUpdateRestriction) {
        const uint8_t maxPidProcessDenom = constrain(motorUpdateRestriction / (samplingTime * gyroConfig()->gyro_sync_denom), 1, MAX_PID_PROCESS_DENOM);
        pidConfigMutable()->pid_process_denom = MIN(pidConfigMutable()->pid_process_denom, maxPidProcessDenom);
    }

    // Prevent overriding the max rate of motors
    if (motorConfig()->dev.useUnsyncedPwm && (motorConfig()->dev.motorPwmProtocol <= PWM_TYPE_BRUSHED) && motorConfig()->dev.motorPwmProtocol != PWM_TYPE_STANDARD) {
        uint32_t maxEscRate = lrintf(1.0f / motorUpdateRestriction);

        if (motorConfig()->dev.motorPwmRate > maxEscRate)
            motorConfigMutable()->dev.motorPwmRate = maxEscRate;
    }
}
#endif

void readEEPROM(void)
{
#ifndef USE_OSD_SLAVE
    suspendRxSignal();
#endif

    // Sanity check, read flash
    if (!loadEEPROM()) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }
#ifndef USE_OSD_SLAVE
    if (systemConfig()->activeRateProfile >= CONTROL_RATE_PROFILE_COUNT) {// sanity check
        systemConfigMutable()->activeRateProfile = 0;
    }
    setControlRateProfile(systemConfig()->activeRateProfile);

    if (systemConfig()->pidProfileIndex >= MAX_PROFILE_COUNT) {// sanity check
        systemConfigMutable()->pidProfileIndex = 0;
    }
    setPidProfile(systemConfig()->pidProfileIndex);
#endif

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
