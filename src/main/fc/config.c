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
#include "config/config_profile.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/accgyro.h"
#include "drivers/compass.h"
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
#include "drivers/serial.h"
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

#include "flight/altitudehold.h"
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
#include "io/vtx.h"

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

pidProfile_t *currentPidProfile;

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
    .enabledFeatures = DEFAULT_FEATURES | DEFAULT_RX_FEATURE | FEATURE_FAILSAFE
);

PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);

PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .pidProfileIndex = 0,
    .activeRateProfile = 0,
    .debug_mode = DEBUG_MODE,
    .task_statistics = true,
    .name = { 0 }
);

#ifdef BEEPER
PG_REGISTER(beeperConfig_t, beeperConfig, PG_BEEPER_CONFIG, 0);
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
PG_REGISTER_WITH_RESET_FN(statusLedConfig_t, statusLedConfig, PG_STATUS_LED_CONFIG, 0);
PG_REGISTER_WITH_RESET_FN(serialPinConfig_t, serialPinConfig, PG_SERIAL_PIN_CONFIG, 0);

#ifdef USE_FLASHFS
PG_REGISTER_WITH_RESET_TEMPLATE(flashConfig_t, flashConfig, PG_FLASH_CONFIG, 0);
#ifdef M25P16_CS_PIN
#define FLASH_CONFIG_CSTAG   IO_TAG(M25P16_CS_PIN)
#else
#define FLASH_CONFIG_CSTAG   IO_TAG_NONE
#endif

PG_RESET_TEMPLATE(flashConfig_t, flashConfig,
    .csTag = FLASH_CONFIG_CSTAG
);
#endif // USE_FLASH_FS

#ifdef USE_SDCARD
PG_REGISTER_WITH_RESET_TEMPLATE(sdcardConfig_t, sdcardConfig, PG_SDCARD_CONFIG, 0);
#if defined(SDCARD_DMA_CHANNEL_TX)
#define SDCARD_CONFIG_USE_DMA   true
#else
#define SDCARD_CONFIG_USE_DMA   false
#endif
PG_RESET_TEMPLATE(sdcardConfig_t, sdcardConfig,
    .useDma = SDCARD_CONFIG_USE_DMA
);
#endif

// no template required since defaults are zero
PG_REGISTER(vcdProfile_t, vcdProfile, PG_VCD_CONFIG, 0);

#ifdef SONAR
void resetSonarConfig(sonarConfig_t *sonarConfig)
{
#if defined(SONAR_TRIGGER_PIN) && defined(SONAR_ECHO_PIN)
    sonarConfig->triggerTag = IO_TAG(SONAR_TRIGGER_PIN);
    sonarConfig->echoTag = IO_TAG(SONAR_ECHO_PIN);
#else
#error Sonar not defined for target
#endif
}
#endif

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




// Default pin (NONE).
// XXX Does this mess belong here???
#ifdef USE_UART1
# if !defined(UART1_RX_PIN)
#  define UART1_RX_PIN NONE
# endif
# if !defined(UART1_TX_PIN)
#  define UART1_TX_PIN NONE
# endif
#endif

#ifdef USE_UART2
# if !defined(UART2_RX_PIN)
#  define UART2_RX_PIN NONE
# endif
# if !defined(UART2_TX_PIN)
#  define UART2_TX_PIN NONE
# endif
#endif

#ifdef USE_UART3
# if !defined(UART3_RX_PIN)
#  define UART3_RX_PIN NONE
# endif
# if !defined(UART3_TX_PIN)
#  define UART3_TX_PIN NONE
# endif
#endif

#ifdef USE_UART4
# if !defined(UART4_RX_PIN)
#  define UART4_RX_PIN NONE
# endif
# if !defined(UART4_TX_PIN)
#  define UART4_TX_PIN NONE
# endif
#endif

#ifdef USE_UART5
# if !defined(UART5_RX_PIN)
#  define UART5_RX_PIN NONE
# endif
# if !defined(UART5_TX_PIN)
#  define UART5_TX_PIN NONE
# endif
#endif

#ifdef USE_UART6
# if !defined(UART6_RX_PIN)
#  define UART6_RX_PIN NONE
# endif
# if !defined(UART6_TX_PIN)
#  define UART6_TX_PIN NONE
# endif
#endif

#ifdef USE_UART7
# if !defined(UART7_RX_PIN)
#  define UART7_RX_PIN NONE
# endif
# if !defined(UART7_TX_PIN)
#  define UART7_TX_PIN NONE
# endif
#endif

#ifdef USE_UART8
# if !defined(UART8_RX_PIN)
#  define UART8_RX_PIN NONE
# endif
# if !defined(UART8_TX_PIN)
#  define UART8_TX_PIN NONE
# endif
#endif

#ifdef USE_SOFTSERIAL1
# if !defined(SOFTSERIAL1_RX_PIN)
#  define SOFTSERIAL1_RX_PIN NONE
# endif
# if !defined(SOFTSERIAL1_TX_PIN)
#  define SOFTSERIAL1_TX_PIN NONE
# endif
#endif

#ifdef USE_SOFTSERIAL2
# if !defined(SOFTSERIAL2_RX_PIN)
#  define SOFTSERIAL2_RX_PIN NONE
# endif
# if !defined(SOFTSERIAL2_TX_PIN)
#  define SOFTSERIAL2_TX_PIN NONE
# endif
#endif

void pgResetFn_serialPinConfig(serialPinConfig_t *serialPinConfig)
{
    for (int port = 0 ; port < SERIAL_PORT_MAX_INDEX ; port++) {
        serialPinConfig->ioTagRx[port] = IO_TAG(NONE);
        serialPinConfig->ioTagTx[port] = IO_TAG(NONE);
    }

    for (int index = 0 ; index < SERIAL_PORT_COUNT ; index++) {
        switch (serialPortIdentifiers[index]) {
        case SERIAL_PORT_USART1:
#ifdef USE_UART1
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART1)] = IO_TAG(UART1_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART1)] = IO_TAG(UART1_TX_PIN);
#endif
            break;
        case SERIAL_PORT_USART2:
#ifdef USE_UART2
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART2)] = IO_TAG(UART2_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART2)] = IO_TAG(UART2_TX_PIN);
#endif
            break;
        case SERIAL_PORT_USART3:
#ifdef USE_UART3
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART3)] = IO_TAG(UART3_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART3)] = IO_TAG(UART3_TX_PIN);
#endif
            break;
        case SERIAL_PORT_UART4:
#ifdef USE_UART4
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_UART4)] = IO_TAG(UART4_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_UART4)] = IO_TAG(UART4_TX_PIN);
#endif
            break;
        case SERIAL_PORT_UART5:
#ifdef USE_UART5
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_UART5)] = IO_TAG(UART5_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_UART5)] = IO_TAG(UART5_TX_PIN);
#endif
            break;
        case SERIAL_PORT_USART6:
#ifdef USE_UART6
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART6)] = IO_TAG(UART6_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART6)] = IO_TAG(UART6_TX_PIN);
#endif
            break;
        case SERIAL_PORT_USART7:
#ifdef USE_UART7
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART7)] = IO_TAG(UART7_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART7)] = IO_TAG(UART7_TX_PIN);
#endif
            break;
        case SERIAL_PORT_USART8:
#ifdef USE_UART8
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART8)] = IO_TAG(UART8_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_USART8)] = IO_TAG(UART8_TX_PIN);
#endif
            break;
        case SERIAL_PORT_SOFTSERIAL1:
#ifdef USE_SOFTSERIAL1
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_SOFTSERIAL1)] = IO_TAG(SOFTSERIAL1_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_SOFTSERIAL1)] = IO_TAG(SOFTSERIAL1_TX_PIN);
#endif
            break;
        case SERIAL_PORT_SOFTSERIAL2:
#ifdef USE_SOFTSERIAL2
            serialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_SOFTSERIAL2)] = IO_TAG(SOFTSERIAL2_RX_PIN);
            serialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(SERIAL_PORT_SOFTSERIAL2)] = IO_TAG(SOFTSERIAL2_TX_PIN);
#endif
            break;
        case SERIAL_PORT_USB_VCP:
            break;
        case SERIAL_PORT_NONE:
            break;
        }
    }
}

#ifdef SWAP_SERIAL_PORT_0_AND_1_DEFAULTS
#define FIRST_PORT_INDEX 1
#define SECOND_PORT_INDEX 0
#else
#define FIRST_PORT_INDEX 0
#define SECOND_PORT_INDEX 1
#endif

void pgResetFn_statusLedConfig(statusLedConfig_t *statusLedConfig)
{
    for (int i = 0; i < LED_NUMBER; i++) {
        statusLedConfig->ledTags[i] = IO_TAG_NONE;
    }

#ifdef LED0
    statusLedConfig->ledTags[0] = IO_TAG(LED0);
#endif
#ifdef LED1
    statusLedConfig->ledTags[1] = IO_TAG(LED1);
#endif
#ifdef LED2
    statusLedConfig->ledTags[2] = IO_TAG(LED2);
#endif

    statusLedConfig->polarity = 0
#ifdef LED0_INVERTED
    | BIT(0)
#endif
#ifdef LED1_INVERTED
    | BIT(1)
#endif
#ifdef LED2_INVERTED
    | BIT(2)
#endif
    ;
}

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

#ifndef USE_PARAMETER_GROUPS
void createDefaultConfig(master_t *config)
{
    // Clear all configuration
    memset(config, 0, sizeof(master_t));

    uint32_t *featuresPtr = &config->featureConfig.enabledFeatures;

    intFeatureClearAll(featuresPtr);
    intFeatureSet(DEFAULT_RX_FEATURE | FEATURE_FAILSAFE , featuresPtr);

#ifdef DEFAULT_FEATURES
    intFeatureSet(DEFAULT_FEATURES, featuresPtr);
#endif

#ifndef USE_PARAMETER_GROUPS
#ifdef USE_MSP_DISPLAYPORT
    resetDisplayPortProfile(&config->displayPortProfileMsp);
#endif
#ifdef USE_MAX7456
    resetDisplayPortProfile(&config->displayPortProfileMax7456);
#endif

#ifdef USE_MAX7456
    resetMax7456Config(&config->vcdProfile);
#endif

#ifdef OSD
    osdResetConfig(&config->osdConfig);
#endif
#endif // USE_PARAMETER_GROUPS

    config->version = EEPROM_CONF_VERSION;

    // global settings
#ifndef USE_PARAMETER_GROUPS
    config->systemConfig.pidProfileIndex = 0;    // default profile
    config->systemConfig.debug_mode = DEBUG_MODE;
    config->systemConfig.task_statistics = true;
#endif


#ifndef USE_PARAMETER_GROUPS
    config->imuConfig.dcm_kp = 2500;                // 1.0 * 10000
    config->imuConfig.dcm_ki = 0;                   // 0.003 * 10000
    config->imuConfig.small_angle = 25;
    config->imuConfig.accDeadband.xy = 40;
    config->imuConfig.accDeadband.z = 40;
    config->imuConfig.acc_unarmedcal = 1;
#endif
#ifndef USE_PARAMETER_GROUPS
    config->gyroConfig.gyro_lpf = GYRO_LPF_256HZ;    // 256HZ default
#ifdef STM32F10X
    config->gyroConfig.gyro_sync_denom = 8;
    config->pidConfig.pid_process_denom = 1;
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500)  || defined(USE_GYRO_SPI_ICM20689)
    config->gyroConfig.gyro_sync_denom = 1;
    config->pidConfig.pid_process_denom = 4;
#else
    config->gyroConfig.gyro_sync_denom = 4;
    config->pidConfig.pid_process_denom = 2;
#endif
    config->gyroConfig.gyro_soft_lpf_type = FILTER_PT1;
    config->gyroConfig.gyro_soft_lpf_hz = 90;
    config->gyroConfig.gyro_soft_notch_hz_1 = 400;
    config->gyroConfig.gyro_soft_notch_cutoff_1 = 300;
    config->gyroConfig.gyro_soft_notch_hz_2 = 200;
    config->gyroConfig.gyro_soft_notch_cutoff_2 = 100;
    config->gyroConfig.gyro_align = ALIGN_DEFAULT;
    config->gyroConfig.gyroMovementCalibrationThreshold = 48;
#endif

#ifndef USE_PARAMETER_GROUPS
    resetCompassConfig(&config->compassConfig);
#endif

#ifndef USE_PARAMETER_GROUPS
    resetAccelerometerTrims(&config->accelerometerConfig.accZero);
    config->accelerometerConfig.acc_align = ALIGN_DEFAULT;
    config->accelerometerConfig.acc_hardware = ACC_DEFAULT;     // default/autodetect
    resetRollAndPitchTrims(&config->accelerometerConfig.accelerometerTrims);
    config->accelerometerConfig.acc_lpf_hz = 10.0f;
#endif
#ifndef USE_PARAMETER_GROUPS
    config->boardAlignment.rollDegrees = 0;
    config->boardAlignment.pitchDegrees = 0;
    config->boardAlignment.yawDegrees = 0;
#endif
    config->rcControlsConfig.yaw_control_direction = 1;

    // xxx_hardware: 0:default/autodetect, 1: disable
    config->compassConfig.mag_hardware = 1;

    config->barometerConfig.baro_hardware = 1;

#ifndef USE_PARAMETER_GROUPS
    resetBatteryConfig(&config->batteryConfig);

#if defined(USE_PWM) || defined(USE_PPM)
    resetPpmConfig(&config->ppmConfig);
    resetPwmConfig(&config->pwmConfig);
#endif
#ifdef USE_PWM
    config->pwmConfig.inputFilteringMode = INPUT_FILTERING_DISABLED;
#endif

#ifdef TELEMETRY
    resetTelemetryConfig(&config->telemetryConfig);
#endif
#endif

#ifndef USE_PARAMETER_GROUPS
#ifdef USE_ADC
    resetAdcConfig(&config->adcConfig);
#endif

#ifdef BEEPER
    resetBeeperDevConfig(&config->beeperDevConfig);
#endif
#endif

#ifdef SONAR
    resetSonarConfig(&config->sonarConfig);
#endif

#ifndef USE_PARAMETER_GROUPS
#ifdef USE_SDCARD
    intFeatureSet(FEATURE_SDCARD, featuresPtr);
    resetsdcardConfig(&config->sdcardConfig);
#endif

#ifdef SERIALRX_PROVIDER
    config->rxConfig.serialrx_provider = SERIALRX_PROVIDER;
#else
    config->rxConfig.serialrx_provider = 0;
#endif
    config->rxConfig.halfDuplex = 0;
    config->rxConfig.rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL;
    config->rxConfig.sbus_inversion = 1;
    config->rxConfig.spektrum_sat_bind = 0;
    config->rxConfig.spektrum_sat_bind_autoreset = 1;
    config->rxConfig.midrc = 1500;
    config->rxConfig.mincheck = 1100;
    config->rxConfig.maxcheck = 1900;
    config->rxConfig.rx_min_usec = 885;          // any of first 4 channels below this value will trigger rx loss detection
    config->rxConfig.rx_max_usec = 2115;         // any of first 4 channels above this value will trigger rx loss detection

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfig_t *channelFailsafeConfig = &config->rxConfig.failsafe_channel_configurations[i];
        channelFailsafeConfig->mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        channelFailsafeConfig->step = (i == THROTTLE) ? CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.rx_min_usec) : CHANNEL_VALUE_TO_RXFAIL_STEP(config->rxConfig.midrc);
    }

    config->rxConfig.rssi_channel = 0;
    config->rxConfig.rssi_scale = RSSI_SCALE_DEFAULT;
    config->rxConfig.rssi_invert = 0;
    config->rxConfig.rcInterpolation = RC_SMOOTHING_AUTO;
    config->rxConfig.rcInterpolationChannels = 0;
    config->rxConfig.rcInterpolationInterval = 19;
    config->rxConfig.fpvCamAngleDegrees = 0;
    config->rxConfig.max_aux_channel = DEFAULT_AUX_CHANNEL_COUNT;
    config->rxConfig.airModeActivateThreshold = 1350;

    resetAllRxChannelRangeConfigurations(config->rxConfig.channelRanges);
#endif

#ifndef USE_PARAMETER_GROUPS
    config->armingConfig.gyro_cal_on_first_arm = 0;  // TODO - Cleanup retarded arm support
    config->armingConfig.disarm_kill_switch = 1;
    config->armingConfig.auto_disarm_delay = 5;

    config->airplaneConfig.fixedwing_althold_dir = 1;

    // Motor/ESC/Servo
    resetMixerConfig(&config->mixerConfig);
    resetMotorConfig(&config->motorConfig);
#ifdef USE_SERVOS
    resetServoConfig(&config->servoConfig);
#endif
    resetFlight3DConfig(&config->flight3DConfig);

#ifdef LED_STRIP
    resetLedStripConfig(&config->ledStripConfig);
#endif

#ifdef GPS
    // gps/nav stuff
    config->gpsConfig.provider = GPS_NMEA;
    config->gpsConfig.sbasMode = SBAS_AUTO;
    config->gpsConfig.autoConfig = GPS_AUTOCONFIG_ON;
    config->gpsConfig.autoBaud = GPS_AUTOBAUD_OFF;
#endif

    resetSerialPinConfig(&config->serialPinConfig);

    resetSerialConfig(&config->serialConfig);

    for (int ii = 0; ii < MAX_PROFILE_COUNT; ++ii) {
        resetProfile(&config->profile[ii]);
    }
    for (int ii = 0; ii < CONTROL_RATE_PROFILE_COUNT; ++ii) {
        resetControlRateProfile(&config->controlRateProfile[ii]);
    }
#endif

    config->compassConfig.mag_declination = 0;

#ifdef BARO
#ifndef USE_PARAMETER_GROUPS
    resetBarometerConfig(&config->barometerConfig);
#endif
#endif

    // Radio
#ifdef RX_CHANNELS_TAER
    parseRcChannels("TAER1234", &config->rxConfig);
#else
    parseRcChannels("AETR1234", &config->rxConfig);
#endif

#ifndef USE_PARAMETER_GROUPS
    resetRcControlsConfig(&config->rcControlsConfig);

    config->throttleCorrectionConfig.throttle_correction_value = 0;      // could 10 with althold or 40 for fpv
    config->throttleCorrectionConfig.throttle_correction_angle = 800;    // could be 80.0 deg with atlhold or 45.0 for fpv

    // Failsafe Variables
    config->failsafeConfig.failsafe_delay = 10;                            // 1sec
    config->failsafeConfig.failsafe_off_delay = 10;                        // 1sec
    config->failsafeConfig.failsafe_throttle = 1000;                       // default throttle off.
    config->failsafeConfig.failsafe_kill_switch = 0;                       // default failsafe switch action is identical to rc link loss
    config->failsafeConfig.failsafe_throttle_low_delay = 100;              // default throttle low delay for "just disarm" on failsafe condition
    config->failsafeConfig.failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT;// default full failsafe procedure is 0: auto-landing
#endif

#ifdef USE_SERVOS
#ifndef USE_PARAMETER_GROUPS
    // servos
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        config->servoProfile.servoConf[i].min = DEFAULT_SERVO_MIN;
        config->servoProfile.servoConf[i].max = DEFAULT_SERVO_MAX;
        config->servoProfile.servoConf[i].middle = DEFAULT_SERVO_MIDDLE;
        config->servoProfile.servoConf[i].rate = 100;
        config->servoProfile.servoConf[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }

    // gimbal
    config->gimbalConfig.mode = GIMBAL_MODE_NORMAL;
#endif

    // Channel forwarding;
    config->servoConfig.channelForwardingStartChannel = AUX1;
#endif

#ifndef USE_PARAMETER_GROUPS
#ifdef GPS
    resetNavigationConfig(&config->navigationConfig);
#endif
#endif

    // custom mixer. clear by defaults.
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        config->customMotorMixer[i].throttle = 0.0f;
    }

#ifndef USE_PARAMETER_GROUPS
#ifdef VTX
    config->vtxConfig.vtx_band = 4;    //Fatshark/Airwaves
    config->vtxConfig.vtx_channel = 1; //CH1
    config->vtxConfig.vtx_mode = 0;    //CH+BAND mode
    config->vtxConfig.vtx_mhz = 5740;  //F0
#endif
#endif

#ifdef TRANSPONDER
    static const uint8_t defaultTransponderData[6] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC }; // Note, this is NOT a valid transponder code, it's just for testing production hardware

    memcpy(config->transponderConfig.data, &defaultTransponderData, sizeof(defaultTransponderData));
#endif

#ifndef USE_PARAMETER_GROUPS
#ifdef BLACKBOX
#if defined(ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT)
    intFeatureSet(FEATURE_BLACKBOX, featuresPtr);
    config->blackboxConfig.device = BLACKBOX_DEVICE_FLASH;
#elif defined(ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT)
    intFeatureSet(FEATURE_BLACKBOX, featuresPtr);
    config->blackboxConfig.device = BLACKBOX_DEVICE_SDCARD;
#else
    config->blackboxConfig.device = BLACKBOX_DEVICE_SERIAL;
#endif
    config->blackboxConfig.rate_num = 1;
    config->blackboxConfig.rate_denom = 1;
    config->blackboxConfig.on_motor_test = 0; // default off
#endif // BLACKBOX
#endif

#ifdef SERIALRX_UART
    if (featureConfigured(FEATURE_RX_SERIAL)) {
        int serialIndex = findSerialPortIndexByIdentifier(SERIALRX_UART);
        if (serialIndex >= 0) {
            config->serialConfig.portConfigs[serialIndex].functionMask = FUNCTION_RX_SERIAL;
        }
    }
#endif

#ifndef USE_PARAMETER_GROUPS
#ifdef USE_FLASHFS
    resetFlashConfig(&config->flashConfig);
#endif

    resetStatusLedConfig(&config->statusLedConfig);
#endif

#if defined(TARGET_CONFIG)
    targetConfiguration(config);
#endif
}
#endif

void resetConfigs(void)
{
#ifndef USE_PARAMETER_GROUPS
    createDefaultConfig(&masterConfig);
#endif
    pgResetAll(MAX_PROFILE_COUNT);
    pgActivateProfile(0);

    setPidProfile(0);
    setControlRateProfile(0);

#ifdef LED_STRIP
    reevaluateLedConfig();
#endif
}

void activateConfig(void)
{
    generateThrottleCurve();

    resetAdjustmentStates();

    useRcControlsConfig(modeActivationConditions(0), currentPidProfile);
    useAdjustmentConfig(currentPidProfile);

#ifdef GPS
    gpsUsePIDs(currentPidProfile);
#endif

    failsafeReset();
    setAccelerationTrims(&accelerometerConfigMutable()->accZero);
    setAccelerationFilter(accelerometerConfig()->acc_lpf_hz);

    imuConfigure(throttleCorrectionConfig()->throttle_correction_angle);

    configureAltitudeHold(currentPidProfile);
}

void validateAndFixConfig(void)
{
    if((motorConfig()->dev.motorPwmProtocol == PWM_TYPE_BRUSHED) && (motorConfig()->mincommand < 1000)){
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

    if (featureConfigured(FEATURE_RX_SPI)) {
        featureClear(FEATURE_RX_SERIAL | FEATURE_RX_PARALLEL_PWM | FEATURE_RX_PPM | FEATURE_RX_MSP);
    }

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

#ifndef USE_PARAMETER_GROUPS
    serialConfig_t *serialConfig = &masterConfig.serialConfig;
    if (!isSerialConfigValid(serialConfig)) {
        resetSerialConfig(serialConfig);
    }
#else
    if (!isSerialConfigValid(serialConfig())) {
        pgResetFn_serialConfig(serialConfigMutable());
    }
#endif

    // Prevent invalid notch cutoff
    if (currentPidProfile->dterm_notch_cutoff >= currentPidProfile->dterm_notch_hz) {
        currentPidProfile->dterm_notch_hz = 0;
    }

    validateAndFixGyroConfig();

#if defined(TARGET_VALIDATECONFIG)
    targetValidateConfiguration();
#endif
}

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

#if !defined(GYRO_USES_SPI) || !defined(USE_MPU_DATA_READY_SIGNAL)
    gyroConfigMutable()->gyro_isr_update = false;
#endif

    // check for looptime restrictions based on motor protocol. Motor times have safety margin
    const float pidLooptime = samplingTime * gyroConfig()->gyro_sync_denom * pidConfig()->pid_process_denom;
    float motorUpdateRestriction;
    switch(motorConfig()->dev.motorPwmProtocol) {
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

        if(motorConfig()->dev.motorPwmRate > maxEscRate)
            motorConfigMutable()->dev.motorPwmRate = maxEscRate;
    }
}

void readEEPROM(void)
{
    suspendRxSignal();

    // Sanity check, read flash
    if (!loadEEPROM()) {
        failureMode(FAILURE_INVALID_EEPROM_CONTENTS);
    }

    if (systemConfig()->activeRateProfile >= CONTROL_RATE_PROFILE_COUNT) {// sanity check
        systemConfigMutable()->activeRateProfile = 0;
    }
    setControlRateProfile(systemConfig()->activeRateProfile);

    if (systemConfig()->pidProfileIndex >= MAX_PROFILE_COUNT) {// sanity check
        systemConfigMutable()->pidProfileIndex = 0;
    }
    setPidProfile(systemConfig()->pidProfileIndex);

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

void changePidProfile(uint8_t pidProfileIndex)
{
    if (pidProfileIndex >= MAX_PROFILE_COUNT) {
        pidProfileIndex = MAX_PROFILE_COUNT - 1;
    }
    systemConfigMutable()->pidProfileIndex = pidProfileIndex;
    currentPidProfile = pidProfilesMutable(pidProfileIndex);
    beeperConfirmationBeeps(pidProfileIndex + 1);
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
