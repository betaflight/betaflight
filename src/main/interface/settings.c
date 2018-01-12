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

#include "platform.h"

#include "build/debug.h"

#include "blackbox/blackbox.h"

#include "cms/cms.h"

#include "common/utils.h"

#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/camera_control.h"
#include "drivers/light_led.h"
#include "drivers/vtx_common.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_core.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"

#include "flight/altitude.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "interface/settings.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/max7456.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx_pwm.h"
#include "pg/sdcard.h"
#include "pg/vcd.h"

#include "rx/rx.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/spektrum.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#include "telemetry/frsky_hub.h"
#include "telemetry/telemetry.h"

// Sensor names (used in lookup tables for *_hardware settings and in status command output)
// sync with accelerationSensor_e
const char * const lookupTableAccHardware[] = {
    "AUTO", "NONE", "ADXL345", "MPU6050", "MMA8452", "BMA280", "LSM303DLHC",
    "MPU6000", "MPU6500", "MPU9250", "ICM20601", "ICM20602", "ICM20608", "ICM20649", "ICM20689",
    "BMI160", "FAKE"
};

// sync with gyroSensor_e
const char * const lookupTableGyroHardware[] = {
    "AUTO", "NONE", "MPU6050", "L3G4200D", "MPU3050", "L3GD20",
    "MPU6000", "MPU6500", "MPU9250", "ICM20601", "ICM20602", "ICM20608G", "ICM20649", "ICM20689",
    "BMI160", "FAKE"
};

#if defined(USE_SENSOR_NAMES) || defined(USE_BARO)
// sync with baroSensor_e
const char * const lookupTableBaroHardware[] = {
    "AUTO", "NONE", "BMP085", "MS5611", "BMP280", "LPS"
};
#endif
#if defined(USE_SENSOR_NAMES) || defined(USE_MAG)
// sync with magSensor_e
const char * const lookupTableMagHardware[] = {
    "AUTO", "NONE", "HMC5883", "AK8975", "AK8963"
};
#endif
#if defined(USE_SENSOR_NAMES) || defined(USE_RANGEFINDER)
const char * const lookupTableRangefinderHardware[] = {
    "NONE", "HCSR04", "TFMINI", "TF02"
};
#endif

static const char * const lookupTableOffOn[] = {
    "OFF", "ON"
};

static const char * const lookupTableCrashRecovery[] = {
    "OFF", "ON" ,"BEEP"
};

static const char * const lookupTableUnit[] = {
    "IMPERIAL", "METRIC"
};

static const char * const lookupTableAlignment[] = {
    "DEFAULT",
    "CW0",
    "CW90",
    "CW180",
    "CW270",
    "CW0FLIP",
    "CW90FLIP",
    "CW180FLIP",
    "CW270FLIP"
};

#ifdef USE_GPS
static const char * const lookupTableGPSProvider[] = {
    "NMEA", "UBLOX"
};

static const char * const lookupTableGPSSBASMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN"
};
#endif

static const char * const lookupTableCurrentSensor[] = {
    "NONE", "ADC", "VIRTUAL", "ESC", "MSP"
};

static const char * const lookupTableBatterySensor[] = {
    "NONE", "ADC", "ESC"
};

#ifdef USE_SERVOS
static const char * const lookupTableGimbalMode[] = {
    "NORMAL", "MIXTILT"
};
#endif

#ifdef USE_BLACKBOX
static const char * const lookupTableBlackboxDevice[] = {
    "NONE", "SPIFLASH", "SDCARD", "SERIAL"
};

static const char * const lookupTableBlackboxMode[] = {
    "NORMAL", "MOTOR_TEST", "ALWAYS"
};
#endif

#ifdef USE_SERIAL_RX
static const char * const lookupTableSerialRX[] = {
    "SPEK1024",
    "SPEK2048",
    "SBUS",
    "SUMD",
    "SUMH",
    "XB-B",
    "XB-B-RJ01",
    "IBUS",
    "JETIEXBUS",
    "CRSF",
    "SRXL",
    "CUSTOM",
    "FPORT",
};
#endif

#ifdef USE_RX_SPI
// sync with rx_spi_protocol_e
static const char * const lookupTableRxSpi[] = {
    "V202_250K",
    "V202_1M",
    "SYMA_X",
    "SYMA_X5C",
    "CX10",
    "CX10A",
    "H8_3D",
    "INAV",
    "FRSKY_D",
    "FRSKY_X",
    "FLYSKY",
    "FLYSKY_2A"
};
#endif

static const char * const lookupTableGyroLpf[] = {
    "OFF",
    "188HZ",
    "98HZ",
    "42HZ",
    "20HZ",
    "10HZ",
    "5HZ",
    "EXPERIMENTAL"
};

#ifdef USE_OSD
static const char * const lookupTableOsdType[] = {
    "AUTO",
    "PAL",
    "NTSC"
};
#endif

#ifdef USE_CAMERA_CONTROL
static const char * const lookupTableCameraControlMode[] = {
    "HARDWARE_PWM",
    "SOFTWARE_PWM",
    "DAC"
};
#endif

static const char * const lookupTableSuperExpoYaw[] = {
    "OFF", "ON", "ALWAYS"
};

static const char * const lookupTablePwmProtocol[] = {
    "OFF", "ONESHOT125", "ONESHOT42", "MULTISHOT", "BRUSHED",
#ifdef USE_DSHOT
    "DSHOT150", "DSHOT300", "DSHOT600", "DSHOT1200", "PROSHOT1000"
#endif
};

static const char * const lookupTableRcInterpolation[] = {
    "OFF", "PRESET", "AUTO", "MANUAL"
};

static const char * const lookupTableRcInterpolationChannels[] = {
    "RP", "RPY", "RPYT"
};

static const char * const lookupTableLowpassType[] = {
    "PT1", "BIQUAD", "FIR"
};

static const char * const lookupTableFailsafe[] = {
    "AUTO-LAND", "DROP"
};

static const char * const lookupTableBusType[] = {
    "NONE", "I2C", "SPI", "SLAVE"
};

#ifdef USE_MAX7456
static const char * const lookupTableMax7456Clock[] = {
    "HALF", "DEFAULT", "FULL"
};
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
static const char * const lookupTableGyroOverflowCheck[] = {
    "OFF", "YAW", "ALL"
};
#endif

const lookupTableEntry_t lookupTables[] = {
    { lookupTableOffOn, sizeof(lookupTableOffOn) / sizeof(char *) },
    { lookupTableUnit, sizeof(lookupTableUnit) / sizeof(char *) },
    { lookupTableAlignment, sizeof(lookupTableAlignment) / sizeof(char *) },
#ifdef USE_GPS
    { lookupTableGPSProvider, sizeof(lookupTableGPSProvider) / sizeof(char *) },
    { lookupTableGPSSBASMode, sizeof(lookupTableGPSSBASMode) / sizeof(char *) },
#endif
#ifdef USE_BLACKBOX
    { lookupTableBlackboxDevice, sizeof(lookupTableBlackboxDevice) / sizeof(char *) },
    { lookupTableBlackboxMode, sizeof(lookupTableBlackboxMode) / sizeof(char *) },
#endif
    { lookupTableCurrentSensor, sizeof(lookupTableCurrentSensor) / sizeof(char *) },
    { lookupTableBatterySensor, sizeof(lookupTableBatterySensor) / sizeof(char *) },
#ifdef USE_SERVOS
    { lookupTableGimbalMode, sizeof(lookupTableGimbalMode) / sizeof(char *) },
#endif
#ifdef USE_SERIAL_RX
    { lookupTableSerialRX, sizeof(lookupTableSerialRX) / sizeof(char *) },
#endif
#ifdef USE_RX_SPI
    { lookupTableRxSpi, sizeof(lookupTableRxSpi) / sizeof(char *) },
#endif
    { lookupTableGyroLpf, sizeof(lookupTableGyroLpf) / sizeof(char *) },
    { lookupTableGyroHardware, sizeof(lookupTableGyroHardware) / sizeof(char *) },
    { lookupTableAccHardware, sizeof(lookupTableAccHardware) / sizeof(char *) },
#ifdef USE_BARO
    { lookupTableBaroHardware, sizeof(lookupTableBaroHardware) / sizeof(char *) },
#endif
#ifdef USE_MAG
    { lookupTableMagHardware, sizeof(lookupTableMagHardware) / sizeof(char *) },
#endif
    { debugModeNames, sizeof(debugModeNames) / sizeof(char *) },
    { lookupTableSuperExpoYaw, sizeof(lookupTableSuperExpoYaw) / sizeof(char *) },
    { lookupTablePwmProtocol, sizeof(lookupTablePwmProtocol) / sizeof(char *) },
    { lookupTableRcInterpolation, sizeof(lookupTableRcInterpolation) / sizeof(char *) },
    { lookupTableRcInterpolationChannels, sizeof(lookupTableRcInterpolationChannels) / sizeof(char *) },
    { lookupTableLowpassType, sizeof(lookupTableLowpassType) / sizeof(char *) },
    { lookupTableFailsafe, sizeof(lookupTableFailsafe) / sizeof(char *) },
    { lookupTableCrashRecovery, sizeof(lookupTableCrashRecovery) / sizeof(char *) },
#ifdef USE_OSD
    { lookupTableOsdType, sizeof(lookupTableOsdType) / sizeof(char *) },
#endif
#ifdef USE_CAMERA_CONTROL
    { lookupTableCameraControlMode, sizeof(lookupTableCameraControlMode) / sizeof(char *) },
#endif
    { lookupTableBusType, sizeof(lookupTableBusType) / sizeof(char *) },
#ifdef USE_MAX7456
    { lookupTableMax7456Clock, sizeof(lookupTableMax7456Clock) / sizeof(char *) },
#endif
#ifdef USE_RANGEFINDER
    { lookupTableRangefinderHardware, sizeof(lookupTableRangefinderHardware) / sizeof(char *) },
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    { lookupTableGyroOverflowCheck, sizeof(lookupTableGyroOverflowCheck) / sizeof(char *) },
#endif
};

const clivalue_t valueTable[] = {
// PG_GYRO_CONFIG
    { "align",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_align) },
    { "lpf",                   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_LPF }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf) },
#if defined(USE_GYRO_SPI_ICM20649)
    { "high_range",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_high_fsr) },
#endif
    { "sync_denom",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1, 32 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_sync_denom) },
    { "lowpass_type",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LOWPASS_TYPE }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_lpf_type) },
    { "lowpass_hz",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  255 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_lpf_hz) },
    { "notch1_hz",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_1) },
    { "notch1_cutoff",         VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_1) },
    { "notch2_hz",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_2) },
    { "notch2_cutoff",         VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_2) },
#if defined(USE_GYRO_FAST_KALMAN)
    { "kalman_q",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_kalman_q) },
    { "kalman_r",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_kalman_r) },
    { "kalman_p",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 16000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_kalman_p) },
#endif
    { "moron_threshold",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  200 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroMovementCalibrationThreshold) },
#ifdef USE_GYRO_OVERFLOW_CHECK
    { "overflow_detect",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_OVERFLOW_CHECK }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, checkOverflow) },
#endif
#if defined(GYRO_USES_SPI)
#if defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU9250) || defined(USE_GYRO_SPI_ICM20689)
    { "use_32khz",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_use_32khz) },
#endif
#endif
#ifdef USE_DUAL_GYRO
    { "to_use",                VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 1 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_to_use) },
#endif

// PG_ACCELEROMETER_CONFIG
    { "align",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_align) },
    { "hardware",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ACC_HARDWARE }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_hardware) },
#if defined(USE_GYRO_SPI_ICM20649)
    { "high_range",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_high_fsr) },
#endif
    { "lpf_hz",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 400 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_lpf_hz) },
    { "trim_pitch",             VAR_INT16  | MASTER_VALUE, .config.minmax = { -300, 300 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accelerometerTrims.values.pitch) },
    { "trim_roll",              VAR_INT16  | MASTER_VALUE, .config.minmax = { -300, 300 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accelerometerTrims.values.roll) },

// PG_COMPASS_CONFIG
#ifdef USE_MAG
    { "align",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_align) },
    { "bustype",                VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_bustype) },
    { "i2c_device",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, I2CDEV_COUNT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_i2c_device) },
    { "i2c_address",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, I2C_ADDR7_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_i2c_address) },
    { "spi_device",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, SPIDEV_COUNT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_spi_device) },
    { "hardware",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAG_HARDWARE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_hardware) },
    { "declination",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -18000, 18000 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_declination) },
    { "zero_x",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN, INT16_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw[X]) },
    { "zero_y",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN, INT16_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw[Y]) },
    { "zero_z",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN, INT16_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw[Z]) },
#endif

// PG_BAROMETER_CONFIG
#ifdef USE_BARO
    { "bustype",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_bustype) },
    { "spi_device",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 5 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_spi_device) },
    { "i2c_device",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 5 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_i2c_device) },
    { "i2c_address",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, I2C_ADDR7_MAX }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_i2c_address) },
    { "hardware",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BARO_HARDWARE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_hardware) },
    { "tab_size",              VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, BARO_SAMPLE_COUNT_MAX }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_sample_count) },
    { "noise_lpf",             VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, 1000 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_noise_lpf) },
    { "cf_vel",                VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, 1000 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_cf_vel) },
    { "cf_alt",                VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, 1000 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_cf_alt) },
#endif

// PG_RX_CONFIG
    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1200, 1700 }, PG_RX_CONFIG, offsetof(rxConfig_t, midrc) },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, mincheck) },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, maxcheck) },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE, .config.minmax = { 0, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_channel) },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmax = { RSSI_SCALE_MIN, RSSI_SCALE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_scale) },
    { "rssi_invert",                VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_invert) },
    { "rc_interp",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_INTERPOLATION }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolation) },
    { "rc_interp_ch",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_INTERPOLATION_CHANNELS }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolationChannels) },
    { "rc_interp_int",              VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1, 50 }, PG_RX_CONFIG, offsetof(rxConfig_t, rcInterpolationInterval) },
    { "fpv_mix_degrees",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 50 }, PG_RX_CONFIG, offsetof(rxConfig_t, fpvCamAngleDegrees) },
    { "max_aux_channels",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, MAX_AUX_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, max_aux_channel) },
#ifdef USE_SERIAL_RX
    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SERIAL_RX }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_provider) },
    { "serialrx_inverted",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_inverted) },
#endif
#ifdef USE_SPEKTRUM_BIND
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { SPEKTRUM_SAT_BIND_DISABLED, SPEKTRUM_SAT_BIND_MAX}, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind) },
    { "spektrum_sat_bind_autoreset",VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind_autoreset) },
#endif
    { "airmode_start_throttle",     VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000, 2000 }, PG_RX_CONFIG, offsetof(rxConfig_t, airModeActivateThreshold) },
    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_min_usec) },
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_max_usec) },
    { "serialrx_halfduplex",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, halfDuplex) },
#ifdef USE_RX_SPI
    { "rx_spi_protocol",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RX_SPI }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_spi_protocol) },
#endif

// PG_ADC_CONFIG
#if defined(ADC)
    { "device",                 VAR_INT8   | MASTER_VALUE, .config.minmax = { 0, ADCDEV_COUNT }, PG_ADC_CONFIG, offsetof(adcConfig_t, device) },
#endif

// PG_PWM_CONFIG
#if defined(USE_PWM)
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  .config.lookup = { TABLE_OFF_ON }, PG_PWM_CONFIG, offsetof(pwmConfig_t, inputFilteringMode) },
#endif

// PG_BLACKBOX_CONFIG
#ifdef USE_BLACKBOX
    { "p_ratio",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, INT16_MAX }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, p_denom) },
    { "device",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_DEVICE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, device) },
    { "record_acc",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, record_acc) },
    { "mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_MODE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, mode) },
#endif

// PG_MOTOR_CONFIG
    { "min_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, minthrottle) },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, maxthrottle) },
    { "min_command",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, mincommand) },
#ifdef USE_DSHOT
    { "dshot_idle_value",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, 2000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, digitalIdleOffsetValue) },
#endif
    { "use_unsynced_pwm",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useUnsyncedPwm) },
    { "pwm_protocol",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MOTOR_PWM_PROTOCOL }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmProtocol) },
    { "pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 200, 32000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmRate) },
    { "pwm_inversion",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmInversion) },

// PG_THROTTLE_CORRECTION_CONFIG
    { "value",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  150 }, PG_THROTTLE_CORRECTION_CONFIG, offsetof(throttleCorrectionConfig_t, throttle_correction_value) },
    { "angle",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1,  900 }, PG_THROTTLE_CORRECTION_CONFIG, offsetof(throttleCorrectionConfig_t, throttle_correction_angle) },

// PG_FAILSAFE_CONFIG
    { "delay",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_delay) },
    { "off_delay",         VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_off_delay) },
    { "throttle",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle) },
    { "kill_switch",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_kill_switch) },
    { "throttle_low_delay",VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 300 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle_low_delay) },
    { "procedure",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FAILSAFE }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_procedure) },

// PG_BOARDALIGNMENT_CONFIG
    { "roll",           VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, rollDegrees) },
    { "pitch",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, pitchDegrees) },
    { "yaw",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, yawDegrees) },

// PG_GIMBAL_CONFIG
#ifdef USE_SERVOS
    { "mode",                VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GIMBAL_MODE }, PG_GIMBAL_CONFIG, offsetof(gimbalConfig_t, mode) },
#endif

// PG_BATTERY_CONFIG
    { "bat_capacity",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 20000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, batteryCapacity) },
    { "vbat_max_cell_voltage",      VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10, 50 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmaxcellvoltage) },
    { "vbat_full_cell_voltage",     VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10, 50 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatfullcellvoltage) },
    { "vbat_min_cell_voltage",      VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10, 50 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmincellvoltage) },
    { "vbat_warning_cell_voltage",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10, 50 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatwarningcellvoltage) },
    { "vbat_hysteresis",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 250 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbathysteresis) },
    { "current_meter",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CURRENT_METER }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, currentMeterSource) },
    { "battery_meter",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VOLTAGE_METER }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, voltageMeterSource) },
    { "vbat_detect_cell_voltage",   VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 200 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatnotpresentcellvoltage) },
    { "use_vbat_alerts",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, useVBatAlerts) },
    { "use_cbat_alerts",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, useConsumptionAlerts) },
    { "cbat_alert_percent",         VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, consumptionWarningPercentage) },
    { "vbat_cutoff_percent",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, lvcPercentage) },

//  PG_VOLTAGE_SENSOR_ADC_CONFIG
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmax = { VBAT_SCALE_MIN, VBAT_SCALE_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatscale) },

// PG_CURRENT_SENSOR_ADC_CONFIG
    { "ibata_scale",                VAR_INT16  | MASTER_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_ADC_CONFIG, offsetof(currentSensorADCConfig_t, scale) },
    { "ibata_offset",               VAR_INT16  | MASTER_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_ADC_CONFIG, offsetof(currentSensorADCConfig_t, offset) },
// PG_CURRENT_SENSOR_ADC_CONFIG
#ifdef USE_VIRTUAL_CURRENT_METER
    { "ibatv_scale",                VAR_INT16  | MASTER_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, offsetof(currentSensorVirtualConfig_t, scale) },
    { "ibatv_offset",               VAR_INT16  | MASTER_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, offsetof(currentSensorVirtualConfig_t, offset) },
#endif

#ifdef BEEPER
// PG_BEEPER_DEV_CONFIG
    { "inversion",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, isInverted) },
    { "od",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, isOpenDrain) },
    { "frequency",           VAR_INT16  | MASTER_VALUE, .config.minmax = { 0, 16000 }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, frequency) },

// PG_BEEPER_CONFIG
#ifdef USE_DSHOT
    { "dshot_beacon_tone",   VAR_UINT8  | MASTER_VALUE, .config.minmax = {0, DSHOT_CMD_BEACON5 }, PG_BEEPER_CONFIG, offsetof(beeperConfig_t, dshotBeaconTone) },
#endif
#endif

// PG_MIXER_CONFIG
    { "yaw_motors_reversed",        VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, yaw_motors_reversed) },

// PG_MOTOR_3D_CONFIG
    { "deadband_low",            VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_RANGE_MIDDLE }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_low) },
    { "deadband_high",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_MIDDLE, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_high) },
    { "neutral",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, neutral3d) },
    { "deadband_throttle",       VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_throttle) },

// PG_SERVO_CONFIG
#ifdef USE_SERVOS
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_SERVO_CONFIG, offsetof(servoConfig_t, dev.servoCenterPulse) },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 50, 498 }, PG_SERVO_CONFIG, offsetof(servoConfig_t, dev.servoPwmRate) },
    { "servo_lowpass_hz",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 400}, PG_SERVO_CONFIG, offsetof(servoConfig_t, servo_lowpass_freq) },
    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SERVO_CONFIG, offsetof(servoConfig_t, tri_unarmed_servo) },
    { "channel_forwarding_start",   VAR_UINT8  | MASTER_VALUE, .config.minmax = { AUX1, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_SERVO_CONFIG, offsetof(servoConfig_t, channelForwardingStartChannel) },
#endif

// PG_CONTROLRATE_PROFILES
    { "rc_rate",                    VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, 255 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRate8) },
    { "rc_rate_yaw",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, 255 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcYawRate8) },
    { "rc_expo",                    VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo8) },
    { "rc_expo_yaw",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcYawExpo8) },
    { "thr_mid",                    VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrMid8) },
    { "thr_expo",                   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrExpo8) },
    { "roll_srate",                 VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_ROLL]) },
    { "pitch_srate",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_PITCH]) },
    { "yaw_srate",                  VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, CONTROL_RATE_CONFIG_YAW_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_YAW]) },
    { "tpa_rate",                   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmax = { 0, CONTROL_RATE_CONFIG_TPA_MAX}, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, dynThrPID) },
    { "tpa_breakpoint",             VAR_UINT16 | PROFILE_RATE_VALUE, .config.minmax = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, tpa_breakpoint) },

// PG_SERIAL_CONFIG
    { "reboot_character",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 48, 126 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, reboot_character) },
    { "serial_update_rate_hz",      VAR_UINT16 | MASTER_VALUE, .config.minmax = { 100, 2000 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, serial_update_rate_hz) },

// PG_IMU_CONFIG
    { "accxy_deadband",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_IMU_CONFIG, offsetof(imuConfig_t, accDeadband.xy) },
    { "accz_deadband",              VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_IMU_CONFIG, offsetof(imuConfig_t, accDeadband.z) },
    { "acc_unarmedcal",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_IMU_CONFIG, offsetof(imuConfig_t, acc_unarmedcal) },
    { "imu_dcm_kp",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 32000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_kp) },
    { "imu_dcm_ki",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 32000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_ki) },
    { "small_angle",                VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 180 }, PG_IMU_CONFIG, offsetof(imuConfig_t, small_angle) },

// PG_ARMING_CONFIG
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 60 }, PG_ARMING_CONFIG, offsetof(armingConfig_t, auto_disarm_delay) },
    { "disarm_kill_switch",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ARMING_CONFIG, offsetof(armingConfig_t, disarm_kill_switch) },
    { "gyro_cal_on_first_arm",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ARMING_CONFIG, offsetof(armingConfig_t, gyro_cal_on_first_arm) },


// PG_GPS_CONFIG
#ifdef USE_GPS
    { "provider",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_PROVIDER }, PG_GPS_CONFIG, offsetof(gpsConfig_t, provider) },
    { "sbas_mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_SBAS_MODE }, PG_GPS_CONFIG, offsetof(gpsConfig_t, sbasMode) },
    { "auto_config",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, autoConfig) },
    { "auto_baud",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, autoBaud) },
#endif

// PG_NAVIGATION_CONFIG
#ifdef USE_NAV
    { "gps_wp_radius",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 2000 }, PG_NAVIGATION_CONFIG, offsetof(navigationConfig_t, gps_wp_radius) },
    { "controls_heading",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_NAVIGATION_CONFIG, offsetof(navigationConfig_t, nav_controls_heading) },
    { "speed_min",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10, 2000 }, PG_NAVIGATION_CONFIG, offsetof(navigationConfig_t, nav_speed_min) },
    { "speed_max",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10, 2000 }, PG_NAVIGATION_CONFIG, offsetof(navigationConfig_t, nav_speed_max) },
    { "slew_rate",              VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_NAVIGATION_CONFIG, offsetof(navigationConfig_t, nav_slew_rate) },
#endif

// PG_AIRPLANE_CONFIG
#if defined(USE_ALT_HOLD)
    { "althold_reversed", VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_AIRPLANE_CONFIG, offsetof(airplaneConfig_t, fixedwing_althold_reversed) },
#endif

// PG_RC_CONTROLS_CONFIG
#if defined(USE_ALT_HOLD)
    { "alt_hold_deadband",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1, 250 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, alt_hold_deadband) },
    { "alt_hold_fast_change",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, alt_hold_fast_change) },
#endif
    { "deadband",                   VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 32 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, deadband) },
    { "yaw_deadband",               VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_deadband) },
    { "yaw_control_reversed",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_control_reversed) },

// PG_PID_CONFIG
    { "pid_process_denom",          VAR_UINT8  | MASTER_VALUE,  .config.minmax = { 1, MAX_PID_PROCESS_DENOM }, PG_PID_CONFIG, offsetof(pidConfig_t, pid_process_denom) },

// PG_PID_PROFILE
    { "dterm_lowpass_type",         VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LOWPASS_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_filter_type) },
    { "dterm_lowpass",              VAR_INT16  | PROFILE_VALUE, .config.minmax = { 0, 16000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf_hz) },
    { "dterm_notch_hz",             VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 16000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_notch_hz) },
    { "dterm_notch_cutoff",         VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 16000 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_notch_cutoff) },
    { "vbat_pid_gain",              VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, vbatPidCompensation) },
    { "pid_at_min_throttle",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, pidAtMinThrottle) },
    { "anti_gravity_threshold",     VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 20, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermThrottleThreshold) },
    { "anti_gravity_gain",          VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 1000, 30000 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermAcceleratorGain) },
    { "setpoint_relax_ratio",       VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, setpointRelaxRatio) },
    { "dterm_setpoint_weight",      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 254 }, PG_PID_PROFILE, offsetof(pidProfile_t, dtermSetpointWeight) },
    { "acc_limit_yaw",              VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 1, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, yawRateAccelLimit) },
    { "acc_limit",                  VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 1, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, rateAccelLimit) },
    { "crash_dthreshold",           VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_dthreshold) },
    { "crash_gthreshold",           VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_gthreshold) },
    { "crash_setpoint_threshold",   VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_setpoint_threshold) },
    { "crash_time",                 VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 5000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_time) },
    { "crash_delay",                VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_delay) },
    { "crash_recovery_angle",       VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 30 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery_angle) },
    { "crash_recovery_rate",        VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery_rate) },
    { "crash_limit_yaw",            VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_limit_yaw) },
    { "crash_recovery",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CRASH_RECOVERY }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery) },

    { "iterm_windup",               VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 30, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermWindupPointPercent) },
    { "iterm_limit",                VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermLimit) },
    { "pidsum_limit",               VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 100, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimit) },
    { "pidsum_limit_yaw",           VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 100, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimitYaw) },
    { "yaw_lowpass",                VAR_UINT16 | PROFILE_VALUE, .config.minmax = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, yaw_lpf_hz) },

    { "p_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].P) },
    { "i_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].I) },
    { "d_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].D) },
    { "p_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].P) },
    { "i_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].I) },
    { "d_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].D) },
    { "p_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].P) },
    { "i_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].I) },
    { "d_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].D) },

#ifdef USE_ALT_HOLD
    { "p_alt",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ALT].P) },
    { "i_alt",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ALT].I) },
    { "d_alt",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ALT].D) },
    { "p_vel",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_VEL].P) },
    { "i_vel",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_VEL].I) },
    { "d_vel",                      VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_VEL].D) },
#endif

    { "p_level",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].P) },
    { "i_level",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].I) },
    { "d_level",                    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].D) },

    { "level_limit",                VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 10, 90 }, PG_PID_PROFILE, offsetof(pidProfile_t, levelAngleLimit) },

    { "horizon_tilt_effect",        VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  250 }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_tilt_effect) },
    { "horizon_tilt_expert_mode",   VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_tilt_expert_mode) },
#ifdef USE_NAV
    { "gps_pos_p",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_POS].P) },
    { "gps_pos_i",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_POS].I) },
    { "gps_pos_d",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_POS].D) },
    { "gps_posr_p",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_POSR].P) },
    { "gps_posr_i",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_POSR].I) },
    { "gps_posr_d",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_POSR].D) },
    { "gps_nav_p",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_NAVR].P) },
    { "gps_nav_i",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_NAVR].I) },
    { "gps_nav_d",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_NAVR].D) },
#endif

// PG_TELEMETRY_CONFIG
#ifdef USE_TELEMETRY
    { "tlm_switch",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, telemetry_switch) },
    { "tlm_inverted",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, telemetry_inverted) },
    { "tlm_halfduplex",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, halfDuplex) },
#if defined(USE_TELEMETRY_FRSKY_HUB)
#if defined(USE_GPS)
    { "frsky_default_lat",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -9000, 9000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLatitude) },
    { "frsky_default_long",         VAR_INT16  | MASTER_VALUE, .config.minmax = { -18000, 18000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLongitude) },
    { "frsky_gps_format",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, FRSKY_FORMAT_NMEA }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_coordinate_format) },
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_unit) },
#endif
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE, .config.minmax = { FRSKY_VFAS_PRECISION_LOW,  FRSKY_VFAS_PRECISION_HIGH }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_vfas_precision) },
#endif // USE_TELEMETRY_FRSKY_HUB
    { "hott_alarm_int",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 120 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, hottAlarmSoundInterval) },
    { "pid_in_tlm",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = {TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, pidValuesAsTelemetry) },
#if defined(USE_TELEMETRY_IBUS)
    { "report_cell_voltage",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, report_cell_voltage) },
#endif
#endif // USE_TELEMETRY

// PG_LED_STRIP_CONFIG
#ifdef USE_LED_STRIP
    { "visual_beeper",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_visual_beeper) },
#endif

// PG_SDCARD_CONFIG
#ifdef USE_SDCARD
    { "dma",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, useDma) },
#endif

// PG_OSD_CONFIG
#ifdef USE_OSD
    { "units",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_OSD_CONFIG, offsetof(osdConfig_t, units) },
    { "warnings",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings) },

    { "rssi_alarm",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_OSD_CONFIG, offsetof(osdConfig_t, rssi_alarm) },
    { "cap_alarm",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 20000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, cap_alarm) },
    { "alt_alarm",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 10000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, alt_alarm) },

    { "ah_max_pit",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 90 }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahMaxPitch) },
    { "ah_max_rol",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 90 }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahMaxRoll) },

    { "tim1",                   VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, timers[OSD_TIMER_1]) },
    { "tim2",                   VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, timers[OSD_TIMER_2]) },

    { "vbat_pos",               VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MAIN_BATT_VOLTAGE]) },
    { "rssi_pos",               VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_RSSI_VALUE]) },
    { "tim_1_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ITEM_TIMER_1]) },
    { "tim_2_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ITEM_TIMER_2]) },
    { "remaining_time_estimate_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_REMAINING_TIME_ESTIMATE]) },
    { "flymode_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_FLYMODE]) },
    { "throttle_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_THROTTLE_POS]) },
    { "vtx_channel_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_VTX_CHANNEL]) },
    { "crosshairs",             VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CROSSHAIRS]) },
    { "ah_sbar",                VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_HORIZON_SIDEBARS]) },
    { "ah_pos",                 VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ARTIFICIAL_HORIZON]) },
    { "current_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CURRENT_DRAW]) },
    { "mah_drawn_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MAH_DRAWN]) },
    { "craft_name_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CRAFT_NAME]) },
    { "gps_speed_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_SPEED]) },
    { "gps_lon_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_LON]) },
    { "gps_lat_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_LAT]) },
    { "gps_sats_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_SATS]) },
    { "home_dir_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_HOME_DIR]) },
    { "home_dist_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_HOME_DIST]) },
    { "compass_bar_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_COMPASS_BAR]) },
    { "altitude_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ALTITUDE]) },
    { "pid_roll_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ROLL_PIDS]) },
    { "pid_pitch_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PITCH_PIDS]) },
    { "pid_yaw_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_YAW_PIDS]) },
    { "debug_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_DEBUG]) },
    { "power_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_POWER]) },
    { "pidrate_profile_pos",    VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PIDRATE_PROFILE]) },
    { "warnings_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_WARNINGS]) },
    { "avg_cell_voltage_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_AVG_CELL_VOLTAGE]) },
    { "pit_ang_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PITCH_ANGLE]) },
    { "rol_ang_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ROLL_ANGLE]) },
    { "battery_usage_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MAIN_BATT_USAGE]) },
    { "disarmed_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_DISARMED]) },
    { "nheading_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_NUMERICAL_HEADING]) },
    { "nvario_pos",             VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_NUMERICAL_VARIO]) },
    { "esc_tmp_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ESC_TMP]) },
    { "esc_rpm_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ESC_RPM]) },
    { "rtc_date_time_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_RTC_DATETIME]) },
    { "adjustment_range_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0, OSD_POSCFG_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ADJUSTMENT_RANGE]) },

    { "stat_max_spd",           VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_MAX_SPEED])},
    { "stat_max_dist",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_MAX_DISTANCE])},
    { "stat_min_batt",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_MIN_BATTERY])},
    { "stat_min_rssi",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_MIN_RSSI])},
    { "stat_max_curr",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_MAX_CURRENT])},
    { "stat_used_mah",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_USED_MAH])},
    { "stat_max_alt",           VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_MAX_ALTITUDE])},
    { "stat_bbox",              VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_BLACKBOX])},
    { "stat_endbatt",           VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_END_BATTERY])},
    { "stat_bb_no",             VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_BLACKBOX_NUMBER])},
    { "stat_tim_1",             VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_TIMER_1])},
    { "stat_tim_2",             VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats[OSD_STAT_TIMER_2])},
#endif

// PG_SYSTEM_CONFIG
#ifndef SKIP_TASK_STATISTICS
    { "task_statistics",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, task_statistics) },
#endif
    { "debug_mode",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DEBUG }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, debug_mode) },
#if defined(STM32F4) && !defined(DISABLE_OVERCLOCK)
    { "cpu_overclock",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, cpu_overclock) },
#endif
    { "pwr_on_arm_grace",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 30 }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, powerOnArmingGraceTime) },

// PG_VTX_CONFIG
#ifdef USE_VTX_COMMON
    { "band",                   VAR_UINT8  | MASTER_VALUE, .config.minmax = { VTX_SETTINGS_MIN_BAND, VTX_SETTINGS_MAX_BAND }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, band) },
    { "channel",                VAR_UINT8  | MASTER_VALUE, .config.minmax = { VTX_SETTINGS_MIN_CHANNEL, VTX_SETTINGS_MAX_CHANNEL }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, channel) },
    { "power",                  VAR_UINT8  | MASTER_VALUE, .config.minmax = { VTX_SETTINGS_MIN_POWER, VTX_SETTINGS_POWER_COUNT-1 }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, power) },
    { "low_power_disarm",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, lowPowerDisarm) },
#ifdef VTX_SETTINGS_FREQCMD
    { "freq",                   VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, VTX_SETTINGS_MAX_FREQUENCY_MHZ }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, freq) },
    { "pit_mode_freq",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, VTX_SETTINGS_MAX_FREQUENCY_MHZ }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, pitModeFreq) },
#endif
#endif

// PG_VTX_CONFIG
#if defined(USE_VTX_CONTROL) && defined(USE_VTX_COMMON)
    { "vtx_halfduplex",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_VTX_CONFIG, offsetof(vtxConfig_t, halfDuplex) },
#endif

// PG_VCD_CONFIG
#ifdef USE_MAX7456
    { "video_system",           VAR_UINT8   | MASTER_VALUE, .config.minmax = { 0, 2 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, video_system) },
    { "h_offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -32, 31 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, h_offset) },
    { "v_offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -15, 16 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, v_offset) },
#endif

// PG_MAX7456_CONFIG
#ifdef USE_MAX7456
    { "clock",              VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAX7456_CLOCK }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, clockConfig) },
#endif

// PG_DISPLAY_PORT_MSP_CONFIG
#ifdef USE_MSP_DISPLAYPORT
    { "col_adjust", VAR_INT8    | MASTER_VALUE, .config.minmax = { -6, 0 }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, colAdjust) },
    { "row_adjust", VAR_INT8    | MASTER_VALUE, .config.minmax = { -3, 0 }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, rowAdjust) },
#endif

// PG_DISPLAY_PORT_MSP_CONFIG
#ifdef USE_MAX7456
    { "col_adjust", VAR_INT8| MASTER_VALUE, .config.minmax = { -6, 0 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, colAdjust) },
    { "row_adjust", VAR_INT8| MASTER_VALUE, .config.minmax = { -3, 0 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, rowAdjust) },
    { "inv",        VAR_UINT8| MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, invert) },
    { "blk",        VAR_UINT8| MASTER_VALUE, .config.minmax = { 0, 3 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, blackBrightness) },
    { "wht",        VAR_UINT8| MASTER_VALUE, .config.minmax = { 0, 3 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, whiteBrightness) },
#endif

#ifdef USE_ESC_SENSOR
    { "halfduplex",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ESC_SENSOR_CONFIG, offsetof(escSensorConfig_t, halfDuplex) },
#endif

#ifdef USE_RX_FRSKY_SPI
    { "autobind",             VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_FRSKY_SPI_CONFIG, offsetof(rxFrSkySpiConfig_t, autoBind) },
    { "tx_id",                VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 2, PG_RX_FRSKY_SPI_CONFIG, offsetof(rxFrSkySpiConfig_t, bindTxId) },
    { "offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -127, 127 }, PG_RX_FRSKY_SPI_CONFIG, offsetof(rxFrSkySpiConfig_t, bindOffset) },
    { "bind_hop_data",        VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 50, PG_RX_FRSKY_SPI_CONFIG, offsetof(rxFrSkySpiConfig_t, bindHopData) },
    { "x_rx_num",                 VAR_UINT8   | MASTER_VALUE, .config.minmax = { 0, 255 }, PG_RX_FRSKY_SPI_CONFIG, offsetof(rxFrSkySpiConfig_t, rxNum) },
#endif

    { "inversion",                  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, ((1 << STATUS_LED_NUMBER) - 1) }, PG_STATUS_LED_CONFIG, offsetof(statusLedConfig_t, inversion) },

#ifdef USE_DASHBOARD
    { "i2c_bus",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, I2CDEV_COUNT }, PG_DASHBOARD_CONFIG, offsetof(dashboardConfig_t, device) },
    { "i2c_addr",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { I2C_ADDR7_MIN, I2C_ADDR7_MAX }, PG_DASHBOARD_CONFIG, offsetof(dashboardConfig_t, address) },
#endif

// PG_CAMERA_CONTROL_CONFIG
#ifdef USE_CAMERA_CONTROL
    { "mode", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CAMERA_CONTROL_MODE }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, mode) },
    { "ref_voltage", VAR_UINT16 | MASTER_VALUE, .config.minmax = { 200, 400 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, refVoltage) },
    { "key_delay", VAR_UINT16 | MASTER_VALUE, .config.minmax = { 100, 500 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, keyDelayMs) },
    { "internal_resistance", VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10, 1000 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, internalResistance) },
#endif

// PG_RANGEFINDER_CONFIG
#ifdef USE_RANGEFINDER
    { "hardware", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RANGEFINDER_HARDWARE }, PG_RANGEFINDER_CONFIG, offsetof(rangefinderConfig_t, rangefinder_hardware) },
#endif
};

const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);

void settingsBuildCheck() {
    BUILD_BUG_ON(LOOKUP_TABLE_COUNT != ARRAYLEN(lookupTables));
}
