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
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#include "platform.h"

// FIXME remove this for targets that don't need a CLI.  Perhaps use a no-op macro when USE_CLI is not enabled
// signal that we're in cli mode
uint8_t cliMode = 0;
extern uint8_t __config_start;   // configured via linker script when building binaries.
extern uint8_t __config_end;

#ifdef USE_CLI

#include "blackbox/blackbox.h"

#include "build/assert.h"
#include "build/build_config.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"

#include "config/config_eeprom.h"
#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/buf_writer.h"
#include "drivers/bus_i2c.h"
#include "drivers/compass/compass.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/logging.h"
#include "drivers/pwm_rx.h"
#include "drivers/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/stack_check.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "fc/cli.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/osd.h"
#include "io/serial.h"

#include "navigation/navigation.h"

#include "rx/rx.h"
#include "rx/spektrum.h"
#include "rx/eleres.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/diagnostics.h"
#include "sensors/gyro.h"
#include "sensors/pitotmeter.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

#include "telemetry/frsky.h"
#include "telemetry/telemetry.h"
#include "build/debug.h"

#if FLASH_SIZE > 128
#define PLAY_SOUND
#endif

extern timeDelta_t cycleTime; // FIXME dependency on mw.c
extern uint8_t detectedSensors[SENSOR_INDEX_COUNT];

static serialPort_t *cliPort;
static bufWriter_t *cliWriter;
static uint8_t cliWriteBuffer[sizeof(*cliWriter) + 128];

#if defined(USE_ASSERT)
static void cliAssert(char *cmdline);
#endif

#if defined(BOOTLOG)
static void cliBootlog(char *cmdline);
#endif

static const char* const emptyName = "-";

// buffer
static char cliBuffer[64];
static uint32_t bufferIndex = 0;

#ifndef USE_QUAD_MIXER_ONLY
// sync this with mixerMode_e
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", NULL
};
#endif

// sync this with features_e
static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "",
    "", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DASHBOARD", "",
    "BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", "AIRMODE",
    "SUPEREXPO", "VTX", "RX_SPI", "SOFTSPI", "PWM_SERVO_DRIVER", "PWM_OUTPUT_ENABLE", "OSD", NULL
};

/* Sensor names (used in lookup tables for *_hardware settings and in status command output) */
// sync with accelerationSensor_e
static const char * const lookupTableAccHardware[] = { "NONE", "AUTO", "ADXL345", "MPU6050", "MMA845x", "BMA280", "LSM303DLHC", "MPU6000", "MPU6500", "MPU9250", "FAKE"};
#if (FLASH_SIZE > 64)
// sync with gyroSensor_e
static const char * const gyroNames[] = { "NONE", "AUTO", "MPU6050", "L3G4200D", "MPU3050", "L3GD20", "MPU6000", "MPU6500", "MPU9250", "FAKE"};
// sync with baroSensor_e
static const char * const lookupTableBaroHardware[] = { "NONE", "AUTO", "BMP085", "MS5611", "BMP280", "MS5607", "FAKE"};
// sync with magSensor_e
static const char * const lookupTableMagHardware[] = { "NONE", "AUTO", "HMC5883", "AK8975", "GPSMAG", "MAG3110", "AK8963", "IST8310", "FAKE"};
// sycn with rangefinderType_e
static const char * const lookupTableRangefinderHardware[] = { "NONE", "HCSR04", "SRF10"};
// sync with pitotSensor_e
static const char * const lookupTablePitotHardware[] = { "NONE", "AUTO", "MS4525", "ADC", "VIRTUAL", "FAKE"};

// sync this with sensors_e
static const char * const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "RANGEFINDER", "PITOT", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG | SENSOR_RANGEFINDER | SENSOR_PITOT)

static const char * const hardwareSensorStatusNames[] = {
    "NONE", "OK", "UNAVAILABLE", "FAILING"
};

static const char * const *sensorHardwareNames[] = {
        gyroNames,
        lookupTableAccHardware,
        lookupTableBaroHardware,
        lookupTableMagHardware,
        lookupTableRangefinderHardware,
        lookupTablePitotHardware
};
#endif

static const char * const lookupTableOffOn[] = {
    "OFF", "ON"
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

#ifdef GPS
static const char * const lookupTableGPSProvider[] = {
    "NMEA", "UBLOX", "I2C-NAV", "NAZA", "UBLOX7"
};

static const char * const lookupTableGPSSBASMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN", "NONE"
};

static const char * const lookupTableGpsModel[] = {
    "PEDESTRIAN", "AIR_1G", "AIR_4G"
};
#endif

static const char * const lookupTableCurrentSensor[] = {
    "NONE", "ADC", "VIRTUAL"
};

#ifdef USE_SERVOS
static const char * const lookupTableGimbalMode[] = {
    "NORMAL", "MIXTILT"
};
#endif

#ifdef BLACKBOX
static const char * const lookupTableBlackboxDevice[] = {
    "SERIAL", "SPIFLASH", "SDCARD"
};
#endif

#ifdef SERIAL_RX
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
    "CRSF"
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
    "ELERES"
};
#endif

static const char * const lookupTableGyroLpf[] = {
    "256HZ",
    "188HZ",
    "98HZ",
    "42HZ",
    "20HZ",
    "10HZ"
};

static const char * const lookupTableFailsafeProcedure[] = {
    "SET-THR", "DROP", "RTH", "NONE"
};

#ifdef NAV
static const char * const lookupTableNavControlMode[] = {
    "ATTI", "CRUISE"
};

static const char * const lookupTableNavRthAltMode[] = {
    "CURRENT", "EXTRA", "FIXED", "MAX", "AT_LEAST"
};

static const char * const lookupTableNavResetAltitude[] = {
    "NEVER", "FIRST_ARM", "EACH_ARM"
};

#endif

static const char * const lookupTableAuxOperator[] = {
    "OR", "AND"
};

#ifdef OSD
static const char * const lookupTableOsdType[] = {
    "AUTO",
    "PAL",
    "NTSC"
};
#endif

static const char * const lookupTablePwmProtocol[] = {
    "STANDARD", "ONESHOT125", "ONESHOT42", "MULTISHOT", "BRUSHED"
};

#ifdef ASYNC_GYRO_PROCESSING
static const char * const lookupTableAsyncMode[] = {
    "NONE", "GYRO", "ALL"
};
#endif

static const char * const lookupTableDebug[DEBUG_COUNT] = {
    "NONE",
    "GYRO",
    "NOTCH",
    "NAV_LANDING",
    "FW_ALTITUDE",
    "RANGEFINDER"
};

#ifdef TELEMETRY_LTM
static const char * const lookupTableLTMRates[] = {
    "NORMAL", "MEDIUM", "SLOW"
};
#endif

typedef struct lookupTableEntry_s {
    const char * const *values;
    const uint8_t valueCount;
} lookupTableEntry_t;

typedef enum {
    TABLE_OFF_ON = 0,
    TABLE_UNIT,
    TABLE_ALIGNMENT,
#ifdef GPS
    TABLE_GPS_PROVIDER,
    TABLE_GPS_SBAS_MODE,
    TABLE_GPS_DYN_MODEL,
#endif
#ifdef BLACKBOX
    TABLE_BLACKBOX_DEVICE,
#endif
    TABLE_CURRENT_SENSOR,
#ifdef USE_SERVOS
    TABLE_GIMBAL_MODE,
#endif
#ifdef SERIAL_RX
    TABLE_SERIAL_RX,
#endif
#ifdef USE_RX_SPI
    TABLE_RX_SPI,
#endif
    TABLE_GYRO_LPF,
    TABLE_ACC_HARDWARE,
#ifdef BARO
    TABLE_BARO_HARDWARE,
#endif
#ifdef MAG
    TABLE_MAG_HARDWARE,
#endif
#ifdef USE_RANGEFINDER
    TABLE_RANGEFINDER_HARDWARE,   // currently not used
#endif
#ifdef PITOT
    TABLE_PITOT_HARDWARE,
#endif
#ifdef NAV
    TABLE_NAV_USER_CTL_MODE,
    TABLE_NAV_RTH_ALT_MODE,
    TABLE_NAV_RESET_ALTITUDE,
#endif
    TABLE_AUX_OPERATOR,
    TABLE_MOTOR_PWM_PROTOCOL,
    TABLE_FAILSAFE_PROCEDURE,
#ifdef ASYNC_GYRO_PROCESSING
    TABLE_ASYNC_MODE,
#endif
#ifdef OSD
    TABLE_OSD,
#endif
    TABLE_DEBUG,
#ifdef TELEMETRY_LTM
    TABLE_LTM_UPDATE_RATE,
#endif
    LOOKUP_TABLE_COUNT
} lookupTableIndex_e;

static const lookupTableEntry_t lookupTables[] = {
    { lookupTableOffOn, sizeof(lookupTableOffOn) / sizeof(char *) },
    { lookupTableUnit, sizeof(lookupTableUnit) / sizeof(char *) },
    { lookupTableAlignment, sizeof(lookupTableAlignment) / sizeof(char *) },
#ifdef GPS
    { lookupTableGPSProvider, sizeof(lookupTableGPSProvider) / sizeof(char *) },
    { lookupTableGPSSBASMode, sizeof(lookupTableGPSSBASMode) / sizeof(char *) },
    { lookupTableGpsModel, sizeof(lookupTableGpsModel) / sizeof(char *) },
#endif
#ifdef BLACKBOX
    { lookupTableBlackboxDevice, sizeof(lookupTableBlackboxDevice) / sizeof(char *) },
#endif
    { lookupTableCurrentSensor, sizeof(lookupTableCurrentSensor) / sizeof(char *) },
#ifdef USE_SERVOS
    { lookupTableGimbalMode, sizeof(lookupTableGimbalMode) / sizeof(char *) },
#endif
#ifdef SERIAL_RX
    { lookupTableSerialRX, sizeof(lookupTableSerialRX) / sizeof(char *) },
#endif
#ifdef USE_RX_SPI
    { lookupTableRxSpi, sizeof(lookupTableRxSpi) / sizeof(char *) },
#endif
    { lookupTableGyroLpf, sizeof(lookupTableGyroLpf) / sizeof(char *) },
    { lookupTableAccHardware, sizeof(lookupTableAccHardware) / sizeof(char *) },
#ifdef BARO
    { lookupTableBaroHardware, sizeof(lookupTableBaroHardware) / sizeof(char *) },
#endif
#ifdef MAG
    { lookupTableMagHardware, sizeof(lookupTableMagHardware) / sizeof(char *) },
#endif
#ifdef USE_RANGEFINDER
    { lookupTableRangefinderHardware, sizeof(lookupTableRangefinderHardware) / sizeof(char *) },
#endif
#ifdef PITOT
    { lookupTablePitotHardware, sizeof(lookupTablePitotHardware) / sizeof(char *) },
#endif
#ifdef NAV
    { lookupTableNavControlMode, sizeof(lookupTableNavControlMode) / sizeof(char *) },
    { lookupTableNavRthAltMode, sizeof(lookupTableNavRthAltMode) / sizeof(char *) },
    { lookupTableNavResetAltitude, sizeof(lookupTableNavResetAltitude) / sizeof(char *) },
#endif
    { lookupTableAuxOperator, sizeof(lookupTableAuxOperator) / sizeof(char *) },
    { lookupTablePwmProtocol, sizeof(lookupTablePwmProtocol) / sizeof(char *) },
    { lookupTableFailsafeProcedure, sizeof(lookupTableFailsafeProcedure) / sizeof(char *) },
#ifdef ASYNC_GYRO_PROCESSING
    { lookupTableAsyncMode, sizeof(lookupTableAsyncMode) / sizeof(char *) },
#endif
#ifdef OSD
    { lookupTableOsdType, sizeof(lookupTableOsdType) / sizeof(char *) },
#endif
    { lookupTableDebug, sizeof(lookupTableDebug) / sizeof(char *) },
#ifdef TELEMETRY_LTM
    {lookupTableLTMRates, sizeof(lookupTableLTMRates) / sizeof(char *) },
#endif
};

#define VALUE_TYPE_OFFSET 0
#define VALUE_SECTION_OFFSET 4
#define VALUE_MODE_OFFSET 6

typedef enum {
    // value type, bits 0-3
    VAR_UINT8 = (0 << VALUE_TYPE_OFFSET),
    VAR_INT8 = (1 << VALUE_TYPE_OFFSET),
    VAR_UINT16 = (2 << VALUE_TYPE_OFFSET),
    VAR_INT16 = (3 << VALUE_TYPE_OFFSET),
    VAR_UINT32 = (4 << VALUE_TYPE_OFFSET),
    VAR_FLOAT = (5 << VALUE_TYPE_OFFSET), // 0x05

    // value section, bits 4-5
    MASTER_VALUE = (0 << VALUE_SECTION_OFFSET),
    PROFILE_VALUE = (1 << VALUE_SECTION_OFFSET),
    CONTROL_RATE_VALUE = (2 << VALUE_SECTION_OFFSET), // 0x20
    // value mode, bits 6-7
    MODE_DIRECT = (0 << VALUE_MODE_OFFSET),
    MODE_LOOKUP = (1 << VALUE_MODE_OFFSET), // 0x40
    MODE_MAX = (2 << VALUE_MODE_OFFSET), // 0x80
} cliValueFlag_e;

#define VALUE_TYPE_MASK (0x0F)
#define VALUE_SECTION_MASK (0x30)
#define VALUE_MODE_MASK (0xC0)

typedef struct cliMinMaxConfig_s {
    const int16_t min;
    const int16_t max;
} cliMinMaxConfig_t;

typedef struct cliMaxConfig_s {
    const uint32_t max;
} cliMaxConfig_t;

typedef struct cliLookupTableConfig_s {
    const lookupTableIndex_e tableIndex;
} cliLookupTableConfig_t;

typedef union {
    cliLookupTableConfig_t lookup;
    cliMinMaxConfig_t minmax;
    cliMaxConfig_t max;
} cliValueConfig_t;

typedef struct {
    const char *name;
    const uint8_t type; // see cliValueFlag_e
    const cliValueConfig_t config;

    pgn_t pgn;
    uint16_t offset;
} __attribute__((packed)) clivalue_t;

static const clivalue_t valueTable[] = {
// PG_GYRO_CONFIG
    { "looptime",                   VAR_UINT16 | MASTER_VALUE, .config.minmax = {0, 9000}, PG_GYRO_CONFIG, offsetof(gyroConfig_t, looptime) },
    { "gyro_sync",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroSync) },
    { "gyro_sync_denom",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1,  32 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroSyncDenominator) },
    { "align_gyro",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_align) },
    { "gyro_hardware_lpf",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_LPF }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf) },
    { "gyro_lpf_hz",                VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 200 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_lpf_hz)  },
    { "moron_threshold",            VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  128 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroMovementCalibrationThreshold) },
#ifdef USE_GYRO_NOTCH_1
    { "gyro_notch1_hz",             VAR_UINT16 | MASTER_VALUE, .config.minmax = {0, 500 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_1)  },
    { "gyro_notch1_cutoff",         VAR_UINT16 | MASTER_VALUE, .config.minmax = {1, 500 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_1)  },
#endif
#ifdef USE_GYRO_NOTCH_2
    { "gyro_notch2_hz",             VAR_UINT16 | MASTER_VALUE, .config.minmax = {0, 500 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_2)  },
    { "gyro_notch2_cutoff",         VAR_UINT16 | MASTER_VALUE, .config.minmax = {1, 500 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_2)  },
#endif
#ifdef USE_DUAL_GYRO
    { "gyro_to_use",                VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 1 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_to_use) },
#endif

// PG_ADC_CHANNEL_CONFIG
#ifdef USE_ADC
    { "vbat_adc_channel",           VAR_UINT8 | MASTER_VALUE, .config.minmax = {ADC_CHN_NONE, ADC_CHN_MAX}, PG_ADC_CHANNEL_CONFIG, offsetof(adcChannelConfig_t, adcFunctionChannel[ADC_BATTERY]) },
    { "rssi_adc_channel",           VAR_UINT8 | MASTER_VALUE, .config.minmax = {ADC_CHN_NONE, ADC_CHN_MAX}, PG_ADC_CHANNEL_CONFIG, offsetof(adcChannelConfig_t, adcFunctionChannel[ADC_RSSI]) },
    { "current_adc_channel",        VAR_UINT8 | MASTER_VALUE, .config.minmax = {ADC_CHN_NONE, ADC_CHN_MAX}, PG_ADC_CHANNEL_CONFIG, offsetof(adcChannelConfig_t, adcFunctionChannel[ADC_CURRENT]) },
    { "airspeed_adc_channel",       VAR_UINT8 | MASTER_VALUE, .config.minmax = {ADC_CHN_NONE, ADC_CHN_MAX}, PG_ADC_CHANNEL_CONFIG, offsetof(adcChannelConfig_t, adcFunctionChannel[ADC_AIRSPEED]) },
#endif

#ifdef USE_ACC_NOTCH
    { "acc_notch_hz",               VAR_UINT8  | MASTER_VALUE, .config.minmax = {0, 255 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t,acc_notch_hz)  },
    { "acc_notch_cutoff",           VAR_UINT8  | MASTER_VALUE, .config.minmax = {1, 255 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_notch_cutoff)  },
#endif

// PG_ACCELEROMETER_CONFIG
    { "align_acc",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_align) },
    { "acc_hardware",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ACC_HARDWARE }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_hardware) },
    { "acc_lpf_hz",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = {0, 200 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_lpf_hz) },
    { "acczero_x",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN,  INT16_MAX }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accZero.raw[X]) },
    { "acczero_y",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN,  INT16_MAX }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accZero.raw[Y]) },
    { "acczero_z",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN,  INT16_MAX }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accZero.raw[Z]) },
    { "accgain_x",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { 1,  8192 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accGain.raw[X]) },
    { "accgain_y",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { 1,  8192 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accGain.raw[Y]) },
    { "accgain_z",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { 1,  8192 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accGain.raw[Z]) },

// PG_RANGEFINDER_CONFIG
#ifdef USE_RANGEFINDER
    { "rangefinder_hardware",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RANGEFINDER_HARDWARE }, PG_RANGEFINDER_CONFIG, offsetof(rangefinderConfig_t, rangefinder_hardware) },
#endif

// PG_COMPASS_CONFIG
#ifdef MAG
    { "align_mag",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_align) },
    { "mag_hardware",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAG_HARDWARE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_hardware) },
    { "mag_declination",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -18000,  18000 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_declination) },
    { "magzero_x",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN,  INT16_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw[X]) },
    { "magzero_y",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN,  INT16_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw[Y]) },
    { "magzero_z",                  VAR_INT16  | MASTER_VALUE, .config.minmax = { INT16_MIN,  INT16_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw[Z]) },
    { "mag_calibration_time",       VAR_UINT8  | MASTER_VALUE, .config.minmax = { 30,  120 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magCalibrationTimeLimit) },
#endif

// PG_BAROMETER_CONFIG
#ifdef BARO
    { "baro_hardware",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BARO_HARDWARE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_hardware) },
    { "baro_use_median_filter",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, use_median_filtering) },
#endif

// PG_PITOTMETER_CONFIG
#ifdef PITOT
    { "pitot_hardware",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_PITOT_HARDWARE }, PG_PITOTMETER_CONFIG, offsetof(pitotmeterConfig_t, pitot_hardware) },
    { "pitot_use_median_filter",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PITOTMETER_CONFIG, offsetof(pitotmeterConfig_t, use_median_filtering) },
    { "pitot_noise_lpf",            VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0, 1 }, PG_PITOTMETER_CONFIG, offsetof(pitotmeterConfig_t, pitot_noise_lpf) },
    { "pitot_scale",                VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_PITOTMETER_CONFIG, offsetof(pitotmeterConfig_t, pitot_scale) },
#endif

// PG_RX_CONFIG
    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1200,  1700 }, PG_RX_CONFIG, offsetof(rxConfig_t, midrc) },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, mincheck) },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, maxcheck) },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE, .config.minmax = { 0,  MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_channel) },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmax = { RSSI_SCALE_MIN,  RSSI_SCALE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_scale) },
    { "rssi_invert",                VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rssiInvert) },
    { "rc_smoothing",               VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rcSmoothing) },
#ifdef SERIAL_RX
    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SERIAL_RX }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_provider) },
    { "sbus_inversion",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, sbus_inversion) },
#endif
#ifdef USE_RX_SPI
    { "rx_spi_protocol",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RX_SPI }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_spi_protocol) },
    { "rx_spi_id",                  VAR_UINT32 | MASTER_VALUE, .config.minmax = { 0, 0 }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_spi_id) },
    { "rx_spi_rf_channel_count",    VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 8 }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_spi_rf_channel_count) },
#endif
#ifdef SPEKTRUM_BIND
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { SPEKTRUM_SAT_BIND_DISABLED,  SPEKTRUM_SAT_BIND_MAX}, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind) },
#endif
    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN,  PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_min_usec) },
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_PULSE_MIN,  PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_max_usec) },
#ifdef STM32F4
    { "serialrx_halfduplex",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, halfDuplex) },
#endif

// PG_BLACKBOX_CONFIG
#ifdef BLACKBOX
    { "blackbox_rate_num",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1,  255 }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, rate_num) },
    { "blackbox_rate_denom",        VAR_UINT8  | MASTER_VALUE, .config.minmax = { 1,  255 }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, rate_denom) },
    { "blackbox_device",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_DEVICE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, device) },
#endif

#ifdef USE_SDCARD
    { "sdcard_detect_inverted",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, invertedCardDetection) },
#endif

// PG_MOTOR_CONFIG
    { "min_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, minthrottle) },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, maxthrottle) },
    { "min_command",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, mincommand) },
    { "motor_pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 50,  32000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, motorPwmRate) },
    { "motor_pwm_protocol",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MOTOR_PWM_PROTOCOL }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, motorPwmProtocol) },

// PG_FAILSAFE_CONFIG
    { "failsafe_delay",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_delay) },
    { "failsafe_recovery_delay",    VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_recovery_delay) },
    { "failsafe_off_delay",         VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_off_delay) },
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000, 2000 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle) },
    { "failsafe_throttle_low_delay",VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  300 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle_low_delay) },
    { "failsafe_procedure",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FAILSAFE_PROCEDURE }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_procedure) },
    { "failsafe_stick_threshold",   VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  500 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_stick_motion_threshold) },
    { "failsafe_fw_roll_angle",     VAR_INT16 | MASTER_VALUE, .config.minmax = { -800,  800 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_fw_roll_angle) },
    { "failsafe_fw_pitch_angle",    VAR_INT16 | MASTER_VALUE, .config.minmax = { -800,  800 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_fw_pitch_angle) },
    { "failsafe_fw_yaw_rate",       VAR_INT16 | MASTER_VALUE, .config.minmax = { -1000,  1000 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_fw_yaw_rate) },

// PG_BOARDALIGNMENT_CONFIG
    { "align_board_roll",           VAR_INT16  | MASTER_VALUE, .config.minmax = { -1800,  3600 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, rollDeciDegrees) },
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -1800,  3600 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, pitchDeciDegrees) },
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -1800,  3600 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, yawDeciDegrees) },

// PG_GIMBAL_CONFIG
#ifdef USE_SERVOS
    { "gimbal_mode",                VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GIMBAL_MODE }, PG_GIMBAL_CONFIG, offsetof(gimbalConfig_t, mode) },
#endif

// PG_BATTERY_CONFIG
    { "battery_capacity",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  20000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, batteryCapacity) },
#ifdef USE_ADC
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmax = { VBAT_SCALE_MIN,  VBAT_SCALE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatscale) },
    { "vbat_max_cell_voltage",      VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10,  50 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmaxcellvoltage) },
    { "vbat_min_cell_voltage",      VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10,  50 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmincellvoltage) },
    { "vbat_warning_cell_voltage",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10,  50 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatwarningcellvoltage) },
#endif
    { "current_meter_scale",        VAR_INT16  | MASTER_VALUE, .config.minmax = { -10000,  10000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, currentMeterScale) },
    { "current_meter_offset",       VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  3300 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, currentMeterOffset) },
    { "multiwii_current_meter_output", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, multiwiiCurrentMeterOutput) },
    { "current_meter_type",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CURRENT_SENSOR }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, currentMeterType) },

// PG_MIXER_CONFIG
    { "yaw_motor_direction",        VAR_INT8   | MASTER_VALUE, .config.minmax = { -1,  1 }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, yaw_motor_direction) },
    { "yaw_jump_prevention_limit",  VAR_UINT16 | MASTER_VALUE, .config.minmax = { YAW_JUMP_PREVENTION_LIMIT_LOW,  YAW_JUMP_PREVENTION_LIMIT_HIGH }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, yaw_jump_prevention_limit) },

// PG_MOTOR_3D_CONFIG
    { "3d_deadband_low",            VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_low) }, // FIXME upper limit should match code in the mixer, 1500 currently
    { "3d_deadband_high",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_high) }, // FIXME lower limit should match code in the mixer, 1500 currently,
    { "3d_neutral",                 VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, neutral3d) },

#ifdef USE_SERVOS
// PG_SERVO_CONFIG
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_SERVO_CONFIG, offsetof(servoConfig_t, servoCenterPulse) },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 50,  498 }, PG_SERVO_CONFIG, offsetof(servoConfig_t, servoPwmRate) },
    { "servo_lpf_hz",               VAR_INT16  | MASTER_VALUE, .config.minmax = { 0,  400}, PG_SERVO_CONFIG, offsetof(servoConfig_t, servo_lowpass_freq) },
    { "flaperon_throw_offset",      VAR_INT16  | MASTER_VALUE, .config.minmax = { FLAPERON_THROW_MIN,  FLAPERON_THROW_MAX}, PG_SERVO_CONFIG, offsetof(servoConfig_t, flaperon_throw_offset) },
    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SERVO_CONFIG, offsetof(servoConfig_t, tri_unarmed_servo) },
#endif

// PG_CONTROLRATE_PROFILE
    { "rc_expo",                    VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo8) },
    { "rc_yaw_expo",                VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcYawExpo8) },
    { "thr_mid",                    VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrMid8) },
    { "thr_expo",                   VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrExpo8) },

    // New rates are in dps/10. That means, Rate of 20 means 200dps of rotation speed on given axis.
    // Rate 180 (1800dps) is max. value gyro can measure reliably
    { "roll_rate",                  VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MIN,  CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_ROLL]) },
    { "pitch_rate",                 VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MIN,  CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_PITCH]) },
    { "yaw_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { CONTROL_RATE_CONFIG_YAW_RATE_MIN,  CONTROL_RATE_CONFIG_YAW_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_YAW]) },

    { "tpa_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, .config.minmax = { 0,  CONTROL_RATE_CONFIG_TPA_MAX}, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, dynThrPID) },
    { "tpa_breakpoint",             VAR_UINT16 | CONTROL_RATE_VALUE, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX}, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, tpa_breakpoint) },

// PG_SERIAL_CONFIG
    { "reboot_character",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 48,  126 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, reboot_character) },

// PG_IMU_CONFIG
    { "imu_dcm_kp",                 VAR_UINT16 | MASTER_VALUE | MODE_MAX, .config.max = { UINT16_MAX }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_kp_acc) },
    { "imu_dcm_ki",                 VAR_UINT16 | MASTER_VALUE | MODE_MAX, .config.max = { UINT16_MAX }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_ki_acc) },
    { "imu_dcm_kp_mag",             VAR_UINT16 | MASTER_VALUE | MODE_MAX, .config.max = { UINT16_MAX }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_kp_mag) },
    { "imu_dcm_ki_mag",             VAR_UINT16 | MASTER_VALUE | MODE_MAX, .config.max = { UINT16_MAX }, PG_IMU_CONFIG, offsetof(imuConfig_t, dcm_ki_mag) },
    { "small_angle",                VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  180 }, PG_IMU_CONFIG, offsetof(imuConfig_t, small_angle) },

// PG_ARMING_CONFIG
    { "fixed_wing_auto_arm",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ARMING_CONFIG, offsetof(armingConfig_t, fixed_wing_auto_arm) },
    { "disarm_kill_switch",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ARMING_CONFIG, offsetof(armingConfig_t, disarm_kill_switch) },
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  60 }, PG_ARMING_CONFIG, offsetof(armingConfig_t, auto_disarm_delay) },

// PG_GPS_CONFIG
#ifdef GPS
    { "gps_provider",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_PROVIDER }, PG_GPS_CONFIG, offsetof(gpsConfig_t, provider) },
    { "gps_sbas_mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_SBAS_MODE }, PG_GPS_CONFIG, offsetof(gpsConfig_t, sbasMode) },
    { "gps_dyn_model",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_DYN_MODEL }, PG_GPS_CONFIG, offsetof(gpsConfig_t, dynModel) },
    { "gps_auto_config",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, autoConfig) },
    { "gps_auto_baud",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_CONFIG, offsetof(gpsConfig_t, autoBaud) },
    { "gps_min_sats",               VAR_UINT8  | MASTER_VALUE, .config.minmax = { 5,  10}, PG_GPS_CONFIG, offsetof(gpsConfig_t, gpsMinSats) },
#endif

// PG_RC_CONTROLS_CONFIG
    { "deadband",                   VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  32 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, deadband) },
    { "yaw_deadband",               VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  100 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_deadband) },
    { "pos_hold_deadband",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10,  250 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, pos_hold_deadband) },
    { "alt_hold_deadband",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 10,  250 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, alt_hold_deadband) },
    { "3d_deadband_throttle",       VAR_UINT16 | MASTER_VALUE, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, deadband3d_throttle) },

// PG_PID_PROFILE
//    { "default_rate_profile",       VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  MAX_CONTROL_RATE_PROFILE_COUNT - 1 }, PG_PID_CONFIG, offsetof(pidProfile_t, defaultRateProfileIndex) },
    { "mc_p_pitch",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_PITCH].P) },
    { "mc_i_pitch",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_PITCH].I) },
    { "mc_d_pitch",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_PITCH].D) },
    { "mc_p_roll",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_ROLL].P) },
    { "mc_i_roll",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_ROLL].I) },
    { "mc_d_roll",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_ROLL].D) },
    { "mc_p_yaw",                   VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_YAW].P) },
    { "mc_i_yaw",                   VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_YAW].I) },
    { "mc_d_yaw",                   VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_YAW].D) },

    { "mc_p_level",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_LEVEL].P) },
    { "mc_i_level",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  100 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_LEVEL].I) },
    { "mc_d_level",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  100 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_LEVEL].D) },

    { "fw_p_pitch",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_PITCH].P) },
    { "fw_i_pitch",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_PITCH].I) },
    { "fw_ff_pitch",                VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_PITCH].D) },
    { "fw_p_roll",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_ROLL].P) },
    { "fw_i_roll",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_ROLL].I) },
    { "fw_ff_roll",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_ROLL].D) },
    { "fw_p_yaw",                   VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_YAW].P) },
    { "fw_i_yaw",                   VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_YAW].I) },
    { "fw_ff_yaw",                  VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  200 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_YAW].D) },

    { "fw_p_level",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_LEVEL].P) },
    { "fw_i_level",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  100 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_LEVEL].I) },
    { "fw_d_level",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  100 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_LEVEL].D) },

    { "max_angle_inclination_rll",  VAR_INT16  | PROFILE_VALUE, .config.minmax = { 100,  900 }, PG_PID_PROFILE, offsetof(pidProfile_t, max_angle_inclination[FD_ROLL]) },
    { "max_angle_inclination_pit",  VAR_INT16  | PROFILE_VALUE, .config.minmax = { 100,  900 }, PG_PID_PROFILE, offsetof(pidProfile_t, max_angle_inclination[FD_PITCH]) },

    { "dterm_lpf_hz",               VAR_UINT8  | PROFILE_VALUE, .config.minmax = {0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf_hz) },
    { "yaw_lpf_hz",                 VAR_UINT8  | PROFILE_VALUE, .config.minmax = {0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, yaw_lpf_hz) },
    { "dterm_setpoint_weight",      VAR_FLOAT  | PROFILE_VALUE, .config.minmax = {0, 2 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_setpoint_weight) },
#ifdef USE_SERVOS
    { "fw_iterm_throw_limit",       VAR_UINT16 | PROFILE_VALUE, .config.minmax = { FW_ITERM_THROW_LIMIT_MIN,  FW_ITERM_THROW_LIMIT_MAX}, PG_PID_PROFILE, offsetof(pidProfile_t, fixedWingItermThrowLimit) },
    { "fw_reference_airspeed",      VAR_FLOAT  | PROFILE_VALUE, .config.minmax = { 1,  5000}, PG_PID_PROFILE, offsetof(pidProfile_t, fixedWingReferenceAirspeed) },
    { "fw_turn_assist_yaw_gain",    VAR_FLOAT  | PROFILE_VALUE, .config.minmax = { 0,  2}, PG_PID_PROFILE, offsetof(pidProfile_t, fixedWingCoordinatedYawGain) },
#endif
#ifdef USE_DTERM_NOTCH
    { "dterm_notch_hz",             VAR_UINT16 | PROFILE_VALUE, .config.minmax = {0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_soft_notch_hz) },
    { "dterm_notch_cutoff",         VAR_UINT16 | PROFILE_VALUE, .config.minmax = {1, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_soft_notch_cutoff) },
#endif

    { "pidsum_limit",               VAR_UINT16 | PROFILE_VALUE, .config.minmax = { PID_SUM_LIMIT_MIN,  PID_SUM_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimit) },

    { "yaw_p_limit",                VAR_UINT16 | PROFILE_VALUE, .config.minmax = { YAW_P_LIMIT_MIN,  YAW_P_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, yaw_p_limit) },

    { "iterm_ignore_threshold",     VAR_UINT16 | PROFILE_VALUE, .config.minmax = {15, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, rollPitchItermIgnoreRate) },
    { "yaw_iterm_ignore_threshold", VAR_UINT16 | PROFILE_VALUE, .config.minmax = {15, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, yawItermIgnoreRate) },

    { "rate_accel_limit_roll_pitch",VAR_UINT32 | PROFILE_VALUE | MODE_MAX, .config.max = { 500000 }, PG_PID_PROFILE, offsetof(pidProfile_t, axisAccelerationLimitRollPitch) },
    { "rate_accel_limit_yaw",       VAR_UINT32 | PROFILE_VALUE | MODE_MAX, .config.max = { 500000 }, PG_PID_PROFILE, offsetof(pidProfile_t, axisAccelerationLimitYaw) },

    { "heading_hold_rate_limit",    VAR_UINT8  | PROFILE_VALUE, .config.minmax = { HEADING_HOLD_RATE_LIMIT_MIN,  HEADING_HOLD_RATE_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, heading_hold_rate_limit) },

#ifdef NAV
    { "nav_mc_pos_z_p",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_POS_Z].P) },
    { "nav_mc_pos_z_i",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_POS_Z].I) },
    { "nav_mc_pos_z_d",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_POS_Z].D) },

    { "nav_mc_vel_z_p",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_VEL_Z].P) },
    { "nav_mc_vel_z_i",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_VEL_Z].I) },
    { "nav_mc_vel_z_d",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_VEL_Z].D) },

    { "nav_mc_pos_xy_p",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_POS_XY].P) },
    { "nav_mc_pos_xy_i",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_POS_XY].I) },
    { "nav_mc_pos_xy_d",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_POS_XY].D) },
    { "nav_mc_vel_xy_p",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_VEL_XY].P) },
    { "nav_mc_vel_xy_i",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_VEL_XY].I) },
    { "nav_mc_vel_xy_d",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_mc.pid[PID_VEL_XY].D) },

    { "nav_fw_pos_z_p",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_POS_Z].P) },
    { "nav_fw_pos_z_i",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_POS_Z].I) },
    { "nav_fw_pos_z_d",             VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_POS_Z].D) },

    { "nav_fw_pos_xy_p",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_POS_XY].P) },
    { "nav_fw_pos_xy_i",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_POS_XY].I) },
    { "nav_fw_pos_xy_d",            VAR_UINT8  | PROFILE_VALUE, .config.minmax = { 0,  255 }, PG_PID_PROFILE, offsetof(pidProfile_t, bank_fw.pid[PID_POS_XY].D) },

// PG_PID_AUTOTUNE_CONFIG
    { "fw_autotune_overshoot_time", VAR_UINT16 | MASTER_VALUE, .config.minmax = { 50,  500 }, PG_PID_AUTOTUNE_CONFIG, offsetof(pidAutotuneConfig_t, fw_overshoot_time) },
    { "fw_autotune_undershoot_time",VAR_UINT16 | MASTER_VALUE, .config.minmax = { 50,  500 }, PG_PID_AUTOTUNE_CONFIG, offsetof(pidAutotuneConfig_t, fw_undershoot_time) },
    { "fw_autotune_threshold",      VAR_UINT8 | MASTER_VALUE, .config.minmax = { 0,  100 }, PG_PID_AUTOTUNE_CONFIG, offsetof(pidAutotuneConfig_t, fw_max_rate_threshold) },
    { "fw_autotune_ff_to_p_gain",   VAR_UINT8 | MASTER_VALUE, .config.minmax = { 0,  100 }, PG_PID_AUTOTUNE_CONFIG, offsetof(pidAutotuneConfig_t, fw_ff_to_p_gain) },
    { "fw_autotune_ff_to_i_tc",     VAR_UINT16 | MASTER_VALUE, .config.minmax = { 100,  5000 }, PG_PID_AUTOTUNE_CONFIG, offsetof(pidAutotuneConfig_t, fw_ff_to_i_time_constant) },

// PG_POSITION_ESTIMATION_CONFIG
#if defined(NAV_AUTO_MAG_DECLINATION)
    { "inav_auto_mag_decl",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, automatic_mag_declination) },
#endif
    { "inav_gravity_cal_tolerance", VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  255 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, gravity_calibration_tolerance) },
    { "inav_use_gps_velned",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, use_gps_velned) },
    { "inav_gps_delay",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  500 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, gps_delay_ms) },
    { "inav_reset_altitude",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_NAV_RESET_ALTITUDE }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, reset_altitude_type) },

    { "inav_max_surface_altitude",    VAR_UINT16  | MASTER_VALUE, .config.minmax = { 0,  1000 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, max_surface_altitude) },

    { "inav_w_z_surface_p",           VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_z_surface_p) },
    { "inav_w_z_surface_v",           VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_z_surface_v) },
    { "inav_w_z_baro_p",            VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_z_baro_p) },
    { "inav_w_z_gps_p",             VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_z_gps_p) },
    { "inav_w_z_gps_v",             VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_z_gps_v) },
    { "inav_w_xy_gps_p",            VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_xy_gps_p) },
    { "inav_w_xy_gps_v",            VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_xy_gps_v) },
    { "inav_w_z_res_v",             VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_z_res_v) },
    { "inav_w_xy_res_v",            VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_xy_res_v) },
    { "inav_w_acc_bias",            VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  1 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, w_acc_bias) },

    { "inav_max_eph_epv",           VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  9999 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, max_eph_epv) },
    { "inav_baro_epv",              VAR_FLOAT  | MASTER_VALUE, .config.minmax = { 0,  9999 }, PG_POSITION_ESTIMATION_CONFIG, offsetof(positionEstimationConfig_t, baro_epv) },

// PG_NAV_CONFIG
    { "nav_disarm_on_landing",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.disarm_on_landing) },
    { "nav_use_midthr_for_althold", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.use_thr_mid_for_althold) },
    { "nav_extra_arming_safety",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.extra_arming_safety) },
    { "nav_user_control_mode",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_NAV_USER_CTL_MODE }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.user_control_mode) },
    { "nav_position_timeout",       VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  10 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.pos_failure_timeout) },
    { "nav_wp_radius",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10,  10000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.waypoint_radius) },
    { "nav_wp_safe_distance",       VAR_UINT16 | MASTER_VALUE | MODE_MAX, .config.max = { 65000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.waypoint_safe_distance) },
    { "nav_auto_speed",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.max_auto_speed) },
    { "nav_auto_climb_rate",        VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.max_auto_climb_rate) },
    { "nav_manual_speed",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.max_manual_speed) },
    { "nav_manual_climb_rate",      VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.max_manual_climb_rate ) },
    { "nav_landing_speed",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { 100,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.land_descent_rate) },
    { "nav_land_slowdown_minalt",   VAR_UINT16 | MASTER_VALUE, .config.minmax = { 50,  1000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.land_slowdown_minalt) },
    { "nav_land_slowdown_maxalt",   VAR_UINT16 | MASTER_VALUE, .config.minmax = { 500,  4000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.land_slowdown_maxalt) },
    { "nav_emerg_landing_speed",    VAR_UINT16 | MASTER_VALUE, .config.minmax = { 100,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.emerg_descent_rate) },
    { "nav_min_rth_distance",       VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  5000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.min_rth_distance) },
    { "nav_rth_climb_first",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.rth_climb_first) },
    { "nav_rth_climb_ignore_emerg", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.rth_climb_ignore_emerg) },
    { "nav_rth_tail_first",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.rth_tail_first) },
    { "nav_rth_allow_landing",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.rth_allow_landing) },
    { "nav_rth_alt_mode",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_NAV_RTH_ALT_MODE }, PG_NAV_CONFIG, offsetof(navConfig_t, general.flags.rth_alt_control_mode) },
    { "nav_rth_abort_threshold",    VAR_UINT16 | MASTER_VALUE | MODE_MAX, .config.max = { 65000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.rth_abort_threshold) },
    { "nav_rth_altitude",           VAR_UINT16 | MASTER_VALUE | MODE_MAX, .config.max = { 65000 }, PG_NAV_CONFIG, offsetof(navConfig_t, general.rth_altitude) },

    { "nav_mc_bank_angle",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 15,  45 }, PG_NAV_CONFIG, offsetof(navConfig_t, mc.max_bank_angle) },
    { "nav_mc_hover_thr",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, mc.hover_throttle) },
    { "nav_mc_auto_disarm_delay",   VAR_UINT16 | MASTER_VALUE, .config.minmax = { 100,  10000 }, PG_NAV_CONFIG, offsetof(navConfig_t, mc.auto_disarm_delay) },

    { "nav_fw_cruise_thr",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.cruise_throttle) },
    { "nav_fw_min_thr",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.min_throttle) },
    { "nav_fw_max_thr",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.max_throttle) },
    { "nav_fw_bank_angle",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 5,  80 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.max_bank_angle) },
    { "nav_fw_climb_angle",         VAR_UINT8  | MASTER_VALUE, .config.minmax = { 5,  80 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.max_climb_angle) },
    { "nav_fw_dive_angle",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 5,  80 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.max_dive_angle) },
    { "nav_fw_pitch2thr",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  100 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.pitch_to_throttle) },
    { "nav_fw_loiter_radius",       VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  10000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.loiter_radius) },

#ifdef FIXED_WING_LANDING
    { "nav_fw_land_dive_angle",     VAR_INT8  | MASTER_VALUE, .config.minmax = { -20,  20 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.land_dive_angle) },
#endif

    { "nav_fw_launch_velocity",     VAR_UINT16 | MASTER_VALUE, .config.minmax = { 100,  10000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_velocity_thresh) },
    { "nav_fw_launch_accel",        VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000,  20000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_accel_thresh) },
    { "nav_fw_launch_max_angle",    VAR_UINT8  | MASTER_VALUE, .config.minmax = { 5,  180 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_max_angle) },
    { "nav_fw_launch_detect_time",  VAR_UINT16 | MASTER_VALUE, .config.minmax = { 10,  1000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_time_thresh) },
    { "nav_fw_launch_thr",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_throttle) },
    { "nav_fw_launch_idle_thr",     VAR_UINT16 | MASTER_VALUE, .config.minmax = { 1000,  2000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_idle_throttle) },
    { "nav_fw_launch_motor_delay",  VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  5000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_motor_timer) },
    { "nav_fw_launch_spinup_time",  VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0,  1000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_motor_spinup_time) },
    { "nav_fw_launch_timeout",      VAR_UINT16 | MASTER_VALUE | MODE_MAX , .config.max = { 60000 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_timeout) },
    { "nav_fw_launch_climb_angle",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 5,  45 }, PG_NAV_CONFIG, offsetof(navConfig_t, fw.launch_climb_angle) },
#endif

#ifdef TELEMETRY
// PG_TELEMETRY_CONFIG
    { "telemetry_switch",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, telemetry_switch) },
    { "telemetry_inversion",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, telemetry_inversion) },
    { "frsky_default_latitude",    VAR_FLOAT  | MASTER_VALUE, .config.minmax = { -90,  90 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLatitude) },
    { "frsky_default_longitude",    VAR_FLOAT  | MASTER_VALUE, .config.minmax = { -180, 180 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLongitude) },
    { "frsky_coordinates_format",   VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  FRSKY_FORMAT_NMEA }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_coordinate_format) },
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_unit) },
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE, .config.minmax = { FRSKY_VFAS_PRECISION_LOW,  FRSKY_VFAS_PRECISION_HIGH }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_vfas_precision) },
    { "frsky_vfas_cell_voltage",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_vfas_cell_voltage) },
    { "hott_alarm_sound_interval",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  120 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, hottAlarmSoundInterval) },
#ifdef TELEMETRY_SMARTPORT
    { "smartport_uart_unidir",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, smartportUartUnidirectional) },
#endif
    { "ibus_telemetry_type",       VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  255 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, ibusTelemetryType ) },
#ifdef TELEMETRY_LTM
    {"ltm_update_rate",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LTM_UPDATE_RATE }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, ltmUpdateRate) },
#endif
#endif
#ifdef USE_RX_ELERES
//PG_ELERES_CONFIG
    { "eleres_freq",                VAR_FLOAT | MASTER_VALUE, .config.minmax = { 415, 450 }, PG_ELERES_CONFIG, offsetof(eleresConfig_t, eleresFreq) },
    { "eleres_telemetry_en",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ELERES_CONFIG, offsetof(eleresConfig_t, eleresTelemetryEn) },
    { "eleres_telemetry_power",     VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 7 }, PG_ELERES_CONFIG, offsetof(eleresConfig_t, eleresTelemetryPower) },
    { "eleres_loc_en",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ELERES_CONFIG, offsetof(eleresConfig_t, eleresLocEn) },
    { "eleres_loc_power",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 7 }, PG_ELERES_CONFIG, offsetof(eleresConfig_t, eleresLocPower) },
    { "eleres_loc_delay",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 30, 1800 }, PG_ELERES_CONFIG, offsetof(eleresConfig_t, eleresLocDelay) },
    { "eleres_signature",           VAR_UINT32 | MASTER_VALUE | MODE_MAX , .config.max = { 4294967295 }, PG_ELERES_CONFIG, offsetof(eleresConfig_t, eleresSignature) },

#endif
#ifdef LED_STRIP
    { "ledstrip_visual_beeper",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_visual_beeper) },
#endif
#ifdef OSD
    { "osd_video_system",           VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 2 }, PG_OSD_CONFIG, offsetof(osdConfig_t, video_system) },
    { "osd_row_shiftdown",          VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 1 }, PG_OSD_CONFIG, offsetof(osdConfig_t, row_shiftdown) },
    { "osd_units",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_OSD_CONFIG, offsetof(osdConfig_t, units) },

    { "osd_rssi_alarm",             VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 100 }, PG_OSD_CONFIG, offsetof(osdConfig_t, rssi_alarm) },
    { "osd_cap_alarm",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 20000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, cap_alarm) },
    { "osd_time_alarm",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 60 }, PG_OSD_CONFIG, offsetof(osdConfig_t, time_alarm) },
    { "osd_alt_alarm",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, 10000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, alt_alarm) },

    { "osd_main_voltage_pos",       VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MAIN_BATT_VOLTAGE]) },
    { "osd_rssi_pos",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_RSSI_VALUE]) },
    { "osd_flytimer_pos",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_FLYTIME]) },
    { "osd_ontime_pos",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ONTIME]) },
    { "osd_flymode_pos",            VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_FLYMODE]) },
    { "osd_throttle_pos",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_THROTTLE_POS]) },
    { "osd_vtx_channel_pos",        VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_VTX_CHANNEL]) },
    { "osd_crosshairs",             VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CROSSHAIRS]) },
    { "osd_artificial_horizon",     VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ARTIFICIAL_HORIZON]) },
    { "osd_current_draw_pos",       VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CURRENT_DRAW]) },
    { "osd_mah_drawn_pos",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_MAH_DRAWN]) },
    { "osd_craft_name_pos",         VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_CRAFT_NAME]) },
    { "osd_gps_speed_pos",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_SPEED]) },
    { "osd_gps_sats_pos",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_SATS]) },
    { "osd_gps_lon",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_LON]) },
    { "osd_gps_lat",                VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_GPS_LAT]) },
    { "osd_home_dir",               VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_HOME_DIR]) },
    { "osd_home_dist",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_HOME_DIST]) },
    { "osd_altitude_pos",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ALTITUDE]) },
    { "osd_vario",                  VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_VARIO]) },
    { "osd_vario_num",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_VARIO_NUM]) },
    { "osd_pid_roll_pos",           VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_ROLL_PIDS]) },
    { "osd_pid_pitch_pos",          VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_PITCH_PIDS]) },
    { "osd_pid_yaw_pos",            VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_YAW_PIDS]) },
    { "osd_power_pos",              VAR_UINT16 | MASTER_VALUE, .config.minmax = { 0, OSD_POS_MAX_CLI }, PG_OSD_CONFIG, offsetof(osdConfig_t, item_pos[OSD_POWER]) },
#endif
// PG_SYSTEM_CONFIG
    { "i2c_overclock",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, i2c_overclock) },
    { "debug_mode",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DEBUG }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, debug_mode) },
#ifdef ASYNC_GYRO_PROCESSING
    { "acc_task_frequency",         VAR_UINT16 | MASTER_VALUE, .config.minmax = { ACC_TASK_FREQUENCY_MIN,  ACC_TASK_FREQUENCY_MAX }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, accTaskFrequency) },
    { "attitude_task_frequency",    VAR_UINT16 | MASTER_VALUE, .config.minmax = { ATTITUDE_TASK_FREQUENCY_MIN,  ATTITUDE_TASK_FREQUENCY_MAX }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, attitudeTaskFrequency) },
    { "async_mode",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ASYNC_MODE }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, asyncMode) },
#endif
    { "throttle_tilt_comp_str",     VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0,  100 }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, throttle_tilt_compensation_strength) },
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, pwmRxInputFilteringMode) },
    { "mode_range_logic_operator",  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_AUX_OPERATOR }, PG_MODE_ACTIVATION_OPERATOR_CONFIG, offsetof(modeActivationOperatorConfig_t, modeActivationOperator) },
//PG_STATS_CONFIG
#ifdef STATS
    { "stats",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_enabled) },
    { "stats_total_time",    VAR_UINT32 | MASTER_VALUE | MODE_MAX, .config.max = { INT32_MAX }, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_time) },
    { "stats_total_dist",    VAR_UINT32 | MASTER_VALUE | MODE_MAX, .config.max = { INT32_MAX }, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_dist) },
#endif
};


static void cliPrint(const char *str)
{
    while (*str) {
        bufWriterAppend(cliWriter, *str++);
    }
}

static void cliPrintLinefeed()
{
    cliPrint("\r\n");
}

#ifdef CLI_MINIMAL_VERBOSITY
#define cliPrintHashLine(str)
#else
static void cliPrintHashLine(const char *str)
{
    cliPrint("\r\n# ");
    cliPrint(str);
    cliPrint("\r\n");
}
#endif

static void cliPutp(void *p, char ch)
{
    bufWriterAppend(p, ch);
}

typedef enum {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_RATES = (1 << 2),
    DUMP_ALL = (1 << 3),
    DO_DIFF = (1 << 4),
    SHOW_DEFAULTS = (1 << 5),
    HIDE_UNUSED = (1 << 6)
} dumpFlags_e;

static bool cliDumpPrintf(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if (!((dumpMask & DO_DIFF) && equalsDefault)) {
        va_list va;
        va_start(va, format);
        tfp_format(cliWriter, cliPutp, format, va);
        va_end(va);

        return true;
    } else {
        return false;
    }
}

static void cliWrite(uint8_t ch)
{
    bufWriterAppend(cliWriter, ch);
}

static bool cliDefaultPrintf(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if ((dumpMask & SHOW_DEFAULTS) && !equalsDefault) {
        cliWrite('#');

        va_list va;
        va_start(va, format);
        tfp_format(cliWriter, cliPutp, format, va);
        va_end(va);

        return true;
    } else {
        return false;
    }
}

static void cliPrintf(const char *format, ...)
{
    va_list va;
    va_start(va, format);
    tfp_format(cliWriter, cliPutp, format, va);
    va_end(va);
}

static void printValuePointer(const clivalue_t *var, const void *valuePointer, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    switch (var->type & VALUE_TYPE_MASK) {
    case VAR_UINT8:
        value = *(uint8_t *)valuePointer;
        break;

    case VAR_INT8:
        value = *(int8_t *)valuePointer;
        break;

    case VAR_UINT16:
        value = *(uint16_t *)valuePointer;
        break;

    case VAR_INT16:
        value = *(int16_t *)valuePointer;
        break;

    case VAR_UINT32:
        value = *(uint32_t *)valuePointer;
        break;

    case VAR_FLOAT:
        cliPrintf("%s", ftoa(*(float *)valuePointer, buf));
        if (full) {
            if ((var->type & VALUE_MODE_MASK) == MODE_DIRECT) {
                cliPrintf(" %s", ftoa((float)var->config.minmax.min, buf));
                cliPrintf(" %s", ftoa((float)var->config.minmax.max, buf));
            } else if ((var->type & VALUE_MODE_MASK) == MODE_MAX) {
                cliPrintf("0 %s", ftoa((float)var->config.max.max, buf));
            }
        }
        return; // return from case for float only
    }

    switch (var->type & VALUE_MODE_MASK) {
    case MODE_MAX:
    case MODE_DIRECT:
        if ((var->type & VALUE_TYPE_MASK) == VAR_UINT32)
            cliPrintf("%u", value);
        else
            cliPrintf("%d", value);
        if (full) {
            if ((var->type & VALUE_MODE_MASK) == MODE_DIRECT) {
                cliPrintf(" %d %d", var->config.minmax.min, var->config.minmax.max);
            } else {
                if ((var->type & VALUE_TYPE_MASK) == VAR_UINT32)
                    cliPrintf(" 0 %u", var->config.max.max);
                else
                    cliPrintf(" 0 %d", var->config.max.max);
            }
        }
        break;
    case MODE_LOOKUP:
        if (var->config.lookup.tableIndex < ARRAYLEN(lookupTables)) {
            cliPrintf(lookupTables[var->config.lookup.tableIndex].values[value]);
        } else{
            cliPrintf("VALUE %s OUT OF RANGE\r\n", var->name);
        }
        break;
    }
}

static bool valuePtrEqualsDefault(uint8_t type, const void *ptr, const void *ptrDefault)
{
    bool result = false;
    switch (type & VALUE_TYPE_MASK) {
    case VAR_UINT8:
        result = *(uint8_t *)ptr == *(uint8_t *)ptrDefault;
        break;

    case VAR_INT8:
        result = *(int8_t *)ptr == *(int8_t *)ptrDefault;
        break;

    case VAR_UINT16:
        result = *(uint16_t *)ptr == *(uint16_t *)ptrDefault;
        break;

    case VAR_INT16:
        result = *(int16_t *)ptr == *(int16_t *)ptrDefault;
        break;

    case VAR_UINT32:
        result = *(uint32_t *)ptr == *(uint32_t *)ptrDefault;
        break;

    case VAR_FLOAT:
        result = *(float *)ptr == *(float *)ptrDefault;
        break;
    }
    return result;
}

static uint16_t getValueOffset(const clivalue_t *value)
{
    switch (value->type & VALUE_SECTION_MASK) {
    case MASTER_VALUE:
        return value->offset;
    case PROFILE_VALUE:
        return value->offset + sizeof(pidProfile_t) * getConfigProfile();
    case CONTROL_RATE_VALUE:
        return value->offset + sizeof(controlRateConfig_t) * getConfigProfile();
    }
    return 0;
}

static void *getValuePointer(const clivalue_t *value)
{
    const pgRegistry_t* pg = pgFind(value->pgn);
    return pg->address + getValueOffset(value);
}

static void dumpPgValue(const clivalue_t *value, uint8_t dumpMask)
{
    const pgRegistry_t* pg = pgFind(value->pgn);

    const char *format = "set %s = ";
    const char *defaultFormat = "#set %s = ";
    const int valueOffset = getValueOffset(value);
    const bool equalsDefault = valuePtrEqualsDefault(value->type, pg->copy + valueOffset, pg->address + valueOffset);
    if (((dumpMask & DO_DIFF) == 0) || !equalsDefault) {
        if (dumpMask & SHOW_DEFAULTS && !equalsDefault) {
            cliPrintf(defaultFormat, value->name);
            printValuePointer(value, (uint8_t*)pg->address + valueOffset, 0);
            cliPrintLinefeed();
        }
        cliPrintf(format, value->name);
        printValuePointer(value, pg->copy + valueOffset, 0);
        cliPrintLinefeed();
    }
}

static void dumpAllValues(uint16_t valueSection, uint8_t dumpMask)
{
    for (uint32_t i = 0; i < ARRAYLEN(valueTable); i++) {
        const clivalue_t *value = &valueTable[i];
        bufWriterFlush(cliWriter);
        if ((value->type & VALUE_SECTION_MASK) == valueSection) {
            dumpPgValue(value, dumpMask);
        }
    }
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    const void *ptr = getValuePointer(var);

    printValuePointer(var, ptr, full);
}

static void cliPrintVarRange(const clivalue_t *var)
{
    switch (var->type & VALUE_MODE_MASK) {
    case (MODE_DIRECT):
        cliPrintf("Allowed range: %d - %d\r\n", var->config.minmax.min, var->config.minmax.max);
        break;
    case (MODE_MAX):
        if ((var->type & VALUE_TYPE_MASK) == VAR_UINT32)
            cliPrintf("Allowed range: 0- %u\r\n", var->config.max.max);
        else
            cliPrintf("Allowed range: 0- %d\r\n", var->config.max.max);
        break;
    case (MODE_LOOKUP): {
        const lookupTableEntry_t *tableEntry = &lookupTables[var->config.lookup.tableIndex];
        cliPrint("Allowed values:");
        for (uint32_t i = 0; i < tableEntry->valueCount ; i++) {
            if (i > 0)
                cliPrint(",");
            cliPrintf(" %s", tableEntry->values[i]);
        }
        cliPrint("\r\n");
    }
    break;
    }
}

typedef union {
    uint32_t uint_value;
    int32_t int_value;
    float float_value;
} int_float_value_t;

static void cliSetVar(const clivalue_t *var, const int_float_value_t value)
{
    void *ptr = getValuePointer(var);

    switch (var->type & VALUE_TYPE_MASK) {
    case VAR_UINT8:
    case VAR_INT8:
        *(int8_t *)ptr = value.int_value;
        break;

    case VAR_UINT16:
    case VAR_INT16:
        *(int16_t *)ptr = value.int_value;
        break;

    case VAR_UINT32:
        *(uint32_t *)ptr = value.uint_value;
        break;

    case VAR_FLOAT:
        *(float *)ptr = (float)value.float_value;
        break;
    }
}

static void cliPrompt(void)
{
    cliPrint("\r\n# ");
    bufWriterFlush(cliWriter);
}

static void cliShowParseError(void)
{
    cliPrint("Parse error\r\n");
}

static void cliShowArgumentRangeError(char *name, int min, int max)
{
    cliPrintf("%s must be between %d and %d\r\n", name, min, max);
}

static const char *nextArg(const char *currentArg)
{
    const char *ptr = strchr(currentArg, ' ');
    while (ptr && *ptr == ' ') {
        ptr++;
    }

    return ptr;
}

static const char *processChannelRangeArgs(const char *ptr, channelRange_t *range, uint8_t *validArgumentCount)
{
    for (uint32_t argIndex = 0; argIndex < 2; argIndex++) {
        ptr = nextArg(ptr);
        if (ptr) {
            int val = atoi(ptr);
            val = CHANNEL_VALUE_TO_STEP(val);
            if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
                if (argIndex == 0) {
                    range->startStep = val;
                } else {
                    range->endStep = val;
                }
                (*validArgumentCount)++;
            }
        }
    }

    return ptr;
}

// Check if a string's length is zero
static bool isEmpty(const char *string)
{
    return (string == NULL || *string == '\0') ? true : false;
}

#if defined(USE_ASSERT)
static void cliAssert(char *cmdline)
{
    UNUSED(cmdline);

    if (assertFailureLine) {
        if (assertFailureFile) {
            cliPrintf("Assertion failed at line %d, file %s\r\n", assertFailureLine, assertFailureFile);
        }
        else {
            cliPrintf("Assertion failed at line %d\r\n", assertFailureLine);
        }
    }
    else {
        cliPrintf("No assert() failed\r\n");
    }
}
#endif

#if defined(BOOTLOG)
static void cliBootlog(char *cmdline)
{
    UNUSED(cmdline);

    int bootEventCount = getBootlogEventCount();

#if defined(BOOTLOG_DESCRIPTIONS)
    cliPrintf("Time Evt            Description  Parameters\r\n");
#else
    cliPrintf("Time Evt Parameters\r\n");
#endif

    for (int idx = 0; idx < bootEventCount; idx++) {
        bootLogEntry_t * event = getBootlogEvent(idx);

#if defined(BOOTLOG_DESCRIPTIONS)
        const char * eventDescription = getBootlogEventDescription(event->eventCode);
        if (!eventDescription) {
            eventDescription = "";
        }

        cliPrintf("%4d: %2d %22s ", event->timestamp, event->eventCode, eventDescription);
#else
        cliPrintf("%4d: %2d ", event->timestamp, event->eventCode);
#endif

        if (event->eventFlags & BOOT_EVENT_FLAGS_PARAM16) {
            cliPrintf(" (%d, %d, %d, %d)\r\n", event->params.u16[0], event->params.u16[1], event->params.u16[2], event->params.u16[3]);
        }
        else if (event->eventFlags & BOOT_EVENT_FLAGS_PARAM32) {
            cliPrintf(" (%d, %d)\r\n", event->params.u32[0], event->params.u32[1]);
        }
        else {
            cliPrintf("\r\n");
        }
    }
}
#endif

static void printAux(uint8_t dumpMask, const modeActivationCondition_t *modeActivationConditions, const modeActivationCondition_t *defaultModeActivationConditions)
{
    const char *format = "aux %u %u %u %u %u\r\n";
    // print out aux channel settings
    for (uint32_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = &modeActivationConditions[i];
        bool equalsDefault = false;
        if (defaultModeActivationConditions) {
            const modeActivationCondition_t *macDefault = &defaultModeActivationConditions[i];
            equalsDefault = mac->modeId == macDefault->modeId
                && mac->auxChannelIndex == macDefault->auxChannelIndex
                && mac->range.startStep == macDefault->range.startStep
                && mac->range.endStep == macDefault->range.endStep;
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                macDefault->modeId,
                macDefault->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(macDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(macDefault->range.endStep)
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            mac->modeId,
            mac->auxChannelIndex,
            MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep)
        );
    }
}

static void cliAux(char *cmdline)
{
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printAux(DUMP_MASTER, modeActivationConditions(0), NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            uint8_t validArgumentCount = 0;
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < CHECKBOX_ITEM_COUNT) {
                    mac->modeId = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    mac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &mac->range, &validArgumentCount);

            if (validArgumentCount != 4) {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_MODE_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}

static void printSerial(uint8_t dumpMask, const serialConfig_t *serialConfig, const serialConfig_t *serialConfigDefault)
{
    const char *format = "serial %d %d %ld %ld %ld %ld\r\n";
    for (uint32_t i = 0; i < SERIAL_PORT_COUNT; i++) {
        if (!serialIsPortAvailable(serialConfig->portConfigs[i].identifier)) {
            continue;
        };
        bool equalsDefault = false;
        if (serialConfigDefault) {
            equalsDefault = serialConfig->portConfigs[i].identifier == serialConfigDefault->portConfigs[i].identifier
                && serialConfig->portConfigs[i].functionMask == serialConfigDefault->portConfigs[i].functionMask
                && serialConfig->portConfigs[i].msp_baudrateIndex == serialConfigDefault->portConfigs[i].msp_baudrateIndex
                && serialConfig->portConfigs[i].gps_baudrateIndex == serialConfigDefault->portConfigs[i].gps_baudrateIndex
                && serialConfig->portConfigs[i].telemetry_baudrateIndex == serialConfigDefault->portConfigs[i].telemetry_baudrateIndex
                && serialConfig->portConfigs[i].blackbox_baudrateIndex == serialConfigDefault->portConfigs[i].blackbox_baudrateIndex;
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                serialConfigDefault->portConfigs[i].identifier,
                serialConfigDefault->portConfigs[i].functionMask,
                baudRates[serialConfigDefault->portConfigs[i].msp_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].gps_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].telemetry_baudrateIndex],
                baudRates[serialConfigDefault->portConfigs[i].blackbox_baudrateIndex]
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            serialConfig->portConfigs[i].identifier,
            serialConfig->portConfigs[i].functionMask,
            baudRates[serialConfig->portConfigs[i].msp_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].gps_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].telemetry_baudrateIndex],
            baudRates[serialConfig->portConfigs[i].blackbox_baudrateIndex]
            );
    }
}

static void cliSerial(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printSerial(DUMP_MASTER, serialConfig(), NULL);
        return;
    }
    serialPortConfig_t portConfig;
    memset(&portConfig, 0 , sizeof(portConfig));

    serialPortConfig_t *currentConfig;

    uint8_t validArgumentCount = 0;

    const char *ptr = cmdline;

    int val = atoi(ptr++);
    currentConfig = serialFindPortConfiguration(val);
    if (currentConfig) {
        portConfig.identifier = val;
        validArgumentCount++;
    }

    ptr = nextArg(ptr);
    if (ptr) {
        val = atoi(ptr);
        portConfig.functionMask = val & 0xFFFF;
        validArgumentCount++;
    }

    for (int i = 0; i < 4; i ++) {
        ptr = nextArg(ptr);
        if (!ptr) {
            break;
        }

        val = atoi(ptr);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch (i) {
            case 0:
                if (baudRateIndex < BAUD_1200 || baudRateIndex > BAUD_2470000) {
                    continue;
                }
                portConfig.msp_baudrateIndex = baudRateIndex;
                break;
            case 1:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.gps_baudrateIndex = baudRateIndex;
                break;
            case 2:
                if (baudRateIndex != BAUD_AUTO && baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.telemetry_baudrateIndex = baudRateIndex;
                break;
            case 3:
                if (baudRateIndex < BAUD_19200 || baudRateIndex > BAUD_250000) {
                    continue;
                }
                portConfig.blackbox_baudrateIndex = baudRateIndex;
                break;
        }

        validArgumentCount++;
    }

    if (validArgumentCount < 6) {
        cliShowParseError();
        return;
    }

    memcpy(currentConfig, &portConfig, sizeof(portConfig));
}

#ifdef USE_SERIAL_PASSTHROUGH
static void cliSerialPassthrough(char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliShowParseError();
        return;
    }

    int id = -1;
    uint32_t baud = 0;
    unsigned mode = 0;
    char* tok = strtok(cmdline, " ");
    int index = 0;

    while (tok != NULL) {
        switch (index) {
            case 0:
                id = atoi(tok);
                break;
            case 1:
                baud = atoi(tok);
                break;
            case 2:
                if (strstr(tok, "rx") || strstr(tok, "RX"))
                    mode |= MODE_RX;
                if (strstr(tok, "tx") || strstr(tok, "TX"))
                    mode |= MODE_TX;
                break;
        }
        index++;
        tok = strtok(NULL, " ");
    }

    serialPort_t *passThroughPort;
    serialPortUsage_t *passThroughPortUsage = findSerialPortUsageByIdentifier(id);
    if (!passThroughPortUsage || passThroughPortUsage->serialPort == NULL) {
        if (!baud) {
            printf("Port %d is closed, must specify baud.\r\n", id);
            return;
        }
        if (!mode)
            mode = MODE_RXTX;

        passThroughPort = openSerialPort(id, FUNCTION_NONE, NULL,
                                         baud, mode,
                                         SERIAL_NOT_INVERTED);
        if (!passThroughPort) {
            printf("Port %d could not be opened.\r\n", id);
            return;
        }
        printf("Port %d opened, baud = %d.\r\n", id, baud);
    } else {
        passThroughPort = passThroughPortUsage->serialPort;
        // If the user supplied a mode, override the port's mode, otherwise
        // leave the mode unchanged. serialPassthrough() handles one-way ports.
        printf("Port %d already open.\r\n", id);
        if (mode && passThroughPort->mode != mode) {
            printf("Adjusting mode from %d to %d.\r\n",
                   passThroughPort->mode, mode);
            serialSetMode(passThroughPort, mode);
        }
        // If this port has a rx callback associated we need to remove it now.
        // Otherwise no data will be pushed in the serial port buffer!
        if (passThroughPort->rxCallback) {
            printf("Removing rxCallback\r\n");
            passThroughPort->rxCallback = 0;
        }
    }

    printf("Forwarding data to %d, power cycle to exit.\r\n", id);

    serialPassthrough(cliPort, passThroughPort, NULL, NULL);
}
#endif

static void printAdjustmentRange(uint8_t dumpMask, const adjustmentRange_t *adjustmentRanges, const adjustmentRange_t *defaultAdjustmentRanges)
{
    const char *format = "adjrange %u %u %u %u %u %u %u\r\n";
     // print out adjustment ranges channel settings
    for (uint32_t i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
        const adjustmentRange_t *ar = &adjustmentRanges[i];
        bool equalsDefault = false;
        if (defaultAdjustmentRanges) {
            const adjustmentRange_t *arDefault = &defaultAdjustmentRanges[i];
            equalsDefault = ar->auxChannelIndex == arDefault->auxChannelIndex
                && ar->range.startStep == arDefault->range.startStep
                && ar->range.endStep == arDefault->range.endStep
                && ar->adjustmentFunction == arDefault->adjustmentFunction
                && ar->auxSwitchChannelIndex == arDefault->auxSwitchChannelIndex
                && ar->adjustmentIndex == arDefault->adjustmentIndex;
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                arDefault->adjustmentIndex,
                arDefault->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(arDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(arDefault->range.endStep),
                arDefault->adjustmentFunction,
                arDefault->auxSwitchChannelIndex
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            ar->adjustmentIndex,
            ar->auxChannelIndex,
            MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
            ar->adjustmentFunction,
            ar->auxSwitchChannelIndex
        );
    }
}

static void cliAdjustmentRange(char *cmdline)
{
    int i, val = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printAdjustmentRange(DUMP_MASTER, adjustmentRanges(0), NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = adjustmentRangesMutable(i);
            uint8_t validArgumentCount = 0;

            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                    ar->adjustmentIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }

            ptr = processChannelRangeArgs(ptr, &ar->range, &validArgumentCount);

            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < ADJUSTMENT_FUNCTION_COUNT) {
                    ar->adjustmentFunction = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxSwitchChannelIndex = val;
                    validArgumentCount++;
                }
            }

            if (validArgumentCount != 6) {
                memset(ar, 0, sizeof(adjustmentRange_t));
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_ADJUSTMENT_RANGE_COUNT - 1);
        }
    }
}

#ifndef USE_QUAD_MIXER_ONLY
static void printMotorMix(uint8_t dumpMask, const motorMixer_t *customMotorMixer, const motorMixer_t *defaultCustomMotorMixer)
{
    const char *format = "mmix %d %s %s %s %s\r\n";
    char buf0[8];
    char buf1[8];
    char buf2[8];
    char buf3[8];
    for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (customMotorMixer[i].throttle == 0.0f)
            break;
        const float thr = customMotorMixer[i].throttle;
        const float roll = customMotorMixer[i].roll;
        const float pitch = customMotorMixer[i].pitch;
        const float yaw = customMotorMixer[i].yaw;
        bool equalsDefault = false;
        if (defaultCustomMotorMixer) {
            const float thrDefault = defaultCustomMotorMixer[i].throttle;
            const float rollDefault = defaultCustomMotorMixer[i].roll;
            const float pitchDefault = defaultCustomMotorMixer[i].pitch;
            const float yawDefault = defaultCustomMotorMixer[i].yaw;
            const bool equalsDefault = thr == thrDefault && roll == rollDefault && pitch == pitchDefault && yaw == yawDefault;

            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                ftoa(thrDefault, buf0),
                ftoa(rollDefault, buf1),
                ftoa(pitchDefault, buf2),
                ftoa(yawDefault, buf3));
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            ftoa(thr, buf0),
            ftoa(roll, buf1),
            ftoa(pitch, buf2),
            ftoa(yaw, buf3));
    }
}

static void cliMotorMix(char *cmdline)
{
    int check = 0;
    uint8_t len;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printMotorMix(DUMP_MASTER, customMotorMixer(0), NULL);
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            customMotorMixerMutable(i)->throttle = 0.0f;
        }
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = nextArg(cmdline);
        if (ptr) {
            len = strlen(ptr);
            for (uint32_t i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrint("Invalid name\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i, customMotorMixerMutable(0));
                    cliPrintf("Loaded %s\r\n", mixerNames[i]);
                    cliMotorMix("");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        uint32_t i = atoi(ptr); // get motor number
        if (i < MAX_SUPPORTED_MOTORS) {
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->throttle = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->roll = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->pitch = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixerMutable(i)->yaw = fastA2F(ptr);
                check++;
            }
            if (check != 4) {
                cliShowParseError();
            } else {
                printMotorMix(DUMP_MASTER, customMotorMixer(0), NULL);
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_SUPPORTED_MOTORS - 1);
        }
    }
}
#endif // USE_QUAD_MIXER_ONLY

static void printRxRange(uint8_t dumpMask, const rxChannelRangeConfig_t *channelRangeConfigs, const rxChannelRangeConfig_t *defaultChannelRangeConfigs)
{
    const char *format = "rxrange %u %u %u\r\n";
    for (uint32_t i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        bool equalsDefault = false;
        if (defaultChannelRangeConfigs) {
            equalsDefault = channelRangeConfigs[i].min == defaultChannelRangeConfigs[i].min
                && channelRangeConfigs[i].max == defaultChannelRangeConfigs[i].max;
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                defaultChannelRangeConfigs[i].min,
                defaultChannelRangeConfigs[i].max
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            channelRangeConfigs[i].min,
            channelRangeConfigs[i].max
        );
    }
}

static void cliRxRange(char *cmdline)
{
    int i, validArgumentCount = 0;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printRxRange(DUMP_MASTER, rxChannelRangeConfigs(0), NULL);
    } else if (strcasecmp(cmdline, "reset") == 0) {
        resetAllRxChannelRangeConfigurations();
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i >= 0 && i < NON_AUX_CHANNEL_COUNT) {
            int rangeMin, rangeMax;

            ptr = nextArg(ptr);
            if (ptr) {
                rangeMin = atoi(ptr);
                validArgumentCount++;
            }

            ptr = nextArg(ptr);
            if (ptr) {
                rangeMax = atoi(ptr);
                validArgumentCount++;
            }

            if (validArgumentCount != 2) {
                cliShowParseError();
            } else if (rangeMin < PWM_PULSE_MIN || rangeMin > PWM_PULSE_MAX || rangeMax < PWM_PULSE_MIN || rangeMax > PWM_PULSE_MAX) {
                cliShowParseError();
            } else {
                rxChannelRangeConfig_t *channelRangeConfig = rxChannelRangeConfigsMutable(i);
                channelRangeConfig->min = rangeMin;
                channelRangeConfig->max = rangeMax;
            }
        } else {
            cliShowArgumentRangeError("channel", 0, NON_AUX_CHANNEL_COUNT - 1);
        }
    }
}

#ifdef LED_STRIP
static void printLed(uint8_t dumpMask, const ledConfig_t *ledConfigs, const ledConfig_t *defaultLedConfigs)
{
    const char *format = "led %u %s\r\n";
    char ledConfigBuffer[20];
    char ledConfigDefaultBuffer[20];
    for (uint32_t i = 0; i < LED_MAX_STRIP_LENGTH; i++) {
        ledConfig_t ledConfig = ledConfigs[i];
        generateLedConfig(&ledConfig, ledConfigBuffer, sizeof(ledConfigBuffer));
        bool equalsDefault = false;
        if (defaultLedConfigs) {
            ledConfig_t ledConfigDefault = defaultLedConfigs[i];
            equalsDefault = ledConfig == ledConfigDefault;
            generateLedConfig(&ledConfigDefault, ledConfigDefaultBuffer, sizeof(ledConfigDefaultBuffer));
            cliDefaultPrintf(dumpMask, equalsDefault, format, i, ledConfigDefaultBuffer);
        }
        cliDumpPrintf(dumpMask, equalsDefault, format, i, ledConfigBuffer);
    }
}

static void cliLed(char *cmdline)
{
    int i;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printLed(DUMP_MASTER, ledStripConfig()->ledConfigs, NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < LED_MAX_STRIP_LENGTH) {
            ptr = nextArg(cmdline);
            if (!parseLedStripConfig(i, ptr)) {
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, LED_MAX_STRIP_LENGTH - 1);
        }
    }
}

static void printColor(uint8_t dumpMask, const hsvColor_t *colors, const hsvColor_t *defaultColors)
{
    const char *format = "color %u %d,%u,%u\r\n";
    for (uint32_t i = 0; i < LED_CONFIGURABLE_COLOR_COUNT; i++) {
        const hsvColor_t *color = &colors[i];
        bool equalsDefault = false;
        if (defaultColors) {
            const hsvColor_t *colorDefault = &defaultColors[i];
            equalsDefault = color->h == colorDefault->h
                && color->s == colorDefault->s
                && color->v == colorDefault->v;
            cliDefaultPrintf(dumpMask, equalsDefault, format, i,colorDefault->h, colorDefault->s, colorDefault->v);
        }
        cliDumpPrintf(dumpMask, equalsDefault, format, i, color->h, color->s, color->v);
    }
}

static void cliColor(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printColor(DUMP_MASTER, ledStripConfig()->colors, NULL);
    } else {
        const char *ptr = cmdline;
        const int i = atoi(ptr);
        if (i < LED_CONFIGURABLE_COLOR_COUNT) {
            ptr = nextArg(cmdline);
            if (!parseColor(i, ptr)) {
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, LED_CONFIGURABLE_COLOR_COUNT - 1);
        }
    }
}

static void printModeColor(uint8_t dumpMask, const ledStripConfig_t *ledStripConfig, const ledStripConfig_t *defaultLedStripConfig)
{
    const char *format = "mode_color %u %u %u\r\n";
    for (uint32_t i = 0; i < LED_MODE_COUNT; i++) {
        for (uint32_t j = 0; j < LED_DIRECTION_COUNT; j++) {
            int colorIndex = ledStripConfig->modeColors[i].color[j];
            bool equalsDefault = false;
            if (defaultLedStripConfig) {
                int colorIndexDefault = defaultLedStripConfig->modeColors[i].color[j];
                equalsDefault = colorIndex == colorIndexDefault;
                cliDefaultPrintf(dumpMask, equalsDefault, format, i, j, colorIndexDefault);
            }
            cliDumpPrintf(dumpMask, equalsDefault, format, i, j, colorIndex);
        }
    }

    for (uint32_t j = 0; j < LED_SPECIAL_COLOR_COUNT; j++) {
        const int colorIndex = ledStripConfig->specialColors.color[j];
        bool equalsDefault = false;
        if (defaultLedStripConfig) {
            const int colorIndexDefault = defaultLedStripConfig->specialColors.color[j];
            equalsDefault = colorIndex == colorIndexDefault;
            cliDefaultPrintf(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndexDefault);
        }
        cliDumpPrintf(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndex);
    }
}

static void cliModeColor(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printModeColor(DUMP_MASTER, ledStripConfig(), NULL);
    } else {
        enum {MODE = 0, FUNCTION, COLOR, ARGS_COUNT};
        int args[ARGS_COUNT];
        int argNo = 0;
        const char* ptr = strtok(cmdline, " ");
        while (ptr && argNo < ARGS_COUNT) {
            args[argNo++] = atoi(ptr);
            ptr = strtok(NULL, " ");
        }

        if (ptr != NULL || argNo != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        int modeIdx  = args[MODE];
        int funIdx = args[FUNCTION];
        int color = args[COLOR];
        if (!setModeColor(modeIdx, funIdx, color)) {
            cliShowParseError();
            return;
        }
        // values are validated
        cliPrintf("mode_color %u %u %u\r\n", modeIdx, funIdx, color);
    }
}
#endif

#ifdef USE_SERVOS
static void printServo(uint8_t dumpMask, const servoParam_t *servoParam, const servoParam_t *defaultServoParam)
{
    // print out servo settings
    const char *format = "servo %u %d %d %d %d %d \r\n";
    for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const servoParam_t *servoConf = &servoParam[i];
        bool equalsDefault = false;
        if (defaultServoParam) {
            const servoParam_t *servoConfDefault = &defaultServoParam[i];
            equalsDefault = servoConf->min == servoConfDefault->min
                && servoConf->max == servoConfDefault->max
                && servoConf->middle == servoConfDefault->middle
                && servoConf->rate == servoConfDefault->rate
                && servoConf->forwardFromChannel == servoConfDefault->forwardFromChannel;
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                servoConfDefault->min,
                servoConfDefault->max,
                servoConfDefault->middle,
                servoConfDefault->rate,
                servoConfDefault->forwardFromChannel
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            servoConf->min,
            servoConf->max,
            servoConf->middle,
            servoConf->rate,
            servoConf->forwardFromChannel
        );
    }

    // print servo directions
    if (defaultServoParam) {
        for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            const servoParam_t *servoConf = &servoParam[i];
            const servoParam_t *servoConfDefault = &defaultServoParam[i];
            bool equalsDefault = servoConf->reversedSources == servoConfDefault->reversedSources;
            for (uint32_t channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                equalsDefault = ~(servoConf->reversedSources ^ servoConfDefault->reversedSources) & (1 << channel);
                const char *format = "smix reverse %d %d r\r\n";
                if (servoConfDefault->reversedSources & (1 << channel)) {
                    cliDefaultPrintf(dumpMask, equalsDefault, format, i , channel);
                }
                if (servoConf->reversedSources & (1 << channel)) {
                    cliDumpPrintf(dumpMask, equalsDefault, format, i , channel);
                }
            }
        }
    }
}

static void cliServo(char *cmdline)
{
    enum { SERVO_ARGUMENT_COUNT = 6 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    servoParam_t *servo;

    int i;
    const char *ptr;

    if (isEmpty(cmdline)) {
        printServo(DUMP_MASTER, servoParams(0), NULL);
    } else {
        int validArgumentCount = 0;

        ptr = cmdline;

        // Command line is integers (possibly negative) separated by spaces, no other characters allowed.

        // If command line doesn't fit the format, don't modify the config
        while (*ptr) {
            if (*ptr == '-' || (*ptr >= '0' && *ptr <= '9')) {
                if (validArgumentCount >= SERVO_ARGUMENT_COUNT) {
                    cliShowParseError();
                    return;
                }

                arguments[validArgumentCount++] = atoi(ptr);

                do {
                    ptr++;
                } while (*ptr >= '0' && *ptr <= '9');
            } else if (*ptr == ' ') {
                ptr++;
            } else {
                cliShowParseError();
                return;
            }
        }

        enum {INDEX = 0, MIN, MAX, MIDDLE, RATE, FORWARD};

        i = arguments[INDEX];

        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if (validArgumentCount != SERVO_ARGUMENT_COUNT || i < 0 || i >= MAX_SUPPORTED_SERVOS) {
            cliShowParseError();
            return;
        }

        servo = servoParamsMutable(i);

        if (
            arguments[MIN] < PWM_PULSE_MIN || arguments[MIN] > PWM_PULSE_MAX ||
            arguments[MAX] < PWM_PULSE_MIN || arguments[MAX] > PWM_PULSE_MAX ||
            arguments[MIDDLE] < arguments[MIN] || arguments[MIDDLE] > arguments[MAX] ||
            arguments[MIN] > arguments[MAX] || arguments[MAX] < arguments[MIN] ||
            arguments[RATE] < -100 || arguments[RATE] > 100 ||
            arguments[FORWARD] >= MAX_SUPPORTED_RC_CHANNEL_COUNT
        ) {
            cliShowParseError();
            return;
        }

        servo->min = arguments[MIN];
        servo->max = arguments[MAX];
        servo->middle = arguments[MIDDLE];
        servo->rate = arguments[RATE];
        servo->forwardFromChannel = arguments[FORWARD];
    }
}

static void printServoMix(uint8_t dumpMask, const servoMixer_t *customServoMixers, const servoMixer_t *defaultCustomServoMixers)
{
    const char *format = "smix %d %d %d %d %d\r\n";
    for (uint32_t i = 0; i < MAX_SERVO_RULES; i++) {
        servoMixer_t customServoMixer = customServoMixers[i];
        if (customServoMixer.rate == 0) {
            break;
        }

        bool equalsDefault = false;
        if (defaultCustomServoMixers) {
            servoMixer_t customServoMixerDefault = defaultCustomServoMixers[i];
            equalsDefault = customServoMixer.targetChannel == customServoMixerDefault.targetChannel
                && customServoMixer.inputSource == customServoMixerDefault.inputSource
                && customServoMixer.rate == customServoMixerDefault.rate
                && customServoMixer.speed == customServoMixerDefault.speed;

            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                customServoMixerDefault.targetChannel,
                customServoMixerDefault.inputSource,
                customServoMixerDefault.rate,
                customServoMixerDefault.speed
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            customServoMixer.targetChannel,
            customServoMixer.inputSource,
            customServoMixer.rate,
            customServoMixer.speed
        );
    }
}

static void cliServoMix(char *cmdline)
{
    uint8_t len;
    const char *ptr;
    int args[8], check = 0;
    len = strlen(cmdline);

    if (len == 0) {
        printServoMix(DUMP_MASTER, customServoMixers(0), NULL);
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        pgResetCopy(customServoMixersMutable(0), PG_SERVO_MIXER);
        for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoParamsMutable(i)->reversedSources = 0;
        }
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = nextArg(cmdline);
        if (ptr) {
            len = strlen(ptr);
            for (uint32_t i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrintf("Invalid name\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    servoMixerLoadMix(i);
                    cliPrintf("Loaded %s\r\n", mixerNames[i]);
                    cliServoMix("");
                    break;
                }
            }
        }
    } else if (strncasecmp(cmdline, "reverse", 7) == 0) {
        enum {SERVO = 0, INPUT, REVERSE, ARGS_COUNT};
        char *ptr = strchr(cmdline, ' ');

        len = strlen(ptr);
        if (len == 0) {
            cliPrintf("s");
            for (uint32_t inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                cliPrintf("\ti%d", inputSource);
            cliPrintf("\r\n");

            for (uint32_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
                cliPrintf("%d", servoIndex);
                for (uint32_t inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                    cliPrintf("\t%s  ", (servoParams(servoIndex)->reversedSources & (1 << inputSource)) ? "r" : "n");
                cliPrintf("\r\n");
            }
            return;
        }

        ptr = strtok(ptr, " ");
        while (ptr != NULL && check < ARGS_COUNT - 1) {
            args[check++] = atoi(ptr);
            ptr = strtok(NULL, " ");
        }

        if (ptr == NULL || check != ARGS_COUNT - 1) {
            cliShowParseError();
            return;
        }

        if (args[SERVO] >= 0 && args[SERVO] < MAX_SUPPORTED_SERVOS
                && args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT
                && (*ptr == 'r' || *ptr == 'n')) {
            if (*ptr == 'r')
                servoParamsMutable(args[SERVO])->reversedSources |= 1 << args[INPUT];
            else
                servoParamsMutable(args[SERVO])->reversedSources &= ~(1 << args[INPUT]);
        } else
            cliShowParseError();

        cliServoMix("reverse");
    } else {
        enum {RULE = 0, TARGET, INPUT, RATE, SPEED, ARGS_COUNT};
        ptr = strtok(cmdline, " ");
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = atoi(ptr);
            ptr = strtok(NULL, " ");
        }

        if (ptr != NULL || check != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        int32_t i = args[RULE];
        if (i >= 0 && i < MAX_SERVO_RULES &&
            args[TARGET] >= 0 && args[TARGET] < MAX_SUPPORTED_SERVOS &&
            args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT &&
            args[RATE] >= -125 && args[RATE] <= 125 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED) {
            customServoMixersMutable(i)->targetChannel = args[TARGET];
            customServoMixersMutable(i)->inputSource = args[INPUT];
            customServoMixersMutable(i)->rate = args[RATE];
            customServoMixersMutable(i)->speed = args[SPEED];
            cliServoMix("");
        } else {
            cliShowParseError();
        }
    }
}
#endif // USE_SERVOS

#ifdef USE_SDCARD

static void cliWriteBytes(const uint8_t *buffer, int count)
{
    while (count > 0) {
        cliWrite(*buffer);
        buffer++;
        count--;
    }
}

static void cliSdInfo(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("SD card: ");

    if (!sdcard_isInserted()) {
        cliPrint("None inserted\r\n");
        return;
    }

    if (!sdcard_isInitialized()) {
        cliPrint("Startup failed\r\n");
        return;
    }

    const sdcardMetadata_t *metadata = sdcard_getMetadata();

    cliPrintf("Manufacturer 0x%x, %ukB, %02d/%04d, v%d.%d, '",
        metadata->manufacturerID,
        metadata->numBlocks / 2, /* One block is half a kB */
        metadata->productionMonth,
        metadata->productionYear,
        metadata->productRevisionMajor,
        metadata->productRevisionMinor
    );

    cliWriteBytes((uint8_t*)metadata->productName, sizeof(metadata->productName));

    cliPrint("'\r\n" "Filesystem: ");

    switch (afatfs_getFilesystemState()) {
        case AFATFS_FILESYSTEM_STATE_READY:
            cliPrint("Ready");
        break;
        case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
            cliPrint("Initializing");
        break;
        case AFATFS_FILESYSTEM_STATE_UNKNOWN:
        case AFATFS_FILESYSTEM_STATE_FATAL:
            cliPrint("Fatal");

            switch (afatfs_getLastError()) {
                case AFATFS_ERROR_BAD_MBR:
                    cliPrint(" - no FAT MBR partitions");
                break;
                case AFATFS_ERROR_BAD_FILESYSTEM_HEADER:
                    cliPrint(" - bad FAT header");
                break;
                case AFATFS_ERROR_GENERIC:
                case AFATFS_ERROR_NONE:
                    ; // Nothing more detailed to print
                break;
            }
        break;
    }
    cliPrint("\r\n");
}

#endif

#ifdef USE_FLASHFS

static void cliFlashInfo(char *cmdline)
{
    const flashGeometry_t *layout = flashfsGetGeometry();

    UNUSED(cmdline);

    cliPrintf("Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u, usedSize=%u\r\n",
            layout->sectors, layout->sectorSize, layout->pagesPerSector, layout->pageSize, layout->totalSize, flashfsGetOffset());
}

static void cliFlashErase(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintf("Erasing...\r\n");
    flashfsEraseCompletely();

    while (!flashfsIsReady()) {
        delay(100);
    }

    cliPrintf("Done.\r\n");
}

#ifdef USE_FLASH_TOOLS

static void cliFlashWrite(char *cmdline)
{
    uint32_t address = atoi(cmdline);
    char *text = strchr(cmdline, ' ');

    if (!text) {
        cliShowParseError();
    } else {
        flashfsSeekAbs(address);
        flashfsWrite((uint8_t*)text, strlen(text), true);
        flashfsFlushSync();

        cliPrintf("Wrote %u bytes at %u.\r\n", strlen(text), address);
    }
}

static void cliFlashRead(char *cmdline)
{
    uint32_t address = atoi(cmdline);
    uint32_t length;

    uint8_t buffer[32];

    char *nextArg = strchr(cmdline, ' ');

    if (!nextArg) {
        cliShowParseError();
    } else {
        length = atoi(nextArg);

        cliPrintf("Reading %u bytes at %u:\r\n", length, address);

        while (length > 0) {
            int bytesRead = flashfsReadAbs(address, buffer, length < sizeof(buffer) ? length : sizeof(buffer));

            for (int i = 0; i < bytesRead; i++) {
                cliWrite(buffer[i]);
            }

            length -= bytesRead;
            address += bytesRead;

            if (bytesRead == 0) {
                //Assume we reached the end of the volume or something fatal happened
                break;
            }
        }
        cliPrintf("\r\n");
    }
}

#endif
#endif


static void printFeature(uint8_t dumpMask, const featureConfig_t *featureConfig, const featureConfig_t *featureConfigDefault)
{
    uint32_t mask = featureConfig->enabledFeatures;
    uint32_t defaultMask = featureConfigDefault->enabledFeatures;
    for (uint32_t i = 0; ; i++) { // disable all feature first
        if (featureNames[i] == NULL)
            break;
        if (featureNames[i][0] == '\0')
            continue;
        const char *format = "feature -%s\r\n";
        cliDefaultPrintf(dumpMask, (defaultMask | ~mask) & (1 << i), format, featureNames[i]);
        cliDumpPrintf(dumpMask, (~defaultMask | mask) & (1 << i), format, featureNames[i]);
    }
    for (uint32_t i = 0; ; i++) {  // reenable what we want.
        if (featureNames[i] == NULL)
            break;
        if (featureNames[i][0] == '\0')
            continue;
        const char *format = "feature %s\r\n";
        if (defaultMask & (1 << i)) {
            cliDefaultPrintf(dumpMask, (~defaultMask | mask) & (1 << i), format, featureNames[i]);
        }
        if (mask & (1 << i)) {
            cliDumpPrintf(dumpMask, (defaultMask | ~mask) & (1 << i), format, featureNames[i]);
        }
    }
}

static void cliFeature(char *cmdline)
{
    uint32_t len = strlen(cmdline);
    uint32_t mask = featureMask();

    if (len == 0) {
        cliPrint("Enabled: ");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (featureNames[i][0] == '\0')
                continue;
            if (mask & (1 << i))
                cliPrintf("%s ", featureNames[i]);
        }
        cliPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available: ");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (featureNames[i][0] == '\0')
                continue;
            cliPrintf("%s ", featureNames[i]);
        }
        cliPrint("\r\n");
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            // remove feature
            remove = true;
            cmdline++; // skip over -
            len--;
        }

        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                cliPrint("Invalid name\r\n");
                break;
            }

            if (strncasecmp(cmdline, featureNames[i], len) == 0) {

                mask = 1 << i;
#ifndef GPS
                if (mask & FEATURE_GPS) {
                    cliPrint("unavailable\r\n");
                    break;
                }
#endif
                if (remove) {
                    featureClear(mask);
                    cliPrint("Disabled");
                } else {
                    featureSet(mask);
                    cliPrint("Enabled");
                }
                cliPrintf(" %s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

#ifdef BEEPER
static void printBeeper(uint8_t dumpMask, const beeperConfig_t *beeperConfig, const beeperConfig_t *beeperConfigDefault)
{
    const uint8_t beeperCount = beeperTableEntryCount();
    const uint32_t mask = beeperConfig->beeper_off_flags;
    const uint32_t defaultMask = beeperConfigDefault->beeper_off_flags;
    for (int i = 0; i < beeperCount - 2; i++) {
        const char *formatOff = "beeper -%s\r\n";
        const char *formatOn = "beeper %s\r\n";
        cliDefaultPrintf(dumpMask, ~(mask ^ defaultMask) & (1 << i), mask & (1 << i) ? formatOn : formatOff, beeperNameForTableIndex(i));
        cliDumpPrintf(dumpMask, ~(mask ^ defaultMask) & (1 << i), mask & (1 << i) ? formatOff : formatOn, beeperNameForTableIndex(i));
    }
}

static void cliBeeper(char *cmdline)
{
    uint32_t len = strlen(cmdline);
    uint8_t beeperCount = beeperTableEntryCount();
    uint32_t mask = getBeeperOffMask();

    if (len == 0) {
        cliPrintf("Disabled:");
        for (int32_t i = 0; ; i++) {
            if (i == beeperCount - 2){
                if (mask == 0)
                    cliPrint("  none");
                break;
            }
            if (mask & (1 << (beeperModeForTableIndex(i) - 1)))
                cliPrintf("  %s", beeperNameForTableIndex(i));
        }
        cliPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available:");
        for (uint32_t i = 0; i < beeperCount; i++)
            cliPrintf("  %s", beeperNameForTableIndex(i));
        cliPrint("\r\n");
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            remove = true;     // this is for beeper OFF condition
            cmdline++;
            len--;
        }

        for (uint32_t i = 0; ; i++) {
            if (i == beeperCount) {
                cliPrint("Invalid name\r\n");
                break;
            }
            if (strncasecmp(cmdline, beeperNameForTableIndex(i), len) == 0) {
                if (remove) { // beeper off
                    if (i == BEEPER_ALL-1)
                        beeperOffSetAll(beeperCount-2);
                    else
                        if (i == BEEPER_PREFERENCE-1)
                            setBeeperOffMask(getPreferredBeeperOffMask());
                        else {
                            mask = 1 << (beeperModeForTableIndex(i) - 1);
                            beeperOffSet(mask);
                        }
                    cliPrint("Disabled");
                }
                else { // beeper on
                    if (i == BEEPER_ALL-1)
                        beeperOffClearAll();
                    else
                        if (i == BEEPER_PREFERENCE-1)
                            setPreferredBeeperOffMask(getBeeperOffMask());
                        else {
                            mask = 1 << (beeperModeForTableIndex(i) - 1);
                            beeperOffClear(mask);
                        }
                    cliPrint("Enabled");
                }
            cliPrintf(" %s\r\n", beeperNameForTableIndex(i));
            break;
            }
        }
    }
}
#endif

static void printMap(uint8_t dumpMask, const rxConfig_t *rxConfig, const rxConfig_t *defaultRxConfig)
{
    bool equalsDefault = true;
    char buf[16];
    char bufDefault[16];
    uint32_t i;
    for (i = 0; i < MAX_MAPPABLE_RX_INPUTS; i++) {
        buf[rxConfig->rcmap[i]] = rcChannelLetters[i];
        if (defaultRxConfig) {
            bufDefault[defaultRxConfig->rcmap[i]] = rcChannelLetters[i];
            equalsDefault = equalsDefault && (rxConfig->rcmap[i] == defaultRxConfig->rcmap[i]);
        }
    }
    buf[i] = '\0';

    const char *formatMap = "map %s\r\n";
    cliDefaultPrintf(dumpMask, equalsDefault, formatMap, bufDefault);
    cliDumpPrintf(dumpMask, equalsDefault, formatMap, buf);
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    char out[9];

    len = strlen(cmdline);

    if (len == 8) {
        // uppercase it
        for (uint32_t i = 0; i < 8; i++)
            cmdline[i] = toupper((unsigned char)cmdline[i]);
        for (uint32_t i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            cliShowParseError();
            return;
        }
        parseRcChannels(cmdline);
    }
    cliPrint("Map: ");
    uint32_t i;
    for (i = 0; i < 8; i++)
        out[rxConfig()->rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    cliPrintf("%s\r\n", out);
}

static const char *checkCommand(const char *cmdLine, const char *command)
{
    if (!strncasecmp(cmdLine, command, strlen(command))   // command names match
        && !isalnum((unsigned)cmdLine[strlen(command)])) {   // next characted in bufffer is not alphanumeric (command is correctly terminated)
        return cmdLine + strlen(command) + 1;
    } else {
        return 0;
    }
}

static void cliRebootEx(bool bootLoader)
{
    cliPrint("\r\nRebooting");
    bufWriterFlush(cliWriter);
    waitForSerialPortToFinishTransmitting(cliPort);

    stopMotors();
    stopPwmAllMotors();

    delay(1000);
    if (bootLoader) {
        systemResetToBootloader();
        return;
    }
    systemReset();
}

static void cliReboot(void)
{
    cliRebootEx(false);
}

static void cliDfu(char *cmdline)
{
    UNUSED(cmdline);
#ifndef CLI_MINIMAL_VERBOSITY
    cliPrint("\r\nRestarting in DFU mode");
#endif
    cliRebootEx(true);
}

#ifdef USE_RX_ELERES
static void cliEleresBind(char *cmdline)
{
    UNUSED(cmdline);

    if (!feature(FEATURE_RX_SPI)) {
        cliPrint("Eleres not active. Please enable feature ELERES and restart IMU\r\n");
        return;
    }

    cliPrint("Waiting for correct bind signature....\r\n");
    bufWriterFlush(cliWriter);
    if (eleresBind()) {
        cliPrint("Bind timeout!\r\n");
    } else {
        cliPrint("Bind OK!\r\nPlease restart your transmitter.\r\n");
    }
}
#endif // USE_RX_ELERES

static void cliExit(char *cmdline)
{
    UNUSED(cmdline);

#ifndef CLI_MINIMAL_VERBOSITY
    cliPrint("\r\nLeaving CLI mode, unsaved changes lost.\r\n");
#endif
    bufWriterFlush(cliWriter);

    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
    // incase a motor was left running during motortest, clear it here
    mixerResetDisarmedMotors();
    cliReboot();

    cliWriter = NULL;
}

#ifdef GPS
static void cliGpsPassthrough(char *cmdline)
{
    UNUSED(cmdline);

    gpsEnablePassthrough(cliPort);
}
#endif

#ifndef USE_QUAD_MIXER_ONLY
static void cliMixer(char *cmdline)
{
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrintf("Mixer: %s\r\n", mixerNames[mixerConfigMutable()->mixerMode - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available mixers: ");
        for (uint32_t i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            cliPrintf("%s ", mixerNames[i]);
        }
        cliPrint("\r\n");
        return;
    }

    for (uint32_t i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            cliPrint("Invalid name\r\n");
            return;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            mixerConfigMutable()->mixerMode = i + 1;
            break;
        }
    }

    cliMixer("");
}
#endif

static void cliMotor(char *cmdline)
{
    int motor_index = 0;
    int motor_value = 0;
    int index = 0;
    char *pch = NULL;
    char *saveptr;

    if (isEmpty(cmdline)) {
        cliShowParseError();
        return;
    }

    pch = strtok_r(cmdline, " ", &saveptr);
    while (pch != NULL) {
        switch (index) {
            case 0:
                motor_index = atoi(pch);
                break;
            case 1:
                motor_value = atoi(pch);
                break;
        }
        index++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    if (motor_index < 0 || motor_index >= MAX_SUPPORTED_MOTORS) {
        cliShowArgumentRangeError("index", 0, MAX_SUPPORTED_MOTORS - 1);
        return;
    }

    if (index == 2) {
        if (motor_value < PWM_RANGE_MIN || motor_value > PWM_RANGE_MAX) {
            cliShowArgumentRangeError("value", 1000, 2000);
            return;
        } else {
            motor_disarmed[motor_index] = motor_value;
        }
    }

    cliPrintf("motor %d: %d\r\n", motor_index, motor_disarmed[motor_index]);
}

static void printName(uint8_t dumpMask, const systemConfig_t * sConfig)
{
    bool equalsDefault = strlen(sConfig->name) == 0;
    cliDumpPrintf(dumpMask, equalsDefault, "name %s\r\n", equalsDefault ? emptyName : sConfig->name);
}

static void cliName(char *cmdline)
{
    int32_t len = strlen(cmdline);
    if (len > 0) {
        memset(systemConfigMutable()->name, 0, ARRAYLEN(systemConfigMutable()->name));
        if (strncmp(cmdline, emptyName, len)) {
            strncpy(systemConfigMutable()->name, cmdline, MIN(len, MAX_NAME_LENGTH));
        }
    }
    printName(DUMP_MASTER, systemConfig());
}

#ifdef PLAY_SOUND
static void cliPlaySound(char *cmdline)
{
    int i;
    const char *name;
    static int lastSoundIdx = -1;

    if (isEmpty(cmdline)) {
        i = lastSoundIdx + 1;     //next sound index
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            while (true) {   //no name for index; try next one
                if (++i >= beeperTableEntryCount())
                    i = 0;   //if end then wrap around to first entry
                if ((name=beeperNameForTableIndex(i)) != NULL)
                    break;   //if name OK then play sound below
                if (i == lastSoundIdx + 1) {     //prevent infinite loop
                    cliPrintf("Error playing sound\r\n");
                    return;
                }
            }
        }
    } else {       //index value was given
        i = atoi(cmdline);
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            cliPrintf("No sound for index %d\r\n", i);
            return;
        }
    }
    lastSoundIdx = i;
    beeperSilence();
    cliPrintf("Playing sound %d: %s\r\n", i, name);
    beeper(beeperModeForTableIndex(i));
}
#endif

static void cliProfile(char *cmdline)
{
    // CLI profile index is 1-based
    if (isEmpty(cmdline)) {
        cliPrintf("profile %d\r\n", getConfigProfile() + 1);
        return;
    } else {
        const int i = atoi(cmdline) - 1;
        if (i >= 0 && i < MAX_PROFILE_COUNT) {
            setConfigProfileAndWriteEEPROM(i);
            cliProfile("");
        }
    }
}

static void cliDumpProfile(uint8_t profileIndex, uint8_t dumpMask)
{
    if (profileIndex >= MAX_PROFILE_COUNT) {
        // Faulty values
        return;
    }
    setConfigProfile(profileIndex);
    cliPrintHashLine("profile");
    cliPrintf("profile %d\r\n\r\n", getConfigProfile() + 1);
    dumpAllValues(PROFILE_VALUE, dumpMask);
    dumpAllValues(CONTROL_RATE_VALUE, dumpMask);
}

static void cliSave(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("Saving");
    //copyCurrentProfileToProfileSlot(getConfigProfile();
    writeEEPROM();
    cliReboot();
}

static void cliDefaults(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("Resetting to defaults");
    resetEEPROM();

    if (!checkCommand(cmdline, "noreboot"))
        cliReboot();
}

static void cliGet(char *cmdline)
{
    const clivalue_t *val;
    int matchedCommands = 0;

    for (uint32_t i = 0; i < ARRAYLEN(valueTable); i++) {
        if (strstr(valueTable[i].name, cmdline)) {
            val = &valueTable[i];
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(val, 0);
            cliPrint("\r\n");
            cliPrintVarRange(val);
            cliPrint("\r\n");

            matchedCommands++;
        }
    }


    if (matchedCommands) {
        return;
    }

    cliPrint("Invalid name\r\n");
}

static void cliSet(char *cmdline)
{
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrint("Current settings: \r\n");
        for (uint32_t i = 0; i < ARRAYLEN(valueTable); i++) {
            val = &valueTable[i];
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            cliPrint("\r\n");
        }
    } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
        // has equals

        char *lastNonSpaceCharacter = eqptr;
        while (*(lastNonSpaceCharacter - 1) == ' ') {
            lastNonSpaceCharacter--;
        }
        uint8_t variableNameLength = lastNonSpaceCharacter - cmdline;

        // skip the '=' and any ' ' characters
        eqptr++;
        while (*(eqptr) == ' ') {
            eqptr++;
        }

        for (uint32_t i = 0; i < ARRAYLEN(valueTable); i++) {
            val = &valueTable[i];
            // ensure exact match when setting to prevent setting variables with shorter names
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0 && variableNameLength == strlen(valueTable[i].name)) {

                bool changeValue = false;
                int_float_value_t tmp = {0};
                const int mode = valueTable[i].type & VALUE_MODE_MASK;
                switch (mode) {
                    case MODE_MAX:
                    case MODE_DIRECT: {
                            if (*eqptr != 0 && strspn(eqptr, "0123456789.+-") == strlen(eqptr)) {
                                int32_t value = 0;
                                uint32_t uvalue = 0;
                                float valuef = 0;

                                value = atoi(eqptr);
                                valuef = fastA2F(eqptr);
                                uvalue = strtoul(eqptr, NULL, 10);
                                // note: compare float values
                                if ((mode == MODE_DIRECT && (valuef >= valueTable[i].config.minmax.min && valuef <= valueTable[i].config.minmax.max))
                                     || (mode == MODE_MAX && (valuef >= 0 && valuef <= valueTable[i].config.max.max))) {

                                    if ((valueTable[i].type & VALUE_TYPE_MASK) == VAR_FLOAT)
                                        tmp.float_value = valuef;
                                    else if ((valueTable[i].type & VALUE_TYPE_MASK) == VAR_UINT32)
                                        tmp.uint_value = uvalue;
                                    else
                                        tmp.int_value = value;

                                    changeValue = true;
                                }
                            }
                        }
                        break;
                    case MODE_LOOKUP: {
                            const lookupTableEntry_t *tableEntry = &lookupTables[valueTable[i].config.lookup.tableIndex];
                            bool matched = false;
                            for (uint32_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
                                matched = strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

                                if (matched) {
                                    tmp.int_value = tableValueIndex;
                                    changeValue = true;
                                }
                            }
                        }
                        break;
                }

                if (changeValue) {
                    cliSetVar(val, tmp);

                    cliPrintf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    cliPrint("Invalid value.");
                    cliPrintVarRange(val);
                    cliPrint("\r\n");
                }

                return;
            }
        }
        cliPrint("Invalid name\r\n");
    } else {
        // no equals, check for matching variables.
        cliGet(cmdline);
    }
}

static const char * getBatteryStateString(void)
{
    static const char * const batteryStateStrings[] = {"OK", "WARNING", "CRITICAL", "NOT PRESENT"};

    return batteryStateStrings[getBatteryState()];
}

static void cliStatus(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintf("System Uptime: %d seconds\r\n", millis() / 1000, vbat);
    cliPrintf("Voltage: %d * 0.1V (%dS battery - %s)\r\n", vbat, batteryCellCount, getBatteryStateString());
    cliPrintf("CPU Clock=%dMHz", (SystemCoreClock / 1000000));

#if (FLASH_SIZE > 64)
    const uint32_t detectedSensorsMask = sensorsMask();

    for (int i = 0; i < SENSOR_INDEX_COUNT; i++) {

        const uint32_t mask = (1 << i);
        if ((detectedSensorsMask & mask) && (mask & SENSOR_NAMES_MASK)) {
            const int sensorHardwareIndex = detectedSensors[i];
            const char *sensorHardware = sensorHardwareNames[i][sensorHardwareIndex];

            cliPrintf(", %s=%s", sensorTypeNames[i], sensorHardware);

            if (mask == SENSOR_ACC && acc.dev.revisionCode) {
                cliPrintf(".%c", acc.dev.revisionCode);
            }
        }
    }
    cliPrint("\r\n");

    cliPrintf("Sensor status: GYRO=%s, ACC=%s, MAG=%s, BARO=%s, RANGEFINDER=%s, GPS=%s\r\n",
        hardwareSensorStatusNames[getHwGyroStatus()],
        hardwareSensorStatusNames[getHwAccelerometerStatus()],
        hardwareSensorStatusNames[getHwCompassStatus()],
        hardwareSensorStatusNames[getHwBarometerStatus()],
        hardwareSensorStatusNames[getHwRangefinderStatus()],
        hardwareSensorStatusNames[getHwGPSStatus()]
    );
#endif

#ifdef USE_SDCARD
    cliSdInfo(NULL);
#endif
#ifdef USE_I2C
    const uint16_t i2cErrorCounter = i2cGetErrorCounter();
#else
    const uint16_t i2cErrorCounter = 0;
#endif

#ifdef STACK_CHECK
    cliPrintf("Stack used: %d, ", stackUsedSize());
#endif
    cliPrintf("Stack size: %d, Stack address: 0x%x\r\n", stackTotalSize(), stackHighMem());

    cliPrintf("I2C Errors: %d, config size: %d, max available config: %d\r\n", i2cErrorCounter, getEEPROMConfigSize(), &__config_end - &__config_start);

#ifdef USE_ADC
    static char * adcFunctions[] = { "BATTERY", "RSSI", "CURRENT", "AIRSPEED" };
    cliPrint("ADC channel usage:\r\n");
    for (int i = 0; i < ADC_FUNCTION_COUNT; i++) {
        cliPrintf("  %8s :", adcFunctions[i]);

        cliPrint(" configured = ");
        if (adcChannelConfig()->adcFunctionChannel[i] == ADC_CHN_NONE) {
            cliPrint("none");
        }
        else {
            cliPrintf("ADC %d", adcChannelConfig()->adcFunctionChannel[i]);
        }

        cliPrint(", used = ");
        if (adcGetFunctionChannelAllocation(i) == ADC_CHN_NONE) {
            cliPrint("none\r\n");
        }
        else {
            cliPrintf("ADC %d\r\n", adcGetFunctionChannelAllocation(i));
        }
    }
#endif

    cliPrintf("System load: %d", averageSystemLoadPercent);
#ifdef ASYNC_GYRO_PROCESSING
    const timeDelta_t pidTaskDeltaTime = getTaskDeltaTime(TASK_PID);
#else
    const timeDelta_t pidTaskDeltaTime = getTaskDeltaTime(TASK_GYROPID);
#endif
    const int pidRate = pidTaskDeltaTime == 0 ? 0 : (int)(1000000.0f / ((float)pidTaskDeltaTime));
    const int rxRate = getTaskDeltaTime(TASK_RX) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_RX)));
    const int systemRate = getTaskDeltaTime(TASK_SYSTEM) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_SYSTEM)));
    cliPrintf(", cycle time: %d, PID rate: %d, RX rate: %d, System rate: %d\r\n",  (uint16_t)cycleTime, pidRate, rxRate, systemRate);
}

#ifndef SKIP_TASK_STATISTICS
static void cliTasks(char *cmdline)
{
    UNUSED(cmdline);
    int maxLoadSum = 0;
    int averageLoadSum = 0;
    cfCheckFuncInfo_t checkFuncInfo;

    cliPrintf("Task list         rate/hz  max/us  avg/us maxload avgload     total/ms\r\n");
    for (cfTaskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        cfTaskInfo_t taskInfo;
        getTaskInfo(taskId, &taskInfo);
        if (taskInfo.isEnabled) {
            const int taskFrequency = taskInfo.latestDeltaTime == 0 ? 0 : (int)(1000000.0f / ((float)taskInfo.latestDeltaTime));
            const int maxLoad = (taskInfo.maxExecutionTime * taskFrequency + 5000) / 1000;
            const int averageLoad = (taskInfo.averageExecutionTime * taskFrequency + 5000) / 1000;
            if (taskId != TASK_SERIAL) {
                maxLoadSum += maxLoad;
                averageLoadSum += averageLoad;
            }
            cliPrintf("%2d - %12s  %6d   %5d   %5d %4d.%1d%% %4d.%1d%%  %8d\r\n",
                    taskId, taskInfo.taskName, taskFrequency, (uint32_t)taskInfo.maxExecutionTime, (uint32_t)taskInfo.averageExecutionTime,
                    maxLoad/10, maxLoad%10, averageLoad/10, averageLoad%10, (uint32_t)taskInfo.totalExecutionTime / 1000);
        }
    }
    getCheckFuncInfo(&checkFuncInfo);
    cliPrintf("Task check function %13d %7d %25d\r\n", (uint32_t)checkFuncInfo.maxExecutionTime, (uint32_t)checkFuncInfo.averageExecutionTime, (uint32_t)checkFuncInfo.totalExecutionTime / 1000);
    cliPrintf("Total (excluding SERIAL) %21d.%1d%% %4d.%1d%%\r\n", maxLoadSum/10, maxLoadSum%10, averageLoadSum/10, averageLoadSum%10);
}
#endif

static void cliVersion(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintf("# %s/%s %s %s / %s (%s)\r\n",
        FC_FIRMWARE_NAME,
        targetName,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision
    );
}

#if !defined(SKIP_TASK_STATISTICS) && !defined(SKIP_CLI_RESOURCES)
static void cliResource(char *cmdline)
{
    UNUSED(cmdline);
    cliPrintf("IO:\r\n----------------------\r\n");
    for (unsigned i = 0; i < DEFIO_IO_USED_COUNT; i++) {
        const char* owner;
        owner = ownerNames[ioRecs[i].owner];

        const char* resource;
        resource = resourceNames[ioRecs[i].resource];

        if (ioRecs[i].index > 0) {
            cliPrintf("%c%02d: %s%d %s\r\n", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner, ioRecs[i].index, resource);
        } else {
            cliPrintf("%c%02d: %s %s\r\n", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner, resource);
        }
    }
}
#endif

static void backupConfigs(void)
{
    // make copies of configs to do differencing
    PG_FOREACH(pg) {
        if (pgIsProfile(pg)) {
            memcpy(pg->copy, pg->address, pgSize(pg) * MAX_PROFILE_COUNT);
        } else {
            memcpy(pg->copy, pg->address, pgSize(pg));
        }
    }
}

static void restoreConfigs(void)
{
    PG_FOREACH(pg) {
        if (pgIsProfile(pg)) {
            memcpy(pg->address, pg->copy, pgSize(pg) * MAX_PROFILE_COUNT);
        } else {
            memcpy(pg->address, pg->copy, pgSize(pg));
        }
    }
}

static void printConfig(const char *cmdline, bool doDiff)
{
    uint8_t dumpMask = DUMP_MASTER;
    const char *options;
    if ((options = checkCommand(cmdline, "master"))) {
        dumpMask = DUMP_MASTER; // only
    } else if ((options = checkCommand(cmdline, "profile"))) {
        dumpMask = DUMP_PROFILE; // only
    } else if ((options = checkCommand(cmdline, "all"))) {
        dumpMask = DUMP_ALL;   // all profiles and rates
    } else {
        options = cmdline;
    }

    if (doDiff) {
        dumpMask = dumpMask | DO_DIFF;
    }

    const int currentProfileIndexSave = getConfigProfile();
    backupConfigs();
    // reset all configs to defaults to do differencing
    resetConfigs();
    // restore the profile indices, since they should not be reset for proper comparison
    setConfigProfile(currentProfileIndexSave);

    if (checkCommand(options, "showdefaults")) {
        dumpMask = dumpMask | SHOW_DEFAULTS;   // add default values as comments for changed values
    }

    if ((dumpMask & DUMP_MASTER) || (dumpMask & DUMP_ALL)) {
        cliPrintHashLine("version");
        cliVersion(NULL);

        if ((dumpMask & (DUMP_ALL | DO_DIFF)) == (DUMP_ALL | DO_DIFF)) {
#ifndef CLI_MINIMAL_VERBOSITY
            cliPrintHashLine("reset configuration to default settings\r\ndefaults noreboot");
#else
            cliPrintf("defaults noreboot\r\n");
#endif
        }

        cliPrintHashLine("resources");
        //printResource(dumpMask, &defaultConfig);

#ifndef USE_QUAD_MIXER_ONLY
        cliPrintHashLine("mixer");
        const bool equalsDefault = mixerConfig_Copy.mixerMode == mixerConfig()->mixerMode;
        const char *formatMixer = "mixer %s\r\n";
        cliDefaultPrintf(dumpMask, equalsDefault, formatMixer, mixerNames[mixerConfig()->mixerMode - 1]);
        cliDumpPrintf(dumpMask, equalsDefault, formatMixer, mixerNames[mixerConfig_Copy.mixerMode - 1]);

        cliDumpPrintf(dumpMask, customMotorMixer(0)->throttle == 0.0f, "\r\nmmix reset\r\n\r\n");

        printMotorMix(dumpMask, customMotorMixer_CopyArray, customMotorMixer(0));

#ifdef USE_SERVOS
        cliPrintHashLine("servo");
        printServo(dumpMask, servoParams_CopyArray, servoParams(0));

        cliPrintHashLine("servo mix");
        // print custom servo mixer if exists
        cliDumpPrintf(dumpMask, customServoMixers(0)->rate == 0, "smix reset\r\n\r\n");
        printServoMix(dumpMask, customServoMixers_CopyArray, customServoMixers(0));
#endif
#endif

        cliPrintHashLine("feature");
        printFeature(dumpMask, &featureConfig_Copy, featureConfig());

#ifdef BEEPER
        cliPrintHashLine("beeper");
        printBeeper(dumpMask, &beeperConfig_Copy, beeperConfig());
#endif

        cliPrintHashLine("map");
        printMap(dumpMask, &rxConfig_Copy, rxConfig());

        cliPrintHashLine("name");
        printName(dumpMask, &systemConfig_Copy);

        cliPrintHashLine("serial");
        printSerial(dumpMask, &serialConfig_Copy, serialConfig());

#ifdef LED_STRIP
        cliPrintHashLine("led");
        printLed(dumpMask, ledStripConfig_Copy.ledConfigs, ledStripConfig()->ledConfigs);

        cliPrintHashLine("color");
        printColor(dumpMask, ledStripConfig_Copy.colors, ledStripConfig()->colors);

        cliPrintHashLine("mode_color");
        printModeColor(dumpMask, &ledStripConfig_Copy, ledStripConfig());
#endif

        cliPrintHashLine("aux");
        printAux(dumpMask, modeActivationConditions_CopyArray, modeActivationConditions(0));

        cliPrintHashLine("adjrange");
        printAdjustmentRange(dumpMask, adjustmentRanges_CopyArray, adjustmentRanges(0));

        cliPrintHashLine("rxrange");
        printRxRange(dumpMask, rxChannelRangeConfigs_CopyArray, rxChannelRangeConfigs(0));

        cliPrintHashLine("master");
        dumpAllValues(MASTER_VALUE, dumpMask);

        if (dumpMask & DUMP_ALL) {
            // dump all profiles
            const int currentProfileIndexSave = getConfigProfile();
            for (int ii = 0; ii < MAX_PROFILE_COUNT; ++ii) {
                cliDumpProfile(ii, dumpMask);
            }
            setConfigProfile(currentProfileIndexSave);
            cliPrintHashLine("restore original profile selection");
            cliPrintf("profile %d\r\n", currentProfileIndexSave + 1);

            cliPrintHashLine("save configuration\r\nsave");
        } else {
            // dump just the current profile
            cliDumpProfile(getConfigProfile(), dumpMask);
        }
    }

    if (dumpMask & DUMP_PROFILE) {
        cliDumpProfile(getConfigProfile(), dumpMask);
    }
    // restore configs from copies
    restoreConfigs();
}

static void cliDump(char *cmdline)
{
    printConfig(cmdline, false);
}

static void cliDiff(char *cmdline)
{
    printConfig(cmdline, true);
}

typedef struct {
    const char *name;
#ifndef SKIP_CLI_COMMAND_HELP
    const char *description;
    const char *args;
#endif
    void (*func)(char *cmdline);
} clicmd_t;

#ifndef SKIP_CLI_COMMAND_HELP
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name , \
    description , \
    args , \
    method \
}
#else
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name, \
    method \
}
#endif

static void cliHelp(char *cmdline);

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    CLI_COMMAND_DEF("adjrange", "configure adjustment ranges", NULL, cliAdjustmentRange),
#if defined(USE_ASSERT)
    CLI_COMMAND_DEF("assert", "", NULL, cliAssert),
#endif
    CLI_COMMAND_DEF("aux", "configure modes", NULL, cliAux),
#ifdef BEEPER
    CLI_COMMAND_DEF("beeper", "turn on/off beeper", "list\r\n"
            "\t<+|->[name]", cliBeeper),
#endif
#if defined(BOOTLOG)
    CLI_COMMAND_DEF("bootlog", "show boot events", NULL, cliBootlog),
#endif
#ifdef LED_STRIP
    CLI_COMMAND_DEF("color", "configure colors", NULL, cliColor),
    CLI_COMMAND_DEF("mode_color", "configure mode and special colors", NULL, cliModeColor),
#endif
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cliDefaults),
    CLI_COMMAND_DEF("dfu", "DFU mode on reboot", NULL, cliDfu),
    CLI_COMMAND_DEF("diff", "list configuration changes from default",
        "[master|profile|rates|all] {showdefaults}", cliDiff),
    CLI_COMMAND_DEF("dump", "dump configuration",
        "[master|profile|rates|all] {showdefaults}", cliDump),
#ifdef USE_RX_ELERES
    CLI_COMMAND_DEF("eleres_bind", NULL, NULL, cliEleresBind),
#endif // USE_RX_ELERES
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("feature", "configure features",
        "list\r\n"
        "\t<+|->[name]", cliFeature),
#ifdef USE_FLASHFS
    CLI_COMMAND_DEF("flash_erase", "erase flash chip", NULL, cliFlashErase),
    CLI_COMMAND_DEF("flash_info", "show flash chip info", NULL, cliFlashInfo),
#ifdef USE_FLASH_TOOLS
    CLI_COMMAND_DEF("flash_read", NULL, "<length> <address>", cliFlashRead),
    CLI_COMMAND_DEF("flash_write", NULL, "<address> <message>", cliFlashWrite),
#endif
#endif
    CLI_COMMAND_DEF("get", "get variable value",
            "[name]", cliGet),
#ifdef GPS
    CLI_COMMAND_DEF("gpspassthrough", "passthrough gps to serial", NULL, cliGpsPassthrough),
#endif
    CLI_COMMAND_DEF("help", NULL, NULL, cliHelp),
#ifdef LED_STRIP
    CLI_COMMAND_DEF("led", "configure leds", NULL, cliLed),
#endif
    CLI_COMMAND_DEF("map", "configure rc channel order",
        "[<map>]", cliMap),
#ifndef USE_QUAD_MIXER_ONLY
    CLI_COMMAND_DEF("mixer", "configure mixer",
        "list\r\n"
        "\t<name>", cliMixer),
    CLI_COMMAND_DEF("mmix", "custom motor mixer", NULL, cliMotorMix),
#endif
    CLI_COMMAND_DEF("motor",  "get/set motor",
       "<index> [<value>]", cliMotor),
    CLI_COMMAND_DEF("name", "name of craft", NULL, cliName),
#ifdef PLAY_SOUND
    CLI_COMMAND_DEF("play_sound", NULL,
        "[<index>]\r\n", cliPlaySound),
#endif
    CLI_COMMAND_DEF("profile", "change profile",
        "[<index>]", cliProfile),
#if !defined(SKIP_TASK_STATISTICS) && !defined(SKIP_CLI_RESOURCES)
    CLI_COMMAND_DEF("resource", "view currently used resources", NULL, cliResource),
#endif
    CLI_COMMAND_DEF("rxrange", "configure rx channel ranges", NULL, cliRxRange),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
    CLI_COMMAND_DEF("serial", "configure serial ports", NULL, cliSerial),
#ifdef USE_SERIAL_PASSTHROUGH
    CLI_COMMAND_DEF("serialpassthrough", "passthrough serial data to port", "<id> [baud] [mode] : passthrough to serial", cliSerialPassthrough),
#endif
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("servo", "configure servos", NULL, cliServo),
#endif
    CLI_COMMAND_DEF("set", "change setting", "[<name>=<value>]", cliSet),
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("smix", "servo mixer",
        "<rule> <servo> <source> <rate> <speed>\r\n"
        "\treset\r\n"
        "\tload <mixer>\r\n"
        "\treverse <servo> <source> r|n", cliServoMix),
#endif
#ifdef USE_SDCARD
    CLI_COMMAND_DEF("sd_info", "sdcard info", NULL, cliSdInfo),
#endif
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
#ifndef SKIP_TASK_STATISTICS
    CLI_COMMAND_DEF("tasks", "show task stats", NULL, cliTasks),
#endif
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
};

static void cliHelp(char *cmdline)
{
    UNUSED(cmdline);

    for (uint32_t i = 0; i < ARRAYLEN(cmdTable); i++) {
        cliPrint(cmdTable[i].name);
#ifndef SKIP_CLI_COMMAND_HELP
        if (cmdTable[i].description) {
            cliPrintf(" - %s", cmdTable[i].description);
        }
        if (cmdTable[i].args) {
            cliPrintf("\r\n\t%s", cmdTable[i].args);
        }
#endif
        cliPrint("\r\n");
    }
}

void cliProcess(void)
{
    if (!cliWriter) {
        return;
    }

    // Be a little bit tricky.  Flush the last inputs buffer, if any.
    bufWriterFlush(cliWriter);

    while (serialRxBytesWaiting(cliPort)) {
        uint8_t c = serialRead(cliPort);
        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            uint32_t i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; bufferIndex++) {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex] && bufferIndex < sizeof(cliBuffer) - 2) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        cliBuffer[bufferIndex] = '\0';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                cliPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    cliPrint(cmd->name);
                    cliWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                cliWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {   // CTRL-D
            cliExit(cliBuffer);
            return;
        } else if (c == 12) {                  // NewPage / CTRL-L
            // clear screen
            cliPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            cliPrint("\r\n");

            // Strip comment starting with # from line
            char *p = cliBuffer;
            p = strchr(p, '#');
            if (NULL != p) {
                bufferIndex = (uint32_t)(p - cliBuffer);
            }

            // Strip trailing whitespace
            while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
                bufferIndex--;
            }

            // Process non-empty lines
            if (bufferIndex > 0) {
                cliBuffer[bufferIndex] = 0; // null terminate

                const clicmd_t *cmd;
                for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                    if (!strncasecmp(cliBuffer, cmd->name, strlen(cmd->name))   // command names match
                       && !isalnum((unsigned)cliBuffer[strlen(cmd->name)]))    // next characted in bufffer is not alphanumeric (command is correctly terminated)
                        break;
                }
                if (cmd < cmdTable + ARRAYLEN(cmdTable))
                    cmd->func(cliBuffer + strlen(cmd->name) + 1);
                else
                    cliPrint("Unknown command, try 'help'");
                bufferIndex = 0;
            }

            memset(cliBuffer, 0, sizeof(cliBuffer));

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;

            cliPrompt();
        } else if (c == 127) {
            // backspace
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                cliPrint("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == ' ')
                continue; // Ignore leading spaces
            cliBuffer[bufferIndex++] = c;
            cliWrite(c);
        }
    }
}

void cliEnter(serialPort_t *serialPort)
{
    cliMode = 1;
    cliPort = serialPort;
    setPrintfSerialPort(cliPort);
    cliWriter = bufWriterInit(cliWriteBuffer, sizeof(cliWriteBuffer),
                              (bufWrite_t)serialWriteBufShim, serialPort);

#ifndef CLI_MINIMAL_VERBOSITY
    cliPrint("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
#else
    cliPrint("\r\nCLI\r\n");
#endif
    cliPrompt();
    ENABLE_ARMING_FLAG(PREVENT_ARMING);
}

void cliInit(const serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);
    BUILD_BUG_ON(LOOKUP_TABLE_COUNT != ARRAYLEN(lookupTables));
}
#endif// USE_CLI
