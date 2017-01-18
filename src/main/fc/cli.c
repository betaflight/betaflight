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

#ifdef USE_CLI

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cms/cms.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/typeconversion.h"

#include "config/config_eeprom.h"
#include "config/config_profile.h"
#include "config/config_master.h"
#include "config/feature.h"

#include "drivers/accgyro.h"
#include "drivers/buf_writer.h"
#include "drivers/bus_i2c.h"
#include "drivers/compass.h"
#include "drivers/dma.h"
#include "drivers/flash.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rx_pwm.h"
#include "drivers/sdcard.h"
#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/stack_check.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/vcd.h"
#include "drivers/display.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/cli.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/pid.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/motors.h"
#include "io/osd.h"
#include "io/serial.h"
#include "io/servos.h"
#include "io/vtx.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/frsky.h"
#include "telemetry/telemetry.h"

static serialPort_t *cliPort;
static bufWriter_t *cliWriter;
static uint8_t cliWriteBuffer[sizeof(*cliWriter) + 128];

static char cliBuffer[48];
static uint32_t bufferIndex = 0;

typedef enum {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_RATES = (1 << 2),
    DUMP_ALL = (1 << 3),
    DO_DIFF = (1 << 4),
    SHOW_DEFAULTS = (1 << 5),
    HIDE_UNUSED = (1 << 6)
} dumpFlags_e;

static const char* const emptyName = "-";

#ifndef USE_QUAD_MIXER_ONLY
// sync this with mixerMode_e
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", "QUADX1234", NULL
};
#endif

// sync this with features_e
static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "FAILSAFE",
    "SONAR", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DISPLAY", "OSD",
    "BLACKBOX", "CHANNEL_FORWARDING", "TRANSPONDER", "AIRMODE",
    "SDCARD", "VTX", "RX_SPI", "SOFTSPI", "ESC_SENSOR", NULL
};

// sync this with rxFailsafeChannelMode_e
static const char rxFailsafeModeCharacters[] = "ahs";

static const rxFailsafeChannelMode_e rxFailsafeModesTable[RX_FAILSAFE_TYPE_COUNT][RX_FAILSAFE_MODE_COUNT] = {
    { RX_FAILSAFE_MODE_AUTO, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_INVALID },
    { RX_FAILSAFE_MODE_INVALID, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_SET }
};

// sync this with accelerationSensor_e
static const char * const lookupTableAccHardware[] = {
    "AUTO",
    "NONE",
    "ADXL345",
    "MPU6050",
    "MMA8452",
    "BMA280",
    "LSM303DLHC",
    "MPU6000",
    "MPU6500",
    "MPU9250",
    "ICM20689",
    "FAKE"
};

#ifdef BARO
// sync this with baroSensor_e
static const char * const lookupTableBaroHardware[] = {
    "AUTO",
    "NONE",
    "BMP085",
    "MS5611",
    "BMP280"
};
#endif

#ifdef MAG
// sync this with magSensor_e
static const char * const lookupTableMagHardware[] = {
    "AUTO",
    "NONE",
    "HMC5883",
    "AK8975",
    "AK8963"
};
#endif

#if defined(USE_SENSOR_NAMES)
// sync this with sensors_e
static const char * const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "SONAR", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)

static const char * const sensorHardwareNames[4][15] = {
    { "", "None", "MPU6050", "L3G4200D", "MPU3050", "L3GD20", "MPU6000", "MPU6500", "MPU9250", "ICM20689", "ICM20608G", "ICM20602", "FAKE", NULL },
    { "", "None", "ADXL345", "MPU6050", "MMA845x", "BMA280", "LSM303DLHC", "MPU6000", "MPU6500", "ICM20689", "MPU9250", "ICM20608G", "ICM20602", "FAKE", NULL },
    { "", "None", "BMP085", "MS5611", "BMP280", NULL },
    { "", "None", "HMC5883", "AK8975", "AK8963", NULL }
};
#endif /* USE_SENSOR_NAMES */

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
    "NMEA", "UBLOX"
};

static const char * const lookupTableGPSSBASMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN"
};
#endif

static const char * const lookupTableCurrentSensor[] = {
    "NONE", "ADC", "VIRTUAL", "ESC"
};

static const char * const lookupTableBatterySensor[] = {
    "ADC", "ESC"
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
    "CRSF",
    "SRXL"
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
    "INAV"
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

static const char * const lookupTableDebug[DEBUG_COUNT] = {
    "NONE",
    "CYCLETIME",
    "BATTERY",
    "GYRO",
    "ACCELEROMETER",
    "MIXER",
    "AIRMODE",
    "PIDLOOP",
    "NOTCH",
    "RC_INTERPOLATION",
    "VELOCITY",
    "DFILTER",
    "ANGLERATE",
    "ESC_SENSOR",
    "SCHEDULER",
    "STACK"
};

#ifdef OSD
static const char * const lookupTableOsdType[] = {
    "AUTO",
    "PAL",
    "NTSC"
};
#endif

static const char * const lookupTableSuperExpoYaw[] = {
    "OFF", "ON", "ALWAYS"
};

static const char * const lookupTablePwmProtocol[] = {
    "OFF", "ONESHOT125", "ONESHOT42", "MULTISHOT", "BRUSHED",
#ifdef USE_DSHOT
    "DSHOT150", "DSHOT300", "DSHOT600", "DSHOT1200",
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
#endif
#ifdef BLACKBOX
    TABLE_BLACKBOX_DEVICE,
#endif
    TABLE_CURRENT_SENSOR,
    TABLE_BATTERY_SENSOR,
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
    TABLE_DEBUG,
    TABLE_SUPEREXPO_YAW,
    TABLE_MOTOR_PWM_PROTOCOL,
    TABLE_RC_INTERPOLATION,
    TABLE_RC_INTERPOLATION_CHANNELS,
    TABLE_LOWPASS_TYPE,
    TABLE_FAILSAFE,
#ifdef OSD
    TABLE_OSD,
#endif
} lookupTableIndex_e;

static const lookupTableEntry_t lookupTables[] = {
    { lookupTableOffOn, sizeof(lookupTableOffOn) / sizeof(char *) },
    { lookupTableUnit, sizeof(lookupTableUnit) / sizeof(char *) },
    { lookupTableAlignment, sizeof(lookupTableAlignment) / sizeof(char *) },
#ifdef GPS
    { lookupTableGPSProvider, sizeof(lookupTableGPSProvider) / sizeof(char *) },
    { lookupTableGPSSBASMode, sizeof(lookupTableGPSSBASMode) / sizeof(char *) },
#endif
#ifdef BLACKBOX
    { lookupTableBlackboxDevice, sizeof(lookupTableBlackboxDevice) / sizeof(char *) },
#endif
    { lookupTableCurrentSensor, sizeof(lookupTableCurrentSensor) / sizeof(char *) },
    { lookupTableBatterySensor, sizeof(lookupTableBatterySensor) / sizeof(char *) },
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
    { lookupTableDebug, sizeof(lookupTableDebug) / sizeof(char *) },
    { lookupTableSuperExpoYaw, sizeof(lookupTableSuperExpoYaw) / sizeof(char *) },
    { lookupTablePwmProtocol, sizeof(lookupTablePwmProtocol) / sizeof(char *) },
    { lookupTableRcInterpolation, sizeof(lookupTableRcInterpolation) / sizeof(char *) },
    { lookupTableRcInterpolationChannels, sizeof(lookupTableRcInterpolationChannels) / sizeof(char *) },    
    { lookupTableLowpassType, sizeof(lookupTableLowpassType) / sizeof(char *) },
    { lookupTableFailsafe, sizeof(lookupTableFailsafe) / sizeof(char *) },
#ifdef OSD
    { lookupTableOsdType, sizeof(lookupTableOsdType) / sizeof(char *) },
#endif
};

#define VALUE_TYPE_OFFSET 0
#define VALUE_SECTION_OFFSET 4
#define VALUE_MODE_OFFSET 6

typedef enum {
    // value type
    VAR_UINT8 = (0 << VALUE_TYPE_OFFSET),
    VAR_INT8 = (1 << VALUE_TYPE_OFFSET),
    VAR_UINT16 = (2 << VALUE_TYPE_OFFSET),
    VAR_INT16 = (3 << VALUE_TYPE_OFFSET),
    VAR_UINT32 = (4 << VALUE_TYPE_OFFSET),
    VAR_FLOAT = (5 << VALUE_TYPE_OFFSET),

    // value section
    MASTER_VALUE = (0 << VALUE_SECTION_OFFSET),
    PROFILE_VALUE = (1 << VALUE_SECTION_OFFSET),
    PROFILE_RATE_VALUE = (2 << VALUE_SECTION_OFFSET),
    // value mode
    MODE_DIRECT = (0 << VALUE_MODE_OFFSET),
    MODE_LOOKUP = (1 << VALUE_MODE_OFFSET)
} cliValueFlag_e;

#define VALUE_TYPE_MASK (0x0F)
#define VALUE_SECTION_MASK (0x30)
#define VALUE_MODE_MASK (0xC0)

typedef struct cliMinMaxConfig_s {
    const int32_t min;
    const int32_t max;
} cliMinMaxConfig_t;

typedef struct cliLookupTableConfig_s {
    const lookupTableIndex_e tableIndex;
} cliLookupTableConfig_t;

typedef union {
    cliLookupTableConfig_t lookup;
    cliMinMaxConfig_t minmax;
} cliValueConfig_t;

typedef struct {
    const char *name;
    const uint8_t type; // see cliValueFlag_e
    void *ptr;
    const cliValueConfig_t config;
} clivalue_t;

const clivalue_t valueTable[] = {
#ifndef SKIP_TASK_STATISTICS
    { "task_statistics",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.task_statistics, .config.lookup = { TABLE_OFF_ON } },
#endif
    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE,  &rxConfig()->midrc, .config.minmax = { 1200,  1700 } },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE,  &rxConfig()->mincheck, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE,  &rxConfig()->maxcheck, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE,  &rxConfig()->rssi_channel, .config.minmax = { 0,  MAX_SUPPORTED_RC_CHANNEL_COUNT } },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE,  &rxConfig()->rssi_scale, .config.minmax = { RSSI_SCALE_MIN,  RSSI_SCALE_MAX } },
    { "rc_interpolation",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &rxConfig()->rcInterpolation, .config.lookup = { TABLE_RC_INTERPOLATION } },
    { "rc_interpolation_channels",  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &rxConfig()->rcInterpolationChannels, .config.lookup = { TABLE_RC_INTERPOLATION_CHANNELS } },
    { "rc_interpolation_interval",  VAR_UINT8  | MASTER_VALUE,  &rxConfig()->rcInterpolationInterval, .config.minmax = { 1,  50 } },
    { "rssi_ppm_invert",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &rxConfig()->rssi_ppm_invert, .config.lookup = { TABLE_OFF_ON } },
#if defined(USE_PWM)
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &pwmConfig()->inputFilteringMode, .config.lookup = { TABLE_OFF_ON } },
#endif
    { "roll_yaw_cam_mix_degrees",   VAR_UINT8  | MASTER_VALUE,  &rxConfig()->fpvCamAngleDegrees, .config.minmax = { 0,  50 } },
    { "max_aux_channels",           VAR_UINT8  | MASTER_VALUE,  &rxConfig()->max_aux_channel, .config.minmax = { 0,  13 } },
    { "debug_mode",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.debug_mode, .config.lookup = { TABLE_DEBUG } },

    { "min_throttle",               VAR_UINT16 | MASTER_VALUE,  &motorConfig()->minthrottle, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE,  &motorConfig()->maxthrottle, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "min_command",                VAR_UINT16 | MASTER_VALUE,  &motorConfig()->mincommand, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
#ifdef USE_DSHOT
    { "digital_idle_percent",       VAR_FLOAT  | MASTER_VALUE,  &motorConfig()->digitalIdleOffsetPercent, .config.minmax = { 0,  20} },
#endif
    { "3d_deadband_low",            VAR_UINT16 | MASTER_VALUE,  &flight3DConfig()->deadband3d_low, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } }, // FIXME upper limit should match code in the mixer, 1500 currently
    { "3d_deadband_high",           VAR_UINT16 | MASTER_VALUE,  &flight3DConfig()->deadband3d_high, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } }, // FIXME lower limit should match code in the mixer, 1500 currently,
    { "3d_neutral",                 VAR_UINT16 | MASTER_VALUE,  &flight3DConfig()->neutral3d, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "3d_deadband_throttle",       VAR_UINT16 | MASTER_VALUE,  &flight3DConfig()->deadband3d_throttle, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },

    { "use_unsynced_pwm",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &motorConfig()->useUnsyncedPwm, .config.lookup = { TABLE_OFF_ON } },
    { "motor_pwm_protocol",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &motorConfig()->motorPwmProtocol, .config.lookup = { TABLE_MOTOR_PWM_PROTOCOL } },
    { "motor_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &motorConfig()->motorPwmRate, .config.minmax = { 200, 32000 } },

    { "disarm_kill_switch",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &armingConfig()->disarm_kill_switch, .config.lookup = { TABLE_OFF_ON } },
    { "gyro_cal_on_first_arm",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &armingConfig()->gyro_cal_on_first_arm, .config.lookup = { TABLE_OFF_ON } },
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE,  &armingConfig()->auto_disarm_delay, .config.minmax = { 0,  60 } },
    { "small_angle",                VAR_UINT8  | MASTER_VALUE,  &imuConfig()->small_angle, .config.minmax = { 0,  180 } },

    { "fixedwing_althold_dir",      VAR_INT8   | MASTER_VALUE,  &airplaneConfig()->fixedwing_althold_dir, .config.minmax = { -1,  1 } },

    { "reboot_character",           VAR_UINT8  | MASTER_VALUE, &serialConfig()->reboot_character, .config.minmax = { 48,  126 } },
    { "serial_update_rate_hz",      VAR_UINT16 | MASTER_VALUE, &serialConfig()->serial_update_rate_hz, .config.minmax = { 100,  2000 } },

#ifdef GPS
    { "gps_provider",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gpsConfig()->provider, .config.lookup = { TABLE_GPS_PROVIDER } },
    { "gps_sbas_mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gpsConfig()->sbasMode, .config.lookup = { TABLE_GPS_SBAS_MODE } },
    { "gps_auto_config",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gpsConfig()->autoConfig, .config.lookup = { TABLE_OFF_ON } },
    { "gps_auto_baud",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gpsConfig()->autoBaud, .config.lookup = { TABLE_OFF_ON } },

    { "gps_pos_p",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDPOS], .config.minmax = { 0,  200 } },
    { "gps_pos_i",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDPOS], .config.minmax = { 0,  200 } },
    { "gps_pos_d",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDPOS], .config.minmax = { 0,  200 } },
    { "gps_posr_p",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDPOSR], .config.minmax = { 0,  200 } },
    { "gps_posr_i",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDPOSR], .config.minmax = { 0,  200 } },
    { "gps_posr_d",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDPOSR], .config.minmax = { 0,  200 } },
    { "gps_nav_p",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDNAVR], .config.minmax = { 0,  200 } },
    { "gps_nav_i",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDNAVR], .config.minmax = { 0,  200 } },
    { "gps_nav_d",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDNAVR], .config.minmax = { 0,  200 } },
    { "gps_wp_radius",              VAR_UINT16 | MASTER_VALUE, &gpsProfile()->gps_wp_radius, .config.minmax = { 0,  2000 } },
    { "nav_controls_heading",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &gpsProfile()->nav_controls_heading, .config.lookup = { TABLE_OFF_ON } },
    { "nav_speed_min",              VAR_UINT16 | MASTER_VALUE, &gpsProfile()->nav_speed_min, .config.minmax = { 10,  2000 } },
    { "nav_speed_max",              VAR_UINT16 | MASTER_VALUE, &gpsProfile()->nav_speed_max, .config.minmax = { 10,  2000 } },
    { "nav_slew_rate",              VAR_UINT8  | MASTER_VALUE, &gpsProfile()->nav_slew_rate, .config.minmax = { 0,  100 } },
#endif

#ifdef BEEPER
    { "beeper_inversion",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &beeperConfig()->isInverted, .config.lookup = { TABLE_OFF_ON } },
    { "beeper_od",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &beeperConfig()->isOpenDrain, .config.lookup = { TABLE_OFF_ON } },
#endif

#ifdef SERIAL_RX
    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &rxConfig()->serialrx_provider, .config.lookup = { TABLE_SERIAL_RX } },
#endif

    { "sbus_inversion",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &rxConfig()->sbus_inversion, .config.lookup = { TABLE_OFF_ON } },

#ifdef SPEKTRUM_BIND
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE,  &rxConfig()->spektrum_sat_bind, .config.minmax = { SPEKTRUM_SAT_BIND_DISABLED,  SPEKTRUM_SAT_BIND_MAX} },
    { "spektrum_sat_bind_autoreset",VAR_UINT8  | MASTER_VALUE,  &rxConfig()->spektrum_sat_bind_autoreset, .config.minmax = { 0,  1} },
#endif

#ifdef TELEMETRY
    { "telemetry_switch",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &telemetryConfig()->telemetry_switch, .config.lookup = { TABLE_OFF_ON } },
    { "telemetry_inversion",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &telemetryConfig()->telemetry_inversion, .config.lookup = { TABLE_OFF_ON } },
    { "sport_halfduplex",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &telemetryConfig()->sportHalfDuplex, .config.lookup = { TABLE_OFF_ON } },
    { "frsky_default_lattitude",    VAR_FLOAT  | MASTER_VALUE,  &telemetryConfig()->gpsNoFixLatitude, .config.minmax = { -90.0,  90.0 } },
    { "frsky_default_longitude",    VAR_FLOAT  | MASTER_VALUE,  &telemetryConfig()->gpsNoFixLongitude, .config.minmax = { -180.0,  180.0 } },
    { "frsky_coordinates_format",   VAR_UINT8  | MASTER_VALUE,  &telemetryConfig()->frsky_coordinate_format, .config.minmax = { 0,  FRSKY_FORMAT_NMEA } },
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &telemetryConfig()->frsky_unit, .config.lookup = { TABLE_UNIT } },
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE,  &telemetryConfig()->frsky_vfas_precision, .config.minmax = { FRSKY_VFAS_PRECISION_LOW,  FRSKY_VFAS_PRECISION_HIGH } },
    { "frsky_vfas_cell_voltage",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &telemetryConfig()->frsky_vfas_cell_voltage, .config.lookup = { TABLE_OFF_ON } },
    { "hott_alarm_sound_interval",  VAR_UINT8  | MASTER_VALUE,  &telemetryConfig()->hottAlarmSoundInterval, .config.minmax = { 0,  120 } },
    { "pid_values_as_telemetry",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &telemetryConfig()->pidValuesAsTelemetry, .config.lookup = {TABLE_OFF_ON } },
#if defined(TELEMETRY_IBUS)
    { "ibus_report_cell_voltage",   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &ibusTelemetryConfig()->report_cell_voltage, .config.lookup = { TABLE_OFF_ON } },
#endif
#endif

    { "battery_capacity",           VAR_UINT16 | MASTER_VALUE,  &batteryConfig()->batteryCapacity, .config.minmax = { 0,  20000 } },
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE,  &batteryConfig()->vbatscale, .config.minmax = { VBAT_SCALE_MIN,  VBAT_SCALE_MAX } },
    { "vbat_max_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &batteryConfig()->vbatmaxcellvoltage, .config.minmax = { 10,  50 } },
    { "vbat_min_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &batteryConfig()->vbatmincellvoltage, .config.minmax = { 10,  50 } },
    { "vbat_warning_cell_voltage",  VAR_UINT8  | MASTER_VALUE,  &batteryConfig()->vbatwarningcellvoltage, .config.minmax = { 10,  50 } },
    { "vbat_hysteresis",            VAR_UINT8  | MASTER_VALUE,  &batteryConfig()->vbathysteresis, .config.minmax = { 0,  250 } },
    { "current_meter_scale",        VAR_INT16  | MASTER_VALUE,  &batteryConfig()->currentMeterScale, .config.minmax = { -16000,  16000 } },
    { "current_meter_offset",       VAR_INT16 | MASTER_VALUE,  &batteryConfig()->currentMeterOffset, .config.minmax = { -16000, 16000 } },
    { "multiwii_current_meter_output", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &batteryConfig()->multiwiiCurrentMeterOutput, .config.lookup = { TABLE_OFF_ON } },
    { "current_meter_type",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &batteryConfig()->currentMeterType, .config.lookup = { TABLE_CURRENT_SENSOR } },
    { "battery_meter_type",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &batteryConfig()->batteryMeterType, .config.lookup = { TABLE_BATTERY_SENSOR } },
    { "battery_notpresent_level",   VAR_UINT8  | MASTER_VALUE, &batteryConfig()->batterynotpresentlevel, .config.minmax = { 0, 200 } },
    { "use_vbat_alerts",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &batteryConfig()->useVBatAlerts, .config.lookup = { TABLE_OFF_ON } },
    { "use_consumption_alerts",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &batteryConfig()->useConsumptionAlerts, .config.lookup = { TABLE_OFF_ON } },
    { "consumption_warning_percentage", VAR_UINT8  | MASTER_VALUE, &batteryConfig()->consumptionWarningPercentage, .config.minmax = { 0, 100 } },
    { "align_gyro",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gyroConfig()->gyro_align, .config.lookup = { TABLE_ALIGNMENT } },
    { "align_acc",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &accelerometerConfig()->acc_align, .config.lookup = { TABLE_ALIGNMENT } },
    { "align_mag",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &compassConfig()->mag_align, .config.lookup = { TABLE_ALIGNMENT } },

    { "align_board_roll",           VAR_INT16  | MASTER_VALUE,  &boardAlignment()->rollDegrees, .config.minmax = { -180,  360 } },
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE,  &boardAlignment()->pitchDegrees, .config.minmax = { -180,  360 } },
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE,  &boardAlignment()->yawDegrees, .config.minmax = { -180,  360 } },

    { "gyro_lpf",                   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gyroConfig()->gyro_lpf, .config.lookup = { TABLE_GYRO_LPF } },
    { "gyro_sync_denom",            VAR_UINT8  | MASTER_VALUE,  &gyroConfig()->gyro_sync_denom, .config.minmax = { 1,  32 } },
#if defined(GYRO_USES_SPI) && defined(USE_MPU_DATA_READY_SIGNAL)
    { "gyro_isr_update",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gyroConfig()->gyro_isr_update, .config.lookup = { TABLE_OFF_ON } },
#endif
    { "gyro_use_32khz",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gyroConfig()->gyro_use_32khz, .config.lookup = { TABLE_OFF_ON } },
    { "gyro_lowpass_type",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &gyroConfig()->gyro_soft_lpf_type, .config.lookup = { TABLE_LOWPASS_TYPE } },
    { "gyro_lowpass",               VAR_UINT8  | MASTER_VALUE,  &gyroConfig()->gyro_soft_lpf_hz, .config.minmax = { 0,  255 } },
    { "gyro_notch1_hz",             VAR_UINT16 | MASTER_VALUE,  &gyroConfig()->gyro_soft_notch_hz_1, .config.minmax = { 0,  1000 } },
    { "gyro_notch1_cutoff",         VAR_UINT16 | MASTER_VALUE,  &gyroConfig()->gyro_soft_notch_cutoff_1, .config.minmax = { 1,  1000 } },
    { "gyro_notch2_hz",             VAR_UINT16 | MASTER_VALUE,  &gyroConfig()->gyro_soft_notch_hz_2, .config.minmax = { 0,  1000 } },
    { "gyro_notch2_cutoff",         VAR_UINT16 | MASTER_VALUE,  &gyroConfig()->gyro_soft_notch_cutoff_2, .config.minmax = { 1, 1000 } },
    { "moron_threshold",            VAR_UINT8  | MASTER_VALUE,  &gyroConfig()->gyroMovementCalibrationThreshold, .config.minmax = { 0,  128 } },
    { "imu_dcm_kp",                 VAR_UINT16 | MASTER_VALUE,  &imuConfig()->dcm_kp, .config.minmax = { 0,  50000 } },
    { "imu_dcm_ki",                 VAR_UINT16 | MASTER_VALUE,  &imuConfig()->dcm_ki, .config.minmax = { 0,  50000 } },

    { "alt_hold_deadband",          VAR_UINT8  | MASTER_VALUE, &rcControlsConfig()->alt_hold_deadband, .config.minmax = { 1,  250 } },
    { "alt_hold_fast_change",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &rcControlsConfig()->alt_hold_fast_change, .config.lookup = { TABLE_OFF_ON } },
    { "deadband",                   VAR_UINT8  | MASTER_VALUE, &rcControlsConfig()->deadband, .config.minmax = { 0,  32 } },
    { "yaw_deadband",               VAR_UINT8  | MASTER_VALUE, &rcControlsConfig()->yaw_deadband, .config.minmax = { 0,  100 } },

    { "throttle_correction_value",  VAR_UINT8  | MASTER_VALUE, &throttleCorrectionConfig()->throttle_correction_value, .config.minmax = { 0,  150 } },
    { "throttle_correction_angle",  VAR_UINT16 | MASTER_VALUE, &throttleCorrectionConfig()->throttle_correction_angle, .config.minmax = { 1,  900 } },

    { "yaw_control_direction",      VAR_INT8   | MASTER_VALUE,  &rcControlsConfig()->yaw_control_direction, .config.minmax = { -1,  1 } },

    { "yaw_motor_direction",        VAR_INT8   | MASTER_VALUE, &mixerConfig()->yaw_motor_direction, .config.minmax = { -1,  1 } },
    { "yaw_p_limit",                VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.yaw_p_limit, .config.minmax = { YAW_P_LIMIT_MIN, YAW_P_LIMIT_MAX } },
    { "pidsum_limit",               VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.pidSumLimit, .config.minmax = { 0.1, 1.0 } },
#ifdef USE_SERVOS
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE,  &servoConfig()->servoCenterPulse, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, &servoMixerConfig()->tri_unarmed_servo, .config.lookup = { TABLE_OFF_ON } },
    { "servo_lowpass_freq",         VAR_UINT16 | MASTER_VALUE, &servoMixerConfig()->servo_lowpass_freq, .config.minmax = { 10,  400} },
    { "servo_lowpass_enable",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, &servoMixerConfig()->servo_lowpass_enable, .config.lookup = { TABLE_OFF_ON } },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &servoConfig()->servoPwmRate, .config.minmax = { 50,  498 } },
    { "gimbal_mode",                VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &gimbalConfig()->mode, .config.lookup = { TABLE_GIMBAL_MODE } },
#endif

    { "rc_rate",                    VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].rcRate8, .config.minmax = { 0,  255 } },
    { "rc_rate_yaw",                VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].rcYawRate8, .config.minmax = { 0,  255 } },
    { "rc_expo",                    VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].rcExpo8, .config.minmax = { 0,  100 } },
    { "rc_yaw_expo",                VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].rcYawExpo8, .config.minmax = { 0,  100 } },
    { "thr_mid",                    VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].thrMid8, .config.minmax = { 0,  100 } },
    { "thr_expo",                   VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].thrExpo8, .config.minmax = { 0,  100 } },
    { "roll_srate",                 VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].rates[FD_ROLL], .config.minmax = { 0,  CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX } },
    { "pitch_srate",                VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].rates[FD_PITCH], .config.minmax = { 0,  CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX } },
    { "yaw_srate",                  VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].rates[FD_YAW], .config.minmax = { 0,  CONTROL_RATE_CONFIG_YAW_RATE_MAX } },
    { "tpa_rate",                   VAR_UINT8  | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].dynThrPID, .config.minmax = { 0,  CONTROL_RATE_CONFIG_TPA_MAX} },
    { "tpa_breakpoint",             VAR_UINT16 | PROFILE_RATE_VALUE, &masterConfig.profile[0].controlRateProfile[0].tpa_breakpoint, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX} },
    { "airmode_activate_throttle",  VAR_UINT16 | MASTER_VALUE, &rxConfig()->airModeActivateThreshold, .config.minmax = {1000, 2000 } },

    { "failsafe_delay",             VAR_UINT8  | MASTER_VALUE,  &failsafeConfig()->failsafe_delay, .config.minmax = { 0,  200 } },
    { "failsafe_off_delay",         VAR_UINT8  | MASTER_VALUE,  &failsafeConfig()->failsafe_off_delay, .config.minmax = { 0,  200 } },
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE,  &failsafeConfig()->failsafe_throttle, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX } },
    { "failsafe_kill_switch",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &failsafeConfig()->failsafe_kill_switch, .config.lookup = { TABLE_OFF_ON } },
    { "failsafe_throttle_low_delay",VAR_UINT16 | MASTER_VALUE,  &failsafeConfig()->failsafe_throttle_low_delay, .config.minmax = { 0,  300 } },
    { "failsafe_procedure",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &failsafeConfig()->failsafe_procedure, .config.lookup = { TABLE_FAILSAFE } },

    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE,  &rxConfig()->rx_min_usec, .config.minmax = { PWM_PULSE_MIN,  PWM_PULSE_MAX } },
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE,  &rxConfig()->rx_max_usec, .config.minmax = { PWM_PULSE_MIN,  PWM_PULSE_MAX } },

    { "acc_hardware",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &accelerometerConfig()->acc_hardware, .config.lookup = { TABLE_ACC_HARDWARE } },
    { "acc_lpf_hz",                 VAR_UINT16 | MASTER_VALUE, &accelerometerConfig()->acc_lpf_hz, .config.minmax = { 0,  400 } },
    { "accxy_deadband",             VAR_UINT8  | MASTER_VALUE, &imuConfig()->accDeadband.xy, .config.minmax = { 0,  100 } },
    { "accz_deadband",              VAR_UINT8  | MASTER_VALUE, &imuConfig()->accDeadband.z, .config.minmax = { 0,  100 } },
    { "acc_unarmedcal",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &imuConfig()->acc_unarmedcal, .config.lookup = { TABLE_OFF_ON } },
    { "acc_trim_pitch",             VAR_INT16  | MASTER_VALUE, &accelerometerConfig()->accelerometerTrims.values.pitch, .config.minmax = { -300,  300 } },
    { "acc_trim_roll",              VAR_INT16  | MASTER_VALUE, &accelerometerConfig()->accelerometerTrims.values.roll, .config.minmax = { -300,  300 } },

#ifdef BARO
    { "baro_tab_size",              VAR_UINT8  | MASTER_VALUE, &barometerConfig()->baro_sample_count, .config.minmax = { 0,  BARO_SAMPLE_COUNT_MAX } },
    { "baro_noise_lpf",             VAR_FLOAT  | MASTER_VALUE, &barometerConfig()->baro_noise_lpf, .config.minmax = { 0 , 1 } },
    { "baro_cf_vel",                VAR_FLOAT  | MASTER_VALUE, &barometerConfig()->baro_cf_vel, .config.minmax = { 0 , 1 } },
    { "baro_cf_alt",                VAR_FLOAT  | MASTER_VALUE, &barometerConfig()->baro_cf_alt, .config.minmax = { 0 , 1 } },
    { "baro_hardware",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &barometerConfig()->baro_hardware, .config.lookup = { TABLE_BARO_HARDWARE } },
#endif

#ifdef MAG
    { "mag_hardware",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &compassConfig()->mag_hardware, .config.lookup = { TABLE_MAG_HARDWARE } },
    { "mag_declination",            VAR_INT16  | MASTER_VALUE, &compassConfig()->mag_declination, .config.minmax = { -18000,  18000 } },
#endif
    { "dterm_lowpass_type",         VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].pidProfile.dterm_filter_type, .config.lookup = { TABLE_LOWPASS_TYPE } },
    { "dterm_lowpass",              VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.dterm_lpf_hz, .config.minmax = {0, 500 } },
    { "dterm_notch_hz",             VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.dterm_notch_hz, .config.minmax = { 0,  500 } },
    { "dterm_notch_cutoff",         VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.dterm_notch_cutoff, .config.minmax = { 1,  500 } },
    { "vbat_pid_compensation",      VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].pidProfile.vbatPidCompensation, .config.lookup = { TABLE_OFF_ON } },
    { "pid_at_min_throttle",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, &masterConfig.profile[0].pidProfile.pidAtMinThrottle, .config.lookup = { TABLE_OFF_ON } },
    { "anti_gravity_threshold",     VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.itermThrottleThreshold, .config.minmax = {20, 1000 } },
    { "anti_gravity_gain",          VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.itermAcceleratorGain, .config.minmax = {1, 30 } },
    { "setpoint_relax_ratio",       VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.setpointRelaxRatio, .config.minmax = {0, 100 } },
    { "dterm_setpoint_weight",      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.dtermSetpointWeight, .config.minmax = {0, 255 } },
    { "yaw_accel_limit",            VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.yawRateAccelLimit, .config.minmax = {0.1f, 50.0f } },
    { "accel_limit",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.rateAccelLimit, .config.minmax = {0.1f, 50.0f } },

    { "accum_threshold",            VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.rollPitchItermIgnoreRate, .config.minmax = {15, 1000 } },
    { "yaw_accum_threshold",        VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.yawItermIgnoreRate, .config.minmax = {15, 1000 } },
    { "yaw_lowpass",                VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.yaw_lpf_hz, .config.minmax = {0, 500 } },
    { "pid_process_denom",          VAR_UINT8  | MASTER_VALUE,  &pidConfig()->pid_process_denom, .config.minmax = { 1,  16 } },

    { "p_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PITCH], .config.minmax = { 0,  200 } },
    { "i_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PITCH], .config.minmax = { 0,  200 } },
    { "d_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PITCH], .config.minmax = { 0,  200 } },
    { "p_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[ROLL], .config.minmax = { 0,  200 } },
    { "i_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[ROLL], .config.minmax = { 0,  200 } },
    { "d_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[ROLL], .config.minmax = { 0,  200 } },
    { "p_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[YAW], .config.minmax = { 0,  200 } },
    { "i_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[YAW], .config.minmax = { 0,  200 } },
    { "d_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[YAW], .config.minmax = { 0,  200 } },

    { "p_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDALT], .config.minmax = { 0,  200 } },
    { "i_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDALT], .config.minmax = { 0,  200 } },
    { "d_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDALT], .config.minmax = { 0,  200 } },

    { "p_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDLEVEL], .config.minmax = { 0,  200 } },
    { "i_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDLEVEL], .config.minmax = { 0,  200 } },
    { "d_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDLEVEL], .config.minmax = { 0,  200 } },

    { "p_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDVEL], .config.minmax = { 0,  200 } },
    { "i_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDVEL], .config.minmax = { 0,  200 } },
    { "d_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDVEL], .config.minmax = { 0,  200 } },

    { "level_stick_sensitivity",    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.levelSensitivity, .config.minmax = { 10,  200 } },
    { "level_angle_limit",          VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.levelAngleLimit, .config.minmax = { 10,  120 } },

#ifdef BLACKBOX
    { "blackbox_rate_num",          VAR_UINT8  | MASTER_VALUE,  &blackboxConfig()->rate_num, .config.minmax = { 1,  32 } },
    { "blackbox_rate_denom",        VAR_UINT8  | MASTER_VALUE,  &blackboxConfig()->rate_denom, .config.minmax = { 1,  32 } },
    { "blackbox_device",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &blackboxConfig()->device, .config.lookup = { TABLE_BLACKBOX_DEVICE } },
    { "blackbox_on_motor_test",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &blackboxConfig()->on_motor_test, .config.lookup = { TABLE_OFF_ON } },
#endif

#ifdef VTX
    { "vtx_band",                   VAR_UINT8  | MASTER_VALUE,  &masterConfig.vtx_band, .config.minmax = { 1, 5 } },
    { "vtx_channel",                VAR_UINT8  | MASTER_VALUE,  &masterConfig.vtx_channel, .config.minmax = { 1, 8 } },
    { "vtx_mode",                   VAR_UINT8  | MASTER_VALUE,  &masterConfig.vtx_mode, .config.minmax = { 0, 2 } },
    { "vtx_mhz",                    VAR_UINT16 | MASTER_VALUE,  &masterConfig.vtx_mhz, .config.minmax = { 5600, 5950 } },
#endif

#ifdef MAG
    { "magzero_x",                  VAR_INT16  | MASTER_VALUE, &compassConfig()->magZero.raw[X], .config.minmax = { -32768,  32767 } },
    { "magzero_y",                  VAR_INT16  | MASTER_VALUE, &compassConfig()->magZero.raw[Y], .config.minmax = { -32768,  32767 } },
    { "magzero_z",                  VAR_INT16  | MASTER_VALUE, &compassConfig()->magZero.raw[Z], .config.minmax = { -32768,  32767 } },
#endif
#ifdef LED_STRIP
    { "ledstrip_visual_beeper",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &ledStripConfig()->ledstrip_visual_beeper, .config.lookup = { TABLE_OFF_ON } },
#endif
#if defined(USE_RTC6705)
    { "vtx_channel",                VAR_UINT8  | MASTER_VALUE, &masterConfig.vtx_channel, .config.minmax = { 0,  39 } },
    { "vtx_power",                  VAR_UINT8  | MASTER_VALUE, &masterConfig.vtx_power,   .config.minmax = { 0,  1 } },
#endif
#ifdef USE_SDCARD
    { "sdcard_dma",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &sdcardConfig()->useDma, .config.lookup = { TABLE_OFF_ON } },
#endif
#ifdef OSD
    { "osd_units",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, &osdProfile()->units, .config.lookup = { TABLE_UNIT } },

    { "osd_rssi_alarm",             VAR_UINT8  | MASTER_VALUE, &osdProfile()->rssi_alarm, .config.minmax = { 0, 100 } },
    { "osd_cap_alarm",              VAR_UINT16 | MASTER_VALUE, &osdProfile()->cap_alarm, .config.minmax = { 0, 20000 } },
    { "osd_time_alarm",             VAR_UINT16 | MASTER_VALUE, &osdProfile()->time_alarm, .config.minmax = { 0, 60 } },
    { "osd_alt_alarm",              VAR_UINT16 | MASTER_VALUE, &osdProfile()->alt_alarm, .config.minmax = { 0, 10000 } },

    { "osd_main_voltage_pos",       VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_MAIN_BATT_VOLTAGE], .config.minmax = { 0, UINT16_MAX } },
    { "osd_rssi_pos",               VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_RSSI_VALUE], .config.minmax = { 0, UINT16_MAX } },
    { "osd_flytimer_pos",           VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_FLYTIME], .config.minmax = { 0, UINT16_MAX } },
    { "osd_ontime_pos",             VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_ONTIME], .config.minmax = { 0, UINT16_MAX } },
    { "osd_flymode_pos",            VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_FLYMODE], .config.minmax = { 0, UINT16_MAX } },
    { "osd_throttle_pos",           VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_THROTTLE_POS], .config.minmax = { 0, UINT16_MAX } },
    { "osd_vtx_channel_pos",        VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_VTX_CHANNEL], .config.minmax = { 0, UINT16_MAX } },
    { "osd_crosshairs",             VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_CROSSHAIRS], .config.minmax = { 0, UINT16_MAX } },
    { "osd_artificial_horizon",     VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_ARTIFICIAL_HORIZON], .config.minmax = { 0, UINT16_MAX } },
    { "osd_current_draw_pos",       VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_CURRENT_DRAW], .config.minmax = { 0, UINT16_MAX } },
    { "osd_mah_drawn_pos",          VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_MAH_DRAWN], .config.minmax = { 0, UINT16_MAX } },
    { "osd_craft_name_pos",         VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_CRAFT_NAME], .config.minmax = { 0, UINT16_MAX } },
    { "osd_gps_speed_pos",          VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_GPS_SPEED], .config.minmax = { 0, UINT16_MAX } },
    { "osd_gps_sats_pos",           VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_GPS_SATS], .config.minmax = { 0, UINT16_MAX } },
    { "osd_altitude_pos",           VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_ALTITUDE], .config.minmax = { 0, UINT16_MAX } },
    { "osd_pid_roll_pos",           VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_ROLL_PIDS], .config.minmax = { 0, UINT16_MAX } },
    { "osd_pid_pitch_pos",          VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_PITCH_PIDS], .config.minmax = { 0, UINT16_MAX } },
    { "osd_pid_yaw_pos",            VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_YAW_PIDS], .config.minmax = { 0, UINT16_MAX } },
    { "osd_power_pos",              VAR_UINT16  | MASTER_VALUE, &osdProfile()->item_pos[OSD_POWER], .config.minmax = { 0, UINT16_MAX } },
#endif
#ifdef USE_MAX7456
    { "vcd_video_system",           VAR_UINT8   | MASTER_VALUE, &vcdProfile()->video_system, .config.minmax = { 0, 2 } },
    { "vcd_h_offset",               VAR_INT8    | MASTER_VALUE, &vcdProfile()->h_offset, .config.minmax = { -32, 31 } },
    { "vcd_v_offset",               VAR_INT8    | MASTER_VALUE, &vcdProfile()->v_offset, .config.minmax = { -15, 16 } },
#endif
#ifdef USE_MSP_DISPLAYPORT
    { "displayport_msp_col_adjust", VAR_INT8    | MASTER_VALUE, &displayPortProfileMsp()->colAdjust, .config.minmax = { -6, 0 } },
    { "displayport_msp_row_adjust", VAR_INT8    | MASTER_VALUE, &displayPortProfileMsp()->rowAdjust, .config.minmax = { -3, 0 } },
#endif
#ifdef OSD
    { "displayport_max7456_col_adjust", VAR_INT8    | MASTER_VALUE, &displayPortProfileMax7456()->colAdjust, .config.minmax = { -6, 0 } },
    { "displayport_max7456_row_adjust", VAR_INT8    | MASTER_VALUE, &displayPortProfileMax7456()->rowAdjust, .config.minmax = { -3, 0 } },
#endif
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(clivalue_t))

static void cliPrint(const char *str)
{
    while (*str) {
        bufWriterAppend(cliWriter, *str++);
    }
    bufWriterFlush(cliWriter);
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

static bool cliDumpPrintf(uint8_t dumpMask, bool equalsDefault, const char *format, ...)
{
    if (!((dumpMask & DO_DIFF) && equalsDefault)) {
        va_list va;
        va_start(va, format);
        tfp_format(cliWriter, cliPutp, format, va);
        va_end(va);
        bufWriterFlush(cliWriter);
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
        bufWriterFlush(cliWriter);
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
    bufWriterFlush(cliWriter);
}

static void printValuePointer(const clivalue_t *var, void *valuePointer, uint32_t full)
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
            if (full && (var->type & VALUE_MODE_MASK) == MODE_DIRECT) {
                cliPrintf(" %s", ftoa((float)var->config.minmax.min, buf));
                cliPrintf(" %s", ftoa((float)var->config.minmax.max, buf));
            }
            return; // return from case for float only
    }

    switch(var->type & VALUE_MODE_MASK) {
        case MODE_DIRECT:
            cliPrintf("%d", value);
            if (full) {
                cliPrintf(" %d %d", var->config.minmax.min, var->config.minmax.max);
            }
            break;
        case MODE_LOOKUP:
            cliPrintf(lookupTables[var->config.lookup.tableIndex].values[value]);
            break;
    }
}

void *getValuePointer(const clivalue_t *value)
{
    void *ptr = value->ptr;

    if ((value->type & VALUE_SECTION_MASK) == PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }

    if ((value->type & VALUE_SECTION_MASK) == PROFILE_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
    }

    return ptr;
}

static void *getDefaultPointer(void *valuePointer, const master_t *defaultConfig)
{
    return ((uint8_t *)valuePointer) - (uint32_t)&masterConfig + (uint32_t)defaultConfig;
}

static bool valueEqualsDefault(const clivalue_t *value, const master_t *defaultConfig)
{
    void *ptr = getValuePointer(value);

    void *ptrDefault = getDefaultPointer(ptr, defaultConfig);

    bool result = false;
    switch (value->type & VALUE_TYPE_MASK) {
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

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    void *ptr = getValuePointer(var);

    printValuePointer(var, ptr, full);
}

static void cliPrintVarDefault(const clivalue_t *var, uint32_t full, const master_t *defaultConfig)
{
    void *ptr = getValuePointer(var);

    void *defaultPtr = getDefaultPointer(ptr, defaultConfig);

    printValuePointer(var, defaultPtr, full);
}

static void dumpValues(uint16_t valueSection, uint8_t dumpMask, const master_t *defaultConfig)
{
    const clivalue_t *value;
    for (uint32_t i = 0; i < VALUE_COUNT; i++) {
        value = &valueTable[i];

        if ((value->type & VALUE_SECTION_MASK) != valueSection) {
            continue;
        }

        const char *format = "set %s = ";
        if (cliDefaultPrintf(dumpMask, valueEqualsDefault(value, defaultConfig), format, valueTable[i].name)) {
            cliPrintVarDefault(value, 0, defaultConfig);
            cliPrint("\r\n");
        }
        if (cliDumpPrintf(dumpMask, valueEqualsDefault(value, defaultConfig), format, valueTable[i].name)) {
            cliPrintVar(value, 0);
            cliPrint("\r\n");
        }
    }
}

static void cliPrintVarRange(const clivalue_t *var)
{
    switch (var->type & VALUE_MODE_MASK) {
        case (MODE_DIRECT): {
            cliPrintf("Allowed range: %d - %d\r\n", var->config.minmax.min, var->config.minmax.max);
        }
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
    int32_t int_value;
    float float_value;
} int_float_value_t;

static void cliSetVar(const clivalue_t *var, const int_float_value_t value)
{
    void *ptr = var->ptr;
    if ((var->type & VALUE_SECTION_MASK) == PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }
    if ((var->type & VALUE_SECTION_MASK) == PROFILE_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
    }

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
            *(uint32_t *)ptr = value.int_value;
            break;

        case VAR_FLOAT:
            *(float *)ptr = (float)value.float_value;
            break;
    }
}

#ifndef CLI_MINIMAL_VERBOSITY
static void cliRepeat(char ch, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        bufWriterAppend(cliWriter, ch);
    }
    cliPrint("\r\n");
}
#endif

static void cliPrompt(void)
{
    cliPrint("\r\n# ");
}

static void cliShowParseError(void)
{
    cliPrint("Parse error\r\n");
}

static void cliShowArgumentRangeError(char *name, int min, int max)
{
    cliPrintf("%s must be between %d and %d\r\n", name, min, max);
}

static char *nextArg(char *currentArg)
{
    char *ptr = strchr(currentArg, ' ');
    while (ptr && *ptr == ' ') {
        ptr++;
    }

    return ptr;
}

static char *processChannelRangeArgs(char *ptr, channelRange_t *range, uint8_t *validArgumentCount)
{
    int val;

    for (uint32_t argIndex = 0; argIndex < 2; argIndex++) {
        ptr = nextArg(ptr);
        if (ptr) {
            val = atoi(ptr);
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
    return *string == '\0';
}

static void printRxFail(uint8_t dumpMask, const rxConfig_t *rxConfig, const rxConfig_t *defaultRxConfig)
{
    // print out rxConfig failsafe settings
    for (uint32_t channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
        const rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &rxConfig->failsafe_channel_configurations[channel];
        const rxFailsafeChannelConfiguration_t *channelFailsafeConfigurationDefault;
        bool equalsDefault = true;
        if (defaultRxConfig) {
            channelFailsafeConfigurationDefault = &defaultRxConfig->failsafe_channel_configurations[channel];
            equalsDefault = channelFailsafeConfiguration->mode == channelFailsafeConfigurationDefault->mode
                    && channelFailsafeConfiguration->step == channelFailsafeConfigurationDefault->step;
        }
        const bool requireValue = channelFailsafeConfiguration->mode == RX_FAILSAFE_MODE_SET;
        if (requireValue) {
            const char *format = "rxfail %u %c %d\r\n";
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[channelFailsafeConfigurationDefault->mode],
                RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfigurationDefault->step)
            );
            cliDumpPrintf(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[channelFailsafeConfiguration->mode],
                RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfiguration->step)
            );
        } else {
            const char *format = "rxfail %u %c\r\n";
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[channelFailsafeConfigurationDefault->mode]
            );
            cliDumpPrintf(dumpMask, equalsDefault, format,
                channel,
                rxFailsafeModeCharacters[channelFailsafeConfiguration->mode]
            );
        }
    }
}

static void cliRxFail(char *cmdline)
{
    uint8_t channel;
    char buf[3];

    if (isEmpty(cmdline)) {
        // print out rxConfig failsafe settings
        for (channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
            cliRxFail(itoa(channel, buf, 10));
        }
    } else {
        char *ptr = cmdline;
        channel = atoi(ptr++);
        if ((channel < MAX_SUPPORTED_RC_CHANNEL_COUNT)) {

            rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &rxConfig()->failsafe_channel_configurations[channel];

            uint16_t value;
            rxFailsafeChannelType_e type = (channel < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_TYPE_FLIGHT : RX_FAILSAFE_TYPE_AUX;
            rxFailsafeChannelMode_e mode = channelFailsafeConfiguration->mode;
            bool requireValue = channelFailsafeConfiguration->mode == RX_FAILSAFE_MODE_SET;

            ptr = nextArg(ptr);
            if (ptr) {
                char *p = strchr(rxFailsafeModeCharacters, *(ptr));
                if (p) {
                    uint8_t requestedMode = p - rxFailsafeModeCharacters;
                    mode = rxFailsafeModesTable[type][requestedMode];
                } else {
                    mode = RX_FAILSAFE_MODE_INVALID;
                }
                if (mode == RX_FAILSAFE_MODE_INVALID) {
                    cliShowParseError();
                    return;
                }

                requireValue = mode == RX_FAILSAFE_MODE_SET;

                ptr = nextArg(ptr);
                if (ptr) {
                    if (!requireValue) {
                        cliShowParseError();
                        return;
                    }
                    value = atoi(ptr);
                    value = CHANNEL_VALUE_TO_RXFAIL_STEP(value);
                    if (value > MAX_RXFAIL_RANGE_STEP) {
                        cliPrint("Value out of range\r\n");
                        return;
                    }

                    channelFailsafeConfiguration->step = value;
                } else if (requireValue) {
                    cliShowParseError();
                    return;
                }
                channelFailsafeConfiguration->mode = mode;

            }

            char modeCharacter = rxFailsafeModeCharacters[channelFailsafeConfiguration->mode];

            // double use of cliPrintf below
            // 1. acknowledge interpretation on command,
            // 2. query current setting on single item,

            if (requireValue) {
                cliPrintf("rxfail %u %c %d\r\n",
                    channel,
                    modeCharacter,
                    RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfiguration->step)
                );
            } else {
                cliPrintf("rxfail %u %c\r\n",
                    channel,
                    modeCharacter
                );
            }
        } else {
            cliShowArgumentRangeError("channel", 0, MAX_SUPPORTED_RC_CHANNEL_COUNT - 1);
        }
    }
}

static void printAux(uint8_t dumpMask, const modeActivationProfile_t *modeActivationProfile, const modeActivationProfile_t *defaultModeActivationProfile)
{
    const char *format = "aux %u %u %u %u %u\r\n";
    // print out aux channel settings
    for (uint32_t i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *mac = &modeActivationProfile->modeActivationConditions[i];
        bool equalsDefault = true;
        if (defaultModeActivationProfile) {
            const modeActivationCondition_t *macDefault = &defaultModeActivationProfile->modeActivationConditions[i];
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
    char *ptr;

    if (isEmpty(cmdline)) {
        printAux(DUMP_MASTER, modeActivationProfile(), NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = &modeActivationProfile()->modeActivationConditions[i];
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
        bool equalsDefault = true;
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
    int i, val;
    char *ptr;

    if (isEmpty(cmdline)) {
        printSerial(DUMP_MASTER, serialConfig(), NULL);

    return;
    }
    serialPortConfig_t portConfig;
    memset(&portConfig, 0 , sizeof(portConfig));

    serialPortConfig_t *currentConfig;

    uint8_t validArgumentCount = 0;

    ptr = cmdline;

    val = atoi(ptr++);
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

    for (i = 0; i < 4; i ++) {
        ptr = nextArg(ptr);
        if (!ptr) {
            break;
        }

        val = atoi(ptr);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch(i) {
            case 0:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_1000000) {
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

#ifndef SKIP_SERIAL_PASSTHROUGH
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
        switch(index) {
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
            printf("Port %d is not open, you must specify baud\r\n", id);
            return;
        }
        if (!mode)
            mode = MODE_RXTX;

        passThroughPort = openSerialPort(id, FUNCTION_NONE, NULL,
                                         baud, mode,
                                         SERIAL_NOT_INVERTED);
        if (!passThroughPort) {
            printf("Port %d could not be opened\r\n", id);
            return;
        }
        printf("Port %d opened, baud=%d\r\n", id, baud);
    } else {
        passThroughPort = passThroughPortUsage->serialPort;
        // If the user supplied a mode, override the port's mode, otherwise
        // leave the mode unchanged. serialPassthrough() handles one-way ports.
        printf("Port %d already open\r\n", id);
        if (mode && passThroughPort->mode != mode) {
            printf("Adjusting mode from configured value %d to %d\r\n",
                   passThroughPort->mode, mode);
            serialSetMode(passThroughPort, mode);
        }
        // If this port has a rx callback associated we need to remove it now.
        // Otherwise no data will be pushed in the serial port buffer!
        if (passThroughPort->rxCallback) {
            printf("Removing rxCallback from port\r\n");
            passThroughPort->rxCallback = 0;
        }
    }

    printf("Relaying data to device on port %d, Reset your board to exit "
           "serial passthrough mode.\r\n", id);

    serialPassthrough(cliPort, passThroughPort, NULL, NULL);
}
#endif

static void printAdjustmentRange(uint8_t dumpMask, const adjustmentProfile_t *adjustmentProfile, const adjustmentProfile_t *defaultAdjustmentProfile)
{
    const char *format = "adjrange %u %u %u %u %u %u %u\r\n";
    // print out adjustment ranges channel settings
    for (uint32_t i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
        const adjustmentRange_t *ar = &adjustmentProfile->adjustmentRanges[i];
        bool equalsDefault = true;
        if (defaultAdjustmentProfile) {
            const adjustmentRange_t *arDefault = &defaultAdjustmentProfile->adjustmentRanges[i];
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
    char *ptr;

    if (isEmpty(cmdline)) {
        printAdjustmentRange(DUMP_MASTER, adjustmentProfile(), NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = &adjustmentProfile()->adjustmentRanges[i];
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
        if (customMotorMixer(i)->throttle == 0.0f)
            break;
        const float thr = customMotorMixer[i].throttle;
        const float roll = customMotorMixer[i].roll;
        const float pitch = customMotorMixer[i].pitch;
        const float yaw = customMotorMixer[i].yaw;
        bool equalsDefault = true;
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
#endif // USE_QUAD_MIXER_ONLY

static void cliMotorMix(char *cmdline)
{
#ifdef USE_QUAD_MIXER_ONLY
    UNUSED(cmdline);
#else
    int check = 0;
    uint8_t len;
    char *ptr;

    if (isEmpty(cmdline)) {
        printMotorMix(DUMP_MASTER, customMotorMixer(0), NULL);
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++)
            customMotorMixer(i)->throttle = 0.0f;
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
                    mixerLoadMix(i, masterConfig.customMotorMixer);
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
                customMotorMixer(i)->throttle = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixer(i)->roll = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixer(i)->pitch = fastA2F(ptr);
                check++;
            }
            ptr = nextArg(ptr);
            if (ptr) {
                customMotorMixer(i)->yaw = fastA2F(ptr);
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
#endif
}

static void printRxRange(uint8_t dumpMask, const rxConfig_t *rxConfig, const rxConfig_t *defaultRxConfig)
{
    const char *format = "rxrange %u %u %u\r\n";
    for (uint32_t i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        const rxChannelRangeConfiguration_t *channelRangeConfiguration = &rxConfig->channelRanges[i];
        bool equalsDefault = true;
        if (defaultRxConfig) {
            const rxChannelRangeConfiguration_t *channelRangeConfigurationDefault = &defaultRxConfig->channelRanges[i];
            equalsDefault = channelRangeConfiguration->min == channelRangeConfigurationDefault->min
                && channelRangeConfiguration->max == channelRangeConfigurationDefault->max;
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                channelRangeConfigurationDefault->min,
                channelRangeConfigurationDefault->max
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            channelRangeConfiguration->min,
            channelRangeConfiguration->max
        );
    }
}

static void cliRxRange(char *cmdline)
{
    int i, validArgumentCount = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        printRxRange(DUMP_MASTER, rxConfig(), NULL);
    } else if (strcasecmp(cmdline, "reset") == 0) {
        resetAllRxChannelRangeConfigurations(rxConfig()->channelRanges);
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
                rxChannelRangeConfiguration_t *channelRangeConfiguration = &rxConfig()->channelRanges[i];
                channelRangeConfiguration->min = rangeMin;
                channelRangeConfiguration->max = rangeMax;
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
        bool equalsDefault = true;
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
    char *ptr;

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
        bool equalsDefault = true;
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
    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        printColor(DUMP_MASTER, ledStripConfig()->colors, NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
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
            bool equalsDefault = true;
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
        bool equalsDefault = true;
        if (defaultLedStripConfig) {
            const int colorIndexDefault = defaultLedStripConfig->specialColors.color[j];
            equalsDefault = colorIndex == colorIndexDefault;
            cliDefaultPrintf(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndexDefault);
        }
        cliDumpPrintf(dumpMask, equalsDefault, format, LED_SPECIAL, j, colorIndex);
    }

    const int ledStripAuxChannel = ledStripConfig->ledstrip_aux_channel;
    bool equalsDefault = true;
    if (defaultLedStripConfig) {
        const int ledStripAuxChannelDefault = defaultLedStripConfig->ledstrip_aux_channel;
        equalsDefault = ledStripAuxChannel == ledStripAuxChannelDefault;
        cliDefaultPrintf(dumpMask, equalsDefault, format, LED_AUX_CHANNEL, 0, ledStripAuxChannelDefault);
    }
    cliDumpPrintf(dumpMask, equalsDefault, format, LED_AUX_CHANNEL, 0, ledStripAuxChannel);
}

static void cliModeColor(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printModeColor(DUMP_MASTER, ledStripConfig(), NULL);
    } else {
        enum {MODE = 0, FUNCTION, COLOR, ARGS_COUNT};
        int args[ARGS_COUNT];
        int argNo = 0;
        char* ptr = strtok(cmdline, " ");
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
        if(!setModeColor(modeIdx, funIdx, color)) {
            cliShowParseError();
            return;
        }
        // values are validated
        cliPrintf("mode_color %u %u %u\r\n", modeIdx, funIdx, color);
    }
}
#endif

#ifdef USE_SERVOS
static void printServo(uint8_t dumpMask, servoProfile_t *defaultServoProfile)
{
    // print out servo settings
    const char *format = "servo %u %d %d %d %d %d %d %d\r\n";
    for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const servoParam_t *servoConf = &servoProfile()->servoConf[i];
        bool equalsDefault = true;
        if (defaultServoProfile) {
            const servoParam_t *servoConfDefault = &defaultServoProfile->servoConf[i];
            equalsDefault = servoConf->min == servoConfDefault->min
                && servoConf->max == servoConfDefault->max
                && servoConf->middle == servoConfDefault->middle
                && servoConf->angleAtMin == servoConfDefault->angleAtMax
                && servoConf->rate == servoConfDefault->rate
                && servoConf->forwardFromChannel == servoConfDefault->forwardFromChannel;
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                servoConfDefault->min,
                servoConfDefault->max,
                servoConfDefault->middle,
                servoConfDefault->angleAtMin,
                servoConfDefault->angleAtMax,
                servoConfDefault->rate,
                servoConfDefault->forwardFromChannel
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            servoConf->min,
            servoConf->max,
            servoConf->middle,
            servoConf->angleAtMin,
            servoConf->angleAtMax,
            servoConf->rate,
            servoConf->forwardFromChannel
        );
    }
}

static void cliServo(char *cmdline)
{
    enum { SERVO_ARGUMENT_COUNT = 8 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    servoParam_t *servo;

    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        printServo(DUMP_MASTER, NULL);
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

        enum {INDEX = 0, MIN, MAX, MIDDLE, ANGLE_AT_MIN, ANGLE_AT_MAX, RATE, FORWARD};

        i = arguments[INDEX];

        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if (validArgumentCount != SERVO_ARGUMENT_COUNT || i < 0 || i >= MAX_SUPPORTED_SERVOS) {
            cliShowParseError();
            return;
        }

        servo = &servoProfile()->servoConf[i];

        if (
            arguments[MIN] < PWM_PULSE_MIN || arguments[MIN] > PWM_PULSE_MAX ||
            arguments[MAX] < PWM_PULSE_MIN || arguments[MAX] > PWM_PULSE_MAX ||
            arguments[MIDDLE] < arguments[MIN] || arguments[MIDDLE] > arguments[MAX] ||
            arguments[MIN] > arguments[MAX] || arguments[MAX] < arguments[MIN] ||
            arguments[RATE] < -100 || arguments[RATE] > 100 ||
            arguments[FORWARD] >= MAX_SUPPORTED_RC_CHANNEL_COUNT ||
            arguments[ANGLE_AT_MIN] < 0 || arguments[ANGLE_AT_MIN] > 180 ||
            arguments[ANGLE_AT_MAX] < 0 || arguments[ANGLE_AT_MAX] > 180
        ) {
            cliShowParseError();
            return;
        }

        servo->min = arguments[1];
        servo->max = arguments[2];
        servo->middle = arguments[3];
        servo->angleAtMin = arguments[4];
        servo->angleAtMax = arguments[5];
        servo->rate = arguments[6];
        servo->forwardFromChannel = arguments[7];
    }
}
#endif

#ifdef USE_SERVOS
static void printServoMix(uint8_t dumpMask, const master_t *defaultConfig)
{
    const char *format = "smix %d %d %d %d %d %d %d %d\r\n";
    for (uint32_t i = 0; i < MAX_SERVO_RULES; i++) {
        servoMixer_t customServoMixer = *customServoMixer(i);
        if (customServoMixer.rate == 0) {
            break;
        }

        bool equalsDefault = true;
        if (defaultConfig) {
            servoMixer_t customServoMixerDefault = defaultConfig->customServoMixer[i];
            equalsDefault = customServoMixer.targetChannel == customServoMixerDefault.targetChannel
                && customServoMixer.inputSource == customServoMixerDefault.inputSource
                && customServoMixer.rate == customServoMixerDefault.rate
                && customServoMixer.speed == customServoMixerDefault.speed
                && customServoMixer.min == customServoMixerDefault.min
                && customServoMixer.max == customServoMixerDefault.max
                && customServoMixer.box == customServoMixerDefault.box;

            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                customServoMixerDefault.targetChannel,
                customServoMixerDefault.inputSource,
                customServoMixerDefault.rate,
                customServoMixerDefault.speed,
                customServoMixerDefault.min,
                customServoMixerDefault.max,
                customServoMixerDefault.box
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            customServoMixer.targetChannel,
            customServoMixer.inputSource,
            customServoMixer.rate,
            customServoMixer.speed,
            customServoMixer.min,
            customServoMixer.max,
            customServoMixer.box
        );
    }

    cliPrint("\r\n");

    // print servo directions
    for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const char *format = "smix reverse %d %d r\r\n";
        const servoParam_t *servoConf = &servoProfile()->servoConf[i];
        if (defaultConfig) {
            const servoParam_t *servoConfDefault = &defaultConfig->servoProfile.servoConf[i];
            bool equalsDefault = servoConf->reversedSources == servoConfDefault->reversedSources;
            for (uint32_t channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                equalsDefault = ~(servoConf->reversedSources ^ servoConfDefault->reversedSources) & (1 << channel);
                if (servoConfDefault->reversedSources & (1 << channel)) {
                    cliDefaultPrintf(dumpMask, equalsDefault, format, i , channel);
                }
                if (servoConf->reversedSources & (1 << channel)) {
                    cliDumpPrintf(dumpMask, equalsDefault, format, i , channel);
                }
            }
        } else {
            for (uint32_t channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                if (servoConf->reversedSources & (1 << channel)) {
                    cliDumpPrintf(dumpMask, true, format, i , channel);
                }
            }
        }
    }
}

static void cliServoMix(char *cmdline)
{
    uint8_t len;
    char *ptr;
    int args[8], check = 0;
    len = strlen(cmdline);

    if (len == 0) {
        printServoMix(DUMP_MASTER, NULL);
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        memset(masterConfig.customServoMixer, 0, sizeof(masterConfig.customServoMixer));
        for (uint32_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoProfile()->servoConf[i].reversedSources = 0;
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
                    servoMixerLoadMix(i, masterConfig.customServoMixer);
                    cliPrintf("Loaded %s\r\n", mixerNames[i]);
                    cliServoMix("");
                    break;
                }
            }
        }
    } else if (strncasecmp(cmdline, "reverse", 7) == 0) {
        enum {SERVO = 0, INPUT, REVERSE, ARGS_COUNT};
        ptr = strchr(cmdline, ' ');

        len = strlen(ptr);
        if (len == 0) {
            cliPrintf("s");
            for (uint32_t inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                cliPrintf("\ti%d", inputSource);
            cliPrintf("\r\n");

            for (uint32_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
                cliPrintf("%d", servoIndex);
                for (uint32_t inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                    cliPrintf("\t%s  ", (servoProfile()->servoConf[servoIndex].reversedSources & (1 << inputSource)) ? "r" : "n");
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
                servoProfile()->servoConf[args[SERVO]].reversedSources |= 1 << args[INPUT];
            else
                servoProfile()->servoConf[args[SERVO]].reversedSources &= ~(1 << args[INPUT]);
        } else
            cliShowParseError();

        cliServoMix("reverse");
    } else {
        enum {RULE = 0, TARGET, INPUT, RATE, SPEED, MIN, MAX, BOX, ARGS_COUNT};
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
            args[RATE] >= -100 && args[RATE] <= 100 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED &&
            args[MIN] >= 0 && args[MIN] <= 100 &&
            args[MAX] >= 0 && args[MAX] <= 100 && args[MIN] < args[MAX] &&
            args[BOX] >= 0 && args[BOX] <= MAX_SERVO_BOXES) {
            customServoMixer(i)->targetChannel = args[TARGET];
            customServoMixer(i)->inputSource = args[INPUT];
            customServoMixer(i)->rate = args[RATE];
            customServoMixer(i)->speed = args[SPEED];
            customServoMixer(i)->min = args[MIN];
            customServoMixer(i)->max = args[MAX];
            customServoMixer(i)->box = args[BOX];
            cliServoMix("");
        } else {
            cliShowParseError();
        }
    }
}
#endif

#ifdef USE_SDCARD

static void cliWriteBytes(const uint8_t *buffer, int count)
{
    while (count > 0) {
        cliWrite(*buffer);
        buffer++;
        count--;
    }
}

static void cliSdInfo(char *cmdline) {
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

#ifndef CLI_MINIMAL_VERBOSITY
    uint32_t i;
    cliPrintf("Erasing, please wait ... \r\n");
#else
    cliPrintf("Erasing,\r\n");
#endif

    bufWriterFlush(cliWriter);
    flashfsEraseCompletely();

    while (!flashfsIsReady()) {
#ifndef CLI_MINIMAL_VERBOSITY
        cliPrintf(".");
        if (i++ > 120) {
            i=0;
            cliPrintf("\r\n");
        }

        bufWriterFlush(cliWriter);
#endif
        delay(100);
    }

    cliPrintf("\r\nDone.\r\n");
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

#ifdef VTX
static void printVtx(uint8_t dumpMask, const master_t *defaultConfig)
{
    // print out vtx channel settings
    const char *format = "vtx %u %u %u %u %u %u\r\n";
    bool equalsDefault = true;
    for (uint32_t i = 0; i < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; i++) {
        const vtxChannelActivationCondition_t *cac = &masterConfig.vtxChannelActivationConditions[i];
        if (defaultConfig) {
            const vtxChannelActivationCondition_t *cacDefault = &defaultConfig->vtxChannelActivationConditions[i];
            equalsDefault = cac->auxChannelIndex == cacDefault->auxChannelIndex
                && cac->band == cacDefault->band
                && cac->channel == cacDefault->channel
                && cac->range.startStep == cacDefault->range.startStep
                && cac->range.endStep == cacDefault->range.endStep;
            cliDefaultPrintf(dumpMask, equalsDefault, format,
                i,
                cacDefault->auxChannelIndex,
                cacDefault->band,
                cacDefault->channel,
                MODE_STEP_TO_CHANNEL_VALUE(cacDefault->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(cacDefault->range.endStep)
            );
        }
        cliDumpPrintf(dumpMask, equalsDefault, format,
            i,
            cac->auxChannelIndex,
            cac->band,
            cac->channel,
            MODE_STEP_TO_CHANNEL_VALUE(cac->range.startStep),
            MODE_STEP_TO_CHANNEL_VALUE(cac->range.endStep)
        );
    }
}

static void cliVtx(char *cmdline)
{
    int i, val = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        printVtx(DUMP_MASTER, NULL);
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT) {
            vtxChannelActivationCondition_t *cac = &masterConfig.vtxChannelActivationConditions[i];
            uint8_t validArgumentCount = 0;
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    cac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= VTX_BAND_MIN && val <= VTX_BAND_MAX) {
                    cac->band = val;
                    validArgumentCount++;
                }
            }
            ptr = nextArg(ptr);
            if (ptr) {
                val = atoi(ptr);
                if (val >= VTX_CHANNEL_MIN && val <= VTX_CHANNEL_MAX) {
                    cac->channel = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &cac->range, &validArgumentCount);

            if (validArgumentCount != 5) {
                memset(cac, 0, sizeof(vtxChannelActivationCondition_t));
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_CHANNEL_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}
#endif

static void printName(uint8_t dumpMask)
{
    bool equalsDefault = strlen(masterConfig.name) == 0;
    cliDumpPrintf(dumpMask, equalsDefault, "name %s\r\n", equalsDefault ? emptyName : masterConfig.name);
}

static void cliName(char *cmdline)
{
    uint32_t len = strlen(cmdline);
    if (len > 0) {
        memset(masterConfig.name, 0, ARRAYLEN(masterConfig.name));
        if (strncmp(cmdline, emptyName, len)) {
            strncpy(masterConfig.name, cmdline, MIN(len, MAX_NAME_LENGTH));
        }
    }
    printName(DUMP_MASTER);
}

static void printFeature(uint8_t dumpMask, const master_t *defaultConfig)
{
    uint32_t mask = featureMask();
    uint32_t defaultMask = defaultConfig->enabledFeatures;
    for (uint32_t i = 0; ; i++) { // disable all feature first
        if (featureNames[i] == NULL)
            break;
        const char *format = "feature -%s\r\n";
        cliDefaultPrintf(dumpMask, (defaultMask | ~mask) & (1 << i), format, featureNames[i]);
        cliDumpPrintf(dumpMask, (~defaultMask | mask) & (1 << i), format, featureNames[i]);
    }
    for (uint32_t i = 0; ; i++) {  // reenable what we want.
        if (featureNames[i] == NULL)
            break;
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
            if (mask & (1 << i))
                cliPrintf("%s ", featureNames[i]);
        }
        cliPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available: ");
        for (uint32_t i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
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
#ifndef SONAR
                if (mask & FEATURE_SONAR) {
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
static void printBeeper(uint8_t dumpMask, const master_t *defaultConfig)
{
    uint8_t beeperCount = beeperTableEntryCount();
    uint32_t mask = getBeeperOffMask();
    uint32_t defaultMask = defaultConfig->beeper_off_flags;
    for (int32_t i = 0; i < beeperCount - 2; i++) {
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
            if (mask & (1 << i))
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
                            mask = 1 << i;
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
                            mask = 1 << i;
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
        parseRcChannels(cmdline, &masterConfig.rxConfig);
    }
    cliPrint("Map: ");
    uint32_t i;
    for (i = 0; i < 8; i++)
        out[rxConfig()->rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    cliPrintf("%s\r\n", out);
}

static char *checkCommand(char *cmdLine, const char *command)
{
    if(!strncasecmp(cmdLine, command, strlen(command))   // command names match
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
    stopPwmAllMotors();
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

static void cliDfu(char *cmdLine)
{
    UNUSED(cmdLine);

    cliPrintHashLine("restarting in DFU mode");
    cliRebootEx(true);
}

static void cliExit(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintHashLine("leaving CLI mode, unsaved changes lost");
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

#ifdef USE_ESCSERIAL
static void cliEscPassthrough(char *cmdline)
{
    uint8_t mode = 0;
    int index = 0;
    int i = 0;
    char *pch = NULL;
    char *saveptr;

    if (isEmpty(cmdline)) {
        cliShowParseError();
        return;
    }

    pch = strtok_r(cmdline, " ", &saveptr);
    while (pch != NULL) {
        switch (i) {
            case 0:
                if(strncasecmp(pch, "sk", strlen(pch)) == 0)
                {
                    mode = 0;
                }
                else if(strncasecmp(pch, "bl", strlen(pch)) == 0)
                {
                    mode = 1;
                }
                else if(strncasecmp(pch, "ki", strlen(pch)) == 0)
                {
                    mode = 2;
                }
                else if(strncasecmp(pch, "cc", strlen(pch)) == 0)
                {
                    mode = 4;
                }
                else
                {
                    cliShowParseError();
                    return;
                }
                break;
            case 1:
                index = atoi(pch);
                if(mode == 2 && index == 255)
                {
                    printf("passthru on all pwm outputs enabled\r\n");
                }
                else{
                    if ((index >= 0) && (index < USABLE_TIMER_CHANNEL_COUNT)) {
                        printf("passthru at pwm output %d enabled\r\n", index);
                    }
                    else {
                        printf("invalid pwm output, valid range: 1 to %d\r\n", USABLE_TIMER_CHANNEL_COUNT);
                        return;
                    }
                }
                break;
        }
        i++;
        pch = strtok_r(NULL, " ", &saveptr);
    }
    escEnablePassthrough(cliPort,index,mode);
}
#endif

#ifndef USE_QUAD_MIXER_ONLY
static void cliMixer(char *cmdline)
{
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrintf("Mixer: %s\r\n", mixerNames[mixerConfig()->mixerMode - 1]);
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
            mixerConfig()->mixerMode = i + 1;
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
            motor_disarmed[motor_index] = convertExternalToMotor(motor_value);
        }
    }

    cliPrintf("motor %d: %d\r\n", motor_index, convertMotorToExternal(motor_disarmed[motor_index]));
}

#if (FLASH_SIZE > 128)
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
    int i;

    if (isEmpty(cmdline)) {
        cliPrintf("profile %d\r\n", getCurrentProfile());
        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i < MAX_PROFILE_COUNT) {
            masterConfig.current_profile_index = i;
            writeEEPROM();
            readEEPROM();
            cliProfile("");
        }
    }
}

static void cliRateProfile(char *cmdline)
{
    int i;

    if (isEmpty(cmdline)) {
        cliPrintf("rateprofile %d\r\n", getCurrentControlRateProfile());
        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i < MAX_RATEPROFILES) {
            changeControlRateProfile(i);
            cliRateProfile("");
        }
    }
}

static void cliDumpProfile(uint8_t profileIndex, uint8_t dumpMask, const master_t *defaultConfig)
{
    if (profileIndex >= MAX_PROFILE_COUNT) {
        // Faulty values
        return;
    }
    changeProfile(profileIndex);
    cliPrintHashLine("profile");
    cliProfile("");
    cliPrint("\r\n");
    dumpValues(PROFILE_VALUE, dumpMask, defaultConfig);
    cliRateProfile("");
}

static void cliDumpRateProfile(uint8_t rateProfileIndex, uint8_t dumpMask, const master_t *defaultConfig)
{
    if (rateProfileIndex >= MAX_RATEPROFILES) {
        // Faulty values
        return;
    }
    changeControlRateProfile(rateProfileIndex);
    cliPrintHashLine("rateprofile");
    cliRateProfile("");
    cliPrint("\r\n");
    dumpValues(PROFILE_RATE_VALUE, dumpMask, defaultConfig);
}

static void cliSave(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintHashLine("saving");
    writeEEPROM();
    cliReboot();
}

static void cliDefaults(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintHashLine("resetting to defaults");
    resetEEPROM();
    cliReboot();
}

static void cliGet(char *cmdline)
{
    const clivalue_t *val;
    int matchedCommands = 0;

    for (uint32_t i = 0; i < VALUE_COUNT; i++) {
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
        for (uint32_t i = 0; i < VALUE_COUNT; i++) {
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

        for (uint32_t i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            // ensure exact match when setting to prevent setting variables with shorter names
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0 && variableNameLength == strlen(valueTable[i].name)) {

                bool changeValue = false;
                int_float_value_t tmp = { 0 };
                switch (valueTable[i].type & VALUE_MODE_MASK) {
                    case MODE_DIRECT: {
                            int32_t value = 0;
                            float valuef = 0;

                            value = atoi(eqptr);
                            valuef = fastA2F(eqptr);

                            if (valuef >= valueTable[i].config.minmax.min && valuef <= valueTable[i].config.minmax.max) { // note: compare float value

                                if ((valueTable[i].type & VALUE_TYPE_MASK) == VAR_FLOAT)
                                    tmp.float_value = valuef;
                                else
                                    tmp.int_value = value;

                                changeValue = true;
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
                    cliPrint("Invalid value\r\n");
                    cliPrintVarRange(val);
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

static void cliStatus(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintf("System Uptime: %d seconds\r\n", millis() / 1000);
    cliPrintf("Voltage: %d * 0.1V (%dS battery - %s)\r\n", getVbat(), batteryCellCount, getBatteryStateString());

    cliPrintf("CPU Clock=%dMHz", (SystemCoreClock / 1000000));

#if defined(USE_SENSOR_NAMES)
    const uint32_t detectedSensorsMask = sensorsMask();
    for (uint32_t i = 0; ; i++) {
        if (sensorTypeNames[i] == NULL) {
            break;
        }
        const uint32_t mask = (1 << i);
        if ((detectedSensorsMask & mask) && (mask & SENSOR_NAMES_MASK)) {
            const uint8_t sensorHardwareIndex = detectedSensors[i];
            const char *sensorHardware = sensorHardwareNames[i][sensorHardwareIndex];
            cliPrintf(", %s=%s", sensorTypeNames[i], sensorHardware);
            if (mask == SENSOR_ACC && acc.dev.revisionCode) {
                cliPrintf(".%c", acc.dev.revisionCode);
            }
        }
    }
#endif /* USE_SENSOR_NAMES */
    cliPrint("\r\n");

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

    cliPrintf("I2C Errors: %d, config size: %d\r\n", i2cErrorCounter, sizeof(master_t));

    const int gyroRate = getTaskDeltaTime(TASK_GYROPID) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_GYROPID)));
    const int rxRate = getTaskDeltaTime(TASK_RX) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_RX)));
    const int systemRate = getTaskDeltaTime(TASK_SYSTEM) == 0 ? 0 : (int)(1000000.0f / ((float)getTaskDeltaTime(TASK_SYSTEM)));
    cliPrintf("CPU:%d%%, cycle time: %d, GYRO rate: %d, RX rate: %d, System rate: %d\r\n",
            constrain(averageSystemLoadPercent, 0, 100), getTaskDeltaTime(TASK_GYROPID), gyroRate, rxRate, systemRate);

}

#ifndef SKIP_TASK_STATISTICS
static void cliTasks(char *cmdline)
{
    UNUSED(cmdline);
    int maxLoadSum = 0;
    int averageLoadSum = 0;

#ifndef CLI_MINIMAL_VERBOSITY
    if (masterConfig.task_statistics) {
        cliPrintf("Task list           rate/hz  max/us  avg/us maxload avgload     total/ms\r\n");
    } else {
        cliPrintf("Task list           rate/hz\r\n");
    }
#endif
    for (cfTaskId_e taskId = 0; taskId < TASK_COUNT; taskId++) {
        cfTaskInfo_t taskInfo;
        getTaskInfo(taskId, &taskInfo);
        if (taskInfo.isEnabled) {
            int taskFrequency;
            int subTaskFrequency;
            if (taskId == TASK_GYROPID) {
                subTaskFrequency = taskInfo.latestDeltaTime == 0 ? 0 : (int)(1000000.0f / ((float)taskInfo.latestDeltaTime));
                taskFrequency = subTaskFrequency / pidConfig()->pid_process_denom;
                if (pidConfig()->pid_process_denom > 1) {
                    cliPrintf("%02d - (%13s) ", taskId, taskInfo.taskName);
                } else {
                    taskFrequency = subTaskFrequency;
                    cliPrintf("%02d - (%9s/%3s) ", taskId, taskInfo.subTaskName, taskInfo.taskName);
                }
            } else {
                taskFrequency = taskInfo.latestDeltaTime == 0 ? 0 : (int)(1000000.0f / ((float)taskInfo.latestDeltaTime));
                cliPrintf("%02d - (%13s) ", taskId, taskInfo.taskName);
            }
            const int maxLoad = taskInfo.maxExecutionTime == 0 ? 0 :(taskInfo.maxExecutionTime * taskFrequency + 5000) / 1000;
            const int averageLoad = taskInfo.averageExecutionTime == 0 ? 0 : (taskInfo.averageExecutionTime * taskFrequency + 5000) / 1000;
            if (taskId != TASK_SERIAL) {
                maxLoadSum += maxLoad;
                averageLoadSum += averageLoad;
            }
            if (masterConfig.task_statistics) {
                cliPrintf("%6d %7d %7d %4d.%1d%% %4d.%1d%% %9d\r\n",
                        taskFrequency, taskInfo.maxExecutionTime, taskInfo.averageExecutionTime,
                        maxLoad/10, maxLoad%10, averageLoad/10, averageLoad%10, taskInfo.totalExecutionTime / 1000);
            } else {
                cliPrintf("%6d\r\n", taskFrequency);
            }
            if (taskId == TASK_GYROPID && pidConfig()->pid_process_denom > 1) {
                cliPrintf("   - (%13s) %6d\r\n", taskInfo.subTaskName, subTaskFrequency);
            }
        }
    }
    if (masterConfig.task_statistics) {
        cfCheckFuncInfo_t checkFuncInfo;
        getCheckFuncInfo(&checkFuncInfo);
        cliPrintf("RX Check Function %17d %7d %25d\r\n", checkFuncInfo.maxExecutionTime, checkFuncInfo.averageExecutionTime, checkFuncInfo.totalExecutionTime / 1000);
        cliPrintf("Total (excluding SERIAL) %23d.%1d%% %4d.%1d%%\r\n", maxLoadSum/10, maxLoadSum%10, averageLoadSum/10, averageLoadSum%10);
    }
}
#endif

static void cliVersion(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintf("# %s / %s %s %s / %s (%s)\r\n",
        FC_FIRMWARE_NAME,
        targetName,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision
    );
}

#if defined(USE_RESOURCE_MGMT)

typedef struct {
    const uint8_t owner;
    ioTag_t *ptr;
    const uint8_t maxIndex;
} cliResourceValue_t;

const cliResourceValue_t resourceTable[] = {
#ifdef BEEPER
    { OWNER_BEEPER,        &beeperConfig()->ioTag, 0 },
#endif
    { OWNER_MOTOR,         &motorConfig()->ioTags[0], MAX_SUPPORTED_MOTORS },
#ifdef USE_SERVOS
    { OWNER_SERVO,         &servoConfig()->ioTags[0], MAX_SUPPORTED_SERVOS },
#endif
#if defined(USE_PWM) || defined(USE_PPM)
    { OWNER_PPMINPUT,      &ppmConfig()->ioTag, 0 },
    { OWNER_PWMINPUT,      &pwmConfig()->ioTags[0], PWM_INPUT_PORT_COUNT },
#endif
#ifdef SONAR
    { OWNER_SONAR_TRIGGER, &sonarConfig()->triggerTag, 0 },
    { OWNER_SONAR_ECHO,    &sonarConfig()->echoTag,    0 },
#endif
#ifdef LED_STRIP
    { OWNER_LED_STRIP,     &ledStripConfig()->ioTag,   0 },
#endif
};

static void printResource(uint8_t dumpMask, const master_t *defaultConfig)
{
    for (unsigned int i = 0; i < ARRAYLEN(resourceTable); i++) {
        const char* owner;
        owner = ownerNames[resourceTable[i].owner];

        if (resourceTable[i].maxIndex > 0) {
            for (int index = 0; index < resourceTable[i].maxIndex; index++) {
                ioTag_t ioTag = *(resourceTable[i].ptr + index);
                ioTag_t ioTagDefault = *(resourceTable[i].ptr + index - (uint32_t)&masterConfig + (uint32_t)defaultConfig);

                bool equalsDefault = ioTag == ioTagDefault;
                const char *format = "resource %s %d %c%02d\r\n";
                const char *formatUnassigned = "resource %s %d NONE\r\n";
                if (!ioTagDefault) {
                    cliDefaultPrintf(dumpMask, equalsDefault, formatUnassigned, owner, RESOURCE_INDEX(index));
                } else {
                    cliDefaultPrintf(dumpMask, equalsDefault, format, owner, RESOURCE_INDEX(index), IO_GPIOPortIdxByTag(ioTagDefault) + 'A', IO_GPIOPinIdxByTag(ioTagDefault));
                }
                if (!ioTag) {
                    if (!(dumpMask & HIDE_UNUSED)) {
                        cliDumpPrintf(dumpMask, equalsDefault, formatUnassigned, owner, RESOURCE_INDEX(index));
                    }
                } else {
                    cliDumpPrintf(dumpMask, equalsDefault, format, owner, RESOURCE_INDEX(index), IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag));
                }
            }
        } else {
            ioTag_t ioTag = *resourceTable[i].ptr;
            ioTag_t ioTagDefault = *(resourceTable[i].ptr - (uint32_t)&masterConfig + (uint32_t)defaultConfig);

            bool equalsDefault = ioTag == ioTagDefault;
            const char *format = "resource %s %c%02d\r\n";
            const char *formatUnassigned = "resource %s NONE\r\n";
            if (!ioTagDefault) {
                cliDefaultPrintf(dumpMask, equalsDefault, formatUnassigned, owner);
            } else {
                cliDefaultPrintf(dumpMask, equalsDefault, format, owner, IO_GPIOPortIdxByTag(ioTagDefault) + 'A', IO_GPIOPinIdxByTag(ioTagDefault));
            }
            if (!ioTag) {
                if (!(dumpMask & HIDE_UNUSED)) {
                    cliDumpPrintf(dumpMask, equalsDefault, formatUnassigned, owner);
                }
            } else {
                cliDumpPrintf(dumpMask, equalsDefault, format, owner, IO_GPIOPortIdxByTag(ioTag) + 'A', IO_GPIOPinIdxByTag(ioTag));
            }
        }
    }
}

#ifndef CLI_MINIMAL_VERBOSITY
static bool resourceCheck(uint8_t resourceIndex, uint8_t index, ioTag_t tag)
{
    const char * format = "\r\n* %s * %c%d also used by %s";
    for (int r = 0; r < (int)ARRAYLEN(resourceTable); r++) {
        for (int i = 0; i < (resourceTable[r].maxIndex == 0 ? 1 : resourceTable[r].maxIndex); i++) {
            if (*(resourceTable[r].ptr + i) == tag) {
                bool error = false;
                if (r == resourceIndex) {
                    if (i == index) {
                        continue;
                    }
                    error = true;
                }

                cliPrintf(format, error ? "ERROR" : "WARNING", DEFIO_TAG_GPIOID(tag) + 'A', DEFIO_TAG_PIN(tag), ownerNames[resourceTable[r].owner]);
                if (resourceTable[r].maxIndex > 0) {
                    cliPrintf(" %d", RESOURCE_INDEX(i));
                }
                cliPrint("\r\n");
                if (error) {
                    return false;
                }
            }
        }
    }
    return true;
}
#endif

static void cliResource(char *cmdline)
{
    int len = strlen(cmdline);

    if (len == 0) {
        printResource(DUMP_MASTER | HIDE_UNUSED, NULL);

        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
#ifndef CLI_MINIMAL_VERBOSITY
        cliPrintf("Currently active IO resource assignments:\r\n(reboot to update)\r\n");
        cliRepeat('-', 20);
#endif
        for (int i = 0; i < DEFIO_IO_USED_COUNT; i++) {
            const char* owner;
            owner = ownerNames[ioRecs[i].owner];

            if (ioRecs[i].index > 0) {
                cliPrintf("%c%02d: %s %d\r\n", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner, ioRecs[i].index);
            } else {
                cliPrintf("%c%02d: %s \r\n", IO_GPIOPortIdx(ioRecs + i) + 'A', IO_GPIOPinIdx(ioRecs + i), owner);
            }
        }

        cliPrintf("\r\n\r\nCurrently active DMA:\r\n");
#ifndef CLI_MINIMAL_VERBOSITY
        cliRepeat('-', 20);
#endif
        for (int i = 0; i < DMA_MAX_DESCRIPTORS; i++) {
            const char* owner;
            owner = ownerNames[dmaGetOwner(i)];

            cliPrintf(DMA_OUTPUT_STRING, i / DMA_MOD_VALUE + 1, (i % DMA_MOD_VALUE) + DMA_MOD_OFFSET);
            uint8_t resourceIndex = dmaGetResourceIndex(i);
            if (resourceIndex > 0) {
                cliPrintf(" %s %d\r\n", owner, resourceIndex);
            } else {
                cliPrintf(" %s\r\n", owner);
            }
        }

#ifndef CLI_MINIMAL_VERBOSITY
        cliPrintf("\r\nUse: 'resource' to see how to change resources.\r\n");
#endif

        return;
    }

    uint8_t resourceIndex = 0;
    int index = 0;
    char *pch = NULL;
    char *saveptr;

    pch = strtok_r(cmdline, " ", &saveptr);
    for (resourceIndex = 0; ; resourceIndex++) {
        if (resourceIndex >= ARRAYLEN(resourceTable)) {
            cliPrint("Invalid resource\r\n");
            return;
        }

        if (strncasecmp(pch, ownerNames[resourceTable[resourceIndex].owner], len) == 0) {
            break;
        }
    }

    if (resourceTable[resourceIndex].maxIndex > 0) {
        pch = strtok_r(NULL, " ", &saveptr);
        index = atoi(pch);

        if (index <= 0 || index > resourceTable[resourceIndex].maxIndex) {
            cliShowArgumentRangeError("index", 1, resourceTable[resourceIndex].maxIndex);
            return;
        }
        index -= 1;
    }

    pch = strtok_r(NULL, " ", &saveptr);
    ioTag_t *tag = (ioTag_t*)(resourceTable[resourceIndex].ptr + index);

    uint8_t pin = 0;
    if (strlen(pch) > 0) {
        if (strcasecmp(pch, "NONE") == 0) {
            *tag = IO_TAG_NONE;
            cliPrintf("Resource is freed!\r\n");
            return;
        } else {
            uint8_t port = (*pch) - 'A';
            if (port >= 8) {
                port = (*pch) - 'a';
            }

            if (port < 8) {
                pch++;
                pin = atoi(pch);
                if (pin < 16) {
                    ioRec_t *rec = IO_Rec(IOGetByTag(DEFIO_TAG_MAKE(port, pin)));
                    if (rec) {
#ifndef CLI_MINIMAL_VERBOSITY
                        if (!resourceCheck(resourceIndex, index, DEFIO_TAG_MAKE(port, pin))) {
                            return;
                        }
                        cliPrintf("\r\nResource is set to %c%02d!\r\n", port + 'A', pin);
#else
                        cliPrintf("Set to %c%02d!", port + 'A', pin);
#endif
                        *tag = DEFIO_TAG_MAKE(port, pin);
                    } else {
                        cliPrintf("Resource is invalid!\r\n");
                    }
                    return;
                }
            }
        }
    }

    cliShowParseError();
}
#endif /* USE_RESOURCE_MGMT */

static void printConfig(char *cmdline, bool doDiff)
{
    uint8_t dumpMask = DUMP_MASTER;
    char *options;
    if ((options = checkCommand(cmdline, "master"))) {
        dumpMask = DUMP_MASTER; // only
    } else if ((options = checkCommand(cmdline, "profile"))) {
        dumpMask = DUMP_PROFILE; // only
    } else if ((options = checkCommand(cmdline, "rates"))) {
        dumpMask = DUMP_RATES; // only
    } else if ((options = checkCommand(cmdline, "all"))) {
        dumpMask = DUMP_ALL;   // all profiles and rates
    } else {
        options = cmdline;
    }

    static master_t defaultConfig;
    if (doDiff) {
        dumpMask = dumpMask | DO_DIFF;
    }

    createDefaultConfig(&defaultConfig);

    if (checkCommand(options, "showdefaults")) {
        dumpMask = dumpMask | SHOW_DEFAULTS;   // add default values as comments for changed values
    }

    if ((dumpMask & DUMP_MASTER) || (dumpMask & DUMP_ALL)) {
        cliPrintHashLine("version");
        cliVersion(NULL);

        if ((dumpMask & (DUMP_ALL | DO_DIFF)) == (DUMP_ALL | DO_DIFF)) {
            cliPrintHashLine("reset configuration to default settings");
            cliPrint("defaults\r\n");
        }

        cliPrintHashLine("name");
        printName(dumpMask);

#ifdef USE_RESOURCE_MGMT
        cliPrintHashLine("resources");
        printResource(dumpMask, &defaultConfig);
#endif 

#ifndef USE_QUAD_MIXER_ONLY
        cliPrintHashLine("mixer");
        const bool equalsDefault = mixerConfig()->mixerMode == defaultConfig.mixerConfig.mixerMode;
        const char *formatMixer = "mixer %s\r\n";
        cliDefaultPrintf(dumpMask, equalsDefault, formatMixer, mixerNames[defaultConfig.mixerConfig.mixerMode - 1]);
        cliDumpPrintf(dumpMask, equalsDefault, formatMixer, mixerNames[mixerConfig()->mixerMode - 1]);

        cliDumpPrintf(dumpMask, masterConfig.customMotorMixer[0].throttle == 0.0f, "\r\nmmix reset\r\n\r\n");

        printMotorMix(dumpMask, customMotorMixer(0), defaultConfig.customMotorMixer);

#ifdef USE_SERVOS
        cliPrintHashLine("servo");
        printServo(dumpMask, &defaultConfig.servoProfile);

        cliPrintHashLine("servo mix");
        // print custom servo mixer if exists
        cliDumpPrintf(dumpMask, masterConfig.customServoMixer[0].rate == 0, "smix reset\r\n\r\n");
        printServoMix(dumpMask, &defaultConfig);
#endif
#endif

        cliPrintHashLine("feature");
        printFeature(dumpMask, &defaultConfig);

#ifdef BEEPER
        cliPrintHashLine("beeper");
        printBeeper(dumpMask, &defaultConfig);
#endif

        cliPrintHashLine("map");
        printMap(dumpMask, rxConfig(), &defaultConfig.rxConfig);

        cliPrintHashLine("serial");
        printSerial(dumpMask, serialConfig(), &defaultConfig.serialConfig);

#ifdef LED_STRIP
        cliPrintHashLine("led");
        printLed(dumpMask, ledStripConfig()->ledConfigs, defaultConfig.ledStripConfig.ledConfigs);

        cliPrintHashLine("color");
        printColor(dumpMask, ledStripConfig()->colors, defaultConfig.ledStripConfig.colors);

        cliPrintHashLine("mode_color");
        printModeColor(dumpMask, ledStripConfig(), &defaultConfig.ledStripConfig);
#endif

        cliPrintHashLine("aux");
        printAux(dumpMask, modeActivationProfile(), &defaultConfig.modeActivationProfile);

        cliPrintHashLine("adjrange");
        printAdjustmentRange(dumpMask, adjustmentProfile(), &defaultConfig.adjustmentProfile);

        cliPrintHashLine("rxrange");
        printRxRange(dumpMask, rxConfig(), &defaultConfig.rxConfig);

#ifdef VTX
        cliPrintHashLine("vtx");
        printVtx(dumpMask, &defaultConfig);
#endif

        cliPrintHashLine("rxfail");
        printRxFail(dumpMask, rxConfig(), &defaultConfig.rxConfig);

        cliPrintHashLine("master");
        dumpValues(MASTER_VALUE, dumpMask, &defaultConfig);

        if (dumpMask & DUMP_ALL) {
            uint8_t activeProfile = masterConfig.current_profile_index;
            for (uint32_t profileCount=0; profileCount<MAX_PROFILE_COUNT;profileCount++) {
                cliDumpProfile(profileCount, dumpMask, &defaultConfig);

                uint8_t currentRateIndex = currentProfile->activeRateProfile;
                for (uint32_t rateCount = 0; rateCount<MAX_RATEPROFILES; rateCount++) {
                    cliDumpRateProfile(rateCount, dumpMask, &defaultConfig);
                }

                changeControlRateProfile(currentRateIndex);
                cliPrintHashLine("restore original rateprofile selection");
                cliRateProfile("");
            }

            changeProfile(activeProfile);
            cliPrintHashLine("restore original profile selection");
            cliProfile("");

            cliPrintHashLine("save configuration");
            cliPrint("save");
        } else {
            cliDumpProfile(masterConfig.current_profile_index, dumpMask, &defaultConfig);
            cliDumpRateProfile(currentProfile->activeRateProfile, dumpMask, &defaultConfig);
        }
    }

    if (dumpMask & DUMP_PROFILE) {
        cliDumpProfile(masterConfig.current_profile_index, dumpMask, &defaultConfig);
    }

    if (dumpMask & DUMP_RATES) {
        cliDumpRateProfile(currentProfile->activeRateProfile, dumpMask, &defaultConfig);
    }
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
    CLI_COMMAND_DEF("aux", "configure modes", NULL, cliAux),
#ifdef BEEPER
    CLI_COMMAND_DEF("beeper", "turn on/off beeper", "list\r\n"
        "\t<+|->[name]", cliBeeper),
#endif
#ifdef LED_STRIP
    CLI_COMMAND_DEF("color", "configure colors", NULL, cliColor),
#endif
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cliDefaults),
    CLI_COMMAND_DEF("dfu", "DFU mode on reboot", NULL, cliDfu),
    CLI_COMMAND_DEF("diff", "list configuration changes from default",
        "[master|profile|rates|all] {showdefaults}", cliDiff),
    CLI_COMMAND_DEF("dump", "dump configuration",
        "[master|profile|rates|all] {showdefaults}", cliDump),
#ifdef USE_ESCSERIAL
    CLI_COMMAND_DEF("escprog", "passthrough esc to serial", "<mode [sk/bl/ki/cc]> <index>", cliEscPassthrough),
#endif
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
    CLI_COMMAND_DEF("get", "get variable value", "[name]", cliGet),
#ifdef GPS
    CLI_COMMAND_DEF("gpspassthrough", "passthrough gps to serial", NULL, cliGpsPassthrough),
#endif
    CLI_COMMAND_DEF("help", NULL, NULL, cliHelp),
#ifdef LED_STRIP
    CLI_COMMAND_DEF("led", "configure leds", NULL, cliLed),
#endif
    CLI_COMMAND_DEF("map", "configure rc channel order", "[<map>]", cliMap),
#ifndef USE_QUAD_MIXER_ONLY
    CLI_COMMAND_DEF("mixer", "configure mixer", "list\r\n\t<name>", cliMixer),
#endif
    CLI_COMMAND_DEF("mmix", "custom motor mixer", NULL, cliMotorMix),
#ifdef LED_STRIP
    CLI_COMMAND_DEF("mode_color", "configure mode and special colors", NULL, cliModeColor),
#endif
    CLI_COMMAND_DEF("motor",  "get/set motor", "<index> [<value>]", cliMotor), 
    CLI_COMMAND_DEF("name", "name of craft", NULL, cliName),
#if (FLASH_SIZE > 128)
    CLI_COMMAND_DEF("play_sound", NULL, "[<index>]", cliPlaySound),
#endif
    CLI_COMMAND_DEF("profile", "change profile", "[<index>]", cliProfile),
    CLI_COMMAND_DEF("rateprofile", "change rate profile", "[<index>]", cliRateProfile),
#if defined(USE_RESOURCE_MGMT)
    CLI_COMMAND_DEF("resource", "show/set resources", NULL, cliResource),
#endif
    CLI_COMMAND_DEF("rxfail", "show/set rx failsafe settings", NULL, cliRxFail),
    CLI_COMMAND_DEF("rxrange", "configure rx channel ranges", NULL, cliRxRange),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
#ifdef USE_SDCARD
    CLI_COMMAND_DEF("sd_info", "sdcard info", NULL, cliSdInfo),
#endif
    CLI_COMMAND_DEF("serial", "configure serial ports", NULL, cliSerial),
#ifndef SKIP_SERIAL_PASSTHROUGH
    CLI_COMMAND_DEF("serialpassthrough", "passthrough serial data to port", "<id> [baud] [mode] : passthrough to serial", cliSerialPassthrough),
#endif
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("servo", "configure servos", NULL, cliServo),
#endif
    CLI_COMMAND_DEF("set", "change setting", "[<name>=<value>]", cliSet),
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("smix", "servo mixer", "<rule> <servo> <source> <rate> <speed> <min> <max> <box>\r\n"
        "\treset\r\n"
        "\tload <mixer>\r\n"
        "\treverse <servo> <source> r|n", cliServoMix),
#endif
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
#ifndef SKIP_TASK_STATISTICS
    CLI_COMMAND_DEF("tasks", "show task stats", NULL, cliTasks),
#endif
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
#ifdef VTX
    CLI_COMMAND_DEF("vtx", "vtx channels on switch", NULL, cliVtx),
#endif
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(clicmd_t))

static void cliHelp(char *cmdline)
{
    UNUSED(cmdline);

    for (uint32_t i = 0; i < CMD_COUNT; i++) {
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
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
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
                char *options;
                for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
                    if ((options = checkCommand(cliBuffer, cmd->name))) {
                        break;
            }
                }
                if(cmd < cmdTable + CMD_COUNT)
                    cmd->func(options);
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
    cliWriter = bufWriterInit(cliWriteBuffer, sizeof(cliWriteBuffer), (bufWrite_t)serialWriteBufShim, serialPort);

    schedulerSetCalulateTaskStatistics(masterConfig.task_statistics);

#ifndef CLI_MINIMAL_VERBOSITY
    cliPrint("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
#else
    cliPrint("\r\nCLI\r\n");
#endif
    cliPrompt();

    ENABLE_ARMING_FLAG(PREVENT_ARMING);
}

void cliInit(serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);
}
#endif // USE_CLI
