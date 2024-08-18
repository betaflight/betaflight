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

#include "platform.h"

#include "build/debug.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_fielddefs.h"

#include "cms/cms.h"

#include "common/utils.h"
#include "common/time.h"

#include "config/simplified_tuning.h"

#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/dshot_command.h"
#include "drivers/camera_control_impl.h"
#include "drivers/light_led.h"
#include "drivers/mco.h"
#include "drivers/pinio.h"
#include "drivers/sdio.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/gps_lap_timer.h"
#include "fc/parameter_names.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/dashboard.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "io/vtx_rtc6705.h"

#include "osd/osd.h"

#include "pg/adc.h"
#include "pg/beeper.h"
#include "pg/beeper_dev.h"
#include "pg/bus_i2c.h"
#include "pg/dashboard.h"
#include "pg/displayport_profiles.h"
#include "pg/dyn_notch.h"
#include "pg/flash.h"
#include "pg/gyrodev.h"
#include "pg/max7456.h"
#include "pg/mco.h"
#include "pg/motor.h"
#include "pg/msp.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "pg/rx.h"
#include "pg/rx_pwm.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"
#include "pg/rx_spi_expresslrs.h"
#include "pg/sdcard.h"
#include "pg/vcd.h"
#include "pg/vtx_io.h"
#include "pg/usb.h"
#include "pg/scheduler.h"
#include "pg/sdio.h"
#include "pg/rcdevice.h"
#include "pg/stats.h"
#include "pg/board.h"

#include "rx/a7105_flysky.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_sfhss.h"
#include "rx/crsf.h"
#include "rx/cyrf6936_spektrum.h"
#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"

#include "scheduler/scheduler.h"

#include "telemetry/frsky_hub.h"
#include "telemetry/ibus_shared.h"
#include "telemetry/telemetry.h"

#include "settings.h"


// Sensor names (used in lookup tables for *_hardware settings and in status command output)
// sync with accelerationSensor_e
const char * const lookupTableAccHardware[] = {
    "AUTO", "NONE", "ADXL345", "MPU6050", "MMA8452", "BMA280", "LSM303DLHC",
    "MPU6000", "MPU6500", "MPU9250", "ICM20601", "ICM20602", "ICM20608G", "ICM20649", "ICM20689", "ICM42605", "ICM42688P",
    "BMI160", "BMI270", "LSM6DSO", "LSM6DSV16X", "VIRTUAL"
};

// sync with gyroHardware_e
const char * const lookupTableGyroHardware[] = {
    "AUTO", "NONE", "MPU6050", "L3G4200D", "MPU3050", "L3GD20",
    "MPU6000", "MPU6500", "MPU9250", "ICM20601", "ICM20602", "ICM20608G", "ICM20649", "ICM20689", "ICM42605", "ICM42688P",
    "BMI160", "BMI270", "LSM6DSO", "LSM6DSV16X", "VIRTUAL"
};

#if defined(USE_SENSOR_NAMES) || defined(USE_BARO)
// sync with baroSensor_e
const char * const lookupTableBaroHardware[] = {
    "AUTO", "NONE", "BMP085", "MS5611", "BMP280", "LPS", "QMP6988", "BMP388", "DPS310", "2SMPB_02B", "LPS22DF", "VIRTUAL"
};
#endif
#if defined(USE_SENSOR_NAMES) || defined(USE_MAG)
// sync with magSensor_e
const char * const lookupTableMagHardware[] = {
    "AUTO", "NONE", "HMC5883", "AK8975", "AK8963", "QMC5883", "LIS2MDL", "LIS3MDL", "MPU925X_AK8963", "IST8310"
};
#endif
#if defined(USE_SENSOR_NAMES) || defined(USE_RANGEFINDER)
const char * const lookupTableRangefinderHardware[] = {
    "NONE", "HCSR04", "TFMINI", "TF02"
};
#endif

const char * const lookupTableOffOn[] = {
    "OFF", "ON"
};

static const char * const lookupTableCrashRecovery[] = {
    "OFF", "ON" ,"BEEP", "DISARM"
};

static const char * const lookupTableUnit[] = {
    "IMPERIAL", "METRIC", "BRITISH"
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
    "CW270FLIP",
    "CUSTOM",
};

#ifdef USE_MULTI_GYRO
static const char * const lookupTableGyro[] = {
    "FIRST", "SECOND", "BOTH"
};
#endif

#ifdef USE_GPS
static const char * const lookupTableGpsProvider[] = {
    "NMEA", "UBLOX", "MSP"
};

static const char * const lookupTableGpsSbasMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN", "NONE"
};

static const char * const lookupTableGpsUbloxModels[] = {
    "PORTABLE", "STATIONARY", "PEDESTRIAN", "AUTOMOTIVE", "AT_SEA", "AIRBORNE_1G", "AIRBORNE_2G", "AIRBORNE_4G"
};

static const char * const lookupTableGpsUbloxUtcStandard[] = {
    "AUTO", "USNO", "EU", "SU", "NTSC"
};
#endif

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

static const char * const lookupTableBlackboxSampleRate[] = {
    "1/1", "1/2", "1/4", "1/8", "1/16"
};
#endif

#ifdef USE_SERIALRX
static const char * const lookupTableSerialRX[] = {
    "NONE",
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
    "SRXL2",
    "GHST",
    "SPEK1024",
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
    "FLYSKY_2A",
    "KN",
    "SFHSS",
    "SPEKTRUM",
    "FRSKY_X_LBT",
    "REDPINE",
    "FRSKY_X_V2",
    "FRSKY_X_LBT_V2",
    "EXPRESSLRS",
};
#endif

static const char * const lookupTableGyroHardwareLpf[] = {
    "NORMAL",
    "OPTION_1",
    "OPTION_2",
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    "EXPERIMENTAL",
#endif
};

#ifdef USE_CAMERA_CONTROL
static const char * const lookupTableCameraControlMode[] = {
    "HARDWARE_PWM",
    "SOFTWARE_PWM",
    "DAC"
};
#endif

static const char * const lookupTablePwmProtocol[] = {
    "PWM", "ONESHOT125", "ONESHOT42", "MULTISHOT", "BRUSHED",
    "DSHOT150", "DSHOT300", "DSHOT600", "PROSHOT1000",
    "DISABLED"
};

static const char * const lookupTableLowpassType[] = {
    "PT1",
    "BIQUAD",
    "PT2",
    "PT3",
};

static const char * const lookupTableDtermLowpassType[] = {
    "PT1",
    "BIQUAD",
    "PT2",
    "PT3",
};

static const char * const lookupTableFailsafe[] = {
    "AUTO-LAND", "DROP", "GPS-RESCUE"
};

static const char * const lookupTableFailsafeSwitchMode[] = {
    "STAGE1", "KILL", "STAGE2"
};

static const char * const lookupTableBusType[] = {
    "NONE", "I2C", "SPI", "SLAVE",
#if defined(USE_SPI_GYRO) && defined(USE_I2C_GYRO)
    "GYROAUTO"
#endif
};

#ifdef USE_MAX7456
static const char * const lookupTableMax7456Clock[] = {
    "HALF", "NOMINAL", "DOUBLE"
};
#endif

#ifdef USE_RX_FRSKY_SPI
static const char * const lookupTableFrskySpiA1Source[] = {
    "VBAT", "EXTADC", "CONST"
};
#endif

#ifdef USE_GYRO_OVERFLOW_CHECK
static const char * const lookupTableGyroOverflowCheck[] = {
    "OFF", "YAW", "ALL"
};
#endif

static const char * const lookupTableRatesType[] = {
    "BETAFLIGHT", "RACEFLIGHT", "KISS", "ACTUAL", "QUICK"
};

#ifdef USE_OVERCLOCK
static const char * const lookupOverclock[] = {
    "OFF",
#if defined(STM32F40_41xxx) || defined(STM32G4)
    "192MHZ", "216MHZ", "240MHZ"
#elif defined(STM32F411xE)
    "108MHZ", "120MHZ"
#elif defined(STM32F7)
    "240MHZ"
#endif
};
#endif

#ifdef USE_LED_STRIP
    static const char * const lookupLedStripFormatRGB[] = {
        "GRB", "RGB", "GRBW"
    };
#endif

const char * const lookupTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};

#ifdef USE_GPS_RESCUE
static const char * const lookupTableRescueSanityType[] = {
    "RESCUE_SANITY_OFF", "RESCUE_SANITY_ON", "RESCUE_SANITY_FS_ONLY"
};
const char * const lookupTableRescueAltitudeMode[] = {
    "MAX_ALT", "FIXED_ALT", "CURRENT_ALT"
};
#endif

#if defined(USE_VIDEO_SYSTEM)
static const char * const lookupTableVideoSystem[] = {
    "AUTO", "PAL", "NTSC", "HD"
};
#endif

#if defined(USE_ITERM_RELAX)
const char * const lookupTableItermRelax[] = {
    "OFF", "RP", "RPY", "RP_INC", "RPY_INC"
};
const char * const lookupTableItermRelaxType[] = {
    "GYRO", "SETPOINT"
};
#endif

#ifdef USE_ACRO_TRAINER
static const char * const lookupTableAcroTrainerDebug[] = {
    "ROLL", "PITCH"
};
#endif // USE_ACRO_TRAINER

#ifdef USE_RC_SMOOTHING_FILTER
static const char * const lookupTableRcSmoothingDebug[] = {
    "ROLL", "PITCH", "YAW", "THROTTLE"
};
#endif // USE_RC_SMOOTHING_FILTER

#ifdef USE_VTX_COMMON
static const char * const lookupTableVtxLowPowerDisarm[] = {
    "OFF", "ON", "UNTIL_FIRST_ARM"
};
#endif

#ifdef USE_SDCARD
static const char * const lookupTableSdcardMode[] = {
    "OFF", "SPI", "SDIO"
};
#endif

#ifdef USE_LAUNCH_CONTROL
static const char * const lookupTableLaunchControlMode[] = {
    "NORMAL", "PITCHONLY", "FULL"
};
#endif

#ifdef USE_TPA_MODE
static const char * const lookupTableTpaMode[] = {
    "PD", "D"
};
#endif

static const char * const lookupTableSpaMode[] = {
    "OFF", "I_FREEZE", "I", "PID", "PD_I_FREEZE"
};

#ifdef USE_LED_STRIP
#ifdef USE_LED_STRIP_STATUS_MODE
static const char * const lookupTableLEDProfile[] = {
    "RACE", "BEACON", "STATUS"
};
#else
static const char * const lookupTableLEDProfile[] = {
    "RACE", "BEACON"
};
#endif
#endif

const char * const lookupTableLedstripColors[COLOR_COUNT] = {
    "BLACK",
    "WHITE",
    "RED",
    "ORANGE",
    "YELLOW",
    "LIME_GREEN",
    "GREEN",
    "MINT_GREEN",
    "CYAN",
    "LIGHT_BLUE",
    "BLUE",
    "DARK_VIOLET",
    "MAGENTA",
    "DEEP_PINK"
};

static const char * const lookupTableGyroFilterDebug[] = {
    "ROLL", "PITCH", "YAW"
};

static const char * const lookupTablePositionAltitudeSource[] = {
    "DEFAULT", "BARO_ONLY", "GPS_ONLY"
};

static const char * const lookupTableOffOnAuto[] = {
    "OFF", "ON", "AUTO"
};

const char* const lookupTableFeedforwardAveraging[] = {
    "OFF", "2_POINT", "3_POINT", "4_POINT"
};

static const char* const lookupTableDshotBitbangedTimer[] = {
    "AUTO", "TIM1", "TIM8"
};

const char * const lookupTableOsdDisplayPortDevice[] = {
    "NONE", "AUTO", "MAX7456", "MSP", "FRSKYOSD"
};

#ifdef USE_OSD
static const char * const lookupTableOsdLogoOnArming[] = {
    "OFF", "ON", "FIRST_ARMING",
};
#endif
const char * const lookupTableSimplifiedTuningPidsMode[] = {
    "OFF", "RP", "RPY",
};

const char* const lookupTableMixerType[] = {
    "LEGACY", "LINEAR", "DYNAMIC", "EZLANDING",
};

#ifdef USE_OSD
const char * const lookupTableCMSMenuBackgroundType[] = {
    "TRANSPARENT", "BLACK", "GRAY", "LIGHT_GRAY"
};
#endif

#ifdef USE_RX_EXPRESSLRS
static const char* const lookupTableFreqDomain[] = {
#ifdef USE_RX_SX127X
    "AU433", "AU915", "EU433", "EU868", "IN866", "FCC915",
#endif
#ifdef USE_RX_SX1280
    "ISM2400", "CE2400"
#endif
#if !defined(USE_RX_SX127X) && !defined(USE_RX_SX1280)
    "NONE",
#endif
};
#endif

#define LOOKUP_TABLE_ENTRY(name) { name, ARRAYLEN(name) }

const lookupTableEntry_t lookupTables[] = {
    LOOKUP_TABLE_ENTRY(lookupTableOffOn),
    LOOKUP_TABLE_ENTRY(lookupTableUnit),
    LOOKUP_TABLE_ENTRY(lookupTableAlignment),
#ifdef USE_GPS
    LOOKUP_TABLE_ENTRY(lookupTableGpsProvider),
    LOOKUP_TABLE_ENTRY(lookupTableGpsSbasMode),
    LOOKUP_TABLE_ENTRY(lookupTableGpsUbloxModels),
    LOOKUP_TABLE_ENTRY(lookupTableGpsUbloxUtcStandard),
#ifdef USE_GPS_RESCUE
    LOOKUP_TABLE_ENTRY(lookupTableRescueSanityType),
    LOOKUP_TABLE_ENTRY(lookupTableRescueAltitudeMode),
#endif
#endif
#ifdef USE_BLACKBOX
    LOOKUP_TABLE_ENTRY(lookupTableBlackboxDevice),
    LOOKUP_TABLE_ENTRY(lookupTableBlackboxMode),
    LOOKUP_TABLE_ENTRY(lookupTableBlackboxSampleRate),
#endif
    LOOKUP_TABLE_ENTRY(currentMeterSourceNames),
    LOOKUP_TABLE_ENTRY(voltageMeterSourceNames),
#ifdef USE_SERVOS
    LOOKUP_TABLE_ENTRY(lookupTableGimbalMode),
#endif
#ifdef USE_SERIALRX
    LOOKUP_TABLE_ENTRY(lookupTableSerialRX),
#endif
#ifdef USE_RX_SPI
    LOOKUP_TABLE_ENTRY(lookupTableRxSpi),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableGyroHardwareLpf),
    LOOKUP_TABLE_ENTRY(lookupTableAccHardware),
#ifdef USE_BARO
    LOOKUP_TABLE_ENTRY(lookupTableBaroHardware),
#endif
#ifdef USE_MAG
    LOOKUP_TABLE_ENTRY(lookupTableMagHardware),
#endif
    LOOKUP_TABLE_ENTRY(debugModeNames),
    LOOKUP_TABLE_ENTRY(lookupTablePwmProtocol),
    LOOKUP_TABLE_ENTRY(lookupTableLowpassType),
    LOOKUP_TABLE_ENTRY(lookupTableDtermLowpassType),
    LOOKUP_TABLE_ENTRY(lookupTableFailsafe),
    LOOKUP_TABLE_ENTRY(lookupTableFailsafeSwitchMode),
    LOOKUP_TABLE_ENTRY(lookupTableCrashRecovery),
#ifdef USE_CAMERA_CONTROL
    LOOKUP_TABLE_ENTRY(lookupTableCameraControlMode),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableBusType),
#ifdef USE_MAX7456
    LOOKUP_TABLE_ENTRY(lookupTableMax7456Clock),
#endif
#ifdef USE_RX_FRSKY_SPI
    LOOKUP_TABLE_ENTRY(lookupTableFrskySpiA1Source),
#endif
#ifdef USE_RANGEFINDER
    LOOKUP_TABLE_ENTRY(lookupTableRangefinderHardware),
#endif
#ifdef USE_GYRO_OVERFLOW_CHECK
    LOOKUP_TABLE_ENTRY(lookupTableGyroOverflowCheck),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableRatesType),
#ifdef USE_OVERCLOCK
    LOOKUP_TABLE_ENTRY(lookupOverclock),
#endif
#ifdef USE_LED_STRIP
    LOOKUP_TABLE_ENTRY(lookupLedStripFormatRGB),
#endif
#ifdef USE_MULTI_GYRO
    LOOKUP_TABLE_ENTRY(lookupTableGyro),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableThrottleLimitType),
#if defined(USE_VIDEO_SYSTEM)
    LOOKUP_TABLE_ENTRY(lookupTableVideoSystem),
#endif
#if defined(USE_ITERM_RELAX)
    LOOKUP_TABLE_ENTRY(lookupTableItermRelax),
    LOOKUP_TABLE_ENTRY(lookupTableItermRelaxType),
#endif
#ifdef USE_ACRO_TRAINER
    LOOKUP_TABLE_ENTRY(lookupTableAcroTrainerDebug),
#endif // USE_ACRO_TRAINER
#ifdef USE_RC_SMOOTHING_FILTER
    LOOKUP_TABLE_ENTRY(lookupTableRcSmoothingDebug),
#endif // USE_RC_SMOOTHING_FILTER
#ifdef USE_VTX_COMMON
    LOOKUP_TABLE_ENTRY(lookupTableVtxLowPowerDisarm),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableGyroHardware),
#ifdef USE_SDCARD
    LOOKUP_TABLE_ENTRY(lookupTableSdcardMode),
#endif
#ifdef USE_LAUNCH_CONTROL
    LOOKUP_TABLE_ENTRY(lookupTableLaunchControlMode),
#endif
#ifdef USE_TPA_MODE
    LOOKUP_TABLE_ENTRY(lookupTableTpaMode),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableSpaMode),
#ifdef USE_LED_STRIP
    LOOKUP_TABLE_ENTRY(lookupTableLEDProfile),
    LOOKUP_TABLE_ENTRY(lookupTableLedstripColors),
#endif

    LOOKUP_TABLE_ENTRY(lookupTableGyroFilterDebug),

    LOOKUP_TABLE_ENTRY(lookupTablePositionAltitudeSource),
    LOOKUP_TABLE_ENTRY(lookupTableOffOnAuto),
    LOOKUP_TABLE_ENTRY(lookupTableFeedforwardAveraging),
    LOOKUP_TABLE_ENTRY(lookupTableDshotBitbangedTimer),
    LOOKUP_TABLE_ENTRY(lookupTableOsdDisplayPortDevice),

#ifdef USE_OSD
    LOOKUP_TABLE_ENTRY(lookupTableOsdLogoOnArming),
#endif
    LOOKUP_TABLE_ENTRY(lookupTableMixerType),
    LOOKUP_TABLE_ENTRY(lookupTableSimplifiedTuningPidsMode),
#ifdef USE_OSD
    LOOKUP_TABLE_ENTRY(lookupTableCMSMenuBackgroundType),
#endif
#ifdef USE_RX_EXPRESSLRS
    LOOKUP_TABLE_ENTRY(lookupTableFreqDomain),
#endif
};

#undef LOOKUP_TABLE_ENTRY

const clivalue_t valueTable[] = {
// PG_GYRO_CONFIG
    { PARAM_NAME_GYRO_HARDWARE_LPF, VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_HARDWARE_LPF }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_hardware_lpf) },

#if defined(USE_GYRO_SPI_ICM20649)
    { "gyro_high_range",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_high_fsr) },
#endif

    { PARAM_NAME_GYRO_LPF1_TYPE,      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_LPF_TYPE }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf1_type) },
    { PARAM_NAME_GYRO_LPF1_STATIC_HZ, VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, LPF_MAX_HZ }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf1_static_hz) },

    { PARAM_NAME_GYRO_LPF2_TYPE,      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_LPF_TYPE }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf2_type) },
    { PARAM_NAME_GYRO_LPF2_STATIC_HZ, VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0,  LPF_MAX_HZ }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf2_static_hz) },

    { "gyro_notch1_hz",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, LPF_MAX_HZ }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_1) },
    { "gyro_notch1_cutoff",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, LPF_MAX_HZ }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_1) },
    { "gyro_notch2_hz",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, LPF_MAX_HZ }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_hz_2) },
    { "gyro_notch2_cutoff",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, LPF_MAX_HZ }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_soft_notch_cutoff_2) },

    { "gyro_calib_duration",        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50,  3000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroCalibrationDuration) },
    { "gyro_calib_noise_limit",     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0,  200 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyroMovementCalibrationThreshold) },
    { "gyro_offset_yaw",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -1000, 1000 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_offset_yaw) },
#ifdef USE_GYRO_OVERFLOW_CHECK
    { "gyro_overflow_detect",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_OVERFLOW_CHECK }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, checkOverflow) },
#endif
#ifdef USE_YAW_SPIN_RECOVERY
    { "yaw_spin_recovery",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON_AUTO }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, yaw_spin_recovery) },
    { "yaw_spin_threshold",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { YAW_SPIN_RECOVERY_THRESHOLD_MIN,  YAW_SPIN_RECOVERY_THRESHOLD_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, yaw_spin_threshold) },
#endif

#ifdef USE_MULTI_GYRO
    { PARAM_NAME_GYRO_TO_USE,       VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_to_use) },
#endif
#if defined(USE_DYN_NOTCH_FILTER)
    { PARAM_NAME_DYN_NOTCH_COUNT,   VAR_UINT8   | MASTER_VALUE, .config.minmaxUnsigned = { 0, DYN_NOTCH_COUNT_MAX }, PG_DYN_NOTCH_CONFIG, offsetof(dynNotchConfig_t, dyn_notch_count) },
    { PARAM_NAME_DYN_NOTCH_Q,       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 1000 }, PG_DYN_NOTCH_CONFIG, offsetof(dynNotchConfig_t, dyn_notch_q) },
    { PARAM_NAME_DYN_NOTCH_MIN_HZ,  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 20, 250 }, PG_DYN_NOTCH_CONFIG, offsetof(dynNotchConfig_t, dyn_notch_min_hz) },
    { PARAM_NAME_DYN_NOTCH_MAX_HZ,  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 200, 1000 }, PG_DYN_NOTCH_CONFIG, offsetof(dynNotchConfig_t, dyn_notch_max_hz) },
#endif
#ifdef USE_DYN_LPF
    { "gyro_lpf1_dyn_min_hz",       VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, DYN_LPF_MAX_HZ }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf1_dyn_min_hz) },
    { "gyro_lpf1_dyn_max_hz",       VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, DYN_LPF_MAX_HZ }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf1_dyn_max_hz) },
    { "gyro_lpf1_dyn_expo",         VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10 }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_lpf1_dyn_expo) },
#endif
    { "gyro_filter_debug_axis",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GYRO_FILTER_DEBUG }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, gyro_filter_debug_axis) },

// PG_ACCELEROMETER_CONFIG
#if defined(USE_ACC)
    { PARAM_NAME_ACC_HARDWARE,      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ACC_HARDWARE }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_hardware) },
#if defined(USE_GYRO_SPI_ICM20649)
    { "acc_high_range",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_high_fsr) },
#endif
    { PARAM_NAME_ACC_LPF_HZ,        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, acc_lpf_hz) },
    { "acc_trim_pitch",             VAR_INT16  | MASTER_VALUE, .config.minmax = { -300, 300 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accelerometerTrims.values.pitch) },
    { "acc_trim_roll",              VAR_INT16  | MASTER_VALUE, .config.minmax = { -300, 300 }, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accelerometerTrims.values.roll) },

    // 4 elements are output for the ACC calibration - The 3 axis values and the 4th representing whether calibration has been performed
    { "acc_calibration",            VAR_INT16  | MASTER_VALUE | MODE_ARRAY, .config.array.length = 4, PG_ACCELEROMETER_CONFIG, offsetof(accelerometerConfig_t, accZero.raw) },
#endif

// PG_COMPASS_CONFIG
#ifdef USE_MAG
    { "align_mag",                  VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_alignment) },
    { "mag_align_roll",             VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_customAlignment.roll) },
    { "mag_align_pitch",            VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_customAlignment.pitch) },
    { "mag_align_yaw",              VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_customAlignment.yaw) },
    { "mag_bustype",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_busType) },
    { "mag_i2c_device",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_i2c_device) },
    { "mag_i2c_address",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_i2c_address) },
    { "mag_spi_device",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_spi_device) },
    { PARAM_NAME_MAG_HARDWARE,      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAG_HARDWARE }, PG_COMPASS_CONFIG, offsetof(compassConfig_t, mag_hardware) },
    { "mag_calibration",            VAR_INT16  | MASTER_VALUE | MODE_ARRAY, .config.array.length = XYZ_AXIS_COUNT, PG_COMPASS_CONFIG, offsetof(compassConfig_t, magZero.raw) },
#endif

// PG_BAROMETER_CONFIG
#ifdef USE_BARO
    { "baro_bustype",               VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_busType) },
    { "baro_spi_device",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 5 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_spi_device) },
    { "baro_i2c_device",            VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 5 }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_i2c_device) },
    { "baro_i2c_address",           VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_i2c_address) },
    { PARAM_NAME_BARO_HARDWARE,     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BARO_HARDWARE }, PG_BAROMETER_CONFIG, offsetof(barometerConfig_t, baro_hardware) },
#endif

// PG_RX_CONFIG
    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1200, 1700 }, PG_RX_CONFIG, offsetof(rxConfig_t, midrc) },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, mincheck) },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, maxcheck) },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE, .config.minmax = { 0, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_channel) },
    { "rssi_src_frame_errors",      VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_src_frame_errors) },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { RSSI_SCALE_MIN, RSSI_SCALE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_scale) },
    { "rssi_offset",                VAR_INT8   | MASTER_VALUE, .config.minmax = { -100, 100 }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_offset) },
    { "rssi_invert",                VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_invert) },
    { "rssi_src_frame_lpf_period",  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_src_frame_lpf_period) },
    { "rssi_smoothing",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rssi_smoothing) },

#ifdef USE_RC_SMOOTHING_FILTER
    { PARAM_NAME_RC_SMOOTHING,                   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_mode) },
    { PARAM_NAME_RC_SMOOTHING_AUTO_FACTOR,       VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { RC_SMOOTHING_AUTO_FACTOR_MIN, RC_SMOOTHING_AUTO_FACTOR_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_auto_factor_rpy) },
    { PARAM_NAME_RC_SMOOTHING_AUTO_FACTOR_THROTTLE, VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { RC_SMOOTHING_AUTO_FACTOR_MIN, RC_SMOOTHING_AUTO_FACTOR_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_auto_factor_throttle) },
    { PARAM_NAME_RC_SMOOTHING_SETPOINT_CUTOFF,    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_setpoint_cutoff) },
    { PARAM_NAME_RC_SMOOTHING_FEEDFORWARD_CUTOFF, VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_feedforward_cutoff) },
    { PARAM_NAME_RC_SMOOTHING_THROTTLE_CUTOFF,    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_throttle_cutoff) },
    { PARAM_NAME_RC_SMOOTHING_DEBUG_AXIS,         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RC_SMOOTHING_DEBUG }, PG_RX_CONFIG, offsetof(rxConfig_t, rc_smoothing_debug_axis) },
#endif // USE_RC_SMOOTHING_FILTER

    { "fpv_mix_degrees",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_RX_CONFIG, offsetof(rxConfig_t, fpvCamAngleDegrees) },
    { "max_aux_channels",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, MAX_AUX_CHANNEL_COUNT }, PG_RX_CONFIG, offsetof(rxConfig_t, max_aux_channel) },
#ifdef USE_SERIALRX
    { PARAM_NAME_SERIAL_RX_PROVIDER, VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SERIAL_RX }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_provider) },
    { "serialrx_inverted",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, serialrx_inverted) },
#endif
#ifdef USE_SPEKTRUM_BIND
    { "spektrum_sat_bind",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { SPEKTRUM_SAT_BIND_DISABLED, SPEKTRUM_SAT_BIND_MAX}, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind) },
    { "spektrum_sat_bind_autoreset", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, spektrum_sat_bind_autoreset) },
#endif
#ifdef USE_SERIALRX_SRXL2
    { "srxl2_unit_id",               VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 0xf }, PG_RX_CONFIG, offsetof(rxConfig_t, srxl2_unit_id) },
    { "srxl2_baud_fast",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, srxl2_baud_fast) },
#endif
#if defined(USE_SERIALRX_SBUS)
    { "sbus_baud_fast",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, sbus_baud_fast) },
#endif
#if defined(USE_CRSF_V3)
    { "crsf_use_negotiated_baud",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, crsf_use_negotiated_baud) },
#endif
    { "airmode_start_throttle_percent", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_RX_CONFIG, offsetof(rxConfig_t, airModeActivateThreshold) },
    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_min_usec) },
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_RX_CONFIG, offsetof(rxConfig_t, rx_max_usec) },
    { "serialrx_halfduplex",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, halfDuplex) },
#if defined(USE_RX_MSP_OVERRIDE)
    { "msp_override_channels_mask",      VAR_UINT32 | MASTER_VALUE, .config.u32Max = (1 << MAX_SUPPORTED_RC_CHANNEL_COUNT) - 1, PG_RX_CONFIG, offsetof(rxConfig_t, msp_override_channels_mask)},
    { "msp_override_failsafe",      VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CONFIG, offsetof(rxConfig_t, msp_override_failsafe)},
#endif
#ifdef USE_RX_SPI
    { "rx_spi_protocol",            VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RX_SPI }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, rx_spi_protocol) },
    { "rx_spi_bus",                 VAR_UINT8   | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, spibus) },
    { "rx_spi_led_inversion",       VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_SPI_CONFIG, offsetof(rxSpiConfig_t, ledInversion) },
#endif

// PG_ADC_CONFIG
#if defined(USE_ADC)
    { "adc_device",                 VAR_INT8 | HARDWARE_VALUE, .config.minmax = { 0, ADCDEV_COUNT }, PG_ADC_CONFIG, offsetof(adcConfig_t, device) },
    { "adc_vrefint_calibration",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, vrefIntCalibration) },
    { "adc_tempsensor_calibration30", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, tempSensorCalibration1) },
    { "adc_tempsensor_calibration110", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_ADC_CONFIG, offsetof(adcConfig_t, tempSensorCalibration2) },
#endif

// PG_PWM_CONFIG
#if defined(USE_RX_PWM)
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  .config.lookup = { TABLE_OFF_ON }, PG_PWM_CONFIG, offsetof(pwmConfig_t, inputFilteringMode) },
#endif

// PG_BLACKBOX_CONFIG
#ifdef USE_BLACKBOX
    { "blackbox_sample_rate",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_SAMPLE_RATE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, sample_rate) },
    { "blackbox_device",            VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_DEVICE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, device) },
    { "blackbox_disable_pids",      VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_PID,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
    { "blackbox_disable_rc",        VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_RC_COMMANDS,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
    { "blackbox_disable_setpoint",  VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_SETPOINT,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
    { "blackbox_disable_bat",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_BATTERY,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
#ifdef USE_MAG
    { "blackbox_disable_mag",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_MAG,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
#endif
#if defined(USE_BARO) || defined(USE_RANGEFINDER)
    { "blackbox_disable_alt",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_ALTITUDE,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
#endif
    { "blackbox_disable_rssi",      VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_RSSI,       PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
    { "blackbox_disable_gyro",      VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_GYRO,       PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
    { "blackbox_disable_gyrounfilt",VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_GYROUNFILT, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
#if defined(USE_ACC)
    { "blackbox_disable_acc",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_ACC,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
#endif
    { "blackbox_disable_debug",     VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_DEBUG_LOG,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
    { "blackbox_disable_motors",    VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_MOTOR,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
#ifdef USE_DSHOT_TELEMETRY
    { "blackbox_disable_rpm",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_RPM,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
#endif
#ifdef USE_GPS
    { "blackbox_disable_gps",       VAR_UINT32 | MASTER_VALUE | MODE_BITSET, .config.bitpos = FLIGHT_LOG_FIELD_SELECT_GPS,   PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, fields_disabled_mask) },
#endif
    { "blackbox_mode",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BLACKBOX_MODE }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, mode) },
    { "blackbox_high_resolution",   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BLACKBOX_CONFIG, offsetof(blackboxConfig_t, high_resolution) },
#endif

// PG_MOTOR_CONFIG
    { "min_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, minthrottle) },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, maxthrottle) },
    { "min_command",                VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, mincommand) },
    { "motor_kv",                   VAR_UINT16 | HARDWARE_VALUE, .config.minmaxUnsigned = { 1, 40000 },                   PG_MOTOR_CONFIG, offsetof(motorConfig_t, kv) },
#ifdef USE_DSHOT
    { PARAM_NAME_DSHOT_IDLE_VALUE,  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, digitalIdleOffsetValue) },
#ifdef USE_DSHOT_DMAR
    { "dshot_burst",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON_AUTO }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useBurstDshot) },
#endif
#ifdef USE_DSHOT_TELEMETRY
    { PARAM_NAME_DSHOT_BIDIR,       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useDshotTelemetry) },
    { "dshot_edt",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useDshotEdt) },
#endif
#ifdef USE_DSHOT_BITBANG
    { "dshot_bitbang",               VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON_AUTO }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useDshotBitbang) },
    { "dshot_bitbang_timer",         VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DSHOT_BITBANGED_TIMER }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useDshotBitbangedTimer) },
#endif
#endif
    { PARAM_NAME_USE_UNSYNCED_PWM,  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.useUnsyncedPwm) },
    { PARAM_NAME_MOTOR_PWM_PROTOCOL, VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MOTOR_PWM_PROTOCOL }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmProtocol) },
    { PARAM_NAME_MOTOR_PWM_RATE,    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 200, 32000 }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmRate) },
    { "motor_pwm_inversion",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorPwmInversion) },
    { PARAM_NAME_MOTOR_POLES,       VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 4, UINT8_MAX }, PG_MOTOR_CONFIG, offsetof(motorConfig_t, motorPoleCount) },
    { "motor_output_reordering",    VAR_UINT8  | MASTER_VALUE | MODE_ARRAY, .config.array.length = MAX_SUPPORTED_MOTORS, PG_MOTOR_CONFIG, offsetof(motorConfig_t, dev.motorOutputReordering)},

// PG_THROTTLE_CORRECTION_CONFIG
    { "thr_corr_value",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0,  150 }, PG_THROTTLE_CORRECTION_CONFIG, offsetof(throttleCorrectionConfig_t, throttle_correction_value) },
    { "thr_corr_angle",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1,  900 }, PG_THROTTLE_CORRECTION_CONFIG, offsetof(throttleCorrectionConfig_t, throttle_correction_angle) },

// PG_FAILSAFE_CONFIG
    { "failsafe_delay",             VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { PERIOD_RXDATA_RECOVERY / MILLIS_PER_TENTH_SECOND, UINT16_MAX}, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_delay) },
    { "failsafe_off_delay",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_off_delay) },
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle) },
    { "failsafe_switch_mode",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FAILSAFE_SWITCH_MODE }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_switch_mode) },
    { "failsafe_throttle_low_delay",VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 300 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_throttle_low_delay) },
    { "failsafe_procedure",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FAILSAFE }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_procedure) },
    { "failsafe_recovery_delay",    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1, 200 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_recovery_delay) },
    { "failsafe_stick_threshold",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 50 }, PG_FAILSAFE_CONFIG, offsetof(failsafeConfig_t, failsafe_stick_threshold) },

// PG_BOARDALIGNMENT_CONFIG
    { "align_board_roll",           VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, rollDegrees) },
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, pitchDegrees) },
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE, .config.minmax = { -180, 360 }, PG_BOARD_ALIGNMENT, offsetof(boardAlignment_t, yawDegrees) },

// PG_GIMBAL_CONFIG
#ifdef USE_SERVOS
    { "gimbal_mode",                VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GIMBAL_MODE }, PG_GIMBAL_CONFIG, offsetof(gimbalConfig_t, mode) },
#endif

// PG_BATTERY_CONFIG
    { "bat_capacity",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 20000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, batteryCapacity) },
    { "vbat_max_cell_voltage",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmaxcellvoltage) },
    { "vbat_full_cell_voltage",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatfullcellvoltage) },
    { "vbat_min_cell_voltage",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatmincellvoltage) },
    { "vbat_warning_cell_voltage",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatwarningcellvoltage) },
    { "vbat_hysteresis",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbathysteresis) },
    { "current_meter",              VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CURRENT_METER }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, currentMeterSource) },
    { "battery_meter",              VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VOLTAGE_METER }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, voltageMeterSource) },
    { "vbat_detect_cell_voltage",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 2000 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatnotpresentcellvoltage) },
    { "use_vbat_alerts",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, useVBatAlerts) },
    { "use_cbat_alerts",            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, useConsumptionAlerts) },
    { "cbat_alert_percent",         VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, consumptionWarningPercentage) },
    { "vbat_cutoff_percent",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, lvcPercentage) },
    { "force_battery_cell_count",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 24 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, forceBatteryCellCount) },
    { "vbat_display_lpf_period",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, UINT8_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatDisplayLpfPeriod) },
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    { "vbat_sag_lpf_period",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, UINT8_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatSagLpfPeriod) },
#endif
    { "ibat_lpf_period",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, ibatLpfPeriod) },
    { "vbat_duration_for_warning",  VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 150 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatDurationForWarning) },
    { "vbat_duration_for_critical", VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, 150 }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, vbatDurationForCritical) },

//  PG_VOLTAGE_SENSOR_ADC_CONFIG
    { "vbat_scale",                 VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { VBAT_SCALE_MIN, VBAT_SCALE_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatscale) },
    { "vbat_divider",               VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_DIVIDER_MIN, VBAT_DIVIDER_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatresdivval) },
    { "vbat_multiplier",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { VBAT_MULTIPLIER_MIN, VBAT_MULTIPLIER_MAX }, PG_VOLTAGE_SENSOR_ADC_CONFIG, offsetof(voltageSensorADCConfig_t, vbatresdivmultiplier) },

// PG_CURRENT_SENSOR_ADC_CONFIG
    { "ibata_scale",                VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_ADC_CONFIG, offsetof(currentSensorADCConfig_t, scale) },
    { "ibata_offset",               VAR_INT16  | MASTER_VALUE, .config.minmax = { -32000, 32000 }, PG_CURRENT_SENSOR_ADC_CONFIG, offsetof(currentSensorADCConfig_t, offset) },
// PG_CURRENT_SENSOR_ADC_CONFIG
#ifdef USE_VIRTUAL_CURRENT_METER
    { "ibatv_scale",                VAR_INT16  | MASTER_VALUE, .config.minmax = { -16000, 16000 }, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, offsetof(currentSensorVirtualConfig_t, scale) },
    { "ibatv_offset",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_CURRENT_SENSOR_VIRTUAL_CONFIG, offsetof(currentSensorVirtualConfig_t, offset) },
#endif
#ifdef USE_BATTERY_CONTINUE
    { "battery_continue",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BATTERY_CONFIG, offsetof(batteryConfig_t, isBatteryContinueEnabled) },
#endif

#ifdef USE_BEEPER
// PG_BEEPER_DEV_CONFIG
    { "beeper_inversion",           VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, isInverted) },
    { "beeper_od",                  VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, isOpenDrain) },
    { "beeper_frequency",           VAR_INT16  | HARDWARE_VALUE, .config.minmax = { 0, 16000 }, PG_BEEPER_DEV_CONFIG, offsetof(beeperDevConfig_t, frequency) },

// PG_BEEPER_CONFIG
#ifdef USE_DSHOT
    { "beeper_dshot_beacon_tone",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = {1, DSHOT_CMD_BEACON5 }, PG_BEEPER_CONFIG, offsetof(beeperConfig_t, dshotBeaconTone) },
#endif
#endif // USE_BEEPER

// PG_MIXER_CONFIG
    { "yaw_motors_reversed",        VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, yaw_motors_reversed) },
    { PARAM_NAME_MIXER_TYPE,        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MIXER_TYPE }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, mixer_type) },
    { "crashflip_motor_percent",    VAR_UINT8 |  MASTER_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, crashflip_motor_percent) },
    { "crashflip_expo",             VAR_UINT8 |  MASTER_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, crashflip_expo) },
#ifdef USE_RPM_LIMIT
    { "rpm_limit",                  VAR_INT8   |  MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, rpm_limit) },
    { "rpm_limit_p",                VAR_UINT16 |  MASTER_VALUE,  .config.minmaxUnsigned = { 0, 100 },        PG_MIXER_CONFIG, offsetof(mixerConfig_t, rpm_limit_p) },
    { "rpm_limit_i",                VAR_UINT16 |  MASTER_VALUE,  .config.minmaxUnsigned = { 0, 1000 },       PG_MIXER_CONFIG, offsetof(mixerConfig_t, rpm_limit_i) },
    { "rpm_limit_d",                VAR_UINT16 |  MASTER_VALUE,  .config.minmaxUnsigned = { 0, 100 },        PG_MIXER_CONFIG, offsetof(mixerConfig_t, rpm_limit_d) },
    { "rpm_limit_value",            VAR_UINT16 |  MASTER_VALUE,  .config.minmaxUnsigned = { 1, UINT16_MAX }, PG_MIXER_CONFIG, offsetof(mixerConfig_t, rpm_limit_value) },
#endif

// PG_MOTOR_3D_CONFIG
    { "3d_deadband_low",            VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_RANGE_MIDDLE }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_low) },
    { "3d_deadband_high",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_RANGE_MIDDLE, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_high) },
    { "3d_neutral",                 VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, neutral3d) },
    { "3d_deadband_throttle",       VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1, 100 }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, deadband3d_throttle) },
    { "3d_limit_low",               VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_RANGE_MIDDLE }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, limit3d_low) },
    { "3d_limit_high",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_RANGE_MIDDLE, PWM_PULSE_MAX }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, limit3d_high) },
    { "3d_switched_mode",           VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MOTOR_3D_CONFIG, offsetof(flight3DConfig_t, switched_mode3d) },

// PG_SERVO_CONFIG
#ifdef USE_SERVOS
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { PWM_PULSE_MIN, PWM_PULSE_MAX }, PG_SERVO_CONFIG, offsetof(servoConfig_t, dev.servoCenterPulse) },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 498 }, PG_SERVO_CONFIG, offsetof(servoConfig_t, dev.servoPwmRate) },
    { "servo_lowpass_hz",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 400}, PG_SERVO_CONFIG, offsetof(servoConfig_t, servo_lowpass_freq) },
    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SERVO_CONFIG, offsetof(servoConfig_t, tri_unarmed_servo) },
    { "channel_forwarding_start",   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { AUX1, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_SERVO_CONFIG, offsetof(servoConfig_t, channelForwardingStartChannel) },
#endif

// PG_CONTROLRATE_PROFILES
#ifdef USE_PROFILE_NAMES
    { "rateprofile_name",           VAR_UINT8  | PROFILE_RATE_VALUE | MODE_STRING, .config.string = { 1, MAX_RATE_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, profileName) },
#endif
    { PARAM_NAME_THR_MID,           VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrMid8) },
    { PARAM_NAME_THR_EXPO,          VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, thrExpo8) },
    { PARAM_NAME_RATES_TYPE,        VAR_UINT8  | PROFILE_RATE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RATES_TYPE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates_type) },
    { "quickrates_rc_expo",         VAR_UINT8  | PROFILE_RATE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, quickRatesRcExpo) },
    { "roll_rc_rate",               VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_ROLL]) },
    { "pitch_rc_rate",              VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_PITCH]) },
    { "yaw_rc_rate",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 1, CONTROL_RATE_CONFIG_RC_RATES_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcRates[FD_YAW]) },
    { "roll_expo",                  VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_ROLL]) },
    { "pitch_expo",                 VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_PITCH]) },
    { "yaw_expo",                   VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rcExpo[FD_YAW]) },
    { "roll_srate",                 VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_ROLL]) },
    { "pitch_srate",                VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_PITCH]) },
    { "yaw_srate",                  VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 0, CONTROL_RATE_CONFIG_RATE_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rates[FD_YAW]) },
    { PARAM_NAME_THROTTLE_LIMIT_TYPE,    VAR_UINT8  | PROFILE_RATE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_THROTTLE_LIMIT_TYPE }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, throttle_limit_type) },
    { PARAM_NAME_THROTTLE_LIMIT_PERCENT, VAR_UINT8  | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { 25, 100 }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, throttle_limit_percent) },
    { "roll_rate_limit",            VAR_UINT16 | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { CONTROL_RATE_CONFIG_RATE_LIMIT_MIN, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rate_limit[FD_ROLL]) },
    { "pitch_rate_limit",           VAR_UINT16 | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { CONTROL_RATE_CONFIG_RATE_LIMIT_MIN, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rate_limit[FD_PITCH]) },
    { "yaw_rate_limit",             VAR_UINT16 | PROFILE_RATE_VALUE, .config.minmaxUnsigned = { CONTROL_RATE_CONFIG_RATE_LIMIT_MIN, CONTROL_RATE_CONFIG_RATE_LIMIT_MAX }, PG_CONTROL_RATE_PROFILES, offsetof(controlRateConfig_t, rate_limit[FD_YAW]) },

// PG_SERIAL_CONFIG
    { "reboot_character",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 48, 126 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, reboot_character) },
    { "serial_update_rate_hz",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 2000 }, PG_SERIAL_CONFIG, offsetof(serialConfig_t, serial_update_rate_hz) },

// PG_IMU_CONFIG
    { PARAM_NAME_IMU_DCM_KP,          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, imu_dcm_kp) },
    { PARAM_NAME_IMU_DCM_KI,          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32000 }, PG_IMU_CONFIG, offsetof(imuConfig_t, imu_dcm_ki) },
    { PARAM_NAME_IMU_SMALL_ANGLE,     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0,   180 }, PG_IMU_CONFIG, offsetof(imuConfig_t, small_angle) },
    { PARAM_NAME_IMU_PROCESS_DENOM,   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1,     4 }, PG_IMU_CONFIG, offsetof(imuConfig_t, imu_process_denom) },
#ifdef USE_MAG
    { PARAM_NAME_IMU_MAG_DECLINATION, VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0,  3599 }, PG_IMU_CONFIG, offsetof(imuConfig_t, mag_declination) },
#endif

// PG_ARMING_CONFIG
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 60 }, PG_ARMING_CONFIG, offsetof(armingConfig_t, auto_disarm_delay) },
    { PARAM_NAME_GYRO_CAL_ON_FIRST_ARM, VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ARMING_CONFIG, offsetof(armingConfig_t, gyro_cal_on_first_arm) },

// PG_GPS_CONFIG
#ifdef USE_GPS
    { PARAM_NAME_GPS_PROVIDER,               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_PROVIDER },   PG_GPS_CONFIG, offsetof(gpsConfig_t, provider) },
    { PARAM_NAME_GPS_SBAS_MODE,              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_SBAS_MODE },  PG_GPS_CONFIG, offsetof(gpsConfig_t, sbasMode) },
    { PARAM_NAME_GPS_AUTO_CONFIG,            VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },         PG_GPS_CONFIG, offsetof(gpsConfig_t, autoConfig) },
    { PARAM_NAME_GPS_AUTO_BAUD,              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },         PG_GPS_CONFIG, offsetof(gpsConfig_t, autoBaud) },
    { PARAM_NAME_GPS_UBLOX_ACQUIRE_MODEL,    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_UBLOX_MODELS }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_ublox_acquire_model) },
    { PARAM_NAME_GPS_UBLOX_FLIGHT_MODEL,     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_UBLOX_MODELS }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_ublox_flight_model) },
    { PARAM_NAME_GPS_UPDATE_RATE_HZ,         VAR_UINT8  | MASTER_VALUE,               .config.minmaxUnsigned = {1, 20},          PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_update_rate_hz) },
    { PARAM_NAME_GPS_UBLOX_UTC_STANDARD,     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_UBLOX_UTC_STANDARD }, PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_ublox_utc_standard) },
    { PARAM_NAME_GPS_UBLOX_USE_GALILEO,      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },         PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_ublox_use_galileo) },
    { PARAM_NAME_GPS_SET_HOME_POINT_ONCE,    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },         PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_set_home_point_once) },
    { PARAM_NAME_GPS_USE_3D_SPEED,           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },         PG_GPS_CONFIG, offsetof(gpsConfig_t, gps_use_3d_speed) },
    { PARAM_NAME_GPS_SBAS_INTEGRITY,         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON },         PG_GPS_CONFIG, offsetof(gpsConfig_t, sbas_integrity) },
    { PARAM_NAME_GPS_NMEA_CUSTOM_COMMANDS,   VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, NMEA_CUSTOM_COMMANDS_MAX_LENGTH, STRING_FLAGS_NONE }, PG_GPS_CONFIG, offsetof(gpsConfig_t, nmeaCustomCommands) },

#ifdef USE_GPS_RESCUE
    // PG_GPS_RESCUE
    { PARAM_NAME_GPS_RESCUE_MIN_START_DIST,  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 10, 30 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, minStartDistM) },
    { PARAM_NAME_GPS_RESCUE_ALT_MODE,        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_RESCUE_ALT_MODE }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, altitudeMode) },
    { PARAM_NAME_GPS_RESCUE_INITIAL_CLIMB,   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, initialClimbM) },
    { PARAM_NAME_GPS_RESCUE_ASCEND_RATE,     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 2500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, ascendRate) },

    { PARAM_NAME_GPS_RESCUE_RETURN_ALT,      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 1000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, returnAltitudeM) },
    { PARAM_NAME_GPS_RESCUE_GROUND_SPEED,    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 3000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, groundSpeedCmS) },
    { PARAM_NAME_GPS_RESCUE_MAX_RESCUE_ANGLE, VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 30, 60 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, maxRescueAngle) },
    { PARAM_NAME_GPS_RESCUE_ROLL_MIX,        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, rollMix) },
    { PARAM_NAME_GPS_RESCUE_PITCH_CUTOFF,    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 10, 255 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, pitchCutoffHz) },
    { PARAM_NAME_GPS_RESCUE_IMU_YAW_GAIN,    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 20 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, imuYawGain) },

    { PARAM_NAME_GPS_RESCUE_DESCENT_DIST,    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 10, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, descentDistanceM) },
    { PARAM_NAME_GPS_RESCUE_DESCEND_RATE,    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 25, 500 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, descendRate) },
    { PARAM_NAME_GPS_RESCUE_LANDING_ALT,     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 15 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, targetLandingAltitudeM) },
    { PARAM_NAME_GPS_RESCUE_DISARM_THRESHOLD, VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 250 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, disarmThreshold) },

    { PARAM_NAME_GPS_RESCUE_THROTTLE_MIN,    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleMin) },
    { PARAM_NAME_GPS_RESCUE_THROTTLE_MAX,    VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleMax) },
    { PARAM_NAME_GPS_RESCUE_THROTTLE_HOVER,  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1000, 2000 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleHover) },

    { PARAM_NAME_GPS_RESCUE_SANITY_CHECKS,   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_GPS_RESCUE_SANITY_CHECK }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, sanityChecks) },
    { PARAM_NAME_GPS_RESCUE_MIN_SATS,        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 50 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, minSats) },
    { PARAM_NAME_GPS_RESCUE_ALLOW_ARMING_WITHOUT_FIX, VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, allowArmingWithoutFix) },

    { PARAM_NAME_GPS_RESCUE_THROTTLE_P,      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleP) },
    { PARAM_NAME_GPS_RESCUE_THROTTLE_I,      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleI) },
    { PARAM_NAME_GPS_RESCUE_THROTTLE_D,      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, throttleD) },
    { PARAM_NAME_GPS_RESCUE_VELOCITY_P,      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velP) },
    { PARAM_NAME_GPS_RESCUE_VELOCITY_I,      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velI) },
    { PARAM_NAME_GPS_RESCUE_VELOCITY_D,      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, velD) },
    { PARAM_NAME_GPS_RESCUE_YAW_P,           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, yawP) },

#ifdef USE_MAG
    { PARAM_NAME_GPS_RESCUE_USE_MAG,         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GPS_RESCUE, offsetof(gpsRescueConfig_t, useMag) },
#endif // USE_MAG
#endif // USE_GPS_RESCUE

#ifdef USE_GPS_LAP_TIMER
    { PARAM_NAME_GPS_LAP_TIMER_GATE_LAT,       VAR_INT32  | MASTER_VALUE, .config.d32Max = 900000000,           PG_GPS_LAP_TIMER, offsetof(gpsLapTimerConfig_t, gateLat) },
    { PARAM_NAME_GPS_LAP_TIMER_GATE_LON,       VAR_INT32  | MASTER_VALUE, .config.d32Max = 1800000000,          PG_GPS_LAP_TIMER, offsetof(gpsLapTimerConfig_t, gateLon) },
    { PARAM_NAME_GPS_LAP_TIMER_MIN_LAP_TIME,   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 3000 }, PG_GPS_LAP_TIMER, offsetof(gpsLapTimerConfig_t, minimumLapTimeSeconds) },
    { PARAM_NAME_GPS_LAP_TIMER_GATE_TOLERANCE, VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 100 },  PG_GPS_LAP_TIMER, offsetof(gpsLapTimerConfig_t, gateToleranceM) },
#endif // USE_GPS_LAP_TIMER

#endif // USE_GPS

    { PARAM_NAME_DEADBAND,          VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 32 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, deadband) },
    { PARAM_NAME_YAW_DEADBAND,      VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_deadband) },
    { "yaw_control_reversed",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RC_CONTROLS_CONFIG, offsetof(rcControlsConfig_t, yaw_control_reversed) },

// PG_PID_CONFIG
    { PARAM_NAME_PID_PROCESS_DENOM, VAR_UINT8  | MASTER_VALUE,  .config.minmaxUnsigned = { 1, MAX_PID_PROCESS_DENOM }, PG_PID_CONFIG, offsetof(pidConfig_t, pid_process_denom) },
#ifdef USE_RUNAWAY_TAKEOFF
    { "runaway_takeoff_prevention", VAR_UINT8  | MODE_LOOKUP,  .config.lookup = { TABLE_OFF_ON }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_prevention) },    // enables/disables runaway takeoff prevention
    { "runaway_takeoff_deactivate_delay",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 100, 1000 }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_deactivate_delay) },           // deactivate time in ms
    { "runaway_takeoff_deactivate_throttle_percent",  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_CONFIG, offsetof(pidConfig_t, runaway_takeoff_deactivate_throttle) }, // minimum throttle percentage during deactivation phase
#endif

// PG_PID_PROFILE
#ifdef USE_PROFILE_NAMES
    { "profile_name",               VAR_UINT8  | PROFILE_VALUE | MODE_STRING, .config.string = { 1, MAX_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PID_PROFILE, offsetof(pidProfile_t, profileName) },
#endif
#ifdef USE_DYN_LPF
    { "dterm_lpf1_dyn_min_hz",      VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, DYN_LPF_MAX_HZ }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf1_dyn_min_hz) },
    { "dterm_lpf1_dyn_max_hz",      VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, DYN_LPF_MAX_HZ }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf1_dyn_max_hz) },
    { "dterm_lpf1_dyn_expo",        VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 10 }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf1_dyn_expo) },
#endif
    { PARAM_NAME_DTERM_LPF1_TYPE,       VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DTERM_LPF_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf1_type) },
    { PARAM_NAME_DTERM_LPF1_STATIC_HZ,  VAR_INT16  | PROFILE_VALUE, .config.minmax = { 0, LPF_MAX_HZ }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf1_static_hz) },
    { PARAM_NAME_DTERM_LPF2_TYPE,       VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DTERM_LPF_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf2_type) },
    { PARAM_NAME_DTERM_LPF2_STATIC_HZ,  VAR_INT16  | PROFILE_VALUE, .config.minmax = { 0, LPF_MAX_HZ }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_lpf2_static_hz) },
    { PARAM_NAME_DTERM_NOTCH_HZ,        VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, LPF_MAX_HZ }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_notch_hz) },
    { PARAM_NAME_DTERM_NOTCH_CUTOFF,    VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, LPF_MAX_HZ }, PG_PID_PROFILE, offsetof(pidProfile_t, dterm_notch_cutoff) },
#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
    { PARAM_NAME_VBAT_SAG_COMPENSATION, VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 150 }, PG_PID_PROFILE, offsetof(pidProfile_t, vbat_sag_compensation) },
#endif
    { PARAM_NAME_PID_AT_MIN_THROTTLE, VAR_UINT8 | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, pidAtMinThrottle) },
    { PARAM_NAME_ANTI_GRAVITY_GAIN,   VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { ITERM_ACCELERATOR_GAIN_OFF, ITERM_ACCELERATOR_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, anti_gravity_gain) },
    { PARAM_NAME_ANTI_GRAVITY_CUTOFF_HZ, VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 2, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, anti_gravity_cutoff_hz) },
    { PARAM_NAME_ANTI_GRAVITY_P_GAIN, VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, anti_gravity_p_gain) },
    { PARAM_NAME_ACC_LIMIT_YAW,     VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, yawRateAccelLimit) },
    { PARAM_NAME_ACC_LIMIT,         VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, rateAccelLimit) },
    { "crash_dthreshold",           VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_dthreshold) },
    { "crash_gthreshold",           VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 100, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_gthreshold) },
    { "crash_setpoint_threshold",   VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 50, 2000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_setpoint_threshold) },
    { "crash_time",                 VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 100, 5000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_time) },
    { "crash_delay",                VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_delay) },
    { "crash_recovery_angle",       VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 5, 30 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery_angle) },
    { "crash_recovery_rate",        VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 50, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery_rate) },
    { "crash_limit_yaw",            VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_limit_yaw) },
    { "crash_recovery",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CRASH_RECOVERY }, PG_PID_PROFILE, offsetof(pidProfile_t, crash_recovery) },

    { "iterm_rotation",             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_rotation) },
#if defined(USE_ITERM_RELAX)
    { PARAM_NAME_ITERM_RELAX,        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ITERM_RELAX }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax) },
    { PARAM_NAME_ITERM_RELAX_TYPE,   VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ITERM_RELAX_TYPE }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax_type) },
    { PARAM_NAME_ITERM_RELAX_CUTOFF, VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, iterm_relax_cutoff) },
#endif
    { PARAM_NAME_ITERM_WINDUP,      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 30, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermWindupPointPercent) },
    { "iterm_limit",                VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, itermLimit) },
    { PARAM_NAME_PIDSUM_LIMIT,      VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PIDSUM_LIMIT_MIN, PIDSUM_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimit) },
    { PARAM_NAME_PIDSUM_LIMIT_YAW,  VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PIDSUM_LIMIT_MIN, PIDSUM_LIMIT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pidSumLimitYaw) },
    { PARAM_NAME_YAW_LOWPASS_HZ,    VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_PID_PROFILE, offsetof(pidProfile_t, yaw_lowpass_hz) },

#if defined(USE_THROTTLE_BOOST)
    { PARAM_NAME_THROTTLE_BOOST,             VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, throttle_boost) },
    { PARAM_NAME_THROTTLE_BOOST_CUTOFF,      VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 5, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, throttle_boost_cutoff) },
#endif

#ifdef USE_ACRO_TRAINER
    { "acro_trainer_angle_limit",   VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 80 }, PG_PID_PROFILE, offsetof(pidProfile_t, acro_trainer_angle_limit) },
    { "acro_trainer_lookahead_ms",  VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, acro_trainer_lookahead_ms) },
    { "acro_trainer_debug_axis",    VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ACRO_TRAINER_DEBUG }, PG_PID_PROFILE, offsetof(pidProfile_t, acro_trainer_debug_axis) },
    { "acro_trainer_gain",          VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 25, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, acro_trainer_gain) },
#endif // USE_ACRO_TRAINER

    { "p_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].P) },
    { "i_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].I) },
    { "d_pitch",                    VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].D) },
    { "f_pitch",                    VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, F_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].F) },
    { "p_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].P) },
    { "i_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].I) },
    { "d_roll",                     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].D) },
    { "f_roll",                     VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, F_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].F) },
    { "p_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].P) },
    { "i_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].I) },
    { "d_yaw",                      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].D) },
    { "f_yaw",                      VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, F_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].F) },

#ifdef USE_WING
    { PARAM_NAME_S_PITCH,           VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_PITCH].S) },
    { PARAM_NAME_S_ROLL,            VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_ROLL].S) },
    { PARAM_NAME_S_YAW,             VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, PID_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_YAW].S) },
#endif // #ifdef USE_WING

    { PARAM_NAME_ANGLE_P_GAIN,          VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].P) },
    { PARAM_NAME_ANGLE_FEEDFORWARD,     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].F) },
    { PARAM_NAME_ANGLE_FF_SMOOTHING_MS, VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, angle_feedforward_smoothing_ms) },
    { PARAM_NAME_ANGLE_LIMIT,           VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 85 }, PG_PID_PROFILE, offsetof(pidProfile_t, angle_limit) },
    { PARAM_NAME_ANGLE_EARTH_REF,       VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, angle_earth_ref) },

    { PARAM_NAME_HORIZON_LEVEL_STRENGTH, VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].I) },
    { PARAM_NAME_HORIZON_LIMIT_STICKS,   VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, pid[PID_LEVEL].D) },
    { PARAM_NAME_HORIZON_LIMIT_DEGREES,  VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 10,  250 }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_limit_degrees) },
    { PARAM_NAME_HORIZON_IGNORE_STICKS,  VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_ignore_sticks) },
    { PARAM_NAME_HORIZON_DELAY_MS,       VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 10,  5000 }, PG_PID_PROFILE, offsetof(pidProfile_t, horizon_delay_ms) },

#if defined(USE_ABSOLUTE_CONTROL)
    { PARAM_NAME_ABS_CONTROL_GAIN,  VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 20 }, PG_PID_PROFILE, offsetof(pidProfile_t, abs_control_gain) },
    { "abs_control_limit",          VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 10, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, abs_control_limit) },
    { "abs_control_error_limit",    VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 1, 45 }, PG_PID_PROFILE, offsetof(pidProfile_t, abs_control_error_limit) },
    { "abs_control_cutoff",         VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 1, 45 }, PG_PID_PROFILE, offsetof(pidProfile_t, abs_control_cutoff) },
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    { PARAM_NAME_USE_INTEGRATED_YAW, VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, use_integrated_yaw) },
    { "integrated_yaw_relax",        VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, integrated_yaw_relax) },
#endif

#ifdef USE_D_MIN
    { "d_min_roll",                 VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, D_MIN_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min[FD_ROLL]) },
    { "d_min_pitch",                VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, D_MIN_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min[FD_PITCH]) },
    { "d_min_yaw",                  VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, D_MIN_GAIN_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min[FD_YAW]) },
    { PARAM_NAME_D_MAX_GAIN,        VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min_gain) },
    { PARAM_NAME_D_MAX_ADVANCE,     VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, d_min_advance) },
#endif

    { PARAM_NAME_MOTOR_OUTPUT_LIMIT, VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { MOTOR_OUTPUT_LIMIT_PERCENT_MIN, MOTOR_OUTPUT_LIMIT_PERCENT_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, motor_output_limit) },
    { "auto_profile_cell_count",     VAR_INT8 | PROFILE_VALUE,  .config.minmax = { AUTO_PROFILE_CELL_COUNT_CHANGE, MAX_AUTO_DETECT_CELL_COUNT }, PG_PID_PROFILE, offsetof(pidProfile_t, auto_profile_cell_count) },

#ifdef USE_LAUNCH_CONTROL
    { "launch_control_mode",        VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LAUNCH_CONTROL_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlMode) },
    { "launch_trigger_allow_reset", VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlAllowTriggerReset) },
    { "launch_trigger_throttle_percent", VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlThrottlePercent) },
    { "launch_angle_limit",         VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 80 }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlAngleLimit) },
    { "launch_control_gain",        VAR_UINT8 | PROFILE_VALUE,  .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, launchControlGain) },
#endif

#ifdef USE_THRUST_LINEARIZATION
    { "thrust_linear",              VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 150 }, PG_PID_PROFILE, offsetof(pidProfile_t, thrustLinearization) },
#endif

#ifdef USE_AIRMODE_LPF
    { "transient_throttle_limit",   VAR_UINT8 | PROFILE_VALUE, .config.minmax = { 0, 30 }, PG_PID_PROFILE, offsetof(pidProfile_t, transient_throttle_limit) },
#endif

#ifdef USE_FEEDFORWARD
    { PARAM_NAME_FEEDFORWARD_TRANSITION,     VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_PID_PROFILE, offsetof(pidProfile_t, feedforward_transition) },
    { PARAM_NAME_FEEDFORWARD_AVERAGING,      VAR_UINT8 | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FEEDFORWARD_AVERAGING }, PG_PID_PROFILE, offsetof(pidProfile_t, feedforward_averaging) },
    { PARAM_NAME_FEEDFORWARD_SMOOTH_FACTOR,  VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = {0, 95}, PG_PID_PROFILE, offsetof(pidProfile_t, feedforward_smooth_factor) },
    { PARAM_NAME_FEEDFORWARD_JITTER_FACTOR,  VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = {0, 20}, PG_PID_PROFILE, offsetof(pidProfile_t, feedforward_jitter_factor) },
    { PARAM_NAME_FEEDFORWARD_BOOST,          VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 50 }, PG_PID_PROFILE, offsetof(pidProfile_t, feedforward_boost) },
    { PARAM_NAME_FEEDFORWARD_MAX_RATE_LIMIT, VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = {0, 200}, PG_PID_PROFILE, offsetof(pidProfile_t, feedforward_max_rate_limit) },
#endif

#ifdef USE_DYN_IDLE
    { PARAM_NAME_DYN_IDLE_MIN_RPM,           VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_min_rpm) },
    { PARAM_NAME_DYN_IDLE_P_GAIN,            VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_p_gain) },
    { PARAM_NAME_DYN_IDLE_I_GAIN,            VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 1, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_i_gain) },
    { PARAM_NAME_DYN_IDLE_D_GAIN,            VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_d_gain) },
    { PARAM_NAME_DYN_IDLE_MAX_INCREASE,      VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_max_increase) },
    { PARAM_NAME_DYN_IDLE_START_INCREASE,    VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 10, 255 }, PG_PID_PROFILE, offsetof(pidProfile_t, dyn_idle_start_increase) },
#endif
    { "level_race_mode",            VAR_UINT8 | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, level_race_mode) },

#ifdef USE_SIMPLIFIED_TUNING
    { PARAM_NAME_SIMPLIFIED_PIDS_MODE,               VAR_UINT8 | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SIMPLIFIED_TUNING_PIDS_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_pids_mode) },
    { PARAM_NAME_SIMPLIFIED_MASTER_MULTIPLIER,       VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_master_multiplier) },
    { PARAM_NAME_SIMPLIFIED_I_GAIN,                  VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_i_gain) },
    { PARAM_NAME_SIMPLIFIED_D_GAIN,                  VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_d_gain) },
    { PARAM_NAME_SIMPLIFIED_PI_GAIN,                 VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_pi_gain) },
    { PARAM_NAME_SIMPLIFIED_DMAX_GAIN,               VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_dmin_ratio) },
    { PARAM_NAME_SIMPLIFIED_FEEDFORWARD_GAIN,        VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_feedforward_gain) },
    { PARAM_NAME_SIMPLIFIED_PITCH_D_GAIN,            VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_roll_pitch_ratio) },
    { PARAM_NAME_SIMPLIFIED_PITCH_PI_GAIN,           VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { SIMPLIFIED_TUNING_PIDS_MIN, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_pitch_pi_gain) },

    { PARAM_NAME_SIMPLIFIED_DTERM_FILTER,            VAR_UINT8 | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_dterm_filter) },
    { PARAM_NAME_SIMPLIFIED_DTERM_FILTER_MULTIPLIER, VAR_UINT8 | PROFILE_VALUE, .config.minmaxUnsigned = { SIMPLIFIED_TUNING_FILTERS_MIN, SIMPLIFIED_TUNING_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, simplified_dterm_filter_multiplier) },

    { PARAM_NAME_SIMPLIFIED_GYRO_FILTER,             VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, simplified_gyro_filter) },
    { PARAM_NAME_SIMPLIFIED_GYRO_FILTER_MULTIPLIER,  VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { SIMPLIFIED_TUNING_FILTERS_MIN, SIMPLIFIED_TUNING_MAX }, PG_GYRO_CONFIG, offsetof(gyroConfig_t, simplified_gyro_filter_multiplier) },
#endif
#ifdef USE_TPA_MODE
    { PARAM_NAME_TPA_MODE,             VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_TPA_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_mode) },
#endif
    { PARAM_NAME_TPA_RATE,          VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, TPA_MAX}, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_rate) },
    { PARAM_NAME_TPA_BREAKPOINT,    VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PWM_RANGE_MIN, PWM_RANGE_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_breakpoint) },
    { PARAM_NAME_TPA_LOW_RATE,            VAR_INT8  | PROFILE_VALUE, .config.minmax = { TPA_LOW_RATE_MIN, TPA_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_low_rate) },
    { PARAM_NAME_TPA_LOW_BREAKPOINT,      VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { PWM_RANGE_MIN, PWM_RANGE_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_low_breakpoint) },
    { PARAM_NAME_TPA_LOW_ALWAYS, VAR_UINT8  | PROFILE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_low_always) },

#ifdef USE_WING
    { PARAM_NAME_TPA_DELAY_MS, VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_delay_ms) },
    { PARAM_NAME_TPA_GRAVITY_THR0, VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, TPA_GRAVITY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_gravity_thr0) },
    { PARAM_NAME_TPA_GRAVITY_THR100, VAR_UINT16 | PROFILE_VALUE, .config.minmaxUnsigned = { 0, TPA_GRAVITY_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, tpa_gravity_thr100) },
#endif

    { PARAM_NAME_EZ_LANDING_THRESHOLD,      VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 200 }, PG_PID_PROFILE, offsetof(pidProfile_t, ez_landing_threshold) },
    { PARAM_NAME_EZ_LANDING_LIMIT,          VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 75 }, PG_PID_PROFILE, offsetof(pidProfile_t, ez_landing_limit) },
    { PARAM_NAME_EZ_LANDING_SPEED,          VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, ez_landing_speed) },
    { PARAM_NAME_EZ_DISARM_THRESHOLD,       VAR_UINT8  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, 250 }, PG_PID_PROFILE, offsetof(pidProfile_t, ez_landing_disarm_threshold) },

#ifdef USE_WING
    { PARAM_NAME_SPA_ROLL_CENTER,    VAR_UINT16  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_center[FD_ROLL]) },
    { PARAM_NAME_SPA_ROLL_WIDTH,     VAR_UINT16  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_width[FD_ROLL]) },
    { PARAM_NAME_SPA_ROLL_MODE,      VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SPA_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_mode[FD_ROLL]) },
    { PARAM_NAME_SPA_PITCH_CENTER,   VAR_UINT16  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_center[FD_PITCH]) },
    { PARAM_NAME_SPA_PITCH_WIDTH,    VAR_UINT16  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_width[FD_PITCH]) },
    { PARAM_NAME_SPA_PITCH_MODE,     VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SPA_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_mode[FD_PITCH]) },
    { PARAM_NAME_SPA_YAW_CENTER,     VAR_UINT16  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_center[FD_YAW]) },
    { PARAM_NAME_SPA_YAW_WIDTH,      VAR_UINT16  | PROFILE_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_width[FD_YAW]) },
    { PARAM_NAME_SPA_YAW_MODE,       VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SPA_MODE }, PG_PID_PROFILE, offsetof(pidProfile_t, spa_mode[FD_YAW]) },
#endif

// PG_TELEMETRY_CONFIG
#ifdef USE_TELEMETRY
    { "tlm_inverted",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, telemetry_inverted) },
    { "tlm_halfduplex",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, halfDuplex) },
#if defined(USE_TELEMETRY_FRSKY_HUB)
#if defined(USE_GPS)
    { "frsky_default_lat",          VAR_INT16  | MASTER_VALUE, .config.minmax = { -9000, 9000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLatitude) },
    { "frsky_default_long",         VAR_INT16  | MASTER_VALUE, .config.minmax = { -18000, 18000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, gpsNoFixLongitude) },
    { "frsky_gps_format",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, FRSKY_FORMAT_NMEA }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_coordinate_format) },
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_unit) },
#endif
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { FRSKY_VFAS_PRECISION_LOW,  FRSKY_VFAS_PRECISION_HIGH }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, frsky_vfas_precision) },
#endif // USE_TELEMETRY_FRSKY_HUB
    { "hott_alarm_int",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 120 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, hottAlarmSoundInterval) },
    { "pid_in_tlm",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, pidValuesAsTelemetry) },
    { "report_cell_voltage",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, report_cell_voltage) },
#if defined(USE_TELEMETRY_IBUS)
    { "ibus_sensor",                VAR_UINT8  | MASTER_VALUE | MODE_ARRAY, .config.array.length = IBUS_SENSOR_COUNT, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, flysky_sensors)},
#endif
#ifdef USE_TELEMETRY_MAVLINK
    // Support for misusing the heading field in MAVlink to indicate mAh drawn for Connex Prosight OSD
    // Set to 10 to show a tenth of your capacity drawn.
    // Set to $size_of_battery to get a percentage of battery used.
    { "mavlink_mah_as_heading_divisor", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 30000 }, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, mavlink_mah_as_heading_divisor) },
#endif
#ifdef USE_TELEMETRY_SENSORS_DISABLED_DETAILS
    { "telemetry_disabled_voltage",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_VOLTAGE),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_current",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_CURRENT),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_fuel",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_FUEL),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_mode",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_MODE),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_acc_x",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_X),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_acc_y",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_Y),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_acc_z",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ACC_Z),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_pitch",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_PITCH),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_roll",            VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ROLL),            PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_heading",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_HEADING),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_altitude",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_ALTITUDE),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_vario",           VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_VARIO),           PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_lat_long",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_LAT_LONG),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_ground_speed",    VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_GROUND_SPEED),    PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_distance",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_DISTANCE),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_esc_current",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_CURRENT),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_esc_voltage",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_VOLTAGE),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_esc_rpm",         VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_RPM),         PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_esc_temperature", VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(ESC_SENSOR_TEMPERATURE), PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_temperature",     VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_TEMPERATURE),     PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
    { "telemetry_disabled_cap_used",        VAR_UINT32  | MASTER_VALUE | MODE_BITSET, .config.bitpos = LOG2(SENSOR_CAP_USED),        PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
#else
    { "telemetry_disabled_sensors", VAR_UINT32 | MASTER_VALUE, .config.u32Max = SENSOR_ALL, PG_TELEMETRY_CONFIG, offsetof(telemetryConfig_t, disabledSensors)},
#endif
#endif // USE_TELEMETRY

// PG_LED_STRIP_CONFIG
#ifdef USE_LED_STRIP
    { "ledstrip_visual_beeper",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_visual_beeper) },
    { "ledstrip_visual_beeper_color",VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_visual_beeper_color) },
    { "ledstrip_grb_rgb",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RGB_GRB }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_grb_rgb) },
    { "ledstrip_profile",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LED_PROFILE }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_profile) },
    { "ledstrip_race_color",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_race_color) },
    { "ledstrip_beacon_color",      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_LEDSTRIP_COLOR }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_color) },
    { "ledstrip_beacon_period_ms",  VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 50, 10000 }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_period_ms) },
    { "ledstrip_beacon_percent",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_percent) },
    { "ledstrip_beacon_armed_only", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_beacon_armed_only) },
    { "ledstrip_brightness",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 100 }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_brightness) },
    { "ledstrip_rainbow_delta",     VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, HSV_HUE_MAX }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_rainbow_delta) },
    { "ledstrip_rainbow_freq",      VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1, 2000 }, PG_LED_STRIP_CONFIG, offsetof(ledStripConfig_t, ledstrip_rainbow_freq) },
#endif

// PG_SDCARD_CONFIG
#ifdef USE_SDCARD
    { "sdcard_detect_inverted",     VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, cardDetectInverted) },
    { "sdcard_mode",                VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_SDCARD_MODE }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, mode) },
#endif
#ifdef USE_SDCARD_SPI
    { "sdcard_spi_bus",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_SDCARD_CONFIG, offsetof(sdcardConfig_t, device) },
#endif
#ifdef USE_SDCARD_SDIO
    { "sdio_clk_bypass",            VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, clockBypass) },
    { "sdio_use_cache",             VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, useCache) },
    { "sdio_use_4bit_width",        VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, use4BitWidth) },
#ifdef STM32H7
    { "sdio_device",                VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SDIODEV_COUNT }, PG_SDIO_CONFIG, offsetof(sdioConfig_t, device) },
#endif
#endif

// PG_OSD_CONFIG
#ifdef USE_OSD
    { "osd_units",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_UNIT }, PG_OSD_CONFIG, offsetof(osdConfig_t, units) },

    // Enabled OSD warning flags are stored as bitmapped values inside a 32bit parameter
    { "osd_warn_bitmask",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_OSD_CONFIG, offsetof(osdConfig_t, enabledWarnings)},

    { "osd_rssi_alarm",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_OSD_CONFIG, offsetof(osdConfig_t, rssi_alarm) },
#ifdef USE_RX_LINK_QUALITY_INFO
    { "osd_link_quality_alarm",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_OSD_CONFIG, offsetof(osdConfig_t, link_quality_alarm) },
#endif
#ifdef USE_RX_RSSI_DBM
    { "osd_rssi_dbm_alarm",         VAR_INT16   | MASTER_VALUE, .config.minmax = { CRSF_RSSI_MIN, CRSF_RSSI_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, rssi_dbm_alarm) },
#endif
#ifdef USE_RX_RSNR
    { "osd_rsnr_alarm",             VAR_INT16   | MASTER_VALUE, .config.minmax = { CRSF_SNR_MIN, CRSF_SNR_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, rsnr_alarm) },
#endif
    { "osd_cap_alarm",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 20000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, cap_alarm) },
    { "osd_alt_alarm",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, alt_alarm) },
    { "osd_distance_alarm",         VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, distance_alarm) },
    { "osd_esc_temp_alarm",         VAR_UINT8  | MASTER_VALUE, .config.minmax = { 0, UINT8_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_temp_alarm) },
    { "osd_esc_rpm_alarm",          VAR_INT16  | MASTER_VALUE, .config.minmax = { ESC_RPM_ALARM_OFF, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_rpm_alarm) },
    { "osd_esc_current_alarm",      VAR_INT16  | MASTER_VALUE, .config.minmax = { ESC_CURRENT_ALARM_OFF, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, esc_current_alarm) },
#ifdef USE_ADC_INTERNAL
    { "osd_core_temp_alarm",        VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, core_temp_alarm) },
#endif

    { "osd_ah_max_pit",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahMaxPitch) },
    { "osd_ah_max_rol",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 90 }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahMaxRoll) },
    { "osd_ah_invert",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, ahInvert) },
    { "osd_logo_on_arming",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OSD_LOGO_ON_ARMING }, PG_OSD_CONFIG, offsetof(osdConfig_t, logo_on_arming) },
    { "osd_logo_on_arming_duration",VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 5, 50 }, PG_OSD_CONFIG, offsetof(osdConfig_t, logo_on_arming_duration) },
    { "osd_arming_logo",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, DISPLAYPORT_SEVERITY_COUNT - 1 }, PG_OSD_CONFIG, offsetof(osdConfig_t, arming_logo) },
#ifdef USE_OSD_QUICK_MENU
    { "osd_use_quick_menu",   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, osd_use_quick_menu) },
#endif // USE_OSD_QUICK_MENU
#ifdef USE_SPEC_PREARM_SCREEN
    { "osd_show_spec_prearm",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, osd_show_spec_prearm) },
#endif // USE_SPEC_PREARM_SCREEN
    { "osd_tim1",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, timers[OSD_TIMER_1]) },
    { "osd_tim2",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, INT16_MAX }, PG_OSD_CONFIG, offsetof(osdConfig_t, timers[OSD_TIMER_2]) },

    { "osd_vbat_pos",               VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_MAIN_BATT_VOLTAGE]) },
    { "osd_rssi_pos",               VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RSSI_VALUE]) },
#ifdef USE_RX_LINK_QUALITY_INFO
    { "osd_link_quality_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_LINK_QUALITY]) },
#endif
#ifdef USE_RX_LINK_UPLINK_POWER
    { "osd_link_tx_power_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_TX_UPLINK_POWER]) },
#endif
#ifdef USE_RX_RSSI_DBM
    { "osd_rssi_dbm_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RSSI_DBM_VALUE]) },
#endif
#ifdef USE_RX_RSNR
    { "osd_rsnr_pos",               VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RSNR_VALUE]) },
#endif
    { "osd_tim_1_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ITEM_TIMER_1]) },
    { "osd_tim_2_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ITEM_TIMER_2]) },
    { "osd_remaining_time_estimate_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_REMAINING_TIME_ESTIMATE]) },
    { "osd_flymode_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_FLYMODE]) },
    { "osd_anti_gravity_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ANTI_GRAVITY]) },
    { "osd_g_force_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_G_FORCE]) },
    { "osd_throttle_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_THROTTLE_POS]) },
    { "osd_vtx_channel_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_VTX_CHANNEL]) },
    { "osd_crosshairs_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CROSSHAIRS]) },
    { "osd_ah_sbar_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_HORIZON_SIDEBARS]) },
    { "osd_ah_pos",                 VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ARTIFICIAL_HORIZON]) },
    { "osd_current_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CURRENT_DRAW]) },
    { "osd_mah_drawn_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_MAH_DRAWN]) },
    { "osd_wh_drawn_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_WATT_HOURS_DRAWN]) },
    { "osd_motor_diag_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_MOTOR_DIAG]) },
    { "osd_craft_name_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CRAFT_NAME]) },
    { "osd_pilot_name_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PILOT_NAME]) },
    { "osd_gps_speed_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_SPEED]) },
    { "osd_gps_lon_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_LON]) },
    { "osd_gps_lat_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_LAT]) },
#ifdef USE_GPS_LAP_TIMER
    { "osd_gps_lap_curr_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_LAP_TIME_CURRENT]) },
    { "osd_gps_lap_prev_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_LAP_TIME_PREVIOUS]) },
    { "osd_gps_lap_best3_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_LAP_TIME_BEST3]) },
#endif // USE_GPS_LAP_TIMER
    { "osd_gps_sats_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_GPS_SATS]) },
    { "osd_home_dir_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_HOME_DIR]) },
    { "osd_home_dist_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_HOME_DIST]) },
    { "osd_flight_dist_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_FLIGHT_DIST]) },
    { "osd_compass_bar_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_COMPASS_BAR]) },
    { "osd_altitude_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ALTITUDE]) },
    { "osd_pid_roll_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ROLL_PIDS]) },
    { "osd_pid_pitch_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PITCH_PIDS]) },
    { "osd_pid_yaw_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_YAW_PIDS]) },
    { "osd_debug_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_DEBUG]) },
    { "osd_debug2_pos",             VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_DEBUG2]) },
    { "osd_power_pos",              VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_POWER]) },
    { "osd_pidrate_profile_pos",    VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PIDRATE_PROFILE]) },
    { "osd_warnings_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_WARNINGS]) },
    { "osd_avg_cell_voltage_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_AVG_CELL_VOLTAGE]) },
    { "osd_pit_ang_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PITCH_ANGLE]) },
    { "osd_rol_ang_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ROLL_ANGLE]) },
    { "osd_battery_usage_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_MAIN_BATT_USAGE]) },
    { "osd_disarmed_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_DISARMED]) },
    { "osd_nheading_pos",           VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_NUMERICAL_HEADING]) },
    { "osd_up_down_reference_pos",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_UP_DOWN_REFERENCE]) },
    { "osd_ready_mode_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_READY_MODE]) },
#ifdef USE_VARIO
    { "osd_nvario_pos",             VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_NUMERICAL_VARIO]) },
#endif
    { "osd_esc_tmp_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ESC_TMP]) },
    { "osd_esc_rpm_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ESC_RPM]) },
    { "osd_esc_rpm_freq_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ESC_RPM_FREQ]) },
    { "osd_rtc_date_time_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RTC_DATETIME]) },
    { "osd_adjustment_range_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_ADJUSTMENT_RANGE]) },
    { "osd_flip_arrow_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_FLIP_ARROW]) },
#ifdef USE_ADC_INTERNAL
    { "osd_core_temp_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CORE_TEMPERATURE]) },
#endif
#ifdef USE_BLACKBOX
    { "osd_log_status_pos",         VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_LOG_STATUS]) },
#endif

#ifdef USE_OSD_STICK_OVERLAY
    { "osd_stick_overlay_left_pos",    VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_STICK_OVERLAY_LEFT]) },
    { "osd_stick_overlay_right_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_STICK_OVERLAY_RIGHT]) },

    { "osd_stick_overlay_radio_mode",  VAR_UINT8   | MASTER_VALUE, .config.minmaxUnsigned = { 1, 4 }, PG_OSD_CONFIG, offsetof(osdConfig_t, overlay_radio_mode) },
#endif

#ifdef USE_PROFILE_NAMES
    { "osd_rate_profile_name_pos",  VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RATE_PROFILE_NAME]) },
    { "osd_pid_profile_name_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PID_PROFILE_NAME]) },
#endif

#ifdef USE_OSD_PROFILES
    { "osd_profile_name_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_PROFILE_NAME]) },
#endif

    { "osd_rcchannels_pos",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_RC_CHANNELS]) },
    { "osd_camera_frame_pos",   VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_CAMERA_FRAME]) },
    { "osd_efficiency_pos",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_EFFICIENCY]) },
    { "osd_total_flights_pos",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_TOTAL_FLIGHTS]) },
    { "osd_aux_pos",            VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_AUX_VALUE]) },

#ifdef USE_MSP_DISPLAYPORT
    { "osd_sys_goggle_voltage_pos", VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_GOGGLE_VOLTAGE]) },
    { "osd_sys_vtx_voltage_pos",    VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_VTX_VOLTAGE]) },
    { "osd_sys_bitrate_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_BITRATE]) },
    { "osd_sys_delay_pos",          VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_DELAY]) },
    { "osd_sys_distance_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_DISTANCE]) },
    { "osd_sys_lq_pos",             VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_LQ]) },
    { "osd_sys_goggle_dvr_pos",     VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_GOGGLE_DVR]) },
    { "osd_sys_vtx_dvr_pos",        VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_VTX_DVR]) },
    { "osd_sys_warnings_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_WARNINGS]) },
    { "osd_sys_vtx_temp_pos",       VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_VTX_TEMP]) },
    { "osd_sys_fan_speed_pos",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, OSD_POSCFG_MAX }, PG_OSD_ELEMENT_CONFIG, offsetof(osdElementConfig_t, item_pos[OSD_SYS_FAN_SPEED]) },
#endif

    // OSD stats enabled flags are stored as bitmapped values inside a 32bit parameter
    { "osd_stat_bitmask",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_OSD_CONFIG, offsetof(osdConfig_t, enabled_stats)},

#ifdef USE_OSD_PROFILES
    { "osd_profile",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, OSD_PROFILE_COUNT }, PG_OSD_CONFIG, offsetof(osdConfig_t, osdProfileIndex) },
    { "osd_profile_1_name",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[0]) },
    { "osd_profile_2_name",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[1]) },
    { "osd_profile_3_name",         VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, OSD_PROFILE_NAME_LENGTH, STRING_FLAGS_NONE }, PG_OSD_CONFIG, offsetof(osdConfig_t, profile[2]) },
#endif
    { "osd_gps_sats_show_pdop",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, gps_sats_show_pdop) },
    { "osd_displayport_device",     VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OSD_DISPLAYPORT_DEVICE }, PG_OSD_CONFIG, offsetof(osdConfig_t, displayPortDevice) },

    { "osd_rcchannels",             VAR_INT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = OSD_RCCHANNELS_COUNT, PG_OSD_CONFIG, offsetof(osdConfig_t, rcChannels) },
    { "osd_camera_frame_width",     VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { OSD_CAMERA_FRAME_MIN_WIDTH, OSD_CAMERA_FRAME_MAX_WIDTH }, PG_OSD_CONFIG, offsetof(osdConfig_t, camera_frame_width) },
    { "osd_camera_frame_height",    VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { OSD_CAMERA_FRAME_MIN_HEIGHT, OSD_CAMERA_FRAME_MAX_HEIGHT }, PG_OSD_CONFIG, offsetof(osdConfig_t, camera_frame_height) },
    { "osd_stat_avg_cell_value",    VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, stat_show_cell_value) },
    { "osd_framerate_hz",           VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { OSD_FRAMERATE_MIN_HZ, OSD_FRAMERATE_MAX_HZ }, PG_OSD_CONFIG, offsetof(osdConfig_t, framerate_hz) },
    { "osd_menu_background",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CMS_BACKGROUND }, PG_OSD_CONFIG, offsetof(osdConfig_t, cms_background_type) },
    { "osd_aux_channel",            VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, MAX_SUPPORTED_RC_CHANNEL_COUNT }, PG_OSD_CONFIG, offsetof(osdConfig_t, aux_channel) },
    { "osd_aux_scale",              VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 1, 1000 }, PG_OSD_CONFIG, offsetof(osdConfig_t, aux_scale) },
    { "osd_aux_symbol",             VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 255 },  PG_OSD_CONFIG, offsetof(osdConfig_t, aux_symbol) },
#ifdef OSD_CANVAS_SIZE_DEBUG
    { "osd_canvas_width",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 63 }, PG_OSD_CONFIG, offsetof(osdConfig_t, canvas_cols) },
    { "osd_canvas_height",          VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 31 }, PG_OSD_CONFIG, offsetof(osdConfig_t, canvas_rows) },
#endif
#ifdef USE_CRAFTNAME_MSGS
    { "osd_craftname_msgs",   VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_OSD_CONFIG, offsetof(osdConfig_t, osd_craftname_msgs) },
#endif //USE_CRAFTNAME_MSGS
#endif // end of #ifdef USE_OSD

// PG_SYSTEM_CONFIG
#if defined(STM32F4) || defined(STM32G4)
    { "system_hse_mhz",             VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 30 }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, hseMhz) },
#endif
    { "task_statistics",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, task_statistics) },
    { PARAM_NAME_DEBUG_MODE,        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_DEBUG }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, debug_mode) },
    { "rate_6pos_switch",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, rateProfile6PosSwitch) },
#ifdef USE_OVERCLOCK
    { "cpu_overclock",              VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OVERCLOCK }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, cpu_overclock) },
#endif
    { "pwr_on_arm_grace",           VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 30 }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, powerOnArmingGraceTime) },
    { "enable_stick_arming",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_SYSTEM_CONFIG, offsetof(systemConfig_t, enableStickArming) },

// PG_VTX_CONFIG
#ifdef USE_VTX_COMMON
    { "vtx_band",                   VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_BANDS }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, band) },
    { "vtx_channel",                VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_CHANNELS }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, channel) },
    { "vtx_power",                  VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_TABLE_MAX_POWER_LEVELS - 1 }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, power) },
    { "vtx_low_power_disarm",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VTX_LOW_POWER_DISARM }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, lowPowerDisarm) },
    { "vtx_softserial_alt",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, softserialAlt) },
#ifdef VTX_SETTINGS_FREQCMD
    { "vtx_freq",                   VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_SETTINGS_MAX_FREQUENCY_MHZ }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, freq) },
    { "vtx_pit_mode_freq",          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, VTX_SETTINGS_MAX_FREQUENCY_MHZ }, PG_VTX_SETTINGS_CONFIG, offsetof(vtxSettingsConfig_t, pitModeFreq) },
#endif
#endif

// PG_VTX_CONFIG
#if defined(USE_VTX_CONTROL) && defined(USE_VTX_COMMON)
    { "vtx_halfduplex",             VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_VTX_CONFIG, offsetof(vtxConfig_t, halfDuplex) },
#endif

// PG_VTX_IO
#ifdef USE_VTX_RTC6705
    { "vtx_spi_bus",                VAR_UINT8  | HARDWARE_VALUE | MASTER_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_VTX_IO_CONFIG, offsetof(vtxIOConfig_t, spiDevice) },
#endif

// PG_VCD_CONFIG
#if defined(USE_VIDEO_SYSTEM)
    { "vcd_video_system",           VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_VIDEO_SYSTEM }, PG_VCD_CONFIG, offsetof(vcdProfile_t, video_system) },
#endif
#if defined(USE_MAX7456)
    { "vcd_h_offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -32, 31 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, h_offset) },
    { "vcd_v_offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -15, 16 }, PG_VCD_CONFIG, offsetof(vcdProfile_t, v_offset) },
#endif

// PG_MAX7456_CONFIG
#ifdef USE_MAX7456
    { "max7456_clock",              VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_MAX7456_CLOCK }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, clockConfig) },
    { "max7456_spi_bus",            VAR_UINT8   | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, spiDevice) },
    { "max7456_preinit_opu",        VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MAX7456_CONFIG, offsetof(max7456Config_t, preInitOPU) },
#endif

// PG_DISPLAY_PORT_MSP_CONFIG
#ifdef USE_MSP_DISPLAYPORT
    { "displayport_msp_col_adjust", VAR_INT8    | MASTER_VALUE, .config.minmax = { -6, 0 }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, colAdjust) },
    { "displayport_msp_row_adjust", VAR_INT8    | MASTER_VALUE, .config.minmax = { -3, 0 }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, rowAdjust) },
    { "displayport_msp_fonts",      VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 4, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, fontSelection) },
    { "displayport_msp_use_device_blink",   VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_DISPLAY_PORT_MSP_CONFIG, offsetof(displayPortProfile_t, useDeviceBlink) },
#endif

// PG_DISPLAY_PORT_MSP_CONFIG
#ifdef USE_MAX7456
    { "displayport_max7456_col_adjust", VAR_INT8| MASTER_VALUE, .config.minmax = { -6, 0 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, colAdjust) },
    { "displayport_max7456_row_adjust", VAR_INT8| MASTER_VALUE, .config.minmax = { -3, 0 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, rowAdjust) },
    { "displayport_max7456_inv",        VAR_UINT8| MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, invert) },
    { "displayport_max7456_blk",        VAR_UINT8| MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, blackBrightness) },
    { "displayport_max7456_wht",        VAR_UINT8| MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_DISPLAY_PORT_MAX7456_CONFIG, offsetof(displayPortProfile_t, whiteBrightness) },
#endif

#ifdef USE_ESC_SENSOR
    { "esc_sensor_halfduplex",          VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_ESC_SENSOR_CONFIG, offsetof(escSensorConfig_t, halfDuplex) },
    { "esc_sensor_current_offset",      VAR_UINT16  | MASTER_VALUE, .config.minmaxUnsigned = { 0, 16000 }, PG_ESC_SENSOR_CONFIG, offsetof(escSensorConfig_t, offset) },
#endif

#ifdef USE_RX_FRSKY_SPI
    { "frsky_spi_autobind",             VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, autoBind) },
    { "frsky_spi_tx_id",                VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 3, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindTxId) },
    { "frsky_spi_offset",               VAR_INT8    | MASTER_VALUE, .config.minmax = { -127, 127 }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindOffset) },
    { "frsky_spi_bind_hop_data",        VAR_UINT8   | MASTER_VALUE | MODE_ARRAY, .config.array.length = 50, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, bindHopData) },
    { "frsky_x_rx_num",                 VAR_UINT8   | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, rxNum) },
    { "frsky_spi_a1_source",            VAR_UINT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RX_FRSKY_SPI_A1_SOURCE }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, a1Source) },
    { "cc2500_spi_chip_detect",         VAR_UINT8   | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_RX_CC2500_SPI_CONFIG, offsetof(rxCc2500SpiConfig_t, chipDetectEnabled) },
#endif
    { "led_inversion",                  VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, ((1 << STATUS_LED_NUMBER) - 1) }, PG_STATUS_LED_CONFIG, offsetof(statusLedConfig_t, inversion) },
#ifdef USE_DASHBOARD
    { "dashboard_i2c_bus",           VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_DASHBOARD_CONFIG, offsetof(dashboardConfig_t, device) },
    { "dashboard_i2c_addr",          VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { I2C_ADDR7_MIN, I2C_ADDR7_MAX }, PG_DASHBOARD_CONFIG, offsetof(dashboardConfig_t, address) },
#endif

// PG_CAMERA_CONTROL_CONFIG
#ifdef USE_CAMERA_CONTROL
    { "camera_control_mode", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_CAMERA_CONTROL_MODE }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, mode) },
    { "camera_control_ref_voltage", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 200, 400 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, refVoltage) },
    { "camera_control_key_delay", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 500 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, keyDelayMs) },
    { "camera_control_internal_resistance", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 10, 1000 }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, internalResistance) },
    { "camera_control_button_resistance",   VAR_UINT16 | MASTER_VALUE | MODE_ARRAY, .config.array.length = CAMERA_CONTROL_KEYS_COUNT, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, buttonResistanceValues) },
    { "camera_control_inverted", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_CAMERA_CONTROL_CONFIG, offsetof(cameraControlConfig_t, inverted) },
#endif

// PG_RANGEFINDER_CONFIG
#ifdef USE_RANGEFINDER
    { "rangefinder_hardware", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_RANGEFINDER_HARDWARE }, PG_RANGEFINDER_CONFIG, offsetof(rangefinderConfig_t, rangefinder_hardware) },
#endif

// PG_PINIO_CONFIG
#ifdef USE_PINIO
    { "pinio_config", VAR_UINT8 | HARDWARE_VALUE | MODE_ARRAY, .config.array.length = PINIO_COUNT, PG_PINIO_CONFIG, offsetof(pinioConfig_t, config) },
#ifdef USE_PINIOBOX
    { "pinio_box", VAR_UINT8 | HARDWARE_VALUE | MODE_ARRAY, .config.array.length = PINIO_COUNT, PG_PINIOBOX_CONFIG, offsetof(pinioBoxConfig_t, permanentId) },
#endif
#endif

//PG USB
#ifdef USE_USB_CDC_HID
    { "usb_hid_cdc", VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_USB_CONFIG, offsetof(usbDev_t, type) },
#endif
#ifdef USE_USB_MSC
    { "usb_msc_pin_pullup", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_USB_CONFIG, offsetof(usbDev_t, mscButtonUsePullup) },
#endif
// PG_FLASH_CONFIG
#ifdef USE_FLASH_SPI
    { "flash_spi_bus", VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_FLASH_CONFIG, offsetof(flashConfig_t, spiDevice) },
#endif
// RCDEVICE
#ifdef USE_RCDEVICE
    { "rcdevice_init_dev_attempts", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 10 }, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, initDeviceAttempts) },
    { "rcdevice_init_dev_attempt_interval", VAR_UINT32 | MASTER_VALUE, .config.u32Max = 5000, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, initDeviceAttemptInterval) },
    { "rcdevice_protocol_version", VAR_UINT8 | MASTER_VALUE, .config.minmax = { 0, 1 }, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, protocolVersion) },
    { "rcdevice_feature", VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = {0, 65535}, PG_RCDEVICE_CONFIG, offsetof(rcdeviceConfig_t, feature) },
#endif

// PG_GYRO_DEVICE_CONFIG
    { "gyro_1_bustype", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, busType) },
    { "gyro_1_spibus",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, spiBus) },
    { "gyro_1_i2cBus",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, i2cBus) },
    { "gyro_1_i2c_address", VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, i2cAddress) },
    { "gyro_1_sensor_align", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, alignment) },
    { "gyro_1_align_roll", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, customAlignment.roll) },
    { "gyro_1_align_pitch", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, customAlignment.pitch) },
    { "gyro_1_align_yaw", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 0, customAlignment.yaw) },
#ifdef USE_MULTI_GYRO
    { "gyro_2_bustype", VAR_UINT8 | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_BUS_TYPE }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, busType) },
    { "gyro_2_spibus",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, SPIDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, spiBus) },
    { "gyro_2_i2cBus",  VAR_UINT8 | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2CDEV_COUNT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, i2cBus) },
    { "gyro_2_i2c_address", VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, I2C_ADDR7_MAX }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, i2cAddress) },
    { "gyro_2_sensor_align", VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_ALIGNMENT }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, alignment) },
    { "gyro_2_align_roll", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, customAlignment.roll) },
    { "gyro_2_align_pitch", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, customAlignment.pitch) },
    { "gyro_2_align_yaw", VAR_INT16  | HARDWARE_VALUE, .config.minmax = { -3600, 3600 }, PG_GYRO_DEVICE_CONFIG, PG_ARRAY_ELEMENT_OFFSET(gyroDeviceConfig_t, 1, customAlignment.yaw) },
#endif
#ifdef I2C_FULL_RECONFIGURABILITY
#ifdef USE_I2C_DEVICE_1
    { "i2c1_pullup",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 0, pullUp) },
    { "i2c1_clockspeed_khz", VAR_UINT16 | HARDWARE_VALUE, .config.minmax = { I2C_CLOCKSPEED_MIN_KHZ, I2C_CLOCKSPEED_MAX_KHZ }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 0, clockSpeed) },
#endif
#ifdef USE_I2C_DEVICE_2
    { "i2c2_pullup",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 1, pullUp) },
    { "i2c2_clockspeed_khz", VAR_UINT16 | HARDWARE_VALUE, .config.minmax = { I2C_CLOCKSPEED_MIN_KHZ, I2C_CLOCKSPEED_MAX_KHZ }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 1, clockSpeed) },
#endif
#ifdef USE_I2C_DEVICE_3
    { "i2c3_pullup",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 2, pullUp) },
    { "i2c3_clockspeed_khz", VAR_UINT16 | HARDWARE_VALUE, .config.minmax = { I2C_CLOCKSPEED_MIN_KHZ, I2C_CLOCKSPEED_MAX_KHZ }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 2, clockSpeed) },
#endif
#ifdef USE_I2C_DEVICE_4
    { "i2c4_pullup",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 3, pullUp) },
    { "i2c4_clockspeed_khz", VAR_UINT16 | HARDWARE_VALUE, .config.minmax = { I2C_CLOCKSPEED_MIN_KHZ, I2C_CLOCKSPEED_MAX_KHZ }, PG_I2C_CONFIG, PG_ARRAY_ELEMENT_OFFSET(i2cConfig_t, 3, clockSpeed) },
#endif
#endif
#ifdef USE_MCO
#ifdef STM32G4
    { "mco_on_pa8",     VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MCO_CONFIG, PG_ARRAY_ELEMENT_OFFSET(mcoConfig_t, 0, enabled) },
    { "mco_source",     VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, MCO_SOURCE_COUNT - 1 }, PG_MCO_CONFIG, PG_ARRAY_ELEMENT_OFFSET(mcoConfig_t, 0, source) },
    { "mco_divider",    VAR_UINT8  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, MCO_DIVIDER_COUNT - 1 }, PG_MCO_CONFIG, PG_ARRAY_ELEMENT_OFFSET(mcoConfig_t, 0, divider) },
#else
    { "mco2_on_pc9",    VAR_UINT8  | HARDWARE_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MCO_CONFIG, PG_ARRAY_ELEMENT_OFFSET(mcoConfig_t, 1, enabled) },
#endif
#endif
#ifdef USE_RX_SPEKTRUM
    { "spektrum_spi_protocol",     VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, protocol) },
    { "spektrum_spi_mfg_id",       VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = 4, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, mfgId) },
    { "spektrum_spi_num_channels", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, DSM_MAX_CHANNEL_COUNT }, PG_RX_SPEKTRUM_SPI_CONFIG, offsetof(spektrumConfig_t, numChannels) },
#endif
#ifdef USE_RX_EXPRESSLRS
    { "expresslrs_uid",         VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = 6, PG_RX_EXPRESSLRS_SPI_CONFIG, offsetof(rxExpressLrsSpiConfig_t, UID) },
    { "expresslrs_domain",      VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_FREQ_DOMAIN }, PG_RX_EXPRESSLRS_SPI_CONFIG, offsetof(rxExpressLrsSpiConfig_t, domain) },
    { "expresslrs_model_id",    VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, UINT8_MAX }, PG_RX_EXPRESSLRS_SPI_CONFIG, offsetof(rxExpressLrsSpiConfig_t, modelId) },
#endif

    { "scheduler_relax_rx",  VAR_UINT16  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_SCHEDULER_CONFIG, PG_ARRAY_ELEMENT_OFFSET(schedulerConfig_t, 0, rxRelaxDeterminism) },
    { "scheduler_relax_osd", VAR_UINT16  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, 500 }, PG_SCHEDULER_CONFIG, PG_ARRAY_ELEMENT_OFFSET(schedulerConfig_t, 0, osdRelaxDeterminism) },

    { "scheduler_debug_task", VAR_UINT16  | HARDWARE_VALUE, .config.minmaxUnsigned = { 0, TASK_COUNT }, PG_SCHEDULER_CONFIG, PG_ARRAY_ELEMENT_OFFSET(schedulerConfig_t, 0, debugTask) },

#ifdef USE_LATE_TASK_STATISTICS
    { "cpu_late_limit_permille", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_SCHEDULER_CONFIG, offsetof(schedulerConfig_t, cpuLatePercentageLimit) },
#endif

    { "serialmsp_halfduplex", VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_MSP_CONFIG, offsetof(mspConfig_t, halfDuplex) },

// PG_TIMECONFIG
#ifdef USE_RTC_TIME
    { "timezone_offset_minutes",  VAR_INT16 | MASTER_VALUE, .config.minmax = { TIMEZONE_OFFSET_MINUTES_MIN, TIMEZONE_OFFSET_MINUTES_MAX }, PG_TIME_CONFIG, offsetof(timeConfig_t, tz_offsetMinutes) },
#endif

#ifdef USE_RPM_FILTER
    { PARAM_NAME_RPM_FILTER_HARMONICS,     VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 3 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, rpm_filter_harmonics) },
    { PARAM_NAME_RPM_FILTER_WEIGHTS,       VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = RPM_FILTER_HARMONICS_MAX, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, rpm_filter_weights) },
    { PARAM_NAME_RPM_FILTER_Q,             VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 250, 3000 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, rpm_filter_q) },
    { PARAM_NAME_RPM_FILTER_MIN_HZ,        VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 30, 200 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, rpm_filter_min_hz) },
    { PARAM_NAME_RPM_FILTER_FADE_RANGE_HZ, VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 1000 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, rpm_filter_fade_range_hz) },
    { PARAM_NAME_RPM_FILTER_LPF_HZ,        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 100, 500 }, PG_RPM_FILTER_CONFIG, offsetof(rpmFilterConfig_t, rpm_filter_lpf_hz) },
#endif

#ifdef USE_RX_FLYSKY
    { "flysky_spi_tx_id",       VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_FLYSKY_CONFIG, offsetof(flySkyConfig_t, txId) },
    { "flysky_spi_rf_channels", VAR_UINT8 | MASTER_VALUE | MODE_ARRAY, .config.array.length = 16, PG_FLYSKY_CONFIG, offsetof(flySkyConfig_t, rfChannelMap) },
#endif

#ifdef USE_PERSISTENT_STATS
    { "stats_min_armed_time_s",   VAR_INT8   | MASTER_VALUE, .config.minmax = { STATS_OFF, INT8_MAX }, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_min_armed_time_s) },
    { "stats_total_flights",    VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_flights) },

    { "stats_total_time_s",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_time_s) },
    { "stats_total_dist_m",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_total_dist_m) },

#ifdef USE_BATTERY_CONTINUE
    { "stats_mah_used",     VAR_UINT32 | MASTER_VALUE, .config.u32Max = UINT32_MAX, PG_STATS_CONFIG, offsetof(statsConfig_t, stats_mah_used) },
#endif

#endif // USE_PERSISTENT_STATS

    { "craft_name",       VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, MAX_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PILOT_CONFIG, offsetof(pilotConfig_t, craftName) },
#ifdef USE_OSD
    { "pilot_name",     VAR_UINT8  | MASTER_VALUE | MODE_STRING, .config.string = { 1, MAX_NAME_LENGTH, STRING_FLAGS_NONE }, PG_PILOT_CONFIG, offsetof(pilotConfig_t, pilotName) },
#endif

// PG_POSITION
    { "altitude_source",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_POSITION_ALT_SOURCE }, PG_POSITION, offsetof(positionConfig_t, altitude_source) },
    { "altitude_prefer_baro",  VAR_INT8   | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_POSITION, offsetof(positionConfig_t, altitude_prefer_baro) },
    { "altitude_lpf",          VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 10, 1000 }, PG_POSITION, offsetof(positionConfig_t, altitude_lpf) },
    { "altitude_d_lpf",        VAR_UINT16 | MASTER_VALUE, .config.minmaxUnsigned = { 10, 1000 }, PG_POSITION, offsetof(positionConfig_t, altitude_d_lpf) },

// PG_MODE_ACTIVATION_CONFIG
#if defined(USE_CUSTOM_BOX_NAMES)
    { "box_user_1_name", VAR_UINT8 | HARDWARE_VALUE | MODE_STRING, .config.string = { 1, MAX_BOX_USER_NAME_LENGTH, STRING_FLAGS_NONE }, PG_MODE_ACTIVATION_CONFIG, offsetof(modeActivationConfig_t, box_user_1_name) },
    { "box_user_2_name", VAR_UINT8 | HARDWARE_VALUE | MODE_STRING, .config.string = { 1, MAX_BOX_USER_NAME_LENGTH, STRING_FLAGS_NONE }, PG_MODE_ACTIVATION_CONFIG, offsetof(modeActivationConfig_t, box_user_2_name) },
    { "box_user_3_name", VAR_UINT8 | HARDWARE_VALUE | MODE_STRING, .config.string = { 1, MAX_BOX_USER_NAME_LENGTH, STRING_FLAGS_NONE }, PG_MODE_ACTIVATION_CONFIG, offsetof(modeActivationConfig_t, box_user_3_name) },
    { "box_user_4_name", VAR_UINT8 | HARDWARE_VALUE | MODE_STRING, .config.string = { 1, MAX_BOX_USER_NAME_LENGTH, STRING_FLAGS_NONE }, PG_MODE_ACTIVATION_CONFIG, offsetof(modeActivationConfig_t, box_user_4_name) },
#endif
};

const uint16_t valueTableEntryCount = ARRAYLEN(valueTable);

STATIC_ASSERT(LOOKUP_TABLE_COUNT == ARRAYLEN(lookupTables), LOOKUP_TABLE_COUNT_incorrect);
