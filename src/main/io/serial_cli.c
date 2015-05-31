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
#include "version.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/color.h"
#include "common/typeconversion.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"


#include "io/escservo.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/rc_controls.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/beeper.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "common/printf.h"

#include "serial_cli.h"

// FIXME remove this for targets that don't need a CLI.  Perhaps use a no-op macro when USE_CLI is not enabled
// signal that we're in cli mode
uint8_t cliMode = 0;

#ifdef USE_CLI

extern uint16_t cycleTime; // FIXME dependency on mw.c

void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort);

static serialPort_t *cliPort;

static void cliAux(char *cmdline);
static void cliAdjustmentRange(char *cmdline);
static void cliCMix(char *cmdline);
static void cliDefaults(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliMotor(char *cmdline);
static void cliPlaySound(char *cmdline);
static void cliProfile(char *cmdline);
static void cliRateProfile(char *cmdline);
static void cliReboot(void);
static void cliSave(char *cmdline);
static void cliSerial(char *cmdline);
static void cliServo(char *cmdline);
static void cliSet(char *cmdline);
static void cliGet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);

#ifdef GPS
static void cliGpsPassthrough(char *cmdline);
#endif

static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);

#ifdef LED_STRIP
static void cliLed(char *cmdline);
static void cliColor(char *cmdline);
#endif

#ifndef USE_QUAD_MIXER_ONLY
static void cliMixer(char *cmdline);
#endif

#ifdef USE_FLASHFS
static void cliFlashInfo(char *cmdline);
static void cliFlashErase(char *cmdline);
static void cliFlashWrite(char *cmdline);
static void cliFlashRead(char *cmdline);
#endif

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

#ifndef USE_QUAD_MIXER_ONLY
// sync this with mixerMode_e
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", NULL
};
#endif

// sync this with features_e
static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "FAILSAFE",
    "SONAR", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DISPLAY", "ONESHOT125",
    "BLACKBOX", NULL
};

#ifndef CJMCU
// sync this with sensors_e
static const char * const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "SONAR", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)

static const char * const sensorHardwareNames[4][11] = {
    { "", "None", "MPU6050", "L3G4200D", "MPU3050", "L3GD20", "MPU6000", "MPU6500", "FAKE", NULL },
    { "", "None", "ADXL345", "MPU6050", "MMA845x", "BMA280", "LSM303DLHC", "MPU6000", "MPU6500", "FAKE", NULL },
    { "", "None", "BMP085", "MS5611", NULL },
    { "", "None", "HMC5883", "AK8975", NULL }
};
#endif

typedef struct {
    const char *name;
    const char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    { "adjrange", "show/set adjustment ranges settings", cliAdjustmentRange },
    { "aux", "show/set aux settings", cliAux },
    { "cmix", "design custom mixer", cliCMix },
#ifdef LED_STRIP
    { "color", "configure colors", cliColor },
#endif
    { "defaults", "reset to defaults and reboot", cliDefaults },
    { "dump", "dump configuration", cliDump },
    { "exit", "", cliExit },
    { "feature", "list or -val or val", cliFeature },
#ifdef USE_FLASHFS
    { "flash_erase", "erase flash chip", cliFlashErase },
    { "flash_info", "get flash chip details", cliFlashInfo },
    { "flash_read", "read text from the given address", cliFlashRead },
    { "flash_write", "write text to the given address", cliFlashWrite },
#endif
    { "get", "get variable value", cliGet },
#ifdef GPS
    { "gpspassthrough", "passthrough gps to serial", cliGpsPassthrough },
#endif
    { "help", "", cliHelp },
#ifdef LED_STRIP
    { "led", "configure leds", cliLed },
#endif
    { "map", "mapping of rc channel order", cliMap },
#ifndef USE_QUAD_MIXER_ONLY
    { "mixer", "mixer name or list", cliMixer },
#endif
    { "motor", "get/set motor output value", cliMotor },
    { "play_sound", "index, or none for next", cliPlaySound },
    { "profile", "index (0 to 2)", cliProfile },
    { "rateprofile", "index (0 to 2)", cliRateProfile },
    { "save", "save and reboot", cliSave },
    { "serial", "show/set serial settings", cliSerial },
#ifdef USE_SERVOS
    { "servo", "servo config", cliServo },
#endif
    { "set", "name=value or blank or * for list", cliSet },
    { "status", "show system status", cliStatus },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(clicmd_t))

typedef enum {
    VAR_UINT8 = (1 << 0),
    VAR_INT8 = (1 << 1),
    VAR_UINT16 = (1 << 2),
    VAR_INT16 = (1 << 3),
    VAR_UINT32 = (1 << 4),
    VAR_FLOAT = (1 << 5),

    MASTER_VALUE = (1 << 6),
    PROFILE_VALUE = (1 << 7),
    CONTROL_RATE_VALUE = (1 << 8)
} cliValueFlag_e;

#define VALUE_TYPE_MASK (VAR_UINT8 | VAR_INT8 | VAR_UINT16 | VAR_INT16 | VAR_UINT32 | VAR_FLOAT)
#define SECTION_MASK (MASTER_VALUE | PROFILE_VALUE | CONTROL_RATE_VALUE)

typedef struct {
    const char *name;
    const uint16_t type; // cliValueFlag_e - specify one of each from VALUE_TYPE_MASK and SECTION_MASK
    void *ptr;
    const int32_t min;
    const int32_t max;
} clivalue_t;

const clivalue_t valueTable[] = {
    { "looptime",                   VAR_UINT16 | MASTER_VALUE,  &masterConfig.looptime, 0, 9000 },
    { "emf_avoidance",              VAR_UINT8  | MASTER_VALUE,  &masterConfig.emf_avoidance, 0, 1 },

    { "mid_rc",                     VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.midrc, 1200, 1700 },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.mincheck, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.maxcheck, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE,  &masterConfig.rxConfig.rssi_channel, 0, MAX_SUPPORTED_RC_CHANNEL_COUNT },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.rssi_scale, RSSI_SCALE_MIN, RSSI_SCALE_MAX },
    { "rssi_ppm_invert",            VAR_INT8   | MASTER_VALUE,  &masterConfig.rxConfig.rssi_ppm_invert, 0, 1 },
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE,  &masterConfig.inputFilteringMode, 0, 1 },

    { "min_throttle",               VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.minthrottle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.maxthrottle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "min_command",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.mincommand, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "servo_center_pulse",         VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.servoCenterPulse, PWM_RANGE_ZERO, PWM_RANGE_MAX },

    { "3d_deadband_low",            VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_low, PWM_RANGE_ZERO, PWM_RANGE_MAX }, // FIXME upper limit should match code in the mixer, 1500 currently
    { "3d_deadband_high",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_high, PWM_RANGE_ZERO, PWM_RANGE_MAX }, // FIXME lower limit should match code in the mixer, 1500 currently,
    { "3d_neutral",                 VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.neutral3d, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "3d_deadband_throttle",       VAR_UINT16 | MASTER_VALUE,  &masterConfig.flight3DConfig.deadband3d_throttle, PWM_RANGE_ZERO, PWM_RANGE_MAX },

    { "motor_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &masterConfig.motor_pwm_rate, 50, 32000 },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &masterConfig.servo_pwm_rate, 50, 498 },

    { "retarded_arm",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.retarded_arm, 0, 1 },
    { "disarm_kill_switch",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.disarm_kill_switch, 0, 1 },
    { "auto_disarm_delay",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.auto_disarm_delay, 0, 60 },
    { "small_angle",                VAR_UINT8  | MASTER_VALUE,  &masterConfig.small_angle, 0, 180 },

    { "flaps_speed",                VAR_UINT8  | MASTER_VALUE,  &masterConfig.airplaneConfig.flaps_speed, 0, 100 },

    { "fixedwing_althold_dir",      VAR_INT8   | MASTER_VALUE,  &masterConfig.airplaneConfig.fixedwing_althold_dir, -1, 1 },

    { "reboot_character",           VAR_UINT8  | MASTER_VALUE,  &masterConfig.serialConfig.reboot_character, 48, 126 },

#ifdef GPS
    { "gps_provider",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.gpsConfig.provider, 0, GPS_PROVIDER_MAX },
    { "gps_sbas_mode",              VAR_UINT8  | MASTER_VALUE,  &masterConfig.gpsConfig.sbasMode, 0, SBAS_MODE_MAX },
    { "gps_auto_config",            VAR_UINT8  | MASTER_VALUE,  &masterConfig.gpsConfig.autoConfig, GPS_AUTOCONFIG_OFF, GPS_AUTOCONFIG_ON },
    { "gps_auto_baud",              VAR_UINT8  | MASTER_VALUE,  &masterConfig.gpsConfig.autoBaud, GPS_AUTOBAUD_OFF, GPS_AUTOBAUD_ON },

    { "gps_pos_p",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDPOS], 0, 200 },
    { "gps_pos_i",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDPOS], 0, 200 },
    { "gps_pos_d",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDPOS], 0, 200 },
    { "gps_posr_p",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDPOSR], 0, 200 },
    { "gps_posr_i",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDPOSR], 0, 200 },
    { "gps_posr_d",                 VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDPOSR], 0, 200 },
    { "gps_nav_p",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDNAVR], 0, 200 },
    { "gps_nav_i",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDNAVR], 0, 200 },
    { "gps_nav_d",                  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDNAVR], 0, 200 },
    { "gps_wp_radius",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.gps_wp_radius, 0, 2000 },
    { "nav_controls_heading",       VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_controls_heading, 0, 1 },
    { "nav_speed_min",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_speed_min, 10, 2000 },
    { "nav_speed_max",              VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_speed_max, 10, 2000 },
    { "nav_slew_rate",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].gpsProfile.nav_slew_rate, 0, 100 },
#endif

    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.serialrx_provider, 0, SERIALRX_PROVIDER_MAX },
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.spektrum_sat_bind, SPEKTRUM_SAT_BIND_DISABLED, SPEKTRUM_SAT_BIND_MAX},

    { "telemetry_switch",           VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.telemetry_switch, 0, 1 },
    { "telemetry_inversion",        VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.telemetry_inversion, 0, 1 },
    { "frsky_default_lattitude",    VAR_FLOAT  | MASTER_VALUE,  &masterConfig.telemetryConfig.gpsNoFixLatitude, -90.0, 90.0 },
    { "frsky_default_longitude",    VAR_FLOAT  | MASTER_VALUE,  &masterConfig.telemetryConfig.gpsNoFixLongitude, -180.0, 180.0 },
    { "frsky_coordinates_format",   VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.frsky_coordinate_format, 0, FRSKY_FORMAT_NMEA },
    { "frsky_unit",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.frsky_unit, 0, FRSKY_UNIT_IMPERIALS },
    { "frsky_vfas_precision",       VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.frsky_vfas_precision, FRSKY_VFAS_PRECISION_LOW, FRSKY_VFAS_PRECISION_HIGH },
    { "hott_alarm_sound_interval",  VAR_UINT8  | MASTER_VALUE,  &masterConfig.telemetryConfig.hottAlarmSoundInterval, 0, 120 },

    { "battery_capacity",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.batteryConfig.batteryCapacity, 0, 20000 },
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatscale, VBAT_SCALE_MIN, VBAT_SCALE_MAX },
    { "vbat_max_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatmaxcellvoltage, 10, 50 },
    { "vbat_min_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatmincellvoltage, 10, 50 },
    { "vbat_warning_cell_voltage",  VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatwarningcellvoltage, 10, 50 },
    { "current_meter_scale",        VAR_INT16  | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterScale, -10000, 10000 },
    { "current_meter_offset",       VAR_UINT16 | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterOffset, 0, 3300 },
    { "multiwii_current_meter_output", VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.multiwiiCurrentMeterOutput, 0, 1 },
    { "current_meter_type",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterType, 0, CURRENT_SENSOR_MAX },

    { "align_gyro",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.sensorAlignmentConfig.gyro_align, 0, 8 },
    { "align_acc",                  VAR_UINT8  | MASTER_VALUE,  &masterConfig.sensorAlignmentConfig.acc_align, 0, 8 },
    { "align_mag",                  VAR_UINT8  | MASTER_VALUE,  &masterConfig.sensorAlignmentConfig.mag_align, 0, 8 },

    { "align_board_roll",           VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.rollDegrees, -180, 360 },
    { "align_board_pitch",          VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.pitchDegrees, -180, 360 },
    { "align_board_yaw",            VAR_INT16  | MASTER_VALUE,  &masterConfig.boardAlignment.yawDegrees, -180, 360 },

    { "max_angle_inclination",      VAR_UINT16 | MASTER_VALUE,  &masterConfig.max_angle_inclination, 100, 900 },

    { "gyro_lpf",                   VAR_UINT16 | MASTER_VALUE,  &masterConfig.gyro_lpf, 0, 256 },
    { "moron_threshold",            VAR_UINT8  | MASTER_VALUE,  &masterConfig.gyroConfig.gyroMovementCalibrationThreshold, 0, 128 },
    { "gyro_cmpf_factor",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.gyro_cmpf_factor, 100, 1000 },
    { "gyro_cmpfm_factor",          VAR_UINT16 | MASTER_VALUE,  &masterConfig.gyro_cmpfm_factor, 100, 1000 },

    { "alt_hold_deadband",          VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.alt_hold_deadband, 1, 250 },
    { "alt_hold_fast_change",       VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.alt_hold_fast_change, 0, 1 },
    { "deadband",                   VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.deadband, 0, 32 },
    { "yaw_deadband",               VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.yaw_deadband, 0, 100 },

    { "throttle_correction_value",  VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].throttle_correction_value, 0, 150 },
    { "throttle_correction_angle",  VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].throttle_correction_angle, 1, 900 },

    { "yaw_control_direction",      VAR_INT8   | MASTER_VALUE,  &masterConfig.yaw_control_direction, -1, 1 },

    { "pid_at_min_throttle",        VAR_UINT8  | MASTER_VALUE, &masterConfig.mixerConfig.pid_at_min_throttle, 0, 1 },
    { "yaw_direction",              VAR_INT8   | MASTER_VALUE, &masterConfig.mixerConfig.yaw_direction, -1, 1 },
    { "yaw_jump_prevention_limit",  VAR_UINT16 | MASTER_VALUE, &masterConfig.mixerConfig.yaw_jump_prevention_limit, YAW_JUMP_PREVENTION_LIMIT_LOW, YAW_JUMP_PREVENTION_LIMIT_HIGH },
#ifdef USE_SERVOS
    { "tri_unarmed_servo",          VAR_INT8   | MASTER_VALUE, &masterConfig.mixerConfig.tri_unarmed_servo, 0, 1 },
    { "servo_lowpass_freq",         VAR_INT16  | MASTER_VALUE, &masterConfig.mixerConfig.servo_lowpass_freq, 10, 400},
    { "servo_lowpass_enable",       VAR_INT8   | MASTER_VALUE, &masterConfig.mixerConfig.servo_lowpass_enable, 0, 1 },
#endif

    { "default_rate_profile",       VAR_UINT8  | PROFILE_VALUE , &masterConfig.profile[0].defaultRateProfileIndex, 0, MAX_CONTROL_RATE_PROFILE_COUNT - 1 },
    { "rc_rate",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcRate8, 0, 250 },
    { "rc_expo",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcExpo8, 0, 100 },
    { "rc_yaw_expo",                VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcYawExpo8, 0, 100 },
    { "thr_mid",                    VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].thrMid8, 0, 100 },
    { "thr_expo",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].thrExpo8, 0, 100 },
    { "roll_rate",                  VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_ROLL], 0, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX },
    { "pitch_rate",                 VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_PITCH], 0, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX },
    { "yaw_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_YAW], 0, CONTROL_RATE_CONFIG_YAW_RATE_MAX },
    { "tpa_rate",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].dynThrPID, 0, CONTROL_RATE_CONFIG_TPA_MAX},
    { "tpa_breakpoint",             VAR_UINT16 | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].tpa_breakpoint, PWM_RANGE_MIN, PWM_RANGE_MAX},

    { "failsafe_delay",             VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_delay, 0, 200 },
    { "failsafe_off_delay",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_off_delay, 0, 200 },
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_throttle, PWM_RANGE_MIN, PWM_RANGE_MAX },

    { "rx_min_usec",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.rx_min_usec, PWM_PULSE_MIN, PWM_PULSE_MAX },
    { "rx_max_usec",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.rx_max_usec, PWM_PULSE_MIN, PWM_PULSE_MAX },

#ifdef USE_SERVOS
    { "gimbal_flags",               VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].gimbalConfig.gimbal_flags, 0, 255},
#endif

    { "acc_hardware",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.acc_hardware, 0, ACC_MAX },
    { "acc_lpf_factor",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].acc_lpf_factor, 0, 250 },
    { "accxy_deadband",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].accDeadband.xy, 0, 100 },
    { "accz_deadband",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].accDeadband.z, 0, 100 },
    { "accz_lpf_cutoff",            VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].accz_lpf_cutoff, 1, 20 },
    { "acc_unarmedcal",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].acc_unarmedcal, 0, 1 },
    { "acc_trim_pitch",             VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].accelerometerTrims.values.pitch, -300, 300 },
    { "acc_trim_roll",              VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].accelerometerTrims.values.roll, -300, 300 },

    { "baro_tab_size",              VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_sample_count, 0, BARO_SAMPLE_COUNT_MAX },
    { "baro_noise_lpf",             VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_noise_lpf, 0, 1 },
    { "baro_cf_vel",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_cf_vel, 0, 1 },
    { "baro_cf_alt",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].barometerConfig.baro_cf_alt, 0, 1 },

    { "mag_hardware",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.mag_hardware, 0, MAG_MAX },
    { "mag_declination",            VAR_INT16  | PROFILE_VALUE, &masterConfig.profile[0].mag_declination, -18000, 18000 },

    { "pid_controller",             VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.pidController, 0, 5 },

    { "p_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PITCH], 0, 200 },
    { "i_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PITCH], 0, 200 },
    { "d_pitch",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PITCH], 0, 200 },
    { "p_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[ROLL], 0, 200 },
    { "i_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[ROLL], 0, 200 },
    { "d_roll",                     VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[ROLL], 0, 200 },
    { "p_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[YAW], 0, 200 },
    { "i_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[YAW], 0, 200 },
    { "d_yaw",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[YAW], 0, 200 },

    { "p_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[PITCH], 0, 100 },
    { "i_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[PITCH], 0, 100 },
    { "d_pitchf",                   VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[PITCH], 0, 100 },
    { "p_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[ROLL], 0, 100 },
    { "i_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[ROLL], 0, 100 },
    { "d_rollf",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[ROLL], 0, 100 },
    { "p_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[YAW], 0, 100 },
    { "i_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[YAW], 0, 100 },
    { "d_yawf",                     VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[YAW], 0, 100 },

    { "level_horizon",              VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.H_level, 0, 10 },
    { "level_angle",                VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.A_level, 0, 10 },
    { "sensitivity_horizon",        VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.H_sensitivity, 0, 250 },

    { "p_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDALT], 0, 200 },
    { "i_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDALT], 0, 200 },
    { "d_alt",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDALT], 0, 200 },

    { "p_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDLEVEL], 0, 200 },
    { "i_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDLEVEL], 0, 200 },
    { "d_level",                    VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDLEVEL], 0, 200 },

    { "p_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P8[PIDVEL], 0, 200 },
    { "i_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I8[PIDVEL], 0, 200 },
    { "d_vel",                      VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D8[PIDVEL], 0, 200 },

    { "yaw_p_limit",                VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.yaw_p_limit, YAW_P_LIMIT_MIN, YAW_P_LIMIT_MAX },

#ifdef BLACKBOX
    { "blackbox_rate_num",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.blackbox_rate_num, 1, 32 },
    { "blackbox_rate_denom",        VAR_UINT8  | MASTER_VALUE,  &masterConfig.blackbox_rate_denom, 1, 32 },
    { "blackbox_device",            VAR_UINT8  | MASTER_VALUE,  &masterConfig.blackbox_device, 0, 1 },
#endif
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(clivalue_t))


typedef union {
    int32_t int_value;
    float float_value;
} int_float_value_t;

static void cliSetVar(const clivalue_t *var, const int_float_value_t value);
static void cliPrintVar(const clivalue_t *var, uint32_t full);
static void cliPrint(const char *str);
static void cliWrite(uint8_t ch);
static void cliPrompt(void)
{
    cliPrint("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static char *processChannelRangeArgs(char *ptr, channelRange_t *range, uint8_t *validArgumentCount)
{
    int val;
    ptr = strchr(ptr, ' ');
    if (ptr) {
        val = atoi(++ptr);
        val = CHANNEL_VALUE_TO_STEP(val);
        if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
            range->startStep = val;
            (*validArgumentCount)++;
        }
    }
    ptr = strchr(ptr, ' ');
    if (ptr) {
        val = atoi(++ptr);
        val = CHANNEL_VALUE_TO_STEP(val);
        if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
            range->endStep = val;
            (*validArgumentCount)++;
        }
    }
    return ptr;
}

// Check if a string's length is zero
static bool isEmpty(const char *string)
{
    return *string == '\0';
}

static void cliAux(char *cmdline)
{
    int i, val = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        // print out aux channel settings
        for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            modeActivationCondition_t *mac = &currentProfile->modeActivationConditions[i];
            printf("aux %u %u %u %u %u\r\n",
                i,
                mac->modeId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep)
            );
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = &currentProfile->modeActivationConditions[i];
            uint8_t validArgumentCount = 0;
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < CHECKBOX_ITEM_COUNT) {
                    mac->modeId = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
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
            printf("index: must be < %u\r\n", MAX_MODE_ACTIVATION_CONDITION_COUNT);
        }
    }
}

static void cliSerial(char *cmdline)
{
    int i, val;
    char *ptr;

    if (isEmpty(cmdline)) {
        for (i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(masterConfig.serialConfig.portConfigs[i].identifier)) {
                continue;
            };
            printf("serial %d %d %ld %ld %ld %ld\r\n" ,
                masterConfig.serialConfig.portConfigs[i].identifier,
                masterConfig.serialConfig.portConfigs[i].functionMask,
                baudRates[masterConfig.serialConfig.portConfigs[i].msp_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].gps_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].telemetry_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].blackbox_baudrateIndex]
            );
        }
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

    ptr = strchr(ptr, ' ');
    if (ptr) {
        val = atoi(++ptr);
        portConfig.functionMask = val & 0xFFFF;
        validArgumentCount++;
    }

    for (i = 0; i < 4; i ++) {
        ptr = strchr(ptr, ' ');
        if (!ptr) {
            break;
        }

        val = atoi(++ptr);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch(i) {
            case 0:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
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
        cliPrint("Parse error\r\n");
        return;
    }

    memcpy(currentConfig, &portConfig, sizeof(portConfig));

}

static void cliAdjustmentRange(char *cmdline)
{
    int i, val = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        // print out adjustment ranges channel settings
        for (i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            adjustmentRange_t *ar = &currentProfile->adjustmentRanges[i];
            printf("adjrange %u %u %u %u %u %u %u\r\n",
                i,
                ar->adjustmentIndex,
                ar->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
                ar->adjustmentFunction,
                ar->auxSwitchChannelIndex
            );
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = &currentProfile->adjustmentRanges[i];
            uint8_t validArgumentCount = 0;
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                    ar->adjustmentIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &ar->range, &validArgumentCount);
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < ADJUSTMENT_FUNCTION_COUNT) {
                    ar->adjustmentFunction = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxSwitchChannelIndex = val;
                    validArgumentCount++;
                }
            }

            if (validArgumentCount != 6) {
                memset(ar, 0, sizeof(adjustmentRange_t));
            }
        } else {
            printf("index: must be < %u\r\n", MAX_ADJUSTMENT_RANGE_COUNT);
        }
    }
}

static void cliCMix(char *cmdline)
{
#ifdef USE_QUAD_MIXER_ONLY
    UNUSED(cmdline);
#else
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char buf[16];
    float mixsum[3];
    char *ptr;

    if (isEmpty(cmdline)) {
        cliPrint("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (masterConfig.customMixer[i].throttle == 0.0f)
                break;
            num_motors++;
            printf("#%d:\t", i + 1);
            printf("%s\t", ftoa(masterConfig.customMixer[i].throttle, buf));
            printf("%s\t", ftoa(masterConfig.customMixer[i].roll, buf));
            printf("%s\t", ftoa(masterConfig.customMixer[i].pitch, buf));
            printf("%s\r\n", ftoa(masterConfig.customMixer[i].yaw, buf));
        }
        mixsum[0] = mixsum[1] = mixsum[2] = 0.0f;
        for (i = 0; i < num_motors; i++) {
            mixsum[0] += masterConfig.customMixer[i].roll;
            mixsum[1] += masterConfig.customMixer[i].pitch;
            mixsum[2] += masterConfig.customMixer[i].yaw;
        }
        cliPrint("Sanity check:\t");
        for (i = 0; i < 3; i++)
            cliPrint(fabsf(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        cliPrint("\r\n");
        return;
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
            masterConfig.customMixer[i].throttle = 0.0f;
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrint("Invalid mixer type\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i, masterConfig.customMixer);
                    printf("Loaded %s mix\r\n", mixerNames[i]);
                    cliCMix("");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (--i < MAX_SUPPORTED_MOTORS) {
            ptr = strchr(ptr, ' ');
            if (ptr) {
                masterConfig.customMixer[i].throttle = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                masterConfig.customMixer[i].roll = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                masterConfig.customMixer[i].pitch = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                masterConfig.customMixer[i].yaw = fastA2F(++ptr);
                check++;
            }
            if (check != 4) {
                cliPrint("Wrong number of arguments, needs idx thr roll pitch yaw\r\n");
            } else {
                cliCMix("");
            }
        } else {
            printf("Motor number must be between 1 and %d\r\n", MAX_SUPPORTED_MOTORS);
        }
    }
#endif
}

#ifdef LED_STRIP
static void cliLed(char *cmdline)
{
    int i;
    char *ptr;
    char ledConfigBuffer[20];

    if (isEmpty(cmdline)) {
        for (i = 0; i < MAX_LED_STRIP_LENGTH; i++) {
            generateLedConfig(i, ledConfigBuffer, sizeof(ledConfigBuffer));
            printf("led %u %s\r\n", i, ledConfigBuffer);
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < MAX_LED_STRIP_LENGTH) {
            ptr = strchr(cmdline, ' ');
            if (!parseLedStripConfig(i, ++ptr)) {
                cliPrint("Parse error\r\n");
            }
        } else {
            printf("Invalid led index: must be < %u\r\n", MAX_LED_STRIP_LENGTH);
        }
    }
}

static void cliColor(char *cmdline)
{
    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        for (i = 0; i < CONFIGURABLE_COLOR_COUNT; i++) {
            printf("color %u %d,%u,%u\r\n", i, masterConfig.colors[i].h, masterConfig.colors[i].s, masterConfig.colors[i].v);
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < CONFIGURABLE_COLOR_COUNT) {
            ptr = strchr(cmdline, ' ');
            if (!parseColor(i, ++ptr)) {
                cliPrint("Parse error\r\n");
            }
        } else {
            printf("Invalid color index: must be < %u\r\n", CONFIGURABLE_COLOR_COUNT);
        }
    }
}
#endif

static void cliServo(char *cmdline)
{
#ifndef USE_SERVOS
    UNUSED(cmdline);
#else
    enum { SERVO_ARGUMENT_COUNT = 6 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    servoParam_t *servo;

    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        // print out servo settings
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo = &currentProfile->servoConf[i];

            printf("servo %u %d %d %d %d %d\r\n",
                i,
                servo->min,
                servo->max,
                servo->middle,
                servo->rate,
                servo->forwardFromChannel
            );
        }
    } else {
        int validArgumentCount = 0;

        ptr = cmdline;

        // Command line is integers (possibly negative) separated by spaces, no other characters allowed.

        // If command line doesn't fit the format, don't modify the config
        while (*ptr) {
            if (*ptr == '-' || (*ptr >= '0' && *ptr <= '9')) {
                if (validArgumentCount >= SERVO_ARGUMENT_COUNT) {
                    cliPrint("Parse error\r\n");
                    return;
                }

                arguments[validArgumentCount++] = atoi(ptr);

                do {
                    ptr++;
                } while (*ptr >= '0' && *ptr <= '9');
            } else if (*ptr == ' ') {
                ptr++;
            } else {
                cliPrint("Parse error\r\n");
                return;
            }
        }

        // Check we got the right number of args and the servo index is correct (don't validate the other values)
        if (validArgumentCount != SERVO_ARGUMENT_COUNT || arguments[0] < 0 || arguments[0] >= MAX_SUPPORTED_SERVOS) {
            cliPrint("Parse error\r\n");
            return;
        }

        servo = &currentProfile->servoConf[arguments[0]];

        servo->min = arguments[1];
        servo->max = arguments[2];
        servo->middle = arguments[3];
        servo->rate = arguments[4];
        servo->forwardFromChannel = arguments[5];
    }
#endif
}

#ifdef USE_FLASHFS

static void cliFlashInfo(char *cmdline)
{
    const flashGeometry_t *layout = flashfsGetGeometry();

    UNUSED(cmdline);

    printf("Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u, usedSize=%u\r\n",
            layout->sectors, layout->sectorSize, layout->pagesPerSector, layout->pageSize, layout->totalSize, flashfsGetOffset());
}

static void cliFlashErase(char *cmdline)
{
    UNUSED(cmdline);

    printf("Erasing, please wait...\r\n");
    flashfsEraseCompletely();

    while (!flashfsIsReady()) {
        delay(100);
    }

    printf("Done.\r\n");
}

static void cliFlashWrite(char *cmdline)
{
    uint32_t address = atoi(cmdline);
    char *text = strchr(cmdline, ' ');

    if (!text) {
        printf("Missing text to write.\r\n");
    } else {
        flashfsSeekAbs(address);
        flashfsWrite((uint8_t*)text, strlen(text), true);
        flashfsFlushSync();

        printf("Wrote %u bytes at %u.\r\n", strlen(text), address);
    }
}

static void cliFlashRead(char *cmdline)
{
    uint32_t address = atoi(cmdline);
    uint32_t length;
    int i;

    uint8_t buffer[32];

    char *nextArg = strchr(cmdline, ' ');

    if (!nextArg) {
        printf("Missing length argument.\r\n");
    } else {
        length = atoi(nextArg);

        printf("Reading %u bytes at %u:\r\n", length, address);

        while (length > 0) {
            int bytesRead;

            bytesRead = flashfsReadAbs(address, buffer, length < sizeof(buffer) ? length : sizeof(buffer));

            for (i = 0; i < bytesRead; i++) {
                cliWrite(buffer[i]);
            }

            length -= bytesRead;
            address += bytesRead;

            if (bytesRead == 0) {
                //Assume we reached the end of the volume or something fatal happened
                break;
            }
        }
        printf("\r\n");
    }
}

#endif

static void dumpValues(uint16_t mask)
{
    uint32_t i;
    const clivalue_t *value;
    for (i = 0; i < VALUE_COUNT; i++) {
        value = &valueTable[i];

        if ((value->type & mask) == 0) {
            continue;
        }

        printf("set %s = ", valueTable[i].name);
        cliPrintVar(value, 0);
        cliPrint("\r\n");
    }
}

typedef enum {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_CONTROL_RATE_PROFILE = (1 << 2)
} dumpFlags_e;

#define DUMP_ALL (DUMP_MASTER | DUMP_PROFILE | DUMP_CONTROL_RATE_PROFILE)


static const char* const sectionBreak = "\r\n";

#define printSectionBreak() printf((char *)sectionBreak)

static void cliDump(char *cmdline)
{
    unsigned int i;
    char buf[16];
    uint32_t mask;

#ifndef USE_QUAD_MIXER_ONLY
    float thr, roll, pitch, yaw;
#endif

    uint8_t dumpMask = DUMP_ALL;
    if (strcasecmp(cmdline, "master") == 0) {
        dumpMask = DUMP_MASTER; // only
    }
    if (strcasecmp(cmdline, "profile") == 0) {
        dumpMask = DUMP_PROFILE; // only
    }
    if (strcasecmp(cmdline, "rates") == 0) {
        dumpMask = DUMP_CONTROL_RATE_PROFILE; // only
    }

    if (dumpMask & DUMP_MASTER) {

        cliPrint("\r\n# version\r\n");
        cliVersion(NULL);

        cliPrint("\r\n# dump master\r\n");
        cliPrint("\r\n# mixer\r\n");

#ifndef USE_QUAD_MIXER_ONLY
        printf("mixer %s\r\n", mixerNames[masterConfig.mixerMode - 1]);

        if (masterConfig.customMixer[0].throttle != 0.0f) {
            for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
                if (masterConfig.customMixer[i].throttle == 0.0f)
                    break;
                thr = masterConfig.customMixer[i].throttle;
                roll = masterConfig.customMixer[i].roll;
                pitch = masterConfig.customMixer[i].pitch;
                yaw = masterConfig.customMixer[i].yaw;
                printf("cmix %d", i + 1);
                if (thr < 0)
                    cliWrite(' ');
                printf("%s", ftoa(thr, buf));
                if (roll < 0)
                    cliWrite(' ');
                printf("%s", ftoa(roll, buf));
                if (pitch < 0)
                    cliWrite(' ');
                printf("%s", ftoa(pitch, buf));
                if (yaw < 0)
                    cliWrite(' ');
                printf("%s\r\n", ftoa(yaw, buf));
            }
            printf("cmix %d 0 0 0 0\r\n", i + 1);
        }
#endif

        cliPrint("\r\n\r\n# feature\r\n");

        mask = featureMask();
        for (i = 0; ; i++) { // disable all feature first
            if (featureNames[i] == NULL)
                break;
            printf("feature -%s\r\n", featureNames[i]);
        }
        for (i = 0; ; i++) {  // reenable what we want.
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                printf("feature %s\r\n", featureNames[i]);
        }

        cliPrint("\r\n\r\n# map\r\n");

        for (i = 0; i < 8; i++)
            buf[masterConfig.rxConfig.rcmap[i]] = rcChannelLetters[i];
        buf[i] = '\0';
        printf("map %s\r\n", buf);

        cliPrint("\r\n\r\n# serial\r\n");
        cliSerial("");

#ifdef LED_STRIP
        cliPrint("\r\n\r\n# led\r\n");
        cliLed("");

        cliPrint("\r\n\r\n# color\r\n");
        cliColor("");
#endif
        printSectionBreak();
        dumpValues(MASTER_VALUE);
    }

    if (dumpMask & DUMP_PROFILE) {
        cliPrint("\r\n# dump profile\r\n");

        cliPrint("\r\n# profile\r\n");
        cliProfile("");

        cliPrint("\r\n# aux\r\n");

        cliAux("");

        cliPrint("\r\n# adjrange\r\n");

        cliAdjustmentRange("");

        cliPrint("\r\n# servo\r\n");

        cliServo("");

        printSectionBreak();

        dumpValues(PROFILE_VALUE);
    }

    if (dumpMask & DUMP_CONTROL_RATE_PROFILE) {
        cliPrint("\r\n# dump rates\r\n");

        cliPrint("\r\n# rateprofile\r\n");
        cliRateProfile("");

        printSectionBreak();

        dumpValues(CONTROL_RATE_VALUE);
    }
}

void cliEnter(serialPort_t *serialPort)
{
    cliMode = 1;
    cliPort = serialPort;
    setPrintfSerialPort(cliPort);
    cliPrint("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
    cliPrompt();
}

static void cliExit(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("\r\nLeaving CLI mode, unsaved changes lost.\r\n");
    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
    // incase a motor was left running during motortest, clear it here
    mixerResetMotors();
    cliReboot();

    cliPort = NULL;
}

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask();

    if (len == 0) {
        cliPrint("Enabled features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                printf("%s ", featureNames[i]);
        }
        cliPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            printf("%s ", featureNames[i]);
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

        for (i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                cliPrint("Invalid feature name\r\n");
                break;
            }

            if (strncasecmp(cmdline, featureNames[i], len) == 0) {

                mask = 1 << i;
#ifndef GPS
                if (mask & FEATURE_GPS) {
                    cliPrint("GPS unavailable\r\n");
                    break;
                }
#endif
#ifndef SONAR
                if (mask & FEATURE_SONAR) {
                    cliPrint("SONAR unavailable\r\n");
                    break;
                }
#endif
                if (remove) {
                    featureClear(mask);
                    cliPrint("Disabled ");
                } else {
                    featureSet(mask);
                    cliPrint("Enabled ");
                }
                printf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

#ifdef GPS
static void cliGpsPassthrough(char *cmdline)
{
    UNUSED(cmdline);

    gpsEnablePassthrough(cliPort);
}
#endif

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

    UNUSED(cmdline);

    cliPrint("Available commands:\r\n");
    for (i = 0; i < CMD_COUNT; i++)
        printf("%s\t%s\r\n", cmdTable[i].name, cmdTable[i].param);
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];

    len = strlen(cmdline);

    if (len == 8) {
        // uppercase it
        for (i = 0; i < 8; i++)
            cmdline[i] = toupper((unsigned char)cmdline[i]);
        for (i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            cliPrint("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline, &masterConfig.rxConfig);
    }
    cliPrint("Current assignment: ");
    for (i = 0; i < 8; i++)
        out[masterConfig.rxConfig.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    printf("%s\r\n", out);
}

#ifndef USE_QUAD_MIXER_ONLY
static void cliMixer(char *cmdline)
{
    int i;
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        printf("Current mixer: %s\r\n", mixerNames[masterConfig.mixerMode - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            printf("%s ", mixerNames[i]);
        }
        cliPrint("\r\n");
        return;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            cliPrint("Invalid mixer type\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            masterConfig.mixerMode = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}
#endif

static void cliMotor(char *cmdline)
{
    int motor_index = 0;
    int motor_value = 0;
    int index = 0;
    char *pch = NULL;

    if (isEmpty(cmdline)) {
        cliPrint("Usage:\r\nmotor index [value] - show [or set] motor value\r\n");
        return;
    }

    pch = strtok(cmdline, " ");
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
        pch = strtok(NULL, " ");
    }

    if (motor_index < 0 || motor_index >= MAX_SUPPORTED_MOTORS) {
        printf("No such motor, use a number [0, %d]\r\n", MAX_SUPPORTED_MOTORS);
        return;
    }

    if (index < 2) {
        printf("Motor %d is set at %d\r\n", motor_index, motor_disarmed[motor_index]);
        return;
    }

    if (motor_value < PWM_RANGE_MIN || motor_value > PWM_RANGE_MAX) {
        cliPrint("Invalid motor value, 1000..2000\r\n");
        return;
    }

    printf("Setting motor %d to %d\r\n", motor_index, motor_value);
    motor_disarmed[motor_index] = motor_value;
}

static void cliPlaySound(char *cmdline)
{
#if FLASH_SIZE <= 64
    UNUSED(cmdline);
#else
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
                    printf("Error playing sound\r\n");
                    return;
                }
            }
        }
    } else {       //index value was given
        i = atoi(cmdline);
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            printf("No sound for index %d\r\n", i);
            return;
        }
    }
    lastSoundIdx = i;
    beeperSilence();
    printf("Playing sound %d: %s\r\n", i, name);
    beeper(beeperModeForTableIndex(i));
#endif
}

static void cliProfile(char *cmdline)
{
    int i;

    if (isEmpty(cmdline)) {
        printf("profile %d\r\n", getCurrentProfile());
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
        printf("rateprofile %d\r\n", getCurrentControlRateProfile());
        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i < MAX_CONTROL_RATE_PROFILE_COUNT) {
            changeControlRateProfile(i);
            cliRateProfile("");
        }
    }
}

static void cliReboot(void) {
    cliPrint("\r\nRebooting");
    waitForSerialPortToFinishTransmitting(cliPort);
    stopMotors();
    handleOneshotFeatureChangeOnRestart();
    systemReset();
}

static void cliSave(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("Saving");
    //copyCurrentProfileToProfileSlot(masterConfig.current_profile_index);
    writeEEPROM();
    cliReboot();
}

static void cliDefaults(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("Resetting to defaults");
    resetEEPROM();
    cliReboot();
}

static void cliPrint(const char *str)
{
    while (*str)
        serialWrite(cliPort, *(str++));
}

static void cliWrite(uint8_t ch)
{
    serialWrite(cliPort, ch);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    void *ptr = var->ptr;
    if (var->type & PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }
    if (var->type & CONTROL_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
    }

    switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            value = *(uint8_t *)ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)ptr;
            break;

        case VAR_FLOAT:
            printf("%s", ftoa(*(float *)ptr, buf));
            if (full) {
                printf(" %s", ftoa((float)var->min, buf));
                printf(" %s", ftoa((float)var->max, buf));
            }
            return; // return from case for float only
    }
    printf("%d", value);
    if (full)
        printf(" %d %d", var->min, var->max);
}

static void cliSetVar(const clivalue_t *var, const int_float_value_t value)
{
    void *ptr = var->ptr;
    if (var->type & PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }
    if (var->type & CONTROL_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
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

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;
    int32_t value = 0;
    float valuef = 0;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            printf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            cliPrint("\r\n");
        }
    } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
        // has equal, set var
        char *lastNonSpaceCharacter = eqptr;
        while (*(lastNonSpaceCharacter - 1) == ' ') {
            lastNonSpaceCharacter--;
        }
        uint8_t variableNameLength = lastNonSpaceCharacter - cmdline;

        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = fastA2F(eqptr);
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            // ensure exact match when setting to prevent setting variables with shorter names
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0 && variableNameLength == strlen(valueTable[i].name)) {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max) { // here we compare the float value since... it should work, RIGHT?
                    int_float_value_t tmp;
                    if (valueTable[i].type & VAR_FLOAT)
                        tmp.float_value = valuef;
                    else
                        tmp.int_value = value;
                    cliSetVar(val, tmp);
                    printf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    cliPrint("Value assignment out of range\r\n");
                }
                return;
            }
        }
        cliPrint("Unknown variable name\r\n");
    } else {
        // no equals, check for matching variables.
    	cliGet(cmdline);
    }
}

static void cliGet(char *cmdline)
{
    uint32_t i;
    const clivalue_t *val;
    int matchedCommands = 0;

    for (i = 0; i < VALUE_COUNT; i++) {
        if (strstr(valueTable[i].name, cmdline)) {
            val = &valueTable[i];
            printf("%s = ", valueTable[i].name);
            cliPrintVar(val, 0);
            cliPrint("\r\n");

            matchedCommands++;
        }
    }


    if (matchedCommands) {
    	return;
    }

    cliPrint("Unknown variable name\r\n");
}

static void cliStatus(char *cmdline)
{
    UNUSED(cmdline);

    printf("System Uptime: %d seconds, Voltage: %d * 0.1V (%dS battery)\r\n",
        millis() / 1000, vbat, batteryCellCount);


    printf("CPU Clock=%dMHz", (SystemCoreClock / 1000000));

#ifndef CJMCU
    uint8_t i;
    uint32_t mask;
    uint32_t detectedSensorsMask = sensorsMask();

    for (i = 0; ; i++) {

        if (sensorTypeNames[i] == NULL)
            break;

        mask = (1 << i);
        if ((detectedSensorsMask & mask) && (mask & SENSOR_NAMES_MASK)) {
            const char *sensorHardware;
            uint8_t sensorHardwareIndex = detectedSensors[i];
            sensorHardware = sensorHardwareNames[i][sensorHardwareIndex];

            printf(", %s=%s", sensorTypeNames[i], sensorHardware);

            if (mask == SENSOR_ACC && acc.revisionCode) {
                printf(".%c", acc.revisionCode);
            }
        }
    }
#endif
    cliPrint("\r\n");

#ifdef USE_I2C
    uint16_t i2cErrorCounter = i2cGetErrorCounter();
#else
    uint16_t i2cErrorCounter = 0;
#endif

    printf("Cycle Time: %d, I2C Errors: %d, config size: %d\r\n", cycleTime, i2cErrorCounter, sizeof(master_t));
}

static void cliVersion(char *cmdline)
{
    UNUSED(cmdline);

    printf("# Cleanflight/%s %s %s / %s (%s)",
        targetName,
        FC_VERSION_STRING,
        buildDate,
        buildTime,
        shortGitRevision
    );
}

void cliProcess(void)
{
    if (!cliPort) {
        return;
    }

    while (serialTotalBytesWaiting(cliPort)) {
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
        } else if (c == 12) {
            // clear screen
            cliPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            clicmd_t *cmd = NULL;
            clicmd_t target;
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
                target.name = cliBuffer;
                target.param = NULL;

                cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
                if (cmd)
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

void cliInit(serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);
}
#endif
