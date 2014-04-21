#include <stdarg.h>

#include "board.h"
#include "mw.h"

#include "common/printf.h"

#include "drivers/serial_common.h"
#include "serial_common.h"

#include "telemetry_common.h"
#include "gps_common.h"
#include "sensors_acceleration.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliAux(char *cmdline);
static void cliCMix(char *cmdline);
static void cliDefaults(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliGpsPassthrough(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliMotor(char *cmdline);
static void cliProfile(char *cmdline);
static void cliSave(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);

// from sensors.c
extern uint8_t batteryCellCount;

// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// from mixer.c
extern int16_t motor_disarmed[MAX_MOTORS];

// signal that we're in cli mode
uint8_t cliMode = 0;

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

static float _atof(const char *p);
static char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", 
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "CUSTOM", NULL
};

// sync this with AvailableFeatures enum from board.h
static const char * const featureNames[] = {
    "PPM", "VBAT", "INFLIGHT_ACC_CAL", "SERIALRX", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "LED_RING", "GPS",
    "FAILSAFE", "SONAR", "TELEMETRY", "POWERMETER", "VARIO", "3D",
    NULL
};

// sync this with AvailableSensors enum from board.h
static const char * const sensorNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "SONAR", "GPS", "GPS+MAG", NULL
};

static const char * const accNames[] = {
    "", "ADXL345", "MPU6050", "MMA845x", "BMA280", "None", NULL
};

typedef struct {
    const char *name;
    const char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    { "aux", "feature_name auxflag or blank for list", cliAux },
    { "cmix", "design custom mixer", cliCMix },
    { "defaults", "reset to defaults and reboot", cliDefaults },
    { "dump", "print configurable settings in a pastable form", cliDump },
    { "exit", "", cliExit },
    { "feature", "list or -val or val", cliFeature },
    { "gpspassthrough", "passthrough gps to serial", cliGpsPassthrough },
    { "help", "", cliHelp },
    { "map", "mapping of rc channel order", cliMap },
    { "mixer", "mixer name or list", cliMixer },
    { "motor", "get/set motor output value", cliMotor },
    { "profile", "index (0 to 2)", cliProfile },
    { "save", "save and reboot", cliSave },
    { "set", "name=value or blank or * for list", cliSet },
    { "status", "show system status", cliStatus },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(clicmd_t))

typedef enum {
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

typedef struct {
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
} clivalue_t;

const clivalue_t valueTable[] = {
    { "looptime", VAR_UINT16, &mcfg.looptime, 0, 9000 },
    { "midrc", VAR_UINT16, &mcfg.rxConfig.midrc, 1200, 1700 },
    { "minthrottle", VAR_UINT16, &mcfg.minthrottle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "maxthrottle", VAR_UINT16, &mcfg.maxthrottle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "mincommand", VAR_UINT16, &mcfg.mincommand, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "mincheck", VAR_UINT16, &mcfg.rxConfig.mincheck, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "maxcheck", VAR_UINT16, &mcfg.rxConfig.maxcheck, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "deadband3d_low", VAR_UINT16, &mcfg.deadband3d_low, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "deadband3d_high", VAR_UINT16, &mcfg.deadband3d_high, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "neutral3d", VAR_UINT16, &mcfg.neutral3d, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "deadband3d_throttle", VAR_UINT16, &mcfg.deadband3d_throttle, PWM_RANGE_ZERO, PWM_RANGE_MAX },
    { "motor_pwm_rate", VAR_UINT16, &mcfg.motor_pwm_rate, 50, 32000 },
    { "servo_pwm_rate", VAR_UINT16, &mcfg.servo_pwm_rate, 50, 498 },
    { "retarded_arm", VAR_UINT8, &mcfg.retarded_arm, 0, 1 },
    { "flaps_speed", VAR_UINT8, &mcfg.flaps_speed, 0, 100 },
    { "fixedwing_althold_dir", VAR_INT8, &mcfg.fixedwing_althold_dir, -1, 1 },
    { "reboot_character", VAR_UINT8, &mcfg.serialConfig.reboot_character, 48, 126 },
    { "serial_baudrate", VAR_UINT32, &mcfg.serialConfig.port1_baudrate, 1200, 115200 },
    { "softserial_baudrate", VAR_UINT32, &mcfg.serialConfig.softserial_baudrate, 1200, 19200 },
    { "softserial_1_inverted", VAR_UINT8, &mcfg.serialConfig.softserial_1_inverted, 0, 1 },
    { "softserial_2_inverted", VAR_UINT8, &mcfg.serialConfig.softserial_2_inverted, 0, 1 },
    { "gps_type", VAR_UINT8, &mcfg.gps_type, 0, GPS_HARDWARE_MAX },
    { "gps_baudrate", VAR_INT8, &mcfg.gps_baudrate, 0, GPS_BAUD_MAX },
    { "serialrx_type", VAR_UINT8, &mcfg.rxConfig.serialrx_type, 0, 3 },
    { "telemetry_provider", VAR_UINT8, &mcfg.telemetry_provider, 0, TELEMETRY_PROVIDER_MAX },
    { "telemetry_port", VAR_UINT8, &mcfg.telemetry_port, 0, TELEMETRY_PORT_MAX },
    { "telemetry_switch", VAR_UINT8, &mcfg.telemetry_switch, 0, 1 },
    { "vbatscale", VAR_UINT8, &mcfg.batteryConfig.vbatscale, 10, 200 },
    { "vbatmaxcellvoltage", VAR_UINT8, &mcfg.batteryConfig.vbatmaxcellvoltage, 10, 50 },
    { "vbatmincellvoltage", VAR_UINT8, &mcfg.batteryConfig.vbatmincellvoltage, 10, 50 },
    { "power_adc_channel", VAR_UINT8, &mcfg.power_adc_channel, 0, 9 },
    { "align_gyro", VAR_UINT8, &mcfg.gyro_align, 0, 8 },
    { "align_acc", VAR_UINT8, &mcfg.acc_align, 0, 8 },
    { "align_mag", VAR_UINT8, &mcfg.mag_align, 0, 8 },
    { "align_board_roll", VAR_INT16, &mcfg.boardAlignment.rollDegrees, -180, 360 },
    { "align_board_pitch", VAR_INT16, &mcfg.boardAlignment.pitchDegrees, -180, 360 },
    { "align_board_yaw", VAR_INT16, &mcfg.boardAlignment.yawDegrees, -180, 360 },
    { "yaw_control_direction", VAR_INT8, &mcfg.yaw_control_direction, -1, 1 },
    { "acc_hardware", VAR_UINT8, &mcfg.acc_hardware, 0, 5 },
    { "max_angle_inclination", VAR_UINT16, &mcfg.max_angle_inclination, 100, 900 },
    { "moron_threshold", VAR_UINT8, &mcfg.moron_threshold, 0, 128 },
    { "gyro_lpf", VAR_UINT16, &mcfg.gyro_lpf, 0, 256 },
    { "gyro_cmpf_factor", VAR_UINT16, &mcfg.gyro_cmpf_factor, 100, 1000 },
    { "gyro_cmpfm_factor", VAR_UINT16, &mcfg.gyro_cmpfm_factor, 100, 1000 },
    { "pid_controller", VAR_UINT8, &cfg.pidController, 0, 1 },
    { "deadband", VAR_UINT8, &cfg.deadband, 0, 32 },
    { "yawdeadband", VAR_UINT8, &cfg.yawdeadband, 0, 100 },
    { "alt_hold_throttle_neutral", VAR_UINT8, &cfg.alt_hold_throttle_neutral, 1, 250 },
    { "alt_hold_fast_change", VAR_UINT8, &cfg.alt_hold_fast_change, 0, 1 },
    { "throttle_correction_value", VAR_UINT8, &cfg.throttle_correction_value, 0, 150 },
    { "throttle_correction_angle", VAR_UINT16, &cfg.throttle_correction_angle, 1, 900 },
    { "rc_rate", VAR_UINT8, &cfg.controlRateConfig.rcRate8, 0, 250 },
    { "rc_expo", VAR_UINT8, &cfg.controlRateConfig.rcExpo8, 0, 100 },
    { "thr_mid", VAR_UINT8, &cfg.controlRateConfig.thrMid8, 0, 100 },
    { "thr_expo", VAR_UINT8, &cfg.controlRateConfig.thrExpo8, 0, 100 },
    { "roll_pitch_rate", VAR_UINT8, &cfg.controlRateConfig.rollPitchRate, 0, 100 },
    { "yawrate", VAR_UINT8, &cfg.controlRateConfig.yawRate, 0, 100 },
    { "tparate", VAR_UINT8, &cfg.dynThrPID, 0, 100},
    { "tpa_breakpoint", VAR_UINT16, &cfg.tpaBreakPoint, PWM_RANGE_MIN, PWM_RANGE_MAX},
    { "failsafe_delay", VAR_UINT8, &cfg.failsafeConfig.failsafe_delay, 0, 200 },
    { "failsafe_off_delay", VAR_UINT8, &cfg.failsafeConfig.failsafe_off_delay, 0, 200 },
    { "failsafe_throttle", VAR_UINT16, &cfg.failsafeConfig.failsafe_throttle, PWM_RANGE_MIN, PWM_RANGE_MAX },
    { "failsafe_detect_threshold", VAR_UINT16, &cfg.failsafeConfig.failsafe_detect_threshold, 100, PWM_RANGE_MAX },
    { "rssi_aux_channel", VAR_INT8, &mcfg.rssi_aux_channel, 0, 4 },
    { "yaw_direction", VAR_INT8, &cfg.yaw_direction, -1, 1 },
    { "tri_unarmed_servo", VAR_INT8, &cfg.tri_unarmed_servo, 0, 1 },
    { "gimbal_flags", VAR_UINT8, &cfg.gimbal_flags, 0, 255},
    { "acc_lpf_factor", VAR_UINT8, &cfg.acc_lpf_factor, 0, 250 },
    { "accxy_deadband", VAR_UINT8, &cfg.accxy_deadband, 0, 100 },
    { "accz_deadband", VAR_UINT8, &cfg.accz_deadband, 0, 100 },
    { "acc_unarmedcal", VAR_UINT8, &cfg.acc_unarmedcal, 0, 1 },
    { "acc_trim_pitch", VAR_INT16, &cfg.angleTrim[PITCH], -300, 300 },
    { "acc_trim_roll", VAR_INT16, &cfg.angleTrim[ROLL], -300, 300 },
    { "baro_tab_size", VAR_UINT8, &cfg.baro_tab_size, 0, BARO_TAB_SIZE_MAX },
    { "baro_noise_lpf", VAR_FLOAT, &cfg.baro_noise_lpf, 0, 1 },
    { "baro_cf_vel", VAR_FLOAT, &cfg.baro_cf_vel, 0, 1 },
    { "baro_cf_alt", VAR_FLOAT, &cfg.baro_cf_alt, 0, 1 },
    { "mag_declination", VAR_INT16, &cfg.mag_declination, -18000, 18000 },
    { "gps_pos_p", VAR_UINT8, &cfg.P8[PIDPOS], 0, 200 },
    { "gps_pos_i", VAR_UINT8, &cfg.I8[PIDPOS], 0, 200 },
    { "gps_pos_d", VAR_UINT8, &cfg.D8[PIDPOS], 0, 200 },
    { "gps_posr_p", VAR_UINT8, &cfg.P8[PIDPOSR], 0, 200 },
    { "gps_posr_i", VAR_UINT8, &cfg.I8[PIDPOSR], 0, 200 },
    { "gps_posr_d", VAR_UINT8, &cfg.D8[PIDPOSR], 0, 200 },
    { "gps_nav_p", VAR_UINT8, &cfg.P8[PIDNAVR], 0, 200 },
    { "gps_nav_i", VAR_UINT8, &cfg.I8[PIDNAVR], 0, 200 },
    { "gps_nav_d", VAR_UINT8, &cfg.D8[PIDNAVR], 0, 200 },
    { "gps_wp_radius", VAR_UINT16, &cfg.gps_wp_radius, 0, 2000 },
    { "nav_controls_heading", VAR_UINT8, &cfg.nav_controls_heading, 0, 1 },
    { "nav_speed_min", VAR_UINT16, &cfg.nav_speed_min, 10, 2000 },
    { "nav_speed_max", VAR_UINT16, &cfg.nav_speed_max, 10, 2000 },
    { "nav_slew_rate", VAR_UINT8, &cfg.nav_slew_rate, 0, 100 },
    { "p_pitch", VAR_UINT8, &cfg.P8[PITCH], 0, 200 },
    { "i_pitch", VAR_UINT8, &cfg.I8[PITCH], 0, 200 },
    { "d_pitch", VAR_UINT8, &cfg.D8[PITCH], 0, 200 },
    { "p_roll", VAR_UINT8, &cfg.P8[ROLL], 0, 200 },
    { "i_roll", VAR_UINT8, &cfg.I8[ROLL], 0, 200 },
    { "d_roll", VAR_UINT8, &cfg.D8[ROLL], 0, 200 },
    { "p_yaw", VAR_UINT8, &cfg.P8[YAW], 0, 200 },
    { "i_yaw", VAR_UINT8, &cfg.I8[YAW], 0, 200 },
    { "d_yaw", VAR_UINT8, &cfg.D8[YAW], 0, 200 },
    { "p_alt", VAR_UINT8, &cfg.P8[PIDALT], 0, 200 },
    { "i_alt", VAR_UINT8, &cfg.I8[PIDALT], 0, 200 },
    { "d_alt", VAR_UINT8, &cfg.D8[PIDALT], 0, 200 },
    { "p_level", VAR_UINT8, &cfg.P8[PIDLEVEL], 0, 200 },
    { "i_level", VAR_UINT8, &cfg.I8[PIDLEVEL], 0, 200 },
    { "d_level", VAR_UINT8, &cfg.D8[PIDLEVEL], 0, 200 },
    { "p_vel", VAR_UINT8, &cfg.P8[PIDVEL], 0, 200 },
    { "i_vel", VAR_UINT8, &cfg.I8[PIDVEL], 0, 200 },
    { "d_vel", VAR_UINT8, &cfg.D8[PIDVEL], 0, 200 },
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

#ifndef HAVE_ITOA_FUNCTION

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0)
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0) {
        *a = '-';
        *i2a(-(unsigned)i, a + 1, r) = 0;
    } else
        *i2a(i, a, r) = 0;
    return a;
}

#endif

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
static float _atof(const char *p)
{
    int frac = 0;
    float sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p) ) {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0f;
    if (*p == '-') {
        sign = -1.0f;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0f;
    while (valid_digit(*p)) {
        value = value * 10.0f + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.') {
        float pow10 = 10.0f;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0f;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0f;
    if ((*p == 'e') || (*p == 'E')) {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.
        frac = 0;
        if (*p == '-') {
            frac = 1;
            p += 1;

        } else if (*p == '+') {
            p += 1;
        }

        // Get digits of exponent, if any.
        expon = 0;
        while (valid_digit(*p)) {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) 
            expon = 308;

        // Calculate scaling factor.
        // while (expon >= 50) { scale *= 1E50f; expon -= 50; }
        while (expon >=  8) { scale *= 1E8f;  expon -=  8; }
        while (expon >   0) { scale *= 10.0f; expon -=  1; }
    }

    // Return signed and scaled floating point result.
    return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////
static char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t) (x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1) {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 2) {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 3) {
        intString2[1] = '0';
        strcat(intString2, intString1);
    } else {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

static void cliPrompt(void)
{
    cliPrint("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static void cliAux(char *cmdline)
{
    int i, val = 0;
    uint8_t len;
    char *ptr;

    len = strlen(cmdline);
    if (len == 0) {
        // print out aux channel settings
        for (i = 0; i < CHECKBOX_ITEM_COUNT; i++)
            printf("aux %u %u\r\n", i, cfg.activate[i]);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < CHECKBOX_ITEM_COUNT) {
            ptr = strchr(cmdline, ' ');
            val = atoi(ptr);
            cfg.activate[i] = val;
        } else {
            printf("Invalid Feature index: must be < %u\r\n", CHECKBOX_ITEM_COUNT);
        }
    }
}

static void cliCMix(char *cmdline)
{
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char buf[16];
    float mixsum[3];
    char *ptr;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrint("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_MOTORS; i++) {
            if (mcfg.customMixer[i].throttle == 0.0f)
                break;
            num_motors++;
            printf("#%d:\t", i + 1);
            printf("%s\t", ftoa(mcfg.customMixer[i].throttle, buf));
            printf("%s\t", ftoa(mcfg.customMixer[i].roll, buf));
            printf("%s\t", ftoa(mcfg.customMixer[i].pitch, buf));
            printf("%s\r\n", ftoa(mcfg.customMixer[i].yaw, buf));
        }
        mixsum[0] = mixsum[1] = mixsum[2] = 0.0f;
        for (i = 0; i < num_motors; i++) {
            mixsum[0] += mcfg.customMixer[i].roll;
            mixsum[1] += mcfg.customMixer[i].pitch;
            mixsum[2] += mcfg.customMixer[i].yaw;
        }
        cliPrint("Sanity check:\t");
        for (i = 0; i < 3; i++)
            cliPrint(fabsf(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        cliPrint("\r\n");
        return;
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (i = 0; i < MAX_MOTORS; i++)
            mcfg.customMixer[i].throttle = 0.0f;
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrint("Invalid mixer type...\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i);
                    printf("Loaded %s mix...\r\n", mixerNames[i]);
                    cliCMix("");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (--i < MAX_MOTORS) {
            ptr = strchr(ptr, ' ');
            if (ptr) {
                mcfg.customMixer[i].throttle = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                mcfg.customMixer[i].roll = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                mcfg.customMixer[i].pitch = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                mcfg.customMixer[i].yaw = _atof(++ptr);
                check++;
            }
            if (check != 4) {
                cliPrint("Wrong number of arguments, needs idx thr roll pitch yaw\r\n");
            } else {
                cliCMix("");
            }
        } else {
            printf("Motor number must be between 1 and %d\r\n", MAX_MOTORS);
        }
    }
}

static void cliDump(char *cmdline)
{
    int i;
    char buf[16];
    float thr, roll, pitch, yaw;
    uint32_t mask;
    const clivalue_t *setval;

    printf("Current Config: Copy everything below here...\r\n");

    // print out aux switches
    cliAux("");

    // print out current motor mix
    printf("mixer %s\r\n", mixerNames[mcfg.mixerConfiguration - 1]);

    // print custom mix if exists
    if (mcfg.customMixer[0].throttle != 0.0f) {
        for (i = 0; i < MAX_MOTORS; i++) {
            if (mcfg.customMixer[i].throttle == 0.0f)
                break;
            thr = mcfg.customMixer[i].throttle;
            roll = mcfg.customMixer[i].roll;
            pitch = mcfg.customMixer[i].pitch;
            yaw = mcfg.customMixer[i].yaw;
            printf("cmix %d", i + 1);
            if (thr < 0)
                printf(" ");
            printf("%s", ftoa(thr, buf));
            if (roll < 0)
                printf(" ");
            printf("%s", ftoa(roll, buf));
            if (pitch < 0)
                printf(" ");
            printf("%s", ftoa(pitch, buf));
            if (yaw < 0)
                printf(" ");
            printf("%s\r\n", ftoa(yaw, buf));
        }
        printf("cmix %d 0 0 0 0\r\n", i + 1);
    }

    // print enabled features
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

    // print RC MAPPING
    for (i = 0; i < 8; i++)
        buf[mcfg.rxConfig.rcmap[i]] = rcChannelLetters[i];
    buf[i] = '\0';
    printf("map %s\r\n", buf);

    // print settings
    for (i = 0; i < VALUE_COUNT; i++) {
        setval = &valueTable[i];
        printf("set %s = ", valueTable[i].name);
        cliPrintVar(setval, 0);
        cliPrint("\r\n");
    }
}

static void cliExit(char *cmdline)
{
    cliPrint("\r\nLeaving CLI mode...\r\n");
    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
    // incase some idiot leaves a motor running during motortest, clear it here
    mixerResetMotors();
    // save and reboot... I think this makes the most sense
    cliSave(cmdline);
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
                cliPrint("Invalid feature name...\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0) {
                if (remove) {
                    featureClear(1 << i);
                    cliPrint("Disabled ");
                } else {
                    featureSet(1 << i);
                    cliPrint("Enabled ");
                }
                printf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

static void cliGpsPassthrough(char *cmdline)
{
    if (gpsSetPassthrough() == -1)
        cliPrint("Error: Enable and plug in GPS first\r\n");
    else
        cliPrint("Enabling GPS passthrough...\r\n");
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

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
        parseRcChannels(cmdline, &mcfg.rxConfig);
    }
    cliPrint("Current assignment: ");
    for (i = 0; i < 8; i++)
        out[mcfg.rxConfig.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    printf("%s\r\n", out);
}

static void cliMixer(char *cmdline)
{
    int i;
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        printf("Current mixer: %s\r\n", mixerNames[mcfg.mixerConfiguration - 1]);
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
            cliPrint("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            mcfg.mixerConfiguration = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}

static void cliMotor(char *cmdline)
{
    int motor_index = 0;
    int motor_value = 0;
    int len, index = 0;
    char *pch = NULL;

    len = strlen(cmdline);
    if (len == 0) {
        printf("Usage:\r\nmotor index [value] - show [or set] motor value\r\n");
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

    if (motor_index < 0 || motor_index >= MAX_MOTORS) {
        printf("No such motor, use a number [0, %d]\r\n", MAX_MOTORS);
        return;
    }

    if (index < 2) {
        printf("Motor %d is set at %d\r\n", motor_index, motor_disarmed[motor_index]);
        return;
    }

    if (motor_value < PWM_RANGE_MIN || motor_value > PWM_RANGE_MAX) {
        printf("Invalid motor value, 1000..2000\r\n");
        return;
    }

    printf("Setting motor %d to %d\r\n", motor_index, motor_value);
    motor_disarmed[motor_index] = motor_value;
}

static void cliProfile(char *cmdline)
{
    uint8_t len;
    int i;

    len = strlen(cmdline);
    if (len == 0) {
        printf("Current profile: %d\r\n", mcfg.current_profile);
        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i <= 2) {
            mcfg.current_profile = i;
            writeEEPROM();
            readEEPROM();
            cliProfile("");
        }
    }
}

static void cliReboot(void) {
    cliPrint("\r\nRebooting...");
    delay(10);
    systemReset(false);
}

static void cliSave(char *cmdline)
{
    cliPrint("Saving...");
    copyCurrentProfileToProfileSlot(mcfg.current_profile);
    writeEEPROM();
    cliReboot();
}

static void cliDefaults(char *cmdline)
{
    cliPrint("Resetting to defaults...");
    resetEEPROM();
    cliReboot();
}

static void cliPrint(const char *str)
{
    while (*str)
        serialWrite(serialPorts.mainport, *(str++));
}

static void cliWrite(uint8_t ch)
{
    serialWrite(serialPorts.mainport, ch);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    switch (var->type) {
        case VAR_UINT8:
            value = *(uint8_t *)var->ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)var->ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)var->ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)var->ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)var->ptr;
            break;

        case VAR_FLOAT:
            printf("%s", ftoa(*(float *)var->ptr, buf));
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
    switch (var->type) {
        case VAR_UINT8:
        case VAR_INT8:
            *(char *)var->ptr = (char)value.int_value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(short *)var->ptr = (short)value.int_value;
            break;

        case VAR_UINT32:
            *(int *)var->ptr = (int)value.int_value;
            break;

        case VAR_FLOAT:
            *(float *)var->ptr = (float)value.float_value;
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
        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0) {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max) { // here we compare the float value since... it should work, RIGHT?
                    int_float_value_t tmp;
                    if (valueTable[i].type == VAR_FLOAT)
                        tmp.float_value = valuef;
                    else
                        tmp.int_value = value;
                    cliSetVar(val, tmp);
                    printf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    cliPrint("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        cliPrint("ERR: Unknown variable name\r\n");
    } else {
        // no equals, check for matching variables.
        for (i = 0; i < VALUE_COUNT; i++) {
            if (strstr(valueTable[i].name, cmdline)) {
                val = &valueTable[i];
                printf("%s = ", valueTable[i].name);
                cliPrintVar(val, 0);
                printf("\r\n");
            }
        }
    }
}

static void cliStatus(char *cmdline)
{
    uint8_t i;
    uint32_t mask;

    printf("System Uptime: %d seconds, Voltage: %d * 0.1V (%dS battery)\r\n",
        millis() / 1000, vbat, batteryCellCount);
    mask = sensorsMask();

    printf("CPU %dMHz, detected sensors: ", (SystemCoreClock / 1000000));
    for (i = 0; ; i++) {
        if (sensorNames[i] == NULL)
            break;
        if (mask & (1 << i))
            printf("%s ", sensorNames[i]);
    }
    if (sensors(SENSOR_ACC)) {
        printf("ACCHW: %s", accNames[accHardware]);
        if (acc.revisionCode)
            printf(".%c", acc.revisionCode);
    }
    cliPrint("\r\n");

    printf("Cycle Time: %d, I2C Errors: %d, config size: %d\r\n", cycleTime, i2cGetErrorCounter(), sizeof(master_t));
}

static void cliVersion(char *cmdline)
{
    cliPrint("Afro32 CLI version 2.2 " __DATE__ " / " __TIME__);
}

void cliProcess(void)
{
    if (!cliMode) {
        cliMode = 1;
        cliPrint("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
        cliPrompt();
    }

    while (serialTotalBytesWaiting(serialPorts.mainport)) {
        uint8_t c = serialRead(serialPorts.mainport);
        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            int i = bufferIndex;
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
        } else if (!bufferIndex && c == 4) {
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
            cliBuffer[bufferIndex] = 0; // null terminate

            target.name = cliBuffer;
            target.param = NULL;

            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd)
                cmd->func(cliBuffer + strlen(cmd->name) + 1);
            else
                cliPrint("ERR: Unknown command, try 'help'");

            memset(cliBuffer, 0, sizeof(cliBuffer));
            bufferIndex = 0;

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
            if (!bufferIndex && c == 32)
                continue;
            cliBuffer[bufferIndex++] = c;
            cliWrite(c);
        }
    }
}
