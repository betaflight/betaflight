#include "board.h"
#include "mw.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliAux(char *cmdline);
static void cliCMix(char *cmdline);
static void cliDefaults(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliProfile(char *cmdline);
static void cliSave(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);

// from sensors.c
extern uint8_t batteryCellCount;
extern uint8_t accHardware;

// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

static float _atof(const char *p);
static char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", "CUSTOM", NULL
};

// sync this with AvailableFeatures enum from board.h
const char * const featureNames[] = {
    "PPM", "VBAT", "INFLIGHT_ACC_CAL", "SPEKTRUM", "MOTOR_STOP",
    "SERVO_TILT", "GYRO_SMOOTHING", "LED_RING", "GPS",
    "FAILSAFE", "SONAR", "TELEMETRY", "POWERMETER", "VARIO",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char * const sensorNames[] = {
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

// 
const char * const accNames[] = {
    "", "ADXL345", "MPU6050", "MMA845x", NULL
};

typedef struct {
    char *name;
    char *param;
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
    { "help", "", cliHelp },
    { "map", "mapping of rc channel order", cliMap },
    { "mixer", "mixer name or list", cliMixer },
    { "profile", "index (0 to 2)", cliProfile },
    { "save", "save and reboot", cliSave },
    { "set", "name=value or blank or * for list", cliSet },
    { "status", "show system status", cliStatus },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

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
    { "midrc", VAR_UINT16, &mcfg.midrc, 1200, 1700 },
    { "minthrottle", VAR_UINT16, &mcfg.minthrottle, 0, 2000 },
    { "maxthrottle", VAR_UINT16, &mcfg.maxthrottle, 0, 2000 },
    { "mincommand", VAR_UINT16, &mcfg.mincommand, 0, 2000 },
    { "mincheck", VAR_UINT16, &mcfg.mincheck, 0, 2000 },
    { "maxcheck", VAR_UINT16, &mcfg.maxcheck, 0, 2000 },
    { "retarded_arm", VAR_UINT8, &mcfg.retarded_arm, 0, 1 },
    { "motor_pwm_rate", VAR_UINT16, &mcfg.motor_pwm_rate, 50, 498 },
    { "servo_pwm_rate", VAR_UINT16, &mcfg.servo_pwm_rate, 50, 498 },
    { "serial_baudrate", VAR_UINT32, &mcfg.serial_baudrate, 1200, 115200 },
    { "gps_baudrate", VAR_UINT32, &mcfg.gps_baudrate, 1200, 115200 },
    { "spektrum_hires", VAR_UINT8, &mcfg.spektrum_hires, 0, 1 },
    { "vbatscale", VAR_UINT8, &mcfg.vbatscale, 10, 200 },
    { "vbatmaxcellvoltage", VAR_UINT8, &mcfg.vbatmaxcellvoltage, 10, 50 },
    { "vbatmincellvoltage", VAR_UINT8, &mcfg.vbatmincellvoltage, 10, 50 },
    { "power_adc_channel", VAR_UINT8, &mcfg.power_adc_channel, 0, 9 },
    { "align_gyro_x", VAR_INT8, &mcfg.align[ALIGN_GYRO][0], -3, 3 },
    { "align_gyro_y", VAR_INT8, &mcfg.align[ALIGN_GYRO][1], -3, 3 },
    { "align_gyro_z", VAR_INT8, &mcfg.align[ALIGN_GYRO][2], -3, 3 },
    { "align_acc_x", VAR_INT8, &mcfg.align[ALIGN_ACCEL][0], -3, 3 },
    { "align_acc_y", VAR_INT8, &mcfg.align[ALIGN_ACCEL][1], -3, 3 },
    { "align_acc_z", VAR_INT8, &mcfg.align[ALIGN_ACCEL][2], -3, 3 },
    { "align_mag_x", VAR_INT8, &mcfg.align[ALIGN_MAG][0], -3, 3 },
    { "align_mag_y", VAR_INT8, &mcfg.align[ALIGN_MAG][1], -3, 3 },
    { "align_mag_z", VAR_INT8, &mcfg.align[ALIGN_MAG][2], -3, 3 },
    { "acc_hardware", VAR_UINT8, &mcfg.acc_hardware, 0, 3 },
    { "moron_threshold", VAR_UINT8, &mcfg.moron_threshold, 0, 128 },
    { "gyro_lpf", VAR_UINT16, &mcfg.gyro_lpf, 0, 256 },
    { "gyro_cmpf_factor", VAR_UINT16, &mcfg.gyro_cmpf_factor, 100, 1000 },
    { "gyro_cmpfm_factor", VAR_UINT16, &mcfg.gyro_cmpfm_factor, 100, 1000 },
    { "gps_type", VAR_UINT8, &mcfg.gps_type, 0, 3 },
    { "deadband", VAR_UINT8, &cfg.deadband, 0, 32 },
    { "yawdeadband", VAR_UINT8, &cfg.yawdeadband, 0, 100 },
    { "alt_hold_throttle_neutral", VAR_UINT8, &cfg.alt_hold_throttle_neutral, 1, 250 },
    { "rc_rate", VAR_UINT8, &cfg.rcRate8, 0, 250 },
    { "rc_expo", VAR_UINT8, &cfg.rcExpo8, 0, 100 },
    { "thr_mid", VAR_UINT8, &cfg.thrMid8, 0, 100 },
    { "thr_expo", VAR_UINT8, &cfg.thrExpo8, 0, 250 },
    { "roll_pitch_rate", VAR_UINT8, &cfg.rollPitchRate, 0, 100 },
    { "yawrate", VAR_UINT8, &cfg.yawRate, 0, 100 },
    { "failsafe_delay", VAR_UINT8, &cfg.failsafe_delay, 0, 200 },
    { "failsafe_off_delay", VAR_UINT8, &cfg.failsafe_off_delay, 0, 200 },
    { "failsafe_throttle", VAR_UINT16, &cfg.failsafe_throttle, 1000, 2000 },
    { "failsafe_detect_threshold", VAR_UINT16, &cfg.failsafe_detect_threshold, 100, 2000 },
    { "yaw_direction", VAR_INT8, &cfg.yaw_direction, -1, 1 },
    { "tri_yaw_middle", VAR_UINT16, &cfg.tri_yaw_middle, 0, 2000 },
    { "tri_yaw_min", VAR_UINT16, &cfg.tri_yaw_min, 0, 2000 },
    { "tri_yaw_max", VAR_UINT16, &cfg.tri_yaw_max, 0, 2000 },
    { "wing_left_min", VAR_UINT16, &cfg.wing_left_min, 0, 2000 },
    { "wing_left_mid", VAR_UINT16, &cfg.wing_left_mid, 0, 2000 },
    { "wing_left_max", VAR_UINT16, &cfg.wing_left_max, 0, 2000 },
    { "wing_right_min", VAR_UINT16, &cfg.wing_right_min, 0, 2000 },
    { "wing_right_mid", VAR_UINT16, &cfg.wing_right_mid, 0, 2000 },
    { "wing_right_max", VAR_UINT16, &cfg.wing_right_max, 0, 2000 },
    { "pitch_direction_l", VAR_INT8, &cfg.pitch_direction_l, -1, 1 },
    { "pitch_direction_r", VAR_INT8, &cfg.pitch_direction_r, -1, 1 },
    { "roll_direction_l", VAR_INT8, &cfg.roll_direction_l, -1, 1 },
    { "roll_direction_r", VAR_INT8, &cfg.roll_direction_r, -1, 1 },
    { "gimbal_flags", VAR_UINT8, &cfg.gimbal_flags, 0, 255},
    { "gimbal_pitch_gain", VAR_INT8, &cfg.gimbal_pitch_gain, -100, 100 },
    { "gimbal_roll_gain", VAR_INT8, &cfg.gimbal_roll_gain, -100, 100 },
    { "gimbal_pitch_min", VAR_UINT16, &cfg.gimbal_pitch_min, 100, 3000 },
    { "gimbal_pitch_max", VAR_UINT16, &cfg.gimbal_pitch_max, 100, 3000 },
    { "gimbal_pitch_mid", VAR_UINT16, &cfg.gimbal_pitch_mid, 100, 3000 },
    { "gimbal_roll_min", VAR_UINT16, &cfg.gimbal_roll_min, 100, 3000 },
    { "gimbal_roll_max", VAR_UINT16, &cfg.gimbal_roll_max, 100, 3000 },
    { "gimbal_roll_mid", VAR_UINT16, &cfg.gimbal_roll_mid, 100, 3000 },
    { "acc_lpf_factor", VAR_UINT8, &cfg.acc_lpf_factor, 0, 250 },
    { "acc_trim_pitch", VAR_INT16, &cfg.angleTrim[PITCH], -300, 300 },
    { "acc_trim_roll", VAR_INT16, &cfg.angleTrim[ROLL], -300, 300 },
    { "baro_tab_size", VAR_UINT8, &cfg.baro_tab_size, 0, BARO_TAB_SIZE_MAX },
    { "baro_noise_lpf", VAR_FLOAT, &cfg.baro_noise_lpf, 0, 1 },
    { "baro_cf", VAR_FLOAT, &cfg.baro_cf, 0, 1 },
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
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

static void cliSetVar(const clivalue_t *var, const int32_t value);
static void cliPrintVar(const clivalue_t *var, uint32_t full);

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
    double sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p) ) {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0;
    if (*p == '-') {
        sign = -1.0;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0;
    while (valid_digit(*p)) {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.') {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0;
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
        if (expon > 308) expon = 308;

        // Calculate scaling factor.
        while (expon >= 50) { scale *= 1E50; expon -= 50; }
        while (expon >=  8) { scale *= 1E8;  expon -=  8; }
        while (expon >   0) { scale *= 10.0; expon -=  1; }
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
    uartPrint("\r\n# ");
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
        for (i = 0; i < CHECKBOXITEMS; i++)
            printf("aux %u %u\r\n", i, cfg.activate[i]);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < CHECKBOXITEMS) {
            ptr = strchr(cmdline, ' ');
            val = atoi(ptr);
            cfg.activate[i] = val;
        } else {
            printf("Invalid Feature index: must be < %u\r\n", CHECKBOXITEMS);
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
        uartPrint("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
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
        uartPrint("Sanity check:\t");
        for (i = 0; i < 3; i++)
            uartPrint(fabs(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        uartPrint("\r\n");
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
                    uartPrint("Invalid mixer type...\r\n");
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
                uartPrint("Wrong number of arguments, needs idx thr roll pitch yaw\r\n");
            } else {
                cliCMix("");
            }
        } else {
            printf("Motor number must be between 1 and %d\r\n", MAX_MOTORS);
        }
    }
}

static void cliDefaults(char *cmdline)
{
    uartPrint("Resetting to defaults...\r\n");
    checkFirstTime(true);
    uartPrint("Rebooting...");
    delay(10);
    systemReset(false);
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
        buf[mcfg.rcmap[i]] = rcChannelLetters[i];
    buf[i] = '\0';
    printf("map %s\r\n", buf);

    // print settings
    for (i = 0; i < VALUE_COUNT; i++) {
        setval = &valueTable[i];
        printf("set %s = ", valueTable[i].name);
        cliPrintVar(setval, 0);
        uartPrint("\r\n");
    }
}

static void cliExit(char *cmdline)
{
    uartPrint("\r\nLeaving CLI mode...\r\n");
    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
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
        uartPrint("Enabled features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        uartPrint("Available features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            printf("%s ", featureNames[i]);
        }
        uartPrint("\r\n");
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
                uartPrint("Invalid feature name...\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0) {
                if (remove) {
                    featureClear(1 << i);
                    uartPrint("Disabled ");
                } else {
                    featureSet(1 << i);
                    uartPrint("Enabled ");
                }
                printf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

    uartPrint("Available commands:\r\n");    
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
            cmdline[i] = toupper(cmdline[i]);
        for (i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            uartPrint("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline);
    }
    uartPrint("Current assignment: ");
    for (i = 0; i < 8; i++)
        out[mcfg.rcmap[i]] = rcChannelLetters[i];
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
        uartPrint("Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            printf("%s ", mixerNames[i]);
        }
        uartPrint("\r\n");
        return;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            uartPrint("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            mcfg.mixerConfiguration = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
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
            writeEEPROM(0, false);
            cliProfile("");
        }
    }
}

static void cliSave(char *cmdline)
{
    uartPrint("Saving...");
    writeEEPROM(0, true);
    uartPrint("\r\nRebooting...");
    delay(10);
    systemReset(false);
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

static void cliSetVar(const clivalue_t *var, const int32_t value)
{
    switch (var->type) {
        case VAR_UINT8:
        case VAR_INT8:
            *(char *)var->ptr = (char)value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(short *)var->ptr = (short)value;
            break;

        case VAR_UINT32:
            *(int *)var->ptr = (int)value;
            break;

        case VAR_FLOAT:
            *(float *)var->ptr = *(float *)&value;
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
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            printf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            uartPrint("\r\n");
        }
    } else if ((eqptr = strstr(cmdline, "="))) {
        // has equal, set var
        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0) {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max) { // here we compare the float value since... it should work, RIGHT?
                    cliSetVar(val, valueTable[i].type == VAR_FLOAT ? *(uint32_t *)&valuef : value); // this is a silly dirty hack. please fix me later.
                    printf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    uartPrint("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        uartPrint("ERR: Unknown variable name\r\n");
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
        if (accHardware == ACC_MPU6050)
            printf(".%c", mcfg.mpu6050_scale ? 'o' : 'n');
    }
    uartPrint("\r\n");

    printf("Cycle Time: %d, I2C Errors: %d, config size: %d\r\n", cycleTime, i2cGetErrorCounter(), sizeof(master_t));
}

static void cliVersion(char *cmdline)
{
    uartPrint("Afro32 CLI version 2.1 " __DATE__ " / " __TIME__);
}

void cliProcess(void)
{
    if (!cliMode) {
        cliMode = 1;
        uartPrint("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
        cliPrompt();
    }

    while (uartAvailable()) {
        uint8_t c = uartRead();

        /* first step: translate "ESC[" -> "CSI" */
        if (c == '\033') {
            c = uartReadPoll();
            if (c == '[')
                c = 0x9b;
            else
                /* ignore unknown sequences */
                c = 0;
        }

        /* second step: translate known CSI sequence into singlebyte control sequences */
        if (c == 0x9b) {
            c = uartReadPoll();
            if (c == 'A')       //up
                c = 0x0b;
            else if (c == 'B')  //down
                c = 0x0a;
            else if (c == 'C')  //right
                c = 0x0c;
            else if (c == 'D')  //left
                c = 0x08;
            else if (c == 0x33 && uartReadPoll() == 0x7e)       //delete
                c = 0xff;       // nonstandard, borrowing 0xff for the delete key
            else
                c = 0;
        }

        /* from here on everything is a single byte */
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
                uartPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    uartPrint(cmd->name);
                    uartWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                uartWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {
            cliExit(cliBuffer);
            return;
        } else if (c == 0x15) {
            // ctrl+u == delete line
            uartPrint("\033[G\033[K# ");
            bufferIndex = 0;
            *cliBuffer = '\0';
        } else if (c == 0x0b) {
            //uartPrint("up unimplemented");
        } else if (c == 0x0a) {
            //uartPrint("down unimplemend");
        } else if (c == 0x08) {
            if (bufferIndex > 0) {
                bufferIndex--;
                uartPrint("\033[D");
            }
        } else if (c == 12) {
            if (cliBuffer[bufferIndex]) {
                bufferIndex++;
                uartPrint("\033[C");
            }
        } else if (c == 0xff) {
            // delete key
            if (cliBuffer[bufferIndex]) {
                int len = strlen(cliBuffer + bufferIndex);
                memmove(cliBuffer + bufferIndex, cliBuffer + bufferIndex + 1, len + 1);
                printf("%s \033[%dD", cliBuffer + bufferIndex, len);
            }
        } else if (*cliBuffer && (c == '\n' || c == '\r')) {
            // enter pressed
            clicmd_t *cmd = NULL;
            clicmd_t target;
            uartPrint("\r\n");

            target.name = cliBuffer;
            target.param = NULL;

            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd)
                cmd->func(cliBuffer + strlen(cmd->name) + 1);
            else
                uartPrint("ERR: Unknown command, try 'help'");

            *cliBuffer = '\0';
            bufferIndex = 0;

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;
            cliPrompt();
        } else if (c == 127) {
            // backspace
            if (bufferIndex && *cliBuffer) {
                int len = strlen(cliBuffer + bufferIndex);

                --bufferIndex;
                memmove(cliBuffer + bufferIndex, cliBuffer + bufferIndex + 1, len + 1);
                printf("\033[D%s \033[%dD", cliBuffer + bufferIndex, len + 1);
            }
        } else if (strlen(cliBuffer) + 1 < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            int len;

            if (!bufferIndex && c == 32)
                continue;

            len = strlen(cliBuffer + bufferIndex);

            memmove(cliBuffer + bufferIndex + 1, cliBuffer + bufferIndex, len + 1);
            cliBuffer[bufferIndex] = c;
            printf("%s \033[%dD", cliBuffer + bufferIndex, len + 1);
            ++bufferIndex;
        }
    }
}
