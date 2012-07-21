#include "board.h"
#include "mw.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliDefaults(char *cmdline);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
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
static char cliBuffer[32];
static uint32_t bufferIndex = 0;

// sync this with MultiType enum from mw.h
const char *mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", NULL
};

// sync this with AvailableFeatures enum from board.h
const char *featureNames[] = {
    "PPM", "VBAT", "INFLIGHT_ACC_CAL", "SPEKTRUM", "MOTOR_STOP",
    "SERVO_TILT", "GYRO_SMOOTHING", "LED_RING", "GPS",
    "FAILSAFE", "SONAR", "TELEMETRY",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char *sensorNames[] = {
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

// 
const char *accNames[] = {
    "", "ADXL345", "MPU6050", "MMA845x", NULL
};

typedef struct {
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    { "defaults", "reset to defaults and reboot", cliDefaults },
    { "exit", "", cliExit },
    { "feature", "list or -val or val", cliFeature },
    { "help", "", cliHelp },
    { "map", "mapping of rc channel order", cliMap },
    { "mixer", "mixer name or list", cliMixer },
    { "save", "save and reboot", cliSave },
    { "set", "name=value or blank for list", cliSet },
    { "status", "show system status", cliStatus },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

typedef enum {
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32
} vartype_e;

typedef struct {
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
} clivalue_t;

const clivalue_t valueTable[] = {
    { "deadband", VAR_UINT8, &cfg.deadband, 0, 32 },
    { "yawdeadband", VAR_UINT8, &cfg.yawdeadband, 0, 100 },
    { "alt_hold_throttle_neutral", VAR_UINT8, &cfg.alt_hold_throttle_neutral, 1, 250 },
    { "midrc", VAR_UINT16, &cfg.midrc, 1200, 1700 },
    { "minthrottle", VAR_UINT16, &cfg.minthrottle, 0, 2000 },
    { "maxthrottle", VAR_UINT16, &cfg.maxthrottle, 0, 2000 },
    { "mincommand", VAR_UINT16, &cfg.mincommand, 0, 2000 },
    { "mincheck", VAR_UINT16, &cfg.mincheck, 0, 2000 },
    { "maxcheck", VAR_UINT16, &cfg.maxcheck, 0, 2000 },
    { "retarded_arm", VAR_UINT8, &cfg.retarded_arm, 0, 1 },
    { "failsafe_delay", VAR_UINT8, &cfg.failsafe_delay, 0, 200 },
    { "failsafe_off_delay", VAR_UINT8, &cfg.failsafe_off_delay, 0, 200 },
    { "failsafe_throttle", VAR_UINT16, &cfg.failsafe_throttle, 1000, 2000 },
    { "motor_pwm_rate", VAR_UINT16, &cfg.motor_pwm_rate, 50, 498 },
    { "servo_pwm_rate", VAR_UINT16, &cfg.servo_pwm_rate, 50, 498 },
    { "serial_baudrate", VAR_UINT32, &cfg.serial_baudrate, 1200, 115200 },
    { "gps_baudrate", VAR_UINT32, &cfg.gps_baudrate, 1200, 115200 },
    { "spektrum_hires", VAR_UINT8, &cfg.spektrum_hires, 0, 1 },
    { "vbatscale", VAR_UINT8, &cfg.vbatscale, 10, 200 },
    { "vbatmaxcellvoltage", VAR_UINT8, &cfg.vbatmaxcellvoltage, 10, 50 },
    { "vbatmincellvoltage", VAR_UINT8, &cfg.vbatmincellvoltage, 10, 50 },
    { "yaw_direction", VAR_INT8, &cfg.yaw_direction, -1, 1 },
    { "tri_yaw_middle", VAR_UINT16, &cfg.tri_yaw_middle, 0, 2000 },
    { "tri_yaw_min", VAR_UINT16, &cfg.tri_yaw_min, 0, 2000 },
    { "tri_yaw_max", VAR_UINT16, &cfg.tri_yaw_max, 0, 2000 },
    { "gimbal_flags", VAR_UINT8, &cfg.gimbal_flags, 0, 255},
    { "gimbal_pitch_gain", VAR_INT8, &cfg.gimbal_pitch_gain, -100, 100 },
    { "gimbal_roll_gain", VAR_INT8, &cfg.gimbal_roll_gain, -100, 100 },
    { "gimbal_pitch_min", VAR_UINT16, &cfg.gimbal_pitch_min, 100, 3000 },
    { "gimbal_pitch_max", VAR_UINT16, &cfg.gimbal_pitch_max, 100, 3000 },
    { "gimbal_pitch_mid", VAR_UINT16, &cfg.gimbal_pitch_mid, 100, 3000 },
    { "gimbal_roll_min", VAR_UINT16, &cfg.gimbal_roll_min, 100, 3000 },
    { "gimbal_roll_max", VAR_UINT16, &cfg.gimbal_roll_max, 100, 3000 },
    { "gimbal_roll_mid", VAR_UINT16, &cfg.gimbal_roll_mid, 100, 3000 },
    { "acc_hardware", VAR_UINT8, &cfg.acc_hardware, 0, 3 },
    { "acc_lpf_factor", VAR_UINT8, &cfg.acc_lpf_factor, 0, 250 },
    { "gyro_lpf", VAR_UINT16, &cfg.gyro_lpf, 0, 256 },
    { "gyro_cmpf_factor", VAR_UINT16, &cfg.gyro_cmpf_factor, 100, 1000 },
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
    { "looptime", VAR_UINT16, &cfg.looptime, 0, 9000 },
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

static void cliPrompt(void)
{
    uartPrint("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static void cliDefaults(char *cmdline)
{
    uartPrint("Resetting to defaults...\r\n");
    checkFirstTime(true);
    uartPrint("Rebooting...");
    delay(10);
    systemReset(false);
}

static void cliExit(char *cmdline)
{
    uartPrint("\r\nLeaving CLI mode...\r\n");
    memset(cliBuffer, 0, sizeof(cliBuffer));
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
                uartPrint((char *)featureNames[i]);
            uartWrite(' ');
        }
        uartPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        uartPrint("Available features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            uartPrint((char *)featureNames[i]);
            uartWrite(' ');
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
                uartPrint((char *)featureNames[i]);
                uartPrint("\r\n");
                break;
            }
        }
    }
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

    uartPrint("Available commands:\r\n");    

    for (i = 0; i < CMD_COUNT; i++) {
        uartPrint(cmdTable[i].name);
        uartWrite('\t');
        uartPrint(cmdTable[i].param);
        uartPrint("\r\n");
        while (!uartTransmitEmpty());
    }
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
        out[cfg.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    uartPrint(out);
    uartPrint("\r\n");
}

static void cliMixer(char *cmdline)
{
    uint8_t i;
    uint8_t len;
    
    len = strlen(cmdline);

    if (len == 0) {
        uartPrint("Current mixer: ");
        uartPrint((char *)mixerNames[cfg.mixerConfiguration - 1]);
        uartPrint("\r\n");
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        uartPrint("Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            uartPrint((char *)mixerNames[i]);
            uartWrite(' ');
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
            cfg.mixerConfiguration = i + 1;
            uartPrint("Mixer set to ");
            uartPrint((char *)mixerNames[i]);
            uartPrint("\r\n");
            break;
        }
    }
}

static void cliSave(char *cmdline)
{
    uartPrint("Saving...");
    writeParams(0);
    uartPrint("\r\nRebooting...");
    delay(10);
    systemReset(false);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[16];

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
    }
    itoa(value, buf, 10);
    uartPrint(buf);
    if (full) {
        uartPrint(" ");
        itoa(var->min, buf, 10);
        uartPrint(buf);
        uartPrint(" ");
        itoa(var->max, buf, 10);
        uartPrint(buf);
    }
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
    }
}

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;
    int32_t value = 0;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            uartPrint((char *)valueTable[i].name);
            uartPrint(" = ");
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            uartPrint("\r\n");
            while (!uartTransmitEmpty());
        }
    } else if ((eqptr = strstr(cmdline, "="))) {
        // has equal, set var
        eqptr++;
        len--;
        value = atoi(eqptr);
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0) {
                // found
                if (value >= valueTable[i].min && value <= valueTable[i].max) {
                    cliSetVar(val, value);
                    uartPrint((char *)valueTable[i].name);
                    uartPrint(" set to ");
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
    char buf[16];
    uint8_t i;
    uint32_t mask;

    uartPrint("System Uptime: ");
    itoa(millis() / 1000, buf, 10);
    uartPrint(buf);
    uartPrint(" seconds, Voltage: ");
    itoa(vbat, buf, 10);
    uartPrint(buf);
    uartPrint(" * 0.1V (");
    itoa(batteryCellCount, buf, 10);
    uartPrint(buf);
    uartPrint("S battery)\r\n");

    mask = sensorsMask();

    uartPrint("Detected sensors: ");
    for (i = 0; ; i++) {
        if (sensorNames[i] == NULL)
            break;
        if (mask & (1 << i))
            uartPrint((char *)sensorNames[i]);
        uartWrite(' ');
    }
    if (sensors(SENSOR_ACC)) {
        uartPrint("ACCHW: ");
        uartPrint((char *)accNames[accHardware]);
    }
    uartPrint("\r\n");

    uartPrint("Cycle Time: ");
    itoa(cycleTime, buf, 10);
    uartPrint(buf);
    uartPrint(", I2C Errors: ");
    itoa(i2cGetErrorCounter(), buf, 10);
    uartPrint(buf);
    uartPrint("\r\n");
}

static void cliVersion(char *cmdline)
{
    uartPrint("Afro32 CLI version 2.0 " __DATE__ " / " __TIME__);
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
                    if (!pstart->name[bufferIndex]) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
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
        } else if (c == 12) {
            // clear screen
            uartPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            clicmd_t *cmd = NULL;
            clicmd_t target;
            uartPrint("\r\n");
            cliBuffer[bufferIndex] = 0; // null terminate
            
            target.name = cliBuffer;
            target.param = NULL;
            
            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd)
                cmd->func(cliBuffer + strlen(cmd->name) + 1);
            else
                uartPrint("ERR: Unknown command, try 'help'");

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
                uartPrint("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == 32)
                continue;
            cliBuffer[bufferIndex++] = c;
            uartWrite(c);
        }
    }
}
