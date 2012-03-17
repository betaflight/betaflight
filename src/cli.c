#include "board.h"
#include "mw.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliDefaults(char *cmdline);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMixer(char *cmdline);
static void cliSave(char *cmdline);
static void cliSet(char *cmdline);
static void cliVersion(char *cmdline);

// buffer
static char cliBuffer[32];
static uint8_t bufferIndex = 0;

// sync this with MultiType enum from mw.h
const char *mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", NULL
};

// sync this with AvailableFeatures enum from board.h
const char *featureNames[] = {
    "PPM", "VBAT", "INFLIGHT_ACC_CAL", "DIGITAL_SERVO", "MOTOR_STOP",
    "SERVO_TILT", "CAMTRIG", "GYRO_SMOOTHING", "LED_RING",
    NULL
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
    { "mixer", "mixer name or list", cliMixer },
    { "save", "save and reboot", cliSave },
    { "set", "name=value or blank for list", cliSet },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

typedef enum {
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16
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
    { "midrc", VAR_UINT16, &cfg.midrc, 1200, 1700 },
    { "minthrottle", VAR_UINT16, &cfg.minthrottle, 0, 2000 },
    { "maxthrottle", VAR_UINT16, &cfg.maxthrottle, 0, 2000 },
    { "mincommand", VAR_UINT16, &cfg.mincommand, 0, 2000 },
    { "yaw_direction", VAR_INT8, &cfg.yaw_direction, -1, 1 },
    { "wing_left_mid", VAR_UINT16, &cfg.wing_left_mid, 0, 2000 },
    { "wing_right_mid", VAR_UINT16, &cfg.wing_right_mid, 0, 2000 },
    { "tri_yaw_middle", VAR_UINT16, &cfg.tri_yaw_middle, 0, 2000 },
    { "tri_yaw_min", VAR_UINT16, &cfg.tri_yaw_min, 0, 2000 },
    { "tri_yaw_max", VAR_UINT16, &cfg.tri_yaw_max, 0, 2000 },
    { "tilt_pitch_prop", VAR_INT8, &cfg.tilt_pitch_prop, -100, 100 },
    { "tilt_roll_prop", VAR_INT8, &cfg.tilt_roll_prop, -100, 100 },
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

static void cliSetVar(const clivalue_t *var, const int32_t value);
static void cliPrintVar(const clivalue_t *var);

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
    uint8_t i;
    uint8_t len;
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
    uint8_t i = 0;

    uartPrint("Available commands:\r\n");    

    for (i = 0; i < CMD_COUNT; i++) {
    	uartPrint(cmdTable[i].name);
    	uartWrite(' ');
    	uartPrint(cmdTable[i].param);
    	uartPrint("\r\n");
    }
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
    writeParams();
    uartPrint("\r\nRebooting...");
    delay(10);
    systemReset(false);
}

static void cliPrintVar(const clivalue_t *var)
{
    int32_t value;
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
    }
    itoa(value, buf, 10);
    uartPrint(buf);
}

static void cliSetVar(const clivalue_t *var, const int32_t value)
{
    switch (var->type) {
        case VAR_UINT8:
            *(uint8_t *)var->ptr = (uint8_t)value;
            break;
            
        case VAR_INT8:
            *(int8_t *)var->ptr = (int8_t)value;
            break;
            
        case VAR_UINT16:
            *(uint16_t *)var->ptr = (uint16_t)value;
            break;
            
        case VAR_INT16:
            *(int16_t *)var->ptr = (int16_t)value;
            break;
    }
}

static void cliSet(char *cmdline)
{
    uint8_t i;
    uint8_t len;
    const clivalue_t *val;
    char *eqptr = NULL;
    int32_t value = 0;
    
    len = strlen(cmdline);

    if (len == 0) {
        uartPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            uartPrint((char *)valueTable[i].name);
            uartPrint(" = ");
            cliPrintVar(val);
            uartPrint("\r\n");
            delay(10);
        }
    } else if (eqptr = strstr(cmdline, "=")) {
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
                    cliPrintVar(val);
                } else {
                    uartPrint("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        uartPrint("ERR: Unknown variable name\r\n");
    }
}

static void cliVersion(char *cmdline)
{
    uartPrint("Afro32 CLI version 2.0-pre3");
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
                for (cmd = pstart;cmd <= pend;cmd++) {
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
