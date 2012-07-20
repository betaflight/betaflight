#include "board.h"
#include "mw.h"

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0
#define PLATFORM_32BIT           0x80000000

#define MSP_IDENT                100    //out message         multitype + version
#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         8 servos
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         1 altitude
#define MSP_BAT                  110    //out message         vbat, powermetersum
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113    //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114    //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203    //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
#define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_WP_SET               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250    //in message          no param

#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4

#define INBUF_SIZE 64

static const char boxnames[] =
    "ACC;"
    "BARO;"
    "MAG;"
    "CAMSTAB;"
    "CAMTRIG;"
    "ARM;"
    "GPS HOME;"
    "GPS HOLD;"
    "PASSTHRU;"
    "HEADFREE;"
    "BEEPER;"
    "LEDMAX;"
    "LLIGHTS;"
    "HEADADJ;";

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

static uint8_t checksum, indRX, inBuf[INBUF_SIZE];
static uint8_t cmdMSP;
static bool guiConnected = false;
// signal that we're in cli mode
uint8_t cliMode = 0;

void serialize32(uint32_t a)
{
    static uint8_t t;
    t = a;
    uartWrite(t);
    checksum ^= t;
    t = a >> 8;
    uartWrite(t);
    checksum ^= t;
    t = a >> 16;
    uartWrite(t);
    checksum ^= t;
    t = a >> 24;
    uartWrite(t);
    checksum ^= t;
}

void serialize16(int16_t a)
{
    static uint8_t t;
    t = a;
    uartWrite(t);
    checksum ^= t;
    t = a >> 8 & 0xff;
    uartWrite(t);
    checksum ^= t;
}

void serialize8(uint8_t a)
{
    uartWrite(a);
    checksum ^= a;
}

uint8_t read8(void)
{
    return inBuf[indRX++] & 0xff;
}

uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t) read8() << 8;
    return t;
}

uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t) read16() << 16;
    return t;
}

void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(cmdMSP);
}

void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

void tailSerialReply(void)
{
    serialize8(checksum);
}

void serializeNames(const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(*c);
}

void serialInit(uint32_t baudrate)
{
    uartInit(baudrate);
}

static void evaluateCommand(void)
{
    uint32_t i;
    uint8_t wp_no;

    switch (cmdMSP) {
    case MSP_SET_RAW_RC:
        for (i = 0; i < 8; i++)
            rcData[i] = read16();
        headSerialReply(0);
        break;
    case MSP_SET_RAW_GPS:
        f.GPS_FIX = read8();
        GPS_numSat = read8();
        GPS_coord[LAT] = read32();
        GPS_coord[LON] = read32();
        GPS_altitude = read16();
        GPS_speed = read16();
        GPS_update |= 2;        // New data signalisation to GPS functions
        headSerialReply(0);
        break;
    case MSP_SET_PID:
        for (i = 0; i < PIDITEMS; i++) {
            cfg.P8[i] = read8();
            cfg.I8[i] = read8();
            cfg.D8[i] = read8();
        }
        headSerialReply(0);
        break;
    case MSP_SET_BOX:
        for (i = 0; i < CHECKBOXITEMS; i++)
            cfg.activate[i] = read16();
        headSerialReply(0);
        break;
    case MSP_SET_RC_TUNING:
        cfg.rcRate8 = read8();
        cfg.rcExpo8 = read8();
        cfg.rollPitchRate = read8();
        cfg.yawRate = read8();
        cfg.dynThrPID = read8();
        cfg.thrMid8 = read8();
        cfg.thrExpo8 = read8();
        headSerialReply(0);
        break;
    case MSP_SET_MISC:
        headSerialReply(0);
        break;
    case MSP_IDENT:
        headSerialReply(7);
        serialize8(VERSION);                // multiwii version
        serialize8(cfg.mixerConfiguration); // type of multicopter
        serialize8(MSP_VERSION);            // MultiWii Serial Protocol Version
        serialize32(PLATFORM_32BIT);        // "capability"
        break;
    case MSP_STATUS:
        headSerialReply(10);
        serialize16(cycleTime);
        serialize16(i2cGetErrorCounter());
        serialize16(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
        serialize32(f.ACC_MODE << BOXACC | f.BARO_MODE << BOXBARO | f.MAG_MODE << BOXMAG | f.ARMED << BOXARM | rcOptions[BOXCAMSTAB] << BOXCAMSTAB | rcOptions[BOXCAMTRIG] << BOXCAMTRIG | 
                    f.GPS_HOME_MODE << BOXGPSHOME | f.GPS_HOLD_MODE << BOXGPSHOLD | f.HEADFREE_MODE << BOXHEADFREE | f.PASSTHRU_MODE << BOXPASSTHRU | 
                    rcOptions[BOXBEEPERON] << BOXBEEPERON | rcOptions[BOXLEDMAX] << BOXLEDMAX | rcOptions[BOXLLIGHTS] << BOXLLIGHTS | rcOptions[BOXHEADADJ] << BOXHEADADJ);
        break;
    case MSP_RAW_IMU:
        headSerialReply(18);
        for (i = 0; i < 3; i++)
            serialize16(accSmooth[i]);
        for (i = 0; i < 3; i++)
            serialize16(gyroData[i]);
        for (i = 0; i < 3; i++)
            serialize16(magADC[i]);
        break;
    case MSP_SERVO:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
            serialize16(servo[i]);
        break;
    case MSP_MOTOR:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
            serialize16(motor[i]);
        break;
    case MSP_RC:
        headSerialReply(16);
        for (i = 0; i < 8; i++)
            serialize16(rcData[i]);
        break;
    case MSP_RAW_GPS:
        headSerialReply(14);
        serialize8(f.GPS_FIX);
        serialize8(GPS_numSat);
        serialize32(GPS_coord[LAT]);
        serialize32(GPS_coord[LON]);
        serialize16(GPS_altitude);
        serialize16(GPS_speed);
        break;
    case MSP_COMP_GPS:
        headSerialReply(5);
        serialize16(GPS_distanceToHome);
        serialize16(GPS_directionToHome);
        serialize8(GPS_update & 1);
        break;
    case MSP_ATTITUDE:
        headSerialReply(8);
        for (i = 0; i < 2; i++)
            serialize16(angle[i]);
        serialize16(heading);
        serialize16(headFreeModeHold);
        break;
    case MSP_ALTITUDE:
        headSerialReply(4);
        serialize32(EstAlt);
        break;
    case MSP_BAT:
        headSerialReply(3);
        serialize8(vbat);
        serialize16(0); // power meter trash
        break;
    case MSP_RC_TUNING:
        headSerialReply(7);
        serialize8(cfg.rcRate8);
        serialize8(cfg.rcExpo8);
        serialize8(cfg.rollPitchRate);
        serialize8(cfg.yawRate);
        serialize8(cfg.dynThrPID);
        serialize8(cfg.thrMid8);
        serialize8(cfg.thrExpo8);
        break;
    case MSP_PID:
        headSerialReply(3 * PIDITEMS);
        for (i = 0; i < PIDITEMS; i++) {
            serialize8(cfg.P8[i]);
            serialize8(cfg.I8[i]);
            serialize8(cfg.D8[i]);
        }
        break;
    case MSP_BOX:
        headSerialReply(2 * CHECKBOXITEMS);
        for (i = 0; i < CHECKBOXITEMS; i++)
            serialize16(cfg.activate[i]);
        break;
    case MSP_BOXNAMES:
        headSerialReply(sizeof(boxnames) - 1);
        serializeNames(boxnames);
        break;
    case MSP_PIDNAMES:
        headSerialReply(sizeof(pidnames) - 1);
        serializeNames(pidnames);
        break;
    case MSP_MISC:
        headSerialReply(2);
        serialize16(0); // intPowerTrigger1
        break;
    case MSP_MOTOR_PINS:
        headSerialReply(8);
        for (i = 0; i < 8; i++)
            serialize8(i + 1);
        break;
    case MSP_WP:
        wp_no = read8();    // get the wp number
        headSerialReply(12);
        if (wp_no == 0) {
            serialize8(0);                   // wp0
            serialize32(GPS_home[LAT]);
            serialize32(GPS_home[LON]);
            serialize16(0);                  // altitude will come here
            serialize8(0);                   // nav flag will come here
        } else if (wp_no == 16) {
            serialize8(16);                  // wp16
            serialize32(GPS_hold[LAT]);
            serialize32(GPS_hold[LON]);
            serialize16(0);                  // altitude will come here
            serialize8(0);                   // nav flag will come here
        }
        break;
    case MSP_RESET_CONF:
        checkFirstTime(true);
        headSerialReply(0);
        break;
    case MSP_ACC_CALIBRATION:
        calibratingA = 400;
        headSerialReply(0);
        break;
    case MSP_MAG_CALIBRATION:
        f.CALIBRATE_MAG = 1;
        headSerialReply(0);
        break;
    case MSP_EEPROM_WRITE:
        writeParams(0);
        headSerialReply(0);
        break;
    case MSP_DEBUG:
        headSerialReply(8);
        for (i = 0; i < 4; i++)
            serialize16(debug[i]);      // 4 variables are here for general monitoring purpose
        break;
    default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
        headSerialError(0);
        break;
    }
    tailSerialReply();
}

// evaluate all other incoming serial data
static void evaluateOtherData(uint8_t sr)
{
    switch (sr) {
        case '#':
            cliProcess();
            break;
        case 'R':
            systemReset(true);      // reboot to bootloader
            break;
    }
}

void serialCom(void)
{
    uint8_t c;
    static uint8_t offset;
    static uint8_t dataSize;
    static enum _serial_state {
        IDLE,
        HEADER_START,
        HEADER_M,
        HEADER_ARROW,
        HEADER_SIZE,
        HEADER_CMD,
    } c_state = IDLE;

    // in cli mode, all uart stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }

    while (uartAvailable()) {
        c = uartRead();

        if (c_state == IDLE) {
            c_state = (c == '$') ? HEADER_START : IDLE;
            if (c_state == IDLE)
                evaluateOtherData(c); // evaluate all other incoming serial data
        } else if (c_state == HEADER_START) {
            c_state = (c == 'M') ? HEADER_M : IDLE;
        } else if (c_state == HEADER_M) {
            c_state = (c == '<') ? HEADER_ARROW : IDLE;
        } else if (c_state == HEADER_ARROW) {
            if (c > INBUF_SIZE) {       // now we are expecting the payload size
                c_state = IDLE;
                continue;
            }
            dataSize = c;
            offset = 0;
            checksum = 0;
            indRX = 0;
            checksum ^= c;
            c_state = HEADER_SIZE;      // the command is to follow
            guiConnected = true;
        } else if (c_state == HEADER_SIZE) {
            cmdMSP = c;
            checksum ^= c;
            c_state = HEADER_CMD;
        } else if (c_state == HEADER_CMD && offset < dataSize) {
            checksum ^= c;
            inBuf[offset++] = c;
        } else if (c_state == HEADER_CMD && offset >= dataSize) {
            if (checksum == c) {        // compare calculated and transferred checksum
                evaluateCommand();      // we got a valid packet, evaluate it
            }
            c_state = IDLE;
        }
    }
    if (!cliMode && !uartAvailable() && feature(FEATURE_TELEMETRY) && f.ARMED) { // The first 2 conditions should never evaluate to true but I'm putting it here anyway - silpstream
        sendTelemetry();
        return;
    }
}
