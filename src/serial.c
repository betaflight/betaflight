#include "board.h"
#include "mw.h"

#define MSP_IDENT                100   //out message         multitype + version
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t checksum, stateMSP, indRX, inBuf[64];

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

uint32_t read32(void)
{
    uint32_t t = inBuf[indRX++];
    t+= inBuf[indRX++] << 8;
    t+= (uint32_t)inBuf[indRX++] << 16;
    t+= (uint32_t)inBuf[indRX++] << 24;
    return t;
}

uint16_t read16(void)
{
    uint16_t t = inBuf[indRX++];
    t+= inBuf[indRX++] << 8;
    return t;
}

uint8_t read8(void)
{
    return inBuf[indRX++] & 0xff;
}

void headSerialReply(uint8_t c, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8('>');
    serialize8(s);
    serialize8(c);
    checksum = 0;
}

void tailSerialReply(void)
{
    serialize8(checksum);
    // no need to send
    // UartSendData();
}

// signal that we're in cli mode
uint8_t cliMode = 0;

void serialInit(uint32_t baudrate)
{
    uartInit(baudrate);
}

void serialCom(void)
{
    uint8_t i, c;
    static uint8_t offset, dataSize;

    // in cli mode, all uart stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }

    while (uartAvailable()) {
        c = uartRead();

        if (stateMSP > 99) {    // a message with a length indication, indicating a non null payload
            if (offset <= dataSize) {   // there are still some octets to read (including checksum) to complete a full message
                if (offset < dataSize)
                    checksum ^= c;      // the checksum is computed, except for the last octet
                inBuf[offset++] = c;
            } else {            // we have read all the payload
                if (checksum == inBuf[dataSize]) {      // we check is the computed checksum is ok
                    switch (stateMSP) { // if yes, then we execute different code depending on the message code. read8/16/32 will look into the inBuf buffer
                    case MSP_SET_RAW_RC:
                        for (i = 0; i < 8; i++) {
                            rcData[i] = read16();
                        }
                        break;
                    case MSP_SET_RAW_GPS:
                        GPS_fix = read8();
                        GPS_numSat = read8();
                        GPS_latitude = read32();
                        GPS_longitude = read32();
                        GPS_altitude = read16();
                        GPS_speed = read16();
                        GPS_update = 1;
                        break;
                    case MSP_SET_PID:
                        for (i = 0; i < PIDITEMS; i++) {
                            cfg.P8[i] = read8();
                            cfg.I8[i] = read8();
                            cfg.D8[i] = read8();
                        }
                        break;
                    case MSP_SET_BOX:
                        for (i = 0; i < CHECKBOXITEMS; i++) {
                            cfg.activate[i] = read16();
                        }
                        break;
                    case MSP_SET_RC_TUNING:
                        cfg.rcRate8 = read8();
                        cfg.rcExpo8 = read8();
                        cfg.rollPitchRate = read8();
                        cfg.yawRate = read8();
                        cfg.dynThrPID = read8();
                        cfg.thrMid8 = read8();
                        cfg.thrExpo8 = read8();
                        break;
                    case MSP_SET_MISC:
                        break;
                    }
                }
                stateMSP = 0;   // in any case we reset the MSP state
            }
        }

        if (stateMSP < 5) {
            if (stateMSP == 4) {        // this protocol state indicates we have a message with a lenght indication, and we read here the message code (fifth octet) 
                if (c > 99) {   // we check if it's a valid code (should be >99)
                    stateMSP = c;       // the message code is then reuse to feed the protocol state
                    offset = 0;
                    checksum = 0;
                    indRX = 0;  // and we init some values which will be used in the next loops to grasp the payload
                } else {
                    stateMSP = 0;       // the message code seems to be invalid. this should not happen => we reset the protocol state
                }
            }
            if (stateMSP == 3) {        // here, we need to check if the fourth octet indicates a code indication (>99) or a payload lenght indication (<100)
                if (c < 100) {  // a message with a length indication, indicating a non null payload
                    stateMSP++; // we update the protocol state to read the next octet
                    dataSize = c;       // we store the payload lenght
                    if (dataSize > 63)
                        dataSize = 63;  // in order to avoid overflow, we limit the size. this should not happen
                } else {
                    switch (c) {        // if we are here, the fourth octet indicates a code message
                    case MSP_IDENT:    // and we check message code to execute the relative code
                        headSerialReply(c, 2);  // we reply with an header indicating a payload lenght of 2 octets
                        serialize8(VERSION);    // the first octet. serialize8/16/32 is used also to compute a checksum
                        serialize8(cfg.mixerConfiguration);  // the second one
                        tailSerialReply();
                        break;  // mainly to send the last octet which is the checksum
                    case MSP_STATUS:
                        headSerialReply(c, 8);
                        serialize16(cycleTime);
                        serialize16(i2cGetErrorCounter());
                        serialize16(sensors(SENSOR_ACC) | sensors(SENSOR_BARO) << 1 | sensors(SENSOR_MAG) << 2 | sensors(SENSOR_GPS) << 3 | sensors(SENSOR_SONAR) << 4);
                        serialize16(accMode << BOXACC | baroMode << BOXBARO | magMode << BOXMAG | armed << BOXARM | GPSModeHome << BOXGPSHOME | GPSModeHold << BOXGPSHOLD | headFreeMode << BOXHEADFREE);
                        tailSerialReply();
                        break;
                    case MSP_RAW_IMU:
                        headSerialReply(c, 18);
                        for (i = 0; i < 3; i++)
                            serialize16(accSmooth[i]);
                        for (i = 0; i < 3; i++)
                            serialize16(gyroData[i]);
                        for (i = 0; i < 3; i++)
                            serialize16(magADC[i]);
                        tailSerialReply();
                        break;
                    case MSP_SERVO:
                        headSerialReply(c, 16);
                        for (i = 0; i < 8; i++)
                            serialize16(servo[i]);
                        tailSerialReply();
                        break;
                    case MSP_MOTOR:
                        headSerialReply(c, 16);
                        for (i = 0; i < 8; i++)
                            serialize16(motor[i]);
                        tailSerialReply();
                        break;
                    case MSP_RC:
                        headSerialReply(c, 16);
                        for (i = 0; i < 8; i++)
                            serialize16(rcData[i]);
                        tailSerialReply();
                        break;
                    case MSP_RAW_GPS:
                        headSerialReply(c, 14);
                        serialize8(GPS_fix);
                        serialize8(GPS_numSat);
                        serialize32(GPS_latitude);
                        serialize32(GPS_longitude);
                        serialize16(GPS_altitude);
                        serialize16(GPS_speed);
                        tailSerialReply();
                        break;
                    case MSP_COMP_GPS:
                        headSerialReply(c, 5);
                        serialize16(GPS_distanceToHome);
                        serialize16(GPS_directionToHome + 180);
                        serialize8(GPS_update);
                        tailSerialReply();
                        break;
                    case MSP_ATTITUDE:
                        headSerialReply(c, 6);
                        for (i = 0; i < 2; i++)
                            serialize16(angle[i]);
                        serialize16(heading);
                        tailSerialReply();
                        break;
                    case MSP_ALTITUDE:
                        headSerialReply(c, 4);
                        serialize32(EstAlt);
                        tailSerialReply();
                        break;
                    case MSP_BAT:
                        headSerialReply(c, 3);
                        serialize8(vbat);
                        serialize16(0);
                        tailSerialReply();
                        break;
                    case MSP_RC_TUNING:
                        headSerialReply(c, 7);
                        serialize8(cfg.rcRate8);
                        serialize8(cfg.rcExpo8);
                        serialize8(cfg.rollPitchRate);
                        serialize8(cfg.yawRate);
                        serialize8(cfg.dynThrPID);
                        serialize8(cfg.thrMid8);
                        serialize8(cfg.thrExpo8);
                        tailSerialReply();
                        break;
                    case MSP_PID:
                        headSerialReply(c, 3 * PIDITEMS);
                        for (i = 0; i < PIDITEMS; i++) {
                            serialize8(cfg.P8[i]);
                            serialize8(cfg.I8[i]);
                            serialize8(cfg.D8[i]);
                        }
                        tailSerialReply();
                        break;
                    case MSP_BOX:
                        headSerialReply(c, 2 * CHECKBOXITEMS);
                        for (i = 0; i < CHECKBOXITEMS; i++) {
                            serialize16(cfg.activate[i]);
                        }
                        tailSerialReply();
                        break;
                    case MSP_MISC:
                        headSerialReply(c, 2);
                        serialize16(0);
                        tailSerialReply();
                        break;
                    case MSP_RESET_CONF:
                        checkFirstTime(true);
                        break;
                    case MSP_ACC_CALIBRATION:
                        calibratingA = 400;
                        break;
                    case MSP_MAG_CALIBRATION:
                        calibratingM = 1;
                        break;
                    case MSP_EEPROM_WRITE:
                        writeParams(0);
                        break;
                    case MSP_DEBUG:
                        headSerialReply(c, 8);
                        serialize16(debug1);    // 4 variables are here for general monitoring purpose
                        serialize16(debug2);
                        serialize16(debug3);
                        serialize16(debug4);
                        tailSerialReply();
                        break;
                    }
                    stateMSP = 0;       // we reset the protocol state for the next loop
                }
            } else {
                switch (c) {    // header detection $MW<
                case '$':
                    if (stateMSP == 0)
                        stateMSP++;
                    break;      // first octet ok, no need to go further
                case 'M':
                    if (stateMSP == 1)
                        stateMSP++;
                    break;      // second octet ok, no need to go further
                case '<':
                    if (stateMSP == 2)
                        stateMSP++;
                    break;      // third octet ok, no need to go further
                }
            }
        }
        if (stateMSP == 0) {    // still compliant with older single octet command
            // enable CLI
            if (c == '#')
                cliProcess();
            else if (c == 'R')
                systemReset(true); // reboot to bootloader
        }
    }
}

#if 0
void oldSserialCom(void)
{
    uint8_t i;

    // in cli mode, all uart stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }

    if (uartAvailable()) {
        switch (uartRead()) {
        case '#':
            cliProcess();
            break;
#ifdef BTSERIAL
        case 'K':              // receive RC data from Bluetooth Serial adapter as a remote
            rcData[THROTTLE] = (SerialRead(0) * 4) + 1000;
            rcData[ROLL] = (SerialRead(0) * 4) + 1000;
            rcData[PITCH] = (SerialRead(0) * 4) + 1000;
            rcData[YAW] = (SerialRead(0) * 4) + 1000;
            rcData[AUX1] = (SerialRead(0) * 4) + 1000;
            break;
#endif
        case 'M':              // Multiwii @ arduino to GUI all data
            serialize8('M');
            serialize8(VERSION);        // MultiWii Firmware version
            for (i = 0; i < 3; i++)
                serialize16(accSmooth[i]);
            for (i = 0; i < 3; i++)
                serialize16(gyroData[i]);
            for (i = 0; i < 3; i++)
                serialize16(magADC[i]);
            serialize16(EstAlt / 10);
            serialize16(heading);       // compass
            for (i = 0; i < 8; i++)
                serialize16(servo[i]);
            for (i = 0; i < 8; i++)
                serialize16(motor[i]);
            for (i = 0; i < 8; i++)
                serialize16(rcData[i]);
            serialize8(sensors(SENSOR_ACC) << 1 | sensors(SENSOR_BARO) << 2 | sensors(SENSOR_MAG) << 3 | sensors(SENSOR_GPS) << 4);
            serialize8(accMode | baroMode << 1 | magMode << 2 | GPSModeHome << 3 | GPSModeHold << 4 | armed << 5);
#if defined(LOG_VALUES)
            serialize16(cycleTimeMax);
            cycleTimeMax = 0;
#else
            serialize16(cycleTime);
#endif
            serialize16(i2cGetErrorCounter());
            for (i = 0; i < 2; i++)
                serialize16(angle[i]);
            serialize8(cfg.mixerConfiguration);
            for (i = 0; i < PIDITEMS; i++) {
                serialize8(cfg.P8[i]);
                serialize8(cfg.I8[i]);
                serialize8(cfg.D8[i]);
            }
            serialize8(cfg.rcRate8);
            serialize8(cfg.rcExpo8);
            serialize8(cfg.rollPitchRate);
            serialize8(cfg.yawRate);
            serialize8(cfg.dynThrPID);
            for (i = 0; i < CHECKBOXITEMS; i++)
                serialize16(cfg.activate[i]);
            serialize16(GPS_distanceToHome);
            serialize16(GPS_directionToHome + 180);
            serialize8(GPS_numSat);
            serialize8(GPS_fix);
            serialize8(GPS_update);
            serialize16(0);                 // power meter, removed
            serialize16(0);                 // power meter, removed
            serialize8(vbat);
            serialize16(BaroAlt / 10);      // 4 variables are here for general monitoring purpose
            serialize16(debug2);            // debug2
            serialize16(debug3);            // debug3
            serialize16(debug4);            // debug4
            serialize8('M');
            break;
        case 'O':              // arduino to OSD data - contribution from MIS
            serialize8('O');
            for (i = 0; i < 3; i++)
                serialize16(accSmooth[i]);
            for (i = 0; i < 3; i++)
                serialize16(gyroData[i]);
            serialize16(EstAlt * 10.0f);
            serialize16(heading);       // compass - 16 bytes
            for (i = 0; i < 2; i++)
                serialize16(angle[i]);  //20
            for (i = 0; i < 6; i++)
                serialize16(motor[i]);  //32
            for (i = 0; i < 6; i++) {
                serialize16(rcData[i]);
            }                   //44
            serialize8(sensors(SENSOR_ACC) << 1 | sensors(SENSOR_BARO) << 2 | sensors(SENSOR_MAG) << 3 | sensors(SENSOR_GPS) << 4);
            serialize8(accMode | baroMode << 1 | magMode << 2 | GPSModeHome << 3 | GPSModeHold << 4 | armed << 5);
            serialize8(vbat);   // Vbatt 47
            serialize8(VERSION);        // MultiWii Firmware version
            serialize8(GPS_fix);        // Fix indicator for OSD
            serialize8(GPS_numSat);
            serialize16(GPS_latitude);
            serialize16(GPS_latitude >> 16);
            serialize16(GPS_longitude);
            serialize16(GPS_longitude >> 16);
            serialize16(GPS_altitude);
            serialize16(GPS_speed);            // Speed for OSD
            serialize8('O');    // NOT 49 anymore
            break;
        case 'R':               // reboot to bootloader (oops, apparently this w as used for other trash, fix later)
            systemReset(true);
            break;
        case 'W':              //GUI write params to eeprom @ arduino
            // while (uartAvailable() < (7 + 3 * PIDITEMS + 2 * CHECKBOXITEMS)) { }
            for (i = 0; i < PIDITEMS; i++) {
                cfg.P8[i] = uartReadPoll();
                cfg.I8[i] = uartReadPoll();
                cfg.D8[i] = uartReadPoll();
            }
            cfg.rcRate8 = uartReadPoll();
            cfg.rcExpo8 = uartReadPoll();    //2
            cfg.rollPitchRate = uartReadPoll();
            cfg.yawRate = uartReadPoll();    //4
            cfg.dynThrPID = uartReadPoll();  //5
            for (i = 0; i < CHECKBOXITEMS; i++)
                cfg.activate[i] = uartReadPoll();
            uartReadPoll();     // power meter crap, removed
            uartReadPoll();     // power meter crap, removed
            writeParams(0);
            break;
        case 'S':              // GUI to arduino ACC calibration request
            calibratingA = 400;
            break;
        case 'E':              // GUI to arduino MAG calibration request
            calibratingM = 1;
            break;
        }
    }
}
#endif
