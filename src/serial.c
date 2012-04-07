#include "board.h"
#include "mw.h"

// signal that we're in cli mode
uint8_t cliMode = 0;

void serialize16(int16_t a)
{
    uartWrite(a);
    uartWrite(a >> 8 & 0xff);
}

void serialize8(uint8_t a)
{
    uartWrite(a);
}

void serialInit(uint32_t baudrate)
{
    uartInit(baudrate);
}

void serialCom(void)
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
            for (i = 0; i < CHECKBOXITEMS; i++) {
                serialize8(cfg.activate1[i]);
                serialize8(cfg.activate2[i] | (rcOptions[i] << 7)); // use highest bit to transport state in mwc
            }
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
            for (i = 0; i < CHECKBOXITEMS; i++) {
                cfg.activate1[i] = uartReadPoll();
                cfg.activate2[i] = uartReadPoll();
            }
            uartReadPoll();     // power meter crap, removed
            uartReadPoll();     // power meter crap, removed
            writeParams();
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
