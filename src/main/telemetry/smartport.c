/*
 * SmartPort Telemetry implementation by frank26080115
 * see https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#if defined(TELEMETRY) && defined(TELEMETRY_SMARTPORT)

#include "common/axis.h"
#include "common/color.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/serial.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/gps.h"
#include "io/serial.h"
#include "io/ledstrip.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"

#include "flight/imu.h"
#include "flight/navigation_rewrite.h"

#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"

#include "config/config.h"
#include "config/feature.h"

enum
{
    SPSTATE_UNINITIALIZED,
    SPSTATE_INITIALIZED,
    SPSTATE_WORKING,
    SPSTATE_TIMEDOUT,
};

enum
{
    FSSP_START_STOP = 0x7E,
    FSSP_DATA_FRAME = 0x10,

    // ID of sensor. Must be something that is polled by FrSky RX
    FSSP_SENSOR_ID1 = 0x1B,
    FSSP_SENSOR_ID2 = 0x0D,
    FSSP_SENSOR_ID3 = 0x34,
    FSSP_SENSOR_ID4 = 0x67,
    // there are 32 ID's polled by smartport master
    // remaining 3 bits are crc (according to comments in openTx code)
};

// these data identifiers are obtained from https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky.h
enum
{
    FSSP_DATAID_SPEED      = 0x0830 ,
    FSSP_DATAID_VFAS       = 0x0210 ,
    FSSP_DATAID_CURRENT    = 0x0200 ,
    FSSP_DATAID_RPM        = 0x050F ,
    FSSP_DATAID_ALTITUDE   = 0x0100 ,
    FSSP_DATAID_FUEL       = 0x0600 ,
    FSSP_DATAID_ADC1       = 0xF102 ,
    FSSP_DATAID_ADC2       = 0xF103 ,
    FSSP_DATAID_LATLONG    = 0x0800 ,
    FSSP_DATAID_CAP_USED   = 0x0600 ,
    FSSP_DATAID_VARIO      = 0x0110 ,
    FSSP_DATAID_CELLS      = 0x0300 ,
    FSSP_DATAID_CELLS_LAST = 0x030F ,
    FSSP_DATAID_HEADING    = 0x0840 ,
    FSSP_DATAID_ACCX       = 0x0700 ,
    FSSP_DATAID_ACCY       = 0x0710 ,
    FSSP_DATAID_ACCZ       = 0x0720 ,
    FSSP_DATAID_T1         = 0x0400 ,
    FSSP_DATAID_T2         = 0x0410 ,
    FSSP_DATAID_GPS_ALT    = 0x0820 ,
    FSSP_DATAID_ASPD       = 0x0A00 ,
    FSSP_DATAID_A3         = 0x0900 ,
    FSSP_DATAID_A4         = 0x0910 ,
};

const uint16_t frSkyDataIdTable[] = {
    FSSP_DATAID_SPEED     ,
    FSSP_DATAID_VFAS      ,
    FSSP_DATAID_CURRENT   ,
    //FSSP_DATAID_RPM       ,
    FSSP_DATAID_ALTITUDE  ,
    FSSP_DATAID_FUEL      ,
    //FSSP_DATAID_ADC1      ,
    //FSSP_DATAID_ADC2      ,
    FSSP_DATAID_LATLONG   ,
    FSSP_DATAID_LATLONG   , // twice
    //FSSP_DATAID_CAP_USED  ,
    FSSP_DATAID_VARIO     ,
    //FSSP_DATAID_CELLS     ,
    //FSSP_DATAID_CELLS_LAST,
    FSSP_DATAID_HEADING   ,
    FSSP_DATAID_ACCX      ,
    FSSP_DATAID_ACCY      ,
    FSSP_DATAID_ACCZ      ,
    FSSP_DATAID_T1        ,
    FSSP_DATAID_T2        ,
    FSSP_DATAID_GPS_ALT   ,
    FSSP_DATAID_ASPD      ,
    FSSP_DATAID_A4        ,
    0
};

#define __USE_C99_MATH // for roundf()
#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX
#define SMARTPORT_SERVICE_TIMEOUT_MS 1 // max allowed time to find a value to send
#define SMARTPORT_NOT_CONNECTED_TIMEOUT_MS 7000

static serialPort_t *smartPortSerialPort = NULL; // The 'SmartPort'(tm) Port.
static serialPortConfig_t *portConfig;

static telemetryConfig_t *telemetryConfig;
static bool smartPortTelemetryEnabled =  false;
static portSharing_e smartPortPortSharing;

char smartPortState = SPSTATE_UNINITIALIZED;
static uint8_t smartPortHasRequest = 0;
static uint8_t smartPortIdCnt = 0;
static uint32_t smartPortLastRequestTime = 0;

static void smartPortDataReceive(uint16_t c)
{
    uint32_t now = millis();

    // look for a valid request sequence
    static uint8_t lastChar;
    if (lastChar == FSSP_START_STOP) {
        smartPortState = SPSTATE_WORKING;
        if (c == FSSP_SENSOR_ID1 && (serialRxBytesWaiting(smartPortSerialPort) == 0)) {
            smartPortLastRequestTime = now;
            smartPortHasRequest = 1;
            // we only responde to these IDs
            // the X4R-SB does send other IDs, we ignore them, but take note of the time
        }
    }
    lastChar = c;
}

static void smartPortSendByte(uint8_t c, uint16_t *crcp)
{
    // smart port escape sequence
    if (c == 0x7D || c == 0x7E) {
        serialWrite(smartPortSerialPort, 0x7D);
        c ^= 0x20;
    }

    serialWrite(smartPortSerialPort, c);

    if (crcp == NULL)
        return;

    uint16_t crc = *crcp;
    crc += c;
    crc += crc >> 8;
    crc &= 0x00FF;
    *crcp = crc;
}

static void smartPortSendPackage(uint16_t id, uint32_t val)
{
    uint16_t crc = 0;
    smartPortSendByte(FSSP_DATA_FRAME, &crc);
    uint8_t *u8p = (uint8_t*)&id;
    smartPortSendByte(u8p[0], &crc);
    smartPortSendByte(u8p[1], &crc);
    u8p = (uint8_t*)&val;
    smartPortSendByte(u8p[0], &crc);
    smartPortSendByte(u8p[1], &crc);
    smartPortSendByte(u8p[2], &crc);
    smartPortSendByte(u8p[3], &crc);
    smartPortSendByte(0xFF - (uint8_t)crc, NULL);
}

void initSmartPortTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_SMARTPORT);
    smartPortPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_SMARTPORT);
}

void freeSmartPortTelemetryPort(void)
{
    closeSerialPort(smartPortSerialPort);
    smartPortSerialPort = NULL;

    smartPortState = SPSTATE_UNINITIALIZED;
    smartPortTelemetryEnabled = false;
}

void configureSmartPortTelemetryPort(void)
{
    portOptions_t portOptions;

    if (!portConfig) {
        return;
    }

    if (telemetryConfig->smartportUartUnidirectional) {
        portOptions = SERIAL_UNIDIR;
    } else {
        portOptions = SERIAL_BIDIR;
    }

    if (telemetryConfig->telemetry_inversion) {
        portOptions |= SERIAL_INVERTED;
    }

    smartPortSerialPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_SMARTPORT, NULL, SMARTPORT_BAUD, SMARTPORT_UART_MODE, portOptions);

    if (!smartPortSerialPort) {
        return;
    }

    smartPortState = SPSTATE_INITIALIZED;
    smartPortTelemetryEnabled = true;
    smartPortLastRequestTime = millis();
}

bool canSendSmartPortTelemetry(void)
{
    return smartPortSerialPort && (smartPortState == SPSTATE_INITIALIZED || smartPortState == SPSTATE_WORKING);
}

bool isSmartPortTimedOut(void)
{
    return smartPortState >= SPSTATE_TIMEDOUT;
}

void checkSmartPortTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(smartPortPortSharing);

    if (newTelemetryEnabledValue == smartPortTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureSmartPortTelemetryPort();
    else
        freeSmartPortTelemetryPort();
}

void handleSmartPortTelemetry(void)
{
    uint32_t smartPortLastServiceTime = millis();

    if (!smartPortTelemetryEnabled) {
        return;
    }

    if (!canSendSmartPortTelemetry()) {
        return;
    }

    while (serialRxBytesWaiting(smartPortSerialPort) > 0) {
        uint8_t c = serialRead(smartPortSerialPort);
        smartPortDataReceive(c);
    }

    uint32_t now = millis();

    // if timed out, reconfigure the UART back to normal so the GUI or CLI works
    if ((now - smartPortLastRequestTime) > SMARTPORT_NOT_CONNECTED_TIMEOUT_MS) {
        smartPortState = SPSTATE_TIMEDOUT;
        return;
    }

    while (smartPortHasRequest) {
        // Ensure we won't get stuck in the loop if there happens to be nothing available to send in a timely manner - dump the slot if we loop in there for too long.
        if ((millis() - smartPortLastServiceTime) > SMARTPORT_SERVICE_TIMEOUT_MS) {
            smartPortHasRequest = 0;
            return;
        }

        // we can send back any data we want, our table keeps track of the order and frequency of each data type we send
        uint16_t id = frSkyDataIdTable[smartPortIdCnt];
        if (id == 0) { // end of table reached, loop back
            smartPortIdCnt = 0;
            id = frSkyDataIdTable[smartPortIdCnt];
        }
        smartPortIdCnt++;

        int32_t tmpi;

        switch(id) {
#ifdef GPS
            case FSSP_DATAID_SPEED      :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    //convert to knots: 1cm/s = 0.0194384449 knots
                    //Speed should be sent in knots/1000 (GPS speed is in cm/s)
                    uint32_t tmpui = gpsSol.groundSpeed * 1944 / 100;
                    smartPortSendPackage(id, tmpui);
                    smartPortHasRequest = 0;
                }
                break;
#endif
            case FSSP_DATAID_VFAS       :
                if (feature(FEATURE_VBAT)) {
                    uint16_t vfasVoltage;
                    if (telemetryConfig->frsky_vfas_cell_voltage) {
                        vfasVoltage = vbat / batteryCellCount;
                    } else {
                        vfasVoltage = vbat;
                    }
                    smartPortSendPackage(id, vfasVoltage * 10); // given in 0.1V, convert to volts
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_CURRENT    :
                if (feature(FEATURE_CURRENT_METER)) {
                    smartPortSendPackage(id, amperage / 10); // given in 10mA steps, unknown requested unit
                    smartPortHasRequest = 0;
                }
                break;
            //case FSSP_DATAID_RPM        :
            case FSSP_DATAID_ALTITUDE   :
                if (sensors(SENSOR_BARO)) {
                    smartPortSendPackage(id, baro.BaroAlt); // unknown given unit, requested 100 = 1 meter
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_FUEL       :
                if (feature(FEATURE_CURRENT_METER)) {
                    smartPortSendPackage(id, mAhDrawn); // given in mAh, unknown requested unit
                    smartPortHasRequest = 0;
                }
                break;
            //case FSSP_DATAID_ADC1       :
            //case FSSP_DATAID_ADC2       :
#ifdef GPS
            case FSSP_DATAID_LATLONG    :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    uint32_t tmpui = 0;
                    // the same ID is sent twice, one for longitude, one for latitude
                    // the MSB of the sent uint32_t helps FrSky keep track
                    // the even/odd bit of our counter helps us keep track
                    if (smartPortIdCnt & 1) {
                        tmpui = abs(gpsSol.llh.lon);  // now we have unsigned value and one bit to spare
                        tmpui = (tmpui + tmpui / 2) / 25 | 0x80000000;  // 6/100 = 1.5/25, division by power of 2 is fast
                        if (gpsSol.llh.lon < 0) tmpui |= 0x40000000;
                    }
                    else {
                        tmpui = abs(gpsSol.llh.lat);  // now we have unsigned value and one bit to spare
                        tmpui = (tmpui + tmpui / 2) / 25;  // 6/100 = 1.5/25, division by power of 2 is fast
                        if (gpsSol.llh.lat < 0) tmpui |= 0x40000000;
                    }
                    smartPortSendPackage(id, tmpui);
                    smartPortHasRequest = 0;
                }
                break;
#endif
            //case FSSP_DATAID_CAP_USED   :
            case FSSP_DATAID_VARIO      :
                if (sensors(SENSOR_BARO)) {
                    smartPortSendPackage(id, lrintf(getEstimatedActualVelocity(Z))); // unknown given unit but requested in 100 = 1m/s
                    smartPortHasRequest = 0;
                }
                break;
            case FSSP_DATAID_HEADING    :
                smartPortSendPackage(id, attitude.values.yaw * 10); // given in 10*deg, requested in 10000 = 100 deg
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCX       :
                smartPortSendPackage(id, 100 * acc.accADC[X] / acc.dev.acc_1G);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCY       :
                smartPortSendPackage(id, 100 * acc.accADC[Y] / acc.dev.acc_1G);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_ACCZ       :
                smartPortSendPackage(id, 100 * acc.accADC[Z] / acc.dev.acc_1G);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_T1         :
                // we send all the flags as decimal digits for easy reading

                tmpi =  10000; // start off with at least one digit so the most significant 0 won't be cut off
                // the Taranis seems to be able to fit 5 digits on the screen
                // the Taranis seems to consider this number a signed 16 bit integer

                if (ARMING_FLAG(OK_TO_ARM))
                    tmpi += 1;
                if (ARMING_FLAG(PREVENT_ARMING))
                    tmpi += 2;
                if (ARMING_FLAG(ARMED))
                    tmpi += 4;

                if (FLIGHT_MODE(ANGLE_MODE))
                    tmpi += 10;
                if (FLIGHT_MODE(HORIZON_MODE))
                    tmpi += 20;
                if (FLIGHT_MODE(UNUSED_MODE))
                    tmpi += 40;
                if (FLIGHT_MODE(PASSTHRU_MODE))
                    tmpi += 40;

                if (FLIGHT_MODE(MAG_MODE))
                    tmpi += 100;
                if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
                    tmpi += 200;
                if (FLIGHT_MODE(NAV_POSHOLD_MODE))
                    tmpi += 400;

                if (FLIGHT_MODE(NAV_RTH_MODE))
                    tmpi += 1000;
                if (FLIGHT_MODE(NAV_WP_MODE))
                    tmpi += 2000;
                if (FLIGHT_MODE(HEADFREE_MODE))
                    tmpi += 4000;

                smartPortSendPackage(id, (uint32_t)tmpi);
                smartPortHasRequest = 0;
                break;
            case FSSP_DATAID_T2         :
                if (sensors(SENSOR_GPS)) {
#ifdef GPS
                    // provide GPS lock status
                    smartPortSendPackage(id, (STATE(GPS_FIX) ? 1000 : 0) + (STATE(GPS_FIX_HOME) ? 2000 : 0) + gpsSol.numSat);
                    smartPortHasRequest = 0;
#endif
                }
                else if (feature(FEATURE_GPS)) {
                    smartPortSendPackage(id, 0);
                    smartPortHasRequest = 0;
                }
                break;
#ifdef GPS
            case FSSP_DATAID_GPS_ALT    :
                if (sensors(SENSOR_GPS) && STATE(GPS_FIX)) {
                    smartPortSendPackage(id, gpsSol.llh.alt); // cm
                    smartPortHasRequest = 0;
                }
                break;
#endif
#ifdef PITOT
            case FSSP_DATAID_ASPD    :
                if (sensors(SENSOR_PITOT)) {
                    smartPortSendPackage(id, pitot.airSpeed*0.194384449f); // cm/s to knots*10
                    smartPortHasRequest = 0;
                }
                break;
#endif
            case FSSP_DATAID_A4         :
                if (feature(FEATURE_VBAT)) {
                    smartPortSendPackage(id, vbat * 10 / batteryCellCount ); // given in 0.1V, convert to volts
                    smartPortHasRequest = 0;
                }
                break;
            default:
                break;
                // if nothing is sent, smartPortHasRequest isn't cleared, we already incremented the counter, just loop back to the start
        }
    }
}

#endif
