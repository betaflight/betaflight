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

/*
 * Initial FrSky Telemetry implementation by silpstream @ rcgroups.
 * Addition protocol work by airmamaf @ github.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef TELEMETRY

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"


#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/rc_controls.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"

static serialPort_t *frskyPort = NULL;
static serialPortConfig_t *portConfig;

#define FRSKY_BAUDRATE 9600
#define FRSKY_INITIAL_PORT_MODE MODE_TX

static telemetryConfig_t *telemetryConfig;
static bool frskyTelemetryEnabled =  false;
static portSharing_e frskyPortSharing;


extern batteryConfig_t *batteryConfig;

extern int16_t telemTemperature1; // FIXME dependency on mw.c

#define CYCLETIME             125

#define PROTOCOL_HEADER       0x5E
#define PROTOCOL_TAIL         0x5E

// Data Ids  (bp = before decimal point; af = after decimal point)
// Official data IDs
#define ID_GPS_ALTIDUTE_BP    0x01
#define ID_GPS_ALTIDUTE_AP    0x09
#define ID_TEMPRATURE1        0x02
#define ID_RPM                0x03
#define ID_FUEL_LEVEL         0x04
#define ID_TEMPRATURE2        0x05
#define ID_VOLT               0x06
#define ID_ALTITUDE_BP        0x10
#define ID_ALTITUDE_AP        0x21
#define ID_GPS_SPEED_BP       0x11
#define ID_GPS_SPEED_AP       0x19
#define ID_LONGITUDE_BP       0x12
#define ID_LONGITUDE_AP       0x1A
#define ID_E_W                0x22
#define ID_LATITUDE_BP        0x13
#define ID_LATITUDE_AP        0x1B
#define ID_N_S                0x23
#define ID_COURSE_BP          0x14
#define ID_COURSE_AP          0x1C
#define ID_DATE_MONTH         0x15
#define ID_YEAR               0x16
#define ID_HOUR_MINUTE        0x17
#define ID_SECOND             0x18
#define ID_ACC_X              0x24
#define ID_ACC_Y              0x25
#define ID_ACC_Z              0x26
#define ID_VOLTAGE_AMP        0x39
#define ID_VOLTAGE_AMP_BP     0x3A
#define ID_VOLTAGE_AMP_AP     0x3B
#define ID_CURRENT            0x28
// User defined data IDs
#define ID_GYRO_X             0x40
#define ID_GYRO_Y             0x41
#define ID_GYRO_Z             0x42

#define ID_VERT_SPEED         0x30 //opentx vario

#define GPS_BAD_QUALITY       300
#define GPS_MAX_HDOP_VAL      9999
#define DELAY_FOR_BARO_INITIALISATION (5 * 1000) //5s
#define BLADE_NUMBER_DIVIDER  5 // should set 12 blades in Taranis

static uint32_t lastCycleTime = 0;
static uint8_t cycleNum = 0;
static void sendDataHead(uint8_t id)
{
    serialWrite(frskyPort, PROTOCOL_HEADER);
    serialWrite(frskyPort, id);
}

static void sendTelemetryTail(void)
{
    serialWrite(frskyPort, PROTOCOL_TAIL);
}

static void serializeFrsky(uint8_t data)
{
    // take care of byte stuffing
    if (data == 0x5e) {
        serialWrite(frskyPort, 0x5d);
        serialWrite(frskyPort, 0x3e);
    } else if (data == 0x5d) {
        serialWrite(frskyPort, 0x5d);
        serialWrite(frskyPort, 0x3d);
    } else
        serialWrite(frskyPort, data);
}

static void serialize16(int16_t a)
{
    uint8_t t;
    t = a;
    serializeFrsky(t);
    t = a >> 8 & 0xff;
    serializeFrsky(t);
}

static void sendAccel(void)
{
    int i;

    for (i = 0; i < 3; i++) {
        sendDataHead(ID_ACC_X + i);
        serialize16(((float)accSmooth[i] / acc_1G) * 1000);
    }
}

static void sendBaro(void)
{
    sendDataHead(ID_ALTITUDE_BP);
    serialize16(BaroAlt / 100);
    sendDataHead(ID_ALTITUDE_AP);
    serialize16(ABS(BaroAlt % 100));
}

#ifdef GPS
static void sendGpsAltitude(void)
{
    uint16_t altitude = GPS_altitude;
    //Send real GPS altitude only if it's reliable (there's a GPS fix)
    if (!STATE(GPS_FIX)) {
        altitude = 0;
    }
    sendDataHead(ID_GPS_ALTIDUTE_BP);
    serialize16(altitude);
    sendDataHead(ID_GPS_ALTIDUTE_AP);
    serialize16(0);
}
#endif

static void sendThrottleOrBatterySizeAsRpm(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    uint16_t throttleForRPM = rcCommand[THROTTLE] / BLADE_NUMBER_DIVIDER;
    sendDataHead(ID_RPM);
    if (ARMING_FLAG(ARMED)) {
        throttleStatus_e throttleStatus = calculateThrottleStatus(rxConfig, deadband3d_throttle);
        if (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP))
                    throttleForRPM = 0;
        serialize16(throttleForRPM);
    } else {
        serialize16((batteryConfig->batteryCapacity / BLADE_NUMBER_DIVIDER));
    }

}

static void sendTemperature1(void)
{
    sendDataHead(ID_TEMPRATURE1);
#ifdef BARO
    serialize16((baroTemperature + 50)/ 100); //Airmamaf
#else
    serialize16(telemTemperature1 / 10);
#endif
}

#ifdef GPS
static void sendSatalliteSignalQualityAsTemperature2(void)
{
    uint16_t satellite = GPS_numSat;
    if (GPS_hdop > GPS_BAD_QUALITY && ( (cycleNum % 16 ) < 8)) {//Every 1s
        satellite = constrain(GPS_hdop, 0, GPS_MAX_HDOP_VAL);
    }
    sendDataHead(ID_TEMPRATURE2);

    if (telemetryConfig->frsky_unit == FRSKY_UNIT_METRICS) {
        serialize16(satellite);
    } else {
        float tmp = (satellite - 32) / 1.8f;
        //Round the value
        tmp += (tmp < 0) ? -0.5f : 0.5f;
        serialize16(tmp);
    }
}

static void sendSpeed(void)
{
    if (!STATE(GPS_FIX)) {
        return;
    }
    //Speed should be sent in knots (GPS speed is in cm/s)
    sendDataHead(ID_GPS_SPEED_BP);
    //convert to knots: 1cm/s = 0.0194384449 knots
    serialize16(GPS_speed * 1944 / 100000);
    sendDataHead(ID_GPS_SPEED_AP);
    serialize16((GPS_speed * 1944 / 100) % 100);
}
#endif

static void sendTime(void)
{
    uint32_t seconds = millis() / 1000;
    uint8_t minutes = (seconds / 60) % 60;

    // if we fly for more than an hour, something's wrong anyway
    sendDataHead(ID_HOUR_MINUTE);
    serialize16(minutes << 8);
    sendDataHead(ID_SECOND);
    serialize16(seconds % 60);
}

// Frsky pdf: dddmm.mmmm
// .mmmm is returned in decimal fraction of minutes.
static void GPStoDDDMM_MMMM(int32_t mwiigps, gpsCoordinateDDDMMmmmm_t *result)
{
    int32_t absgps, deg, min;
    absgps = ABS(mwiigps);
    deg    = absgps / GPS_DEGREES_DIVIDER;
    absgps = (absgps - deg * GPS_DEGREES_DIVIDER) * 60;        // absgps = Minutes left * 10^7
    min    = absgps / GPS_DEGREES_DIVIDER;                     // minutes left

    if (telemetryConfig->frsky_coordinate_format == FRSKY_FORMAT_DMS) {
        result->dddmm = deg * 100 + min;
    } else {
        result->dddmm = deg * 60 + min;
    }

    result->mmmm  = (absgps - min * GPS_DEGREES_DIVIDER) / 1000;
}

static void sendLatLong(int32_t coord[2])
{
    gpsCoordinateDDDMMmmmm_t coordinate;
    GPStoDDDMM_MMMM(coord[LAT], &coordinate);
    sendDataHead(ID_LATITUDE_BP);
    serialize16(coordinate.dddmm);
    sendDataHead(ID_LATITUDE_AP);
    serialize16(coordinate.mmmm);
    sendDataHead(ID_N_S);
    serialize16(coord[LAT] < 0 ? 'S' : 'N');

    GPStoDDDMM_MMMM(coord[LON], &coordinate);
    sendDataHead(ID_LONGITUDE_BP);
    serialize16(coordinate.dddmm);
    sendDataHead(ID_LONGITUDE_AP);
    serialize16(coordinate.mmmm);
    sendDataHead(ID_E_W);
    serialize16(coord[LON] < 0 ? 'W' : 'E');
}

static void sendFakeLatLong(void)
{
    // Heading is only displayed on OpenTX if non-zero lat/long is also sent
    int32_t coord[2] = {0,0};
    
    coord[LAT] = (telemetryConfig->gpsNoFixLatitude * GPS_DEGREES_DIVIDER);
    coord[LON] = (telemetryConfig->gpsNoFixLongitude * GPS_DEGREES_DIVIDER);

    sendLatLong(coord);
}

static void sendFakeLatLongThatAllowsHeadingDisplay(void)
{
    // Heading is only displayed on OpenTX if non-zero lat/long is also sent
    int32_t coord[2] = {
        1 * GPS_DEGREES_DIVIDER,
        1 * GPS_DEGREES_DIVIDER
    };

    sendLatLong(coord);
}

#ifdef GPS
static void sendGPSLatLong(void)
{
    static uint8_t gpsFixOccured = 0;

    if (STATE(GPS_FIX) || gpsFixOccured == 1) {
        // If we have ever had a fix, send the last known lat/long
        gpsFixOccured = 1;
        sendLatLong(GPS_coord);
    } else {
        // otherwise send fake lat/long in order to display compass value
        sendFakeLatLong();
    }
}
#endif

/*
 * Send vertical speed for opentx. ID_VERT_SPEED
 * Unit is cm/s
 */
static void sendVario(void)
{
    sendDataHead(ID_VERT_SPEED);
    serialize16(vario);
}

/*
 * Send voltage via ID_VOLT
 *
 * NOTE: This sends voltage divided by batteryCellCount. To get the real
 * battery voltage, you need to multiply the value by batteryCellCount.
 */
static void sendVoltage(void)
{
    static uint16_t currentCell = 0;
    uint32_t cellVoltage;
    uint16_t payload;

    /*
     * Format for Voltage Data for single cells is like this:
     *
     *  llll llll cccc hhhh
     *  l: Low voltage bits
     *  h: High voltage bits
     *  c: Cell number (starting at 0)
     *
     * The actual value sent for cell voltage has resolution of 0.002 volts
     * Since vbat has resolution of 0.1 volts it has to be multiplied by 50
     */
    cellVoltage = ((uint32_t)vbat * 100 + batteryCellCount) / (batteryCellCount * 2);

    // Cell number is at bit 9-12
    payload = (currentCell << 4);

    // Lower voltage bits are at bit 0-8
    payload |= ((cellVoltage & 0x0ff) << 8);

    // Higher voltage bits are at bits 13-15
    payload |= ((cellVoltage & 0xf00) >> 8);

    sendDataHead(ID_VOLT);
    serialize16(payload);

    currentCell++;
    currentCell %= batteryCellCount;
}

/*
 * Send voltage with ID_VOLTAGE_AMP
 */
static void sendVoltageAmp(void)
{
    if (telemetryConfig->frsky_vfas_precision == FRSKY_VFAS_PRECISION_HIGH) {
        /*
         * Use new ID 0x39 to send voltage directly in 0.1 volts resolution
         */
        sendDataHead(ID_VOLTAGE_AMP);
        serialize16(vbat);
    } else {
        uint16_t voltage = (vbat * 110) / 21;

        sendDataHead(ID_VOLTAGE_AMP_BP);
        serialize16(voltage / 100);
        sendDataHead(ID_VOLTAGE_AMP_AP);
        serialize16(((voltage % 100) + 5) / 10);
    }
}

static void sendAmperage(void)
{
    sendDataHead(ID_CURRENT);
    serialize16((uint16_t)(amperage / 10));
}

static void sendFuelLevel(void)
{
    sendDataHead(ID_FUEL_LEVEL);

    if (batteryConfig->batteryCapacity > 0) {
        serialize16((uint16_t)calculateBatteryCapacityRemainingPercentage());
    } else {
        serialize16((uint16_t)constrain(mAhDrawn, 0, 0xFFFF));
    }
}

static void sendHeading(void)
{
    sendDataHead(ID_COURSE_BP);
    serialize16(heading);
    sendDataHead(ID_COURSE_AP);
    serialize16(0);
}

void initFrSkyTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_FRSKY);
    frskyPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_FRSKY);
}

void freeFrSkyTelemetryPort(void)
{
    closeSerialPort(frskyPort);
    frskyPort = NULL;
    frskyTelemetryEnabled = false;
}

void configureFrSkyTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    frskyPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_FRSKY, NULL, FRSKY_BAUDRATE, FRSKY_INITIAL_PORT_MODE, telemetryConfig->telemetry_inversion ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
    if (!frskyPort) {
        return;
    }

    frskyTelemetryEnabled = true;
}

bool hasEnoughTimeLapsedSinceLastTelemetryTransmission(uint32_t currentMillis)
{
    return currentMillis - lastCycleTime >= CYCLETIME;
}

void checkFrSkyTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(frskyPortSharing);

    if (newTelemetryEnabledValue == frskyTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureFrSkyTelemetryPort();
    else
        freeFrSkyTelemetryPort();
}

void handleFrSkyTelemetry(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    if (!frskyTelemetryEnabled) {
        return;
    }

    uint32_t now = millis();

    if (!hasEnoughTimeLapsedSinceLastTelemetryTransmission(now)) {
        return;
    }

    lastCycleTime = now;

    cycleNum++;

    // Sent every 125ms
    sendAccel();
    sendVario();
    sendTelemetryTail();

    if ((cycleNum % 4) == 0) {      // Sent every 500ms
        if (lastCycleTime > DELAY_FOR_BARO_INITIALISATION) { //Allow 5s to boot correctly
            sendBaro();
        }
        sendHeading();
        sendTelemetryTail();
    }

    if ((cycleNum % 8) == 0) {      // Sent every 1s
        sendTemperature1();
        sendThrottleOrBatterySizeAsRpm(rxConfig, deadband3d_throttle);

        if (feature(FEATURE_VBAT)) {
            sendVoltage();
            sendVoltageAmp();
            sendAmperage();
            sendFuelLevel();
        }

#ifdef GPS
        if (sensors(SENSOR_GPS)) {
            sendSpeed();
            sendGpsAltitude();
            sendSatalliteSignalQualityAsTemperature2();
            sendGPSLatLong();
        }
        else {
            sendFakeLatLongThatAllowsHeadingDisplay();
        }
#else
        sendFakeLatLongThatAllowsHeadingDisplay();
#endif

        sendTelemetryTail();
    }

    if (cycleNum == 40) {     //Frame 3: Sent every 5s
        cycleNum = 0;
        sendTime();
        sendTelemetryTail();
    }
}

#endif
