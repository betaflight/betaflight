/* 
 * FrSky Telemetry implementation by silpstream @ rcgroups
 */
#include "board.h"
#include "mw.h"

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
#define ID_VOLTAGE_AMP_BP     0x3A
#define ID_VOLTAGE_AMP_AP     0x3B
#define ID_CURRENT            0x28
// User defined data IDs
#define ID_GYRO_X             0x40
#define ID_GYRO_Y             0x41
#define ID_GYRO_Z             0x42

static void sendDataHead(uint8_t id)
{
    uartWrite(PROTOCOL_HEADER);
    uartWrite(id);
}

static void sendTelemetryTail(void)
{
    uartWrite(PROTOCOL_TAIL);
}

static void serializeFrsky(uint8_t data)
{
    // take care of byte stuffing
    if (data == 0x5e) {
        uartWrite(0x5d);
        uartWrite(0x3e);
    } else if (data == 0x5d) {
        uartWrite(0x5d);
        uartWrite(0x3d);
    } else
        uartWrite(data);
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
    serialize16(EstAlt / 100);
    sendDataHead(ID_ALTITUDE_AP);
    serialize16(EstAlt % 100);
}

static void sendTemperature1(void)
{
    sendDataHead(ID_TEMPRATURE1);
    serialize16(telemTemperature1 / 10);
}

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

static void sendGPS(void)
{
    sendDataHead(ID_LATITUDE_BP);
    serialize16(abs(GPS_coord[LAT]) / 100000);
    sendDataHead(ID_LATITUDE_AP);
    serialize16((abs(GPS_coord[LAT]) / 100000) % 10000);

    sendDataHead(ID_N_S);
    serialize16(GPS_coord[LAT] < 0 ? 'S' : 'N');

    sendDataHead(ID_LONGITUDE_BP);
    serialize16(abs(GPS_coord[LON]) / 100000);
    sendDataHead(ID_LONGITUDE_AP);
    serialize16((abs(GPS_coord[LON]) / 100000) % 10000);
    sendDataHead(ID_E_W);
    serialize16(GPS_coord[LON] < 0 ? 'W' : 'E');
}

static void sendVoltage(void)
{
    uint16_t voltage;

    voltage = (vbat * 110) / 21;

    sendDataHead(ID_VOLTAGE_AMP_BP);
    serialize16(voltage / 100);
    sendDataHead(ID_VOLTAGE_AMP_AP);
    serialize16(((voltage % 100) + 5) / 10);
}

static void sendHeading(void)
{
    sendDataHead(ID_COURSE_BP);
    serialize16(heading);
    sendDataHead(ID_COURSE_AP);
    serialize16(0);
}

static bool telemetryEnabled = false;

void initTelemetry(bool State)
{
    if (State != telemetryEnabled) {
        if (State)
            serialInit(9600);
        else
            serialInit(cfg.serial_baudrate);
        telemetryEnabled = State;
    }
}

static uint32_t lastCycleTime = 0;
static uint8_t cycleNum = 0;

void sendTelemetry(void)
{
    if (millis() - lastCycleTime >= CYCLETIME) {
        lastCycleTime = millis();
        cycleNum++;

        // Sent every 125ms
        sendAccel();
        sendTelemetryTail();

        if ((cycleNum % 4) == 0) {      // Sent every 500ms
            sendBaro();
            sendHeading();
            sendTelemetryTail();
        }

        if ((cycleNum % 8) == 0) {      // Sent every 1s
            sendTemperature1();
            if (feature(FEATURE_VBAT))
                sendVoltage();
            if (sensors(SENSOR_GPS))
                sendGPS();
            sendTelemetryTail();
        }

        if (cycleNum == 40) {     //Frame 3: Sent every 5s
            cycleNum = 0;
            sendTime();
            sendTelemetryTail();
        }
    }
}
